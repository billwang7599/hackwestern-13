"""Receive motion/video without a screen; return status to the mounted iPhone."""
import argparse
import json
import os
import queue
import signal
import socket
import threading
import time
from dashboard import Feed, stop_feed
from video import VideoFeed
from control import ControlLink, DebugDrive, MotorLink, VoiceRelay, classified_obstacle
from mapping import SceneFeed
from edge_status import EdgeStatus
from headless_keyboard import HeadlessKeyboard


class MissionTee:
    """Opt-in copy of the phone status for the laptop mission control (iter2-autonomy/mission/).

    Only runs when TARS_MISSION_TEE_PORT is set (mission_relay listens on 127.0.0.1:5576).
    Read-only: once a second it sends the status this process last published for the phone,
    one JSON line, to the relay. Its own thread and socket with a short timeout; any error
    closes the link and retries later. It never touches the phone link, motors or the guard.
    """
    def __init__(self, link, port, period=1.0):
        self.link, self.port, self.period = link, port, period
        self.stop = threading.Event()
        self.sent = 0

    def run(self):
        sock = None
        while not self.stop.wait(self.period):
            with self.link.lock:
                status = self.link.latest
            if not status:
                continue
            try:
                if sock is None:
                    sock = socket.create_connection(('127.0.0.1', self.port), timeout=0.5)
                    sock.settimeout(0.5)
                sock.sendall(json.dumps(status, allow_nan=False).encode() + b'\n')
                self.sent += 1
            except (OSError, ValueError):
                if sock is not None:
                    sock.close()
                sock = None
                self.stop.wait(2)
        if sock is not None:
            sock.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=0)
    args = ap.parse_args()
    done = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: done.set())
    link, drive = ControlLink(), DebugDrive()
    motors = MotorLink()
    motor_thread = threading.Thread(target=motors.run, daemon=True)
    motor_thread.start()
    keyboard = HeadlessKeyboard()
    keyboard_thread = threading.Thread(target=keyboard.run, daemon=True)
    keyboard_thread.start()
    last_keyboard_generation = -1
    voice = VoiceRelay()
    threading.Thread(target=voice.run, daemon=True).start()
    edge = EdgeStatus()
    edge_thread = threading.Thread(target=edge.run, daemon=True)
    edge_thread.start()
    link_thread = threading.Thread(target=link.run, daemon=True)
    link_thread.start()
    tee = None
    if os.environ.get('TARS_MISSION_TEE_PORT'):
        tee = MissionTee(link, int(os.environ['TARS_MISSION_TEE_PORT']))
        threading.Thread(target=tee.run, daemon=True).start()
    feed, video = Feed(5557), VideoFeed(5558)
    scene = SceneFeed(fusion_port=5569)
    workers = []
    enabled = False

    def set_streams(on):
        nonlocal feed, video, scene, enabled
        if on == enabled:
            return
        if on:
            feed, video = Feed(5557), VideoFeed(5558)
            scene = SceneFeed(mapper=scene.mapper, fusion_port=5569)
            for source in (feed, video, scene):
                thread = threading.Thread(target=source.run, daemon=True)
                workers.append(thread)
                thread.start()
        else:
            stop_feed(feed)
            stop_feed(video)
            stop_feed(scene)
        enabled = on
        print('Streams ' + ('started' if on else 'stopped'), flush=True)

    start = previous_time = time.monotonic()
    previous_samples = previous_frames = 0
    motion_hz = camera_fps = 0.0
    map_snapshot, last_map = {}, 0.0
    set_streams(True)
    try:
        while not done.wait(0.01):
            now = time.monotonic()
            if args.seconds and now - start >= args.seconds:
                break
            while True:
                try:
                    command = link.commands.get_nowait()
                except queue.Empty:
                    break
                if command['op'] == 'control_phone':
                    keyboard.take_phone_control()
                    drive.command({'op': 'halt'}, now)
                if command['op'] in ('stop', 'halt'):
                    motors.request_stop()
                    keyboard.drive.stop()
                    keyboard.armed = False
                if command['op'] == 'voice':
                    text = str(command.get('text', ''))
                    if VoiceRelay.is_stop(text):   # stop first, locally; tars_voice also sends S + CANCEL
                        motors.request_stop()
                        keyboard.drive.stop()
                        keyboard.armed = False
                        drive.command({'op': 'halt'}, now)
                    voice.submit(text)
                    continue
                if command['op'] == 'drive' and keyboard.drive.claimed:
                    continue
                drive.command(command, now)
                if command['op'] in ('map_save', 'map_load', 'map_clear', 'select_target'):
                    scene.mapper.command(command)
                    last_map = 0
                if command['op'] in ('start', 'stop'):
                    set_streams(command['op'] == 'start')
                    previous_time = now
                    previous_samples, previous_frames = feed.count, video.count
                    motion_hz = camera_fps = 0.0
            ticket = video.take()
            if ticket:
                # Headless mode validates framing/envelope, then discards the JPEG.
                # There is no decode or display workload and no video recording.
                ticket['ok'] = enabled
                ticket['done'].set()
            with feed.lock:
                samples, gaps, age = feed.count, feed.gaps, now - feed.received
                sample = feed.sample
            with video.lock:
                frames, camera_age = video.count, now - video.received
            if now - previous_time >= 1:
                motion_hz = (samples - previous_samples) / (now - previous_time)
                camera_fps = (frames - previous_frames) / (now - previous_time)
                previous_time, previous_samples, previous_frames = now, samples, frames
            if now - last_map >= 0.5:
                map_snapshot = scene.mapper.snapshot()
                last_map = now
            edge_snapshot = edge.snapshot()
            permitted = edge_snapshot['safety'].get('permit') is True and edge_snapshot['safety']['live']
            left, right = drive.state(now, enabled and link.connected and permitted)
            native = motors.snapshot()
            was_claimed = keyboard.drive.claimed
            previous_mask = keyboard.previous
            keyboard_ready = enabled and native.get('connected', False)
            override_ready = keyboard_ready  # Explicit Shift+WASD also works without phone data.
            keyboard_live = keyboard.sample(keyboard_ready and permitted, override_ready)
            if keyboard_live and keyboard.previous & 16 and not previous_mask & 16:
                motors.request_stop()
            if was_claimed and not keyboard.drive.claimed:
                drive.command({'op': 'halt'}, now)
                left = right = 0
            if keyboard.drive.claimed:
                left, right = keyboard.drive.state(keyboard_live and keyboard_ready and (permitted or (keyboard.override_key and override_ready)))
                if keyboard.drive.generation != last_keyboard_generation:
                    drive.generation += 1
                    last_keyboard_generation = keyboard.drive.generation
            motors.update(left, right, drive.generation, keyboard.override_key if keyboard.drive.claimed else None)
            motor_status = motors.snapshot()
            motor_status['serial_device'] = os.environ.get('TARS_MOTOR_DEVICE')
            motor_status['serial_device_present'] = os.path.exists(os.environ.get('TARS_MOTOR_DEVICE', ''))
            link.publish(dict(type='status', mode='headless', streaming=enabled,
                              keyboard_live=keyboard_live, keyboard_local=keyboard.drive.claimed,
                              keyboard=keyboard.snapshot(), voice=voice.snapshot(),
                              motion_hz=round(motion_hz, 1), camera_fps=round(camera_fps, 1),
                              samples=samples, gaps=gaps, frames=frames,
                              motion_live=enabled and age < 0.5,
                              camera_live=enabled and camera_age < 1,
                              accel_m_s2=sample['accel_m_s2'] if sample else None,
                              gyro_rad_s=sample['gyro_rad_s'] if sample else None,
                              receipt_age_ms=round(max(0, age) * 1000) if sample else None,
                              motor_mode=motor_status.get('mode', 'unknown'), motor=motor_status, left=left, right=right, surroundings=map_snapshot, **edge_snapshot))
    finally:
        if tee is not None:
            tee.stop.set()
        keyboard.close()
        voice.stop.set()
        motors.stop.set()
        motor_thread.join(timeout=1)
        stop_feed(feed)
        stop_feed(video)
        stop_feed(scene)
        edge.stop.set()
        edge_thread.join(timeout=1)
        link.stop.set()
        for worker in workers + [link_thread]:
            worker.join(timeout=3)
        print(f'Headless stopped: {feed.count} motion samples, {feed.gaps} gaps, {video.count} camera frames', flush=True)
        summary = scene.mapper.snapshot()
        print('Scene summary: ' + str({key: summary[key] for key in
              ('frames', 'voxels', 'tracking', 'floor_ready', 'scan_degrees', 'message')}), flush=True)
        print('Map cells: free=' + str(summary['grid'].count('1')) +
              ' occupied=' + str(summary['grid'].count('2')) +
              ' targets=' + str(len(summary['objects'])), flush=True)


if __name__ == '__main__':
    main()
