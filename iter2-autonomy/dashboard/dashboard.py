"""Native SDL display and guarded motor command bridge on QNX."""
import argparse
import ctypes as C
import json
import queue
from pathlib import Path
import socket
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'iphone-usb'))
from receive_motion import validate
from video import VideoFeed
from control import ControlLink, DebugDrive, MotorLink, VoiceRelay, classified_obstacle
from keyboard_drive import KeyboardDrive
from mapping import SceneFeed
from edge_status import EdgeStatus


class Rect(C.Structure):
    _fields_ = [(k, C.c_int) for k in ('x', 'y', 'w', 'h')]


class Color(C.Structure):
    _fields_ = [(k, C.c_uint8) for k in ('r', 'g', 'b', 'a')]


def bind(lib, name, result, *args):
    fn = getattr(lib, name)
    fn.restype, fn.argtypes = result, list(args)
    return fn


class Feed:
    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()
        self.sample = None
        self.received = 0.0
        self.count = self.gaps = 0
        self.status = 'Connecting to iPhone'
        self.stop = threading.Event()
        self.sock = None

    def run(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=2) as sock:
                    with self.lock:
                        self.sock = sock
                    sock.settimeout(1)
                    buf = b''
                    previous = None
                    while not self.stop.is_set():
                        data = sock.recv(8192)
                        if not data:
                            raise RuntimeError('Phone disconnected')
                        buf += data
                        if len(buf) > 65536:
                            raise ValueError('Oversized telemetry buffer')
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            obj = json.loads(line)
                            if not isinstance(obj, dict):
                                raise ValueError('Expected a sample object')
                            s = validate(obj)
                            if previous is not None and s['seq'] <= previous:
                                raise ValueError('Non-increasing sequence')
                            with self.lock:
                                self.gaps += max(0, s['seq'] - previous - 1) if previous is not None else 0
                                self.sample, self.received = s, time.monotonic()
                                self.count += 1
                                self.status = 'LIVE / USB'
                            previous = s['seq']
            except (OSError, ValueError, RuntimeError) as e:
                with self.lock:
                    self.status = str(e)
                self.stop.wait(1)


def stop_feed(feed):
    feed.stop.set()
    with feed.lock:
        sock = feed.sock
        pending = getattr(feed, 'pending', None)
        if pending:
            pending['done'].set()
    if sock:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--paused', action='store_true', help='Wait for Start before opening streams')
    ap.add_argument('--port', type=int, default=5557)
    ap.add_argument('--video-port', type=int, default=5558)
    ap.add_argument('--seconds', type=float, default=0, help='Exit after a timed smoke test')
    ap.add_argument('--font', default='/usr/share/fonts/dejavu/DejaVuSans.ttf')
    args = ap.parse_args()
    sdl, ttf = C.CDLL('libSDL2.so'), C.CDLL('libSDL2_ttf.so')
    img = C.CDLL('libSDL2_image.so')
    P, I = C.c_void_p, C.c_int
    init = bind(sdl, 'SDL_Init', I, C.c_uint32)
    error = bind(sdl, 'SDL_GetError', C.c_char_p)
    window = bind(sdl, 'SDL_CreateWindow', P, C.c_char_p, I, I, I, I, C.c_uint32)
    renderer = bind(sdl, 'SDL_CreateRenderer', P, P, I, C.c_uint32)
    color = bind(sdl, 'SDL_SetRenderDrawColor', I, P, C.c_uint8, C.c_uint8, C.c_uint8, C.c_uint8)
    clear = bind(sdl, 'SDL_RenderClear', I, P)
    fill = bind(sdl, 'SDL_RenderFillRect', I, P, C.POINTER(Rect))
    line = bind(sdl, 'SDL_RenderDrawLine', I, P, I, I, I, I)
    present = bind(sdl, 'SDL_RenderPresent', None, P)
    poll = bind(sdl, 'SDL_PollEvent', I, P)
    texture = bind(sdl, 'SDL_CreateTextureFromSurface', P, P, P)
    query = bind(sdl, 'SDL_QueryTexture', I, P, P, P, C.POINTER(I), C.POINTER(I))
    copy = bind(sdl, 'SDL_RenderCopy', I, P, P, C.POINTER(Rect), C.POINTER(Rect))
    free_surface = bind(sdl, 'SDL_FreeSurface', None, P)
    destroy = bind(sdl, 'SDL_DestroyTexture', None, P)
    rw_mem = bind(sdl, 'SDL_RWFromConstMem', P, P, I)
    load_jpeg = bind(img, 'IMG_Load_RW', P, P, I)
    bind(img, 'IMG_Init', I, I)(1)
    bind(ttf, 'TTF_Init', I)()
    font = bind(ttf, 'TTF_OpenFont', P, C.c_char_p, I)(args.font.encode(), 24)
    text_surface = bind(ttf, 'TTF_RenderUTF8_Blended', P, P, C.c_char_p, Color)
    if init(0x20) or not font:
        raise RuntimeError(error().decode())
    win = window(b'MotionUSB - Robot telemetry', 0x2fff0000, 0x2fff0000, 1280, 800, 4)
    ren = renderer(win, -1, 0)
    if not win or not ren:
        raise RuntimeError(error().decode())
    bind(sdl, 'SDL_RenderSetLogicalSize', I, P, I, I)(ren, 1280, 800)

    def text(msg, x, y, rgb=(221, 230, 241)):
        surf = text_surface(font, msg.encode(), Color(*rgb, 255))
        if not surf:
            return
        tex = texture(ren, surf)
        free_surface(surf)
        if tex:
            w, h = I(), I()
            query(tex, None, None, C.byref(w), C.byref(h))
            copy(ren, tex, None, C.byref(Rect(x, y, w.value, h.value)))
            destroy(tex)

    feed = Feed(args.port)
    video = VideoFeed(args.video_port)
    scene = SceneFeed(fusion_port=5569)
    workers = []
    enabled = False
    link, drive = ControlLink(), DebugDrive()
    motors = MotorLink()
    motor_thread = threading.Thread(target=motors.run, daemon=True)
    motor_thread.start()
    voice = VoiceRelay()
    threading.Thread(target=voice.run, daemon=True).start()
    edge = EdgeStatus()
    edge_thread = threading.Thread(target=edge.run, daemon=True)
    edge_thread.start()
    link_thread = threading.Thread(target=link.run, daemon=True)
    link_thread.start()
    workers.append(link_thread)

    def toggle_streams():
        nonlocal feed, video, scene, enabled
        if enabled:
            stop_feed(feed)
            stop_feed(video)
            stop_feed(scene)
            enabled = False
            print('Streams stopped by user', flush=True)
        else:
            feed, video = Feed(args.port), VideoFeed(args.video_port)
            scene = SceneFeed(mapper=scene.mapper, fusion_port=5569)
            for source in (feed, video, scene):
                worker = threading.Thread(target=source.run, daemon=True)
                worker.start()
                workers.append(worker)
            enabled = True
            print('Streams started', flush=True)

    if not args.paused:
        toggle_streams()
    map_snapshot, last_map = {}, 0.0
    camera_texture = None
    camera_w, camera_h = I(), I()
    camera_rate = 0.0
    last_video_count = 0
    event = (C.c_uint64 * 16)()
    start = last_rate = time.monotonic()
    last_count = 0
    rate = 0.0
    running = True
    keyboard = KeyboardDrive()
    last_keyboard_generation = -1
    print('Dashboard window ready', flush=True)
    try:
        while running:
            now = time.monotonic()
            while poll(C.byref(event)):
                kind = C.cast(event, C.POINTER(C.c_uint32))[0]
                # SDL_KeyboardEvent keysym.sym is at byte 20.
                key = C.c_int32.from_buffer(event, 20).value
                repeat = C.c_uint8.from_buffer(event, 13).value
                keyboard.event(kind, key, bool(repeat), C.c_uint8.from_buffer(event, 12).value,
                               C.c_uint16.from_buffer(event, 24).value)
                if kind == 0x300 and key in (32, 27) and not repeat:
                    motors.request_stop()
                if kind == 0x300 and key == ord('f') and not repeat:
                    keyboard.stop()
                    keyboard.claimed = False
                    drive.command({'op': 'halt'}, now)
                click = kind == 0x401 and C.c_uint8.from_buffer(event, 16).value == 1
                x, y = C.c_int32.from_buffer(event, 20).value, C.c_int32.from_buffer(event, 24).value
                if (kind == 0x300 and key == ord('p') and not repeat) or (click and 1035 <= x < 1250 and 20 <= y < 68):
                    toggle_streams()
                    last_count = feed.count
                    last_video_count = video.count
                    last_rate = time.monotonic()
                    rate = camera_rate = 0.0
                if kind == 0x100 or (kind == 0x300 and key == 27):
                    running = False
            while True:
                try:
                    command = link.commands.get_nowait()
                except queue.Empty:
                    break
                if command['op'] == 'control_phone':
                    keyboard.stop()
                    keyboard.claimed = False
                    drive.command({'op': 'halt'}, now)
                if command['op'] in ('halt', 'stop'):
                    motors.request_stop()
                    keyboard.stop()
                if command['op'] == 'voice':
                    if VoiceRelay.is_stop(command['text']):   # stop first, locally
                        motors.request_stop()
                        keyboard.stop()
                        drive.command({'op': 'halt'}, now)
                    voice.submit(command['text'])
                    continue
                if command['op'] == 'drive' and keyboard.claimed:
                    continue
                drive.command(command, now)
                if command['op'] in ('map_save', 'map_load', 'map_clear', 'select_target'):
                    scene.mapper.command(command)
                    last_map = 0
                if command['op'] in ('start', 'stop') and (command['op'] == 'start') != enabled:
                    toggle_streams()
                    last_count, last_video_count, last_rate = feed.count, video.count, now
                    rate = camera_rate = 0.0
            with feed.lock:
                sample, age = feed.sample, max(0, time.monotonic() - feed.received)
                count, gaps, status = feed.count, feed.gaps, feed.status
            ticket = video.take()
            if ticket and not enabled:
                ticket['done'].set()
                ticket = None
            if ticket:
                try:
                    memory = C.create_string_buffer(ticket['jpeg'])
                    surf = load_jpeg(rw_mem(memory, len(ticket['jpeg'])), 1)
                    if surf:
                        new_texture = texture(ren, surf)
                        free_surface(surf)
                        if new_texture:
                            if camera_texture:
                                destroy(camera_texture)
                            camera_texture = new_texture
                            query(camera_texture, None, None, C.byref(camera_w), C.byref(camera_h))
                            ticket['ok'] = True
                finally:
                    ticket['done'].set()
            with video.lock:
                video_count = video.count
                video_age = max(0, time.monotonic() - video.received)
                video_status = video.status
            if now - last_rate >= 1:
                rate = (count - last_count) / (now - last_rate)
                camera_rate = (video_count - last_video_count) / (now - last_rate)
                last_count, last_rate, last_video_count = count, now, video_count
            color(ren, 13, 20, 33, 255)
            clear(ren)
            text('ROBOT / CAMERA + MOTION', 28, 24)
            color(ren, *( (150, 59, 65) if enabled else (25, 130, 102) ), 255)
            fill(ren, C.byref(Rect(1035, 20, 215, 48)))
            text('Stop streams' if enabled else 'Start streams', 1050, 28)
            text('iPHONE REAR CAMERA', 28, 84)
            camera_live = enabled and camera_texture and video_age < 1
            text(f'{camera_rate:.1f} fps  |  Frames {video_count}', 28, 124)
            color(ren, 5, 10, 18, 255)
            fill(ren, C.byref(Rect(28, 175, 720, 540)))
            if camera_texture and camera_w.value and camera_h.value:
                scale = min(720 / camera_w.value, 540 / camera_h.value)
                w, h = int(camera_w.value * scale), int(camera_h.value * scale)
                copy(ren, camera_texture, None, C.byref(Rect(28 + (720-w)//2, 175 + (540-h)//2, w, h)))
            if not camera_live:
                text('CAMERA STALE / waiting for video' if enabled else 'STOPPED / press Start or P', 40, 190, (255, 147, 112))
            text(video_status[:52] if enabled else 'Streams stopped on Pi', 28, 730, (64, 221, 172) if camera_live else (255, 147, 112))
            live = enabled and sample is not None and age < 0.5
            text('MOTION LIVE / USB' if live else ('MOTION STALE' if enabled else 'MOTION STOPPED'), 790, 84,
                 (64, 221, 172) if live else (255, 147, 112))
            text(f'{rate:.1f} Hz  |  Gaps {gaps}', 790, 124)
            text(f'Receipt age: {age * 1000:.0f} ms' if sample else status[:32], 790, 164)
            if sample:
                for column, (field, title, scale) in enumerate((('accel_m_s2', 'ACCELERATION / m/s2', 12), ('gyro_rad_s', 'GYROSCOPE / rad/s', 3))):
                    x = 790
                    top = 230 + column * 245
                    text(title, x, top)
                    for axis, value in enumerate(sample[field]):
                        y = top + 45 + axis * 59
                        rgb = ((94, 175, 255), (97, 224, 175), (245, 182, 100))[axis]
                        text(f'{"XYZ"[axis]}    {value:+9.4f}', x, y, rgb)
                        color(ren, 39, 51, 70, 255)
                        fill(ren, C.byref(Rect(x, y + 32, 440, 7)))
                        length = int(max(-1, min(1, value / scale)) * 220)
                        color(ren, *rgb, 255)
                        fill(ren, C.byref(Rect(x + 220 + min(0, length), y + 32, abs(length), 7)))
            text('WASD: drive | Space: motor stop | click window to focus', 28, 770, (144, 160, 184))
            text('P: streams | F: phone | Esc: close', 875, 770, (144, 160, 184))
            if now - last_map >= 0.5:
                map_snapshot = scene.mapper.snapshot()
                last_map = now
            edge_snapshot = edge.snapshot()
            permitted = edge_snapshot['safety'].get('permit') is True and edge_snapshot['safety']['live']
            left, right = drive.state(now, enabled and link.connected and permitted)
            if keyboard.claimed:
                native = motors.snapshot()
                left, right = keyboard.state(enabled and native.get('connected', False) and
                                             (permitted or keyboard.shifted))
                if keyboard.generation != last_keyboard_generation:
                    drive.generation += 1
                    last_keyboard_generation = keyboard.generation
            override_key = chr(keyboard.key) if keyboard.claimed and keyboard.key and keyboard.shifted else None
            motors.update(left, right, drive.generation, override_key)
            motor_status = motors.snapshot()
            link.publish(dict(type='status', mode='dashboard', streaming=enabled, voice=voice.snapshot(),
                              motion_hz=round(rate, 1), camera_fps=round(camera_rate, 1),
                              samples=count, gaps=gaps, frames=video_count,
                              motion_live=bool(live), camera_live=bool(camera_live),
                              accel_m_s2=sample['accel_m_s2'] if sample else None,
                              gyro_rad_s=sample['gyro_rad_s'] if sample else None,
                              receipt_age_ms=round(age * 1000) if sample else None,
                              motor_mode=motor_status.get('mode', 'unknown'), motor=motor_status, left=left, right=right, surroundings=map_snapshot, **edge_snapshot))
            present(ren)
            if args.seconds and now - start >= args.seconds:
                break
            time.sleep(0.025)
    finally:
        voice.stop.set()
        motors.stop.set()
        motor_thread.join(timeout=1)
        edge.stop.set()
        edge_thread.join(timeout=1)
        link.stop.set()
        elapsed = time.monotonic() - start
        with feed.lock:
            final_samples, final_gaps = feed.count, feed.gaps
        with video.lock:
            final_frames = video.count
        stop_feed(feed)
        stop_feed(video)
        stop_feed(scene)
        for worker in workers:
            worker.join(timeout=3)
        if camera_texture:
            destroy(camera_texture)
        bind(img, 'IMG_Quit', None)()
        bind(ttf, 'TTF_CloseFont', None, P)(font)
        bind(sdl, 'SDL_DestroyRenderer', None, P)(ren)
        bind(sdl, 'SDL_DestroyWindow', None, P)(win)
        bind(ttf, 'TTF_Quit', None)()
        bind(sdl, 'SDL_Quit', None)()
        print(f'Dashboard stopped after {elapsed:.2f}s: {final_samples} samples, {final_gaps} gaps; {final_frames} camera frames', flush=True)


if __name__ == '__main__':
    main()
