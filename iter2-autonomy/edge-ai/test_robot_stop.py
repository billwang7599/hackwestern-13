"""Real native guard -> real native motor stub integration, on private ports.

Synthetic LiDAR verifies command inhibition, not physical sensing or braking.
Run after building both binaries. Optional ROBOT_STOP_REPORT writes observations.
"""
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time
import unittest

from test_guard import packet, floor_points

HERE = Path(__file__).resolve().parent
GUARD = os.environ.get('GUARD_BINARY', str(HERE / 'build/lidar_guard'))
MOTOR = os.environ.get('MOTOR_BINARY', str(HERE.parents[1] / 'iter1-wasd/pi/build/tars_motor_host'))
OBSERVATIONS = []


def spare_port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


class RobotStopTests(unittest.TestCase):
    def setUp(self):
        self.done = threading.Event()
        self.mode = 'clear'
        self.sequence = 0
        self.feed_error = None
        self.guard_paused = False
        self.children = []
        self.sockets = []
        self.sensor_server = socket.socket()
        self.sockets.append(self.sensor_server)
        self.sensor_server.bind(('127.0.0.1', 0))
        self.sensor_server.listen(1)
        self.sensor_server.settimeout(3)
        self.motor_port, gate_port = spare_port(), spare_port(socket.SOCK_DGRAM)
        self.fusion_port = spare_port(socket.SOCK_DGRAM)
        self.fusion_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sockets.append(self.fusion_socket)
        self.motor = subprocess.Popen([MOTOR, '-b', 'stub', '-l', '-p', str(self.motor_port),
                                       '-g', str(gate_port), '-w', '500', '-r', '10'],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(self.motor)
        deadline = time.monotonic() + 3
        while True:
            try:
                self.command_socket = socket.create_connection(('127.0.0.1', self.motor_port), .2)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        self.sockets.append(self.command_socket)
        self.command_socket.settimeout(1)
        self.reply = b''
        self.guard = subprocess.Popen([GUARD, '--port', str(self.sensor_server.getsockname()[1]),
                                       '--status-port', str(spare_port(socket.SOCK_DGRAM)),
                                       '--motor-port', str(gate_port), '--fusion-port', str(self.fusion_port)],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.children.append(self.guard)
        self.sensor, _ = self.sensor_server.accept()
        self.sockets.append(self.sensor)
        self.sensor.settimeout(.5)
        self.feeder = threading.Thread(target=self.feed, daemon=True)
        self.feeder.start()
        self.wait_status(lambda s: s['guard_permit'] == '1')

    def tearDown(self):
        self.done.set()
        if self.guard_paused:
            self.guard.send_signal(signal.SIGCONT)
        for sock in self.sockets:
            sock.close()
        self.feeder.join(timeout=1)
        for child in reversed(self.children):
            child.terminate()
            try:
                child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2)

    @classmethod
    def tearDownClass(cls):
        report = {'synthetic_lidar': True, 'backend': 'stub', 'observations': OBSERVATIONS}
        print('\nRobot stop observations: ' + json.dumps(report))
        if os.environ.get('ROBOT_STOP_REPORT'):
            destination = Path(os.environ['ROBOT_STOP_REPORT'])
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(report, indent=2) + '\n')

    def feed(self):
        while not self.done.is_set():
            mode = self.mode
            if mode == 'disconnect':
                self.sensor.close()
                return
            if mode != 'silent':
                try:
                    self.sequence += 1
                    points = floor_points()
                    if mode == 'blocked':
                        points.append([0, .2, .3])
                    self.sensor.sendall(packet(seq=self.sequence, points=points,
                                               flags=0 if mode == 'unknown' else 7))
                    if self.sensor.recv(1) != b'\x01':
                        raise RuntimeError('Missing guard acknowledgement')
                except (OSError, RuntimeError) as error:
                    if not self.done.is_set():
                        self.feed_error = str(error)
                    return
            self.done.wait(.05)

    def command(self, line):
        self.command_socket.sendall(line.encode() + b'\n')
        while b'\n' not in self.reply:
            block = self.command_socket.recv(4096)
            if not block:
                self.fail('Motor connection closed')
            self.reply += block
        line, self.reply = self.reply.split(b'\n', 1)
        return line.decode()

    def status(self):
        words = self.command('STATUS').split()
        self.assertEqual(words[0], 'OK')
        return dict(left=float(words[1]), right=float(words[2]),
                    **dict(word.split('=', 1) for word in words if '=' in word))

    def wait_status(self, predicate, timeout=1):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.status()
            if predicate(last):
                return last
            time.sleep(.01)
        self.fail('Motor status condition failed: ' + repr(last))

    def start_drive(self):
        self.command_epoch = self.status()['guard_epoch']
        self.assertTrue(self.command(f'DL {self.command_epoch} .3 .3').startswith('OK'))
        return self.wait_status(lambda s: s['left'] > .05 and s['right'] > .05)

    def stopped(self, status):
        return status['left'] == 0 and status['right'] == 0 and status['guard_permit'] == '0'

    def observe_stop(self, cause, trigger):
        self.start_drive()
        start = time.monotonic()
        trigger()
        result = self.wait_status(self.stopped, timeout=.4)
        OBSERVATIONS.append(dict(cause=cause, observed_stop_ms=round((time.monotonic()-start)*1000, 3),
                                 guard_reason=result.get('guard_reason')))

    def fusion(self, nearest, ttl=.3):
        sent = time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000
        expires = sent + int(ttl * 1_000_000) if ttl else 0
        self.fusion_socket.sendto(struct.pack('!4sQQf', b'FUS1', sent, expires, nearest),
                                 ('127.0.0.1', self.fusion_port))

    def test_inferred_obstacle_stops_then_expires_without_resuming_old_pulse(self):
        self.observe_stop('inferred_obstacle', lambda: self.fusion(.3))
        old_epoch = self.command_epoch
        # No further AI output: its contribution expires, leaving the live
        # measured LiDAR decision. An expired stop must not resurrect a pulse.
        self.wait_status(lambda s: s['guard_permit'] == '1', timeout=.6)
        self.assertTrue(self.command(f'DL {old_epoch} .3 .3').startswith('ERR'))
        self.assertEqual((self.status()['left'], self.status()['right']), (0, 0))
        self.start_drive()

    def test_inference_clear_never_overrides_measured_obstacle_or_unknown(self):
        self.mode = 'blocked'
        self.wait_status(self.stopped)
        self.fusion(-1)
        time.sleep(.05)
        self.assertTrue(self.stopped(self.status()))
        self.mode = 'unknown'
        self.fusion(-1)
        time.sleep(.08)
        self.assertTrue(self.stopped(self.status()))

    def test_obstacle_stop_clears_intent_and_requires_new_command(self):
        self.observe_stop('obstacle', lambda: setattr(self, 'mode', 'blocked'))
        old_epoch = self.command_epoch
        self.mode = 'clear'
        self.wait_status(lambda s: s['guard_permit'] == '1')
        self.assertTrue(self.command(f'DL {old_epoch} .3 .3').startswith('ERR'))
        time.sleep(.06)
        self.assertEqual((self.status()['left'], self.status()['right']), (0, 0))
        self.start_drive()

    def test_sensor_silence_unknown_and_disconnect_stop(self):
        for mode in ('silent', 'unknown', 'disconnect'):
            self.observe_stop(mode, lambda mode=mode: setattr(self, 'mode', mode))
            if mode != 'disconnect':
                self.mode = 'clear'
                self.wait_status(lambda s: s['guard_permit'] == '1')

    def test_stopped_guard_expires_independently_of_command_watchdog(self):
        def pause():
            self.guard.send_signal(signal.SIGSTOP)
            self.guard_paused = True
        self.observe_stop('guard_SIGSTOP', pause)
        # The guard timeout is shorter than the motor's 500 ms command watchdog.
        self.assertLess(OBSERVATIONS[-1]['observed_stop_ms'], 300)


if __name__ == '__main__':
    unittest.main()
