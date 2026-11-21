"""Native motor gate protocol tests. Uses private loopback ports, no hardware."""
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import threading
import time
import unittest

BINARY = os.environ.get('MOTOR_BINARY', str(Path(__file__).parent / 'build/tars_motor_host'))


def port(kind=socket.SOCK_STREAM):
    with socket.socket(socket.AF_INET, kind) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class GateHarness(unittest.TestCase):
    """tars_motor_host -b stub with the GAT1 guard gate on a private UDP port."""
    def setUp(self):
        self.tcp, self.udp = port(), port(socket.SOCK_DGRAM)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.proc = subprocess.Popen([BINARY, '-b', 'stub', '-l', '-g', str(self.udp),
                                      '-p', str(self.tcp)], env=dict(os.environ, TARS_KEYBOARD_AUTH='k'*32), stderr=subprocess.DEVNULL)
        for _ in range(100):
            try:
                self.sock = socket.create_connection(('127.0.0.1', self.tcp), timeout=.2)
                break
            except OSError:
                time.sleep(.01)
        else:
            self.proc.kill()
            self.fail('Motor server did not start')
        self.sock.settimeout(1)
        self.stream = self.sock.makefile('rb')

    def tearDown(self):
        self.stream.close(); self.sock.close(); self.sender.close()
        self.proc.terminate()
        try: self.proc.wait(2)
        except subprocess.TimeoutExpired:
            self.proc.kill(); self.proc.wait()

    def command(self, value):
        self.sock.sendall(value.encode() + b'\n')
        return self.stream.readline().decode().strip()

    def status(self):
        words = self.command('STATUS').split()
        return dict(item.split('=', 1) for item in words[6:] if '=' in item)

    def send(self, permit, stamp=None):
        stamp = stamp if stamp is not None else time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000
        packet = struct.pack('!4sQB', b'GAT1', stamp, permit)
        self.sender.sendto(packet, ('127.0.0.1', self.udp))
        return packet

    def clear(self):
        self.send(1); time.sleep(.03)
        self.assertEqual(self.status()['guard_permit'], '1')


class GateTests(GateHarness):
    def test_missing_and_malformed_gate_deny_motion(self):
        self.assertTrue(self.command('D .2 .2').startswith('ERR'))
        self.clear()
        self.sender.sendto(b'invalid', ('127.0.0.1', self.udp)); time.sleep(.03)
        self.assertEqual(self.status()['guard_permit'], '0')
        self.assertTrue(self.command('MOVE 100').startswith('ERR'))
        self.assertTrue(self.command('TURN 90').startswith('ERR'))

    def test_replay_cannot_refresh_lease(self):
        packet = self.send(1); time.sleep(.03)
        self.assertEqual(self.status()['guard_permit'], '1')
        deadline = time.monotonic() + .16
        while time.monotonic() < deadline:
            self.sender.sendto(packet, ('127.0.0.1', self.udp)); time.sleep(.01)
        self.assertEqual(self.status()['guard_permit'], '0')
        self.assertTrue(self.command('D .2 .2').startswith('ERR'))

    def test_old_and_future_datagrams_cannot_clear(self):
        self.send(1, time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000 - 200000)
        self.send(1, time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000 + 5000000)
        time.sleep(.03)
        self.assertEqual(self.status()['guard_permit'], '0')

    def test_denied_then_clear_same_queue_consumes_old_motion(self):
        self.clear()
        self.assertEqual(self.command('D .3 .3'), 'OK')
        time.sleep(.03)
        self.send(0); self.send(1)
        time.sleep(.03)
        self.send(1); time.sleep(.03)
        values = self.command('STATUS').split()
        self.assertEqual([float(x) for x in values[1:3]], [0, 0])
        self.assertEqual(self.status()['guard_permit'], '1')

    def test_brief_denial_invalidates_repeated_lease(self):
        self.clear()
        epoch = self.status()['guard_epoch']
        self.assertEqual(self.command(f'DL {epoch} .3 .3'), 'OK')
        time.sleep(.03)
        self.send(0); self.send(1)
        time.sleep(.03)
        self.send(1); time.sleep(.03)
        self.assertNotEqual(self.status()['guard_epoch'], epoch)
        self.assertTrue(self.command(f'DL {epoch} .3 .3').startswith('ERR'))
        self.assertEqual([float(x) for x in self.command('STATUS').split()[1:3]], [0, 0])
        new_epoch = self.status()['guard_epoch']
        self.assertEqual(self.command(f'DL {new_epoch} .3 .3'), 'OK')

    def test_stop_cancels_not_yet_dispatched_move(self):
        self.clear()
        # Both commands can be received before the next native control tick.
        self.sock.sendall(b'MOVE 200\nS\n')
        self.assertTrue(self.stream.readline().startswith(b'OK'))
        self.assertEqual(self.stream.readline().strip(), b'OK')
        time.sleep(.04)
        self.assertEqual(self.status()['move'], 'aborted')
        self.assertEqual(self.status()['move_why'], 'stop')

    def test_phone_bridge_requires_new_press_after_denial(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'iter2-autonomy' / 'dashboard'))
        from control import MotorLink
        link = MotorLink(self.tcp)
        done = threading.Event()
        state = {'permit': True, 'generation': 1}
        def refresh():
            while not done.wait(.01):
                self.send(int(state['permit']))
                link.update(.3, .3, state['generation'])
        producer = threading.Thread(target=refresh)
        worker = threading.Thread(target=link.run)
        producer.start(); worker.start()
        def wait_for(predicate):
            end = time.monotonic() + 1
            while time.monotonic() < end:
                if predicate(): return
                time.sleep(.01)
            self.fail('Bridge state did not converge: ' + str(link.snapshot()))
        def moving():
            return float(self.command('STATUS').split()[1]) > .05
        try:
            wait_for(lambda: link.snapshot()['connected'])
            state['generation'] = 2
            wait_for(moving)
            state['permit'] = False
            time.sleep(.035)
            state['permit'] = True
            time.sleep(.12)
            self.assertFalse(moving())
            self.assertEqual(link.snapshot()['reason'], 'new_command_required')
            state['generation'] = 3
            wait_for(moving)
        finally:
            done.set(); link.stop.set(); producer.join(2); worker.join(2)

    def test_shift_only_bypasses_a_classified_obstacle(self):
        key = 'KD ' + 'k'*32 + ' w'
        def obstacle():
            stamp = time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000
            self.sender.sendto(struct.pack('!4sQB', b'GOB1', stamp, 0), ('127.0.0.1', self.udp))
        def wheels():
            return list(map(float, self.command('STATUS').split()[1:3]))

        self.assertTrue(self.command(key).startswith('ERR'))
        self.send(0); time.sleep(.03)
        self.assertTrue(self.command(key).startswith('ERR'))
        obstacle(); time.sleep(.03)
        self.assertTrue(self.command('KD invalid w').startswith('ERR'))
        self.assertTrue(self.command('D .25 .25').startswith('ERR'))
        self.assertEqual(self.command(key), 'OK')
        time.sleep(.04)
        self.assertGreater(min(wheels()), 0)
        # Continued obstacle reports cannot renew a released keyboard lease.
        for _ in range(6):
            obstacle(); time.sleep(.04)
        self.assertEqual(wheels(), [0, 0])
        self.assertEqual(self.command(key), 'OK')
        self.assertEqual(self.command('S'), 'OK')
        time.sleep(.03)
        self.assertEqual(wheels(), [0, 0])
        self.assertEqual(self.command(key), 'OK')
        self.send(0); time.sleep(.03)
        self.assertEqual(wheels(), [0, 0])
        self.assertTrue(self.command(key).startswith('ERR'))
        time.sleep(.12)
        self.assertTrue(self.command(key).startswith('ERR'))

    def test_nonfinite_drive_rejected(self):
        self.clear()
        for command in ('D inf 0', 'D 0 -inf', 'D nan 0', 'MOVE inf'):
            self.assertTrue(self.command(command).startswith('ERR'), command)


class PivotLimitTests(GateHarness):
    """Option B: an in-place TURN under a fresh, valid guard_blocked is limited to
    speed <= 0.40 (clamped) and |deg| <= 180 (refused above). CLEAR is unchanged."""
    CAP = 0.40

    def setUp(self):
        super().setUp()
        self.mode = None               # None = send nothing, 1 = clear, 0 = blocked, 'bad' = invalid
        self.stop_feed = threading.Event()
        self.feeder = threading.Thread(target=self._feed, daemon=True)
        self.feeder.start()

    def tearDown(self):
        self.stop_feed.set(); self.feeder.join(1)
        super().tearDown()

    def _feed(self):
        while not self.stop_feed.wait(.02):
            mode = self.mode
            if mode == 'bad':
                self.sender.sendto(b'invalid', ('127.0.0.1', self.udp))
            elif mode is not None:
                self.send(mode)

    def gate(self, mode, reason):
        self.mode = mode
        end = time.monotonic() + 1
        while time.monotonic() < end:
            if self.status()['guard_reason'] == reason:
                return
            time.sleep(.01)
        self.fail(f'guard never read {reason}: {self.status()}')

    def turn(self, deg, speed=None):
        return self.command(f'TURN {deg}' + (f' {speed}' if speed is not None else ''))

    def est_ms(self, reply):
        self.assertTrue(reply.startswith('OK'), reply)
        return int(reply.split()[2])

    def wheels(self):
        return [float(x) for x in self.command('STATUS').split()[1:3]]

    def test_blocked_turn_clamped_to_cap(self):
        self.gate(1, 'clear')
        ref = self.est_ms(self.turn(90, self.CAP))       # same TURN at the cap while CLEAR
        self.command('S')
        self.gate(0, 'guard_blocked')
        r = self.turn(90, 1.0)
        self.assertEqual(self.est_ms(r), ref, 'speed 1.0 under BLOCKED must run as 0.40')
        st = self.status()
        self.assertEqual((st['move'], st['move_kind']), ('running', 'TURN'))
        time.sleep(.1)                                   # and the clamped pivot keeps running
        self.assertEqual(self.status()['move'], 'running')

    def test_blocked_turn_over_180_refused_motors_untouched(self):
        self.gate(0, 'guard_blocked')
        before = self.status()
        r = self.turn(181, .3)
        self.assertEqual(r, 'ERR guard_blocked pivot>180')
        self.assertTrue(self.turn(-270).startswith('ERR guard_blocked pivot>180'))
        st = self.status()
        self.assertEqual((st['move'], st['move_id']), (before['move'], before['move_id']))
        self.assertEqual(self.wheels(), [0, 0])

    def test_blocked_turn_minus_180_ok(self):
        self.gate(0, 'guard_blocked')
        self.assertTrue(self.turn(-180, .3).startswith('OK'))
        time.sleep(.06)
        self.assertEqual(self.status()['move'], 'running')

    def test_stale_missing_invalid_still_refuse_turn(self):
        self.assertTrue(self.turn(90, .3).startswith('ERR guard_missing'))
        self.gate(0, 'guard_blocked')
        self.gate(None, 'guard_stale')
        self.assertTrue(self.turn(90, .3).startswith('ERR guard_stale'))
        self.gate('bad', 'guard_invalid')
        self.assertTrue(self.turn(90, .3).startswith('ERR guard_invalid'))

    def test_clear_turn_unchanged(self):
        self.gate(1, 'clear')
        fast = self.est_ms(self.turn(270, 1.0))
        self.command('S')
        slow = self.est_ms(self.turn(270, self.CAP))
        self.assertLess(fast, slow, 'CLEAR: speed 1.0 must not be clamped')
        time.sleep(.06)
        self.assertEqual(self.status()['move'], 'running')

    def test_running_fast_turn_aborts_when_blocked(self):
        self.gate(1, 'clear')
        self.assertTrue(self.turn(90, 1.0).startswith('OK'))
        epoch = self.status()['guard_epoch']
        self.gate(0, 'guard_blocked')
        time.sleep(.04)
        st = self.status()
        self.assertEqual((st['move'], st['move_why']), ('aborted', 'lidar_guard'))
        self.assertNotEqual(st['guard_epoch'], epoch)

    def test_running_long_turn_aborts_when_blocked(self):
        self.gate(1, 'clear')
        self.assertTrue(self.turn(270, self.CAP).startswith('OK'))
        self.gate(0, 'guard_blocked')
        time.sleep(.04)
        st = self.status()
        self.assertEqual((st['move'], st['move_why']), ('aborted', 'lidar_guard'))

    def test_running_slow_turn_continues_when_blocked(self):
        self.gate(1, 'clear')
        self.assertTrue(self.turn(-180, self.CAP).startswith('OK'))
        self.gate(0, 'guard_blocked')
        time.sleep(.1)
        self.assertEqual(self.status()['move'], 'running')

    def test_drive_and_move_still_refused_when_blocked(self):
        self.gate(0, 'guard_blocked')
        self.assertTrue(self.command('D .2 .2').startswith('ERR guard_blocked'))
        self.assertTrue(self.command('D -.2 .2').startswith('ERR guard_blocked'))
        epoch = self.status()['guard_epoch']
        self.assertTrue(self.command(f'DL {epoch} .2 .2').startswith('ERR'))
        self.assertTrue(self.command('MOVE 100 .3').startswith('ERR guard_blocked'))
        self.assertTrue(self.command('MOVE -100 .3').startswith('ERR guard_blocked'))
        self.assertEqual(self.wheels(), [0, 0])

    def test_stop_ends_blocked_pivot(self):
        self.gate(0, 'guard_blocked')
        self.assertTrue(self.turn(90, 1.0).startswith('OK'))
        self.assertEqual(self.command('S'), 'OK')
        time.sleep(.04)
        st = self.status()
        self.assertEqual((st['move'], st['move_why']), ('aborted', 'stop'))

    def test_tipped_refuses_blocked_pivot(self):
        self.gate(0, 'guard_blocked')
        self.assertEqual(self.command('SIM T 40'), 'OK')
        time.sleep(.04)
        self.assertTrue(self.turn(90, .3).startswith('ERR tipped'))


if __name__ == '__main__':
    unittest.main()
