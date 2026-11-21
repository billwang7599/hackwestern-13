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


class GateTests(unittest.TestCase):
    def setUp(self):
        self.tcp, self.udp = port(), port(socket.SOCK_DGRAM)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.proc = subprocess.Popen([BINARY, '-b', 'stub', '-l', '-g', str(self.udp),
                                      '-p', str(self.tcp)], stderr=subprocess.DEVNULL)
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
        sys.path.insert(0, str(Path(__file__).parent / 'dashboard'))
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

    def test_nonfinite_drive_rejected(self):
        self.clear()
        for command in ('D inf 0', 'D 0 -inf', 'D nan 0', 'MOVE inf'):
            self.assertTrue(self.command(command).startswith('ERR'), command)


if __name__ == '__main__':
    unittest.main()
