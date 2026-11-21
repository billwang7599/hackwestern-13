import time
import socket
import threading
import unittest
from control import MotorLink


class MotorBridgeTests(unittest.TestCase):
    def test_status_and_no_false_hardware_feedback(self):
        result = MotorLink.parse_status('OK .25 -.25 5 -1 0 why=none backend=stub guard_enabled=1 guard_permit=1 guard_age_ms=4 move=idle')
        self.assertEqual(result['output'], [.25, -.25])
        self.assertEqual(result['mode'], 'stub')
        self.assertTrue(result['guard_enabled'])
        for bad in ('OK nan 0 1 -1 0 why=none', 'OK inf 0 1 -1 0 why=none', 'ERR no'):
            with self.assertRaises(ValueError): MotorLink.parse_status(bad)

    def test_mailbox_lease_and_consumed_command(self):
        link = MotorLink()
        link.update(.2, .2, 1)
        self.assertEqual(link.desired(), ((.2, .2), 1))
        link.expires = time.monotonic() - 1
        self.assertEqual(link.desired(), ((0, 0), 1))
        link.update(.2, .2, 1)
        link.denied_generation = 1
        self.assertEqual(link.desired(), ((0, 0), 1))
        link.update(.2, .2, 2)
        self.assertEqual(link.desired(), ((.2, .2), 2))

    def test_unguarded_server_is_rejected_before_any_drive(self):
        commands = []
        finished = threading.Event()
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0)); server.listen(1); server.settimeout(2)
            link = MotorLink(server.getsockname()[1])
            def fake_server():
                conn, _ = server.accept()
                with conn, conn.makefile('rb') as stream:
                    conn.settimeout(2)
                    commands.append(stream.readline().decode().strip())
                    conn.sendall(b'OK\n')
                    commands.append(stream.readline().decode().strip())
                    conn.sendall(b'OK 0 0 0 -1 0 why=none backend=stub guard_enabled=0\n')
                finished.set()
            server_thread = threading.Thread(target=fake_server)
            worker = threading.Thread(target=link.run)
            server_thread.start(); worker.start()
            try:
                link.update(.3, .3, 1)
                self.assertTrue(finished.wait(2))
                self.assertEqual(commands, ['S', 'STATUS'])
            finally:
                link.stop.set(); worker.join(2); server_thread.join(2)
            self.assertFalse(worker.is_alive())

    def test_idle_release_explicit_stop_and_guard_denial(self):
        commands = []
        seen = threading.Condition()
        blocked = threading.Event()
        def wait_for(check):
            with seen:
                self.assertTrue(seen.wait_for(lambda: check(commands), timeout=2), commands[-12:])
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            link = MotorLink(server.getsockname()[1])
            link.watch_navigation = lambda: None
            def fake_server():
                first_drive = True
                previous_tick_idle = False
                conn, _ = server.accept()
                with conn, conn.makefile('rb') as stream:
                    for line in stream:
                        command = line.decode().strip()
                        with seen:
                            commands.append(command)
                            seen.notify_all()
                        reply = 'OK'
                        if command.startswith('DL '):
                            previous_tick_idle = first_drive
                            first_drive = False
                            if blocked.is_set():
                                reply = 'ERR guard_blocked'
                        if command == 'STATUS':
                            inhibit = blocked.is_set() or previous_tick_idle
                            why = 'guard_blocked' if blocked.is_set() else 'watchdog' if inhibit else 'none'
                            reply = f'OK 0 0 0 -1 {int(inhibit)} why={why} guard_enabled=1 guard_epoch=1'
                            previous_tick_idle = False
                        conn.sendall((reply + '\n').encode())
            peer = threading.Thread(target=fake_server)
            worker = threading.Thread(target=link.run)
            peer.start()
            worker.start()
            try:
                wait_for(lambda c: c.count('STATUS') >= 4)
                self.assertEqual(commands.count('S'), 1)
                # Refresh while held as the headless loop does.
                for _ in range(8):
                    link.update(.25, .25, 1)
                    time.sleep(.04)
                self.assertGreaterEqual(sum(c.startswith('DL ') for c in commands), 3)
                link.update(0, 0, 1)
                wait_for(lambda c: c.count('S') == 2)
                link.request_stop()
                wait_for(lambda c: c.count('S') == 3)
                blocked.set()
                link.update(.25, .25, 2)
                wait_for(lambda c: c.count('S') == 4)
                attempted = commands.count('DL 1 0.250 0.250')
                blocked.clear()
                link.update(.25, .25, 2)
                time.sleep(.16)
                self.assertEqual(commands.count('DL 1 0.250 0.250'), attempted)
                self.assertEqual(commands.count('S'), 4)
            finally:
                link.stop.set()
                worker.join(2)
                peer.join(2)

    def test_missing_status_never_claims_clear(self):
        value = MotorLink().snapshot()
        self.assertFalse(value['connected'])
        self.assertTrue(value['inhibited'])
        self.assertEqual(value['output'], [0, 0])


if __name__ == '__main__': unittest.main()
