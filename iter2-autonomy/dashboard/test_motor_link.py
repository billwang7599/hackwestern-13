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

    def test_missing_status_never_claims_clear(self):
        value = MotorLink().snapshot()
        self.assertFalse(value['connected'])
        self.assertTrue(value['inhibited'])
        self.assertEqual(value['output'], [0, 0])


if __name__ == '__main__': unittest.main()
