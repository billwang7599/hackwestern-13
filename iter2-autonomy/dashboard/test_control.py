import unittest
import json
import socket
import threading
from control import ControlLink, DebugDrive, parse_command


class CommandTests(unittest.TestCase):
    def test_status_and_fragmented_command(self):
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            server.settimeout(2)
            link = ControlLink(server.getsockname()[1])
            link.publish({'type': 'status', 'motor_mode': 'simulation',
                          'surroundings': {'grid': '0120' * 10000}})
            thread = threading.Thread(target=link.run)
            thread.start()
            try:
                conn, _ = server.accept()
                with conn:
                    conn.settimeout(2)
                    with conn.makefile('rb') as stream:
                        status = json.loads(stream.readline())
                        self.assertEqual(status['motor_mode'], 'simulation')
                        self.assertEqual(len(status['surroundings']['grid']), 40000)
                    conn.sendall(b'{"op":')
                    conn.sendall(b'"stop"}\n')
                    self.assertEqual(link.commands.get(timeout=2), {'op': 'stop'})
            finally:
                link.stop.set()
                thread.join(timeout=3)
            self.assertFalse(thread.is_alive())

    def test_invalid_commands(self):
        for value in (None, {}, {'op': 'shell'}, {'op': 'drive', 'left': float('nan'), 'right': 0},
                      {'op': 'drive', 'left': True, 'right': 0}, {'op': 'drive', 'left': 2, 'right': 0}):
            with self.assertRaises(ValueError):
                parse_command(value)

    def test_map_commands(self):
        for op in ('map_save', 'map_load', 'map_clear', 'control_phone'):
            self.assertEqual(parse_command({'op': op}), {'op': op})
        self.assertEqual(parse_command({'op': 'select_target', 'id': '12'}),
                         {'op': 'select_target', 'id': '12'})
        for target in ('', 'x' * 65, None, 3, []):
            with self.assertRaises(ValueError):
                parse_command({'op': 'select_target', 'id': target})

    def test_voice_commands(self):
        self.assertEqual(parse_command({'op': 'voice', 'text': ' turn left '}), {'op': 'voice', 'text': 'turn left'})
        for text in ('', '   ', 'x' * 201, 'stop\nS', None, 3):
            with self.assertRaises(ValueError):
                parse_command({'op': 'voice', 'text': text})

    def test_deadman_and_disconnect(self):
        drive = DebugDrive()
        command = parse_command(dict(op='drive', left=0.2, right=-0.2))
        drive.command(command, 10)
        self.assertEqual(drive.state(10.4, True), (1.0, -1.0))  # phone buttons = full-power intent (Pi field change)
        self.assertEqual(drive.state(10.51, True), (0, 0))
        drive.command(command, 11)
        self.assertEqual(drive.state(11.1, False), (0, 0))
        drive.command(command, 12)
        drive.command({'op': 'halt'}, 12.1)
        self.assertEqual(drive.state(12.2, True), (0, 0))


if __name__ == '__main__':
    unittest.main()
