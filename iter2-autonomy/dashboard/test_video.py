import socket
import struct
import threading
import unittest
from video import MAX_FRAME, read_frame


class FramingTests(unittest.TestCase):
    def receive(self, chunks):
        a, b = socket.socketpair()
        def send():
            with a:
                for chunk in chunks:
                    a.sendall(chunk)
        worker = threading.Thread(target=send)
        worker.start()
        try:
            with b:
                b.settimeout(1)
                return read_frame(b)
        finally:
            worker.join()

    def test_fragmented_frame(self):
        jpeg = b'\xff\xd8abc\xff\xd9'
        packet = struct.pack('!I', len(jpeg)) + jpeg
        self.assertEqual(self.receive([bytes([x]) for x in packet]), jpeg)

    def test_oversized_frame(self):
        with self.assertRaises(ValueError):
            self.receive([struct.pack('!I', MAX_FRAME + 1)])

    def test_truncated_frame(self):
        with self.assertRaises(ConnectionError):
            self.receive([struct.pack('!I', 10), b'\xff\xd8'])

    def test_invalid_jpeg(self):
        with self.assertRaises(ValueError):
            self.receive([struct.pack('!I', 4), b'nope'])


if __name__ == '__main__':
    unittest.main()
