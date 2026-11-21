"""Bounded JPEG stream: uint32 big-endian length, JPEG, then client ACK 0x01."""
import socket
import struct
import threading
import time

MAX_FRAME = 2_000_000


def read_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError('Camera disconnected')
        data.extend(chunk)
    return bytes(data)


def read_frame(sock):
    size, = struct.unpack('!I', read_exact(sock, 4))
    if not 4 <= size <= MAX_FRAME:
        raise ValueError('Invalid camera frame length')
    jpeg = read_exact(sock, size)
    if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
        raise ValueError('Invalid JPEG envelope')
    return jpeg


class VideoFeed:
    def __init__(self, port):
        self.port = port
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sock = None
        self.pending = None
        self.count = 0
        self.received = 0.0
        self.status = 'Waiting for iPhone camera'

    def run(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=2) as sock:
                    with self.lock:
                        self.sock = sock
                    sock.settimeout(2)
                    while not self.stop.is_set():
                        jpeg = read_frame(sock)
                        ticket = {'jpeg': jpeg, 'done': threading.Event(), 'ok': False}
                        with self.lock:
                            self.pending = ticket
                        if not ticket['done'].wait(1.5) or not ticket['ok']:
                            raise ValueError('Camera frame could not be displayed')
                        sock.sendall(b'\x01')
                        with self.lock:
                            self.count += 1
                            self.received = time.monotonic()
                            self.status = 'LIVE / iPhone rear camera'
            except (OSError, ValueError) as error:
                with self.lock:
                    self.pending = None
                    self.status = str(error)
                self.stop.wait(1)

    def take(self):
        with self.lock:
            ticket, self.pending = self.pending, None
            return ticket
