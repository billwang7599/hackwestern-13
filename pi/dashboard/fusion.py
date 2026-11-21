"""Optional short-lived inferred-obstacle veto for the native LiDAR guard.

This channel can only add an obstacle. Silence/expiry removes its contribution;
the independent LiDAR guard still owns measured clearance and freshness.
"""
import math
import socket
import struct
import time


class FusionPublisher:
    def __init__(self, port=0, clock=time.monotonic):
        self.port = port
        self.clock = clock
        self.sock = None

    def publish(self, contribution):
        if not self.port:
            return
        now = self.clock()
        nearest, expiry = contribution['nearest_m'], contribution['expires']
        if not math.isfinite(nearest) or not math.isfinite(expiry):
            nearest, expiry = -1.0, 0.0
        if expiry <= now:
            nearest, expiry = -1.0, 0.0
        else:
            expiry = min(expiry, now + .4)
        packet = struct.pack('!4sQQf', b'FUS1', int(now * 1_000_000),
                             int(expiry * 1_000_000), nearest)
        try:
            if self.sock is None:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.setblocking(False)
            self.sock.sendto(packet, ('127.0.0.1', self.port))
        except OSError:
            # Inference is optional. A failed send must never stall sensor IO.
            pass

    def clear(self):
        self.publish({'nearest_m': -1.0, 'expires': 0.0})

    def close(self):
        self.clear()
        if self.sock is not None:
            self.sock.close()
            self.sock = None
