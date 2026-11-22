#!/usr/bin/env python3
"""A fake MotionUSB for bench tests: the phone's LDR1 safety stream (5562) and RGB1 camera stream (5563).

    python3 bench/phone/fake_phone.py --ldr 0 --rgb 0          # prints the ports; then
    python3 bench/run_sim.py --phone tcp:127.0.0.1 --phone-ports <ldr>,<rgb>

Wire formats as SceneCapture.swift sends them (one frame in flight, the client ACKs each with 0x01):
  LDR1: u32 BE length + "LDR1" u32 seq, u64 source_us, u32 flags (1 tracking | 2 floor | 4 heading),
        u32 n, f32 x y z yaw floor, then n x (f32 x y z)  -- ARKit frame, metres, big-endian
  RGB1: u32 BE length + "RGB1" u64 frame id + one JPEG
The scene: the phone 0.138 m above the floor at ARKit (x, z) = pose, facing yaw (0 = -z); floor points
0.3-1.4 m ahead and a wall 2.0 m ahead, `points` points in total (1024 = the phone's maximum, 12332 bytes:
over macOS's 9216-byte UDP datagram cap, which is why bench/phone/link.py tees LDR1 to goto over TCP).
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_JPEG = os.path.join(REPO, "iter2-autonomy", "edge-ai", "models", "dog.jpg")
CAM_H = 0.138


class FakePhone:
    def __init__(self, ldr_port=0, rgb_port=0, points=1024, hz=15.0, jpeg=DEFAULT_JPEG, fps=5.0):
        self.pose = [0.0, 0.0, 0.0]          # ARKit x, z, yaw (rad); tests may change it
        self.tracking = True
        self.points, self.hz, self.fps = points, hz, fps
        self.jpeg = open(jpeg, "rb").read()
        self.sent = {"ldr": 0, "rgb": 0, "ldr_bytes": 0}
        self.stop = threading.Event()
        self.ldr = self._listen(ldr_port)
        self.rgb = self._listen(rgb_port)
        self.ldr_port = self.ldr.getsockname()[1]
        self.rgb_port = self.rgb.getsockname()[1]
        threading.Thread(target=self._serve, args=(self.ldr, self._ldr_loop), daemon=True).start()
        threading.Thread(target=self._serve, args=(self.rgb, self._rgb_loop), daemon=True).start()

    @staticmethod
    def _listen(port):
        s = socket.socket()
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", port))
        s.listen(1)
        return s

    def _serve(self, srv, loop):
        while not self.stop.is_set():
            try:
                c, _ = srv.accept()
            except OSError:
                return
            c.settimeout(3)
            try:
                loop(c)
            except OSError:
                pass
            finally:
                c.close()

    def ldr_packet(self, seq: int) -> bytes:
        x, z, yaw = self.pose
        fx, fz = -math.sin(yaw), -math.cos(yaw)          # ARKit forward for this yaw
        rx, rz = math.cos(yaw), -math.sin(yaw)           # right
        pts = []
        if self.tracking:
            n_floor = self.points // 2
            for i in range(n_floor):
                d = 0.3 + 1.1 * (i % 32) / 31
                s = -0.5 + (i // 32) / max(1, n_floor // 32 - 1)
                pts.append((x + fx * d + rx * s, 0.0, z + fz * d + rz * s))
            for i in range(self.points - n_floor):
                s = -1.0 + 2.0 * (i % 64) / 63
                hgt = 0.05 + 0.45 * (i // 64) / max(1, (self.points - n_floor) // 64 - 1)
                pts.append((x + fx * 2.0 + rx * s, hgt, z + fz * 2.0 + rz * s))
        flags = 7 if self.tracking else 0
        body = b"LDR1" + struct.pack("!IQII", seq, int(time.monotonic() * 1e6), flags, len(pts))
        body += struct.pack("!5f", x, CAM_H, z, yaw, 0.0)
        body += b"".join(struct.pack("!3f", *p) for p in pts)
        return struct.pack("!I", len(body)) + body

    def _ldr_loop(self, c):
        seq = 0
        while not self.stop.is_set():
            seq += 1
            pkt = self.ldr_packet(seq)
            c.sendall(pkt)
            if c.recv(1) != b"\x01":
                return
            self.sent["ldr"] += 1
            self.sent["ldr_bytes"] = len(pkt)
            time.sleep(1.0 / self.hz)

    def _rgb_loop(self, c):
        fid = 0
        while not self.stop.is_set():
            fid += 1
            body = b"RGB1" + struct.pack("!Q", fid) + self.jpeg
            c.sendall(struct.pack("!I", len(body)) + body)
            if c.recv(1) != b"\x01":
                return
            self.sent["rgb"] += 1
            time.sleep(1.0 / self.fps)

    def close(self):
        self.stop.set()
        for s in (self.ldr, self.rgb):
            s.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="fake MotionUSB LDR1 + RGB1 streams")
    ap.add_argument("--ldr", type=int, default=0)
    ap.add_argument("--rgb", type=int, default=0)
    ap.add_argument("--points", type=int, default=1024)
    a = ap.parse_args(argv)
    ph = FakePhone(a.ldr, a.rgb, a.points)
    print(f"fake phone: LDR1 on {ph.ldr_port}, RGB1 on {ph.rgb_port} "
          f"(run_sim.py --phone tcp:127.0.0.1 --phone-ports {ph.ldr_port},{ph.rgb_port})", flush=True)
    try:
        while True:
            time.sleep(5)
            print(f"fake phone: sent {ph.sent}", flush=True)
    except KeyboardInterrupt:
        ph.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
