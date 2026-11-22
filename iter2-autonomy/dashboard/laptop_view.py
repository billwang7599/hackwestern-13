#!/usr/bin/env python3
"""Laptop telemetry viewer: python3 laptop_view.py [--pi 172.23.43.185] [--phone 172.23.43.117]
then open http://127.0.0.1:8080. No login, read-only.

Serves telemetry.html and its endpoints (/status.json, /map.json, /camera.mjpg):
  - Pi up:   proxies the Pi's room_view (port 5580): phone camera, YOLO boxes, LiDAR map, every status.
  - Pi down: dials the phone directly over Wi-Fi: camera (5558) and LiDAR (5562, LDR1) -> local floor map.
The phone serves one client per port, so the laptop only dials it while the Pi is unreachable
(a connection from the Pi takes the stream back automatically).
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import socket
import struct
import threading
import time
import urllib.request

HERE = Path(__file__).parent
RES = 0.05        # local map cell (m)
N = 160           # 8 m x 8 m around the phone


def read_exact(sock, size):
    data = bytearray()
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError('closed')
        data.extend(chunk)
    return bytes(data)


def read_packet(sock, limit):
    size, = struct.unpack('!I', read_exact(sock, 4))
    if not 4 <= size <= limit:
        raise ValueError('bad length')
    return read_exact(sock, size)


class State:
    def __init__(self, pi, phone):
        self.pi, self.phone = pi, phone
        self.lock = threading.Lock()
        self.pi_up = False
        self.pi_seen = 0.0
        self.jpeg, self.jpeg_t, self.frames, self.fps = None, 0.0, 0, 0.0
        self.cam_live = self.lidar_live = False
        self.pose, self.floor, self.lidar_t, self.packets = None, None, 0.0, 0
        self.cells = {}           # (i, j) -> 1 free / 2 obstacle, around a moving origin
        self.origin = None

    def direct(self):
        return not self.pi_up and time.monotonic() - self.pi_seen > 3

    # ---- Pi health ----
    def watch_pi(self):
        while True:
            try:
                with urllib.request.urlopen(f'http://{self.pi}:5580/status.json', timeout=1.5) as r:
                    r.read()
                self.pi_up, self.pi_seen = True, time.monotonic()
            except OSError:
                self.pi_up = False
            time.sleep(2)

    # ---- direct phone camera ----
    def watch_camera(self):
        last = time.monotonic(); count = 0
        while True:
            if not self.direct():
                self.cam_live = False; time.sleep(1); continue
            try:
                with socket.create_connection((self.phone, 5558), timeout=2) as sock:
                    sock.settimeout(3)
                    while self.direct():
                        jpeg = read_packet(sock, 2_000_000)
                        sock.sendall(b'\x01')
                        now = time.monotonic(); count += 1
                        if now - last >= 1:
                            self.fps, count, last = count / (now - last), 0, now
                        with self.lock:
                            self.jpeg, self.jpeg_t, self.cam_live = jpeg, now, True
                            self.frames += 1
            except (OSError, ValueError):
                self.cam_live = False
                time.sleep(1)

    # ---- direct phone LiDAR (LDR1) ----
    def watch_lidar(self):
        while True:
            if not self.direct():
                self.lidar_live = False; time.sleep(1); continue
            try:
                with socket.create_connection((self.phone, 5562), timeout=2) as sock:
                    sock.settimeout(3)
                    while self.direct():
                        data = read_packet(sock, 12_332)
                        sock.sendall(b'\x01')
                        self.ingest(data)
            except (OSError, ValueError, struct.error):
                self.lidar_live = False
                time.sleep(1)

    def ingest(self, data):
        if data[:4] != b'LDR1':
            raise ValueError('not LDR1')
        _seq, _ts, flags, n = struct.unpack('!IQII', data[4:24])
        x, y, z, yaw, floor = struct.unpack('!5f', data[24:44])
        pts = struct.unpack(f'!{3 * n}f', data[44:44 + 12 * n]) if n else ()
        floor_y = floor if flags & 2 else None
        if floor_y is None and pts:                       # no Set floor yet: lowest decile of the points
            ys = sorted(pts[1::3]); floor_y = ys[len(ys) // 10]
        with self.lock:
            self.pose, self.floor, self.lidar_t, self.lidar_live = [x, y, z, yaw], floor_y, time.monotonic(), True
            self.packets += 1
            if self.origin is None or abs(x - self.origin[0]) > 2 or abs(z - self.origin[1]) > 2:
                ox, oz = x - N * RES / 2, z - N * RES / 2
                if self.origin is not None:                # re-centre, keep what's still in range
                    di, dj = round((ox - self.origin[0]) / RES), round((oz - self.origin[1]) / RES)
                    self.cells = {(i - di, j - dj): v for (i, j), v in self.cells.items()
                                  if 0 <= i - di < N and 0 <= j - dj < N}
                    ox, oz = self.origin[0] + di * RES, self.origin[1] + dj * RES
                self.origin = (ox, oz)
            for k in range(0, len(pts), 3):
                px, py, pz = pts[k], pts[k + 1], pts[k + 2]
                i, j = int((px - self.origin[0]) / RES), int((pz - self.origin[1]) / RES)
                if not (0 <= i < N and 0 <= j < N):
                    continue
                h = py - floor_y
                if 0.05 < h < 1.5:
                    self.cells[(i, j)] = 2
                elif h <= 0.05 and self.cells.get((i, j)) != 2:
                    self.cells[(i, j)] = 1

    # ---- endpoints in direct mode ----
    def status(self):
        now = time.monotonic()
        with self.lock:
            age = round((now - self.jpeg_t) * 1000) if self.jpeg_t else None
            lidar_age = now - self.lidar_t if self.lidar_t else None
            return {
                'publisher_live': True, 'view_source': 'phone direct (Pi offline)', 'phone_connected': self.cam_live or self.lidar_live,
                'motion_live': self.lidar_live, 'streaming': self.cam_live or self.lidar_live,
                'camera_live': self.cam_live, 'camera_fps': self.fps, 'camera_frame_age_ms': age,
                'frames': self.frames,
                'safety': {'verdict': 'PI OFFLINE', 'reason': 'Pi unreachable: direct phone view'},
                'motor': {'inhibited': True, 'reason': 'Pi offline'},
                'goto': {'live': False, 'nav': {'packets': self.packets, 'feed_age': lidar_age,
                         'pose': ' '.join(f'{v:.2f}' for v in self.pose[:3]) if self.pose else None}},
            }

    def map(self):
        with self.lock:
            if self.origin is None:
                return {}
            grid = ['0'] * (N * N)
            for (i, j), v in self.cells.items():
                grid[j * N + i] = str(v)
            return {'grid': ''.join(grid), 'size': N, 'resolution': RES, 'origin': list(self.origin),
                    'pose': self.pose, 'floor_ready': self.floor is not None, 'frames': self.packets,
                    'tracking': 'live' if self.lidar_live else 'waiting', 'grid_source': 'phone LiDAR (direct)'}


def serve(state, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, kind):
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def proxy(self, path):
            try:
                with urllib.request.urlopen(f'http://{state.pi}:5580{path}', timeout=2) as r:
                    if path.startswith('/camera.mjpg'):
                        self.send_response(200)
                        self.send_header('Content-Type', r.headers['Content-Type'])
                        self.send_header('Cache-Control', 'no-store')
                        self.end_headers()
                        while True:
                            chunk = r.read1(65536)
                            if not chunk:
                                return
                            self.wfile.write(chunk)
                    body = r.read()
                    if path == '/status.json':        # tag the source for the header banner
                        s = json.loads(body); s['view_source'] = 'Pi'; body = json.dumps(s).encode()
                    self.send(200, body, r.headers['Content-Type'] or 'application/json')
            except OSError:
                if path.startswith('/camera.mjpg'):
                    return
                self.send(502, b'{}', 'application/json')

        def mjpeg(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = None
            while state.direct():
                with state.lock:
                    jpeg = state.jpeg
                if jpeg is not None and jpeg is not last:
                    last = jpeg
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpeg)
                                     + jpeg + b'\r\n')
                time.sleep(0.03)

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            try:
                if path in ('/', '/telemetry'):
                    return self.send(200, (HERE / 'telemetry.html').read_bytes(), 'text/html; charset=utf-8')
                if path not in ('/status.json', '/map.json', '/camera.mjpg'):
                    return self.send(404, b'not found', 'text/plain')
                if state.pi_up:
                    return self.proxy(self.path)
                if path == '/status.json':
                    return self.send(200, json.dumps(state.status(), allow_nan=False).encode(), 'application/json')
                if path == '/map.json':
                    return self.send(200, json.dumps(state.map()).encode(), 'application/json')
                if state.direct():
                    return self.mjpeg()
                self.send(503, b'', 'text/plain')
            except (BrokenPipeError, ConnectionResetError):
                pass

    ThreadingHTTPServer.daemon_threads = True
    ThreadingHTTPServer(('0.0.0.0', port), Handler).serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--pi', default='172.23.43.185')
    ap.add_argument('--phone', default='172.23.43.117')
    ap.add_argument('--port', type=int, default=8080)
    a = ap.parse_args()
    state = State(a.pi, a.phone)
    for fn in (state.watch_pi, state.watch_camera, state.watch_lidar):
        threading.Thread(target=fn, daemon=True).start()
    print(f'Telemetry: http://127.0.0.1:{a.port}  (Pi {a.pi} when up, else phone {a.phone} direct)', flush=True)
    serve(state, a.port)


if __name__ == '__main__':
    main()
