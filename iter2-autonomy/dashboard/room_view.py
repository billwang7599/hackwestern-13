"""Read-only web view of what the phone and the Pi see, for a laptop on the same Wi-Fi:
http://<pi-ip>:5580/telemetry (everything the phone app shows + the phone camera + goto/detector),
/ (room map), /status.json, /map.json, /camera.mjpg, /camera.jpg. No login: nothing here can
drive or change the robot. TARS_WEB_LOCAL=1 keeps it on loopback.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import threading
import time
from urllib.parse import urlsplit

PORT = 5580
PAGE = Path(__file__).with_name('room_view.html')
TELEMETRY = Path(__file__).with_name('telemetry.html')
BIND = '127.0.0.1' if os.environ.get('TARS_WEB_LOCAL') == '1' else '0.0.0.0'
NAV = {}            # latest goto STATUS / DETS / NAV, polled by watch_goto()
NAV_LOCK = threading.Lock()


def _goto_lines(lines, port=5570, timeout=0.4):
    """One short read-only exchange with goto (STATUS / DETS / NAV); {} if it isn't running."""
    out = {}
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=timeout) as sock:
            sock.settimeout(timeout)
            stream = sock.makefile('rb')
            for line in lines:
                sock.sendall(line.encode() + b'\n')
                out[line] = stream.readline(65536).decode('utf-8', 'replace').strip()
    except OSError:
        pass
    return out


def _parse_goto(rep):
    result = {'live': bool(rep)}
    status = rep.get('STATUS', '')
    if status.startswith('OK '):
        result['status'] = dict(p.split('=', 1) for p in status[3:].split() if '=' in p)
    nav = rep.get('NAV', '')
    if nav.startswith('OK '):
        result['nav'] = dict(p.split('=', 1) for p in nav[3:].split() if '=' in p)
    dets = rep.get('DETS', '')
    if dets.startswith('OK age='):
        age, _, body = dets[7:].partition(' ')
        try:
            result['dets'] = json.loads(body) if body else None
            result['dets_age_s'] = float(age)
        except ValueError:
            pass
    return result


def watch_goto(stop):
    while not stop.wait(0.3):
        value = _parse_goto(_goto_lines(('STATUS', 'DETS', 'NAV')))
        with NAV_LOCK:
            NAV.clear()
            NAV.update(value)


def _json_safe(value):
    """Status values for the browser: floats that JSON can carry, nothing else changed."""
    if isinstance(value, float):
        return value if value == value and abs(value) != float('inf') else None
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def serve_room(link, stop):
    class Handler(BaseHTTPRequestHandler):
        def camera(self):
            frame = getattr(link, 'camera', None)
            return frame if frame and time.monotonic() - frame[1] < 3 else None

        def mjpeg(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            sent = 0.0
            try:
                while not stop.is_set():
                    frame = self.camera()
                    if frame and frame[1] != sent:
                        sent = frame[1]
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(frame[0])).encode() + b'\r\n\r\n' + frame[0] + b'\r\n')
                        self.wfile.flush()
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

        def do_GET(self):
            self.path = urlsplit(self.path).path
            if self.path == '/camera.mjpg':
                self.mjpeg()
                return
            if self.path == '/camera.jpg':
                frame = self.camera()
                if not frame:
                    self.send_error(503, 'no fresh phone camera frame')
                    return
                content, kind = frame[0], 'image/jpeg'
            elif self.path == '/':
                content, kind = PAGE.read_bytes(), 'text/html; charset=utf-8'
            elif self.path == '/telemetry':
                content, kind = TELEMETRY.read_bytes(), 'text/html; charset=utf-8'
            elif self.path == '/status.json':
                with link.lock:
                    latest = dict(link.latest)
                    stamp = link.room_published
                latest.pop('surroundings', None)   # the big map lives at /map.json
                latest['publisher_live'] = stamp > 0 and time.monotonic() - stamp < 3
                latest['phone_connected'] = bool(link.connected)
                frame = self.camera()
                latest['camera_frame_age_ms'] = round((time.monotonic() - frame[1]) * 1000) if frame else None
                with NAV_LOCK:
                    latest['goto'] = dict(NAV)
                content = json.dumps(_json_safe(latest), separators=(',', ':'), allow_nan=False).encode()
                kind = 'application/json'
            elif self.path == '/room_controls.js':
                content, kind = PAGE.with_name('room_controls.js').read_bytes(), 'text/javascript; charset=utf-8'
            elif self.path == '/map.json':
                with link.lock:
                    latest = link.latest
                    stamp = link.room_published
                room = latest.get('surroundings', {})
                keys = ('session', 'frames', 'tracking_live', 'tracking', 'pose', 'origin', 'resolution',
                        'size', 'grid', 'path', 'objects', 'points3d', 'estimated3d', 'distant3d', 'distant_confidence',
                        'surface_splats', 'floor_y', 'floor_assumed', 'floor_ready', 'camera_height_m',
                        'lidar_height_m', 'voxels', 'scan_degrees',
                        'grid_source', 'grid_version', 'grid_epoch', 'native_map_live', 'plan_status')
                data = {k: room[k] for k in keys if k in room}
                data['publisher_live'] = stamp > 0 and time.monotonic()-stamp < 3
                for key in ('accel_m_s2', 'gyro_rad_s', 'motion_live', 'motion_hz'):
                    if key in latest:
                        data[key] = latest[key]
                content = json.dumps(_json_safe(data), separators=(',', ':'), allow_nan=False).encode()
                kind = 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    threading.Thread(target=watch_goto, args=(stop,), daemon=True).start()
    while not stop.is_set():
        try:
            with ThreadingHTTPServer((BIND, PORT), Handler) as server:
                server.timeout = 0.5
                while not stop.is_set():
                    server.handle_request()
        except OSError:
            stop.wait(2)
