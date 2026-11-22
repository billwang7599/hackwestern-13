"""Read-only web view of the status the Pi sends the phone: the room map (/) and every
telemetry value the phone app shows (/telemetry, data at /status.json).

Loopback only by default. TARS_WEB_TOKEN (12+ characters) opens it on all interfaces for a
laptop on the same Wi-Fi: http://<pi-ip>:5580/telemetry?token=<token> (sets a cookie).
Nothing here can drive or change the robot.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import json
import os
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlsplit

PORT = 5580
PAGE = Path(__file__).with_name('room_view.html')
TELEMETRY = Path(__file__).with_name('telemetry.html')
TOKEN = os.environ.get('TARS_WEB_TOKEN', '')
if len(TOKEN) < 12:
    TOKEN = ''


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
        def authorized(self, query):
            if not TOKEN:
                return True
            given = (query.get('token') or [''])[0]
            for part in self.headers.get('Cookie', '').split(';'):
                name, _, value = part.strip().partition('=')
                if name == 'tars_web' and not given:
                    given = value
            return hmac.compare_digest(given.encode(), TOKEN.encode())

        def do_GET(self):
            url = urlsplit(self.path)
            query = parse_qs(url.query)
            if not self.authorized(query):
                self.send_error(401, 'open with ?token=<TARS_WEB_TOKEN>')
                return
            set_cookie = bool(TOKEN and query.get('token'))
            self.path = url.path
            if self.path == '/':
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
            if set_cookie:
                self.send_header('Set-Cookie', f'tars_web={TOKEN}; HttpOnly; SameSite=Strict; Path=/')
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    while not stop.is_set():
        try:
            with ThreadingHTTPServer(('0.0.0.0' if TOKEN else '127.0.0.1', PORT), Handler) as server:
                server.timeout = 0.5
                while not stop.is_set():
                    server.handle_request()
        except OSError:
            stop.wait(2)
