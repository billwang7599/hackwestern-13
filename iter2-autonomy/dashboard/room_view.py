"""Read-only loopback view of an existing phone dashboard snapshot."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time

PORT = 5580
PAGE = Path(__file__).with_name('room_view.html')


def serve_room(link, stop):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == '/':
                content, kind = PAGE.read_bytes(), 'text/html; charset=utf-8'
            elif self.path == '/room_controls.js':
                content, kind = PAGE.with_name('room_controls.js').read_bytes(), 'text/javascript; charset=utf-8'
            elif self.path == '/map.json':
                with link.lock:
                    latest = link.latest
                    stamp = link.room_published
                room = latest.get('surroundings', {})
                keys = ('session', 'frames', 'tracking_live', 'tracking', 'pose', 'origin', 'resolution',
                        'size', 'grid', 'path', 'objects', 'points3d', 'estimated3d', 'distant3d', 'distant_confidence',
                        'surface_splats', 'floor_y', 'voxels', 'scan_degrees')
                data = {k: room[k] for k in keys if k in room}
                data['publisher_live'] = stamp > 0 and time.monotonic()-stamp < 3
                for key in ('accel_m_s2', 'gyro_rad_s', 'motion_live', 'motion_hz'):
                    if key in latest:
                        data[key] = latest[key]
                content = json.dumps(data, separators=(',', ':'), allow_nan=False).encode()
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

    while not stop.is_set():
        try:
            with ThreadingHTTPServer(('127.0.0.1', PORT), Handler) as server:
                server.timeout = 0.5
                while not stop.is_set():
                    server.handle_request()
        except OSError:
            stop.wait(2)
