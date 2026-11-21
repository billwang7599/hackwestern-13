"""Read-only aggregation of independent guard and inference results."""
import json
import select
import socket
import threading
import time


class EdgeStatus:
    def __init__(self):
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = {}

    def run(self):
        sockets = {}
        last_report = 0.0
        try:
            for name, port in (('safety', 5564), ('ai', 5565), ('depth', 5567)):
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
                sock.bind(('127.0.0.1', port))
                sock.setblocking(False)
                sockets[sock] = name
            while not self.stop.is_set():
                readable, _, _ = select.select(list(sockets), [], [], 0.1)
                for sock in readable:
                    data, _ = sock.recvfrom(65535)
                    try:
                        value = json.loads(data)
                        if not isinstance(value, dict):
                            continue
                        # Re-encode once to reject non-finite data before status publication.
                        json.dumps(value, allow_nan=False)
                        with self.lock:
                            self.latest[sockets[sock]] = (value, time.monotonic())
                    except (ValueError, UnicodeError, RecursionError):
                        continue
                if time.monotonic() - last_report >= 5:
                    last_report = time.monotonic()
                    snapshot = self.snapshot()
                    guard, ai, depth = snapshot['safety'], snapshot['ai'], snapshot['depth']
                    print('Edge status: ' + json.dumps({
                        'guard': guard.get('verdict'), 'guard_frames': guard.get('frames'),
                        'lidar_verdict': guard.get('lidar_verdict'), 'stop_source': guard.get('stop_source'),
                        'nearest_obstacle_m': guard.get('nearest_obstacle_m'),
                        'inferred_live': guard.get('inferred_live'),
                        'inferred_nearest_m': guard.get('inferred_nearest_m'),
                        'sample_age_ms': guard.get('sample_age_ms'),
                        'tick_hz': guard.get('tick_hz'), 'missed_ticks': guard.get('missed_ticks'),
                        'max_lateness_ms': guard.get('max_lateness_ms'),
                        'ai_live': ai.get('live'), 'ai_frame': ai.get('frame_id'),
                        'objects': len(ai.get('objects', [])), 'ai_ms': ai.get('inference_ms'),
                        'ai_error': ai.get('error'),
                        'depth_live': depth.get('live'), 'depth_frame': depth.get('frame_id'),
                        'depth_ms': depth.get('inference_ms'), 'depth_error': depth.get('error')
                    }, allow_nan=False), flush=True)
        finally:
            for sock in sockets:
                sock.close()

    def snapshot(self):
        now = time.monotonic()
        result = {}
        with self.lock:
            for name, timeout in (('safety', 0.3), ('ai', 3), ('depth', 6)):
                value, received = self.latest.get(name, ({}, 0))
                value = dict(value)
                fresh = bool(received) and now - received < timeout
                value['live'] = fresh and value.get('live', True)
                value['status_age_ms'] = round((now - received) * 1000, 1) if received else None
                if name == 'safety':
                    if not fresh:
                        value.update(reason='guard_unavailable', verdict='STALE', permit=False, lidar_verdict='STALE',
                                     stop_source='lidar', inferred_live=False, inferred_nearest_m=None,
                                     nearest_obstacle_m=None, nearest_m=None)
                    if not value['live'] or value.get('connected') is not True or value.get('verdict') not in ('CLEAR', 'CAUTION'):
                        value['permit'] = False
                result[name] = value
        return result
