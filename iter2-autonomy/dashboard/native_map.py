"""Bounded transport/display adapter for goto's authoritative native map.
No occupancy classification, path planning, or motor commands live here.
"""
import hashlib
import json
import math
import socket
import threading
import time

SIZE = 200
MAX_REPLY = 65536


def validate_snapshot(value):
    def number(x):
        return type(x) in (int, float) and math.isfinite(x) and abs(x) <= 1e12
    if not isinstance(value, dict) or value.get('source') != 'native-nav':
        raise ValueError('Not a native navigation snapshot')
    if value.get('size') != SIZE or value.get('resolution') != .1 or value.get('origin') != [-10, -10]:
        raise ValueError('Unsupported native grid geometry')
    grid = value.get('grid')
    if not isinstance(grid, str) or len(grid) != SIZE * SIZE or set(grid) - set('012'):
        raise ValueError('Invalid native grid')
    for key in ('instance', 'epoch', 'version', 'path_index'):
        if type(value.get(key)) is not int or not 0 <= value[key] <= 2**63-1:
            raise ValueError('Invalid native revision')
    for key in ('live', 'floor_ready'):
        if type(value.get(key)) is not bool: raise ValueError('Invalid native readiness')
    for key in ('floor_y', 'feed_age_s', 'robot_radius'):
        if not number(value.get(key)): raise ValueError('Invalid native metric')
    if not 0 <= value['robot_radius'] <= 2: raise ValueError('Invalid robot footprint')
    pose, path = value.get('pose'), value.get('path')
    if not isinstance(pose, list) or len(pose) != 4 or not all(map(number, pose)):
        raise ValueError('Invalid native pose')
    if not isinstance(path, list) or len(path) > 256:
        raise ValueError('Invalid native route')
    if any(not isinstance(p, list) or len(p) != 2 or not all(map(number, p)) for p in path):
        raise ValueError('Invalid native waypoint')
    if value['path_index'] > len(path): raise ValueError('Invalid route progress')
    if not isinstance(value.get('state'), str) or len(value['state']) > 64:
        raise ValueError('Invalid native state')
    session = value.get('session')
    if not isinstance(session, str) or len(session) != 64 or set(session) - set('0123456789abcdef'):
        raise ValueError('Invalid native session')
    return value


class NativeGrid:
    def __init__(self, port=5570, clock=time.monotonic):
        self.port, self.clock = port, clock
        self.lock = threading.Lock()
        self.io_lock = threading.Lock()
        self.session = ''
        self.reset_requested = self.reset_done = 0
        self.latest = None
        self.received = -math.inf
        self.error = 'Waiting for native navigation map'

    def set_session(self, session):
        token = hashlib.sha256(session.encode()).hexdigest()
        with self.lock:
            if token != self.session:
                self.session, self.latest = token, None

    def reset(self):
        with self.lock:
            self.reset_requested += 1
            self.latest = None

    def poll(self):
        # A stream restart can briefly overlap two display workers. Serialize I/O.
        with self.io_lock:
            with self.lock:
                token, reset = self.session, self.reset_requested
                reset_done = self.reset_done
            if not token: return
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=.3) as sock:
                    sock.settimeout(.3)
                    with sock.makefile('rb') as stream:
                        def query(command):
                            sock.sendall(command.encode('ascii') + b'\n')
                            line = stream.readline(MAX_REPLY + 1)
                            if len(line) > MAX_REPLY or not line.endswith(b'\n'):
                                raise ValueError('Truncated/oversized native map response')
                            return line.decode('ascii').strip()
                        if query('NAV SESSION ' + token) != 'OK session':
                            raise ValueError('Native map session unavailable')
                        if reset != reset_done:
                            if query('NAV RESET') != 'OK reset': raise ValueError('Native reset failed')
                        value = validate_snapshot(json.loads(query('NAV SNAPSHOT')))
                        if value['session'] != token: raise ValueError('Native map session mismatch')
                with self.lock:
                    if token == self.session and reset == self.reset_requested:
                        self.latest, self.received = value, self.clock()
                        self.reset_done, self.error = reset, ''
            except (OSError, ValueError, RecursionError) as error:
                with self.lock:
                    self.latest = None
                    self.error = 'Native navigation unavailable: ' + str(error)[:120]

    def run(self, stop):
        while not stop.is_set():
            self.poll()
            stop.wait(.2)

    def snapshot(self):
        with self.lock:
            value = self.latest
            age = self.clock() - self.received
            error = self.error
        if value is None or not 0 <= age <= 1:
            return dict(grid='0' * (SIZE * SIZE), size=SIZE, resolution=.1, origin=[-10., -10.],
                        path=[], live=False, floor_ready=False, native_map_live=False,
                        grid_source='native-nav', plan_status=error or 'Native navigation map is stale')
        live = value['live'] and value['floor_ready'] and 0 <= value['feed_age_s'] <= .5
        return dict(grid=value['grid'], size=SIZE, resolution=.1, origin=value['origin'],
                    pose=value['pose'], floor_y=value['floor_y'], floor_ready=value['floor_ready'],
                    floor_assumed=False, path=value['path'] if live else [], live=live,
                    tracking_live=live, native_map_live=live, grid_source='native-nav',
                    grid_version=value['version'], grid_epoch=value['epoch'], grid_instance=value['instance'],
                    path_index=value['path_index'], robot_radius=value['robot_radius'],
                    plan_status=('Native A*: ' + value['state']) if live else 'Waiting for fresh calibrated LiDAR')
