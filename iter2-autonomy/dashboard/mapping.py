"""Bounded ARKit world-point mapping and path previews. No motor control.

The grid is a conservative top-down projection of observed ground endpoints
and obstacles, not a certified traversability map. Unknown cells are blocked.
World axes are ARKit x/y/z (y up); grid rows increase with world z.
"""
from collections import OrderedDict
import base64
import heapq
import json
import math
from pathlib import Path
import socket
import struct
import threading
import time
from fusion import FusionPublisher
from native_map import NativeGrid

MAX_PACKET = 512 * 1024
SIZE = 200
RESOLUTION = 0.1
ORIGIN = (-10.0, -10.0)
MAX_VOXELS = 30000
MAX_OBJECTS = 100
FRESH_SECONDS = 1.0
TARGET_FRESH_SECONDS = 5.0
INFERENCE_SECONDS = 2.0
MAX_INFERRED_VOXELS = 1000
MAX_PERSISTENT_VOXELS = 5000
PERSISTENT_SECONDS = 120.0
PERSISTENT_RANGE = 20.0
OBJECT_VISIBLE_SECONDS = 1.5
# iPhone 17 Pro Max, power-button edge up. Measured from the midpoint of that
# edge in the rear-face plane. Coordinates are ARKit landscape +X towards USB,
# +Y towards the power edge. Apple mechanical drawing, sheet 1, detail D (mm).
MOUNT_EDGE_HEIGHT = 0.1524
CAMERA_FROM_EDGE = ((33.61 - 163.43 / 2) / 1000, -0.01437)
LIDAR_FROM_EDGE = ((34.16 - 163.43 / 2) / 1000, -0.06416)


def mount_heights(columns):
    # Older senders may omit the attitude. Use the upright mounting estimate.
    up_x, up_y = 0., 1.
    if isinstance(columns, list) and len(columns) == 4 and all(_vector(c, 4) for c in columns):
        vertical = [columns[i][1] for i in range(3)]
        if abs(sum(v*v for v in vertical) - 1) < .01:
            up_x, up_y = vertical[:2]
    return tuple(MOUNT_EDGE_HEIGHT + x*up_x + y*up_y for x,y in (CAMERA_FROM_EDGE, LIDAR_FROM_EDGE))
ROBOT_RADIUS = 0.35  # Provisional footprint for path preview only.
MIN_HEIGHT, MAX_HEIGHT = 0.08, 1.2


def _number(value, limit=1000):
    return type(value) in (int, float) and abs(value) <= limit and math.isfinite(value)


def _vector(value, length):
    return isinstance(value, list) and len(value) == length and all(_number(x) for x in value)


def _calibrated_observation(scene):
    """Return original phone observation time only for a usable metric fit."""
    fit = scene.get('depth_calibration')
    if not isinstance(fit, dict):
        return None
    scale, offset = fit.get('scale'), fit.get('offset')
    error, samples = fit.get('heldout_p90_m'), fit.get('samples')
    observed = fit.get('observation_timestamp')
    if (not _number(scale, 1e9) or scale <= 0 or not _number(offset, 1e9)
            or not _number(error, 1) or not 0 <= error <= 0.25
            or type(samples) is not int or not 40 <= samples <= 100000
            or not _number(observed, 1e12) or observed < 0
            or not 0 <= scene['timestamp'] - observed < INFERENCE_SECONDS):
        return None
    return float(observed)


def _persistent_observation(scene):
    """Uncertain visual memory has its own metadata, never a safety fit."""
    fit = scene.get('persistent_calibration')
    if (not isinstance(fit, dict) or fit.get('mode') not in ('anchors', 'cached', 'direct')
            or fit.get('usage') != 'persistent_map' or fit.get('visualization_only') is not True):
        return None
    observed, confidence, anchor_age = (fit.get('observation_timestamp'),
                                        fit.get('confidence'), fit.get('anchor_age_s'))
    if (not _number(observed, 1e12) or observed < 0
            or not 0 <= scene['timestamp'] - observed < PERSISTENT_SECONDS
            or not _number(confidence, 1) or not 0 < confidence <= 1
            or not _number(anchor_age, 30) or anchor_age < 0):
        return None
    calibrated = fit.get('calibration_timestamp')
    if (not _number(calibrated, 1e12) or not 0 <= calibrated <= observed
            or (fit['mode'] == 'cached' and observed - calibrated > 3)):
        return None
    for key in ('scale', 'offset', 'heldout_p90_m', 'heldout_median_m'):
        if not _number(fit.get(key), 1e9):
            return None
    if (fit['scale'] <= 0 or not 0 <= fit['heldout_p90_m'] <= .25
            or not 0 <= fit['heldout_median_m'] <= fit['heldout_p90_m']
            or type(fit.get('samples')) is not int or not 40 <= fit['samples'] <= 100000):
        return None
    return float(observed), float(confidence), fit['mode'], float(calibrated)


class _MeasuredRays:
    """Sparse same-ray free-volume evidence; never clears a whole floor column."""
    ANGLE = 0.01
    RADIUS = 0.025

    def __init__(self, origin, points):
        self.origin = origin[:3]
        self.bins = {}
        for point in points:
            delta = tuple(point[i] - origin[i] for i in range(3))
            distance = math.sqrt(sum(v*v for v in delta))
            if not 0.2 <= distance <= 5:
                continue
            direction = tuple(v / distance for v in delta)
            key = self._key(direction)
            self.bins.setdefault(key, []).append((direction, distance))

    @classmethod
    def _key(cls, direction):
        return (math.floor(math.atan2(direction[0], direction[2]) / cls.ANGLE),
                math.floor(math.asin(max(-1, min(1, direction[1]))) / cls.ANGLE))

    def clears(self, point):
        delta = tuple(point[i] - self.origin[i] for i in range(3))
        squared = sum(v*v for v in delta)
        if squared < 0.04:
            return False
        distance = math.sqrt(squared)
        key = self._key(tuple(v / distance for v in delta))
        # At least one measured ray must pass within 2.5 cm of the estimate,
        # with its measured endpoint at least 10 cm behind it. Sparse rays may
        # miss a phantom, in which case the two-second inference TTL wins.
        reach = min(14, math.ceil(self.RADIUS / max(distance, 0.2) / self.ANGLE) + 1)
        for dx in range(-reach, reach+1):
            for dy in range(-reach, reach+1):
                for direction, end in self.bins.get((key[0]+dx, key[1]+dy), ()):
                    along = sum(delta[i] * direction[i] for i in range(3))
                    if (0.1 < along < end - 0.1
                            and max(0, squared - along*along) <= self.RADIUS*self.RADIUS):
                        return True
        return False


SURFACE_LIMIT = 8000
SURFACE_RECORD = struct.Struct('<3h3b4B')


def valid_surfaces(rows, limit):
    return (isinstance(rows, list) and len(rows) <= limit and
            all(isinstance(row, list) and len(row) == 10 and all(type(v) is int for v in row)
                and all(abs(v) <= 30000 for v in row[:3])
                and all(abs(v) <= 127 for v in row[3:6])
                and 120**2 <= sum(v*v for v in row[3:6]) <= 130**2
                and all(0 <= v <= 255 for v in row[6:9]) and 5 <= row[9] <= 16 for row in rows))


def surface_key(row):
    return tuple(v // 10 for v in row[:3])


def validate_scene(value):
    """Validate all fields before an invalid packet can reset or change a map."""
    if not isinstance(value, dict) or type(value.get('version')) is not int or value['version'] != 1:
        raise ValueError('Unsupported scene format')
    session = value.get('session')
    if not isinstance(session, str) or not 1 <= len(session) <= 128 or not session.isprintable():
        raise ValueError('Invalid scene session')
    if not _number(value.get('timestamp'), 1e12) or value['timestamp'] < 0:
        raise ValueError('Invalid scene timestamp')
    if value.get('tracking') not in ('normal', 'limited', 'unavailable') or not _vector(value.get('pose'), 4):
        raise ValueError('Invalid tracking or pose')
    floor = value.get('floor_y')
    if floor is not None and not _number(floor):
        raise ValueError('Invalid floor height')
    if not valid_surfaces(value.get("surfaces", []), 512):
        raise ValueError("Invalid visual surface samples")
    points, objects = value.get('points'), value.get('objects')
    if not isinstance(points, list) or len(points) > 1500 or not all(_vector(p, 3) for p in points):
        raise ValueError('Invalid scene points')
    if not isinstance(objects, list) or len(objects) > 20:
        raise ValueError('Invalid scene objects')
    for obj in objects:
        if (not isinstance(obj, dict) or not isinstance(obj.get('label'), str)
                or not 1 <= len(obj['label']) <= 64 or not obj['label'].isprintable()
                or not _vector(obj.get('position'), 3)
                or not _number(obj.get('confidence'), 1) or obj['confidence'] < 0):
            raise ValueError('Invalid detected object')
        observed = obj.get('observation_timestamp', value['timestamp'])
        if (not _number(observed, 1e12) or observed < 0
                or observed > value['timestamp'] + 0.001):
            raise ValueError('Invalid object observation timestamp')
    estimated = value.get('estimated_points', [])
    if not isinstance(estimated, list) or len(estimated) > 256 or not all(_vector(p, 3) for p in estimated):
        raise ValueError('Invalid estimated visualization points')
    persistent = value.get('persistent_points', [])
    if not isinstance(persistent, list) or len(persistent) > 256 or not all(_vector(p, 3) for p in persistent):
        raise ValueError('Invalid persistent visualization points')
    return value


def read_scene(sock):
    def exact(size):
        data = bytearray()
        while len(data) < size:
            chunk = sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError('Scene stream disconnected')
            data.extend(chunk)
        return data
    size, = struct.unpack('!I', exact(4))
    if not 2 <= size <= MAX_PACKET:
        raise ValueError('Invalid scene packet size')
    try:
        return validate_scene(json.loads(exact(size)))
    except (UnicodeError, RecursionError) as error:
        raise ValueError('Invalid scene JSON') from error


class Map:
    def __init__(self, save_path=None, clock=time.monotonic, native=None):
        self.native = native
        self.lock = threading.RLock()
        self.clock = clock
        self.save_path = Path(save_path) if save_path else Path(__file__).resolve().parent / 'state/map.json'
        self.pending_restore = None
        self.restore_baseline = None
        self.session = ''
        self.timestamp = -1.0
        self.received = None
        self.connected = False
        self.source = None
        self.tracking = 'unavailable'
        self.floor_y = None
        self.mount_camera, self.mount_lidar = mount_heights(None)
        self.pose = [0.0, 0.0, 0.0, 0.0]
        self.frames = 0
        self.message = ''
        self._clear_geometry()

    def _clear_geometry(self):
        self.grid = bytearray(SIZE * SIZE)
        self.surfaces = OrderedDict()
        self.voxels = OrderedDict()
        self.estimated_voxels = OrderedDict()
        self.persistent_voxels = OrderedDict()
        self.persistent_observation = -math.inf
        self.inferred_obstacles = OrderedDict()
        self.inference_observation = -math.inf
        self.inference_deadline = 0.0
        self.inference_available = False
        self.last_inferred_cells = frozenset()
        self.objects = OrderedDict()
        self.next_object = 1
        self.selected = None
        self.path = []
        self.plan_status = 'Select an observed object to preview a path'
        self.scan_bins = set()
        self.scan_stations = OrderedDict()
        self.scan_station = None
        self.last_plan = -math.inf
        self.plan_dirty = True

    @staticmethod
    def cell(x, z):
        ix, iz = math.floor((x - ORIGIN[0]) / RESOLUTION), math.floor((z - ORIGIN[1]) / RESOLUTION)
        return (ix, iz) if 0 <= ix < SIZE and 0 <= iz < SIZE else None

    @staticmethod
    def world(cell):
        return [round(ORIGIN[0] + (cell[0] + 0.5) * RESOLUTION, 3),
                round(ORIGIN[1] + (cell[1] + 0.5) * RESOLUTION, 3)]

    def disconnect(self, source=None):
        with self.lock:
            if source is not None and source is not self.source:
                return
            self.connected = False
            self._invalidate_inference()
            self.path = []
            self.plan_status = 'Scene disconnected'
            self.plan_dirty = True

    def update(self, value, source=None):
        scene = validate_scene(value)
        now = self.clock()
        with self.lock:
            if scene['session'] == self.session and scene['timestamp'] <= self.timestamp:
                raise ValueError('Non-increasing scene timestamp')
            if scene['session'] != self.session:
                if self.pending_restore is not None and scene['session'] != self.pending_restore['session']:
                    if self.restore_baseline is None:
                        self.restore_baseline = scene['session']
                    elif scene['session'] != self.restore_baseline:
                        self.pending_restore = None
                        self.restore_baseline = None
                        self.message = 'New AR session; cancelled pending saved-map restore'
                self._clear_geometry()
                self.session = scene['session']
                if self.native is not None: self.native.set_session(self.session)
                self.floor_y = None
                self.frames = 0
            self.timestamp, self.received = scene['timestamp'], now
            self.connected = True
            self.source = source
            self.tracking = scene['tracking']
            self.pose = list(scene['pose'])
            self.mount_camera, self.mount_lidar = mount_heights(scene.get('camera_transform_columns'))
            self.frames += 1
            self.plan_dirty = True
            if self.tracking != 'normal':
                self._invalidate_inference()
                self.path = []
                self.plan_status = 'Waiting for normal AR tracking'
                return
            if self.pending_restore is not None and self.pending_restore['session'] == self.session:
                self._restore(self.pending_restore)
                self.pending_restore = None
                self.restore_baseline = None
                self.message = 'Saved geometry restored; reobserve targets before planning'
            floor = scene['floor_y']
            if floor is None:
                if self.floor_y is not None:
                    self._clear_geometry()
                self.floor_y = None
                self.path = []
                self.plan_status = 'Calibrate the floor before mapping'
                self._remember_cloud(scene, now)
                self._objects(scene['objects'], now, scene['timestamp'])
                return
            if self.floor_y is not None and abs(self.floor_y - floor) > 0.1:
                self._clear_geometry()
                self.message = 'Floor calibration changed; cleared geometry'
            self.floor_y = float(floor)
            position = (self.pose[0], self.pose[2])
            if self.scan_station is None or math.dist(position, self.scan_station) > 1.0:
                self.scan_station = next((station for station in reversed(self.scan_stations)
                                          if math.dist(position, station) <= 0.8), position)
                self.scan_bins = self.scan_stations.setdefault(self.scan_station, set())
                self.scan_stations.move_to_end(self.scan_station)
                if len(self.scan_stations) > 256:
                    self.scan_stations.popitem(last=False)
            if scene['points']:
                self.scan_bins.add(int((self.pose[3] % (2 * math.pi)) / (2 * math.pi) * 36))
            if self.native is None:  # Offline reference model used by planner parity tests only.
                # Only directly observed floor endpoints become free. Do not infer
                # drive clearance beneath high camera rays or behind obstacles.
                ground, obstacles = set(), set()
                for point in scene['points']:
                    key = tuple(math.floor(c / RESOLUTION) for c in point)
                    self.voxels[key] = None
                    self.voxels.move_to_end(key)
                    while len(self.voxels) > MAX_VOXELS:
                        self.voxels.popitem(last=False)
                    cell = self.cell(point[0], point[2])
                    if cell is None:
                        continue
                    index = cell[1] * SIZE + cell[0]
                    height = point[1] - self.floor_y
                    if abs(height) <= 0.06:
                        ground.add(index)
                    elif MIN_HEIGHT <= height <= MAX_HEIGHT:
                        obstacles.add(index)
                for index in ground:
                    if self.grid[index] == 0:
                        self.grid[index] = 1
                for index in obstacles:
                    self.grid[index] = 2
            if self.native is not None:
                for point in scene['points']:
                    key = tuple(math.floor(c / RESOLUTION) for c in point)
                    self.voxels[key] = None
                    self.voxels.move_to_end(key)
                    while len(self.voxels) > MAX_VOXELS: self.voxels.popitem(last=False)
            self._remember_cloud(scene, now)
            self._objects(scene['objects'], now, scene['timestamp'])
            self._plan(now)

    def _invalidate_inference(self):
        self.inferred_obstacles.clear()
        self.inference_available = False
        self.plan_dirty = True

    def _remember_cloud(self, scene, now):
        # Measured occupancy is unchanged. Calibrated inference only adds a
        # separate temporary blocking layer; it can never mark a cell free.
        for row in scene.get('surfaces', []):
            key = surface_key(row)
            old = self.surfaces.get(key)
            if old is not None and sum(a*b for a, b in zip(old[3:6], row[3:6])) > 12000:
                row = row.copy()
                row[6:9] = [(3*a+b+2)//4 for a, b in zip(old[6:9], row[6:9])]
            self.surfaces[key] = row
            self.surfaces.move_to_end(key)
        while len(self.surfaces) > SURFACE_LIMIT:
            self.surfaces.popitem(last=False)
        rays = _MeasuredRays(scene['pose'], scene['points'])
        for point in scene['points']:
            key = tuple(math.floor(c / RESOLUTION) for c in point)
            self.voxels[key] = None
            self.voxels.move_to_end(key)
            # Reliable measurements replace nearby prior estimates.
            if self.estimated_voxels or self.inferred_obstacles or self.persistent_voxels:
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            nearby = (key[0]+dx, key[1]+dy, key[2]+dz)
                            self.estimated_voxels.pop(nearby, None)
                            self.inferred_obstacles.pop(nearby, None)
                            self.persistent_voxels.pop(nearby, None)
        while len(self.voxels) > MAX_VOXELS:
            self.voxels.popitem(last=False)
        for key, (point, expiry) in list(self.inferred_obstacles.items()):
            if expiry <= now or rays.clears(point):
                self.inferred_obstacles.pop(key, None)
                self.estimated_voxels.pop(key, None)
        for key in list(self.estimated_voxels):
            point = [(c + .5) * RESOLUTION for c in key]
            if rays.clears(point):
                self.estimated_voxels.pop(key, None)
        observed = _calibrated_observation(scene)
        valid = observed is not None and scene['tracking'] == 'normal'
        new_observation = valid and observed > self.inference_observation
        if new_observation:
            self.inference_observation = observed
            self.inference_deadline = now + INFERENCE_SECONDS - (scene['timestamp'] - observed)
        usable = (valid and observed == self.inference_observation
                  and now < self.inference_deadline and self.floor_y is not None
                  and scene.get('depth_calibration', {}).get('usage') == 'conservative_obstacle_veto')
        self.inference_available = usable
        if not usable:
            self._invalidate_inference()
        # Repeated scene packets carry the same cached result. Do not reinsert
        # an estimate that LiDAR already overrode or extend its original age.
        for point in scene.get('estimated_points', []) if new_observation else []:
            key = tuple(math.floor(c / RESOLUTION) for c in point)
            measured_nearby = any((key[0]+dx, key[1]+dy, key[2]+dz) in self.voxels
                                  for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1))
            if not measured_nearby and not rays.clears(point):
                self.estimated_voxels[key] = self.inference_deadline - INFERENCE_SECONDS
                self.estimated_voxels.move_to_end(key)
                if usable and MIN_HEIGHT <= point[1] - self.floor_y <= MAX_HEIGHT:
                    self.inferred_obstacles[key] = (tuple(point), self.inference_deadline)
                    self.inferred_obstacles.move_to_end(key)
        while len(self.estimated_voxels) > MAX_INFERRED_VOXELS:
            self.estimated_voxels.popitem(last=False)
        while len(self.inferred_obstacles) > MAX_INFERRED_VOXELS:
            self.inferred_obstacles.popitem(last=False)
        for key, seen in list(self.estimated_voxels.items()):
            if now - seen > 30:
                del self.estimated_voxels[key]
        self._remember_persistent(scene, now, rays)

    @staticmethod
    def _persistent_confidence(item, now):
        # Confidence is a display weight, not a calibrated probability.
        return item['confidence'] * math.exp(-max(0, now - item['seen']) / 60.0)

    def _prune_persistent(self, now):
        for key, item in list(self.persistent_voxels.items()):
            if now - item['seen'] >= PERSISTENT_SECONDS:
                del self.persistent_voxels[key]

    def _remember_persistent(self, scene, now, rays):
        """Longer-lived visual memory; intentionally absent from all planning."""
        self._prune_persistent(now)
        for key, item in list(self.persistent_voxels.items()):
            if rays.clears(item['position']):
                del self.persistent_voxels[key]
        metadata = _persistent_observation(scene)
        if metadata is None or scene['tracking'] != 'normal':
            return
        observed, confidence, mode, calibrated = metadata
        if observed <= self.persistent_observation:
            return
        self.persistent_observation = observed
        seen = now - (scene['timestamp'] - observed)
        matched = set()
        for point in scene.get('persistent_points', []):
            if not 0.2 <= math.dist(point, scene['pose'][:3]) <= PERSISTENT_RANGE:
                continue
            key = tuple(math.floor(c / RESOLUTION) for c in point)
            if key in matched:
                continue
            matched.add(key)
            measured_nearby = any((key[0]+dx, key[1]+dy, key[2]+dz) in self.voxels
                                  for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1))
            if measured_nearby or rays.clears(point):
                continue
            previous = self.persistent_voxels.get(key)
            # Multiple independent source frames strengthen repeat observations;
            # retries of one cached frame cannot refresh age or confidence.
            prior = self._persistent_confidence(previous, now) if previous else 0.0
            independent = mode != 'cached' and (previous is None or calibrated > previous['calibration_timestamp'])
            if independent:
                strength = min(.85, 1 - (1 - prior) * (1 - min(.65, confidence)))
            else:
                # Reusing a fit is correlated evidence even for a new RGB
                # frame; it cannot manufacture confidence by repetition.
                strength = max(prior, min(.35 if mode == 'cached' else .65, confidence))
            self.persistent_voxels[key] = dict(position=tuple(point), seen=seen,
                                              confidence=strength, mode=mode,
                                              calibration_timestamp=max(calibrated, previous['calibration_timestamp'] if previous else calibrated),
                                              observations=min(255, (previous['observations'] if previous else 0) + int(independent)))
            self.persistent_voxels.move_to_end(key)
        while len(self.persistent_voxels) > MAX_PERSISTENT_VOXELS:
            self.persistent_voxels.popitem(last=False)

    def _persistent_snapshot(self, now):
        self._prune_persistent(now)
        values = list(self.persistent_voxels.values())
        # Uniformly sample the bounded memory to preserve its spatial extent.
        sampled = values[::max(1, math.ceil(len(values) / 1500))]
        strengths = [self._persistent_confidence(item, now) for item in values]
        return {
            'distant3d': [[round(v, 3) for v in item['position']] for item in sampled],
            'distant_confidence': [round(self._persistent_confidence(item, now), 3) for item in sampled],
            'distant_memory': {
                'voxels': len(values), 'displayed': len(sampled), 'ttl_s': PERSISTENT_SECONDS,
                'max_range_m': PERSISTENT_RANGE, 'visual_only': True,
                'mean_confidence': round(sum(strengths) / len(strengths), 3) if strengths else 0,
                'oldest_age_s': round(max((now - item['seen'] for item in values), default=0), 2),
                'reobserved': sum(item['observations'] > 1 for item in values),
                'modes': {mode: sum(item['mode'] == mode for item in values)
                          for mode in ('anchors', 'cached', 'direct')}
            }
        }

    def _inferred_cells(self, now):
        if self._gate(now) is not None:
            return frozenset()
        cells = set()
        for key, (point, expiry) in list(self.inferred_obstacles.items()):
            if expiry <= now:
                del self.inferred_obstacles[key]
                continue
            cell = self.cell(point[0], point[2])
            if cell is not None:
                cells.add(cell[1] * SIZE + cell[0])
        return frozenset(cells)

    def inference_veto(self):
        """Optional native-guard input. No inference result can grant clearance."""
        now = self.clock()
        with self.lock:
            if (self._gate(now) is not None or not self.inference_available
                    or self.inference_deadline <= now):
                return {'nearest_m': -1.0, 'expires': 0.0}
            fx, fz = math.sin(self.pose[3]), math.cos(self.pose[3])
            nearest, expiry = -1.0, self.inference_deadline
            for point, deadline in self.inferred_obstacles.values():
                if deadline <= now:
                    continue
                dx, dz = point[0] - self.pose[0], point[2] - self.pose[2]
                forward, side = fx * dx + fz * dz, fz * dx - fx * dz
                if (abs(side) < .45 and forward > 0
                        and MIN_HEIGHT <= point[1] - self.floor_y <= MAX_HEIGHT
                        and (nearest < 0 or forward < nearest)):
                    nearest, expiry = forward, deadline
            return {'nearest_m': nearest, 'expires': min(expiry, now + .4)}

    ingest = update

    def _objects(self, detections, now, scene_timestamp):
        matched = set()
        for detection in detections:
            if detection['confidence'] < 0.35:
                continue
            observed = detection.get('observation_timestamp', scene_timestamp)
            observation_age = max(0.0, scene_timestamp - observed)
            # Keep cached inference results from refreshing a target forever.
            # Observation time is in the phone's monotonic clock domain; only
            # its age within this same scene clock is applied to our clock.
            if observation_age > TARGET_FRESH_SECONDS:
                continue
            best, distance = None, 0.6
            for key, obj in self.objects.items():
                if key in matched or obj['label'] != detection['label']:
                    continue
                d = math.dist(obj['position'], detection['position'])
                if d < distance:
                    best, distance = key, d
            if best is None:
                best = str(self.next_object)
                self.next_object += 1
                if len(self.objects) >= MAX_OBJECTS:
                    removed, _ = self.objects.popitem(last=False)
                    if self.selected == removed:
                        self.selected = None
                self.objects[best] = {'id': best, 'label': detection['label'],
                                     'position': list(detection['position'])}
            obj = self.objects[best]
            if obj.get('observation_timestamp', -math.inf) >= observed:
                matched.add(best)
                continue
            # Most recent position avoids averaging a moving target into space
            # that it never occupied. Identity association is intentionally basic.
            obj.update(position=list(detection['position']), confidence=float(detection['confidence']),
                       seen=now - observation_age, observation_timestamp=observed, restored=False)
            self.objects.move_to_end(best)
            matched.add(best)

    def _gate(self, now):
        if not self.connected or self.received is None or now - self.received > FRESH_SECONDS:
            return 'Scene stale or disconnected'
        if self.tracking != 'normal':
            return 'Waiting for normal AR tracking'
        if self.floor_y is None:
            return 'Calibrate the floor before mapping'
        if self.pending_restore is not None:
            return 'Saved map waiting for matching AR session'
        return None

    def _plan(self, now, force=False):
        if self.native is not None:
            self.path = []
            self.plan_status = 'Native A* route shown when navigation is active'
            return
        inferred_cells = self._inferred_cells(now)
        if inferred_cells != self.last_inferred_cells:
            self.last_inferred_cells = inferred_cells
            self.plan_dirty = True
        gate = self._gate(now)
        if gate:
            self.path, self.plan_status = [], gate
            return
        if self.selected is None:
            self.path, self.plan_status = [], 'Select an observed object to preview a path'
            return
        obj = self.objects.get(self.selected)
        if obj is None or obj['seen'] is None or now - obj['seen'] > TARGET_FRESH_SECONDS:
            self.path, self.plan_status = [], 'Target stale; reobserve it before planning'
            return
        if not force:
            if not self.plan_dirty:
                return
            if now - self.last_plan < 0.5:
                self.path = []
                self.plan_status = 'Updating path preview'
                return
        self.plan_dirty = False
        self.last_plan = now
        self.path = []
        clear = set()
        radius = math.ceil(ROBOT_RADIUS / RESOLUTION)
        # A square enclosing the provisional circular footprint must be wholly
        # observed free. This inflates both obstacles and unknown space, using
        # a summed-area table to bound clearance work to O(grid size).
        stride = SIZE + 1
        prefix = [0] * (stride * stride)
        for z in range(SIZE):
            row_sum = 0
            for x in range(SIZE):
                index = z * SIZE + x
                row_sum += self.grid[index] == 1 and index not in inferred_cells
                prefix[(z + 1) * stride + x + 1] = prefix[z * stride + x + 1] + row_sum
        required = (2 * radius + 1) ** 2
        for z in range(radius, SIZE - radius):
            top, bottom = (z - radius) * stride, (z + radius + 1) * stride
            for x in range(radius, SIZE - radius):
                left, right = x - radius, x + radius + 1
                count = prefix[bottom + right] - prefix[top + right] - prefix[bottom + left] + prefix[top + left]
                if count == required:
                    clear.add((x, z))
        def free(cell):
            return cell in clear
        start = self.cell(self.pose[0], self.pose[2])
        if not free(start):
            self.plan_status = 'Observe clear floor around the robot before planning'
            return
        target = self.cell(obj['position'][0], obj['position'][2])
        if target is None:
            self.plan_status = 'Target outside the 20 m map'
            return
        goals = set()
        for dx in range(-9, 10):
            for dz in range(-9, 10):
                candidate = (target[0] + dx, target[1] + dz)
                wx, wz = self.world(candidate)
                distance = math.hypot(wx - obj['position'][0], wz - obj['position'][2])
                if 0.6 <= distance <= 0.85 and free(candidate):
                    goals.add(candidate)
        if not goals:
            self.plan_status = 'No observed clear approach point near target'
            return
        # Admissible lower bound to the approach ring, avoiding per-goal work.
        def heuristic(cell):
            return max(0.0, math.hypot(cell[0] - target[0], cell[1] - target[1]) - 9)
        frontier = [(heuristic(start), 0.0, start)]
        costs, parent = {start: 0.0}, {}
        end = None
        while frontier:
            _, cost, current = heapq.heappop(frontier)
            if cost > costs[current]:
                continue
            if current in goals:
                end = current
                break
            for dx, dz in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (current[0] + dx, current[1] + dz)
                if not free(nxt):
                    continue
                # Never cut diagonally between blocked or unknown cells.
                if dx and dz and (not free((current[0] + dx, current[1])) or not free((current[0], current[1] + dz))):
                    continue
                new = cost + (math.sqrt(2) if dx and dz else 1)
                if new < costs.get(nxt, math.inf):
                    costs[nxt], parent[nxt] = new, current
                    heapq.heappush(frontier, (new + heuristic(nxt), new, nxt))
        if end is None:
            self.plan_status = 'No route through observed clear floor'
            return
        route = [end]
        while route[-1] != start:
            route.append(parent[route[-1]])
        route.reverse()
        # Keep every turn, removing only collinear intermediate cells. Never
        # subsample arbitrary corners, which could draw across an obstacle.
        corners = route[:1]
        for index in range(1, len(route) - 1):
            before = (route[index][0] - route[index - 1][0], route[index][1] - route[index - 1][1])
            after = (route[index + 1][0] - route[index][0], route[index + 1][1] - route[index][1])
            if before != after:
                corners.append(route[index])
        if len(route) > 1:
            corners.append(route[-1])
        if len(corners) > 256:
            self.plan_status = 'Route exceeds preview complexity limit'
            return
        self.path = [self.world(cell) for cell in corners]
        self.plan_status = 'Preview only — footprint and clearance are unvalidated'

    def snapshot(self):
        now = self.clock()
        with self.lock:
            self._plan(now)
            inferred_cells = self._inferred_cells(now)
            fused_grid = bytearray(self.grid)
            for index in inferred_cells:
                if fused_grid[index] != 2:
                    fused_grid[index] = 3
            tracking_live = (self.connected and self.received is not None
                             and now - self.received <= FRESH_SECONDS and self.tracking == 'normal')
            # A horizontal visual reference is useful before floor calibration,
            # but it must not enter the occupancy model or the guard stream.
            floor_assumed = self.floor_y is None and self.frames > 0
            display_floor = self.pose[1] - self.mount_camera if floor_assumed else self.floor_y
            objects = []
            for obj in self.objects.values():
                age = max(0, now - obj['seen']) if obj['seen'] is not None else None
                objects.append({'id': obj['id'], 'label': obj['label'],
                                'position': [round(v, 3) for v in obj['position']],
                                'confidence': round(obj['confidence'], 4),
                                'age_s': round(age, 2) if age is not None else None,
                                'last_seen_age_s': round(age, 2) if age is not None else None,
                                'currently_seen': tracking_live and age is not None and age <= OBJECT_VISIBLE_SECONDS,
                                'stale': not tracking_live or age is None or age > TARGET_FRESH_SECONDS,
                                'restored': obj.get('restored', False)})
            result = {'session': self.session, 'tracking': self.tracking,
                    **self._persistent_snapshot(now),
                    'live': self._gate(now) is None, 'floor_ready': self.floor_y is not None,
                    'tracking_live': tracking_live, 'connected': self.connected,
                    'grid': ''.join(str(v) for v in fused_grid), 'resolution': RESOLUTION,
                    'origin': list(ORIGIN), 'size': SIZE, 'pose': list(self.pose),
                    'floor_y': display_floor, 'floor_assumed': floor_assumed,
                    'assumed_phone_height_m': self.mount_camera,
                    'surface_splats': base64.b64encode(b''.join(
                        SURFACE_RECORD.pack(*row) for row in list(self.surfaces.values())
                        [::max(1, math.ceil(len(self.surfaces)/1500))])).decode('ascii'),
                    'points3d': [[round((c+.5)*RESOLUTION, 3) for c in key] for key in list(self.voxels)[::max(1, math.ceil(len(self.voxels)/2000))]],
                    'estimated3d': [[round((c+.5)*RESOLUTION, 3) for c in key] for key, seen in self.estimated_voxels.items() if now-seen <= 30],
                    'active_estimated3d': [[round(v, 3) for v in point]
                                           for point, expiry in self.inferred_obstacles.values()
                                           if expiry > now and self._gate(now) is None],
                    'inferred_obstacle_cells': len(inferred_cells),
                    'inference_live': self.inference_available and self.inference_deadline > now and self._gate(now) is None,
                    'inferred_age_s': round(max(0, now - self.inference_deadline + INFERENCE_SECONDS), 3)
                                      if math.isfinite(self.inference_observation) else None,
                    'objects': objects,
                    'path': [list(p) for p in self.path], 'plan_status': self.plan_status,
                    'selected': self.selected, 'voxels': len(self.voxels), 'frames': self.frames,
                    'scan_degrees': len(self.scan_bins) * 10,
                    'message': (self.message + (' · Scan this area: turn slowly and pause for detections'
                                if len(self.scan_bins) < 32 else ' · Local area scanned'))
                               if tracking_live and self.floor_y is not None else self.message}
            if self.native is not None:
                # Replace all occupancy/route fields atomically. No Python-map fallback.
                result.update(self.native.snapshot())
                result['inferred_obstacle_cells'] = 0
                if not result.get('floor_ready') and floor_assumed:
                    result.update(floor_assumed=True, floor_y=display_floor)
            if self.frames and display_floor is not None:
                # Use one scene timestamp for pose, attitude and floor height.
                camera_height = self.pose[1] - display_floor
                result['camera_height_m'] = camera_height
                result['lidar_height_m'] = camera_height + self.mount_lidar - self.mount_camera
            return result

    def _saved(self):
        native = self.native.snapshot() if self.native is not None else None
        if native is not None and not native['native_map_live']:
            raise ValueError('Native navigation map is unavailable; cannot save occupancy')
        return {'version': 1, 'session': self.session, 'floor_y': self.floor_y,
                'grid': native['grid'] if native is not None else ''.join(str(v) for v in self.grid),
                'surfaces': list(self.surfaces.values()),
                'voxels': [list(k) for k in self.voxels],
                'objects': [{k: obj[k] for k in ('id', 'label', 'position', 'confidence')}
                            for obj in self.objects.values()]}

    def _read_saved(self):
        if self.save_path.stat().st_size > 2_000_000:
            raise ValueError('Saved map exceeds size limit')
        value = json.loads(self.save_path.read_text())
        if (not isinstance(value, dict) or type(value.get('version')) is not int or value['version'] != 1
                or not isinstance(value.get('session'), str) or not 1 <= len(value['session']) <= 128
                or not value['session'].isprintable()
                or not _number(value.get('floor_y'))):
            raise ValueError('Invalid saved map metadata')
        grid = value.get('grid')
        if not isinstance(grid, str) or len(grid) != SIZE * SIZE or any(c not in '012' for c in grid):
            raise ValueError('Invalid saved grid')
        voxels = value.get('voxels')
        if (not isinstance(voxels, list) or len(voxels) > MAX_VOXELS
                or not all(isinstance(p, list) and len(p) == 3 and all(type(x) is int and abs(x) <= 10000 for x in p) for p in voxels)):
            raise ValueError('Invalid saved voxels')
        if not valid_surfaces(value.get('surfaces', []), SURFACE_LIMIT):
            raise ValueError('Invalid saved surfaces')
        objects = value.get('objects')
        if not isinstance(objects, list) or len(objects) > MAX_OBJECTS:
            raise ValueError('Invalid saved objects')
        for obj in objects:
            if (not isinstance(obj, dict) or not isinstance(obj.get('id'), str)
                    or not obj['id'].isdigit() or len(obj['id']) > 12
                    or not isinstance(obj.get('label'), str) or not 1 <= len(obj['label']) <= 64
                    or not obj['label'].isprintable()
                    or not _vector(obj.get('position'), 3)
                    or not _number(obj.get('confidence'), 1) or obj['confidence'] < 0):
                raise ValueError('Invalid saved object')
        if len({obj['id'] for obj in objects}) != len(objects):
            raise ValueError('Duplicate saved object ID')
        return value

    def _restore(self, value):
        self._clear_geometry()
        self.surfaces.update((surface_key(row), row) for row in value.get('surfaces', []))
        if self.native is None: self.grid = bytearray(int(c) for c in value['grid'])
        self.voxels = OrderedDict((tuple(p), None) for p in value['voxels'])
        self.floor_y = value['floor_y']
        self.objects = OrderedDict((obj['id'], dict(
            {key: obj[key] for key in ('id', 'label', 'position', 'confidence')},
            seen=None, restored=True)) for obj in value['objects'])
        self.next_object = max((int(k) for k in self.objects), default=0) + 1

    def command(self, command):
        """Return {ok, message}; user-facing failures never kill a receiver."""
        with self.lock:
            try:
                if not isinstance(command, dict):
                    raise ValueError('Invalid map command')
                op = command.get('op')
                if op == 'map_clear':
                    if self.native is not None: self.native.reset()
                    self._clear_geometry()
                    self.pending_restore = None
                    self.restore_baseline = None
                    message = 'Map cleared; scan again'
                elif op == 'map_save':
                    if self._gate(self.clock()) is not None:
                        raise ValueError('Need fresh tracking and a calibrated floor to save')
                    self.save_path.parent.mkdir(parents=True, exist_ok=True)
                    temp = self.save_path.with_suffix('.tmp')
                    temp.write_text(json.dumps(self._saved(), allow_nan=False))
                    temp.replace(self.save_path)
                    message = 'Map saved for this AR session'
                elif op == 'map_load':
                    self.pending_restore = self._read_saved()
                    self.restore_baseline = self.session or None
                    self.path = []
                    self.plan_dirty = True
                    message = ('Restoring visual memory; native navigation requires fresh observations'
                               if self.native is not None else 'Waiting for matching saved AR session and normal tracking')
                elif op == 'select_target':
                    target = command.get('id')
                    if not isinstance(target, str) or target not in self.objects:
                        raise ValueError('Target is not in this map')
                    self.selected = target
                    self.plan_dirty = True
                    self._plan(self.clock(), force=True)
                    message = self.plan_status
                else:
                    raise ValueError('Unknown map command')
                self.message = message
                return {'ok': True, 'message': message}
            except (OSError, ValueError, TypeError, RecursionError) as error:
                self.message = str(error)
                return {'ok': False, 'message': self.message}


class SceneFeed:
    def __init__(self, port=5561, mapper=None, fusion_port=0):
        self.port = port
        self.mapper = mapper if mapper is not None else Map(native=NativeGrid())
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.sock = None
        self.count = 0
        self.received = 0.0
        self.status = 'Waiting for ARKit depth and pose'
        self.source = object()
        self.last_report = -math.inf
        self.fusion = FusionPublisher(fusion_port)

    def run(self):
        if self.mapper.native is not None:
            threading.Thread(target=self.mapper.native.run, args=(self.stop,), daemon=True).start()
        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=2) as sock:
                    with self.lock:
                        self.sock = sock
                    sock.settimeout(1)
                    while not self.stop.is_set():
                        scene = read_scene(sock)
                        if self.stop.is_set():
                            break
                        self.mapper.update(scene, source=self.source)
                        self.fusion.publish(self.mapper.inference_veto())
                        sock.sendall(b'\x01')
                        with self.lock:
                            self.count += 1
                            self.received = time.monotonic()
                            self.status = 'Receiving ARKit scene'
                        if time.monotonic() - self.last_report >= 10:
                            self.last_report = time.monotonic()
                            with self.mapper.lock:
                                print('Scene: frames=' + str(self.mapper.frames) +
                                      ' tracking=' + self.mapper.tracking +
                                      ' floor=' + str(self.mapper.floor_y is not None) +
                                      ' voxels=' + str(len(self.mapper.voxels)) +
                                      ' free=' + str(self.mapper.grid.count(1)) +
                                      ' occupied=' + str(self.mapper.grid.count(2)) +
                                      ' targets=' + str(len(self.mapper.objects)) +
                                      ' inferred=' + str(len(self.mapper.estimated_voxels)), flush=True)

            except (OSError, ValueError) as error:
                with self.lock:
                    self.status = str(error)
            finally:
                self.mapper.disconnect(source=self.source)
                self.fusion.clear()
                with self.lock:
                    self.sock = None
            self.stop.wait(1)
        self.fusion.close()

    def snapshot(self):
        return self.mapper.snapshot()
