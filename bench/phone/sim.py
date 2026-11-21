"""Fake phone scene stream: ray-cast a room and emit Andrew's version-1 scene packets.

Room input is plain data in the bench world frame (metres, x forward at start, y left, z up),
the same shape as the sim world's bench/world/rooms/*.json:
    walls:   [[x1, y1, x2, y2], ...]   vertical, "wall_h" tall (default 1.0)
    objects: [{"label": "trash can", "x": .., "y": .., "r": .., "h": ..},        cylinder
              {"label": "chair", "x": .., "y": .., "w": .., "d": .., "h": ..}]  box, w along x
             "solid": false = no geometry (a door); a label makes it detectable.
`room_from_json()` also takes "boxes" for "objects", "pos"/"position"/"size", and the room
nested under "room"/"world"/"state" (a world-state message). `SimPhone.set_room()` swaps the
geometry live, so obstacles the sim world adds/moves/deletes show up in the next scene.

Frame conversion to ARKit (y up), the frame the phone sends:
    ARKit (x, y, z) = (world y, world z, world x);  pose = [x, cam_height, z, yaw],
    yaw = world heading th (forward = (sin yaw, cos yaw) in ARKit x/z, Andrew's convention).
floor_y = 0. Only an approximation of the phone: a uniform depth grid with range noise
and dropouts, no confidence map, no RGB surfaces, no inference/persistent points.
"""
from __future__ import annotations

import json
import math
import random
import time

DEFAULT_ROOM = {
    # Used when no sim world is loaded. 5 m x 4 m, start pose at the origin facing +x.
    # The inner wall at x = 2 hides the bin; the gap at y 0.6..2 is the doorway.
    "wall_h": 1.0,
    "walls": [[-1, -2, 4, -2], [4, -2, 4, 2], [4, 2, -1, 2], [-1, 2, -1, -2],
              [2, -2, 2, 0.6]],
    "objects": [{"label": "trash can", "x": 3.2, "y": -1.0, "r": 0.15, "h": 0.5},
                {"label": "chair", "x": 1.0, "y": 1.3, "w": 0.45, "d": 0.45, "h": 0.8}],
}


def find_room(value):
    """The room inside a world-state message, or None. Accepts the room at the top level or
    under "room"/"world"/"state" (the sim world's topic format is still being written)."""
    if not isinstance(value, dict):
        return None
    for key in ("room", "world", "state"):
        inner = value.get(key)
        if isinstance(inner, dict) and ("walls" in inner or "objects" in inner or "boxes" in inner):
            return inner
    return value if ("walls" in value or "objects" in value or "boxes" in value) else None


def room_from_json(value: dict) -> dict:
    """Plain room -> {"walls", "boxes", "cylinders", "wall_height"}. Objects with "r" are
    vertical cylinders, otherwise axis-aligned boxes; "solid": false (a door, a marker) is
    skipped for geometry but still detectable if it has a label."""
    value = find_room(value) or value
    if "cylinders" in value and "wall_height" in value:  # already normalized
        return {"walls": [list(w) for w in value["walls"]], "boxes": [dict(b) for b in value["boxes"]],
                "cylinders": [dict(c) for c in value["cylinders"]], "wall_height": value["wall_height"]}
    walls = [list(map(float, w[:4])) for w in value.get("walls", [])
             if isinstance(w, (list, tuple)) and len(w) >= 4]
    boxes, cylinders = [], []
    for o in value.get("boxes", value.get("objects", [])) or []:
        if not isinstance(o, dict):
            continue
        pos = o.get("pos") or o.get("position") or [o.get("x", 0), o.get("y", 0)]
        h = float(o.get("h", o.get("height", 0.5)))
        label = o.get("label", "") or ""
        solid = o.get("solid", True) is not False
        if "r" in o or "radius" in o:
            cylinders.append({"label": label, "x": float(pos[0]), "y": float(pos[1]),
                              "r": float(o.get("r", o.get("radius"))), "h": h, "solid": solid})
            continue
        size = o.get("size") or [o.get("w", 0.3), o.get("d", 0.3), h]
        if len(size) == 2:
            size = [size[0], size[1], h]
        boxes.append({"label": label, "x": float(pos[0]), "y": float(pos[1]),
                      "w": float(size[0]), "d": float(size[1]), "h": float(size[2]), "solid": solid})
    return {"walls": walls, "boxes": boxes, "cylinders": cylinders,
            "wall_height": float(value.get("wall_h", value.get("wall_height", 1.0)))}


def to_arkit(x, y, z):
    return [y, z, x]


class ScriptedPath:
    """Pose from (duration_s, v m/s, w rad/s) legs, integrated from a start pose."""
    DEFAULT = [(10.5, 0.0, 0.6), (4.0, 0.3, 0.0), (10.5, 0.0, -0.6), (3.0, 0.25, 0.25)]

    def __init__(self, legs=None, start=(0.0, 0.0, 0.0)):
        self.legs, self.start = legs or self.DEFAULT, start
        self.total = sum(leg[0] for leg in self.legs)

    def pose(self, t):
        x, y, th = self.start
        t = min(t, self.total)
        for dur, v, w in self.legs:
            dt, t = min(dur, t), t - min(dur, t)
            steps = max(1, int(dt / 0.02))
            for _ in range(steps):
                h = dt / steps
                x += v * math.cos(th) * h
                y += v * math.sin(th) * h
                th += w * h
            if t <= 0:
                break
        return x, y, th


def _faces(room):
    """Vertical faces (x1, y1, x2, y2, z_top, object key or None) and box tops."""
    faces = [(*w, room["wall_height"], None) for w in room["walls"]]
    tops = []
    for i, b in enumerate(room["boxes"]):
        if not b.get("solid", True):
            continue
        x0, x1 = b["x"] - b["w"] / 2, b["x"] + b["w"] / 2
        y0, y1 = b["y"] - b["d"] / 2, b["y"] + b["d"] / 2
        key = ("box", i)
        faces += [(x0, y0, x1, y0, b["h"], key), (x1, y0, x1, y1, b["h"], key),
                  (x1, y1, x0, y1, b["h"], key), (x0, y1, x0, y0, b["h"], key)]
        tops.append((x0, x1, y0, y1, b["h"]))
    return faces, tops


def _hit_segment(ox, oy, dx, dy, x1, y1, x2, y2):
    """t >= 0 along the horizontal ray (ox,oy)+t(dx,dy) hitting the segment, else None."""
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    qx, qy = x1 - ox, y1 - oy
    t = (qx * ey - qy * ex) / den
    u = (qx * dy - qy * dx) / den
    return t if t > 1e-6 and 0.0 <= u <= 1.0 else None


def _hit_circle(ox, oy, dx, dy, cx, cy, r):
    """First t > 0 where the ray (in xy, (dx,dy) need not be unit) enters the circle."""
    a = dx * dx + dy * dy
    if a < 1e-12:
        return None
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - a * c
    if disc < 0:
        return None
    t = (-b - math.sqrt(disc)) / a
    return t if t > 1e-6 else None


class Caster:
    def __init__(self, room):
        self.room = room
        self.faces, self.tops = _faces(room)
        self.cyl = [(c["x"], c["y"], c["r"], c["h"], ("cyl", i))
                    for i, c in enumerate(room["cylinders"]) if c.get("solid", True)]

    def cast(self, o, d, max_range):
        """Nearest hit distance of the 3D ray o + t d (|d| = 1), or None."""
        best = max_range
        if d[2] < -1e-9:
            best = min(best, -o[2] / d[2])
        for x1, y1, x2, y2, top, _ in self.faces:
            t = _hit_segment(o[0], o[1], d[0], d[1], x1, y1, x2, y2)
            if t is not None and t < best and 0.0 <= o[2] + t * d[2] <= top:
                best = t
        for cx, cy, r, h, _ in self.cyl:
            t = _hit_circle(o[0], o[1], d[0], d[1], cx, cy, r)
            if t is not None and t < best:
                z = o[2] + t * d[2]
                if 0.0 <= z <= h:
                    best = t
                elif z > h and d[2] < -1e-9 and o[2] > h:  # lid
                    tl = (h - o[2]) / d[2]
                    if tl < best and math.hypot(o[0] + tl * d[0] - cx, o[1] + tl * d[1] - cy) <= r:
                        best = tl
        if d[2] < -1e-9:
            for x0, x1, y0, y1, h in self.tops:
                if o[2] > h:
                    t = (h - o[2]) / d[2]
                    if t < best and x0 <= o[0] + t * d[0] <= x1 and y0 <= o[1] + t * d[1] <= y1:
                        best = t
        return best if best < max_range else None

    def wall_distance(self, ox, oy, dx, dy, skip):
        """Nearest blocker along a horizontal unit ray, ignoring object `skip`."""
        best = math.inf
        for x1, y1, x2, y2, _top, key in self.faces:
            if key != skip:
                t = _hit_segment(ox, oy, dx, dy, x1, y1, x2, y2)
                if t is not None:
                    best = min(best, t)
        for cx, cy, r, _h, key in self.cyl:
            if key != skip:
                t = _hit_circle(ox, oy, dx, dy, cx, cy, r)
                if t is not None:
                    best = min(best, t)
        return best


def room_key(room) -> str:
    return json.dumps(room, sort_keys=True)


class SimPhone:
    """Makes scene dicts. Tunables are plain attributes (the plugin exposes them).
    set_room() swaps the geometry live (the sim world's obstacle placement)."""

    def __init__(self, room=None, seed=1, session="sim-room", cam_height=0.138, pitch_deg=0.0,
                 hfov_deg=64.0, vfov_deg=50.0, cols=48, rows=30, depth_range=(0.2, 4.0),
                 range_noise_m=0.01, dropout=0.02, tracking="normal", floor_y=0.0, detect=True):
        self.rng = random.Random(seed)
        self.session = session
        self.cam_height, self.pitch = cam_height, math.radians(pitch_deg)
        self.hfov, self.vfov = math.radians(hfov_deg), math.radians(vfov_deg)
        self.cols, self.rows = cols, rows
        self.depth_range = depth_range
        self.range_noise, self.dropout = range_noise_m, dropout
        self.tracking, self.floor_y, self.detect = tracking, floor_y, detect
        self.room_source = "static"
        self.set_room(room if room is not None else DEFAULT_ROOM, "static")

    def set_room(self, room, source="world") -> bool:
        """True if the geometry changed. Safe while scene() runs on another thread."""
        norm = room_from_json(room)
        key = room_key(norm)
        if getattr(self, "_key", None) == key:
            return False
        self.room, self._key, self.room_source = norm, key, source
        self.caster = Caster(norm)  # one attribute swap: scene() uses whichever it grabbed
        return True

    def _targets(self, room):
        for i, b in enumerate(room["boxes"]):
            yield b, ("box", i)
        for i, c in enumerate(room["cylinders"]):
            yield c, ("cyl", i)

    def scene(self, timestamp, x, y, th):
        caster = self.caster
        room = caster.room
        hc = self.cam_height
        o = (x, y, hc)
        points = []
        lo, hi = self.depth_range
        if self.tracking == "normal":
            for j in range(self.rows):
                el = self.pitch + self.vfov * (0.5 - (j + 0.5) / self.rows)
                ce, se = math.cos(el), math.sin(el)
                for i in range(self.cols):
                    az = th + self.hfov * (0.5 - (i + 0.5) / self.cols)
                    d = (ce * math.cos(az), ce * math.sin(az), se)
                    t = caster.cast(o, d, hi + 0.5)
                    if t is None or self.rng.random() < self.dropout:
                        continue
                    t += self.rng.gauss(0.0, self.range_noise)
                    if not lo <= t <= hi:
                        continue
                    points.append([round(v, 3) for v in to_arkit(*(o[k] + t * d[k] for k in range(3)))])
        objects = []
        if self.detect and self.tracking == "normal":
            for b, key in self._targets(room):
                if not b["label"]:
                    continue
                dx, dy = b["x"] - x, b["y"] - y
                dist = math.hypot(dx, dy)
                bearing = (math.atan2(dy, dx) - th + math.pi) % (2 * math.pi) - math.pi
                if dist < 0.3 or dist > hi or abs(bearing) > self.hfov / 2:
                    continue
                if caster.wall_distance(x, y, dx / dist, dy / dist, key) < dist - 0.05:
                    continue  # occluded by a wall or another object
                objects.append({"label": b["label"], "confidence": round(max(0.4, 0.9 - 0.08 * dist), 3),
                                "position": [round(v, 3) for v in to_arkit(b["x"], b["y"], b["h"] / 2)],
                                "observation_timestamp": timestamp})
        return {"version": 1, "session": self.session, "timestamp": round(timestamp, 4),
                "tracking": self.tracking,
                "pose": [round(y, 4), round(hc, 4), round(x, 4), round(th, 5)],
                "floor_y": self.floor_y, "points": points[:1500], "objects": objects[:20],
                "depth_range_m": [lo, hi], "image_orientation": "right",
                "bench_sim": True}


def sim_factory(phone: SimPhone, pose_fn, rate_hz=10.0, duration_s=None, realtime=True):
    """SceneServer factory. pose_fn(t_since_start) -> (x, y, th) or None (no pose yet)."""
    def gen():
        k = 0
        period = 1.0 / rate_hz
        while duration_s is None or k * period <= duration_s:
            t = k * period
            pose = pose_fn(t)
            k += 1
            if pose is None:  # e.g. waiting for the first car truth
                time.sleep(period)
                continue
            # Realtime: phone-like uptime stamps, so a receiver reconnect (same session)
            # never sees time go backwards. Not realtime: deterministic 1000 + t.
            stamp = time.monotonic() if realtime else 1000.0 + t
            yield (period if realtime and k > 1 else 0.0), phone.scene(stamp, *pose)
    return gen
