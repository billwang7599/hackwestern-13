"""Shared by the world sim, the stub car and run_sim: rooms, geometry and frame conversions.

Frames (see bench/README.md "World frame" and bench/world/README.md):
  world   metres, x = the car's forward at its start pose, y = left, th CCW (rad)
  motor   tars_motor STATUS pose=: mm, x forward, y RIGHT, heading deg CLOCKWISE,
          origin = where tars_motor started (or was POSE RESET) = the room's start pose
  ark     what goto's nav feed and lidar_guard use (LDR1): x_ark = left, z_ark = forward,
          y up, yaw = CCW (forward = (sin yaw, cos yaw) in x,z). Origin = the room's start
          pose (the phone's ARKit session starts on the robot at the start pose).
          Same mapping as iter2-autonomy/goto/tests/test_fullstack.py and nav_note_dr().
"""
from __future__ import annotations

import copy
import json
import math
import os
import re
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOMS = os.path.join(HERE, "rooms")
REPO = os.path.dirname(os.path.dirname(HERE))
GOTO_DIR = os.path.join(REPO, "iter2-autonomy", "goto")

NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")


# ------------------------------------------------------------------ rooms
def list_rooms() -> list[str]:
    return sorted(f[:-5] for f in os.listdir(ROOMS) if f.endswith(".json"))


def load_room_file(name: str) -> dict:
    if not NAME_RE.match(name):
        raise ValueError(f"bad room name {name!r}")
    with open(os.path.join(ROOMS, name + ".json")) as f:
        return normalise_room(json.load(f), name)


def normalise_room(room: dict, name: str | None = None) -> dict:
    """Fill defaults, give every wall and object an id. Raises ValueError on bad input."""
    r = copy.deepcopy(room)
    r["name"] = r.get("name") or name or "untitled"
    s = r.get("start") or {}
    r["start"] = {"x": float(s.get("x", 0)), "y": float(s.get("y", 0)), "th": float(s.get("th", 0))}
    r["wall_h"] = float(r.get("wall_h", 2.0))
    walls = []
    for w in r.get("walls", []):
        seg = w["seg"] if isinstance(w, dict) else w
        if len(seg) != 4:
            raise ValueError(f"wall needs 4 numbers: {seg}")
        walls.append([float(v) for v in seg])
    r["walls"] = walls
    objs, seen = [], set()
    for i, o in enumerate(r.get("objects", [])):
        o = normalise_object(o, f"o{i}")
        while o["id"] in seen:
            o["id"] += "_"
        seen.add(o["id"])
        objs.append(o)
    r["objects"] = objs
    r["memory"] = [m if isinstance(m, dict) else {"id": m} for m in r.get("memory", [])]
    return r


def normalise_object(o: dict, default_id: str) -> dict:
    o = dict(o)
    o["id"] = str(o.get("id") or default_id)
    o["label"] = str(o.get("label") or "")
    o["x"], o["y"] = float(o["x"]), float(o["y"])
    o["h"] = float(o.get("h", 0.5))
    o["yaw"] = float(o.get("yaw", 0.0))
    if "r" in o:
        o["r"] = max(0.02, float(o["r"]))
        o.pop("w", None)
        o.pop("d", None)
    else:
        o["w"] = max(0.02, float(o.get("w", 0.3)))
        o["d"] = max(0.02, float(o.get("d", 0.3)))
    o["solid"] = bool(o.get("solid", True))
    return o


def room_for_disk(room: dict) -> dict:
    """The room as a file: walls as plain [x1,y1,x2,y2], no runtime keys."""
    out = {"name": room["name"], "about": room.get("about", ""), "start": room["start"],
           "wall_h": room["wall_h"], "walls": [list(map(_r3, w)) for w in room["walls"]],
           "objects": [], "memory": room.get("memory", [])}
    for o in room["objects"]:
        d = {k: (_r3(v) if isinstance(v, float) else v) for k, v in o.items()}
        if d.get("solid", True):
            d.pop("solid", None)
        if not d.get("yaw"):
            d.pop("yaw", None)
        out["objects"].append(d)
    return out


def _r3(v):
    return round(v, 3) if isinstance(v, float) else v


# ------------------------------------------------------------------ geometry
def object_corners(o: dict) -> list[tuple[float, float]]:
    """footprint polygon: box = 4 corners, cylinder = 8-gon"""
    if "r" in o:
        return [(o["x"] + o["r"] * math.cos(k * math.pi / 4), o["y"] + o["r"] * math.sin(k * math.pi / 4)) for k in range(8)]
    c, s = math.cos(o["yaw"]), math.sin(o["yaw"])
    hx, hy = o["w"] / 2, o["d"] / 2
    return [(o["x"] + c * ax - s * ay, o["y"] + s * ax + c * ay) for ax, ay in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]


def object_segments(o: dict) -> list[list[float]]:
    p = object_corners(o)
    return [[p[i][0], p[i][1], p[(i + 1) % len(p)][0], p[(i + 1) % len(p)][1]] for i in range(len(p))]


def collision_segments(room: dict) -> list[list[float]]:
    """what the car can't drive through: walls + every solid object's footprint (box = 4 segments)"""
    segs = [list(w) for w in room["walls"]]
    for o in room["objects"]:
        if o.get("solid", True):
            segs += object_segments(o)
    return [[round(v, 4) for v in s] for s in segs]


def seg_hit(px, py, dx, dy, x1, y1, x2, y2):
    """ray p + t d (d unit) against segment -> t or None"""
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((x1 - px) * ey - (y1 - py) * ex) / den
    u = ((x1 - px) * dy - (y1 - py) * dx) / den
    return t if t > 1e-6 and -1e-9 <= u <= 1 + 1e-9 else None


def circle_hit(px, py, dx, dy, cx, cy, r):
    fx, fy = px - cx, py - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    sq = math.sqrt(disc)
    t = -b - sq
    if t > 1e-6:
        return t
    t = -b + sq
    return t if t > 1e-6 and c > 0 else (1e-6 if c <= 0 else None)


class Scene:
    """Ray casting over walls + solid objects. Rebuilt whenever the room changes."""

    def __init__(self, room: dict):
        self.wall_h = room["wall_h"]
        self.items = []    # (kind, id, height, data) kind: "seg" data=(x1,y1,x2,y2) | "circ" data=(cx,cy,r)
        for i, w in enumerate(room["walls"]):
            self.items.append(("seg", f"w{i}", self.wall_h, tuple(w)))
        for o in room["objects"]:
            if not o.get("solid", True):
                continue
            if "r" in o:
                self.items.append(("circ", o["id"], o["h"], (o["x"], o["y"], o["r"])))
            else:
                for s in object_segments(o):
                    self.items.append(("seg", o["id"], o["h"], tuple(s)))

    def ray(self, px, py, ang, maxd, skip: str | None = None, min_h: float = 0.0):
        """-> (t, id, height) of the first hit within maxd, or (maxd, None, 0)"""
        dx, dy = math.cos(ang), math.sin(ang)
        best, bid, bh = maxd, None, 0.0
        for kind, oid, h, d in self.items:
            if oid == skip or h < min_h:
                continue
            t = seg_hit(px, py, dx, dy, *d) if kind == "seg" else circle_hit(px, py, dx, dy, *d)
            if t is not None and t < best:
                best, bid, bh = t, oid, h
        return best, bid, bh


def dist_point_seg(px, py, x1, y1, x2, y2) -> float:
    ex, ey = x2 - x1, y2 - y1
    L = ex * ex + ey * ey
    u = 0.0 if L == 0 else max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / L))
    return math.hypot(px - x1 - u * ex, py - y1 - u * ey)


# ------------------------------------------------------------------ frames
def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def world_to_rel(start: dict, x: float, y: float, th: float = 0.0):
    """world -> start-pose-relative (forward, left, dth)"""
    c, s = math.cos(start["th"]), math.sin(start["th"])
    dx, dy = x - start["x"], y - start["y"]
    return c * dx + s * dy, -s * dx + c * dy, wrap(th - start["th"])


def rel_to_world(start: dict, f: float, l: float, dth: float = 0.0):
    c, s = math.cos(start["th"]), math.sin(start["th"])
    return start["x"] + c * f - s * l, start["y"] + s * f + c * l, wrap(start["th"] + dth)


def world_to_ark(start: dict, x, y, th=0.0):
    f, l, dth = world_to_rel(start, x, y, th)
    return l, f, dth                       # x_ark, z_ark, yaw


def ark_to_world(start: dict, xa, za, yaw=0.0):
    return rel_to_world(start, za, xa, yaw)


def motor_to_world(start: dict, x_mm, y_mm, h_deg):
    return rel_to_world(start, x_mm / 1000.0, -y_mm / 1000.0, -math.radians(h_deg))


def world_to_motor(start: dict, x, y, th=0.0):
    f, l, dth = world_to_rel(start, x, y, th)
    return f * 1000.0, -l * 1000.0, -math.degrees(dth)


# goto re-bases its memory frame whenever tars_motor's pose_epoch differs from the one in the file
# ("assume the robot is still where we last knew it"). An epoch tars_motor never uses makes the
# first STATUS goto reads re-base onto the 'pose' line below, i.e. onto where the car really is.
SEED_EPOCH = 4294967295


def goto_memory_text(room: dict, now: float | None = None, car=None) -> str | None:
    """room["memory"] -> goto memory.txt (format of test_fullstack.mem_file): 'inst' in motor mm,
    'navw' in the ark frame. An entry {"id": obj} remembers that object where it is now;
    {"id", "x", "y"} remembers it somewhere else (the moved-bin demo); {"label", "x", "y"} remembers
    a <label> at x, y with no object behind it. "state": fresh (default) | stale | gone.
    car = (x, y, th) world pose of the car right now: adds goto's 'pose' line so its memory frame
    (motor mm from the room start) agrees with where the car is, even away from the start.
    None = no memory and no car pose."""
    if not room.get("memory") and car is None:
        return None
    now = time.time() if now is None else now
    objs = {o["id"]: o for o in room["objects"]}
    out, k = ["v 2"], 0
    if car is not None:
        cx, cy, ch = world_to_motor(room["start"], car[0], car[1], car[2])
        out.append(f"pose {SEED_EPOCH} {cx:.1f} {cy:.1f} {ch:.2f} 0.0 0.0 0.000 {now:.1f}")
    for m in room.get("memory") or []:
        o = objs.get(m.get("id"))
        label = m.get("label") or (o and o["label"])
        if not label:
            continue
        x = float(m.get("x", o["x"] if o else 0.0))
        y = float(m.get("y", o["y"] if o else 0.0))
        k += 1
        state = m.get("state") if m.get("state") in MEM_STATES else "fresh"
        mx, my, _ = world_to_motor(room["start"], x, y)
        xa, za, _ = world_to_ark(room["start"], x, y)
        out.append(f"inst {k} {state} {mx:.1f} {my:.1f} 0.00 1000 0.800 3 {now:.1f} {label}")
        out.append(f"navw {k} {xa:.3f} {za:.3f} {label}")
    return "\n".join(out) + "\n" if k or car is not None else None


MEM_STATES = ("fresh", "stale", "gone")
_MEM_ENTRY = re.compile(r"^(?P<label>.+?)#(?P<id>\d+) (?P<state>\w+)(?P<rest>(?: \S+=\S*)*)\s*$")


def parse_goto_memory(reply: str) -> dict | None:
    """goto's MEMORY reply ("OK n=2 pose=x,y,h; trash can#1 fresh x=.. y=.. age=.. conf=.. map=xa,za; ...")
    -> {"n", "pose": (x_mm, y_mm, h_deg) | None, "entries": [{"label", "id", "state", "x_mm", "y_mm",
    "age", "conf", "map": (xa, za) | None}]}. None if it isn't an OK MEMORY reply."""
    reply = (reply or "").strip()
    if not reply.startswith("OK"):
        return None
    parts = [p.strip() for p in reply[2:].split(";")]
    head = dict(x.split("=", 1) for x in parts[0].split() if "=" in x)
    pose = None
    if "pose" in head:
        try:
            pose = tuple(float(v) for v in head["pose"].split(","))[:3]
        except ValueError:
            pose = None
    out = []
    for p in parts[1:]:
        m = _MEM_ENTRY.match(p)
        if not m:
            continue
        kv = dict(x.split("=", 1) for x in m["rest"].split() if "=" in x)

        def num(k, d=None):
            try:
                return float(kv[k])
            except (KeyError, ValueError):
                return d
        mp = None
        if kv.get("map") and kv["map"] != "-":
            try:
                xa, za = (float(v) for v in kv["map"].split(","))
                mp = (xa, za)
            except ValueError:
                mp = None
        out.append({"label": m["label"], "id": int(m["id"]), "state": m["state"], "x_mm": num("x", 0.0),
                    "y_mm": num("y", 0.0), "age": num("age"), "conf": num("conf"), "map": mp})
    return {"n": int(head.get("n", len(out)) or 0), "pose": pose, "entries": out}


def memory_to_world(start: dict, parsed: dict, car=None) -> list[dict]:
    """goto's parsed MEMORY -> room-frame entries [{"key": "label#id", "label", "id", "state", "x", "y",
    "age", "conf", "frame"}]. Uses the entry's map (ARKit) position when goto has one (fixed to the
    room in the sim); otherwise places it relative to the car: goto's memory-frame car pose
    (MEMORY pose=) against the car's real world pose `car` (x, y, th). Last resort: motor mm from the
    room start."""
    pose = parsed.get("pose")
    res = []
    for e in parsed.get("entries", []):
        if e.get("map"):
            x, y, _ = ark_to_world(start, e["map"][0], e["map"][1])
            frame = "map"
        elif car is not None and pose is not None:
            px, py, ph = pose
            dx, dy = (e["x_mm"] - px) / 1000.0, -(e["y_mm"] - py) / 1000.0   # memory frame, y left
            a = car[2] + math.radians(ph)
            x = car[0] + math.cos(a) * dx - math.sin(a) * dy
            y = car[1] + math.sin(a) * dx + math.cos(a) * dy
            frame = "car"
        else:
            x, y, _ = motor_to_world(start, e["x_mm"], e["y_mm"], 0.0)
            frame = "start"
        res.append({"key": f"{e['label']}#{e['id']}", "label": e["label"], "id": e["id"], "state": e["state"],
                    "x": round(x, 3), "y": round(y, 3), "age": e.get("age"), "conf": e.get("conf"), "frame": frame})
    return res


# ------------------------------------------------------------------ goto config
def goto_conf(path: str | None = None) -> dict:
    path = path or os.path.join(GOTO_DIR, "goto.conf")
    out = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if "=" in line:
                    k, v = (s.strip() for s in line.split("=", 1))
                    try:
                        out[k] = float(v)
                    except ValueError:
                        out[k] = v
    except OSError:
        pass
    return out


def goto_objects(path: str | None = None) -> list[str]:
    """canonical object names, in objects.txt order"""
    path = path or os.path.join(GOTO_DIR, "objects.txt")
    names = []
    try:
        with open(path) as f:
            for line in f:
                line = line.split("#", 1)[0].strip()
                if ":" in line:
                    names.append(line.split(":", 1)[0].strip())
    except OSError:
        pass
    return names
