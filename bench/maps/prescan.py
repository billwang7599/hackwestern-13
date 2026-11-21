"""Prescans and occupancy grids for the bench's map demo modes (laptop only, SIMULATION).

A prescan is a saved map of a room: goto's occupancy grid, the objects seen in it and Andrew's
surface splats, all in the scan's ARKit frame. The bench turns it into
  * a sim room (walls from the occupied cells, so the virtual car collides with what was scanned),
  * a seed for goto's nav map (LDR1 packets, one point per mapped cell, through the world sim),
  * goto memory (the objects), and
  * the 3D backdrop (the splats, into the phone part's map).

FRAME: the sim world frame IS the scan's ARKit frame. A prescan room has start (0, 0, 0), so
    world x = z_ark (forward at the scan's start), world y = x_ark (left), z = y_ark - floor_y
(the same mapping as bench/world/common.py with start = origin, and bench/phone/sim.py).

Grid layout = goto's nav.h and Andrew's mapping.py: 200 x 200 cells of 0.1 m, cell (0, 0) has its
corner at ark (-10, -10), index = iz * 200 + ix, values 0 unknown / 1 free / 2 occupied.

File format (bench/world/prescans/<name>.json):
  {"format": "tars-bench-prescan", "version": 1, "name", "source", "created", "frame",
   "floor_y": 0.0, "grid": {"size": 200, "res": 0.1, "origin": [-10, -10],
   "box": [ix0, iz0, w, h], "rle": "v x n,..."},            (goto NAV GRID's run-length code)
   "objects": [{"label", "x_ark", "z_ark", "conf"}],
   "surfaces": [[x_cm, y_cm, z_cm, nx, ny, nz, r, g, b, size_cm], ...]}   (mapping.py rows)
"""
from __future__ import annotations

import base64
import json
import math
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
PRESCANS = os.environ.get("BENCH_PRESCANS") or os.path.join(BENCH, "world", "prescans")   # tests: a temp dir
DASHBOARD = os.path.join(REPO, "iter2-autonomy", "dashboard")

SIZE, RES, ORIGIN = 200, 0.1, (-10.0, -10.0)
UNKNOWN, FREE, OCC = 0, 1, 2
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
FRAME = ("the scan's ARKit frame: x_ark left, y up, z_ark forward at the scan start; "
         "bench world x = z_ark, y = x_ark, z = y - floor_y (prescan rooms start at the origin)")
MAX_SURFACES = 1500
MAX_WALL_SEGS = 220          # the room travels in one UDP datagram (macOS caps those at 9216 bytes)

# default footprints for objects placed from a scan (same as the 2D editor's defaults)
SIZES = {"trash can": {"r": 0.15, "h": 0.4}, "chair": {"w": 0.45, "d": 0.45, "h": 0.85},
         "person": {"r": 0.2, "h": 1.7}, "backpack": {"w": 0.32, "d": 0.22, "h": 0.45},
         "water bottle": {"r": 0.04, "h": 0.25}, "table": {"w": 1.2, "d": 0.7, "h": 0.75},
         "laptop": {"w": 0.32, "d": 0.22, "h": 0.02}, "cup": {"r": 0.05, "h": 0.1},
         "box": {"w": 0.4, "d": 0.3, "h": 0.3}}


# ------------------------------------------------------------------ cells and frames
def cell_center(ix: int, iz: int) -> tuple[float, float]:
    return ORIGIN[0] + (ix + 0.5) * RES, ORIGIN[1] + (iz + 0.5) * RES


def ark_cell(x: float, z: float):
    ix, iz = math.floor((x - ORIGIN[0]) / RES), math.floor((z - ORIGIN[1]) / RES)
    return (ix, iz) if 0 <= ix < SIZE and 0 <= iz < SIZE else None


def ark_to_world(xa: float, za: float) -> tuple[float, float]:
    return za, xa


def world_to_ark(x: float, y: float) -> tuple[float, float]:
    return y, x


# ------------------------------------------------------------------ goto NAV GRID <-> bytes
def parse_nav_grid(reply: str) -> bytearray | None:
    """goto 'OK grid v=.. size=200 res=0.10 origin=-10.00,-10.00 box=ix0,iz0,w,h rle=v x n,...'
    -> a 200*200 bytearray (index iz*200+ix), or None if the reply isn't a grid."""
    if not isinstance(reply, str) or not reply.startswith("OK grid"):
        return None
    kv = dict(t.split("=", 1) for t in reply.split()[2:] if "=" in t)
    if int(kv.get("size", SIZE)) != SIZE:
        return None
    g = bytearray(SIZE * SIZE)
    if kv.get("box", "-") == "-":
        return g
    return decode_rle(kv["box"], kv.get("rle", "-"), g)


def decode_rle(box, rle: str, g: bytearray | None = None) -> bytearray:
    g = g if g is not None else bytearray(SIZE * SIZE)
    x0, z0, w, h = (int(v) for v in (box.split(",") if isinstance(box, str) else box))
    k = 0
    if rle in ("", "-"):
        return g
    for run in rle.split(","):
        v, n = (int(a) for a in run.split("x"))
        for _ in range(n):
            iz, ix = z0 + k // w, x0 + k % w
            if 0 <= ix < SIZE and 0 <= iz < SIZE and k < w * h:
                g[iz * SIZE + ix] = v
            k += 1
    return g


def encode_rle(g) -> dict:
    """bounding box of known cells + run-length code, as goto's NAV GRID writes it"""
    xs = [i % SIZE for i, v in enumerate(g) if v]
    if not xs:
        return {"box": None, "rle": "-"}
    zs = [i // SIZE for i, v in enumerate(g) if v]
    x0, x1, z0, z1 = min(xs), max(xs), min(zs), max(zs)
    runs, rv, rn = [], -1, 0
    for iz in range(z0, z1 + 1):
        for ix in range(x0, x1 + 1):
            v = g[iz * SIZE + ix]
            if v == rv:
                rn += 1
                continue
            if rn:
                runs.append(f"{rv}x{rn}")
            rv, rn = v, 1
    runs.append(f"{rv}x{rn}")
    return {"box": [x0, z0, x1 - x0 + 1, z1 - z0 + 1], "rle": ",".join(runs)}


def grid_from_prescan(p: dict) -> bytearray:
    gr = p["grid"]
    if gr.get("box") is None:
        return bytearray(SIZE * SIZE)
    return decode_rle(gr["box"], gr["rle"])


def counts(g) -> dict:
    return {"free": g.count(FREE), "occ": g.count(OCC)}


# ------------------------------------------------------------------ goto MEMORY -> objects
_MEM = re.compile(r";\s*([^;#]+?)#(\d+)\s+(\S+)[^;]*?\bconf=([\d.]+)(?:[^;]*?\bmap=(-?[\d.]+),(-?[\d.]+))?")


def parse_memory(reply: str) -> list[dict]:
    """goto 'OK n=.. pose=..; trash can#1 fresh x=.. ... conf=0.80 map=0.01,2.40; ...' -> objects with a
    map (ARKit) position. Entries without map= (dead-reckoning only) are skipped."""
    out = []
    for m in _MEM.finditer(reply or ""):
        if m.group(5) is None:
            continue
        out.append({"label": m.group(1).strip(), "id": int(m.group(2)), "status": m.group(3),
                    "conf": float(m.group(4)), "x_ark": float(m.group(5)), "z_ark": float(m.group(6))})
    return out


# ------------------------------------------------------------------ prescan files
def new_prescan(name: str, source: str, grid, objects, surfaces, floor_y: float = 0.0, about: str = "") -> dict:
    surfaces = [list(map(int, s)) for s in surfaces]
    if len(surfaces) > MAX_SURFACES:
        step = math.ceil(len(surfaces) / MAX_SURFACES)
        surfaces = surfaces[::step]
    return {"format": "tars-bench-prescan", "version": 1, "name": name, "source": source, "about": about,
            "created": time.strftime("%Y-%m-%d %H:%M:%S"), "frame": FRAME, "floor_y": floor_y,
            "grid": {"size": SIZE, "res": RES, "origin": list(ORIGIN), **encode_rle(grid)},
            "objects": [{"label": o["label"], "x_ark": round(float(o["x_ark"]), 3),
                         "z_ark": round(float(o["z_ark"]), 3), "conf": round(float(o.get("conf", 0.8)), 2)}
                        for o in _dedupe(objects)],
            "surfaces": surfaces}


def list_prescans(directory: str = PRESCANS) -> list[str]:
    try:
        return sorted(f[:-5] for f in os.listdir(directory) if f.endswith(".json"))
    except OSError:
        return []


def save_prescan(p: dict, directory: str = PRESCANS, overwrite: bool = False) -> str:
    if not NAME_RE.match(p["name"]):
        raise ValueError("name: letters, digits, _ or -, up to 40")
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, p["name"] + ".json")
    if os.path.exists(path) and not overwrite:
        raise FileExistsError(f"{p['name']} exists")
    with open(path + ".tmp", "w") as f:
        json.dump(p, f, separators=(",", ":"))
        f.write("\n")
    os.replace(path + ".tmp", path)
    return path


def load_prescan(name_or_path: str, directory: str = PRESCANS) -> dict:
    path = name_or_path if name_or_path.endswith(".json") else os.path.join(directory, name_or_path + ".json")
    if not name_or_path.endswith(".json") and not NAME_RE.match(name_or_path):
        raise ValueError(f"bad prescan name {name_or_path!r}")
    with open(path) as f:
        p = json.load(f)
    if p.get("format") != "tars-bench-prescan" or p.get("version") != 1:
        raise ValueError("not a tars-bench-prescan v1 file")
    return p


# ------------------------------------------------------------------ import: a real scan
def _mapping():
    if DASHBOARD not in sys.path:
        sys.path.insert(0, DASHBOARD)
    import mapping  # Andrew's, unchanged
    return mapping


def from_mapper(mapper, name: str, source: str, about: str = "") -> dict:
    """A prescan from one of Andrew's mapping.Map objects (native=None: its own grid)."""
    with mapper.lock:
        floor = mapper.floor_y if mapper.floor_y is not None else 0.0
        objs = [{"label": o["label"], "x_ark": o["position"][0], "z_ark": o["position"][2],
                 "conf": o.get("confidence", 0.8)} for o in mapper.objects.values()]
        surfaces = [_shift_surface(s, floor) for s in mapper.surfaces.values()]
        grid = bytearray(mapper.grid)
    return new_prescan(name, source, grid, _dedupe(objs), surfaces, 0.0, about)


def _shift_surface(row, floor_y: float):
    """surface heights relative to the floor (the sim floor is y = 0)"""
    r = list(row)
    r[1] = int(round(r[1] - 100 * floor_y))
    return r


def _dedupe(objs: list[dict], radius: float = 0.6) -> list[dict]:
    """one object per label within `radius` (the most confident): goto's memory (before the fix on
    kothar/goto-fixes 466274c) and the phone's detections both keep repeat sightings of one thing"""
    out = []
    for o in sorted(objs, key=lambda o: -o.get("conf", 0)):
        if not any(q["label"] == o["label"] and math.hypot(q["x_ark"] - o["x_ark"], q["z_ark"] - o["z_ark"]) < radius
                   for q in out):
            out.append(o)
    return out


def from_recording(path: str, name: str) -> dict:
    """A phone3d recording (bench/recordings/*.jsonl or bench/phone/recordings/*.jsonl: a header line,
    then {"t", "scene"} per scene) replayed offline through Andrew's Map -> a prescan."""
    mapping = _mapping()
    clock = [0.0]
    m = mapping.Map(save_path=os.path.join(os.path.dirname(path) or ".", ".unused-map.json"),
                    clock=lambda: clock[0], native=None)
    n = bad = 0
    with open(path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                bad += 1
                continue
            scene = rec.get("scene") if isinstance(rec, dict) else None
            if not isinstance(scene, dict):
                continue
            clock[0] = float(rec.get("t", clock[0] + 0.1))
            try:
                m.update(scene)
                n += 1
            except ValueError:
                bad += 1
    if not n:
        raise ValueError(f"no usable scenes in {path}")
    p = from_mapper(m, name, f"recording:{os.path.basename(path)}",
                    f"{n} scenes replayed through mapping.Map" + (f", {bad} rejected" if bad else ""))
    if m.floor_y is None:
        p["about"] += "; WARNING: the floor was never calibrated, so the grid is empty"
    return p


def from_mapper_save(path: str, name: str) -> dict:
    """Andrew's mapper save file (mapping.Map._saved(): version, session, floor_y, grid, surfaces,
    voxels, objects)."""
    with open(path) as f:
        v = json.load(f)
    if not isinstance(v, dict) or v.get("version") != 1 or not isinstance(v.get("grid"), str) \
            or len(v["grid"]) != SIZE * SIZE:
        raise ValueError("not a mapping.py map save (version 1, 200x200 grid)")
    floor = float(v.get("floor_y") or 0.0)
    grid = bytearray(int(c) for c in v["grid"])
    objs = [{"label": o["label"], "x_ark": o["position"][0], "z_ark": o["position"][2],
             "conf": o.get("confidence", 0.8)} for o in v.get("objects", [])]
    surf = [_shift_surface(s, floor) for s in v.get("surfaces", [])]
    return new_prescan(name, f"mapper-save:{os.path.basename(path)}", grid, _dedupe(objs), surf, 0.0,
                       f"session {v.get('session', '?')}")


# ------------------------------------------------------------------ prescan -> sim room
def _footprint_cells(objects) -> set:
    """cells under (and just around) the scanned objects: the object replaces them"""
    out = set()
    for o in objects:
        size = SIZES.get(o["label"], {"w": 0.3, "d": 0.3})
        r = size.get("r", 0.5 * max(size.get("w", 0.3), size.get("d", 0.3))) + 0.15
        n = int(math.ceil(r / RES)) + 1
        c = ark_cell(o["x_ark"], o["z_ark"])
        if c is None:
            continue
        for dx in range(-n, n + 1):
            for dz in range(-n, n + 1):
                cx, cz = cell_center(c[0] + dx, c[1] + dz)
                if math.hypot(cx - o["x_ark"], cz - o["z_ark"]) <= r:
                    out.add((c[0] + dx, c[1] + dz))
    return out


def grid_to_walls(g, skip: set = frozenset(), max_segs: int = MAX_WALL_SEGS) -> tuple[list, int]:
    """Occupied cells -> wall segments in the WORLD frame. Rows of 2+ cells along ark x become one
    segment through their centre line; the cells left over are joined along ark z. A lone cell is a
    0.1 m segment. Returns (segments, dropped) where dropped = segments over the cap (shortest go)."""
    occ = {(i % SIZE, i // SIZE) for i, v in enumerate(g) if v == OCC} - set(skip)
    used, segs = set(), []
    for iz in sorted({c[1] for c in occ}):
        row = sorted(ix for ix, z in occ if z == iz)
        k = 0
        while k < len(row):
            j = k
            while j + 1 < len(row) and row[j + 1] == row[j] + 1:
                j += 1
            if j > k:
                zc = ORIGIN[1] + (iz + 0.5) * RES
                xa0, xa1 = ORIGIN[0] + row[k] * RES, ORIGIN[0] + (row[j] + 1) * RES
                segs.append((xa0, zc, xa1, zc))
                used.update((ix, iz) for ix in row[k:j + 1])
            k = j + 1
    rest = occ - used
    for ix in sorted({c[0] for c in rest}):
        col = sorted(iz for x, iz in rest if x == ix)
        k = 0
        while k < len(col):
            j = k
            while j + 1 < len(col) and col[j + 1] == col[j] + 1:
                j += 1
            xc = ORIGIN[0] + (ix + 0.5) * RES
            za0, za1 = ORIGIN[1] + col[k] * RES, ORIGIN[1] + (col[j] + 1) * RES
            segs.append((xc, za0, xc, za1))
            k = j + 1
    segs.sort(key=lambda s: -math.hypot(s[2] - s[0], s[3] - s[1]))
    dropped = max(0, len(segs) - max_segs)
    segs = segs[:max_segs]
    out = []
    for xa0, za0, xa1, za1 in segs:
        x0, y0 = ark_to_world(xa0, za0)
        x1, y1 = ark_to_world(xa1, za1)
        out.append([round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)])
    return out, dropped


def to_room(p: dict, name: str | None = None) -> tuple[dict, dict]:
    """prescan -> (sim room in the bench room format, info). Objects from the scan become room
    objects with default sizes, and goto remembers all of them."""
    g = grid_from_prescan(p)
    objs = p.get("objects", [])
    walls, dropped = grid_to_walls(g, _footprint_cells(objs))
    room_objs, memory, ids = [], [], set()
    for o in objs:
        base = re.sub(r"[^A-Za-z0-9_]", "_", o["label"]) or "obj"
        k = 1
        while f"{base}{k}" in ids:
            k += 1
        oid = f"{base}{k}"
        ids.add(oid)
        x, y = ark_to_world(o["x_ark"], o["z_ark"])
        room_objs.append({"id": oid, "label": o["label"], "x": round(x, 3), "y": round(y, 3),
                          **SIZES.get(o["label"], {"w": 0.3, "d": 0.3, "h": 0.5})})
        memory.append({"id": oid})
    room = {"name": name or f"prescan_{p['name']}"[:40],
            "about": f"PRESCAN (simulation only) from {p.get('source', '?')}: walls rebuilt from the scan's "
                     f"occupied cells, objects where the scan saw them. {p.get('about', '')}".strip(),
            "start": {"x": 0.0, "y": 0.0, "th": 0.0}, "wall_h": 1.2, "walls": walls,
            "objects": room_objs, "memory": memory, "prescan": p["name"]}
    info = {"walls": len(walls), "walls_dropped": dropped, "objects": len(room_objs), **counts(g)}
    return room, info


def memory_text(room: dict, drop: dict, now: float | None = None) -> str | None:
    """goto memory.txt for a prescan room with the car dropped at `drop` (world x, y, th):
    'inst' in tars_motor's frame (POSE RESET at the drop pose), 'navw' in the ARKit (scan) frame.
    Same line format as bench/world/common.goto_memory_text."""
    sys.path.insert(0, os.path.join(BENCH, "world"))
    import common as C
    now = time.time() if now is None else now
    objs = {o["id"]: o for o in room["objects"]}
    out, k = ["v 2"], 0
    for m in room.get("memory", []):
        o = objs.get(m.get("id"))
        if not o or not o["label"]:
            continue
        k += 1
        mx, my, _ = C.world_to_motor(drop, o["x"], o["y"])
        xa, za, _ = C.world_to_ark(room["start"], o["x"], o["y"])
        out.append(f"inst {k} fresh {mx:.1f} {my:.1f} 0.00 1000 0.800 3 {now:.1f} {o['label']}")
        out.append(f"navw {k} {xa:.3f} {za:.3f} {o['label']}")
    return "\n".join(out) + "\n" if k else None


def seed_points(g) -> list[list[float]]:
    """one LDR1 point per mapped cell, ARKit frame, floor at y = 0: free -> a floor hit, occupied ->
    a hit 0.3 m up (nav.c: |h| <= 0.06 free, 0.08..1.2 occupied)"""
    pts = []
    for i, v in enumerate(g):
        if v in (FREE, OCC):
            x, z = cell_center(i % SIZE, i // SIZE)
            pts.append([round(x, 3), 0.0 if v == FREE else 0.3, round(z, 3)])
    return pts


def surfaces_b64(rows) -> str:
    import struct
    rec = struct.Struct("<3h3b4B")
    return base64.b64encode(b"".join(rec.pack(*r) for r in rows)).decode("ascii")
