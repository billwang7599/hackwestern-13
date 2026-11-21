"""bench plugin `maps`: the two map demo modes (SIMULATION; see bench/maps/README.md).

  A. Explore live: the car starts knowing nothing (fresh goto: empty nav map, no memory; the 3D map
     cleared), then goto SCAN and/or a bench-side explorer (SCAN, frontier hop, SCAN, ...) until goto
     remembers the target, then GOTO it. goto's map only ever holds what the sim LiDAR saw.
  B. Prescan: a saved map (from a sim run of A, an iPhone recording, or Andrew's map save) becomes the
     sim room (walls from its occupied cells), goto's prior map (seeded through the phone feed) and
     memory (its objects), and the 3D backdrop (its splats). Drop the car anywhere, then GOTO.

Routes (/api/maps/...):
  GET  info                      prescans, recordings, explorer state, the loaded prescan
  POST fresh {}                  mode A start: reload the current room with no memory, fresh goto, 3D map cleared
  POST scan {}                   goto SCAN
  POST explore {target, max_hops, then_goto}   start the explorer   · POST explore_stop {}  (S + CANCEL)
  POST save_prescan {name, overwrite}          goto's map + memory + the 3D map's splats -> prescans/<name>.json
  POST import {kind: recording|mapper, path, name, overwrite}   a real scan -> prescans/<name>.json
  POST load_prescan {name, drop?: {x, y, th}}  mode B
  POST drop {x, y, th}           put the car there; goto restarted (memory relative to the drop), map re-seeded
Topics: `navgrid` (goto NAV GRID, ~1 Hz on change), `maps` (explorer + mode state).
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

_BENCH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BENCH not in sys.path:
    sys.path.insert(0, _BENCH)
from maps import explore as E  # noqa: E402
from maps import prescan as PS  # noqa: E402

RECORDING_DIRS = [os.path.join(_BENCH, "recordings"), os.path.join(_BENCH, "phone", "recordings")]

_bench = None
_stop = threading.Event()
_explorer: E.Explorer | None = None
_mode = {"mode": None, "prescan": None, "room": None, "drop": None, "seed": None, "note": ""}
_lock = threading.Lock()
_grid_v = [None]


def _p(name):
    return _bench.plugins.get(name) if _bench else None


def _send(dest, line, timeout=3.0, src="maps"):
    r = _p("robot")
    if r is None:
        return "ERR robot plugin not loaded"
    try:
        return r.send(dest, line, timeout=timeout, src=src)
    except TypeError:
        return r.send(dest, line)


def _body(b: bytes) -> dict:
    try:
        return json.loads(b or b"{}")
    except ValueError:
        return {}


def _pub_mode():
    ex = _explorer.state if _explorer else None
    _bench.publish("maps", {"mode": _mode["mode"], "prescan": _mode["prescan"], "drop": _mode["drop"],
                            "seed": _mode["seed"], "note": _mode["note"], "explore": ex})


def _world_room() -> dict | None:
    w = _p("world")
    if w is None:
        return None
    rep = w.ctl({"op": "get_room"})
    return rep.get("room") if rep.get("ok") else None


def _phone_pipe():
    ph = _p("phone")
    return getattr(ph, "pipe", None) if ph else None


# ------------------------------------------------------------------ nav grid -> topic
def _grid_loop():
    while not _stop.wait(1.0):
        r = _p("robot")
        if r is None:
            continue
        try:
            rep = r.send("goto", "NAV GRID", timeout=2.0, src="poll")
        except Exception:
            continue
        if not isinstance(rep, str) or not rep.startswith("OK grid"):
            continue
        kv = dict(t.split("=", 1) for t in rep.split()[2:] if "=" in t)
        key = (kv.get("v"), kv.get("box"))
        if key == _grid_v[0]:
            continue
        _grid_v[0] = key
        g = PS.parse_nav_grid(rep)
        _bench.publish("navgrid", {"v": kv.get("v"), "box": kv.get("box"), "rle": kv.get("rle"),
                                   "res": PS.RES, "origin": list(PS.ORIGIN), "size": PS.SIZE,
                                   **(PS.counts(g) if g is not None else {})})


# ------------------------------------------------------------------ shared steps
def _restart_goto(memory: str | None) -> dict:
    w = _p("world")
    sup = w.supervisor({"op": "restart_goto", "memory": memory}) if w else None
    if sup is None:
        return {"ok": False, "error": "not started by bench/run_sim.py: can't restart goto (no supervisor)"}
    _grid_v[0] = None
    return sup


def _wait_nav(timeout=8.0) -> dict | None:
    """until the fresh goto gets live phone packets with an ARKit pose"""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rep = _send("goto", "NAV", 1.0, "poll")
        p = E.parse_nav(rep)
        kv = dict(t.split("=", 1) for t in rep.split()[1:] if "=" in t) if rep.startswith("OK") else {}
        try:
            if p and p[0] == "arkit" and int(kv.get("packets", 0)) > 3 and 0 <= float(kv.get("feed_age", -1)) < 0.5:
                return kv
        except ValueError:
            pass
        time.sleep(0.2)
    return None


def _seed(g) -> dict:
    """the prescan's cells into goto's nav map through the world sim's phone feed; verified"""
    w = _p("world")
    pts = PS.seed_points(g)
    want = PS.counts(g)
    out = {"points": len(pts), "want": want}
    for attempt in (1, 2):
        rep = w.ctl({"op": "seed_nav", "points": pts}, timeout=10.0)
        if not rep.get("ok"):
            return {**out, "ok": False, "error": rep.get("error")}
        time.sleep(0.6)
        p = E.parse_nav(_send("goto", "NAV", 2.0, "poll"))
        got = {"free": p[4], "occ": p[5]} if p else None
        out.update(got=got, attempts=attempt, packets=rep.get("packets"))
        if got and got["free"] >= 0.9 * want["free"]:
            return {**out, "ok": True}
    return {**out, "ok": False, "error": "goto's map didn't take the seed"}


def _backdrop(p: dict | None):
    """the 3D view: clear Andrew's map; with a prescan, its splats go in as the backdrop"""
    pipe = _phone_pipe()
    if pipe is None:
        return "phone part not loaded"
    pipe.command({"op": "map_clear"})
    if p and p.get("surfaces"):
        m = pipe.mapper
        with m.lock:
            for row in p["surfaces"]:
                m.surfaces[tuple(v // 10 for v in row[:3])] = list(row)   # mapping.surface_key
        return f"{len(p['surfaces'])} prescan splats in the 3D view"
    return "3D map cleared"


def _car_at(x, y, th, timeout=3.0) -> bool:
    w = _p("world")
    w.ctl({"op": "teleport", "x": x, "y": y, "th": th})
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = w.ctl({"op": "get_state"}).get("state", {})
        t = st.get("truth") or {}
        if t and math.hypot(t.get("x", 1e9) - x, t.get("y", 1e9) - y) < 0.05:
            return True
        time.sleep(0.1)
    return False


def _stop_explorer():
    global _explorer
    if _explorer is not None and _explorer.state.get("running"):
        _explorer.cancel()
        if _explorer.thread:
            _explorer.thread.join(5)


# ------------------------------------------------------------------ routes
def r_info(q, b):
    recs = []
    for d in RECORDING_DIRS:
        try:
            recs += [os.path.relpath(os.path.join(d, f), PS.REPO) for f in sorted(os.listdir(d)) if f.endswith(".jsonl")]
        except OSError:
            pass
    return 200, {"prescans": PS.list_prescans(), "recordings": recs, "mode": _mode,
                 "explore": _explorer.state if _explorer else None,
                 "supervisor": bool(getattr(_p("world"), "SUP", 0)), "robot": _p("robot") is not None}


def r_fresh(q, b):
    """mode A: same room, nothing remembered, empty maps"""
    _stop_explorer()
    w = _p("world")
    if w is None:
        return 503, {"ok": False, "error": "world plugin not loaded (start with bench/run_sim.py)"}
    room = _body(b).get("room") or _world_room()
    if not room:
        return 503, {"ok": False, "error": "no room from the world sim"}
    room = {k: v for k, v in room.items() if k in ("name", "about", "start", "wall_h", "walls", "objects")}
    room["memory"] = []
    rep = w.ctl({"op": "load", "room": room})
    if not rep.get("ok"):
        return 400, rep
    sup = _restart_goto(None)
    nav = _wait_nav()
    with _lock:
        _mode.update(mode="explore", prescan=None, room=room.get("name"), drop=None, seed=None,
                     note="Mode A: goto knows nothing; its map fills only from what the sim LiDAR sees")
    out = {"ok": bool(sup.get("ok")), "goto_restart": sup, "nav_live": nav is not None, "phone": _backdrop(None)}
    _bench.log("maps", f"explore mode: room {room.get('name')} reloaded with no memory, goto fresh")
    _pub_mode()
    return 200, out


def r_scan(q, b):
    r = _send("goto", "SCAN")
    _bench.log("maps", f"SCAN -> {r}")
    return 200, {"ok": r.startswith("OK"), "reply": r}


def r_explore(q, b):
    global _explorer
    robot = _p("robot")
    if robot is None:
        return 503, {"ok": False, "error": "robot plugin not loaded"}
    if _explorer is not None and _explorer.state.get("running"):
        return 409, {"ok": False, "error": "already exploring"}
    m = _body(b)
    _explorer = E.Explorer(robot, lambda st: _pub_mode(), _bench.log, target=str(m.get("target") or "trash can"),
                           max_hops=int(m.get("max_hops", 6)), then_goto=bool(m.get("then_goto", True))).start()
    return 200, {"ok": True}


def r_explore_stop(q, b):
    if _explorer is not None:
        _explorer.cancel()
    r = _p("robot")
    rep = r.stop("explore stop") if r else {}
    return 200, {"ok": True, "stop": rep}


def r_save_prescan(q, b):
    m = _body(b)
    name = str(m.get("name", "")).strip()
    if not PS.NAME_RE.match(name):
        return 400, {"ok": False, "error": "name: letters, digits, _ or -, up to 40"}
    g = PS.parse_nav_grid(_send("goto", "NAV GRID", 3.0))
    if g is None:
        return 503, {"ok": False, "error": "no NAV GRID from goto"}
    objs = PS.parse_memory(_send("goto", "MEMORY", 3.0))
    objs = [o for o in objs if o["status"] != "gone"]
    pipe = _phone_pipe()
    surf = []
    room = _world_room() or {}
    if pipe is not None:
        with pipe.mapper.lock:
            surf = [list(s) for s in pipe.mapper.surfaces.values()]
        st = room.get("start") or {"x": 0, "y": 0, "th": 0}
        if any(abs(st.get(k, 0)) > 1e-6 for k in ("x", "y", "th")):
            surf = [_phone_to_start(s, st) for s in surf]   # phone sim frame (world origin) -> goto's frame
    p = PS.new_prescan(name, f"sim:{room.get('name', '?')}", g, objs, surf,
                       about=f"made in the bench sim by scanning room {room.get('name', '?')}")
    try:
        path = PS.save_prescan(p, overwrite=bool(m.get("overwrite")))
    except FileExistsError as e:
        return 409, {"ok": False, "error": str(e), "exists": True}
    info = {**PS.counts(g), "objects": [o["label"] for o in p["objects"]], "surfaces": len(p["surfaces"]),
            "bytes": os.path.getsize(path), "memory_entries": len(objs)}
    _bench.log("maps", f"prescan saved: {os.path.relpath(path, PS.REPO)} {info}")
    return 200, {"ok": True, "path": os.path.relpath(path, PS.REPO), **info}


def _phone_to_start(row, start):
    sys.path.insert(0, os.path.join(_BENCH, "world"))
    import common as C
    x, y = row[2] / 100, row[0] / 100            # phone sim ARKit (y_w, z, x_w), cm
    xa, za, _ = C.world_to_ark(start, x, y)
    c, s = math.cos(start["th"]), math.sin(start["th"])
    nx, ny = row[5], row[3]
    nf, nl = c * nx + s * ny, -s * nx + c * ny
    return [int(round(100 * xa)), row[1], int(round(100 * za)), int(round(nl)), row[4], int(round(nf)), *row[6:]]


def r_import(q, b):
    m = _body(b)
    name = str(m.get("name", "")).strip()
    path = str(m.get("path", ""))
    if not os.path.isabs(path):
        path = os.path.join(PS.REPO, path)
    try:
        if m.get("kind") == "mapper":
            p = PS.from_mapper_save(path, name)
        else:
            p = PS.from_recording(path, name)
        out = PS.save_prescan(p, overwrite=bool(m.get("overwrite")))
    except FileExistsError as e:
        return 409, {"ok": False, "error": str(e), "exists": True}
    except (OSError, ValueError, KeyError, TypeError) as e:
        return 400, {"ok": False, "error": f"import: {e}"}
    g = PS.grid_from_prescan(p)
    info = {**PS.counts(g), "objects": [o["label"] for o in p["objects"]], "surfaces": len(p["surfaces"]),
            "about": p.get("about", "")}
    _bench.log("maps", f"imported {os.path.relpath(path, PS.REPO)} -> {os.path.relpath(out, PS.REPO)} {info}")
    return 200, {"ok": True, "path": os.path.relpath(out, PS.REPO), **info}


def _place(room: dict, p: dict | None, drop: dict) -> dict:
    """car to `drop`, fresh goto with memory relative to it, then (prescan) seed its map"""
    out = {"car_at_drop": _car_at(drop["x"], drop["y"], drop["th"])}
    out["goto_restart"] = _restart_goto(PS.memory_text(room, drop))
    out["nav_live"] = _wait_nav() is not None
    if p is not None:
        out["seed"] = _seed(PS.grid_from_prescan(p)) if out["nav_live"] else {"ok": False, "error": "no live nav feed"}
    return out


def r_load_prescan(q, b):
    _stop_explorer()
    m = _body(b)
    w = _p("world")
    if w is None or not getattr(w, "SUP", 0):
        return 503, {"ok": False, "error": "needs the sim stack: start with bench/run_sim.py"}
    try:
        p = PS.load_prescan(str(m.get("name", "")))
    except (OSError, ValueError) as e:
        return 400, {"ok": False, "error": f"prescan: {e}"}
    room, info = PS.to_room(p)
    for cap in (PS.MAX_WALL_SEGS, 160, 110, 70):          # one UDP datagram for the world state
        if len(json.dumps(room)) < 7000:
            break
        g = PS.grid_from_prescan(p)
        room["walls"], dropped = PS.grid_to_walls(g, PS._footprint_cells(p["objects"]), cap)
        info.update(walls=len(room["walls"]), walls_dropped=dropped)
    rep = w.ctl({"op": "load", "room": room})
    if not rep.get("ok"):
        return 400, rep
    d = m.get("drop") or {}
    drop = {"x": float(d.get("x", 0.0)), "y": float(d.get("y", 0.0)), "th": float(d.get("th", 0.0))}
    out = _place(room, p, drop)
    out["phone"] = _backdrop(p)
    with _lock:
        _mode.update(mode="prescan", prescan=p["name"], room=room["name"], drop=drop, seed=out.get("seed"),
                     note="Mode B (SIMULATION ONLY): room, goto map and memory come from the prescan")
    _bench.log("maps", f"prescan {p['name']} loaded: {info}, seed {out.get('seed')}")
    _pub_mode()
    ok = out["goto_restart"].get("ok") and (out.get("seed") or {}).get("ok")
    return 200, {"ok": bool(ok), "room": info, **out}


def r_drop(q, b):
    _stop_explorer()
    m = _body(b)
    room = _world_room()
    if room is None:
        return 503, {"ok": False, "error": "no world sim"}
    drop = {"x": float(m.get("x", 0)), "y": float(m.get("y", 0)), "th": float(m.get("th", 0))}
    p = None
    if _mode["mode"] == "prescan" and _mode["prescan"]:
        try:
            p = PS.load_prescan(_mode["prescan"])
        except (OSError, ValueError):
            p = None
    if _mode["mode"] == "explore":
        room = dict(room, memory=[])
    out = _place(room, p, drop)
    with _lock:
        _mode.update(drop=drop, seed=out.get("seed"))
    _bench.log("maps", f"car dropped at {drop['x']:.2f}, {drop['y']:.2f}, {math.degrees(drop['th']):.0f} deg: {out}")
    _pub_mode()
    return 200, {"ok": bool(out["goto_restart"].get("ok")), **out}


def setup(bench):
    global _bench
    _bench = bench
    bench.route("GET", "/api/maps/info", r_info)
    for name, fn in (("fresh", r_fresh), ("scan", r_scan), ("explore", r_explore), ("explore_stop", r_explore_stop),
                     ("save_prescan", r_save_prescan), ("import", r_import), ("load_prescan", r_load_prescan),
                     ("drop", r_drop)):
        bench.route("POST", f"/api/maps/{name}", fn)
    threading.Thread(target=_grid_loop, daemon=True, name="maps-grid").start()


def shutdown():
    _stop.set()
    if _explorer is not None:
        _explorer.cancel()
