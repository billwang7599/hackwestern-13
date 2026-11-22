"""bench plugin `live`: the sim's 2D room view, drawn from the REAL robot's data.

Active on the real targets only (`--target robot`: the bench runs on the Pi itself, or `--target pi`:
through mission_relay). On the sim the world plugin owns topic `world`; here this plugin publishes the
same topic in the same shape, so the `world2d` panel draws the real robot the way it draws the sim car:

  robot pose   goto NAV (the phone's ARKit pose on goto's map; "dr" = dead reckoning, "stale")
  route        goto NAV ROUTE
  objects      goto MEMORY: every remembered object at its map position (fresh / stale / gone)
  goto, guard  goto STATUS (robot plugin) and tars_motor STATUS (guard_permit / guard_reason)
  walls        none: the map itself is the walls. The maps plugin publishes goto NAV GRID as topic
               `navgrid` and its layer paints the free / obstacle cells under the view.

Frame: goto's map (ARKit) frame with the room start at the origin, the same conversion as the sim
(bench/world/common.py ark_to_world: world x = map z, world y = map x). Nothing is edited or sent
to the robot from here: it only reads (GOTO from the view goes through the robot plugin, as in sim).

Topic `world` (~4 Hz): {"type": "state", "live": true, "alive", "t", "truth": {x, y, th, src} | None,
"room": {"name", "start", "walls": [], "objects": [...memory...], "memory": [], "labels", "params"},
"route", "route_i", "goto", "guard": {verdict, permit, reason}, "dets": [], "car_kind": "real", "robot"}
Route: GET /api/live/state (the latest event).
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "world"))
import common as C  # noqa: E402

REAL_TARGETS = ("robot", "pi")
START = {"x": 0.0, "y": 0.0, "th": 0.0}       # the map origin is the room origin
RATE_S = 0.25                                  # pose + publish
ROUTE_S = 0.5
MEMORY_S = 1.0

_bench = None
_stop = threading.Event()
_last: dict = {}
_hfov = [60.0]
ACTIVE = False          # bench.py: panels.json "live" (the plugin module loads on every target)


def _robot():
    return _bench.plugins.get("robot") if _bench else None


def _send(r, line: str, timeout: float = 1.0) -> str:
    try:
        rep = r.send("goto", line, timeout=timeout, src="poll")
    except Exception:  # noqa: BLE001  (link down: the view says so)
        return ""
    return rep if isinstance(rep, str) else ""


def parse_nav(rep: str) -> dict | None:
    """goto 'OK pose=src,x,z,yaw_deg free=.. occ=.. ...' -> {"src", "x", "y", "th"} in the world frame,
    None without a pose."""
    if not rep.startswith("OK pose="):
        return None
    kv = dict(t.split("=", 1) for t in rep[3:].split() if "=" in t)
    try:
        src, xa, za, yaw = kv["pose"].split(",")
        xa, za, yaw = float(xa), float(za), math.radians(float(yaw))
    except (KeyError, ValueError):
        return None
    if src == "none":
        return None
    x, y, th = C.ark_to_world(START, xa, za, yaw)
    out = {"src": src, "x": round(x, 3), "y": round(y, 3), "th": round(th, 4), "v": 0.0}
    for k in ("free", "occ", "feed_age", "version"):
        if k in kv:
            try:
                out[k] = float(kv[k])
            except ValueError:
                pass
    return out


def parse_route(rep: str) -> tuple[list | None, int]:
    """goto 'OK route=x,z;x,z;... i=<n>' -> world-frame points, next index"""
    if not rep.startswith("OK route="):
        return None, 0
    body = rep[len("OK route="):].split()
    pts = []
    if body and body[0] != "-" and not body[0].startswith("i="):
        for p in body[0].split(";"):
            try:
                xa, za = (float(v) for v in p.split(","))
            except ValueError:
                continue
            x, y, _ = C.ark_to_world(START, xa, za)
            pts.append([round(x, 3), round(y, 3)])
    i = 0
    for tok in body:
        if tok.startswith("i="):
            try:
                i = int(tok[2:])
            except ValueError:
                pass
    return pts or None, i


def memory_objects(rep: str, car) -> list[dict]:
    """goto MEMORY -> view objects (a ring per remembered thing, walk-through so it isn't drawn solid)"""
    parsed = C.parse_goto_memory(rep)
    if not parsed:
        return []
    out = []
    for e in C.memory_to_world(START, parsed, car):
        out.append({"id": e["key"], "label": e["label"], "x": e["x"], "y": e["y"], "r": 0.15, "h": 0.4,
                    "solid": False, "state": e["state"], "frame": e["frame"], "remembered": True})
    return out


def guard_view(st: dict | None) -> dict:
    if not st:
        return {"verdict": "UNKNOWN", "permit": False, "reason": ""}
    reason = str(st.get("guard_reason", ""))
    if not st.get("guard_enabled"):
        return {"verdict": "OFF", "permit": True, "reason": reason}
    permit = bool(st.get("guard_permit"))
    verdict = "CLEAR" if permit else {"guard_blocked": "BLOCKED", "guard_stale": "STALE",
                                      "guard_missing": "STALE", "guard_invalid": "INVALID"}.get(reason, "UNKNOWN")
    return {"verdict": verdict, "permit": permit, "reason": reason}


def _loop() -> None:
    route, route_i, objects = None, 0, []
    t_route = t_mem = 0.0
    while not _stop.wait(RATE_S):
        r = _robot()
        if r is None:
            continue
        now = time.monotonic()
        nav_rep = _send(r, "NAV")
        truth = parse_nav(nav_rep)
        g = None
        try:
            g = r.goto_status()
        except Exception:  # noqa: BLE001
            pass
        busy = bool(g) and g.get("state") not in (None, "idle")
        if now - t_route >= ROUTE_S and (busy or route):
            t_route = now
            route, route_i = parse_route(_send(r, "NAV ROUTE"))
        if now - t_mem >= MEMORY_S:
            t_mem = now
            car = (truth["x"], truth["y"], truth["th"]) if truth else None
            rep = _send(r, "MEMORY", timeout=2.0)
            if rep.startswith("OK"):
                objects = memory_objects(rep, car)
        st = None
        try:
            st = r.status()
        except Exception:  # noqa: BLE001
            pass
        ev = {
            "type": "state", "live": True, "t": time.time(), "alive": bool(nav_rep),
            "car_kind": "real", "robot": True, "truth": truth, "phone": None, "dets": [],
            "guard": guard_view(st), "faults": {}, "hfov_deg": _hfov[0],
            "room": {"name": "live: real robot", "about": "goto's map from the iPhone LiDAR; nothing here is simulated",
                     "start": START, "wall_h": 0, "walls": [], "objects": objects, "memory": [],
                     "labels": [], "params": {}, "rev": 0},
            "route": route, "route_i": route_i,
            "goto": {k: g.get(k) for k in ("state", "target", "result", "map", "path", "replans", "mode")} if g else None,
        }
        _last.clear()
        _last.update(ev)
        _bench.publish("world", ev)


def setup(bench) -> None:
    global _bench, ACTIVE
    _bench = bench
    if bench.target["name"] not in REAL_TARGETS:
        return                      # sim: the world plugin owns topic `world`
    ACTIVE = True
    try:
        _hfov[0] = float(C.goto_conf().get("hfov_deg", 60) or 60)
    except (OSError, ValueError, TypeError):
        pass
    bench.route("GET", "/api/live/state", lambda q, b: (200, dict(_last) or {"live": True, "alive": False}))
    threading.Thread(target=_loop, daemon=True, name="live-view").start()
    bench.log("live", "live view: the 2D room view shows the real robot (goto NAV / ROUTE / MEMORY)")


def shutdown() -> None:
    _stop.set()
