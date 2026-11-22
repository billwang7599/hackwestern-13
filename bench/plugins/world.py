"""bench plugin `world`: the simulated room (bench/world/worldsim.py) in the browser.

Listens for world state on UDP (env BENCH_WORLD_STATE, default 5593) and publishes topic
`world` at <= 10 Hz. Every event carries the FULL current room under "room" (walls as
[x1,y1,x2,y2] segments + objects, including everything placed live), plus the sensor state
and goto's route converted to the world frame:

  {"type": "state", "t", "rev", "alive", "car_kind", "truth", "phone", "guard", "dets",
   "faults", "hfov_deg", "room": {"name", "about", "start", "wall_h", "walls", "objects",
   "memory", "labels", "params", "rev"}, "route": [[x, y], ...] | null, "route_i",
   "goto": {"state", "target", "result", "map", "path"} | null, "supervisor": bool}

Routes (all world edits go through world control, env BENCH_WORLD_CTL, default 5592):
  GET  /api/world/room          current room
  GET  /api/world/rooms         room files
  POST /api/world/ctl  {...}    any world-control message (bench/world/README.md), reply as JSON
  POST /api/world/load {name}   load a room; with run_sim, goto restarts with the room's memory
  POST /api/world/reset         reload the current room from where it was loaded/saved (car to start too)
  POST /api/world/car_to_start  S + CANCEL, then only the car jumps back to the room start: room,
                                drawn walls, goto's map and memory are kept (goto is NOT restarted)
  POST /api/world/reset_map     room back as loaded/saved + goto restarted (empty map, the room's
                                memory); the car stays where it is
  POST /api/world/save {name, overwrite?}
  POST /api/world/goto {label}  GOTO <label> through the robot plugin (if loaded)
"""
from __future__ import annotations

import json
import os
import socket
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "world"))
import common as C  # noqa: E402

CTL = int(os.environ.get("BENCH_WORLD_CTL", "5592"))
STATE = int(os.environ.get("BENCH_WORLD_STATE", "5593"))
SUP = int(os.environ.get("BENCH_SUPERVISOR", "0") or 0)
CAR_KIND = os.environ.get("BENCH_CAR_KIND", "")

_bench = None
_stop = threading.Event()
_lock = threading.Lock()
_room: dict | None = None
_state: dict | None = None
_state_t = 0.0
_route: dict = {"route": None, "i": 0, "t": 0.0}
_ctl_lock = threading.Lock()
_ctl_sock = None


def ctl(msg: dict, timeout: float = 3.0) -> dict:
    """one world-control request over TCP (keeps one connection, reconnects)"""
    global _ctl_sock
    with _ctl_lock:
        for attempt in (0, 1):
            try:
                if _ctl_sock is None:
                    s = socket.create_connection(("127.0.0.1", CTL), timeout=timeout)
                    s.settimeout(timeout)
                    _ctl_sock = (s, s.makefile("rwb"))
                s, f = _ctl_sock
                f.write((json.dumps(msg) + "\n").encode())
                f.flush()
                line = f.readline()
                if not line:
                    raise OSError("world sim closed the connection")
                return json.loads(line)
            except (OSError, ValueError) as e:
                try:
                    _ctl_sock and _ctl_sock[0].close()
                except OSError:
                    pass
                _ctl_sock = None
                if attempt:
                    return {"ok": False, "error": f"world sim not reachable on {CTL}: {e}"}
    return {"ok": False, "error": "unreachable"}


def supervisor(msg: dict, timeout: float = 15.0) -> dict | None:
    if not SUP:
        return None
    try:
        with socket.create_connection(("127.0.0.1", SUP), timeout=timeout) as s:
            s.settimeout(timeout)
            f = s.makefile("rw", encoding="utf-8", newline="\n")
            f.write(json.dumps(msg) + "\n")
            f.flush()
            return json.loads(f.readline() or "{}")
    except (OSError, ValueError) as e:
        return {"ok": False, "error": str(e)}


def _robot():
    return _bench.plugins.get("robot") if _bench else None


def _goto_status():
    r = _robot()
    if r is None:
        return None
    try:
        return r.goto_status()
    except Exception:
        return None


def _rx_loop():
    global _room, _state, _state_t
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", STATE))
    except OSError as e:
        _bench.log("world", f"can't listen for world state on UDP {STATE}: {e}")
        return
    s.settimeout(0.3)
    last_pub = 0.0
    while not _stop.is_set():
        try:
            msg = json.loads(s.recv(65535))
        except socket.timeout:
            msg = None
        except (OSError, ValueError):
            continue
        now = time.monotonic()
        if msg and msg.get("type") == "room":
            with _lock:
                _room = msg
        elif msg and msg.get("type") == "state":
            with _lock:
                _state, _state_t = msg, now
                stale_room = _room is None or _room.get("rev") != msg.get("rev")
            if stale_room:
                r = ctl({"op": "get_room"}, timeout=1.0)
                if r.get("ok"):
                    with _lock:
                        _room = r["room"]
        if now - last_pub >= 0.1:
            last_pub = now
            _publish()


def _publish():
    with _lock:
        st = dict(_state or {"type": "state"})
        room = _room
        alive = _state is not None and time.monotonic() - _state_t < 1.0
    g = _goto_status()
    st.update({"t": time.time(), "alive": alive, "car_kind": CAR_KIND or ("stub" if (st.get("truth") or {}).get("stub") else ""),
               "room": room, "supervisor": bool(SUP), "robot": _robot() is not None,
               "goto": {k: g.get(k) for k in ("state", "target", "result", "map", "path", "replans", "mode")} if g else None,
               "route": _route["route"], "route_i": _route["i"]})
    _bench.publish("world", st)


def _route_loop():
    """goto's planned route (NAV ROUTE, ark frame) -> world frame, ~2 Hz while goto is busy"""
    while not _stop.wait(0.5):
        r = _robot()
        g = _goto_status()
        if r is None or not g:
            continue
        busy = g.get("state") not in (None, "idle")
        if not busy and _route["route"] is None:
            continue
        try:
            rep = r.send("goto", "NAV ROUTE", timeout=1.0, src="poll")
        except TypeError:
            rep = r.send("goto", "NAV ROUTE")
        except Exception:
            continue
        with _lock:
            start = (_room or {}).get("start")
        if not rep.startswith("OK route=") or start is None:
            continue
        body = rep[len("OK route="):].split()
        pts = []
        if body and body[0] != "-" and not body[0].startswith("i="):
            for p in body[0].split(";"):
                try:
                    xa, za = (float(v) for v in p.split(","))
                except ValueError:
                    continue
                x, y, _ = C.ark_to_world(start, xa, za)
                pts.append([round(x, 3), round(y, 3)])
        i = 0
        for tok in body:
            if tok.startswith("i="):
                try:
                    i = int(tok[2:])
                except ValueError:
                    pass
        _route.update({"route": pts or None, "i": i, "t": time.time()})


def _body(b: bytes) -> dict:
    try:
        return json.loads(b or b"{}")
    except ValueError:
        return {}


def _after_room_change(rep: dict, why: str) -> dict:
    """a room (re)load: with run_sim, S + a fresh goto with the room's memory + tars_motor pose reset"""
    if not rep.get("ok"):
        return rep
    _route.update({"route": None, "i": 0})
    sup = supervisor({"op": "restart_goto", "memory": rep.get("goto_memory")})
    if sup is None:
        rep["note"] = "not started by run_sim: goto keeps its old map and memory, tars_motor its pose"
        r = _robot()
        if r is not None:
            r.stop(f"world {why}")
    else:
        rep["goto_restart"] = sup
    _bench.log("world", f"{why}: room {(_room or {}).get('name', '?')} "
                        f"{'(goto restarted, memory seeded)' if rep.get('goto_memory') and sup else ''}")
    return rep


def _car_to_start():
    """only the car goes back to the start. S + CANCEL first so nothing is driving when it jumps.
    goto keeps running: its nav pose is the phone's ARKit pose, whose frame is fixed to the room,
    so the next LDR1 packet puts goto at the start on its own map (dead reckoning re-anchors to it)."""
    r = _robot()
    stopped = r.stop("car to start") if r is not None else None
    if stopped is not None:
        time.sleep(0.2)          # let S land before the car jumps
    _route.update({"route": None, "i": 0})
    rep = ctl({"op": "car_to_start"})
    if rep.get("ok"):
        rep["stopped"] = stopped
        if r is None:
            rep["note"] = "robot plugin not loaded: no S/CANCEL sent"
        _bench.log("world", "car to start (room, map and memory kept)")
    return (200 if rep.get("ok") else 400), rep


def setup(bench) -> None:
    global _bench
    _bench = bench

    def r_room(q, b):
        with _lock:
            room = _room
        if room is None:
            rep = ctl({"op": "get_room"})
            return (200, rep["room"]) if rep.get("ok") else (503, rep)
        return 200, room

    def r_rooms(q, b):
        return 200, {"rooms": C.list_rooms()}

    def r_ctl(q, b):
        msg = _body(b)
        if msg.get("op") == "car_to_start":
            return _car_to_start()
        rep = ctl(msg)
        if msg.get("op") in ("load", "reset", "reset_map"):
            rep = _after_room_change(rep, msg["op"])
        return (200 if rep.get("ok") else 400), rep

    def r_load(q, b):
        name = _body(b).get("name") or q.get("name", "")
        return r_ctl(q, json.dumps({"op": "load", "name": name}).encode())

    def r_reset(q, b):
        return r_ctl(q, b'{"op":"reset"}')

    def r_car_to_start(q, b):
        return r_ctl(q, b'{"op":"car_to_start"}')

    def r_reset_map(q, b):
        return r_ctl(q, b'{"op":"reset_map"}')

    def r_save(q, b):
        m = _body(b)
        return r_ctl(q, json.dumps({"op": "save", "name": m.get("name", ""), "overwrite": bool(m.get("overwrite")),
                                    **({"about": m["about"]} if "about" in m else {})}).encode())

    def r_goto(q, b):
        label = " ".join(str(_body(b).get("label", "")).split())
        r = _robot()
        if r is None:
            return 503, {"ok": False, "error": "robot plugin not loaded: can't send GOTO"}
        if not label:
            return 400, {"ok": False, "error": "no label"}
        rep = r.send("goto", f"GOTO {label}", src="world2d")
        bench.log("world", f"GOTO {label} -> {rep}")
        return 200, {"ok": rep.startswith("OK"), "reply": rep}

    bench.route("GET", "/api/world/room", r_room)
    bench.route("GET", "/api/world/rooms", r_rooms)
    bench.route("POST", "/api/world/ctl", r_ctl)
    bench.route("POST", "/api/world/load", r_load)
    bench.route("POST", "/api/world/reset", r_reset)
    bench.route("POST", "/api/world/car_to_start", r_car_to_start)
    bench.route("POST", "/api/world/reset_map", r_reset_map)
    bench.route("POST", "/api/world/save", r_save)
    bench.route("POST", "/api/world/goto", r_goto)
    threading.Thread(target=_rx_loop, daemon=True).start()
    threading.Thread(target=_route_loop, daemon=True).start()


def shutdown() -> None:
    _stop.set()
