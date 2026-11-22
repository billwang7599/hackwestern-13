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
  GET  /api/world/memory        goto's live object memory in room metres (goto MEMORY, polled on request):
                                {"ok", "entries": [{"key": "trash can#1", "label", "id", "state", "x", "y",
                                "age", "conf", "frame"}], "room_memory": the room's saved "memory", "raw"}
  POST /api/world/memory {op, ...}  edit goto's memory to set up a test case (SIM, run_sim only for edits):
     forget {key?}              goto FORGET <key> (one, e.g. "trash can#1") or everything; no restart
     add {label, x, y, state?}  goto remembers a <label> at x, y (room metres); state fresh|stale|gone
     add {object}               ... the room object <object id> where it really is
     update {key, x?, y?, state?}  move an entry / change its state
     set {entries: [{label, x, y, state?}]}  replace the whole memory
     to_room {write?}           the live memory becomes the room's "memory" (reloads with it; Save room keeps
                                it); write: true also saves the room file under its current name
   add/update/set: S + CANCEL first (stop always wins), then goto is restarted with the edited memory
   (its nav map is lost; the car stays where it is). The reply carries the new live "entries".
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


# ------------------------------------------------------------------ goto memory (Memory panel)
def _memory_live() -> dict:
    """goto MEMORY -> room-frame entries (common.memory_to_world)"""
    r = _robot()
    if r is None:
        return {"ok": False, "error": "robot plugin not loaded: can't ask goto"}
    try:
        raw = r.send("goto", "MEMORY", timeout=2.0, src="memory")
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "error": f"goto: {e}"}
    parsed = C.parse_goto_memory(raw)
    room = _room or (ctl({"op": "get_room"}).get("room"))
    if parsed is None or not room:
        return {"ok": False, "error": f"goto MEMORY: {raw!r}" if parsed is None else "no room yet", "raw": raw}
    st = ctl({"op": "get_state"}).get("state") or {}
    t = st.get("truth")
    car = (float(t["x"]), float(t["y"]), float(t["th"])) if t else None
    return {"ok": True, "entries": C.memory_to_world(room["start"], parsed, car), "pose": parsed["pose"],
            "car": car and [round(v, 3) for v in car], "room_memory": room.get("memory", []), "raw": raw}


def _memory_restart(entries: list[dict], why: str) -> tuple[int, dict]:
    """S + CANCEL, then a fresh goto whose memory is `entries` (room metres); the car stays put"""
    if not SUP:
        return 503, {"ok": False, "error": "not started by run_sim: memory edits need goto restarted (forget works)"}
    r = _robot()
    stopped = r.stop(f"memory {why}") if r is not None else None
    room = _room or ctl({"op": "get_room"}).get("room")
    st = ctl({"op": "get_state"}).get("state") or {}
    t = st.get("truth")
    car = (float(t["x"]), float(t["y"]), float(t["th"])) if t else None
    if not room or car is None:
        return 503, {"ok": False, "error": "no room or car pose from the world sim"}
    mem = [{"label": e["label"], "x": float(e["x"]), "y": float(e["y"]),
            **({"state": e["state"]} if e.get("state") in C.MEM_STATES else {})} for e in entries]
    text = C.goto_memory_text({"start": room["start"], "objects": room["objects"], "memory": mem}, car=car)
    _route.update({"route": None, "i": 0})
    sup = supervisor({"op": "restart_goto", "memory": text})
    if not sup or not sup.get("ok"):
        return 500, {"ok": False, "error": f"goto restart failed: {sup}"}
    live = {}
    end = time.monotonic() + 6
    while time.monotonic() < end:          # goto re-bases its memory on the first STATUS: wait for it
        time.sleep(0.3)
        live = _memory_live()
        if live.get("ok") and len(live["entries"]) == len(mem) and \
                all(e["frame"] == "map" for e in live["entries"]):
            break
    _bench.log("memory", f"{why}: goto restarted with {len(mem)} remembered thing(s)")
    return 200, {**live, "ok": bool(live.get("ok")), "stopped": stopped, "goto_restart": sup}


def _memory_op(m: dict) -> tuple[int, dict]:
    op = m.get("op")
    if op == "forget":
        r = _robot()
        if r is None:
            return 503, {"ok": False, "error": "robot plugin not loaded"}
        key = " ".join(str(m.get("key") or "").split())
        rep = r.send("goto", f"FORGET {key}".strip(), timeout=2.0, src="memory")
        _bench.log("memory", f"FORGET {key or '(all)'} -> {rep}")
        return 200, {**_memory_live(), "ok": rep.startswith("OK"), "reply": rep}
    if op not in ("add", "update", "set", "to_room"):
        return 400, {"ok": False, "error": "ops: forget, add, update, set, to_room"}
    live = _memory_live()
    if not live.get("ok"):
        return 503, live
    entries = [dict(e) for e in live["entries"]]
    state = m.get("state")
    if state is not None and state not in C.MEM_STATES:
        return 400, {"ok": False, "error": f"state: one of {C.MEM_STATES}"}
    try:
        if op == "add":
            if m.get("object"):
                o = next((o for o in (_room or {}).get("objects", []) if o["id"] == m["object"]), None)
                if o is None or not o.get("label"):
                    return 400, {"ok": False, "error": f"no labelled object {m['object']!r}"}
                e = {"label": o["label"], "x": o["x"], "y": o["y"]}
            else:
                label = " ".join(str(m.get("label") or "").split())
                if not label:
                    return 400, {"ok": False, "error": "add needs label (or object)"}
                e = {"label": label, "x": float(m["x"]), "y": float(m["y"])}
            entries.append({**e, "state": state or "fresh"})
        elif op == "update":
            e = next((e for e in entries if e["key"] == m.get("key")), None)
            if e is None:
                return 404, {"ok": False, "error": f"no memory entry {m.get('key')!r}"}
            if "x" in m:
                e["x"] = float(m["x"])
            if "y" in m:
                e["y"] = float(m["y"])
            if state:
                e["state"] = state
        elif op == "set":
            entries = [{"label": " ".join(str(e["label"]).split()), "x": float(e["x"]), "y": float(e["y"]),
                        "state": e.get("state") or "fresh"} for e in m.get("entries") or []]
            if any(not e["label"] or e["state"] not in C.MEM_STATES for e in entries):
                return 400, {"ok": False, "error": "every entry needs label, x, y (state fresh|stale|gone)"}
    except (KeyError, TypeError, ValueError) as e:
        return 400, {"ok": False, "error": f"bad request: {e}"}
    if op == "to_room":
        mem = [{"label": e["label"], "x": e["x"], "y": e["y"],
                **({"state": e["state"]} if e["state"] != "fresh" else {})} for e in entries]
        rep = ctl({"op": "set_memory", "memory": mem})
        if rep.get("ok") and m.get("write"):
            name = (_room or {}).get("name") or ctl({"op": "get_room"}).get("room", {}).get("name", "")
            sv = ctl({"op": "save", "name": name, "overwrite": True})
            rep = {**rep, "saved": sv}
            if not sv.get("ok"):
                rep["ok"] = False
        rep.pop("goto_memory", None)
        _bench.log("memory", f"live memory -> room memory ({len(mem)}){' and saved' if m.get('write') else ''}")
        return (200 if rep.get("ok") else 400), rep
    return _memory_restart(entries, op)


def setup(bench) -> None:
    global _bench
    _bench = bench
    if bench.target["name"] != "sim":
        # real robot: there is no world sim; plugins/live.py publishes topic `world` from the robot.
        # Only GOTO from the view is kept (it goes through the robot plugin like everything else).
        def r_goto_real(q, b):
            label = " ".join(str(_body(b).get("label", "")).split())
            r = _robot()
            if r is None or not label:
                return 400, {"ok": False, "error": "no robot link or no label"}
            rep = r.send("goto", f"GOTO {label}", src="world2d")
            bench.log("world", f"GOTO {label} -> {rep}")
            return 200, {"ok": rep.startswith("OK"), "reply": rep}
        bench.route("POST", "/api/world/goto", r_goto_real)
        bench.route("GET", "/api/world/rooms", lambda q, b: (200, {"rooms": []}))
        return

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

    def r_memory_get(q, b):
        rep = _memory_live()
        return (200 if rep.get("ok") else 503), rep

    def r_memory_post(q, b):
        return _memory_op(_body(b))

    bench.route("GET", "/api/world/memory", r_memory_get)
    bench.route("POST", "/api/world/memory", r_memory_post)
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
