"""Bench plugin: the virtual SM11 (bench/car/). Bridges its truth into topic `car`.

The car sends truth to UDP 5590 (the world sim's port). This plugin doesn't take that
port: it binds its own and subscribes with {"sub": <port>} on the car's control port
(BENCH_CAR_CTL, default 5591), renewed every 2 s, so the world sim and the bench both
get truth. If no car is running, topic `car` says alive=false and the routes answer 503.

Topic `car` (about 20 Hz): {"alive", "age_ms", "truth": <5590 datagram>, "params": {...}}
Routes:
  GET  /api/car/state                 latest truth + params
  POST /api/car/set    {key: value}   profile values, live (bench/car/physics.h PARAM_DEFS)
  POST /api/car/fault  {"fault": "push"|"lift_left"|"lift_right"|"tip"|"bump"|"unplug"|
                        "brownout"|"clear", "ms"?, "v"?, "vx"?, "vy"?, "g"?}
  POST /api/car/pose   {"x","y","th"}  teleport
  POST /api/car/button {"button": "tap"|"hold"|"down"|"up"}   the kit's KEY (TEACH) button
  POST /api/car/reset  {}              the Uno's reset button
"""
from __future__ import annotations

import json
import os
import socket
import threading
import time

CTL_PORT = int(os.environ.get("BENCH_CAR_CTL", "5591"))
# run_sim.py: the world sim (sole reader of 5590) copies every truth datagram to this port.
# Bind it instead of subscribing: it works for the stub car too (no {"sub"} there) and
# avoids getting each datagram twice.
TRUTH_COPY = int(os.environ.get("BENCH_CAR_TRUTH", "0") or 0)
PUBLISH_HZ = 20

_stop = threading.Event()
_state = {"truth": None, "params": None, "at": 0.0}
_lock = threading.Lock()
_sock: socket.socket | None = None


def _ctl(msg: dict, timeout: float = 0.5) -> dict | None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(msg).encode(), ("127.0.0.1", CTL_PORT))
        data, _ = s.recvfrom(65536)
        return json.loads(data)
    except (OSError, ValueError):
        return None
    finally:
        s.close()


def _refresh_params() -> None:
    r = _ctl({"get": "profile"})
    if r and r.get("profile"):
        with _lock:
            _state["params"] = r["profile"]


def _rx(bench) -> None:
    last_pub = 0.0
    last_sub = 0.0
    port = _sock.getsockname()[1]
    _sock.settimeout(0.2)
    while not _stop.is_set():
        now = time.time()
        if now - last_sub > 2.0:
            last_sub = now
            if TRUTH_COPY:
                if _state["params"] is None:
                    _refresh_params()
            elif _ctl({"sub": port}) is not None and _state["params"] is None:
                _refresh_params()
        try:
            data = _sock.recv(65536)
            d = json.loads(data)
            with _lock:
                _state["truth"] = d
                _state["at"] = time.time()
        except socket.timeout:
            pass
        except (OSError, ValueError):
            if _stop.is_set():
                return
        now = time.time()
        if now - last_pub >= 1.0 / PUBLISH_HZ:
            last_pub = now
            bench.publish("car", _snapshot())


def control(msg: dict) -> dict | None:
    """One car-control datagram (5591 format) and the car's reply, None if no car answered.
    The scenarios plugin uses this for faults, so they follow BENCH_CAR_CTL."""
    return _ctl(msg)


CONTROL_PORT = CTL_PORT


def _snapshot() -> dict:
    with _lock:
        age = time.time() - _state["at"] if _state["at"] else None
        return {"alive": age is not None and age < 1.0,
                "age_ms": int(age * 1000) if age is not None else None,
                "truth": _state["truth"], "params": _state["params"], "ctl_port": CTL_PORT}


def _json_body(body: bytes) -> dict:
    try:
        v = json.loads(body or b"{}")
        return v if isinstance(v, dict) else {}
    except ValueError:
        return {}


def _send(msg: dict):
    r = _ctl(msg)
    if r is None:
        return 503, {"error": f"no virtual car on udp {CTL_PORT} (start it: python3 bench/car/launch.py)"}
    return (200 if r.get("ok") else 400), r


def setup(bench) -> None:
    global _sock
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _sock.bind(("127.0.0.1", TRUTH_COPY))

    def r_state(q, body):
        return 200, _snapshot()

    def r_set(q, body):
        vals = _json_body(body)
        out = _send({"set": vals})
        _refresh_params()
        bench.log("car", f"set {vals} -> {out[1].get('error', 'ok')}")
        return out

    def r_fault(q, body):
        m = _json_body(body)
        if "fault" not in m:
            return 400, {"error": "want {\"fault\": ...}"}
        out = _send(m)
        bench.log("car", f"fault {m} -> {out[1].get('error', 'ok')}")
        return out

    def r_pose(q, body):
        return _send({"pose": _json_body(body)})

    def r_button(q, body):
        return _send({"button": _json_body(body).get("button", "tap")})

    def r_reset(q, body):
        return _send({"reset": True})

    bench.route("GET", "/api/car/state", r_state)
    bench.route("POST", "/api/car/set", r_set)
    bench.route("POST", "/api/car/fault", r_fault)
    bench.route("POST", "/api/car/pose", r_pose)
    bench.route("POST", "/api/car/button", r_button)
    bench.route("POST", "/api/car/reset", r_reset)
    threading.Thread(target=_rx, args=(bench,), daemon=True, name="car-truth").start()


def shutdown() -> None:
    _stop.set()
    if _sock:
        _sock.close()
