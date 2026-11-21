"""iPhone LiDAR -> laptop 3D map (bench part "phone"). See bench/phone/README.md.

Routes (all under /api/phone/):
  GET  status        pipeline state, rates, counts, room view URL
  GET  map.json      the room JSON Andrew's room_view.html reads (same keys as his /map.json)
  GET  devices       iPhones usbmuxd sees on this Mac
  GET  recordings    saved recordings in bench/phone/recordings/
  POST source        {"kind": off|sim|replay|live|iproxy, "path", "speed", "loop", "port",
                      "pose": auto|car|script, "room": {...}}
  POST record        {"on": true|false, "name"}
  POST command       Andrew's map commands: {"op": map_clear|map_save|map_load|select_target, "id"}
Topic `map3d`: the status dict about twice a second.
Listens to topics `car` (truth overlay + sim pose) and `world` (live room for the sim feed),
and polls goto `NAV ROUTE` through the robot plugin (if loaded) for the route overlay.
Env: TARS_BENCH_PHONE_ROOM_PORT (5594), TARS_BENCH_PHONE_SCENE_PORT (5595),
     TARS_BENCH_PHONE_SOURCE (default: sim when --target sim, else off).
"""
from __future__ import annotations

import json
import os
import sys
import threading

_BENCH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BENCH not in sys.path:
    sys.path.insert(0, _BENCH)
from phone import pipeline as P  # noqa: E402  (bench/phone/, a package)
from phone import usbmux  # noqa: E402

pipe: P.PhonePipeline | None = None
_stop = threading.Event()
_bench = None


def _json(body: bytes) -> dict:
    return json.loads(body or b"{}") if body else {}


def parse_route(reply: str):
    """'OK route=x,z;x,z;... i=<n>' -> [[x, z], ...] or None."""
    if not isinstance(reply, str) or "route=" not in reply:
        return None
    text = reply.split("route=", 1)[1].split()[0]
    if text in ("-", ""):
        return None
    try:
        return [[float(a) for a in pt.split(",")[:2]] for pt in text.split(";") if pt]
    except ValueError:
        return None


def _tap(topic, data):
    if pipe is None or not isinstance(data, dict):
        return
    if topic == "car":
        t = data.get("truth", data)
        if isinstance(t, dict) and all(k in t for k in ("x", "y", "th")):
            pipe.set_truth(t["x"], t["y"], t["th"])
    elif topic == "world":
        if pipe.set_world(data) and _bench is not None:
            _bench.log("phone", "sim feed: room geometry changed, re-ray-casting")


def _route_loop(bench):
    while not _stop.wait(1.0):
        robot = bench.plugins.get("robot")
        send = getattr(robot, "send", None)
        if pipe is None or pipe.kind == "off" or send is None:
            continue
        try:
            pipe.set_route(parse_route(send("goto", "NAV ROUTE")))
        except Exception:
            pipe.set_route(None)


def setup(bench):
    global pipe, _bench
    _bench = bench
    room_port = int(os.environ.get("TARS_BENCH_PHONE_ROOM_PORT", 5594))
    scene_port = int(os.environ.get("TARS_BENCH_PHONE_SCENE_PORT", 5595))
    pipe = P.PhonePipeline(room_port=room_port, scene_port=scene_port,
                           on_update=lambda st: bench.publish("map3d", st))
    if pipe.room_error:
        bench.log("phone", pipe.room_error)
    bench.tap(_tap)

    def status(q, b):
        return 200, pipe.status()

    def room(q, b):
        # Exactly the keys room_view.py serves, from the same latest status.
        latest = pipe.room()  # fresh, not the 2 Hz copy
        src = latest.get("surroundings", {})
        keys = ("session", "frames", "tracking_live", "tracking", "pose", "origin", "resolution",
                "size", "grid", "path", "objects", "points3d", "estimated3d", "distant3d",
                "distant_confidence", "surface_splats", "floor_y", "floor_assumed", "floor_ready",
                "camera_height_m", "lidar_height_m", "voxels", "scan_degrees", "grid_source",
                "grid_version", "grid_epoch", "native_map_live", "plan_status")
        data = {k: src[k] for k in keys if k in src}
        data["publisher_live"] = True
        for k in ("accel_m_s2", "gyro_rad_s", "motion_live", "motion_hz"):
            if k in latest:
                data[k] = latest[k]
        return 200, data

    def devices(q, b):
        try:
            return 200, {"devices": usbmux.list_devices()}
        except usbmux.UsbmuxError as e:
            return 200, {"devices": [], "error": str(e)}

    def source(q, b):
        req = _json(b)
        kind = req.pop("kind", "off")
        try:
            st = pipe.set_source(kind, **req)
        except ValueError as e:
            return 400, {"error": str(e)}
        bench.log("phone", f"source -> {st['source']}" + (f" ({st['error']})" if st["error"] else ""))
        return 200, st

    def record(q, b):
        req = _json(b)
        out = pipe.record(bool(req.get("on")), req.get("name"))
        bench.log("phone", "recording " + json.dumps(out))
        return 200, out

    def command(q, b):
        return 200, pipe.command(_json(b))

    bench.route("GET", "/api/phone/status", status)
    bench.route("GET", "/api/phone/map.json", room)
    bench.route("GET", "/api/phone/devices", devices)
    bench.route("GET", "/api/phone/recordings", lambda q, b: (200, {"recordings": pipe.recordings()}))
    bench.route("POST", "/api/phone/source", source)
    bench.route("POST", "/api/phone/record", record)
    bench.route("POST", "/api/phone/command", command)
    threading.Thread(target=_route_loop, args=(bench,), daemon=True, name="phone-route").start()

    default = os.environ.get("TARS_BENCH_PHONE_SOURCE") or ("sim" if bench.target.get("name") == "sim" else "off")
    if default != "off":
        pipe.set_source(default)


def shutdown():
    _stop.set()
    if pipe is not None:
        pipe.close()
