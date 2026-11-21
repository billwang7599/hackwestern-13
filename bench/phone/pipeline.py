"""Phone -> Andrew's mapping -> Andrew's room view, on the laptop.

Everything downstream of the source is Andrew's code, imported unchanged:
  scene source on 127.0.0.1:<scene_port>  (live phone via usbmux | iproxy | replay | sim)
    -> dashboard/mapping.SceneFeed (reads 4-byte length + JSON, ACKs 0x01)
    -> dashboard/mapping.Map (native=None: his Python reference grid; see README)
    -> dashboard/room_view.serve_room on 127.0.0.1:<room_port> (his room_view.html,
       room_controls.js and /map.json, read-only)
Bench additions sit at the edges: a Map subclass that tees each validated scene into a
recording, and overlays (sim car truth, goto's NAV ROUTE) added to the published copy.
"""
from __future__ import annotations

import math
import os
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DASH = os.path.join(REPO, "iter2-autonomy", "dashboard")
USB = os.path.join(REPO, "iter2-autonomy", "iphone-usb")
for p in (DASH, USB):
    if p not in sys.path:
        sys.path.insert(0, p)

import mapping  # noqa: E402  (Andrew's)
import room_view  # noqa: E402  (Andrew's)
from receive_motion import validate as validate_motion  # noqa: E402  (Andrew's)

from . import scene_io, sim, usbmux  # noqa: E402

PHONE_SCENE_PORT, PHONE_MOTION_PORT = 5561, 5557
RECORDINGS = os.path.join(HERE, "recordings")
SOURCES = ("off", "sim", "replay", "live", "iproxy")


class RecordingMap(mapping.Map):
    """Andrew's Map; every scene that passes his validation is also offered to the recorder."""

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.recorder = None

    def update(self, value, source=None):
        result = super().update(value, source)
        rec = self.recorder
        if rec is not None:
            rec.add(value)
        return result


class MotionReader:
    """The phone's 5557 JSON-line motion stream over usbmux, checked with Andrew's validate()."""

    def __init__(self, usbmux_path):
        self.path = usbmux_path
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.sample, self.received, self.count, self.gaps = None, 0.0, 0, 0
        self.status = "connecting"
        threading.Thread(target=self.run, daemon=True, name="phone-motion").start()

    def run(self):
        while not self.stop.is_set():
            try:
                with usbmux.connect_phone(PHONE_MOTION_PORT, self.path) as s:
                    s.settimeout(1)
                    buf, prev = b"", None
                    self.status = "live"
                    while not self.stop.is_set():
                        data = s.recv(8192)
                        if not data:
                            raise ConnectionError("phone closed motion stream")
                        buf += data
                        if len(buf) > 65536:
                            raise ValueError("oversized motion buffer")
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            smp = validate_motion(__import__("json").loads(line))
                            with self.lock:
                                if prev is not None:
                                    self.gaps += max(0, smp["seq"] - prev - 1)
                                prev = smp["seq"]
                                self.sample, self.received = smp, time.monotonic()
                                self.count += 1
            except (OSError, ValueError, usbmux.UsbmuxError) as e:
                self.status = str(e)[:160]
                self.stop.wait(1)


def _stop_feed(feed):
    # Same three steps as dashboard.stop_feed (not imported: dashboard.py is the SDL app).
    feed.stop.set()
    with feed.lock:
        sock = feed.sock
    if sock:
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass


def port_free(port: int) -> bool:
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("127.0.0.1", port))
            return True
        except OSError:
            return False


class PhonePipeline:
    def __init__(self, room_port=5594, scene_port=5595, usbmux_path=usbmux.SOCKET_PATH,
                 state_dir=None, on_update=None, publish_period=0.5):
        self.room_port, self.scene_port, self.usbmux_path = room_port, scene_port, usbmux_path
        self.state_dir = state_dir or os.path.join(tempfile.gettempdir(), "tars-bench-phone")
        os.makedirs(self.state_dir, exist_ok=True)
        self.on_update, self.period = on_update, publish_period
        self.lock = threading.RLock()
        self.kind, self.opts, self.error = "off", {}, ""
        self.feed = self.server = self.forwarder = self.motion = self.thread = None
        self.mapper = self._new_map()
        self.recorder = None
        self.truth = None          # (x, y, th, monotonic) from the sim car, world frame
        self.route = None          # [[x, z], ...] from goto NAV ROUTE
        self.sim_phone = None
        self.world_room = None     # latest room from the sim world (topic `world`), if any
        self.world_seen = 0.0
        self._rate = (time.monotonic(), 0, 0.0)
        self.closed = threading.Event()
        self.link = SimpleNamespace(latest={}, room_published=0, lock=threading.Lock())
        self.room_error = ""
        if not port_free(room_port):
            self.room_error = f"room view port {room_port} busy"
        room_view.PORT = room_port  # serve_room reads the module global when it binds
        threading.Thread(target=room_view.serve_room, args=(self.link, self.closed), daemon=True,
                         name="room-view").start()
        threading.Thread(target=self._publish_loop, daemon=True, name="phone-publish").start()

    # ---- sources --------------------------------------------------------------------------
    def _new_map(self):
        # native=None: Andrew's Python reference grid. NativeGrid would open goto's
        # 5570 itself, and on the bench only the robot plugin talks to goto.
        return RecordingMap(save_path=os.path.join(self.state_dir, "map.json"), native=None)

    def _teardown(self):
        if self.feed is not None:
            _stop_feed(self.feed)
        for obj in (self.server, self.forwarder):
            if obj is not None:
                obj.close()
        if self.motion is not None:
            self.motion.stop.set()
        if self.thread is not None:
            self.thread.join(timeout=3)
        self.feed = self.server = self.forwarder = self.motion = self.thread = None
        self.sim_phone = None

    def set_source(self, kind: str, **opts) -> dict:
        if kind not in SOURCES:
            raise ValueError(f"source must be one of {SOURCES}")
        with self.lock:
            self._teardown()
            if self.recorder is not None:
                self.mapper.recorder = None
            self.mapper = self._new_map()
            self.mapper.recorder = self.recorder
            self.kind, self.opts, self.error = kind, opts, ""
            self.route = None
            port = int(opts.get("port") or self.scene_port)
            try:
                if kind == "off":
                    return self.status()
                if kind == "sim":
                    room = opts.get("room") or self.world_room
                    self.sim_phone = sim.SimPhone(room=room, seed=int(opts.get("seed", 1)),
                                                  session=f"sim-{int(time.time())}")
                    if room is not None:
                        self.sim_phone.room_source = "request" if opts.get("room") else "world"
                    if "range_noise" in opts:
                        self.sim_phone.range_noise = float(opts["range_noise"])
                    if "dropout" in opts:
                        self.sim_phone.dropout = float(opts["dropout"])
                    if "pitch_deg" in opts:
                        self.sim_phone.pitch = math.radians(float(opts["pitch_deg"]))
                    pose_fn = self._sim_pose_fn(opts.get("pose", "auto"))
                    self.server = scene_io.SceneServer(
                        sim.sim_factory(self.sim_phone, pose_fn, float(opts.get("rate", 10)),
                                        opts.get("duration"), realtime=opts.get("realtime", True)),
                        port, "sim-phone")
                elif kind == "replay":
                    path = self._recording_path(opts["path"])
                    self.server = scene_io.SceneServer(
                        scene_io.replay_factory(path, float(opts.get("speed", 1)), bool(opts.get("loop"))),
                        port, "replay")
                elif kind == "live":
                    self.forwarder = usbmux.Forwarder(port, PHONE_SCENE_PORT, self.usbmux_path)
                    self.motion = MotionReader(self.usbmux_path)
                # iproxy: the user runs `iproxy -l -s 127.0.0.1 <port>:5561`; we only connect.
            except (OSError, ValueError, KeyError) as e:
                self.error = f"{kind}: {e}"
                self.kind = "off"
                self._teardown()
                return self.status()
            self.feed = mapping.SceneFeed(port=port, mapper=self.mapper, fusion_port=0)
            self.thread = threading.Thread(target=self.feed.run, daemon=True, name="scene-feed")
            self.thread.start()
            return self.status()

    def _sim_pose_fn(self, mode):
        path = sim.ScriptedPath()

        def pose(t):
            truth = self.truth
            if mode in ("auto", "car") and truth and time.monotonic() - truth[3] < 1.0:
                return truth[:3]
            if mode == "car":
                return None
            return path.pose(t)
        return pose

    def _recording_path(self, name):
        path = name if os.path.isabs(name) else os.path.join(RECORDINGS, os.path.basename(name))
        if not os.path.isfile(path):
            raise ValueError(f"no recording {name}")
        return path

    def recordings(self):
        os.makedirs(RECORDINGS, exist_ok=True)
        return sorted(f for f in os.listdir(RECORDINGS) if f.endswith(".jsonl"))

    # ---- recording --------------------------------------------------------------------------
    def record(self, on: bool, name: str | None = None, directory: str | None = None) -> dict:
        with self.lock:
            if on and self.recorder is None:
                directory = directory or RECORDINGS
                os.makedirs(directory, exist_ok=True)
                name = os.path.basename(name or time.strftime(f"{self.kind}-%Y%m%d-%H%M%S"))
                if not name.endswith(".jsonl"):
                    name += ".jsonl"
                self.recorder = scene_io.Recorder(os.path.join(directory, name), self.kind)
                self.mapper.recorder = self.recorder
            elif not on and self.recorder is not None:
                self.mapper.recorder = None
                self.recorder.close()
                done = self.recorder
                self.recorder = None
                return {"saved": done.path, "scenes": done.count}
            return {"recording": self.recorder.path if self.recorder else None}

    # ---- overlays ------------------------------------------------------------------------
    def set_truth(self, x, y, th):
        self.truth = (float(x), float(y), float(th), time.monotonic())  # car truth wins

    def set_world(self, data) -> bool:
        """A `world` message from the sim world. If it carries a room, the sim feed
        re-ray-casts against it from the next scene on. Returns True if geometry changed.
        A robot pose in it ("car"/"robot"/"pose" with x, y, th) is used when no car truth."""
        room = sim.find_room(data)
        if room is None:
            return False
        try:
            norm = sim.room_from_json(room)
        except (TypeError, ValueError, KeyError, IndexError):
            return False
        self.world_room, self.world_seen = norm, time.monotonic()
        if isinstance(data, dict):
            for key in ("car", "robot", "pose"):
                p = data.get(key)
                if isinstance(p, dict) and all(k in p for k in ("x", "y", "th")) and not (
                        self.truth and time.monotonic() - self.truth[3] < 0.5 and self.truth[4:] == ()):
                    self.truth = (float(p["x"]), float(p["y"]), float(p["th"]), time.monotonic(), "world")
                    break
        phone = self.sim_phone
        if phone is not None and self.opts.get("room") is None:
            return phone.set_room(norm, "world")
        return False

    def set_route(self, route):
        self.route = route

    def command(self, cmd: dict) -> dict:
        return self.mapper.command(cmd)  # Andrew's map_clear / map_save / map_load / select_target

    # ---- output ----------------------------------------------------------------------------
    def room(self) -> dict:
        """What room_view.html reads (Andrew's snapshot + motion + bench overlays)."""
        snap = self.mapper.snapshot()
        objects = list(snap.get("objects", []))
        truth = self.truth
        if self.kind == "sim" and truth and time.monotonic() - truth[3] < 1.0:
            objects.append({"id": "truth", "label": "sim car truth", "confidence": 1.0,
                            "position": sim.to_arkit(truth[0], truth[1], 0.0), "age_s": 0,
                            "currently_seen": True, "stale": False, "restored": False})
        snap["objects"] = objects
        if self.route and not snap.get("path"):
            snap["path"] = self.route
            snap["plan_status"] = "goto NAV ROUTE (goto's nav frame)"
        status = {"type": "status", "surroundings": snap}
        m = self.motion
        if m is not None:
            with m.lock:
                smp, age = m.sample, time.monotonic() - m.received
            if smp:
                status.update(accel_m_s2=smp["accel_m_s2"], gyro_rad_s=smp["gyro_rad_s"],
                              motion_live=age < 0.5, motion_hz=getattr(m, "hz", 0.0))
        return status

    def status(self) -> dict:
        feed, m = self.feed, self.mapper
        with m.lock:
            grid = bytes(m.grid)
            info = {"frames": m.frames, "voxels": len(m.voxels), "tracking": m.tracking,
                    "floor_ready": m.floor_y is not None, "objects": len(m.objects),
                    "free_cells": grid.count(1), "occupied_cells": grid.count(2),
                    "message": m.message}
        link = "off"
        if feed is not None:
            with feed.lock:
                link = feed.status
        src = self.server or self.forwarder
        return {"source": self.kind, "opts": {k: v for k, v in self.opts.items() if k != "room"},
                "link": link, "source_state": getattr(src, "state", ""), "error": self.error,
                "scenes_per_s": round(self._rate[2], 1), "scene_port": int(self.opts.get("port") or self.scene_port),
                "room_url": f"http://127.0.0.1:{self.room_port}/", "room_error": self.room_error,
                "motion": None if self.motion is None else {
                    "status": self.motion.status, "count": self.motion.count, "gaps": self.motion.gaps,
                    "hz": round(getattr(self.motion, "hz", 0.0), 1)},
                "recording": None if self.recorder is None else {"path": self.recorder.path,
                                                                 "scenes": self.recorder.count},
                "room_source": (self.sim_phone.room_source if self.sim_phone else
                                ("world" if self.world_room else "static")),
                "world_loaded": self.world_room is not None,
                "room_counts": None if self.sim_phone is None else {
                    k: len(self.sim_phone.room[k]) for k in ("walls", "boxes", "cylinders")},
                "truth_overlay": bool(self.truth and time.monotonic() - self.truth[3] < 1.0),
                "route_overlay": bool(self.route), **info}

    def _publish_loop(self):
        last_motion = (time.monotonic(), 0)
        while not self.closed.wait(self.period):
            try:
                now = time.monotonic()
                t0, n0, _ = self._rate
                count = self.feed.count if self.feed is not None else 0
                if now - t0 >= 1.0:
                    self._rate = (now, count, max(0.0, (count - n0) / (now - t0)) if count >= n0 else 0.0)
                m = self.motion
                if m is not None and now - last_motion[0] >= 1.0:
                    m.hz = (m.count - last_motion[1]) / (now - last_motion[0])
                    last_motion = (now, m.count)
                room = self.room()
                with self.link.lock:
                    self.link.latest = room
                    self.link.room_published = now
                if self.on_update:
                    self.on_update(self.status())
            except Exception as e:  # never let the publisher die
                self.error = f"publish: {e}"

    def close(self):
        with self.lock:
            self._teardown()
            if self.recorder is not None:
                self.recorder.close()
        self.closed.set()
