#!/usr/bin/env python3
"""TARS bench world sim (laptop only): a room, and the phone + webcam sensors in it.

    python3 bench/world/worldsim.py --room default \
        --truth 5590 --car-ctl 5591 --ctl 5592 --state-to 127.0.0.1:5593 \
        --guard 5600 --nav 5572 --det 5571

Reads the car's TRUE pose (virtual car or bench/world/stub_car.py, UDP JSON on --truth) and
produces what the robot really receives, in the receivers' own formats:
  * LiDAR guard verdicts -> tars_motor -g port (GAT1, 13 bytes, CLOCK_MONOTONIC us), computed
    with lidar_guard.cpp's rules (|side| < 0.45 m, BLOCKED <= 0.6 m, CLEAR needs floor hits
    in three bands, permit on CLEAR or CAUTION, STALE after 150 ms, tracking must be OK)
  * phone LDR1 packets (ARKit pose + depth points) -> goto's nav port, TCP length-prefixed
    (iter2-autonomy/goto/src/nav.c nav_ingest_ldr1)
  * YOLO detection JSON -> goto's detection port, UDP (iter2-autonomy/goto/DETECTIONS.md)
and sends the room's collision segments to the car (--car-ctl, {"walls": [...]}) whenever the
room changes. World control (--ctl, TCP lines or UDP datagrams of JSON) edits the room and
the sensor models live; world state goes to --state-to (UDP JSON). Formats: bench/world/README.md.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import random
import socket
import struct
import sys
import threading
import time
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C  # noqa: E402

GATE_STALE_S = 0.150          # lidar_guard STALE_MS
PERSON = {"label": "person", "r": 0.20, "h": 1.70}

# live-tunable parameters ({"op": "set", "params": {...}}), with their defaults
DEFAULTS = {
    # phone pose (ARKit)
    "pose_noise_m": 0.01,        # sigma, per packet
    "pose_noise_deg": 0.5,
    "pose_latency_ms": 60,       # the pose and depth describe the world this long ago
    "dropout_per_min": 0.0,      # random tracking-limited spells per minute
    "dropout_s": 1.0,
    "reloc_m": 0.25,             # size of a relocalisation jump (fault "reloc")
    "reloc_deg": 8.0,
    "phone_hz": 20,              # LDR1 rate
    "phone_h": 0.25,             # phone height above the floor
    # LiDAR
    "lidar_hfov_deg": 60,
    "lidar_range_m": 5.0,
    "lidar_rays": 25,
    "depth_noise_m": 0.01,
    "guard_hz": 100,             # GAT1 rate, like lidar_guard (tars_motor's TTL is 100 ms)
    "guard_stop_m": 0.6,
    "guard_half_w": 0.45,
    "guard_gob1": True,          # tag obstacle denials GOB1, as the robot runs lidar_guard (service.py
                                 # passes --identify-obstacles 1); only reason "obstacle", never tracking
    # webcam + YOLO
    "cam_hz": 8,
    "cam_latency_ms": 120,
    "cam_h": 0.12,
    "det_range_m": 6.0,
    "conf_near": 0.85,           # confidence up to 2 m ...
    "conf_far": 0.35,            # ... falling to this at det_range_m
    "miss_rate": 0.05,           # per object per frame
    "fp_rate": 0.0,              # random false detection per frame
    "det_jitter_px": 2.0,
}
FAULTS = ("pose_stale", "pose_lost", "pose_frozen", "reloc", "guard_silent", "guard_invalid",
          "guard_stale", "false_det", "person", "cam_off")


class World:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.rng = random.Random(args.seed)
        self.params = dict(DEFAULTS)
        self.conf = C.goto_conf()
        self.hfov = math.radians(float(self.conf.get("hfov_deg", 60)))
        self.labels = C.goto_objects() or ["trash can"]
        self.faults: dict[str, float] = {}          # name -> expiry (monotonic) or inf
        self.fault_data: dict[str, dict] = {}
        self.reloc = (0.0, 0.0, 0.0)                # persistent ARKit frame offset (dx, dy, dth) world
        self.truth = None                            # latest truth dict (+ "_rx" monotonic)
        self.hist = collections.deque(maxlen=400)    # (rx monotonic, x, y, th)
        self.rev = 0
        self.t0 = time.monotonic()
        self.seq = 0
        self.frozen = None
        self.dropout_until = 0.0
        self.last_frame = None                       # latest LiDAR frame result (for guard + state)
        self.last_dets: list = []
        self.guard_out = {"verdict": "UNKNOWN", "permit": 0, "reason": "waiting_for_frame", "sent": "-"}
        self.counts = collections.Counter()
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.navsock = None
        self.nav_cv = threading.Condition()
        self.nav_pending: bytes | None = None
        self.nav_seed: list[bytes] = []              # prescan seed packets, sent before live ones
        self.state_to = [_addr(a) for a in args.state_to]
        self.load(C.load_room_file(args.room) if not args.room.endswith(".json")
                  else C.normalise_room(json.load(open(args.room))))

    # ---------------------------------------------------------------- room
    def load(self, room: dict, teleport=True):
        with self.lock:
            self.room = room
            self.original = json.loads(json.dumps(room))
            self.reloc = (0.0, 0.0, 0.0)
            self.faults.clear()
            self.fault_data.clear()
            self._changed()
        if teleport:
            s = room["start"]
            self.car_ctl({"pose": {"x": s["x"], "y": s["y"], "th": s["th"]}})
            self.car_ctl({"fault": "clear"})

    def _changed(self):
        """room edited: rebuild ray casting, resend the car's collision walls, push the room"""
        self.scene = C.Scene(self.room)
        self.rev += 1
        self.send_walls()
        self.send_room()

    def send_walls(self):
        self.car_ctl({"walls": C.collision_segments(self.room)})

    def car_ctl(self, msg: dict):
        if self.args.car_ctl:
            try:
                self.udp.sendto(json.dumps(msg, separators=(",", ":")).encode(), ("127.0.0.1", self.args.car_ctl))
            except OSError:
                pass

    def room_msg(self) -> dict:
        with self.lock:
            r = self.room
            return {"type": "room", "rev": self.rev, "name": r["name"], "about": r.get("about", ""),
                    "start": r["start"], "wall_h": r["wall_h"], "walls": r["walls"],
                    "objects": r["objects"], "memory": r.get("memory", []),
                    "labels": self.labels, "hfov_deg": math.degrees(self.hfov), "params": self.params}

    def send_room(self):
        self._send_state(self.room_msg())

    def _send_state(self, msg: dict):
        data = json.dumps(msg, separators=(",", ":")).encode()
        for a in self.state_to:
            try:
                self.udp.sendto(data, a)
            except OSError as e:          # macOS: datagram > 9216 bytes. Rooms must stay small.
                self.counts["state_send_err"] += 1
                if self.counts["state_send_err"] == 1:
                    print(f"world: state datagram of {len(data)} bytes not sent ({e})", file=sys.stderr)

    # ---------------------------------------------------------------- control
    def control(self, msg: dict) -> dict:
        op = msg.get("op")
        if op is None:              # shorthand: {"set": {...}} / {"fault": "x"}
            if "set" in msg:
                op, msg = "set", {"params": msg["set"]}
            elif "fault" in msg:
                op, msg = "fault", {**msg, "name": msg["fault"]}
        with self.lock:
            if op == "ping":
                return {"ok": True, "rev": self.rev}
            if op == "get_room":
                return {"ok": True, "room": self.room_msg()}
            if op == "get_state":
                return {"ok": True, "state": self.state_msg()}
            if op == "rooms":
                return {"ok": True, "rooms": C.list_rooms()}
            if op == "set":
                bad = [k for k in msg.get("params", {}) if k not in DEFAULTS]
                if bad:
                    return {"ok": False, "error": f"unknown params {bad}"}
                for k, v in msg["params"].items():
                    self.params[k] = type(DEFAULTS[k])(v) if not isinstance(DEFAULTS[k], bool) else bool(v)
                self.send_room()
                return {"ok": True, "params": self.params}
            if op == "fault":
                return self._fault(msg)
            if op == "add":
                o = msg.get("obj") or {}
                if o.get("kind") == "person" or o.get("label") == "person" and "r" not in o:
                    o = {**PERSON, **o}
                oid = o.get("id") or self._new_id(o.get("label") or "obstacle")
                if any(x["id"] == oid for x in self.room["objects"]):
                    return {"ok": False, "error": f"id {oid} exists"}
                try:
                    o = C.normalise_object(o, oid)
                except (KeyError, ValueError, TypeError) as e:
                    return {"ok": False, "error": f"bad object: {e}"}
                self.room["objects"].append(o)
                self._changed()
                return {"ok": True, "id": oid, "rev": self.rev}
            if op == "add_wall":
                seg = [float(v) for v in msg["seg"]]
                if len(seg) != 4 or math.hypot(seg[2] - seg[0], seg[3] - seg[1]) < 0.02:
                    return {"ok": False, "error": "wall needs [x1,y1,x2,y2], at least 2 cm long"}
                self.room["walls"].append(seg)
                self._changed()
                return {"ok": True, "id": f"w{len(self.room['walls']) - 1}", "rev": self.rev}
            if op in ("update", "move"):
                return self._update(msg)
            if op == "remove":
                oid = str(msg.get("id"))
                if oid.startswith("w") and oid[1:].isdigit():
                    i = int(oid[1:])
                    if not 0 <= i < len(self.room["walls"]):
                        return {"ok": False, "error": "no such wall"}
                    del self.room["walls"][i]
                else:
                    n = len(self.room["objects"])
                    self.room["objects"] = [o for o in self.room["objects"] if o["id"] != oid]
                    if len(self.room["objects"]) == n:
                        return {"ok": False, "error": f"no object {oid}"}
                self._changed()
                return {"ok": True, "rev": self.rev}
            if op == "load":
                try:
                    room = C.normalise_room(msg["room"]) if "room" in msg else C.load_room_file(msg["name"])
                except (OSError, ValueError, KeyError, TypeError) as e:
                    return {"ok": False, "error": f"load: {e}"}
                self.load(room)
                return {"ok": True, "rev": self.rev, "goto_memory": C.goto_memory_text(self.room)}
            if op == "reset":
                self.load(C.normalise_room(self.original))
                return {"ok": True, "rev": self.rev, "goto_memory": C.goto_memory_text(self.room)}
            if op == "save":
                name = str(msg.get("name", ""))
                if not C.NAME_RE.match(name):
                    return {"ok": False, "error": "name: letters, digits, _ or -, up to 40"}
                path = os.path.join(C.ROOMS, name + ".json")
                if os.path.exists(path) and not msg.get("overwrite"):
                    return {"ok": False, "error": f"{name} exists (send overwrite: true)", "exists": True}
                self.room["name"] = name
                if "about" in msg:
                    self.room["about"] = str(msg["about"])
                with open(path + ".tmp", "w") as f:
                    json.dump(C.room_for_disk(self.room), f, indent=1)
                    f.write("\n")
                os.replace(path + ".tmp", path)
                self.original = json.loads(json.dumps(self.room))
                self.send_room()
                return {"ok": True, "path": os.path.relpath(path, C.REPO)}
            if op == "teleport":
                self.car_ctl({"pose": {k: float(msg.get(k, 0)) for k in ("x", "y", "th")}})
                return {"ok": True}
            if op == "seed_nav":      # bench map modes: a prescan's cells into goto's nav map
                return self.seed_nav(msg.get("points") or [])
            if op == "car":           # pass-through to the car's control port (5591 format)
                self.car_ctl(msg.get("msg", {}))
                return {"ok": True}
        return {"ok": False, "error": f"unknown op {op!r}"}

    def _new_id(self, label: str) -> str:
        base = "".join(ch for ch in label.replace(" ", "_") if ch.isalnum() or ch == "_") or "obj"
        ids = {o["id"] for o in self.room["objects"]}
        k = 1
        while f"{base}{k}" in ids:
            k += 1
        return f"{base}{k}"

    def _update(self, msg: dict) -> dict:
        oid = str(msg.get("id"))
        if oid.startswith("w") and oid[1:].isdigit() and "seg" in msg:
            i = int(oid[1:])
            if not 0 <= i < len(self.room["walls"]):
                return {"ok": False, "error": "no such wall"}
            self.room["walls"][i] = [float(v) for v in msg["seg"]]
        else:
            o = next((o for o in self.room["objects"] if o["id"] == oid), None)
            if o is None:
                return {"ok": False, "error": f"no object {oid}"}
            new = dict(o)
            for k in ("x", "y", "yaw", "w", "d", "r", "h"):
                if k in msg:
                    new[k] = float(msg[k])
            for k in ("label",):
                if k in msg:
                    new[k] = str(msg[k])
            if "solid" in msg:
                new["solid"] = bool(msg["solid"])
            o.clear()
            o.update(C.normalise_object(new, oid))
        self._changed()
        return {"ok": True, "rev": self.rev}

    def _fault(self, msg: dict) -> dict:
        name = msg.get("name")
        if name == "clear":
            self.faults.clear()
            self.fault_data.clear()
            self.room["objects"] = [o for o in self.room["objects"] if not o["id"].startswith("_fault_")]
            self._changed()
            return {"ok": True}
        if name not in FAULTS:
            return {"ok": False, "error": f"faults: {', '.join(FAULTS)}, clear"}
        on = msg.get("on", True)
        if not on:
            self._end_fault(name)
            return {"ok": True}
        dur = float(msg.get("s", 0) or 0)
        until = time.monotonic() + dur if dur > 0 else math.inf
        if name == "reloc":         # one-shot: the ARKit frame jumps and stays jumped
            p = self.params
            a = self.rng.uniform(0, 2 * math.pi)
            dth = math.radians(p["reloc_deg"]) * self.rng.choice((-1, 1))
            dx, dy, d0 = self.reloc
            self.reloc = (dx + p["reloc_m"] * math.cos(a), dy + p["reloc_m"] * math.sin(a), d0 + dth)
            return {"ok": True, "reloc": self.reloc}
        if name == "person":
            # a person steps in front of the car: a real obstacle in the room (guard, LiDAR map,
            # YOLO "person"), gone when the fault ends
            tr = self.pose_now()
            if tr is None:
                return {"ok": False, "error": "no car truth yet"}
            d = float(msg.get("dist", 0.45))
            x, y, th = tr
            o = C.normalise_object({**PERSON, "id": "_fault_person", "x": x + d * math.cos(th), "y": y + d * math.sin(th)}, "_fault_person")
            self.room["objects"] = [q for q in self.room["objects"] if q["id"] != "_fault_person"] + [o]
            self._changed()
        if name == "false_det":
            self.fault_data[name] = {"label": msg.get("label", "trash can"), "u": float(msg.get("u", 0.5))}
        if name == "pose_frozen":
            self.frozen = None
        self.faults[name] = until
        return {"ok": True}

    def _end_fault(self, name: str):
        self.faults.pop(name, None)
        self.fault_data.pop(name, None)
        if name == "person":
            self.room["objects"] = [o for o in self.room["objects"] if o["id"] != "_fault_person"]
            self._changed()

    def fault(self, name: str) -> bool:
        until = self.faults.get(name)
        if until is None:
            return False
        if time.monotonic() >= until:
            with self.lock:
                self._end_fault(name)
            return False
        return True

    # ---------------------------------------------------------------- truth
    def on_truth(self, msg: dict):
        now = time.monotonic()
        msg["_rx"] = now
        self.truth = msg
        self.hist.append((now, float(msg["x"]), float(msg["y"]), float(msg["th"])))

    def pose_now(self):
        t = self.truth
        if t is None or time.monotonic() - t["_rx"] > 0.5:
            return None
        return float(t["x"]), float(t["y"]), float(t["th"])

    def pose_at(self, latency_s: float):
        """truth latency_s ago (the sensors see the past)"""
        if not self.hist:
            return None
        now = time.monotonic()
        if now - self.hist[-1][0] > 0.5:
            return None
        want = now - latency_s
        for rx, x, y, th in reversed(self.hist):
            if rx <= want:
                return x, y, th
        return self.hist[0][1:]

    # ---------------------------------------------------------------- phone: LiDAR frame
    def lidar_frame(self, pose):
        """rays over the LiDAR FOV from the true pose -> points relative to the robot
        [(forward, left, height)], plus the rays for the 2D view"""
        p = self.params
        x, y, th = pose
        n = max(3, int(p["lidar_rays"]))
        hf = math.radians(p["lidar_hfov_deg"]) / 2
        jit = self.rng.uniform(-0.5, 0.5) * (2 * hf / (n - 1))
        pts, rays = [], []
        floor_step = 0.10
        for k in range(n):
            a = -hf + 2 * hf * k / (n - 1) + jit
            t, oid, h = self.scene.ray(x, y, th + a, p["lidar_range_m"])
            t_n = t + self.rng.gauss(0, p["depth_noise_m"]) if oid else t
            rays.append((round(a, 3), round(t, 3), oid))
            ca, sa = math.cos(a), math.sin(a)
            s = 0.25 + self.rng.uniform(0, floor_step)
            while s < min(t - 0.05, 3.0) and len(pts) < 880:
                pts.append((s * ca, s * sa, self.rng.gauss(0, 0.005)))
                s += floor_step
            if oid:
                top = min(h, 1.2)
                for hh in (0.1, 0.3, 0.6, 1.0):
                    if hh <= top and len(pts) < 1024:
                        pts.append((t_n * ca, t_n * sa, hh))
        return pts, rays

    def guard_verdict(self, pts, flags):
        """lidar_guard.cpp parse(): the same rules on the same points"""
        p = self.params
        hw, stop = p["guard_half_w"], p["guard_stop_m"]
        nearest, blocked = None, False
        bands = [0, 0, 0]
        for f, side, h in pts:
            if abs(side) < hw and f > 0 and 0.08 <= h <= 1.2:
                nearest = f if nearest is None or f < nearest else nearest
                if f <= stop:
                    blocked = True
            if abs(side) < hw and 0.15 <= f <= 1.5 and abs(h) <= 0.06:
                bands[0 if side < -0.15 else (2 if side > 0.15 else 1)] += 1
        if flags != 7:
            return "UNKNOWN", None, bands
        if blocked:
            return "BLOCKED", nearest, bands
        # lidar_guard's side < -.15 band is its "right" (side = left in our points: same sign rule)
        if len(pts) >= 15 and min(bands) >= 2:
            return "CLEAR", nearest, bands
        return ("CAUTION" if pts else "UNKNOWN"), nearest, bands

    def phone_tick(self):
        p = self.params
        now = time.monotonic()
        pose = self.pose_at(p["pose_latency_ms"] / 1000)
        if pose is None or self.fault("pose_stale"):
            self.last_frame = None if pose is None else self.last_frame
            return
        if p["dropout_per_min"] > 0 and now >= self.dropout_until and \
                self.rng.random() < p["dropout_per_min"] / 60 / max(1, p["phone_hz"]):
            self.dropout_until = now + p["dropout_s"]
        tracking = not (self.fault("pose_lost") or now < self.dropout_until)
        flags = 7 if tracking else 0
        pts, rays = self.lidar_frame(pose)
        verdict, nearest, bands = self.guard_verdict(pts, flags)
        # what the phone believes: truth + its frame offset (relocalisation) + noise; frozen pins it
        x, y, th = pose
        dx, dy, dth = self.reloc
        bx = x + dx + self.rng.gauss(0, p["pose_noise_m"])
        by = y + dy + self.rng.gauss(0, p["pose_noise_m"])
        bth = th + dth + math.radians(self.rng.gauss(0, p["pose_noise_deg"]))
        if self.fault("pose_frozen"):
            if self.frozen is None:
                self.frozen = (bx, by, bth)
            bx, by, bth = self.frozen
        start = self.room["start"]
        xa, za, yaw = C.world_to_ark(start, bx, by, bth)
        cy, sy = math.cos(yaw), math.sin(yaw)
        ark = [(xa + f * sy + l * cy, h, za + f * cy - l * sy) for f, l, h in pts]
        self.seq += 1
        uptime = int((now - self.t0) * 1e6) + 1
        body = b"LDR1" + struct.pack(">IQII", self.seq, uptime, flags, len(ark))
        body += struct.pack(">4f", xa, p["phone_h"], za, yaw) + struct.pack(">f", 0.0)
        body += struct.pack(f">{3 * len(ark)}f", *(v for q in ark for v in q))
        self.last_frame = {"rx": now, "flags": flags, "verdict": verdict, "nearest": nearest, "bands": bands,
                           "rays": rays, "phone": (bx, by, bth), "pose": pose, "npts": len(ark)}
        if self.args.nav:
            # a slow or restarting goto must never stall the frames the guard verdict comes from:
            # hand the packet to the sender thread; newest wins, older unsent ones are dropped
            with self.nav_cv:
                if self.nav_pending is not None:
                    self.counts["ldr1_dropped"] += 1
                self.nav_pending = struct.pack(">I", len(body)) + body
                self.nav_cv.notify()

    def seed_nav(self, points) -> dict:
        """SIM ONLY (bench map demo B): replay a prescan into goto's nav map as LDR1 packets on the
        phone's own feed: the points (ARKit frame, floor y = 0) in chunks of <= 1000, stamped with
        this feed's sequence/uptime and the pose the phone believes right now, tracking OK. goto has no
        map-load command; on the robot this would need one plus relocalisation (bench/maps/README.md)."""
        if not self.args.nav:
            return {"ok": False, "error": "no nav port (--nav)"}
        try:
            pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points]
        except (TypeError, ValueError, IndexError):
            return {"ok": False, "error": "points: [[x_ark, y, z_ark], ...]"}
        fr = self.last_frame
        if fr is not None:
            bx, by, bth = fr["phone"]
        else:
            pose = self.pose_now() or (self.room["start"]["x"], self.room["start"]["y"], self.room["start"]["th"])
            bx, by, bth = pose
        xa, za, yaw = C.world_to_ark(self.room["start"], bx, by, bth)
        pkts = []
        for k in range(0, len(pts), 1000):
            chunk = pts[k:k + 1000]
            with self.lock:
                self.seq += 1
                uptime = int((time.monotonic() - self.t0) * 1e6) + 1 + k
            body = b"LDR1" + struct.pack(">IQII", self.seq, uptime, 7, len(chunk))
            body += struct.pack(">4f", xa, self.params["phone_h"], za, yaw) + struct.pack(">f", 0.0)
            body += struct.pack(f">{3 * len(chunk)}f", *(v for q in chunk for v in q))
            pkts.append(struct.pack(">I", len(body)) + body)
        with self.nav_cv:
            self.nav_seed += pkts
            self.nav_cv.notify()
        self.counts["seed_pkts"] += len(pkts)
        return {"ok": True, "packets": len(pkts), "points": len(pts)}

    def nav_sender(self, stop: threading.Event):
        while not stop.is_set():
            with self.nav_cv:
                while self.nav_pending is None and not self.nav_seed and not stop.is_set():
                    self.nav_cv.wait(0.2)
                if self.nav_seed:
                    pkt = self.nav_seed.pop(0)
                else:
                    pkt, self.nav_pending = self.nav_pending, None
            if pkt is None:
                continue
            try:
                if self.navsock is None:
                    self.navsock = socket.create_connection(("127.0.0.1", self.args.nav), timeout=0.5)
                self.navsock.sendall(pkt)
                self.counts["ldr1"] += 1
            except OSError:
                try:
                    self.navsock and self.navsock.close()
                except OSError:
                    pass
                self.navsock = None
                self.counts["ldr1_fail"] += 1
                stop.wait(0.2)

    # ---------------------------------------------------------------- guard
    def guard_tick(self):
        if not self.args.guard or self.fault("guard_silent"):
            self.guard_out = {**self.guard_out, "sent": "silent"}
            return
        us = int(time.clock_gettime(time.CLOCK_MONOTONIC) * 1e6)
        if self.fault("guard_invalid"):
            pkt = b"GAT1" + us.to_bytes(8, "big") + bytes([7])       # permit byte > 1: invalid
            self.udp.sendto(pkt, ("127.0.0.1", self.args.guard))
            self.guard_out = {"verdict": "INVALID", "permit": 0, "reason": "invalid packets", "sent": "GAT1 bad"}
            return
        fr = self.last_frame
        now = time.monotonic()
        last = getattr(self, "_gat_last", None)
        if last is not None and now - last > self.counts.get("gat_max_gap_ms", 0) / 1000:
            self.counts["gat_max_gap_ms"] = int(1000 * (now - last))
        if last is not None and now - last >= 0.1 and not self.faults:
            self.counts["sim_guard_late"] += 1        # >= tars_motor's 100 ms TTL: it saw guard_stale (sim's fault)
        self._gat_last = now
        if fr is None or now - fr["rx"] >= GATE_STALE_S:
            verdict, reason = "STALE", "frame_late"
            if fr is not None and not self.faults:
                self.counts["sim_stale_ticks"] += 1       # the sim itself fell behind (not a fault)
        else:
            verdict = fr["verdict"]
            reason = "none" if verdict in ("CLEAR", "CAUTION") else \
                ("tracking_limited" if fr["flags"] != 7 else "obstacle")
        permit = verdict in ("CLEAR", "CAUTION")
        if self.fault("guard_stale"):
            us -= 300000                       # 300 ms old: tars_motor drops it (TTL 100 ms)
        # lidar_guard.cpp:389 tags GOB1 only for BLOCKED with reason "obstacle"
        magic = b"GOB1" if self.params["guard_gob1"] and verdict == "BLOCKED" and reason == "obstacle" else b"GAT1"
        self.udp.sendto(magic + us.to_bytes(8, "big") + bytes([1 if permit else 0]), ("127.0.0.1", self.args.guard))
        self.counts["gat"] += 1
        self.guard_out = {"verdict": verdict, "permit": int(permit), "reason": reason,
                          "sent": "stale stamps" if self.fault("guard_stale") else magic.decode()}

    # ---------------------------------------------------------------- webcam + YOLO
    def cam_tick(self):
        p = self.params
        pose = self.pose_at(p["cam_latency_ms"] / 1000)
        if pose is None or self.fault("cam_off"):
            self.last_dets = []
            return
        x, y, th = pose
        W, H = 640, 480
        fx = (W / 2) / math.tan(self.hfov / 2)
        dets, shown = [], []
        with self.lock:
            objs = [o for o in self.room["objects"] if o["label"]]
            scene = self.scene
        for o in objs:
            dx, dy = o["x"] - x, o["y"] - y
            f = dx * math.cos(th) + dy * math.sin(th)
            l = -dx * math.sin(th) + dy * math.cos(th)
            d = math.hypot(dx, dy)
            if f < 0.15 or d > p["det_range_m"]:
                continue
            half = o["r"] if "r" in o else 0.5 * max(o["w"], o["d"])
            # visibility: rays to 5 points across the object, perpendicular to the line of sight
            vis = 0
            for k in (-0.8, -0.4, 0.0, 0.4, 0.8):
                px, py = o["x"] - k * half * dy / d, o["y"] + k * half * dx / d
                dd = math.hypot(px - x, py - y)
                t, oid, _ = scene.ray(x, y, math.atan2(py - y, px - x), dd, skip=o["id"])
                if oid is None or t >= dd - half - 0.02:
                    vis += 1
            if vis == 0:
                continue
            frac = vis / 5
            if self.rng.random() < p["miss_rate"]:
                continue
            # pinhole, level camera: left of centre = smaller u
            ul = W / 2 - fx * (l + half) / f
            ur = W / 2 - fx * (l - half) / f
            top = H / 2 - fx * (o["h"] - p["cam_h"]) / f
            bot = H / 2 + fx * p["cam_h"] / f
            j = p["det_jitter_px"]
            box = [max(0.0, ul + self.rng.gauss(0, j)), max(0.0, top + self.rng.gauss(0, j)),
                   min(W, ur + self.rng.gauss(0, j)), min(H, bot + self.rng.gauss(0, j))]
            if box[2] - box[0] < 2 or box[3] - box[1] < 2:
                continue
            k = max(0.0, min(1.0, (d - 2.0) / max(0.1, p["det_range_m"] - 2.0)))
            conf = (p["conf_near"] + (p["conf_far"] - p["conf_near"]) * k) * (0.5 + 0.5 * frac)
            conf = max(0.05, min(0.99, conf + self.rng.gauss(0, 0.03)))
            dets.append({"label": o["label"], "conf": round(conf, 2), "box": [round(v, 1) for v in box]})
            shown.append({"id": o["id"], "label": o["label"], "conf": round(conf, 2), "x": o["x"], "y": o["y"], "vis": frac})
        if p["fp_rate"] > 0 and self.rng.random() < p["fp_rate"]:
            u = self.rng.uniform(40, 600)
            lab = self.rng.choice(self.labels)
            dets.append({"label": lab, "conf": round(self.rng.uniform(0.3, 0.6), 2), "box": [u - 30, 200, u + 30, 330]})
            shown.append({"id": None, "label": lab, "fp": True, "bearing": math.atan((W / 2 - u) / fx)})
        if self.fault("false_det"):
            fd = self.fault_data.get("false_det", {})
            u = W * fd.get("u", 0.5)
            dets.append({"label": fd.get("label", "trash can"), "conf": 0.7, "box": [u - 40, 180, u + 40, 360]})
            shown.append({"id": None, "label": fd.get("label", "trash can"), "fp": True, "bearing": math.atan((W / 2 - u) / fx)})
        line = json.dumps({"t": round(time.time(), 3), "w": W, "h": H, "dets": dets}, separators=(",", ":"))
        if self.args.det:
            try:
                self.udp.sendto(line.encode(), ("127.0.0.1", self.args.det))
                self.counts["det_frames"] += 1
            except OSError:
                pass
        self.last_dets = shown

    # ---------------------------------------------------------------- state for the 2D view
    def state_msg(self) -> dict:
        t = self.truth
        fr = self.last_frame
        truth = None
        if t is not None:
            truth = {k: t.get(k) for k in ("x", "y", "th", "v", "w", "contact", "stub", "batt_v")}
            truth["age_ms"] = int(1000 * (time.monotonic() - t["_rx"]))
        phone = None
        if fr:
            bx, by, bth = fr["phone"]
            phone = {"x": round(bx, 3), "y": round(by, 3), "th": round(bth, 4), "tracking": fr["flags"] == 7,
                     "age_ms": int(1000 * (time.monotonic() - fr["rx"])), "points": fr["npts"]}
        now = time.monotonic()
        return {"type": "state", "rev": self.rev, "room": self.room["name"], "truth": truth, "phone": phone,
                "guard": {**self.guard_out, "nearest": fr and fr["nearest"] and round(fr["nearest"], 3),
                          "bands": fr and fr["bands"], "rays": fr and fr["rays"],
                          "pose": fr and [round(v, 3) for v in fr["pose"]]},
                "dets": self.last_dets, "hfov_deg": round(math.degrees(self.hfov), 1),
                "lidar_hfov_deg": self.params["lidar_hfov_deg"],
                "faults": {k: (None if v == math.inf else round(v - now, 1)) for k, v in self.faults.items()},
                "reloc": [round(v, 3) for v in self.reloc], "counts": dict(self.counts)}


def _addr(s: str):
    h, _, p = s.rpartition(":")
    return (h or "127.0.0.1", int(p))


def every(hz_fn, fn, stop: threading.Event, name: str):
    nxt = time.monotonic()
    while not stop.is_set():
        try:
            fn()
        except Exception:
            print(f"world: {name} tick failed", file=sys.stderr)
            traceback.print_exc()
            time.sleep(0.5)
        nxt += 1.0 / max(1.0, float(hz_fn()))
        d = nxt - time.monotonic()
        if d < -0.5:
            nxt = time.monotonic()
        elif d > 0:
            stop.wait(d)


def guard_loop(w, stop: threading.Event):
    """GAT1 sender on its own plain-sleep schedule: tars_motor drops the guard after 100 ms"""
    nxt = time.monotonic()
    while not stop.is_set():
        try:
            w.guard_tick()
        except Exception:
            traceback.print_exc()
        nxt += 1.0 / max(10.0, float(w.params["guard_hz"]))
        d = nxt - time.monotonic()
        if d < -0.05:
            nxt = time.monotonic()
        elif d > 0:
            time.sleep(d)


def serve(args) -> int:
    sys.setswitchinterval(0.001)     # the GAT1 thread must not wait 5 ms+ per GIL handoff
    w = World(args)
    stop = threading.Event()

    # truth in
    tsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tsock.bind(("127.0.0.1", args.truth))
    tsock.settimeout(0.2)
    fwd = [_addr(a) for a in args.truth_fwd]

    def truth_loop():
        while not stop.is_set():
            try:
                data = tsock.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            for a in fwd:
                try:
                    w.udp.sendto(data, a)
                except OSError:
                    pass
            try:
                w.on_truth(json.loads(data))
            except (ValueError, KeyError, TypeError):
                w.counts["bad_truth"] += 1

    # control in: UDP datagrams (reply to sender) and TCP lines (reply per line)
    usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    usock.bind(("127.0.0.1", args.ctl))
    usock.settimeout(0.2)
    lsock = socket.socket()
    lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsock.bind(("127.0.0.1", args.ctl))
    lsock.listen(8)
    lsock.settimeout(0.2)

    def handle(raw: bytes) -> bytes:
        try:
            rep = w.control(json.loads(raw))
        except Exception as e:
            traceback.print_exc()
            rep = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        return (json.dumps(rep, separators=(",", ":")) + "\n").encode()

    def udp_ctl():
        while not stop.is_set():
            try:
                data, a = usock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                return
            rep = handle(data)
            try:
                usock.sendto(rep if len(rep) < 9000 else b'{"ok":false,"error":"reply too big for UDP: use TCP"}\n', a)
            except OSError:
                pass

    def tcp_client(c):
        f = c.makefile("rwb")
        try:
            for line in f:
                if line.strip():
                    f.write(handle(line))
                    f.flush()
        except (OSError, ValueError):
            pass
        finally:
            c.close()

    def tcp_ctl():
        while not stop.is_set():
            try:
                c, _ = lsock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            c.settimeout(None)
            threading.Thread(target=tcp_client, args=(c,), daemon=True).start()

    def state_tick():
        w._send_state(w.state_msg())

    k = [0]

    def walls_tick():            # periodic resend: the car may have restarted
        k[0] += 1
        w.send_walls()
        w.send_room()

    P = w.params
    threads = [threading.Thread(target=f, daemon=True) for f in (truth_loop, udp_ctl, tcp_ctl)]
    threads.append(threading.Thread(target=w.nav_sender, args=(stop,), daemon=True))
    threads.append(threading.Thread(target=guard_loop, args=(w, stop), daemon=True))
    threads += [threading.Thread(target=every, args=(hz, fn, stop, n), daemon=True) for hz, fn, n in (
        (lambda: P["phone_hz"], w.phone_tick, "phone"), 
        (lambda: P["cam_hz"], w.cam_tick, "cam"), (lambda: 10, state_tick, "state"), (lambda: 0.5, walls_tick, "walls"))]
    for t in threads:
        t.start()
    print(f"world: room={w.room['name']} truth=:{args.truth} car_ctl=:{args.car_ctl} ctl=:{args.ctl} "
          f"guard=:{args.guard} nav=:{args.nav} det=:{args.det} state->{args.state_to}", file=sys.stderr, flush=True)
    import signal
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while not stop.is_set():
            stop.wait(0.5)
    except KeyboardInterrupt:
        pass
    stop.set()
    for s in (tsock, usock, lsock):
        s.close()
    return 0


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="TARS bench world sim")
    ap.add_argument("--room", default="default", help="room name in bench/world/rooms, or a .json path")
    ap.add_argument("--truth", type=int, default=5590, help="UDP: car truth in")
    ap.add_argument("--truth-fwd", action="append", default=[], help="host:port to copy truth datagrams to")
    ap.add_argument("--car-ctl", type=int, default=5591, help="UDP: car control (walls, teleport); 0 = none")
    ap.add_argument("--ctl", type=int, default=5592, help="world control (TCP lines + UDP)")
    ap.add_argument("--state-to", action="append", default=[], help="UDP host:port for world state (default 127.0.0.1:5593)")
    ap.add_argument("--guard", type=int, default=0, help="tars_motor -g port (GAT1); 0 = no guard")
    ap.add_argument("--nav", type=int, default=0, help="goto nav port (LDR1 over TCP); 0 = no phone")
    ap.add_argument("--det", type=int, default=0, help="goto detection port (UDP JSON); 0 = no camera")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args(argv)
    if not a.state_to:
        a.state_to = ["127.0.0.1:5593"]
    return a


if __name__ == "__main__":
    sys.exit(serve(parse_args()))
