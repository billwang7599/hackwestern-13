#!/usr/bin/env python3
"""Socket-level tests + 2D simulator for the C goto controller (Mac/Linux host).

    python3 tests/test_goto_sim.py build/goto_host [--trials 40] [--seed 1] [--quick]

Pieces (stdlib only):
  FakeMotor   a tars_motor stand-in on TCP: D / S / TURN / MOVE / STATUS / PING /
              BEEP / LED, 500 ms watchdog, stops= counter, and differential-drive
              kinematics of the 28BYJ-48 rover (12 RPM, 65 mm wheels, 100 mm track).
              STATUS pose= is DEAD-RECKONED from the commands like the real one, so
              it drifts from the true pose (turn_err, drive_err).
  SimCamera   pinhole camera (60 deg HFOV, 640x480, mounted 12 cm up) looking at
              cylinders in the world; sends iter2-autonomy/goto/DETECTIONS.md JSON lines over UDP
              with latency, jitter, missed detections and motion blur.
goto runs with -T (clock N times faster) so a 60 s approach takes a few seconds.

Also runs goto against the REAL tars_motor host build (-b stub) if ../../iter1-wasd/pi/build has it,
to check the protocol end to end.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import re
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GOTO_DIR = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(GOTO_DIR))

WHEEL_D = 65.0
TRACK = 100.0
MAX_RPM = 12.0
V_MAX = MAX_RPM / 60.0 * math.pi * WHEEL_D / 1000.0   # m/s at D=1 (~0.041)
STEPS_PER_REV = 4075.7728
ACCEL_SPS2 = 3000.0

passed = failed = 0


def check(cond: bool, what: str) -> None:
    global passed, failed
    if cond:
        passed += 1
        print(f"  ok   {what}")
    else:
        failed += 1
        print(f"  FAIL {what}")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def move_secs(kind: str, amount: float, speed: float) -> float:
    """Same trapezoid as iter1-wasd/pi/src/wheels.h."""
    mm = abs(amount) if kind == "M" else math.pi * TRACK * abs(amount) / 360.0
    steps = mm * STEPS_PER_REV / (math.pi * WHEEL_D)
    v = MAX_RPM * STEPS_PER_REV / 60.0 * speed
    ramp = v * v / ACCEL_SPS2
    return 2 * v / ACCEL_SPS2 + (steps - ramp) / v if steps >= ramp else 2 * math.sqrt(steps / ACCEL_SPS2)


# ---------------------------------------------------------------- fake motor
class FakeMotor:
    def __init__(self, scale: float = 1.0, turn_err: float = 0.03, rng: random.Random | None = None,
                 drive_err: float = 0.0):
        self.scale = scale
        self.rng = rng or random.Random(0)
        self.turn_err = turn_err
        # true wheel speed / commanded (unequal wheels make it curve; odometry doesn't know)
        self.wl_err = 1.0 + (self.rng.uniform(-drive_err, drive_err) if drive_err else 0.0)
        self.wr_err = 1.0 + (self.rng.uniform(-drive_err, drive_err) if drive_err else 0.0)
        self.odo = [0.0, 0.0, 0.0]               # tars_motor pose: mm fwd, mm right, deg clockwise
        self.epoch = 4242
        self.lock = threading.Lock()
        self.x = self.y = self.th = 0.0          # m, m, rad (th=0 -> +x, + = counter-clockwise)
        self.tl = self.tr = 0.0                  # target wheel fractions
        self.vl = self.vr = 0.0                  # actual
        self.last_cmd = -10.0
        self.stops = 0
        self.move = None                         # dict(kind, id, start, dur, total_rad|total_m, done)
        self.move_id = 0
        self.move_state = "idle"
        self.move_why = "-"
        self.log: list[tuple[float, str]] = []   # (sim time, line)
        self.beeps: list[int] = []
        self.leds: list[str] = []
        self.port = free_port()
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(8)
        self.running = True
        self.t_last = self.now()
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._physics, daemon=True).start()

    def now(self) -> float:
        return time.monotonic() * self.scale

    def close(self) -> None:
        self.running = False
        try:
            self.srv.close()
        except OSError:
            pass

    # physics at ~200 Hz sim time
    def _physics(self) -> None:
        while self.running:
            time.sleep(0.005 / self.scale)
            with self.lock:
                self._step()

    def _step(self) -> None:
        t = self.now()
        dt = t - self.t_last
        self.t_last = t
        if dt <= 0:
            return
        if self.move and t - self.last_cmd > 0.5:   # deadman: exact moves need keepalive (main 23c8a36)
            self._abort_move("watchdog")
        if self.move:
            m = self.move
            frac = min(1.0, (t - m["start"]) / m["dur"])
            if m["kind"] == "R":
                ang = m["total"] * frac
                self.th = m["th0"] - ang            # + = right = clockwise
            else:
                d = m["total"] * frac
                self.x = m["x0"] + d * math.cos(self.th)
                self.y = m["y0"] + d * math.sin(self.th)
            m["done"] = frac
            if frac >= 1.0:
                self._odo_move(m, 1.0)
                self.move_state = "done"
                self.move = None
            return
        if t - self.last_cmd > 0.5:              # watchdog
            self.tl = self.tr = 0.0
        acc = 2.0 * dt                          # fraction/s slew (stepper ramp)
        self.vl += max(-acc, min(acc, self.tl - self.vl))
        self.vr += max(-acc, min(acc, self.tr - self.vr))
        self._odo_drive(self.vl * V_MAX * 1000, self.vr * V_MAX * 1000, dt)
        vl, vr = self.vl * V_MAX * self.wl_err, self.vr * V_MAX * self.wr_err
        v = (vl + vr) / 2.0
        w = (vr - vl) / (TRACK / 1000.0)
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.th += w * dt

    def _odo_advance(self, d_mm: float, dh_deg: float) -> None:
        mid = math.radians(self.odo[2] + dh_deg / 2)
        self.odo[0] += d_mm * math.cos(mid)
        self.odo[1] += d_mm * math.sin(mid)
        self.odo[2] = (self.odo[2] + dh_deg + 180) % 360 - 180

    def _odo_drive(self, vl: float, vr: float, dt: float) -> None:
        if abs(vl) > 1e-9 or abs(vr) > 1e-9:
            self._odo_advance((vl + vr) / 2 * dt, math.degrees((vl - vr) / TRACK * dt))

    def _odo_move(self, m: dict, frac: float) -> None:
        # the Uno reports the COMMANDED steps done, not what the floor did
        if m["kind"] == "R":
            self._odo_advance(0, m["cmd"] * frac)
        else:
            self._odo_advance(m["cmd"] * frac, 0)

    def restart_pose(self) -> None:
        # tars_motor restarted: pose back to 0, new epoch; the robot didn't move
        with self.lock:
            self.odo = [0.0, 0.0, 0.0]
            self.epoch += 1

    def angular_speed(self) -> float:
        with self.lock:
            if self.move and self.move["kind"] == "R":
                return math.radians(45)
            return abs((self.vr - self.vl) * V_MAX / (TRACK / 1000.0))

    def pose(self) -> tuple[float, float, float]:
        with self.lock:
            return self.x, self.y, self.th

    def set_pose(self, x: float, y: float, th: float) -> None:
        with self.lock:
            self.x, self.y, self.th = x, y, th

    def _abort_move(self, why: str) -> None:
        if self.move:
            self._odo_move(self.move, self.move["done"])
            self.move = None
            self.move_state = "aborted"
            self.move_why = why

    def _accept(self) -> None:
        while self.running:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._client, args=(c,), daemon=True).start()

    def _client(self, c: socket.socket) -> None:
        f = c.makefile("rw", encoding="ascii", newline="\n")
        try:
            for line in f:
                reply = self.handle(line.strip())
                f.write(reply + "\n")
                f.flush()
        except (OSError, ValueError):
            pass
        with self.lock:                          # disconnect -> stop, like tars_motor
            self.tl = self.tr = 0.0
            self._abort_move("disconnect")
        c.close()

    def handle(self, line: str) -> str:
        p = line.split()
        if not p:
            return "ERR empty"
        cmd = p[0].upper()
        with self.lock:
            t = self.now()
            self.log.append((t, line))
            if cmd == "D":
                self.last_cmd = t
                l, r = float(p[1]), float(p[2])
                if self.move and (abs(l) >= 0.03 or abs(r) >= 0.03):
                    self._abort_move("drive")
                self.tl, self.tr = max(-1, min(1, l)), max(-1, min(1, r))
                return "OK"
            if cmd == "S":
                self.stops += 1
                self.last_cmd = t
                self.tl = self.tr = self.vl = self.vr = 0.0
                self._abort_move("stop")
                return "OK"
            if cmd in ("TURN", "MOVE"):
                self.last_cmd = t
                amt = float(p[1])
                spd = float(p[2]) if len(p) > 2 else 1.0
                kind = "R" if cmd == "TURN" else "M"
                self.move_id += 1
                dur = move_secs(kind, amt, spd)
                err = 1.0 + self.rng.uniform(-self.turn_err, self.turn_err)
                self.tl = self.tr = self.vl = self.vr = 0.0
                self.move = {"kind": kind, "id": self.move_id, "start": t, "dur": max(dur, 1e-3),
                             "total": (math.radians(amt) if kind == "R" else amt / 1000.0) * err,
                             "th0": self.th, "x0": self.x, "y0": self.y, "done": 0.0, "cmd": amt}
                self.move_state = "running"
                self.move_why = "-"
                return f"OK {self.move_id} {int(dur * 1000)}"
            if cmd == "STATUS":
                kind = "-"
                return (f"OK {self.vl:.2f} {self.vr:.2f} {int((t - self.last_cmd) * 1000)} -1.00 0 why=none "
                        f"tipped=0 move={self.move_state} move_id={self.move_id} move_kind={kind} "
                        f"move_why={self.move_why} stops={self.stops} "
                        f"pose={self.odo[0]:.0f},{self.odo[1]:.0f},{self.odo[2]:.1f} pose_epoch={self.epoch}")
            if cmd == "PING":
                self.last_cmd = t
                return "OK PONG"
            if cmd == "BEEP":
                self.beeps.append(int(p[1]))
                return "OK"
            if cmd == "LED":
                self.leds.append(p[1])
                return "OK"
            return f"ERR unknown command '{p[0]}'"


# ---------------------------------------------------------------- sim camera
W, H = 640, 480
HFOV = math.radians(60.0)
FPX = (W / 2) / math.tan(HFOV / 2)
CAM_H = 0.12


class Obj:
    def __init__(self, label: str, x: float, y: float, r: float, h: float):
        self.label, self.x, self.y, self.r, self.h = label, x, y, r, h


def project(pose, o: Obj):
    """Box of a cylinder seen from the robot, or None if not in view."""
    x, y, th = pose
    dx, dy = o.x - x, o.y - y
    d = math.hypot(dx, dy)
    if d < 0.05:
        return None
    rel = math.atan2(dy, dx) - th
    rel = math.atan2(math.sin(rel), math.cos(rel))
    # camera: +x image = to the robot's right = negative rel
    if abs(rel) > math.pi / 2:
        return None
    zc = d * math.cos(rel)                      # depth along the optical axis
    if zc < 0.05:
        return None
    xc = -d * math.sin(rel)
    u = W / 2 + FPX * xc / zc
    hw = FPX * o.r / zc
    top = H / 2 - FPX * (o.h - CAM_H) / zc
    bot = H / 2 + FPX * CAM_H / zc
    x1, x2 = max(0.0, u - hw), min(float(W), u + hw)
    y1, y2 = max(0.0, top), min(float(H), bot)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None
    return [x1, y1, x2, y2]


class SimCamera:
    def __init__(self, motor: FakeMotor, objs: list[Obj], det_port: int, scale: float, rng: random.Random,
                 fps: float = 8.0, latency: float = 0.15, miss: float = 0.10, jitter_px: float = 3.0):
        self.m, self.objs, self.port, self.scale, self.rng = motor, objs, det_port, scale, rng
        self.fps, self.latency, self.miss, self.jitter = fps, latency, miss, jitter_px
        self.running = True
        self.paused = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frames = 0
        threading.Thread(target=self._run, daemon=True).start()

    def close(self) -> None:
        self.running = False

    def _run(self) -> None:
        pending: list[tuple[float, str]] = []
        next_t = self.m.now()
        while self.running:
            t = self.m.now()
            if t >= next_t and not self.paused:
                next_t = t + 1.0 / self.fps
                pose = self.m.pose()
                blur = self.m.angular_speed() > math.radians(25)
                dets = []
                for o in self.objs:
                    b = project(pose, o)
                    if b is None:
                        continue
                    if self.rng.random() < (0.6 if blur else self.miss):
                        continue
                    j = self.rng.gauss
                    b = [b[0] + j(0, self.jitter), b[1] + j(0, self.jitter), b[2] + j(0, self.jitter), b[3] + j(0, self.jitter)]
                    dets.append({"label": o.label, "conf": round(self.rng.uniform(0.45, 0.9), 2),
                                 "box": [round(v, 1) for v in b]})
                if self.rng.random() < 0.02:     # occasional false positive somewhere
                    dets.append({"label": "chair", "conf": 0.35, "box": [10, 200, 60, 300]})
                line = json.dumps({"t": round(t, 3), "w": W, "h": H, "dets": dets})
                pending.append((t + self.latency, line))
            while pending and pending[0][0] <= self.m.now():
                try:
                    self.sock.sendto(pending.pop(0)[1].encode(), ("127.0.0.1", self.port))
                    self.frames += 1
                except OSError:
                    pass
            time.sleep(0.004 / self.scale)


# ---------------------------------------------------------------- goto proc
class Goto:
    def __init__(self, exe: str, motor_port: int, scale: float = 1.0, extra: list[str] | None = None,
                 stdin_pipe: bool = False, mem: str | None = None):
        self.ctl_port = free_port()
        self.det_port = free_port()
        # every goto gets its own memory file (fresh unless `mem` names one to reuse)
        if mem is None:
            mem = os.path.join(GOTO_DIR, "build", f"mem_{self.ctl_port}.txt")
            if os.path.exists(mem):
                os.remove(mem)
        self.mem = mem
        args = [exe, "-m", f"127.0.0.1:{motor_port}", "-p", str(self.ctl_port), "-u", str(self.det_port),
                "-r", "0", "-T", str(scale), "-o", os.path.join(GOTO_DIR, "objects.txt"), "-M", mem] + (extra or [])
        self.log_path = os.path.join(GOTO_DIR, "build", f"goto_{self.ctl_port}.log")
        self.logf = open(self.log_path, "w")
        self.p = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=self.logf,
                                  stdin=subprocess.PIPE if stdin_pipe else subprocess.DEVNULL)
        self.sock = None
        for _ in range(100):
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.ctl_port), timeout=2)
                break
            except OSError:
                time.sleep(0.02)
        assert self.sock, "goto didn't open its control port"
        self.f = self.sock.makefile("rw", encoding="ascii", newline="\n")

    def cmd(self, line: str) -> str:
        self.f.write(line + "\n")
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if not r.startswith("EVENT"):
                return r

    def status(self) -> dict:
        r = self.cmd("STATUS")
        return dict(kv.split("=", 1) for kv in r.split()[1:] if "=" in kv)

    def close(self) -> None:
        try:
            self.sock.close()
        except Exception:
            pass
        self.p.terminate()
        try:
            self.p.wait(2)
        except subprocess.TimeoutExpired:
            self.p.kill()
        self.logf.close()


def wait_for(pred, timeout: float, step: float = 0.01) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return False


# ---------------------------------------------------------------- tests
def test_protocol(exe: str) -> None:
    print("control protocol")
    m = FakeMotor()
    g = Goto(exe, m.port)
    try:
        check(g.cmd("PING") == "OK PONG", "PING")
        objs = g.cmd("OBJECTS")
        check(objs.startswith("OK trash can,chair,person"), f"OBJECTS ({objs[:40]}...)")
        check(g.cmd("RESOLVE the garbage bin") == "OK trash can", "RESOLVE alias")
        check(g.cmd("RESOLVE banana").startswith("ERR"), "RESOLVE unknown")
        n_beeps = len(m.beeps)
        r = g.cmd("GOTO banana")
        check(r.startswith("ERR unknown object"), f"GOTO unknown -> {r}")
        check(wait_for(lambda: len(m.beeps) > n_beeps and m.beeps[-1] == 3, 1), "unknown object -> BEEP 3 on the robot")
        check(g.status()["state"] == "idle", "still idle")
        check(g.cmd("BOGUS").startswith("ERR"), "unknown command -> ERR")
        n = m.stops
        check(g.cmd("CANCEL") == "OK" and m.stops == n + 1, "CANCEL when idle still sends S")
        check(g.cmd("GOTO the trash can") == "OK goto trash can", "GOTO the trash can")
        st = g.status()
        check(st["state"] == "look" and st["target"] == "trash_can", f"STATUS {st['state']} {st['target']}")
        time.sleep(3.3)                          # no camera at all
        st = g.status()
        check(st["state"] == "idle" and "no_camera" in st["result"], f"no frames -> fail ({st['result']})")
        check(3 in m.beeps, "failure beeps 3")
    finally:
        g.close()
        m.close()

    print("motor server down")
    port = free_port()
    g = Goto(exe, port)
    try:
        r = g.cmd("GOTO chair")
        check(r.startswith("ERR no motor server"), f"GOTO with no tars_motor -> {r}")
    finally:
        g.close()


def drive_scene(exe: str, scale: float, seed: int, dist: float, bearing_deg: float, obj_h: float = 0.6,
                obj_r: float = 0.15, label: str = "trash can", say: str = "trash can", timeout: float = 260,
                distractor: bool = True, extra: list[str] | None = None):
    rng = random.Random(seed)
    m = FakeMotor(scale, rng=rng)
    b = math.radians(bearing_deg)
    objs = [Obj(label, dist * math.cos(b), dist * math.sin(b), obj_r, obj_h)]
    if distractor:   # a chair somewhere else in the room
        a = b + math.radians(rng.choice([-1, 1]) * rng.uniform(60, 150))
        objs.append(Obj("chair", 2.0 * math.cos(a), 2.0 * math.sin(a), 0.25, 0.9))
    g = Goto(exe, m.port, scale, extra)
    cam = SimCamera(m, objs, g.det_port, scale, rng)
    t0 = m.now()
    min_d = 1e9
    res = {}
    try:
        g.cmd(f"GOTO {say}")
        while m.now() - t0 < timeout:
            time.sleep(0.05)
            x, y, _ = m.pose()
            min_d = min(min_d, math.hypot(objs[0].x - x, objs[0].y - y) - obj_r)
            st = g.status()
            if st["state"] == "idle":
                res = st
                break
        x, y, th = m.pose()
        o = objs[0]
        d = math.hypot(o.x - x, o.y - y) - obj_r
        rel = math.degrees(math.atan2(math.sin(math.atan2(o.y - y, o.x - x) - th), math.cos(math.atan2(o.y - y, o.x - x) - th)))
        return {"result": res.get("result", "timeout"), "t": m.now() - t0, "dist": d, "bearing": rel,
                "min_d": min_d, "log": m.log, "stops": m.stops, "beeps": m.beeps}
    finally:
        cam.close()
        g.close()
        m.close()


def test_mission_cmds(exe: str) -> None:
    """DETS / NAV ROUTE / NAV GRID: read-only views for the laptop mission control."""
    print("mission control views (DETS, NAV ROUTE, NAV GRID)")
    m = FakeMotor()
    nav_port = free_port()
    g = Goto(exe, m.port, extra=["-n", str(nav_port)])
    try:
        check(g.cmd("DETS") == "OK age=-1.00 {}", "DETS before any detection")
        check(g.cmd("NAV ROUTE") == "OK route=- i=0", "NAV ROUTE when idle")
        r = g.cmd("NAV GRID")
        check(r.startswith("OK grid v=") and r.endswith("box=- rle=-"), f"NAV GRID empty map ({r[:60]})")
        det = json.dumps({"w": 640, "h": 480, "dets": [{"label": "chair", "conf": 0.8, "box": [10, 20, 110, 220]}]})
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.sendto(det.encode(), ("127.0.0.1", g.det_port))
        ok = wait_for(lambda: g.cmd("DETS").endswith(det), 1)
        r = g.cmd("DETS")
        check(ok and float(r.split()[1][4:]) < 1.0, f"DETS returns the last frame as sent ({r[:70]})")
        u.sendto(b"not json", ("127.0.0.1", g.det_port))
        time.sleep(0.05)
        check(g.cmd("DETS").endswith(det), "a bad line doesn't replace the last good frame")
        u.close()
        # one LDR1 packet: a floor patch ahead of the phone plus a wall -> free and occupied cells
        pts = [(x * 0.1, -0.3, 1.0 + z * 0.1) for x in range(-3, 4) for z in range(0, 5)]
        pts += [(x * 0.1, 0.2, 2.0) for x in range(-3, 4)]
        body = b"LDR1" + struct.pack("!IQII", 1, 1000, 1 | 2 | 4, len(pts)) + struct.pack("!5f", 0, 0, 0, 0, -0.3)
        body += b"".join(struct.pack("!3f", *p) for p in pts)
        t = socket.create_connection(("127.0.0.1", nav_port))
        t.sendall(struct.pack("!I", len(body)) + body)
        ok = wait_for(lambda: "occ=0" not in g.cmd("NAV"), 1)
        nav = dict(kv.split("=", 1) for kv in g.cmd("NAV").split()[1:])
        grid = dict(kv.split("=", 1) for kv in g.cmd("NAV GRID").split()[2:])
        runs = [tuple(map(int, r.split("x"))) for r in grid["rle"].split(",")]
        bx = list(map(int, grid["box"].split(",")))
        counts = {v: sum(n for c, n in runs if c == v) for v in (0, 1, 2)}
        check(ok and sum(n for _, n in runs) == bx[2] * bx[3], f"RLE covers the box ({bx})")
        check(counts[1] == int(nav["free"]) and counts[2] == int(nav["occ"]) and counts[2] > 0,
              f"GRID free/occ {counts[1]}/{counts[2]} match NAV {nav['free']}/{nav['occ']}")
        check(grid["v"] == nav["version"], "GRID version = NAV version")
        t.close()
    finally:
        g.close()
        m.close()


def test_scenarios(exe: str, scale: float) -> None:
    print(f"2D sim scenarios (goto -T {scale})")
    r = drive_scene(exe, scale, 1, 1.5, 0.0)
    check(r["result"] == "arrived", f"straight ahead 1.5 m: {r['result']} in {r['t']:.0f} s, stopped {r['dist']:.2f} m away")
    turns = [l for _, l in r["log"] if l.startswith("TURN")]
    check(not turns, f"no search turns when it's in view ({turns[:3]})")
    r = drive_scene(exe, scale, 5, 3.0, 5.0, obj_h=0.4)
    check(r["result"] == "arrived" and r["dist"] < 1.0, f"small can 3 m ahead: arrived in {r['t']:.0f} s, stopped {r['dist']:.2f} m away")
    ds = [(t, l) for t, l in r["log"] if l.startswith("D ")]
    gaps = [b[0] - a[0] for a, b in zip(ds, ds[1:])]
    check(gaps and max(gaps) < 0.3, f"D refreshed while driving (max gap {max(gaps) if gaps else 0:.2f} s < 0.3, watchdog 0.5)")

    r = drive_scene(exe, scale, 2, 2.0, 150.0)
    turns = [l for _, l in r["log"] if l.startswith("TURN")]
    check(r["result"] == "arrived" and len(turns) >= 3, f"behind the robot: searched ({len(turns)} turns) then arrived: {r['result']}")

    r = drive_scene(exe, scale, 3, 1.5, 0.0, label="trash can", say="door", distractor=False, timeout=120)
    turns = [l for _, l in r["log"] if l.startswith("TURN")]
    check(r["result"] == "failed:not_found" and len(turns) == 12 and 3 in r["beeps"],
          f"object not in the room: 12 x 30 deg then give up + BEEP 3 ({r['result']}, {len(turns)} turns)")

    r = drive_scene(exe, scale, 4, 2.5, 20.0, obj_h=1.7, obj_r=0.2, label="person", say="me")
    check(r["result"] == "arrived" and 0.15 < r["dist"] < 1.0,
          f"'come to me' (person, arrive at 80% height): stopped {r['dist']:.2f} m away")


def test_cancel_latency(exe: str) -> None:
    print("CANCEL and external stop (real time)")
    lat = []
    for i in range(5):
        rng = random.Random(10 + i)
        m = FakeMotor(1.0, rng=rng)
        g = Goto(exe, m.port, 1.0)
        cam = SimCamera(m, [Obj("trash can", 2.0, 0.0, 0.15, 0.6)], g.det_port, 1.0, rng, miss=0.0)
        try:
            g.cmd("GOTO trash can")
            ok = wait_for(lambda: any(l.startswith("D ") and float(l.split()[1]) > 0.1 for _, l in m.log), 5)
            if not ok:
                check(False, "never started driving")
                continue
            n = len(m.log)
            t0 = time.monotonic()
            r = g.cmd("CANCEL")
            t_reply = time.monotonic() - t0
            got = wait_for(lambda: any(l == "S" for _, l in m.log[n:]), 1, 0.0005)
            t_s = next(t for t, l in m.log[n:] if l == "S") - t0 if got else 99
            lat.append(t_s * 1000)
            time.sleep(0.5)
            later = [l for _, l in m.log[n:] if l.startswith("D ") and (abs(float(l.split()[1])) > 0.01 or abs(float(l.split()[2])) > 0.01)]
            if i == 0:
                check(r == "OK" and t_reply < 0.1, f"CANCEL replied OK in {t_reply * 1000:.1f} ms")
                check(not later, "no drive commands after CANCEL")
        finally:
            cam.close()
            g.close()
            m.close()
    if lat:
        check(max(lat) < 20, f"CANCEL -> S at tars_motor: median {sorted(lat)[len(lat) // 2]:.2f} ms, max {max(lat):.2f} ms (5 runs)")

    # someone else (voice "stop", teleop, hand) sends S straight to tars_motor
    m = FakeMotor(1.0)
    g = Goto(exe, m.port, 1.0)
    cam = SimCamera(m, [Obj("trash can", 2.0, 0.0, 0.15, 0.6)], g.det_port, 1.0, random.Random(5), miss=0.0)
    try:
        g.cmd("GOTO trash can")
        wait_for(lambda: any(l.startswith("D ") and float(l.split()[1]) > 0.1 for _, l in m.log), 5)
        other = socket.create_connection(("127.0.0.1", m.port))
        t_stop = m.now()
        other.sendall(b"S\n")
        other.recv(16)
        time.sleep(0.6)
        later = [l for t, l in m.log if t > t_stop + 0.3 and l.startswith("D ") and abs(float(l.split()[1])) > 0.01]
        st = g.status()
        check(not later and st["state"] == "idle" and "cancelled:stop" in st["result"],
              f"S from another client cancels the GOTO within one STATUS poll ({st['result']}, {len(later)} D after)")
        other.close()
    finally:
        cam.close()
        g.close()
        m.close()


def test_inputs(exe: str) -> None:
    print("detection inputs: TCP stream and stdin")
    for mode in ("tcp", "stdin"):
        m = FakeMotor()
        g = Goto(exe, m.port, 1.0, ["-i"] if mode == "stdin" else None, stdin_pipe=mode == "stdin")
        try:
            g.cmd("GOTO cup")
            line = json.dumps({"t": 0, "w": 640, "h": 480,
                               "dets": [{"label": "mug", "conf": 0.8, "box": [300, 300, 340, 330]}]}) + "\n"
            if mode == "tcp":
                s = socket.create_connection(("127.0.0.1", g.det_port))
            for _ in range(8):
                if mode == "tcp":
                    s.sendall(line.encode())
                else:
                    g.p.stdin.write(line.encode())
                    g.p.stdin.flush()
                time.sleep(0.1)
            st = g.status()
            check(st["state"] == "approach" and int(st["hits"]) >= 2, f"{mode}: 'mug' boxes drive a GOTO cup ({st['state']})")
            if mode == "tcp":
                s.close()
        finally:
            g.close()
            m.close()


def test_real_tars_motor(exe: str) -> None:
    tm = os.path.join(REPO, "iter1-wasd", "pi", "build", "tars_motor_host")
    if not os.path.exists(tm):
        print("real tars_motor: skipped (run make -C iter1-wasd/pi host)")
        return
    print("against the real tars_motor (host build, -b stub)")
    port = free_port()
    p = subprocess.Popen([tm, "-b", "stub", "-p", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        c = None
        for _ in range(100):
            try:
                c = socket.create_connection(("127.0.0.1", port), timeout=2)
                break
            except OSError:
                time.sleep(0.02)
        cf = c.makefile("rw", encoding="ascii", newline="\n")

        def tm_status() -> dict:
            cf.write("STATUS\n")
            cf.flush()
            parts = cf.readline().split()
            d = dict(kv.split("=", 1) for kv in parts[6:] if "=" in kv)
            d["l"], d["r"] = float(parts[1]), float(parts[2])
            return d

        g = Goto(exe, port, 1.0)
        det = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        def send(box) -> None:
            dets = [{"label": "trash can", "conf": 0.8, "box": box}] if box else []
            det.sendto(json.dumps({"t": 0, "w": 640, "h": 480, "dets": dets}).encode(), ("127.0.0.1", g.det_port))

        try:
            check(g.cmd("GOTO bin") == "OK goto trash can", "GOTO bin")
            for _ in range(6):
                send(None)
                time.sleep(0.1)
            st = tm_status()
            check(st["move_kind"] == "TURN" and st["move"] in ("running", "done"), f"empty frames -> tars_motor runs a TURN ({st['move_kind']} {st['move']})")
            wait_for(lambda: g.status()["state"] == "look", 3)
            time.sleep(0.4)
            for _ in range(10):
                send([300, 250, 360, 330])
                time.sleep(0.1)
            st = tm_status()
            check(g.status()["state"] == "approach" and st["l"] > 0.3 and st["r"] > 0.3 and st["why"] == "none",
                  f"centred box -> wheels driving forward ({st['l']:.2f} {st['r']:.2f}, estop why={st['why']})")
            for _ in range(3):
                send([280, 100, 380, 470])
                time.sleep(0.1)
            time.sleep(0.1)
            st = tm_status()
            gs = g.status()
            check(gs["result"] == "arrived" and st["l"] == 0.0 and st["r"] == 0.0, f"big box -> arrived, wheels 0 ({gs['result']})")
            check(int(st.get("stops", 0)) >= 1, "tars_motor counted the S (stops=)")
            gs = g.status()
            check(gs.get("pose", "-") != "-" and abs(float(gs["pose"].split(",")[2]) - 30.0) < 0.5,
                  f"goto follows tars_motor's dead-reckoned pose (after one TURN 30: {gs.get('pose')})")
            mem = g.cmd("MEMORY")
            check(mem.startswith("OK n=1 ") and "; trash can#1 fresh x=" in mem, f"MEMORY has the trash can ({mem[:70]}...)")
        finally:
            g.close()
    finally:
        p.terminate()
        p.wait(2)


class Watcher:
    """second control connection with WATCH: collects EVENT lines"""
    def __init__(self, port: int):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=2)
        self.s.settimeout(None)              # events can be minutes apart
        self.f = self.s.makefile("rw", encoding="ascii", newline="\n")
        self.f.write("WATCH\n")
        self.f.flush()
        self.events: list[str] = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        try:
            for line in self.f:
                if line.startswith("EVENT "):
                    self.events.append(line[6:].strip())
        except (OSError, ValueError):
            pass

    def close(self) -> None:
        try:
            self.s.close()
        except OSError:
            pass


def motor_do(port: int, line: str, scale: float) -> None:
    """a teleop-style exact move on the (fake) tars_motor; waits for it to finish"""
    c = socket.create_connection(("127.0.0.1", port))
    f = c.makefile("rw", encoding="ascii", newline="\n")
    f.write(line + "\n")
    f.flush()
    r = f.readline().split()
    end = time.monotonic() + int(r[2]) / 1000 / scale + 0.1
    while time.monotonic() < end:                # keep the move deadman fed, like teleop does
        f.write("PING\n")
        f.flush()
        f.readline()
        time.sleep(0.1 / scale)
    f.close()
    c.close()


def wait_idle(g: Goto, m: FakeMotor, timeout: float) -> dict:
    t0 = m.now()
    st = g.status()
    while st["state"] != "idle" and m.now() - t0 < timeout:
        time.sleep(0.05)
        st = g.status()
    return st


def memory_scene(exe: str, scale: float, seed: int, dist: float, bearing_deg: float, obj_h: float,
                 use_memory: bool, turn: float, move_mm: float) -> dict:
    """SCAN (if use_memory) at the start pose, then a teleop TURN + MOVE, then GOTO trash can (timed)."""
    rng = random.Random(seed)
    m = FakeMotor(scale, rng=rng, drive_err=0.02)
    b = math.radians(bearing_deg)
    objs = [Obj("trash can", dist * math.cos(b), dist * math.sin(b), 0.15, obj_h)]
    a = b + math.radians(rng.choice([-1, 1]) * rng.uniform(60, 150))
    objs.append(Obj("chair", 2.0 * math.cos(a), 2.0 * math.sin(a), 0.25, 0.9))
    g = Goto(exe, m.port, scale)
    cam = SimCamera(m, objs, g.det_port, scale, rng)
    try:
        if use_memory:
            g.cmd("SCAN")
            wait_idle(g, m, 120)
        motor_do(m.port, f"TURN {turn:.0f}", scale)
        motor_do(m.port, f"MOVE {move_mm:.0f}", scale)
        time.sleep(0.4 / scale)
        n0 = len(m.log)
        t0 = m.now()
        g.cmd("GOTO trash can")
        st = wait_idle(g, m, 260)
        turns = [l for _, l in m.log[n0:] if l.startswith("TURN")]
        x, y, _ = m.pose()
        o = objs[0]
        return {"result": st.get("result"), "t": m.now() - t0, "turns": len(turns), "mem": st.get("mem"),
                "dist": math.hypot(o.x - x, o.y - y) - o.r}
    finally:
        cam.close()
        g.close()
        m.close()


def test_memory(exe: str, scale: float) -> None:
    print(f"object memory: SCAN, MEMORY/FORGET, memory-guided GOTO, persistence (goto -T {scale})")
    rng = random.Random(21)
    m = FakeMotor(scale, rng=rng, drive_err=0.02)
    objs = [Obj("trash can", 2.0 * math.cos(math.radians(150)), 2.0 * math.sin(math.radians(150)), 0.15, 0.6),
            Obj("chair", 2.0 * math.cos(math.radians(-60)), 2.0 * math.sin(math.radians(-60)), 0.25, 0.9)]
    g = Goto(exe, m.port, scale)
    w = Watcher(g.ctl_port)
    cam = SimCamera(m, objs, g.det_port, scale, rng)
    mem_file = g.mem
    try:
        check(g.cmd("MEMORY").startswith("OK n=0 "), "MEMORY empty at first")
        n0 = len(m.log)
        check(g.cmd("SCAN") == "OK scan", "SCAN")
        st = wait_idle(g, m, 120)
        turns = [l for _, l in m.log[n0:] if l.startswith("TURN")]
        check(st["result"].startswith("scanned:") and len(turns) == 12 and all(l.startswith("TURN 30.0") for l in turns),
              f"SCAN = 12 x TURN 30 ({st['result']}, {len(turns)} turns)")
        time.sleep(0.1)
        ev = [e for e in w.events if e.startswith("scanned ")]
        check(bool(ev) and "trash can" in ev[-1] and "chair" in ev[-1], f"EVENT {ev[-1] if ev else '-'}")
        mem = g.cmd("MEMORY")
        check("; trash can#1 fresh " in mem and "; chair#1 fresh " in mem and mem.count("; ") == 2,
              f"MEMORY: one instance each ({mem[:110]}...)")
        tc = re.search(r"; trash can#1 fresh x=(-?\d+) y=(-?\d+) rel_bearing=(-?\d+) rel_dist=(\d+)", mem)
        # sim: 150 deg counter-clockwise = -150 in tars_motor's clockwise convention; 1.85 m to the can's face
        check(bool(tc) and abs(int(tc.group(3)) + 150) < 15 and 1200 < int(tc.group(4)) < 2600,
              f"trash can remembered at bearing {tc.group(3) if tc else '?'} (~-150), {tc.group(4) if tc else '?'} mm (~2000 to its centre)")
        time.sleep(3.0 / scale + 0.3)
        check(os.path.exists(mem_file) and "trash can" in open(mem_file).read(), "memory saved to the -M file")

        # teleop moves the robot, then GOTO: turns straight to it
        motor_do(m.port, "TURN 60", scale)
        motor_do(m.port, "MOVE 300", scale)
        n0 = len(m.log)
        g.cmd("GOTO bin")
        time.sleep(0.3 / scale + 0.05)
        first = next((l for _, l in m.log[n0:] if l.startswith("TURN")), "")
        st = wait_idle(g, m, 200)
        turns = [l for _, l in m.log[n0:] if l.startswith("TURN")]
        check(st["result"] == "arrived" and len(turns) <= 3,
              f"memory-guided GOTO after a teleop move: {st['result']}, {len(turns)} turns, first '{first}'")
        check(bool(first) and abs(float(first.split()[1])) > 150, f"first TURN goes straight to the remembered can ({first})")
        check(any(e.startswith("remembered trash can #1") for e in w.events), "EVENT remembered trash can #1")

        cam.paused = True
        time.sleep(0.3)
        check(g.cmd("FORGET the chair") == "OK forgot 1" and "; chair#" not in g.cmd("MEMORY"), "FORGET chair")
        w.close()
        g.close()                                # SIGTERM: saves on exit

        # restart goto with the same file, and tars_motor restarted too (pose 0, new epoch)
        m.restart_pose()
        g = Goto(exe, m.port, scale, mem=mem_file)
        cam.port = g.det_port
        mem = g.cmd("MEMORY")
        check(mem.startswith("OK n=1 ") and "; trash can#1 " in mem, f"memory survives a goto restart ({mem[:60]}...)")
        time.sleep(0.5)                          # first pose poll: re-base on the saved pose
        motor_do(m.port, "TURN 90", scale)       # then the robot turns away (goto sees it)
        cam.paused = False
        n0 = len(m.log)
        g.cmd("GOTO trash can")
        st = wait_idle(g, m, 200)
        turns = [l for _, l in m.log[n0:] if l.startswith("TURN")]
        first = turns[0] if turns else ""
        check(st["result"] == "arrived" and bool(first) and abs(abs(float(first.split()[1])) - 90) < 25,
              f"after a tars_motor restart (re-based on the saved pose): first TURN '{first}' (~90 back), {st['result']}")
        cam.paused = True
        time.sleep(0.3)
        r = g.cmd("FORGET")
        check(re.fullmatch(r"OK forgot [1-9]\d*", r) is not None and g.cmd("MEMORY").startswith("OK n=0 "), f"FORGET everything ({r})")
    finally:
        cam.close()
        g.close()
        m.close()


def run_memory_trials(exe: str, scale: float, n: int, seed: int, par: int) -> None:
    print(f"memory vs blind search: {n} random scenes; each: (SCAN), teleop TURN + MOVE 300, GOTO (timed)")
    rng = random.Random(seed + 77)
    jobs = [(seed * 1000 + 500 + i, rng.uniform(0.8, 3.0), rng.uniform(-180, 180), rng.choice([0.4, 0.6, 0.8]),
             rng.choice([-1, 1]) * rng.uniform(30, 180)) for i in range(n)]
    res: dict = {}

    def worker(idx: int, use: bool) -> None:
        s, d, b, h, turn = jobs[idx]
        res[(idx, use)] = memory_scene(exe, scale, s, d, b, h, use, turn, 300)

    tasks = [(i, u) for i in range(n) for u in (False, True)]
    for i in range(0, len(tasks), par):
        ts = [threading.Thread(target=worker, args=t) for t in tasks[i:i + par]]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
    out = {}
    for use in (False, True):
        rs = [res[(i, use)] for i in range(n)]
        ok = [r for r in rs if r["result"] == "arrived"]
        times = sorted(r["t"] for r in ok)
        turns = sorted(r["turns"] for r in ok)
        med = times[len(times) // 2] if times else 0
        out[use] = (len(ok), med, turns[len(turns) // 2] if turns else 0, times[-1] if times else 0)
        print(f"  {'memory' if use else 'blind '}: arrived {len(ok)}/{n}, time median {med:.0f} s (max {out[use][3]:.0f} s), "
              f"turns median {out[use][2]}")
    check(out[True][0] >= 0.9 * n, f"memory-guided GOTO arrives {out[True][0]}/{n} (>= 90%)")
    check(out[True][1] < out[False][1], f"memory-guided is faster: median {out[True][1]:.0f} s vs blind {out[False][1]:.0f} s")


def run_trials(exe: str, scale: float, n: int, seed: int, par: int) -> None:
    print(f"random start poses: {n} trials, goto -T {scale}")
    rng = random.Random(seed)
    jobs = [(seed * 1000 + i, rng.uniform(0.8, 3.0), rng.uniform(-180, 180), rng.choice([0.4, 0.6, 0.8])) for i in range(n)]
    results: list[dict] = [None] * n

    def worker(idx: int) -> None:
        s, d, b, h = jobs[idx]
        r = drive_scene(exe, scale, s, d, b, obj_h=h)
        r.pop("log")
        r.update(start_dist=d, start_bearing=b, obj_h=h)
        results[idx] = r

    for i in range(0, n, par):
        ts = [threading.Thread(target=worker, args=(k,)) for k in range(i, min(n, i + par))]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
    ok = [r for r in results if r["result"] == "arrived" and r["dist"] < 1.3 and abs(r["bearing"]) < 15 and r["min_d"] > 0]
    crashed = [r for r in results if r["min_d"] <= 0]
    times = sorted(r["t"] for r in ok)
    dists = sorted(r["dist"] for r in ok)
    print(f"  {'start d':>7} {'bearing':>7} {'obj h':>5}  result              time   stop dist  bearing")
    for r in results:
        print(f"  {r['start_dist']:7.2f} {r['start_bearing']:7.0f} {r['obj_h']:5.1f}  {r['result']:<18} {r['t']:6.0f}s  {r['dist']:7.2f} m  {r['bearing']:6.1f}")
    rate = len(ok) / n
    print(f"  success {len(ok)}/{n} = {rate:.0%}; time to arrive median {times[len(times) // 2] if times else 0:.0f} s "
          f"(max {times[-1] if times else 0:.0f} s); stop distance median {dists[len(dists) // 2] if dists else 0:.2f} m "
          f"(range {dists[0] if dists else 0:.2f}-{dists[-1] if dists else 0:.2f})")
    check(rate >= 0.9, f"success rate {rate:.0%} >= 90% (arrived, within 1.3 m, facing it within 15 deg)")
    check(not crashed, f"never drove into the object ({len(crashed)} did)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("exe")
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--scale", type=float, default=8.0)
    ap.add_argument("--par", type=int, default=6)
    ap.add_argument("--quick", action="store_true", help="skip the random trials")
    ap.add_argument("--mem-trials", type=int, default=16)
    a = ap.parse_args()
    exe = os.path.abspath(a.exe)
    os.makedirs(os.path.join(GOTO_DIR, "build"), exist_ok=True)
    t0 = time.monotonic()
    test_protocol(exe)
    test_mission_cmds(exe)
    test_inputs(exe)
    test_cancel_latency(exe)
    test_real_tars_motor(exe)
    test_scenarios(exe, a.scale)
    test_memory(exe, a.scale)
    if not a.quick:
        run_trials(exe, a.scale, a.trials, a.seed, a.par)
        run_memory_trials(exe, a.scale, a.mem_trials, a.seed, a.par)
    print(f"{passed}/{passed + failed} checks passed ({time.monotonic() - t0:.0f} s)")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
