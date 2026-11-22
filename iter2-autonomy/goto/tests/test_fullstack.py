#!/usr/bin/env python3
"""Scripted full-stack sim (Mac only): the real native processes, fake sensors.

  real  iter1-wasd/pi/build/tars_motor_host -b stub   motion + pose + guard gate (-g, GAT1 UDP)
  real  iter2-autonomy/goto/build/goto_host               control 5570-style port, dets 5571-style, nav 5572-style
  real  iter2-autonomy/voice/build/tars_voice_host        (voice scenarios: wav in, whisper, intent, routing)
  fake  lidar_guard   GAT1 to tars_motor: lidar_guard.cpp's zone (60 deg cone, <= 0.6 m ahead, |side| < 0.45 m)
  fake  phone         LDR1 packets (pose + raycast LiDAR, 60 deg FOV) to goto's nav port, 20 Hz
  fake  YOLO          detection JSON lines to goto's det port, 8 Hz, occlusion-aware
  fake  phone speech  TCP listener for voice's "SAY <text>" (5573-style)

Ground truth: the robot is wherever tars_motor's stub says it is (STATUS pose=, dead
reckoned from the exact moves it ran). World frame = tars_motor's (x forward, y right, m);
the fake phone reports it in the ARKit frame goto expects (x_ark = -y, z_ark = x, yaw = -hdg).
Sim speed-up: WHEELCAL 260 400 (4x the stepper's linear speed, same turn rate). Not physical.

Scenarios: obstacle (wall between robot and remembered bin), occlusion (something steps
between robot and bin at the spot -> OCCLUDED -> DETOUR), occlusion_close (the same, 0.39 m in
front of the robot: inside its footprint and the guard zone -> pivot away + forward, DETOUR), moved object (memory says A, bin is
at B), STOP mid-path (S from another client, like voice), visual-only fallback (no phone),
voice memory query (MEMORY SAY -> print + BEEP + SAY to the phone) and voice "go back to
where the bin was" (RECALL). Ports: every process gets free ephemeral ports so runs don't clash;
the defaults (5570/5571/5572/5573) are checked by grep separately.

usage: python3 iter2-autonomy/goto/tests/test_fullstack.py [--only NAME] [-v]
"""
from __future__ import annotations

import math
import os
import re
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
import wave

HERE = os.path.dirname(os.path.abspath(__file__))
GOTO_DIR = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(GOTO_DIR))
MOTOR = os.path.join(REPO, "iter1-wasd", "pi", "build", "tars_motor_host")
GOTO = os.path.join(GOTO_DIR, "build", "goto_host")
VOICE = os.path.join(REPO, "iter2-autonomy", "voice", "build", "tars_voice_host")
VMODEL = os.path.join(REPO, "iter2-autonomy", "voice", "models", "ggml-tiny.en-q5_1.bin")
BUILD = os.path.join(GOTO_DIR, "build")

# fake lidar_guard = lidar_guard.cpp parse(): a LiDAR point with |side| < 0.45 m and
# 0 < forward <= 0.6 m (robot = phone pose) is BLOCKED. The phone only sees its 60 deg cone,
# so at 0.6 m the zone is +-0.35 m wide (tan 30 deg x 0.6); the 0.45 m half-width matters
# only further out. (This used to be 3 parallel rays at +-0.18 m to 0.55 m: much narrower
# than the real guard, so detours that pass 0.2-0.35 m from a box looked fine here.)
GUARD_M = float(os.environ.get("FULLSTACK_GUARD_STOP", "0.6"))        # lidar_guard --stop-distance
GUARD_HALF_W = float(os.environ.get("FULLSTACK_GUARD_HALF_W", "0.45"))  # lidar_guard --half-width
# FULLSTACK_CONF_EXTRA="car_length_m=0.20;car_width_m=0.175;guard_stop_m=0.30": appended to goto.conf
# for this run (later keys win). Set FULLSTACK_GUARD_* to what `goto --guard-flags` prints for it.
CONF_EXTRA = [kv.strip() for kv in os.environ.get("FULLSTACK_CONF_EXTRA", "").split(";") if kv.strip()]
GUARD_HFOV = math.radians(60)
GUARD_RAYS = 31           # every 2 deg across the cone
CAN_R, CAN_H = 0.15, 0.40
FX = 320 / math.tan(math.radians(30))   # 640 px, 60 deg hfov, square pixels
CAM_H = 0.12

passed = failed = 0
LOCK = threading.Lock()
VERBOSE = "-v" in sys.argv


def check(ok: bool, what: str) -> None:
    global passed, failed
    with LOCK:
        if ok:
            passed += 1
            print(f"  ok   {what}")
        else:
            failed += 1
            print(f"  FAIL {what}")


_handed_out: set[int] = set()


def free_port() -> int:
    # The scenarios run in parallel, so never hand the same number to two stacks.
    # (A child binds it a moment later; that gap is still a race with other sockets,
    # which is why a scenario that dies in setup must count as a FAIL, see main().)
    with LOCK:
        while True:
            s = socket.socket()
            s.bind(("127.0.0.1", 0))
            p = s.getsockname()[1]
            s.close()
            if p not in _handed_out:
                _handed_out.add(p)
                return p


# ------------------------------------------------------------------ world
def seg_hit(px, py, dx, dy, a, b):
    """ray p + t d against segment a-b -> t or None"""
    ex, ey = b[0] - a[0], b[1] - a[1]
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((a[0] - px) * ey - (a[1] - py) * ex) / den
    u = ((a[0] - px) * dy - (a[1] - py) * dx) / den
    return t if t > 1e-6 and 0 <= u <= 1 else None


def box(cx, cy, hx, hy):
    a, b, c, d = (cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)
    return [(a, b), (b, c), (c, d), (d, a)]


class World:
    def __init__(self, size=4.0, x0=-1.0, y0=-2.0):
        self.walls: list = []
        x1, y1 = x0 + size, y0 + size
        self.walls += [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]
        self.cans: list[list[float]] = []      # [x, y]
        self.glass: list = []                  # walls only the guard sees (not the phone map, not YOLO)
        self.lock = threading.Lock()

    def segments(self, with_cans=True, glass=False):
        segs = list(self.walls) + (list(self.glass) if glass else [])
        if with_cans:
            for c in self.cans:
                segs += box(c[0], c[1], CAN_R, CAN_R)
        return segs

    def ray(self, px, py, ang, maxd=4.0, with_cans=True, glass=False):
        dx, dy = math.cos(ang), math.sin(ang)
        best = maxd
        hit = False
        with self.lock:
            segs = self.segments(with_cans, glass)
        for a, b in segs:
            t = seg_hit(px, py, dx, dy, a, b)
            if t is not None and t < best:
                best, hit = t, True
        return best, hit

    def blocked_between(self, p, q):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        ang = math.atan2(q[1] - p[1], q[0] - p[0])
        t, hit = self.ray(p[0], p[1], ang, d - CAN_R - 0.02, with_cans=False)
        return hit

    def min_dist_walls(self, p):
        best = 1e9
        for a, b in self.walls:
            ex, ey = b[0] - a[0], b[1] - a[1]
            L = ex * ex + ey * ey
            u = 0 if L == 0 else max(0, min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / L))
            best = min(best, math.hypot(p[0] - a[0] - u * ex, p[1] - a[1] - u * ey))
        return best


# ------------------------------------------------------------------ tars_motor
class Motor:
    def __init__(self, guard_port: int):
        self.port = free_port()
        self.log: list[str] = []
        self.p = subprocess.Popen([MOTOR, "-b", "stub", "-p", str(self.port), "-r", "0", "-g", str(guard_port)],
                                  stderr=subprocess.PIPE, text=True)
        threading.Thread(target=self._read, daemon=True).start()
        for _ in range(100):
            try:
                self.s = socket.create_connection(("127.0.0.1", self.port), timeout=2)
                break
            except OSError:
                time.sleep(0.02)
        else:
            raise RuntimeError(f"tars_motor never accepted on {self.port} (exit={self.p.poll()}, stderr={self.log[-3:]})")
        self.f = self.s.makefile("rw", encoding="ascii", newline="\n")
        self.lock = threading.Lock()
        self.cmd("WHO sim")
        self.cmd("WHEELCAL 260 400")

    def _read(self):
        for line in self.p.stderr:
            self.log.append(line.rstrip())

    def cmd(self, line: str) -> str:
        with self.lock:
            self.f.write(line + "\n")
            self.f.flush()
            return self.f.readline().strip()

    def pose(self):
        r = self.cmd("STATUS")
        m = re.search(r" pose=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)", r)
        mv = re.search(r" move=(\w+)", r)
        x, y, h = (float(v) for v in m.groups())
        return x / 1000, y / 1000, math.radians(h), mv.group(1) if mv else "-", r

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass
        self.p.terminate()
        try:
            self.p.wait(3)
        except subprocess.TimeoutExpired:
            self.p.kill()
        # keep tars_motor's log on disk next to goto's (build/fs_goto_<port>.log), for post-mortems
        with open(os.path.join(BUILD, f"fs_motor_{self.port}.log"), "w") as f:
            f.write("\n".join(self.log) + "\n")


# ------------------------------------------------------------------ fake sensors
class Sensors:
    """fake lidar_guard + phone (LDR1) + YOLO, all driven from tars_motor's pose"""

    def __init__(self, world: World, motor: Motor, guard_port: int, nav_port: int | None, det_port: int):
        self.w, self.m = world, motor
        self.guard_port, self.nav_port, self.det_port = guard_port, nav_port, det_port
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.run = True
        self.pose = (0.0, 0.0, 0.0)
        self.trace: list[tuple[float, float]] = []
        self.guard_denials = 0
        self.guard_gaps: list[float] = []      # seconds between GAT1 sends (tars_motor's TTL is 100 ms)
        self.seq = 0
        self.t0 = time.monotonic()
        self.phone_on = nav_port is not None
        import random
        self.rng = random.Random(7)
        self.navsock = None
        self.threads = [threading.Thread(target=f, daemon=True) for f in (self._pose_loop, self._guard_loop,
                                                                           self._phone_loop, self._yolo_loop)]
        # a separate motor connection for polling the pose (truth)
        self.mc = socket.create_connection(("127.0.0.1", motor.port))
        self.mf = self.mc.makefile("rw", encoding="ascii", newline="\n")
        self.mf.write("WHO truth\n"); self.mf.flush(); self.mf.readline()
        for t in self.threads:
            t.start()

    def _pose_loop(self):
        while self.run:
            try:
                self.mf.write("STATUS\n"); self.mf.flush()
                r = self.mf.readline()
            except (OSError, ValueError):
                return
            m = re.search(r" pose=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)", r)
            if m:
                x, y, h = (float(v) for v in m.groups())
                self.pose = (x / 1000, y / 1000, math.radians(h))
                self.trace.append((x / 1000, y / 1000))
            time.sleep(0.02)

    def _guard_loop(self):
        last = None
        while self.run:
            t = time.monotonic()
            if last is not None:
                self.guard_gaps.append(t - last)
            last = t
            x, y, h = self.pose
            blocked = False
            for k in range(GUARD_RAYS):
                a = -GUARD_HFOV / 2 + GUARD_HFOV * k / (GUARD_RAYS - 1)
                t, hit = self.w.ray(x, y, h + a, GUARD_M / math.cos(a) + 1e-6, glass=True)
                if hit and t * math.cos(a) <= GUARD_M and abs(t * math.sin(a)) < GUARD_HALF_W:
                    blocked = True
                    break
            permit = 0 if blocked else 1
            if blocked:
                self.guard_denials += 1
            us = int(time.clock_gettime(time.CLOCK_MONOTONIC) * 1e6)
            self.udp.sendto(b"GAT1" + us.to_bytes(8, "big") + bytes([permit]), ("127.0.0.1", self.guard_port))
            time.sleep(0.02)

    def _phone_loop(self):
        while self.run:
            if not self.phone_on or self.nav_port is None:
                time.sleep(0.05)
                continue
            x, y, h = self.pose
            pts = []
            # a real LiDAR depth map is dense; here 25 rays over the 60 deg FOV with a random
            # sub-step offset per packet, floor samples every 0.08 m to 3.5 m: fills in over time
            jit = self.rng.uniform(-1.25, 1.25)
            for k in range(-12, 13):
                ang = h + math.radians(2.5 * k + jit)
                t, hit = self.w.ray(x, y, ang, 3.5)
                s = 0.3 + self.rng.uniform(0, 0.08)
                while s < t - 0.05 and len(pts) < 1000:
                    pts.append((x + s * math.cos(ang), 0.0, y + s * math.sin(ang)))
                    s += 0.08
                if hit and len(pts) < 1023:
                    pts.append((x + t * math.cos(ang), 0.3, y + t * math.sin(ang)))
            # world (x fwd, y right) -> ARKit (x = -y, z = x, y up)
            ark = [(-py, hgt, px) for px, hgt, py in pts]
            self.seq += 1
            uptime = int((time.monotonic() - self.t0) * 1e6) + 1
            body = b"LDR1" + struct.pack(">IQII", self.seq, uptime, 7, len(ark))
            body += struct.pack(">4f", -y, 0.25, x, -h) + struct.pack(">f", 0.0)
            body += b"".join(struct.pack(">3f", *p) for p in ark)
            # TCP, length-prefixed, like the phone's LDR1 link (a 12 KB packet won't fit a macOS UDP datagram)
            try:
                if self.navsock is None:
                    self.navsock = socket.create_connection(("127.0.0.1", self.nav_port), timeout=1)
                self.navsock.sendall(struct.pack(">I", len(body)) + body)
            except OSError:
                self.navsock = None
            time.sleep(0.05)

    def _yolo_loop(self):
        while self.run:
            x, y, h = self.pose
            dets = []
            with self.w.lock:
                cans = [tuple(c) for c in self.w.cans]
            for cx, cy in cans:
                dx, dy = cx - x, cy - y
                f = dx * math.cos(h) + dy * math.sin(h)
                r = -dx * math.sin(h) + dy * math.cos(h)
                if f < 0.15:
                    continue
                b = math.atan2(r, f)
                if abs(b) > math.radians(29) or self.w.blocked_between((x, y), (cx, cy)):
                    continue
                d = math.hypot(dx, dy)
                u = 320 + FX * math.tan(b)
                half = FX * CAN_R / d
                top = max(0.0, 240 - FX * (CAN_H - CAM_H) / d)
                bot = min(480.0, 240 + FX * CAM_H / d)
                dets.append({"label": "trash can", "conf": 0.8,
                             "box": [max(0, u - half), top, min(640, u + half), bot]})
            line = '{"t":0,"w":640,"h":480,"dets":[' + ",".join(
                '{"label":"%s","conf":%.2f,"box":[%.1f,%.1f,%.1f,%.1f]}' % (d["label"], d["conf"], *d["box"]) for d in dets) + "]}"
            self.udp.sendto(line.encode(), ("127.0.0.1", self.det_port))
            time.sleep(0.125)

    def close(self):
        self.run = False
        time.sleep(0.15)
        try:
            self.mc.close()
        except OSError:
            pass


# ------------------------------------------------------------------ goto
class Goto:
    def __init__(self, motor_port: int, mem_text: str | None = None, nav: bool = True):
        self.ctl, self.det = free_port(), free_port()
        self.nav = free_port() if nav else None
        self.mem = os.path.join(BUILD, f"fs_mem_{self.ctl}.txt")
        if mem_text is not None:
            with open(self.mem, "w") as f:
                f.write(mem_text)
        elif os.path.exists(self.mem):
            os.remove(self.mem)
        self.logp = os.path.join(BUILD, f"fs_goto_{self.ctl}.log")
        self.logf = open(self.logp, "w")
        conf = os.path.join(GOTO_DIR, "goto.conf")
        if CONF_EXTRA:
            conf = os.path.join(BUILD, f"fs_conf_{self.ctl}.conf")
            with open(os.path.join(GOTO_DIR, "goto.conf")) as src, open(conf, "w") as dst:
                dst.write(src.read() + "\n" + "\n".join(CONF_EXTRA) + "\n")
        args = [GOTO, "-m", f"127.0.0.1:{motor_port}", "-p", str(self.ctl), "-u", str(self.det),
                "-n", str(self.nav if nav else 0), "-r", "0", "-M", self.mem, "-v",
                "-o", os.path.join(GOTO_DIR, "objects.txt"), "-c", conf]
        self.p = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=self.logf)
        for _ in range(100):
            try:
                self.s = socket.create_connection(("127.0.0.1", self.ctl), timeout=3)
                break
            except OSError:
                time.sleep(0.02)
        else:
            raise RuntimeError(f"goto never accepted on {self.ctl} (exit={self.p.poll()}, log {self.logp})")
        self.f = self.s.makefile("rw", encoding="ascii", newline="\n")
        self.events: list[str] = []
        # a WATCH connection collects EVENT lines
        self.ws = socket.create_connection(("127.0.0.1", self.ctl))
        self.wf = self.ws.makefile("rw", encoding="ascii", newline="\n")
        self.wf.write("WATCH\n"); self.wf.flush(); self.wf.readline()
        threading.Thread(target=self._watch, daemon=True).start()
        self.lock = threading.Lock()

    def _watch(self):
        try:
            for line in self.wf:
                if line.startswith("EVENT "):
                    self.events.append(line[6:].strip())
        except (OSError, ValueError):
            pass

    def cmd(self, line: str) -> str:
        with self.lock:
            self.f.write(line + "\n"); self.f.flush()
            while True:
                r = self.f.readline().strip()
                if not r.startswith("EVENT"):
                    return r

    def status(self) -> dict:
        r = self.cmd("STATUS")
        return dict(kv.split("=", 1) for kv in r.split()[1:] if "=" in kv)

    def wait_idle(self, timeout: float, hook=None) -> dict:
        end = time.monotonic() + timeout
        st = self.status()
        while time.monotonic() < end:
            st = self.status()
            if hook:
                hook(st)
            if st["state"] == "idle":
                return st
            time.sleep(0.1)
        return st

    def close(self):
        for s in (self.s, self.ws):
            try:
                s.close()
            except OSError:
                pass
        self.p.terminate()
        try:
            self.p.wait(3)
        except subprocess.TimeoutExpired:
            self.p.kill()
        self.logf.close()


def mem_file(entries) -> str:
    """remembered bins at world (x, y) m -> memory.txt (motor frame mm + navw ARKit metres)"""
    now = time.time()
    out = ["v 2"]
    for i, (x, y) in enumerate(entries, 1):
        out.append(f"inst {i} fresh {x * 1000:.1f} {y * 1000:.1f} 0.00 1000 0.800 3 {now:.1f} trash can")
        out.append(f"navw {i} {-y:.3f} {x:.3f} trash can")
    return "\n".join(out) + "\n"


class Stack:
    def __init__(self, world: World, mem_text: str | None, nav: bool = True):
        self.guard_port = free_port()
        self.world = world
        self.motor = Motor(self.guard_port)
        self.goto = Goto(self.motor.port, mem_text, nav)
        self.sens = Sensors(world, self.motor, self.guard_port, self.goto.nav, self.goto.det)
        time.sleep(1.0)                          # guard CLEAR, phone map, first pose polls
        # ... and tars_motor has actually applied a CLEAR from the fake guard. On a loaded Mac
        # its control loop could still be on the initial guard_reason=disabled after 1 s, and the
        # first TURN was refused ("ERR disabled" -> failed:disabled before anything happened).
        end = time.monotonic() + 10
        while "guard_reason=clear" not in self.motor.cmd("STATUS") and time.monotonic() < end:
            time.sleep(0.05)

    def close(self):
        self.sens.close()
        self.goto.close()
        self.motor.close()

    def min_wall_clearance(self):
        return min((self.world.min_dist_walls(p) for p in self.sens.trace), default=9)

    def dump(self, name):
        if VERBOSE or True:
            print(f"    [{name}] events: {' | '.join(self.goto.events)[:900]}")
            st = self.goto.status()
            print(f"    [{name}] guard_refusals={st.get('guard_refusals')} guard_aborts={st.get('guard_aborts')} "
                  f"guard_stops_other={st.get('guard_stops_other')} backtracks={st.get('backtracks')} replans={st.get('replans')} "
                  f"replans_here={st.get('replans_here')} elapsed={st.get('elapsed')}")
            tr = list(self.sens.trace)
            drove = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(tr, tr[1:]))
            nbm = sum(e.startswith("blocked:map") for e in self.goto.events)
            print(f"    [{name}] drove {drove:.2f} m (pose trace), blocked:map {nbm}, "
                  f"heading_fixes={st.get('heading_fixes', '-')} heading_unchecked={st.get('heading_unchecked', '-')}")
            g = self.sens.guard_gaps
            if g:
                print(f"    [{name}] fake guard: max gap {1000 * max(g):.0f} ms, {sum(x >= 0.1 for x in g)} gaps >= 100 ms (tars_motor TTL)")


def check_no_refusals(st: "Stack", s: dict, name: str) -> None:
    """The map knows every obstacle in this room, so the planner must never pick a leg the guard
    then stops: zero refusals (ERR when asked) and zero aborts (stopped while running) on path legs
    with a fresh guard_blocked. A refusal is only allowed for something the map doesn't hold."""
    r, a = int(s.get("guard_refusals", -1)), int(s.get("guard_aborts", -1))
    check(r == 0 and a == 0, f"{name}: known map, no guard refusals on path legs (refused {r}, aborted {a})")


# ------------------------------------------------------------------ scenarios
def sc_obstacle():
    print("full-stack: OBSTACLE: wall between the robot and the remembered bin")
    w = World()
    w.walls.append(((1.3, -0.6), (1.3, 0.6)))
    w.cans.append([2.4, 0.0])
    st = Stack(w, mem_file([(2.4, 0.0)]))
    try:
        r = st.goto.cmd("GOTO trash can")
        check(r.startswith("OK goto"), f"GOTO accepted ({r})")
        s = st.goto.wait_idle(150)
        st.dump("obstacle")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.4, y)
        ev = st.goto.events
        check(s["result"] == "arrived", f"arrived ({s['result']})")
        check(any(e.startswith("route trash can #1") for e in ev), "routed over the map (EVENT route)")
        check(d < 0.95, f"stopped next to the bin ({d:.2f} m centre to centre)")
        check(st.min_wall_clearance() > 0.2, f"never closer than 0.2 m to a wall ({st.min_wall_clearance():.2f} m)")
        check(not any(p[0] > 1.15 and p[0] < 1.45 and abs(p[1]) < 0.6 for p in st.sens.trace), "never drove through the wall")
        check_no_refusals(st, s, "obstacle")
    finally:
        st.close()


def sc_occlusion(ahead: float | None = None):
    """ahead=None: the board lands 0.36 m in front of the bin, wherever the robot stopped.
    ahead=k: the board lands k m in front of the ROBOT (occlusion_close: 0.39 m, inside its own
    footprint square and lidar_guard's 0.55 m stop zone; that used to end in not_found)."""
    tag = "occlusion" if ahead is None else "occlusion_close"
    print(f"full-stack: {tag.upper()}: something steps between the robot and the bin at the spot"
          + ("" if ahead is None else f", {ahead:.2f} m in front of the robot"))
    w = World()
    w.cans.append([2.2, 0.0])
    st = Stack(w, mem_file([(2.2, 0.0)]))
    placed = []

    def hook(s):
        # at the spot, facing it, before YOLO confirms: drop a board in between
        i, n = (int(v) for v in s.get("path", "0/0").split("/"))
        if VERBOSE:
            print("    hook", s["state"], s.get("map"), s.get("path"), [round(v, 2) for v in st.sens.pose])
        routed = any(e.startswith("route") for e in st.goto.events)
        if not placed and routed and s.get("map") == "route" and n > 0 and i >= n and s["state"] in ("follow_path", "look"):
            x, y, h = st.sens.pose
            d = math.hypot(2.2 - x, -y)
            if d < 1.3:
                ux, uy = (2.2 - x) / d, (0.0 - y) / d
                k = d - 0.36 if ahead is None else ahead    # 0.36 m in front of the bin / k ahead of us
                cx, cy = x + ux * k, y + uy * k
                with w.lock:
                    w.walls.append(((cx - uy * 0.3, cy + ux * 0.3), (cx + uy * 0.3, cy - ux * 0.3)))
                placed.append((cx, cy, d))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(200, hook)
        st.dump(tag)
        ev = st.goto.events
        check(bool(placed), f"occluder placed at the spot ({placed[0][2]:.2f} m from the bin)" if placed else "occluder placed")
        if ahead is not None:
            check(bool(placed) and placed[0][2] - ahead > 0.36,
                  f"the board is {ahead:.2f} m ahead, inside the guard zone ({GUARD_M} m), the bin behind it")
        check(any(e.startswith("occluded trash can #1") for e in ev), "LOOK fails + map occupied -> OCCLUDED")
        check(not any(e.startswith("not_there trash can #1") for e in ev[:ev.index(next((e for e in ev if e.startswith('occluded')), ev[-1])) + 1]),
              "occluded is not 'gone' (no not_there before it)")
        check(any(e.startswith("detour trash can #1") for e in ev), "-> DETOUR to a vantage cell")
        x, y, _ = st.sens.pose
        dk = math.hypot(x - 2.2, y)
        # docked at the BIN: a guard stop on the board's edge must not pass as "arrived"
        # (guard zone 0.55 m + bin radius 0.15 m: a real dock is <= ~0.75 m centre to centre)
        check(s["result"] == "arrived" and dk < 0.8, f"arrived after the detour, at the bin ({s['result']}, {dk:.2f} m)")
    finally:
        st.close()


def sc_dropbox():
    """bench demo_detour: a 0.35 x 0.9 m box lands across the route 0.95 m ahead of the robot
    while it drives to a bin 3.2 m down a wide hall. The guard stops it about 0.6 m from the box
    with the box filling its cone; goto has to get round it, and every forward MOVE that would
    keep the box in the guard's zone is refused by tars_motor."""
    print("full-stack: DROPBOX: a box dropped across the route mid-GOTO (bench demo_detour)")
    w = World(size=5.6, x0=-1.0, y0=-2.8)
    w.cans.append([3.2, 0.0])
    st = Stack(w, mem_file([(3.2, 0.0)]))
    placed = []

    def dropper():
        # tars_motor's stub backend only updates pose= when an exact MOVE is DONE (it jumps a
        # metre at the end), so a box dropped mid-MOVE is never seen by the fake guard/phone
        # in time. Recreate where the bench's guard stop leaves the robot instead: the moment
        # goto has its first route, put the box across the line to the bin with its face 0.5 m
        # ahead (inside the guard's 0.6 m zone, filling the cone). Every straight-ish MOVE from
        # here is refused; goto must turn clear of the guard's zone first.
        end = time.monotonic() + 60
        while not placed and time.monotonic() < end:
            if any(e.startswith("route trash can") for e in st.goto.events):
                x, y, _ = st.sens.pose
                a = math.atan2(0.0 - y, 3.2 - x)
                cx, cy = x + 0.675 * math.cos(a), y + 0.675 * math.sin(a)
                with w.lock:
                    w.walls.extend(box(cx, cy, 0.175, 0.45))
                placed.append((cx, cy, x))
            time.sleep(0.01)
    try:
        st.goto.cmd("GOTO trash can")
        threading.Thread(target=dropper, daemon=True).start()
        s = st.goto.wait_idle(200)
        st.dump("dropbox")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 3.2, y)
        check(bool(placed), f"box dropped on the route at ({placed[0][0]:.2f}, {placed[0][1]:.2f}), robot at x={placed[0][2]:.2f}"
              if placed else "box dropped")
        inside = placed and any(abs(p[0] - placed[0][0]) < 0.175 + 0.1 and abs(p[1] - placed[0][1]) < 0.45 + 0.1
                                for p in st.sens.trace)
        check(not inside, "never drove into the box (centre stays > 0.1 m outside it)")
        if GUARD_M >= 0.5:                      # the box face lands 0.5 m ahead: inside a 0.6 m guard only
            check(st.sens.guard_denials > 0, f"the guard saw the box ({st.sens.guard_denials} denials)")
        check(s["result"] == "arrived" and d < 0.95,
              f"got round the box and docked at the bin ({s['result']}, {d:.2f} m, replans={s.get('replans')})")
    finally:
        st.close()


def sc_wallmem():
    """user's bench case (open_bin room): the bin at (2.0, 0.3) is remembered; a 0.3 x 1.2 m wall
    is then put between the robot and the bin (x 1.25-1.55, y -0.2..1.0), so the camera can't see
    the bin from the start. GOTO must route round the wall to the remembered spot, look, dock."""
    print("full-stack: WALLMEM: remembered bin, a wall put in between, GOTO routes round it")
    w = World(size=5.0, x0=-1.0, y0=-2.0)
    w.cans.append([2.0, 0.3])
    w.walls.extend(box(1.4, 0.4, 0.15, 0.6))
    st = Stack(w, mem_file([(2.0, 0.3)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(240)
        st.dump("wallmem")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.0, y - 0.3)
        nav = st.goto.cmd("NAV")
        check(any(e.startswith("route trash can") for e in st.goto.events), f"routed over the map ({nav[:60]})")
        inside = any(1.25 - 0.05 < p[0] < 1.55 + 0.05 and -0.2 - 0.05 < p[1] < 1.0 + 0.05 for p in st.sens.trace)
        check(not inside, "never drove through the wall")
        check(s["result"] == "arrived" and d < 0.95, f"routed round the wall and docked at the bin ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "wallmem")
    finally:
        st.close()


def sc_wallmem_search():
    """the same wall, but memory is stale (older than memory_max_age_s), so GOTO searches first:
    a full search sees nothing (the wall hides the bin), then it must fall back to the remembered
    spot by route over the map, not fail not_found"""
    print("full-stack: WALLMEM_SEARCH: stale memory, wall in between: search fails, then route to memory")
    w = World(size=5.0, x0=-1.0, y0=-2.0)
    w.cans.append([2.0, 0.3])
    w.walls.extend(box(1.4, 0.4, 0.15, 0.6))
    old = mem_file([(2.0, 0.3)])
    now = time.time()
    old = re.sub(r"(inst 1 fresh .* 3 )([\d.]+)( trash can)", lambda m: f"{m.group(1)}{now - 30 * 86400:.1f}{m.group(3)}", old)
    st = Stack(w, old)
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("wallmem_search")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.0, y - 0.3)
        ev = st.goto.events
        check(any(e.startswith("route trash can") for e in ev), "routed over the map to the remembered spot")
        inside = any(1.25 - 0.05 < p[0] < 1.55 + 0.05 and -0.2 - 0.05 < p[1] < 1.0 + 0.05 for p in st.sens.trace)
        check(not inside, "never drove through the wall")
        check(s["result"] == "arrived" and d < 0.95, f"arrived at the bin ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "wallmem_search")
    finally:
        st.close()


def rot_box(cx, cy, hl, hw, ang):
    """a thin rotated rectangle (wall) as 4 segments: half length hl along ang, half width hw"""
    ux, uy, vx, vy = math.cos(ang), math.sin(ang), -math.sin(ang), math.cos(ang)
    p = [(cx + sl * hl * ux + sw * hw * vx, cy + sl * hl * uy + sw * hw * vy) for sl, sw in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return [(p[i], p[(i + 1) % 4]) for i in range(4)]


def sc_diagwall():
    """user's live demo (bench open_bin + a wall drawn diagonally between the start and the bin):
    the remembered bin sits ~0.5 m behind a 1.4 m diagonal wall. Main's goto reveal-spun, then
    called no_route for every remembered instance while the room was open round both wall ends."""
    print("full-stack: DIAGWALL: remembered bin behind a diagonal wall (bench open_bin + drawn wall)")
    w = World(size=5.0, x0=-1.0, y0=-2.0)
    w.cans.append([2.0, 0.3])
    w.walls.extend(rot_box(1.45, 0.2, 0.7, 0.05, math.radians(65)))
    st = Stack(w, mem_file([(2.0, 0.3)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("diagwall")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.0, y - 0.3)
        check(not any(e.startswith("no_route") for e in st.goto.events), "no no_route while the room is open round the wall")
        check(s["result"] == "arrived" and d < 0.95, f"routed round the diagonal wall and docked ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "diagwall")
    finally:
        st.close()


def sc_shortwall():
    """user's second demo (bench open_bin 10 x 8 m): a short wall 0.4 m in front of the start,
    the remembered bin 2 m behind it, open floor all round. Ended failed:blocked (replans 7)."""
    print("full-stack: SHORTWALL: a short wall 0.4 m in front of the robot, remembered bin behind it")
    w = World(size=7.0, x0=-2.0, y0=-3.5)
    w.cans.append([2.4, 0.0])
    w.walls.extend(box(0.45, 0.0, 0.05, 0.5))          # face 0.4 m ahead, 1.0 m wide
    st = Stack(w, mem_file([(2.4, 0.0)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("shortwall")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.4, y)
        inside = any(0.35 < p[0] < 0.55 and abs(p[1]) < 0.55 for p in st.sens.trace)
        check(not inside, "never drove through the wall")
        check(s["result"] == "arrived" and d < 0.95, f"got round the short wall and docked ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "shortwall")
    finally:
        st.close()


def sc_offsetwall():
    """user's third demo: a short wall ~0.5 m ahead, off to one side, open floor everywhere else.
    The guard refused the first 130-160 mm MOVE 7 times; every re-plan gave the same route."""
    print("full-stack: OFFSETWALL: a short wall 0.5 m ahead, off to one side, remembered bin behind it")
    w = World(size=7.0, x0=-2.0, y0=-3.5)
    w.cans.append([2.4, 0.3])
    w.walls.extend(box(0.55, 0.35, 0.05, 0.4))         # face 0.5 m ahead, y -0.05 .. 0.75
    st = Stack(w, mem_file([(2.4, 0.3)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("offsetwall")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.4, y - 0.3)
        inside = any(0.45 < p[0] < 0.65 and -0.1 < p[1] < 0.8 for p in st.sens.trace)
        check(not inside, "never drove through the wall")
        check(s["result"] == "arrived" and d < 0.95, f"got round the offset wall and docked ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "offsetwall")
    finally:
        st.close()


def sc_openbin():
    """bench open_bin (10 x 8 m, start at the origin, bin at (2.0, 0.3), water bottle at (1.2, -1.2))"""
    print("full-stack: OPENBIN: the bench's open_bin room, remembered bin")
    w = World(size=10.0, x0=-3.0, y0=-5.0)
    w.cans.append([2.0, 0.3])
    w.walls.extend(box(1.2, -1.2, 0.04, 0.04))          # the water bottle
    st = Stack(w, mem_file([(2.0, 0.3)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(200)
        st.dump("openbin")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.0, y - 0.3)
        check(s["result"] == "arrived" and d < 0.95, f"open_bin: docked at the bin ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "openbin")
    finally:
        st.close()


def sc_lpocket():
    """user's 4th demo (bench open_bin, kothar/bench f64db61): the bin sits in a pocket behind an
    L-shaped wall, the memory spot is right. Main's goto: detour, a guard stop on a detour leg went
    straight to next_candidate, then a 360 search from the wrong side of the wall: failed:not_found.
    Must: route round the L's open end into the pocket, zero guard refusals on the mapped walls,
    never a not_there for the hidden spot, arrived."""
    print("full-stack: LPOCKET: remembered bin in a pocket behind an L-shaped wall")
    w = World(size=7.0, x0=-2.0, y0=-3.5)
    w.cans.append([2.6, 0.1])
    w.walls.extend(box(1.8, 0.0, 0.05, 0.8))           # the L's long bar, between the robot and the bin
    w.walls.extend(box(2.55, 0.85, 0.8, 0.05))         # the L's foot along the pocket's far side
    st = Stack(w, mem_file([(2.6, 0.1)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("lpocket")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.6, y - 0.1)
        ev = st.goto.events
        inside = any((1.7 < p[0] < 1.9 and -0.85 < p[1] < 0.85) or (1.7 < p[0] < 3.4 and 0.75 < p[1] < 0.95)
                     for p in st.sens.trace)
        check(not inside, "never drove through the L")
        check(not any(e.startswith("not_there") for e in ev), "no not_there for a spot the wall hides")
        check(s["result"] == "arrived" and d < 0.95, f"routed into the pocket and docked ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "lpocket")
    finally:
        st.close()


def wall_moved_world(hidden: bool):
    w = World(size=7.0, x0=-2.0, y0=-3.5)
    w.walls.extend(box(1.3, 0.0, 0.05, 0.6))           # wall between the start and spot A
    if hidden:
        w.cans.append([2.6, 2.3])                       # B, behind a second wall
        w.walls.extend(box(2.6, 1.5, 1.0, 0.05))        # x 1.6..3.6 at y 1.5: hides B from A's side
    else:
        w.cans.append([3.0, 1.3])                       # B, in view from A
    return w


def sc_wall_moved(hidden: bool = False):
    """memory says A (behind a wall), the bin was moved to B. hidden=False: B is in view from A:
    route round the wall (no guard refusals), clear view at A, not_there, search/find B, dock, A
    downgraded. hidden=True: B is behind a second wall, so the spin at A can't see it: no
    failed:not_found after one spin; a search hop to a cell that sees new floor finds it."""
    tag = "wall_moved_hidden" if hidden else "wall_moved"
    print(f"full-stack: {tag.upper()}: wall between the start and remembered A, bin moved to B"
          + (" behind a second wall" if hidden else " (in view from A)"))
    w = wall_moved_world(hidden)
    bx, by = w.cans[0]
    st = Stack(w, mem_file([(2.5, 0.0)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(360)
        st.dump(tag)
        x, y, _ = st.sens.pose
        d = math.hypot(x - bx, y - by)
        ev = st.goto.events
        inside = any(1.2 < p[0] < 1.4 and abs(p[1]) < 0.7 for p in st.sens.trace)
        check(not inside, f"{tag}: never drove through the wall")
        check(any(e.startswith("route trash can #1") for e in ev), f"{tag}: routed over the map to A")
        # at A: a clear view and nothing -> not_there; or (B in view from A) it sees B there and goes straight on
        check(any(e.startswith("not_there trash can #1") for e in ev) or (not hidden and s["result"] == "arrived"),
              f"{tag}: clear view at A: not_there, or B seen from A")
        check(not any(e.startswith("occluded trash can #1") for e in ev), f"{tag}: A not reported occluded")
        check(s["result"] == "arrived" and d < 0.95, f"{tag}: found B and docked ({s['result']}, {d:.2f} m from B)")
        if hidden:
            check(any(e.startswith("search_hop") for e in ev), f"{tag}: spin at A saw nothing -> search hop to a new view")
        mem = st.goto.cmd("MEMORY")
        # A is downgraded: no fresh bin remembered near A any more (#1 may be stale/gone, or re-anchored
        # at B when goto docked there: arrived() refreshes the instance it was looking for)
        fresh_at_a = [m for m in re.finditer(r"trash can#(\d+) (\w+) x=(-?\d+) y=(-?\d+)", mem)
                      if m.group(2) == "fresh" and math.hypot(int(m.group(3)) / 1000 - 2.5, int(m.group(4)) / 1000) < 0.5]
        check(not fresh_at_a, f"{tag}: A downgraded in memory (no fresh bin at A: {mem[:220]})")
        check_no_refusals(st, s, tag)
    finally:
        st.close()


def pocket_mouth_world():
    """bench room tests/rooms/pocket_mouth.json (user's 5th demo), shifted so the car starts at the origin"""
    import json
    r = json.load(open(os.path.join(HERE, "rooms", "pocket_mouth.json")))
    sx, sy = r["start"]["x"], r["start"]["y"]
    w = World()
    w.walls = [((a - sx, b - sy), (c - sx, d - sy)) for a, b, c, d in r["walls"]]
    for o in r["objects"]:
        if o["label"] == "trash can":
            w.cans.append([o["x"] - sx, o["y"] - sy])
        else:
            w.walls.extend(box(o["x"] - sx, o["y"] - sy, o["r"], o["r"]))
    return w


def sc_pocket_mouth():
    """user's 5th demo: '<' pocket open only toward +x, car behind the apex, bin remembered inside.
    The way in is round an arm's end through floor never seen: main's goto said no_route, spun, and
    failed:not_found. Optimistic planning: route through unknown floor, re-plan as walls appear."""
    print("full-stack: POCKET_MOUTH: bin in a '<' pocket, the way in through unseen floor (optimistic planning)")
    w = pocket_mouth_world()
    bx, by = w.cans[0]
    st = Stack(w, mem_file([(bx, by)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("pocket_mouth")
        x, y, _ = st.sens.pose
        d = math.hypot(x - bx, y - by)
        ev = st.goto.events
        check(any("optimistic" in e for e in ev), "routed through unknown floor (route ... optimistic)")
        check(not any(e.startswith(("search_here", "next_candidate")) for e in ev), "never spun / gave up from behind the wall")
        check(st.min_wall_clearance() > 0.15, f"never closer than 0.15 m to a wall ({st.min_wall_clearance():.2f} m)")
        check(s["result"] == "arrived" and d < 0.95, f"pocket_mouth: arrived at the bin ({s['result']}, {d:.2f} m)")
        check_no_refusals(st, s, "pocket_mouth")
    finally:
        st.close()


def sc_pocket_mouth_remembered():
    """bench room pocket_mouth as the user runs it on the dashboard: goto starts out remembering the
    bin where it really is, behind the '<' wall. It must route from memory (round an arm's end, in
    through the opening) and arrive, with no 360 search before the first route leg."""
    print("full-stack: POCKET_MOUTH_REMEMBERED: bin remembered behind the wall, route from memory, no search first")
    w = pocket_mouth_world()
    bx, by = w.cans[0]
    st = Stack(w, mem_file([(bx, by)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("pocket_mouth_remembered")
        x, y, _ = st.sens.pose
        d = math.hypot(x - bx, y - by)
        ev = st.goto.events
        first_route = next((i for i, e in enumerate(ev) if e.startswith("route ")), None)
        first_search = next((i for i, e in enumerate(ev) if e.startswith(("search_here", "standoff_search"))), None)
        check(first_route is not None, "routed from memory (a 'route' event)")
        check(first_route is not None and (first_search is None or first_route < first_search),
              f"no 360 search before the first leg (route at {first_route}, search at {first_search})")
        check(not any(e.startswith("next_candidate") for e in ev), "never gave up on the remembered spot")
        check(st.min_wall_clearance() > 0.15, f"never closer than 0.15 m to a wall ({st.min_wall_clearance():.2f} m)")
        check(s["result"] == "arrived" and d < 0.95, f"pocket_mouth_remembered: arrived at the bin ({s['result']}, {d:.2f} m)")
    finally:
        st.close()

def sc_enclosed():
    """the bin sits in a fully walled box: no route even through unknown floor once the walls are
    mapped -> failed:unreachable within the caps (no endless wandering / spinning)"""
    print("full-stack: ENCLOSED: remembered bin inside a closed box -> failed:unreachable")
    w = World(size=7.0, x0=-2.0, y0=-3.5)
    w.cans.append([2.5, 0.0])
    w.walls.extend(box(2.5, 0.0, 0.5, 0.5))
    st = Stack(w, mem_file([(2.5, 0.0)]))
    try:
        t0 = time.monotonic()
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("enclosed")
        inside = any(2.0 < p[0] < 3.0 and -0.5 < p[1] < 0.5 for p in st.sens.trace)
        check(not inside, "never drove into the box")
        check(s["result"] == "failed:unreachable", f"enclosed: failed:unreachable ({s['result']}, {time.monotonic() - t0:.0f} s)")
        check(not any(e.startswith("search_here") for e in st.goto.events), "no search spin from behind the walls")
    finally:
        st.close()


def glass_world():
    w = World(size=7.0, x0=-2.0, y0=-3.5)
    w.cans.append([2.6, 0.0])
    # a glass pane across the straight line to the bin: the guard sees it, the phone map and YOLO don't
    w.glass.extend(box(1.3, 0.0, 0.03, 0.45))
    return w


def sc_glass():
    """something the map can't hold (glass: lidar_guard stops on it, the phone's map never gets it).
    The planner's straight route is stopped by the guard; goto marks the zone, backtracks along the
    trail and re-plans round it."""
    print("full-stack: GLASS: an obstacle only the guard sees -> refused leg, backtrack, re-plan round it")
    w = glass_world()
    st = Stack(w, mem_file([(2.6, 0.0)]))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(300)
        st.dump("glass")
        x, y, _ = st.sens.pose
        d = math.hypot(x - 2.6, y)
        ev = st.goto.events
        inside = any(1.24 < p[0] < 1.36 and abs(p[1]) < 0.48 for p in st.sens.trace)
        check(not inside, "never drove through the glass")
        check(any(e.startswith("backtrack") for e in ev), "stopped by the guard on the unmapped pane -> backtrack event")
        check(s["result"] == "arrived" and d < 0.95, f"backtracked, re-planned round it and docked ({s['result']}, {d:.2f} m)")
        # a narrow guard corridor (SM11: +-0.17 m) marks a narrow strip per refusal: more tries for a 0.9 m pane
        check(int(s.get("guard_aborts", 0)) + int(s.get("guard_refusals", 0)) <= (3 if GUARD_HALF_W >= 0.4 else 6),
              f"few guard stops for the one unmapped pane ({s.get('guard_refusals')} refused, {s.get('guard_aborts')} aborted)")
    finally:
        st.close()


def sc_stop_backtrack():
    """S (voice 'stop' / hand) in the middle of a backtrack reverse: wheels stop at once, goto cancels"""
    print("full-stack: STOP mid-backtrack: S from another client while goto reverses along its trail")
    w = glass_world()
    st = Stack(w, mem_file([(2.6, 0.0)]))
    try:
        st.goto.cmd("GOTO trash can")
        end = time.monotonic() + 120
        hit = False
        while time.monotonic() < end and not hit:
            hit = any(e.startswith("backtrack") for e in st.goto.events)
            time.sleep(0.01)
        check(hit, "a backtrack started")
        v = socket.create_connection(("127.0.0.1", st.motor.port))
        vf = v.makefile("rw", encoding="ascii", newline="\n")
        vf.write("WHO voice\nS\n"); vf.flush(); vf.readline(); r = vf.readline().strip()
        t_stop = time.monotonic()
        g = socket.create_connection(("127.0.0.1", st.goto.ctl))
        g.sendall(b"CANCEL\n")
        mv = "running"
        while time.monotonic() - t_stop < 0.2 and mv == "running":     # tars_motor aborts on its next 20 ms tick
            _, _, _, mv, _ = st.motor.pose()
        check(r == "OK" and mv != "running", f"S answered, the reverse is stopped within 200 ms ({r}, {mv}, {1000 * (time.monotonic() - t_stop):.0f} ms)")
        time.sleep(0.6)
        s = st.goto.status()
        p0 = st.sens.pose
        time.sleep(1.5)
        p1 = st.sens.pose
        st.dump("stop_backtrack")
        check(s["state"] == "idle" and s["result"].startswith("cancelled"), f"goto cancelled ({s['result']})")
        check(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 0.005 and abs(p1[2] - p0[2]) < 0.01, "wheels stay stopped")
        v.close(); g.close()
    finally:
        st.close()


def sc_moved():
    print("full-stack: MOVED OBJECT: memory says A, the bin is now at B")
    w = World()
    w.cans.append([0.3, 1.4])                  # B: off to the right
    st = Stack(w, mem_file([(2.2, -0.4)]))     # A: ahead-left, empty now
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(220)
        st.dump("moved")
        ev = st.goto.events
        x, y, _ = st.sens.pose
        check(any(e.startswith("not_there trash can #1") for e in ev), "at A, clear view, nothing -> not_there")
        check(s["result"] == "arrived" and math.hypot(x - 0.3, y - 1.4) < 1.0,
              f"searched, found it at B and docked ({s['result']}, {math.hypot(x - 0.3, y - 1.4):.2f} m)")
        mem = st.goto.cmd("MEMORY")
        check("trash can#1 stale" in mem or "trash can#1 gone" in mem, "old spot A downgraded in memory")
        check_no_refusals(st, s, "moved")
    finally:
        st.close()


def sc_stop():
    print("full-stack: STOP mid-path: S from another client (voice 'stop') wins")
    w = World()
    w.cans.append([2.6, 0.8])
    st = Stack(w, mem_file([(2.6, 0.8)]))
    try:
        st.goto.cmd("GOTO trash can")
        end = time.monotonic() + 30
        while time.monotonic() < end and not (st.goto.status()["state"] == "follow_path"
                                               and any(e.startswith("route") for e in st.goto.events)):
            time.sleep(0.05)
        check(any(e.startswith("route") for e in st.goto.events), "following a planned route when the stop comes")
        time.sleep(1.0)
        v = socket.create_connection(("127.0.0.1", st.motor.port))
        vf = v.makefile("rw", encoding="ascii", newline="\n")
        vf.write("WHO voice\nS\n"); vf.flush(); vf.readline(); vf.readline()
        g = socket.create_connection(("127.0.0.1", st.goto.ctl))
        g.sendall(b"CANCEL\n")
        time.sleep(0.6)
        s = st.goto.status()
        p0 = st.sens.pose
        time.sleep(1.5)
        p1 = st.sens.pose
        check(s["state"] == "idle" and s["result"].startswith("cancelled"), f"goto cancelled ({s['result']})")
        check(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 0.005 and abs(p1[2] - p0[2]) < 0.01, "wheels stay stopped")
        _, _, _, mv, _ = st.motor.pose()
        check(mv != "running", f"no exact move running ({mv})")
        v.close(); g.close()
    finally:
        st.close()


def sc_visual_only():
    print("full-stack: VISUAL FALLBACK: no phone feed at all, bin in view")
    w = World()
    w.cans.append([2.0, 0.3])
    st = Stack(w, None, nav=False)
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(120)
        st.dump("visual")
        x, y, _ = st.sens.pose
        check(s["result"] == "arrived" and s["map"] == "ready", f"arrived visually ({s['result']}, map={s['map']})")
        check(not any(e.startswith("route") for e in st.goto.events), "no map routing without the phone")
        check(math.hypot(x - 2.0, y - 0.3) < 0.95, "docked at the bin")
        nav = st.goto.cmd("NAV")
        check(nav.startswith("OK pose=none") or nav.startswith("OK pose=stale"), f"NAV: no ARKit pose ({nav[:40]})")
    finally:
        st.close()


def synth_wav(lines: list[str], path: str) -> None:
    import numpy as np
    SR = 16000
    parts = [np.zeros(SR, np.int16)]
    for i, t in enumerate(lines):
        aiff = os.path.join(BUILD, f"fs_say_{i}.aiff")
        wav = os.path.join(BUILD, f"fs_say_{i}.wav")
        subprocess.run(["say", "-o", aiff, t], check=True)
        subprocess.run(["afconvert", "-f", "WAVE", "-d", "LEI16@16000", "-c", "1", aiff, wav], check=True)
        with wave.open(wav) as w:
            parts.append(np.frombuffer(w.readframes(w.getnframes()), np.int16))
        parts.append(np.zeros(int(1.5 * SR), np.int16))
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes(np.concatenate(parts).tobytes())


class FakePhoneSpeech:
    def __init__(self):
        self.port = free_port()
        self.lines: list[str] = []
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(4)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            data = b""
            c.settimeout(1)
            try:
                while True:
                    b = c.recv(4096)
                    if not b:
                        break
                    data += b
            except OSError:
                pass
            self.lines += [l for l in data.decode(errors="replace").split("\n") if l]
            c.close()


def sc_voice():
    print("full-stack: VOICE: 'what do you remember' (MEMORY SAY) and 'go back to where the bin was' (RECALL)")
    if not (os.path.exists(VOICE) and os.path.exists(VMODEL)):
        print("  skip: build voice (make -C iter2-autonomy/voice host) and fetch its model (iter2-autonomy/voice/get_model.sh)")
        return
    try:
        import numpy  # noqa: F401
        import shutil
        assert shutil.which("say") and shutil.which("afconvert")
    except Exception:
        print("  skip: needs macOS say + numpy")
        return
    w = World()
    w.walls.append(((1.3, -0.6), (1.3, 0.6)))
    w.cans.append([2.4, 0.0])
    st = Stack(w, mem_file([(2.4, 0.0)]))
    phone = FakePhoneSpeech()
    wav = os.path.join(BUILD, "fs_voice.wav")
    synth_wav(["Tars, what do you remember?", "Tars, go back to where the bin was."], wav)
    try:
        r = subprocess.run([VOICE, "--input", f"wav:{wav}", "--model", VMODEL, "--port", str(st.motor.port),
                            "--goto-port", str(st.goto.ctl), "--say-port", str(phone.port), "--threads", "2"],
                           capture_output=True, text=True, timeout=300)
        out = r.stdout
        if VERBOSE:
            print(out)
        mem_line = next((l for l in out.splitlines() if l.startswith("[memory] ")), "")
        check(mem_line.startswith("[memory] I remember 1 thing: a trash can about"), f"voice prints the memory sentence ({mem_line[:90]})")
        check("BEEP 1" in out, "beeps once per remembered object (1)")
        time.sleep(0.3)
        check(any(l.startswith("SAY I remember 1 thing") for l in phone.lines), f"SAY <sentence> reached the phone port ({phone.lines[:1]})")
        check("[goto] RECALL trash can" in out, "'go back to where the bin was' -> RECALL trash can")
        s = st.goto.wait_idle(150)
        st.dump("voice")
        check(s["result"] == "arrived" and any(e.startswith("route") for e in st.goto.events),
              f"RECALL routed around the wall and docked ({s['result']})")
    finally:
        phone.srv.close()
        st.close()


SCENARIOS = {"obstacle": sc_obstacle, "occlusion": sc_occlusion,
             "occlusion_close": lambda: sc_occlusion(ahead=0.39), "dropbox": sc_dropbox, "wallmem": sc_wallmem, "wallmem_search": sc_wallmem_search, "diagwall": sc_diagwall, "moved": sc_moved, "stop": sc_stop,
             "visual": sc_visual_only, "voice": sc_voice, "shortwall": sc_shortwall, "offsetwall": sc_offsetwall,
             "openbin": sc_openbin, "lpocket": sc_lpocket, "pocket_mouth": sc_pocket_mouth, "enclosed": sc_enclosed, "wall_moved": sc_wall_moved,
             "wall_moved_hidden": lambda: sc_wall_moved(hidden=True), "glass": sc_glass, "stop_backtrack": sc_stop_backtrack}

SCENARIOS["pocket_mouth_remembered"] = sc_pocket_mouth_remembered

# known failures, run with --only NAME (not in the default run until fixed)
EXTRA: dict = {}


def run_scenario(name: str) -> None:
    # An exception in a scenario thread used to vanish into stderr: its checks never
    # ran, so the total shrank (24/24) and the suite still exited 0. Count it as a FAIL.
    try:
        (SCENARIOS.get(name) or EXTRA[name])()
    except Exception as e:
        traceback.print_exc()
        check(False, f"{name}: scenario crashed before finishing its checks ({e!r})")


def main() -> int:
    for b in (MOTOR, GOTO):
        if not os.path.exists(b):
            print(f"missing {b}: make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host")
            return 2
    only = sys.argv[sys.argv.index("--only") + 1] if "--only" in sys.argv else None
    t0 = time.monotonic()
    names = only.split(",") if only else list(SCENARIOS)
    threads = []
    # independent stacks on their own ports: run them together, but not all 20 at once: on a laptop
    # that starves the fake guard (> 100 ms between GAT1 packets = tars_motor guard_stale stops)
    gate = threading.Semaphore(int(os.environ.get("FULLSTACK_PARALLEL", "7")))

    def gated(n):
        with gate:
            run_scenario(n)
    for n in names:
        t = threading.Thread(target=gated, args=(n,))
        t.start()
        threads.append(t)
        time.sleep(0.5)
    for t in threads:
        t.join()
    print(f"{passed}/{passed + failed} checks passed ({time.monotonic() - t0:.0f} s)")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
