#!/usr/bin/env python3
"""Scripted full-stack sim (Mac only): the real native processes, fake sensors.

  real  iter1-wasd/pi/build/tars_motor_host -b stub   motion + pose + guard gate (-g, GAT1 UDP)
  real  iter2-autonomy/goto/build/goto_host               control 5570-style port, dets 5571-style, nav 5572-style
  real  iter2-autonomy/voice/build/tars_voice_host        (voice scenarios: wav in, whisper, intent, routing)
  fake  lidar_guard   GAT1 to tars_motor: BLOCKED when anything is < GUARD_M ahead (forward only)
  fake  phone         LDR1 packets (pose + raycast LiDAR, 60 deg FOV) to goto's nav port, 20 Hz
  fake  YOLO          detection JSON lines to goto's det port, 8 Hz, occlusion-aware
  fake  phone speech  TCP listener for voice's "SAY <text>" (5573-style)

Ground truth: the robot is wherever tars_motor's stub says it is (STATUS pose=, dead
reckoned from the exact moves it ran). World frame = tars_motor's (x forward, y right, m);
the fake phone reports it in the ARKit frame goto expects (x_ark = -y, z_ark = x, yaw = -hdg).
Sim speed-up: WHEELCAL 260 400 (4x the stepper's linear speed, same turn rate). Not physical.

Scenarios: obstacle (wall between robot and remembered bin), occlusion (something steps
between robot and bin at the spot -> OCCLUDED -> DETOUR), moved object (memory says A, bin is
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

GUARD_M = 0.55            # fake lidar_guard: stop zone from the robot centre, forward corridor
GUARD_HALF_W = 0.18
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
        self.lock = threading.Lock()

    def segments(self, with_cans=True):
        segs = list(self.walls)
        if with_cans:
            for c in self.cans:
                segs += box(c[0], c[1], CAN_R, CAN_R)
        return segs

    def ray(self, px, py, ang, maxd=4.0, with_cans=True):
        dx, dy = math.cos(ang), math.sin(ang)
        best = maxd
        hit = False
        with self.lock:
            segs = self.segments(with_cans)
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
        while self.run:
            x, y, h = self.pose
            blocked = False
            for off in (-GUARD_HALF_W, 0.0, GUARD_HALF_W):
                ox, oy = x - off * math.sin(h), y + off * math.cos(h)
                t, hit = self.w.ray(ox, oy, h, GUARD_M)
                blocked |= hit
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
        args = [GOTO, "-m", f"127.0.0.1:{motor_port}", "-p", str(self.ctl), "-u", str(self.det),
                "-n", str(self.nav if nav else 0), "-r", "0", "-M", self.mem, "-v",
                "-o", os.path.join(GOTO_DIR, "objects.txt"), "-c", os.path.join(GOTO_DIR, "goto.conf")]
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

    def close(self):
        self.sens.close()
        self.goto.close()
        self.motor.close()

    def min_wall_clearance(self):
        return min((self.world.min_dist_walls(p) for p in self.sens.trace), default=9)

    def dump(self, name):
        if VERBOSE or True:
            print(f"    [{name}] events: {' | '.join(self.goto.events)[:900]}")


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
    finally:
        st.close()


def sc_occlusion():
    print("full-stack: OCCLUSION: something steps between the robot and the bin at the spot")
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
                k = d - 0.36                     # 0.36 m in front of the bin
                cx, cy = x + ux * k, y + uy * k
                with w.lock:
                    w.walls.append(((cx - uy * 0.3, cy + ux * 0.3), (cx + uy * 0.3, cy - ux * 0.3)))
                placed.append((cx, cy, d))
    try:
        st.goto.cmd("GOTO trash can")
        s = st.goto.wait_idle(200, hook)
        st.dump("occlusion")
        ev = st.goto.events
        check(bool(placed), f"occluder placed at the spot ({placed[0][2]:.2f} m from the bin)" if placed else "occluder placed")
        check(any(e.startswith("occluded trash can #1") for e in ev), "LOOK fails + map occupied -> OCCLUDED")
        check(not any(e.startswith("not_there trash can #1") for e in ev[:ev.index(next((e for e in ev if e.startswith('occluded')), ev[-1])) + 1]),
              "occluded is not 'gone' (no not_there before it)")
        check(any(e.startswith("detour trash can #1") for e in ev), "-> DETOUR to a vantage cell")
        check(s["result"] == "arrived", f"arrived after the detour ({s['result']})")
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


SCENARIOS = {"obstacle": sc_obstacle, "occlusion": sc_occlusion, "moved": sc_moved, "stop": sc_stop,
             "visual": sc_visual_only, "voice": sc_voice}


def run_scenario(name: str) -> None:
    # An exception in a scenario thread used to vanish into stderr: its checks never
    # ran, so the total shrank (24/24) and the suite still exited 0. Count it as a FAIL.
    try:
        SCENARIOS[name]()
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
    names = [only] if only else list(SCENARIOS)
    threads = []
    for n in names:                       # independent stacks on their own ports: run them together
        t = threading.Thread(target=run_scenario, args=(n,))
        t.start()
        threads.append(t)
        time.sleep(0.5)
    for t in threads:
        t.join()
    print(f"{passed}/{passed + failed} checks passed ({time.monotonic() - t0:.0f} s)")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
