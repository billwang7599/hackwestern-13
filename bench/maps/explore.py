"""Bench-side explorer for map demo A (laptop only, SIMULATION or a real stack: it only uses the
robot link). No goto changes: it uses existing commands only.

    loop:  goto SCAN (spin in place; the phone maps what it sees, YOLO notes objects)
           -> goto MEMORY has the target with a map position?  done (then optionally GOTO it)
           -> else pick a frontier on goto's own grid (NAV GRID: a free cell next to unknown floor,
              reachable through free cells kept clear of obstacles) and drive there with TURN / MOVE
              to tars_motor (the robot plugin keeps the move alive with PING; the guard can refuse it)

Stop always wins: a STOP anywhere (the robot plugin's stop counter moves), a refused/aborted move, or
cancel() ends the run, and nothing new is sent after that. All motion goes through tars_motor.
"""
from __future__ import annotations

import heapq
import math
import threading
import time

from . import prescan as PS

ROBOT_CLEAR_M = 0.45      # keep paths this far from occupied cells (lidar_guard's corridor is +-0.45 m)
FALLBACK_CLEAR_M = 0.30   # ... or this, if nothing is reachable with the wide margin
HOP_MAX_M = 1.6           # drive at most this far before scanning again
MIN_GOAL_M = 0.5


class Aborted(Exception):
    pass


def parse_nav(reply: str):
    """goto 'OK pose=arkit,x,z,yawdeg free=.. occ=..' -> (src, x, z, yaw_rad, free, occ) or None"""
    if not isinstance(reply, str) or not reply.startswith("OK pose="):
        return None
    kv = dict(t.split("=", 1) for t in reply.split()[1:] if "=" in t)
    try:
        src, x, z, yaw = kv["pose"].split(",")
        return src, float(x), float(z), math.radians(float(yaw)), int(kv.get("free", 0)), int(kv.get("occ", 0))
    except (KeyError, ValueError):
        return None


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------ planning on goto's grid
def _blocked(g, clear_m: float) -> bytearray:
    """1 where the robot's centre may not go: unknown, or within clear_m of an occupied cell"""
    S = PS.SIZE
    out = bytearray(1 if v != PS.FREE else 0 for v in g)
    r = int(math.ceil(clear_m / PS.RES))
    offs = [(dx, dz) for dx in range(-r, r + 1) for dz in range(-r, r + 1) if math.hypot(dx, dz) * PS.RES <= clear_m]
    for i, v in enumerate(g):
        if v == PS.OCC:
            ix, iz = i % S, i // S
            for dx, dz in offs:
                x, z = ix + dx, iz + dz
                if 0 <= x < S and 0 <= z < S:
                    out[z * S + x] = 1
    return out


def plan_frontier(g, x: float, z: float, clear_m: float = ROBOT_CLEAR_M, avoid=(), max_hop: float = HOP_MAX_M):
    """-> (goal (x, z), [waypoints (x, z)], info) or None. A frontier cell is a reachable free cell
    with an unknown 4-neighbour, at least MIN_GOAL_M away and not near an `avoid` point (goals that
    already failed or were visited). Nearest first (Dijkstra over 8-connected free cells)."""
    S = PS.SIZE
    start = PS.ark_cell(x, z)
    if start is None:
        return None
    blk = _blocked(g, clear_m)
    # the cells around the robot itself are often unobserved (the phone sees 0.25 m+ ahead): allow them
    for dx in range(-3, 4):
        for dz in range(-3, 4):
            cx, cz = start[0] + dx, start[1] + dz
            if 0 <= cx < S and 0 <= cz < S and g[cz * S + cx] != PS.OCC and math.hypot(dx, dz) <= 3:
                blk[cz * S + cx] = 0
    si = start[1] * S + start[0]
    dist = {si: 0.0}
    prev = {}
    pq = [(0.0, si)]
    best = None
    while pq:
        d, i = heapq.heappop(pq)
        if d > dist.get(i, 1e9):
            continue
        ix, iz = i % S, i // S
        if d >= MIN_GOAL_M and g[i] == PS.FREE:
            fr = any(0 <= ix + a < S and 0 <= iz + b < S and g[(iz + b) * S + ix + a] == PS.UNKNOWN
                     for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1)))
            if fr:
                cx, cz = PS.cell_center(ix, iz)
                if not any(math.hypot(cx - ax, cz - az) < 0.6 for ax, az in avoid):
                    best = i
                    break
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                if not a and not b:
                    continue
                nx, nz = ix + a, iz + b
                if not (0 <= nx < S and 0 <= nz < S):
                    continue
                j = nz * S + nx
                if blk[j]:
                    continue
                nd = d + PS.RES * (1.4142 if a and b else 1.0)
                if nd < dist.get(j, 1e9):
                    dist[j], prev[j] = nd, i
                    heapq.heappush(pq, (nd, j))
    if best is None:
        return None
    cells = [best]
    while cells[-1] != si:
        cells.append(prev[cells[-1]])
    cells.reverse()
    pts = [PS.cell_center(i % S, i // S) for i in cells]
    pts[0] = (x, z)
    wps = _simplify(pts, blk)
    # cap the hop length: the robot scans again on the way
    out, run = [], 0.0
    px, pz = x, z
    for wx, wz in wps[1:]:
        L = math.hypot(wx - px, wz - pz)
        if run + L > max_hop:
            f = (max_hop - run) / L
            out.append((px + f * (wx - px), pz + f * (wz - pz)))
            break
        out.append((wx, wz))
        run += L
        px, pz = wx, wz
    goal = PS.cell_center(best % S, best // S)
    return goal, out, {"path_m": round(dist[best], 2), "clear_m": clear_m}


def _line_free(a, b, blk) -> bool:
    S = PS.SIZE
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (PS.RES / 2)))
    for k in range(1, n):
        x = a[0] + (b[0] - a[0]) * k / n
        z = a[1] + (b[1] - a[1]) * k / n
        c = PS.ark_cell(x, z)
        if c is None or blk[c[1] * S + c[0]]:
            return False
    return True


def _simplify(pts, blk):
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not _line_free(pts[i], pts[j], blk):
            j -= 1
        out.append(pts[j])
        i = j
    return out


# ------------------------------------------------------------------ the run
class Explorer:
    """One exploration run on its own thread. `robot` is the robot plugin module (send, status,
    goto_status, stop_count). publish(dict) reports progress (topic `maps`)."""

    def __init__(self, robot, publish, log, target: str = "trash can", max_hops: int = 6,
                 then_goto: bool = True, scan_timeout: float = 120.0):
        self.robot, self.publish, self.log = robot, publish, log
        self.target, self.max_hops, self.then_goto = target, max_hops, then_goto
        self.scan_timeout = scan_timeout
        self.cancelled = threading.Event()
        self.state = {"kind": "explore", "running": False, "phase": "idle", "hops": 0, "scans": 0,
                      "found": None, "result": None, "goal": None, "path": None, "target": target}
        self.thread = None
        self.stops0 = robot.stop_count()

    # ---- helpers
    def _pub(self, **kw):
        self.state.update(kw)
        self.publish(dict(self.state))

    def check(self):
        if self.robot.stop_count() != self.stops0:
            raise Aborted("STOP pressed")
        if self.cancelled.is_set():
            raise Aborted("cancelled")

    def send(self, dest, line, timeout=3.0):
        self.check()
        return self.robot.send(dest, line, timeout=timeout, src="explore")

    def sleep(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            self.check()
            time.sleep(0.05)

    def pose(self):
        p = parse_nav(self.robot.send("goto", "NAV", timeout=2.0, src="poll"))
        if p is None:
            raise Aborted("goto gave no nav pose")
        return p

    def found(self):
        rep = self.robot.send("goto", "MEMORY", timeout=2.0, src="poll")
        for o in PS.parse_memory(rep):
            if o["label"] == self.target and o["status"] in ("fresh", "stale"):
                return o
        return None

    def scan(self):
        self._pub(phase="scanning")
        r = self.send("goto", "SCAN")
        if not r.startswith("OK"):
            raise Aborted(f"SCAN refused: {r}")
        self.state["scans"] += 1
        end = time.monotonic() + self.scan_timeout
        started = False
        while time.monotonic() < end:
            self.check()
            g = self.robot.goto_status() or {}
            st, res = g.get("state"), str(g.get("result", ""))
            if st not in (None, "idle"):
                started = True
            elif started or time.monotonic() > end - self.scan_timeout + 3:
                if st == "idle":
                    if res.startswith("scanned"):
                        return res
                    if res.startswith("cancelled"):
                        raise Aborted(f"scan ended: {res}")
                    raise Refused(f"scan ended: {res}")
            time.sleep(0.15)
        raise Aborted("scan timed out")

    def wait_move(self, move_id: int, timeout: float) -> dict:
        """until tars_motor's STATUS shows OUR move finished (the 10 Hz copy can still show the last one)"""
        end = time.monotonic() + timeout
        t0 = time.monotonic()
        while time.monotonic() < end:
            self.check()
            st = self.robot.status()
            if st and st.get("move_id") == move_id and st.get("move") != "running":
                return st
            if st and st.get("move_id") not in (move_id, None) and time.monotonic() - t0 > 1.5:
                return st                    # someone else's move replaced ours
            time.sleep(0.05)
        self.robot.send("motor", "S", src="explore")
        raise Aborted("move timed out (S sent)")

    def motion(self, line: str, timeout: float) -> dict:
        r = self.send("motor", line)
        if not r.startswith("OK "):
            raise Refused(f"{line}: {r}")
        mid = int(r.split()[1])
        st = self.wait_move(mid, timeout)
        if st.get("move_id") != mid or st.get("move") == "aborted":
            raise Refused(f"{line}: {st.get('move')} ({st.get('move_why', '-')})")
        self.sleep(0.35)                     # let a fresh ARKit pose arrive
        return st

    def drive(self, waypoints):
        for wx, wz in waypoints:
            for _ in range(3):               # turn, then correct an under-turn (uncalibrated car)
                _, x, z, yaw, *_ = self.pose()
                want = math.atan2(wx - x, wz - z)
                d = math.degrees(wrap(want - yaw))
                if abs(d) < 8:
                    break
                self._pub(phase=f"turning {-d:+.0f} deg")
                self.motion(f"TURN {-d:.0f}", 25)
            _, x, z, yaw, *_ = self.pose()
            dist = math.hypot(wx - x, wz - z)
            if dist < 0.08:
                continue
            self._pub(phase=f"moving {dist:.2f} m")
            self.motion(f"MOVE {1000 * dist:.0f}", 6 + 12 * dist)

    def run(self):
        self._pub(running=True, phase="starting", result=None, found=None, hops=0, scans=0)
        tried = []
        try:
            failed_scans = 0
            while True:
                try:
                    self.scan()
                    failed_scans = 0
                except Refused as e:          # a scan that ended badly (turn timeout, guard): the map
                    failed_scans += 1         # still grew; carry on unless it keeps happening
                    self.log("explore", f"scan {self.state['scans']}: {e}")
                    if failed_scans >= 3:
                        raise Aborted(f"3 scans in a row failed ({e})")
                o = self.found()
                if o:
                    self._pub(found=o, phase="found")
                    self.log("explore", f"{self.target} in goto's memory at map {o['x_ark']:.2f},{o['z_ark']:.2f}")
                    break
                if self.state["hops"] >= self.max_hops:
                    raise Aborted(f"not found after {self.max_hops} hops")
                g = PS.parse_nav_grid(self.robot.send("goto", "NAV GRID", timeout=3.0, src="poll"))
                if g is None:
                    raise Aborted("goto gave no NAV GRID")
                _, x, z, _, *_ = self.pose()
                plan = plan_frontier(g, x, z, ROBOT_CLEAR_M, tried) or plan_frontier(g, x, z, FALLBACK_CLEAR_M, tried)
                if plan is None:
                    raise Aborted("no reachable frontier left (the map is explored)")
                goal, wps, info = plan
                tried.append(goal)
                self.state["hops"] += 1
                self._pub(phase="driving to a frontier", goal=goal, path=[[x, z]] + [list(p) for p in wps])
                self.log("explore", f"hop {self.state['hops']}: frontier {goal[0]:.2f},{goal[1]:.2f} "
                                    f"({info['path_m']} m, clearance {info['clear_m']} m), {len(wps)} legs")
                try:
                    self.drive(wps)
                except Refused as e:     # the guard refused / aborted: scan from here, pick another
                    self.log("explore", f"hop {self.state['hops']} cut short: {e}")
            if self.then_goto:
                self._pub(phase="GOTO " + self.target)
                r = self.send("goto", f"GOTO {self.target}")
                self._pub(result=f"found; GOTO {self.target} -> {r}")
            else:
                self._pub(result="found")
        except Aborted as e:
            self._pub(result=f"stopped: {e}")
            self.log("explore", f"stopped: {e}")
        except Exception as e:                       # never leave a move running
            self.robot.send("motor", "S", src="explore")
            self._pub(result=f"error: {type(e).__name__}: {e}")
            raise
        finally:
            self._pub(running=False, phase="done")

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True, name="explore")
        self.thread.start()
        return self

    def cancel(self):
        self.cancelled.set()


class Refused(Exception):
    pass
