"""Stand-in for the Pi's goalie_motor server (stdlib only), for testing the
vision side with no Pi. Implements docs/architecture.md link 1 roughly:

  F / MODE / HUMAN / AUTO / GOAL / RESET / PING / STATUS   (+ SIMHIT <mag>)

A 200 Hz control thread does predict -> target -> P on paddle x -> duty, and
the F reply returns the latest values from that thread (like the real server).
MODE LINUX injects random 0-30 ms stalls into that thread (an EMULATION of a
non-real-time OS, same as the real server), so its replies go stale and the
jitter stats in STATUS jump.

The predictor/controller here are deliberately simple; the real ones live in
goalie/pi/src (C) and goalie/sim (Python).

    python mock_pi.py                # port 5556
    python mock_pi.py --port 0       # pick a free port, print it
"""
from __future__ import annotations

import argparse
import random
import socketserver
import threading
import time
from collections import deque

from field import BALL_R, DEFAULT_PORT, FIELD_W, GOAL_HALF_W, WD_MS

TICK_S = 0.005            # 200 Hz
FIT_N = 4                 # samples in the velocity fit
VY_IN = -0.05             # in flight if vy below this
VY_OUT = 0.05             # reversed if vy above this
SAVE_ZONE_Y = 0.10
REARM_Y = 0.25
REARM_UNSEEN_S = 1.0
KP = 5.0                  # duty per metre of error
KD = 0.15                 # duty per (m/s) of paddle velocity
LINUX_STALL_P = 0.25      # chance per tick of an injected stall in LINUX mode
LINUX_STALL_MAX_S = 0.030


def predict_cross_x(hist) -> tuple[float | None, float, float]:
    """hist: [(t, x, y)] newest last. Returns (x_cross | None, vx, vy)."""
    if len(hist) < 2:
        return None, 0.0, 0.0
    pts = list(hist)[-FIT_N:]
    n = len(pts)
    tm = sum(p[0] for p in pts) / n
    xm = sum(p[1] for p in pts) / n
    ym = sum(p[2] for p in pts) / n
    stt = sum((p[0] - tm) ** 2 for p in pts) or 1e-9
    vx = sum((p[0] - tm) * (p[1] - xm) for p in pts) / stt
    vy = sum((p[0] - tm) * (p[2] - ym) for p in pts) / stt
    if vy >= VY_IN:
        return None, vx, vy
    t_hit = pts[-1][2] / -vy
    x = pts[-1][1] + vx * t_hit
    wall = FIELD_W / 2 - BALL_R           # unfold side-wall bounces
    period = 4 * wall
    x = (x + wall) % period
    x = x - wall if x <= 2 * wall else 3 * wall - x
    return max(-GOAL_HALF_W, min(GOAL_HALF_W, x)), vx, vy


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = "QNX"
        self.ctl = "AUTO"
        self.human_x = 0.0
        self.saves = self.goals = self.hits = 0
        self.target = self.duty = 0.0
        self.paddle = None
        self.paddle_hist = deque(maxlen=4)
        self.ball_hist = deque(maxlen=16)
        self.last_feed = 0.0          # F / PING / HUMAN
        self.last_ball_seen = 0.0
        self.wd = 1
        self.armed = True
        self.in_flight = False
        self.vy = 0.0
        self.jit_sum = self.jit_n = self.jit_max = 0
        self.ticks = deque(maxlen=200)
        self.running = True

    def reset_jitter(self):
        self.jit_sum = self.jit_n = self.jit_max = 0
        self.ticks.clear()

    # -- events -----------------------------------------------------------
    def on_frame(self, t_ms, bs, bx, by, ps, px):
        now = time.monotonic()
        self.last_feed = now
        t = t_ms / 1000.0
        if ps:
            self.paddle = px
            self.paddle_hist.append((t, px))
        else:
            self.paddle = None
        if not bs:
            if now - self.last_ball_seen > REARM_UNSEEN_S:
                self.armed, self.in_flight = True, False
                self.ball_hist.clear()
            return
        self.last_ball_seen = now
        if self.ball_hist and t <= self.ball_hist[-1][0]:
            self.ball_hist.clear()                         # time went backwards
        self.ball_hist.append((t, bx, by))
        if by > REARM_Y:
            self.armed = True
        _, _, vy = predict_cross_x(self.ball_hist)
        self.vy = vy
        if vy < VY_IN and self.armed:
            self.in_flight = True
        recent_min_y = min(p[2] for p in list(self.ball_hist)[-FIT_N - 2:])
        if self.in_flight and vy > VY_OUT and recent_min_y < SAVE_ZONE_Y:
            self._save()

    def _save(self):
        if self.armed:
            self.saves += 1
        self.armed, self.in_flight = False, False

    def on_hit(self):
        self.hits += 1
        if self.in_flight and self.vy < VY_IN:
            self._save()

    def on_goal(self):
        self.goals += 1
        self.armed, self.in_flight = False, False

    # -- 200 Hz control loop ----------------------------------------------
    def control_loop(self):
        nxt = time.monotonic()
        while self.running:
            nxt += TICK_S
            with self.lock:
                linux = self.mode == "LINUX"
            if linux and random.random() < LINUX_STALL_P:
                time.sleep(random.uniform(0, LINUX_STALL_MAX_S))   # emulated jitter
            d = nxt - time.monotonic()
            if d > 0:
                time.sleep(d)
            now = time.monotonic()
            late_us = int(max(0.0, now - nxt) * 1e6)
            if now - nxt > TICK_S:
                nxt = now                       # re-anchor: report per-tick lateness, don't spiral
            with self.lock:
                self.jit_sum += late_us
                self.jit_n += 1
                self.jit_max = max(self.jit_max, late_us)
                self.ticks.append(now)
                self._control(now)

    def _control(self, now):
        self.wd = 1 if (now - self.last_feed) * 1000 > WD_MS else 0
        if self.wd:
            self.target = 0.0
        elif self.ctl == "HUMAN":
            self.target = self.human_x
        else:
            x, _, _ = predict_cross_x(self.ball_hist)
            self.target = 0.0 if x is None else x
        if self.paddle is None:
            self.duty = 0.0
            return
        v = 0.0
        if len(self.paddle_hist) >= 2:
            (t0, x0), (t1, x1) = self.paddle_hist[0], self.paddle_hist[-1]
            v = (x1 - x0) / (t1 - t0) if t1 > t0 else 0.0
        d = KP * (self.target - self.paddle) - KD * v
        self.duty = max(-1.0, min(1.0, d))

    def status_line(self):
        hz = 0.0
        if len(self.ticks) > 1:
            hz = (len(self.ticks) - 1) / (self.ticks[-1] - self.ticks[0])
        msf = int((time.monotonic() - self.last_feed) * 1000) if self.last_feed else 99999
        return (f"OK mode={self.mode} ctl={self.ctl} saves={self.saves} goals={self.goals} "
                f"target={self.target:.4f} paddle={(self.paddle or 0.0):.4f} duty={self.duty:.3f} "
                f"wd={self.wd} ms_since_frame={msf} "
                f"jit_avg_us={self.jit_sum // max(1, self.jit_n)} jit_max_us={self.jit_max} "
                f"loop_hz={hz:.1f} hits={self.hits} mock=1")


def handle(st: State, line: str) -> str:
    p = line.split()
    if not p:
        return "ERR empty"
    cmd = p[0].upper()
    try:
        with st.lock:
            if cmd == "F":
                t_ms, bs, bx, by, ps, px = int(p[1]), int(p[2]), float(p[3]), float(p[4]), int(p[5]), float(p[6])
                st.on_frame(t_ms, bs, bx, by, ps, px)
                return f"OK {st.target:.4f} {st.duty:.3f}"
            if cmd == "MODE":
                m = p[1].upper()
                if m not in ("QNX", "LINUX"):
                    return "ERR mode must be QNX or LINUX"
                st.mode = m
                st.reset_jitter()
                return f"OK {m}"
            if cmd == "HUMAN":
                st.ctl, st.human_x = "HUMAN", max(-GOAL_HALF_W, min(GOAL_HALF_W, float(p[1])))
                st.last_feed = time.monotonic()
                return "OK"
            if cmd == "AUTO":
                st.ctl = "AUTO"
                return "OK"
            if cmd == "GOAL":
                st.on_goal()
                return f"OK goals={st.goals}"
            if cmd == "RESET":
                st.saves = st.goals = st.hits = 0
                st.reset_jitter()
                return "OK"
            if cmd == "PING":
                st.last_feed = time.monotonic()
                return "OK PONG"
            if cmd == "STATUS":
                return st.status_line()
            if cmd == "SIMHIT":
                st.on_hit()
                return f"OK hits={st.hits}"
    except (IndexError, ValueError):
        return f"ERR bad args for {cmd}"
    return f"ERR unknown command {cmd}"


class _Handler(socketserver.StreamRequestHandler):
    def setup(self):
        super().setup()
        self.server.conns.add(self.request)

    def finish(self):
        self.server.conns.discard(self.request)
        super().finish()

    def handle(self):
        for raw in self.rfile:
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            self.wfile.write((handle(self.server.st, line) + "\n").encode())


class _Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def start(port: int = DEFAULT_PORT, host: str = "127.0.0.1"):
    """Start in background threads. Returns (server, port). Stop with stop(server)."""
    srv = _Server((host, port), _Handler)
    srv.st = State()
    srv.conns = set()
    threading.Thread(target=srv.st.control_loop, daemon=True).start()
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


def stop(srv) -> None:
    srv.st.running = False
    srv.shutdown()
    srv.server_close()
    for c in list(srv.conns):        # drop clients too, like a killed process
        try:
            c.shutdown(2)
            c.close()
        except OSError:
            pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--host", default="0.0.0.0")
    a = ap.parse_args()
    srv, port = start(a.port, a.host)
    print(f"mock_pi listening on {a.host}:{port}", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop(srv)
