"""Synthetic shooter + goalie paddle, to drive the whole stack with no camera.

FakeWorld rolls balls from y=0.6 m at a random point in the goal and moves a
simulated paddle from the server's reply (`F` -> `OK <target_x> <duty>`):
    paddle accel = (duty * vmax - v) / tau        (first-order DC motor, default)
    or --follow target: paddle chases target_x with lag tau, capped at vmax
It reports what a camera would see: noisy, delayed (latency) and at `fps`.

Standalone headless run against a server (real goalie_motor or mock_pi.py):

    python fake_ball.py --host 127.0.0.1 --port 5556 --shots 20 --speed 1.0

prints one line per shot and a summary comparing the local truth with the
server's saves/goals counters. On a goal it sends GOAL (as track.py would).
track.py --fake uses FakeWorld as its camera.
"""
from __future__ import annotations

import argparse
import math
import random
import sys
import time
from collections import deque
from dataclasses import dataclass

from field import BALL_R, FIELD_L, FIELD_W, GOAL_HALF_W

PADDLE_Y = 0.03           # blade line in front of the goal line
PADDLE_HALF = 0.035       # half blade width (7 cm blade)
PADDLE_LIMIT = GOAL_HALF_W
RESTITUTION = 0.5
SUBSTEP = 0.001


@dataclass
class FakeConfig:
    speed: float = 1.0          # m/s shot speed
    speed_jitter: float = 0.1   # +- fraction
    fps: float = 30.0
    latency_ms: float = 40.0
    noise_m: float = 0.003      # camera position noise (1 sigma)
    dropout: float = 0.02       # chance a detection is missed
    paddle_vmax: float = 1.5    # m/s at duty 1 (~150 RPM, 12 cm arm, roughly)
    paddle_tau: float = 0.06    # s
    follow: str = "duty"        # "duty" | "target"
    gap_s: float = 0.6          # ball unseen between shots
    after_s: float = 0.35       # keep simulating after the outcome
    seed: int | None = None


class FakeWorld:
    def __init__(self, cfg: FakeConfig | None = None):
        self.cfg = cfg or FakeConfig()
        self.rng = random.Random(self.cfg.seed)
        self.t = 0.0
        self.px, self.pv = 0.0, 0.0
        self.target, self.duty = 0.0, 0.0
        self.ball = None                 # [x, y, vx, vy] or None (no ball on field)
        self.phase = "idle"              # idle | play | after
        self.phase_t = 0.0
        self.shot_no = 0
        self.aim = 0.0
        self.outcome = None              # outcome of the current shot
        self.events = []                 # finished shots not yet collected
        self.hist = deque()              # (t, ball tuple | None, px) truth history

    # -- control input from the server reply -------------------------------
    def command(self, target: float, duty: float) -> None:
        self.target, self.duty = target, max(-1.0, min(1.0, duty))

    # -- physics -----------------------------------------------------------
    def _new_shot(self):
        c = self.cfg
        self.shot_no += 1
        self.aim = self.rng.uniform(-GOAL_HALF_W + 0.01, GOAL_HALF_W - 0.01)
        x0 = self.rng.uniform(-FIELD_W / 2 + 0.05, FIELD_W / 2 - 0.05)
        y0 = FIELD_L - 0.02
        sp = c.speed * (1 + self.rng.uniform(-c.speed_jitter, c.speed_jitter))
        dx, dy = self.aim - x0, -y0
        n = math.hypot(dx, dy)
        self.ball = [x0, y0, sp * dx / n, sp * dy / n]
        self.shot_speed = sp
        self.outcome = None
        self.phase, self.phase_t = "play", self.t

    def _finish(self, what: str):
        self.outcome = what
        self.events.append({"shot": self.shot_no, "result": what, "aim": self.aim,
                            "speed": self.shot_speed, "paddle": self.px, "t": self.t})
        self.phase, self.phase_t = "after", self.t

    def _step(self, dt):
        c = self.cfg
        # paddle
        if c.follow == "target":
            want = (self.target - self.px) / c.paddle_tau
            self.pv = max(-c.paddle_vmax, min(c.paddle_vmax, want))
        else:
            self.pv += (self.duty * c.paddle_vmax - self.pv) * dt / c.paddle_tau
        self.px += self.pv * dt
        if abs(self.px) > PADDLE_LIMIT:
            self.px = math.copysign(PADDLE_LIMIT, self.px)
            self.pv = 0.0
        # ball
        if self.phase == "idle":
            if self.t - self.phase_t >= c.gap_s:
                self._new_shot()
            return
        b = self.ball
        y_prev = b[1]
        b[0] += b[2] * dt
        b[1] += b[3] * dt
        wall = FIELD_W / 2 - BALL_R
        if abs(b[0]) > wall:
            b[0] = math.copysign(2 * wall - abs(b[0]), b[0])
            b[2] = -b[2]
        if self.phase == "play":
            line = PADDLE_Y + BALL_R
            if b[3] < 0 and y_prev >= line > b[1] and abs(b[0] - self.px) <= PADDLE_HALF + BALL_R:
                b[1] = 2 * line - b[1]
                b[3] = -b[3] * RESTITUTION
                b[2] += 0.3 * self.pv                      # paddle drags the ball sideways
                self._finish("SAVE")
            elif y_prev >= 0 > b[1]:
                self._finish("GOAL" if abs(b[0]) <= GOAL_HALF_W else "WIDE")
            elif b[1] > FIELD_L + 0.05 or self.t - self.phase_t > 4.0:
                self._finish("STOPPED")
        else:  # after
            if b[1] < -0.05:                               # in the net
                b[1], b[2], b[3] = -0.05, 0.0, 0.0
            if self.t - self.phase_t >= c.after_s:
                self.ball, self.phase, self.phase_t = None, "idle", self.t

    def step_to(self, t: float) -> None:
        """Advance the simulation to time t (seconds since start)."""
        while self.t < t:
            dt = min(SUBSTEP, t - self.t)
            self._step(dt)
            self.t += dt
            self.hist.append((self.t, tuple(self.ball[:2]) if self.ball else None, self.px))
        horizon = self.t - self.cfg.latency_ms / 1000.0 - 0.05
        while len(self.hist) > 2 and self.hist[1][0] < horizon:
            self.hist.popleft()

    # -- camera ------------------------------------------------------------
    def observe(self) -> tuple[tuple[float, float] | None, float | None]:
        """What the camera sees now: state from `latency` ago, plus noise/dropout."""
        c = self.cfg
        want = self.t - c.latency_ms / 1000.0
        snap = self.hist[0] if self.hist else (self.t, None, self.px)
        for h in self.hist:
            if h[0] > want:
                break
            snap = h
        _, ball, px = snap
        n = lambda: self.rng.gauss(0.0, c.noise_m)
        if ball is not None and self.rng.random() >= c.dropout:
            ball = (ball[0] + n(), ball[1] + n())
        else:
            ball = None
        paddle = px + n() if self.rng.random() >= c.dropout else None
        return ball, paddle

    def pop_events(self) -> list[dict]:
        ev, self.events = self.events, []
        return ev


class FakeCamera:
    """Real-time wrapper used by track.py --fake: read() returns (ball, paddle) in metres."""

    def __init__(self, cfg: FakeConfig | None = None):
        self.world = FakeWorld(cfg)
        self.t0 = time.monotonic()
        self.fps = self.world.cfg.fps

    def read(self):
        self.world.step_to(time.monotonic() - self.t0)
        return self.world.observe()

    def command(self, target, duty):
        self.world.command(target, duty)


def run_headless(host: str, port: int, shots: int, cfg: FakeConfig,
                 reset: bool = True, quiet: bool = False, mode: str | None = None) -> dict:
    """Shoot `shots` balls at a live server in real time. Returns a summary dict."""
    from goalie_link import GoalieLink, LinkDown

    say = (lambda *a: None) if quiet else print
    link = GoalieLink(host, port, timeout=0.25, connect_timeout=0.5)
    try:
        if reset:
            link.reset()
        if mode:
            link.mode(mode)
    except LinkDown as e:
        say(f"!! no server at {host}:{port}: {e}")
        return {"ok": False, "error": str(e)}
    cam = FakeCamera(cfg)
    period = 1.0 / cfg.fps
    results, link_errors = [], 0
    nxt = time.monotonic()
    deadline = nxt + shots * (cfg.gap_s + 0.6 / max(cfg.speed, 0.2) * 2 + cfg.after_s) + 10
    while len(results) < shots and time.monotonic() < deadline:
        ball, paddle = cam.read()
        try:
            target, duty = link.frame(int(time.monotonic() * 1000), ball, paddle)
            cam.command(target, duty)
        except (LinkDown, OSError):
            link_errors += 1
            cam.command(0.0, 0.0)
        for ev in cam.world.pop_events():
            results.append(ev)
            if ev["result"] == "GOAL":
                try:
                    link.goal()
                except (LinkDown, OSError):
                    link_errors += 1
            say(f"shot {ev['shot']:3d}: aim {ev['aim']:+.3f} m  {ev['speed']:.2f} m/s  "
                f"-> {ev['result']:<7s} (paddle {ev['paddle']:+.3f})")
        nxt += period
        d = nxt - time.monotonic()
        if d > 0:
            time.sleep(d)
        else:
            nxt = time.monotonic()
    # the server counts a save a few frames after the bounce; keep feeding briefly
    t_end = time.monotonic() + 0.3
    while time.monotonic() < t_end:
        ball, paddle = cam.read()
        try:
            cam.command(*link.frame(int(time.monotonic() * 1000), ball, paddle))
        except (LinkDown, OSError):
            link_errors += 1
        time.sleep(period)
    try:
        st = link.status()
    except (LinkDown, OSError):
        st = {}
    link.close()
    count = lambda r: sum(1 for e in results if e["result"] == r)
    summary = {"ok": True, "shots": len(results), "saves": count("SAVE"), "goals": count("GOAL"),
               "wide": count("WIDE") + count("STOPPED"), "server_saves": st.get("saves"),
               "server_goals": st.get("goals"), "server_mode": st.get("mode"),
               "jit_max_us": st.get("jit_max_us"), "link_errors": link_errors, "results": results}
    say(f"\n{summary['shots']} shots at {cfg.speed:.2f} m/s: {summary['saves']} saves, "
        f"{summary['goals']} goals, {summary['wide']} wide/stopped "
        f"(save rate {summary['saves'] / max(1, summary['shots']):.0%})")
    say(f"server: mode={st.get('mode')} saves={st.get('saves')} goals={st.get('goals')} "
        f"jit_max_us={st.get('jit_max_us')} loop_hz={st.get('loop_hz')}  link errors: {link_errors}")
    if st and (st.get("saves") != summary["saves"] or st.get("goals") != summary["goals"]):
        say("note: server counters differ from the fake-world truth (server save detection is vision-based)")
    return summary


def config_from_args(a) -> FakeConfig:
    return FakeConfig(speed=a.speed, fps=a.fps, latency_ms=a.latency_ms, noise_m=a.noise,
                      dropout=a.dropout, paddle_vmax=a.paddle_vmax, paddle_tau=a.paddle_tau,
                      follow=a.follow, seed=a.seed)


def add_fake_args(ap: argparse.ArgumentParser) -> None:
    g = ap.add_argument_group("fake ball")
    g.add_argument("--speed", type=float, default=1.0, help="shot speed m/s")
    g.add_argument("--fps", type=float, default=30.0)
    g.add_argument("--latency-ms", type=float, default=40.0)
    g.add_argument("--noise", type=float, default=0.003, help="camera noise sigma, m")
    g.add_argument("--dropout", type=float, default=0.02)
    g.add_argument("--paddle-vmax", type=float, default=1.5, help="paddle m/s at duty 1")
    g.add_argument("--paddle-tau", type=float, default=0.06)
    g.add_argument("--follow", choices=["duty", "target"], default="duty",
                   help="paddle obeys the server's duty (default) or chases target_x")
    g.add_argument("--seed", type=int, default=None)


if __name__ == "__main__":
    from goalie_link import DEFAULT_HOST, DEFAULT_PORT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--shots", type=int, default=20)
    ap.add_argument("--mode", choices=["QNX", "LINUX"], default=None, help="send MODE first")
    ap.add_argument("--no-reset", action="store_true", help="don't send RESET first")
    add_fake_args(ap)
    a = ap.parse_args()
    s = run_headless(a.host, a.port, a.shots, config_from_args(a), reset=not a.no_reset, mode=a.mode)
    sys.exit(0 if s.get("ok") else 2)
