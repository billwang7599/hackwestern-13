"""Glue: camera model + 200 Hz Pi loop + 1 kHz physics, one shot at a time.

Timeline of a shot:
  0 .. pre_roll       ball held still at (x0, field_len), in view; goalie shadows it
  pre_roll            ball released at `speed` towards (aim, 0)
  ball centre crosses y = 0  -> "save" (paddle overlaps it), "goal" (inside the
                                posts) or "wide"; if it stops first -> "stopped"

Camera: a frame is captured every 1/fps s with Gaussian position noise on the
ball and on the paddle tip, and is handed to the Pi `latency` s later (that's
when predictor/observer see it).  The Pi loop runs every 1/rate s and the duty
it sends is held until the next tick.
"""
from __future__ import annotations

import copy
import random
from collections import deque
from dataclasses import dataclass

from controller import Controller
from params import SimParams
from plant import Ball, Goalie, paddle_blocks, q_of_x
from predict import Predictor


@dataclass
class ShotResult:
    outcome: str           # save | goal | wide | stopped
    speed: float
    aim: float
    x_cross: float = float("nan")    # ball x where it crossed the goal line
    paddle_x: float = float("nan")   # paddle tip x at that moment
    t_flight: float = float("nan")   # release -> crossing


def tuned(p: SimParams) -> SimParams:
    """Give the Pi's model the motor's real numbers (as if RPM was measured)."""
    p = copy.deepcopy(p)
    m, g = p.motor, p.goalie
    p.control.model_rpm = m.rpm
    if g.geometry == "pivot":
        j_load = g.paddle_mass * (g.paddle_len ** 2 + g.paddle_w ** 2) / 3.0
    else:
        j_load = g.carriage_mass * g.spool_radius ** 2
    p.control.model_tau = (m.inertia + j_load) * m.w_noload / m.stall_torque
    return p


class Sim:
    def __init__(self, p: SimParams, seed: int | None = None):
        self.p = p
        self.rng = random.Random(p.seed if seed is None else seed)
        self.ball = Ball(p)
        self.goalie = Goalie(p)
        gp = p.goalie
        self.pred = Predictor(p.control, p.field.goal_half_w, gp.x_max, p.field.walls,
                              p.field.field_half_w, p.ball.radius)
        self.ctl = Controller(p.control, gp, gp.x_max)
        self.t = 0.0
        self.n = 0                        # plant steps
        self.ticks_per_ctl = max(1, int(round(1.0 / (p.control.rate * p.dt))))
        self.next_frame = 0.0
        self.pending: deque = deque()     # (t_arrive, t_cap, bseen, bx, by, pseen, px)
        self.duty = 0.0
        self.target = 0.0
        self.saves = self.goals = 0
        self.last_result: ShotResult | None = None
        self.shot: tuple[float, float, float] | None = None   # (t_release, speed, aim)
        self.trail: deque = deque(maxlen=60)
        self.log: list | None = None      # set to [] to record (t, bx, by, target, x_est, x, duty)

    # ---------------------------------------------------------------- camera
    def _capture(self):
        c, b = self.p.camera, self.ball
        g = self.rng.gauss
        bseen = self.rng.random() >= c.drop
        pseen = self.rng.random() >= c.drop
        bx, by = b.x + g(0, c.noise), b.y + g(0, c.noise)
        px = self.goalie.x + g(0, c.noise)
        self.pending.append((self.t + c.latency, self.t, bseen, bx, by, pseen, px))

    # ---------------------------------------------------------------- Pi loop
    def _control(self):
        while self.pending and self.pending[0][0] <= self.t + 1e-12:
            _, tc, bseen, bx, by, pseen, px = self.pending.popleft()
            self.pred.update(tc, bseen, bx, by, t_now=self.t)
            if pseen:
                self.ctl.on_paddle(px)
        self.target = self.pred.target(self.t)
        self.duty = self.ctl.tick(self.target)
        if self.log is not None:
            self.log.append((self.t, self.ball.x, self.ball.y, self.target,
                             self.ctl.x_est, self.goalie.x, self.duty))

    # ---------------------------------------------------------------- one step
    def step(self) -> ShotResult | None:
        p = self.p
        if self.t >= self.next_frame - 1e-12:
            self._capture()
            self.next_frame += 1.0 / p.camera.fps
        if self.n % self.ticks_per_ctl == 0:
            self._control()
        y_prev, x_prev = self.ball.y, self.ball.x
        self.ball.step(p.dt)
        self.goalie.step(self.duty, p.dt)
        self.t += p.dt
        self.n += 1
        if self.n % 10 == 0:
            self.trail.append((self.ball.x, self.ball.y))
        res = None
        if self.shot and y_prev > 0.0 >= self.ball.y:
            f = y_prev / (y_prev - self.ball.y)
            xc = x_prev + f * (self.ball.x - x_prev)
            t0, speed, aim = self.shot
            r = ShotResult("", speed, aim, xc, self.goalie.x, self.t - t0)
            if abs(xc) > p.field.goal_half_w:
                r.outcome = "wide"
            elif paddle_blocks(self.goalie.q, xc, p):
                r.outcome = "save"
                self.saves += 1
                self.ball.y = -self.ball.y
                self.ball.vy = -0.4 * self.ball.vy      # bounce back out (for the live view)
                self.ball.vx *= 0.5
            else:
                r.outcome = "goal"
                self.goals += 1
            self.shot = None
            self.last_result = res = r
        elif self.shot and not (self.ball.vx or self.ball.vy):
            t0, speed, aim = self.shot
            self.shot = None
            self.last_result = res = ShotResult("stopped", speed, aim)
        return res

    def place(self, x0: float, y0: float | None = None):
        self.ball.place(x0, self.p.field.field_len if y0 is None else y0)
        self.shot = None

    def shoot(self, speed: float, aim: float):
        self.ball.shoot(speed, aim)
        self.shot = (self.t, speed, aim)
        self.trail.clear()


def run_shot(p: SimParams, speed: float, x0: float, aim: float, seed: int,
             log: list | None = None) -> ShotResult:
    s = Sim(p, seed)
    s.log = log
    q0 = q_of_x(0.0, p.goalie)
    s.goalie.q = s.goalie.qm = q0
    s.place(x0)
    while s.t < p.pre_roll:
        s.step()
    s.shoot(speed, aim)
    t_end = s.t + 4.0
    while s.t < t_end:
        r = s.step()
        if r:
            return r
    return ShotResult("stopped", speed, aim)


def shot_set(n: int, p: SimParams, seed: int = 1234, aim_mode: str = "uniform"):
    """n reproducible (x0, aim, seed) tuples: release spot across the far end,
    aim point stratified across the goal mouth (ball fully inside the posts).
    aim_mode "corners": only the outer half of each side (what a human aims for)."""
    rng = random.Random(seed)
    gh = p.field.goal_half_w - p.ball.radius
    out = []
    for i in range(n):
        x0 = rng.uniform(-0.15, 0.15)
        f = (i + rng.random()) / n                           # stratified in [0, 1)
        if aim_mode == "corners":
            side = 1.0 if i % 2 else -1.0
            aim = side * gh * (0.5 + 0.5 * f)
        else:
            aim = -gh + 2 * gh * f
        out.append((x0, aim, rng.randrange(1 << 30)))
    return out


def save_rate(p: SimParams, speed: float, n: int = 60, seed: int = 1234,
              aim_mode: str = "uniform"):
    """Fraction of on-target shots saved (and all ShotResults)."""
    res = [run_shot(p, speed, x0, aim, sd) for x0, aim, sd in shot_set(n, p, seed, aim_mode)]
    on = [r for r in res if r.outcome in ("save", "goal")]
    if not on:
        return 0.0, res
    return sum(r.outcome == "save" for r in on) / len(on), res

