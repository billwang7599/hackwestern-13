"""Paddle position controller.  (Port target: goalie/pi/src/main.c control loop)

Plain scalar code, fixed-size ring buffer: ports line for line to C.

WHY AN OBSERVER
---------------
The motor has no encoder.  The only position feedback is the overhead camera:
30-60 fps and 30-60 ms late.  At 150 RPM the shaft turns ~0.8 rad in 50 ms,
i.e. by the time the camera shows the paddle at the target it has already
overshot by a whole goal width.  A PD loop straight on camera data has to be
detuned until it's far too slow.  So the Pi runs a tiny motor model at 200 Hz
(it knows every duty it sent), PD-controls the *model*, and nudges the model
towards the camera whenever a frame arrives, comparing the camera with what
the model said *latency_est seconds ago* (Smith-predictor style).

Everything is in SHAFT ANGLE q (rad).  Geometry (architecture.md):
    pivot: x = L*sin(q),  q = asin(clamp(x/L, -1, 1))     L = paddle length
    rail:  x = R*q,       q = x/R                          R = spool radius

ALGORITHM (every tick, dt = 1/RATE = 5 ms)
---------
State: q_est, w_est (rad, rad/s), u_prev (last duty sent), hist[HIST] ring of
q_est per tick (HIST = 64 ticks = 320 ms >= latency), initialised = false.

tick(target_x):
  1. Model step with the duty sent last tick:
        u_eff = (|u_prev| < DB_COMP) ? 0 : sign(u_prev) * max(|u_prev| - UOFF, 0)
              (UOFF = duty eaten by gearbox friction; without it the model
               runs ahead of the real motor at small duties)
        w_est += (W0 * u_eff - w_est) * dt / TAU      W0 = MODEL_RPM * 2pi/60
        q_est += w_est * dt;  clamp q_est to +-Q_LIM  (Q_LIM = q_max + 10deg for pivot)
        push q_est into hist
  2. q_t = q_of_x(clamp(target_x, -X_MAX, X_MAX))
     u = KP * (q_t - q_est) - KD * w_est
     if |u| < MIN_DUTY: u = 0
     else:              u += sign(u) * DB_COMP           (deadband compensation)
     u = clamp(u, -1, 1);  u_prev = u;  send "M <u>"

on_paddle(x_meas)  (each camera frame with paddle_seen = 1, applied before the tick):
     q_m = q_of_x(x_meas)
     if not initialised: q_est = q_m, w_est = 0, fill hist with q_m, initialised = true; return
     k = round(LATENCY_EST / dt), clamp to [0, HIST-1]
     innov = q_m - hist[k ticks ago]
     q_est += L1 * innov;  w_est += L2 * innov
     add L1 * innov to every hist entry (so the same error isn't corrected twice)

Defaults (params.ControlParams, picked with tune.py): KP 3.0 duty/rad, KD 0.10 duty/(rad/s),
DB_COMP 0.12, MIN_DUTY 0.06, UOFF 0.10, L1 0.5, L2 2.0 1/s, LATENCY_EST 0.05 s,
MODEL_RPM = measured no-load RPM, TAU 0.04 s (see run.py --help for retuning).
If the paddle hasn't been seen for a while the model runs open loop; the Pi's
watchdog (architecture.md) decides when to give up and send S.
"""
from __future__ import annotations

import math

RPM = 2.0 * math.pi / 60.0
HIST = 64


class Controller:
    def __init__(self, cfg, goalie, x_max: float):
        self.c = cfg
        self.g = goalie
        self.x_max = x_max
        self.dt = 1.0 / cfg.rate
        if goalie.geometry == "pivot":
            self.q_lim = goalie.q_max + 10 * math.pi / 180
        else:
            self.q_lim = (goalie.x_max + 0.01) / goalie.spool_radius
        self.hist = [0.0] * HIST
        self.head = 0                # index of the newest entry
        self.q_est = 0.0
        self.w_est = 0.0
        self.u_prev = 0.0
        self.init = False
        self.q_t = 0.0

    # geometry (same as plant.x_of_q / q_of_x)
    def q_of_x(self, x: float) -> float:
        if self.g.geometry == "pivot":
            return math.asin(max(-1.0, min(1.0, x / self.g.paddle_len)))
        return x / self.g.spool_radius

    def x_of_q(self, q: float) -> float:
        if self.g.geometry == "pivot":
            return self.g.paddle_len * math.sin(q)
        return self.g.spool_radius * q

    @property
    def x_est(self) -> float:
        return self.x_of_q(self.q_est)

    def on_paddle(self, x_meas: float):
        c = self.c
        qm = self.q_of_x(x_meas)
        if not self.init:
            self.q_est, self.w_est = qm, 0.0
            self.hist = [qm] * HIST
            self.init = True
            return
        k = min(max(int(round(c.latency_est / self.dt)), 0), HIST - 1)
        innov = qm - self.hist[(self.head - k) % HIST]
        self.q_est += c.obs_l1 * innov
        self.w_est += c.obs_l2 * innov
        corr = c.obs_l1 * innov
        for i in range(HIST):
            self.hist[i] += corr

    def tick(self, target_x: float) -> float:
        c, dt = self.c, self.dt
        up = self.u_prev
        u_eff = 0.0 if abs(up) < c.db_comp else math.copysign(max(abs(up) - c.model_uoff, 0.0), up)
        w0 = c.model_rpm * RPM
        self.w_est += (w0 * u_eff - self.w_est) * dt / c.model_tau
        self.q_est += self.w_est * dt
        if abs(self.q_est) > self.q_lim:
            self.q_est = math.copysign(self.q_lim, self.q_est)
            self.w_est = 0.0
        self.head = (self.head + 1) % HIST
        self.hist[self.head] = self.q_est

        self.q_t = self.q_of_x(min(max(target_x, -self.x_max), self.x_max))
        u = c.kp * (self.q_t - self.q_est) - c.kd * self.w_est
        if abs(u) < c.min_duty:
            u = 0.0
        else:
            u += math.copysign(c.db_comp, u)
        u = min(max(u, -1.0), 1.0)
        self.u_prev = u
        return u
