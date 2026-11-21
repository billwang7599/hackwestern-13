"""Where will the ball cross the goal line?  (Port target: goalie/pi/src/predict.c)

Written as plain scalar code on purpose: no numpy, fixed-size buffers, one
float per field, so it ports line for line to C.

ALGORITHM
---------
Inputs, once per camera frame:  update(t, seen, x, y)
    t    capture time of the frame in seconds (in C: the laptop's t_ms / 1000.0
         from the F line; only differences between frames are used, so the
         laptop's clock is fine)
    seen 0/1, x, y ball centre in field metres (architecture.md coordinates)

State: ring buffer of the last WINDOW (=5) *seen* samples (t, x, y), plus
t_last_seen and the last computed target.

update():
  1. If not seen: do nothing (the buffer is kept; see target()).
  2. If seen and the buffer is non-empty and t - t_newest > STALE_GAP (0.15 s),
     clear the buffer first (the old samples belong to another roll).
  3. Push (t, x, y); drop the oldest if more than WINDOW.
  4. Recompute the target (below) and store it.

Recompute:
  n = samples in buffer.
  a. n == 0              -> target = 0 (park centre).
  b. n < MIN_SAMPLES (3) -> target = clamp(SHADOW_GAIN * x_newest)
  c. Least-squares straight-line fit over the buffer, times relative to the
     newest sample (tau_i = t_i - t_newest <= 0):
        tm = mean(tau), xm = mean(x), ym = mean(y)
        Stt = sum((tau-tm)^2)                       (if Stt < 1e-9 -> case b)
        vx = sum((tau-tm)(x-xm)) / Stt,  vy = sum((tau-tm)(y-ym)) / Stt
        x0 = xm - vx*tm,  y0 = ym - vy*tm            (fitted position "now-ish")
  d. If vy >= VY_IN (-0.15 m/s): not incoming (rolling away, held, stopped).
     A still ball's 2 mm noise fits to |vy| up to ~0.05 m/s, so VY_IN must be
     well above that.   -> target = clamp(SHADOW_GAIN * x0)  (shadow it, half way)
  e. Incoming:  t_hit = max(y0, 0) / (-vy)    (seconds after the newest frame;
                                                deceleration is ignored: it
                                                changes timing, not the line)
        if t_hit > T_HIT_MAX (2 s): treat as case d (creeping ball, a long
        extrapolation would just amplify noise)
        xc = x0 + vx * t_hit
        if WALLS: fold xc into [-WB, WB], WB = FIELD_HALF_W - BALL_R, as a
           mirror (triangle wave):
              u = fmod(xc + WB, 4*WB); if u < 0: u += 4*WB
              if u > 2*WB: u = 4*WB - u
              xc = u - WB
        target = clamp(xc)
  clamp(v) = min(max(v, -X_LIM), X_LIM), X_LIM = min(GOAL_HALF_W, paddle x_max).
  A shot aimed wide of the goal is clamped to the post: harmless.

target(t_now):  if t_now - t_last_seen > UNSEEN (0.25 s): clear the buffer and
                return 0 (park centre). Otherwise return the stored target.
                In C: t_now and t_last_seen must be on the same clock: use the
                Pi's receive time for both, not the laptop's t_ms.

Constants (params.ControlParams): WINDOW 5, MIN_SAMPLES 3, VY_IN -0.15,
SHADOW_GAIN 0.5, UNSEEN 0.25 s, STALE_GAP 0.15 s, T_HIT_MAX 2.0 s.
"""
from __future__ import annotations

import math


class Predictor:
    def __init__(self, cfg, goal_half_w: float, x_max: float,
                 walls: bool = False, field_half_w: float = 0.25, ball_r: float = 0.02):
        self.c = cfg
        self.x_lim = min(goal_half_w, x_max)
        self.walls = walls
        self.wb = field_half_w - ball_r
        self.buf: list[tuple[float, float, float]] = []   # (t, x, y), oldest first
        self.t_last_seen = -1e9          # on the caller's "now" clock
        self.tgt = 0.0
        # for display / telemetry
        self.incoming = False
        self.x_cross = 0.0
        self.t_hit = 0.0
        self.vx = self.vy = 0.0

    def clamp(self, v: float) -> float:
        return min(max(v, -self.x_lim), self.x_lim)

    def update(self, t: float, seen: bool, x: float, y: float, t_now: float | None = None):
        if not seen:
            return
        self.t_last_seen = t if t_now is None else t_now
        if self.buf and t - self.buf[-1][0] > self.c.stale_gap_s:
            self.buf.clear()
        self.buf.append((t, x, y))
        if len(self.buf) > self.c.window:
            self.buf.pop(0)
        self.tgt = self._recompute()

    def target(self, t_now: float) -> float:
        if t_now - self.t_last_seen > self.c.unseen_s:
            self.buf.clear()
            self.incoming = False
            self.tgt = 0.0
        return self.tgt

    def _recompute(self) -> float:
        c = self.c
        n = len(self.buf)
        self.incoming = False
        if n == 0:
            return 0.0
        tn, xn, yn = self.buf[-1]
        if n < c.min_samples:
            return self.clamp(c.shadow_gain * xn)
        tm = xm = ym = 0.0
        for t, x, y in self.buf:
            tm += t - tn
            xm += x
            ym += y
        tm /= n
        xm /= n
        ym /= n
        stt = sxt = syt = 0.0
        for t, x, y in self.buf:
            d = (t - tn) - tm
            stt += d * d
            sxt += d * (x - xm)
            syt += d * (y - ym)
        if stt < 1e-9:
            return self.clamp(c.shadow_gain * xn)
        vx, vy = sxt / stt, syt / stt
        x0, y0 = xm - vx * tm, ym - vy * tm
        self.vx, self.vy = vx, vy
        if vy >= c.vy_in:
            return self.clamp(c.shadow_gain * x0)
        t_hit = max(y0, 0.0) / (-vy)
        if t_hit > c.t_hit_max:
            return self.clamp(c.shadow_gain * x0)
        xc = x0 + vx * t_hit
        if self.walls:
            wb = self.wb
            u = math.fmod(xc + wb, 4 * wb)
            if u < 0:
                u += 4 * wb
            if u > 2 * wb:
                u = 4 * wb - u
            xc = u - wb
        self.incoming = True
        self.x_cross = xc
        self.t_hit = t_hit
        return self.clamp(xc)
