"""The physical world: a rolling ping-pong ball and the DC-motor goalie.

Ball: rolls in a straight line (hollow sphere rolling without slipping, so the
effective mass is 5/3 m), slowed by rolling resistance and air drag; optional
bounces off the side walls.

Goalie: one DC gearmotor, output-shaft model

    J * dw/dt = stall * (u_eff - w / w_noload) - coulomb * sign(w) - tau_load(q_paddle)

with u_eff = 0 inside the deadband (H-bridge brake), the motor shaft angle qm
integrated from w, and the paddle angle q following qm through a backlash band
(kinematic "play" operator: the paddle stays put until a gear tooth pushes it).

The paddle sits in the goal-line plane y = 0.  A shot is decided when the
ball centre crosses y = 0: saved if it overlaps the paddle there.
"""
from __future__ import annotations

import math

from params import SimParams

G = 9.81


# ---------------------------------------------------------------- geometry
def x_of_q(q: float, gp) -> float:
    """Shaft angle -> paddle x on the goal line (tip centre for the pivot)."""
    if gp.geometry == "pivot":
        return gp.paddle_len * math.sin(q)
    return gp.spool_radius * q


def q_of_x(x: float, gp) -> float:
    """Paddle x -> shaft angle (clamped to what the geometry can reach)."""
    if gp.geometry == "pivot":
        s = max(-1.0, min(1.0, x / gp.paddle_len))
        return math.asin(s)
    return x / gp.spool_radius


def paddle_blocks(q: float, bx: float, p: SimParams) -> bool:
    """Does the paddle at shaft angle q stop a ball whose centre crosses y=0 at bx?"""
    gp, r = p.goalie, p.ball.radius
    if gp.geometry == "rail":
        x = x_of_q(q, gp)
        return abs(bx - x) < gp.paddle_len / 2 + r
    # Pivot: paddle = rectangle in the x-z plane hanging from the pivot.
    # Ball cross-section in that plane = circle radius r centred (bx, r).
    L, W = gp.paddle_len, gp.paddle_w
    pz = L + gp.pivot_gap                          # pivot height above the table
    ax, az = math.sin(q), -math.cos(q)             # along the arm, pivot -> tip
    cx, cz = math.cos(q), math.sin(q)              # across the blade
    dx, dz = bx, r - pz
    s = min(max(dx * ax + dz * az, 0.0), L)
    u = min(max(dx * cx + dz * cz, -W / 2), W / 2)
    ex = dx - (s * ax + u * cx)
    ez = dz - (s * az + u * cz)
    return ex * ex + ez * ez < r * r


# ---------------------------------------------------------------- ball
class Ball:
    def __init__(self, p: SimParams):
        self.p = p
        self.x = self.y = self.vx = self.vy = 0.0
        self.moving = False

    def place(self, x: float, y: float):
        self.x, self.y, self.vx, self.vy, self.moving = x, y, 0.0, 0.0, False

    def shoot(self, speed: float, aim_x: float):
        """Roll from the current spot towards (aim_x, 0) at `speed` m/s (no walls)."""
        dx, dy = aim_x - self.x, -self.y
        n = math.hypot(dx, dy)
        self.vx, self.vy = speed * dx / n, speed * dy / n
        self.moving = True

    def step(self, dt: float):
        if not self.moving:
            return
        b, f = self.p.ball, self.p.field
        v = math.hypot(self.vx, self.vy)
        if v < 1e-3:
            self.vx = self.vy = 0.0
            return
        area = math.pi * b.radius ** 2
        drag = 0.5 * b.air_rho * b.drag_cd * area * v * v / (b.mass * 5.0 / 3.0)
        dv = min(v, (b.roll_decel + drag) * dt)
        k = (v - dv) / v
        self.vx *= k
        self.vy *= k
        self.x += self.vx * dt
        self.y += self.vy * dt
        if f.walls:
            wb = f.field_half_w - b.radius
            if self.x > wb:
                self.x, self.vx = 2 * wb - self.x, -self.vx * f.wall_restitution
            elif self.x < -wb:
                self.x, self.vx = -2 * wb - self.x, -self.vx * f.wall_restitution


# ---------------------------------------------------------------- goalie
class Goalie:
    def __init__(self, p: SimParams, q0: float = 0.0):
        self.p = p
        m, g = p.motor, p.goalie
        self.w0 = m.w_noload
        self.stall = m.stall_torque
        self.coulomb = m.coulomb_frac * self.stall
        if g.geometry == "pivot":
            j_load = g.paddle_mass * (g.paddle_len ** 2 + g.paddle_w ** 2) / 3.0
            self.grav = g.paddle_mass * G * g.paddle_len / 2.0     # * sin(q)
            self.load_fric = 0.0
        else:
            j_load = g.carriage_mass * g.spool_radius ** 2
            self.grav = 0.0
            self.load_fric = g.rail_friction * g.spool_radius
        self.J = m.inertia + j_load
        self.qm = q0          # motor-side shaft angle
        self.q = q0           # paddle angle (after backlash)
        self.w = 0.0
        self.u = 0.0

    @property
    def x(self) -> float:
        return x_of_q(self.q, self.p.goalie)

    def step(self, duty: float, dt: float):
        m, g = self.p.motor, self.p.goalie
        u = max(-1.0, min(1.0, duty))
        self.u = u
        u_eff = 0.0 if abs(u) < m.deadband else u
        drive = self.stall * (u_eff - self.w / self.w0) - self.grav * math.sin(self.q)
        fric = self.coulomb + self.load_fric
        if abs(self.w) < 1e-3 and abs(drive) <= fric:
            self.w = 0.0                      # stiction
        else:
            sgn = 1.0 if (self.w if abs(self.w) >= 1e-3 else drive) > 0 else -1.0
            w_new = self.w + (drive - fric * sgn) / self.J * dt
            if self.w * w_new < 0 and abs(drive) <= fric:
                w_new = 0.0                   # friction stops it, doesn't reverse it
            self.w = w_new
        self.qm += self.w * dt
        if g.geometry == "pivot":
            # hard stops just past the software limit (cardboard hits the posts)
            lim = g.q_max + 10 * math.pi / 180
            if abs(self.qm) > lim:
                self.qm = math.copysign(lim, self.qm)
                self.w = 0.0
        else:
            lim = (g.x_max + 0.01) / g.spool_radius
            if abs(self.qm) > lim:
                self.qm = math.copysign(lim, self.qm)
                self.w = 0.0
        half = m.backlash / 2.0
        self.q = min(max(self.q, self.qm - half), self.qm + half)
