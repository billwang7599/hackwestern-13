"""Planar wheeled inverted pendulum + drivetrain + IMU model.

Generalized coordinates: x (wheel/axle position, m), th (body pitch from vertical,
+ = leaning forward). The body CoM sits com_height above the axle, rotated forward
by atan(com_offset/com_height). Wheels roll without slipping (optional traction cap
for steppers). Lagrange equations (standard WIP model):

    A*xdd + B*thdd = tau/r + F + m*lc*sin(thc)*thd^2 - c*xd
    B*xdd + J*thdd = m*g*lc*sin(thc) + F*h*cos(th) - tau

    A = M_wheels + I_wheels/r^2 + m,  B = m*lc*cos(thc),  J = I_body + m*lc^2

tau = total drive torque on the wheels (the body gets -tau), F = horizontal push
at height h on the slab.

Drivetrains:
  stepper  the rotor tracks the commanded step-rate (stiff velocity servo). The
           torque that takes is computed by inverse dynamics and checked against
           the pull-out torque curve; above it the motor STALLS (loses steps) and
           only the limit torque is applied.
  dc       motor-side inertia with its own state (angle, speed relative to the
           body), torque-speed line, Coulomb/static friction, PWM deadband, and a
           gearbox with backlash + compliance between motor and wheel.
           (Small approximation: the motor-side inertia ignores the body's angular
           acceleration; it is tiny next to the body inertia.)

State vector: [x, xd, th, thd, psi_m, w_m]  (psi_m, w_m only used by dc)
"""
from __future__ import annotations

import math
import random

from params import RobotParams

TWO_PI = 2.0 * math.pi


class Plant:
    def __init__(self, p: RobotParams):
        self.p = p
        m = p.body_mass
        self.m = m
        self.lc = math.hypot(p.com_height, p.com_offset)
        self.delta = math.atan2(p.com_offset, p.com_height)
        Iw = 0.5 * p.wheel_mass * p.wheel_radius ** 2
        self.A = p.wheel_mass + Iw / p.wheel_radius ** 2 + m
        self.J = p.inertia() + m * self.lc ** 2
        self.r = p.wheel_radius
        self.h_push = p.body_height
        self.M_tot = p.wheel_mass + m
        # dc
        self.w_nl = p.dc_noload_rpm / 60.0 * TWO_PI
        self.Jm = 2.0 * p.dc_rotor_inertia
        self.kg = 2.0 * p.dc_gear_stiffness
        self.cg = 2.0 * 0.5 * math.sqrt(p.dc_gear_stiffness * p.dc_rotor_inertia)  # ~0.5 crit
        # inputs (held constant over a physics step)
        self.v_step = 0.0     # stepper: commanded ground speed (m/s)
        self.duty = 0.0       # dc: effective duty after deadband handling
        self.motors_on = False
        self.held = True      # hand/kickstand holds the body until armed
        self.F = 0.0          # push force, N
        # diagnostics from the last derivative evaluation
        self.last = {}

    def initial_state(self, pitch: float):
        psi_w = -pitch        # wheel angle relative to body (x = 0)
        return [0.0, 0.0, pitch, 0.0, psi_w, 0.0]

    # ---------- dynamics ----------
    def _forward(self, tau, sc, cc, thd, xd, th):
        """Solve the 2x2 system for (xdd, thdd) given drive torque tau."""
        p = self.p
        B = self.m * self.lc * cc
        r1 = tau / self.r + self.F + self.m * self.lc * sc * thd * thd - p.rolling_damping * xd
        r2 = self.m * p.gravity * self.lc * sc + self.F * self.h_push * math.cos(th) - tau
        det = self.A * self.J - B * B
        xdd = (r1 * self.J - B * r2) / det
        thdd = (self.A * r2 - B * r1) / det
        return xdd, thdd

    def _thdd_given_xdd(self, a, tau, sc, th):
        p = self.p
        r2 = self.m * p.gravity * self.lc * sc + self.F * self.h_push * math.cos(th) - tau
        return (r2 - self.m * self.lc * math.cos(th + self.delta) * a) / self.J

    def _tau_for_accel(self, a, sc, cc, thd, xd, th):
        p = self.p
        B = self.m * self.lc * cc
        G = self.m * p.gravity * self.lc * sc + self.F * self.h_push * math.cos(th)
        C = self.m * self.lc * sc * thd * thd
        d = p.rolling_damping * xd
        return ((self.A - B * B / self.J) * a + (B / self.J) * G - self.F - C + d) / (1.0 / self.r + B / self.J)

    def _ground_force(self, xdd, thdd, sc, cc, thd):
        return self.M_tot * xdd + self.m * self.lc * (cc * thdd - sc * thd * thd) - self.F

    def _normal(self, thdd, sc, cc, thd):
        return self.M_tot * self.p.gravity - self.m * self.lc * (sc * thdd + cc * thd * thd)

    def deriv(self, s):
        p = self.p
        x, xd, th, thd, pm, wm = s
        thc = th + self.delta
        sc, cc = math.sin(thc), math.cos(thc)
        stall = slip = False
        tau = 0.0
        dpm = dwm = 0.0

        if p.drive == "stepper":
            if self.motors_on:
                a = p.stepper_stiffness * (self.v_step - xd)
                tau = self._tau_for_accel(a, sc, cc, thd, xd, th)
                w_rel = xd / self.r - thd
                lim = 2.0 * p.stepper_torque * max(0.05, 1.0 - abs(w_rel) / (TWO_PI * p.stepper_max_rps))
                if abs(tau) > lim:
                    stall = True
                    tau = math.copysign(lim, tau)
                    xdd, thdd = self._forward(tau, sc, cc, thd, xd, th)
                else:
                    xdd = a
                    thdd = self._thdd_given_xdd(a, tau, sc, th)
                if p.wheel_slip:
                    fg = self._ground_force(xdd, thdd, sc, cc, thd)
                    fmax = p.mu * max(0.0, self._normal(thdd, sc, cc, thd))
                    if abs(fg) > fmax:
                        slip = True
                        # F_g is linear in a: find the accel that sits on the friction limit
                        a0 = 0.0
                        t0 = self._tau_for_accel(a0, sc, cc, thd, xd, th)
                        th0 = self._thdd_given_xdd(a0, t0, sc, th)
                        f0 = self._ground_force(a0, th0, sc, cc, thd)
                        k = (fg - f0) / xdd if abs(xdd) > 1e-9 else 0.0
                        if abs(k) > 1e-9:
                            xdd = (math.copysign(fmax, fg) - f0) / k
                            tau = self._tau_for_accel(xdd, sc, cc, thd, xd, th)
                            thdd = self._thdd_given_xdd(xdd, tau, sc, th)
            else:
                xdd, thdd = self._forward(0.0, sc, cc, thd, xd, th)
        else:  # dc
            psi_w = x / self.r - th
            psi_wd = xd / self.r - thd
            gap = pm - psi_w
            gapd = wm - psi_wd
            b2 = 0.5 * p.dc_backlash
            if gap > b2:
                tau = max(0.0, self.kg * (gap - b2) + self.cg * gapd)
            elif gap < -b2:
                tau = min(0.0, self.kg * (gap + b2) + self.cg * gapd)
            u = self.duty if self.motors_on else 0.0
            tau_m = 2.0 * p.dc_stall_torque * (u - wm / self.w_nl)
            if not self.motors_on:
                tau_m = 0.0   # coasting (driver outputs off)
            tau_f = 2.0 * p.dc_coulomb * math.tanh(wm / 0.3)
            dpm = wm
            dwm = (tau_m - tau_f - tau) / self.Jm
            xdd, thdd = self._forward(tau, sc, cc, thd, xd, th)

        if self.held:
            # body held by hand: pivot fixed, wheels fixed too
            xdd = -xd * 50.0
            thdd = -thd * 50.0
            dpm, dwm = 0.0, -wm * 50.0

        self.last = dict(xdd=xdd, thdd=thdd, tau=tau, stall=stall, slip=slip)
        return [xd, xdd, thd, thdd, dpm, dwm]

    def rk4(self, s, dt):
        k1 = self.deriv(s)
        k2 = self.deriv([s[i] + 0.5 * dt * k1[i] for i in range(6)])
        k3 = self.deriv([s[i] + 0.5 * dt * k2[i] for i in range(6)])
        k4 = self.deriv([s[i] + dt * k3[i] for i in range(6)])
        out = [s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(6)]
        self.deriv(out)  # refresh self.last (accelerations for the IMU) at the new state
        return out


class IMU:
    """MPU6050-style raw IMU (or BNO085-style fused angle) at imu_height on the slab."""

    def __init__(self, p: RobotParams, rng: random.Random):
        self.p = p
        self.rng = rng

    def sample(self, s, xdd, thdd):
        p = self.p
        x, xd, th, thd = s[0], s[1], s[2], s[3]
        h = p.imu_height
        # acceleration of the IMU point, world frame (x fwd, z up)
        ax = xdd + h * math.cos(th) * thdd - h * math.sin(th) * thd * thd
        az = -h * math.sin(th) * thdd - h * math.cos(th) * thd * thd
        fx, fz = ax, az + p.gravity               # specific force
        tm = th + p.imu_mount_error                 # sensor frame is tilted a bit
        acc_fwd = fx * math.cos(tm) - fz * math.sin(tm)
        acc_up = fx * math.sin(tm) + fz * math.cos(tm)
        g = self.rng.gauss
        return dict(
            gyro=thd + p.gyro_bias + g(0.0, p.gyro_noise),
            acc_fwd=acc_fwd + g(0.0, p.accel_noise),
            acc_up=acc_up + g(0.0, p.accel_noise),
            fused=tm + g(0.0, p.fused_noise),
        )
