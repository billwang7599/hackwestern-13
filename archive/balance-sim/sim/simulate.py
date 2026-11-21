"""Simulation loop: physics at physics_hz, controller at control_hz (with jitter and
IMU latency), drive model in between. Used by run.py (headless/live) and tune.py."""
from __future__ import annotations

import math
import random
from collections import deque

from controller import Controller, RUNNING, FALLEN, STATE_NAMES
from params import DEG, RobotParams, Scenario
from plant import IMU, Plant

GROUND_TILT = 45.0 * DEG   # beyond this we call it on the floor


class Sim:
    def __init__(self, p: RobotParams, gains: dict | None = None, backend="auto",
                 scenario: Scenario | None = None):
        self.p = p
        self.sc = scenario or Scenario()
        self.rng = random.Random(p.seed)
        self.plant = Plant(p)
        self.imu = IMU(p, self.rng)
        ctrl_over = dict(track_width=p.track_width)
        if p.drive == "dc":
            # can't command more speed than the motor's no-load speed
            ctrl_over["max_speed"] = 0.95 * self.plant.w_nl * p.wheel_radius
        ctrl_over.update(gains or {})
        self.ctrl = Controller(backend, ctrl_over)
        self.dt = 1.0 / p.physics_hz
        self.t = 0.0
        self.s = self.plant.initial_state(p.initial_pitch)
        self.lat_n = max(0, round(p.imu_latency * p.physics_hz))
        self.imu_buf = deque(maxlen=self.lat_n + 1)
        self.next_ctrl = 0.0
        self.last_ctrl_t = -1.0 / p.control_hz
        self.cmd = (0.0, 0.0, 0.0, 0.0, 0.0, 0, 0)
        self.speed_sp = 0.0
        self.turn_sp = 0.0
        self.forces = []   # (t0, t1, F)
        for t0, imp in self.sc.pushes:
            self.add_push(imp, t0)
        self.sp_events = sorted(self.sc.setpoints)
        self.armed_at = None
        self.fell_at = None
        self.stall_time = 0.0
        self.slip_time = 0.0
        self.duty = 0.0
        self.log = {k: [] for k in ("t", "pitch", "pitch_est", "pitch_target", "x", "xd",
                                    "v_cmd", "accel", "tau", "duty", "F", "state")}
        self.plant.deriv(self.s)

    # ---------- inputs ----------
    def add_push(self, impulse, t0=None, width=None):
        """Horizontal impulse (N*s) at the top of the slab, + = pushes it forward."""
        w = width or self.sc.push_width
        t0 = self.t if t0 is None else t0
        self.forces.append((t0, t0 + w, impulse / w))

    def force_at(self, t):
        return sum(F for t0, t1, F in self.forces if t0 <= t < t1)

    # ---------- drive glue (what the Pico/QNX firmware does with the command) ----------
    def _apply_drive(self):
        left, right, _a, _pe, _pt, state, on = self.cmd
        p, pl = self.p, self.plant
        v = 0.5 * (left + right)
        pl.motors_on = bool(on) and self.fell_at is None
        if p.drive == "stepper":
            pl.v_step = v
        else:
            v_nl = pl.w_nl * p.wheel_radius
            u = max(-1.0, min(1.0, v / v_nl))          # voltage ~ speed feed-forward
            if abs(u) > 0.005:
                u = math.copysign(min(1.0, abs(u) + p.dc_deadband_comp), u)
            self.duty = u
            pl.duty = 0.0 if abs(u) < p.dc_deadband else u

    def _wheel_speed_meas(self):
        p = self.p
        if p.drive == "dc" and p.dc_encoder:
            return self.s[1] + self.rng.gauss(0.0, p.encoder_noise)
        return None   # stepper: step count == command; dc w/o encoder: nothing

    # ---------- main step ----------
    def step(self):
        p, pl = self.p, self.plant
        t = self.t
        while self.sp_events and self.sp_events[0][0] <= t:
            _, self.speed_sp, self.turn_sp = self.sp_events.pop(0)

        self.imu_buf.append(self.imu.sample(self.s, pl.last["xdd"], pl.last["thdd"]))
        if t >= self.next_ctrl:
            m = self.imu_buf[0]   # oldest = latency ago
            dt = t - self.last_ctrl_t
            self.last_ctrl_t = t
            self.cmd = self.ctrl.step(m["gyro"], m["acc_fwd"], m["acc_up"], m["fused"],
                                      self._wheel_speed_meas(), self.speed_sp, self.turn_sp,
                                      1, dt)
            period = 1.0 / p.control_hz
            self.next_ctrl += max(0.2 * period, period + self.rng.gauss(0.0, p.control_jitter))
            if self.cmd[5] == RUNNING and self.armed_at is None:
                self.armed_at = t
                pl.held = False
            self._apply_drive()

        pl.F = self.force_at(t) if not pl.held else 0.0
        self.s = pl.rk4(self.s, self.dt)
        self.t = t + self.dt
        if pl.last["stall"] and pl.motors_on:
            self.stall_time += self.dt
        if pl.last["slip"]:
            self.slip_time += self.dt
        if self.fell_at is None and (abs(self.s[2]) > GROUND_TILT or self.cmd[5] == FALLEN):
            self.fell_at = self.t

        L = self.log
        L["t"].append(self.t); L["pitch"].append(self.s[2]); L["x"].append(self.s[0])
        L["xd"].append(self.s[1]); L["pitch_est"].append(self.cmd[3])
        L["pitch_target"].append(self.cmd[4]); L["v_cmd"].append(0.5 * (self.cmd[0] + self.cmd[1]))
        L["accel"].append(self.cmd[2]); L["tau"].append(pl.last["tau"])
        L["duty"].append(self.duty); L["F"].append(pl.F); L["state"].append(self.cmd[5])

    def run(self, duration=None, stop_on_fall=True):
        end = duration if duration is not None else self.sc.duration
        n = int(round((end - self.t) / self.dt))
        for _ in range(n):
            self.step()
            if stop_on_fall and self.fell_at is not None and self.t > self.fell_at + 0.3:
                break
        return self

    # ---------- results ----------
    def metrics(self):
        L, p = self.log, self.p
        t, th, xd = L["t"], L["pitch"], L["xd"]
        res = dict(fell=self.fell_at is not None, fell_at=self.fell_at,
                   armed_at=self.armed_at, stall_time=self.stall_time,
                   slip_time=self.slip_time, backend=self.ctrl.backend, pushes=[])
        start = self.armed_at if self.armed_at is not None else 0.0
        idx0 = next((k for k, tt in enumerate(t) if tt >= start), len(t))
        res["max_tilt_deg"] = max((abs(v) for v in th[idx0:]), default=0.0) / DEG
        res["max_x"] = max((abs(v) for v in L["x"][idx0:]), default=0.0)
        res["final_x"] = L["x"][-1] if L["x"] else 0.0
        pushes = sorted(self.forces)
        for k, (t0, t1, F) in enumerate(pushes):
            seg_end = pushes[k + 1][0] if k + 1 < len(pushes) else t[-1]
            i0 = next((j for j, tt in enumerate(t) if tt >= t0), None)
            i1 = next((j for j, tt in enumerate(t) if tt >= seg_end), len(t))
            if i0 is None:
                continue
            # equilibrium pitch = mean pitch over the 0.3 s before the push
            pre = [th[j] for j in range(max(idx0, i0 - int(0.3 / self.dt)), i0)] or [0.0]
            eq = sum(pre) / len(pre)
            peak = max(abs(th[j] - eq) for j in range(i0, i1)) / DEG if i1 > i0 else 0.0
            settled_at = None
            for j in range(i1 - 1, i0 - 1, -1):     # last time it was outside the band
                if abs(th[j] - eq) > 1.5 * DEG or abs(xd[j]) > 0.10:
                    settled_at = t[j + 1] if j + 1 < len(t) else None
                    break
            else:
                settled_at = t[i0]
            fell_here = self.fell_at is not None and t0 <= self.fell_at <= seg_end + 1e-9
            ok = (not fell_here) and settled_at is not None and settled_at < seg_end - 0.05
            res["pushes"].append(dict(t=t0, impulse=F * (t1 - t0), peak_tilt_deg=peak,
                                      settle_s=(settled_at - t0) if settled_at else None,
                                      recovered=ok))
        res["passed"] = (not res["fell"] and self.armed_at is not None
                         and all(pu["recovered"] for pu in res["pushes"])
                         and self.stall_time < 0.02)
        return res


def run_scenario(p, gains=None, scenario=None, backend="auto"):
    s = Sim(p, gains, backend, scenario).run()
    return s.metrics(), s


def survives_push(p, gains, impulse, backend="auto", seed=None):
    import copy
    q = copy.copy(p)
    if seed is not None:
        q.seed = seed
    sc = Scenario(duration=5.0, pushes=[(1.5, impulse)])
    m, _ = run_scenario(q, gains, sc, backend)
    return m["passed"]


def max_push(p, gains=None, backend="auto", hi=1.5, iters=7, both_ways=True):
    """Largest impulse (N*s at the top of the slab) the robot recovers from (bisection)."""
    def ok(imp):
        if not survives_push(p, gains, imp, backend):
            return False
        return (not both_ways) or survives_push(p, gains, -imp, backend)
    if not ok(0.0):
        return 0.0, False   # cannot even stand still
    lo = 0.0
    if ok(hi):
        return hi, True
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo, True


def fmt_metrics(name, m):
    lines = [f"[{name}] {'PASS' if m['passed'] else 'FAIL'}  backend={m['backend']}  "
             f"armed_at={m['armed_at']}  fell={m['fell']}"
             + (f" (t={m['fell_at']:.2f}s)" if m['fell'] else "")]
    lines.append(f"  max tilt {m['max_tilt_deg']:.1f} deg   max |x| {m['max_x']:.2f} m   "
                 f"final x {m['final_x']:+.2f} m   stall {m['stall_time']*1000:.0f} ms"
                 + (f"   slip {m['slip_time']*1000:.0f} ms" if m['slip_time'] else ""))
    for pu in m["pushes"]:
        st = f"{pu['settle_s']:.2f}s" if pu["settle_s"] is not None else "never"
        lines.append(f"  push {pu['impulse']:+.2f} N*s @ {pu['t']:.1f}s: peak {pu['peak_tilt_deg']:.1f} deg, "
                     f"settle {st}, {'recovered' if pu['recovered'] else 'NOT recovered'}")
    return "\n".join(lines)


__all__ = ["Sim", "run_scenario", "max_push", "fmt_metrics", "STATE_NAMES"]
