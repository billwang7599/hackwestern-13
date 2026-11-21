#!/usr/bin/env python3
"""C vs Python parity check for the predictor + controller.

Runs real closed-loop shots in goalie/sim (Python Predictor + Controller driving
the simulated motor), records every call the Pi loop makes (ball sample,
paddle sample, tick) and what Python answered, then replays the same event
stream through the C code (tools/parity_harness.c, built from src/predict.c +
src/control.c, i.e. what goalie_motor runs) and compares target and duty per
tick.

    make parity                         # from goalie/pi (uses ../sim/.venv if present)
    python3 tools/parity.py --shots 20 --speed 1.5 --walls

Needs the sim's deps (numpy, matplotlib). Exit 0 = max differences under tolerance.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PI = os.path.dirname(HERE)
SIM = os.path.join(PI, "..", "sim")
sys.path.insert(0, os.path.abspath(SIM))

from params import SimParams  # noqa: E402
from plant import q_of_x  # noqa: E402
from simulate import Sim, shot_set, tuned  # noqa: E402

HARNESS = os.path.join(PI, "build", "parity_harness")


class Recorder:
    """Wraps Predictor + Controller, logs calls and Python outputs."""

    def __init__(self, sim: Sim, ev: list, want: list):
        self.sim, self.ev, self.want = sim, ev, want
        self.pred, self.ctl = sim.pred, sim.ctl
        sim.pred, sim.ctl = self, self       # Sim calls update/target/on_paddle/tick on us
        self.t_now = 0.0
        self.pending_target = 0.0

    # predictor side
    def update(self, t, seen, x, y, t_now=None):
        self.pred.update(t, seen, x, y, t_now=t_now)
        if seen:
            self.ev.append(f"B {t!r} {x!r} {y!r} {t_now!r}")
        self._last_tc = t

    def target(self, t_now):
        self.t_now = t_now
        return self.pred.target(t_now)

    # controller side
    def on_paddle(self, px):
        self.ctl.on_paddle(px)
        self.ev.append(f"P {px!r} {getattr(self, '_last_tc', 0.0)!r}")

    def tick(self, target):
        u = self.ctl.tick(target)
        self.ev.append(f"T {self.t_now!r}")
        self.want.append((target, u, self.ctl.x_est))
        return u

    @property
    def x_est(self):
        return self.ctl.x_est


def run(args) -> int:
    p = SimParams()
    p.motor.rpm = args.rpm
    p.field.walls = args.walls
    p.camera.drop = args.drop
    p = tuned(p)
    c, g = p.control, p.goalie
    ev = [f"C {c.model_rpm!r} {c.model_tau!r} {c.latency_est!r} "
          f"{p.field.field_half_w if p.field.walls else 0.0!r} "
          f"{1 if g.geometry == 'rail' else 0} {g.paddle_len!r} {g.spool_radius!r}"]
    want: list = []
    outcomes = []
    for x0, aim, seed in shot_set(args.shots, p, args.seed):
        s = Sim(p, seed)
        Recorder(s, ev, want)
        q0 = q_of_x(0.0, g)
        s.goalie.q = s.goalie.qm = q0
        s.place(x0)
        while s.t < p.pre_roll:
            s.step()
        s.shoot(args.speed, aim)
        t_end = s.t + 4.0
        r = None
        while s.t < t_end and r is None:
            r = s.step()
        outcomes.append(r.outcome if r else "stopped")
        # each shot is a fresh Sim: restart the C side too
        ev.append(ev[0])

    if not os.path.exists(HARNESS):
        print(f"missing {HARNESS}; run `make parity` (builds it)", file=sys.stderr)
        return 2
    # split on config lines: one C run per shot keeps state identical to a fresh Sim
    out = subprocess.run([HARNESS], input="\n".join(ev) + "\n", capture_output=True,
                         text=True, check=True).stdout.split("\n")
    got = [tuple(float(v) for v in ln.split()) for ln in out if ln.strip()]
    if len(got) != len(want):
        print(f"FAIL: tick count C={len(got)} Python={len(want)}")
        return 1
    dt = max(abs(a[0] - b[0]) for a, b in zip(got, want))
    du = max(abs(a[1] - b[1]) for a, b in zip(got, want))
    dx = max(abs(a[2] - b[2]) for a, b in zip(got, want))
    n_bad = sum(abs(a[1] - b[1]) > args.tol for a, b in zip(got, want))
    saves = outcomes.count("save")
    goals = outcomes.count("goal")
    print(f"parity: {args.shots} shots @ {args.speed} m/s, rpm={args.rpm:g}, walls={args.walls}, "
          f"drop={args.drop}: sim outcome {saves} saves / {goals} goals / "
          f"{len(outcomes) - saves - goals} other")
    print(f"  {len(want)} ticks compared: max |target diff| {dt:.2e} m, "
          f"max |duty diff| {du:.2e}, max |x_est diff| {dx:.2e} m, ticks over tol: {n_bad}")
    ok = dt < args.tol and du < args.tol and dx < args.tol
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", type=int, default=12)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--rpm", type=float, default=150.0)
    ap.add_argument("--walls", action="store_true")
    ap.add_argument("--drop", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--tol", type=float, default=1e-6)
    return run(ap.parse_args())


if __name__ == "__main__":
    sys.exit(main())
