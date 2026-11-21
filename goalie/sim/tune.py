"""Grid-search the controller/observer gains.

    python tune.py                         # default grid, prints best first
    python tune.py --rpm 90 --latency-ms 60   # any run.py option: tune for your bench

Score per gain set:
  fast  corner-shot save rate at a hard speed for this motor (3.25 m/s at 150 RPM,
        scaled with RPM; 2.0 m/s at RPM/3)
  rest  ball held still for 1.5 s: mean |paddle - target| (mm) and how often the
        duty flips sign per second (gear chatter / buzzing; lower is better)
"""
from __future__ import annotations

import itertools
import sys

import run
from simulate import Sim, save_rate

GRID = dict(kp=[3, 6, 10], kd=[0.1, 0.25, 0.4], obs_l1=[0.3, 0.5, 0.7], obs_l2=[0, 2, 4])


def rest_metrics(p):
    errs, flips, prev, n = [], 0, 0.0, 0
    for x0 in (-0.14, 0.10, 0.16):
        s = Sim(p, 5)
        s.place(x0)
        while s.t < 1.5:
            s.step()
            if s.t > 0.8 and s.n % 5 == 0:
                errs.append(abs(s.goalie.x - p.control.shadow_gain * x0))
                n += 1
                if s.duty * prev < 0:
                    flips += 1
                if s.duty:
                    prev = s.duty
    return 1000 * sum(errs) / len(errs), flips / (n / p.control.rate)


def main():
    a = run.parse_args(sys.argv[1:])
    rpm = a.rpm or 150.0
    v_hi, v_lo = 3.25 * min(1.0, (rpm / 150) ** 0.5), 2.0 * min(1.0, (rpm / 150) ** 0.5)
    rows = []
    for vals in itertools.product(*GRID.values()):
        g = ",".join(f"{k}={v}" for k, v in zip(GRID, vals))
        hi = save_rate(run.build(a, gains=g, rpm=rpm), v_hi, 40, aim_mode="corners")[0]
        lo = save_rate(run.build(a, gains=g, rpm=rpm / 3), v_lo, 40, aim_mode="corners")[0]
        err, fl = rest_metrics(run.build(a, gains=g, rpm=rpm))
        rows.append((hi + lo - 0.02 * err - 0.01 * fl, g, hi, lo, err, fl))
    rows.sort(reverse=True)
    print(f"corner shots: {v_hi:.2f} m/s at {rpm:.0f} RPM, {v_lo:.2f} m/s at {rpm/3:.0f} RPM")
    for sc, g, hi, lo, err, fl in rows[:15]:
        print(f"{g:<40} fast={hi:.2f}/{lo:.2f}  rest_err={err:4.1f} mm  flips={fl:5.1f}/s  score={sc:.2f}")


if __name__ == "__main__":
    main()
