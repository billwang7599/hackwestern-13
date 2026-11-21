"""Gain search on the simulator.

    python tune.py                      # NEMA17 (default), ~1 min
    python tune.py --motor tt --n 150   # the TT gearmotors
    python tune.py --quick              # smoke test, few seconds
    python tune.py --save               # write winners to gains.json (run.py picks them up)

Random search (log-uniform around the current gains) followed by a few rounds of
coordinate refinement. Each candidate is scored on +/- pushes with several noise
seeds; falls and stepper stalls are heavily penalized. Prints suggested starting
gains for the real robot. They are STARTING points: see docs/balance.md.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import time
from multiprocessing import Pool

from params import DEG, Scenario, robot_for_motor
from simulate import Sim, max_push, fmt_metrics, run_scenario
from controller import Controller

HERE = os.path.dirname(os.path.abspath(__file__))
GAINS_FILE = os.path.join(HERE, "gains.json")

# name: (lo, hi, log-scale?)
SPACE = {
    # capped: above this the sim still "works" but the real robot chatters and
    # loses margin to latency (checked: 84/22 beats 250/32 with 10 ms latency)
    "kp": (10.0, 150.0, True),
    "kd": (0.5, 30.0, True),
    "speed_kp": (0.0, 0.6, False),
    "speed_ki": (0.0, 0.4, False),
}


def load_gains(motor: str) -> dict:
    if os.path.exists(GAINS_FILE):
        with open(GAINS_FILE) as f:
            return json.load(f).get(motor, {}).get("gains", {})
    return {}


# Scored on the nominal robot AND a pessimistic one, so the tuner can't win by
# cranking gains that only work in a perfect world.
VARIANTS = [
    {},
    {"body_mass_scale": 1.25, "com_height_scale": 1.2, "imu_latency": 0.005,
     "gyro_noise": 0.03, "accel_noise": 0.6, "control_jitter": 0.0005},
]


def make_robot(motor, variant, over=None):
    p = robot_for_motor(motor)
    p.update(over or {})
    v = dict(variant)
    p.body_mass *= v.pop("body_mass_scale", 1.0)
    p.com_height *= v.pop("com_height_scale", 1.0)
    return p.update(v)


def cost(args):
    motor, gains, push, seeds, over = args
    total = 0.0
    for variant, seed, imp in \
            [(v, s, i) for v in VARIANTS for s in seeds for i in (push, -push)]:
        p = make_robot(motor, variant, over)
        p.seed = seed
        sc = Scenario(duration=5.0, pushes=[(1.5, imp)])
        s = Sim(p, gains, "auto", sc).run()
        L, dt = s.log, s.dt
        T = sc.duration
        if s.fell_at is not None or s.armed_at is None:
            total += 1000.0 + 200.0 * (T - (s.fell_at or 0.0))
            continue
        i0 = int(s.armed_at / dt)
        j = 0.0
        for k in range(i0, len(L["t"])):
            th = L["pitch"][k] / DEG
            j += (0.02 * th * th + 2.0 * L["xd"][k] ** 2 + 0.002 * L["accel"][k] ** 2) * dt
        # chatter: high-frequency content of the accel command (noise amplification)
        acc = L["accel"][i0:]
        chat = sum((acc[k] - acc[k - 1]) ** 2 for k in range(1, len(acc))) * dt
        j += 0.05 * chat
        j += 5.0 * L["x"][-1] ** 2 + 50.0 * L["xd"][-1] ** 2
        j += 1000.0 * s.stall_time
        m = s.metrics()
        if not m["passed"]:
            j += 20.0
        total += j
    return total


def sample(rng, center, scale):
    g = {}
    for k, (lo, hi, lg) in SPACE.items():
        c = center.get(k, 0.5 * (lo + hi))
        if lg:
            v = math.exp(math.log(max(c, lo)) + rng.gauss(0, scale))
        else:
            v = c + rng.gauss(0, scale * (hi - lo) * 0.5)
        g[k] = min(hi, max(lo, v))
    return g


def search(motor, over=None, n=120, seeds=(1, 2), rounds=2, push=0.3, fixed=None,
           center=None, jobs=None, log=print):
    """Random search + coordinate refinement. Returns (best_gains, best_cost)."""
    rng = random.Random(0)
    fixed = fixed or {}
    defaults = Controller("auto").get_cfg()
    c0 = {k: defaults[k] for k in SPACE}
    c0.update(center or {})

    def full(g):
        g = dict(g)
        g.update(fixed)
        return g

    t0 = time.time()
    cands = [c0] + [sample(rng, c0, 0.8) for _ in range(n)]
    with Pool(jobs or os.cpu_count()) as pool:
        scores = pool.map(cost, [(motor, full(g), push, list(seeds), over) for g in cands])
        bi = min(range(len(cands)), key=lambda i: scores[i])
        best, best_c = full(cands[bi]), scores[bi]
        log(f"random search: {len(cands)} candidates in {time.time()-t0:.0f}s, best cost {best_c:.2f}")
        for r in range(rounds):
            for k, (lo, hi, lg) in SPACE.items():
                if k in fixed:
                    continue
                vals = ([best[k] * f for f in (0.6, 0.8, 1.25, 1.6)] if lg else
                        [best[k] + d * (hi - lo) for d in (-0.15, -0.05, 0.05, 0.15)])
                trials = [dict(best, **{k: min(hi, max(lo, v))}) for v in vals]
                sc = pool.map(cost, [(motor, g, push, list(seeds), over) for g in trials])
                i = min(range(len(sc)), key=lambda q: sc[q])
                if sc[i] < best_c:
                    best, best_c = trials[i], sc[i]
            log(f"refine round {r+1}: cost {best_c:.2f}  {fmt_g(best)}")
    return best, best_c


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motor", default="nema17")
    ap.add_argument("--n", type=int, default=120, help="random candidates")
    ap.add_argument("--push", type=float, default=0.3, help="impulse N*s used for scoring")
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--rounds", type=int, default=2, help="coordinate refinement rounds")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--save", action="store_true", help="store result in gains.json")
    ap.add_argument("--no-outer", action="store_true", help="tune with the speed loop off")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    a = ap.parse_args()
    if a.quick:
        a.n, a.seeds, a.rounds = 12, 1, 0

    t0 = time.time()
    fixed = {"speed_kp": 0.0, "speed_ki": 0.0} if a.no_outer else {}
    best, best_c = search(a.motor, n=a.n, seeds=range(1, a.seeds + 1), rounds=a.rounds,
                          push=a.push, fixed=fixed, center=load_gains(a.motor), jobs=a.jobs)

    p = robot_for_motor(a.motor)
    m, _ = run_scenario(p, best)
    mp, stands = max_push(p, best)
    print()
    print(fmt_metrics(f"{a.motor} tuned", m))
    print(f"  max recoverable push (both directions): "
          f"{mp:.2f} N*s" if stands else "  cannot stand still")
    print()
    print("suggested starting gains (balance_cfg_t):")
    for k in SPACE:
        print(f"  cfg.{k:<9}= {best[k]:.4g}f;")
    print(f"done in {time.time()-t0:.0f}s")

    if a.save:
        data = {}
        if os.path.exists(GAINS_FILE):
            with open(GAINS_FILE) as f:
                data = json.load(f)
        data[a.motor] = {"gains": {k: round(v, 4) for k, v in best.items()},
                         "cost": round(best_c, 3), "max_push": round(mp, 3),
                         "passed_default_scenario": m["passed"]}
        with open(GAINS_FILE, "w") as f:
            json.dump(data, f, indent=2)
        print(f"saved to {GAINS_FILE}")


def fmt_g(g):
    return " ".join(f"{k}={g[k]:.3g}" for k in SPACE)


if __name__ == "__main__":
    main()
