#!/usr/bin/env python3
"""2WD vs 4WD virtual SM11 through the REAL firmware (tars.ino CAR_SM11) and the REAL host
tars_motor, with the REAL iter1-wasd/laptop/wheel_cal.py doing the calibration.

    python3 bench/tests/test_car_4wd.py

Cars (each on a fresh virtual Uno, so it starts uncalibrated: firmware defaults
150 mm/s, 120 deg/s, slow 60 deg/s):
  2wd         profiles/sm11_2wd.json  (1 TT motor per side on a caster)
  4wd         profiles/sm11.json      (2 TT motors per side, tyre scrub, 0.85 kg loaded,
                                       geometry estimated from the user's photo)
  4wd-light   sm11.json with mass_kg 0.45, scrub_mu 0.2 (bare kit on a slick floor:
              the optimistic case, to show what a 4WD that CAN spin does)
  4wd-taped   profiles/sm11_taped.json (rear tyres wrapped in tape: reported, not asserted)
For each: MOVE 500, TURN 90, TURN 90 0.4 (slow) uncalibrated; wheel_cal.py dc-move,
dc-spin, dc-spin --slow; then MOVE 500, TURN 90, TURN 90 0.4 again. Also the lowest
battery voltage (on-phase sag) during the first TURN, and whether the Uno reset.

Every number in the profiles is a guess (measured: false). Laptop only: no real car,
battery, Uno or Pi has run any of this.
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_car as tc  # noqa: E402  (Rig, run_wheel_cal, check, car launcher)

CARS = [
    ("2wd", "sm11_2wd", {}),
    ("4wd", "sm11", {}),
    ("4wd-light", "sm11", {"mass_kg": 0.45, "scrub_mu": 0.2}),
    ("4wd-taped", "sm11_taped", {}),
]


def spin_cal(rig: tc.Rig, args: list[str]) -> tuple[int, str]:
    acc = {"a": 0.0, "prev": rig.truth()["th"]}
    stop = threading.Event()

    def track():
        while not stop.is_set():
            th = rig.truth()["th"]
            acc["a"] += math.atan2(math.sin(th - acc["prev"]), math.cos(th - acc["prev"]))
            acc["prev"] = th
            time.sleep(0.01)
    t = threading.Thread(target=track, daemon=True)
    t.start()
    rc, out = tc.run_wheel_cal(rig, args, lambda: abs(math.degrees(acc["a"])))
    stop.set()
    t.join()
    return rc, out


def saved(out: str) -> str:
    lines = [l for l in out.splitlines() if l.startswith("saved")]
    if lines:
        return lines[0]
    err = [l for l in out.splitlines() if "Error" in l or "error" in l]
    return ("FAILED: " + err[-1].strip()) if err else "FAILED: " + out[-160:].replace("\n", " | ")


def turn(rig: tc.Rig, line: str) -> tuple[float, float, int, int]:
    """-> (degrees turned clockwise, lowest battery V, Uno boots before, after)"""
    b0 = rig.truth()["boots"]
    rig.ctl({"stats_reset": True})
    st, _, dth = rig.exact(line)
    d = rig.truth()
    return -dth, d.get("batt_v_low", d["batt_v"]), b0, d["boots"]


def one(name: str, profile: str, sets: dict) -> dict:
    rig = tc.Rig(seed=11, profile=profile)
    r: dict = {"name": name}
    try:
        rig.wait(lambda: rig.status().get("ready") == "1", 6.0)
        if sets:
            rig.ctl({"set": sets})
        _, d, _ = rig.exact("MOVE 500")
        r["move_uncal"] = d * 1000
        r["turn_uncal"], r["batt_low"], b0, b1 = turn(rig, "TURN 90")
        r["reset_in_turn"] = b1 != b0
        r["slow_uncal"], r["batt_low_slow"], _, _ = turn(rig, "TURN 90 0.4")
        mark = rig.truth()
        x0, y0 = mark["x"], mark["y"]
        rc, out = tc.run_wheel_cal(rig, ["dc-move", "1000"],
                                   lambda: 1000 * math.hypot(rig.truth()["x"] - x0, rig.truth()["y"] - y0))
        r["cal_move"] = saved(out)
        rc, out = spin_cal(rig, ["dc-spin", "360"])
        r["cal_spin"] = saved(out)
        rc, out = spin_cal(rig, ["dc-spin", "--slow"])
        r["cal_slow"] = saved(out)
        st = rig.status()
        r["speeds"] = f"{st.get('dc_mm_s')} mm/s, {st.get('dc_deg_s')} deg/s, slow {st.get('dc_deg_s_slow', '?')}"
        _, d, h = rig.exact("MOVE 500")
        r["move_cal"], r["move_cal_heading"] = d * 1000, h
        r["turn_cal"], _, _, _ = turn(rig, "TURN 90")
        r["slow_cal"], _, _, _ = turn(rig, "TURN 90 0.4")
        r["sag_resets"] = rig.truth().get("sag_resets", 0)
    finally:
        rig.close()
    return r


def main() -> int:
    tc.car.build()
    tc.car.build_motor()
    print(f"load {tc.load()}", flush=True)
    res = {}
    for name, prof, sets in CARS:
        print(f"--- {name} ({prof}{' + ' + str(sets) if sets else ''})", flush=True)
        r = one(name, prof, sets)
        res[name] = r
        for k, v in r.items():
            if k != "name":
                print(f"  ..   {k}: {v:.1f}" if isinstance(v, float) else f"  ..   {k}: {v}", flush=True)
        print(f"  ..   load {tc.load()}", flush=True)

    two, four, lite = res["2wd"], res["4wd"], res["4wd-light"]
    tc.check(abs(two["move_cal"] - 500) < 40 and abs(two["turn_cal"] - 90) < 10,
             f"2WD calibrated: MOVE 500 -> {two['move_cal']:.0f} mm, TURN 90 -> {two['turn_cal']:.1f} deg")
    tc.check(abs(four["move_cal"] - 500) < 40,
             f"4WD calibrated MOVE 500 -> {four['move_cal']:.0f} mm (dc-move absorbs the slower 4WD straight speed)")
    tc.check(four["move_uncal"] < two["move_uncal"] - 20,
             f"4WD uncalibrated MOVE 500 falls shorter than 2WD ({four['move_uncal']:.0f} vs {two['move_uncal']:.0f} mm): more driver drop")
    tc.check(abs(four["turn_uncal"]) < 10 and abs(four["slow_uncal"]) < 10,
             f"4WD (loaded, mu 0.4, photo geometry): TURN 90 turns {four['turn_uncal']:.1f} deg, slow {four['slow_uncal']:.1f} deg: "
             "the predicted tyre-scrub stall at the firmware's 0.45/0.36 duty")
    tc.check(four["cal_spin"].startswith("FAILED"),
             f"4WD: wheel_cal.py dc-spin can't calibrate a car that doesn't turn ({four['cal_spin'][:90]})")
    tc.check(not four["reset_in_turn"] and four["batt_low"] > 6.0,
             f"4WD TURN at 7.4 V: battery low {four['batt_low']:.2f} V, Uno reset: {four['reset_in_turn']}")
    tc.check(0 < lite["turn_uncal"] < 80,
             f"4WD-light uncalibrated TURN 90 under-turns: {lite['turn_uncal']:.1f} deg")
    tc.check(abs(lite["turn_cal"] - 90) < 15,
             f"4WD-light after dc-spin: TURN 90 -> {lite['turn_cal']:.1f} deg (tol 15)")

    tp = res["4wd-taped"]
    print(f"  ..   4wd-taped (reported only): TURN 90 {tp['turn_uncal']:.1f} -> {tp['turn_cal']:.1f} deg after dc-spin "
          f"({tp['cal_spin'][:70]}); slow {tp['slow_uncal']:.1f} -> {tp['slow_cal']:.1f}")

    print("\n  car        MOVE500 unc/cal   TURN90 unc/cal   slow TURN90 unc/cal   batt low in TURN")
    for n in ("2wd", "4wd", "4wd-light", "4wd-taped"):
        r = res[n]
        print(f"  {n:<10} {r['move_uncal']:5.0f} / {r['move_cal']:5.0f} mm   {r['turn_uncal']:5.1f} / {r['turn_cal']:5.1f}"
              f"     {r['slow_uncal']:5.1f} / {r['slow_cal']:5.1f}            {r['batt_low']:.2f} V")
    print(f"load {tc.load()}")
    print(f"{tc.total - tc.fails}/{tc.total} checks passed")
    return 1 if tc.fails else 0


if __name__ == "__main__":
    sys.exit(main())
