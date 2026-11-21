#!/usr/bin/env python3
"""goto on a DC smart car (Kuman SM11): faster wheels, SLOPPY open-loop MOVE/TURN.

    python3 tests/test_dc_car.py build/goto_host            checks (part of make test)
    python3 tests/test_dc_car.py build/goto_host --sweep    speed sweep table (picks the cap)

Reuses test_goto_sim.py's fake tars_motor, camera and goto harness (visual-only GOTO:
search TURNs, align TURN, D approach), with the motor model swapped for a DC car:
  - D 1 = V m/s straight (the Uno's DC_MAX_DUTY cap), spin from D with an effective
    (skid-steer) track, so D 1 -1 spins at DEG_S.
  - MOVE/TURN are timed at D 1, like the SM11 firmware. The floor does MOVE x m_scale
    and TURN x r_scale (the Uno doesn't know: tars_motor's pose uses the commanded
    amount), and straight driving curves by drift deg/m (one side faster).
The map-guided (iPhone) path: tests/test_fullstack_dc.py.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_goto_sim as sim  # noqa: E402

STEPPER_TRACK = sim.TRACK


class Car:
    """the model the patched module uses (set by use_car)"""
    v = 0.15           # m/s at D 1
    track_eff = 0.20   # m: D l r -> spin rate; also DEG_S = 2 v / track_eff
    m_scale = 1.0      # floor / commanded for MOVE
    r_scale = 1.0      # ... for TURN
    drift = 0.0        # deg per metre driven, + = curves right (clockwise)
    timed = True       # MOVE/TURN timed at D 1 (DC) instead of the stepper trapezoid

    @classmethod
    def deg_s(cls) -> float:
        return math.degrees(2 * cls.v / cls.track_eff)


_stepper_move_secs = sim.move_secs


def dc_move_secs(kind: str, amount: float, speed: float) -> float:
    if not Car.timed:
        return _stepper_move_secs(kind, amount, speed)
    return abs(amount) / (Car.v * 1000.0) if kind == "M" else abs(amount) / Car.deg_s()


class DCMotor(sim.FakeMotor):
    """FakeMotor with exact scale errors + drift instead of random +-3%."""

    def __init__(self, scale: float = 1.0, turn_err: float = 0.0, rng: random.Random | None = None,
                 drive_err: float = 0.0):
        super().__init__(scale, turn_err=0.0, rng=rng, drive_err=0.0)
        # D driving: one side faster so the path curves by Car.drift deg/m
        b = math.radians(Car.drift) * Car.track_eff / 2.0
        self.wl_err, self.wr_err = 1.0 + b, 1.0 - b

    def handle(self, line: str) -> str:
        r = super().handle(line)
        p = line.split()
        if p and p[0].upper() in ("TURN", "MOVE") and r.startswith("OK") and self.move:
            with self.lock:
                amt = float(p[1])
                if p[0].upper() == "TURN":
                    self.move["total"] = math.radians(amt) * Car.r_scale
                else:
                    self.move["total"] = amt / 1000.0 * Car.m_scale
        return r

    def _step(self) -> None:
        m = self.move
        if m and m["kind"] == "M":
            # same as FakeMotor, plus the heading drifts with distance driven
            t = self.now()
            dt = t - self.t_last
            self.t_last = t
            if dt <= 0:
                return
            if t - self.last_cmd > 0.5:
                self._abort_move("watchdog")
                return
            frac = min(1.0, (t - m["start"]) / m["dur"])
            d_new = m["total"] * frac
            d_old = m.get("d_prev", 0.0)
            step = d_new - d_old
            m["d_prev"] = d_new
            self.th -= math.radians(Car.drift) * abs(step) / 2
            self.x += step * math.cos(self.th)
            self.y += step * math.sin(self.th)
            self.th -= math.radians(Car.drift) * abs(step) / 2
            m["done"] = frac
            if frac >= 1.0:
                self._odo_move(m, 1.0)
                self.move_state = "done"
                self.move = None
            return
        super()._step()

    def angular_speed(self) -> float:
        with self.lock:
            if self.move and self.move["kind"] == "R":
                return math.radians(Car.deg_s())
            return abs((self.vr - self.vl) * sim.V_MAX / (sim.TRACK / 1000.0))


def use_car(v: float, m_scale: float = 1.0, r_scale: float = 1.0, drift: float = 0.0, stepper: bool = False) -> None:
    Car.v, Car.m_scale, Car.r_scale, Car.drift, Car.timed = v, m_scale, r_scale, drift, not stepper
    Car.track_eff = STEPPER_TRACK / 1000.0 if stepper else 0.20
    sim.V_MAX = v
    sim.TRACK = Car.track_eff * 1000.0
    sim.move_secs = dc_move_secs
    sim.FakeMotor = DCMotor


# --------------------------------------------------------------------- runs
def scenes(n: int, seed: int) -> list[tuple]:
    rng = random.Random(seed)
    return [(seed * 1000 + i, rng.uniform(0.8, 3.0), rng.uniform(-180, 180), rng.choice([0.4, 0.6, 0.8]))
            for i in range(n)]


def run_batch(exe: str, scale: float, jobs: list[tuple], par: int) -> list[dict]:
    out: list[dict] = [None] * len(jobs)

    def worker(i: int) -> None:
        s, d, b, h = jobs[i]
        r = sim.drive_scene(exe, scale, s, d, b, obj_h=h)
        turns = [l for _, l in r["log"] if l.startswith("TURN")]
        r.pop("log")
        r.update(start_dist=d, start_bearing=b, obj_h=h, turns=len(turns))
        out[i] = r

    for i in range(0, len(jobs), par):
        ts = [threading.Thread(target=worker, args=(k,)) for k in range(i, min(len(jobs), i + par))]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
    return out


def summarize(rs: list[dict]) -> dict:
    ok = [r for r in rs if r["result"] == "arrived" and r["dist"] < 1.3 and abs(r["bearing"]) < 15 and r["min_d"] > 0]
    times = sorted(r["t"] for r in ok)
    dists = sorted(r["dist"] for r in ok)
    return {"n": len(rs), "ok": len(ok), "crash": sum(r["min_d"] <= 0 for r in rs),
            "t_med": times[len(times) // 2] if times else 0, "d_min": dists[0] if dists else 0,
            "d_med": dists[len(dists) // 2] if dists else 0, "d_max": dists[-1] if dists else 0,
            "too_close": sum(r["min_d"] < 0.30 for r in rs),
            "results": sorted({r["result"] for r in rs if r["result"] != "arrived"})}


def line(name: str, s: dict) -> str:
    return (f"  {name:<34} {s['ok']:>2}/{s['n']:<2} arrived  crash {s['crash']}  <0.30 m {s['too_close']}  "
            f"t~{s['t_med']:4.0f} s  stop {s['d_min']:.2f}/{s['d_med']:.2f}/{s['d_max']:.2f} m  {' '.join(s['results'])}")


# --------------------------------------------------------------------- checks
passed = failed = 0


def check(ok: bool, what: str) -> None:
    global passed, failed
    if ok:
        passed += 1
        print(f"  ok   {what}")
    else:
        failed += 1
        print(f"  FAIL {what}")


CAP_V = 0.15        # the SM11 cap the firmware targets (docs/PI_TEST.md "Kuman SM11")
UNCALIBRATED = [    # (MOVE scale, TURN scale, drift deg/m): +-25% = V never run / wrong battery
    (1.25, 1.25, +15.0),
    (0.75, 0.75, -15.0),
    (1.25, 0.75, +15.0),
    (0.75, 1.25, -15.0),
]
CALIBRATED = [      # +-15% = after wheel_cal.py speeds (battery sag, floor change)
    (1.15, 0.85, +15.0),
    (0.85, 1.15, -15.0),
]


def test_sloppy(exe: str, scale: float, n: int, par: int) -> None:
    print(f"DC car at the cap ({CAP_V * 1000:.0f} mm/s, {math.degrees(2 * CAP_V / 0.20):.0f} deg/s), "
          f"sloppy timed MOVE/TURN, {n} random starts each (goto -T {scale})")
    jobs = scenes(n, 7)
    use_car(CAP_V)
    base = summarize(run_batch(exe, scale, jobs, par))
    print(line("exact (scale 1, no drift)", base))
    check(base["ok"] >= n - 1 and base["crash"] == 0, f"exact timing: {base['ok']}/{n} arrived, {base['crash']} crashes")
    tot = ok = 0
    for group, errs in (("uncalibrated", UNCALIBRATED), ("calibrated", CALIBRATED)):
        for ms, rs, dr in errs:
            use_car(CAP_V, ms, rs, dr)
            s = summarize(run_batch(exe, scale, jobs, par))
            name = f"MOVE x{ms:.2f} TURN x{rs:.2f} drift {dr:+.0f}/m"
            print(line(name, s))
            # A TURN that really does only 75% leaves a blind wedge in a 12 x 30 deg search
            # (12 x 22.5 = 270 deg + 60 deg FOV < 360): an object in it is "not found"
            # (BEEP 3, nothing hit). ~1 in 12 random placements. Uncalibrated rows may miss
            # those, cleanly; after calibration (<= 15%, 324 + 60 > 360) there's no wedge.
            graceful = all(r == "failed:not_found" for r in s["results"])
            need = 0.75 * n if group == "uncalibrated" else n - 1
            check(s["ok"] >= need and graceful and s["crash"] == 0 and s["too_close"] == 0,
                  f"{group} {name}: {s['ok']}/{n} arrived (>= {need:.0f}), misses only not_found: "
                  f"{graceful}, {s['crash']} crashes, {s['too_close']} closer than 0.30 m, "
                  f"stop median {s['d_med']:.2f} m")
            tot += n
            ok += s["ok"]
    check(ok >= 0.85 * tot, f"all sloppy rows: {ok}/{tot} arrived (>= 85%)")


def sweep(exe: str, scale: float, n: int, par: int) -> None:
    print(f"speed sweep: {n} random starts per row, exact moves then +-25%/15 deg/m (goto -T {scale})")
    jobs = scenes(n, 3)
    rows = [("stepper 41 mm/s (reference)", dict(v=0.0408, stepper=True))]
    for v in (0.10, 0.15, 0.20, 0.30, 0.45):
        rows.append((f"dc {v * 1000:.0f} mm/s exact", dict(v=v)))
        rows.append((f"dc {v * 1000:.0f} mm/s x1.25 +15/m", dict(v=v, m_scale=1.25, r_scale=1.25, drift=15.0)))
        rows.append((f"dc {v * 1000:.0f} mm/s x0.75 -15/m", dict(v=v, m_scale=0.75, r_scale=0.75, drift=-15.0)))
    for name, kw in rows:
        use_car(**kw)
        print(line(name, summarize(run_batch(exe, scale, jobs, par))), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("exe")
    ap.add_argument("--scale", type=float, default=8.0)
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--par", type=int, default=8)
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()
    exe = os.path.abspath(a.exe)
    os.makedirs(os.path.join(sim.GOTO_DIR, "build"), exist_ok=True)
    t0 = time.monotonic()
    if a.sweep:
        sweep(exe, a.scale, a.n, a.par)
        return
    test_sloppy(exe, a.scale, a.n, a.par)
    print(f"{passed}/{passed + failed} checks passed ({time.monotonic() - t0:.0f} s)")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
