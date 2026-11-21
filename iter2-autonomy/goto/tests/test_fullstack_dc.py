#!/usr/bin/env python3
"""Full-stack sim with SLOPPY motion (DC smart car, open-loop timed MOVE/TURN).

    python3 iter2-autonomy/goto/tests/test_fullstack_dc.py [--only obstacle,moved] [-v]
    (make -C iter2-autonomy/goto test-fullstack-dc)

Same real processes and fake sensors as test_fullstack.py (imported, not changed),
except where the robot really is. There, truth = tars_motor's dead-reckoned pose. Here,
truth integrates tars_motor's pose CHANGES with errors the robot can't see:
    distance x m_scale, turning x r_scale, plus drift deg per metre (one side faster).
tars_motor (and the Uno) still believe the commanded amounts; the fake iPhone (LDR1 pose
+ LiDAR), the fake lidar_guard and the fake YOLO all see the truth. goto re-plans from the
phone's pose after every leg (5b954ce), so it should still get there.
Known (2026-11-21, 64/65): "moved" with MOVE x1.25 TURN x0.75 fails "old spot A downgraded":
goto's object memory merges a sighting by tars_motor's dead-reckoned x/y (here 2 m off),
so the bin seen at B refreshes instance #1 (A) instead of downgrading it. It still docks
at B and #1's map= (ARKit) spot moves to B. Fix belongs in goto_core.c (merge by navw
when both have one), left for after the deflake merge.
The stub's WHEELCAL 260 400 (test_fullstack.Motor) runs ~163 mm/s straight: about the
SM11 cap (150 mm/s), so the speeds roughly match the DC car too.
"""
from __future__ import annotations

import math
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_fullstack as fs  # noqa: E402

_cfg = threading.local()     # per scenario thread: (m_scale, r_scale, drift_deg_per_m)


class DriftSensors(fs.Sensors):
    def __init__(self, *a, **kw):
        self.err = getattr(_cfg, "err", (1.0, 1.0, 0.0))   # set before the parent starts its threads
        super().__init__(*a, **kw)

    def _pose_loop(self):
        ms, rs, drift = self.err
        prev = None
        tx = ty = th = 0.0                       # truth: m, m, rad (+ = clockwise, like tars_motor)
        while self.run:
            try:
                self.mf.write("STATUS\n"); self.mf.flush()
                r = self.mf.readline()
            except (OSError, ValueError):
                return
            m = re.search(r" pose=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)", r)
            if m:
                x, y, h = (float(v) for v in m.groups())
                if prev is not None:
                    px, py, ph = prev
                    dh = (h - ph + 180.0) % 360.0 - 180.0
                    mid = math.radians(ph + dh / 2)
                    ds = ((x - px) * math.cos(mid) + (y - py) * math.sin(mid)) / 1000.0   # signed m
                    dth = math.radians(dh) * rs + math.radians(drift) * abs(ds) * ms
                    tmid = th + dth / 2
                    tx += ds * ms * math.cos(tmid)
                    ty += ds * ms * math.sin(tmid)
                    th += dth
                prev = (x, y, h)
                self.pose = (tx, ty, th)
                self.trace.append((tx, ty))
            time.sleep(0.02)


fs.Sensors = DriftSensors

ERRORS = [   # (MOVE/distance scale, TURN scale, drift deg/m)
    (1.0, 1.0, 0.0),
    (1.25, 1.25, +15.0),
    (0.75, 0.75, -15.0),
    (1.25, 0.75, +15.0),
    (0.75, 1.25, -15.0),
]
DEFAULT = ["obstacle", "moved", "visual"]


def run(name: str, err) -> None:
    _cfg.err = err
    print(f"== {name} with MOVE x{err[0]:.2f} TURN x{err[1]:.2f} drift {err[2]:+.0f} deg/m")
    fs.run_scenario(name)


def main() -> int:
    only = DEFAULT
    if "--only" in sys.argv:
        only = sys.argv[sys.argv.index("--only") + 1].split(",")
    for exe in (fs.MOTOR, fs.GOTO):
        if not os.path.exists(exe):
            print(f"missing {exe}: make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host")
            return 2
    t0 = time.monotonic()
    for err in ERRORS:
        before = (fs.passed, fs.failed)
        ts = [threading.Thread(target=run, args=(n, err)) for n in only]
        for t in ts:
            t.start()
        for t in ts:
            t.join()
        print(f"-- MOVE x{err[0]:.2f} TURN x{err[1]:.2f} drift {err[2]:+.0f}/m: "
              f"{fs.passed - before[0]}/{fs.passed - before[0] + fs.failed - before[1]} checks", flush=True)
    print(f"{fs.passed}/{fs.passed + fs.failed} checks passed ({time.monotonic() - t0:.0f} s)")
    return 1 if fs.failed else 0


if __name__ == "__main__":
    sys.exit(main())
