"""Calibrate the stepper wheels from the laptop (or the Pi).

    python wheel_cal.py show                  current diameter / track (mm)
    python wheel_cal.py move 1000             drive exactly 1 m: measure what it really did
    python wheel_cal.py turns 12 30           12 x TURN 30 (should be one full turn)
    python wheel_cal.py set <diam> <track>    save new values (Uno EEPROM)

Diameter: after `move 1000`, measured D_new = D_old * measured_mm / 1000.
Track:    after `turns 12 30`, if it really turned T degrees in total instead
          of 360: track_new = track_old * 360 / T.
          (Over-turning, T > 360, means the real track is narrower than the setting.)
Fix the diameter first (it affects turns too), then the track.
Host: TARS_HOST env var or --host.
"""
from __future__ import annotations

import argparse
import time

from tars_client import DEFAULT_HOST, Tars


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--speed", type=float, default=None, help="fraction of max RPM")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("show")
    m = sub.add_parser("move"); m.add_argument("mm", type=float)
    t = sub.add_parser("turns"); t.add_argument("n", type=int); t.add_argument("deg", type=float)
    s = sub.add_parser("set"); s.add_argument("diam", type=float); s.add_argument("track", type=float)
    a = ap.parse_args()
    with Tars(a.host) as tars:
        if a.cmd == "show":
            print("diam %.1f mm, track %.1f mm" % tars.wheel_cal())
        elif a.cmd == "set":
            print("saved: diam %.1f mm, track %.1f mm" % tars.wheel_cal(a.diam, a.track))
        elif a.cmd == "move":
            r = tars.move_mm(a.mm, a.speed)
            print(f"{r['move']}: {r.get('done')} mm (asked {a.mm}). Measure it with a tape.")
        elif a.cmd == "turns":
            for i in range(a.n):
                r = tars.turn_deg(a.deg, a.speed)
                print(f"turn {i + 1}/{a.n}: {r['move']} {r.get('done')}")
                if r["move"] != "done":
                    break
                time.sleep(0.3)
            print(f"should total {a.n * a.deg:.0f} deg. Measure the real heading change.")


if __name__ == "__main__":
    main()
