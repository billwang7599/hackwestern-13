"""Calibrate the wheels from the laptop (or the Pi), through tars_motor.

DC wheels (Kuman SM11 kit, the default firmware): MOVE/TURN are TIMED at D 1, so
calibrate the speeds. Put the car on the real floor, with the battery you'll demo on.
    python wheel_cal.py show                  speeds (DC) or diameter / track (steppers)
    python wheel_cal.py dc-move [1000]        MOVE 1000: tape it, type what it really did
    python wheel_cal.py dc-spin [360]         TURN 360: type how far it really turned
    python wheel_cal.py dc-set <mm_s> <deg_s> save speeds directly (Uno EEPROM)
  dc-move: new mm_s = old mm_s * measured_mm / asked_mm (same for dc-spin and deg_s).
  Repeat each once more; it should land within ~5%. The speed cap is ~150 mm/s
  (docs/PI_TEST.md "Kuman SM11"): if dc-move ends far outside 100-200 mm/s, change
  DC_MAX_DUTY in tars.ino instead of living with it. Drifting to one side over 1 m:
  trim DC_LEFT_GAIN / DC_RIGHT_GAIN in tars.ino.
  Pass --measured N to skip the prompt.

Stepper wheels (28BYJ-48 build):
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


def scaled(old: float, asked: float, measured: float) -> float:
    """new speed for a timed move that was asked `asked` and really did `measured`"""
    if asked == 0 or measured <= 0:
        raise SystemExit("measured amount must be > 0")
    return old * measured / abs(asked)


def ask(prompt: str, given: float | None) -> float:
    if given is not None:
        return given
    return float(input(prompt).strip())


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--speed", type=float, default=None, help="steppers: fraction of max RPM")
    ap.add_argument("--measured", type=float, default=None, help="dc-move/dc-spin: skip the prompt")
    ap.add_argument("--no-save", action="store_true", help="dc-move/dc-spin: just print the new value")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("show")
    m = sub.add_parser("move"); m.add_argument("mm", type=float)
    t = sub.add_parser("turns"); t.add_argument("n", type=int); t.add_argument("deg", type=float)
    s = sub.add_parser("set"); s.add_argument("diam", type=float); s.add_argument("track", type=float)
    dm = sub.add_parser("dc-move"); dm.add_argument("mm", type=float, nargs="?", default=1000.0)
    ds = sub.add_parser("dc-spin"); ds.add_argument("deg", type=float, nargs="?", default=360.0)
    dset = sub.add_parser("dc-set"); dset.add_argument("mm_s", type=float); dset.add_argument("deg_s", type=float)
    a = ap.parse_args()
    with Tars(a.host) as tars:
        if a.cmd == "show":
            dc = tars.speed_cal()
            if dc:
                print("DC wheels: %.1f mm/s straight, %.1f deg/s spinning, at D 1 (timed MOVE/TURN)" % dc)
            else:
                print("stepper wheels: diam %.1f mm, track %.1f mm" % tars.wheel_cal())
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
        elif a.cmd in ("dc-move", "dc-spin", "dc-set"):
            cur = tars.speed_cal()
            if cur is None:
                raise SystemExit("tars_motor says stepper wheels: flash the SM11 (DC) firmware, or use move/turns/set")
            mm_s, deg_s = cur
            if a.cmd == "dc-set":
                mm_s, deg_s = a.mm_s, a.deg_s
            else:
                kind = "MOVE" if a.cmd == "dc-move" else "TURN"
                asked = a.mm if kind == "MOVE" else a.deg
                print(f"{kind} {asked:.0f} at {mm_s if kind == 'MOVE' else deg_s:.1f} "
                      f"{'mm/s' if kind == 'MOVE' else 'deg/s'}: stand clear...")
                r = (tars.move_mm if kind == "MOVE" else tars.turn_deg)(asked)
                if r["move"] != "done":
                    raise SystemExit(f"{kind} {r['move']} ({r.get('why')}): nothing saved")
                unit = "mm" if kind == "MOVE" else "deg"
                got = ask(f"how far did it REALLY go ({unit}, asked {asked:.0f})? ", a.measured)
                if kind == "MOVE":
                    mm_s = scaled(mm_s, asked, got)
                else:
                    deg_s = scaled(deg_s, asked, got)
            print(f"new: {mm_s:.1f} mm/s, {deg_s:.1f} deg/s")
            if mm_s > 200 or mm_s < 100:
                print("  note: outside 100-200 mm/s: adjust DC_MAX_DUTY in tars.ino (cap ~150 mm/s)")
            if not a.no_save:
                print("saved: %.1f mm/s, %.1f deg/s" % tars.speed_cal(mm_s, deg_s))


if __name__ == "__main__":
    main()
