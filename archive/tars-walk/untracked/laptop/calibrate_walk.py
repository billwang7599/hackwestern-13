"""Walk calibration run sheet for TARS (crank legs).

    python calibrate_walk.py [host]                      # interactive
    python calibrate_walk.py [host] --distance-cm 42 --actual-deg 300 --yes   # scripted

1. WALK 10: measures the real ms per crank step (sets CAL MS, which is also the
   stall timeout and the timed-mode step length) and asks how far it went.
2. 12 x TURN 30: asks the TOTAL angle actually turned (a full circle = 360) and
   writes the measured degrees per turn step back to the Uno with CAL DEG.

Have a tape mark on the floor at the toe and a strip of tape on the robot's top
pointing forward. Phone compass/level apps make the angle easy to read.
"""
from __future__ import annotations

import argparse
import time

from tars_client import DEFAULT_HOST, Tars, TarsError

WALK_STEPS = 10
TURNS = 12          # 12 x 30 = 360 deg commanded


def ask_float(prompt: str, given: float | None) -> float | None:
    if given is not None:
        print(f"{prompt} {given}")
        return given
    while True:
        s = input(f"{prompt} (blank = skip) ").strip()
        if not s:
            return None
        try:
            return float(s)
        except ValueError:
            print("  a number please")


def confirm(prompt: str, yes: bool) -> bool:
    if yes:
        print(f"{prompt} [y] (--yes)")
        return True
    return input(f"{prompt} [y/N] ").strip().lower().startswith("y")


def pause(msg: str, yes: bool) -> None:
    if yes:
        print(msg)
    else:
        input(f"{msg} -- press Enter ")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", nargs="?", default=DEFAULT_HOST)
    ap.add_argument("--distance-cm", type=float, help="answer for the walk distance")
    ap.add_argument("--actual-deg", type=float, help="answer for the total turned angle")
    ap.add_argument("--yes", action="store_true", help="don't prompt, write the results")
    ap.add_argument("--skip-walk", action="store_true")
    ap.add_argument("--skip-turn", action="store_true")
    a = ap.parse_args()

    with Tars(a.host) as t:
        cal = t.calibration()
        print(f"current calibration: ms/step L={cal['ms'][0]} R={cal['ms'][1]}  "
              f"deg/turn-step={cal['deg']:.2f}  mode={'switch' if cal['mode'] else 'timed'}")

        if not a.skip_walk:
            print(f"\n== 1. walk {WALK_STEPS} steps ==")
            pause("Put TARS on the floor, toes on the tape mark", a.yes)
            t0 = time.monotonic()
            try:
                print(" ", t.walk(WALK_STEPS))
            except TarsError as e:
                print(f"  ! {e}  (fix the stall / wiring first; see docs/tars_build.md)")
                return
            ms = (time.monotonic() - t0) * 1000.0 / WALK_STEPS
            print(f"  {ms:.0f} ms per step (incl. ~50 ms polling overhead per walk)")
            d = ask_float("  distance walked in cm?", a.distance_cm)
            if d is not None:
                print(f"  => {d / WALK_STEPS:.1f} cm per step "
                      f"(tell voice.py's walk tool description if it's way off)")
            if cal["mode"] == 1 and confirm(f"  write CAL MS 2 {ms:.0f} (stall timeout = 2x this)?", a.yes):
                t.cal("MS", 2, int(round(max(100, min(5000, ms)))))
            elif cal["mode"] == 0:
                print("  timed mode: step length IS ms/step. If it walked too far/short, scale it:")
                print(f"    CAL MS 2 <{cal['ms'][0]} x wanted_steps / real_strides>")

        if not a.skip_turn:
            steps_per_turn = max(1, round(30 / cal["deg"]))
            total_steps = TURNS * steps_per_turn
            print(f"\n== 2. turn {TURNS} x 30 deg ({total_steps} turn steps at {cal['deg']:.2f} deg/step) ==")
            pause("Line the tape on its head up with a reference", a.yes)
            for i in range(TURNS):
                try:
                    t.turn(30)
                except TarsError as e:
                    print(f"  ! turn {i + 1}: {e}")
                    return
                print(f"  turn {i + 1}/{TURNS}")
            actual = ask_float("  TOTAL degrees actually turned (360 = full circle)?", a.actual_deg)
            if actual is None or actual <= 0:
                print("  skipped")
            else:
                new = actual / total_steps
                print(f"  => {new:.2f} deg per turn step (was {cal['deg']:.2f})")
                if not 1.0 <= new <= 180.0:
                    print("  out of range 1..180, not written")
                elif confirm(f"  write CAL DEG {new:.2f}?", a.yes):
                    t.cal("DEG", f"{new:.2f}")
                    time.sleep(0.2)
                    print(f"  now: TURN 30 = {max(1, round(30 / new))} step(s), TURN 90 = {max(1, round(90 / new))}")

        time.sleep(0.2)
        cal = t.calibration()
        print(f"\nsaved on the Uno (EEPROM): ms/step L={cal['ms'][0]} R={cal['ms'][1]}  "
              f"deg/turn-step={cal['deg']:.2f}")


if __name__ == "__main__":
    main()
