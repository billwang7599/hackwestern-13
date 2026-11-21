"""Keyboard teleop for TARS.

    python teleop.py [host]

  drive    w a s d   (hold), space = stop, +/- = speed
  arms     q / e     left arm back / forward 15 deg
           z / c     right arm back / forward 15 deg
           0         both arms home
  gestures g wave   p point   u shrug   n nod   t stance   f fold
  safety   r         RESET after a tip-over (stand it up first)
           l         LEVEL: "it's upright now"
           x         quit

Terminals don't report key-up, so: hold a key (OS key-repeat keeps re-sending
it) and the robot moves; let go and it stops after KEY_HOLD_S. That's on top
of the Pi's own 500 ms watchdog.
"""
from __future__ import annotations

import select
import sys
import termios
import time
import tty

from tars_client import DEFAULT_HOST, LEFT, RIGHT, Tars, TarsError

SPEED = 0.6
TURN = 0.5
ARM_STEP = 15.0
KEY_HOLD_S = 0.6      # > macOS initial key-repeat delay, or the first press stutters

GESTURE_KEYS = {"g": "wave", "p": "point", "u": "shrug", "n": "nod", "t": "stance", "f": "fold"}
ARM_KEYS = {"q": (LEFT, -1), "e": (LEFT, +1), "z": (RIGHT, -1), "c": (RIGHT, +1)}


def key_to_cmd(k: str) -> tuple[float, float]:
    return {
        "w": (SPEED, SPEED),
        "s": (-SPEED, -SPEED),
        "a": (-TURN, TURN),     # spin left
        "d": (TURN, -TURN),     # spin right
    }[k]


def say(msg: str) -> None:
    print(f"\r{msg:<60}", end="", flush=True)


def main() -> None:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    global SPEED
    print(f"connecting to {host}...")
    with Tars(host) as t:
        print(f"connected ({t.ping():.0f} ms)")
        print(__doc__.split("\n\n")[1])
        arm_deg = [0.0, 0.0]
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        cmd = (0.0, 0.0)
        last_key = 0.0
        last_err = ""
        try:
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                try:
                    if r:
                        k = sys.stdin.read(1).lower()
                        last_key = time.monotonic()
                        if k == "x":
                            break
                        elif k == " ":
                            cmd = (0.0, 0.0)
                            t.stop()
                        elif k in GESTURE_KEYS:
                            secs = t.gesture(GESTURE_KEYS[k])
                            say(f"{GESTURE_KEYS[k]} (~{secs:.1f} s)")
                        elif k in ARM_KEYS:
                            arm, sign = ARM_KEYS[k]
                            arm_deg[arm] = t.arm(arm, arm_deg[arm] + sign * ARM_STEP)
                            say(f"arms L={arm_deg[0]:+.0f} R={arm_deg[1]:+.0f}")
                        elif k == "0":
                            t.home()
                            arm_deg = [0.0, 0.0]
                            say("arms home")
                        elif k == "r":
                            t.reset()
                            say("reset OK")
                        elif k == "l":
                            t.level()
                            say("level captured")
                        elif k in "+=":
                            SPEED = min(1.0, SPEED + 0.1); say(f"speed {SPEED:.1f}")
                        elif k == "-":
                            SPEED = max(0.1, SPEED - 0.1); say(f"speed {SPEED:.1f}")
                        elif k in "wasd":
                            cmd = key_to_cmd(k)
                    if time.monotonic() - last_key > KEY_HOLD_S:
                        cmd = (0.0, 0.0)
                    t.drive(*cmd)   # sent at ~20 Hz: keeps the Pi watchdog fed
                    last_err = ""
                except TarsError as e:
                    # tipped / bumped: keep running, show why once
                    if str(e) != last_err:
                        say(f"! {e}")
                        last_err = str(e)
                    cmd = (0.0, 0.0)
                    t.ping()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            print("\nstopping")


if __name__ == "__main__":
    main()
