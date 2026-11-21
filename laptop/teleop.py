"""Keyboard teleop: WASD to drive, space to stop, g = wave, +/- speed, x to quit.

    python teleop.py [host]

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

from tars_client import DEFAULT_HOST, Tars

SPEED = 0.6
TURN = 0.5
KEY_HOLD_S = 0.6      # > macOS initial key-repeat delay, or the first press stutters


def key_to_cmd(k: str) -> tuple[float, float]:
    return {
        "w": (SPEED, SPEED),
        "s": (-SPEED, -SPEED),
        "a": (-TURN, TURN),     # spin left
        "d": (TURN, -TURN),     # spin right
    }[k]


def main() -> None:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    global SPEED
    print(f"connecting to {host}...")
    with Tars(host) as t:
        print(f"connected ({t.ping():.0f} ms). WASD drive, space stop, g wave, +/- speed, x quit")
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        cmd = (0.0, 0.0)
        last_key = 0.0
        try:
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if r:
                    k = sys.stdin.read(1).lower()
                    last_key = time.monotonic()
                    if k == "x":
                        break
                    elif k == " ":
                        cmd = (0.0, 0.0)
                    elif k == "g":
                        t.gesture("wave")
                    elif k in "+=":
                        SPEED = min(1.0, SPEED + 0.1); print(f"\rspeed {SPEED:.1f}  ", end="")
                    elif k == "-":
                        SPEED = max(0.1, SPEED - 0.1); print(f"\rspeed {SPEED:.1f}  ", end="")
                    elif k in "wasd":
                        cmd = key_to_cmd(k)
                if time.monotonic() - last_key > KEY_HOLD_S:
                    cmd = (0.0, 0.0)
                t.drive(*cmd)   # sent at ~20 Hz: keeps the Pi watchdog fed
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            print("\nstopping")


if __name__ == "__main__":
    main()
