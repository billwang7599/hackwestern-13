"""Run a walk/turn sequence and print what happened.

    python walk_demo.py [host] [moves...]
    python walk_demo.py tars.local w3 t90 w-2 t-30        # the default sequence

wN = WALK N steps (- = back), tN = TURN N degrees (multiple of 30, + = left).
"""
from __future__ import annotations

import sys
import time

from tars_client import DEFAULT_HOST, Tars, TarsError

DEFAULT = ["w3", "t90", "w-2", "t-30"]


def main() -> int:
    args = sys.argv[1:]
    host = args.pop(0) if args and not args[0][:1] in ("w", "t") else DEFAULT_HOST
    moves = args or DEFAULT
    with Tars(host) as t:
        print(f"connected to {host} ({t.ping():.1f} ms)  cal: {t.cal()}")
        t0 = time.monotonic()
        for m in moves:
            kind, n = m[0], int(m[1:])
            s = time.monotonic()
            try:
                est = t.walk_start(n) if kind == "w" else t.turn_start(n)
                label = f"WALK {n}" if kind == "w" else f"TURN {n}"
                print(f"{time.monotonic() - t0:6.2f}s  {label:<9} est {est:.1f}s ...", end=" ", flush=True)
                res = t.walk_wait(est)
                print(f"{res}  ({time.monotonic() - s:.2f}s)")
            except TarsError as e:
                print(f"! {e}")
                return 1
        st = t.status()
        print(f"{time.monotonic() - t0:6.2f}s  all done: walks={st['walks']} walk={st['walk']} "
              f"last={st['walk_last']}")
        try:
            t.send("TURN 45")
        except TarsError as e:
            print(f"TURN 45 is refused as it should be: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
