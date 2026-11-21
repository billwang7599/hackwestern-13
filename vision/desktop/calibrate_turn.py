#!/usr/bin/env python3
"""Calibrate turn_ms (the 30-degree turn) on the real rover.

Sends N identical gesture-mode turns (wheels +-turn_speed for turn_ms, D
resent every resend_ms exactly like the gesture app, then S) with a pause
between them. Put a piece of tape on the floor along the rover's heading:

    python calibrate_turn.py --host tars.local            # 12 turns: should be 360 deg
    python calibrate_turn.py --host tars.local --n 3      # 3 turns: should be 90 deg
    python calibrate_turn.py --host tars.local --turn-ms 520

new turn_ms = turn_ms * (N * 30) / (degrees it actually turned). Put it in
vision/gesture.conf. Battery voltage and floor change it, so recheck at the venue.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "common"))

import gesture as g  # noqa: E402
from link import MotorLink  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="tars_motor host (omit = dry run)")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--turn-ms", type=float)
    ap.add_argument("--turn-speed", type=float)
    ap.add_argument("--direction", choices=["left", "right"], default="left")
    ap.add_argument("--pause", type=float, default=0.8, help="seconds between turns")
    a = ap.parse_args()

    over = {}
    if a.turn_ms:
        over["turn_ms"] = a.turn_ms
    if a.turn_speed:
        over["turn_speed"] = a.turn_speed
    cfg = g.load_config(overrides=over)
    link = MotorLink(a.host, a.port, verbose=False)
    label = g.POINT_LEFT if a.direction == "left" else g.POINT_RIGHT
    print(f"{a.n} x turn_{a.direction}: turn_speed={cfg['turn_speed']} turn_ms={cfg['turn_ms']:.0f} "
          f"(expect {a.n * 30} deg total)")
    try:
        for i in range(a.n):
            ctl = g.Controller(cfg)                      # fresh: no cooldown carry-over
            t0 = time.monotonic() * 1000
            cmds = []
            for k in range(int(cfg["stable_frames"])):   # same path as a real gesture
                cmds += ctl.on_frame(t0, label)
            for c in cmds:
                link.send(c.text)
            while ctl.move is not None:
                time.sleep(0.005)
                now = time.monotonic() * 1000
                ctl.last_frame_t = now                   # no camera here: keep frame-timeout quiet
                for c in ctl.tick(now):
                    link.send(c.text)
            print(f"  turn {i + 1}/{a.n} done")
            time.sleep(a.pause)
    except KeyboardInterrupt:
        pass
    finally:
        link.send("S")
        link.close()
    print("measure the total angle; new turn_ms = turn_ms * expected / measured")
    return 0


if __name__ == "__main__":
    sys.exit(main())
