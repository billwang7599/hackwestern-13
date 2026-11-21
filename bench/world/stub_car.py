#!/usr/bin/env python3
"""STUB CAR: the fallback when the virtual SM11 (bench/car/) isn't there.

NOT a car model. tars_motor runs `-b stub` (no firmware, no physics). This process polls
tars_motor STATUS and turns its dead-reckoned pose= changes into "truth" on the car-truth
port (bench/README.md 5590 format, with "stub": true), so the world sim has a car to sense.
It also takes the car-control port (5591): "pose" teleports, "walls" are honoured (the truth
stops at a wall and reports contact, while tars_motor's dead reckoning keeps counting, like a
stalled wheel would). Other faults are ignored and logged.

    python3 bench/world/stub_car.py --motor 5555 --truth 5590 --ctl 5591 [--room default]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C  # noqa: E402

RADIUS = 0.12          # SM11-ish footprint radius for wall contact
POSE_RE = re.compile(r" pose=(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)")
EPOCH_RE = re.compile(r" pose_epoch=(\d+)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="stub car: tars_motor -b stub pose -> car truth")
    ap.add_argument("--motor", type=int, default=5555)
    ap.add_argument("--truth", type=int, default=5590, help="UDP: where truth goes")
    ap.add_argument("--ctl", type=int, default=5591, help="UDP: car control in")
    ap.add_argument("--room", default="default", help="start pose from this room until a teleport")
    ap.add_argument("--hz", type=float, default=50)
    a = ap.parse_args(argv)

    start = C.load_room_file(a.room)["start"] if not a.room.endswith(".json") else \
        C.normalise_room(json.load(open(a.room)))["start"]
    lock = threading.Lock()
    st = {"x": start["x"], "y": start["y"], "th": start["th"], "walls": [], "contact": False}
    stop = threading.Event()

    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.bind(("127.0.0.1", a.ctl))
    ctl.settimeout(0.2)

    def ctl_loop():
        while not stop.is_set():
            try:
                data = ctl.recv(65535)
                msg = json.loads(data)
            except socket.timeout:
                continue
            except (OSError, ValueError):
                if stop.is_set():
                    return
                continue
            with lock:
                if "pose" in msg:
                    p = msg["pose"]
                    st["x"], st["y"], st["th"] = float(p.get("x", 0)), float(p.get("y", 0)), float(p.get("th", 0))
                if "walls" in msg:
                    st["walls"] = [tuple(map(float, s)) for s in msg["walls"]]
            if "fault" in msg and msg["fault"] != "clear":
                print(f"stub_car: fault {msg['fault']!r} ignored (stub car has no physics)", file=sys.stderr)
            if "set" in msg:
                print("stub_car: set ignored (stub car has no profile)", file=sys.stderr)

    threading.Thread(target=ctl_loop, daemon=True).start()

    m = None
    for _ in range(200):
        try:
            m = socket.create_connection(("127.0.0.1", a.motor), timeout=2)
            break
        except OSError:
            time.sleep(0.05)
    if m is None:
        print(f"stub_car: tars_motor never accepted on {a.motor}", file=sys.stderr)
        return 1
    f = m.makefile("rw", encoding="ascii", newline="\n")
    f.write("WHO stubcar\n")
    f.flush()
    f.readline()
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    prev = None
    prev_epoch = None
    last = time.monotonic()
    print(f"stub_car: STUB CAR (no physics) motor=:{a.motor} truth->:{a.truth} ctl=:{a.ctl}", file=sys.stderr, flush=True)
    try:
        while True:
            try:
                f.write("STATUS\n")
                f.flush()
                r = f.readline()
            except (OSError, ValueError):
                r = ""
            if not r:
                print("stub_car: tars_motor gone", file=sys.stderr)
                return 0
            mm = POSE_RE.search(r)
            now = time.monotonic()
            dt = max(1e-3, now - last)
            last = now
            v = w = 0.0
            fields = r.split()
            try:
                out_l, out_r = float(fields[1]), float(fields[2])
            except (IndexError, ValueError):
                out_l = out_r = 0.0
            me = EPOCH_RE.search(r)
            epoch = me.group(1) if me else None
            if epoch != prev_epoch:          # POSE RESET (new room): a jump in pose=, not motion
                prev, prev_epoch = None, epoch
            if mm:
                x, y, h = (float(s) for s in mm.groups())
                if prev is not None:
                    px, py, ph = prev
                    dh = (h - ph + 180.0) % 360.0 - 180.0
                    mid = math.radians(ph + dh / 2)
                    ds = ((x - px) * math.cos(mid) + (y - py) * math.sin(mid)) / 1000.0
                    dth = -math.radians(dh)               # motor clockwise -> world CCW
                    with lock:
                        tmid = st["th"] + dth / 2
                        nx, ny = st["x"] + ds * math.cos(tmid), st["y"] + ds * math.sin(tmid)
                        hit = any(C.dist_point_seg(nx, ny, *s) < RADIUS for s in st["walls"])
                        was = any(C.dist_point_seg(st["x"], st["y"], *s) < RADIUS for s in st["walls"])
                        # moving into a wall: stay put (contact). Already touching: allow moving away.
                        if hit and not (was and min(C.dist_point_seg(nx, ny, *s) for s in st["walls"]) >
                                        min(C.dist_point_seg(st["x"], st["y"], *s) for s in st["walls"])):
                            st["contact"] = abs(ds) > 1e-6 or st["contact"]
                        else:
                            st["x"], st["y"] = nx, ny
                            st["contact"] = False if abs(ds) > 1e-6 else st["contact"] and hit
                        st["th"] = C.wrap(st["th"] + dth)
                    v, w = ds / dt, dth / dt
                prev = (x, y, h)
            with lock:
                msg = {"t": int(time.time() * 1000), "x": round(st["x"], 4), "y": round(st["y"], 4),
                       "th": round(st["th"], 5), "v": round(v, 3), "w": round(w, 3),
                       "wl": round(v - w * 0.07, 3), "wr": round(v + w * 0.07, 3),
                       "pwm_l": int(out_l * 255), "pwm_r": int(out_r * 255), "batt_v": None,
                       "contact": st["contact"], "stub": True}
            try:
                out.sendto(json.dumps(msg, separators=(",", ":")).encode(), ("127.0.0.1", a.truth))
            except OSError:
                pass
            time.sleep(1.0 / a.hz)
    except KeyboardInterrupt:
        return 0
    finally:
        stop.set()


if __name__ == "__main__":
    sys.exit(main())
