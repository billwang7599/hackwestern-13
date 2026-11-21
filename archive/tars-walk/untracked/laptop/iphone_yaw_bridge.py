"""iPhone yaw -> TARS (optional closed-loop turns).

    python iphone_yaw_bridge.py --pi tars.local            # listen on UDP 5600
    python iphone_yaw_bridge.py --pi 127.0.0.1 --fake      # no phone: slow yaw ramp

Phone: SensorLog (stream JSON over UDP) or ZIG SIM (JSON, quaternion or
attitude), target = this laptop's IP, port 5600, 30-100 Hz. Tape the phone flat
on TARS's top. Run the Pi server with `-y 5558`.

Every packet is turned into "YAW <deg>" and sent to the Pi (UDP 5558). Accepted:
  * JSON with a yaw-ish key: motionYaw / yaw / attitude.yaw (radians if |v| <= 3.2
    and the key is motion*/attitude*, else degrees; --radians / --degrees forces it)
  * JSON with a quaternion {x, y, z, w}  (ZIG SIM)
  * text "yaw <num>" / CSV "...,yaw,<num>"
If turns get corrected the WRONG way, send `CAL YAWSIGN -1` to the Pi.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import socket
import time


def _walk(o, path=""):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from _walk(v, f"{path}.{k}" if path else str(k))
    else:
        yield path, o


def _quat_yaw(q: dict) -> float | None:
    try:
        x, y, z, w = (float(q[k]) for k in ("x", "y", "z", "w"))
    except (KeyError, TypeError, ValueError):
        return None
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _find_quat(o) -> dict | None:
    if isinstance(o, dict):
        if all(k in o for k in ("x", "y", "z", "w")):
            return o
        for v in o.values():
            q = _find_quat(v)
            if q:
                return q
    return None


def parse_yaw(data: bytes, units: str = "auto") -> float | None:
    """Degrees, or None if the packet has no yaw."""
    text = data.decode("utf-8", "replace").strip()
    try:
        obj = json.loads(text)
    except ValueError:
        obj = None
    if isinstance(obj, dict):
        for path, v in _walk(obj):
            leaf = path.lower().split(".")[-1]
            if leaf.endswith("yaw") and isinstance(v, (int, float)):
                rad = units == "radians" or (units == "auto" and abs(v) <= 3.2 and
                                              ("motion" in path.lower() or "attitude" in path.lower()))
                return math.degrees(v) if rad else float(v)
        q = _find_quat(obj)
        if q:
            return _quat_yaw(q)
        return None
    m = re.search(r"yaw[\s,:=]+(-?\d+(?:\.\d+)?)", text, re.I)
    if m:
        v = float(m.group(1))
        return math.degrees(v) if units == "radians" else v
    return None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--listen", type=int, default=5600)
    ap.add_argument("--pi", default="tars.local")
    ap.add_argument("--pi-port", type=int, default=5558)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--radians", action="store_const", const="radians", dest="units")
    g.add_argument("--degrees", action="store_const", const="degrees", dest="units")
    ap.add_argument("--fake", action="store_true", help="send a slow yaw ramp instead of listening")
    a = ap.parse_args()
    units = a.units or "auto"
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (a.pi, a.pi_port)

    if a.fake:
        print(f"fake yaw ramp -> {dest}")
        t0 = time.monotonic()
        while True:
            y = (time.monotonic() - t0) * 10.0
            out.sendto(f"YAW {((y + 180) % 360) - 180:.2f}".encode(), dest)
            time.sleep(0.02)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", a.listen))
    print(f"listening on udp {a.listen}, forwarding YAW to {dest}")
    n, last_print = 0, 0.0
    while True:
        data, _ = sock.recvfrom(65535)
        y = parse_yaw(data, units)
        if y is None:
            continue
        out.sendto(f"YAW {y:.2f}".encode(), dest)
        n += 1
        if time.monotonic() - last_print > 1.0:
            print(f"\ryaw {y:+7.1f} deg  ({n} pkts)", end="", flush=True)
            last_print = time.monotonic()


if __name__ == "__main__":
    main()
