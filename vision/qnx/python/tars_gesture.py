#!/usr/bin/env python3
"""TARS gesture / follow driving ON the Pi 5 under QNX (fast path).

QNX's prebuilt MediaPipe (`sudo apk add mediapipe`) + the QNX Camera Library
(libcamapi) + the shared gesture logic in vision/common -> tars_motor on
localhost:5555. Headless; logs mode, fps and camera->command latency.

    python3 tars_gesture.py --list-units           # which camera units exist
    python3 tars_gesture.py --dry-run              # print commands, don't drive
    python3 tars_gesture.py                        # drive (tars_motor must be running)
    python3 tars_gesture.py --unit 1 --save-every 30   # save annotated jpgs to check the view

Same flags for tuning as the Mac prototype: --set key=value overrides gesture.conf.

Runs fully on the robot: teach a new pose with the 🤙 TEACH sign (or the Uno
button), learned poses persist in --learned (default vision/learned.json).
At boot it's started by qnx/autostart/tars_autostart.sh. If stdin is a
terminal (ssh), you can also type: teach | forget | list | stop.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "common"))

import cv2  # noqa: E402

import gesture as g  # noqa: E402
from app import DEFAULT_LEARNED, DEFAULT_MODEL, GestureRuntime, now_ms  # noqa: E402
from link import MotorLink  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--unit", type=int, default=0, help="QNX camera unit (0 = first available)")
    ap.add_argument("--list-units", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--dry-run", action="store_true", help="print commands instead of sending")
    ap.add_argument("--conf", default=g.DEFAULT_CONF)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = forever)")
    ap.add_argument("--frame-timeout", type=float, default=1.0, help="seconds to wait per frame")
    ap.add_argument("--packed-order", choices=["yuyv", "uyvy"], default="yuyv",
                    help="byte order for packed 4:2:2 USB frames (swap if colours look wrong)")
    ap.add_argument("--rgb4-order", choices=["bgrx", "rgbx"], default="bgrx")
    ap.add_argument("--save-every", type=int, default=0,
                    help="save the annotated MediaPipe input every N frames to ./gesture_view.jpg")
    ap.add_argument("--learned", default=DEFAULT_LEARNED, help="learned poses JSON ('' = don't load/save)")
    ap.add_argument("--no-buttons", action="store_true", help="don't poll tars_motor STATUS for the Uno button")
    ap.add_argument("--console", choices=["auto", "on", "off"], default="auto",
                    help="stdin commands (auto = only when stdin is a terminal)")
    ap.add_argument("--verbose", "-v", action="store_true")
    a = ap.parse_args()

    from qnx_cam import GestureCamera, list_units

    units = list_units()
    if a.list_units:
        print(f"camera units: {units}")
        return 0
    unit = a.unit or (units[0] if units else 0)
    if not unit:
        print("no camera units found: is the sensor service running with your usb_camera.conf?",
              file=sys.stderr)
        return 1

    over = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        over[k.strip()] = float(v)
    cfg = g.load_config(a.conf, over)

    link = MotorLink(None if a.dry_run else a.host, a.port, verbose=a.verbose)
    rt = GestureRuntime(cfg, link, a.model, learned_path=a.learned or None,
                        poll_buttons=not a.no_buttons)
    if a.console == "on" or (a.console == "auto" and sys.stdin.isatty()):
        rt.start_console()
    cam = GestureCamera(unit, a.packed_order, a.rgb4_order)
    cam.open()
    print(f"[start] camera unit {unit} (available {units}); mode={rt.ctl.mode}; "
          f"{'DRY RUN' if link.dry else f'-> {a.host}:{a.port}'}", flush=True)
    n = 0
    try:
        while True:
            try:
                rgb = cam.read_rgb(a.frame_timeout)
            except TimeoutError:
                rt.camera_stalled()            # tick thread sends S after frame_timeout_ms
                continue
            t_cap = now_ms()
            res = rt.process(rgb, t_cap)
            n += 1
            if a.save_every and n % a.save_every == 0:
                img = cv2.cvtColor(res.image, cv2.COLOR_RGB2BGR)
                if res.lm:
                    H, W = img.shape[:2]
                    for x, y, _ in res.lm:
                        cv2.circle(img, (int(x * W), int(y * H)), 2, (0, 255, 0), -1)
                cv2.putText(img, f"{rt.ctl.mode} {res.label} {rt.brain.state}", (5, 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 0, 255), 1)
                cv2.imwrite("gesture_view.jpg", img)
            if a.frames and n >= a.frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[end] {n} frames; {rt.status_line()}", flush=True)
        rt.close()
        link.close()
        cam.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
