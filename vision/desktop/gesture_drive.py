#!/usr/bin/env python3
"""TARS hand-gesture / follow driving on the Mac webcam (MediaPipe + OpenCV).

Same gesture logic as the QNX apps (vision/common/gesture.py, mirrored in C++).
Default is DRY RUN: commands are printed, nothing is sent.

    python gesture_drive.py                        # dry run, webcam 0, overlay window
    python gesture_drive.py --host 127.0.0.1       # drive the stub server (pi/build/tars_motor_host -b stub)
    python gesture_drive.py --host tars.local      # drive the real robot over Wi-Fi (fallback path)
    python gesture_drive.py --video clip.mp4 --headless --frames 300
    python gesture_drive.py --set follow_kp_turn=1.5 --set stable_frames=3

Keys in the window: q/Esc quit · space STOP · m toggle GESTURE/FOLLOW · p pause
Gestures: open palm STOP · thumbs up / point up FORWARD · fist / point down /
thumbs down BACK · point left/right TURN 30° · peace sign (hold) GESTURE<->FOLLOW
"""
from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "common"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

import gesture as g  # noqa: E402
from app import DEFAULT_MODEL, GestureRuntime, now_ms  # noqa: E402
from link import MotorLink  # noqa: E402

HAND_EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10),
              (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17),
              (17, 18), (18, 19), (19, 20)]
MODE_COLOR = {g.GESTURE_MODE: (255, 200, 0), g.FOLLOW_MODE: (0, 220, 255)}   # BGR


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="tars_motor host (omit = dry run, print commands)")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    ap.add_argument("--video", help="read frames from a video file instead of the webcam")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--conf", default=g.DEFAULT_CONF)
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL",
                    help="override a gesture.conf value (repeatable)")
    ap.add_argument("--model", default=DEFAULT_MODEL)
    ap.add_argument("--headless", action="store_true", help="no window")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = forever)")
    ap.add_argument("--realtime", action="store_true",
                    help="with --video: pace frames at the file's fps (default: as fast as possible)")
    ap.add_argument("--verbose", "-v", action="store_true", help="print every D and every reply")
    return ap.parse_args()


def draw(frame_rgb, res, rt: GestureRuntime, scale: int = 2):
    cfg = rt.cfg
    img = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    img = cv2.resize(img, (img.shape[1] * scale, img.shape[0] * scale), interpolation=cv2.INTER_LINEAR)
    H, W = img.shape[:2]
    mode = rt.ctl.mode
    col = MODE_COLOR[mode]

    if mode == g.FOLLOW_MODE:
        # centre deadband and target size guide
        db = cfg["follow_deadband"] * W
        overlay = img.copy()
        cv2.rectangle(overlay, (int(W / 2 - db), 0), (int(W / 2 + db), H), (0, 120, 160), -1)
        img = cv2.addWeighted(overlay, 0.25, img, 0.75, 0)
        cv2.line(img, (W // 2, 0), (W // 2, H), (0, 200, 255), 1)

    if res.lm:
        pts = [(int(x * W), int(y * H)) for x, y, _ in res.lm]
        for a, b in HAND_EDGES:
            cv2.line(img, pts[a], pts[b], (0, 255, 0), 2)
        for p in pts:
            cv2.circle(img, p, 3, (255, 255, 255), -1)
        if res.feat and mode == g.FOLLOW_MODE:
            cx, size = res.feat
            c = (int(cx * W), int(sum(res.lm[i][1] for i in g.PALM_POINTS) / 5 * H))
            cv2.circle(img, c, int(size * H), (0, 200, 255), 2)                         # current size
            cv2.circle(img, c, int(cfg["follow_target_size"] * H), (255, 255, 255), 1)  # target size

    # wheel bars (what we're commanding)
    if mode == g.FOLLOW_MODE:
        l, r = rt.ctl.f_cur
    elif rt.ctl.move:
        _, l, r, _ = rt.ctl.move
    else:
        l = r = 0.0
    for i, v in enumerate((l, r)):
        x0 = W - 70 + i * 35
        cv2.rectangle(img, (x0, H // 2 - 80), (x0 + 22, H // 2 + 80), (80, 80, 80), 1)
        y = int(H // 2 - v * 80)
        cv2.rectangle(img, (x0, min(y, H // 2)), (x0 + 22, max(y, H // 2)),
                      (0, 255, 0) if v >= 0 else (0, 0, 255), -1)
        cv2.putText(img, "LR"[i], (x0 + 5, H // 2 + 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.rectangle(img, (0, 0), (W, 34), (0, 0, 0), -1)
    cv2.putText(img, f"{mode.upper()}  {res.label}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    st = rt.stats
    stop = f"{st.stop_ms:.0f}ms" if st.stop_ms >= 0 else "-"
    line2 = f"fps {st.fps:.1f}  infer {st.infer_ms:.0f}ms  cam->cmd {st.cam_to_cmd_ms:.0f}ms  palm->STOP {stop}"
    cv2.rectangle(img, (0, H - 28), (W, H), (0, 0, 0), -1)
    cv2.putText(img, line2, (10, H - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    if rt.link.dry:
        cv2.putText(img, "DRY RUN", (W - 110, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return img


def main() -> int:
    a = parse_args()
    over = {}
    for kv in a.set:
        k, v = kv.split("=", 1)
        over[k.strip()] = float(v)
    cfg = g.load_config(a.conf, over)

    src = a.video if a.video else a.camera
    cap = cv2.VideoCapture(src)
    if not a.video:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print(f"cannot open {'video ' + a.video if a.video else 'camera %d' % a.camera}. "
              "macOS: allow camera access for your terminal in System Settings > Privacy.",
              file=sys.stderr)
        return 1
    vid_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0

    link = MotorLink(a.host, a.port, verbose=a.verbose)
    rt = GestureRuntime(cfg, link, a.model)
    print(f"[start] mode={rt.ctl.mode} {'DRY RUN' if link.dry else f'-> {a.host}:{a.port}'}", flush=True)
    labels = {}
    n = 0
    paused = False
    try:
        while True:
            if paused:
                k = cv2.waitKey(30) & 0xFF
                if k == ord("p"):
                    paused = False
                elif k in (ord("q"), 27):
                    break
                continue
            t_wall = time.monotonic()
            ok, bgr = cap.read()
            t_cap = now_ms()
            if not ok:
                if a.video:
                    break
                rt.camera_stalled()
                time.sleep(0.05)
                continue
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            res = rt.process(rgb, t_cap)
            labels[res.label] = labels.get(res.label, 0) + 1
            n += 1
            if not a.headless:
                cv2.imshow("TARS gesture (q quit, space stop, m mode, p pause)", draw(res.image, res, rt))
                k = cv2.waitKey(1) & 0xFF
                if k in (ord("q"), 27):
                    break
                if k == ord(" "):
                    with rt.ctl_lock:
                        cmds = rt.ctl._stop(now_ms(), "key")
                    rt._send(cmds, None)
                if k == ord("m"):
                    with rt.ctl_lock:
                        cmds = rt.ctl._stop(now_ms(), "key")
                        rt.ctl.mode = g.FOLLOW_MODE if rt.ctl.mode == g.GESTURE_MODE else g.GESTURE_MODE
                    rt._send(cmds, None)
                    print(f"[mode] {rt.ctl.mode} (key)", flush=True)
                if k == ord("p"):
                    paused = True
            if a.video and a.realtime:
                time.sleep(max(0.0, 1.0 / vid_fps - (time.monotonic() - t_wall)))
            if a.frames and n >= a.frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[end] {n} frames; labels seen: {labels}", flush=True)
        print(f"[end] {rt.status_line()}", flush=True)
        rt.close()
        link.close()
        cap.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
