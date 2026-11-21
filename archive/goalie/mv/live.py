"""Real-time ball tracker on the webcam (or a recorded clip).

Stages (build/verify them in this order):
  --stage detect   circle + "(u,v) d=..px conf circ blur" + HSV mask inset
  --stage track    + SPEED / HEIGHT / DIST / DIR label and velocity arrow
  --stage predict  + ballistic fit, crossing point on the goal-face view (default)

  python live.py --stage detect --device 0
  python live.py --video out/synth_raw.mp4 --stage track
  python live.py --record                      # also saves raw frames to out/rec_*.mp4
  python live.py --headless 90 --stage detect  # no window: process 90 frames, save
                                               # out/live_last.png, print stats
Keys: c clear track | s save last ~4 s as clip | m mask inset | q / Esc quit
Output hook: --emit stdout | --emit udp:HOST:PORT  ->  lines
  P <t_ms> <x> <y_height> <t_to_cross_ms> <conf>
"""
from __future__ import annotations

import argparse
import collections
import socket
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from config import BALLS, DEFAULT_CONFIG, Config
from overlay import STAGES, render
from tracker import Tracker

OUT = Path(__file__).resolve().parent / "out"


class Grabber(threading.Thread):
    """Capture thread: always holds the newest frame + its grab timestamp, so
    slow processing drops frames instead of building latency."""

    def __init__(self, cap):
        super().__init__(daemon=True)
        self.cap = cap
        self.lock = threading.Lock()
        self.frame, self.t, self.seq = None, 0.0, 0
        self.running = True
        self.fps_meas = 0.0

    def run(self):
        n, t0 = 0, time.monotonic()
        while self.running:
            ok, f = self.cap.read()
            t = time.monotonic()
            if not ok:
                time.sleep(0.005)
                continue
            with self.lock:
                self.frame, self.t, self.seq = f, t, self.seq + 1
            n += 1
            if t - t0 >= 1.0:
                self.fps_meas, n, t0 = n / (t - t0), 0, t

    def latest(self, last_seq):
        with self.lock:
            if self.seq == last_seq:
                return None, 0.0, last_seq
            return self.frame, self.t, self.seq


def open_camera(dev: int, w=640, h=480, fps=60, exposure=None):
    cap = cv2.VideoCapture(dev, cv2.CAP_AVFOUNDATION) if sys.platform == "darwin" else cv2.VideoCapture(dev)
    if not cap.isOpened():
        return None, {}
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    info = {}
    if exposure is not None:
        # AVFoundation mostly ignores these; V4L2 uses 0.25 = manual / 1 = manual
        r1 = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        r2 = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        r3 = cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
        info["exposure_set_ok"] = (r1, r2, r3)
    info.update(width=cap.get(cv2.CAP_PROP_FRAME_WIDTH), height=cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                fps_prop=cap.get(cv2.CAP_PROP_FPS), exposure=cap.get(cv2.CAP_PROP_EXPOSURE),
                auto_exposure=cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
    return cap, info


class Emitter:
    def __init__(self, spec: str | None):
        self.sock = self.addr = None
        self.stdout = spec == "stdout"
        if spec and spec.startswith("udp:"):
            _, host, port = spec.split(":")
            self.addr = (host, int(port))
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def __call__(self, st):
        c = st.cross
        if not (self.stdout or self.sock) or not c.coming:
            return
        line = (f"P {int(st.t * 1000)} {c.x:.4f} {c.y:.4f} "
                f"{max(0, (c.t_cross - st.t) * 1000):.0f} {c.conf:.2f}\n")
        if self.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
        if self.sock:
            self.sock.sendto(line.encode(), self.addr)


def build_cfg(a) -> Config:
    cfg = Config.load(a.config)
    if a.ball:
        cfg.set_ball(a.ball if a.ball in BALLS else float(a.ball))
    if a.rolling:
        cfg.rolling = True
    if a.stage == "detect":
        cfg.detect.static_reject = False   # a ball held still must stay visible
    if a.bg:
        cfg.detect.use_bg = True
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", default="predict", choices=STAGES)
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--video", help="replay a recorded clip instead of the camera")
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    ap.add_argument("--ball", help="pingpong | tennis | soccer | diameter in m")
    ap.add_argument("--rolling", action="store_true", help="ball rolls on the floor")
    ap.add_argument("--bg", action="store_true", help="MOG2 motion mask")
    ap.add_argument("--fps", type=int, default=60, help="requested camera fps")
    ap.add_argument("--exposure", type=float, default=None,
                    help="try manual exposure (driver units, e.g. -7 on UVC/V4L2)")
    ap.add_argument("--record", action="store_true", help="save raw frames to out/rec_<time>.mp4")
    ap.add_argument("--emit", help="stdout | udp:HOST:PORT")
    ap.add_argument("--headless", type=int, default=0, help="process N frames, no window")
    a = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    cfg = build_cfg(a)
    emit = Emitter(a.emit)

    grab, cap, vid_fps = None, None, None
    if a.video:
        cap = cv2.VideoCapture(a.video)
        if not cap.isOpened():
            sys.exit(f"cannot open {a.video}")
        vid_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        info = {"video": a.video, "fps": vid_fps}
    else:
        cap, info = open_camera(a.device, fps=a.fps, exposure=a.exposure)
        if cap is None:
            sys.exit(f"camera {a.device} did not open. On macOS grant Camera access to your "
                     "terminal app (System Settings > Privacy & Security > Camera), then retry. "
                     "Probe devices with: python tools/probe_cam.py")
        grab = Grabber(cap)
        grab.start()
    print("capture:", info, file=sys.stderr)

    trk = None
    rec = None
    ring = collections.deque(maxlen=240)
    show_mask = True
    last_seq, k, t_last = 0, 0, None
    proc_ms = collections.deque(maxlen=30)
    lat_ms = collections.deque(maxlen=30)
    n_det = 0
    win = "goalie mv"
    t_start = time.monotonic()
    while True:
        if a.video:
            ok, frame = cap.read()
            if not ok:
                break
            t = k / vid_fps
        else:
            frame, t, last_seq = grab.latest(last_seq)
            if frame is None:
                if grab.seq == 0 and time.monotonic() - t_start > 5:
                    sys.exit("no frames from the camera after 5 s (permission or wrong --device)")
                time.sleep(0.002)
                continue
        if trk is None:
            h, w = frame.shape[:2]
            if (w, h) != (cfg.camera.w, cfg.camera.h):
                cfg.camera = cfg.camera.scaled(w, h)
            trk = Tracker(cfg)
        p0 = time.monotonic()
        st = trk.step(frame, t)
        proc_ms.append((time.monotonic() - p0) * 1000)
        if not a.video:
            lat_ms.append((time.monotonic() - t) * 1000)
        n_det += st.det is not None
        emit(st)
        fps_s = f"{grab.fps_meas:.0f}" if grab else f"{vid_fps:.0f} (file)"
        hud = [f"stage {a.stage} | cam {fps_s} fps | proc {np.mean(proc_ms):.1f} ms"
               + (f" | latency {np.mean(lat_ms):.0f} ms" if lat_ms else "")
               + (" | REC" if rec else ""),
               f"ball {cfg.ball_diam*1000:.0f}mm {'rolling' if cfg.rolling else 'air'} | c clear  s save  m mask  q quit"]
        out = render(frame, st, cfg, a.stage, trk.det.mask if show_mask else None, hud)
        ring.append(frame)
        if a.record:
            if rec is None:
                h, w = frame.shape[:2]
                rec = cv2.VideoWriter(str(OUT / f"rec_{time.strftime('%H%M%S')}.mp4"),
                                      cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
            rec.write(frame)
        k += 1
        if a.headless:
            if k >= a.headless:
                cv2.imwrite(str(OUT / "live_last.png"), out)
                break
            continue
        cv2.imshow(win, out)
        key = cv2.waitKey(1 if not a.video else max(1, int(1000 / vid_fps))) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("c"):
            trk.clear()
        if key == ord("m"):
            show_mask = not show_mask
        if key == ord("s") and ring:
            path = OUT / f"clip_{time.strftime('%H%M%S')}.mp4"
            h, w = ring[0].shape[:2]
            vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
            for f in ring:
                vw.write(f)
            vw.release()
            print("saved", path, file=sys.stderr)
    if grab:
        grab.running = False
    if rec:
        rec.release()
    cap.release()
    cv2.destroyAllWindows()
    print(f"frames {k}, detections {n_det}, proc {np.mean(proc_ms) if proc_ms else 0:.1f} ms/frame",
          file=sys.stderr)


if __name__ == "__main__":
    main()
