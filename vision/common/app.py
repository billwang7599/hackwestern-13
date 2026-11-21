"""Camera-agnostic runtime shared by the Mac prototype and the QNX Python app.

    frames -> (flip, downscale) -> MediaPipe HandLandmarker -> classify ->
    Controller (gesture/follow) -> MotorLink, plus a 20 ms tick thread for
    resends / move ends / camera-stall stops, plus a once-a-second status line
    that is also sent to tars_motor as "VIS ..." so its STATUS shows it.

Latency we report (the QNX demo number):
    cam->cmd   frame captured -> command written to the socket and ACKed
    stop       same, for the last open-palm STOP specifically
"""
from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

import gesture as g
from link import MotorLink

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(HERE, "..", "models", "hand_landmarker.task")


def now_ms() -> float:
    return time.monotonic() * 1000.0


def make_landmarker(model_path: str, cfg: dict):
    import mediapipe as mp  # noqa: F401  (import here so tests don't need it)
    from mediapipe.tasks import python as mpp
    from mediapipe.tasks.python import vision

    if not os.path.exists(model_path):
        raise SystemExit(f"model not found: {model_path}\n  run vision/get_model.sh")
    opts = vision.HandLandmarkerOptions(
        base_options=mpp.BaseOptions(model_asset_path=model_path),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=cfg["min_detection_confidence"],
        min_hand_presence_confidence=cfg["min_detection_confidence"],
        min_tracking_confidence=cfg["min_tracking_confidence"],
    )
    return vision.HandLandmarker.create_from_options(opts)


@dataclass
class FrameResult:
    label: str
    lm: list | None
    feat: tuple | None
    aspect: float
    infer_ms: float
    cmds: list = field(default_factory=list)
    image: np.ndarray | None = None         # the (flipped, resized) RGB MediaPipe saw


@dataclass
class Stats:
    fps: float = 0.0
    infer_ms: float = 0.0
    cam_to_cmd_ms: float = 0.0
    stop_ms: float = -1.0                   # last palm STOP: camera -> S acked
    stop_ago_s: float = -1.0
    frames: int = 0


class GestureRuntime:
    def __init__(self, cfg: dict, link: MotorLink, model_path: str = DEFAULT_MODEL,
                 log_every_s: float = 1.0):
        self.cfg = cfg
        self.link = link
        self.ctl = g.Controller(cfg)
        self.ctl_lock = threading.Lock()
        self.landmarker = make_landmarker(model_path, cfg)
        self.stats = Stats()
        self._last_frame = None
        self._last_ts = -1
        self._stop_evt = threading.Event()
        self._last_stop_wall = None
        self._log_every = log_every_s
        self._next_log = now_ms() + log_every_s * 1000
        self._last_label = g.NO_HAND
        self._ticker = threading.Thread(target=self._tick_loop, daemon=True)
        self._ticker.start()

    # -- commands ----------------------------------------------------------------
    def _send(self, cmds, t_cap: float | None) -> None:
        for c in cmds:
            self.link.send(c.text)
            if not c.text.startswith("D") or c.reason not in ("resend", "follow"):
                print(f"[cmd] {c.text:<14} {c.reason:<14} mode={self.ctl.mode}", flush=True)
            if t_cap is not None:
                lat = now_ms() - t_cap
                self.stats.cam_to_cmd_ms = lat
                if c.reason == "palm":
                    self.stats.stop_ms = lat
                    self._last_stop_wall = time.monotonic()
                    print(f"[lat] PALM -> STOP: camera frame -> S acked in {lat:.0f} ms", flush=True)

    def _tick_loop(self) -> None:
        while not self._stop_evt.wait(0.02):
            with self.ctl_lock:
                cmds = self.ctl.tick(now_ms())
            if cmds:
                self._send(cmds, None)
            self._maybe_log()

    # -- per frame ---------------------------------------------------------------
    def process(self, rgb: np.ndarray, t_cap: float) -> FrameResult:
        """rgb: HxWx3 uint8 RGB frame straight from the camera. t_cap: now_ms() at capture."""
        import mediapipe as mp

        img = rgb
        if self.cfg["flip"] >= 0.5:
            img = cv2.flip(img, 1)
        w = int(self.cfg["infer_width"])
        if w > 0 and img.shape[1] > w:
            h = int(round(img.shape[0] * w / img.shape[1]))
            img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        img = np.ascontiguousarray(img)
        aspect = img.shape[1] / img.shape[0]

        ts = int(t_cap)
        if ts <= self._last_ts:                  # VIDEO mode needs increasing timestamps
            ts = self._last_ts + 1
        self._last_ts = ts
        t0 = now_ms()
        res = self.landmarker.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=img), ts)
        infer = now_ms() - t0

        lm = None
        if res.hand_landmarks:
            lm = [(p.x, p.y, p.z) for p in res.hand_landmarks[0]]
        label = g.classify(lm, aspect, self.cfg)
        feat = g.hand_features(lm, aspect)
        with self.ctl_lock:
            cmds = self.ctl.on_frame(now_ms(), label, feat)
        self._send(cmds, t_cap)

        st = self.stats
        st.frames += 1
        if self._last_frame is not None:
            dt = t_cap - self._last_frame
            if dt > 0:
                st.fps = 1000.0 / dt if st.fps == 0 else 0.9 * st.fps + 0.1 * (1000.0 / dt)
        self._last_frame = t_cap
        st.infer_ms = infer if st.infer_ms == 0 else 0.9 * st.infer_ms + 0.1 * infer
        if label != self._last_label:
            print(f"[see] {label}", flush=True)
            self._last_label = label
        return FrameResult(label, lm, feat, aspect, infer, cmds, img)

    # -- status ------------------------------------------------------------------
    def status_line(self) -> str:
        st = self.stats
        stop = f"{st.stop_ms:.0f}ms" if st.stop_ms >= 0 else "-"
        c = self.ctl
        extra = ""
        if c.mode == g.FOLLOW_MODE:
            extra = f" wheels={c.f_cur[0]:+.2f},{c.f_cur[1]:+.2f}"
        elif c.moving:
            extra = f" move={c.moving}"
        return (f"mode={c.mode} fps={st.fps:.1f} infer={st.infer_ms:.0f}ms "
                f"cam->cmd={st.cam_to_cmd_ms:.0f}ms palm->stop={stop} "
                f"see={self._last_label}{extra} link_rtt={self.link.last_rtt_ms:.1f}ms")

    def _maybe_log(self) -> None:
        t = now_ms()
        if t < self._next_log:
            return
        self._next_log = t + self._log_every * 1000
        print(f"[status] {self.status_line()}", flush=True)
        if self.link.dry:
            return
        st = self.stats
        # tars_motor shows these in STATUS (vis_mode= vis_fps= vis_stop_ms= ...)
        self.link.send(f"VIS {self.ctl.mode} {st.fps:.1f} {st.cam_to_cmd_ms:.0f} "
                       f"{st.stop_ms:.0f} {self._last_label}")

    def camera_stalled(self) -> None:
        """Call when a frame read timed out; the tick thread does the STOP."""
        print("[cam] frame timeout", file=sys.stderr, flush=True)

    def close(self) -> None:
        self._stop_evt.set()
        self._ticker.join(timeout=1)
        self.link.send("S")
        self.landmarker.close()
