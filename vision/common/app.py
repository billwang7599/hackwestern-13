"""Camera-agnostic runtime shared by the Mac prototype and the QNX Python app.

    frames -> (flip, downscale) -> MediaPipe HandLandmarker -> learn.Brain
    (built-in rules, learned poses, teach state machine, gesture.Controller) ->
    MotorLink, plus a 20 ms tick thread for resends / move ends / camera-stall
    stops / teach timers, plus a once-a-second status line that is also sent to
    tars_motor as "VIS ..." so its STATUS shows it.

Pet mode (GESTURES.md section 8) runs inside the same Brain; its events are
logged as [pet]. If tars_motor refuses a D (S from another client, watchdog,
tip, bump, guard), the move/stream is dropped (Brain.server_stop) so it never
restarts by itself.

Teach triggers that need no laptop: the TEACH hand sign (handled in Brain), and
the Uno button, read by polling tars_motor STATUS for btn_teach= / btn_forget=
counters (see GESTURES.md section 7). Learned poses are saved to learned_path
whenever they change and loaded at start.

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

import re

import gesture as g
import learn as L
from link import MotorLink

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(HERE, "..", "models", "hand_landmarker.task")
DEFAULT_LEARNED = os.path.join(HERE, "..", "learned.json")
BTN_RE = re.compile(r"\bbtn_(teach|forget)=(\d+)")


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
    match: tuple | None = None              # (nearest learned pose, confidence, accepted)
    rule: str = g.NO_HAND                   # built-in rule label (before learned poses)


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
                 log_every_s: float = 1.0, learned_path: str | None = DEFAULT_LEARNED,
                 poll_buttons: bool = True, landmarker=None):
        self.cfg = cfg
        self.link = link
        self.brain = L.Brain(cfg)
        self.ctl = self.brain.ctl
        self.ctl_lock = threading.Lock()
        self.learned_path = learned_path
        if learned_path:
            try:
                n = self.brain.load(learned_path)
                print(f"[teach] loaded {n} learned pose(s) from {learned_path}"
                      + "".join(f"\n[teach]   {c.name} -> {c.action}" for c in self.brain.learner.classes),
                      flush=True)
            except (ValueError, KeyError) as e:
                print(f"[teach] could not read {learned_path} ({e}); starting empty", file=sys.stderr, flush=True)
        self.poll_buttons = poll_buttons and not link.dry
        self._btn: dict[str, int] | None = None
        self._next_poll = 0.0
        self.landmarker = landmarker if landmarker is not None else make_landmarker(model_path, cfg)
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
            reply = self.link.send(c.text)
            if not c.text.startswith("D") or c.reason not in ("resend", "follow", "pet_stream"):
                print(f"[cmd] {c.text:<14} {c.reason:<14} mode={self.ctl.mode}", flush=True)
            if c.text.startswith("D ") and reply and reply.startswith("ERR"):
                # the server stopped us (another client's S, watchdog, tip, guard): stop wins
                with self.ctl_lock:
                    self.brain.server_stop(now_ms())
                print(f"[cmd] server refused D ({reply}); move dropped", flush=True)
                break
            if t_cap is not None:
                lat = now_ms() - t_cap
                self.stats.cam_to_cmd_ms = lat
                if c.reason == "palm":
                    self.stats.stop_ms = lat
                    self._last_stop_wall = time.monotonic()
                    print(f"[lat] PALM -> STOP: camera frame -> S acked in {lat:.0f} ms", flush=True)

    def _after(self) -> None:
        """Drain teach events and persist learned poses (call with ctl_lock held)."""
        for e in self.brain.events:
            print(f"[teach] {e}", flush=True)
        self.brain.events = []
        for e in self.brain.pet.events:
            print(f"[pet] {e} (energy {self.brain.pet.energy:.2f})", flush=True)
        self.brain.pet.events = []
        if self.brain.dirty and self.learned_path:
            try:
                self.brain.save(self.learned_path)
                print(f"[teach] saved {len(self.brain.learner.classes)} pose(s) -> {self.learned_path}", flush=True)
            except OSError as e:
                self.brain.dirty = False
                print(f"[teach] save failed: {e}", file=sys.stderr, flush=True)

    def button(self, kind: str) -> None:
        """'teach' (short press / desktop 't' / console) or 'forget' (long press)."""
        with self.ctl_lock:
            cmds = self.brain.button(now_ms(), kind)
            self._after()
        self._send(cmds, None)

    def _poll_buttons(self) -> None:
        """Uno button via tars_motor: STATUS ... btn_teach=<n> btn_forget=<n> (counters)."""
        t = now_ms()
        if not self.poll_buttons or t < self._next_poll:
            return
        self._next_poll = t + 200
        reply = self.link.send("STATUS")
        if not reply or not reply.startswith("OK"):
            return
        seen = {k: int(v) for k, v in BTN_RE.findall(reply)}
        if not seen:
            return
        if self._btn is None:
            self._btn = seen                  # baseline: presses before we started don't count
            return
        for kind in ("forget", "teach"):
            if seen.get(kind, 0) > self._btn.get(kind, 0):
                print(f"[teach] button: {kind}", flush=True)
                self.button(kind)
        self._btn = seen

    def _tick_loop(self) -> None:
        while not self._stop_evt.wait(0.02):
            with self.ctl_lock:
                cmds = self.brain.tick(now_ms())
                self._after()
            if cmds:
                self._send(cmds, None)
            self._poll_buttons()
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
        return self.process_landmarks(lm, aspect, t_cap, infer, img)

    def process_landmarks(self, lm, aspect: float, t_cap: float, infer: float = 0.0,
                          img: np.ndarray | None = None) -> FrameResult:
        """Everything after MediaPipe (also used by tests with synthetic hands)."""
        feat = g.hand_features(lm, aspect)
        with self.ctl_lock:
            cmds = self.brain.on_frame(now_ms(), lm, aspect)
            label, match, rule = self.brain.label, self.brain.match, self.brain.last_rule
            self._after()
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
        return FrameResult(label, lm, feat, aspect, infer, cmds, img, match, rule)

    # -- status ------------------------------------------------------------------
    def status_line(self) -> str:
        st = self.stats
        stop = f"{st.stop_ms:.0f}ms" if st.stop_ms >= 0 else "-"
        c = self.ctl
        extra = ""
        if c.mode == g.FOLLOW_MODE or (c.mode == g.PET_MODE and c.f_active):
            extra = f" wheels={c.f_cur[0]:+.2f},{c.f_cur[1]:+.2f}"
        elif c.moving:
            extra = f" move={c.moving}"
        b = self.brain
        if b.pet.active:
            extra += f" pet={b.pet.state} energy={b.pet.energy:.2f}"
        if b.state != L.IDLE:
            extra += f" teach={b.state}"
        if b.match:
            extra += f" match={b.match[0]}:{b.match[1]:.2f}"
        extra += f" learned={len(b.learner.classes)}"
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
        mode = self.ctl.mode if self.brain.state == L.IDLE else "teach:" + self.brain.state
        if self.brain.state == L.IDLE and self.brain.pet.active:
            mode = "pet:" + self.brain.pet.state          # <= 15 chars (tars_motor vis_mode)
        self.link.send(f"VIS {mode} {st.fps:.1f} {st.cam_to_cmd_ms:.0f} "
                       f"{st.stop_ms:.0f} {self._last_label}")

    def start_console(self) -> None:
        """Optional stdin commands (ssh / dev only; the robot never needs them):
        teach | forget | list | stop. Ends quietly when stdin is closed (autostart)."""
        def run():
            for line in sys.stdin:
                cmd = line.strip().lower()
                if cmd in ("teach", "t"):
                    self.button("teach")
                elif cmd == "forget":
                    self.button("forget")
                elif cmd in ("list", "l"):
                    with self.ctl_lock:
                        cs = [(c.name, c.action, c.thr, len(c.raw)) for c in self.brain.learner.classes]
                    print(f"[teach] {len(cs)} learned: " + ", ".join(
                        f"{n}->{a} (r={t:.2f}, {k} frames)" for n, a, t, k in cs), flush=True)
                elif cmd in ("stop", "s"):
                    with self.ctl_lock:
                        cmds = self.ctl._stop(now_ms(), "key")
                    self._send(cmds, None)
                elif cmd:
                    print("[console] commands: teach | forget | list | stop", flush=True)
        threading.Thread(target=run, daemon=True).start()

    def camera_stalled(self) -> None:
        """Call when a frame read timed out; the tick thread does the STOP."""
        print("[cam] frame timeout", file=sys.stderr, flush=True)

    def close(self) -> None:
        self._stop_evt.set()
        self._ticker.join(timeout=1)
        self.link.send("S")
        self.landmarker.close()
