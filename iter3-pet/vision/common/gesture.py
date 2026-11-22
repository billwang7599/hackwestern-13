"""QNXplorer gesture logic: hand landmarks -> gesture label -> motor commands.

Pure Python, no dependencies, so it runs on the Mac, on QNX, and in tests.
iter3-pet/vision/common/gesture.hpp is a line-for-line C++ port; tests/test_parity.py
checks the two produce identical labels and command streams. If you change a
rule here, change it there too (and in iter3-pet/vision/GESTURES.md).

    cfg = load_config()                       # iter3-pet/vision/gesture.conf
    ctl = Controller(cfg)
    label = classify(landmarks, aspect, cfg)  # 21 (x, y[, z]) normalized points
    feat = hand_features(landmarks, aspect)   # (palm centre x, size) for follow
    for cmd in ctl.on_frame(t_ms, label, feat):  # call per camera frame
        send(cmd.text)                        # "D 0.50 0.50" / "S"
    for cmd in ctl.tick(t_ms):                # call every ~20 ms from a timer
        send(cmd.text)
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONF = os.path.join(HERE, "..", "gesture.conf")

# ---- labels ------------------------------------------------------------------
NO_HAND = "no_hand"
UNKNOWN = "unknown"
PALM = "palm"
FIST = "fist"
THUMBS_UP = "thumbs_up"
THUMBS_DOWN = "thumbs_down"
POINT_UP = "point_up"
POINT_DOWN = "point_down"
POINT_LEFT = "point_left"
POINT_RIGHT = "point_right"
PEACE = "peace"
TEACH = "teach"          # shaka: index/middle/ring folded, pinky out (thumb ignored)
LABELS = [NO_HAND, UNKNOWN, PALM, FIST, THUMBS_UP, THUMBS_DOWN,
          POINT_UP, POINT_DOWN, POINT_LEFT, POINT_RIGHT, PEACE, TEACH]

GESTURE_MODE, FOLLOW_MODE, PET_MODE = "gesture", "follow", "pet"

# ---- intents -----------------------------------------------------------------
STOP, FORWARD, BACK, TURN_LEFT, TURN_RIGHT = "stop", "forward", "back", "turn_left", "turn_right"
INTENT = {
    PALM: STOP,
    THUMBS_UP: FORWARD, POINT_UP: FORWARD,
    FIST: BACK, THUMBS_DOWN: BACK, POINT_DOWN: BACK,
    POINT_LEFT: TURN_LEFT, POINT_RIGHT: TURN_RIGHT,
}

CONF_KEYS = [
    "ext_ratio", "fold_ratio", "thumb_out", "dir_tolerance_deg",
    "stable_frames", "stop_frames", "lost_ms", "frame_timeout_ms", "cooldown_ms",
    "hold_repeat", "resend_ms", "step_speed", "step_ms", "turn_speed", "turn_ms",
    "point_left_is_left",
    "mode_frames", "mode_cooldown_ms",
    "follow_deadband", "follow_kp_turn", "follow_max_turn", "follow_target_size",
    "follow_size_deadband", "follow_kp_fwd", "follow_max_fwd", "follow_max_back",
    "follow_slew_per_s", "follow_send_ms", "follow_grace_frames", "follow_palm_frames",
    "follow_invert_turn",
    "flip", "infer_width",
    "min_detection_confidence", "min_tracking_confidence",
    # actions (learned poses can map to these)
    "spin_turns", "wiggle_ms", "arms_ms",
    # teach-on-the-spot (iter3-pet/vision/common/learn.py)
    "teach_hold_ms", "teach_countdown_ms", "learn_gap_frames", "learn_cancel_frames",
    "learn_frames", "learn_record_timeout_ms", "learn_assign_ms", "learn_assign_frames",
    "learn_cooldown_ms", "learn_builtin_frac", "learn_similar_frac", "learn_max_classes",
    "learn_k", "learn_augment", "learn_jitter", "learn_angle_weight",
    "learn_thr_mult", "learn_thr_min", "learn_thr_max",
    "feedback_speed", "feedback_ms", "nudge_ms", "feedback_led_beep",
    # pet mode (iter3-pet/vision/common/pet.py, GESTURES.md section 8)
    "pet_in_cycle", "pet_start", "pet_exit_ms", "pet_trick_frames", "pet_gap_frames",
    "pet_sleep_frames", "pet_wave_window_ms", "pet_wave_amp", "pet_wave_swings",
    "pet_send_ms", "pet_settle_ms", "pet_come_size", "pet_come_max_fwd", "pet_come_max_ms",
    "pet_body_ms", "pet_happy_ms", "pet_sad_ms", "pet_look_ms", "pet_max_seg_ms", "pet_max_speed",
    "pet_bored_ms", "pet_look_every_ms", "pet_sleep_after_ms",
    "pet_energy_start", "pet_energy_rest", "pet_energy_decay_per_s", "pet_happy_boost",
    "pet_greet_boost", "pet_come_boost", "pet_sad_drop", "pet_excited",
    "pet_speed_min", "pet_speed_max", "pet_seed",
]


def load_config(path: str = DEFAULT_CONF, overrides: dict | None = None) -> dict:
    """Parse gesture.conf (key = value). Every key in CONF_KEYS is required."""
    cfg: dict[str, float] = {}
    with open(path, encoding="ascii") as f:
        for n, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            if "=" not in line:
                raise ValueError(f"{path}:{n}: expected 'key = value'")
            k, v = (s.strip() for s in line.split("=", 1))
            if k not in CONF_KEYS:
                raise ValueError(f"{path}:{n}: unknown key '{k}'")
            cfg[k] = float(v)
    for k, v in (overrides or {}).items():
        if k not in CONF_KEYS:
            raise ValueError(f"unknown override '{k}'")
        cfg[k] = float(v)
    missing = [k for k in CONF_KEYS if k not in cfg]
    if missing:
        raise ValueError(f"{path}: missing keys {missing}")
    return cfg


# ---- classifier --------------------------------------------------------------
WRIST, THUMB_MCP, THUMB_TIP, INDEX_MCP, MIDDLE_MCP = 0, 2, 4, 5, 9
FINGERS = [(6, 8), (10, 12), (14, 16), (18, 20)]   # (pip, tip): index, middle, ring, pinky
PALM_POINTS = [0, 5, 9, 13, 17]                     # wrist + 4 knuckles: palm centre


def _dist(lm, a: int, b: int, aspect: float) -> float:
    return math.hypot((lm[a][0] - lm[b][0]) * aspect, lm[a][1] - lm[b][1])


def _direction(lm, a: int, b: int, aspect: float, tol: float) -> str | None:
    """Direction of the vector a->b in the image: 'up'/'down'/'left'/'right'/None."""
    dx = (lm[b][0] - lm[a][0]) * aspect
    dy = lm[b][1] - lm[a][1]
    ang = math.degrees(math.atan2(-dy, dx))        # 0 = right, 90 = up (y grows down)
    for name, centre in (("right", 0.0), ("up", 90.0), ("left", 180.0), ("down", -90.0)):
        diff = abs((ang - centre + 180.0) % 360.0 - 180.0)
        if diff <= tol:
            return name
    return None


def classify(lm, aspect: float, cfg: dict) -> str:
    """lm: 21 landmarks (x, y, ...) normalized to [0,1] of the image MediaPipe saw.
    aspect: image width / height (so distances are measured in square units)."""
    if lm is None or len(lm) < 21:
        return NO_HAND
    palm = _dist(lm, WRIST, MIDDLE_MCP, aspect)
    if palm < 1e-6:
        return UNKNOWN
    ext, fold = [], []
    for pip, tip in FINGERS:
        dp = _dist(lm, WRIST, pip, aspect)
        r = _dist(lm, WRIST, tip, aspect) / dp if dp > 1e-9 else 0.0
        ext.append(r > cfg["ext_ratio"])
        fold.append(r < cfg["fold_ratio"])
    tol = cfg["dir_tolerance_deg"]

    if all(ext):
        return PALM
    if ext[0] and ext[1] and fold[2] and fold[3]:
        return PEACE
    if ext[0] and fold[1] and fold[2] and fold[3]:
        d = _direction(lm, INDEX_MCP, 8, aspect, tol)
        return {"up": POINT_UP, "down": POINT_DOWN, "left": POINT_LEFT,
                "right": POINT_RIGHT}.get(d, UNKNOWN)
    if fold[0] and fold[1] and fold[2] and ext[3]:
        return TEACH
    if all(fold):
        thumb_out = _dist(lm, THUMB_TIP, INDEX_MCP, aspect) / palm > cfg["thumb_out"]
        if thumb_out:
            d = _direction(lm, THUMB_MCP, THUMB_TIP, aspect, tol)
            if d == "up":
                return THUMBS_UP
            if d == "down":
                return THUMBS_DOWN
        return FIST
    return UNKNOWN


def hand_features(lm, aspect: float) -> tuple[float, float] | None:
    """(cx, size) for follow mode. cx: palm centre x in [0,1] of the image as
    MediaPipe saw it (i.e. after any flip). size: wrist->middle-knuckle length
    in units of image height (bigger = closer)."""
    if lm is None or len(lm) < 21:
        return None
    cx = sum(lm[i][0] for i in PALM_POINTS) / len(PALM_POINTS)
    return cx, _dist(lm, WRIST, MIDDLE_MCP, aspect)


def next_mode(mode: str, cfg: dict) -> str:
    """Held peace sign cycles GESTURE -> FOLLOW -> PET (if pet_in_cycle) -> GESTURE."""
    if mode == GESTURE_MODE:
        return FOLLOW_MODE
    if mode == FOLLOW_MODE and cfg["pet_in_cycle"] >= 0.5:
        return PET_MODE
    return GESTURE_MODE


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def follow_targets(cx: float, size: float, cfg: dict) -> tuple[float, float]:
    """Wheel targets (l, r) that steer toward the hand and close to target size."""
    c = cfg
    x = 1.0 - cx if c["flip"] >= 0.5 else cx      # x in the rover's own view
    err = x - 0.5                                   # + = hand is to the rover's right
    if c["follow_invert_turn"] >= 0.5:
        err = -err
    db = c["follow_deadband"]
    turn = 0.0
    if abs(err) > db:
        turn = c["follow_kp_turn"] * (abs(err) - db) * (1.0 if err > 0 else -1.0)
        turn = _clamp(turn, -c["follow_max_turn"], c["follow_max_turn"])
    e = c["follow_target_size"] - size              # + = hand too small = too far
    sdb = c["follow_size_deadband"]
    fwd = 0.0
    if e > sdb:
        fwd = _clamp(c["follow_kp_fwd"] * (e - sdb), 0.0, c["follow_max_fwd"])
    elif e < -sdb:
        fwd = _clamp(c["follow_kp_fwd"] * (e + sdb), -c["follow_max_back"], 0.0)
    return _clamp(fwd + turn, -1.0, 1.0), _clamp(fwd - turn, -1.0, 1.0)


# ---- controller (modes, debounce, moves, follow, fail-safes) --------------------
@dataclass
class Cmd:
    text: str      # line for tars_motor, e.g. "D 0.50 -0.50" or "S"
    reason: str    # palm, lost, camera_timeout, done, forward, back, turn_left,
                   # turn_right, resend, follow, mode_gesture, mode_follow

    def __str__(self) -> str:
        return f"{self.text} ({self.reason})"


def _d(l: float, r: float) -> str:
    return "D %.2f %.2f" % (l, r)


def _toward(cur: float, tgt: float, step: float) -> float:
    d = tgt - cur
    return tgt if abs(d) <= step else cur + (step if d > 0 else -step)


# A move is a list of segments: ("D", l, r, ms) drives (D resent every resend_ms),
# ("X", text, 0, ms) sends one line (e.g. "G wave", "BEEP 1") then waits ms.
ACTIONS = ["forward", "back", "turn_left", "turn_right", "spin", "wiggle", "arms"]


class Controller:
    """Gesture/follow state machine. Time is passed in (ms), so it's testable.

    on_frame() once per camera frame (label + hand features), tick() every ~20 ms.
    Both return the Cmds to send, in order. Rules: iter3-pet/vision/GESTURES.md.
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.mode = GESTURE_MODE
        self.cand = NO_HAND
        self.cand_count = 0
        self.miss = 0                     # consecutive no-hand frames
        self.last_hand_t: float | None = None
        self.last_frame_t: float | None = None
        self.need_stop = False            # a D went out since the last S
        # gesture mode
        self.move: tuple[str, float, float, float] | None = None   # (intent, l, r, end_ms)
        self.last_send = -1e18
        self.cooldown_until = -1e18
        self.fired_intent: str | None = None
        self.armed = True
        # mode toggle
        self.toggle_armed = True
        self.mode_cooldown_until = -1e18
        # follow mode
        self.f_active = False
        self.f_tgt = (0.0, 0.0)
        self.f_cur = (0.0, 0.0)
        # multi-segment moves + learned poses
        self.segs: list = []               # segments still to run after the current one
        self.move_d = True                 # current segment is a D (resend it)
        self.learned: dict[str, tuple[str, list]] = {}   # label -> (action, custom segs)

    @property
    def moving(self) -> str | None:
        return self.move[0] if self.move else None

    # -- helpers ---------------------------------------------------------------
    def intent_of(self, label: str) -> str | None:
        it = INTENT.get(label)
        if it is None and label in self.learned:
            it = self.learned[label][0] + "@" + label
        return it

    def action_segments(self, action: str, custom: list | None = None) -> list:
        c = self.cfg
        s, ts = c["step_speed"], c["turn_speed"]
        if action == FORWARD:
            return [("D", s, s, c["step_ms"])]
        if action == BACK:
            return [("D", -s, -s, c["step_ms"])]
        if action in (TURN_LEFT, TURN_RIGHT):
            ccw = (action == TURN_LEFT) == (c["point_left_is_left"] >= 0.5)
            return [("D", -ts, ts, c["turn_ms"])] if ccw else [("D", ts, -ts, c["turn_ms"])]
        if action == "spin":
            return [("D", -ts, ts, c["turn_ms"] * c["spin_turns"])]
        if action == "wiggle":
            w = c["wiggle_ms"]
            return [("D", -ts, ts, w), ("D", ts, -ts, 2 * w), ("D", -ts, ts, 2 * w),
                    ("D", ts, -ts, 2 * w), ("D", -ts, ts, w)]
        if action == "arms":
            return [("X", "G wave", 0.0, c["arms_ms"])]
        if action == "custom":
            return list(custom or [])
        return []

    def _segments(self, intent: str) -> list:
        if "@" in intent:
            act, label = intent.split("@", 1)
            return self.action_segments(act, self.learned.get(label, ("", []))[1])
        return self.action_segments(intent)

    def _start_seg(self, t: float, intent: str) -> list[Cmd]:
        """Start the next queued segment(s). Zero-length X segments chain at once."""
        out: list[Cmd] = []
        while self.segs:
            kind, a, b, ms = self.segs.pop(0)
            self.move = (intent, a if kind == "D" else 0.0, b if kind == "D" else 0.0, t + ms)
            self.move_d = kind == "D"
            self.last_send = t
            self.need_stop = True
            out.append(Cmd(_d(a, b) if kind == "D" else a, intent))
            if ms > 0:
                return out
        out += self._stop(t, "done")
        self.cooldown_until = t + self.cfg["cooldown_ms"]
        return out

    def play(self, t: float, intent: str, segs: list) -> list[Cmd]:
        """Run a segment list now (cancels any move). Used for actions + teach feedback."""
        out = []
        if self.move is not None or self.need_stop or self.f_active:
            out += self._stop(t, "preempt")
        self.segs = list(segs)
        return out + self._start_seg(t, intent)

    def _stop(self, t: float, reason: str) -> list[Cmd]:
        if self.move is not None:
            self.cooldown_until = t + self.cfg["cooldown_ms"]
        self.move = None
        self.segs = []
        self.need_stop = False
        self.f_active = False
        self.f_tgt = self.f_cur = (0.0, 0.0)
        return [Cmd("S", reason)]

    # -- inputs ----------------------------------------------------------------
    def on_frame(self, t: float, label: str, feat: tuple[float, float] | None = None) -> list[Cmd]:
        c = self.cfg
        out: list[Cmd] = []
        self.last_frame_t = t
        if label == NO_HAND:
            self.cand, self.cand_count = NO_HAND, 0
            self.miss += 1
        else:
            self.miss = 0
            self.last_hand_t = t
            if label == self.cand:
                self.cand_count += 1
            else:
                self.cand, self.cand_count = label, 1

        # mode toggle: peace held mode_frames, released since last toggle, cooled down.
        # In PET mode the pet layer owns the peace sign (flash = trick, hold = leave).
        if label == PEACE:
            if (self.mode != PET_MODE and self.cand_count >= c["mode_frames"] and self.toggle_armed
                    and t >= self.mode_cooldown_until):
                new = next_mode(self.mode, c)
                out += self._stop(t, "mode_" + new)
                self.mode = new
                self.toggle_armed = False
                self.mode_cooldown_until = t + c["mode_cooldown_ms"]
                self.cooldown_until = t + c["cooldown_ms"]
                self.fired_intent, self.armed = None, True
                return out + self.tick(t)
        else:
            self.toggle_armed = True

        if self.mode == FOLLOW_MODE:
            out += self._follow_frame(t, label, feat)
        elif self.mode == PET_MODE:
            out += self._pet_frame(t, label)
        else:
            out += self._gesture_frame(t, label)
        return out + self.tick(t)

    def _gesture_frame(self, t: float, label: str) -> list[Cmd]:
        c = self.cfg
        if label == NO_HAND:
            self.armed = True
            lost = self.last_hand_t is None or t - self.last_hand_t >= c["lost_ms"]
            if lost and (self.need_stop or self.move is not None):
                return self._stop(t, "lost")
            return []
        intent = self.intent_of(label)
        if intent != self.fired_intent:
            self.armed = True
        if intent == STOP:
            first = self.cand_count == int(c["stop_frames"])
            if first or (self.cand_count > c["stop_frames"] and (self.need_stop or self.move)):
                out = self._stop(t, "palm")
                self.cooldown_until = t + c["cooldown_ms"]
                return out
        elif intent is not None:
            ready = (self.cand_count >= c["stable_frames"] and self.move is None
                     and t >= self.cooldown_until
                     and (self.armed or c["hold_repeat"] >= 0.5))
            if ready:
                self.fired_intent = intent
                self.armed = False
                self.segs = self._segments(intent)
                return self._start_seg(t, intent)
        return []

    def _follow_frame(self, t: float, label: str, feat) -> list[Cmd]:
        c = self.cfg
        if label == NO_HAND:
            if self.miss >= c["follow_grace_frames"] and (self.need_stop or self.f_active):
                return self._stop(t, "lost")
            return []
        if label == PALM:
            n = self.cand_count
            if n >= c["follow_palm_frames"] and (n == int(c["follow_palm_frames"])
                                                 or self.need_stop or self.f_active):
                return self._stop(t, "palm")
            return []
        if label in (PEACE, TEACH):          # hold still while (maybe) switching modes / teaching
            self.f_tgt = (0.0, 0.0)
            return []
        if feat is not None:
            self.f_tgt = follow_targets(feat[0], feat[1], c)
            if not self.f_active:
                self.f_active = True
                self.last_send = t - c["follow_send_ms"]
        return []

    def _pet_frame(self, t: float, label: str) -> list[Cmd]:
        """Safety floor under PET mode; iter3-pet/vision/common/pet.py decides the behaviour.
        Palm stops (same rule as GESTURE mode), a lost hand stops a D stream after
        follow_grace_frames. Short self-ending body moves run to their end."""
        c = self.cfg
        if label == NO_HAND:
            if self.miss >= c["follow_grace_frames"] and self.f_active:
                return self._stop(t, "lost")
            return []
        if label == PALM:
            n = self.cand_count
            if n == int(c["stop_frames"]) or (n > c["stop_frames"] and (
                    self.need_stop or self.move is not None or self.f_active)):
                return self._stop(t, "palm")
        return []

    def tick(self, t: float) -> list[Cmd]:
        c = self.cfg
        if (self.last_frame_t is not None and t - self.last_frame_t >= c["frame_timeout_ms"]
                and (self.need_stop or self.move is not None or self.f_active)):
            return self._stop(t, "camera_timeout")
        if self.mode == FOLLOW_MODE or (self.mode == PET_MODE and self.f_active):
            pet = self.mode == PET_MODE
            send_ms = c["pet_send_ms"] if pet else c["follow_send_ms"]
            if not self.f_active or t - self.last_send < send_ms:
                return []
            dt = min(t - self.last_send, 2 * send_ms) / 1000.0
            self.last_send = t
            step = c["follow_slew_per_s"] * dt
            self.f_cur = (_toward(self.f_cur[0], self.f_tgt[0], step),
                          _toward(self.f_cur[1], self.f_tgt[1], step))
            self.need_stop = True
            return [Cmd(_d(*self.f_cur), "pet_stream" if pet else "follow")]
        if self.move is None:
            return []
        intent, l, r, end = self.move
        if t >= end:
            return self._start_seg(t, intent)
        if self.move_d and t - self.last_send >= c["resend_ms"]:
            self.last_send = t
            return [Cmd(_d(l, r), "resend")]
        return []
