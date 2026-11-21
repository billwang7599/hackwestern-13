"""Teach TARS a new hand pose on the spot: features, few-shot k-NN, teach flow.

Pure Python, no dependencies. vision/common/learn.hpp is a line-for-line C++
port; tests/test_parity.py runs both on the same sessions (including teaching,
forgetting and save/load) and diffs every line. Rules: vision/GESTURES.md §7.

    brain = Brain(cfg)                       # wraps gesture.Controller
    brain.load(path)                         # learned poses (JSON), optional
    for cmd in brain.on_frame(t_ms, lm, aspect): send(cmd.text)
    for cmd in brain.tick(t_ms):             send(cmd.text)    # every ~20 ms
    brain.button(t_ms, "teach" | "forget")   # Uno button / desktop 't' key
    if brain.dirty: brain.save(path)
    brain.events                             # ["learned pose1 spin", ...] (drain it)

Everything is driven by the robot's own body: no screen, no keyboard needed.
"""
from __future__ import annotations

import json
import math

import gesture as g

# ---- features ------------------------------------------------------------------
CHAINS = [[0, 1, 2, 3, 4], [0, 5, 6, 7, 8], [0, 9, 10, 11, 12], [0, 13, 14, 15, 16],
          [0, 17, 18, 19, 20]]
N_COORD = 42
N_FEAT = 57


def features(lm, aspect: float, angle_weight: float) -> list[float] | None:
    """57 numbers that don't change with hand position, size, in-plane rotation
    or handedness: 21 (x, y) points with the wrist at 0, palm length 1,
    wrist->middle knuckle pointing up, mirrored so the index is on the left;
    then 15 finger-bend angles (radians) x angle_weight. z is ignored (noisy)."""
    if lm is None or len(lm) < 21:
        return None
    wx, wy = lm[0][0] * aspect, lm[0][1]
    mx, my = lm[9][0] * aspect - wx, lm[9][1] - wy
    s = math.sqrt(mx * mx + my * my)
    if s < 1e-6:
        return None
    c, sn = -my / s, -mx / s          # rotation taking (mx, my) to (0, -s)
    xs, ys = [], []
    for i in range(21):
        x, y = lm[i][0] * aspect - wx, lm[i][1] - wy
        xs.append((c * x - sn * y) / s)
        ys.append((sn * x + c * y) / s)
    if xs[5] > xs[17]:                # left hand (or back of the hand): mirror
        xs = [-x for x in xs]
    out = []
    for i in range(21):
        out.append(xs[i])
        out.append(ys[i])
    for ch in CHAINS:
        for j in range(1, 4):
            ax, ay = xs[ch[j]] - xs[ch[j - 1]], ys[ch[j]] - ys[ch[j - 1]]
            bx, by = xs[ch[j + 1]] - xs[ch[j]], ys[ch[j + 1]] - ys[ch[j]]
            out.append(math.atan2(abs(ax * by - ay * bx), ax * bx + ay * by) * angle_weight)
    return out


def dist(a: list[float], b: list[float]) -> float:
    s = 0.0
    for i in range(N_FEAT):
        d = a[i] - b[i]
        s += d * d
    return math.sqrt(s)


class Lcg:
    """Tiny deterministic RNG (same numbers in Python and C++)."""

    def __init__(self, seed: int):
        self.s = seed % 2147483648

    def uniform(self) -> float:            # [0, 1)
        self.s = (self.s * 1103515245 + 12345) % 2147483648
        return self.s / 2147483648.0


def _sum(xs) -> float:
    """Plain left-to-right sum (Python 3.12+ sum() is compensated; C++ isn't)."""
    s = 0.0
    for x in xs:
        s += x
    return s


def _kmean(ds: list[float], k: int) -> float:
    ds = sorted(ds)[:max(1, k)]
    return _sum(ds) / len(ds)


# ---- few-shot classifier -------------------------------------------------------
class PoseClass:
    def __init__(self, name: str, action: str, raw: list, seed: int, cfg: dict,
                 custom: list | None = None, thr: float | None = None):
        self.name, self.action, self.raw, self.seed = name, action, raw, seed
        self.custom = custom or []
        j = cfg["learn_jitter"]
        rng = Lcg(seed)
        self.examples = list(raw)
        for f in raw:
            for _ in range(int(cfg["learn_augment"])):
                self.examples.append([v + (2.0 * rng.uniform() - 1.0) * j for v in f])
        self.thr = thr if thr is not None else self._threshold(cfg)

    def _threshold(self, cfg: dict) -> float:
        """Accept radius from how much the taught frames vary (leave-one-out k-NN)."""
        k = int(cfg["learn_k"])
        n = len(self.raw)
        loo = []
        for i in range(n):
            ds = [dist(self.raw[i], self.raw[j]) for j in range(n) if j != i]
            if ds:
                loo.append(_kmean(ds, k))
        m = _sum(loo) / len(loo) if loo else 0.0
        var = _sum([(d - m) * (d - m) for d in loo]) / len(loo) if loo else 0.0
        t = cfg["learn_thr_mult"] * (m + 2.0 * math.sqrt(var))
        return min(max(t, cfg["learn_thr_min"]), cfg["learn_thr_max"])

    def score(self, f: list[float], k: int) -> float:
        return _kmean([dist(f, e) for e in self.examples], k)


class Learner:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.classes: list[PoseClass] = []
        self.next_seed = 1

    def match(self, f: list[float]):
        """-> (accepted name or None, best name or None, best ratio d/thr, confidence 0..1).
        confidence = 1 / (1 + ratio^2): 0.5 exactly at the accept radius."""
        best, br = None, 1e18
        k = int(self.cfg["learn_k"])
        for c in self.classes:
            r = c.score(f, k) / c.thr
            if r < br:
                best, br = c, r
        if best is None:
            return None, None, 1e18, 0.0
        conf = 1.0 / (1.0 + br * br)
        return (best.name if br <= 1.0 else None), best.name, br, conf

    def similar_to(self, feats: list) -> str | None:
        """Name of a learned pose that already accepts >= learn_similar_frac of feats."""
        k = int(self.cfg["learn_k"])
        for c in self.classes:
            hit = sum(1 for f in feats if c.score(f, k) <= c.thr)
            if hit >= self.cfg["learn_similar_frac"] * len(feats):
                return c.name
        return None

    def free_name(self) -> str:
        names = {c.name for c in self.classes}
        i = 1
        while f"pose{i}" in names:
            i += 1
        return f"pose{i}"

    def add(self, name: str, action: str, raw: list, custom: list | None = None) -> PoseClass:
        c = PoseClass(name, action, raw, self.next_seed, self.cfg, custom)
        self.next_seed += 1
        self.classes.append(c)
        return c

    def forget_all(self) -> int:
        n = len(self.classes)
        self.classes = []
        return n

    # -- persistence ---------------------------------------------------------------
    def to_json(self) -> dict:
        return {"version": 1, "next_seed": self.next_seed, "classes": [
            {"name": c.name, "action": c.action, "seed": c.seed, "thr": c.thr,
             "custom": [list(s) for s in c.custom], "examples": c.raw} for c in self.classes]}

    def from_json(self, d: dict) -> None:
        self.classes = []
        for c in d.get("classes", []):
            custom = [tuple(s) for s in c.get("custom", [])]
            self.classes.append(PoseClass(c["name"], c["action"], c["examples"], int(c["seed"]),
                                          self.cfg, custom, float(c["thr"])))
        self.next_seed = int(d.get("next_seed", len(self.classes) + 1))


# ---- teach flow + combined pipeline ---------------------------------------------
IDLE, COUNTDOWN, RECORD, ASSIGN, CONFIRM = "idle", "countdown", "record", "assign", "confirm"

# action picked by showing an existing gesture after recording
ASSIGN_MAP = {
    g.POINT_RIGHT: g.TURN_RIGHT, g.POINT_LEFT: g.TURN_LEFT,
    g.THUMBS_UP: g.FORWARD, g.POINT_UP: g.FORWARD,
    g.FIST: g.BACK, g.THUMBS_DOWN: g.BACK, g.POINT_DOWN: g.BACK,
    g.PEACE: "spin",
}


class Brain:
    """classify -> (learned k-NN) -> Controller, plus the teach state machine.

    Safety: an open palm is decided by the built-in rules BEFORE any learned
    pose is consulted, and is passed to the Controller in every teach state, so
    ✋ always sends S. Learned poses are only consulted when the rules say
    `unknown`, so they can never shadow a built-in gesture.
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.ctl = g.Controller(cfg)
        self.learner = Learner(cfg)
        self.state = IDLE
        self.state_t = 0.0
        self.events: list[str] = []
        self.dirty = False
        self.last_rule = g.NO_HAND
        self.label = g.NO_HAND
        self.match: tuple | None = None      # (best name, confidence, accepted)
        self.teach_since: float | None = None
        self.gap = 0
        self.released = True
        self.nudges = 0
        self.palm_count = 0
        self.rec: list = []
        self.rec_labels: list[str] = []
        self.cand = ""
        self.cand_count = 0
        self.hold_off: str | None = None
        self.cooldown_until = -1e18

    # -- learned poses -> controller ----------------------------------------------
    def _sync(self) -> None:
        self.ctl.learned = {c.name: (c.action, c.custom) for c in self.learner.classes}

    def load_json(self, d: dict) -> None:
        self.learner.from_json(d)
        self._sync()

    def load(self, path: str) -> int:
        try:
            with open(path, encoding="utf-8") as f:
                self.load_json(json.load(f))
        except FileNotFoundError:
            return 0
        return len(self.learner.classes)

    def save(self, path: str) -> None:
        import os
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.learner.to_json(), f)
        os.replace(tmp, path)
        self.dirty = False

    # -- feedback (the robot's body is the display) ---------------------------------
    def _fb(self, kind: str) -> list:
        c = self.cfg
        f, w, n = c["feedback_speed"], c["feedback_ms"], c["nudge_ms"]
        L, R = ("D", -f, f, w), ("D", f, -f, w)
        fx = []
        if c["feedback_led_beep"] >= 0.5:
            led = {"armed": "teach", "nudge": "", "record": "rec", "recorded": "ok", "no": "no",
                   "forget": "forget", "learned": "ok", "cancel": "off"}[kind]
            beep = {"armed": 1, "nudge": 1, "record": 2, "recorded": 2, "no": 3, "forget": 4,
                    "learned": 0, "cancel": 0}[kind]
            if led:
                fx.append(("X", "LED " + led, 0.0, 0.0))
            if beep:
                fx.append(("X", "BEEP %d" % beep, 0.0, 0.0))
        if kind == "armed":
            return fx + [L, R]
        if kind == "nudge":
            return fx + [("D", f, f, n), ("D", -f, -f, n)]
        if kind == "recorded":
            return fx + [L, R, L, R]
        if kind == "no":
            return fx + [L, ("D", f, -f, 2 * w), L]
        if kind == "forget":
            return fx + [("D", -f, -f, n), ("D", f, f, n), ("D", -f, -f, n), ("D", f, f, n)]
        return fx

    def _event(self, e: str) -> None:
        self.events.append(e)

    # -- state changes ---------------------------------------------------------------
    def _start_teach(self, t: float, via: str) -> list:
        out = self.ctl._stop(t, "teach")
        self.ctl.mode = g.GESTURE_MODE
        self.state, self.state_t = COUNTDOWN, t
        self.released = via == "button"
        self.gap = self.nudges = self.palm_count = 0
        self.rec, self.rec_labels = [], []
        self._event("teach start " + via)
        return out + self.ctl.play(t, "teach_armed", self._fb("armed"))

    def _end(self, t: float) -> None:
        self.state = IDLE
        self.teach_since = None
        self.gap = 0
        self.hold_off = None if self.last_rule in (g.NO_HAND, g.UNKNOWN, g.PALM) else self.last_rule
        self.cooldown_until = t + self.cfg["learn_cooldown_ms"]

    def _reject(self, t: float, why: str) -> list:
        self._event("reject " + why)
        self.state, self.state_t = CONFIRM, t
        return self.ctl.play(t, "teach_no", self._fb("no"))

    def _forget(self, t: float, via: str) -> list:
        n = self.learner.forget_all()
        self._sync()
        self.dirty = True
        self._event("forget %d %s" % (n, via))
        self.state, self.state_t = CONFIRM, t
        return self.ctl.play(t, "teach_forget", self._fb("forget"))

    def _finish_record(self, t: float) -> list:
        c = self.cfg
        counts: dict[str, int] = {}
        for lab in self.rec_labels:
            if lab not in (g.UNKNOWN, g.NO_HAND):
                counts[lab] = counts.get(lab, 0) + 1
        top, topn = "", 0
        for lab in g.LABELS:                       # fixed order: deterministic ties
            if counts.get(lab, 0) > topn:
                top, topn = lab, counts[lab]
        if topn and topn >= c["learn_builtin_frac"] * len(self.rec_labels):
            return self._reject(t, "too_similar " + top)
        if len(self.learner.classes) >= c["learn_max_classes"]:
            return self._reject(t, "full")
        sim = self.learner.similar_to(self.rec)
        if sim is not None:
            return self._reject(t, "too_similar " + sim)
        self.state, self.state_t = ASSIGN, t
        self.cand, self.cand_count = "", 0
        self._event("recorded %d" % len(self.rec))
        return self.ctl.play(t, "teach_recorded", self._fb("recorded"))

    def _assign(self, t: float, action: str, via: str) -> list:
        name = self.learner.free_name()
        self.learner.add(name, action, self.rec)
        self._sync()
        self.dirty = True
        self._event("learned %s %s %s" % (name, action, via))
        self.state, self.state_t = CONFIRM, t
        intent = action + "@" + name
        fx = [s for s in self._fb("learned")]
        return self.ctl.play(t, intent, fx + self.ctl.action_segments(action))

    # -- inputs ----------------------------------------------------------------------
    def classify(self, lm, aspect: float) -> tuple[str, str]:
        """-> (rule label, combined label). Learned poses only when rules say unknown."""
        rule = g.classify(lm, aspect, self.cfg)
        self.match = None
        if rule != g.UNKNOWN or not self.learner.classes:
            return rule, rule
        f = features(lm, aspect, self.cfg["learn_angle_weight"])
        if f is None:
            return rule, rule
        name, best, _, conf = self.learner.match(f)
        self.match = (best, conf, name is not None)
        return rule, (name if name is not None else g.UNKNOWN)

    def button(self, t: float, kind: str) -> list:
        if kind == "forget":
            out = self.ctl._stop(t, "teach") if self.state != IDLE else []
            return out + self._forget(t, "button")
        if kind == "teach":
            if self.state == IDLE:
                return self._start_teach(t, "button")
            if self.state != CONFIRM:              # second press: cancel
                self._event("cancel button")
                out = self.ctl._stop(t, "teach_cancel") + self.ctl.play(t, "teach_cancel", self._fb("cancel"))
                self._end(t)
                return out
        return []

    def on_frame(self, t: float, lm, aspect: float) -> list:
        c = self.cfg
        rule, label = self.classify(lm, aspect)
        self.last_rule = rule
        feat = g.hand_features(lm, aspect)
        if self.state == IDLE:
            self.label = label
            if rule == g.TEACH:
                if self.teach_since is None:
                    self.teach_since = t
                self.gap = 0
            elif self.teach_since is not None:
                self.gap += 1
                if self.gap > c["learn_gap_frames"]:
                    self.teach_since = None
            if (self.teach_since is not None and t - self.teach_since >= c["teach_hold_ms"]
                    and t >= self.cooldown_until):
                return self._start_teach(t, "gesture")
            if self.hold_off is not None:          # don't act on the pose left over from teaching
                if rule == self.hold_off:
                    label = g.UNKNOWN
                else:
                    self.hold_off = None
            return self.ctl.on_frame(t, label, feat)

        # ---- teaching: the controller only ever sees palm (stop) or unknown ----
        self.label = rule
        self.palm_count = self.palm_count + 1 if rule == g.PALM else 0
        out = self.ctl.on_frame(t, g.PALM if rule == g.PALM else g.UNKNOWN, None)
        if self.state != CONFIRM and self.palm_count >= c["learn_cancel_frames"]:
            self._event("cancel palm")
            out += self.ctl.play(t, "teach_cancel", self._fb("cancel"))
            self._end(t)
            return out
        if self.state == COUNTDOWN:
            if rule == g.TEACH:
                self.gap = 0
            else:
                self.gap += 1
                if self.gap > c["learn_gap_frames"]:
                    self.released = True
        elif self.state == RECORD:
            if lm is not None and rule != g.PALM:
                f = features(lm, aspect, c["learn_angle_weight"])
                if f is not None:
                    self.rec.append(f)
                    self.rec_labels.append(rule)
                if len(self.rec) >= c["learn_frames"]:
                    return out + self._finish_record(t)
        elif self.state == ASSIGN:
            act = ASSIGN_MAP.get(rule)
            if act is None:
                self.cand, self.cand_count = "", 0
            elif act == self.cand:
                self.cand_count += 1
            else:
                self.cand, self.cand_count = act, 1
            if act is not None and self.cand_count >= c["learn_assign_frames"]:
                return out + self._assign(t, act, "gesture")
        return out + self._teach_tick(t)

    def _teach_tick(self, t: float) -> list:
        c = self.cfg
        el = t - self.state_t
        if self.state == COUNTDOWN:
            if el >= c["teach_countdown_ms"]:
                if not self.released:
                    return self._forget(t, "gesture")
                self.state, self.state_t = RECORD, t
                self._event("record")
                return self.ctl.play(t, "teach_record", self._fb("record"))
            if (self.nudges < int(math.floor(el / 1000.0))
                    and self.nudges < int(math.floor(c["teach_countdown_ms"] / 1000.0)) - 1):
                self.nudges += 1
                return self.ctl.play(t, "teach_nudge", self._fb("nudge"))
        elif self.state == RECORD:
            if el >= c["learn_record_timeout_ms"]:
                return self._reject(t, "no_hand")
        elif self.state == ASSIGN:
            if el >= c["learn_assign_ms"]:
                return self._assign(t, "wiggle", "timeout")
        elif self.state == CONFIRM:
            if self.ctl.move is None:
                self._event("idle")
                self._end(t)
        return []

    def tick(self, t: float) -> list:
        out = self.ctl.tick(t)
        if self.state != IDLE:
            out += self._teach_tick(t)
        return out
