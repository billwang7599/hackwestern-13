"""TARS pet mode: the rover reacts to your hand like a small pet.

Pure Python, no dependencies. iter3-pet/vision/common/pet.hpp is a line-for-line C++
port; tests/test_parity.py runs both on the same sessions and diffs every
line. Rules: iter3-pet/vision/GESTURES.md section 8. Every number is in gesture.conf.

The robot's only expressions are motion, the Uno LED and the buzzer, so the
"body language" is short, bounded TURN-like wiggles made of D segments, played
through gesture.Controller (resends, move ends, camera-stall stop). Continuous
behaviours (watch the hand, come here) use the Controller's D stream.

    pet = Pet(cfg, ctl)            # ctl = gesture.Controller (learn.Brain owns both)
    pet.sync(t)                    # enter/leave when ctl.mode becomes/leaves "pet"
    pet.on_frame(t, label, feat)   # instead of ctl.on_frame while pet.active
    pet.tick(t)                    # after ctl.tick, every ~20 ms

Safety (stop always wins):
- Every frame goes through ctl.on_frame FIRST. Its PET-mode floor sends S on an
  open palm (same rule as GESTURE mode) and on a hand lost during a D stream.
- Nothing starts moving while a palm is visible. A wave greets only after the
  palm drops.
- Camera stall -> ctl.tick sends S. Server-side S from anyone, the watchdog,
  tip/bump/guard: tars_motor refuses D; the app then calls Brain.server_stop.
- Every body segment is clamped to pet_max_speed and pet_max_seg_ms (the trick
  spin is turn_ms x spin_turns); "come" is capped at pet_come_max_ms.
"""
from __future__ import annotations

import gesture as g

IDLE, WATCH, COME, STAY = "idle", "watch", "come", "stay"
GREET, HAPPY, SAD, TRICK, LOOK, SLEEP = "greet", "happy", "sad", "trick", "look", "sleep"
STATES = [IDLE, WATCH, COME, STAY, GREET, HAPPY, SAD, TRICK, LOOK, SLEEP]
BODY = (GREET, HAPPY, SAD, TRICK, LOOK)        # short self-ending moves
STREAM = (WATCH, COME)                          # D stream


class Rng:
    """Tiny deterministic RNG (same numbers in Python and C++)."""

    def __init__(self, seed: int):
        self.s = seed % 2147483648

    def uniform(self) -> float:            # [0, 1)
        self.s = (self.s * 1103515245 + 12345) % 2147483648
        return self.s / 2147483648.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class Pet:
    def __init__(self, cfg: dict, ctl: g.Controller):
        self.cfg = cfg
        self.ctl = ctl
        self.active = False
        self.state = IDLE
        self.state_t = 0.0
        self.energy = cfg["pet_energy_start"]
        self.energy_t: float | None = None
        self.rng = Rng(int(cfg["pet_seed"]))
        self.events: list[str] = []
        self.last_hand_t = 0.0
        self.next_look = 0.0
        self.fired = ""                  # label whose reaction fired; re-armed by any other label
        self.react_until = -1e18         # no new reaction before this (after a body move)
        self.zero_since: float | None = None
        self._peace_reset(False)
        self._wave_reset()
        self.waved_t: float | None = None

    # -- helpers ------------------------------------------------------------------
    def _peace_reset(self, armed: bool) -> None:
        self.peace_armed = armed         # False right after entering with a held peace sign
        self.peace_since: float | None = None
        self.peace_n = 0
        self.peace_gap = 0

    def _wave_reset(self) -> None:
        self.w_ext: float | None = None  # extreme x of the current swing
        self.w_dir = 0
        self.w_times: list[float] = []   # reversal times

    def _event(self, e: str) -> None:
        self.events.append(e)

    def _set(self, t: float, state: str) -> None:
        self.state, self.state_t = state, t
        self.zero_since = None

    def speed(self) -> float:
        c = self.cfg
        s = c["pet_speed_min"] + (c["pet_speed_max"] - c["pet_speed_min"]) * self.energy
        return min(s, c["pet_max_speed"])

    def _boost(self, d: float) -> None:
        self.energy = _clamp(self.energy + d, 0.0, 1.0)

    def _decay(self, t: float) -> None:
        if self.energy_t is not None and t > self.energy_t:
            step = self.cfg["pet_energy_decay_per_s"] * (t - self.energy_t) / 1000.0
            self.energy = g._toward(self.energy, self.cfg["pet_energy_rest"], step)
        if self.energy_t is None or t > self.energy_t:
            self.energy_t = t

    def _fx(self, led: str, beep: int) -> list:
        if self.cfg["feedback_led_beep"] < 0.5:
            return []
        fx = []
        if led:
            fx.append(("X", "LED " + led, 0.0, 0.0))
        if beep:
            fx.append(("X", "BEEP %d" % beep, 0.0, 0.0))
        return fx

    def _fx_cmds(self, led: str, beep: int, reason: str) -> list:
        return [g.Cmd(s[1], reason) for s in self._fx(led, beep)]

    def _bound(self, segs: list, long_ok: bool) -> list:
        """Clamp every D segment: |speed| <= pet_max_speed, ms <= pet_max_seg_ms (unless long_ok)."""
        c = self.cfg
        mx, ms_max = c["pet_max_speed"], c["pet_max_seg_ms"]
        out = []
        for s in segs:
            if s[0] == "D":
                ms = s[3] if long_ok else min(s[3], ms_max)
                out.append(("D", _clamp(s[1], -mx, mx), _clamp(s[2], -mx, mx), ms))
            else:
                out.append(s)
        return out

    def _play(self, t: float, state: str, segs: list, long_ok: bool = False) -> list:
        self._set(t, state)
        self._event(state)
        return self.ctl.play(t, "pet_" + state, self._bound(segs, long_ok))

    def _stream(self, t: float, state: str) -> None:
        self._set(t, state)
        if not self.ctl.f_active:
            self.ctl.f_active = True
            self.ctl.last_send = t - self.cfg["pet_send_ms"]

    def _halt(self, t: float, reason: str) -> list:
        """Stop if anything is (or may be) moving."""
        ctl = self.ctl
        if ctl.need_stop or ctl.move is not None or ctl.f_active:
            return ctl._stop(t, reason)
        return []

    # -- mode enter / leave -----------------------------------------------------------
    def sync(self, t: float) -> list:
        if self.ctl.mode == g.PET_MODE and not self.active:
            return self._enter(t)
        if self.ctl.mode != g.PET_MODE and self.active:
            self.active = False
            self._set(t, IDLE)
            self._event("leave")
        return []

    def _enter(self, t: float) -> list:
        self.active = True
        self._set(t, IDLE)
        self.energy = self.cfg["pet_energy_start"]
        self.energy_t = t
        self.last_hand_t = t
        self.next_look = t + self.cfg["pet_bored_ms"]
        self.fired = ""
        self.react_until = -1e18
        self._peace_reset(False)
        self._wave_reset()
        self.waved_t = None
        self._event("enter")
        fx = self._fx("ok", 2)
        return self.ctl.play(t, "pet_hello", fx) if fx else []

    def _leave(self, t: float) -> list:
        """Held peace: back to GESTURE mode (same bookkeeping as the Controller's toggle)."""
        c, ctl = self.cfg, self.ctl
        out = ctl._stop(t, "mode_gesture")
        ctl.mode = g.GESTURE_MODE
        ctl.toggle_armed = False
        ctl.mode_cooldown_until = t + c["mode_cooldown_ms"]
        ctl.cooldown_until = t + c["cooldown_ms"]
        ctl.fired_intent, ctl.armed = None, True
        self.active = False
        self._set(t, IDLE)
        self._event("leave peace")
        return out + self._fx_cmds("off", 0, "mode_gesture")

    # -- behaviours ---------------------------------------------------------------------
    def _greet(self, t: float) -> list:
        self._boost(self.cfg["pet_greet_boost"])
        s, w = self.speed(), self.cfg["pet_body_ms"]
        return self._play(t, GREET, self._fx("ok", 2) + [
            ("D", -s, s, w), ("D", s, -s, 2 * w), ("D", -s, s, 2 * w), ("D", s, -s, w)])

    def _trick(self, t: float, state: str, beep: int) -> list:
        c = self.cfg
        ts = c["turn_speed"]
        return self._play(t, state, self._fx("ok", beep) + [
            ("D", -ts, ts, c["turn_ms"] * c["spin_turns"])], long_ok=True)

    def _happy(self, t: float) -> list:
        c = self.cfg
        self._boost(c["pet_happy_boost"])
        if self.energy >= c["pet_excited"]:
            return self._trick(t, HAPPY, 2)            # zoomies
        s, w = self.speed(), c["pet_happy_ms"]
        return self._play(t, HAPPY, self._fx("ok", 1) + [("D", s, -s, w), ("D", -s, s, w)])

    def _sad(self, t: float) -> list:
        self._boost(-self.cfg["pet_sad_drop"])
        s = self.speed()
        return self._play(t, SAD, self._fx("no", 0) + [("D", -s, -s, self.cfg["pet_sad_ms"])])

    def _look(self, t: float) -> list:
        s, w = self.speed(), self.cfg["pet_look_ms"]
        if self.rng.uniform() < 0.5:
            s = -s                                      # look left first
        return self._play(t, LOOK, [("D", s, -s, w), ("D", 0.0, 0.0, w), ("D", -s, s, w)])

    def _sleep(self, t: float, why: str) -> list:
        out = self._halt(t, "pet_sleep")
        self._set(t, SLEEP)
        self._event("sleep " + why)
        return out + self._fx_cmds("off", 0, "pet_sleep")

    def _turn(self, cx: float) -> float:
        """Turn component toward the hand (the follow-mode rule, no forward part)."""
        l, r = g.follow_targets(cx, self.cfg["follow_target_size"], self.cfg)
        return (l - r) / 2.0

    def _targets(self, feat, come: bool) -> tuple[float, float]:
        c = self.cfg
        k = self.speed() / c["pet_speed_max"]
        turn = self._turn(feat[0]) * k
        fwd = 0.0
        if come:
            e = c["pet_come_size"] - feat[1]
            sdb = c["follow_size_deadband"]
            if e > sdb:
                fwd = _clamp(c["follow_kp_fwd"] * (e - sdb), 0.0, c["pet_come_max_fwd"]) * k
        mx = c["pet_max_speed"]
        return _clamp(fwd + turn, -mx, mx), _clamp(fwd - turn, -mx, mx)

    # -- per frame ------------------------------------------------------------------------
    def _reconcile(self, t: float) -> None:
        """Notice that the Controller stopped (palm, lost, camera stall, server stop, end)."""
        if self.state in STREAM and not self.ctl.f_active:
            self._set(t, IDLE)
        elif self.state in BODY and self.ctl.move is None:
            if self.state != LOOK:                      # did something: boredom starts over
                self.next_look = max(self.next_look, t + self.cfg["pet_bored_ms"])
            self._set(t, IDLE)
            self.react_until = t + self.cfg["cooldown_ms"]

    def _wave(self, t: float, x: float) -> None:
        c = self.cfg
        amp = c["pet_wave_amp"]
        rev = False
        if self.w_ext is None:
            self.w_ext = x
        elif self.w_dir == 0:
            if abs(x - self.w_ext) >= amp:
                self.w_dir = 1 if x > self.w_ext else -1
                self.w_ext = x
        elif self.w_dir == 1:
            if x > self.w_ext:
                self.w_ext = x
            elif self.w_ext - x >= amp:
                self.w_dir, self.w_ext, rev = -1, x, True
        else:
            if x < self.w_ext:
                self.w_ext = x
            elif x - self.w_ext >= amp:
                self.w_dir, self.w_ext, rev = 1, x, True
        if rev:
            win = c["pet_wave_window_ms"]
            self.w_times = [k for k in self.w_times if t - k <= win] + [t]
            if len(self.w_times) >= c["pet_wave_swings"]:
                if self.waved_t is None:
                    self._event("wave")
                self.waved_t = t

    def on_frame(self, t: float, label: str, feat) -> list:
        c, ctl = self.cfg, self.ctl
        self._decay(t)
        out = ctl.on_frame(t, label, feat)            # safety floor first: palm S, lost S
        self._reconcile(t)
        if label != self.fired:
            self.fired = ""
        if label != g.NO_HAND:
            self.last_hand_t = t
            self.next_look = t + c["pet_bored_ms"]

        # ---- open palm: stop (done by ctl), stay, watch for a wave ----
        if label == g.PALM:
            self._peace_reset(True)
            if self.state == SLEEP:
                self._event("wake palm")
                out += self._fx_cmds("ok", 0, "pet_wake")
            if self.state != STAY:
                self._set(t, STAY)
            if ctl.f_active:
                ctl.f_tgt = (0.0, 0.0)
            if feat is not None:
                self._wave(t, feat[0])
            if self.waved_t is not None and t - self.waved_t > c["pet_wave_window_ms"]:
                self.waved_t = None                     # waved, then held still: that's "stay"
            return out
        self._wave_reset()
        if self.waved_t is not None:                    # palm dropped after a wave: greet
            fresh = t - self.waved_t <= c["pet_wave_window_ms"]
            self.waved_t = None
            if fresh:                                   # (from sleep: the palm already woke it)
                return out + self._greet(t)

        if self.state == SLEEP:                         # only palm / wave wake it
            return out
        if self.state == STAY:
            self._set(t, IDLE)

        # ---- peace: flash = trick, hold pet_exit_ms = leave pet mode ----
        if label == g.PEACE:
            if self.peace_armed:
                if self.peace_since is None:
                    self.peace_since = t
                self.peace_n += 1
                self.peace_gap = 0
                if t - self.peace_since >= c["pet_exit_ms"]:
                    return out + self._leave(t)
            if ctl.f_active:
                ctl.f_tgt = (0.0, 0.0)                  # hold still
            return out
        if not self.peace_armed:
            self._peace_reset(True)
        elif self.peace_since is not None:
            self.peace_gap += 1
            if self.peace_gap > c["pet_gap_frames"]:
                fire = self.peace_n >= c["pet_trick_frames"]
                self._peace_reset(True)
                if fire:
                    return out + self._trick(t, TRICK, 1)

        if label == g.NO_HAND or self.state in BODY:
            return out
        if label == g.TEACH:                            # hold still (the Brain may start teaching)
            if ctl.f_active:
                ctl.f_tgt = (0.0, 0.0)
            return out

        # ---- reactions: held stable_frames (fist: pet_sleep_frames), once per hold ----
        n = ctl.cand_count if ctl.cand == label else 0
        ready = label != self.fired and t >= self.react_until
        if ready and n >= c["stable_frames"]:
            if label == g.THUMBS_UP:
                self.fired = label
                return out + self._happy(t)
            if label == g.THUMBS_DOWN:
                self.fired = label
                return out + self._sad(t)
            if label == g.POINT_UP and self.state != COME:
                self.fired = label
                self._event("come")
                self._stream(t, COME)
        if ready and label == g.FIST and n >= c["pet_sleep_frames"]:
            self.fired = label
            return out + self._sleep(t, "fist")

        # ---- continuous: come here / watch the hand ----
        if feat is None:
            return out
        if self.state == COME:
            if feat[1] >= c["pet_come_size"] - c["follow_size_deadband"]:
                self._boost(c["pet_come_boost"])
                self._set(t, IDLE)
                self._event("arrived")
                return out + ctl._stop(t, "pet_arrived") + self._fx_cmds("ok", 1, "pet_arrived")
            ctl.f_tgt = self._targets(feat, True)
            return out
        tgt = self._targets(feat, False)
        if self.state == IDLE and tgt != (0.0, 0.0):
            self._stream(t, WATCH)
        if self.state == WATCH:
            ctl.f_tgt = tgt
        return out

    def tick(self, t: float) -> list:
        if not self.active:
            return []
        c, ctl = self.cfg, self.ctl
        self._decay(t)
        self._reconcile(t)
        if self.state == SLEEP:
            return []
        if ctl.last_frame_t is None or t - ctl.last_frame_t >= c["frame_timeout_ms"]:
            return []                                   # camera stalled: never start anything
        if self.state == COME and t - self.state_t >= c["pet_come_max_ms"]:
            self._set(t, IDLE)
            self._event("come limit")
            return ctl._stop(t, "pet_come_limit")
        if self.state == WATCH:
            if ctl.f_tgt == (0.0, 0.0) and ctl.f_cur == (0.0, 0.0):
                if self.zero_since is None:
                    self.zero_since = t
                elif t - self.zero_since >= c["pet_settle_ms"]:
                    self._set(t, IDLE)
                    return ctl._stop(t, "pet_settle")
            else:
                self.zero_since = None
        if self.state == IDLE and not ctl.f_active and ctl.move is None:
            if t - self.last_hand_t >= c["pet_sleep_after_ms"]:
                return self._sleep(t, "bored")
            if t >= self.next_look:
                self.next_look = t + c["pet_look_every_ms"]
                if self.rng.uniform() < 0.3 + 0.7 * self.energy:
                    return self._look(t)
        return []
