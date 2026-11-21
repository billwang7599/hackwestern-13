"""Unit tests for pet mode (iter3-pet/vision/common/pet.py): state transitions, mood,
stop-wins from every state, bounded commands, sleep/wake. Stdlib only:

    python3 iter3-pet/vision/tests/test_pet.py -v
"""
from __future__ import annotations

import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "common"), HERE]

import gesture as g  # noqa: E402
import learn as L  # noqa: E402
import pet as P  # noqa: E402

CFG = g.load_config()
DT = 66.0          # 15 fps camera
TICK = 20.0        # the app's ticker


class Sim:
    """Controller + Pet in PET mode, frames every DT with 20 ms ticks between."""

    def __init__(self, **over):
        self.cfg = g.load_config(overrides=over)
        self.ctl = g.Controller(self.cfg)
        self.ctl.mode = g.PET_MODE
        self.pet = P.Pet(self.cfg, self.ctl)
        self.t = 0.0
        self.log: list[tuple[float, g.Cmd]] = []
        self._rec(self.pet.sync(self.t))

    def _rec(self, cmds):
        for c in cmds:
            self.log.append((self.t, c))
        return cmds

    def mark(self) -> int:
        return len(self.log)

    def since(self, m: int):
        return [c for _, c in self.log[m:]]

    def frame(self, label, cx=0.5, size=0.15):
        feat = None if label == g.NO_HAND else (cx, size)
        out = self.pet.sync(self.t)                   # routed exactly like learn.Brain
        if self.pet.active:
            out += self.pet.on_frame(self.t, label, feat)
        else:
            out += self.ctl.on_frame(self.t, label, feat)
        out = self._rec(out + self.pet.sync(self.t))
        k = self.t + TICK
        while k < self.t + DT:
            save, self.t = self.t, k
            out += self._rec(self.ctl.tick(k) + self.pet.sync(k) + self.pet.tick(k))   # as Brain.tick
            self.t = save
            k += TICK
        self.t += DT
        return out

    def frames(self, label, n, **kw):
        out = []
        for _ in range(n):
            out += self.frame(label, **kw)
        return out

    def hold(self, label, ms, **kw):
        return self.frames(label, max(1, int(ms // DT)), **kw)

    def wave(self, swings=4, amp=0.12):
        """Palm moving side to side: `swings` half-swings of `amp` frame widths."""
        out, x, d = [], 0.5, 1
        for _ in range(swings):
            for _ in range(3):
                x += d * amp / 3
                out += self.frame(g.PALM, cx=x)
            d = -d
        return out

    def ticks_only(self, ms):
        """Time passes with ticks but no camera frames (camera stall)."""
        out, end = [], self.t + ms
        while self.t < end:
            self.t += TICK
            out += self._rec(self.ctl.tick(self.t) + self.pet.sync(self.t) + self.pet.tick(self.t))
        return out


def texts(cmds):
    return [c.text for c in cmds]


def reasons(cmds):
    return [c.reason for c in cmds]


def ds(cmds):
    return [tuple(float(v) for v in c.text.split()[1:]) for c in cmds if c.text.startswith("D ")]


def into(state: str) -> Sim:
    """A Sim that is currently in `state` (with motion running where that applies)."""
    s = Sim()
    s.frames(g.NO_HAND, 3)
    if state == P.WATCH:
        s.frames(g.FIST, 5, cx=0.85)
    elif state == P.COME:
        s.frames(g.POINT_UP, 6, size=0.08)
    elif state == P.STAY:
        s.frames(g.PALM, 3)
    elif state == P.GREET:
        s.wave()
        s.frame(g.NO_HAND)
    elif state == P.HAPPY:
        s.frames(g.THUMBS_UP, 4)
    elif state == P.SAD:
        s.frames(g.THUMBS_DOWN, 4)
    elif state == P.TRICK:
        s.frames(g.PEACE, 4)
        s.frames(g.NO_HAND, 3)
    elif state == P.LOOK:
        s = Sim(pet_energy_start=1.0, pet_energy_rest=1.0)
        s.frames(g.NO_HAND, int(CFG["pet_bored_ms"] // DT) + 2)
    elif state == P.SLEEP:
        s.frames(g.FIST, int(CFG["pet_sleep_frames"]) + 1)
    elif state == P.IDLE:
        pass
    return s


class TestEnterLeave(unittest.TestCase):
    def test_enter_says_hello(self):
        s = Sim()
        self.assertTrue(s.pet.active)
        self.assertEqual(s.pet.state, P.IDLE)
        self.assertEqual(texts(s.since(0)), ["LED ok", "BEEP 2", "S"])

    def test_enter_quiet_without_led_beep(self):
        s = Sim(feedback_led_beep=0)
        self.assertEqual(s.since(0), [])

    def test_peace_held_on_entry_does_nothing(self):
        s = Sim()                                    # e.g. entered via the peace toggle, still held
        m = s.mark()
        s.hold(g.PEACE, 3000)
        self.assertEqual(s.ctl.mode, g.PET_MODE)
        self.assertEqual(s.since(m), [])

    def test_peace_hold_leaves_pet_mode(self):
        s = Sim()
        s.frames(g.NO_HAND, 2)
        m = s.mark()
        s.hold(g.PEACE, CFG["pet_exit_ms"] + 200)
        self.assertEqual(s.ctl.mode, g.GESTURE_MODE)
        self.assertFalse(s.pet.active)
        out = s.since(m)
        self.assertIn(("S", "mode_gesture"), [(c.text, c.reason) for c in out])
        self.assertIn("LED off", texts(out))
        self.assertNotIn("pet_trick", reasons(out))  # a hold is not a trick
        self.assertFalse(s.ctl.toggle_armed)          # the Controller won't flip straight to FOLLOW

    def test_peace_flash_is_a_360_trick(self):
        s = Sim()
        s.frames(g.NO_HAND, 2)
        s.frames(g.PEACE, 4)
        self.assertEqual(s.pet.state, P.IDLE)         # fires on release
        out = s.frames(g.NO_HAND, 3)
        self.assertEqual(s.pet.state, P.TRICK)
        self.assertEqual(texts(out)[:2], ["LED ok", "BEEP 1"])
        ts = CFG["turn_speed"]
        self.assertEqual(ds(out)[0], (-ts, ts))
        self.assertEqual(s.ctl.move[3] - s.log[-len(out)][0] >= 0, True)
        # spin lasts turn_ms x spin_turns, then one S
        t0 = s.t
        s.frames(g.NO_HAND, int(CFG["turn_ms"] * CFG["spin_turns"] // DT) + 3)
        self.assertEqual(s.pet.state, P.IDLE)
        self.assertGreaterEqual(s.t - t0, CFG["turn_ms"] * CFG["spin_turns"] - 3 * DT)

    def test_peace_flicker_tolerated_no_trick_from_one_frame(self):
        s = Sim()
        s.frames(g.NO_HAND, 2)
        m = s.mark()
        s.frames(g.PEACE, 1)
        s.frames(g.NO_HAND, 4)
        self.assertNotIn("pet_trick", reasons(s.since(m)))

    def test_teach_leaves_pet_mode(self):
        b = L.Brain(g.load_config(overrides={"pet_start": 1}))
        self.assertEqual(b.ctl.mode, g.PET_MODE)
        import handgen as h
        t = 0.0
        for _ in range(int(CFG["teach_hold_ms"] // DT) + 3):
            b.on_frame(t, h.make_hand("shaka"), h.ASPECT)
            t += DT
        self.assertEqual(b.state, L.COUNTDOWN)
        self.assertEqual(b.ctl.mode, g.GESTURE_MODE)
        self.assertFalse(b.pet.active)


class TestStopWins(unittest.TestCase):
    STATES = [P.IDLE, P.WATCH, P.COME, P.STAY, P.GREET, P.HAPPY, P.SAD, P.TRICK, P.LOOK, P.SLEEP]

    def test_into_helpers_reach_each_state(self):
        for st in self.STATES:
            self.assertEqual(into(st).pet.state, st, st)

    def test_palm_sends_S_first_frame_from_every_state(self):
        for st in self.STATES:
            if st == P.STAY:
                continue                              # already stopped by its own palm
            s = into(st)
            out = s.frame(g.PALM)
            self.assertIn(("S", "palm"), [(c.text, c.reason) for c in out], st)
            # S comes before anything else, and nothing moves in that frame
            self.assertEqual(out[0].text, "S", st)
            self.assertEqual(ds(out), [], st)
            self.assertFalse(s.ctl.f_active or s.ctl.move is not None, st)

    def test_no_motion_while_palm_is_up_from_every_state(self):
        for st in self.STATES:
            s = into(st)
            s.frame(g.PALM)
            m = s.mark()
            s.frames(g.PALM, 40)                      # still palm, ~2.6 s
            self.assertEqual(ds(s.since(m)), [], st)
            self.assertEqual(s.pet.state, P.STAY, st)

    def test_camera_stall_stops_every_moving_state(self):
        for st in [P.WATCH, P.COME, P.GREET, P.HAPPY, P.SAD, P.TRICK, P.LOOK]:
            s = into(st)
            self.assertTrue(s.ctl.f_active or s.ctl.move is not None, st)
            out = s.ticks_only(CFG["frame_timeout_ms"] + 2 * TICK)
            self.assertIn("camera_timeout", reasons(out), st)
            m = s.mark()
            s.ticks_only(30000)                       # stalled for 30 s: never starts anything
            self.assertEqual(s.since(m), [], st)

    def test_server_stop_ends_every_moving_state(self):
        for st in [P.WATCH, P.COME, P.GREET, P.HAPPY, P.SAD, P.TRICK, P.LOOK]:
            b = L.Brain(g.load_config(overrides={"pet_start": 1}))
            s = into(st)
            b.ctl, b.pet = s.ctl, s.pet               # drive the Sim's state through Brain.server_stop
            b.server_stop(s.t)
            self.assertFalse(s.ctl.f_active or s.ctl.move is not None, st)
            out = s.ticks_only(3 * TICK)
            self.assertEqual(ds(out), [], st)         # it never restarts by itself
            self.assertNotEqual(s.pet.state, st, st)

    def test_lost_hand_stops_watch_and_come(self):
        for st in [P.WATCH, P.COME]:
            s = into(st)
            out = s.frames(g.NO_HAND, int(CFG["follow_grace_frames"]))
            self.assertIn(("S", "lost"), [(c.text, c.reason) for c in out], st)
            self.assertEqual(s.pet.state, P.IDLE, st)

    def test_fuzz_palm_always_stops_and_holds(self):
        labels = [g.PALM, g.FIST, g.THUMBS_UP, g.THUMBS_DOWN, g.POINT_UP, g.PEACE, g.UNKNOWN,
                  g.NO_HAND, g.POINT_LEFT, g.TEACH]
        for seed in range(30):
            rng = random.Random(seed)
            s = Sim(pet_energy_start=rng.random())
            lab = g.NO_HAND
            for _ in range(600):
                if rng.random() < 0.12:
                    lab = rng.choice(labels)
                was_palm = s.ctl.cand == g.PALM and s.ctl.cand_count >= 1
                in_pet = s.pet.active
                out = s.frame(lab, cx=rng.uniform(0.1, 0.9), size=rng.uniform(0.05, 0.4))
                if lab == g.PALM and in_pet:
                    self.assertEqual(ds(out), [], f"seed {seed}: D while palm")
                    if not was_palm:
                        self.assertEqual(out[0].text if out else "", "S", f"seed {seed}")
                for c in out:
                    if c.text.startswith("D "):
                        l, r = (float(v) for v in c.text.split()[1:])
                        lim = max(CFG["pet_max_speed"], CFG["turn_speed"])
                        self.assertLessEqual(abs(l), lim)
                        self.assertLessEqual(abs(r), lim)


class TestBehaviours(unittest.TestCase):
    def test_wave_greets_after_palm_drops(self):
        s = into(P.IDLE)
        m = s.mark()
        s.wave()
        self.assertEqual(ds(s.since(m)), [])          # palm up: still, only the S
        self.assertIn("wave", s.pet.events)
        out = s.frame(g.NO_HAND)
        self.assertEqual(s.pet.state, P.GREET)
        self.assertEqual(texts(out)[:2], ["LED ok", "BEEP 2"])
        d = ds(out)
        self.assertEqual(len(d), 1)
        self.assertLess(d[0][0], 0)                   # first swing left

    def test_still_palm_does_not_greet(self):
        s = into(P.IDLE)
        s.frames(g.PALM, 20, cx=0.5)
        s.frames(g.NO_HAND, 3)
        self.assertNotIn(P.GREET, s.pet.events)

    def test_wave_then_still_palm_is_stay(self):
        s = into(P.IDLE)
        s.wave()
        s.hold(g.PALM, CFG["pet_wave_window_ms"] + 300)
        s.frame(g.NO_HAND)
        self.assertNotEqual(s.pet.state, P.GREET)

    def test_greet_wiggle_is_bounded_and_returns(self):
        s = into(P.IDLE)
        s.wave()
        s.frame(g.NO_HAND)
        rest = list(s.ctl.segs)                       # ("D", l, r, ms) still to run
        self.assertEqual(len(rest), 3)
        w = CFG["pet_body_ms"]
        self.assertEqual([x[3] for x in rest], [min(2 * w, CFG["pet_max_seg_ms"]),
                                                min(2 * w, CFG["pet_max_seg_ms"]), w])
        for _, l, r, _ in [s.ctl.move] + rest:
            self.assertLessEqual(abs(l), CFG["pet_max_speed"])
            self.assertAlmostEqual(l, -r)             # turns in place only
        # left w, right 2w, left 2w, right w: net zero, ends facing you
        signs = [1 if x[1] > 0 else -1 for x in [s.ctl.move] + rest]
        self.assertEqual(sum(sg * d for sg, d in zip(signs, [w, 2 * w, 2 * w, w])), 0)

    def test_thumbs_up_happy_then_zoomies(self):
        s = into(P.IDLE)
        e0 = s.pet.energy
        out = s.frames(g.THUMBS_UP, 4)
        self.assertEqual(s.pet.state, P.HAPPY)
        self.assertGreater(s.pet.energy, e0)
        self.assertEqual(texts(out)[:2], ["LED ok", "BEEP 1"])
        # holding does not repeat; re-showing after the move does, and at high energy -> zoomies
        s.frames(g.THUMBS_UP, 20)
        self.assertEqual(s.pet.events.count(P.HAPPY), 1)
        s.frames(g.NO_HAND, 2)
        s.frames(g.THUMBS_UP, 6)
        self.assertEqual(s.pet.events.count(P.HAPPY), 2)
        self.assertGreaterEqual(s.pet.energy, CFG["pet_excited"])
        ts = CFG["turn_speed"]
        self.assertEqual((s.ctl.move[1], s.ctl.move[2]), (-ts, ts))   # the full spin

    def test_thumbs_down_sad_backs_up_bounded(self):
        s = into(P.IDLE)
        e0 = s.pet.energy
        out = s.frames(g.THUMBS_DOWN, 4)
        self.assertEqual(s.pet.state, P.SAD)
        self.assertLess(s.pet.energy, e0)
        self.assertEqual(texts(out)[0], "LED no")
        l, r = ds(out)[0]
        self.assertLess(l, 0)
        self.assertEqual(l, r)
        self.assertLessEqual(CFG["pet_sad_ms"], CFG["pet_max_seg_ms"])
        s.frames(g.NO_HAND, int(CFG["pet_sad_ms"] // DT) + 2)
        self.assertEqual(s.pet.state, P.IDLE)

    def test_segments_clamped_to_max_seg_ms(self):
        s = Sim(pet_body_ms=5000, pet_max_seg_ms=300)
        s.frames(g.NO_HAND, 2)
        s.wave()
        s.frame(g.NO_HAND)
        self.assertLessEqual(s.ctl.move[3] - (s.t - DT), 300 + 1e-6)
        for seg in s.ctl.segs:
            self.assertLessEqual(seg[3], 300)

    def test_come_approaches_and_arrives(self):
        s = into(P.IDLE)
        out = s.frames(g.POINT_UP, 6, size=0.08)
        self.assertEqual(s.pet.state, P.COME)
        d = [x for x in ds(out)]
        self.assertTrue(d and all(l > 0 and r > 0 for l, r in d[-2:]))
        self.assertTrue(all(abs(l) <= CFG["pet_max_speed"] for l, _ in d))
        out = s.frames(g.UNKNOWN, 3, size=CFG["pet_come_size"])     # hand now close
        self.assertIn(("S", "pet_arrived"), [(c.text, c.reason) for c in out])
        self.assertIn("BEEP 1", texts(out))
        self.assertEqual(s.pet.state, P.IDLE)

    def test_come_is_capped_per_command(self):
        s = into(P.IDLE)
        s.frames(g.POINT_UP, 6, size=0.05)
        t0 = s.t
        out = s.hold(g.UNKNOWN, CFG["pet_come_max_ms"] + 500, size=0.05)  # never gets close
        self.assertIn("pet_come_limit", reasons(out))
        self.assertIn("come limit", s.pet.events)
        stop_t = [t for t, c in s.log if c.reason == "pet_come_limit"][0]
        self.assertLessEqual(stop_t - t0, CFG["pet_come_max_ms"] + DT)

    def test_stream_rate_is_pet_send_ms(self):
        s = into(P.COME)
        m = s.mark()
        s.frames(g.UNKNOWN, 15, size=0.05)
        ts = [t for t, c in s.log[m:] if c.reason == "pet_stream"]
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        self.assertTrue(gaps)
        self.assertGreaterEqual(min(gaps), CFG["pet_send_ms"] - 1e-6)
        self.assertLess(max(gaps), 500)               # watchdog stays fed

    def test_watch_turns_toward_hand_then_settles(self):
        s = into(P.IDLE)
        out = s.frames(g.FIST, 4, cx=0.15)            # image left = rover's right (flip = 1)
        self.assertEqual(s.pet.state, P.WATCH)
        l, r = ds(out)[-1]
        self.assertGreater(l, 0)
        self.assertLess(r, 0)
        self.assertAlmostEqual(l + r, 0.0)            # watch never drives forward
        m = s.mark()
        s.hold(g.UNKNOWN, CFG["pet_settle_ms"] + 1200, cx=0.5)   # centred: settles with one S
        out = s.since(m)
        self.assertEqual(reasons(out).count("pet_settle"), 1)
        self.assertEqual(s.pet.state, P.IDLE)

    def test_centred_hand_does_not_start_a_stream(self):
        s = into(P.IDLE)
        out = s.frames(g.UNKNOWN, 10, cx=0.5)
        self.assertEqual(ds(out), [])
        self.assertEqual(s.pet.state, P.IDLE)


class TestSleepAndBoredom(unittest.TestCase):
    def test_fist_held_sleeps(self):
        s = into(P.IDLE)
        out = s.frames(g.FIST, int(CFG["pet_sleep_frames"]), cx=0.5)
        self.assertEqual(s.pet.state, P.SLEEP)
        self.assertIn("LED off", texts(out))

    def test_sleep_ignores_gestures(self):
        s = into(P.SLEEP)
        m = s.mark()
        for lab in [g.THUMBS_UP, g.THUMBS_DOWN, g.POINT_UP, g.PEACE, g.FIST, g.UNKNOWN]:
            s.frames(lab, 30, cx=0.2, size=0.05)
        self.assertEqual(s.since(m), [])
        self.assertEqual(s.pet.state, P.SLEEP)
        self.assertEqual(s.ctl.mode, g.PET_MODE)      # held peace does not leave while asleep

    def test_palm_wakes(self):
        s = into(P.SLEEP)
        out = s.frame(g.PALM)
        self.assertEqual(texts(out), ["S", "LED ok"])
        self.assertEqual(s.pet.state, P.STAY)
        self.assertIn("wake palm", s.pet.events)
        s.frame(g.NO_HAND)
        self.assertEqual(s.pet.state, P.IDLE)

    def test_wave_wakes_and_greets(self):
        s = into(P.SLEEP)
        s.wave()
        s.frame(g.NO_HAND)
        self.assertEqual(s.pet.state, P.GREET)

    def test_bored_looks_then_sleeps(self):
        s = Sim(pet_energy_start=1.0, pet_energy_rest=1.0)    # always look (chance 0.3 + 0.7 e)
        m = s.mark()
        s.hold(g.NO_HAND, CFG["pet_bored_ms"] - 300)
        self.assertEqual(ds(s.since(m)), [])          # not bored yet
        s.hold(g.NO_HAND, 1000)
        self.assertIn(P.LOOK, s.pet.events)
        looks = s.since(m)
        d = ds(looks)
        self.assertEqual(len([x for x in d if x != (0.0, 0.0)]) >= 2, True)
        s.hold(g.NO_HAND, CFG["pet_sleep_after_ms"])
        self.assertEqual(s.pet.state, P.SLEEP)
        self.assertIn("sleep bored", s.pet.events)
        n_look = s.pet.events.count(P.LOOK)
        expected = 1 + int((CFG["pet_sleep_after_ms"] - CFG["pet_bored_ms"]) // CFG["pet_look_every_ms"])
        self.assertLessEqual(abs(n_look - expected), 1)

    def test_look_is_net_zero_turn(self):
        s = Sim(pet_energy_start=1.0, pet_energy_rest=1.0)
        s.hold(g.NO_HAND, CFG["pet_bored_ms"] + 200)
        segs = [s.ctl.move] + list(s.ctl.segs)
        self.assertEqual(len(segs), 3)
        self.assertAlmostEqual(segs[0][1] + segs[2][1], 0.0)
        self.assertEqual((segs[1][1], segs[1][2]), (0.0, 0.0))

    def test_hand_resets_boredom(self):
        s = Sim()
        for _ in range(10):
            s.hold(g.NO_HAND, CFG["pet_bored_ms"] - 500)
            s.frame(g.UNKNOWN, cx=0.5)
        self.assertNotIn(P.LOOK, s.pet.events)
        self.assertNotEqual(s.pet.state, P.SLEEP)


class TestMood(unittest.TestCase):
    def test_energy_decays_toward_rest(self):
        s = Sim(pet_energy_start=0.9)
        s.ticks_only(0)
        s.frame(g.UNKNOWN)
        e0 = s.pet.energy
        s.hold(g.UNKNOWN, 5000)
        rate = CFG["pet_energy_decay_per_s"]
        self.assertAlmostEqual(s.pet.energy, max(CFG["pet_energy_rest"], e0 - rate * 5.0), delta=0.01)
        s.hold(g.UNKNOWN, 60000)
        self.assertAlmostEqual(s.pet.energy, CFG["pet_energy_rest"])

    def test_energy_scales_speed(self):
        lo, hi = Sim(pet_energy_start=0.0), Sim(pet_energy_start=1.0)
        self.assertAlmostEqual(lo.pet.speed(), CFG["pet_speed_min"])
        self.assertAlmostEqual(hi.pet.speed(), min(CFG["pet_speed_max"], CFG["pet_max_speed"]))

    def test_energy_clamped(self):
        s = Sim()
        for _ in range(10):
            s.pet._boost(0.5)
        self.assertEqual(s.pet.energy, 1.0)
        for _ in range(10):
            s.pet._boost(-0.5)
        self.assertEqual(s.pet.energy, 0.0)

    def test_deterministic(self):
        def run():
            s = Sim(pet_seed=7)
            rng = random.Random(3)
            for _ in range(800):
                s.frame(rng.choice([g.NO_HAND] * 6 + [g.THUMBS_UP, g.UNKNOWN, g.PEACE]),
                        cx=rng.random(), size=rng.uniform(0.05, 0.4))
            return [(t, c.text, c.reason) for t, c in s.log], s.pet.events
        self.assertEqual(run(), run())

    def test_rng_matches_lcg(self):
        a, b = P.Rng(5), L.Lcg(5)
        self.assertEqual([a.uniform() for _ in range(10)], [b.uniform() for _ in range(10)])


if __name__ == "__main__":
    unittest.main()
