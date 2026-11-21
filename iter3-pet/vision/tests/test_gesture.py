"""Unit tests: classifier, gesture-mode debounce/moves/fail-safes, follow mode,
mode toggle. No camera, no MediaPipe, stdlib only:

    python3 vision/tests/test_gesture.py -v
"""
from __future__ import annotations

import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "common"), HERE]

import gesture as g  # noqa: E402
import handgen as h  # noqa: E402

CFG = g.load_config()
FRAME = 33.0          # ~30 fps


def ctl(**over):
    return g.Controller(g.load_config(overrides=over))


def feed(c, t0, labels, dt=FRAME, feat=None):
    """Feed labels one frame apart starting at t0. Returns (cmds, next_t)."""
    out, t = [], t0
    for lab in labels:
        out += c.on_frame(t, lab, feat)
        t += dt
    return out, t


def ticks(c, t0, t1, step=20.0):
    out, t = [], t0
    while t < t1:
        out += c.tick(t)
        t += step
    return out


def texts(cmds):
    return [x.text for x in cmds]


def reasons(cmds):
    return [x.reason for x in cmds]


def parse_d(text):
    _, l, r = text.split()
    return float(l), float(r)


# ---------------------------------------------------------------- classifier
class TestClassifier(unittest.TestCase):
    def test_synthetic_poses_all_positions(self):
        rng = random.Random(7)
        for (pose, d), want in h.EXPECTED.items():
            for _ in range(40):
                lm = h.make_hand(pose, d, cx=rng.uniform(0.2, 0.8), cy=rng.uniform(0.3, 0.7),
                                 size=rng.uniform(0.1, 0.3), jitter_deg=rng.uniform(-15, 15),
                                 noise=0.002, rng=rng)
                self.assertEqual(g.classify(lm, h.ASPECT, CFG), want, (pose, d))

    def test_no_hand(self):
        self.assertEqual(g.classify(None, h.ASPECT, CFG), g.NO_HAND)
        self.assertEqual(g.classify([], h.ASPECT, CFG), g.NO_HAND)

    def test_degenerate_hand_is_unknown(self):
        self.assertEqual(g.classify([(0.5, 0.5, 0)] * 21, h.ASPECT, CFG), g.UNKNOWN)

    def test_diagonal_point_is_unknown(self):
        lm = h.make_hand("point", "up", jitter_deg=45)      # exactly between up and right
        self.assertEqual(g.classify(lm, h.ASPECT, CFG), g.UNKNOWN)

    def test_features(self):
        cx, size = g.hand_features(h.make_hand("fist", cx=0.3, size=0.15), h.ASPECT)
        self.assertAlmostEqual(cx, 0.3, places=6)
        self.assertAlmostEqual(size, 0.15, places=6)
        self.assertIsNone(g.hand_features(None, h.ASPECT))

    def test_config_rejects_typos(self):
        with self.assertRaises(ValueError):
            g.load_config(overrides={"stable_framez": 3})


# ---------------------------------------------------------------- gesture mode
class TestGestureMode(unittest.TestCase):
    def test_debounce_needs_stable_frames(self):
        c = ctl()
        out, t = feed(c, 0, ["thumbs_up"] * 3)
        self.assertEqual(out, [])
        out, t = feed(c, t, ["thumbs_up"])
        self.assertEqual(texts(out), ["D 0.50 0.50"])
        self.assertEqual(reasons(out), ["forward"])

    def test_flicker_resets_debounce(self):
        c = ctl()
        out, _ = feed(c, 0, ["thumbs_up", "thumbs_up", "unknown", "thumbs_up", "thumbs_up", "thumbs_up"])
        self.assertEqual(out, [])

    def test_step_resends_then_stops(self):
        c = ctl()
        out, t = feed(c, 0, ["point_up"] * 4)            # fires at t=99
        t_fire = 3 * FRAME
        while t < t_fire + CFG["step_ms"] + 40:           # hand still there, no command pose
            out += c.on_frame(t, "unknown")
            out += ticks(c, t + 1, t + FRAME)
            t += FRAME
        # resends every resend_ms, then exactly one S when step_ms elapses
        self.assertEqual(reasons(out)[0], "forward")
        self.assertEqual(reasons(out)[-1], "done")
        self.assertEqual(texts(out)[-1], "S")
        n_resend = reasons(out).count("resend")
        self.assertGreaterEqual(n_resend, CFG["step_ms"] // CFG["resend_ms"] - 1)
        # never a gap longer than the motor watchdog (500 ms)
        self.assertLess(CFG["resend_ms"], 500)

    def test_back_and_turns(self):
        for label, want in [("fist", "D -0.50 -0.50"), ("point_down", "D -0.50 -0.50"),
                            ("thumbs_down", "D -0.50 -0.50"),
                            ("point_left", "D -0.50 0.50"), ("point_right", "D 0.50 -0.50")]:
            c = ctl()
            out, _ = feed(c, 0, [label] * 4)
            self.assertEqual(texts(out), [want], label)

    def test_point_left_is_left_flag_swaps(self):
        c = ctl(point_left_is_left=0)
        out, _ = feed(c, 0, ["point_left"] * 4)
        self.assertEqual(texts(out), ["D 0.50 -0.50"])

    def test_turn_duration_is_turn_ms(self):
        c = ctl(turn_ms=450)
        feed(c, 0, ["point_left"] * 4)                    # fires at 99, last frame 99
        out = []
        for t in range(100, 700, 5):
            out += c.on_frame(t, "point_left") if t % 30 == 0 else c.tick(t)
            if out and out[-1].text == "S":
                self.assertGreaterEqual(t, 99 + 450)
                self.assertLess(t, 99 + 450 + 30)
                return
        self.fail("turn never ended")

    def test_palm_stops_mid_move_on_first_frame(self):
        c = ctl()
        _, t = feed(c, 0, ["thumbs_up"] * 4)
        out, _ = feed(c, t, ["palm"])
        self.assertEqual(texts(out), ["S"])
        self.assertEqual(reasons(out), ["palm"])
        self.assertIsNone(c.moving)

    def test_palm_when_idle_sends_one_stop_not_a_stream(self):
        c = ctl()
        out, _ = feed(c, 0, ["palm"] * 10)
        self.assertEqual(texts(out), ["S"])

    def test_cooldown_and_hold_repeat(self):
        c = ctl()
        out, t = feed(c, 0, ["thumbs_up"] * 40, dt=33)    # 1.3 s held
        fired = [x for x in out if x.reason == "forward"]
        self.assertEqual(len(fired), 2)                   # 600 ms step + 400 ms cooldown
        c = ctl(hold_repeat=0)
        out, t = feed(c, 0, ["thumbs_up"] * 60)
        self.assertEqual(reasons(out).count("forward"), 1)
        out, t = feed(c, t, ["unknown"] + ["thumbs_up"] * 4)  # release, then again
        self.assertEqual(reasons(out).count("forward"), 1)

    def test_hand_lost_stops_after_lost_ms(self):
        c = ctl()
        _, t = feed(c, 0, ["thumbs_up"] * 4)
        last_hand = t - FRAME
        out = []
        while not any(x.reason == "lost" for x in out):
            out += c.on_frame(t, "no_hand")
            t += FRAME
        t_stop = t - FRAME
        self.assertGreaterEqual(t_stop - last_hand, CFG["lost_ms"])
        self.assertLess(t_stop - last_hand, CFG["lost_ms"] + FRAME + 1)

    def test_no_hand_while_idle_sends_nothing(self):
        c = ctl()
        out, _ = feed(c, 0, ["no_hand"] * 30)
        self.assertEqual(out, [])

    def test_camera_timeout_stops(self):
        c = ctl()
        _, t = feed(c, 0, ["thumbs_up"] * 4)
        out = ticks(c, t, t + CFG["frame_timeout_ms"] + 40)
        self.assertEqual(out[-1].reason, "camera_timeout")
        # and a step longer than the timeout is cut short by it
        self.assertNotIn("done", reasons(out))


# ---------------------------------------------------------------- follow mode
def follow_ctl(**over):
    c = ctl(**over)
    c.mode = g.FOLLOW_MODE
    return c


def follow_run(c, lm_or_none, n_frames, t0=0.0):
    """Frames at 30 fps with ticks every 10 ms between them."""
    out, t = [], t0
    for _ in range(n_frames):
        if lm_or_none is None:
            out += c.on_frame(t, g.NO_HAND, None)
        else:
            out += c.on_frame(t, g.classify(lm_or_none, h.ASPECT, CFG),
                              g.hand_features(lm_or_none, h.ASPECT))
        for k in (10, 20, 30):
            out += c.tick(t + k)
        t += FRAME
    return out, t


class TestFollow(unittest.TestCase):
    # flip=1: image x is mirrored, so image-left (cx<0.5) = rover's RIGHT.
    def test_targets_left_right_centre(self):
        l, r = g.follow_targets(0.2, CFG["follow_target_size"], CFG)   # rover's right
        self.assertGreater(l, r)
        l, r = g.follow_targets(0.8, CFG["follow_target_size"], CFG)   # rover's left
        self.assertLess(l, r)
        self.assertEqual(g.follow_targets(0.5, CFG["follow_target_size"], CFG), (0.0, 0.0))

    def test_deadband(self):
        x = 0.5 + CFG["follow_deadband"] * 0.9
        self.assertEqual(g.follow_targets(x, CFG["follow_target_size"], CFG), (0.0, 0.0))

    def test_invert_and_noflip(self):
        a = g.follow_targets(0.2, CFG["follow_target_size"], CFG)
        b = g.follow_targets(0.2, CFG["follow_target_size"],
                             g.load_config(overrides={"follow_invert_turn": 1}))
        c = g.follow_targets(0.2, CFG["follow_target_size"], g.load_config(overrides={"flip": 0}))
        self.assertEqual((a[0] > a[1]), not (b[0] > b[1]))
        self.assertEqual((a[0] > a[1]), not (c[0] > c[1]))

    def test_small_far_hand_drives_forward_clamped(self):
        l, r = g.follow_targets(0.5, 0.02, CFG)
        self.assertEqual(l, r)
        self.assertAlmostEqual(l, CFG["follow_max_fwd"])

    def test_large_close_hand_backs_up_slowly(self):
        l, r = g.follow_targets(0.5, 0.6, CFG)
        self.assertAlmostEqual(l, -CFG["follow_max_back"])
        self.assertEqual(l, r)
        l, r = g.follow_targets(0.5, 0.6, g.load_config(overrides={"follow_max_back": 0}))
        self.assertEqual((l, r), (0.0, 0.0))

    def test_turn_clamped(self):
        l, r = g.follow_targets(0.0, CFG["follow_target_size"], CFG)
        self.assertAlmostEqual(l - r, 2 * CFG["follow_max_turn"])

    def test_stream_rate_slew_and_direction(self):
        c = follow_ctl()
        lm = h.make_hand("fist", cx=0.2, size=0.08)      # far + rover's right
        out, t = follow_run(c, lm, 45)                    # 1.5 s
        ds = [x for x in out if x.reason == "follow"]
        # ~15 Hz: 1.5 s / 66 ms ~ 22 messages
        self.assertGreaterEqual(len(ds), 18)
        self.assertLessEqual(len(ds), 25)
        vals = [parse_d(x.text) for x in ds]
        # slew: first message is small, then ramps; never jumps more than slew*2*send
        self.assertLessEqual(abs(vals[0][0]), CFG["follow_slew_per_s"] * CFG["follow_send_ms"] / 1000 + 0.01)
        for (a, b), (c2, d2) in zip(vals, vals[1:]):
            self.assertLessEqual(abs(c2 - a), CFG["follow_slew_per_s"] * 2 * CFG["follow_send_ms"] / 1000 + 0.011)
        l, r = vals[-1]
        self.assertGreater(l, r)                          # turning right
        self.assertGreater(l + r, 0)                      # and going forward
        # reached the target
        tl, tr = g.follow_targets(*g.hand_features(lm, h.ASPECT), CFG)
        self.assertAlmostEqual(l, tl, places=2)
        self.assertAlmostEqual(r, tr, places=2)

    def test_centre_at_target_size_sends_zeros_and_keeps_watchdog_fed(self):
        c = follow_ctl()
        lm = h.make_hand("fist", cx=0.5, size=CFG["follow_target_size"])
        out, _ = follow_run(c, lm, 20)
        self.assertTrue(out)
        self.assertTrue(all(x.text == "D 0.00 0.00" for x in out))

    def test_any_hand_shape_is_followed_except_palm_and_peace(self):
        for pose, d in [("fist", "up"), ("point", "up"), ("thumb", "up"), ("point", "left")]:
            c = follow_ctl()
            out, _ = follow_run(c, h.make_hand(pose, d, cx=0.5, size=0.05), 10)
            self.assertTrue(any(x.reason == "follow" and parse_d(x.text)[0] > 0 for x in out), pose)

    def test_gesture_moves_do_not_fire_in_follow_mode(self):
        c = follow_ctl()
        out, _ = follow_run(c, h.make_hand("thumb", "up", cx=0.5, size=0.05), 20)
        self.assertNotIn("forward", reasons(out))

    def test_lost_after_grace_frames(self):
        c = follow_ctl()
        _, t = follow_run(c, h.make_hand("fist", cx=0.5, size=0.05), 10)
        grace = int(CFG["follow_grace_frames"])
        out, t2 = follow_run(c, None, grace - 1, t)
        self.assertNotIn("S", texts(out))                 # still within grace: keeps going
        self.assertIn("follow", reasons(out))
        out, _ = follow_run(c, None, 1, t2)
        self.assertEqual(texts(out)[0], "S")
        self.assertEqual(out[0].reason, "lost")
        out, _ = follow_run(c, None, 10, t2 + FRAME)
        self.assertEqual(out, [])                         # silent afterwards

    def test_resumes_when_hand_returns(self):
        c = follow_ctl()
        _, t = follow_run(c, h.make_hand("fist", cx=0.5, size=0.05), 5)
        _, t = follow_run(c, None, 10, t)
        out, _ = follow_run(c, h.make_hand("fist", cx=0.5, size=0.05), 5, t)
        self.assertIn("follow", reasons(out))

    def test_palm_held_stops_in_follow_mode(self):
        c = follow_ctl()
        _, t = follow_run(c, h.make_hand("fist", cx=0.5, size=0.05), 10)
        out, t = follow_run(c, h.make_hand("palm", cx=0.5, size=0.05), 1, t)
        self.assertNotIn("palm", reasons(out))           # 1 frame: not yet
        out, t = follow_run(c, h.make_hand("palm", cx=0.5, size=0.05), 1, t)
        self.assertEqual(reasons(out)[0], "palm")
        out, t = follow_run(c, h.make_hand("palm", cx=0.5, size=0.05), 10, t)
        self.assertEqual(out, [])                         # stays stopped, no D

    def test_camera_timeout_in_follow(self):
        c = follow_ctl()
        _, t = follow_run(c, h.make_hand("fist", cx=0.5, size=0.05), 5)
        out = ticks(c, t, t + CFG["frame_timeout_ms"] + 40)
        self.assertEqual(out[-1].reason, "camera_timeout")
        self.assertFalse(c.f_active)


# ---------------------------------------------------------------- mode toggle
class TestModeToggle(unittest.TestCase):
    def test_peace_toggles_after_mode_frames(self):
        c = ctl()
        n = int(CFG["mode_frames"])
        out, t = feed(c, 0, ["peace"] * (n - 1))
        self.assertEqual(c.mode, g.GESTURE_MODE)
        self.assertEqual(out, [])
        out, t = feed(c, t, ["peace"])
        self.assertEqual(c.mode, g.FOLLOW_MODE)
        self.assertEqual(texts(out), ["S"])
        self.assertEqual(reasons(out), ["mode_follow"])

    def test_holding_peace_does_not_flip_back(self):
        c = ctl()
        feed(c, 0, ["peace"] * 200)                       # 6.6 s held
        self.assertEqual(c.mode, g.FOLLOW_MODE)

    def test_cooldown_blocks_quick_second_toggle(self):
        c = ctl(pet_in_cycle=0)                           # two-mode toggle
        n = int(CFG["mode_frames"])
        _, t = feed(c, 0, ["peace"] * n)                  # -> follow at ~165 ms
        _, t = feed(c, t, ["no_hand"] * 2)                # release
        _, t = feed(c, t, ["peace"] * n)                  # within 1.5 s cooldown
        self.assertEqual(c.mode, g.FOLLOW_MODE)
        _, t = feed(c, t, ["no_hand"] * 2)
        t = max(t, CFG["mode_cooldown_ms"] + 200)
        _, t = feed(c, t, ["peace"] * n)
        self.assertEqual(c.mode, g.GESTURE_MODE)

    def test_flicker_does_not_toggle(self):
        c = ctl()
        seq = (["peace"] * 3 + ["unknown"]) * 10
        feed(c, 0, seq)
        self.assertEqual(c.mode, g.GESTURE_MODE)

    def test_toggle_cancels_move_and_stops(self):
        c = ctl(pet_in_cycle=0)
        _, t = feed(c, 0, ["thumbs_up"] * 4)
        self.assertEqual(c.moving, g.FORWARD)
        out, t = feed(c, t, ["peace"] * int(CFG["mode_frames"]))
        self.assertIsNone(c.moving)
        self.assertIn("mode_follow", reasons(out))
        # back to gesture mode: robot is stopped and following is off
        _, t = feed(c, t, ["no_hand"])
        t += CFG["mode_cooldown_ms"]
        _, t = feed(c, t, ["fist"] * 3)                    # follow a bit
        out, t = feed(c, t, ["peace"] * int(CFG["mode_frames"]))
        self.assertEqual(c.mode, g.GESTURE_MODE)
        self.assertFalse(c.f_active)
        self.assertEqual(out[-1].text, "S")

    def test_peace_cycles_gesture_follow_pet(self):
        c = ctl()                                         # pet_in_cycle = 1 (default)
        n = int(CFG["mode_frames"])
        _, t = feed(c, 0, ["peace"] * n)
        self.assertEqual(c.mode, g.FOLLOW_MODE)
        _, t = feed(c, t, ["no_hand"] * 2)
        t = max(t, CFG["mode_cooldown_ms"] + 200)
        out, t = feed(c, t, ["peace"] * n)
        self.assertEqual(c.mode, g.PET_MODE)
        self.assertIn("mode_pet", reasons(out))
        # in PET mode the Controller never toggles on peace (pet.py owns it)
        _, t = feed(c, t + CFG["mode_cooldown_ms"], ["no_hand"] * 2 + ["peace"] * 100)
        self.assertEqual(c.mode, g.PET_MODE)

    def test_next_mode(self):
        self.assertEqual(g.next_mode(g.GESTURE_MODE, CFG), g.FOLLOW_MODE)
        self.assertEqual(g.next_mode(g.FOLLOW_MODE, CFG), g.PET_MODE)
        self.assertEqual(g.next_mode(g.PET_MODE, CFG), g.GESTURE_MODE)
        two = g.load_config(overrides={"pet_in_cycle": 0})
        self.assertEqual(g.next_mode(g.FOLLOW_MODE, two), g.GESTURE_MODE)

    def test_peace_in_follow_holds_still(self):
        c = follow_ctl(mode_frames=100)                   # don't toggle during this test
        _, t = follow_run(c, h.make_hand("fist", cx=0.5, size=0.05), 30)
        out, _ = follow_run(c, h.make_hand("peace", cx=0.5, size=0.05), 10, t)
        last = [parse_d(x.text) for x in out if x.reason == "follow"][-1]
        self.assertEqual(last, (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
