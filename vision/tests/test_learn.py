"""Unit tests for teach-on-the-spot (vision/common/learn.py). No camera, stdlib only.

Covers: invariant features, few-shot learn + recognise, rejection of unseen
poses, too-similar rejection, palm-stop can't be overridden, save/load, and the
whole hands-only teach state machine (trigger, countdown feedback, recording,
assignment by gesture, cancel, timeouts, forget, Uno button) via a fake motor
client that records every line the robot would receive.

    python3 vision/tests/test_learn.py -v
"""
from __future__ import annotations

import json
import os
import random
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "common"), HERE]

import gesture as g  # noqa: E402
import handgen as h  # noqa: E402
import learn as L  # noqa: E402

CFG = g.load_config()
AW = CFG["learn_angle_weight"]


def hand(pose, d="up", rng=None, **kw):
    rng = rng or random.Random(0)
    kw.setdefault("cx", rng.uniform(0.3, 0.7))
    kw.setdefault("size", rng.uniform(0.12, 0.3))
    kw.setdefault("jitter_deg", rng.uniform(-40, 40))
    kw.setdefault("noise", 0.004)
    return h.make_hand(pose, d, rng=rng, **kw)


def feats(pose, n, seed=1, **kw):
    rng = random.Random(seed)
    return [L.features(hand(pose, rng=rng, **kw), h.ASPECT, AW) for _ in range(n)]


class FakeMotor:
    """Stands in for tars_motor: records every line and replies like the server
    (LED/BEEP unsupported -> ERR, as an older tars_motor would)."""

    def __init__(self):
        self.lines: list[tuple[float, str, str]] = []

    def send(self, t, cmds):
        for c in cmds:
            self.lines.append((t, c.text, c.reason))
        return ["ERR unknown" if c.text.split()[0] in ("LED", "BEEP") else "OK" for c in cmds]

    def texts(self, since=0.0):
        return [x for t, x, _ in self.lines if t >= since]

    def reasons(self):
        return [r for _, _, r in self.lines]


class Sim:
    """Drives a Brain like the runtime does: frames every `dt` ms, ticks every 20 ms."""

    def __init__(self, cfg=None, dt=33.0):
        self.b = L.Brain(cfg or CFG)
        self.m = FakeMotor()
        self.t = 0.0
        self.dt = dt
        self.rng = random.Random(5)

    def frames(self, pose, ms, d="up"):
        end = self.t + ms
        while self.t < end:
            k = self.t + 20.0
            while k < self.t + self.dt:
                self.m.send(k, self.b.tick(k))
                k += 20.0
            self.t += self.dt
            lm = None if pose is None else hand(pose, d, rng=self.rng, cx=0.5, size=0.2,
                                                jitter_deg=self.rng.uniform(-10, 10))
            self.m.send(self.t, self.b.on_frame(self.t, lm, h.ASPECT))

    def button(self, kind):
        self.m.send(self.t, self.b.button(self.t, kind))

    def events(self):
        e, self.b.events = self.b.events, []
        return e

    def teach(self, pose, pick, pick_dir="up"):
        """Full hands-only teach: hold shaka 2 s, countdown, show pose, show pick."""
        self.frames("shaka", CFG["teach_hold_ms"] + 100)
        self.frames(None, CFG["teach_countdown_ms"])            # hand down during countdown
        self.frames(pose, 2500)                                  # record
        if pick is None:
            self.frames(None, CFG["learn_assign_ms"] + 200)
        else:
            self.frames(pick, 600, pick_dir)
        self.frames(None, 6000)                                  # confirmation action plays out


# ---------------------------------------------------------------- features
class TestFeatures(unittest.TestCase):
    def test_invariance(self):
        base = h.make_hand("rock", "up", cx=0.5, cy=0.5, size=0.2)
        f0 = L.features(base, h.ASPECT, AW)
        for cx, cy, size, rot in [(0.2, 0.3, 0.1, 0), (0.8, 0.7, 0.35, 0), (0.5, 0.5, 0.2, 73),
                                  (0.3, 0.6, 0.15, -140), (0.5, 0.5, 0.2, 180)]:
            lm = h.make_hand("rock", "up", cx=cx, cy=cy, size=size, jitter_deg=rot)
            self.assertLess(L.dist(f0, L.features(lm, h.ASPECT, AW)), 1e-9, (cx, cy, size, rot))

    def test_mirror_invariance(self):
        lm = h.make_hand("three", "up", jitter_deg=20)
        mir = [(1.0 - x, y, z) for x, y, z in lm]
        self.assertLess(L.dist(L.features(lm, h.ASPECT, AW), L.features(mir, h.ASPECT, AW)), 1e-9)

    def test_aspect(self):
        lm = h.make_hand("rock", "up", aspect=16 / 9)
        f = L.features(lm, 16 / 9, AW)
        self.assertLess(L.dist(f, L.features(h.make_hand("rock", "up"), h.ASPECT, AW)), 1e-9)

    def test_shape_and_degenerate(self):
        self.assertEqual(len(L.features(h.make_hand("fist"), h.ASPECT, AW)), L.N_FEAT)
        self.assertIsNone(L.features(None, h.ASPECT, AW))
        self.assertIsNone(L.features([(0.5, 0.5, 0)] * 21, h.ASPECT, AW))

    def test_lcg_deterministic(self):
        a, b = L.Lcg(7), L.Lcg(7)
        xs = [a.uniform() for _ in range(5)]
        self.assertEqual(xs, [b.uniform() for _ in range(5)])
        self.assertTrue(all(0 <= x < 1 for x in xs))


# ---------------------------------------------------------------- classifier
class TestLearner(unittest.TestCase):
    def setUp(self):
        self.ln = L.Learner(CFG)
        self.ln.add("pose1", "spin", feats("rock", 40, seed=1))

    def test_recognise_new_frames_any_position_rotation(self):
        hits = [self.ln.match(f)[0] for f in feats("rock", 100, seed=2)]
        self.assertEqual(hits.count("pose1"), 100)

    def test_mirrored_hand_recognised(self):
        rng = random.Random(3)
        for _ in range(30):
            lm = [(1 - x, y, z) for x, y, z in hand("rock", rng=rng)]
            self.assertEqual(self.ln.match(L.features(lm, h.ASPECT, AW))[0], "pose1")

    def test_unseen_poses_rejected(self):
        for pose in ["three", "ring_pinky", "middle", "fist", "palm", "peace", "point", "shaka"]:
            for f in feats(pose, 40, seed=4):
                name, best, ratio, conf = self.ln.match(f)
                self.assertIsNone(name, pose)
                self.assertLess(conf, 0.5)

    def test_random_poses_rejected(self):
        rng = random.Random(9)
        for _ in range(200):
            lm = [(rng.uniform(0.3, 0.7), rng.uniform(0.3, 0.7), 0.0) for _ in range(21)]
            f = L.features(lm, h.ASPECT, AW)
            if f is not None:
                self.assertIsNone(self.ln.match(f)[0])

    def test_two_classes(self):
        self.ln.add("pose2", "arms", feats("three", 40, seed=5))
        for f in feats("three", 40, seed=6):
            self.assertEqual(self.ln.match(f)[0], "pose2")
        for f in feats("rock", 40, seed=7):
            self.assertEqual(self.ln.match(f)[0], "pose1")

    def test_confidence(self):
        name, best, ratio, conf = self.ln.match(feats("rock", 1, seed=8)[0])
        self.assertEqual(best, "pose1")
        self.assertGreater(conf, 0.5)
        self.assertAlmostEqual(conf, 1 / (1 + ratio * ratio))

    def test_similar_to(self):
        self.assertEqual(self.ln.similar_to(feats("rock", 40, seed=10)), "pose1")
        self.assertIsNone(self.ln.similar_to(feats("three", 40, seed=11)))

    def test_threshold_clamped(self):
        c = self.ln.classes[0]
        self.assertGreaterEqual(c.thr, CFG["learn_thr_min"])
        self.assertLessEqual(c.thr, CFG["learn_thr_max"])

    def test_json_round_trip(self):
        d = json.loads(json.dumps(self.ln.to_json()))
        ln2 = L.Learner(CFG)
        ln2.from_json(d)
        self.assertEqual(ln2.classes[0].examples, self.ln.classes[0].examples)
        self.assertEqual(ln2.classes[0].thr, self.ln.classes[0].thr)
        for f in feats("rock", 10, seed=12) + feats("three", 10, seed=13):
            self.assertEqual(ln2.match(f), self.ln.match(f))
        self.assertEqual(ln2.free_name(), "pose2")

    def test_benchmark(self):
        """Prints classification µs/frame (features + k-NN) for the judges' slide."""
        ln = L.Learner(CFG)
        for i, p in enumerate(["rock", "three", "ring_pinky", "middle"]):
            ln.add(f"pose{i + 1}", "spin", feats(p, int(CFG["learn_frames"]), seed=20 + i))
        lms = [hand("rock", rng=random.Random(i)) for i in range(50)]
        t0 = time.perf_counter()
        for lm in lms:
            ln.match(L.features(lm, h.ASPECT, AW))
        us = (time.perf_counter() - t0) / len(lms) * 1e6
        print(f"\n[bench] python: {us:.0f} us/frame, 4 learned poses x "
              f"{len(ln.classes[0].examples)} examples", file=sys.stderr)


# ---------------------------------------------------------------- pipeline + safety
class TestBrainSafety(unittest.TestCase):
    def test_learned_never_shadows_builtins(self):
        b = L.Brain(CFG)
        # even a learned class built from PALM / FIST frames can't change their label
        b.learner.add("pose1", "spin", feats("palm", 40))
        b.learner.add("pose2", "spin", feats("fist", 40))
        b._sync()
        for pose, want in [("palm", g.PALM), ("fist", g.FIST), ("peace", g.PEACE)]:
            for lm in [hand(pose, rng=random.Random(i)) for i in range(20)]:
                self.assertEqual(b.classify(lm, h.ASPECT)[1], want)

    def test_palm_stops_learned_action(self):
        s = Sim()
        s.teach("rock", "peace")                          # rock -> spin
        self.assertIn("pose1", s.b.ctl.learned)
        s.events()
        s.frames("rock", 400)
        self.assertEqual(s.b.ctl.moving, "spin@pose1")
        t = s.t
        s.frames("palm", 40)
        self.assertIsNone(s.b.ctl.moving)
        self.assertIn(("S", "palm"), [(x, r) for tt, x, r in s.m.lines if tt >= t])

    def test_palm_stops_during_every_teach_state(self):
        for when in (2200, 3500, 5600, 7500):            # countdown, countdown, record, assign
            s = Sim()
            s.frames("shaka", 2100)
            s.frames(None, 3000) if when > 5100 else None
            s.frames("rock", 2500) if when > 7000 else None
            s.frames("palm", 120)
            self.assertEqual(s.b.state, L.IDLE, when)
            self.assertIn("cancel palm", s.events(), when)
            self.assertIn("palm", s.m.reasons(), when)
            self.assertFalse(s.b.ctl.learned, when)


# ---------------------------------------------------------------- teach state machine
class TestTeachFlow(unittest.TestCase):
    def test_full_teach_by_gestures(self):
        s = Sim()
        s.frames("shaka", 1900)
        self.assertEqual(s.b.state, L.IDLE)                # not yet: needs teach_hold_ms
        s.frames("shaka", 200)
        self.assertEqual(s.b.state, L.COUNTDOWN)
        t_arm = s.m.lines[-1][0]
        s.frames(None, CFG["teach_countdown_ms"])
        self.assertEqual(s.b.state, L.RECORD)
        fb = [x for tt, x, r in s.m.lines if tt >= t_arm and r != "resend"]
        # armed wiggle: LED + BEEP + left/right; one nudge per second; record beep
        self.assertEqual(fb[:5], ["S", "LED teach", "BEEP 1", "D -0.35 0.35", "D 0.35 -0.35"])
        self.assertEqual([r for tt, x, r in s.m.lines if r == "teach_nudge" and x.startswith("D 0.35 0.35")],
                         ["teach_nudge", "teach_nudge"])
        self.assertIn("BEEP 2", fb)
        s.frames("rock", 2000)
        self.assertEqual(s.b.state, L.ASSIGN)
        rec = [x for _, x, r in s.m.lines if r == "teach_recorded"]
        self.assertEqual(rec.count("D -0.35 0.35"), 2)     # double wiggle
        s.frames("point", 400, "right")
        self.assertEqual(s.b.state, L.CONFIRM)
        self.assertEqual(s.b.ctl.moving, "turn_right@pose1")   # performs it once
        s.frames(None, 1500)
        self.assertEqual(s.b.state, L.IDLE)
        ev = s.events()
        self.assertEqual(ev[0], "teach start gesture")
        self.assertIn("learned pose1 turn_right gesture", ev)
        self.assertTrue(s.b.dirty)
        # now the pose drives the robot like a built-in
        s.frames("rock", 400)
        self.assertIn(("D 0.50 -0.50", "turn_right@pose1"), [(x, r) for _, x, r in s.m.lines])

    def test_assign_each_gesture(self):
        for pick, d, act in [("point", "left", "turn_left"), ("thumb", "up", "forward"),
                             ("fist", "up", "back"), ("peace", "up", "spin")]:
            s = Sim()
            s.teach("rock", pick, d)
            self.assertEqual(s.b.ctl.learned["pose1"][0], act, pick)

    def test_assign_timeout_is_wiggle(self):
        s = Sim()
        s.teach("rock", None)
        self.assertEqual(s.b.ctl.learned["pose1"][0], "wiggle")
        self.assertIn("learned pose1 wiggle timeout", s.events())

    def test_assign_pick_does_not_toggle_mode_after(self):
        s = Sim()
        s.frames("shaka", 2100)
        s.frames(None, 3000)
        s.frames("rock", 2000)
        s.frames("peace", 3000)                            # pick spin and keep holding peace
        self.assertEqual(s.b.ctl.mode, g.GESTURE_MODE)

    def test_too_similar_to_builtin(self):
        s = Sim()
        s.frames("shaka", 2100)
        s.frames(None, 3000)
        s.frames("fist", 2000)
        self.assertIn("reject too_similar fist", s.events())
        no = [x for _, x, r in s.m.lines if r == "teach_no"]
        self.assertEqual(no, ["LED no", "BEEP 3", "D -0.35 0.35", "D 0.35 -0.35", "D -0.35 0.35"])
        s.frames(None, 1000)
        self.assertEqual(s.b.state, L.IDLE)
        self.assertFalse(s.b.ctl.learned)

    def test_too_similar_to_teach_sign(self):
        s = Sim()
        s.frames("shaka", 2100)
        s.frames(None, 3000)
        s.frames("shaka", 2000)
        self.assertIn("reject too_similar teach", s.events())

    def test_too_similar_to_learned(self):
        s = Sim()
        s.teach("rock", "peace")
        s.events()
        s.frames("shaka", 2100)
        s.frames(None, 3000)
        s.frames("rock", 2000)
        self.assertIn("reject too_similar pose1", s.events())
        self.assertEqual(list(s.b.ctl.learned), ["pose1"])

    def test_second_pose(self):
        s = Sim()
        s.teach("rock", "peace")
        s.teach("three", "thumb")
        self.assertEqual({k: v[0] for k, v in s.b.ctl.learned.items()},
                         {"pose1": "spin", "pose2": "forward"})

    def test_record_timeout_no_hand(self):
        s = Sim()
        s.frames("shaka", 2100)
        s.frames(None, 3000 + CFG["learn_record_timeout_ms"] + 100)
        self.assertIn("reject no_hand", s.events())

    def test_forget_by_holding(self):
        s = Sim()
        s.teach("rock", "peace")
        s.events()
        s.frames("shaka", CFG["teach_hold_ms"] + CFG["teach_countdown_ms"] + 300)
        ev = s.events()
        self.assertIn("forget 1 gesture", ev)
        self.assertFalse(s.b.ctl.learned)
        self.assertIn("BEEP 4", [x for _, x, r in s.m.lines if r == "teach_forget"])

    def test_brief_flicker_does_not_break_hold(self):
        s = Sim()
        s.frames("shaka", 1000)
        s.frames("fist", 60)                               # 2 frames of flicker
        s.frames("shaka", 1100)
        self.assertEqual(s.b.state, L.COUNTDOWN)

    def test_cooldown_after_teach(self):
        s = Sim()
        s.frames("shaka", 2100)
        s.frames("palm", 150)                              # cancel
        s.frames("shaka", 1000)
        self.assertEqual(s.b.state, L.IDLE)

    def test_buttons(self):
        s = Sim()
        s.button("teach")
        self.assertEqual(s.b.state, L.COUNTDOWN)
        s.frames("shaka", 3100)                            # holding the sign is fine after a button
        self.assertEqual(s.b.state, L.RECORD)              # (button start never means forget)
        s.frames("rock", 2000)
        s.frames("thumb", 400)
        s.frames(None, 1500)
        self.assertEqual(s.b.ctl.learned["pose1"][0], "forward")
        s.button("forget")
        self.assertFalse(s.b.ctl.learned)
        self.assertIn("forget 1 button", s.events())
        s.frames(None, 1000)                               # forget motion plays out
        s.button("teach")
        s.button("teach")                                  # second press cancels
        self.assertEqual(s.b.state, L.IDLE)

    def test_teach_from_follow_mode(self):
        s = Sim()
        s.frames("peace", 400)
        self.assertEqual(s.b.ctl.mode, g.FOLLOW_MODE)
        s.frames("shaka", 2200)                            # holds still while the sign is up
        self.assertEqual(s.b.state, L.COUNTDOWN)
        self.assertEqual(s.b.ctl.mode, g.GESTURE_MODE)
        self.assertNotIn("follow", [r for tt, _, r in s.m.lines if tt > 900 and tt < 2600 and _ != "D 0.00 0.00"])

    def test_led_beep_optional(self):
        cfg = g.load_config(overrides={"feedback_led_beep": 0})
        s = Sim(cfg)
        s.frames("shaka", 2100)
        s.frames(None, 3200)
        self.assertFalse([x for x in s.m.texts() if x.startswith(("LED", "BEEP"))])

    def test_save_load_reboot(self):
        s = Sim()
        s.teach("rock", "point", "left")
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "learned.json")
            s.b.save(p)
            self.assertFalse(s.b.dirty)
            b2 = L.Brain(CFG)
            self.assertEqual(b2.load(p), 1)
            self.assertEqual(b2.ctl.learned["pose1"][0], "turn_left")
            lm = hand("rock", rng=random.Random(1))
            self.assertEqual(b2.classify(lm, h.ASPECT)[1], "pose1")
            self.assertEqual(L.Brain(CFG).load(os.path.join(d, "missing.json")), 0)

    def test_custom_action_from_json(self):
        b = L.Brain(CFG)
        b.learner.add("pose1", "custom", feats("rock", 40), custom=[("D", 0.3, 0.3, 200.0),
                                                                     ("D", -0.3, 0.3, 100.0)])
        b._sync()
        d = json.loads(json.dumps(b.learner.to_json()))
        b2 = L.Brain(CFG)
        b2.load_json(d)
        cmds = []
        t = 0.0
        for i in range(8):
            t += 33
            cmds += b2.on_frame(t, hand("rock", rng=random.Random(i)), h.ASPECT)
        for k in range(30):
            cmds += b2.tick(t + 20 * k)
        texts = [c.text for c in cmds if c.reason == "custom@pose1"]
        self.assertEqual(texts, ["D 0.30 0.30", "D -0.30 0.30"])


if __name__ == "__main__":
    unittest.main()
