"""Teach flow through the real runtime (common/app.py GestureRuntime: ticker
thread, STATUS button polling, JSON persistence) with a fake tars_motor client
and synthetic hands instead of MediaPipe. Runs in real time (~15 s).

    iter3-pet/vision/.venv/bin/python iter3-pet/vision/tests/test_runtime_teach.py -v
"""
from __future__ import annotations

import os
import random
import sys
import tempfile
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "common"), HERE]

import gesture as g  # noqa: E402
import handgen as h  # noqa: E402
import learn as L  # noqa: E402
from app import GestureRuntime, now_ms  # noqa: E402

FAST = {"teach_hold_ms": 600, "teach_countdown_ms": 2000, "learn_frames": 15,
        "learn_assign_ms": 1500, "learn_cooldown_ms": 300}


class FakeMotorLink:
    """Records every line; answers like tars_motor. LED/BEEP -> ERR (unsupported),
    STATUS carries the Uno button counters the vision app polls."""

    dry = False
    last_rtt_ms = 0.1

    def __init__(self):
        self.lines: list[str] = []
        self.btn_teach = 0
        self.btn_forget = 0
        self.lock = threading.Lock()

    def send(self, line: str):
        with self.lock:
            self.lines.append(line)
        if line.split()[0] in ("LED", "BEEP"):
            return "ERR unknown command"
        if line == "STATUS":
            return (f"OK 0.00 0.00 12 -1.00 0 why=none btn_teach={self.btn_teach} "
                    f"btn_forget={self.btn_forget}")
        return "OK"

    def sent(self):
        with self.lock:
            return [x for x in self.lines if x != "STATUS" and not x.startswith("VIS")]

    def close(self):
        pass


class StubLandmarker:
    def close(self):
        pass


class TestRuntimeTeach(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "learned.json")
        self.link = FakeMotorLink()
        self.rng = random.Random(3)
        self.rt = self.make()

    def make(self):
        return GestureRuntime(g.load_config(overrides=FAST), self.link, learned_path=self.path,
                              landmarker=StubLandmarker(), log_every_s=60)

    def tearDown(self):
        self.rt.close()
        self.tmp.cleanup()

    def show(self, pose, seconds, d="up", rt=None):
        rt = rt or self.rt
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            lm = None if pose is None else h.make_hand(pose, d, cx=0.5, size=0.2, noise=0.003,
                                                       jitter_deg=self.rng.uniform(-8, 8), rng=self.rng)
            rt.process_landmarks(lm, h.ASPECT, now_ms())
            time.sleep(0.033)

    def test_teach_by_hand_then_reboot(self):
        self.show("shaka", 0.8)
        self.assertEqual(self.rt.brain.state, L.COUNTDOWN)
        self.show(None, 2.1)
        self.assertEqual(self.rt.brain.state, L.RECORD)
        self.show("rock", 1.0)
        self.assertEqual(self.rt.brain.state, L.ASSIGN)
        self.show("point", 0.5, "left")
        self.show(None, 1.2)
        self.assertEqual(self.rt.brain.state, L.IDLE)
        sent = self.link.sent()
        for want in ("LED teach", "BEEP 1", "BEEP 2", "D -0.35 0.35", "S"):
            self.assertIn(want, sent)
        self.assertIn("D -0.50 0.50", sent)                 # confirmation: turn left once
        self.assertTrue(os.path.exists(self.path))          # saved on learn
        # "reboot": a new runtime loads the pose and it drives the robot
        self.rt.close()
        self.link.lines.clear()
        self.rt = self.make()
        self.assertEqual(self.rt.brain.ctl.learned["pose1"][0], "turn_left")
        self.show("rock", 0.5)
        self.assertIn("D -0.50 0.50", self.link.sent())

    def test_uno_button_short_and_long(self):
        time.sleep(0.3)                                      # first poll = baseline
        self.link.btn_teach += 1
        time.sleep(0.5)
        self.assertEqual(self.rt.brain.state, L.COUNTDOWN)
        self.show(None, 2.1)
        self.show("three", 1.0)
        self.show(None, 1.7)                                 # assign timeout -> wiggle
        self.show(None, 2.0)
        self.assertEqual(self.rt.brain.ctl.learned["pose1"][0], "wiggle")
        self.link.btn_forget += 1
        time.sleep(0.5)
        self.assertFalse(self.rt.brain.ctl.learned)
        self.assertIn("BEEP 4", self.link.sent())

    def test_palm_cancels_and_stops(self):
        self.show("shaka", 0.8)
        n = len(self.link.sent())
        self.show("palm", 0.2)
        self.assertEqual(self.rt.brain.state, L.IDLE)
        self.assertEqual(self.link.sent()[n], "S")


if __name__ == "__main__":
    unittest.main()
