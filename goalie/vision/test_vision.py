"""Headless tests for the vision side (no camera, no Pi, no Anthropic API).

    .venv/bin/python -m pytest -q test_vision.py

Runs against mock_pi.py on a free localhost port. To run the server tests
against the REAL C server instead (goalie/pi, `make -C ../pi host` first):

    GOALIE_SERVER_BIN=../pi/build/goalie_motor_host .venv/bin/python -m pytest -q test_vision.py

(test_link_reconnects_after_server_restart always uses mock_pi: it restarts
the server in-process.)
"""
from __future__ import annotations

import os
import signal
import socket
import subprocess
import time

import cv2
import numpy as np
import pytest

import fake_ball
import mock_pi
import track
from cvutil import Calib, HSVRange, field_to_px, hsv_mask, range_from_patch
from goalie_link import GoalieError, GoalieLink, LinkDown, parse_status
from taunt import CANNED, Taunter


REAL_BIN = os.environ.get("GOALIE_SERVER_BIN")


@pytest.fixture
def server():
    if not REAL_BIN:
        srv, port = mock_pi.start(0)
        yield srv, port
        mock_pi.stop(srv)
        return
    port = free_port()
    proc = subprocess.Popen([REAL_BIN, "-b", "stub", "-p", str(port)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t_end = time.monotonic() + 5
    while True:
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.5).close()
            break
        except OSError:
            if time.monotonic() > t_end or proc.poll() is not None:
                proc.kill()
                raise
            time.sleep(0.05)
    yield proc, port
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# ------------------------------------------------------------------ protocol
def test_parse_status_ignores_unknown():
    d = parse_status("mode=LINUX saves=3 jit_max_us=1200 loop_hz=199.5 weird future=thing junk")
    assert d["mode"] == "LINUX" and d["saves"] == 3 and d["jit_max_us"] == 1200
    assert d["loop_hz"] == pytest.approx(199.5) and d["future"] == "thing" and "junk" not in d


def test_commands_and_status(server):
    _, port = server
    link = GoalieLink("127.0.0.1", port)
    assert link.ping() < 100
    st = link.status()
    for k in ("mode", "ctl", "saves", "goals", "target", "paddle", "duty", "wd",
              "ms_since_frame", "jit_avg_us", "jit_max_us", "loop_hz", "hits"):
        assert k in st, k
    tx, duty = link.frame(1000, (0.05, 0.4), 0.0)
    assert isinstance(tx, float) and -1 <= duty <= 1
    link.goal()
    assert link.status()["goals"] == 1
    link.human(0.05)
    time.sleep(0.03)
    st = link.status()
    assert st["ctl"] == "HUMAN" and st["target"] == pytest.approx(0.05, abs=1e-3)
    link.auto()
    assert link.status()["ctl"] == "AUTO"
    link.reset()
    assert link.status()["goals"] == 0
    with pytest.raises(GoalieError):
        link.send("BOGUS")
    with pytest.raises(GoalieError):
        link.mode("WINDOWS")
    link.close()


def test_mode_switch_changes_mode_and_jitter(server):
    _, port = server
    link = GoalieLink("127.0.0.1", port)
    assert link.mode("QNX") == "QNX"
    time.sleep(0.5)
    q = link.status()
    assert link.mode("LINUX") == "LINUX"
    time.sleep(0.5)
    l = link.status()
    assert q["mode"] == "QNX" and l["mode"] == "LINUX"
    assert l["jit_max_us"] > 15000 > q["jit_max_us"], (q, l)
    assert l["loop_hz"] < q["loop_hz"]
    link.close()


def test_watchdog_parks(server):
    _, port = server
    link = GoalieLink("127.0.0.1", port)
    for i in range(5):
        link.frame(1000 + 33 * i, (0.08, 0.3 - 0.03 * i), 0.0)
    time.sleep(0.03)                                  # let a 200 Hz tick see the frame
    assert link.status()["wd"] == 0
    time.sleep(0.4)
    st = link.status()
    assert st["wd"] == 1 and st["target"] == 0.0
    link.close()


# ------------------------------------------------------------- reconnecting
def test_link_fails_fast_without_server():
    link = GoalieLink("127.0.0.1", free_port())
    t0 = time.monotonic()
    with pytest.raises(LinkDown):
        link.frame(0, None, None)
    with pytest.raises(LinkDown):           # inside the backoff: no connect attempt at all
        link.frame(0, None, None)
    assert time.monotonic() - t0 < 0.5
    assert not link.connected


def test_link_reconnects_after_server_restart():
    port = free_port()
    srv, _ = mock_pi.start(port)
    link = GoalieLink("127.0.0.1", port)
    link.ping()
    assert link.reconnects == 1
    mock_pi.stop(srv)
    with pytest.raises(LinkDown):
        link.ping()
    srv, _ = mock_pi.start(port)
    try:
        t_end = time.monotonic() + 5
        ok = False
        while time.monotonic() < t_end and not ok:
            try:
                link.ping()
                ok = True
            except LinkDown:
                time.sleep(0.05)
        assert ok and link.reconnects == 2
    finally:
        link.close()
        mock_pi.stop(srv)


# ---------------------------------------------------------------- vision math
def test_homography_and_detection_roundtrip():
    c = Calib(corners_px=[[100, 50], [540, 60], [580, 450], [70, 440]])   # skewed camera
    H = c.homography()
    img = np.full((480, 640, 3), (95, 95, 95), np.uint8)                 # grey table
    bx, by, px = 0.06, 0.31, -0.04
    cv2.circle(img, field_to_px(H, bx, by), 9, (0, 140, 255), -1)        # orange ball
    u, v = field_to_px(H, px, 0.03)
    cv2.rectangle(img, (u - 15, v - 4), (u + 15, v + 4), (0, 255, 0), -1)  # green paddle tape
    det = track.Detector(c)
    ball, paddle, dets = det.detect(img)
    assert ball is not None and paddle is not None
    assert abs(ball[0] - bx) < 0.006 and abs(ball[1] - by) < 0.006
    assert abs(paddle - px) < 0.006
    empty = np.full((480, 640, 3), (95, 95, 95), np.uint8)
    assert det.detect(empty)[:2] == (None, None)


def test_hsv_wrap_and_pick():
    hsv = np.zeros((20, 20, 3), np.uint8)
    hsv[:] = (178, 200, 200)                  # red near the wrap
    r = range_from_patch(hsv, 10, 10)
    assert r.h_lo > r.h_hi                    # wrapped range
    assert hsv_mask(hsv, r).mean() > 250
    assert hsv_mask(hsv, HSVRange(5, 25)).mean() == 0


def test_goal_detector_debounce():
    g = track.GoalDetector()
    assert not g.update((0.0, 0.3), 0.0)
    assert g.update((0.02, -0.01), 0.5)
    assert not g.update((0.02, -0.03), 0.55)          # same goal, still in the net
    assert not g.update((0.02, -0.03), 3.0)           # not re-armed yet
    assert not g.update((0.0, 0.3), 3.1)
    assert g.update((-0.05, -0.01), 3.5)
    g2 = track.GoalDetector()
    assert not g2.update((0.2, -0.01), 0.0)           # wide of the goal


# ---------------------------------------------------------- full fake loop
def test_fake_ball_saves_against_mock(server):
    _, port = server
    s = fake_ball.run_headless("127.0.0.1", port, 6, fake_ball.FakeConfig(speed=1.0, seed=7), quiet=True)
    assert s["ok"] and s["shots"] == 6 and s["link_errors"] == 0
    assert s["saves"] >= 4, s
    assert s["server_saves"] == s["saves"]           # server's vision-based count agrees with truth
    assert s["server_goals"] == s["goals"]


def test_fake_ball_goals_on_misses(server):
    _, port = server
    # a sluggish paddle can't get there: goals happen and GOAL reaches the server
    cfg = fake_ball.FakeConfig(speed=1.5, seed=11, paddle_vmax=0.12)
    s = fake_ball.run_headless("127.0.0.1", port, 6, cfg, quiet=True)
    assert s["goals"] >= 2, s
    assert s["server_goals"] == s["goals"]
    assert s["server_saves"] == s["saves"]


def test_track_fake_headless_smoke(server, tmp_path):
    _, port = server
    out = tmp_path / "overlay.png"
    rc = track.main(["--fake", "--host", "127.0.0.1", "--port", str(port), "--headless", "--no-taunt",
                     "--frames", "60", "--out", str(out), "--seed", "1", "--mode", "LINUX"])
    assert rc == 0 and out.exists()
    img = cv2.imread(str(out))
    assert img is not None and img.shape[0] > 300
    link = GoalieLink("127.0.0.1", port)
    assert link.status()["mode"] == "LINUX"
    link.close()


def test_track_no_pi(tmp_path, capsys):
    out = tmp_path / "nopi.png"
    assert track.main(["--fake", "--no-pi", "--headless", "--no-taunt", "--frames", "40",
                       "--out", str(out)]) == 0
    assert "[no-pi] -> F " in capsys.readouterr().out
    assert out.exists()


# ------------------------------------------------------------------- extras
def wait_for(pred, timeout=2.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(0.01)
    return False


def test_taunter_events_llm_and_fallback():
    said = []
    t = Taunter(speaker=said.append, min_gap_s=0.0, llm=lambda ev, s, g: f"LLM {ev} {s}-{g}")
    assert t.update(0, 0) is None                     # baseline
    assert t.update(1, 0) == "save"
    assert wait_for(lambda: said == ["LLM save 1-0"])
    assert t.update(1, 1) == "goal"
    assert wait_for(lambda: len(said) == 2)
    assert t.update(0, 0) is None                     # RESET re-baselines silently

    def boom(*_):
        raise ConnectionError("offline")
    said2 = []
    t2 = Taunter(speaker=said2.append, min_gap_s=0.0, llm=boom)
    t2.update(0, 0)
    t2.update(0, 1)
    assert wait_for(lambda: len(said2) == 1) and said2[0] in CANNED["goal"]


def test_taunter_rate_limit_and_mute():
    said = []
    t = Taunter(speaker=said.append, min_gap_s=10.0, use_llm=False)
    t.update(0, 0)
    t.update(1, 0)
    assert wait_for(lambda: len(said) == 1)
    t.update(2, 0)
    time.sleep(0.1)
    assert len(said) == 1                             # rate-limited
    t.enabled = False
    assert t.update(3, 0) == "save"                   # still detected, just not spoken


def test_switch_input_degrades_without_controller():
    from switch_input import SwitchInput
    sw = SwitchInput()
    sw.poll()                                         # must not raise either way
    assert sw.toggled is False
    if not sw.available:
        assert sw.x == 0.0
