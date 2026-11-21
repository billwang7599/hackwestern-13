"""Scoreboard overlay for track.py (OpenCV drawing only, no windows here).

render(state, camera=None) -> BGR image:
  [ camera view with detections (if any) | top-down schematic field + scoreboard ]
The schematic shows the ball trail, the paddle, the predicted crossing x
(target_x from the Pi's F reply), the mode banner (green QNX real-time vs red
LINUX emulated jitter, with jit_max_us / loop_hz from STATUS), and watchdog /
link indicators.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field as dc_field

import cv2
import numpy as np

from field import BALL_R, FIELD_L, FIELD_W, GOAL_HALF_W

PANEL_W, PANEL_H = 420, 700
SCALE = 700.0                     # px per metre in the schematic
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN, RED, WHITE, GREY = (60, 200, 60), (50, 50, 220), (255, 255, 255), (120, 120, 120)
ORANGE, CYAN, YELLOW = (0, 140, 255), (255, 220, 0), (0, 230, 255)


@dataclass
class UIState:
    ball: tuple[float, float] | None = None
    paddle: float | None = None
    target_x: float | None = None
    duty: float = 0.0
    trail: deque = dc_field(default_factory=lambda: deque(maxlen=40))
    status: dict = dc_field(default_factory=dict)
    link_up: bool = False
    human: bool = False
    human_x: float = 0.0
    taunt_on: bool = True
    taunt_text: str = ""
    taunt_t: float = 0.0
    fps: float = 0.0
    flash: str = ""               # "SAVE!" / "GOAL!" banner
    flash_t: float = 0.0
    field_w: float = FIELD_W
    controller: str = ""

    misses: int = 0

    def push_ball(self, ball):
        self.ball = ball
        if ball is not None:
            self.misses = 0
            self.trail.append(ball)
        else:
            self.misses += 1
            if self.misses > 6:            # ball gone: start a fresh trail next shot
                self.trail.clear()

    def set_flash(self, text):
        self.flash, self.flash_t = text, time.monotonic()


def _f2p(x, y, field_w, ox, oy):
    """field metres -> schematic panel pixels (goal at the bottom)."""
    return int(ox + (x + field_w / 2) * SCALE), int(oy + (FIELD_L - y) * SCALE)


def _text(img, s, org, scale=0.6, col=WHITE, th=1):
    if scale >= 0.7:                       # outline only big text; small text stays crisp
        cv2.putText(img, s, org, FONT, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, col, th, cv2.LINE_AA)


def render_panel(s: UIState) -> np.ndarray:
    img = np.full((PANEL_H, PANEL_W, 3), 30, np.uint8)
    st = s.status
    now = time.monotonic()

    # mode banner
    mode = st.get("mode", "?")
    linux = mode == "LINUX"
    cv2.rectangle(img, (0, 0), (PANEL_W, 54), RED if linux else GREEN, -1)
    _text(img, "LINUX (emulated jitter)" if linux else "QNX real-time", (10, 24), 0.7, WHITE, 2)
    jm, hz = st.get("jit_max_us"), st.get("loop_hz")
    jt = f"jitter max {jm / 1000:.1f} ms" if isinstance(jm, (int, float)) else "jitter max ?"
    ht = f"loop {hz:.0f} Hz" if isinstance(hz, (int, float)) else "loop ? Hz"
    _text(img, f"{jt}   {ht}", (10, 46), 0.5)

    # scoreboard
    saves, goals = st.get("saves", 0), st.get("goals", 0)
    _text(img, "ROBOT", (30, 84), 0.55, CYAN)
    _text(img, "HUMANS", (PANEL_W - 120, 84), 0.55, ORANGE)
    _text(img, f"{saves}", (40, 140), 2.0, CYAN, 4)
    _text(img, ":", (PANEL_W // 2 - 10, 135), 1.6, WHITE, 3)
    _text(img, f"{goals}", (PANEL_W - 110, 140), 2.0, ORANGE, 4)
    _text(img, "saves", (40, 160), 0.45, GREY)
    _text(img, "goals", (PANEL_W - 110, 160), 0.45, GREY)

    # field
    fw = s.field_w
    ox = (PANEL_W - fw * SCALE) / 2
    oy = 180
    tl, br = _f2p(-fw / 2, FIELD_L, fw, ox, oy), _f2p(fw / 2, 0, fw, ox, oy)
    cv2.rectangle(img, tl, br, (40, 90, 40), -1)
    cv2.rectangle(img, tl, br, WHITE, 1)
    g0, g1 = _f2p(-GOAL_HALF_W, 0, fw, ox, oy), _f2p(GOAL_HALF_W, 0, fw, ox, oy)
    cv2.rectangle(img, (g0[0], g0[1]), (g1[0], g1[1] + 14), (200, 200, 200), 2)
    pts = list(s.trail)
    for i in range(1, len(pts)):
        a = _f2p(*pts[i - 1], fw, ox, oy)
        b = _f2p(*pts[i], fw, ox, oy)
        cv2.line(img, a, b, ORANGE, 1 + 2 * i // len(pts), cv2.LINE_AA)
    if s.target_x is not None:
        t = _f2p(s.target_x, 0, fw, ox, oy)
        cv2.drawMarker(img, (t[0], t[1] + 4), YELLOW, cv2.MARKER_TRIANGLE_UP, 16, 2)
        if s.ball is not None and s.ball[1] > 0:
            cv2.line(img, _f2p(*s.ball, fw, ox, oy), t, YELLOW, 1, cv2.LINE_AA)
    if s.paddle is not None:
        a = _f2p(s.paddle - 0.035, 0.03, fw, ox, oy)
        b = _f2p(s.paddle + 0.035, 0.03, fw, ox, oy)
        cv2.line(img, a, b, CYAN, 6)
    if s.human:
        h = _f2p(s.human_x, 0, fw, ox, oy)
        cv2.circle(img, (h[0], h[1] + 4), 6, (255, 0, 255), 2)
    if s.ball is not None:
        cv2.circle(img, _f2p(*s.ball, fw, ox, oy), max(3, int(BALL_R * SCALE)), ORANGE, -1, cv2.LINE_AA)

    # flash
    if s.flash and now - s.flash_t < 1.2:
        col = CYAN if s.flash.startswith("SAVE") else ORANGE
        _text(img, s.flash, (PANEL_W // 2 - 80, oy + 200), 1.5, col, 4)

    # footer: link / watchdog / control
    y = PANEL_H - 52
    up = s.link_up
    cv2.circle(img, (16, y - 5), 7, GREEN if up else RED, -1)
    _text(img, "Pi link" if up else "Pi OFFLINE", (28, y), 0.5)
    wd = st.get("wd", 0) == 1
    cv2.circle(img, (140, y - 5), 7, RED if wd else GREEN, -1)
    _text(img, "WATCHDOG PARKED" if wd else "watchdog ok", (152, y), 0.5)
    _text(img, f"{s.fps:.0f} fps", (PANEL_W - 70, y), 0.5, GREY)
    ctl = "HUMAN goalie" + (f" ({s.controller})" if s.controller else "") if s.human else "AI goalie"
    _text(img, f"{ctl}   duty {s.duty:+.2f}   taunts {'on' if s.taunt_on else 'off'}", (10, y + 22), 0.45)
    if s.taunt_text and now - s.taunt_t < 6:
        _text(img, f'"{s.taunt_text[:48]}"', (10, y + 42), 0.45, YELLOW)
    return img


def render(s: UIState, camera: np.ndarray | None = None, dets: dict | None = None) -> np.ndarray:
    panel = render_panel(s)
    if camera is None:
        return panel
    cam = camera.copy()
    for name, col in (("ball", ORANGE), ("paddle", CYAN)):
        p = (dets or {}).get(name)
        if p:
            cv2.circle(cam, (int(p[0]), int(p[1])), 12, col, 2)
    for poly in (dets or {}).get("polys", []):
        cv2.polylines(cam, [np.array(poly, np.int32)], True, (0, 255, 255), 1)
    h = PANEL_H
    cam = cv2.resize(cam, (int(cam.shape[1] * h / cam.shape[0]), h))
    return np.hstack([cam, panel])
