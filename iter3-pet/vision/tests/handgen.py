"""Synthetic MediaPipe-style hand landmarks (21 points, normalized x/y) for tests.

make_hand("point", direction="left", cx=0.3, cy=0.5, size=0.2) returns what
MediaPipe would report for that pose: x in [0,1] of image width, y in [0,1] of
image height, z = 0. `size` = wrist->middle-knuckle length in image heights.
"""
from __future__ import annotations

import math
import random

ASPECT = 640 / 480

# Canonical upright right hand, palm facing camera, units of palm length,
# y grows DOWN (image convention), wrist at the origin, fingers point "up" (-y).
_MCP = {"index": (-0.35, -0.95), "middle": (0.0, -1.00),
        "ring": (0.15, -0.95), "pinky": (0.38, -0.85)}
_LEN = {"index": 1.0, "middle": 1.05, "ring": 0.95, "pinky": 0.8}
_ORDER = ["index", "middle", "ring", "pinky"]

# which fingers are extended for each pose
_POSES = {
    "palm": {"index", "middle", "ring", "pinky"},
    "point": {"index"},
    "peace": {"index", "middle"},
    "fist": set(),
    "thumb": set(),
    # not built-in gestures (rules say "unknown" / "teach"): for teaching tests
    "shaka": {"pinky"},                 # = TEACH
    "rock": {"index", "pinky"},
    "three": {"index", "middle", "ring"},
    "ring_pinky": {"ring", "pinky"},
    "middle": {"middle"},
}


def _finger(mcp, length, extended):
    x, y = mcp
    if extended:
        return [(x, y), (x, y - 0.45 * length), (x, y - 0.75 * length), (x, y - 1.0 * length)]
    # curled: up to the PIP, then folds back down toward the palm
    return [(x, y), (x, y - 0.35 * length), (x, y - 0.2 * length), (x, y + 0.05 * length)]


def _canonical(pose: str):
    ext = _POSES[pose]
    pts = [(0.0, 0.0)]                                    # 0 wrist
    if pose in ("thumb", "shaka"):                        # thumb sticks out sideways (-x)
        pts += [(-0.25, -0.25), (-0.45, -0.45), (-0.85, -0.55), (-1.2, -0.6)]
    elif pose == "palm":                                  # thumb spread
        pts += [(-0.25, -0.25), (-0.5, -0.45), (-0.7, -0.65), (-0.85, -0.85)]
    else:                                                 # thumb tucked over the fingers
        pts += [(-0.25, -0.25), (-0.4, -0.5), (-0.3, -0.75), (-0.15, -0.85)]
    for f in _ORDER:
        pts += _finger(_MCP[f], _LEN[f], f in ext)
    return pts


_DIR_VEC = {"up": (0.0, -1.0), "down": (0.0, 1.0), "left": (-1.0, 0.0), "right": (1.0, 0.0)}


def make_hand(pose: str, direction: str = "up", cx: float = 0.5, cy: float = 0.5,
              size: float = 0.2, jitter_deg: float = 0.0, noise: float = 0.0,
              rng: random.Random | None = None, aspect: float = ASPECT):
    """pose: palm | point | peace | fist | thumb. direction: where the index finger
    (point) or the thumb (thumb) points; for others, where the fingers point."""
    rng = rng or random.Random(0)
    pts = _canonical(pose)
    if pose == "thumb":
        src = (pts[4][0] - pts[2][0], pts[4][1] - pts[2][1])   # thumb mcp -> tip
    else:
        src = (0.0, -1.0)                                       # fingers
    dst = _DIR_VEC[direction]
    theta = math.atan2(dst[1], dst[0]) - math.atan2(src[1], src[0]) + math.radians(jitter_deg)
    ct, st = math.cos(theta), math.sin(theta)
    rot = [(x * ct - y * st, x * st + y * ct) for x, y in pts]
    # centre the palm (mean of wrist + knuckles) on (cx, cy)
    mx = sum(rot[i][0] for i in (0, 5, 9, 13, 17)) / 5
    my = sum(rot[i][1] for i in (0, 5, 9, 13, 17)) / 5
    out = []
    for x, y in rot:
        X = (x - mx) * size + rng.gauss(0, noise)
        Y = (y - my) * size + rng.gauss(0, noise)
        out.append((cx + X / aspect, cy + Y, 0.0))
    return out


NOVEL = ["rock", "three", "ring_pinky", "middle"]   # rules -> "unknown"

EXPECTED = {
    ("palm", "up"): "palm", ("palm", "left"): "palm", ("palm", "right"): "palm",
    ("fist", "up"): "fist", ("fist", "left"): "fist",
    ("peace", "up"): "peace", ("peace", "left"): "peace",
    ("thumb", "up"): "thumbs_up", ("thumb", "down"): "thumbs_down",
    ("thumb", "left"): "fist", ("thumb", "right"): "fist",
    ("point", "up"): "point_up", ("point", "down"): "point_down",
    ("point", "left"): "point_left", ("point", "right"): "point_right",
    ("shaka", "up"): "teach", ("shaka", "left"): "teach", ("shaka", "down"): "teach",
}
