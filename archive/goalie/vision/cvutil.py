"""OpenCV helpers shared by calibrate.py and track.py: calib.json, homography,
HSV masks and blob detection."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field as dc_field
from pathlib import Path

import cv2
import numpy as np

from field import FIELD_L, FIELD_W

HERE = Path(__file__).resolve().parent
DEFAULT_CALIB = HERE / "calib.json"
EXAMPLE_CALIB = HERE / "calib.example.json"


@dataclass
class HSVRange:
    # OpenCV HSV: H 0-179, S/V 0-255. h_lo > h_hi means the range wraps through red.
    h_lo: int = 5
    h_hi: int = 25
    s_lo: int = 120
    s_hi: int = 255
    v_lo: int = 120
    v_hi: int = 255


@dataclass
class Calib:
    # image pixels of the field corners, in order: far-left, far-right, goal-right, goal-left
    # ("far" = shooter end = top of the image; "goal" = goal line = bottom)
    corners_px: list = dc_field(default_factory=lambda: [[80, 40], [560, 40], [560, 440], [80, 440]])
    field_w: float = FIELD_W
    field_l: float = FIELD_L
    ball: HSVRange = dc_field(default_factory=HSVRange)                       # orange
    paddle: HSVRange = dc_field(default_factory=lambda: HSVRange(45, 85, 80, 255, 60, 255))  # green
    ball_min_area: int = 30
    paddle_min_area: int = 40
    camera: int = 0
    width: int = 640
    height: int = 480

    def field_corners_m(self) -> np.ndarray:
        w, l = self.field_w / 2, self.field_l
        return np.array([[-w, l], [w, l], [w, 0.0], [-w, 0.0]], np.float32)

    def homography(self) -> np.ndarray:
        """pixels -> field metres"""
        return cv2.getPerspectiveTransform(np.array(self.corners_px, np.float32), self.field_corners_m())

    def save(self, path=DEFAULT_CALIB) -> None:
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path=None) -> "Calib":
        p = Path(path) if path else (DEFAULT_CALIB if DEFAULT_CALIB.exists() else EXAMPLE_CALIB)
        d = json.loads(p.read_text())
        d["ball"] = HSVRange(**d.get("ball", {}))
        d["paddle"] = HSVRange(**d.get("paddle", {}))
        return cls(**d)


def px_to_field(H: np.ndarray, u: float, v: float) -> tuple[float, float]:
    p = cv2.perspectiveTransform(np.array([[[u, v]]], np.float32), H)[0, 0]
    return float(p[0]), float(p[1])


def field_to_px(H: np.ndarray, x: float, y: float) -> tuple[int, int]:
    p = cv2.perspectiveTransform(np.array([[[x, y]]], np.float32), np.linalg.inv(H))[0, 0]
    return int(round(p[0])), int(round(p[1]))


def hsv_mask(hsv: np.ndarray, r: HSVRange) -> np.ndarray:
    lo_s, hi_s = (r.s_lo, r.v_lo), (r.s_hi, r.v_hi)
    if r.h_lo <= r.h_hi:
        m = cv2.inRange(hsv, (r.h_lo, *lo_s), (r.h_hi, *hi_s))
    else:  # wraps through 0 (reds)
        m = cv2.inRange(hsv, (r.h_lo, *lo_s), (179, *hi_s)) | cv2.inRange(hsv, (0, *lo_s), (r.h_hi, *hi_s))
    k = np.ones((3, 3), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)


def largest_blob(mask: np.ndarray, min_area: int) -> tuple[float, float, float] | None:
    """(u, v, area) of the largest contour's centroid, or None."""
    cs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cs:
        return None
    c = max(cs, key=cv2.contourArea)
    a = cv2.contourArea(c)
    if a < min_area:
        return None
    m = cv2.moments(c)
    if m["m00"] == 0:
        return None
    return m["m10"] / m["m00"], m["m01"] / m["m00"], a


def range_from_patch(hsv: np.ndarray, u: int, v: int, half: int = 4,
                     dh: int = 10, ds: int = 70, dv: int = 70) -> HSVRange:
    """HSV range around the median of a small patch (click-to-pick)."""
    h, w = hsv.shape[:2]
    patch = hsv[max(0, v - half):min(h, v + half + 1), max(0, u - half):min(w, u + half + 1)].reshape(-1, 3)
    mh, ms, mv = (int(x) for x in np.median(patch, axis=0))
    lo, hi = (mh - dh) % 180, (mh + dh) % 180
    return HSVRange(lo, hi, max(0, ms - ds), 255, max(0, mv - dv), 255)
