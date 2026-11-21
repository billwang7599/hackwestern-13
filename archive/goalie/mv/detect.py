"""Ball detection: HSV colour mask (+ optional motion mask) -> best blob ->
sub-pixel centre and apparent diameter, robust to motion blur.

Pipeline
  1. HSV inRange (hue may wrap) -> binary seed mask; small open/close.
  2. Static rejection (default on): an EMA of the colour mask; pixels that
     have been "ball coloured" for ~10+ frames are static clutter (an orange
     chair) and are removed.  A ball held perfectly still also fades out, so
     live.py turns this off in --stage detect.
     Optional MOG2 background subtraction, AND-ed with the colour mask.
  3. Connected components >= min area.  Each gets a score from fill ratio
     (area / fitted-ellipse area; ~1 for a disc *and* for a blur streak),
     circularity, distance to the predicted pixel position (gating) and size
     consistency.  The best one is kept.
  4. Sub-pixel measurement in a ROI around it:
       weight w = linear "coverage" of ball colour vs. local background
                  (ring median around the blob, or the MOG2 background), in [0,1]
     A disc of radius R swept along a line has, perpendicular to the sweep,
     exactly the disc's own marginal, so the weighted covariance's SMALL
     eigenvalue is R^2/4 whatever the blur.  diameter = 4 sqrt(lambda_min).
     Along the sweep  lambda_max = R^2/4 + L^2/12  -> streak length L.
     cv2.fitEllipse's minor axis is also reported (size_method="ellipse").
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from config import DetectCfg


@dataclass
class Detection:
    u: float
    v: float
    diam: float            # chosen apparent diameter, px
    diam_moments: float
    diam_ellipse: float
    streak_len: float      # px, motion-blur length (0 = sharp)
    streak_angle: float    # deg, image-plane direction of the streak
    area: float
    circularity: float     # 4 pi A / P^2 of the binary blob
    fill: float            # area / ellipse area
    conf: float            # 0..1 blob confidence
    clipped: bool = False  # blob touches the image border: size unreliable


def hsv_mask(hsv: np.ndarray, lo, hi) -> np.ndarray:
    lo, hi = [int(x) for x in lo], [int(x) for x in hi]
    if lo[0] <= hi[0]:
        return cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    a = cv2.inRange(hsv, np.array([lo[0], lo[1], lo[2]], np.uint8), np.array([179, hi[1], hi[2]], np.uint8))
    b = cv2.inRange(hsv, np.array([0, lo[1], lo[2]], np.uint8), np.array([hi[0], hi[1], hi[2]], np.uint8))
    return a | b


class BallDetector:
    def __init__(self, cfg: DetectCfg | None = None):
        self.cfg = cfg or DetectCfg()
        self.bg = cv2.createBackgroundSubtractorMOG2(history=200, varThreshold=25,
                                                     detectShadows=False) if self.cfg.use_bg else None
        self.static = None     # EMA of the colour mask (static clutter)
        self.mask = None       # last colour(&motion) mask, for the UI inset
        self.ball_bgr = None   # running estimate of the ball colour

    def detect(self, frame: np.ndarray, pred_uv=None, pred_diam=None) -> Detection | None:
        c = self.cfg
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = hsv_mask(hsv, c.hsv_lo, c.hsv_hi)
        bg_img = None
        if c.static_reject:
            m01 = (mask > 0).astype(np.float32)
            if self.static is None:
                self.static = np.zeros_like(m01)
            static = cv2.dilate((self.static > 0.6).astype(np.uint8), np.ones((5, 5), np.uint8))
            self.static = 0.9 * self.static + 0.1 * m01
            mask[static > 0] = 0
        if self.bg is not None:
            fg = self.bg.apply(frame)
            fg = cv2.dilate(fg, np.ones((5, 5), np.uint8))
            mask &= fg
            bg_img = self.bg.getBackgroundImage()
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        self.mask = mask
        min_area = math.pi * (c.min_diam_px / 2) ** 2 * 0.6
        n, lab, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
        best, best_score = None, -1e9
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area < min_area:
                continue
            sub = (lab[y:y + h, x:x + w] == i).astype(np.uint8)
            cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not cnts:
                continue
            cnt = max(cnts, key=cv2.contourArea)
            per = cv2.arcLength(cnt, True)
            circ = 4 * math.pi * area / max(per * per, 1e-9)
            if len(cnt) >= 5:
                (_, _), (ea, eb), _ = cv2.fitEllipse(cnt)
                fill = area / max(math.pi * ea * eb / 4, 1e-9)
            else:
                fill = 1.0
            cu, cv_ = cents[i]
            score = -2.0 * abs(1 - min(fill, 1.6)) + 0.5 * min(circ, 1.0) + 0.1 * math.log(area)
            if pred_uv is not None:
                d = math.hypot(cu - pred_uv[0], cv_ - pred_uv[1])
                if d > c.gate_px:
                    continue
                score -= (d / c.gate_px) ** 2
            if pred_diam:
                score -= 0.5 * abs(math.log(max(math.sqrt(area / math.pi) * 2, 1) / pred_diam))
            if score > best_score:
                best, best_score = (i, x, y, w, h, area, circ, fill), score
        if best is None:
            return None
        i, x, y, w, h, area, circ, fill = best
        return self._measure(frame, lab == i, x, y, w, h, area, circ, fill, bg_img, best_score)

    # ------------------------------------------------------------------
    def _measure(self, frame, blobmask, x, y, w, h, area, circ, fill, bg_img, score):
        H, W = frame.shape[:2]
        pad = max(4, int(0.35 * max(w, h)))
        x0, y0 = max(0, x - pad), max(0, y - pad)
        x1, y1 = min(W, x + w + pad), min(H, y + h + pad)
        roi = frame[y0:y1, x0:x1].astype(np.float32)
        bm = blobmask[y0:y1, x0:x1].astype(np.uint8)
        k = max(3, int(round(0.25 * min(w, h))) | 1)
        near = cv2.dilate(bm, np.ones((k + 2, k + 2), np.uint8))
        ring = cv2.dilate(near, np.ones((5, 5), np.uint8)) & (1 - near)
        core = cv2.erode(bm, np.ones((3, 3), np.uint8))
        if core.sum() < 3:
            core = bm
        ball = np.median(roi[core > 0], axis=0)
        self.ball_bgr = ball
        if bg_img is not None:
            bgc = bg_img[y0:y1, x0:x1].astype(np.float32)
        else:
            bgc = np.median(roi[ring > 0], axis=0) if ring.sum() > 5 else np.full(3, 110, np.float32)
            bgc = np.broadcast_to(bgc, roi.shape)
        dcol = ball - bgc
        den = np.maximum((dcol * dcol).sum(-1), 400.0)
        wgt = np.clip(((roi - bgc) * dcol).sum(-1) / den, 0, 1)
        wgt *= near
        wgt[wgt < 0.08] = 0
        m = float(wgt.sum())
        yy, xx = np.mgrid[y0:y1, x0:x1].astype(np.float64)
        if m < 1:
            u, v = x + w / 2, y + h / 2
            lmin = lmax = (w / 4) ** 2
            ang = 0.0
        else:
            u = float((wgt * xx).sum() / m)
            v = float((wgt * yy).sum() / m)
            dx, dy = xx - u, yy - v
            cxx = float((wgt * dx * dx).sum() / m)
            cyy = float((wgt * dy * dy).sum() / m)
            cxy = float((wgt * dx * dy).sum() / m)
            tr, det = cxx + cyy, cxx * cyy - cxy * cxy
            disc = math.sqrt(max(tr * tr / 4 - det, 0))
            lmax, lmin = tr / 2 + disc, max(tr / 2 - disc, 1e-6)
            ang = 0.5 * math.degrees(math.atan2(2 * cxy, cxx - cyy))
        d_mom = 4 * math.sqrt(lmin)
        streak = math.sqrt(max(0.0, 12 * (lmax - lmin)))
        cnts, _ = cv2.findContours(bm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnt = max(cnts, key=cv2.contourArea)
        d_ell = min(cv2.fitEllipse(cnt)[1]) if len(cnt) >= 5 else 2 * math.sqrt(area / math.pi)
        diam = d_ell if self.cfg.size_method == "ellipse" else d_mom
        conf = float(np.clip(0.5 * min(circ, 1.0) + 0.5 * (1 - abs(1 - min(fill, 2.0))), 0, 1))
        if streak > 0.5 * diam:   # streaks are legitimately non-circular
            conf = max(conf, float(np.clip(1 - abs(1 - fill), 0, 1)))
        clipped = x <= 0 or y <= 0 or x + w >= W or y + h >= H
        if clipped:
            conf *= 0.3
        return Detection(u, v, diam, d_mom, d_ell, streak, ang, float(area), float(circ),
                         float(fill), conf, clipped)
