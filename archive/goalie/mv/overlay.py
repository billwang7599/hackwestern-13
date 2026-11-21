"""Drawing shared by live.py (camera + --video replay) and synth.py, so labels
look identical everywhere.  Stages:
  detect  : circle on the ball, "(u,v) d=..px conf circ", HSV mask inset
  track   : + per-ball label (SPEED / HEIGHT / DIST / DIR) and velocity arrow
  predict : + goal-face view (X vs height) with crossing point, 1-sigma
            ellipse and time to cross
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from config import Config
from geometry import project_sphere
from tracker import Labels, TrackState

STAGES = ("detect", "track", "predict")
FONT = cv2.FONT_HERSHEY_SIMPLEX


def _txt(img, s, org, scale=0.45, col=(255, 255, 255), th=1):
    # outline via 1px offsets (thicker Hershey text has a wider advance in
    # OpenCV 5, so a thick black underlay would not line up)
    x, y = org
    for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (1, 1)):
        cv2.putText(img, s, (x + dx, y + dy), FONT, scale, (0, 0, 0), th, cv2.LINE_AA)
    cv2.putText(img, s, org, FONT, scale, col, th, cv2.LINE_AA)


def fmt(v, spec, unit=""):
    return "--" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:{spec}}{unit}"


def label_lines(L: Labels) -> list[str]:
    kmh = None if L.speed is None else L.speed * 3.6
    return [
        f"SPEED  {fmt(L.speed, '.2f', ' m/s')} ({fmt(kmh, '.1f', ' km/h')})",
        f"HEIGHT {fmt(L.height, '.2f', ' m')} ({fmt(None if L.height is None else L.height * 100, '.0f', ' cm')})",
        f"DIST   {fmt(L.dist, '.2f', ' m')}",
        f"DIR    az {fmt(L.heading, '+.0f', 'deg')}  el {fmt(L.elev, '+.0f', 'deg')}",
    ]


def draw_detection(img, st: TrackState, mask=None):
    d = st.det
    if d is not None:
        c = (int(round(d.u)), int(round(d.v)))
        cv2.circle(img, c, max(2, int(round(d.diam / 2))), (0, 255, 0), 1, cv2.LINE_AA)
        if d.streak_len > 1:
            a = math.radians(d.streak_angle)
            dx, dy = 0.5 * d.streak_len * math.cos(a), 0.5 * d.streak_len * math.sin(a)
            cv2.line(img, (int(d.u - dx), int(d.v - dy)), (int(d.u + dx), int(d.v + dy)),
                     (0, 200, 255), 1, cv2.LINE_AA)
        _txt(img, f"({d.u:.1f},{d.v:.1f}) d={d.diam:.1f}px", (c[0] + 10, c[1] - 22), 0.4, (0, 255, 0))
        _txt(img, f"conf {d.conf:.2f} circ {d.circularity:.2f} blur {d.streak_len:.0f}px",
             (c[0] + 10, c[1] - 8), 0.4, (0, 255, 0))
    if mask is not None:
        h, w = img.shape[:2]
        iw, ih = w // 4, h // 4
        small = cv2.resize(mask, (iw, ih), interpolation=cv2.INTER_NEAREST)
        img[h - ih - 4:h - 4, w - iw - 4:w - 4] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(img, (w - iw - 5, h - ih - 5), (w - 4, h - 4), (200, 200, 200), 1)
        _txt(img, "HSV mask", (w - iw, h - ih - 8), 0.4)


def draw_labels(img, st: TrackState, cfg: Config, gt: Labels | None = None, avoid=None):
    L = st.labels
    d = st.det
    if d is None:
        return
    x0, y0 = int(d.u) + 14, int(d.v) + 8
    h, w = img.shape[:2]
    lines = label_lines(L)
    if x0 > w - 230:
        x0 = int(d.u) - 240
    if avoid is not None and x0 < avoid[0] and y0 - 12 < avoid[1]:
        y0 = avoid[1] + 14          # keep clear of the goal-face inset
    n_lines = 4 + (4 if gt is not None else 0)
    y0 = min(y0, h - 40 - 15 * n_lines)
    for i, s in enumerate(lines):
        _txt(img, s, (x0, min(h - 5, y0 + 15 * i)), 0.42, (255, 255, 0))
    if gt is not None:
        for i, s in enumerate(label_lines(gt)):
            _txt(img, "GT " + s, (x0, min(h - 5, y0 + 15 * (i + 4) + 4)), 0.38, (180, 180, 180))
    # velocity arrow: project p and p + v*0.1s
    if L.vel is not None and L.pos is not None:
        try:
            r = cfg.ball_diam / 2
            u1, v1, _ = project_sphere(cfg.camera, L.pos, r)
            u2, v2, _ = project_sphere(cfg.camera, L.pos + L.vel * 0.1, r)
            if all(map(math.isfinite, (u1, v1, u2, v2))):
                cv2.arrowedLine(img, (int(d.u), int(d.v)),
                                (int(d.u + u2 - u1), int(d.v + v2 - v1)), (255, 0, 255), 2,
                                cv2.LINE_AA, tipLength=0.25)
        except (ValueError, ZeroDivisionError):
            pass


def draw_track(img, st: TrackState, cfg: Config):
    r = cfg.ball_diam / 2
    pts = []
    for p in st.track:
        try:
            u, v, _ = project_sphere(cfg.camera, p, r)
            pts.append((int(u), int(v)))
        except (ValueError, ZeroDivisionError):
            pass
    for a, b in zip(pts, pts[1:]):
        cv2.line(img, a, b, (0, 160, 255), 1, cv2.LINE_AA)


def draw_goal_face(img, st: TrackState, cfg: Config, gt_cross=None, size=(220, 150)):
    """Schematic goal seen from the shooter's side... drawn from the camera's
    side (X right = image right).  Crossing point + 1-sigma ellipse."""
    W, H = size
    pad = 14
    gw, gh = cfg.goal_w, cfg.goal_h
    sx = (W - 2 * pad) / (gw * 1.5)
    sy = (H - 2 * pad - 18) / (gh * 1.3)
    s = min(sx, sy)
    panel = np.full((H, W, 3), 30, np.uint8)
    ox, oy = W // 2, H - pad

    def P(x, y):
        return int(ox + x * s), int(oy - y * s)

    cv2.line(panel, (0, oy), (W, oy), (90, 90, 90), 1)
    cv2.rectangle(panel, P(-gw / 2, gh), P(gw / 2, 0), (255, 255, 255), 2)
    c = st.cross
    ok = c.coming and math.isfinite(c.x)
    if gt_cross is not None:
        cv2.drawMarker(panel, P(*gt_cross), (180, 180, 180), cv2.MARKER_CROSS, 10, 1)
    if ok:
        col = (0, 255, 0) if abs(c.x) <= gw / 2 and c.y <= gh else (0, 0, 255)
        if c.cov_xy is not None:
            ev, evec = np.linalg.eigh(c.cov_xy)
            ev = np.sqrt(np.maximum(ev, 0))
            ang = math.degrees(math.atan2(evec[1, 1], evec[0, 1]))
            ax = (max(1, int(ev[1] * s)), max(1, int(ev[0] * s)))
            if max(ax) < 2000:
                cv2.ellipse(panel, P(c.x, c.y), ax, -ang, 0, 360, col, 1, cv2.LINE_AA)
        cv2.circle(panel, P(c.x, c.y), max(2, int(cfg.ball_diam / 2 * s)), col, -1, cv2.LINE_AA)
        tt = (c.t_cross - st.t) * 1000
        _txt(panel, f"x {c.x:+.2f} h {c.y:.2f} m  {tt:.0f}ms", (4, 14), 0.4, col)
        _txt(panel, f"conf {c.conf:.2f}{' bounce' if c.bounced else ''}", (4, 28), 0.38)
    else:
        _txt(panel, "no shot on goal", (4, 14), 0.4, (160, 160, 160))
    h, w = img.shape[:2]
    if h >= H + 4 and w >= W + 4:
        img[4:4 + H, 4:4 + W] = panel


def draw_hud(img, lines):
    h = img.shape[0]
    for i, s in enumerate(lines):
        _txt(img, s, (6, h - 8 - 15 * (len(lines) - 1 - i)), 0.42, (200, 255, 200))


def render(img, st: TrackState, cfg: Config, stage="predict", mask=None, hud=(),
           gt: Labels | None = None, gt_cross=None):
    out = img.copy()
    draw_detection(out, st, mask)
    if stage in ("track", "predict"):
        draw_track(out, st, cfg)
        draw_labels(out, st, cfg, gt, avoid=(228, 158) if stage == "predict" else None)
    if stage == "predict":
        draw_goal_face(out, st, cfg, gt_cross)
    draw_hud(out, list(hud))
    return out
