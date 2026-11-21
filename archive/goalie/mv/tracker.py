"""Per-frame pipeline: frame -> Detection -> 3D point -> track -> fit -> labels
and crossing prediction.  Shared by live.py, replay and synth.py so all three
show identical numbers.

Labels (stage "track"), from a ballistic fit over the last LABEL_WIN points
(that fit IS the smoothing for the fast-changing HEIGHT / DIST / elevation:
its current state has no lag).  SPEED and azimuth, which barely change in
flight, are additionally median-smoothed over the last SMOOTH frames:
  speed  |v| m/s                 (needs >= 3 detections, else None -> "--")
  height Y of the ball centre above the floor, m
  dist   Z from the goal plane, m
  heading (azimuth) = atan2(vx, -vz) deg: 0 = travelling straight at the goal
          plane (perpendicular to it), + = drifting to the right (+X)
  elev   = atan2(vy, hypot(vx, vz)) deg: + rising, - falling
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from config import Config
from detect import BallDetector, Detection
from geometry import Meas3D, backproject, ground_point, project_sphere
from trajectory import BallisticFitter, ConstAccelKF, Crossing, Fit

LABEL_WIN = 6
SMOOTH = 5


@dataclass
class Labels:
    speed: float | None = None
    height: float | None = None
    dist: float | None = None
    heading: float | None = None
    elev: float | None = None
    vel: np.ndarray | None = None   # world velocity (for the arrow)
    pos: np.ndarray | None = None


@dataclass
class TrackState:
    t: float
    det: Detection | None = None
    meas: Meas3D | None = None
    fit: Fit | None = None
    cross: Crossing = field(default_factory=lambda: Crossing(False))
    kf_cross: Crossing = field(default_factory=lambda: Crossing(False))
    labels: Labels = field(default_factory=Labels)
    track: list = field(default_factory=list)   # recent world points
    n_track: int = 0


def labels_from(p, v) -> Labels:
    sp = float(np.linalg.norm(v))
    head = math.degrees(math.atan2(v[0], -v[2])) if sp > 0.05 else None
    elev = math.degrees(math.atan2(v[1], math.hypot(v[0], v[2]))) if sp > 0.05 else None
    return Labels(sp, float(p[1]), float(p[2]), head, elev, np.asarray(v), np.asarray(p))


class Tracker:
    def __init__(self, cfg: Config, max_points=40, max_age=1.5, gap_reset=0.35,
                 use_kf=True):
        self.cfg = cfg
        self.det = BallDetector(cfg.detect)
        r = cfg.ball_diam / 2
        self.fitter = BallisticFitter(drag_k=cfg.drag_k, rolling=cfg.rolling, r_ball=r)
        self.lab_fitter = BallisticFitter(drag_k=cfg.drag_k, rolling=cfg.rolling, r_ball=r)
        self.kf = ConstAccelKF() if use_kf else None
        self.pts: deque[Meas3D] = deque(maxlen=max_points)
        self.max_age, self.gap_reset = max_age, gap_reset
        self.fit: Fit | None = None
        self.last_cross = Crossing(False)
        self.hist = deque(maxlen=SMOOTH)

    def clear(self):
        self.pts.clear()
        self.fit = None
        self.last_cross = Crossing(False)
        self.hist.clear()
        if self.kf:
            self.kf.reset()

    def _predict_px(self, t):
        if self.fit is None or t - self.fit.t_ref > 0.3:
            if self.pts and t - self.pts[-1].t < 0.3:
                p = self.pts[-1].p
            else:
                return None, None
        else:
            p, _ = self.fit.state_at(t - self.fit.t_ref)
        try:
            u, v, d = project_sphere(self.cfg.camera, p, self.cfg.ball_diam / 2)
        except (ValueError, ZeroDivisionError):
            return None, None
        return (u, v), d

    def step(self, frame, t: float) -> TrackState:
        cfg = self.cfg
        pred_uv, pred_d = self._predict_px(t)
        det = self.det.detect(frame, pred_uv, pred_d)
        if det is None and pred_uv is not None:      # lost the gate: try globally
            det = self.det.detect(frame, None, None)
        st = TrackState(t, det)
        # A ball that leaves the frame (lob over the top, behind the shooter's
        # leg) keeps flying ballistically, so the fit and its prediction are
        # kept across gaps; the track is only reset when a NEW detection is
        # inconsistent with it, or the prediction has expired.
        if self.last_cross.coming and t > self.last_cross.t_cross + 0.1:
            self.clear()
        elif self.pts and self.fit is None and t - self.pts[-1].t > self.gap_reset:
            self.clear()
        elif self.pts and t - self.pts[-1].t > self.max_age:
            self.clear()
        while len(self.pts) > 3 and t - self.pts[0].t > self.max_age:
            self.pts.popleft()
        if det is not None and det.diam >= cfg.detect.min_diam_px and not det.clipped:
            dc = cfg.detect
            if cfg.rolling:
                m = ground_point(cfg.camera, det.u, det.v, cfg.ball_diam, t, dc.sigma_center_px)
            else:
                m = backproject(cfg.camera, det.u, det.v, det.diam, cfg.ball_diam, t,
                                dc.sigma_center_px, dc.sigma_diam_px)
            if m is not None:
                # new shot? a point wildly inconsistent with the current fit
                if self.fit is not None and len(self.pts) >= 5:
                    pp, _ = self.fit.state_at(t - self.fit.t_ref)
                    d = m.p - pp
                    S = m.cov + self.fit.cov[:3, :3] + 1e-6 * np.eye(3)
                    if d @ np.linalg.solve(S, d) > 400:
                        self.clear()
                self.pts.append(m)
                st.meas = m
                if self.kf is not None:
                    self.kf.update(t, m.p, m.cov)
        n = len(self.pts)
        if cfg.drag_quad > 0 and self.fit is not None and not cfg.rolling:
            k = cfg.drag_quad * float(np.linalg.norm(self.fit.v))
            self.fitter.k = self.lab_fitter.k = k
        if st.meas is not None and n >= 3:
            ts = [q.t for q in self.pts]
            P = [q.p for q in self.pts]
            Cs = [q.cov for q in self.pts]
            self.fit = self.fitter.fit(ts, P, Cs)
            self.last_cross = self.fitter.crossing(self.fit)
            lw = list(self.pts)[-LABEL_WIN:]
            lf = self.lab_fitter.fit([q.t for q in lw], [q.p for q in lw], [q.cov for q in lw])
            if lf is not None:
                self.hist.append(labels_from(lf.p, lf.v))
        elif n < 3:
            self.fit = None
            self.last_cross = Crossing(False)
        st.fit = self.fit
        st.cross = self.last_cross
        if self.kf is not None:
            st.kf_cross = self.kf.crossing()
        st.labels = self._smoothed(st)
        st.track = [q.p for q in self.pts]
        st.n_track = n
        return st

    def _smoothed(self, st: TrackState) -> Labels:
        if st.meas is None:
            return Labels()
        if not self.hist:
            # < 3 detections: position only
            return Labels(None, float(st.meas.p[1]), float(st.meas.p[2]), None, None, None, st.meas.p)
        L = Labels()

        def med(attr):
            vals = [getattr(h, attr) for h in self.hist if getattr(h, attr) is not None]
            return float(np.median(vals)) if vals else None

        last = self.hist[-1]
        L.speed, L.heading = med("speed"), med("heading")
        L.height, L.dist, L.elev = last.height, last.dist, last.elev
        L.vel = self.hist[-1].vel
        L.pos = self.hist[-1].pos
        return L
