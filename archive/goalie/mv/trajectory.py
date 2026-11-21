"""Trajectory estimation from noisy 3D ball points, and goal-plane crossing.

Model (world frame, see config.py; gravity g on -Y):
    no drag : p(t) = p0 + v0 t + 1/2 a_g t^2,        a_g = (0, -g, 0)
    drag k  : p(t) = p0 + v0 B(t) + a_g G(t)
              B(t) = (1 - e^{-kt})/k,  G(t) = (t - B(t))/k    (-> t, t^2/2 as k->0)
Both are LINEAR in theta = (p0, v0) once g and k are fixed, so the fit is a
generalized weighted least squares with each point's full 3x3 covariance
(depth along the camera ray is the noisy direction):
    min sum_i (y_i - H_i theta)^T S_i^-1 (y_i - H_i theta),
    y_i = p_i - a_g G(t_i),  H_i = [I3, B(t_i) I3]
Times are relative to the newest sample (t_ref), so p0/v0 are the CURRENT
state.  One outlier pass: points with Mahalanobis^2 > outlier_chi2 are dropped
and the fit redone.  The parameter covariance is inflated by the reduced
chi-square when that exceeds 1.

ROLLING mode: Y is fixed at the ball radius, only (x, z, vx, vz) are fitted
(drag k doubles as rolling resistance).

Crossing: solve Z(t) = 0 (Z has no gravity term, so closed form), optional
floor bounce (restitution e) before the crossing.  Uncertainty by numerical
Jacobian of (x_cross, y_cross, t_cross) w.r.t. theta.

ConstAccelKF is the alternative: 9-state [p, v, a] Kalman filter with a prior
a ~ (0, -g, 0) and white-jerk process noise.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

G = 9.81


def _B(t, k):
    if k < 1e-6:
        return t
    return (1 - np.exp(-k * t)) / k


def _G(t, k):
    if k < 1e-6:
        return 0.5 * t * t
    return (t - _B(t, k)) / k


def _dB(t, k):  # velocity factor
    return np.exp(-k * t) if k >= 1e-6 else np.ones_like(t) if isinstance(t, np.ndarray) else 1.0


@dataclass
class Fit:
    t_ref: float
    p: np.ndarray        # position at t_ref
    v: np.ndarray        # velocity at t_ref
    cov: np.ndarray      # 6x6 (p, v)
    n: int
    rchi2: float
    k: float
    g: float
    rolling: bool
    r_ball: float

    def state_at(self, dt: float, theta=None):
        th = self.theta if theta is None else theta
        p0, v0 = th[:3], th[3:]
        ag = np.array([0.0, -self.g, 0.0]) if not self.rolling else np.zeros(3)
        p = p0 + v0 * _B(dt, self.k) + ag * _G(dt, self.k)
        if self.k >= 1e-6:
            v = v0 * math.exp(-self.k * dt) + ag * (1 - math.exp(-self.k * dt)) / self.k
        else:
            v = v0 + ag * dt
        return p, v

    @property
    def theta(self):
        return np.concatenate([self.p, self.v])


@dataclass
class Crossing:
    coming: bool
    t_cross: float = float("nan")    # absolute time of crossing Z = 0
    x: float = float("nan")
    y: float = float("nan")          # ball-centre height at the crossing
    cov_xy: np.ndarray | None = None
    sigma_t: float = float("nan")
    conf: float = 0.0
    bounced: bool = False


class BallisticFitter:
    def __init__(self, g=G, drag_k=0.0, rolling=False, r_ball=0.0335,
                 outlier_chi2=16.0, restitution=0.75, min_pts=5, horizon=1.5):
        self.g, self.k, self.rolling, self.r = g, drag_k, rolling, r_ball
        self.outlier_chi2, self.e = outlier_chi2, restitution
        self.min_pts, self.horizon = min_pts, horizon

    def fit(self, ts, P, covs) -> Fit | None:
        ts = np.asarray(ts, float)
        P = np.asarray(P, float)
        covs = np.asarray(covs, float)
        n = len(ts)
        if n < 3:
            return None
        t_ref = ts[-1]
        dt = ts - t_ref
        keep = np.ones(n, bool)
        for _ in range(2):
            res = self._solve(dt[keep], P[keep], covs[keep])
            if res is None:
                return None
            theta, cov, chi2s = res
            # Mahalanobis on all points, drop outliers once
            m2 = self._m2(theta, dt, P, covs)
            new_keep = m2 < self.outlier_chi2
            if new_keep.sum() < 3 or (new_keep == keep).all():
                break
            keep = new_keep
        dof = max(1, keep.sum() * (2 if self.rolling else 3) - (4 if self.rolling else 6))
        rchi2 = float(chi2s / dof)
        cov = cov * max(1.0, rchi2)
        return Fit(t_ref, theta[:3], theta[3:], cov, int(keep.sum()), rchi2, self.k, self.g,
                   self.rolling, self.r)

    # internal --------------------------------------------------------
    def _axes(self):
        return [0, 2] if self.rolling else [0, 1, 2]

    def _solve(self, dt, P, covs):
        ax = self._axes()
        na = len(ax)
        A = np.zeros((2 * na, 2 * na))
        bvec = np.zeros(2 * na)
        ag = np.array([0.0, -self.g, 0.0])
        rows = []
        for i in range(len(dt)):
            Bt, Gt = _B(dt[i], self.k), _G(dt[i], self.k)
            y = P[i] - (0 if self.rolling else ag * Gt)
            y = y[ax]
            S = covs[i][np.ix_(ax, ax)]
            W = np.linalg.inv(S + 1e-12 * np.eye(na))
            H = np.hstack([np.eye(na), Bt * np.eye(na)])
            A += H.T @ W @ H
            bvec += H.T @ W @ y
            rows.append((H, W, y))
        try:
            cov = np.linalg.inv(A)
        except np.linalg.LinAlgError:
            return None
        th = cov @ bvec
        chi2 = sum(float((y - H @ th) @ W @ (y - H @ th)) for H, W, y in rows)
        theta = np.zeros(6)
        cov6 = np.zeros((6, 6))
        idx = ax + [a + 3 for a in ax]
        theta[idx] = th
        cov6[np.ix_(idx, idx)] = cov
        if self.rolling:
            theta[1] = self.r
        return theta, cov6, chi2

    def _m2(self, theta, dt, P, covs):
        ax = self._axes()
        out = np.empty(len(dt))
        ag = np.array([0.0, -self.g, 0.0]) if not self.rolling else np.zeros(3)
        for i in range(len(dt)):
            pr = theta[:3] + theta[3:] * _B(dt[i], self.k) + ag * _G(dt[i], self.k)
            d = (P[i] - pr)[ax]
            S = covs[i][np.ix_(ax, ax)]
            out[i] = d @ np.linalg.solve(S + 1e-12 * np.eye(len(ax)), d)
        return out

    # crossing --------------------------------------------------------
    def _cross_theta(self, fit: Fit, theta, bounce=True):
        """Deterministic crossing for a given state -> (dt, x, y, bounced) or None."""
        p0, v0 = theta[:3], theta[3:]
        if p0[2] <= 0 or v0[2] >= -1e-3:
            return None
        k = fit.k
        s = -p0[2] / v0[2]            # needed B(t)
        if k >= 1e-6:
            if 1 - k * s <= 0:
                return None           # stops before reaching the goal
            tc = -math.log(1 - k * s) / k
        else:
            tc = s
        p, v = fit.state_at(tc, theta)
        if fit.rolling or not bounce or p[1] >= fit.r_ball or self.e <= 0:
            return tc, p[0], p[1], False
        # floor bounce: first time y hits r in (0, tc)
        lo, hi = 0.0, tc
        if fit.state_at(0.0, theta)[0][1] <= fit.r_ball:
            return tc, p[0], p[1], False
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if fit.state_at(mid, theta)[0][1] > fit.r_ball:
                lo = mid
            else:
                hi = mid
        tb = hi
        pb, vb = fit.state_at(tb, theta)
        vb = vb.copy()
        vb[1] = -self.e * vb[1]
        th2 = np.concatenate([pb, vb])
        r2 = self._cross_theta(fit, th2, bounce=False)
        if r2 is None:
            return None
        return tb + r2[0], r2[1], max(r2[2], fit.r_ball), True

    def crossing(self, fit: Fit | None, sigma_ref=0.03) -> Crossing:
        if fit is None or fit.n < self.min_pts:
            return Crossing(False)
        th = fit.theta
        base = self._cross_theta(fit, th)
        if base is None or base[0] > self.horizon:
            return Crossing(False)
        J = np.zeros((3, 6))
        sc = np.sqrt(np.maximum(np.diag(fit.cov), 1e-12))
        for j in range(6):
            if sc[j] < 1e-9 or (fit.rolling and j in (1, 4)):
                continue
            d = np.zeros(6)
            d[j] = 1e-3 * max(sc[j], 1e-4)
            a = self._cross_theta(fit, th + d)
            b = self._cross_theta(fit, th - d)
            if a is None or b is None:
                continue
            J[:, j] = (np.array(a[:3]) - np.array(b[:3])) / (2 * d[j])
        C = J @ fit.cov @ J.T
        cov_xy = C[1:, 1:]
        sig = math.sqrt(max(cov_xy[0, 0] + cov_xy[1, 1], 0))
        conf = min(1.0, fit.n / 6) / (1 + (sig / sigma_ref) ** 2)
        return Crossing(True, fit.t_ref + base[0], base[1], base[2], cov_xy,
                        math.sqrt(max(C[0, 0], 0)), float(conf), base[3])


class ConstAccelKF:
    """9-state [p, v, a] KF. Measurements are 3D points with full covariance."""

    def __init__(self, g=G, q_jerk=50.0, sigma_a0=2.0, sigma_v0=10.0):
        self.g, self.q, self.sa0, self.sv0 = g, q_jerk, sigma_a0, sigma_v0
        self.x = None
        self.P = None
        self.t = None
        self.n = 0

    def reset(self):
        self.x = None
        self.n = 0

    def update(self, t, z, R):
        if self.x is None:
            self.x = np.concatenate([z, np.zeros(3), [0.0, -self.g, 0.0]])
            self.P = np.diag(np.concatenate([np.diag(R), np.full(3, self.sv0 ** 2),
                                             np.full(3, self.sa0 ** 2)]))
            self.t, self.n = t, 1
            return
        dt = t - self.t
        F = np.eye(9)
        F[:3, 3:6] = dt * np.eye(3)
        F[:3, 6:] = 0.5 * dt * dt * np.eye(3)
        F[3:6, 6:] = dt * np.eye(3)
        q = self.q
        Q1 = q * np.array([[dt ** 5 / 20, dt ** 4 / 8, dt ** 3 / 6],
                           [dt ** 4 / 8, dt ** 3 / 3, dt ** 2 / 2],
                           [dt ** 3 / 6, dt ** 2 / 2, dt]])
        Q = np.kron(Q1, np.eye(3))
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        Hm = np.zeros((3, 9))
        Hm[:, :3] = np.eye(3)
        S = Hm @ self.P @ Hm.T + R
        K = self.P @ Hm.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - Hm @ self.x)
        self.P = (np.eye(9) - K @ Hm) @ self.P
        self.t = t
        self.n += 1

    def crossing(self, sigma_ref=0.03) -> Crossing:
        if self.x is None or self.n < 3:
            return Crossing(False)

        def cross(x):
            p, v, a = x[:3], x[3:6], x[6:]
            if p[2] <= 0:
                return None
            # z + vz t + 0.5 az t^2 = 0, smallest positive root
            A, B, C = 0.5 * a[2], v[2], p[2]
            if abs(A) < 1e-9:
                if B >= -1e-3:
                    return None
                t = -C / B
            else:
                disc = B * B - 4 * A * C
                if disc < 0:
                    return None
                rts = [(-B - math.sqrt(disc)) / (2 * A), (-B + math.sqrt(disc)) / (2 * A)]
                rts = [r for r in rts if r > 0]
                if not rts:
                    return None
                t = min(rts)
            pc = p + v * t + 0.5 * a * t * t
            return np.array([t, pc[0], pc[1]])

        base = cross(self.x)
        if base is None or self.x[5] >= 0:
            return Crossing(False)
        J = np.zeros((3, 9))
        for j in range(9):
            d = np.zeros(9)
            d[j] = 1e-4
            a, b = cross(self.x + d), cross(self.x - d)
            if a is not None and b is not None:
                J[:, j] = (a - b) / 2e-4
        C = J @ self.P @ J.T
        cov_xy = C[1:, 1:]
        sig = math.sqrt(max(cov_xy[0, 0] + cov_xy[1, 1], 0))
        conf = min(1.0, self.n / 6) / (1 + (sig / sigma_ref) ** 2)
        return Crossing(True, self.t + base[0], base[1], base[2], cov_xy,
                        math.sqrt(max(C[0, 0], 0)), float(conf))
