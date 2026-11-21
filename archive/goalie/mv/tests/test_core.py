"""Noiseless round-trip tests: geometry back-projection and trajectory fits
must be (near) exact.  Run:  .venv/bin/python -m pytest -q tests"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config import Camera, look_rotation  # noqa: E402
from geometry import backproject, ground_point, project_sphere  # noqa: E402
from trajectory import BallisticFitter, ConstAccelKF  # noqa: E402


def cam_default():
    c = Camera.nominal(640, 480, 55)
    c.set_pose(height=0.4, back=0.3, pitch_deg=10, yaw_deg=5)
    return c


def test_rotation_orthogonal():
    R = look_rotation(12, 7)
    assert np.allclose(R @ R.T, np.eye(3))
    # optical axis points out toward the shooter (+Z) and slightly down
    assert R[2, 2] > 0.9 and R[2, 1] < 0


@pytest.mark.parametrize("p", [(0, 0.3, 2.0), (0.6, 0.1, 1.2), (-0.8, 0.9, 3.5), (0.25, 0.4, 0.6)])
@pytest.mark.parametrize("D", [0.04, 0.067, 0.22])
def test_backproject_roundtrip(p, D):
    cam = cam_default()
    u, v, d = project_sphere(cam, p, D / 2)
    m = backproject(cam, u, v, d, D)
    assert np.allclose(m.p, p, atol=1e-6), (m.p, p)


def test_simple_formula_on_axis():
    cam = Camera.nominal(640, 480, 55)
    cam.set_pose(height=0, back=0, pitch_deg=0)
    D, Z = 0.067, 2.0
    u, v, d = project_sphere(cam, (0, 0, Z), D / 2)
    assert abs(u - 320) < 1e-9 and abs(v - 240) < 1e-9
    assert abs(d - cam.f * D / Z) / d < 1e-3   # Z = f D / d for small balls


def test_depth_sigma_grows_z2():
    cam = cam_default()
    s, D = [], []
    for Z in (1.0, 2.0):
        u, v, d = project_sphere(cam, (0, 0.4, Z), 0.0335)
        m = backproject(cam, u, v, d, 0.067)
        s.append(math.sqrt(m.ray_cam @ (cam.Rm @ m.cov @ cam.Rm.T) @ m.ray_cam))
        D.append(m.depth)
    assert abs(s[1] / s[0] - (D[1] / D[0]) ** 2) < 0.05   # sigma ~ D^2


def test_ground_point():
    cam = cam_default()
    p = (0.3, 0.0335, 1.7)
    u, v, _ = project_sphere(cam, p, 0.0335)
    m = ground_point(cam, u, v, 0.067)
    # the ray goes through the ellipse centre, which is offset from the sphere
    # centre's projection off-axis: sub-millimetre here
    assert np.allclose(m.p, p, atol=5e-4)


def _shot(p0, v0, ts, k=0.0, g=9.81):
    out = []
    for t in ts:
        if k == 0:
            out.append(np.array(p0) + np.array(v0) * t + 0.5 * np.array([0, -g, 0]) * t * t)
        else:
            B = (1 - math.exp(-k * t)) / k
            Gt = (t - B) / k
            out.append(np.array(p0) + np.array(v0) * B + np.array([0, -g, 0]) * Gt)
    return np.array(out)


@pytest.mark.parametrize("k", [0.0, 0.6])
def test_ballistic_fit_exact(k):
    p0, v0 = (0.2, 0.5, 3.0), (-0.1, 2.5, -6.0)
    ts = np.arange(10) / 30
    P = _shot(p0, v0, ts, k)
    covs = [np.diag([1e-4, 1e-4, 4e-3])] * len(ts)
    f = BallisticFitter(drag_k=k, r_ball=0.0335).fit(ts, P, covs)
    p_now, v_now = P[-1], None
    assert np.allclose(f.p, p_now, atol=1e-8)
    fit = BallisticFitter(drag_k=k, r_ball=0.0335)
    c = fit.crossing(f)
    # ground truth crossing by dense simulation
    tt = np.linspace(0, 1.0, 200001)
    Z = _shot(p0, v0, tt, k)
    i = np.argmax(Z[:, 2] <= 0)
    assert c.coming and not c.bounced
    assert abs(c.t_cross - tt[i]) < 1e-4
    assert abs(c.x - Z[i, 0]) < 1e-4 and abs(c.y - Z[i, 1]) < 1e-4


def test_rolling_fit_exact():
    ts = np.arange(8) / 30
    P = np.array([[0.1 - 0.2 * t, 0.02, 1.0 - 2.0 * t] for t in ts])
    covs = [np.diag([1e-4, 1e-8, 1e-4])] * len(ts)
    fitter = BallisticFitter(rolling=True, r_ball=0.02)
    f = fitter.fit(ts, P, covs)
    c = fitter.crossing(f)
    assert abs(c.t_cross - 0.5) < 1e-9 and abs(c.x - 0.0) < 1e-9 and abs(c.y - 0.02) < 1e-12


def test_moving_away_not_coming():
    ts = np.arange(6) / 30
    P = _shot((0, 0.5, 1.0), (0, 1, 3.0), ts)
    fitter = BallisticFitter()
    c = fitter.crossing(fitter.fit(ts, P, [np.eye(3) * 1e-4] * 6))
    assert not c.coming


def test_bounce():
    # low fast shot that hits the floor before the goal
    ts = np.arange(6) / 60
    P = _shot((0, 0.3, 2.0), (0, -1.0, -4.0), ts)
    fitter = BallisticFitter(r_ball=0.0335, restitution=0.75)
    c = fitter.crossing(fitter.fit(ts, P, [np.eye(3) * 1e-6] * 6))
    assert c.coming and c.bounced and c.y > 0.0335


def test_kf_converges():
    ts = np.arange(30) / 60
    P = _shot((0.2, 0.5, 3.0), (-0.1, 1.5, -6.0), ts)
    kf = ConstAccelKF()
    for t, p in zip(ts, P):
        kf.update(t, p, np.eye(3) * 1e-8)
    c = kf.crossing()
    tt = np.linspace(0, 1.0, 100001)
    Z = _shot((0.2, 0.5, 3.0), (-0.1, 1.5, -6.0), tt)
    i = np.argmax(Z[:, 2] <= 0)
    assert abs(c.x - Z[i, 0]) < 2e-3 and abs(c.y - Z[i, 1]) < 2e-3
