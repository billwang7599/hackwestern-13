"""Pinhole back-projection: (pixel centre, apparent diameter) -> 3D world point.

Size -> depth.  A sphere of radius r whose centre is at distance D from the
camera subtends a cone of half-angle alpha = asin(r / D).  If the cone axis is
theta off the optical axis, the image is an ellipse with
    minor semi-axis  b   = f sin(a) / sqrt(cos^2(t) - sin^2(a))
    major endpoints  rho = f tan(t -+ a)       (radial, from the principal point)
so the ellipse centre is NOT the projection of the sphere centre when off-axis.
We invert that exactly (a few fixed-point steps): given the undistorted centre
radius rho_c and minor semi-axis b, solve for (theta, alpha), then
    D = r / sin(alpha)  along the axis direction.
On-axis and small, this is the familiar  Z = f * D_ball / d_px.

Uncertainty: D ~ f*D_ball/d  =>  sigma_D = D^2 / (f * D_ball) * sigma_d  (along
the ray) and sigma_lat = D / f * sigma_c (across).  Depth is the noisy axis.

Rolling mode: if the ball is known to be on the floor (centre height = r), the
ray/plane intersection gives a far better position than the size, so
`ground_point` is used instead and size is only a consistency check.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from config import Camera


@dataclass
class Meas3D:
    t: float
    p: np.ndarray          # world position (3,)
    cov: np.ndarray        # world covariance (3,3)
    depth: float           # camera-to-ball-centre distance (m)
    ray_cam: np.ndarray    # unit direction in camera frame


def undistort_px(cam: Camera, u: float, v: float) -> tuple[float, float]:
    """Pixel -> normalized image coords (x/z, y/z), lens distortion removed."""
    if not any(cam.dist):
        return (u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy
    pts = np.array([[[u, v]]], np.float64)
    n = cv2.undistortPoints(pts, cam.K, np.asarray(cam.dist, float))
    return float(n[0, 0, 0]), float(n[0, 0, 1])


def sphere_from_ellipse(f: float, rho_c: float, b: float, iters: int = 6):
    """Invert the off-axis sphere projection. rho_c, b in pixels (focal f).
    Returns (theta, alpha) in radians."""
    theta = math.atan2(rho_c, f)
    alpha = 0.0
    for _ in range(iters):
        c2 = math.cos(theta) ** 2
        s2 = b * b * c2 / (f * f + b * b)
        alpha = math.asin(min(math.sqrt(s2), 0.999))
        # Newton step on centre(theta) = f/2 (tan(t+a) + tan(t-a)) = rho_c
        g = 0.5 * f * (math.tan(theta + alpha) + math.tan(theta - alpha)) - rho_c
        dg = 0.5 * f * (1 / math.cos(theta + alpha) ** 2 + 1 / math.cos(theta - alpha) ** 2)
        theta -= g / dg
    return theta, alpha


def project_sphere(cam: Camera, p_world, radius: float):
    """Forward model (no distortion): world sphere -> (u, v) of the ellipse
    centre and its minor diameter in px.  Used by tests and the synth."""
    pc = cam.Rm @ (np.asarray(p_world, float) - cam.Cv)
    D = float(np.linalg.norm(pc))
    alpha = math.asin(min(radius / D, 0.999))
    theta = math.acos(pc[2] / D)
    f = cam.f
    rho = 0.5 * f * (math.tan(theta + alpha) + math.tan(theta - alpha))
    b = f * math.sin(alpha) / math.sqrt(max(math.cos(theta) ** 2 - math.sin(alpha) ** 2, 1e-9))
    rxy = math.hypot(pc[0], pc[1])
    ux, uy = (pc[0] / rxy, pc[1] / rxy) if rxy > 1e-12 else (0.0, 0.0)
    return cam.cx + rho * ux * cam.fx / f, cam.cy + rho * uy * cam.fy / f, 2 * b


def backproject(cam: Camera, u: float, v: float, diam_px: float, ball_diam: float,
                t: float = 0.0, sigma_c: float = 0.5, sigma_d: float = 0.6) -> Meas3D:
    """Pixel centre + apparent (minor-axis) diameter -> 3D world point."""
    xn, yn = undistort_px(cam, u, v)
    f = cam.f
    rho_c = f * math.hypot(xn, yn)
    r = ball_diam / 2
    theta, alpha = sphere_from_ellipse(f, rho_c, diam_px / 2)
    D = r / math.sin(alpha)
    rn = math.hypot(xn, yn)
    ux, uy = (xn / rn, yn / rn) if rn > 1e-12 else (0.0, 0.0)
    d_cam = np.array([math.sin(theta) * ux, math.sin(theta) * uy, math.cos(theta)])
    p_cam = D * d_cam
    # covariance: sigma along the ray, sigma_lat across it (camera frame)
    s_ray = D * D / (f * ball_diam) * sigma_d
    s_lat = D / f * sigma_c
    cov_cam = s_lat ** 2 * np.eye(3) + (s_ray ** 2 - s_lat ** 2) * np.outer(d_cam, d_cam)
    Rm = cam.Rm
    p_w = Rm.T @ p_cam + cam.Cv
    cov_w = Rm.T @ cov_cam @ Rm
    return Meas3D(t, p_w, cov_w, D, d_cam)


def ray_world(cam: Camera, u: float, v: float) -> np.ndarray:
    xn, yn = undistort_px(cam, u, v)
    d = np.array([xn, yn, 1.0])
    d /= np.linalg.norm(d)
    return cam.Rm.T @ d


def ground_point(cam: Camera, u: float, v: float, ball_diam: float, t: float = 0.0,
                 sigma_c: float = 0.5):
    """Rolling ball: intersect the ray through the ball centre with the plane
    Y = r (ball centre height).  Returns Meas3D or None if the ray never hits."""
    r = ball_diam / 2
    d = ray_world(cam, u, v)
    C = cam.Cv
    if abs(d[1]) < 1e-6:
        return None
    s = (r - C[1]) / d[1]
    if s <= 0:
        return None
    p = C + s * d
    # lateral pixel noise maps to the plane with grazing-angle stretch
    s_lat = s / cam.f * sigma_c
    graze = 1.0 / max(abs(d[1]), 0.05)
    along = np.array([d[0], 0.0, d[2]])
    along /= max(np.linalg.norm(along), 1e-9)
    cov = s_lat ** 2 * (np.eye(3) + (graze ** 2 - 1) * np.outer(along, along))
    cov[1, 1] = 1e-8
    return Meas3D(t, p, cov, float(s), cam.Rm @ d)
