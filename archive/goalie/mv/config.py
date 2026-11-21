"""Camera / ball / detector configuration, shared by every goalie/mv module.

World frame (metres), origin at the goal centre ON THE FLOOR:
  X = across the goal, + = right as seen from the camera/goalie looking out at
      the shooter (i.e. image-right for the default pose)
  Y = height above the floor, + = up
  Z = distance out from the goal plane toward the shooter, + = into the field
A ball coming at the goal has vz < 0 and crosses the goal plane at Z = 0.

Camera frame is OpenCV's: x right, y down, z forward (optical axis).
`Camera.R` maps world -> camera directions (its rows are the camera axes in
world coordinates) and `Camera.C` is the camera centre in world coordinates:
    p_cam = R @ (p_world - C)
Note: the world frame above is left-handed as an X/Y/Z triple written that way
(it mirrors OpenCV's), so R for the default pose has det = -1.  That is fine:
R is orthogonal, R^-1 = R^T, and nothing here assumes det = +1.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "camera.json"

# Common balls (diameters, metres)
BALLS = {"pingpong": 0.040, "tennis": 0.067, "soccer": 0.220}
# Quadratic air drag  a = -kq |v| v  (kq = rho Cd A / 2m, 1/m).  Ping-pong is
# light: at 5 m/s it decelerates ~3.5 m/s^2, comparable to a third of g.
DRAG_QUAD = {"pingpong": 0.14, "tennis": 0.018, "soccer": 0.011}


def look_rotation(yaw_deg: float = 0.0, pitch_deg: float = 0.0) -> np.ndarray:
    """World->camera rotation for a camera looking along +Z (out at the shooter),
    turned right by `yaw_deg` (toward +X) and tilted DOWN by `pitch_deg`."""
    th, ps = math.radians(pitch_deg), math.radians(yaw_deg)
    z = np.array([math.sin(ps) * math.cos(th), -math.sin(th), math.cos(ps) * math.cos(th)])
    x = np.array([math.cos(ps), 0.0, -math.sin(ps)])
    y = np.cross(x, z)  # image-down
    return np.vstack([x, y, z])


@dataclass
class Camera:
    w: int = 640
    h: int = 480
    fx: float = 615.0
    fy: float = 615.0
    cx: float = 320.0
    cy: float = 240.0
    dist: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])
    R: list = field(default_factory=lambda: look_rotation(0, 8).tolist())
    C: list = field(default_factory=lambda: [0.0, 0.35, -0.25])

    # --- convenience -------------------------------------------------------
    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    @property
    def Rm(self) -> np.ndarray:
        return np.asarray(self.R, float)

    @property
    def Cv(self) -> np.ndarray:
        return np.asarray(self.C, float)

    @property
    def f(self) -> float:
        return 0.5 * (self.fx + self.fy)

    @classmethod
    def nominal(cls, w=640, h=480, hfov_deg=55.0, **pose) -> "Camera":
        """Intrinsics from a horizontal-FOV guess (C270 ~55-60 deg diagonal-ish)."""
        f = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
        cam = cls(w=w, h=h, fx=f, fy=f, cx=w / 2, cy=h / 2)
        if pose:
            cam.set_pose(**pose)
        return cam

    def set_pose(self, height=0.35, back=0.25, x=0.0, pitch_deg=8.0, yaw_deg=0.0):
        """Simple pose: camera `back` m behind the goal plane, `height` m up,
        tilted down `pitch_deg`, turned right `yaw_deg`."""
        self.C = [float(x), float(height), -float(back)]
        self.R = look_rotation(yaw_deg, pitch_deg).tolist()

    def scaled(self, w: int, h: int) -> "Camera":
        """Same camera at a different capture resolution."""
        sx, sy = w / self.w, h / self.h
        return Camera(w, h, self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy,
                      list(self.dist), [list(r) for r in self.R], list(self.C))


@dataclass
class DetectCfg:
    # OpenCV HSV: H 0-179, S/V 0-255. If h_lo > h_hi the range wraps (reds).
    hsv_lo: list = field(default_factory=lambda: [5, 140, 90])  # S>=140 keeps most skin out
    hsv_hi: list = field(default_factory=lambda: [28, 255, 255])
    min_diam_px: float = 4.0
    use_bg: bool = False          # MOG2 motion mask AND colour mask
    static_reject: bool = True    # ignore colour pixels that stay on ~10+ frames
    size_method: str = "moments"  # "moments" (blur-robust, sub-px) | "ellipse"
    gate_px: float = 80.0         # max jump from the predicted pixel position
    sigma_center_px: float = 0.5  # measurement noise model (fed to the fit)
    sigma_diam_px: float = 0.6


@dataclass
class Config:
    camera: Camera = field(default_factory=Camera)
    ball_diam: float = BALLS["tennis"]
    detect: DetectCfg = field(default_factory=DetectCfg)
    goal_w: float = 0.80
    goal_h: float = 0.50
    rolling: bool = False         # ball on the floor: use ground-plane rays
    drag_k: float = 0.0           # fixed linear drag 1/s (0 = vacuum)
    drag_quad: float = 0.0        # if > 0: fit with k = drag_quad * |v| (adaptive)

    def save(self, path=DEFAULT_CONFIG):
        Path(path).write_text(json.dumps(asdict(self), indent=2))

    @classmethod
    def load(cls, path=DEFAULT_CONFIG) -> "Config":
        p = Path(path)
        if not p.exists():
            return cls()
        d = json.loads(p.read_text())
        cfg = cls()
        cfg.camera = Camera(**d.get("camera", {}))
        cfg.detect = DetectCfg(**d.get("detect", {}))
        for k in ("ball_diam", "goal_w", "goal_h", "rolling", "drag_k", "drag_quad"):
            if k in d:
                setattr(cfg, k, d[k])
        return cfg

    def set_ball(self, name_or_diam):
        """'pingpong' | 'tennis' | 'soccer' | diameter in metres."""
        if isinstance(name_or_diam, str) and name_or_diam in BALLS:
            self.ball_diam = BALLS[name_or_diam]
            self.drag_quad = DRAG_QUAD[name_or_diam]
        else:
            self.ball_diam = float(name_or_diam)
