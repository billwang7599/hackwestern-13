"""Beat the Goalie: physical, sensor and controller parameters.

Coordinates and defaults follow goalie/docs/architecture.md (the contract):
metres and seconds, origin at the centre of the goal mouth on the goal line,
+x to the right (seen from the overhead camera, goal at the bottom of the
image), +y out into the field. The ball comes in with vy < 0.

Two goalie geometries:

  pivot  (default) A cardboard paddle glued straight onto the gearmotor shaft.
         The shaft points along y (towards the shooter), mounted on a crossbar
         above the goal mouth, so the paddle hangs down and swings left/right
         like a windscreen wiper / pendulum in the x-z plane.
         Shaft angle q (rad), q = 0 hanging straight down, +q swings the tip
         towards +x.  Paddle tip x = L * sin(q).

  rail   A carriage on a rail along the goal line, pulled by a string wound
         on a spool on the motor shaft.  x = r_spool * q.  The paddle is a
         blade of width paddle_len lying across the goal.

Every field can be overridden from the CLI (see run.py) or a JSON file.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from dataclasses import field as dc_field

DEG = math.pi / 180.0
RPM = 2.0 * math.pi / 60.0          # rpm -> rad/s


@dataclass
class FieldParams:
    goal_half_w: float = 0.11        # 22 cm goal (contract GOAL_HALF_W)
    field_len: float = 0.60          # shooter end of the table (contract)
    field_half_w: float = 0.25       # side walls at +-25 cm (only matter for bank shots)
    walls: bool = False              # simulate side-wall bounces (bank shots)
    wall_restitution: float = 0.8


@dataclass
class BallParams:
    radius: float = 0.020            # 40 mm ping-pong ball
    mass: float = 0.0027
    roll_decel: float = 0.15         # m/s^2, rolling resistance on a table/cardboard
    drag_cd: float = 0.5             # air drag (matters above ~1.5 m/s for a ping-pong ball)
    air_rho: float = 1.2


@dataclass
class MotorParams:
    """Small plain 2-wire DC gearmotor, values at the OUTPUT shaft, at the supply voltage.

    torque = stall * (u_eff - w / w_noload) - coulomb*sign(w) - gravity(paddle)
    u_eff = 0 when |duty| < deadband, else duty (H-bridge in brake mode at 0).

    rpm_model = "gear": the motor at other RPMs is the same motor with another
    gear ratio (constant power): stall ~ 1/rpm, reflected inertia ~ 1/rpm^2,
    so the mechanical time constant stays the same.  "fixed" keeps stall torque
    and inertia as given and only changes the no-load speed.
    The reference values (ref_*) describe the motor at ref_rpm.
    """
    rpm: float = 150.0               # no-load output RPM (MEASURE IT: docs/rpm_test.md)
    voltage: float = 6.0             # informational; the numbers below are "at this voltage"
    rpm_model: str = "gear"          # "gear" | "fixed"
    ref_rpm: float = 150.0
    ref_stall_torque: float = 0.10   # N*m (~1 kgf*cm), palm-sized plastic/metal gearbox
    ref_inertia: float = 2.5e-4      # kg*m^2 rotor reflected to output (J_rotor * N^2)
    coulomb_frac: float = 0.10       # gearbox friction as a fraction of stall torque
    deadband: float = 0.12           # |duty| below this: motor doesn't move (driver + stiction)
    backlash: float = 3.0 * DEG      # total gear slop at the output shaft

    @property
    def w_noload(self) -> float:
        return self.rpm * RPM

    @property
    def stall_torque(self) -> float:
        if self.rpm_model == "gear":
            return self.ref_stall_torque * self.ref_rpm / self.rpm
        return self.ref_stall_torque

    @property
    def inertia(self) -> float:
        if self.rpm_model == "gear":
            return self.ref_inertia * (self.ref_rpm / self.rpm) ** 2
        return self.ref_inertia


@dataclass
class GoalieParams:
    geometry: str = "pivot"          # "pivot" | "rail"
    paddle_len: float = 0.12         # pivot: arm length shaft->tip; rail: blade width along x
    paddle_w: float = 0.05           # pivot: blade width (cardboard strip width)
    paddle_mass: float = 0.006       # cardboard + glue
    pivot_gap: float = 0.004         # pivot: tip clearance above the table at q = 0
    q_max: float = 75.0 * DEG        # pivot: software angle limit
    spool_radius: float = 0.015      # rail: string spool radius on the shaft (3 cm spool)
    carriage_mass: float = 0.030     # rail
    rail_friction: float = 0.15      # rail: N of sliding friction on the carriage

    @property
    def x_max(self) -> float:
        """Largest |paddle x| the controller will ask for."""
        if self.geometry == "pivot":
            return self.paddle_len * math.sin(self.q_max)
        return 0.11                  # rail end stops at the goal posts


@dataclass
class CameraParams:
    fps: float = 30.0                # C270 does 30 fps at 640x480
    latency: float = 0.050           # s, capture -> Pi (exposure + USB + CV + Wi-Fi)
    noise: float = 0.002             # m, 1-sigma position noise per detection (both objects)
    drop: float = 0.0                # probability a detection is missed


@dataclass
class ControlParams:
    """What runs on the Pi (ported to C).  See predict.py / controller.py."""
    rate: float = 200.0              # Hz (contract)
    latency_est: float = 0.050       # s, the Pi's assumed camera latency (needn't be exact)
    # predictor (predict.py)
    window: int = 5                  # frames in the velocity fit
    min_samples: int = 3
    vy_in: float = -0.15             # m/s: below this the ball is "incoming"
    t_hit_max: float = 2.0           # s: a predicted crossing further away than this is ignored
    shadow_gain: float = 0.5         # not incoming: target = shadow_gain * ball x
    unseen_s: float = 0.25           # no ball this long -> forget it, park centre
    stale_gap_s: float = 0.15        # gap between samples bigger than this -> restart the fit
    # controller (controller.py), units are duty per rad of shaft angle
    kp: float = 3.0
    kd: float = 0.10
    db_comp: float = 0.12            # added to |duty| to cross the motor deadband
    min_duty: float = 0.06           # |u| below this -> 0 (no hunting at target)
    model_uoff: float = 0.10         # Pi model: duty eaten by gearbox friction (~coulomb/stall)
    obs_l1: float = 0.5              # observer position correction gain per camera frame
    obs_l2: float = 2.0              # observer speed correction (rad/s per rad of innovation)
    model_rpm: float = 150.0         # controller's belief of no-load RPM
    model_tau: float = 0.04          # controller's belief of the motor time constant (s)


@dataclass
class SimParams:
    dt: float = 0.001                # plant integration step
    seed: int = 1
    pre_roll: float = 0.4            # s the ball sits in view before the shot (filter settles)
    field: FieldParams = dc_field(default_factory=FieldParams)
    ball: BallParams = dc_field(default_factory=BallParams)
    motor: MotorParams = dc_field(default_factory=MotorParams)
    goalie: GoalieParams = dc_field(default_factory=GoalieParams)
    camera: CameraParams = dc_field(default_factory=CameraParams)
    control: ControlParams = dc_field(default_factory=ControlParams)

    def to_dict(self) -> dict:
        return asdict(self)

    def update(self, d: dict) -> "SimParams":
        """Apply {"motor": {"rpm": 120}, "camera": {"fps": 60}, "dt": ...}."""
        for k, v in d.items():
            cur = getattr(self, k)
            if isinstance(v, dict):
                for kk, vv in v.items():
                    if not any(f.name == kk for f in fields(cur)):
                        raise KeyError(f"unknown param {k}.{kk}")
                    setattr(cur, kk, vv)
            else:
                setattr(self, k, v)
        return self

    @classmethod
    def from_json(cls, path: str | None) -> "SimParams":
        p = cls()
        if path:
            with open(path) as f:
                p.update(json.load(f))
        return p


# Casual hand roll along a table: ~0.8-1.5 m/s (60 cm in 0.4-0.75 s).
CASUAL_SHOT = 1.0       # m/s: the default scenario (pass/fail) shot speed
SAVE_TARGET = 0.90      # fraction of on-target shots that must be saved
