"""Robot + simulation parameters. Override any field with a JSON file:

    python run.py --params my_robot.json      # {"body_mass": 1.6, "com_height": 0.3}

All units SI (kg, m, s, rad, N, N*m).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict, fields

DEG = math.pi / 180.0


@dataclass
class RobotParams:
    # ---- geometry / mass (default: ~45 cm tall slab stack, battery up high) ----
    body_mass: float = 1.30        # everything except the wheels (slabs, Pi, LiPo, steppers)
    body_height: float = 0.45      # slab height above the axle (for drawing / push point)
    body_width: float = 0.10       # slab thickness front-to-back (drawing + inertia)
    com_height: float = 0.25       # CoM distance above the axle. Steppers sit low, battery high
    com_offset: float = 0.004      # CoM forward offset from the slab axis (unknown to controller)
    body_inertia: float | None = None  # about CoM, pitch axis. None -> slab estimate
    wheel_radius: float = 0.045    # 90 mm wheels
    wheel_mass: float = 0.10       # both wheels together
    track_width: float = 0.20

    # ---- drive ----
    drive: str = "stepper"         # "stepper" or "dc"
    # NEMA17 (~45 N*cm holding) on 12 V, 1/8 microstep. Pull-out torque falls ~linearly
    # with speed; these numbers are conservative.
    stepper_torque: float = 0.38   # N*m per motor at low speed (pull-out, not holding)
    stepper_max_rps: float = 12.0  # rev/s where pull-out torque reaches ~0
    stepper_stiffness: float = 300.0  # 1/s, how hard the rotor tracks the step train
    # DC gearmotor option. Output-shaft values, per motor, at the supply voltage:
    #   torque = stall * (u - w/w_noload) - friction,  u in [-1, 1] (PWM duty)
    # Defaults = TT-style yellow gearmotor (1:48, 6 V). See MOTOR_PRESETS.
    dc_stall_torque: float = 0.08  # N*m per motor (~0.8 kgf*cm)
    dc_noload_rpm: float = 200.0
    dc_backlash: float = 3.0 * DEG  # total gear slop at the output shaft
    dc_rotor_inertia: float = 2e-4  # kg*m^2 per motor, reflected to the output (J_rotor*N^2)
    dc_gear_stiffness: float = 20.0  # N*m/rad gear-train compliance once the slop is taken up
    dc_coulomb: float = 0.008      # N*m per motor of friction (static ~ Coulomb, smoothed)
    dc_deadband: float = 0.12      # |duty| below this does nothing (driver + stiction)
    dc_deadband_comp: float = 0.10  # what the firmware adds to |duty| to hide the deadband
    dc_encoder: bool = False       # False -> no wheel-speed feedback at all

    # ---- environment ----
    gravity: float = 9.81
    rolling_damping: float = 0.05  # N per m/s
    wheel_slip: bool = False       # cap traction at mu*N
    mu: float = 0.8

    # ---- sensors (MPU6050-ish after a quick calibration) ----
    imu_height: float = 0.20       # IMU mount height above the axle
    imu_mount_error: float = 0.5 * DEG  # IMU not perfectly square to the slab
    gyro_noise: float = 0.01       # rad/s std per sample (includes stepper vibration)
    gyro_bias: float = 0.5 * DEG   # rad/s constant bias
    accel_noise: float = 0.3       # m/s^2 std per sample
    imu_latency: float = 0.002     # s, sample-to-controller delay
    fused_imu: bool = False        # True -> BNO085-style fused angle
    fused_noise: float = 0.1 * DEG
    encoder_noise: float = 0.01    # m/s (DC mode wheel speed)

    # ---- timing ----
    physics_hz: float = 1000.0
    control_hz: float = 400.0
    control_jitter: float = 0.0002  # s std of loop period jitter

    # ---- run ----
    initial_pitch: float = 2.0 * DEG
    seed: int = 1

    def inertia(self) -> float:
        if self.body_inertia is not None:
            return self.body_inertia
        # uniform slab of body_height x body_width about its centre
        return self.body_mass * (self.body_height ** 2 + self.body_width ** 2) / 12.0

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_json(cls, path: str | None) -> "RobotParams":
        p = cls()
        if path:
            with open(path) as f:
                p.update(json.load(f))
        return p

    def update(self, d: dict):
        names = {f.name for f in fields(self)}
        for k, v in d.items():
            if k not in names:
                raise KeyError(f"unknown robot param {k!r}")
            setattr(self, k, v)
        return self


# Motor presets: python run.py --motor tt
MOTOR_PRESETS = {
    # 2x NEMA17 + TMC2209, direct drive: no gearbox, no backlash, open-loop step count
    "nema17": dict(drive="stepper"),
    # 2x yellow "TT" gearmotor, 1:48 plastic gears, 6 V, no encoder
    "tt": dict(drive="dc", dc_stall_torque=0.08, dc_noload_rpm=200.0,
               dc_backlash=3.0 * DEG, dc_rotor_inertia=2e-4, dc_gear_stiffness=20.0,
               dc_coulomb=0.008, dc_deadband=0.12, dc_deadband_comp=0.10,
               dc_encoder=False),
    # 2x generic 25 mm metal gearmotor (JGA25-370 style), 6 V ~200 RPM, no encoder
    "dc25": dict(drive="dc", dc_stall_torque=0.30, dc_noload_rpm=200.0,
                 dc_backlash=2.0 * DEG, dc_rotor_inertia=4e-4, dc_gear_stiffness=40.0,
                 dc_coulomb=0.015, dc_deadband=0.08, dc_deadband_comp=0.07,
                 dc_encoder=False),
    # 2x Pololu 37D 12 V 30:1 with 64 CPR encoder (what you'd order for DC)
    "pololu37d": dict(drive="dc", dc_stall_torque=1.1, dc_noload_rpm=330.0,
                      dc_backlash=1.0 * DEG, dc_rotor_inertia=6e-4, dc_gear_stiffness=80.0,
                      dc_coulomb=0.02, dc_deadband=0.05, dc_deadband_comp=0.04,
                      dc_encoder=True),
}


def robot_for_motor(name: str, base: "RobotParams | None" = None) -> "RobotParams":
    import copy
    p = copy.deepcopy(base) if base is not None else RobotParams()
    if name not in MOTOR_PRESETS:
        raise KeyError(f"unknown motor {name!r}, pick one of {sorted(MOTOR_PRESETS)}")
    return p.update(MOTOR_PRESETS[name])


# Controller gains/config live in the C library (balance_default_cfg). The sim
# can override them with a dict of balance_cfg_t field names, e.g.
#   {"kp": 60, "kd": 7, "speed_kp": 0.12, "speed_ki": 0.05}
@dataclass
class Scenario:
    duration: float = 8.0
    # (time_s, impulse_N_s) applied horizontally at the top of the slab over push_width
    pushes: list = field(default_factory=lambda: [(2.0, 0.25), (5.0, -0.25)])
    push_width: float = 0.08
    # (time_s, speed_m_s, turn_rad_s) setpoint changes
    setpoints: list = field(default_factory=list)
