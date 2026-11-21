"""Controller backends for the sim.

"c"  -> balance/build/libbalance.{dylib,so} via ctypes (the exact code that runs
        on the robot). Build it with `make -C ../balance`.
"py" -> balance/micropython/balance.py (the Pico port) under CPython.
"auto" picks C if the library is built, else Python.
"""
from __future__ import annotations

import ctypes
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
BAL_DIR = os.path.join(ROOT, "balance")
sys.path.insert(0, os.path.join(BAL_DIR, "micropython"))
import balance as bpy  # noqa: E402

STATE_NAMES = bpy.STATE_NAMES
RUNNING, FALLEN = bpy.RUNNING, bpy.FALLEN

f, i = ctypes.c_float, ctypes.c_int


class CCfg(ctypes.Structure):
    _fields_ = [(n, f) for n in (
        "kp", "ki", "kd", "i_limit",
        "speed_kp", "speed_ki", "speed_i_limit", "max_lean", "outer_hz", "speed_lpf_tau",
        "max_accel", "max_speed", "max_turn_speed", "setpoint_accel", "track_width",
        "comp_tau", "pitch_offset", "gyro_bias_rate", "accel_comp")] + [("use_fused_pitch", i)] + [
        (n, f) for n in ("tilt_cutoff", "rearm_angle", "rearm_time")]


class CImu(ctypes.Structure):
    _fields_ = [("gyro_pitch", f), ("acc_fwd", f), ("acc_up", f), ("fused_pitch", f)]


class CSetpoint(ctypes.Structure):
    _fields_ = [("speed", f), ("turn", f), ("enable", i)]


class CCmd(ctypes.Structure):
    _fields_ = [("left", f), ("right", f), ("accel", f), ("pitch", f), ("pitch_rate", f),
                ("pitch_target", f), ("state", i), ("motors_on", i)]


class CState(ctypes.Structure):
    _fields_ = [("state", i), ("initialized", i)] + [(n, f) for n in (
        "pitch", "gyro_bias", "i_inner", "i_outer", "pitch_target", "vel", "speed_f",
        "speed_sp", "outer_acc", "upright_time", "last_accel")]


CFG_FIELDS = [n for n, _ in CCfg._fields_]


def _lib_path():
    name = "libbalance.dylib" if sys.platform == "darwin" else "libbalance.so"
    return os.path.join(BAL_DIR, "build", name)


_lib = None


def load_lib():
    global _lib
    if _lib is None:
        lib = ctypes.CDLL(_lib_path())
        lib.balance_default_cfg.argtypes = [ctypes.POINTER(CCfg)]
        lib.balance_init.argtypes = [ctypes.POINTER(CState)]
        lib.balance_step.argtypes = [ctypes.POINTER(CState), ctypes.POINTER(CCfg),
                                     ctypes.POINTER(CImu), f, ctypes.POINTER(CSetpoint), f]
        lib.balance_step.restype = CCmd
        _lib = lib
    return _lib


class Controller:
    def __init__(self, backend="auto", overrides: dict | None = None):
        if backend == "auto":
            backend = "c" if os.path.exists(_lib_path()) else "py"
        self.backend = backend
        if backend == "c":
            self.lib = load_lib()
            self.cfg = CCfg()
            self.lib.balance_default_cfg(ctypes.byref(self.cfg))
            self.st = CState()
            self.lib.balance_init(ctypes.byref(self.st))
            self.imu = CImu()
            self.sp = CSetpoint()
        else:
            self.cfg = bpy.Config()
            self.st = bpy.State()
            self.sp = bpy.Setpoint()
        for k, v in (overrides or {}).items():
            if k not in CFG_FIELDS:
                raise KeyError(f"unknown controller cfg field {k!r}")
            setattr(self.cfg, k, v)

    def get_cfg(self) -> dict:
        return {k: getattr(self.cfg, k) for k in CFG_FIELDS}

    def step(self, gyro, acc_fwd, acc_up, fused, wheel_speed, speed, turn, enable, dt):
        """Returns (left, right, accel, pitch_est, pitch_target, state, motors_on)."""
        ws = math.nan if wheel_speed is None else wheel_speed
        if self.backend == "c":
            self.imu.gyro_pitch, self.imu.acc_fwd, self.imu.acc_up, self.imu.fused_pitch = \
                gyro, acc_fwd, acc_up, fused
            self.sp.speed, self.sp.turn, self.sp.enable = speed, turn, int(enable)
            o = self.lib.balance_step(ctypes.byref(self.st), ctypes.byref(self.cfg),
                                      ctypes.byref(self.imu), ws, ctypes.byref(self.sp), dt)
        else:
            self.sp.speed, self.sp.turn, self.sp.enable = speed, turn, int(enable)
            o = bpy.step(self.st, self.cfg, gyro, acc_fwd, acc_up, fused, ws, self.sp, dt)
        return o.left, o.right, o.accel, o.pitch, o.pitch_target, o.state, o.motors_on
