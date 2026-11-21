# balance.py - MicroPython port of balance/balance.c (same math, same names).
#
# Runs on a Raspberry Pi Pico / Pico 2 under MicroPython, and also under
# CPython (the simulator uses it as its pure-Python fallback, and
# sim/check_parity.py checks it against the C library).
#
# Conventions: see balance.h. pitch + = leaning forward, speeds in m/s,
# turn in rad/s (+ = left). Output: left/right wheel ground speed in m/s.
#
# Pico usage sketch (400 Hz):
#   import balance, time
#   cfg = balance.Config(); st = balance.State(); sp = balance.Setpoint()
#   sp.enable = 1
#   t_prev = time.ticks_us()
#   while True:
#       t = time.ticks_us(); dt = time.ticks_diff(t, t_prev) * 1e-6; t_prev = t
#       gy, af, au = read_imu()                    # rad/s, any accel unit
#       cmd = balance.step(st, cfg, gy, af, au, 0.0, None, sp, dt)
#       set_step_rates(cmd.left, cmd.right, cmd.motors_on)
#       sleep_until_next_tick()

import math

DEG2RAD = 0.017453292519943295

DISABLED = 0
WAIT_UPRIGHT = 1
RUNNING = 2
FALLEN = 3
STATE_NAMES = ("DISABLED", "WAIT_UPRIGHT", "RUNNING", "FALLEN")


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class Config:
    # Defaults must match balance_default_cfg() in balance.c.
    def __init__(self):
        self.kp = 84.0
        self.ki = 0.0
        self.kd = 22.0
        self.i_limit = 2.0

        self.speed_kp = 0.26
        self.speed_ki = 0.02
        self.speed_i_limit = 12.0 * DEG2RAD
        self.max_lean = 12.0 * DEG2RAD
        self.outer_hz = 50.0
        self.speed_lpf_tau = 0.05

        self.max_accel = 12.0
        self.max_speed = 1.5
        self.max_turn_speed = 0.3
        self.setpoint_accel = 0.8
        self.track_width = 0.20

        self.comp_tau = 0.75
        self.pitch_offset = 0.0
        self.gyro_bias_rate = 0.5
        self.accel_comp = 1.0
        self.use_fused_pitch = 0

        self.tilt_cutoff = 40.0 * DEG2RAD
        self.rearm_angle = 3.0 * DEG2RAD
        self.rearm_time = 0.5


class Setpoint:
    def __init__(self, speed=0.0, turn=0.0, enable=0):
        self.speed = speed
        self.turn = turn
        self.enable = enable


class Cmd:
    def __init__(self):
        self.left = 0.0
        self.right = 0.0
        self.accel = 0.0
        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.pitch_target = 0.0
        self.state = DISABLED
        self.motors_on = 0


class State:
    def __init__(self):
        self.state = DISABLED
        self.initialized = 0
        self.pitch = 0.0
        self.gyro_bias = 0.0
        self.upright_time = 0.0
        reset_loops(self)


def reset_loops(s):
    s.i_inner = 0.0
    s.i_outer = 0.0
    s.pitch_target = 0.0
    s.vel = 0.0
    s.speed_f = 0.0
    s.speed_sp = 0.0
    s.outer_acc = 0.0
    s.last_accel = 0.0


def set_inner_gains(s, c, kp, ki, kd):
    c.kp = kp
    c.ki = ki
    c.kd = kd
    s.i_inner = 0.0


def step(s, c, gyro_pitch, acc_fwd, acc_up, fused_pitch, wheel_speed, sp, dt):
    """One control tick. wheel_speed=None (or NaN) -> open-loop stepper mode."""
    out = Cmd()

    if not (dt > 0.0):
        dt = 1e-4
    if dt > 0.05:
        dt = 0.05

    # ---------- estimator ----------
    rate = gyro_pitch - s.gyro_bias
    if c.use_fused_pitch:
        s.pitch = fused_pitch
    else:
        af = acc_fwd - c.accel_comp * s.last_accel
        acc_pitch = math.atan2(-af, acc_up)
        if not s.initialized:
            s.pitch = acc_pitch
        else:
            a = c.comp_tau / (c.comp_tau + dt)
            s.pitch = a * (s.pitch + rate * dt) + (1.0 - a) * acc_pitch
    s.initialized = 1
    pitch = s.pitch - c.pitch_offset

    out.pitch = pitch
    out.pitch_rate = rate

    # ---------- state machine ----------
    if not sp.enable:
        if c.gyro_bias_rate > 0.0 and abs(rate) < 0.1:
            k = _clamp(c.gyro_bias_rate * dt, 0.0, 1.0)
            s.gyro_bias += k * (gyro_pitch - s.gyro_bias)
        s.state = DISABLED
        s.upright_time = 0.0
        reset_loops(s)
        out.state = s.state
        return out

    if s.state == RUNNING and abs(pitch) > c.tilt_cutoff:
        s.state = FALLEN
        s.upright_time = 0.0
        reset_loops(s)

    if s.state != RUNNING:
        if s.state == DISABLED:
            s.state = WAIT_UPRIGHT
        if abs(pitch) < c.rearm_angle:
            s.upright_time += dt
            if s.upright_time >= c.rearm_time:
                s.state = RUNNING
                reset_loops(s)
        else:
            s.upright_time = 0.0
        if s.state != RUNNING:
            out.state = s.state
            return out

    # ---------- outer loop: speed PI -> pitch target ----------
    sp_speed = _clamp(sp.speed, -c.max_speed, c.max_speed)
    max_dsp = c.setpoint_accel * dt
    s.speed_sp += _clamp(sp_speed - s.speed_sp, -max_dsp, max_dsp)

    meas = s.vel if (wheel_speed is None or wheel_speed != wheel_speed) else wheel_speed
    s.speed_f += (meas - s.speed_f) * (dt / (c.speed_lpf_tau + dt))

    s.outer_acc += dt
    if c.outer_hz <= 0.0 or s.outer_acc >= 1.0 / c.outer_hz:
        odt = s.outer_acc
        s.outer_acc = 0.0
        e = s.speed_sp - s.speed_f
        i_new = _clamp(s.i_outer + c.speed_ki * e * odt, -c.speed_i_limit, c.speed_i_limit)
        t = c.speed_kp * e + i_new
        tc = _clamp(t, -c.max_lean, c.max_lean)
        if t == tc or (t > tc and i_new < s.i_outer) or (t < tc and i_new > s.i_outer):
            s.i_outer = i_new
        s.pitch_target = tc

    # ---------- inner loop: pitch PID -> wheel acceleration ----------
    e = pitch - s.pitch_target
    i_new = _clamp(s.i_inner + c.ki * e * dt, -c.i_limit, c.i_limit)
    a = c.kp * e + i_new + c.kd * rate
    ac = _clamp(a, -c.max_accel, c.max_accel)
    if a == ac or (a > ac and i_new < s.i_inner) or (a < ac and i_new > s.i_inner):
        s.i_inner = i_new

    vc = _clamp(s.vel + ac * dt, -c.max_speed, c.max_speed)
    s.last_accel = (vc - s.vel) / dt
    s.vel = vc

    # ---------- differential turn ----------
    diff = _clamp(sp.turn * 0.5 * c.track_width, -c.max_turn_speed, c.max_turn_speed)

    out.left = s.vel - diff
    out.right = s.vel + diff
    out.accel = ac
    out.pitch_target = s.pitch_target
    out.state = s.state
    out.motors_on = 1
    return out
