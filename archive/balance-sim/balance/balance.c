/* balance.c - see balance.h. Keep in sync with micropython/balance.py. */
#include "balance.h"

#include <math.h>

#define DEG2RAD 0.017453292519943295f

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

void balance_default_cfg(balance_cfg_t *c)
{
    /* Defaults come from sim/tune.py on the default robot in sim/params.py
     * (1.3 kg body, CoM 25 cm above the axle, 90 mm wheels, NEMA17).
     * Starting points only: re-tune on the real robot (docs/balance.md). */
    c->kp            = 84.0f;
    c->ki            = 0.0f;
    c->kd            = 22.0f;
    c->i_limit       = 2.0f;

    c->speed_kp      = 0.26f;
    c->speed_ki      = 0.02f;
    c->speed_i_limit = 12.0f * DEG2RAD;
    c->max_lean      = 12.0f * DEG2RAD;
    c->outer_hz      = 50.0f;
    c->speed_lpf_tau = 0.05f;

    c->max_accel      = 12.0f;
    c->max_speed      = 1.5f;
    c->max_turn_speed = 0.3f;
    c->setpoint_accel = 0.8f;
    c->track_width    = 0.20f;

    c->comp_tau        = 0.75f;
    c->pitch_offset    = 0.0f;
    c->gyro_bias_rate  = 0.5f;
    c->accel_comp      = 1.0f;
    c->use_fused_pitch = 0;

    c->tilt_cutoff = 40.0f * DEG2RAD;
    c->rearm_angle = 3.0f * DEG2RAD;
    c->rearm_time  = 0.5f;
}

static void reset_loops(balance_state_t *s)
{
    s->i_inner = 0.0f;
    s->i_outer = 0.0f;
    s->pitch_target = 0.0f;
    s->vel = 0.0f;
    s->speed_f = 0.0f;
    s->speed_sp = 0.0f;
    s->outer_acc = 0.0f;
    s->last_accel = 0.0f;
}

void balance_init(balance_state_t *s)
{
    s->state = BAL_DISABLED;
    s->initialized = 0;
    s->pitch = 0.0f;
    s->gyro_bias = 0.0f;
    s->upright_time = 0.0f;
    reset_loops(s);
}

void balance_set_inner_gains(balance_state_t *s, balance_cfg_t *cfg,
                             float kp, float ki, float kd)
{
    cfg->kp = kp;
    cfg->ki = ki;
    cfg->kd = kd;
    s->i_inner = 0.0f;
}

const char *balance_state_name(int st)
{
    switch (st) {
    case BAL_DISABLED:     return "DISABLED";
    case BAL_WAIT_UPRIGHT: return "WAIT_UPRIGHT";
    case BAL_RUNNING:      return "RUNNING";
    case BAL_FALLEN:       return "FALLEN";
    default:               return "?";
    }
}

balance_cmd_t balance_step(balance_state_t *s, const balance_cfg_t *c,
                           const balance_imu_t *imu, float wheel_speed,
                           const balance_setpoint_t *sp, float dt)
{
    balance_cmd_t out = {0};

    if (!(dt > 0.0f)) dt = 1e-4f;          /* also catches NaN */
    if (dt > 0.05f) dt = 0.05f;

    /* ---------- estimator ---------- */
    float rate = imu->gyro_pitch - s->gyro_bias;
    if (c->use_fused_pitch) {
        s->pitch = imu->fused_pitch;
    } else {
        float af = imu->acc_fwd - c->accel_comp * s->last_accel;
        float acc_pitch = atan2f(-af, imu->acc_up);
        if (!s->initialized) {
            s->pitch = acc_pitch;
        } else {
            float a = c->comp_tau / (c->comp_tau + dt);
            s->pitch = a * (s->pitch + rate * dt) + (1.0f - a) * acc_pitch;
        }
    }
    s->initialized = 1;
    float pitch = s->pitch - c->pitch_offset;

    out.pitch = pitch;
    out.pitch_rate = rate;

    /* ---------- state machine ---------- */
    if (!sp->enable) {
        /* Sitting still on the kickstand: slowly learn the gyro bias. */
        if (c->gyro_bias_rate > 0.0f && fabsf(rate) < 0.1f) {
            float k = clampf(c->gyro_bias_rate * dt, 0.0f, 1.0f);
            s->gyro_bias += k * (imu->gyro_pitch - s->gyro_bias);
        }
        s->state = BAL_DISABLED;
        s->upright_time = 0.0f;
        reset_loops(s);
        out.state = s->state;
        return out;                      /* motors_on = 0, speeds 0 */
    }

    if (s->state == BAL_RUNNING && fabsf(pitch) > c->tilt_cutoff) {
        s->state = BAL_FALLEN;
        s->upright_time = 0.0f;
        reset_loops(s);
    }

    if (s->state != BAL_RUNNING) {
        if (s->state == BAL_DISABLED) s->state = BAL_WAIT_UPRIGHT;
        if (fabsf(pitch) < c->rearm_angle) {
            s->upright_time += dt;
            if (s->upright_time >= c->rearm_time) {
                s->state = BAL_RUNNING;
                reset_loops(s);
            }
        } else {
            s->upright_time = 0.0f;
        }
        if (s->state != BAL_RUNNING) {
            out.state = s->state;
            return out;
        }
    }

    /* ---------- outer loop: speed PI -> pitch target ---------- */
    float sp_speed = clampf(sp->speed, -c->max_speed, c->max_speed);
    float max_dsp = c->setpoint_accel * dt;
    s->speed_sp += clampf(sp_speed - s->speed_sp, -max_dsp, max_dsp);

    float meas = isnan(wheel_speed) ? s->vel : wheel_speed;
    s->speed_f += (meas - s->speed_f) * (dt / (c->speed_lpf_tau + dt));

    s->outer_acc += dt;
    if (c->outer_hz <= 0.0f || s->outer_acc >= 1.0f / c->outer_hz) {
        float odt = s->outer_acc;
        s->outer_acc = 0.0f;
        float e = s->speed_sp - s->speed_f;
        float i_new = clampf(s->i_outer + c->speed_ki * e * odt,
                             -c->speed_i_limit, c->speed_i_limit);
        float t = c->speed_kp * e + i_new;
        float tc = clampf(t, -c->max_lean, c->max_lean);
        /* anti-windup: only accept the integral if not saturated further */
        if (t == tc || (t > tc && i_new < s->i_outer) || (t < tc && i_new > s->i_outer))
            s->i_outer = i_new;
        s->pitch_target = tc;
    }

    /* ---------- inner loop: pitch PID -> wheel acceleration ---------- */
    float e = pitch - s->pitch_target;
    float i_new = clampf(s->i_inner + c->ki * e * dt, -c->i_limit, c->i_limit);
    float a = c->kp * e + i_new + c->kd * rate;   /* D on measurement */
    float ac = clampf(a, -c->max_accel, c->max_accel);
    if (a == ac || (a > ac && i_new < s->i_inner) || (a < ac && i_new > s->i_inner))
        s->i_inner = i_new;

    float v = s->vel + ac * dt;
    float vc = clampf(v, -c->max_speed, c->max_speed);
    s->last_accel = (vc - s->vel) / dt;   /* what the wheels actually got asked */
    s->vel = vc;

    /* ---------- differential turn ---------- */
    float diff = clampf(sp->turn * 0.5f * c->track_width,
                        -c->max_turn_speed, c->max_turn_speed);

    out.left = s->vel - diff;
    out.right = s->vel + diff;
    out.accel = ac;
    out.pitch_target = s->pitch_target;
    out.state = s->state;
    out.motors_on = 1;
    return out;
}
