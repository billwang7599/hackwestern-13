/* balance.h - portable self-balancing controller for a two-wheeled robot.
 *
 * C99, no OS dependencies, no malloc, single-precision float only (fast on
 * Pico 2 / RP2350 FPU, Pi 5, anything). Call balance_step() at a fixed-ish
 * rate (200-500 Hz) from your control loop; pass the real elapsed dt.
 *
 * Conventions (map your IMU axes to these!):
 *   pitch        rad, + = body leaning FORWARD (top ahead of the axle)
 *   gyro_pitch   rad/s, + = pitching forward
 *   acc_fwd      specific force along the body's forward axis, m/s^2 (any
 *                unit works if accel_comp = 0). Robot tilted forward at rest
 *                -> acc_fwd NEGATIVE (MPU6050 reads +1 g on the axis pointing up)
 *   acc_up       specific force along the body's up axis (~ +1 g upright)
 *   speed        m/s of ground travel, + = forward
 *   turn         rad/s yaw, + = turn LEFT (counter-clockwise from above)
 *
 * Control structure (cascaded):
 *   outer  (outer_hz, ~50 Hz)  speed PI:  (speed_sp - speed)      -> pitch_target
 *   inner  (every call)        pitch PID: (pitch - pitch_target)  -> wheel accel
 *          derivative is taken on the measurement (gyro rate), not the error
 *   accel is integrated into a wheel velocity command (what a stepper wants);
 *   turn is added differentially on top.
 *
 * Output: left/right wheel ground speed in m/s. For steppers convert to a step
 * rate: steps_per_s = v / (2*pi*r) * steps_per_rev. For DC motors use
 * v / v_max as a duty feed-forward (or feed it to a wheel-speed loop).
 */
#ifndef BALANCE_H
#define BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    BAL_DISABLED = 0,   /* enable == 0: motors off, learning gyro bias      */
    BAL_WAIT_UPRIGHT,   /* enabled, waiting to be held upright to arm       */
    BAL_RUNNING,        /* balancing                                        */
    BAL_FALLEN          /* tilt exceeded cutoff: motors off until re-armed  */
};

typedef struct {
    /* inner loop: pitch error [rad] -> wheel acceleration [m/s^2] */
    float kp;              /* m/s^2 per rad                                   */
    float ki;              /* m/s^2 per (rad*s)                               */
    float kd;              /* m/s^2 per rad/s  (applied to gyro rate)         */
    float i_limit;         /* clamp on the integral term, m/s^2               */

    /* outer loop: speed error [m/s] -> target pitch [rad] */
    float speed_kp;        /* rad per m/s                                     */
    float speed_ki;        /* rad per m  (integral of speed err = position)   */
    float speed_i_limit;   /* clamp on the outer integral, rad                */
    float max_lean;        /* clamp on pitch_target, rad                      */
    float outer_hz;        /* outer loop rate                                 */
    float speed_lpf_tau;   /* s, low-pass on measured speed                   */

    /* actuator limits */
    float max_accel;       /* m/s^2                                           */
    float max_speed;       /* m/s, per wheel before turn is added             */
    float max_turn_speed;  /* m/s, max differential added per wheel           */
    float setpoint_accel;  /* m/s^2 slew on the commanded speed setpoint      */
    float track_width;     /* m, distance between wheels (for turn)           */

    /* estimator */
    float comp_tau;        /* s, complementary filter crossover (0.5-2 s)     */
    float pitch_offset;    /* rad, subtracted from pitch (IMU mount trim)     */
    float gyro_bias_rate;  /* 1/s, bias learning speed while DISABLED, 0=off  */
    float accel_comp;      /* 0..1: subtract our own last wheel accel command
                              from acc_fwd (acc_fwd must then be in m/s^2!) so
                              driving doesn't fool the accelerometer. 1 for
                              steppers, ~0.5-1 for DC, 0 = off              */
    int   use_fused_pitch; /* 1: trust imu.fused_pitch (BNO085), skip filter  */

    /* safety */
    float tilt_cutoff;     /* rad, |pitch| beyond this -> FALLEN, motors off  */
    float rearm_angle;     /* rad, |pitch| within this ...                    */
    float rearm_time;      /* s,  ... for this long -> RUNNING                */
} balance_cfg_t;

typedef struct {
    float gyro_pitch;      /* rad/s (raw, bias is removed internally)         */
    float acc_fwd;         /* see conventions above                           */
    float acc_up;
    float fused_pitch;     /* rad, only used when cfg.use_fused_pitch         */
} balance_imu_t;

typedef struct {
    float speed;           /* m/s forward                                     */
    float turn;            /* rad/s yaw, + = left                             */
    int   enable;          /* 0 = motors off                                  */
} balance_setpoint_t;

typedef struct {
    float left, right;     /* wheel ground speed commands, m/s                */
    float accel;           /* inner-loop acceleration command, m/s^2          */
    float pitch;           /* estimated pitch (offset removed), rad           */
    float pitch_rate;      /* bias-corrected gyro, rad/s                      */
    float pitch_target;    /* from the outer loop, rad                        */
    int   state;           /* BAL_*                                           */
    int   motors_on;       /* 0 -> de-energize / coast                        */
} balance_cmd_t;

typedef struct {
    int   state;
    int   initialized;     /* estimator has seen its first sample             */
    float pitch;           /* filtered pitch, offset NOT removed              */
    float gyro_bias;
    float i_inner;
    float i_outer;
    float pitch_target;
    float vel;             /* integrated wheel velocity command               */
    float speed_f;         /* filtered measured speed                         */
    float speed_sp;        /* slewed speed setpoint                           */
    float outer_acc;       /* time accumulated toward the next outer tick     */
    float upright_time;
    float last_accel;      /* previous accel command, for accel_comp          */
} balance_state_t;

void balance_default_cfg(balance_cfg_t *cfg);
void balance_init(balance_state_t *s);

/* One control tick.
 *   wheel_speed: measured ground speed in m/s. Pass NAN for open-loop
 *                steppers: the controller then uses its own velocity command
 *                (step count == commanded speed unless steps are lost).
 *   dt:          seconds since the previous call (clamped to (0, 0.05]).   */
balance_cmd_t balance_step(balance_state_t *s, const balance_cfg_t *cfg,
                           const balance_imu_t *imu, float wheel_speed,
                           const balance_setpoint_t *sp, float dt);

/* Live tuning helper (e.g. from a `T kp ki kd` command). Resets integrators. */
void balance_set_inner_gains(balance_state_t *s, balance_cfg_t *cfg,
                             float kp, float ki, float kd);

const char *balance_state_name(int state);

#ifdef __cplusplus
}
#endif
#endif /* BALANCE_H */
