/* Paddle position controllers (C port of goalie/sim/controller.py).
 *
 * Two controllers, picked with --ctl:
 *
 *  observer (default)  The sim's model-based observer + PD. The motor has no
 *      encoder and the camera sees the paddle ~50 ms late, so a PD loop on
 *      camera data overshoots by a whole goal width. Instead we run a small
 *      motor model at 200 Hz from the duties we sent, PD-control the MODEL,
 *      and nudge it towards the camera on each frame, comparing the camera
 *      with what the model said LATENCY_EST ago (Smith-predictor style).
 *      Units: shaft angle q (rad); gains are duty per rad.
 *
 *  pd                  The original goalie_motor loop: PD in metres on the
 *      camera paddle position, extrapolated by its age + 40 ms. Kept for
 *      comparison on the bench (--ctl pd).
 *
 * Geometry (architecture.md):
 *   pivot: x = L*sin(q),  q = asin(clamp(x/L, -1, 1))     L = paddle length
 *   rail:  x = R*q,       q = x/R                          R = spool radius
 *
 * Observer algorithm, every tick (dt = 5 ms)          [= controller.py tick()]
 *   1. u_eff = |u_prev| < DB_COMP ? 0 : sign(u_prev)*max(|u_prev| - UOFF, 0)
 *      w += (W0*u_eff - w)*dt/TAU   (W0 = MODEL_RPM*2pi/60);  q += w*dt
 *      clamp q to +-Q_LIM (pivot: q_max + 10 deg) and zero w if clamped;
 *      push q into hist[64].
 *   2. q_t = q_of_x(clamp(target, -X_MAX, X_MAX))
 *      u = KP*(q_t - q) - KD*w;  |u| < MIN_DUTY -> 0, else u += sign(u)*DB_COMP
 *      u = clamp(u, -1, 1); u_prev = u.
 * On each camera frame with the paddle seen        [= controller.py on_paddle()]
 *   first frame: q = q_m, w = 0, hist[] = q_m.
 *   else: k = round(LATENCY_EST/dt) (<= 63); innov = q_m - hist[k ticks ago]
 *         q += L1*innov; w += L2*innov; hist[] += L1*innov.
 * ctl_hold(): the caller sent S (no feedback), so the model sees duty 0.
 */
#ifndef GOALIE_CONTROL_H
#define GOALIE_CONTROL_H

#define CTL_HIST 64

typedef enum { CTL_OBSERVER = 0, CTL_PD = 1 } ctl_kind_t;
typedef enum { GEOM_PIVOT = 0, GEOM_RAIL = 1 } geom_t;

typedef struct {
    ctl_kind_t kind;
    double rate;           /* Hz, 200 */
    /* geometry */
    geom_t geom;
    double paddle_len;     /* pivot arm length, m (0.12) */
    double q_max;          /* pivot software angle limit, rad (75 deg) */
    double spool_r;        /* rail spool radius, m (0.015) */
    double x_max;          /* targets are clamped to +-x_max, m */
    /* observer (sim ControlParams defaults) */
    double kp, kd;         /* duty/rad, duty/(rad/s): 3.0, 0.10 */
    double db_comp;        /* 0.12 */
    double min_duty;       /* 0.06 */
    double uoff;           /* 0.10 duty eaten by gearbox friction */
    double l1, l2;         /* 0.5, 2.0 */
    double latency_est;    /* s, 0.05 */
    double model_rpm;      /* 150 (MEASURE: docs/rpm_test.md) */
    double model_tau;      /* s, 0.04 */
    /* pd (legacy) */
    double pd_kp, pd_kd;   /* duty/m, duty/(m/s): 15, 0.40 */
} ctl_cfg_t;

typedef struct {
    ctl_cfg_t c;
    double q_lim;
    /* observer state */
    double hist[CTL_HIST];
    int head;
    double q_est, w_est, u_prev, q_t;
    int init;
    /* pd state */
    int pd_valid;
    double pd_x, pd_v, pd_t;   /* last paddle x, filtered velocity, laptop time (s) */
    double pd_age_s;           /* set by the caller each tick: age of pd_x */
} controller_t;

void   ctl_defaults(ctl_cfg_t *c);
void   ctl_init(controller_t *k, const ctl_cfg_t *c);
/* Camera frame with paddle seen. t_s = laptop frame time (pd uses it). */
void   ctl_on_paddle(controller_t *k, double x_meas, double t_s);
/* One 200 Hz tick; returns duty in [-1, 1]. paddle_age_s: Pi-time age of the
 * last paddle sighting (pd extrapolates by it; observer ignores it). */
double ctl_tick(controller_t *k, double target_x, double paddle_age_s);
/* The caller stopped the motor (S) instead of sending this tick's duty. */
void   ctl_hold(controller_t *k);
/* Paddle x the controller believes now (observer: model; pd: last measured). */
double ctl_x_est(const controller_t *k);
double ctl_q_of_x(const controller_t *k, double x);
double ctl_x_of_q(const controller_t *k, double q);
const char *ctl_kind_name(ctl_kind_t k);
/* Live tuning (GAINS): sets the active controller's kp/kd. */
void   ctl_set_gains(controller_t *k, double kp, double kd);
void   ctl_get_gains(const controller_t *k, double *kp, double *kd);

#endif
