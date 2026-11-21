/* Paddle position controllers. Algorithm and defaults: see control.h. */
#include <math.h>
#include <string.h>
#include "control.h"

#define DEG          (M_PI / 180.0)
#define RPM2RADS     (2.0 * M_PI / 60.0)

/* legacy pd constants (the original goalie_motor loop) */
#define PD_DUTY_MIN       0.15
#define PD_ERR_DEADBAND   0.004
#define PD_STILL_V        0.05
#define PD_CAM_LATENCY_S  0.040
#define PD_EXTRAP_MAX_S   0.100
#define PD_VEL_ALPHA      0.5
#define PD_VEL_MAX        5.0

static double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

void ctl_defaults(ctl_cfg_t *c)
{
    memset(c, 0, sizeof *c);
    c->kind = CTL_OBSERVER;
    c->rate = 200.0;
    c->geom = GEOM_PIVOT;
    c->paddle_len = 0.12;
    c->q_max = 75.0 * DEG;
    c->spool_r = 0.015;
    c->x_max = 0.11;
    c->kp = 3.0;
    c->kd = 0.10;
    c->db_comp = 0.12;
    c->min_duty = 0.06;
    c->uoff = 0.10;
    c->l1 = 0.5;
    c->l2 = 2.0;
    c->latency_est = 0.050;
    c->model_rpm = 150.0;
    c->model_tau = 0.04;
    c->pd_kp = 15.0;
    c->pd_kd = 0.40;
}

void ctl_init(controller_t *k, const ctl_cfg_t *c)
{
    memset(k, 0, sizeof *k);
    k->c = *c;
    if (c->geom == GEOM_PIVOT) {
        /* never ask for more than the arm can reach inside the angle limit */
        double reach = c->paddle_len * sin(c->q_max);
        if (k->c.x_max > reach) k->c.x_max = reach;
        k->q_lim = c->q_max + 10.0 * DEG;
    } else {
        k->q_lim = (k->c.x_max + 0.01) / c->spool_r;
    }
}

double ctl_q_of_x(const controller_t *k, double x)
{
    if (k->c.geom == GEOM_PIVOT) return asin(clampd(x / k->c.paddle_len, -1.0, 1.0));
    return x / k->c.spool_r;
}

double ctl_x_of_q(const controller_t *k, double q)
{
    if (k->c.geom == GEOM_PIVOT) return k->c.paddle_len * sin(q);
    return k->c.spool_r * q;
}

void ctl_on_paddle(controller_t *k, double x_meas, double t_s)
{
    /* pd bookkeeping (always kept, so --ctl can be compared on the same data) */
    if (k->pd_valid && t_s > k->pd_t && t_s - k->pd_t < 0.2) {
        double v = clampd((x_meas - k->pd_x) / (t_s - k->pd_t), -PD_VEL_MAX, PD_VEL_MAX);
        k->pd_v = PD_VEL_ALPHA * v + (1.0 - PD_VEL_ALPHA) * k->pd_v;
    } else if (!(k->pd_valid && t_s == k->pd_t)) {
        k->pd_v = 0.0;
    }
    k->pd_x = x_meas;
    k->pd_t = t_s;
    k->pd_valid = 1;

    /* observer correction */
    const ctl_cfg_t *c = &k->c;
    double qm = ctl_q_of_x(k, x_meas);
    if (!k->init) {
        k->q_est = qm;
        k->w_est = 0.0;
        for (int i = 0; i < CTL_HIST; i++) k->hist[i] = qm;
        k->init = 1;
        return;
    }
    double dt = 1.0 / c->rate;
    int kk = (int)lround(c->latency_est / dt);
    if (kk < 0) kk = 0;
    if (kk > CTL_HIST - 1) kk = CTL_HIST - 1;
    int idx = ((k->head - kk) % CTL_HIST + CTL_HIST) % CTL_HIST;
    double innov = qm - k->hist[idx];
    double corr = c->l1 * innov;
    k->q_est += corr;
    k->w_est += c->l2 * innov;
    for (int i = 0; i < CTL_HIST; i++) k->hist[i] += corr;
}

static double tick_observer(controller_t *k, double target_x)
{
    const ctl_cfg_t *c = &k->c;
    double dt = 1.0 / c->rate;
    double up = k->u_prev;
    double u_eff = fabs(up) < c->db_comp ? 0.0 : copysign(fmax(fabs(up) - c->uoff, 0.0), up);
    double w0 = c->model_rpm * RPM2RADS;
    k->w_est += (w0 * u_eff - k->w_est) * dt / c->model_tau;
    k->q_est += k->w_est * dt;
    if (fabs(k->q_est) > k->q_lim) {
        k->q_est = copysign(k->q_lim, k->q_est);
        k->w_est = 0.0;
    }
    k->head = (k->head + 1) % CTL_HIST;
    k->hist[k->head] = k->q_est;

    k->q_t = ctl_q_of_x(k, clampd(target_x, -c->x_max, c->x_max));
    double u = c->kp * (k->q_t - k->q_est) - c->kd * k->w_est;
    if (fabs(u) < c->min_duty) u = 0.0;
    else                       u += copysign(c->db_comp, u);
    u = clampd(u, -1.0, 1.0);
    k->u_prev = u;
    return u;
}

static double tick_pd(controller_t *k, double target_x, double age_s)
{
    const ctl_cfg_t *c = &k->c;
    if (!k->pd_valid) return 0.0;
    double tgt = clampd(target_x, -c->x_max, c->x_max);
    double ext = clampd(age_s + PD_CAM_LATENCY_S, 0.0, PD_EXTRAP_MAX_S);
    double est = k->pd_x + k->pd_v * ext;
    double err = tgt - est;
    double raw = c->pd_kp * err - c->pd_kd * k->pd_v;
    double duty = 0.0;
    if (fabs(err) < PD_ERR_DEADBAND && fabs(k->pd_v) < PD_STILL_V) {
        duty = 0.0;
    } else if (raw != 0.0) {
        double mag = fmin(fabs(raw), 1.0);
        duty = copysign(PD_DUTY_MIN + (1.0 - PD_DUTY_MIN) * mag, raw);
    }
    return clampd(duty, -1.0, 1.0);
}

double ctl_tick(controller_t *k, double target_x, double paddle_age_s)
{
    if (k->c.kind == CTL_PD) {
        double u = tick_pd(k, target_x, paddle_age_s);
        k->u_prev = u;
        return u;
    }
    return tick_observer(k, target_x);
}

void ctl_hold(controller_t *k)
{
    /* Motor stopped (S, brake): the model coasts down with duty 0. Run one
     * model step so the history keeps advancing at the tick rate. */
    k->u_prev = 0.0;
    if (k->c.kind == CTL_OBSERVER && k->init) {
        const ctl_cfg_t *c = &k->c;
        double dt = 1.0 / c->rate;
        k->w_est += (0.0 - k->w_est) * dt / c->model_tau;
        k->q_est += k->w_est * dt;
        k->head = (k->head + 1) % CTL_HIST;
        k->hist[k->head] = k->q_est;
    }
}

double ctl_x_est(const controller_t *k)
{
    if (k->c.kind == CTL_PD) return k->pd_x;
    return ctl_x_of_q(k, k->q_est);
}

const char *ctl_kind_name(ctl_kind_t kind) { return kind == CTL_PD ? "pd" : "observer"; }

void ctl_set_gains(controller_t *k, double kp, double kd)
{
    if (k->c.kind == CTL_PD) { k->c.pd_kp = kp; k->c.pd_kd = kd; }
    else                     { k->c.kp = kp;    k->c.kd = kd; }
}

void ctl_get_gains(const controller_t *k, double *kp, double *kd)
{
    if (k->c.kind == CTL_PD) { *kp = k->c.pd_kp; *kd = k->c.pd_kd; }
    else                     { *kp = k->c.kp;    *kd = k->c.kd; }
}
