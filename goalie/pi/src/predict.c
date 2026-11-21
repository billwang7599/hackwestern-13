/* Ball crossing-point predictor. Algorithm and constants: see predict.h. */
#include <math.h>
#include <string.h>
#include "predict.h"

void predict_init(predictor_t *p, double wall_half_w)
{
    memset(p, 0, sizeof *p);
    p->wall_half_w = wall_half_w;
}

void predict_reset(predictor_t *p)
{
    p->head = 0;
    p->count = 0;
}

static const pred_sample_t *newest(const predictor_t *p, int k)
{
    /* k = 0 -> newest, 1 -> one before, ... (k < count) */
    int i = (p->head - 1 - k) % PRED_RING;
    if (i < 0) i += PRED_RING;
    return &p->s[i];
}

void predict_add(predictor_t *p, double t_s, double x, double y)
{
    if (!isfinite(t_s) || !isfinite(x) || !isfinite(y)) return;

    if (p->count > 0) {
        const pred_sample_t *last = newest(p, 0);
        double dt = t_s - last->t;
        if (dt == 0.0) return;                              /* duplicate frame */
        double jump = hypot(x - last->x, y - last->y);
        if (dt < 0.0 ||                                     /* clock went back */
            dt * 1000.0 > PRED_GAP_RESET_MS ||              /* lost for a while */
            jump > PRED_JUMP_RESET_M + PRED_MAX_SPEED * dt) /* teleported */
            predict_reset(p);
    }
    p->s[p->head] = (pred_sample_t){ t_s, x, y };
    p->head = (p->head + 1) % PRED_RING;
    if (p->count < PRED_RING) p->count++;
}

double predict_fold(double x, double w)
{
    if (w <= 0.0) return x;
    double period = 4.0 * w;
    double u = fmod(x + w, period);
    if (u < 0.0) u += period;
    if (u > 2.0 * w) u = period - u;
    return u - w;
}

pred_result_t predict_eval(const predictor_t *p)
{
    pred_result_t r;
    memset(&r, 0, sizeof r);
    r.state = PRED_NONE;
    if (p->count == 0) return r;

    /* Pick the window. */
    const double t_new = newest(p, 0)->t;
    int n = 0;
    double tm = 0, xm = 0, ym = 0;
    for (int k = 0; k < p->count && k < PRED_WINDOW; k++) {
        const pred_sample_t *s = newest(p, k);
        if ((t_new - s->t) * 1000.0 > PRED_MAX_SPAN_MS) break;
        tm += s->t; xm += s->x; ym += s->y;
        n++;
    }
    r.n = n;
    if (n < PRED_MIN_SAMPLES) return r;
    tm /= n; xm /= n; ym /= n;

    /* Least squares slope for x(t) and y(t). */
    double stt = 0, stx = 0, sty = 0;
    for (int k = 0; k < n; k++) {
        const pred_sample_t *s = newest(p, k);
        double dt = s->t - tm;
        stt += dt * dt;
        stx += dt * (s->x - xm);
        sty += dt * (s->y - ym);
    }
    if (stt < 1e-9) return r;
    r.vx = stx / stt;
    r.vy = sty / stt;
    r.x = xm + r.vx * (t_new - tm);
    r.y = ym + r.vy * (t_new - tm);

    if (r.vy >= PRED_VY_INCOMING) { r.state = PRED_AWAY; return r; }

    r.t_cross = r.y > 0.0 ? r.y / -r.vy : 0.0;
    if (r.t_cross > PRED_MAX_T_CROSS_S) { r.state = PRED_AWAY; return r; }

    double xr = predict_fold(r.x + r.vx * r.t_cross, p->wall_half_w);
    if (xr >  PRED_GOAL_HALF_W) xr =  PRED_GOAL_HALF_W;
    if (xr < -PRED_GOAL_HALF_W) xr = -PRED_GOAL_HALF_W;
    r.x_cross = xr;
    r.state = PRED_INCOMING;
    return r;
}

const char *predict_state_name(pred_state_t s)
{
    switch (s) {
    case PRED_AWAY:     return "AWAY";
    case PRED_INCOMING: return "INCOMING";
    default:            return "NONE";
    }
}
