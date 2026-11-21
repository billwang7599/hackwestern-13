/* Ball crossing-point predictor (C side of goalie/sim/predict.py).
 *
 * Coordinates per goalie/docs/architecture.md: metres, origin at the centre of
 * the goal mouth on the goal line, +x right, +y out into the field, incoming
 * ball has vy < 0.
 *
 * Algorithm (keep in sync with sim/predict.py; constants below):
 *  1. predict_add() pushes (t, x, y) samples into a ring buffer, using the
 *     LAPTOP's frame timestamp (seconds), so network jitter doesn't skew speed.
 *     The track is reset (buffer cleared, this sample kept) when:
 *       - t goes backwards (laptop restarted)          [t == last t: ignored]
 *       - gap since the previous sample > PRED_GAP_RESET_MS
 *       - the sample jumps more than PRED_JUMP_RESET_M + PRED_MAX_SPEED*dt
 *         from the previous one (ball picked up / new shot / bad detection)
 *  2. predict_eval() takes the newest <= PRED_WINDOW samples that are no older
 *     than PRED_MAX_SPAN_MS before the newest, and least-squares fits
 *     x(t) = x0 + vx*t and y(t) = y0 + vy*t (independent 1-D fits).
 *     Fewer than PRED_MIN_SAMPLES (or zero time spread) -> PRED_NONE.
 *  3. Position "now" = fit evaluated at the newest sample time.
 *     vy >= PRED_VY_INCOMING (-0.05 m/s)  -> PRED_AWAY (slow/still/leaving).
 *     t_cross = y_now / -vy (0 if y_now <= 0); t_cross > PRED_MAX_T_CROSS_S
 *     -> PRED_AWAY.
 *  4. x_raw = x_now + vx * t_cross. If side walls are enabled
 *     (wall_half_w > 0), reflect x_raw into [-wall_half_w, +wall_half_w]
 *     (triangle-wave unfold = any number of elastic bounces).
 *  5. x_cross = clamp(x_raw, -PRED_GOAL_HALF_W, +PRED_GOAL_HALF_W) -> PRED_INCOMING.
 *
 * What the caller does with the states is policy (see main.c):
 *  INCOMING -> paddle target = x_cross;  AWAY -> park centre;
 *  NONE -> hold the last target; ball unseen for a while -> park centre.
 */
#ifndef GOALIE_PREDICT_H
#define GOALIE_PREDICT_H

#define PRED_GOAL_HALF_W     0.11   /* m, goal spans [-0.11, +0.11]                 */
#define PRED_RING            32     /* ring buffer capacity (samples)               */
#define PRED_WINDOW          6      /* max samples used in the fit (~100 ms @60 fps) */
#define PRED_MIN_SAMPLES     3      /* fewer -> PRED_NONE                           */
#define PRED_MAX_SPAN_MS     250    /* ignore samples older than this vs newest     */
#define PRED_GAP_RESET_MS    200    /* bigger gap between samples -> new track      */
#define PRED_JUMP_RESET_M    0.08   /* + PRED_MAX_SPEED*dt: bigger jump -> new track */
#define PRED_MAX_SPEED       4.0    /* m/s, plausibility bound for the jump test     */
#define PRED_VY_INCOMING    -0.05   /* m/s, vy below this = moving towards the goal  */
#define PRED_MAX_T_CROSS_S   2.0    /* s, further away in time -> treat as AWAY      */
#define PRED_WALL_HALF_W_DEF 0.0    /* m, side walls; 0 = none (contract has no field width yet) */

typedef enum { PRED_NONE = 0, PRED_AWAY = 1, PRED_INCOMING = 2 } pred_state_t;

typedef struct { double t, x, y; } pred_sample_t;

typedef struct {
    pred_sample_t s[PRED_RING];
    int head;            /* index of the next write */
    int count;           /* valid samples, <= PRED_RING */
    double wall_half_w;  /* 0 = no wall reflections */
} predictor_t;

typedef struct {
    pred_state_t state;
    int    n;            /* samples used in the fit */
    double x, y;         /* fitted position at the newest sample time */
    double vx, vy;       /* fitted velocity, m/s */
    double t_cross;      /* s from the newest sample until y = 0 (INCOMING only) */
    double x_cross;      /* clamped crossing x (INCOMING only) */
} pred_result_t;

void          predict_init(predictor_t *p, double wall_half_w);
void          predict_reset(predictor_t *p);
void          predict_add(predictor_t *p, double t_s, double x, double y);
pred_result_t predict_eval(const predictor_t *p);
const char   *predict_state_name(pred_state_t s);

/* Reflect x into [-w, w] as if bouncing elastically between walls at +-w. */
double        predict_fold(double x, double w);

#endif
