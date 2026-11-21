/* Ball crossing-point predictor (C port of goalie/sim/predict.py).
 *
 * Coordinates per goalie/docs/architecture.md: metres, origin at the centre of
 * the goal mouth on the goal line, +x right, +y out into the field, incoming
 * ball has vy < 0.
 *
 * Algorithm (kept in sync with sim/predict.py; tools/parity.py checks it):
 *  1. predict_add() pushes (t, x, y) samples into a ring buffer, using the
 *     LAPTOP's frame timestamp (seconds), so network jitter doesn't skew speed.
 *     The track is reset (buffer cleared, this sample kept) when:
 *       - gap since the previous sample > PRED_GAP_RESET_MS (150 ms, sim STALE_GAP)
 *       - t goes backwards (laptop restarted)          [t == last t: ignored]
 *       - C only: the sample jumps more than PRED_JUMP_RESET_M + PRED_MAX_SPEED*dt
 *         from the previous one (ball picked up / bad detection). Never fires
 *         on a real roll (< 4 m/s), so it doesn't change the sim behaviour.
 *  2. predict_eval() takes the newest <= PRED_WINDOW (5) samples that are no
 *     older than PRED_MAX_SPAN_MS before the newest, and least-squares fits
 *     x(t) = x0 + vx*t and y(t) = y0 + vy*t (independent 1-D fits).
 *     Fewer than PRED_MIN_SAMPLES (or zero time spread) -> PRED_NONE.
 *  3. Position "now" = fit evaluated at the newest sample time.
 *     vy >= PRED_VY_INCOMING (-0.15 m/s)  -> PRED_AWAY (slow/still/leaving;
 *     a still ball's 2 mm noise fits to |vy| ~0.05 m/s, so -0.05 was too tight).
 *     t_cross = y_now / -vy (0 if y_now <= 0); t_cross > PRED_MAX_T_CROSS_S
 *     -> PRED_AWAY.
 *  4. x_raw = x_now + vx * t_cross. If side walls are enabled
 *     (wall_half_w > 0, the wall position, contract: 0.25 m), reflect x_raw
 *     into [-(w - PRED_BALL_R), +(w - PRED_BALL_R)] (the ball centre bounces
 *     one radius from the wall; triangle wave = any number of bounces).
 *  5. x_cross = raw crossing; the caller clamps it -> PRED_INCOMING.
 *
 * Target policy (predict_target(), = sim Predictor._recompute):
 *  INCOMING -> clamp(x_cross);  AWAY -> clamp(SHADOW_GAIN * x_now);
 *  NONE with samples -> clamp(SHADOW_GAIN * x_newest);  no samples -> 0.
 *  clamp = +-x_lim, x_lim = min(goal half width, paddle reach).
 *  The caller parks at 0 when the ball has been unseen > 250 ms.
 */
#ifndef GOALIE_PREDICT_H
#define GOALIE_PREDICT_H

#define PRED_GOAL_HALF_W     0.11   /* m, goal spans [-0.11, +0.11]                 */
#define PRED_RING            32     /* ring buffer capacity (samples)               */
#define PRED_WINDOW          5      /* max samples used in the fit (sim WINDOW)      */
#define PRED_MIN_SAMPLES     3      /* fewer -> PRED_NONE                           */
#define PRED_MAX_SPAN_MS     250    /* ignore samples older than this vs newest     */
#define PRED_GAP_RESET_MS    150    /* bigger gap between samples -> new track (sim STALE_GAP) */
#define PRED_JUMP_RESET_M    0.08   /* + PRED_MAX_SPEED*dt: bigger jump -> new track */
#define PRED_MAX_SPEED       4.0    /* m/s, plausibility bound for the jump test     */
#define PRED_VY_INCOMING    -0.15   /* m/s, vy below this = moving towards the goal (sim VY_IN) */
#define PRED_MAX_T_CROSS_S   2.0    /* s, further away in time -> treat as AWAY      */
#define PRED_WALL_HALF_W_DEF 0.0    /* m, side walls off by default; contract walls at 0.25 */
#define PRED_BALL_R          0.02   /* m, ball radius (40 mm ball), wall fold margin  */
#define PRED_SHADOW_GAIN     0.5    /* not incoming: target = 0.5 * ball x (sim)       */

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
    double x_cross;      /* raw crossing x, after wall folds (INCOMING only) */
    double x_last;       /* newest sample x (valid if count > 0) */
    int    count;        /* samples in the buffer */
} pred_result_t;

void          predict_init(predictor_t *p, double wall_half_w);
void          predict_reset(predictor_t *p);
void          predict_add(predictor_t *p, double t_s, double x, double y);
pred_result_t predict_eval(const predictor_t *p);
const char   *predict_state_name(pred_state_t s);

/* Paddle target for a result (policy above), clamped to +-x_lim. */
double        predict_target(const pred_result_t *r, double x_lim);

/* Reflect x into [-w, w] as if bouncing elastically between walls at +-w. */
double        predict_fold(double x, double w);

#endif
