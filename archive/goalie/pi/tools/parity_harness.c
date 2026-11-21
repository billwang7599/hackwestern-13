/* Replays a recorded sim/ control-loop event stream through the C predictor +
 * controller (the exact code goalie_motor runs) and prints target/duty per tick.
 * Driven by tools/parity.py; not part of the server.
 *
 * stdin lines:
 *   C <rpm> <tau> <latency_s> <wall_half_w|0> <geom 0=pivot 1=rail> <paddle_len> <spool_r>
 *   B <t_cap> <x> <y> <t_now>     ball seen (camera frame arriving at t_now)
 *   P <px> <t_cap>                paddle seen
 *   T <t_now>                     control tick -> prints "<target> <duty> <x_est>"
 */
#include <stdio.h>
#include <string.h>
#include "../src/control.h"
#include "../src/predict.h"

#define UNSEEN_S 0.25

int main(void)
{
    char line[256];
    ctl_cfg_t cfg;
    controller_t k;
    predictor_t p;
    pred_result_t r;
    double t_last_seen = -1e9, x_lim = 0.11;
    int have_cfg = 0;
    memset(&r, 0, sizeof r);

    while (fgets(line, sizeof line, stdin)) {
        double a, b, c, d, e, f, g;
        switch (line[0]) {
        case 'C':
            if (sscanf(line + 1, "%lf %lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f, &g) != 7) return 2;
            ctl_defaults(&cfg);
            cfg.model_rpm = a; cfg.model_tau = b; cfg.latency_est = c;
            cfg.geom = e > 0.5 ? GEOM_RAIL : GEOM_PIVOT;
            cfg.paddle_len = f; cfg.spool_r = g;
            ctl_init(&k, &cfg);
            x_lim = k.c.x_max < PRED_GOAL_HALF_W ? k.c.x_max : PRED_GOAL_HALF_W;
            predict_init(&p, d);
            memset(&r, 0, sizeof r);
            t_last_seen = -1e9;
            have_cfg = 1;
            break;
        case 'B':
            if (!have_cfg || sscanf(line + 1, "%lf %lf %lf %lf", &a, &b, &c, &d) != 4) return 2;
            predict_add(&p, a, b, c);
            r = predict_eval(&p);
            t_last_seen = d;
            break;
        case 'P':
            if (!have_cfg || sscanf(line + 1, "%lf %lf", &a, &b) != 2) return 2;
            ctl_on_paddle(&k, a, b);
            break;
        case 'T': {
            if (!have_cfg || sscanf(line + 1, "%lf", &a) != 1) return 2;
            if (a - t_last_seen > UNSEEN_S) {   /* = main.c BALL_LOST_MS park */
                predict_reset(&p);
                memset(&r, 0, sizeof r);
            }
            double tgt = predict_target(&r, x_lim);
            double u = ctl_tick(&k, tgt, 0.0);
            printf("%.17g %.17g %.17g\n", tgt, u, ctl_x_est(&k));
            break;
        }
        default:
            break;
        }
    }
    return 0;
}
