/* goalie_motor: QNX real-time server for "Beat the Goalie".
 *
 * Protocol, coordinates and defaults: goalie/docs/architecture.md (the contract).
 *
 * TCP :5556, one command per line, exactly one reply line ("OK ..." / "ERR ..."):
 *   F <t_ms> <ball_seen> <bx> <by> <paddle_seen> <px>   camera frame -> "OK <target_x> <duty>"
 *   MODE QNX | MODE LINUX      real-time vs EMULATED Linux (see below)
 *   HUMAN <x> / AUTO           Switch-controller target / back to prediction
 *   GOAL / RESET / PING / STATUS
 * Extensions (not in the contract, safe to ignore):
 *   GAINS <kp> <kd>            live PD tuning on the bench -> "OK <kp> <kd>"
 *   SIMHIT <mag>               stub backend only: fake an Uno "HIT <mag>" line
 *
 * Threads:
 *   main      accept loop, one detached thread per client
 *   control   200 Hz, SCHED_RR (QNX mode), clock_nanosleep(TIMER_ABSTIME):
 *             Uno events -> watchdog -> target (prediction / human / park)
 *             -> PD on camera-measured paddle x -> backend->motor(duty)
 *
 * LINUX mode is an EMULATION for the demo: the control thread drops to
 * SCHED_OTHER and sleeps a random 0..jitter_ms on some ticks to mimic a
 * non-real-time kernel under load. It is not real Linux.
 *
 * Usage: goalie_motor [-b arduino|stub] [-d dev] [-p port] [-w wd_ms] [-r rt_prio]
 *                     [--kp K] [--kd K] [--jitter-ms MS] [--walls HALF_W_M]
 */
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "goalie.h"
#include "predict.h"

/* ---- control constants (keep in sync with sim/controller.py) ---- */
#define TICK_HZ             200
#define GOAL_HALF_W         0.11    /* m, paddle target is clamped to the goal mouth */
#define CTL_KP_DEF          15.0    /* duty per metre of error */
#define CTL_KD_DEF          0.40    /* duty per m/s of paddle velocity (derivative on measurement) */
#define CTL_DUTY_MAX        1.0
#define CTL_DUTY_MIN        0.15    /* deadband compensation: smallest duty that moves the motor */
#define CTL_ERR_DEADBAND    0.004   /* m: |err| below this and paddle still -> duty 0 */
#define CTL_STILL_V         0.05    /* m/s: "paddle still" for the deadband */
#define CTL_CAM_LATENCY_S   0.040   /* s: assumed camera+network latency, paddle is extrapolated */
#define CTL_EXTRAP_MAX_S    0.100   /* s: never extrapolate the paddle further than this */
#define PADDLE_VEL_ALPHA    0.5     /* low-pass on frame-to-frame paddle velocity */
#define PADDLE_VEL_MAX      5.0     /* m/s clamp on the velocity estimate */
#define PADDLE_STALE_MS     500     /* paddle unseen this long -> no feedback -> S */
#define BALL_LOST_MS        250     /* ball unseen this long -> park centre */

/* ---- save logic (contract) ---- */
#define SAVE_VY_IN         -0.05    /* m/s: shot in flight */
#define SAVE_VY_OUT         0.05    /* m/s: ball going back out */
#define SAVE_REVERSE_Y      0.10    /* m: reversal counts only this close to the goal line */
#define SAVE_REARM_Y        0.25    /* m: ball further out -> re-arm */
#define SAVE_REARM_LOST_MS  1000    /* ball unseen this long -> re-arm */
#define SAVE_HIT_WINDOW_MS  300     /* HIT counts if a shot was incoming this recently */

/* ---- scoreboard ---- */
#define NEEDLE_STEPS_PER_PT 128     /* 28BYJ-48: 2048 steps/rev -> 22.5 deg per point */
#define NEEDLE_MAX_STEPS    1024    /* +-half a turn */
#define CELEBRATE_MS        1500

/* ---- LINUX emulation ---- */
#define LINUX_JITTER_PROB   0.25    /* fraction of ticks that get an injected delay */

static const goalie_backend_t *backends[] = { &goalie_arduino, &goalie_stub };

/* ---- config ---- */
static const char *cfg_backend = "arduino";
static const char *cfg_dev     = "/dev/serusb1";  /* TODO(on-device): verify; UART fallback /dev/ser1 */
static int    cfg_port      = 5556;
static int    cfg_wd_ms     = 300;
static int    cfg_rt_prio   = 40;
static int    cfg_jitter_ms = 30;
static double cfg_walls     = PRED_WALL_HALF_W_DEF;

/* ---- shared state (all under `lock`) ---- */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static double kp = CTL_KP_DEF, kd = CTL_KD_DEF;

static struct timespec last_feed;       /* F / PING / HUMAN: watchdog */
static struct timespec last_frame;      /* F only: ms_since_frame */

static predictor_t     pred;
static pred_result_t   pred_last;       /* evaluated on each ball frame */
static struct timespec ball_rx;         /* Pi time of the last ball sighting */
static int             ball_ever;

static int             paddle_valid;
static double          paddle_x, paddle_v, paddle_t;   /* paddle_t: laptop seconds */
static struct timespec paddle_rx;

static int    ctl_human;
static double human_x;

static int    mode_linux;               /* requested mode */
static int    stats_reset_req;

static double target_x, duty_out;
static int    wd_active, motor_stopped;

static int    saves, goals, hits, last_accel_mg = -1;
static int    armed = 1, shot_incoming;
static double shot_min_y = 1e9;
static struct timespec last_incoming;
static int    score_changed, celebrate_req;

static long   jit_n;
static double jit_sum_us, jit_max_us, loop_hz;
static int    sched_rr;                 /* actual policy of the control thread */

static volatile sig_atomic_t running = 1;
static const goalie_backend_t *be;

/* ------------------------------------------------------------------ helpers */
static long ms_since(const struct timespec *t)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->tv_sec) * 1000L + (now.tv_nsec - t->tv_nsec) / 1000000L;
}

static double ts_diff_us(const struct timespec *a, const struct timespec *b)
{
    return (a->tv_sec - b->tv_sec) * 1e6 + (a->tv_nsec - b->tv_nsec) / 1e3;
}

static double clampd(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }

static void sleep_until(const struct timespec *next)
{
#ifdef __APPLE__
    /* macOS has no clock_nanosleep; relative sleep is fine for a host build. */
    struct timespec now, d;
    clock_gettime(CLOCK_MONOTONIC, &now);
    d.tv_sec = next->tv_sec - now.tv_sec;
    d.tv_nsec = next->tv_nsec - now.tv_nsec;
    if (d.tv_nsec < 0) { d.tv_nsec += 1000000000L; d.tv_sec--; }
    if (d.tv_sec >= 0) nanosleep(&d, NULL);
#else
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, next, NULL) == EINTR) {}
#endif
}

static uint32_t rng_state = 0x9e3779b9u;
static double rng01(void)   /* xorshift32, control thread only */
{
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return (x >> 8) / 16777216.0;
}

/* Apply the scheduling policy for the mode to the calling (control) thread. */
static void apply_sched(int linux_mode)
{
    struct sched_param sp;
    int pol = linux_mode ? SCHED_OTHER : SCHED_RR;
    int lo = sched_get_priority_min(pol), hi = sched_get_priority_max(pol);
    int prio = linux_mode ? 10 : cfg_rt_prio;   /* 10 = QNX default user priority */
    sp.sched_priority = prio < lo ? lo : prio > hi ? hi : prio;
    int err = pthread_setschedparam(pthread_self(), pol, &sp);
    if (err)
        fprintf(stderr, "[ctl] setschedparam(%s, %d) failed: %s\n",
                linux_mode ? "OTHER" : "RR", sp.sched_priority, strerror(err));
    int cur_pol;
    if (pthread_getschedparam(pthread_self(), &cur_pol, &sp) == 0)
        sched_rr = cur_pol == SCHED_RR;
}

/* Called with `lock` held. */
static void count_save(const char *why)
{
    saves++;
    armed = 0;
    shot_incoming = 0;
    score_changed = 1;
    celebrate_req = 1;
    fprintf(stderr, "[game] SAVE #%d (%s)\n", saves, why);
}

static void rearm(void)   /* lock held */
{
    armed = 1;
    shot_incoming = 0;
    shot_min_y = 1e9;
}

/* Uno line from the control thread. Lock held. */
static void handle_event(const char *line)
{
    int v;
    if (sscanf(line, "HIT %d", &v) == 1) {
        hits++;
        if (armed && shot_incoming && ms_since(&last_incoming) < SAVE_HIT_WINDOW_MS)
            count_save("hit");
        else
            fprintf(stderr, "[game] HIT %d mg (no shot in flight, not a save)\n", v);
    } else if (sscanf(line, "A %d", &v) == 1) {
        last_accel_mg = v;
    } else if (!strncmp(line, "READY", 5)) {
        fprintf(stderr, "[uno] READY\n");
    } else if (line[0]) {
        fprintf(stderr, "[uno] ? %s\n", line);
    }
}

/* ------------------------------------------------------------------ control */
static void *control_thread(void *arg)
{
    (void)arg;
    const long period_ns = 1000000000L / TICK_HZ;
    struct timespec next, now, hz_t0;
    clock_gettime(CLOCK_MONOTONIC, &next);
    hz_t0 = next;
    long hz_ticks = 0;
    int cur_linux = -1, was_wd = 0, needle_sent = 0;
    struct timespec last_stop_sent = next;

    while (running) {
        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        sleep_until(&next);

        pthread_mutex_lock(&lock);
        int want_linux = mode_linux;
        pthread_mutex_unlock(&lock);

        /* Mode switch (policy change + stats reset). */
        if (want_linux != cur_linux) {
            apply_sched(want_linux);
            if (want_linux)
                fprintf(stderr, "[ctl] MODE LINUX (EMULATED: SCHED_OTHER + random 0-%d ms "
                        "delays on %.0f%% of ticks)\n", cfg_jitter_ms, LINUX_JITTER_PROB * 100);
            else
                fprintf(stderr, "[ctl] MODE QNX (SCHED_RR prio %d, absolute-time ticks)%s\n",
                        cfg_rt_prio, sched_rr ? "" : " [RR not granted: running SCHED_OTHER]");
            cur_linux = want_linux;
            pthread_mutex_lock(&lock);
            stats_reset_req = 1;
            pthread_mutex_unlock(&lock);
        }

        /* LINUX emulation: the "scheduler" sometimes runs us late. */
        if (cur_linux && cfg_jitter_ms > 0 && rng01() < LINUX_JITTER_PROB) {
            long us = (long)(rng01() * cfg_jitter_ms * 1000.0);
            struct timespec d = { us / 1000000L, (us % 1000000L) * 1000L };
            nanosleep(&d, NULL);
        }

        /* Lateness of this tick = jitter. */
        clock_gettime(CLOCK_MONOTONIC, &now);
        double late_us = ts_diff_us(&now, &next);
        if (late_us < 0) late_us = 0;
        if (late_us > period_ns / 1e3) next = now;   /* overran: don't burst to catch up */

        /* Uno events (non-blocking). */
        char ev[8][64];
        int nev = 0;
        while (nev < 8 && be->poll_event(ev[nev], sizeof ev[nev])) nev++;

        pthread_mutex_lock(&lock);
        if (stats_reset_req) {
            jit_n = 0; jit_sum_us = 0; jit_max_us = 0;
            stats_reset_req = 0;
        }
        jit_n++;
        jit_sum_us += late_us;
        if (late_us > jit_max_us) jit_max_us = late_us;

        for (int i = 0; i < nev; i++) handle_event(ev[i]);

        /* Re-arm the save counter if the ball has been gone a while. */
        if (!armed && (!ball_ever || ms_since(&ball_rx) > SAVE_REARM_LOST_MS)) rearm();

        /* Target. */
        int wd = ms_since(&last_feed) > cfg_wd_ms;
        double tgt = target_x;
        if (wd) {
            tgt = 0.0;
        } else if (ctl_human) {
            tgt = human_x;
        } else if (!ball_ever || ms_since(&ball_rx) > BALL_LOST_MS) {
            tgt = 0.0;
        } else if (pred_last.state == PRED_INCOMING) {
            tgt = pred_last.x_cross;
        } else if (pred_last.state == PRED_AWAY) {
            tgt = 0.0;
        } /* PRED_NONE: hold */
        tgt = clampd(tgt, -GOAL_HALF_W, GOAL_HALF_W);

        /* PD on the camera-measured paddle position. */
        long p_age = paddle_valid ? ms_since(&paddle_rx) : 1000000L;
        int have_fb = paddle_valid && p_age < PADDLE_STALE_MS;
        double duty = 0.0;
        if (have_fb) {
            double ext = clampd(p_age / 1000.0 + CTL_CAM_LATENCY_S, 0.0, CTL_EXTRAP_MAX_S);
            double est = paddle_x + paddle_v * ext;
            double err = tgt - est;
            double raw = kp * err - kd * paddle_v;
            if (fabs(err) < CTL_ERR_DEADBAND && fabs(paddle_v) < CTL_STILL_V) {
                duty = 0.0;
            } else if (raw != 0.0) {
                double mag = fmin(fabs(raw), 1.0);
                duty = copysign(CTL_DUTY_MIN + (1.0 - CTL_DUTY_MIN) * mag, raw);
            }
            duty = clampd(duty, -CTL_DUTY_MAX, CTL_DUTY_MAX);
        }
        target_x = tgt;
        duty_out = duty;
        wd_active = wd;
        motor_stopped = !have_fb;

        int needle = 0, do_needle = 0, do_celebrate = 0;
        if (score_changed || !needle_sent) {
            needle = (int)clampd((saves - goals) * NEEDLE_STEPS_PER_PT,
                                 -NEEDLE_MAX_STEPS, NEEDLE_MAX_STEPS);
            do_needle = 1;
            score_changed = 0;
            needle_sent = 1;
        }
        if (celebrate_req) { do_celebrate = 1; celebrate_req = 0; }
        pthread_mutex_unlock(&lock);

        /* Loop rate, once per second. */
        hz_ticks++;
        double el = ts_diff_us(&now, &hz_t0);
        if (el >= 1e6) {
            pthread_mutex_lock(&lock);
            loop_hz = hz_ticks * 1e6 / el;
            pthread_mutex_unlock(&lock);
            hz_ticks = 0;
            hz_t0 = now;
        }

        if (wd && !was_wd)
            fprintf(stderr, "[ctl] WATCHDOG: nothing for %d ms -> park centre\n", cfg_wd_ms);
        else if (!wd && was_wd)
            fprintf(stderr, "[ctl] commands back, watchdog cleared\n");
        was_wd = wd;

        /* Actuate. */
        if (have_fb) {
            be->motor((float)duty);
        } else if (ms_since(&last_stop_sent) >= 100) {
            be->stop();   /* no paddle feedback: can't position-control, stop */
            clock_gettime(CLOCK_MONOTONIC, &last_stop_sent);
        }
        if (do_needle) be->needle(needle);
        if (do_celebrate) be->celebrate(CELEBRATE_MS);
    }
    be->stop();
    return NULL;
}

/* ------------------------------------------------------------------ network */
static int parse_bit(const char *s, int *out)
{
    if (!strcmp(s, "0")) { *out = 0; return 0; }
    if (!strcmp(s, "1")) { *out = 1; return 0; }
    return -1;
}

/* Frame: lock held. */
static void on_frame(double t_s, int bseen, double bx, double by, int pseen, double px)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    last_feed = last_frame = now;

    if (bseen) {
        predict_add(&pred, t_s, bx, by);
        pred_last = predict_eval(&pred);
        ball_rx = now;
        ball_ever = 1;

        if (by > SAVE_REARM_Y) rearm();
        if (pred_last.state != PRED_NONE) {
            if (pred_last.vy < SAVE_VY_IN) {
                shot_incoming = 1;
                last_incoming = now;
            }
            if (shot_incoming && by < shot_min_y) shot_min_y = by;
            if (armed && shot_incoming && pred_last.vy > SAVE_VY_OUT && shot_min_y < SAVE_REVERSE_Y)
                count_save("reversal");
        }
    }

    if (pseen) {
        if (paddle_valid && ms_since(&paddle_rx) < 200 && t_s > paddle_t) {
            double v = (px - paddle_x) / (t_s - paddle_t);
            v = clampd(v, -PADDLE_VEL_MAX, PADDLE_VEL_MAX);
            paddle_v = PADDLE_VEL_ALPHA * v + (1.0 - PADDLE_VEL_ALPHA) * paddle_v;
        } else {
            paddle_v = 0.0;
        }
        paddle_x = px;
        paddle_t = t_s;
        paddle_rx = now;
        paddle_valid = 1;
    }
}

static void handle_line(char *line, char *reply, size_t rlen)
{
    char cmd[16] = "", arg[16] = "";
    sscanf(line, "%15s %15s", cmd, arg);

    if (!strcasecmp(cmd, "F")) {
        long long t_ms;
        char bs[4], ps[4];
        double bx, by, px;
        int bseen, pseen;
        if (sscanf(line, "%*s %lld %3s %lf %lf %3s %lf", &t_ms, bs, &bx, &by, ps, &px) != 6 ||
            parse_bit(bs, &bseen) || parse_bit(ps, &pseen) ||
            (bseen && (!isfinite(bx) || !isfinite(by))) || (pseen && !isfinite(px))) {
            snprintf(reply, rlen, "ERR usage: F <t_ms> <0|1> <bx> <by> <0|1> <px>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        on_frame(t_ms / 1000.0, bseen, bx, by, pseen, px);
        snprintf(reply, rlen, "OK %.4f %.3f\n", target_x, duty_out);
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "MODE")) {
        int lin;
        if (!strcasecmp(arg, "QNX")) lin = 0;
        else if (!strcasecmp(arg, "LINUX")) lin = 1;
        else { snprintf(reply, rlen, "ERR usage: MODE QNX|LINUX\n"); return; }
        pthread_mutex_lock(&lock);
        mode_linux = lin;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK %s\n", lin ? "LINUX" : "QNX");
    } else if (!strcasecmp(cmd, "HUMAN")) {
        double x;
        if (sscanf(line, "%*s %lf", &x) != 1 || !isfinite(x)) {
            snprintf(reply, rlen, "ERR usage: HUMAN <x>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        ctl_human = 1;
        human_x = clampd(x, -GOAL_HALF_W, GOAL_HALF_W);
        clock_gettime(CLOCK_MONOTONIC, &last_feed);
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK HUMAN %.4f\n", human_x);
    } else if (!strcasecmp(cmd, "AUTO")) {
        pthread_mutex_lock(&lock);
        ctl_human = 0;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK AUTO\n");
    } else if (!strcasecmp(cmd, "GOAL")) {
        pthread_mutex_lock(&lock);
        goals++;
        armed = 0;            /* this shot is over; re-armed when the ball goes out/away */
        shot_incoming = 0;
        score_changed = 1;
        int g = goals;
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[game] GOAL #%d\n", g);
        snprintf(reply, rlen, "OK GOAL %d\n", g);
    } else if (!strcasecmp(cmd, "RESET")) {
        pthread_mutex_lock(&lock);
        saves = goals = hits = 0;
        score_changed = 1;
        stats_reset_req = 1;
        rearm();
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK RESET\n");
    } else if (!strcasecmp(cmd, "PING")) {
        pthread_mutex_lock(&lock);
        clock_gettime(CLOCK_MONOTONIC, &last_feed);
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK PONG\n");
    } else if (!strcasecmp(cmd, "STATUS")) {
        pthread_mutex_lock(&lock);
        snprintf(reply, rlen,
            "OK mode=%s ctl=%s saves=%d goals=%d target=%.4f paddle=%.4f duty=%.3f wd=%d "
            "ms_since_frame=%ld jit_avg_us=%.0f jit_max_us=%.0f loop_hz=%.1f hits=%d "
            "sched=%s emulated=%d pred=%s armed=%d stopped=%d paddle_seen=%d kp=%.3f kd=%.3f "
            "accel_mg=%d backend=%s\n",
            mode_linux ? "LINUX" : "QNX", ctl_human ? "HUMAN" : "AUTO", saves, goals,
            target_x, paddle_x, duty_out, wd_active, ms_since(&last_frame),
            jit_n ? jit_sum_us / jit_n : 0.0, jit_max_us, loop_hz, hits,
            sched_rr ? "RR" : "OTHER", mode_linux, predict_state_name(pred_last.state),
            armed, motor_stopped, paddle_valid, kp, kd, last_accel_mg, be->name);
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "GAINS")) {
        double a, b;
        if (sscanf(line, "%*s %lf %lf", &a, &b) != 2 || !isfinite(a) || !isfinite(b) || a < 0 || b < 0) {
            snprintf(reply, rlen, "ERR usage: GAINS <kp> <kd>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        kp = a; kd = b;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK %.3f %.3f\n", a, b);
    } else if (!strcasecmp(cmd, "SIMHIT")) {
        int mag = 1000;
        sscanf(line, "%*s %d", &mag);
        char ev[32];
        snprintf(ev, sizeof ev, "HIT %d", mag);
        if (!be->inject)
            snprintf(reply, rlen, "ERR SIMHIT needs -b stub\n");
        else if (be->inject(ev) != 0)
            snprintf(reply, rlen, "ERR event queue full\n");
        else
            snprintf(reply, rlen, "OK SIMHIT %d\n", mag);
    } else if (cmd[0] == '\0') {
        reply[0] = '\0';   /* blank line: no reply */
    } else {
        snprintf(reply, rlen, "ERR unknown command '%s'\n", cmd);
    }
}

static void *client_thread(void *arg)
{
    int fd = (int)(long)arg;
    char buf[1024], reply[640];
    size_t used = 0;

    fprintf(stderr, "[net] client connected\n");
    for (;;) {
        ssize_t n = recv(fd, buf + used, sizeof buf - 1 - used, 0);
        if (n <= 0) break;
        used += (size_t)n;
        buf[used] = '\0';

        char *start = buf, *nl;
        while ((nl = strchr(start, '\n')) != NULL) {
            *nl = '\0';
            if (nl > start && nl[-1] == '\r') nl[-1] = '\0';
            handle_line(start, reply, sizeof reply);
            if (reply[0]) send(fd, reply, strlen(reply), 0);
            start = nl + 1;
        }
        used = strlen(start);
        memmove(buf, start, used + 1);
        if (used >= sizeof buf - 1) used = 0;   /* line too long: drop it */
    }
    /* No stop here: a STATUS-only client (UI) may come and go. The watchdog
     * parks the paddle 300 ms after the frame stream dies. */
    fprintf(stderr, "[net] client disconnected\n");
    close(fd);
    return NULL;
}

/* ------------------------------------------------------------------ main */
static void on_signal(int sig) { (void)sig; running = 0; }

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [-b arduino|stub] [-d dev] [-p port] [-w wd_ms] [-r rt_prio]\n"
        "          [--kp K] [--kd K] [--jitter-ms MS] [--walls HALF_W_M]\n"
        "  defaults: -b arduino -d /dev/serusb1 -p 5556 -w 300 -r 40 --kp %.1f --kd %.2f\n"
        "            --jitter-ms 30 (LINUX emulation) --walls 0 (no side-wall bounces)\n",
        argv0, CTL_KP_DEF, CTL_KD_DEF);
}

int main(int argc, char **argv)
{
    /* Hand-rolled parser (QNX getopt_long availability not assumed). */
    for (int i = 1; i < argc; i += 2) {
        const char *a = argv[i];
        const char *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(argv[0]); return 0; }
        if (!v) { usage(argv[0]); return 2; }
        if      (!strcmp(a, "-b")) cfg_backend = v;
        else if (!strcmp(a, "-d")) cfg_dev = v;
        else if (!strcmp(a, "-p")) cfg_port = atoi(v);
        else if (!strcmp(a, "-w")) cfg_wd_ms = atoi(v);
        else if (!strcmp(a, "-r")) cfg_rt_prio = atoi(v);
        else if (!strcmp(a, "--kp")) kp = atof(v);
        else if (!strcmp(a, "--kd")) kd = atof(v);
        else if (!strcmp(a, "--jitter-ms")) cfg_jitter_ms = atoi(v);
        else if (!strcmp(a, "--walls")) cfg_walls = atof(v);
        else { usage(argv[0]); return 2; }
    }

    for (size_t i = 0; i < sizeof backends / sizeof *backends; i++)
        if (!strcmp(backends[i]->name, cfg_backend)) be = backends[i];
    if (!be) { fprintf(stderr, "unknown backend '%s'\n", cfg_backend); return 2; }
    if (be->init(cfg_dev) != 0) { fprintf(stderr, "backend init failed\n"); return 1; }
    be->zero_needle();

    predict_init(&pred, cfg_walls);

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;          /* no SA_RESTART: lets accept() return EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    clock_gettime(CLOCK_MONOTONIC, &last_feed);
    last_feed.tv_sec -= 10;             /* start in watchdog-park state */
    last_frame = last_feed;

    /* Real-time control thread (QNX mode at start). */
    pthread_attr_t attr;
    struct sched_param sp = { .sched_priority = cfg_rt_prio };
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setschedparam(&attr, &sp);
    pthread_t ctl;
    if (pthread_create(&ctl, &attr, control_thread, NULL) != 0) {
        fprintf(stderr, "[ctl] can't get SCHED_RR prio %d (need privileges?), using default\n",
                cfg_rt_prio);
        if (pthread_create(&ctl, NULL, control_thread, NULL) != 0) {
            perror("pthread_create");
            return 1;
        }
    }
    pthread_attr_destroy(&attr);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((unsigned short)cfg_port);
    int rc = 0;
    if (srv < 0 || bind(srv, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(srv, 4) < 0) {
        perror("socket/bind/listen");
        running = 0;
        rc = 1;
    } else {
        fprintf(stderr, "goalie_motor: backend=%s dev=%s port=%d watchdog=%dms prio=%d "
                "kp=%.2f kd=%.2f walls=%.2f\n",
                be->name, cfg_dev, cfg_port, cfg_wd_ms, cfg_rt_prio, kp, kd, cfg_walls);
    }

    while (running) {
        int c = accept(srv, NULL, NULL);
        if (c < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        pthread_t t;
        if (pthread_create(&t, NULL, client_thread, (void *)(long)c) == 0)
            pthread_detach(t);
        else
            close(c);
    }

    running = 0;
    pthread_join(ctl, NULL);   /* control thread sends S on exit */
    be->shutdown();
    if (srv >= 0) close(srv);
    fprintf(stderr, "goalie_motor: bye\n");
    return rc;
}
