/* tars_motor: QNX motor/arm server for TARS.
 *
 * TCP line protocol (one command per line, reply "OK ..." or "ERR <why>"):
 *   D <left> <right>     drive, each in [-1, 1] (must be resent < watchdog ms)
 *   S                    stop driving, cancel any gesture, halt both arms
 *   G <name>             gesture: wave point shrug nod stance fold
 *   A <arm> <deg> [dps]  arm 0=left 1=right to an absolute angle -> "OK <deg>"
 *   HOME                 both arms to 0
 *   ZERO                 the arms' current position becomes 0
 *   LEVEL                "the robot is upright now" (recapture tilt reference)
 *   RESET                clear a tip-over latch (refused while still tipped)
 *   GESTURES             -> "OK wave point ..."
 *   PING                 -> "OK PONG" (also feeds the watchdog)
 *   STATUS               -> "OK <l> <r> <ms_since_cmd> <dist_m> <estop> key=val..."
 *   SIM T|B|READY [n]    stub backend only: inject a co-processor event (tests)
 *
 * Threads:
 *   main      accept loop, one detached thread per client
 *   control   SCHED_RR high priority, fixed-rate tick. The ONLY thread that
 *             talks to the backend: drains events (tip/bump/ready), applies
 *             the watchdog / obstacle / tip / bump stops, slews the wheels,
 *             plays gesture keyframes and keeps a model of the arm angles.
 *
 * Usage: tars_motor [-b uno|stub|pico] [-d dev] [-p port] [-w watchdog_ms]
 *                   [-r rt_priority] [-k bump_hold_ms]
 */
#include <errno.h>
#include <math.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "gesture.h"
#include "motor.h"
#include "obstacle.h"

#define TICK_HZ          50
#define SLEW_PER_TICK    0.08f   /* max change per tick: 0 -> full in ~0.25 s */
#define OBSTACLE_STOP_M  0.25f   /* block forward motion closer than this */
#define TIP_CLEAR_MS     1000    /* RESET refused if a T event came this recently */

static const motor_backend_t *motor_backends[] = { &motor_uno_serial, &motor_stub, &motor_pico_uart };

/* ---- config (from argv) ---- */
static const char *cfg_backend = "uno";
static const char *cfg_dev     = NULL;      /* NULL = backend default */
static int cfg_port      = 5555;
static int cfg_wd_ms     = 500;
static int cfg_rt_prio   = 30;
static int cfg_bump_ms   = 1000;

/* ---- shared state (all under `lock`) ---- */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static float target_l, target_r;          /* what clients asked for */
static float out_l, out_r;                /* what we last sent to motors */
static float last_dist = -1.0f;
static int   estop;                       /* 1 = some stop is holding the wheels */
static const char *estop_why = "none";
static struct timespec last_cmd;

/* requests from client threads, executed by the control thread */
typedef struct { int set; float deg, dps; } arm_req_t;
static arm_req_t arm_req[2];
static int   req_halt, req_zero, req_level, req_stop_gesture, req_reset;
static const gesture_t *req_gesture;
static char  req_dev_gesture[48];         /* pico: device-side gesture */
#define SIMQ 8
static motor_event_t simq[SIMQ];
static int   simq_n;

/* published by the control thread for STATUS / command checks */
static float arm_pos[2], arm_tgt[2];
static const char *gesture_name = "none";
static int   tipped, tilt_deg, bumps;
static long long last_tip_ms, bump_until_ms;
static int   ready_count;

static volatile sig_atomic_t running = 1;
static const motor_backend_t *motor;

static long long now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static long ms_since(const struct timespec *t)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->tv_sec) * 1000L + (now.tv_nsec - t->tv_nsec) / 1000000L;
}

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

static float slew(float cur, float tgt)
{
    float d = clampf(tgt - cur, -SLEW_PER_TICK, SLEW_PER_TICK);
    return cur + d;
}

static void sleep_until(struct timespec *next)
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

/* ------------------------------------------------------------------ control
 * Everything below with a ctl_ prefix runs only in the control thread. */
static float ctl_pos[2], ctl_tgt[2], ctl_dps[2] = { ARM_DEFAULT_DPS, ARM_DEFAULT_DPS };
static const gesture_t *ctl_g;
static int ctl_g_idx;
static long long ctl_g_next;

static void ctl_arm(int i, float deg, float dps)
{
    deg = clampf(deg, -ARM_LIMIT_DEG, ARM_LIMIT_DEG);
    dps = clampf(dps, 1.0f, ARM_MAX_DPS);
    ctl_tgt[i] = deg;
    ctl_dps[i] = dps;
    if (motor->arm) motor->arm(i, deg, dps);
}

static void ctl_halt_arms(void)
{
    ctl_tgt[0] = ctl_pos[0];
    ctl_tgt[1] = ctl_pos[1];
    if (motor->arm_halt) motor->arm_halt();
}

static void ctl_gesture_stop(const char *why)
{
    if (!ctl_g) return;
    fprintf(stderr, "[ctl] gesture %s %s\n", ctl_g->name, why);
    ctl_g = NULL;
}

static void ctl_gesture_start(const gesture_t *g)
{
    ctl_gesture_stop("cancelled");
    ctl_g = g;
    ctl_g_idx = -1;
    ctl_g_next = 0;
    fprintf(stderr, "[ctl] gesture %s start (~%d ms)\n", g->name, gesture_duration_ms(g, ctl_pos));
}

/* Next keyframe once the previous one's travel + hold time has passed. */
static void ctl_gesture_tick(long long now)
{
    if (!ctl_g || now < ctl_g_next) return;
    if (++ctl_g_idx >= ctl_g->n) { ctl_gesture_stop("done"); return; }
    const keyframe_t *f = &ctl_g->frames[ctl_g_idx];
    float travel = 0;
    for (int a = 0; a < 2; a++) {
        if (f->deg[a] >= ARM_KEEP) continue;
        float t = fabsf(f->deg[a] - ctl_pos[a]) / clampf(f->dps, 1.0f, ARM_MAX_DPS);
        if (t > travel) travel = t;
        ctl_arm(a, f->deg[a], f->dps);
    }
    ctl_g_next = now + (long long)(travel * 1000.0f * FRAME_MARGIN) + FRAME_PAD_MS + f->hold_ms;
}

/* Dead-reckoned arm angles: the steppers move at a known rate, so the Pi can
 * track them without feedback (until something skips steps or the Uno reboots). */
static void ctl_arm_model(float dt)
{
    for (int i = 0; i < 2; i++) {
        float step = ctl_dps[i] * dt, d = ctl_tgt[i] - ctl_pos[i];
        ctl_pos[i] = fabsf(d) <= step ? ctl_tgt[i] : ctl_pos[i] + (d > 0 ? step : -step);
    }
}

static void ctl_event(const motor_event_t *ev, long long now)
{
    switch (ev->kind) {
    case MOTOR_EV_TIP:
        pthread_mutex_lock(&lock);
        int was = tipped;
        tipped = 1;
        tilt_deg = ev->value;
        last_tip_ms = now;
        target_l = target_r = 0.0f;
        pthread_mutex_unlock(&lock);
        if (!was) {
            fprintf(stderr, "[ctl] TIPPED (%d deg) -> stop, latched until RESET\n", ev->value);
            ctl_gesture_stop("cancelled (tipped)");
            ctl_halt_arms();
        }
        break;
    case MOTOR_EV_BUMP:
        pthread_mutex_lock(&lock);
        bumps++;
        bump_until_ms = now + cfg_bump_ms;
        target_l = target_r = 0.0f;
        pthread_mutex_unlock(&lock);
        fprintf(stderr, "[ctl] BUMP (%d mg) -> stop, drive refused for %d ms\n", ev->value, cfg_bump_ms);
        break;
    case MOTOR_EV_READY:
        pthread_mutex_lock(&lock);
        int n = ++ready_count;
        pthread_mutex_unlock(&lock);
        if (n > 1)
            fprintf(stderr, "[ctl] co-processor READY again (it rebooted): arm angles reset to 0, "
                            "re-zero the arms by hand if they were not at 0\n");
        else
            fprintf(stderr, "[ctl] co-processor READY\n");
        ctl_gesture_stop("cancelled (co-processor reboot)");
        ctl_pos[0] = ctl_pos[1] = ctl_tgt[0] = ctl_tgt[1] = 0.0f;
        break;
    default:
        break;
    }
}

static void *control_thread(void *arg)
{
    (void)arg;
    const long period_ns = 1000000000L / TICK_HZ;
    const float dt = 1.0f / TICK_HZ;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    const char *was_why = "none";

    while (running) {
        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }
        long long now = now_ms();

        /* 1. events: from the co-processor, then injected ones (SIM) */
        motor_event_t ev;
        while (motor->poll && motor->poll(&ev)) ctl_event(&ev, now);
        motor_event_t sims[SIMQ];
        pthread_mutex_lock(&lock);
        int ns = simq_n;
        memcpy(sims, simq, sizeof sims);
        simq_n = 0;
        pthread_mutex_unlock(&lock);
        for (int i = 0; i < ns; i++) ctl_event(&sims[i], now);

        /* 2. take client requests */
        float dist = obstacle_distance_m();
        pthread_mutex_lock(&lock);
        arm_req_t areq[2];
        memcpy(areq, arm_req, sizeof areq);
        memset(&arm_req, 0, sizeof arm_req);
        int halt = req_halt, zero = req_zero, level = req_level;
        int stop_g = req_stop_gesture, reset = req_reset;
        const gesture_t *start_g = req_gesture;
        char dev_g[48];
        memcpy(dev_g, req_dev_gesture, sizeof dev_g);
        req_halt = req_zero = req_level = req_stop_gesture = req_reset = 0;
        req_gesture = NULL;
        req_dev_gesture[0] = '\0';

        /* 3. wheels: watchdog, obstacle, tip latch, bump hold */
        float tl = target_l, tr = target_r;
        int wd_tripped = ms_since(&last_cmd) > cfg_wd_ms;
        if (wd_tripped) { tl = tr = 0.0f; target_l = target_r = 0.0f; }
        /* Obstacle: allow turning/reversing, block net forward motion. */
        int blocked = dist >= 0.0f && dist < OBSTACLE_STOP_M && (tl + tr) > 0.0f;
        int bumped = now < bump_until_ms;
        if (blocked || tipped || bumped) { tl = tr = 0.0f; }
        estop = wd_tripped || blocked || tipped || bumped;
        estop_why = tipped ? "tipped" : bumped ? "bump" : blocked ? "obstacle"
                  : wd_tripped ? "watchdog" : "none";
        last_dist = dist;
        /* Hard stop on estop (no slew); smooth otherwise. */
        out_l = estop ? tl : slew(out_l, tl);
        out_r = estop ? tr : slew(out_r, tr);
        float l = out_l, r = out_r;
        int is_tipped = tipped;
        pthread_mutex_unlock(&lock);

        if (strcmp(estop_why, was_why) && strcmp(estop_why, "none"))
            fprintf(stderr, "[ctl] STOP (%s)\n", estop_why);
        was_why = estop_why;

        /* 4. arms + gestures (backend calls happen only here) */
        if (reset) fprintf(stderr, "[ctl] RESET: tip latch cleared\n");
        if (stop_g) ctl_gesture_stop("cancelled");
        if (halt) ctl_halt_arms();
        if (zero) {
            ctl_gesture_stop("cancelled");
            ctl_pos[0] = ctl_pos[1] = ctl_tgt[0] = ctl_tgt[1] = 0.0f;
            if (motor->arm_zero) motor->arm_zero();
        }
        if (level && motor->level) motor->level();
        if (!is_tipped) {
            for (int i = 0; i < 2; i++)
                if (areq[i].set) ctl_arm(i, areq[i].deg, areq[i].dps);
            if (start_g) ctl_gesture_start(start_g);
            ctl_gesture_tick(now);
        }
        ctl_arm_model(dt);

        motor->drive(l, r);
        if (dev_g[0] && motor->gesture) motor->gesture(dev_g);

        pthread_mutex_lock(&lock);
        for (int i = 0; i < 2; i++) { arm_pos[i] = ctl_pos[i]; arm_tgt[i] = ctl_tgt[i]; }
        gesture_name = ctl_g ? ctl_g->name : "none";
        pthread_mutex_unlock(&lock);

        sleep_until(&next);
    }
    motor->drive(0, 0);
    if (motor->arm_halt) motor->arm_halt();
    return NULL;
}

/* ------------------------------------------------------------------ network */
static void feed_watchdog(void) { clock_gettime(CLOCK_MONOTONIC, &last_cmd); }

/* Called with `lock` held. NULL = motion allowed, else the ERR reason. */
static const char *motion_refused(void)
{
    if (tipped) return "tipped over: stand TARS up, then send RESET";
    return NULL;
}

static void handle_line(char *line, char *reply, size_t rlen)
{
    char cmd[16] = "";
    float a, b, c;
    char name[48];
    const char *why;

    sscanf(line, "%15s", cmd);

    if (!strcasecmp(cmd, "D")) {
        if (sscanf(line, "%*s %f %f", &a, &b) != 2 || isnan(a) || isnan(b)) {
            snprintf(reply, rlen, "ERR usage: D <left> <right>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        feed_watchdog();
        long long left = bump_until_ms - now_ms();
        if ((why = motion_refused()) != NULL) {
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "ERR %s\n", why);
        } else if (left > 0) {
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "ERR bumped: driving paused %lld ms\n", left);
        } else {
            target_l = clampf(a, -1, 1);
            target_r = clampf(b, -1, 1);
            snprintf(reply, rlen, "OK\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "S")) {
        pthread_mutex_lock(&lock);
        target_l = target_r = 0.0f;
        req_stop_gesture = req_halt = 1;
        req_gesture = NULL;
        memset(&arm_req, 0, sizeof arm_req);
        feed_watchdog();
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "G")) {
        if (sscanf(line, "%*s %47s", name) != 1) {
            snprintf(reply, rlen, "ERR usage: G <name>\n");
            return;
        }
        const gesture_t *g = gesture_find(name);
        pthread_mutex_lock(&lock);
        feed_watchdog();
        if ((why = motion_refused()) != NULL) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else if (motor->arm) {
            if (!g) {
                snprintf(reply, rlen, "ERR unknown gesture '%s' (try GESTURES)\n", name);
            } else {
                req_gesture = g;
                memset(&arm_req, 0, sizeof arm_req);
                snprintf(reply, rlen, "OK %d\n", gesture_duration_ms(g, arm_pos));
            }
        } else if (motor->gesture) {
            snprintf(req_dev_gesture, sizeof req_dev_gesture, "%s", name);
            snprintf(reply, rlen, "OK\n");
        } else {
            snprintf(reply, rlen, "ERR backend has no gestures\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "A")) {
        int arm;
        int n = sscanf(line, "%*s %d %f %f", &arm, &a, &c);
        if (n < 2 || (arm != 0 && arm != 1) || isnan(a) || (n == 3 && !(c > 0))) {
            snprintf(reply, rlen, "ERR usage: A <0|1> <deg> [dps>0]\n");
            return;
        }
        if (!motor->arm) { snprintf(reply, rlen, "ERR backend has no arms\n"); return; }
        float deg = clampf(a, -ARM_LIMIT_DEG, ARM_LIMIT_DEG);
        float dps = n == 3 ? clampf(c, 1.0f, ARM_MAX_DPS) : ARM_DEFAULT_DPS;
        pthread_mutex_lock(&lock);
        feed_watchdog();
        if ((why = motion_refused()) != NULL) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else {
            req_stop_gesture = 1;      /* a manual arm move overrides a gesture */
            req_gesture = NULL;
            arm_req[arm].set = 1;
            arm_req[arm].deg = deg;
            arm_req[arm].dps = dps;
            snprintf(reply, rlen, "OK %.1f\n", deg);
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "HOME")) {
        if (!motor->arm) { snprintf(reply, rlen, "ERR backend has no arms\n"); return; }
        pthread_mutex_lock(&lock);
        feed_watchdog();
        if ((why = motion_refused()) != NULL) {
            snprintf(reply, rlen, "ERR %s\n", why);
        } else {
            req_stop_gesture = 1;
            req_gesture = NULL;
            for (int i = 0; i < 2; i++) { arm_req[i].set = 1; arm_req[i].deg = 0; arm_req[i].dps = ARM_DEFAULT_DPS; }
            snprintf(reply, rlen, "OK\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "ZERO") || !strcasecmp(cmd, "LEVEL")) {
        pthread_mutex_lock(&lock);
        if (cmd[0] == 'Z' || cmd[0] == 'z') { req_zero = 1; req_gesture = NULL; }
        else req_level = 1;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "RESET")) {
        pthread_mutex_lock(&lock);
        long long ago = now_ms() - last_tip_ms;
        if (tipped && ago < TIP_CLEAR_MS) {
            snprintf(reply, rlen, "ERR still tipped (last report %lld ms ago)\n", ago);
        } else {
            if (tipped) req_reset = 1;
            tipped = 0;
            bump_until_ms = 0;
            target_l = target_r = 0.0f;
            snprintf(reply, rlen, "OK\n");
        }
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "GESTURES")) {
        size_t off = (size_t)snprintf(reply, rlen, "OK");
        for (int i = 0; gesture_at(i) && off < rlen; i++)
            off += (size_t)snprintf(reply + off, rlen - off, " %s", gesture_at(i)->name);
        if (off < rlen - 1) snprintf(reply + off, rlen - off, "\n");
    } else if (!strcasecmp(cmd, "PING")) {
        pthread_mutex_lock(&lock);
        feed_watchdog();
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK PONG\n");
    } else if (!strcasecmp(cmd, "STATUS")) {
        pthread_mutex_lock(&lock);
        long long bl = bump_until_ms - now_ms();
        snprintf(reply, rlen,
                 "OK %.2f %.2f %ld %.2f %d why=%s arm0=%.1f arm1=%.1f arm0_tgt=%.1f arm1_tgt=%.1f "
                 "gesture=%s tipped=%d tilt=%d bump_ms=%lld bumps=%d ready=%d backend=%s\n",
                 out_l, out_r, ms_since(&last_cmd), last_dist, estop, estop_why,
                 arm_pos[0], arm_pos[1], arm_tgt[0], arm_tgt[1], gesture_name,
                 tipped, tilt_deg, bl > 0 ? bl : 0, bumps, ready_count, motor->name);
        pthread_mutex_unlock(&lock);
    } else if (!strcasecmp(cmd, "SIM")) {
        char kind[16] = "";
        int v = 0;
        if (motor != &motor_stub) { snprintf(reply, rlen, "ERR SIM is only for -b stub\n"); return; }
        if (sscanf(line, "%*s %15s %d", kind, &v) < 1) { snprintf(reply, rlen, "ERR usage: SIM T|B|READY [n]\n"); return; }
        motor_event_t ev = { MOTOR_EV_NONE, v };
        if (!strcasecmp(kind, "T")) ev.kind = MOTOR_EV_TIP;
        else if (!strcasecmp(kind, "B")) ev.kind = MOTOR_EV_BUMP;
        else if (!strcasecmp(kind, "READY")) ev.kind = MOTOR_EV_READY;
        else { snprintf(reply, rlen, "ERR usage: SIM T|B|READY [n]\n"); return; }
        pthread_mutex_lock(&lock);
        if (simq_n < SIMQ) simq[simq_n++] = ev;
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (cmd[0] == '\0') {
        reply[0] = '\0';
    } else {
        snprintf(reply, rlen, "ERR unknown command '%s'\n", cmd);
    }
}

static void *client_thread(void *arg)
{
    int fd = (int)(long)arg;
    char buf[512], reply[320];
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
    /* Safety: a dropped client means stop the wheels now, don't wait for the
     * watchdog. Arms finish their current move (a gesture is harmless). */
    pthread_mutex_lock(&lock);
    target_l = target_r = 0.0f;
    pthread_mutex_unlock(&lock);
    fprintf(stderr, "[net] client disconnected -> stop\n");
    close(fd);
    return NULL;
}

/* ------------------------------------------------------------------ main */
static void on_signal(int sig) { (void)sig; running = 0; }

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [-b uno|stub|pico] [-d serial_dev] [-p port] [-w watchdog_ms]\n"
        "          [-r rt_prio] [-k bump_hold_ms]\n", argv0);
}

int main(int argc, char **argv)
{
    int opt;
    while ((opt = getopt(argc, argv, "b:d:p:w:r:k:h")) != -1) {
        switch (opt) {
        case 'b': cfg_backend = optarg; break;
        case 'd': cfg_dev = optarg; break;
        case 'p': cfg_port = atoi(optarg); break;
        case 'w': cfg_wd_ms = atoi(optarg); break;
        case 'r': cfg_rt_prio = atoi(optarg); break;
        case 'k': cfg_bump_ms = atoi(optarg); break;
        default:  usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }

    for (size_t i = 0; i < sizeof motor_backends / sizeof *motor_backends; i++)
        if (!strcmp(motor_backends[i]->name, cfg_backend)) motor = motor_backends[i];
    if (!motor) { fprintf(stderr, "unknown backend '%s'\n", cfg_backend); return 2; }
    if (!cfg_dev) cfg_dev = motor->default_dev;
    if (motor->init(cfg_dev) != 0) { fprintf(stderr, "backend init failed\n"); return 1; }
    obstacle_init();

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;          /* no SA_RESTART: lets accept() return EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    clock_gettime(CLOCK_MONOTONIC, &last_cmd);
    last_cmd.tv_sec -= 10;              /* start in watchdog-stop state */

    /* Real-time control thread. */
    pthread_attr_t attr;
    struct sched_param sp = { .sched_priority = cfg_rt_prio };
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setschedparam(&attr, &sp);
    pthread_t ctl;
    if (pthread_create(&ctl, &attr, control_thread, NULL) != 0) {
        fprintf(stderr, "[ctl] can't get SCHED_RR prio %d (need root?), using default\n",
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
    if (srv < 0 || bind(srv, (struct sockaddr *)&addr, sizeof addr) < 0 || listen(srv, 4) < 0) {
        perror("socket/bind/listen");
        running = 0;
    } else {
        fprintf(stderr, "tars_motor: backend=%s dev=%s port=%d watchdog=%dms prio=%d\n",
                motor->name, cfg_dev, cfg_port, cfg_wd_ms, cfg_rt_prio);
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
    pthread_join(ctl, NULL);
    motor->shutdown();
    if (srv >= 0) close(srv);
    fprintf(stderr, "tars_motor: bye\n");
    return 0;
}
