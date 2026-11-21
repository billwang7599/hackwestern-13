/* tars_motor: QNX motor server for TARS.
 *
 * TCP line protocol (one command per line, reply "OK ..." or "ERR <why>"):
 *   D <left> <right>   drive, each in [-1, 1]   (must be resent < watchdog ms)
 *   S                  stop
 *   G <name>           gesture
 *   PING               -> "OK PONG" (also feeds the watchdog)
 *   STATUS             -> "OK <l> <r> <ms_since_cmd> <dist_m> <estop>"
 *
 * Threads:
 *   main      accept loop, one detached thread per client
 *   control   SCHED_RR high priority, fixed-rate tick: watchdog, obstacle stop,
 *             slew limiting, calls backend->drive()
 *
 * Usage: tars_motor [-b pico|stub] [-d /dev/serN] [-p port] [-w watchdog_ms]
 *                   [-r rt_priority]
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

#include "motor.h"
#include "obstacle.h"

#define TICK_HZ          50
#define SLEW_PER_TICK    0.08f   /* max change per tick: 0 -> full in ~0.25 s */
#define OBSTACLE_STOP_M  0.25f   /* block forward motion closer than this */

static const motor_backend_t *motor_backends[] = { &motor_pico_uart, &motor_stub };

/* ---- config (from argv) ---- */
static const char *cfg_backend = "pico";
static const char *cfg_dev     = "/dev/ser1";   /* TODO: verify on Pi 5 QNX image */
static int cfg_port      = 5555;
static int cfg_wd_ms     = 500;
static int cfg_rt_prio   = 30;

/* ---- shared state ---- */
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static float target_l, target_r;          /* what clients asked for */
static float out_l, out_r;                /* what we last sent to motors */
static float last_dist = -1.0f;
static int   estop;                       /* 1 = obstacle/watchdog holding stop */
static struct timespec last_cmd;
static char  pending_gesture[48];
static volatile sig_atomic_t running = 1;
static const motor_backend_t *motor;

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

/* ------------------------------------------------------------------ control */
static void *control_thread(void *arg)
{
    (void)arg;
    const long period_ns = 1000000000L / TICK_HZ;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    int was_estop = 0;

    while (running) {
        next.tv_nsec += period_ns;
        if (next.tv_nsec >= 1000000000L) { next.tv_nsec -= 1000000000L; next.tv_sec++; }

        float dist = obstacle_distance_m();
        char gesture[48] = "";

        pthread_mutex_lock(&lock);
        float tl = target_l, tr = target_r;
        int wd_tripped = ms_since(&last_cmd) > cfg_wd_ms;
        if (wd_tripped) { tl = tr = 0.0f; target_l = target_r = 0.0f; }

        /* Obstacle: allow turning/reversing, block net forward motion. */
        int blocked = dist >= 0.0f && dist < OBSTACLE_STOP_M && (tl + tr) > 0.0f;
        if (blocked) { tl = tr = 0.0f; }

        estop = wd_tripped || blocked;
        last_dist = dist;
        /* Hard stop on estop (no slew); smooth otherwise. */
        out_l = estop ? tl : slew(out_l, tl);
        out_r = estop ? tr : slew(out_r, tr);
        float l = out_l, r = out_r;
        if (pending_gesture[0]) {
            memcpy(gesture, pending_gesture, sizeof gesture);
            pending_gesture[0] = '\0';
        }
        pthread_mutex_unlock(&lock);

        if (estop && !was_estop)
            fprintf(stderr, "[ctl] STOP (%s)\n", blocked ? "obstacle" : "watchdog");
        was_estop = estop;

        motor->drive(l, r);
        if (gesture[0] && motor->gesture) motor->gesture(gesture);

        sleep_until(&next);
    }
    motor->drive(0, 0);
    return NULL;
}

/* ------------------------------------------------------------------ network */
static void handle_line(char *line, char *reply, size_t rlen)
{
    char cmd[16] = "";
    float a, b;
    char name[48];

    sscanf(line, "%15s", cmd);

    if (!strcasecmp(cmd, "D")) {
        if (sscanf(line, "%*s %f %f", &a, &b) != 2 || isnan(a) || isnan(b)) {
            snprintf(reply, rlen, "ERR usage: D <left> <right>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        target_l = clampf(a, -1, 1);
        target_r = clampf(b, -1, 1);
        clock_gettime(CLOCK_MONOTONIC, &last_cmd);
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "S")) {
        pthread_mutex_lock(&lock);
        target_l = target_r = 0.0f;
        clock_gettime(CLOCK_MONOTONIC, &last_cmd);
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "G")) {
        if (sscanf(line, "%*s %47s", name) != 1) {
            snprintf(reply, rlen, "ERR usage: G <name>\n");
            return;
        }
        pthread_mutex_lock(&lock);
        snprintf(pending_gesture, sizeof pending_gesture, "%s", name);
        clock_gettime(CLOCK_MONOTONIC, &last_cmd);
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK\n");
    } else if (!strcasecmp(cmd, "PING")) {
        pthread_mutex_lock(&lock);
        clock_gettime(CLOCK_MONOTONIC, &last_cmd);
        pthread_mutex_unlock(&lock);
        snprintf(reply, rlen, "OK PONG\n");
    } else if (!strcasecmp(cmd, "STATUS")) {
        pthread_mutex_lock(&lock);
        snprintf(reply, rlen, "OK %.2f %.2f %ld %.2f %d\n",
                 out_l, out_r, ms_since(&last_cmd), last_dist, estop);
        pthread_mutex_unlock(&lock);
    } else if (cmd[0] == '\0') {
        reply[0] = '\0';
    } else {
        snprintf(reply, rlen, "ERR unknown command '%s'\n", cmd);
    }
}

static void *client_thread(void *arg)
{
    int fd = (int)(long)arg;
    char buf[512], reply[128];
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
    /* Safety: a dropped client means stop now, don't wait for the watchdog. */
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
        "usage: %s [-b pico|stub] [-d uart_dev] [-p port] [-w watchdog_ms] [-r rt_prio]\n",
        argv0);
}

int main(int argc, char **argv)
{
    int opt;
    while ((opt = getopt(argc, argv, "b:d:p:w:r:h")) != -1) {
        switch (opt) {
        case 'b': cfg_backend = optarg; break;
        case 'd': cfg_dev = optarg; break;
        case 'p': cfg_port = atoi(optarg); break;
        case 'w': cfg_wd_ms = atoi(optarg); break;
        case 'r': cfg_rt_prio = atoi(optarg); break;
        default:  usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }

    for (size_t i = 0; i < sizeof motor_backends / sizeof *motor_backends; i++)
        if (!strcmp(motor_backends[i]->name, cfg_backend)) motor = motor_backends[i];
    if (!motor) { fprintf(stderr, "unknown backend '%s'\n", cfg_backend); return 2; }
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
