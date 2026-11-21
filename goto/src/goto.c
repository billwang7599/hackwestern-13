/* goto: "TARS, go to the trash can". QNX-native go-to-object controller.
 *
 *   detections (JSON lines) ──► goto ──► tars_motor (TCP 5555: D / S / TURN / BEEP / LED)
 *   voice / laptop  ──GOTO──►  (control port TCP 5557)
 *
 * Inputs
 *   control  TCP 5557, one command per line, reply "OK ..." / "ERR ...":
 *              GOTO <object words>   start (or retarget). Aliases: goto/objects.txt
 *              CANCEL | STOP | S     stop now (sends S to tars_motor even when idle)
 *              STATUS                "OK state=.. target=.. result=.. ..."
 *              OBJECTS               "OK trash can,chair,..."
 *              RESOLVE <words>       "OK <canonical>" or "ERR unknown object"
 *              WATCH                 this connection also gets "EVENT <what> <object>"
 *                                    lines (searching/arrived/lost/cancelled/failed:..)
 *              PING                  "OK PONG"
 *   detect   one JSON object per line/datagram, see goto/DETECTIONS.md:
 *              UDP 127.0.0.1:5558, or TCP 127.0.0.1:5558 (line stream), or stdin (-i)
 *
 * Safety: S always goes out first; D is resent at tick_hz (>= 5 Hz) while
 * driving so tars_motor's 500 ms watchdog never trips mid-approach, and if
 * this process dies the server sees the disconnect and stops the wheels. A
 * stop sent by ANY other client (STATUS stops= goes up) cancels the GOTO.
 *
 * Usage: goto [-m host:port] [-p ctl_port] [-u det_port] [-i] [-c goto.conf]
 *             [-o objects.txt] [-r rt_prio] [-v] [-T time_scale (tests only)]
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "goto_core.h"

#define MAX_CTL 8
#define MAX_DETC 4
#define LINE_MAX_ 4096

static volatile sig_atomic_t running = 1;
static double time_scale = 1.0;
static int verbose;

static goto_cfg cfg;
static goto_objects objs;
static goto_sm sm;

/* ------------------------------------------------------------------ time */
static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec + ts.tv_nsec * 1e-9) * time_scale;
}

static void logf_(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[goto %8.2f] ", now_s());
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

/* --------------------------------------------------------- line buffers */
typedef struct { int fd; char buf[LINE_MAX_]; size_t used; int watch; } conn;

/* read what's there; call fn for each complete line. -1 on EOF/error. */
static int conn_read(conn *c, void (*fn)(conn *, char *))
{
    ssize_t n = recv(c->fd, c->buf + c->used, sizeof c->buf - 1 - c->used, 0);
    if (n <= 0) return -1;
    c->used += (size_t)n;
    c->buf[c->used] = '\0';
    char *start = c->buf, *nl;
    while ((nl = strchr(start, '\n')) != NULL) {
        *nl = '\0';
        if (nl > start && nl[-1] == '\r') nl[-1] = '\0';
        fn(c, start);
        start = nl + 1;
    }
    c->used = strlen(start);
    memmove(c->buf, start, c->used + 1);
    if (c->used >= sizeof c->buf - 1) c->used = 0;      /* overlong line: drop */
    return 0;
}

static void send_str(int fd, const char *s)
{
    size_t len = strlen(s), off = 0;
    while (off < len) {
        ssize_t n = send(fd, s + off, len - off, 0);
        if (n <= 0) { if (n < 0 && errno == EINTR) continue; return; }
        off += (size_t)n;
    }
}

/* --------------------------------------------------------- motor client */
static char motor_host[128] = "127.0.0.1";
static int motor_port = 5555;
static int motor_fd = -1;
static char motor_rx[2048];
static size_t motor_rx_used;
static double motor_next_try;
static long stops_baseline = -1;          /* stops= we expect (ours included) */
static double next_status_poll;

static void motor_close(const char *why)
{
    if (motor_fd >= 0) {
        close(motor_fd);
        logf_("motor: disconnected (%s)", why);
    }
    motor_fd = -1;
    motor_rx_used = 0;
}

static int motor_connect(void)
{
    if (motor_fd >= 0) return 0;
    double t = now_s();
    if (t < motor_next_try) return -1;
    motor_next_try = t + 1.0 * time_scale;
    struct addrinfo hints = {0}, *res = NULL;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    char port[16];
    snprintf(port, sizeof port, "%d", motor_port);
    if (getaddrinfo(motor_host, port, &hints, &res) || !res) return -1;
    int fd = socket(res->ai_family, res->ai_socktype, 0);
    if (fd < 0) { freeaddrinfo(res); return -1; }
    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        close(fd);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
    motor_fd = fd;
    motor_rx_used = 0;
    logf_("motor: connected to %s:%d", motor_host, motor_port);
    return 0;
}

/* send one line, wait for its reply line (tars_motor answers at once). */
static int motor_cmd(const char *line, char *reply, size_t rlen)
{
    if (motor_connect() < 0) return -1;
    char out[128];
    snprintf(out, sizeof out, "%s\n", line);
    send_str(motor_fd, out);
    double deadline = now_s() + 0.5 * time_scale;
    for (;;) {
        char *nl = memchr(motor_rx, '\n', motor_rx_used);
        if (nl) {
            size_t n = (size_t)(nl - motor_rx);
            size_t k = n < rlen - 1 ? n : rlen - 1;
            memcpy(reply, motor_rx, k);
            reply[k] = '\0';
            if (k && reply[k - 1] == '\r') reply[k - 1] = '\0';
            memmove(motor_rx, nl + 1, motor_rx_used - n - 1);
            motor_rx_used -= n + 1;
            if (verbose > 1) logf_("motor: %s -> %s", line, reply);
            return 0;
        }
        double left = (deadline - now_s()) / time_scale;
        if (left <= 0) { motor_close("reply timeout"); return -1; }
        struct pollfd p = {motor_fd, POLLIN, 0};
        int r = poll(&p, 1, (int)(left * 1000) + 1);
        if (r < 0 && errno == EINTR) continue;
        if (r <= 0) continue;
        if (motor_rx_used >= sizeof motor_rx - 1) motor_rx_used = 0;
        ssize_t n = recv(motor_fd, motor_rx + motor_rx_used, sizeof motor_rx - 1 - motor_rx_used, 0);
        if (n <= 0) { motor_close("closed"); return -1; }
        motor_rx_used += (size_t)n;
    }
}

/* "OK 0.00 0.00 12 -1.00 0 why=none ... move=done move_id=3 ..." -> value of key */
static int kv(const char *status, const char *key, char *out, size_t len)
{
    size_t kl = strlen(key);
    for (const char *p = status; (p = strstr(p, key)) != NULL; p++) {
        if ((p == status || p[-1] == ' ') && p[kl] == '=') {
            p += kl + 1;
            size_t n = strcspn(p, " ");
            if (n >= len) n = len - 1;
            memcpy(out, p, n);
            out[n] = '\0';
            return 0;
        }
    }
    return -1;
}

/* ------------------------------------------------------ control clients */
static conn ctl[MAX_CTL];
static conn detc[MAX_DETC];

static void broadcast_event(const char *text)
{
    char line[128];
    snprintf(line, sizeof line, "EVENT %s\n", text);
    for (int i = 0; i < MAX_CTL; i++)
        if (ctl[i].fd >= 0 && ctl[i].watch) send_str(ctl[i].fd, line);
    logf_("event: %s", text);
}

static void fail_now(const char *why);

/* Run the state machine's queued actions against tars_motor, in order. */
static void run_actions(void)
{
    for (int guard = 0; sm.n_act > 0 && guard < 8; guard++) {
        int n = sm.n_act;
        goto_act acts[GOTO_MAX_ACTS];
        memcpy(acts, sm.act, sizeof acts);
        sm.n_act = 0;
        for (int i = 0; i < n; i++) {
            goto_act *a = &acts[i];
            char line[96], reply[256];
            switch (a->kind) {
            case ACT_DRIVE: snprintf(line, sizeof line, "D %.3f %.3f", a->a, a->b); break;
            case ACT_STOP:  snprintf(line, sizeof line, "S"); break;
            case ACT_TURN:  snprintf(line, sizeof line, "TURN %.1f %.2f", a->a, a->b > 0 && a->b <= 1 ? a->b : 1.0); break;
            case ACT_BEEP:  snprintf(line, sizeof line, "BEEP %d", (int)a->a); break;
            case ACT_LED:   snprintf(line, sizeof line, "LED %s", a->text); break;
            case ACT_EVENT: broadcast_event(a->text); continue;
            default: continue;
            }
            if (a->kind != ACT_DRIVE && verbose) logf_("-> %s", line);
            int r = motor_cmd(line, reply, sizeof reply);
            if (a->kind == ACT_STOP) {
                if (r == 0 && stops_baseline >= 0) stops_baseline++;
                continue;
            }
            if (r < 0) {
                if (goto_sm_busy(&sm)) fail_now("no motor server");
                continue;
            }
            if (!strncmp(reply, "ERR", 3)) {
                /* BEEP/LED unsupported is harmless; motion refused is not */
                if (a->kind == ACT_DRIVE || a->kind == ACT_TURN) {
                    logf_("motor refused '%s': %s", line, reply);
                    char why[48];
                    snprintf(why, sizeof why, "%.40s", reply + 3 + (reply[3] == ' '));
                    for (char *p = why; *p; p++) if (*p == ':') *p = '\0';
                    if (goto_sm_busy(&sm)) fail_now(why);
                }
                continue;
            }
            if (a->kind == ACT_TURN) {
                int id = -1, est = 0;
                if (sscanf(reply, "OK %d %d", &id, &est) == 2) goto_sm_move_started(&sm, now_s(), id, est / 1000.0);
                else if (goto_sm_busy(&sm)) fail_now("bad TURN reply");
            }
        }
    }
}

static void fail_now(const char *why)
{
    goto_sm_fail(&sm, now_s(), why);
    /* run_actions loops until the queue is empty */
}

static long read_stops(void)
{
    char reply[1024], v[32];
    if (motor_cmd("STATUS", reply, sizeof reply) < 0 || strncmp(reply, "OK", 2)) return -2;
    if (kv(reply, "stops", v, sizeof v) < 0) return -1;   /* older server / bridge: no counter */
    return atol(v);
}

static void poll_status(void)
{
    char reply[1024], v[32], move[16] = "", why[32] = "";
    if (motor_cmd("STATUS", reply, sizeof reply) < 0) {
        if (goto_sm_busy(&sm)) { fail_now("no motor server"); run_actions(); }
        return;
    }
    if (strncmp(reply, "OK", 2)) return;
    if (stops_baseline >= 0 && kv(reply, "stops", v, sizeof v) == 0 && atol(v) > stops_baseline) {
        logf_("stop from another client (stops=%s) -> cancel", v);
        stops_baseline = atol(v);
        goto_sm_cancel(&sm, now_s(), "stop");
        run_actions();
        return;
    }
    if (kv(reply, "tipped", v, sizeof v) == 0 && atoi(v)) { fail_now("tipped"); run_actions(); return; }
    if (goto_sm_wants_status(&sm) && kv(reply, "move", move, sizeof move) == 0 &&
        kv(reply, "move_id", v, sizeof v) == 0) {
        kv(reply, "move_why", why, sizeof why);
        goto_sm_move_status(&sm, now_s(), atoi(v), move, why);
        run_actions();
    }
}

static void handle_ctl(conn *c, char *line)
{
    char cmd[16] = "", reply[512];
    sscanf(line, "%15s", cmd);
    const char *arg = line + strlen(cmd);
    while (*arg == ' ') arg++;
    if (!cmd[0]) return;
    if (!strcasecmp(cmd, "CANCEL") || !strcasecmp(cmd, "STOP") || !strcasecmp(cmd, "S")) {
        goto_sm_cancel(&sm, now_s(), "cancel");    /* S goes out before we reply */
        run_actions();
        send_str(c->fd, "OK\n");
    } else if (!strcasecmp(cmd, "GOTO")) {
        int t = goto_objects_resolve(&objs, arg);
        if (t < 0) {
            snprintf(reply, sizeof reply, "ERR unknown object '%s' (try OBJECTS)\n", arg);
            send_str(c->fd, reply);
            char r2[64];
            motor_cmd("LED no", r2, sizeof r2);
            motor_cmd("BEEP 3", r2, sizeof r2);
            return;
        }
        if (motor_connect() < 0) { send_str(c->fd, "ERR no motor server (is tars_motor running?)\n"); return; }
        long st = read_stops();
        stops_baseline = st >= 0 ? st : -1;
        if (st == -2) { send_str(c->fd, "ERR no motor server\n"); return; }
        goto_sm_goto(&sm, now_s(), t);
        logf_("GOTO '%s' -> %s", arg, objs.obj[t]);
        snprintf(reply, sizeof reply, "OK goto %s\n", objs.obj[t]);
        send_str(c->fd, reply);
        run_actions();
    } else if (!strcasecmp(cmd, "STATUS")) {
        char d[400];
        sm.now = now_s();
        goto_sm_describe(&sm, d, sizeof d);
        snprintf(reply, sizeof reply, "OK %s motor=%d\n", d, motor_fd >= 0);
        send_str(c->fd, reply);
    } else if (!strcasecmp(cmd, "OBJECTS")) {
        size_t o = (size_t)snprintf(reply, sizeof reply, "OK ");
        for (int i = 0; i < objs.n_obj && o < sizeof reply - 40; i++)
            o += (size_t)snprintf(reply + o, sizeof reply - o, "%s%s", i ? "," : "", objs.obj[i]);
        snprintf(reply + o, sizeof reply - o, "\n");
        send_str(c->fd, reply);
    } else if (!strcasecmp(cmd, "RESOLVE")) {
        int t = goto_objects_resolve(&objs, arg);
        if (t < 0) snprintf(reply, sizeof reply, "ERR unknown object '%s'\n", arg);
        else snprintf(reply, sizeof reply, "OK %s\n", objs.obj[t]);
        send_str(c->fd, reply);
    } else if (!strcasecmp(cmd, "WATCH")) {
        c->watch = 1;
        send_str(c->fd, "OK\n");
    } else if (!strcasecmp(cmd, "PING")) {
        send_str(c->fd, "OK PONG\n");
    } else {
        snprintf(reply, sizeof reply, "ERR unknown command '%s'\n", cmd);
        send_str(c->fd, reply);
    }
}

static void handle_det_line(conn *c, char *line)
{
    (void)c;
    goto_frame f;
    if (!line[0]) return;
    if (goto_parse_frame(line, &f) < 0) {
        static double last_warn;
        if (now_s() - last_warn > 5 * time_scale) { logf_("bad detection line: %.80s", line); last_warn = now_s(); }
        return;
    }
    goto_sm_frame(&sm, now_s(), &f);
    run_actions();
}

/* ----------------------------------------------------------------- setup */
static int listen_tcp(int port, int loopback_only)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(loopback_only ? INADDR_LOOPBACK : INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(fd, 8) < 0) { close(fd); return -1; }
    return fd;
}

static int bind_udp(int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) < 0) { close(fd); return -1; }
    return fd;
}

static void accept_into(int lfd, conn *arr, int n, const char *what)
{
    int fd = accept(lfd, NULL, NULL);
    if (fd < 0) return;
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
    for (int i = 0; i < n; i++)
        if (arr[i].fd < 0) { arr[i].fd = fd; arr[i].used = 0; arr[i].watch = 0; if (verbose) logf_("%s client connected", what); return; }
    send_str(fd, "ERR too many clients\n");
    close(fd);
}

static void on_signal(int sig) { (void)sig; running = 0; }

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [-m host:port] [-p ctl_port] [-u det_port] [-i] [-c goto.conf]\n"
            "          [-o objects.txt] [-r rt_prio] [-v] [-T time_scale]\n"
            "  -m  tars_motor address (default 127.0.0.1:5555)\n"
            "  -p  control port, GOTO/CANCEL/STATUS (default 5557, all interfaces)\n"
            "  -u  detections port, UDP + TCP on 127.0.0.1 (default 5558)\n"
            "  -i  also read detection lines from stdin (detect | goto -i)\n"
            "  -r  SCHED_RR priority (default 20, below tars_motor's 30; 0 = don't)\n"
            "  -T  run the clock T times faster (simulator tests only)\n", argv0);
}

int main(int argc, char **argv)
{
    int ctl_port = 5557, det_port = 5558, use_stdin = 0, rt_prio = 20;
    const char *cfg_path = NULL, *obj_path = NULL;
    int opt;
    while ((opt = getopt(argc, argv, "m:p:u:ic:o:r:vT:h")) != -1) {
        switch (opt) {
        case 'm': {
            char *colon = strrchr(optarg, ':');
            if (colon) { *colon = '\0'; motor_port = atoi(colon + 1); }
            if (*optarg) snprintf(motor_host, sizeof motor_host, "%s", optarg);
            break;
        }
        case 'p': ctl_port = atoi(optarg); break;
        case 'u': det_port = atoi(optarg); break;
        case 'i': use_stdin = 1; break;
        case 'c': cfg_path = optarg; break;
        case 'o': obj_path = optarg; break;
        case 'r': rt_prio = atoi(optarg); break;
        case 'v': verbose++; break;
        case 'T': time_scale = atof(optarg); if (time_scale <= 0) time_scale = 1; break;
        default: usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    goto_cfg_defaults(&cfg);
    if (cfg_path) {
        char err[128];
        if (goto_cfg_load(&cfg, cfg_path, err, sizeof err) < 0) { fprintf(stderr, "goto: can't read %s\n", cfg_path); return 1; }
        if (err[0]) fprintf(stderr, "goto: %s: %s (ignored)\n", cfg_path, err);
    }
    goto_objects_defaults(&objs);
    if (obj_path && goto_objects_load(&objs, obj_path) < 0) { fprintf(stderr, "goto: can't read %s\n", obj_path); return 1; }
    goto_sm_init(&sm, &cfg, &objs);

    if (rt_prio > 0) {
        struct sched_param sp = { .sched_priority = rt_prio };
        int e = pthread_setschedparam(pthread_self(), SCHED_RR, &sp);
        if (e) fprintf(stderr, "goto: SCHED_RR %d not set (%s), running at normal priority\n", rt_prio, strerror(e));
    }

    int lctl = listen_tcp(ctl_port, 0);
    int ldet = listen_tcp(det_port, 1);
    int udet = bind_udp(det_port);
    if (lctl < 0 || ldet < 0 || udet < 0) {
        fprintf(stderr, "goto: can't open ports ctl=%d det=%d (%s)\n", ctl_port, det_port, strerror(errno));
        return 1;
    }
    for (int i = 0; i < MAX_CTL; i++) ctl[i].fd = -1;
    for (int i = 0; i < MAX_DETC; i++) detc[i].fd = -1;
    conn in = { .fd = use_stdin ? STDIN_FILENO : -1 };

    fprintf(stderr, "goto: motor=%s:%d ctl=%d det=udp+tcp:%d%s objects=%d hfov=%.0f rt=%d T=%.1f\n",
            motor_host, motor_port, ctl_port, det_port, use_stdin ? "+stdin" : "", objs.n_obj,
            cfg.hfov_deg, rt_prio, time_scale);
    motor_connect();

    while (running) {
        struct pollfd p[4 + MAX_CTL + MAX_DETC];
        int map[4 + MAX_CTL + MAX_DETC];   /* 0 lctl 1 ldet 2 udet 3 stdin, 10+i ctl, 20+i detc */
        int n = 0;
        p[n] = (struct pollfd){lctl, POLLIN, 0}; map[n++] = 0;
        p[n] = (struct pollfd){ldet, POLLIN, 0}; map[n++] = 1;
        p[n] = (struct pollfd){udet, POLLIN, 0}; map[n++] = 2;
        if (in.fd >= 0) { p[n] = (struct pollfd){in.fd, POLLIN, 0}; map[n++] = 3; }
        for (int i = 0; i < MAX_CTL; i++) if (ctl[i].fd >= 0) { p[n] = (struct pollfd){ctl[i].fd, POLLIN, 0}; map[n++] = 10 + i; }
        for (int i = 0; i < MAX_DETC; i++) if (detc[i].fd >= 0) { p[n] = (struct pollfd){detc[i].fd, POLLIN, 0}; map[n++] = 20 + i; }

        double tick = 1.0 / cfg.tick_hz;
        int timeout_ms = (int)(tick / 4 / time_scale * 1000);
        if (timeout_ms < 1) timeout_ms = 1;
        int r = poll(p, (nfds_t)n, goto_sm_busy(&sm) ? timeout_ms : 200);
        if (r < 0 && errno != EINTR) break;

        for (int k = 0; r > 0 && k < n; k++) {
            if (!(p[k].revents & (POLLIN | POLLHUP | POLLERR))) continue;
            int m = map[k];
            if (m == 0) accept_into(lctl, ctl, MAX_CTL, "control");
            else if (m == 1) accept_into(ldet, detc, MAX_DETC, "detection");
            else if (m == 2) {
                char buf[LINE_MAX_];
                ssize_t len = recv(udet, buf, sizeof buf - 1, 0);
                if (len <= 0) continue;
                buf[len] = '\0';
                char *save = NULL;
                for (char *l = strtok_r(buf, "\n", &save); l; l = strtok_r(NULL, "\n", &save)) handle_det_line(NULL, l);
            } else if (m == 3) {
                ssize_t len = read(in.fd, in.buf + in.used, sizeof in.buf - 1 - in.used);
                if (len <= 0) { in.fd = -1; logf_("stdin closed"); continue; }
                in.used += (size_t)len;
                in.buf[in.used] = '\0';
                char *start = in.buf, *nl;
                while ((nl = strchr(start, '\n')) != NULL) { *nl = '\0'; handle_det_line(&in, start); start = nl + 1; }
                in.used = strlen(start);
                memmove(in.buf, start, in.used + 1);
                if (in.used >= sizeof in.buf - 1) in.used = 0;
            } else if (m >= 20) {
                conn *c = &detc[m - 20];
                if (conn_read(c, handle_det_line) < 0) { close(c->fd); c->fd = -1; }
            } else if (m >= 10) {
                conn *c = &ctl[m - 10];
                if (conn_read(c, handle_ctl) < 0) { close(c->fd); c->fd = -1; }
            }
        }

        /* periodic: tick (D refresh, lost/timeouts) and STATUS polling */
        double t = now_s();
        goto_sm_tick(&sm, t);
        run_actions();
        if (goto_sm_busy(&sm) && t >= next_status_poll) {
            next_status_poll = t + (goto_sm_wants_status(&sm) ? 0.1 : 0.25);
            poll_status();
        }
        /* keep the motor link up so GOTO starts instantly */
        if (motor_fd < 0) motor_connect();
        else {
            /* notice a server that went away while we were idle */
            struct pollfd mp = {motor_fd, POLLIN, 0};
            if (poll(&mp, 1, 0) > 0) {
                char tmp[256];
                ssize_t len = recv(motor_fd, tmp, sizeof tmp, MSG_PEEK);
                if (len <= 0) motor_close("closed");
            }
        }
    }
    /* leaving: stop the wheels if we were driving them */
    if (goto_sm_busy(&sm)) { goto_sm_cancel(&sm, now_s(), "exit"); run_actions(); }
    if (motor_fd >= 0) close(motor_fd);
    fprintf(stderr, "goto: bye\n");
    return 0;
}
