/* mission_relay: one Wi-Fi port for the laptop mission control. QNX native, normal priority.
 *
 * The robot never needs this (or the laptop). It taps the robot's localhost services
 * read-only and passes the laptop's control lines through, best effort:
 *
 *   laptop ──TCP :5575 (all interfaces)──► mission_relay ──► 127.0.0.1 services
 *
 * Each laptop connection starts with one line:  HELLO <channel> <key>   (-> "OK relay <channel>")
 *   MOTOR   byte pipe to tars_motor   :5555  (its own connection: a laptop that disconnects
 *                                             stops only the drive it started)
 *   GOTO    byte pipe to goto         :5570
 *   VOICE   byte pipe to tars_voice   :5574  (HEAR <text>, AUDIO <bytes> + PCM, ...)
 *   WATCH   state stream, one tagged line each, polled ONCE for all watchers:
 *             M <tars_motor STATUS reply>      5 Hz
 *             G <goto STATUS reply>            5 Hz
 *             Y <goto DETS reply>              5 Hz (last detection frame)
 *             O <goto MEMORY reply>            1 Hz
 *             N <goto NAV reply>               1 Hz
 *             T <goto NAV ROUTE reply>         1 Hz
 *             K <goto NAV GRID reply>          map_hz (1 Hz)
 *             E <goto EVENT line>              as they happen
 *             V <tars_voice event JSON>        as they happen
 *             P <phone status JSON>            <= phone_hz (1 Hz), from the phone tee
 *             R <relay status>                 1 Hz: which upstreams are up, ages, drops
 *   CAM     latest phone RGB frame from the frame broker (:5566 "GET"), <= cam_fps (5),
 *           passed through as the broker sent it: 4-byte BE length + "RGB1" + u64 id + JPEG.
 *           No decode, no re-encode.
 * Phone tee: 127.0.0.1:5576 TCP, newline-delimited JSON (the status the iPhone gets; sent by
 *   iter2-autonomy/dashboard/headless.py when TARS_MISSION_TEE_PORT is set). Latest line kept.
 *
 * Never blocks or slows a robot process:
 *   - every socket is non-blocking; one thread, poll()
 *   - upstream replies are ALWAYS read at once (goto's and tars_motor's sends never back up)
 *   - a laptop that doesn't read: WATCH drops whole lines, CAM skips frames, a pipe is closed
 *     (closing a MOTOR pipe = tars_motor stops that laptop's drive)
 *   - pollers connect only while someone is watching; at most 8 polls outstanding per service
 *
 * Key: REQUIRED. -k <key> or the MISSION_KEY env var (-k wins). Every HELLO must carry it;
 *   a missing or wrong key gets "ERR key" and the connection is closed. Without a key the
 *   relay refuses to start, unless --no-key (local demo stack only: logs a loud warning).
 *
 * Usage: mission_relay [-p 5575] [-b 0.0.0.0] (-k key | MISSION_KEY=key | --no-key) [-v]
 *          [--motor 5555] [--goto 5570] [--voice 5574] [--rgb 5566] [--tee 5576]
 *          [--cam-fps 5] [--map-hz 1] [--phone-hz 1] [--status-hz 5]
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define MAX_LAPTOP     24
#define HELLO_MAX      256
#define HELLO_TIMEOUT  3.0
#define PIPE_UP_LIM    (1024 * 1024)       /* laptop -> service (an AUDIO clip is <= 480 KB) */
#define PIPE_DOWN_LIM  (1024 * 1024)       /* service -> laptop; over this = laptop not reading */
static size_t WATCH_LIM = 2 * 1024 * 1024;   /* per watcher queue; --watch-lim (tests) */
#define POLL_RX_MAX    (1024 * 1024)       /* longest line from a service (NAV GRID worst case) */
#define TEE_RX_MAX     (4 * 1024 * 1024)
#define RGB_MAX        2000012
#define MAX_OUTSTANDING 8

static volatile sig_atomic_t running = 1;
static int verbose;

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static void logv(const char *fmt, ...)
{
    if (!verbose) return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[relay %.2f] ", now_s());
    vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
    va_end(ap);
}

/* ------------------------------------------------------------------ buffers */
typedef struct { unsigned char *b; size_t n, cap; } buf_t;

static int buf_add(buf_t *q, const void *d, size_t n, size_t lim)
{
    if (q->n + n > lim) return -1;
    if (q->n + n > q->cap) {
        size_t c = q->cap ? q->cap : 4096;
        while (c < q->n + n) c *= 2;
        unsigned char *nb = realloc(q->b, c);
        if (!nb) return -1;
        q->b = nb;
        q->cap = c;
    }
    memcpy(q->b + q->n, d, n);
    q->n += n;
    return 0;
}

static void buf_eat(buf_t *q, size_t n)
{
    memmove(q->b, q->b + n, q->n - n);
    q->n -= n;
    if (!q->n && q->cap > 65536) { free(q->b); q->b = NULL; q->cap = 0; }   /* give big buffers back */
}

static void buf_free(buf_t *q) { free(q->b); q->b = NULL; q->n = q->cap = 0; }

/* write what the socket takes now. -1 = connection broken */
static int buf_flush(int fd, buf_t *q)
{
    while (q->n) {
        ssize_t s = send(fd, q->b, q->n, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (s > 0) { buf_eat(q, (size_t)s); continue; }
        if (s < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return 0;
        return -1;
    }
    return 0;
}

static void set_sock(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
}

/* non-blocking connect to 127.0.0.1:port. *pending = 1 while it's still connecting */
static int connect_local(int port, int *pending)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    set_sock(fd);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    *pending = 0;
    if (connect(fd, (struct sockaddr *)&a, sizeof a) == 0) return fd;
    if (errno == EINPROGRESS) { *pending = 1; return fd; }
    close(fd);
    return -1;
}

static int connect_done(int fd)
{
    int err = 0;
    socklen_t l = sizeof err;
    return getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &l) == 0 && err == 0;
}

static int listen_on(const char *host, int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)port);
    if (inet_pton(AF_INET, host, &a.sin_addr) != 1 || bind(fd, (struct sockaddr *)&a, sizeof a) < 0 ||
        listen(fd, 16) < 0) { close(fd); return -1; }
    set_sock(fd);
    return fd;
}

/* ------------------------------------------------------------------ config */
static struct {
    int port, motor, go, voice, rgb, tee;
    double cam_fps, map_hz, phone_hz, status_hz;
    char bind[64], key[128];
} cfg = { 5575, 5555, 5570, 5574, 5566, 5576, 5, 1, 1, 5, "0.0.0.0", "" };

/* ------------------------------------------------------------------ laptops */
enum { CH_HELLO = 0, CH_MOTOR, CH_GOTO, CH_VOICE, CH_WATCH, CH_CAM };
static const char *ch_name[] = { "HELLO", "MOTOR", "GOTO", "VOICE", "WATCH", "CAM" };

typedef struct {
    int fd, ch;
    double t0;
    char hello[HELLO_MAX];
    size_t hello_n;
    int up, up_pending;        /* pipes: the service connection */
    buf_t out, upout;          /* to the laptop, to the service */
    int closing;               /* flush out, then close */
    long drops;
} lap_t;

static lap_t lap[MAX_LAPTOP];
static long total_drops, cam_frames_sent, cam_frames_skipped;

static void lap_close(lap_t *l, const char *why)
{
    if (l->fd < 0) return;
    logv("laptop %s closed: %s", ch_name[l->ch], why);
    close(l->fd);
    if (l->up >= 0) close(l->up);
    buf_free(&l->out);
    buf_free(&l->upout);
    memset(l, 0, sizeof *l);
    l->fd = l->up = -1;
}

static void lap_say(lap_t *l, const char *s, int then_close)
{
    buf_add(&l->out, s, strlen(s), WATCH_LIM);
    if (then_close) l->closing = 1;
}

static int count_ch(int ch)
{
    int n = 0;
    for (int i = 0; i < MAX_LAPTOP; i++) if (lap[i].fd >= 0 && lap[i].ch == ch) n++;
    return n;
}

/* one tagged line to every watcher; a watcher whose queue is full misses this line */
static void broadcast(char tag, const char *line, size_t n)
{
    for (int i = 0; i < MAX_LAPTOP; i++) {
        lap_t *l = &lap[i];
        if (l->fd < 0 || l->ch != CH_WATCH || l->closing) continue;
        if (l->out.n + n + 3 > WATCH_LIM) { l->drops++; total_drops++; continue; }
        char h[2] = { tag, ' ' };
        buf_add(&l->out, h, 2, WATCH_LIM);
        buf_add(&l->out, line, n, WATCH_LIM);
        buf_add(&l->out, "\n", 1, WATCH_LIM);
    }
}

static void hello_done(lap_t *l)
{
    char ch[16] = "", key[128] = "";
    l->hello[l->hello_n] = '\0';
    if (sscanf(l->hello, "HELLO %15s %127s", ch, key) < 1) { lap_say(l, "ERR expected: HELLO <MOTOR|GOTO|VOICE|WATCH|CAM> <key>\n", 1); return; }
    if (cfg.key[0] && strcmp(key, cfg.key)) { lap_say(l, "ERR key\n", 1); return; }
    int c = 0;
    for (int i = CH_MOTOR; i <= CH_CAM; i++) if (!strcasecmp(ch, ch_name[i])) c = i;
    if (!c) { lap_say(l, "ERR unknown channel\n", 1); return; }
    l->ch = c;
    if (c == CH_MOTOR || c == CH_GOTO || c == CH_VOICE) {
        int port = c == CH_MOTOR ? cfg.motor : c == CH_GOTO ? cfg.go : cfg.voice;
        l->up = connect_local(port, &l->up_pending);
        if (l->up < 0) {
            char e[96];
            snprintf(e, sizeof e, "ERR relay: %s unavailable\n", ch_name[c]);
            lap_say(l, e, 1);
            return;
        }
        if (l->up_pending) return;             /* "OK relay ..." once the service answers the connect */
    }
    char ok[48];
    snprintf(ok, sizeof ok, "OK relay %s\n", ch_name[c]);
    lap_say(l, ok, 0);
    logv("laptop %s ready", ch_name[c]);
}

/* bytes from a laptop */
static void lap_read(lap_t *l)
{
    unsigned char b[16384];
    size_t room = sizeof b;
    if (l->ch == CH_MOTOR || l->ch == CH_GOTO || l->ch == CH_VOICE) {
        if (l->upout.n >= PIPE_UP_LIM) return;                /* backpressure on the laptop */
        if (PIPE_UP_LIM - l->upout.n < room) room = PIPE_UP_LIM - l->upout.n;
    }
    ssize_t r = recv(l->fd, b, room, MSG_DONTWAIT);
    if (r == 0) { lap_close(l, "laptop disconnected"); return; }
    if (r < 0) { if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) lap_close(l, "laptop error"); return; }
    size_t o = 0;
    while (l->ch == CH_HELLO && o < (size_t)r && !l->closing) {
        char c = (char)b[o++];
        if (c == '\n') { hello_done(l); break; }
        if (c != '\r') {
            if (l->hello_n >= HELLO_MAX - 1) { lap_say(l, "ERR hello too long\n", 1); return; }
            l->hello[l->hello_n++] = c;
        }
    }
    if (l->closing || o >= (size_t)r) return;
    if (l->ch == CH_MOTOR || l->ch == CH_GOTO || l->ch == CH_VOICE)
        buf_add(&l->upout, b + o, (size_t)r - o, PIPE_UP_LIM);
    /* WATCH / CAM: whatever the laptop sends is ignored */
}

/* bytes from a pipe's service: always read at once */
static void up_read(lap_t *l)
{
    unsigned char b[16384];
    for (;;) {
        ssize_t r = recv(l->up, b, sizeof b, MSG_DONTWAIT);
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return;
        if (r <= 0) {                                 /* service closed: flush, then close the laptop */
            close(l->up);
            l->up = -1;
            lap_say(l, "", 1);
            return;
        }
        if (buf_add(&l->out, b, (size_t)r, PIPE_DOWN_LIM) < 0) { lap_close(l, "laptop not reading its pipe"); return; }
    }
}

/* ------------------------------------------------------------------ pollers (relay's own service links) */
typedef struct {
    const char *name;
    int fd, pending, port;
    double next_try, last_rx;
    buf_t out, rx;
    char tags[MAX_OUTSTANDING * 4];
    int ntag;
    int up;                    /* connected and greeted */
} poller_t;

static poller_t pm = { "motor", -1, 0, 0, 0, 0, {0}, {0}, "", 0, 0 };
static poller_t pg = { "goto", -1, 0, 0, 0, 0, {0}, {0}, "", 0, 0 };
static poller_t pv = { "voice", -1, 0, 0, 0, 0, {0}, {0}, "", 0, 0 };

static void poller_close(poller_t *p, const char *why)
{
    if (p->fd < 0) return;
    logv("%s poller closed: %s", p->name, why);
    close(p->fd);
    p->fd = -1;
    p->up = 0;
    p->ntag = 0;
    buf_free(&p->out);
    buf_free(&p->rx);
    p->next_try = now_s() + 1.0;
}

static void poller_cmd(poller_t *p, char tag, const char *line)
{
    if (p->fd < 0 || p->ntag >= (int)sizeof p->tags) return;
    if (buf_add(&p->out, line, strlen(line), 65536) < 0) return;
    p->tags[p->ntag++] = tag;
}

static void poller_open(poller_t *p, int want)
{
    if (!want) { poller_close(p, "nobody watching"); return; }
    if (p->fd >= 0 || now_s() < p->next_try) return;
    p->fd = connect_local(p->port, &p->pending);
    if (p->fd < 0) { p->next_try = now_s() + 1.0; return; }
    p->next_try = now_s() + 1.0;
    if (p == &pm) poller_cmd(p, 'x', "WHO mission-relay\n");   /* only STATUS after this: never moves */
    else poller_cmd(p, 'x', "WATCH\n");
}

static void poller_line(poller_t *p, char *line, size_t n)
{
    p->last_rx = now_s();
    if (p == &pv && line[0] == '{') { broadcast('V', line, n); return; }
    if (p == &pg && !strncmp(line, "EVENT ", 6)) { broadcast('E', line, n); return; }
    if (!p->ntag) return;                         /* unsolicited: ignore */
    char tag = p->tags[0];
    memmove(p->tags, p->tags + 1, (size_t)--p->ntag);
    p->up = 1;
    if (tag != 'x') broadcast(tag, line, n);
}

static void poller_read(poller_t *p)
{
    unsigned char b[65536];
    for (;;) {
        ssize_t r = recv(p->fd, b, sizeof b, MSG_DONTWAIT);
        if (r == 0) { poller_close(p, "service closed"); return; }
        if (r < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
            poller_close(p, strerror(errno));
            return;
        }
        if (buf_add(&p->rx, b, (size_t)r, POLL_RX_MAX) < 0) { poller_close(p, "line too long"); return; }
        size_t start = 0;
        for (size_t i = 0; i < p->rx.n; i++) {
            if (p->rx.b[i] != '\n') continue;
            size_t n = i - start;
            if (n && p->rx.b[start + n - 1] == '\r') n--;
            p->rx.b[start + n] = '\0';
            poller_line(p, (char *)p->rx.b + start, n);
            if (p->fd < 0) return;
            start = i + 1;
        }
        if (start) buf_eat(&p->rx, start);
    }
}

/* ------------------------------------------------------------------ phone tee */
static int tee_fd = -1;
static buf_t tee_rx;
static char *phone_latest;
static size_t phone_n;
static double phone_t = -1, phone_sent_t;
static int phone_new, tee_skip;

static void tee_read(void)
{
    unsigned char b[65536];
    for (;;) {
        ssize_t r = recv(tee_fd, b, sizeof b, MSG_DONTWAIT);
        if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
            close(tee_fd); tee_fd = -1; buf_free(&tee_rx); logv("phone tee closed"); return;
        }
        if (r < 0) return;
        if (buf_add(&tee_rx, b, (size_t)r, TEE_RX_MAX) < 0) {   /* a monster line: drop it, to its end */
            buf_free(&tee_rx);
            tee_skip = 1;
            continue;
        }
        size_t start = 0, last_s = 0, last_e = 0;
        int got = 0;
        for (size_t i = 0; i < tee_rx.n; i++) {
            if (tee_rx.b[i] != '\n') continue;
            if (tee_skip) tee_skip = 0;              /* the tail of the dropped line */
            else { last_s = start; last_e = i; got = 1; }
            start = i + 1;
        }
        if (got && last_e > last_s) {                 /* keep only the newest complete line */
            char *nl = realloc(phone_latest, last_e - last_s + 1);
            if (nl) {
                phone_latest = nl;
                memcpy(phone_latest, tee_rx.b + last_s, last_e - last_s);
                phone_n = last_e - last_s;
                phone_latest[phone_n] = '\0';
                phone_t = now_s();
                phone_new = 1;
            }
        }
        if (start) buf_eat(&tee_rx, start);
    }
}

/* ------------------------------------------------------------------ camera (frame broker) */
static struct {
    int fd, pending, sent_get;
    double started, next, last_ok;
    unsigned char hdr[4];
    size_t hdr_n;
    unsigned char *frame;
    size_t len, got;
    long gets, frames, empty, errors;
} cam = { -1, 0, 0, 0, 0, -1, {0}, 0, NULL, 0, 0, 0, 0, 0, 0 };

static void cam_end(const char *why)
{
    if (cam.fd >= 0) close(cam.fd);
    cam.fd = -1;
    free(cam.frame);
    cam.frame = NULL;
    cam.hdr_n = cam.len = cam.got = 0;
    cam.sent_get = 0;
    if (why) { cam.errors++; logv("camera: %s", why); }
}

static int cam_ready_watchers(void)
{
    int n = 0;
    for (int i = 0; i < MAX_LAPTOP; i++)
        if (lap[i].fd >= 0 && lap[i].ch == CH_CAM && !lap[i].closing && lap[i].out.n == 0) n++;
    return n;
}

static void cam_deliver(void)
{
    uint32_t be = htonl((uint32_t)cam.len);
    for (int i = 0; i < MAX_LAPTOP; i++) {
        lap_t *l = &lap[i];
        if (l->fd < 0 || l->ch != CH_CAM || l->closing) continue;
        if (l->out.n) { l->drops++; cam_frames_skipped++; continue; }   /* still sending the last one */
        if (buf_add(&l->out, &be, 4, RGB_MAX + 16) < 0 || buf_add(&l->out, cam.frame, cam.len, RGB_MAX + 16) < 0) {
            l->out.n = 0;
            continue;
        }
        cam_frames_sent++;
    }
}

static void cam_tick(double t)
{
    if (cam.fd >= 0 && t - cam.started > 1.0) cam_end("broker timeout");
    if (cam.fd >= 0 || t < cam.next || cfg.cam_fps <= 0 || !cam_ready_watchers()) return;
    cam.next = t + 1.0 / cfg.cam_fps;
    cam.fd = connect_local(cfg.rgb, &cam.pending);
    if (cam.fd < 0) { cam.errors++; return; }
    cam.started = t;
    cam.gets++;
}

static void cam_io(short rev)
{
    if (cam.pending) {
        if (!(rev & (POLLOUT | POLLERR | POLLHUP))) return;
        if (!connect_done(cam.fd)) { cam_end("broker down"); return; }
        cam.pending = 0;
    }
    if (!cam.sent_get) {
        if (send(cam.fd, "GET\n", 4, MSG_NOSIGNAL | MSG_DONTWAIT) != 4) { cam_end("broker send"); return; }
        cam.sent_get = 1;
        return;
    }
    for (;;) {
        ssize_t r;
        if (cam.hdr_n < 4) {
            r = recv(cam.fd, cam.hdr + cam.hdr_n, 4 - cam.hdr_n, MSG_DONTWAIT);
            if (r > 0) {
                cam.hdr_n += (size_t)r;
                if (cam.hdr_n == 4) {
                    cam.len = (size_t)cam.hdr[0] << 24 | (size_t)cam.hdr[1] << 16 | (size_t)cam.hdr[2] << 8 | cam.hdr[3];
                    if (cam.len == 0) { cam.empty++; cam_end(NULL); return; }       /* no fresh frame */
                    if (cam.len < 16 || cam.len > RGB_MAX) { cam_end("bad frame size"); return; }
                    cam.frame = malloc(cam.len);
                    if (!cam.frame) { cam_end("out of memory"); return; }
                }
                continue;
            }
        } else {
            r = recv(cam.fd, cam.frame + cam.got, cam.len - cam.got, MSG_DONTWAIT);
            if (r > 0) {
                cam.got += (size_t)r;
                if (cam.got == cam.len) {
                    if (memcmp(cam.frame, "RGB1", 4) || cam.frame[12] != 0xff || cam.frame[13] != 0xd8) {
                        cam_end("bad frame envelope");
                        return;
                    }
                    cam.frames++;
                    cam.last_ok = now_s();
                    cam_deliver();
                    cam_end(NULL);
                    return;
                }
                continue;
            }
        }
        if (r == 0) { cam_end("broker closed early"); return; }
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
        cam_end("broker read");
        return;
    }
}

/* ------------------------------------------------------------------ main */
static void on_sig(int s) { (void)s; running = 0; }

static void usage(void)
{
    fprintf(stderr, "usage: mission_relay [-p 5575] [-b 0.0.0.0] (-k key | env MISSION_KEY | --no-key) [-v]\n"
                    "         [--motor 5555] [--goto 5570]\n"
                    "         [--voice 5574] [--rgb 5566] [--tee 5576] [--cam-fps 5] [--map-hz 1] [--phone-hz 1]\n"
                    "         [--status-hz 5] [--watch-lim bytes]\n");
}

static void schedule_polls(double t)
{
    static double next_fast, next_slow, next_map, next_status;
    double fast = cfg.status_hz > 0 ? 1.0 / cfg.status_hz : 0.2;
    if (t >= next_fast) {
        next_fast = t + fast;
        if (pm.fd >= 0 && !pm.pending && pm.ntag < 4) poller_cmd(&pm, 'M', "STATUS\n");
        if (pg.fd >= 0 && !pg.pending && pg.ntag < MAX_OUTSTANDING - 4) {
            poller_cmd(&pg, 'G', "STATUS\n");
            poller_cmd(&pg, 'Y', "DETS\n");
        }
    }
    if (t >= next_slow) {
        next_slow = t + 1.0;
        if (pg.fd >= 0 && !pg.pending && pg.ntag < MAX_OUTSTANDING - 3) {
            poller_cmd(&pg, 'O', "MEMORY\n");
            poller_cmd(&pg, 'N', "NAV\n");
            poller_cmd(&pg, 'T', "NAV ROUTE\n");
        }
    }
    if (cfg.map_hz > 0 && t >= next_map) {
        next_map = t + 1.0 / cfg.map_hz;
        if (pg.fd >= 0 && !pg.pending && pg.ntag < MAX_OUTSTANDING) poller_cmd(&pg, 'K', "NAV GRID\n");
    }
    if (phone_new && phone_latest && cfg.phone_hz > 0 && t - phone_sent_t >= 1.0 / cfg.phone_hz) {
        broadcast('P', phone_latest, phone_n);
        phone_sent_t = t;
        phone_new = 0;
    }
    if (t >= next_status) {
        next_status = t + 1.0;
        size_t queued = 0;
        for (int i = 0; i < MAX_LAPTOP; i++) if (lap[i].fd >= 0) queued += lap[i].out.n + lap[i].upout.n;
        char s[600];
        int n = snprintf(s, sizeof s,
            "watchers=%d cams=%d pipes=%d motor=%d goto=%d voice=%d phone_tee=%d phone_age=%.1f "
            "rgb_age=%.1f rgb_gets=%ld rgb_frames=%ld rgb_empty=%ld rgb_errors=%ld cam_sent=%ld cam_skipped=%ld drops=%ld queued=%zu",
            count_ch(CH_WATCH), count_ch(CH_CAM), count_ch(CH_MOTOR) + count_ch(CH_GOTO) + count_ch(CH_VOICE),
            pm.up && pm.fd >= 0, pg.up && pg.fd >= 0, pv.up && pv.fd >= 0, tee_fd >= 0,
            phone_t < 0 ? -1.0 : t - phone_t, cam.last_ok < 0 ? -1.0 : t - cam.last_ok,
            cam.gets, cam.frames, cam.empty, cam.errors, cam_frames_sent, cam_frames_skipped, total_drops, queued);
        broadcast('R', s, (size_t)n);
    }
}

int main(int argc, char **argv)
{
    int no_key = 0;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "-v")) { verbose = 1; continue; }
        if (!strcmp(a, "--no-key")) { no_key = 1; continue; }
        if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(); return 0; }
        if (!v) { usage(); return 2; }
        i++;
        if (!strcmp(a, "-p")) cfg.port = atoi(v);
        else if (!strcmp(a, "-b")) snprintf(cfg.bind, sizeof cfg.bind, "%s", v);
        else if (!strcmp(a, "-k")) snprintf(cfg.key, sizeof cfg.key, "%s", v);
        else if (!strcmp(a, "--motor")) cfg.motor = atoi(v);
        else if (!strcmp(a, "--goto")) cfg.go = atoi(v);
        else if (!strcmp(a, "--voice")) cfg.voice = atoi(v);
        else if (!strcmp(a, "--rgb")) cfg.rgb = atoi(v);
        else if (!strcmp(a, "--tee")) cfg.tee = atoi(v);
        else if (!strcmp(a, "--cam-fps")) cfg.cam_fps = atof(v);
        else if (!strcmp(a, "--map-hz")) cfg.map_hz = atof(v);
        else if (!strcmp(a, "--phone-hz")) cfg.phone_hz = atof(v);
        else if (!strcmp(a, "--status-hz")) cfg.status_hz = atof(v);
        else if (!strcmp(a, "--watch-lim")) WATCH_LIM = (size_t)atol(v) < 4096 ? 4096 : (size_t)atol(v);
        else { usage(); return 2; }
    }
    if (!cfg.key[0] && getenv("MISSION_KEY")) snprintf(cfg.key, sizeof cfg.key, "%s", getenv("MISSION_KEY"));
    if (strpbrk(cfg.key, " \t\r\n")) { fprintf(stderr, "mission_relay: the key can't contain spaces\n"); return 2; }
    if (cfg.key[0] && no_key) { fprintf(stderr, "mission_relay: a key and --no-key? pick one\n"); return 2; }
    if (!cfg.key[0] && !no_key) {
        fprintf(stderr, "mission_relay: refusing to start without a key: anyone on the Wi-Fi could drive the robot.\n"
                        "  Use -k <key> or MISSION_KEY=<key> (and the same --key / MISSION_KEY on the laptop).\n"
                        "  --no-key is for the local demo stack only.\n");
        return 2;
    }
    if (no_key)
        fprintf(stderr, "mission_relay: ************************************************************\n"
                        "mission_relay: WARNING --no-key: ANY client that reaches port %d can drive the robot.\n"
                        "mission_relay: Local demo / tests only. Never on the robot's Wi-Fi.\n"
                        "mission_relay: ************************************************************\n", cfg.port);
    if (cfg.cam_fps > 5) cfg.cam_fps = 5;              /* hard cap: Wi-Fi + the broker are shared */
    if (cfg.map_hz > 2) cfg.map_hz = 2;
    if (cfg.phone_hz > 2) cfg.phone_hz = 2;
    if (cfg.status_hz > 10) cfg.status_hz = 10;
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    pm.port = cfg.motor;
    pg.port = cfg.go;
    pv.port = cfg.voice;
    for (int i = 0; i < MAX_LAPTOP; i++) lap[i].fd = lap[i].up = -1;

    int lfd = listen_on(cfg.bind, cfg.port);
    if (lfd < 0) { fprintf(stderr, "mission_relay: can't listen on %s:%d: %s\n", cfg.bind, cfg.port, strerror(errno)); return 1; }
    int tfd = cfg.tee > 0 ? listen_on("127.0.0.1", cfg.tee) : -1;
    if (cfg.tee > 0 && tfd < 0) fprintf(stderr, "mission_relay: phone tee 127.0.0.1:%d unavailable (%s)\n", cfg.tee, strerror(errno));
    fprintf(stderr, "mission_relay: laptops on %s:%d%s; motor %d goto %d voice %d rgb %d tee %d; cam <= %.0f fps, map %.1f Hz\n",
            cfg.bind, cfg.port, cfg.key[0] ? " (key required)" : " (NO KEY)", cfg.motor, cfg.go, cfg.voice, cfg.rgb, cfg.tee,
            cfg.cam_fps, cfg.map_hz);

    enum { K_LISTEN, K_TEELISTEN, K_TEE, K_LAP, K_UP, K_POLL, K_CAM };
    struct pollfd p[8 + 2 * MAX_LAPTOP];
    int kind[8 + 2 * MAX_LAPTOP], idx[8 + 2 * MAX_LAPTOP];
    poller_t *polls[3] = { &pm, &pg, &pv };

    while (running) {
        double t = now_s();
        int watching = count_ch(CH_WATCH) > 0;
        for (int i = 0; i < 3; i++) poller_open(polls[i], watching);
        schedule_polls(t);
        cam_tick(t);
        for (int i = 0; i < MAX_LAPTOP; i++) {
            lap_t *l = &lap[i];
            if (l->fd < 0) continue;
            if (l->ch == CH_HELLO && t - l->t0 > HELLO_TIMEOUT) lap_close(l, "no HELLO");
            else if (l->closing && !l->out.n) lap_close(l, "done");
        }

        int n = 0;
        p[n] = (struct pollfd){ lfd, POLLIN, 0 }; kind[n] = K_LISTEN; idx[n++] = 0;
        if (tfd >= 0) { p[n] = (struct pollfd){ tfd, POLLIN, 0 }; kind[n] = K_TEELISTEN; idx[n++] = 0; }
        if (tee_fd >= 0) { p[n] = (struct pollfd){ tee_fd, POLLIN, 0 }; kind[n] = K_TEE; idx[n++] = 0; }
        for (int i = 0; i < 3; i++) {
            poller_t *q = polls[i];
            if (q->fd < 0) continue;
            short ev = POLLIN | (q->pending || q->out.n ? POLLOUT : 0);
            p[n] = (struct pollfd){ q->fd, ev, 0 }; kind[n] = K_POLL; idx[n++] = i;
        }
        if (cam.fd >= 0) {
            p[n] = (struct pollfd){ cam.fd, (short)(cam.pending || !cam.sent_get ? POLLOUT : POLLIN), 0 };
            kind[n] = K_CAM; idx[n++] = 0;
        }
        for (int i = 0; i < MAX_LAPTOP; i++) {
            lap_t *l = &lap[i];
            if (l->fd < 0) continue;
            int full = l->upout.n >= PIPE_UP_LIM;      /* pipe backpressure: stop reading the laptop */
            short ev = (l->closing || full ? 0 : POLLIN) | (l->out.n ? POLLOUT : 0);
            p[n] = (struct pollfd){ l->fd, ev, 0 }; kind[n] = K_LAP; idx[n++] = i;
            if (l->up >= 0) {
                short uev = POLLIN | (l->up_pending || l->upout.n ? POLLOUT : 0);
                p[n] = (struct pollfd){ l->up, uev, 0 }; kind[n] = K_UP; idx[n++] = i;
            }
        }
        int r = poll(p, (nfds_t)n, 20);
        if (r < 0 && errno != EINTR) break;
        if (r <= 0) continue;

        for (int k = 0; k < n; k++) {
            short rev = p[k].revents;
            if (!rev) continue;
            if (kind[k] == K_LISTEN) {
                int fd = accept(lfd, NULL, NULL);
                if (fd < 0) continue;
                int slot = -1;
                for (int i = 0; i < MAX_LAPTOP && slot < 0; i++) if (lap[i].fd < 0) slot = i;
                if (slot < 0) { send(fd, "ERR relay full\n", 15, MSG_NOSIGNAL | MSG_DONTWAIT); close(fd); continue; }
                set_sock(fd);
                memset(&lap[slot], 0, sizeof lap[slot]);
                lap[slot].fd = fd;
                lap[slot].up = -1;
                lap[slot].t0 = now_s();
            } else if (kind[k] == K_TEELISTEN) {
                int fd = accept(tfd, NULL, NULL);
                if (fd < 0) continue;
                if (tee_fd >= 0) close(tee_fd);              /* newest sender wins */
                buf_free(&tee_rx);
                set_sock(fd);
                tee_fd = fd;
                logv("phone tee connected");
            } else if (kind[k] == K_TEE) {
                if (tee_fd >= 0) tee_read();
            } else if (kind[k] == K_POLL) {
                poller_t *q = polls[idx[k]];
                if (q->fd < 0 || p[k].fd != q->fd) continue;
                if (q->pending) {
                    if (!(rev & (POLLOUT | POLLERR | POLLHUP))) continue;
                    if (!connect_done(q->fd)) { poller_close(q, "connect failed"); continue; }
                    q->pending = 0;
                    logv("%s poller connected", q->name);
                }
                if (rev & POLLIN) poller_read(q);
                if (q->fd >= 0 && q->out.n && buf_flush(q->fd, &q->out) < 0) poller_close(q, "send failed");
                if (q->fd >= 0 && (rev & (POLLERR | POLLHUP)) && !(rev & POLLIN)) poller_close(q, "hangup");
            } else if (kind[k] == K_CAM) {
                if (cam.fd >= 0 && p[k].fd == cam.fd) cam_io(rev);
            } else if (kind[k] == K_LAP) {
                lap_t *l = &lap[idx[k]];
                if (l->fd < 0 || p[k].fd != l->fd) continue;
                if (rev & POLLIN) lap_read(l);
                if (l->fd >= 0 && l->out.n && buf_flush(l->fd, &l->out) < 0) lap_close(l, "laptop gone");
                if (l->fd >= 0 && (rev & (POLLERR | POLLHUP)) && !(rev & POLLIN)) lap_close(l, "laptop hangup");
            } else if (kind[k] == K_UP) {
                lap_t *l = &lap[idx[k]];
                if (l->fd < 0 || l->up < 0 || p[k].fd != l->up) continue;
                if (l->up_pending) {
                    if (!(rev & (POLLOUT | POLLERR | POLLHUP))) continue;
                    if (!connect_done(l->up)) {
                        char e[96];
                        snprintf(e, sizeof e, "ERR relay: %s unavailable\n", ch_name[l->ch]);
                        close(l->up); l->up = -1;
                        lap_say(l, e, 1);
                        continue;
                    }
                    l->up_pending = 0;
                    char ok[48];
                    snprintf(ok, sizeof ok, "OK relay %s\n", ch_name[l->ch]);
                    lap_say(l, ok, 0);
                }
                if (rev & POLLIN) up_read(l);
                if (l->fd >= 0 && l->up >= 0 && l->upout.n && buf_flush(l->up, &l->upout) < 0) {
                    close(l->up); l->up = -1; lap_say(l, "", 1);
                }
                if (l->fd >= 0 && l->up >= 0 && (rev & (POLLERR | POLLHUP)) && !(rev & POLLIN)) {
                    close(l->up); l->up = -1; lap_say(l, "", 1);
                }
            }
        }
        /* push queued output (broadcasts, camera frames) even for sockets poll() didn't flag */
        for (int i = 0; i < MAX_LAPTOP; i++)
            if (lap[i].fd >= 0 && lap[i].out.n && buf_flush(lap[i].fd, &lap[i].out) < 0) lap_close(&lap[i], "laptop gone");
        for (int i = 0; i < 3; i++)
            if (polls[i]->fd >= 0 && !polls[i]->pending && polls[i]->out.n && buf_flush(polls[i]->fd, &polls[i]->out) < 0)
                poller_close(polls[i], "send failed");
        for (int i = 0; i < MAX_LAPTOP; i++)
            if (lap[i].fd >= 0 && lap[i].up >= 0 && !lap[i].up_pending && lap[i].upout.n &&
                buf_flush(lap[i].up, &lap[i].upout) < 0) { close(lap[i].up); lap[i].up = -1; lap_say(&lap[i], "", 1); }
    }
    for (int i = 0; i < MAX_LAPTOP; i++) lap_close(&lap[i], "exit");
    for (int i = 0; i < 3; i++) poller_close(polls[i], "exit");
    cam_end(NULL);
    if (tee_fd >= 0) close(tee_fd);
    close(lfd);
    if (tfd >= 0) close(tfd);
    return 0;
}
