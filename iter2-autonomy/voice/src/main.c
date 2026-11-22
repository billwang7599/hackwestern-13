/* tars_voice: offline speech commands for TARS, on the Pi (QNX) or a Mac.
 *
 *   mic (io-audio) -> VAD (20 ms frames) -> segment <= 3 s -> whisper.cpp tiny.en
 *     -> text -> intent_decide() (safety stop, wake word, classifier, slots)
 *     -> lines to tars_motor (TCP 5555): S | MOVE | TURN | STATUS, + LED/BEEP
 *     -> "GOTO <object>" / "CANCEL" to the go-to controller (TCP 5570, iter2-autonomy/goto/)
 *   stop  = S to tars_motor FIRST, then CANCEL to the go-to controller
 *   move  = go-to STATUS; if it isn't idle, CANCEL and wait for its OK (goto's S goes
 *           out before it replies), then MOVE/TURN to tars_motor (manual beats go-to).
 *           An idle go-to gets no CANCEL: its S could land after the move and abort it.
 * Talk button (button_talk=1): a short press of the Uno button (STATUS
 * btn_teach= counter, polled every 250 ms) beeps once and arms listening
 * for wake_carry_ms with NO wake word needed. Best fix for a noisy room.
 * Wake word carry-over: "TARS" ... (pause) ... "turn left" works: a segment
 * that is only the wake word arms the next segment for wake_carry_ms.
 *
 * Usage: tars_voice [-c voice.conf] [--input SPEC] [--model FILE] [--host H] [--port P]
 *                   [--threads N] [--any] [--dry] [--save-dir DIR] [--list]
 *   SPEC: qnx | qnx:C,D | wav:FILE | raw:- | phone[:HOST:PORT]   (see audio.h)
 *   phone = the iPhone on the car streams the AirPods mic (TCP 5577, "MIC1"): the same
 *   VAD -> whisper -> wake word -> intent path as the io-audio mic.
 *   --any   don't require the wake word ("TARS ...")
 *   --dry   print the commands, don't connect to tars_motor
 * Every heard segment is logged as one line:
 *   [heard] "TARS, turn left." vad=1.42s stt=212ms -> command left 0.97 (classifier): TURN -30 | BEEP 1
 * Runs at normal priority (whisper must never starve tars_motor's SCHED_RR loop).
 */
#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include "audio.h"
#include "intent.h"
#include "stt.h"
#include "vad.h"

/* Vocabulary bias for whisper. A plain word list on purpose: a prompt full of
 * "TARS, turn left. TARS, turn right." makes tiny.en hallucinate "TARS, turn."
 * on a bare "TARS". */
#define DEFAULT_PROMPT "Robot commands: TARS. forward, back, left, right, stop, freeze, spin, " \
                       "turn around, degrees, meter, steps. Go to the trash can, chair, door, " \
                       "table, backpack, water bottle, laptop, cup, box, person."

typedef struct {
    char input[256], model[256], host[128], save_dir[256], prompt[512];
    char goto_host[128], say_host[128], net_host[64];
    int say_port, say_ms, net_port;
    int port, goto_port, goto_reply_ms, threads, audio_ctx, require_wake, dry, led_ms, wake_carry_ms;
    int button_talk;
    vad_cfg_t vad;
} cfg_t;

static volatile sig_atomic_t running = 1;
static void on_sig(int s) { (void)s; running = 0; }

static long long now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

/* ------------------------------------------------------------------ config */
static void trim(char *s)
{
    char *b = s;
    while (*b == ' ' || *b == '\t') b++;
    memmove(s, b, strlen(b) + 1);
    for (int i = (int)strlen(s) - 1; i >= 0 && strchr(" \t\r\n", s[i]); i--) s[i] = '\0';
}

static int set_kv(cfg_t *c, const char *k, const char *v)
{
#define S(name, field) if (!strcmp(k, name)) { snprintf(c->field, sizeof c->field, "%s", v); return 0; }
#define I(name, field) if (!strcmp(k, name)) { c->field = atoi(v); return 0; }
#define F(name, field) if (!strcmp(k, name)) { c->field = (float)atof(v); return 0; }
    S("input", input) S("model", model) S("host", host) S("save_dir", save_dir) S("prompt", prompt)
    S("goto_host", goto_host) I("goto_port", goto_port) I("goto_reply_ms", goto_reply_ms)
    S("say_host", say_host) I("say_port", say_port) I("say_ms", say_ms)
    S("net_host", net_host) I("net_port", net_port)
    I("wake_carry_ms", wake_carry_ms) I("button_talk", button_talk)
    I("port", port) I("threads", threads) I("audio_ctx", audio_ctx) I("require_wake", require_wake)
    I("dry", dry) I("led_ms", led_ms)
    F("vad_ratio", vad.ratio) F("vad_min_dbfs", vad.min_dbfs) I("vad_start_frames", vad.start_frames)
    I("vad_hang_ms", vad.hang_ms) I("vad_pre_ms", vad.pre_ms) I("vad_min_ms", vad.min_ms)
    I("vad_max_ms", vad.max_ms)
#undef S
#undef I
#undef F
    return -1;
}

static int load_conf(cfg_t *c, const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "[voice] config %s: %s\n", path, strerror(errno)); return -1; }
    char line[700];
    int ln = 0;
    while (fgets(line, sizeof line, f)) {
        ln++;
        char *h = strchr(line, '#');
        if (h && (h == line || h[-1] == ' ' || h[-1] == '\t')) *h = '\0';
        trim(line);
        if (!line[0]) continue;
        char *eq = strchr(line, '=');
        if (!eq) { fprintf(stderr, "[voice] %s:%d: expected key = value\n", path, ln); continue; }
        *eq = '\0';
        char *k = line, *v = eq + 1;
        trim(k); trim(v);
        if (set_kv(c, k, v) < 0) fprintf(stderr, "[voice] %s:%d: unknown key '%s'\n", path, ln, k);
    }
    fclose(f);
    return 0;
}

/* ------------------------------------------------------------------ threads
 * The mic loop (main thread) and the network input (net_thread: laptop text / push-to-talk
 * clips via the mission relay) share the links and the classifier.
 *   act_lock: intent_decide + every link send + the [heard] log. Held for a few ms only, so
 *             a typed "stop" never waits for whisper.
 *   stt_lock: the whisper context (not thread safe). */
static pthread_mutex_t act_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t stt_lock = PTHREAD_MUTEX_INITIALIZER;

/* ---- events for watchers on the net port (one JSON object per line, best effort) */
#define NET_MAX 4
#define NET_AUDIO_MAX (15 * AUDIO_RATE * 2 + 4096)    /* 15 s of 16-bit mono + a WAV header */
typedef struct {
    int fd, watch, dead;
    char line[1024];
    size_t used;
    unsigned char *audio;     /* AUDIO <n>: payload being received */
    size_t audio_n, audio_got;
} netc_t;
static netc_t netc[NET_MAX];
static pthread_mutex_t ev_lock = PTHREAD_MUTEX_INITIALIZER;

static void json_str(char *out, size_t cap, const char *s)
{
    size_t o = 0;
    if (cap < 3) { if (cap) out[0] = '\0'; return; }
    out[o++] = '"';
    for (; s && *s && o + 8 < cap; s++) {
        unsigned char ch = (unsigned char)*s;
        if (ch == '"' || ch == '\\') { out[o++] = '\\'; out[o++] = (char)ch; }
        else if (ch < 0x20) o += (size_t)snprintf(out + o, cap - o, "\\u%04x", ch);
        else out[o++] = (char)ch;
    }
    out[o++] = '"';
    out[o] = '\0';
}

/* send one JSON line to every WATCH client; a client that can't take it whole is dropped
 * (never blocks the mic loop or a stop) */
static void ev_send(const char *json)
{
    char line[2200];
    int n = snprintf(line, sizeof line, "%s\n", json);
    if (n <= 0 || n >= (int)sizeof line) return;
    pthread_mutex_lock(&ev_lock);
    for (int i = 0; i < NET_MAX; i++) {
        if (netc[i].fd < 0 || !netc[i].watch || netc[i].dead) continue;
#ifdef MSG_NOSIGNAL
        ssize_t s = send(netc[i].fd, line, (size_t)n, MSG_DONTWAIT | MSG_NOSIGNAL);
#else
        ssize_t s = send(netc[i].fd, line, (size_t)n, MSG_DONTWAIT);
#endif
        if (s != n) netc[i].dead = 1;
    }
    pthread_mutex_unlock(&ev_lock);
}

static void ev_text(const char *type, const char *key, const char *text)
{
    char q[1100], j[1200];
    json_str(q, sizeof q, text);
    snprintf(j, sizeof j, "{\"type\":\"%s\",\"%s\":%s}", type, key, q);
    ev_send(j);
}

/* ------------------------------------------------------------------ tars_motor link */
typedef struct {
    int fd;
    long long next_try;
    const char *host, *name;
    int port, dry;
    int reply_ms;              /* wait this long for a reply line */
    int reply_optional;        /* go-to controller: no reply in time is fine, keep the link */
} link_t;

static void link_drop(link_t *l, const char *why)
{
    if (l->fd >= 0) { close(l->fd); fprintf(stderr, "[link] %s dropped: %s\n", l->name, why); }
    l->fd = -1;
}

static int link_up(link_t *l)
{
    if (l->dry) return 1;
    if (l->fd >= 0) return 1;
    long long t = now_ms();
    if (t < l->next_try) return 0;
    l->next_try = t + 1000;
    char port[16];
    snprintf(port, sizeof port, "%d", l->port);
    struct addrinfo hints = { 0 }, *res = NULL;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    if (getaddrinfo(l->host, port, &hints, &res) != 0 || !res) {
        fprintf(stderr, "[link] %s: can't resolve %s\n", l->name, l->host);
        return 0;
    }
    int fd = socket(res->ai_family, res->ai_socktype, 0);
    if (fd < 0 || connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        fprintf(stderr, "[link] %s %s:%d unreachable (%s); retrying\n", l->name, l->host, l->port, strerror(errno));
        if (fd >= 0) close(fd);
        freeaddrinfo(res);
        return 0;
    }
    freeaddrinfo(res);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    struct timeval tv = { l->reply_ms / 1000, (l->reply_ms % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    l->fd = fd;
    fprintf(stderr, "[link] connected to %s %s:%d\n", l->name, l->host, l->port);
    return 1;
}

/* Send one line, read one reply line into reply, waiting up to wait_ms (<= 0: the link's
 * reply_ms). Returns 0 on OK/ERR reply (or no reply on a reply_optional link), -1 if no link.
 * No reply in time on a reply_optional link drops the link: the late reply would otherwise be
 * read as the answer to the NEXT line (that desync made every later voice move get cut by
 * goto's late S to tars_motor). */
static int link_cmd_ms(link_t *l, const char *line, char *reply, int rlen, int wait_ms)
{
    if (l->dry) { snprintf(reply, (size_t)rlen, "(dry)"); return 0; }
    if (wait_ms <= 0) wait_ms = l->reply_ms;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!link_up(l)) {
            /* S / CANCEL are worth one immediate retry */
            if (attempt == 0 && (!strcmp(line, "S") || !strcmp(line, "CANCEL"))) { l->next_try = 0; continue; }
            snprintf(reply, (size_t)rlen, "(no link)");
            return -1;
        }
        struct timeval tv = { wait_ms / 1000, (wait_ms % 1000) * 1000 };
        setsockopt(l->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        char out[96];
        int n = snprintf(out, sizeof out, "%s\n", line);
        if (send(l->fd, out, (size_t)n, 0) != n) { link_drop(l, strerror(errno)); l->next_try = 0; continue; }
        int o = 0;
        while (o < rlen - 1) {
            char ch;
            ssize_t r = recv(l->fd, &ch, 1, 0);
            if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && l->reply_optional) {
                o = -2;                    /* no reply in time: fine for the go-to controller */
                break;
            }
            if (r <= 0) { link_drop(l, r == 0 ? "server closed" : strerror(errno)); o = -1; break; }
            if (ch == '\n') break;
            reply[o++] = ch;
        }
        if (o == -2) {
            link_drop(l, "no reply in time (reconnect, so a late reply can't answer the next line)");
            l->next_try = 0;
            snprintf(reply, (size_t)rlen, "(sent, no reply)");
            return 0;
        }
        if (o < 0) { l->next_try = 0; continue; }
        reply[o] = '\0';
        return 0;
    }
    snprintf(reply, (size_t)rlen, "(no link)");
    return -1;
}

static int link_cmd(link_t *l, const char *line, char *reply, int rlen)
{
    return link_cmd_ms(l, line, reply, rlen, 0);
}

/* ---- exact-move keepalive. tars_motor's 500 ms deadman aborts a MOVE/TURN
 * (move_why=watchdog) unless some client keeps commanding. A voice move is a
 * one-shot line and whisper can block the main loop for longer than that, so a
 * small thread on its own connection PINGs while the move runs (STATUS
 * move=running), and gives up at the move's own deadline. S clears it. */
static pthread_mutex_t ka_lock = PTHREAD_MUTEX_INITIALIZER;
static long long ka_until_ms;              /* 0 = no move to keep alive */
static int ka_move_id;

static void ka_arm(const char *line, const char *reply)
{
    int id = 0, est = 0;
    pthread_mutex_lock(&ka_lock);
    if (!strcmp(line, "S")) ka_until_ms = 0;
    else if ((!strncmp(line, "MOVE", 4) || !strncmp(line, "TURN", 4)) &&
             sscanf(reply, "OK %d %d", &id, &est) == 2 && est >= 0) {
        ka_move_id = id;
        ka_until_ms = now_ms() + 2LL * est + 2000;   /* tars_motor's own move timeout */
    }
    pthread_mutex_unlock(&ka_lock);
}

static void *ka_thread(void *arg)
{
    link_t *l = arg;
    while (running) {
        struct timespec ts = { 0, 150 * 1000000L };
        nanosleep(&ts, NULL);
        pthread_mutex_lock(&ka_lock);
        long long until = ka_until_ms;
        int id = ka_move_id;
        pthread_mutex_unlock(&ka_lock);
        if (!until) continue;
        char st[1024], r[64];
        int done = now_ms() > until;
        if (!done && link_cmd(l, "STATUS", st, sizeof st) == 0) {
            const char *m = strstr(st, " move="), *mi = strstr(st, " move_id=");
            done = !m || strncmp(m + 6, "running", 7) || !mi || atoi(mi + 9) != id;
            if (!done) link_cmd(l, "PING", r, sizeof r);
        }
        if (done) {
            pthread_mutex_lock(&ka_lock);
            if (ka_until_ms == until) ka_until_ms = 0;
            pthread_mutex_unlock(&ka_lock);
        }
    }
    return NULL;
}

static void link_send_log(link_t *l, const char *line)
{
    char reply[1024];
    long long t0 = now_ms();
    link_cmd(l, line, reply, sizeof reply);
    if (!l->reply_optional) ka_arm(line, reply);
    long long ms = now_ms() - t0;
    if (!strcmp(line, "STATUS")) fprintf(stderr, "[status] %s\n", reply);
    else printf("[%s] %-16s -> %s (%lld ms)\n", l->name, line, reply, ms);
    fflush(stdout);
    char ql[200], qr[1100], j[1400];
    json_str(ql, sizeof ql, line);
    json_str(qr, sizeof qr, reply);
    snprintf(j, sizeof j, "{\"type\":\"sent\",\"to\":\"%s\",\"line\":%s,\"reply\":%s,\"ms\":%lld}", l->name, ql, qr, ms);
    ev_send(j);
}

/* ---- speech to the phone (TCP 5573, "SAY <text>"): the robot has no speaker. Best effort:
 * non-blocking connect bounded by say_ms, one line, close; any failure is ignored. Only
 * called for informational replies (memory), never on the stop path. */
static const char *g_say_host = "127.0.0.1";
static int g_say_port, g_say_ms = 150;

/* line = a full protocol line without the newline: "SAY <text>" or the urgent JSON form */
static void say_line(const char *line_in)
{
    if (g_say_port <= 0 || !line_in || !*line_in) return;
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((unsigned short)g_say_port);
    if (inet_pton(AF_INET, g_say_host, &a.sin_addr) != 1) return;
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return;
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0 || fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) { close(fd); return; }
    int ok = connect(fd, (struct sockaddr *)&a, sizeof a) == 0;
    if (!ok && errno == EINPROGRESS) {
        struct pollfd p = { fd, POLLOUT, 0 };
        int err = 0;
        socklen_t el = sizeof err;
        ok = poll(&p, 1, g_say_ms) == 1 && getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el) == 0 && err == 0;
    }
    if (ok) {
        char line[1024];
        int n = snprintf(line, sizeof line, "%s\n", line_in);
        if (n > 0 && n < (int)sizeof line) {
#ifdef MSG_NOSIGNAL
            send(fd, line, (size_t)n, MSG_NOSIGNAL);
#else
            send(fd, line, (size_t)n, 0);
#endif
        }
    }
    close(fd);
}

/* "SAY <text>", text cut to the phone's 300-char limit, one line */
static void say_phone(const char *text)
{
    char line[320];
    snprintf(line, sizeof line, "SAY %.300s", text ? text : "");
    for (char *p = line; *p; p++) if (*p == '\n' || *p == '\r') *p = ' ';
    ev_text("say", "text", line + 4);
    say_line(line);
}

/* "@goto MEMORY SAY" -> goto replies "OK n=<k> <sentence>": print it, BEEP min(k,4) (one per
 * remembered object; LED no for none), and hand the sentence to the phone to speak. */
static void memory_say(link_t *motor, link_t *go)
{
    char reply[1024], line[16];
    if (link_cmd(go, "MEMORY SAY", reply, sizeof reply) < 0 || strncmp(reply, "OK n=", 5)) {
        printf("[memory] no answer from goto (%s)\n", reply);
        fflush(stdout);
        link_send_log(motor, "LED no");
        return;
    }
    int k = atoi(reply + 5);
    const char *text = strchr(reply + 5, ' ');
    text = text ? text + 1 : "";
    printf("[memory] %s\n", text);
    fflush(stdout);
    ev_text("memory", "text", text);
    if (k > 0) {
        snprintf(line, sizeof line, "BEEP %d", k < 4 ? k : 4);
        link_send_log(motor, line);
    } else {
        link_send_log(motor, "LED no");
    }
    say_phone(text);
}

/* Route one decision line: "@goto X" -> go-to controller, else tars_motor. */
static void route(link_t *motor, link_t *go, const char *line)
{
    size_t pl = strlen(INTENT_GOTO_PREFIX);
    if (!strcmp(line, INTENT_GOTO_PREFIX "MEMORY SAY")) memory_say(motor, go);
    else if (!strncmp(line, INTENT_GOTO_PREFIX, pl)) link_send_log(go, line + pl);
    else link_send_log(motor, line);
}

/* Manual move (MOVE/TURN): cancel the go-to only if it's doing something. goto answers
 * CANCEL with S to tars_motor even when idle, and an S that lands after our MOVE/TURN aborts
 * it. So ask STATUS first: "state=idle" -> no CANCEL, no S. Busy, or no answer -> CANCEL and
 * wait for its OK (goto sends the S before replying) for up to GOTO_CANCEL_WAIT_MS, so the S
 * is already at tars_motor when the move goes out. tars_motor's S-hold covers only D, not
 * MOVE/TURN. The stop path doesn't use this: S first, then CANCEL, as always. */
#define GOTO_CANCEL_WAIT_MS 1500
static void cancel_goto_for_move(link_t *go)
{
    char st[4096];                         /* goto's STATUS is long; read it whole */
    long long t0 = now_ms();
    if (!go->dry && link_cmd(go, "STATUS", st, sizeof st) == 0 &&
        (!strncmp(st, "OK state=idle ", 14) || !strcmp(st, "OK state=idle"))) {
        printf("[goto] go-to idle: no CANCEL before the move (%lld ms)\n", now_ms() - t0);
        fflush(stdout);
        return;
    }
    char reply[1024];
    t0 = now_ms();
    link_cmd_ms(go, "CANCEL", reply, sizeof reply, GOTO_CANCEL_WAIT_MS);
    long long ms = now_ms() - t0;
    printf("[%s] %-16s -> %s (%lld ms)%s\n", go->name, "CANCEL", reply, ms,
           strncmp(reply, "OK", 2) && !go->dry ? "  (no OK: its S may still cut this move)" : "");
    fflush(stdout);
    char qr[1100], j[1300];
    json_str(qr, sizeof qr, reply);
    snprintf(j, sizeof j, "{\"type\":\"sent\",\"to\":\"%s\",\"line\":\"CANCEL\",\"reply\":%s,\"ms\":%lld}",
             go->name, qr, ms);
    ev_send(j);
}

static long long led_off_at;               /* act_lock: "LED off" due (after a reject) */
static int g_led_ms = 1500;

/* Log one decision and carry it out: the same path for the mic and the net input.
 * where = "at=1.20s len=0.80s stt=212ms" (mic) or "src=net-text stt=0ms" (net).
 * Caller holds act_lock. Order is the decision's: S first, then CANCEL, then the rest. */
static void act_decision(link_t *motor, link_t *go, const char *src, const char *where,
                         const char *text, long long stt_ms, const intent_decision_t *d)
{
    static const char *act[] = { "ignore", "stop", "command", "reject" };
    printf("[heard] \"%s\" %s -> %s %s %.2f (%s)", text, where, act[d->action], d->intent,
           d->prob, d->why ? d->why : "");
    for (int i = 0; i < d->n_lines; i++) printf("%s%s", i ? " | " : ": ", d->lines[i]);
    printf("\n");
    fflush(stdout);
    if (d->action == INTENT_ACT_STOP) {                 /* stop: act before telling anyone */
        for (int i = 0; i < d->n_lines; i++) route(motor, go, d->lines[i]);
    }
    {
        char qt[1100], qi[64], qw[64], ls[700], j[2100];
        json_str(qt, sizeof qt, text);
        json_str(qi, sizeof qi, d->intent);
        json_str(qw, sizeof qw, d->why ? d->why : "");
        size_t o = (size_t)snprintf(ls, sizeof ls, "[");
        for (int i = 0; i < d->n_lines && o < sizeof ls - 80; i++) {
            char q[80];
            json_str(q, sizeof q, d->lines[i]);
            o += (size_t)snprintf(ls + o, sizeof ls - o, "%s%s", i ? "," : "", q);
        }
        snprintf(ls + o, sizeof ls - o, "]");
        snprintf(j, sizeof j, "{\"type\":\"heard\",\"src\":\"%s\",\"text\":%s,\"action\":\"%s\",\"intent\":%s,"
                 "\"prob\":%.3f,\"why\":%s,\"lines\":%s,\"stt_ms\":%lld}",
                 src, qt, act[d->action], qi, d->prob, qw, ls, stt_ms);
        ev_send(j);
    }
    if (d->action != INTENT_ACT_STOP)
        for (int i = 0; i < d->n_lines; i++) {
            if (!strcmp(d->lines[i], INTENT_GOTO_PREFIX "CANCEL")) cancel_goto_for_move(go);
            else route(motor, go, d->lines[i]);
        }
    if (d->action == INTENT_ACT_STOP) link_send_log(motor, "BEEP 1");
    /* only now (S and CANCEL already sent): tell the phone, urgent = cut any speech */
    if (d->action == INTENT_ACT_STOP) {
        ev_text("say", "text", "Stopping.");
        say_line("{\"type\":\"say\",\"text\":\"Stopping.\",\"priority\":\"urgent\"}");
    }
    if (d->action == INTENT_ACT_REJECT) led_off_at = now_ms() + g_led_ms;
}

/* ------------------------------------------------------------------ net input
 * TCP net_host:net_port (default 127.0.0.1:5574; the laptop reaches it through the mission
 * relay, iter2-autonomy/mission/). One command per line:
 *   HEAR <text>     a transcript (typed, or speech-to-text done on the laptop): the same
 *                   intent_decide() as the mic, stop words first. No wake word needed
 *                   (it's push-to-talk). -> "OK heard <action> <intent> <prob>"
 *   AUDIO <bytes>   then <bytes> of 16 kHz mono s16le PCM, or a WAV file (16 kHz mono 16-bit),
 *                   <= 15 s: whisper + the same path. -> "OK heard <action> <intent> <prob> <text>"
 *   WATCH           this connection also gets the event lines (JSON): heard / sent / memory / say
 *   PING            -> "OK PONG"
 * The mic keeps working at the same time. */
typedef struct { int lfd; link_t *motor, *go; stt_t *stt; } net_ctx_t;

static void net_reply(netc_t *c, const char *s)
{
    size_t n = strlen(s);
    pthread_mutex_lock(&ev_lock);
#ifdef MSG_NOSIGNAL
    if (send(c->fd, s, n, MSG_DONTWAIT | MSG_NOSIGNAL) != (ssize_t)n) c->dead = 1;
#else
    if (send(c->fd, s, n, MSG_DONTWAIT) != (ssize_t)n) c->dead = 1;
#endif
    pthread_mutex_unlock(&ev_lock);
}

/* reply on a raw fd (audio jobs reply on their own dup of the client fd) */
static void fd_reply(int fd, const char *s)
{
#ifdef MSG_NOSIGNAL
    send(fd, s, strlen(s), MSG_DONTWAIT | MSG_NOSIGNAL);
#else
    send(fd, s, strlen(s), MSG_DONTWAIT);
#endif
}

static void net_act(net_ctx_t *x, int fd, const char *src, const char *text, long long stt_ms)
{
    intent_decision_t d;
    char where[64], reply[700];
    static const char *act[] = { "ignore", "stop", "command", "reject" };
    snprintf(where, sizeof where, "src=%s stt=%lldms", src, stt_ms);
    pthread_mutex_lock(&act_lock);
    intent_decide(text, 0, &d);
    act_decision(x->motor, x->go, src, where, text, stt_ms, &d);
    pthread_mutex_unlock(&act_lock);
    snprintf(reply, sizeof reply, "OK heard %s %s %.2f %s\n", act[d.action], d.intent, d.prob, text);
    for (char *p = reply; p[1]; p++) if (*p == '\n' || *p == '\r') *p = ' ';
    fd_reply(fd, reply);
}

/* WAV (16 kHz mono 16-bit PCM) or raw s16le -> samples (points into buf). -1 = bad format. */
static int pcm_of(const unsigned char *b, size_t n, const int16_t **pcm, int *ns)
{
    if (n >= 12 && !memcmp(b, "RIFF", 4) && !memcmp(b + 8, "WAVE", 4)) {
        size_t o = 12;
        int fmt_ok = 0;
        while (o + 8 <= n) {
            size_t len = (size_t)b[o + 4] | (size_t)b[o + 5] << 8 | (size_t)b[o + 6] << 16 | (size_t)b[o + 7] << 24;
            if (!memcmp(b + o, "fmt ", 4) && len >= 16 && o + 8 + 16 <= n) {
                const unsigned char *f = b + o + 8;
                unsigned fmt = f[0] | f[1] << 8, ch = f[2] | f[3] << 8, bits = f[14] | f[15] << 8;
                unsigned rate = f[4] | f[5] << 8 | f[6] << 16 | (unsigned)f[7] << 24;
                fmt_ok = fmt == 1 && ch == 1 && bits == 16 && rate == AUDIO_RATE;
                if (!fmt_ok) return -1;
            } else if (!memcmp(b + o, "data", 4)) {
                if (!fmt_ok) return -1;
                if (len > n - o - 8) len = n - o - 8;
                *pcm = (const int16_t *)(const void *)(b + o + 8);
                *ns = (int)(len / 2);
                return 0;
            }
            o += 8 + len + (len & 1);
        }
        return -1;
    }
    *pcm = (const int16_t *)(const void *)b;
    *ns = (int)(n / 2);
    return 0;
}

/* A push-to-talk clip is transcribed on its own thread, one at a time, so the net thread
 * (typed text, a typed "stop") never waits for whisper. */
typedef struct { net_ctx_t *x; int fd; unsigned char *buf; size_t n; } audio_job_t;
static pthread_mutex_t job_lock = PTHREAD_MUTEX_INITIALIZER;
static int job_busy;

static void *audio_job(void *arg)
{
    audio_job_t *j = arg;
    const int16_t *pcm;
    int ns;
    char text[512];
    if (pcm_of(j->buf, j->n, &pcm, &ns) < 0) fd_reply(j->fd, "ERR audio must be 16 kHz mono 16-bit\n");
    else if (ns < AUDIO_RATE / 10) fd_reply(j->fd, "ERR audio shorter than 0.1 s\n");
    else {
        int16_t *copy = malloc((size_t)ns * sizeof *copy);   /* aligned copy for whisper */
        if (!copy) fd_reply(j->fd, "ERR out of memory\n");
        else {
            memcpy(copy, pcm, (size_t)ns * sizeof *copy);
            long long t0 = now_ms();
            pthread_mutex_lock(&stt_lock);
            int r = stt_run(j->x->stt, copy, ns, text, sizeof text);
            pthread_mutex_unlock(&stt_lock);
            free(copy);
            if (r != 0) fd_reply(j->fd, "ERR whisper failed\n");
            else if (running) net_act(j->x, j->fd, "net-audio", text, now_ms() - t0);
        }
    }
    close(j->fd);
    free(j->buf);
    free(j);
    pthread_mutex_lock(&job_lock);
    job_busy = 0;
    pthread_mutex_unlock(&job_lock);
    return NULL;
}

/* the whole clip arrived: hand it to a job thread (takes ownership of c->audio) */
static void net_audio(net_ctx_t *x, netc_t *c)
{
    pthread_mutex_lock(&job_lock);
    int busy = job_busy;
    if (!busy) job_busy = 1;
    pthread_mutex_unlock(&job_lock);
    if (busy) { net_reply(c, "ERR busy: still transcribing the last clip\n"); return; }
    audio_job_t *j = malloc(sizeof *j);
    int fd = j ? dup(c->fd) : -1;
    pthread_t t;
    pthread_attr_t at;
    pthread_attr_init(&at);
    pthread_attr_setdetachstate(&at, PTHREAD_CREATE_DETACHED);
    if (fd >= 0) {
        *j = (audio_job_t){ x, fd, c->audio, c->audio_got };
        if (pthread_create(&t, &at, audio_job, j) == 0) { c->audio = NULL; pthread_attr_destroy(&at); return; }
        close(fd);
    }
    pthread_attr_destroy(&at);
    free(j);
    pthread_mutex_lock(&job_lock);
    job_busy = 0;
    pthread_mutex_unlock(&job_lock);
    net_reply(c, "ERR can't start the transcription\n");
}

static void net_close(netc_t *c)
{
    pthread_mutex_lock(&ev_lock);
    if (c->fd >= 0) close(c->fd);
    c->fd = -1;
    c->watch = c->dead = 0;
    c->used = 0;
    free(c->audio);
    c->audio = NULL;
    c->audio_n = c->audio_got = 0;
    pthread_mutex_unlock(&ev_lock);
}

/* one complete line from a net client (no newline) */
static void net_line(net_ctx_t *x, netc_t *c, char *line)
{
    size_t l = strlen(line);
    if (l && line[l - 1] == '\r') line[--l] = '\0';
    if (!strncasecmp(line, "HEAR ", 5)) {
        const char *t = line + 5;
        while (*t == ' ') t++;
        if (!*t) net_reply(c, "ERR usage: HEAR <text>\n");
        else net_act(x, c->fd, "net-text", t, 0);
    } else if (!strncasecmp(line, "AUDIO ", 6)) {
        long n = atol(line + 6);
        if (n <= 0 || n > NET_AUDIO_MAX) { net_reply(c, "ERR usage: AUDIO <bytes> (1..15 s of 16 kHz mono 16-bit)\n"); return; }
        c->audio = malloc((size_t)n);
        if (!c->audio) { net_reply(c, "ERR out of memory\n"); return; }
        c->audio_n = (size_t)n;
        c->audio_got = 0;
    } else if (!strcasecmp(line, "WATCH")) {
        pthread_mutex_lock(&ev_lock);
        c->watch = 1;
        pthread_mutex_unlock(&ev_lock);
        net_reply(c, "OK watch\n");
    } else if (!strcasecmp(line, "PING")) {
        net_reply(c, "OK PONG\n");
    } else if (l) {
        net_reply(c, "ERR unknown command (HEAR <text> | AUDIO <bytes> | WATCH | PING)\n");
    }
}

static void *net_thread(void *arg)
{
    net_ctx_t *x = arg;
    while (running) {
        struct pollfd p[1 + NET_MAX];
        int map[1 + NET_MAX], n = 0;
        p[n].fd = x->lfd; p[n].events = POLLIN; map[n++] = -1;
        for (int i = 0; i < NET_MAX; i++) {
            if (netc[i].fd >= 0 && netc[i].dead) net_close(&netc[i]);
            if (netc[i].fd >= 0) { p[n].fd = netc[i].fd; p[n].events = POLLIN; map[n++] = i; }
        }
        if (poll(p, (nfds_t)n, 200) <= 0) continue;
        for (int k = 0; k < n; k++) {
            if (!(p[k].revents & (POLLIN | POLLHUP | POLLERR))) continue;
            if (map[k] < 0) {
                int fd = accept(x->lfd, NULL, NULL);
                if (fd < 0) continue;
                int slot = -1;
                for (int i = 0; i < NET_MAX && slot < 0; i++) if (netc[i].fd < 0) slot = i;
                if (slot < 0) { send(fd, "ERR busy\n", 9, MSG_DONTWAIT); close(fd); continue; }
                int one = 1;
                setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
                setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
                pthread_mutex_lock(&ev_lock);
                netc[slot].fd = fd;
                netc[slot].watch = netc[slot].dead = 0;
                netc[slot].used = 0;
                pthread_mutex_unlock(&ev_lock);
                continue;
            }
            netc_t *c = &netc[map[k]];
            unsigned char buf[8192];
            ssize_t r = recv(c->fd, buf, sizeof buf, 0);
            if (r <= 0) { net_close(c); continue; }
            size_t o = 0;
            while (o < (size_t)r && c->fd >= 0 && !c->dead) {
                if (c->audio) {                          /* AUDIO payload */
                    size_t take = c->audio_n - c->audio_got;
                    if (take > (size_t)r - o) take = (size_t)r - o;
                    memcpy(c->audio + c->audio_got, buf + o, take);
                    c->audio_got += take;
                    o += take;
                    if (c->audio_got == c->audio_n) {
                        net_audio(x, c);
                        free(c->audio);
                        c->audio = NULL;
                        c->audio_n = c->audio_got = 0;
                    }
                    continue;
                }
                char ch = (char)buf[o++];
                if (ch == '\n') { c->line[c->used] = '\0'; c->used = 0; net_line(x, c, c->line); continue; }
                if (c->used < sizeof c->line - 1) c->line[c->used++] = ch;
                else { net_reply(c, "ERR line too long\n"); c->dead = 1; }
            }
        }
    }
    for (int i = 0; i < NET_MAX; i++) net_close(&netc[i]);
    return NULL;
}

static int net_listen(const char *host, int port)
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
        listen(fd, 4) < 0) { close(fd); return -1; }
    return fd;
}

/* ------------------------------------------------------------------ main */
static void usage(void)
{
    fprintf(stderr,
        "usage: tars_voice [-c voice.conf] [--input qnx|qnx:C,D|wav:FILE|raw:-|phone[:HOST:PORT]]\n"
        "                  [--model FILE]\n"
        "                  [--host H] [--port P] [--goto-host H] [--goto-port P] [--threads N]\n"
        "                  [--audio-ctx N] [--any] [--dry]\n"
        "                  [--save-dir DIR] [--list]\n");
}

int main(int argc, char **argv)
{
    cfg_t c;
    memset(&c, 0, sizeof c);
    snprintf(c.input, sizeof c.input, "%s",
#ifdef __QNX__
             "qnx"
#else
             "raw:-"
#endif
    );
    snprintf(c.model, sizeof c.model, "models/ggml-tiny.en-q5_1.bin");
    snprintf(c.host, sizeof c.host, "127.0.0.1");
    snprintf(c.goto_host, sizeof c.goto_host, "127.0.0.1");
    c.goto_port = 5570; c.goto_reply_ms = 300;
    snprintf(c.say_host, sizeof c.say_host, "127.0.0.1"); c.say_port = 5573; c.say_ms = 150;
    snprintf(c.net_host, sizeof c.net_host, "127.0.0.1"); c.net_port = 5574; c.wake_carry_ms = 4000; c.button_talk = 1;
    snprintf(c.prompt, sizeof c.prompt, "%s", DEFAULT_PROMPT);
    c.port = 5555; c.threads = 2; c.audio_ctx = 0; c.require_wake = 1; c.led_ms = 1500;
    vad_defaults(&c.vad);

    /* config file first, then command line overrides */
    for (int i = 1; i < argc - 1; i++)
        if (!strcmp(argv[i], "-c")) { if (load_conf(&c, argv[i + 1]) < 0) return 2; }
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "-c") && v) { i++; continue; }
        if (!strcmp(a, "--any")) { c.require_wake = 0; continue; }
        if (!strcmp(a, "--dry")) { c.dry = 1; continue; }
        if (!strcmp(a, "--list")) { return audio_list_devices() < 0 ? 1 : 0; }
        if (!strcmp(a, "-h") || !strcmp(a, "--help")) { usage(); return 0; }
        if (!v || strncmp(a, "--", 2)) { usage(); return 2; }
        char key[32];
        snprintf(key, sizeof key, "%s", a + 2);
        for (char *p = key; *p; p++) if (*p == '-') *p = '_';
        if (set_kv(&c, key, v) < 0) { fprintf(stderr, "unknown option %s\n", a); usage(); return 2; }
        i++;
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    signal(SIGPIPE, SIG_IGN);

    char err[256] = "";
    audio_src_t *au = audio_open(c.input, err, sizeof err);
    if (!au) { fprintf(stderr, "[voice] audio: %s\n", err); return 1; }

    long long t0 = now_ms();
    stt_cfg_t sc = { c.model, c.threads, c.audio_ctx, c.prompt };
    stt_t *stt = stt_init(&sc);
    if (!stt) { fprintf(stderr, "[voice] can't load model %s (run iter2-autonomy/voice/get_model.sh)\n", c.model); return 1; }
    fprintf(stderr, "[voice] model %s loaded in %lld ms; threads=%d input=%s wake=%s %s\n", c.model,
            now_ms() - t0, c.threads, c.input, c.require_wake ? "required" : "optional",
            c.dry ? "(dry run)" : "");

    vad_t vad;
    if (vad_init(&vad, &c.vad) < 0) return 1;
    link_t link = { -1, 0, c.host, "motor", c.port, c.dry, 1000, 0 };
    link_t golink = { -1, 0, c.goto_host, "goto", c.goto_port, c.dry, c.goto_reply_ms, 1 };
    if (link_up(&link) && !c.dry) {
        char r[64];
        link_cmd(&link, "WHO voice", r, sizeof r);
    }
    g_say_host = c.say_host;
    g_say_port = c.dry ? 0 : c.say_port;
    g_say_ms = c.say_ms > 0 && c.say_ms <= 1000 ? c.say_ms : 150;
    link_t kalink = { -1, 0, c.host, "motor-keepalive", c.port, c.dry, 1000, 0 };
    pthread_t ka_tid;
    int ka_started = !c.dry && pthread_create(&ka_tid, NULL, ka_thread, &kalink) == 0;
    g_led_ms = c.led_ms;
    for (int i = 0; i < NET_MAX; i++) netc[i].fd = -1;
    net_ctx_t nctx = { -1, &link, &golink, stt };
    pthread_t net_tid;
    int net_started = 0;
    if (c.net_port > 0) {
        nctx.lfd = net_listen(c.net_host, c.net_port);
        if (nctx.lfd < 0) fprintf(stderr, "[voice] net input %s:%d: can't listen (%s); mic only\n",
                                  c.net_host, c.net_port, strerror(errno));
        else {
            net_started = pthread_create(&net_tid, NULL, net_thread, &nctx) == 0;
            fprintf(stderr, "[voice] net input on %s:%d (HEAR <text> | AUDIO <bytes> | WATCH)\n", c.net_host, c.net_port);
        }
    }

    int16_t frame[VAD_FRAME];
    int fill = 0, nseg = 0;
    long long samples = 0, last_ping = 0;
    long long wake_until = -1;             /* sample count until which a bare "TARS" arms the next segment */
    long long talk_until_ms = 0, last_btn_poll = 0;   /* talk button: wall clock */
    int btn_seen = -1;
    while (running) {
        int r = audio_read(au, frame + fill, VAD_FRAME - fill);
        const int16_t *seg = NULL;
        int segn = 0, done = 0;
        if (r == AUDIO_IDLE) goto housekeeping;          /* phone: link down / muted / no data yet */
        if (r == AUDIO_GAP) {
            /* phone muted (it's talking) or the link broke: what follows isn't continuous with
             * what came before. Finish the segment in progress (speech said before the mute is
             * still the user's), drop the half frame and the pre-roll. */
            fill = 0;
            if (!vad_flush(&vad, &seg, &segn)) {
                vad.ring_fill = 0;
                vad.loud_run = 0;
                goto housekeeping;
            }
        } else if (r < 0) { fprintf(stderr, "[voice] audio read error\n"); break; }
        else if (r == 0) {
            done = 1;
            if (!vad_flush(&vad, &seg, &segn)) break;
        } else {
            fill += r;
            samples += r;
            if (fill < VAD_FRAME) continue;
            fill = 0;
            if (!vad_push(&vad, frame, &seg, &segn)) goto housekeeping;
        }
        {
            /* a speech segment: transcribe + act */
            nseg++;
            if (c.save_dir[0]) {
                char path[400];
                snprintf(path, sizeof path, "%s/seg_%03d.wav", c.save_dir, nseg);
                wav_write(path, seg, segn);
            }
            char text[512];
            long long ts = now_ms();
            pthread_mutex_lock(&stt_lock);
            int sr = stt_run(stt, seg, segn, text, sizeof text);
            pthread_mutex_unlock(&stt_lock);
            if (sr != 0) { fprintf(stderr, "[voice] whisper failed\n"); continue; }
            long long stt_ms = now_ms() - ts;
            intent_decision_t d;
            pthread_mutex_lock(&act_lock);
            int armed = (wake_until >= 0 && samples - segn <= wake_until) || now_ms() < talk_until_ms;
            intent_decide(text, c.require_wake && !armed, &d);
            if (d.action == INTENT_ACT_IGNORE && d.wake && d.why && !strcmp(d.why, "empty")) {
                wake_until = samples + (long long)c.wake_carry_ms * AUDIO_RATE / 1000;
                printf("[wake] \"%s\": listening for a command for %d ms\n", text, c.wake_carry_ms);
            } else if (d.action != INTENT_ACT_IGNORE) {
                wake_until = -1;
                talk_until_ms = 0;
            }
            char where[96];
            snprintf(where, sizeof where, "at=%.2fs len=%.2fs stt=%lldms", (double)samples / AUDIO_RATE,
                     (double)segn / AUDIO_RATE, stt_ms);
            act_decision(&link, &golink, "mic", where, text, stt_ms, &d);
            pthread_mutex_unlock(&act_lock);
        }
        if (done) break;
    housekeeping:
        pthread_mutex_lock(&act_lock);
        if (led_off_at && now_ms() >= led_off_at) { link_send_log(&link, "LED off"); led_off_at = 0; }
        if (c.button_talk && !c.dry && link.fd >= 0 && now_ms() - last_btn_poll >= 250) {
            last_btn_poll = now_ms();
            char st[1024];
            if (link_cmd(&link, "STATUS", st, sizeof st) == 0) {
                const char *b = strstr(st, " btn_teach=");
                if (b) {
                    int n = atoi(b + 11);
                    if (btn_seen >= 0 && n > btn_seen) {
                        talk_until_ms = now_ms() + c.wake_carry_ms;
                        printf("[talk] button pressed: listening %d ms, no wake word needed\n", c.wake_carry_ms);
                        fflush(stdout);
                        link_send_log(&link, "BEEP 1");
                    }
                    btn_seen = n;
                }
            }
        }
        if (!c.dry && now_ms() - last_ping > 2000) {        /* keep the link warm / reconnect */
            last_ping = now_ms();
            if (link.fd < 0) link_up(&link);
            if (golink.fd < 0) link_up(&golink);
        }
        pthread_mutex_unlock(&act_lock);
    }
    running = 0;
    if (ka_started) pthread_join(ka_tid, NULL);
    if (net_started) pthread_join(net_tid, NULL);
    for (int i = 0; i < 1000; i++) {                 /* a clip job still uses whisper: let it finish */
        pthread_mutex_lock(&job_lock);
        int busy = job_busy;
        pthread_mutex_unlock(&job_lock);
        if (!busy) break;
        struct timespec ts = { 0, 10 * 1000000L };
        nanosleep(&ts, NULL);
    }
    if (nctx.lfd >= 0) close(nctx.lfd);
    if (led_off_at) link_send_log(&link, "LED off");
    fprintf(stderr, "[voice] %d segments, %.1f s of audio, noise floor %.1f dBFS\n", nseg,
            (double)samples / AUDIO_RATE, vad_noise_dbfs(&vad));
    vad_free(&vad);
    stt_free(stt);
    audio_close(au);
    link_drop(&link, "exit");
    link_drop(&golink, "exit");
    return 0;
}
