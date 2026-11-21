/* tars_voice: offline speech commands for TARS, on the Pi (QNX) or a Mac.
 *
 *   mic (io-audio) -> VAD (20 ms frames) -> segment <= 3 s -> whisper.cpp tiny.en
 *     -> text -> intent_decide() (safety stop, wake word, classifier, slots)
 *     -> lines to tars_motor (TCP 5555): S | MOVE | TURN | STATUS, + LED/BEEP
 *     -> "GOTO <object>" / "CANCEL" to the go-to controller (TCP 5570, goto/)
 *   stop  = S to tars_motor FIRST, then CANCEL to the go-to controller
 *   move  = CANCEL to go-to, then MOVE/TURN to tars_motor (manual beats go-to)
 * Talk button (button_talk=1): a short press of the Uno button (STATUS
 * btn_teach= counter, polled every 250 ms) beeps once and arms listening
 * for wake_carry_ms with NO wake word needed. Best fix for a noisy room.
 * Wake word carry-over: "TARS" ... (pause) ... "turn left" works: a segment
 * that is only the wake word arms the next segment for wake_carry_ms.
 *
 * Usage: tars_voice [-c voice.conf] [--input SPEC] [--model FILE] [--host H] [--port P]
 *                   [--threads N] [--any] [--dry] [--save-dir DIR] [--list]
 *   SPEC: qnx | qnx:C,D | wav:FILE | raw:-   (see audio.h)
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
    char goto_host[128], say_host[128];
    int say_port, say_ms;
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

/* Send one line, read one reply line into reply. Returns 0 on OK/ERR reply, -1 if no link. */
static int link_cmd(link_t *l, const char *line, char *reply, int rlen)
{
    if (l->dry) { snprintf(reply, (size_t)rlen, "(dry)"); return 0; }
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!link_up(l)) {
            /* S / CANCEL are worth one immediate retry */
            if (attempt == 0 && (!strcmp(line, "S") || !strcmp(line, "CANCEL"))) { l->next_try = 0; continue; }
            snprintf(reply, (size_t)rlen, "(no link)");
            return -1;
        }
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
        if (o == -2) { snprintf(reply, (size_t)rlen, "(sent, no reply)"); return 0; }
        if (o < 0) { l->next_try = 0; continue; }
        reply[o] = '\0';
        return 0;
    }
    snprintf(reply, (size_t)rlen, "(no link)");
    return -1;
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
    if (!strcmp(line, "STATUS")) fprintf(stderr, "[status] %s\n", reply);
    else printf("[%s] %-16s -> %s (%lld ms)\n", l->name, line, reply, now_ms() - t0);
    fflush(stdout);
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

/* ------------------------------------------------------------------ main */
static void usage(void)
{
    fprintf(stderr,
        "usage: tars_voice [-c voice.conf] [--input qnx|qnx:C,D|wav:FILE|raw:-] [--model FILE]\n"
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
    snprintf(c.say_host, sizeof c.say_host, "127.0.0.1"); c.say_port = 5573; c.say_ms = 150; c.wake_carry_ms = 4000; c.button_talk = 1;
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
    if (!stt) { fprintf(stderr, "[voice] can't load model %s (run voice/get_model.sh)\n", c.model); return 1; }
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

    int16_t frame[VAD_FRAME];
    int fill = 0, nseg = 0;
    long long led_off_at = 0, samples = 0, last_ping = 0;
    long long wake_until = -1;             /* sample count until which a bare "TARS" arms the next segment */
    long long talk_until_ms = 0, last_btn_poll = 0;   /* talk button: wall clock */
    int btn_seen = -1;
    while (running) {
        int r = audio_read(au, frame + fill, VAD_FRAME - fill);
        const int16_t *seg = NULL;
        int segn = 0, done = 0;
        if (r < 0) { fprintf(stderr, "[voice] audio read error\n"); break; }
        if (r == 0) {
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
            if (stt_run(stt, seg, segn, text, sizeof text) != 0) { fprintf(stderr, "[voice] whisper failed\n"); continue; }
            long long stt_ms = now_ms() - ts;
            intent_decision_t d;
            int armed = (wake_until >= 0 && samples - segn <= wake_until) || now_ms() < talk_until_ms;
            intent_decide(text, c.require_wake && !armed, &d);
            if (d.action == INTENT_ACT_IGNORE && d.wake && d.why && !strcmp(d.why, "empty")) {
                wake_until = samples + (long long)c.wake_carry_ms * AUDIO_RATE / 1000;
                printf("[wake] \"%s\": listening for a command for %d ms\n", text, c.wake_carry_ms);
            } else if (d.action != INTENT_ACT_IGNORE) {
                wake_until = -1;
                talk_until_ms = 0;
            }
            static const char *act[] = { "ignore", "stop", "command", "reject" };
            printf("[heard] \"%s\" at=%.2fs len=%.2fs stt=%lldms -> %s %s %.2f (%s)", text,
                   (double)samples / AUDIO_RATE, (double)segn / AUDIO_RATE, stt_ms, act[d.action], d.intent,
                   d.prob, d.why ? d.why : "");
            for (int i = 0; i < d.n_lines; i++) printf("%s%s", i ? " | " : ": ", d.lines[i]);
            printf("\n");
            fflush(stdout);
            for (int i = 0; i < d.n_lines; i++) route(&link, &golink, d.lines[i]);
            if (d.action == INTENT_ACT_STOP) link_send_log(&link, "BEEP 1");
            /* only now (S and CANCEL already sent): tell the phone, urgent = cut any speech */
            if (d.action == INTENT_ACT_STOP) say_line("{\"type\":\"say\",\"text\":\"Stopping.\",\"priority\":\"urgent\"}");
            if (d.action == INTENT_ACT_REJECT) led_off_at = now_ms() + c.led_ms;
        }
        if (done) break;
    housekeeping:
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
    }
    running = 0;
    if (ka_started) pthread_join(ka_tid, NULL);
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
