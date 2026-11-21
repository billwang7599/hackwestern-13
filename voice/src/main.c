/* tars_voice: offline speech commands for TARS, on the Pi (QNX) or a Mac.
 *
 *   mic (io-audio) -> VAD (20 ms frames) -> segment <= 3 s -> whisper.cpp tiny.en
 *     -> text -> intent_decide() (safety stop, wake word, classifier, slots)
 *     -> lines to tars_motor (TCP 5555): S | MOVE | TURN | MODE | STATUS, + LED/BEEP
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
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
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

#define DEFAULT_PROMPT "TARS, forward. TARS, back. TARS, turn left. TARS, turn right 90 degrees. " \
                       "TARS, turn around. TARS, stop. TARS, follow me. TARS, gestures. TARS, spin. " \
                       "TARS, go forward half a meter."

typedef struct {
    char input[256], model[256], host[128], save_dir[256], prompt[512];
    int port, threads, audio_ctx, require_wake, dry, led_ms;
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
    const char *host;
    int port, dry;
} link_t;

static void link_drop(link_t *l, const char *why)
{
    if (l->fd >= 0) { close(l->fd); fprintf(stderr, "[link] dropped: %s\n", why); }
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
        fprintf(stderr, "[link] can't resolve %s\n", l->host);
        return 0;
    }
    int fd = socket(res->ai_family, res->ai_socktype, 0);
    if (fd < 0 || connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        fprintf(stderr, "[link] %s:%d unreachable (%s); retrying every 1 s\n", l->host, l->port, strerror(errno));
        if (fd >= 0) close(fd);
        freeaddrinfo(res);
        return 0;
    }
    freeaddrinfo(res);
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
    struct timeval tv = { 1, 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    l->fd = fd;
    fprintf(stderr, "[link] connected to tars_motor %s:%d\n", l->host, l->port);
    return 1;
}

/* Send one line, read one reply line into reply. Returns 0 on OK/ERR reply, -1 if no link. */
static int link_cmd(link_t *l, const char *line, char *reply, int rlen)
{
    if (l->dry) { snprintf(reply, (size_t)rlen, "(dry)"); return 0; }
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!link_up(l)) {
            /* S is the one thing worth one immediate retry */
            if (attempt == 0 && !strcmp(line, "S")) { l->next_try = 0; continue; }
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
            if (r <= 0) { link_drop(l, r == 0 ? "server closed" : strerror(errno)); o = -1; break; }
            if (ch == '\n') break;
            reply[o++] = ch;
        }
        if (o < 0) { l->next_try = 0; continue; }
        reply[o] = '\0';
        return 0;
    }
    snprintf(reply, (size_t)rlen, "(no link)");
    return -1;
}

static void link_send_log(link_t *l, const char *line)
{
    char reply[1024];
    long long t0 = now_ms();
    link_cmd(l, line, reply, sizeof reply);
    if (!strcmp(line, "STATUS")) fprintf(stderr, "[status] %s\n", reply);
    else printf("[cmd] %-14s -> %s (%lld ms)\n", line, reply, now_ms() - t0);
    fflush(stdout);
}

/* ------------------------------------------------------------------ main */
static void usage(void)
{
    fprintf(stderr,
        "usage: tars_voice [-c voice.conf] [--input qnx|qnx:C,D|wav:FILE|raw:-] [--model FILE]\n"
        "                  [--host H] [--port P] [--threads N] [--audio-ctx N] [--any] [--dry]\n"
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
    link_t link = { -1, 0, c.host, c.port, c.dry };
    if (link_up(&link) && !c.dry) {
        char r[64];
        link_cmd(&link, "WHO voice", r, sizeof r);
    }

    int16_t frame[VAD_FRAME];
    int fill = 0, nseg = 0;
    long long led_off_at = 0, samples = 0, last_ping = 0;
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
            intent_decide(text, c.require_wake, &d);
            static const char *act[] = { "ignore", "stop", "command", "reject" };
            printf("[heard] \"%s\" at=%.2fs len=%.2fs stt=%lldms -> %s %s %.2f (%s)", text,
                   (double)samples / AUDIO_RATE, (double)segn / AUDIO_RATE, stt_ms, act[d.action], d.intent,
                   d.prob, d.why ? d.why : "");
            for (int i = 0; i < d.n_lines; i++) printf("%s%s", i ? " | " : ": ", d.lines[i]);
            printf("\n");
            fflush(stdout);
            for (int i = 0; i < d.n_lines; i++) link_send_log(&link, d.lines[i]);
            if (d.action == INTENT_ACT_STOP) link_send_log(&link, "BEEP 1");
            if (d.action == INTENT_ACT_REJECT) led_off_at = now_ms() + c.led_ms;
        }
        if (done) break;
    housekeeping:
        if (led_off_at && now_ms() >= led_off_at) { link_send_log(&link, "LED off"); led_off_at = 0; }
        if (!c.dry && now_ms() - last_ping > 2000) {        /* keep the link warm / reconnect */
            last_ping = now_ms();
            if (link.fd < 0) link_up(&link);
        }
    }
    if (led_off_at) link_send_log(&link, "LED off");
    fprintf(stderr, "[voice] %d segments, %.1f s of audio, noise floor %.1f dBFS\n", nseg,
            (double)samples / AUDIO_RATE, vad_noise_dbfs(&vad));
    vad_free(&vad);
    stt_free(stt);
    audio_close(au);
    link_drop(&link, "exit");
    return 0;
}
