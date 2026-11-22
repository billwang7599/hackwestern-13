/* Audio sources: WAV file, raw s16le stream, QNX io-audio capture, iPhone mic stream (audio.h). */
#include "audio.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifdef __QNX__
#include <sys/asoundlib.h>
#endif

typedef enum { SRC_WAV, SRC_RAW, SRC_QNX, SRC_PHONE } src_kind_t;

#define PHONE_FIFO (2 * AUDIO_RATE)        /* 2 s of samples between the socket and the VAD */
#define PHONE_MAX_PKT 8192
#define PHONE_WAIT_MS 100                  /* audio_read never waits longer than this */
#define PHONE_GAPS 8

typedef struct {
    char host[128];
    int port, fd;
    long long next_try, backoff_ms, last_rx_ms;
    int fails;                     /* connect failures in a row (log the first and every 10th) */
    int hdr_done, hdr_n;
    char hdr[80];
    int pk_n, pk_type, pk_len, pk_got;
    unsigned char pk[3];
    int have_half;
    unsigned char half;
    int muted;
    int16_t *fifo;
    int f_head, f_n;
    long long pushed, popped, dropped;
    long long gaps[PHONE_GAPS];    /* sample positions (pushed count) where the stream broke */
    int ngaps;
} phone_t;

struct audio_src {
    src_kind_t kind;
    FILE *f;
    /* wav conversion */
    int channels, rate;
    double pos;                    /* resampler position in input frames */
    int16_t *in;                   /* decoded mono input at the file's rate */
    long in_n, in_used;
    phone_t *ph;
#ifdef __QNX__
    snd_pcm_t *pcm;
    int frag;                      /* bytes per fragment */
    int16_t *qbuf;
    int q_have, q_off;             /* samples buffered from the last fragment */
#endif
};

static uint32_t rd32(const unsigned char *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint16_t rd16(const unsigned char *p) { return (uint16_t)(p[0] | p[1] << 8); }

/* Load a whole WAV (commands are short), downmix to mono at its own rate. */
static int wav_load(audio_src_t *a, const char *path, char *err, int errlen)
{
    FILE *f = fopen(path, "rb");
    if (!f) { snprintf(err, errlen, "open %s: %s", path, strerror(errno)); return -1; }
    unsigned char h[12], ch[8];
    if (fread(h, 1, 12, f) != 12 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "WAVE", 4)) {
        snprintf(err, errlen, "%s: not a RIFF/WAVE file", path); fclose(f); return -1;
    }
    int fmt_ok = 0, bits = 0;
    while (fread(ch, 1, 8, f) == 8) {
        uint32_t len = rd32(ch + 4);
        if (!memcmp(ch, "fmt ", 4)) {
            unsigned char fm[40];
            uint32_t take = len < sizeof fm ? len : sizeof fm;
            if (fread(fm, 1, take, f) != take) break;
            if (len > take) fseek(f, (long)(len - take), SEEK_CUR);
            int tag = rd16(fm);
            a->channels = rd16(fm + 2);
            a->rate = (int)rd32(fm + 4);
            bits = rd16(fm + 14);
            if (tag == 0xFFFE && take >= 26) tag = rd16(fm + 24);     /* WAVE_FORMAT_EXTENSIBLE */
            fmt_ok = tag == 1 && bits == 16 && a->channels >= 1 && a->rate >= 4000;
            if (len & 1) fseek(f, 1, SEEK_CUR);
        } else if (!memcmp(ch, "data", 4)) {
            if (!fmt_ok) { snprintf(err, errlen, "%s: need 16-bit PCM (got %d-bit)", path, bits); fclose(f); return -1; }
            long frames = (long)(len / (2u * (unsigned)a->channels));
            int16_t *raw = malloc((size_t)frames * 2 * (size_t)a->channels + 2);
            a->in = malloc((size_t)frames * 2 + 2);
            if (!raw || !a->in) { free(raw); snprintf(err, errlen, "out of memory"); fclose(f); return -1; }
            frames = (long)fread(raw, 2 * (size_t)a->channels, (size_t)frames, f);
            for (long i = 0; i < frames; i++) {
                long s = 0;
                for (int c = 0; c < a->channels; c++) s += raw[i * a->channels + c];
                a->in[i] = (int16_t)(s / a->channels);
            }
            free(raw);
            a->in_n = frames;
            fclose(f);
            return 0;
        } else {
            fseek(f, (long)(len + (len & 1)), SEEK_CUR);
        }
    }
    snprintf(err, errlen, "%s: no data chunk", path);
    fclose(f);
    return -1;
}

#ifdef __QNX__
static int qnx_open(audio_src_t *a, const char *arg, char *err, int errlen)
{
    int card = -1, dev = 0, rc;
    if (arg && *arg && sscanf(arg, "%d,%d", &card, &dev) >= 1)
        rc = snd_pcm_open(&a->pcm, card, dev, SND_PCM_OPEN_CAPTURE);
    else
        rc = snd_pcm_open_preferred(&a->pcm, &card, &dev, SND_PCM_OPEN_CAPTURE);
    if (rc < 0) { snprintf(err, errlen, "snd_pcm_open capture (card %d dev %d): %s", card, dev, snd_strerror(rc)); return -1; }
    fprintf(stderr, "[audio] io-audio capture: card %d device %d\n", card, dev);

    snd_pcm_channel_info_t pi;
    memset(&pi, 0, sizeof pi);
    pi.channel = SND_PCM_CHANNEL_CAPTURE;
    if ((rc = snd_pcm_plugin_info(a->pcm, &pi)) < 0) { snprintf(err, errlen, "snd_pcm_plugin_info: %s", snd_strerror(rc)); return -1; }

    snd_pcm_channel_params_t pp;
    memset(&pp, 0, sizeof pp);
    pp.mode = SND_PCM_MODE_BLOCK;
    pp.channel = SND_PCM_CHANNEL_CAPTURE;
    pp.start_mode = SND_PCM_START_DATA;
    pp.stop_mode = SND_PCM_STOP_STOP;
    pp.buf.block.frag_size = 640;              /* 20 ms at 16 kHz mono s16 */
    if (pi.max_fragment_size && pp.buf.block.frag_size > pi.max_fragment_size) pp.buf.block.frag_size = pi.max_fragment_size;
    if (pi.min_fragment_size && pp.buf.block.frag_size < pi.min_fragment_size) pp.buf.block.frag_size = pi.min_fragment_size;
    pp.buf.block.frags_max = -1;
    pp.buf.block.frags_min = 1;
    pp.format.interleave = 1;
    pp.format.rate = AUDIO_RATE;               /* the plugin layer resamples if the mic can't */
    pp.format.voices = 1;
    pp.format.format = SND_PCM_SFMT_S16_LE;
    if ((rc = snd_pcm_plugin_params(a->pcm, &pp)) < 0) { snprintf(err, errlen, "snd_pcm_plugin_params 16k mono s16: %s", snd_strerror(rc)); return -1; }
    if ((rc = snd_pcm_plugin_prepare(a->pcm, SND_PCM_CHANNEL_CAPTURE)) < 0) { snprintf(err, errlen, "prepare: %s", snd_strerror(rc)); return -1; }

    snd_pcm_channel_setup_t setup;
    memset(&setup, 0, sizeof setup);
    setup.channel = SND_PCM_CHANNEL_CAPTURE;
    if ((rc = snd_pcm_plugin_setup(a->pcm, &setup)) < 0) { snprintf(err, errlen, "setup: %s", snd_strerror(rc)); return -1; }
    a->frag = setup.buf.block.frag_size > 0 ? setup.buf.block.frag_size : 640;
    a->qbuf = malloc((size_t)a->frag);
    fprintf(stderr, "[audio] %d Hz, %d voice(s), fragment %d bytes\n", setup.format.rate, setup.format.voices, a->frag);
    if (setup.format.rate != AUDIO_RATE || setup.format.voices != 1)
        fprintf(stderr, "[audio] WARNING: got %d Hz x %d, wanted 16000 x 1\n", setup.format.rate, setup.format.voices);
    return a->qbuf ? 0 : -1;
}

static int qnx_read(audio_src_t *a, int16_t *buf, int n)
{
    int got = 0;
    while (got < n) {
        if (a->q_off < a->q_have) {
            int take = a->q_have - a->q_off;
            if (take > n - got) take = n - got;
            memcpy(buf + got, a->qbuf + a->q_off, (size_t)take * 2);
            a->q_off += take; got += take;
            continue;
        }
        int r = snd_pcm_plugin_read(a->pcm, a->qbuf, (size_t)a->frag);
        if (r < a->frag) {
            snd_pcm_channel_status_t st;
            memset(&st, 0, sizeof st);
            st.channel = SND_PCM_CHANNEL_CAPTURE;
            if (snd_pcm_plugin_status(a->pcm, &st) < 0) return -1;
            if (st.status == SND_PCM_STATUS_READY || st.status == SND_PCM_STATUS_OVERRUN) {
                fprintf(stderr, "[audio] overrun, re-preparing\n");
                if (snd_pcm_plugin_prepare(a->pcm, SND_PCM_CHANNEL_CAPTURE) < 0) return -1;
            }
            if (r <= 0) continue;
        }
        a->q_have = r / 2;
        a->q_off = 0;
    }
    return got;
}

int audio_list_devices(void)
{
    int cards[16], over = 0;
    int nc = snd_cards_list(cards, 16, &over);
    if (nc <= 0) { printf("no sound cards (is io-audio running? `pidin | grep io-audio`)\n"); return nc; }
    int count = 0;
    for (int i = 0; i < nc && i < 16; i++) {
        snd_ctl_t *ctl;
        if (snd_ctl_open(&ctl, cards[i]) < 0) continue;
        snd_ctl_hw_info_t info;
        memset(&info, 0, sizeof info);
        if (snd_ctl_hw_info(ctl, &info) == 0) {
            for (int d = 0; d < (int)info.pcmdevs; d++) {
                snd_pcm_info_t pinfo;
                memset(&pinfo, 0, sizeof pinfo);
                if (snd_ctl_pcm_info(ctl, d, &pinfo) == 0) {
                    printf("card %d device %d: %s / %s%s\n", cards[i], d, info.longname, pinfo.name,
                           (pinfo.flags & SND_PCM_INFO_CAPTURE) ? "  [capture]" : "");
                    count++;
                }
            }
        }
        snd_ctl_close(ctl);
    }
    return count;
}
#else
int audio_list_devices(void)
{
    printf("(device list is QNX-only; on a Mac use `ffmpeg -f avfoundation -list_devices true -i \"\"`)\n");
    return 0;
}
#endif


/* ------------------------------------------------------------------ phone (MIC1 over TCP) */
static long long mono_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void ph_gap(phone_t *p)
{
    if (p->ngaps > 0 && p->gaps[p->ngaps - 1] == p->pushed) return;   /* same spot: one gap */
    if (p->ngaps < PHONE_GAPS) p->gaps[p->ngaps++] = p->pushed;
    else p->gaps[PHONE_GAPS - 1] = p->pushed;                        /* merge into the last */
}

static void ph_push(phone_t *p, int16_t s)
{
    if (p->f_n == PHONE_FIFO) {                /* main loop stuck (whisper): drop the oldest */
        p->f_head = (p->f_head + 1) % PHONE_FIFO;
        p->f_n--;
        p->popped++;
        if (p->dropped++ % AUDIO_RATE == 0) fprintf(stderr, "[phone] audio buffer full, dropping old audio\n");
    }
    p->fifo[(p->f_head + p->f_n) % PHONE_FIFO] = s;
    p->f_n++;
    p->pushed++;
}

static void ph_drop(phone_t *p, const char *why)
{
    if (p->fd >= 0) {
        close(p->fd);
        fprintf(stderr, "[phone] mic link %s:%d dropped: %s\n", p->host, p->port, why);
        if (p->hdr_done) ph_gap(p);
    }
    p->fd = -1;
    p->hdr_done = p->hdr_n = 0;
    p->pk_n = p->pk_len = p->pk_got = p->pk_type = 0;
    p->have_half = 0;
    p->muted = 0;
    p->next_try = mono_ms() + p->backoff_ms;
    p->backoff_ms = p->backoff_ms * 2 > 5000 ? 5000 : p->backoff_ms * 2;
}

/* non-blocking connect bounded by wait_ms; 0 = connected */
static int ph_connect(phone_t *p, int wait_ms)
{
    char port[16];
    snprintf(port, sizeof port, "%d", p->port);
    struct addrinfo hints, *res = NULL;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    int fd = -1, ok = 0, err = 0;
    if (getaddrinfo(p->host, port, &hints, &res) == 0 && res) {
        fd = socket(res->ai_family, res->ai_socktype, 0);
        int fl = fd >= 0 ? fcntl(fd, F_GETFL, 0) : -1;
        if (fl >= 0 && fcntl(fd, F_SETFL, fl | O_NONBLOCK) == 0) {
            ok = connect(fd, res->ai_addr, res->ai_addrlen) == 0;
            if (!ok && errno == EINPROGRESS) {
                struct pollfd pf = { fd, POLLOUT, 0 };
                socklen_t el = sizeof err;
                ok = poll(&pf, 1, wait_ms) == 1 && getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &el) == 0 && err == 0;
            } else if (!ok) err = errno;
        }
        freeaddrinfo(res);
    }
    if (!ok) {
        if (fd >= 0) close(fd);
        if (p->fails++ % 10 == 0)
            fprintf(stderr, "[phone] mic link %s:%d unreachable (%s); retrying\n", p->host, p->port,
                    err ? strerror(err) : "no answer");
        p->next_try = mono_ms() + p->backoff_ms;
        p->backoff_ms = p->backoff_ms * 2 > 5000 ? 5000 : p->backoff_ms * 2;
        return -1;
    }
    p->fd = fd;
    p->fails = 0;
    p->last_rx_ms = mono_ms();
    fprintf(stderr, "[phone] mic link connected to %s:%d\n", p->host, p->port);
    return 0;
}

/* Parse received bytes. -1 = protocol error (caller drops the link). */
static int ph_feed(phone_t *p, const unsigned char *b, int n, char *why, int whylen)
{
    for (int i = 0; i < n; i++) {
        unsigned char ch = b[i];
        if (!p->hdr_done) {
            if (ch == '\n') {
                p->hdr[p->hdr_n] = '\0';
                if (p->hdr_n && p->hdr[p->hdr_n - 1] == '\r') p->hdr[--p->hdr_n] = '\0';
                if (strcmp(p->hdr, "MIC1 16000 1 s16le")) {
                    snprintf(why, (size_t)whylen, "bad header \"%.40s\" (want \"MIC1 16000 1 s16le\")", p->hdr);
                    return -1;
                }
                p->hdr_done = 1;
                p->backoff_ms = 500;           /* a good stream resets the backoff */
                fprintf(stderr, "[phone] mic stream MIC1 16 kHz mono s16le\n");
                continue;
            }
            if (p->hdr_n >= (int)sizeof p->hdr - 1 || ch < 0x20 || ch > 0x7e) {
                snprintf(why, (size_t)whylen, "bad header (not a MIC1 stream)");
                return -1;
            }
            p->hdr[p->hdr_n++] = (char)ch;
            continue;
        }
        if (p->pk_n < 3) {                       /* packet header: type, len lo, len hi */
            p->pk[p->pk_n++] = ch;
            if (p->pk_n < 3) continue;
            p->pk_type = p->pk[0];
            p->pk_len = p->pk[1] | p->pk[2] << 8;
            p->pk_got = 0;
            p->have_half = 0;
            switch (p->pk_type) {
            case 'A':
                if (p->pk_len < 2 || p->pk_len > PHONE_MAX_PKT || (p->pk_len & 1)) {
                    snprintf(why, (size_t)whylen, "bad audio packet length %d", p->pk_len);
                    return -1;
                }
                break;
            case 'M': case 'U': case 'K':
                if (p->pk_len != 0) { snprintf(why, (size_t)whylen, "'%c' packet with length %d", p->pk_type, p->pk_len); return -1; }
                if (p->pk_type == 'M' && !p->muted) { p->muted = 1; ph_gap(p); fprintf(stderr, "[phone] muted (phone is talking)\n"); }
                if (p->pk_type == 'U' && p->muted) { p->muted = 0; fprintf(stderr, "[phone] unmuted\n"); }
                p->pk_n = 0;
                break;
            default:
                snprintf(why, (size_t)whylen, "unknown packet type 0x%02x", p->pk_type);
                return -1;
            }
            continue;
        }
        /* audio payload byte */
        if (p->have_half) {
            if (!p->muted) ph_push(p, (int16_t)(uint16_t)(p->half | (unsigned)ch << 8));
            p->have_half = 0;
        } else {
            p->half = ch;
            p->have_half = 1;
        }
        if (++p->pk_got == p->pk_len) p->pk_n = 0;
    }
    return 0;
}

static int ph_open(audio_src_t *a, const char *arg, char *err, int errlen)
{
    phone_t *p = calloc(1, sizeof *p);
    if (!p || !(p->fifo = malloc(PHONE_FIFO * sizeof(int16_t)))) { free(p); snprintf(err, errlen, "out of memory"); return -1; }
    a->ph = p;
    p->fd = -1;
    snprintf(p->host, sizeof p->host, "127.0.0.1");
    p->port = PHONE_DEFAULT_PORT;
    if (arg && *arg) {
        const char *colon = strrchr(arg, ':');
        if (colon) {
            snprintf(p->host, sizeof p->host, "%.*s", (int)(colon - arg), arg);
            arg = colon + 1;
        }
        char *end;
        long port = strtol(arg, &end, 10);
        if (*end || port <= 0 || port > 65535 || !p->host[0]) {
            snprintf(err, errlen, "bad phone input (phone | phone:PORT | phone:HOST:PORT)");
            return -1;
        }
        p->port = (int)port;
    }
    p->backoff_ms = 500;
    fprintf(stderr, "[phone] mic stream from %s:%d (iPhone/AirPods, MIC1)\n", p->host, p->port);
    return 0;
}

static int ph_read(phone_t *p, int16_t *buf, int n)
{
    long long deadline = mono_ms() + PHONE_WAIT_MS;
    for (;;) {
        int got = 0;
        while (got < n && p->f_n > 0 && !(p->ngaps && p->popped >= p->gaps[0])) {
            buf[got++] = p->fifo[p->f_head];
            p->f_head = (p->f_head + 1) % PHONE_FIFO;
            p->f_n--;
            p->popped++;
        }
        if (got) return got;
        if (p->ngaps && p->popped >= p->gaps[0]) {
            memmove(p->gaps, p->gaps + 1, (size_t)(--p->ngaps) * sizeof p->gaps[0]);
            return AUDIO_GAP;
        }
        long long now = mono_ms(), left = deadline - now;
        if (left <= 0) return AUDIO_IDLE;
        if (p->fd < 0) {
            if (now < p->next_try) {
                long long w = p->next_try - now < left ? p->next_try - now : left;
                struct timespec ts = { (time_t)(w / 1000), (long)(w % 1000) * 1000000L };
                nanosleep(&ts, NULL);
            } else {
                ph_connect(p, (int)left);
            }
            continue;
        }
        struct pollfd pf = { p->fd, POLLIN, 0 };
        int pr = poll(&pf, 1, (int)left);
        if (pr < 0 && errno != EINTR) { ph_drop(p, strerror(errno)); continue; }
        if (pr <= 0) {
            if (mono_ms() - p->last_rx_ms > PHONE_IDLE_MS) ph_drop(p, "no data for 3 s");
            continue;
        }
        unsigned char rb[4096];
        ssize_t r = recv(p->fd, rb, sizeof rb, 0);
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
        if (r <= 0) { ph_drop(p, r == 0 ? "phone closed the stream" : strerror(errno)); continue; }
        p->last_rx_ms = mono_ms();
        char why[96];
        if (ph_feed(p, rb, (int)r, why, sizeof why) < 0) ph_drop(p, why);
    }
}

audio_src_t *audio_open(const char *spec, char *err, int errlen)
{
    audio_src_t *a = calloc(1, sizeof *a);
    if (!a) return NULL;
    if (!strncmp(spec, "wav:", 4)) {
        a->kind = SRC_WAV;
        if (wav_load(a, spec + 4, err, errlen) < 0) { free(a); return NULL; }
    } else if (!strncmp(spec, "raw:", 4)) {
        a->kind = SRC_RAW;
        a->f = !strcmp(spec + 4, "-") ? stdin : fopen(spec + 4, "rb");
        if (!a->f) { snprintf(err, errlen, "open %s: %s", spec + 4, strerror(errno)); free(a); return NULL; }
    } else if (!strcmp(spec, "phone") || !strncmp(spec, "phone:", 6)) {
        a->kind = SRC_PHONE;
        if (ph_open(a, spec[5] == ':' ? spec + 6 : NULL, err, errlen) < 0) { audio_close(a); return NULL; }
    } else if (!strncmp(spec, "qnx", 3)) {
#ifdef __QNX__
        a->kind = SRC_QNX;
        if (qnx_open(a, spec[3] == ':' ? spec + 4 : NULL, err, errlen) < 0) { audio_close(a); return NULL; }
#else
        snprintf(err, errlen, "'qnx' input only exists in the QNX build (use wav:FILE or raw:-)");
        free(a);
        return NULL;
#endif
    } else {
        snprintf(err, errlen, "unknown input '%s' (qnx | qnx:C,D | wav:FILE | raw:-|FILE | phone[:HOST:PORT])", spec);
        free(a);
        return NULL;
    }
    return a;
}

int audio_read(audio_src_t *a, int16_t *buf, int n)
{
    switch (a->kind) {
    case SRC_RAW: {
        size_t r = fread(buf, 2, (size_t)n, a->f);
        return r > 0 ? (int)r : (ferror(a->f) ? -1 : 0);
    }
    case SRC_WAV: {
        int o = 0;
        if (a->rate == AUDIO_RATE) {
            while (o < n && a->in_used < a->in_n) buf[o++] = a->in[a->in_used++];
            return o;
        }
        double step = (double)a->rate / AUDIO_RATE;      /* linear resampler */
        while (o < n) {
            long i = (long)a->pos;
            if (i + 1 >= a->in_n) break;
            double fr = a->pos - (double)i;
            buf[o++] = (int16_t)((1.0 - fr) * a->in[i] + fr * a->in[i + 1]);
            a->pos += step;
        }
        return o;
    }
    case SRC_PHONE:
        return ph_read(a->ph, buf, n);
    case SRC_QNX:
#ifdef __QNX__
        return qnx_read(a, buf, n);
#endif
    default:
        return -1;
    }
}

int audio_is_live(const audio_src_t *a)
{
    return a->kind == SRC_QNX || a->kind == SRC_PHONE || (a->kind == SRC_RAW && a->f == stdin);
}

const char *audio_state(const audio_src_t *a)
{
    if (a->kind != SRC_PHONE) return "";
    return a->ph->fd < 0 || !a->ph->hdr_done ? "down" : a->ph->muted ? "muted" : "up";
}

void audio_close(audio_src_t *a)
{
    if (!a) return;
    if (a->f && a->f != stdin) fclose(a->f);
    if (a->ph) {
        if (a->ph->fd >= 0) close(a->ph->fd);
        free(a->ph->fifo);
        free(a->ph);
    }
#ifdef __QNX__
    if (a->pcm) {
        snd_pcm_plugin_flush(a->pcm, SND_PCM_CHANNEL_CAPTURE);
        snd_pcm_close(a->pcm);
    }
    free(a->qbuf);
#endif
    free(a->in);
    free(a);
}

int wav_write(const char *path, const int16_t *s, int n)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    unsigned char h[44];
    uint32_t data = (uint32_t)n * 2, v;
    memcpy(h, "RIFF", 4); v = 36 + data; memcpy(h + 4, &v, 4);
    memcpy(h + 8, "WAVEfmt ", 8);
    v = 16; memcpy(h + 16, &v, 4);
    uint16_t w = 1; memcpy(h + 20, &w, 2);              /* PCM */
    w = 1; memcpy(h + 22, &w, 2);                       /* mono */
    v = AUDIO_RATE; memcpy(h + 24, &v, 4);
    v = AUDIO_RATE * 2; memcpy(h + 28, &v, 4);
    w = 2; memcpy(h + 32, &w, 2);
    w = 16; memcpy(h + 34, &w, 2);
    memcpy(h + 36, "data", 4); memcpy(h + 40, &data, 4);
    int ok = fwrite(h, 1, 44, f) == 44 && fwrite(s, 2, (size_t)n, f) == (size_t)n;
    return fclose(f) == 0 && ok ? 0 : -1;
}
