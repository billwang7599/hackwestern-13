/* Audio sources: WAV file, raw s16le stream, QNX io-audio capture (audio.h). */
#include "audio.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __QNX__
#include <sys/asoundlib.h>
#endif

typedef enum { SRC_WAV, SRC_RAW, SRC_QNX } src_kind_t;

struct audio_src {
    src_kind_t kind;
    FILE *f;
    /* wav conversion */
    int channels, rate;
    double pos;                    /* resampler position in input frames */
    int16_t *in;                   /* decoded mono input at the file's rate */
    long in_n, in_used;
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
        snprintf(err, errlen, "unknown input '%s' (qnx | qnx:C,D | wav:FILE | raw:-|FILE)", spec);
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
    case SRC_QNX:
#ifdef __QNX__
        return qnx_read(a, buf, n);
#endif
    default:
        return -1;
    }
}

int audio_is_live(const audio_src_t *a) { return a->kind == SRC_QNX || (a->kind == SRC_RAW && a->f == stdin); }

void audio_close(audio_src_t *a)
{
    if (!a) return;
    if (a->f && a->f != stdin) fclose(a->f);
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
