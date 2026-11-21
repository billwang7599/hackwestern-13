#include "stt.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "whisper.h"

struct stt {
    struct whisper_context *ctx;
    stt_cfg_t c;
    float *buf;
    int cap;
};

static void quiet_log(enum ggml_log_level level, const char *text, void *user)
{
    (void)user;
    if (level >= GGML_LOG_LEVEL_WARN) fputs(text, stderr);
}

stt_t *stt_init(const stt_cfg_t *c)
{
    whisper_log_set(quiet_log, NULL);
    struct whisper_context_params cp = whisper_context_default_params();
    cp.use_gpu = false;
    cp.flash_attn = false;
    struct whisper_context *ctx = whisper_init_from_file_with_params(c->model, cp);
    if (!ctx) return NULL;
    stt_t *s = calloc(1, sizeof *s);
    s->ctx = ctx;
    s->c = *c;
    return s;
}

int stt_run(stt_t *s, const int16_t *pcm, int n, char *text, int len)
{
    /* whisper wants >= 1 s; pad short segments with silence */
    int m = n < 16000 + 1600 ? 16000 + 1600 : n;
    if (m > s->cap) {
        free(s->buf);
        s->buf = malloc((size_t)m * sizeof(float));
        s->cap = s->buf ? m : 0;
        if (!s->buf) return -1;
    }
    for (int i = 0; i < n; i++) s->buf[i] = pcm[i] / 32768.0f;
    for (int i = n; i < m; i++) s->buf[i] = 0.0f;

    struct whisper_full_params p = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
    p.n_threads = s->c.threads;
    p.language = "en";
    p.translate = false;
    p.no_context = true;
    p.no_timestamps = true;
    p.single_segment = true;
    p.print_progress = p.print_realtime = p.print_timestamps = p.print_special = false;
    p.suppress_nst = true;
    p.temperature = 0.0f;
    /* Temperature fallback: whisper re-decodes (T = 0.2, 0.4, ...) only when a decode fails
     * its entropy check (entropy_thold 2.4), which is what a "turn left, turn left, ..." loop
     * looks like. A normal command decodes once, so this costs time only on a loop. On the
     * bench's say clips it ended 11 of 11 runaway loops (13/456 clean clips looped before). */
    p.temperature_inc = 0.2f;
    p.max_tokens = 32;
    p.initial_prompt = s->c.prompt;
    if (s->c.audio_ctx > 0) p.audio_ctx = s->c.audio_ctx;
    else if (s->c.audio_ctx == 0) {
        /* Encoder frames: 50 per second of audio + headroom, but at least 512:
         * below ~384 tiny.en starts repeating the sentence ("TARS, turn left.
         * TARS, turn left. ..."). 512 = ~3x faster than the full 1500 window. */
        int ac = (int)((double)m / 16000.0 * 50.0) + 64;
        if (ac < 512) ac = 512;
        p.audio_ctx = ac > 1500 ? 1500 : ac;
    }
    if (whisper_full(s->ctx, p, s->buf, m) != 0) return -1;
    int o = 0;
    text[0] = '\0';
    for (int i = 0; i < whisper_full_n_segments(s->ctx); i++) {
        const char *t = whisper_full_get_segment_text(s->ctx, i);
        int w = snprintf(text + o, (size_t)(len - o), "%s", t);
        if (w < 0 || o + w >= len) break;
        o += w;
    }
    /* trim */
    char *b = text;
    while (*b == ' ') b++;
    memmove(text, b, strlen(b) + 1);
    for (int k = (int)strlen(text) - 1; k >= 0 && text[k] == ' '; k--) text[k] = '\0';
    stt_dedupe(text);
    return 0;
}

/* Whisper (tiny, noisy input, short window) sometimes loops: "TARS, turn. Go
 * back half a meter. TARS, turn. TARS, turn. ..." or, inside one sentence, "Tars,
 * turn left, turn left, turn left, ...". Two passes:
 *  1. drop a sentence that repeats an earlier one (the rest stays: "Turn left. Turn
 *     left. Stop!" used to be cut to "Turn left." and lose the stop)
 *  2. collapse a run of 1-6 words repeated right after itself to one copy
 *     ("turn left, turn left," -> "turn left,"; "stop stop stop" -> "stop")
 * Words are compared lowercase, letters/digits only. Pass 2 only deletes copies of words
 * that stay in the text, so it can't remove a stop word or create a new one. */
static void key_of(const char *a, const char *b, char *k, int kl)
{
    int o = 0;
    for (const char *p = a; p < b && o < kl - 1; p++) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') c += 32;
        if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')) k[o++] = c;
    }
    k[o] = '\0';
}

static int word_char(char c)
{
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '\'';
}

static void collapse_repeats(char *text)
{
    enum { MAXW = 96, MAXN = 6 };
    int ws[MAXW], we[MAXW], nw = 0;               /* word start/end offsets */
    char key[MAXW][32];
    int L = (int)strlen(text);
    for (int i = 0; i < L && nw < MAXW;) {
        while (i < L && !word_char(text[i])) i++;
        if (i >= L) break;
        ws[nw] = i;
        while (i < L && word_char(text[i])) i++;
        we[nw] = i;
        key_of(text + ws[nw], text + we[nw], key[nw], sizeof key[0]);
        nw++;
    }
    if (nw < 2 || nw == MAXW) return;             /* too long to be a command: leave it */
    int keep[MAXW], nk = 0;                       /* kept word indexes, in order */
    for (int i = 0; i < nw; i++) {
        keep[nk++] = i;
        for (int again = 1; again;) {             /* drop a block that repeats the one before it */
            again = 0;
            for (int n = MAXN; n >= 1; n--) {
                if (nk < 2 * n) continue;
                int same = 1;
                for (int k = 0; k < n && same; k++)
                    same = key[keep[nk - 2 * n + k]][0] && !strcmp(key[keep[nk - 2 * n + k]], key[keep[nk - n + k]]);
                if (same) { nk -= n; again = 1; break; }
            }
        }
    }
    if (nk == nw) return;
    /* a loop cut by max_tokens ends in a partial copy ("go to the laptop, go to the"):
     * drop a tail that is the start of the n-word phrase right before it */
    for (int n = MAXN, done = 0; n >= 2 && !done; n--)
        for (int t = n - 1; t >= 1 && !done; t--) {
            if (nk < n + t) continue;
            int same = 1;
            for (int k = 0; k < t && same; k++)
                same = !strcmp(key[keep[nk - t + k]], key[keep[nk - t - n + k]]);
            if (same) { nk -= t; done = 1; }
        }
    char out[1024];
    int o = 0;
    for (int k = 0; k < nk && o < (int)sizeof out - 1; k++) {
        int w = keep[k];
        int from = k == 0 ? 0 : we[w - 1];        /* the word + the separator just before it */
        int n = we[w] - from;
        if (o + n >= (int)sizeof out - 1) break;
        memcpy(out + o, text + from, (size_t)n);
        o += n;
    }
    int tail = L - we[nw - 1];                    /* trailing punctuation after the last word */
    if (tail > 0 && o + tail < (int)sizeof out) { memcpy(out + o, text + we[nw - 1], (size_t)tail); o += tail; }
    out[o] = '\0';
    memcpy(text, out, (size_t)o + 1);             /* never longer than the input */
}

void stt_dedupe(char *text)
{
    enum { MAXS = 16 };
    char keys[MAXS][96], out[1024];
    int ns = 0, o = 0;
    char *start = text;
    for (char *p = text;; p++) {
        if (*p == '.' || *p == '!' || *p == '?' || *p == '\0') {
            char k[96];
            key_of(start, p, k, sizeof k);
            int dup = 0;
            for (int i = 0; i < ns && k[0]; i++) dup |= !strcmp(keys[i], k);
            /* keep each sentence once, in order; never cut what follows (a later "Stop!") */
            if (!dup) {
                if (k[0] && ns < MAXS) snprintf(keys[ns++], sizeof keys[0], "%s", k);
                int n = (int)(p - start) + (*p ? 1 : 0);
                if (o + n < (int)sizeof out) { memcpy(out + o, start, (size_t)n); o += n; }
            }
            if (*p == '\0') break;
            start = p + 1;
        }
    }
    while (o > 0 && out[o - 1] == ' ') o--;
    char *b = out;
    while (o > 0 && *b == ' ') { b++; o--; }
    if (o >= (int)strlen(text) + 1) o = (int)strlen(text);   /* can't grow; defensive */
    memmove(text, b, (size_t)o);
    text[o] = '\0';
    collapse_repeats(text);
}

void stt_free(stt_t *s)
{
    if (!s) return;
    whisper_free(s->ctx);
    free(s->buf);
    free(s);
}
