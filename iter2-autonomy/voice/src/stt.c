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
    p.temperature_inc = 0.0f;           /* no slow fallback re-decodes */
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
 * back half a meter. TARS, turn. TARS, turn. ..." Cut the text at the first
 * sentence that repeats an earlier one (compared lowercase, letters/digits only). */
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

void stt_dedupe(char *text)
{
    enum { MAXS = 16 };
    char keys[MAXS][96];
    int ns = 0;
    char *start = text;
    for (char *p = text;; p++) {
        if (*p == '.' || *p == '!' || *p == '?' || *p == '\0') {
            char k[96];
            key_of(start, p, k, sizeof k);
            if (k[0]) {
                for (int i = 0; i < ns; i++)
                    if (!strcmp(keys[i], k)) {
                        while (start > text && start[-1] == ' ') start--;
                        *start = '\0';
                        return;
                    }
                if (ns < MAXS) snprintf(keys[ns++], sizeof keys[0], "%s", k);
            }
            if (*p == '\0') return;
            start = p + 1;
        }
    }
}

void stt_free(stt_t *s)
{
    if (!s) return;
    whisper_free(s->ctx);
    free(s->buf);
    free(s);
}
