#include "vad.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_MS 20
#define MIN_FLOOR 1e-7             /* ~ -70 dBFS: keeps the ratio test sane in digital silence */

void vad_defaults(vad_cfg_t *c)
{
    c->ratio = 6.0f;
    c->min_dbfs = -45.0f;
    c->start_frames = 3;
    c->hang_ms = 400;
    c->pre_ms = 300;
    c->min_ms = 150;
    c->max_ms = 3000;
}

int vad_init(vad_t *v, const vad_cfg_t *c)
{
    memset(v, 0, sizeof *v);
    v->c = *c;
    v->floor = 1e-5;               /* -50 dBFS until it learns the room */
    v->ring_n = (c->pre_ms / FRAME_MS) * VAD_FRAME;
    v->ring = calloc((size_t)(v->ring_n > 0 ? v->ring_n : 1), sizeof(int16_t));
    v->seg_cap = (c->pre_ms + c->max_ms + 2 * FRAME_MS) * 16;
    v->seg = malloc((size_t)v->seg_cap * sizeof(int16_t));
    return v->ring && v->seg ? 0 : -1;
}

void vad_free(vad_t *v) { free(v->ring); free(v->seg); memset(v, 0, sizeof *v); }

float vad_noise_dbfs(const vad_t *v) { return (float)(10.0 * log10(v->floor)); }

static void seg_add(vad_t *v, const int16_t *s, int n)
{
    if (v->seg_n + n > v->seg_cap) n = v->seg_cap - v->seg_n;
    if (n > 0) { memcpy(v->seg + v->seg_n, s, (size_t)n * 2); v->seg_n += n; }
}

static void ring_add(vad_t *v, const int16_t *s, int n)
{
    if (v->ring_n <= 0) return;
    for (int i = 0; i < n; i++) {
        v->ring[v->ring_pos] = s[i];
        v->ring_pos = (v->ring_pos + 1) % v->ring_n;
    }
    v->ring_fill = v->ring_fill + n > v->ring_n ? v->ring_n : v->ring_fill + n;
}

static int finish(vad_t *v, const int16_t **seg, int *n)
{
    int speech_ms = v->speech_frames * FRAME_MS;
    v->in_speech = 0;
    v->quiet_run = v->loud_run = 0;
    v->ring_fill = 0;
    if (speech_ms < v->c.min_ms) { v->seg_n = 0; v->speech_frames = 0; return 0; }
    *seg = v->seg;
    *n = v->seg_n;
    v->seg_n = 0;
    v->speech_frames = 0;
    return 1;
}

int vad_push(vad_t *v, const int16_t *f, const int16_t **seg, int *n)
{
    double e = 0;
    for (int i = 0; i < VAD_FRAME; i++) { double x = f[i] / 32768.0; e += x * x; }
    e /= VAD_FRAME;
    v->last_dbfs = (float)(10.0 * log10(e + 1e-12));
    v->frames++;

    /* minimum statistics: floor = quietest frame in the last 1.5 s, smoothed upward */
    v->hist[v->hist_pos] = e;
    v->hist_pos = (v->hist_pos + 1) % VAD_HIST;
    if (v->hist_n < VAD_HIST) v->hist_n++;
    double mn = v->hist[0];
    for (int i = 1; i < v->hist_n; i++) if (v->hist[i] < mn) mn = v->hist[i];
    if (mn < MIN_FLOOR) mn = MIN_FLOOR;
    v->floor = mn < v->floor ? mn : 0.9 * v->floor + 0.1 * mn;

    int loud = e > v->floor * v->c.ratio && v->last_dbfs > v->c.min_dbfs;
    if (!v->in_speech) {
        v->loud_run = loud ? v->loud_run + 1 : 0;
        if (v->loud_run >= v->c.start_frames) {
            /* start: pre-roll (which already holds the first loud frames) + this frame */
            v->in_speech = 1;
            v->seg_n = 0;
            int start = (v->ring_pos - v->ring_fill + v->ring_n) % (v->ring_n ? v->ring_n : 1);
            for (int i = 0; i < v->ring_fill; i++) v->seg[v->seg_n++] = v->ring[(start + i) % v->ring_n];
            seg_add(v, f, VAD_FRAME);
            v->speech_frames = v->loud_run;
            v->quiet_run = 0;
            v->seg_start_frame = v->frames;
            return 0;
        }
        ring_add(v, f, VAD_FRAME);
        return 0;
    }
    seg_add(v, f, VAD_FRAME);
    if (loud) { v->speech_frames++; v->quiet_run = 0; }
    else v->quiet_run++;
    int dur_ms = (int)((v->frames - v->seg_start_frame) * FRAME_MS);
    if (v->quiet_run * FRAME_MS >= v->c.hang_ms || dur_ms >= v->c.max_ms)
        return finish(v, seg, n);
    return 0;
}

int vad_flush(vad_t *v, const int16_t **seg, int *n)
{
    if (!v->in_speech) return 0;
    return finish(v, seg, n);
}
