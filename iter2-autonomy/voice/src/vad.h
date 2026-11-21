/* Energy voice-activity detector: cuts the mic stream into short speech
 * segments so whisper only runs when someone talks (CPU stays ~idle otherwise).
 *
 * Works on 20 ms frames. Noise floor = the quietest frame of the last 1.5 s
 * (minimum statistics: pauses between words reach the floor, so it follows
 * steady fan/crowd noise without a "learn while quiet" deadlock). Speech starts after start_frames frames louder than floor * ratio (and above
 * min_dbfs), ends after hang_ms of quiet, or is cut at max_ms. Keeps pre_ms of
 * audio from before the start so the first syllable ("TARS") isn't clipped.
 * Timing is by sample count, so a WAV file gives the same result every run.
 */
#ifndef TARS_VAD_H
#define TARS_VAD_H
#include <stdint.h>

#define VAD_FRAME 320                      /* 20 ms at 16 kHz */

typedef struct {
    float ratio;          /* speech if energy > noise floor * ratio (power), e.g. 6 (~7.8 dB) */
    float min_dbfs;       /* and louder than this, e.g. -45 */
    int   start_frames;   /* this many loud frames in a row to start, e.g. 3 (60 ms) */
    int   hang_ms;        /* quiet this long ends the segment, e.g. 400 */
    int   pre_ms;         /* pre-roll kept before the start, e.g. 300 */
    int   min_ms;         /* drop segments with less speech than this (clicks), e.g. 150 (a short "Stop!" is ~200 ms) */
    int   max_ms;         /* cut segments at this length, e.g. 3000 */
} vad_cfg_t;

typedef struct {
    vad_cfg_t c;
    double floor;          /* noise power estimate */
#define VAD_HIST 75            /* 1.5 s of frame energies for the minimum */
    double hist[VAD_HIST];
    int    hist_n, hist_pos;
    int    in_speech, loud_run, quiet_run, speech_frames;
    int16_t *ring;         /* pre-roll ring buffer */
    int    ring_n, ring_pos, ring_fill;
    int16_t *seg;          /* current segment */
    int    seg_n, seg_cap;
    long long frames;      /* frames seen */
    long long seg_start_frame;
    float  last_dbfs;
} vad_t;

void vad_defaults(vad_cfg_t *c);
int  vad_init(vad_t *v, const vad_cfg_t *c);
void vad_free(vad_t *v);
/* Feed one 20 ms frame. Returns 1 when a segment is complete: then *seg / *n
 * point at it (valid until the next call). */
int  vad_push(vad_t *v, const int16_t *frame, const int16_t **seg, int *n);
/* End of input: flush a segment in progress (same return as vad_push). */
int  vad_flush(vad_t *v, const int16_t **seg, int *n);
float vad_noise_dbfs(const vad_t *v);

#endif
