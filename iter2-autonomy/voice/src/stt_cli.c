/* stt_cli: whisper.cpp transcript of one clip, nothing else (no classifier, no robot links).
 *
 *   stt_cli MODEL FILE.wav [threads]   -> one line on stdout: the transcript
 *
 * Laptop-only helper for mission control's --laptop-stt "cheat": the laptop turns
 * push-to-talk audio into text and sends only "HEAR <text>" to the robot, which runs
 * the classifier and decides. Same whisper wrapper (stt.c) and prompt as tars_voice.
 */
#include <stdio.h>
#include <stdlib.h>

#include "audio.h"
#include "stt.h"

#define PROMPT "Robot commands: TARS. forward, back, left, right, stop, freeze, spin, " \
               "turn around, degrees, meter, steps. Go to the trash can, chair, door, " \
               "table, backpack, water bottle, laptop, cup, box, person."

int main(int argc, char **argv)
{
    if (argc < 3) { fprintf(stderr, "usage: stt_cli MODEL FILE.wav [threads]\n"); return 2; }
    char spec[1100], err[256] = "";
    snprintf(spec, sizeof spec, "wav:%s", argv[2]);
    audio_src_t *a = audio_open(spec, err, sizeof err);
    if (!a) { fprintf(stderr, "stt_cli: %s\n", err); return 1; }
    int cap = 30 * AUDIO_RATE, n = 0, r;
    int16_t *pcm = malloc((size_t)cap * sizeof *pcm);
    if (!pcm) return 1;
    while (n < cap && (r = audio_read(a, pcm + n, cap - n)) > 0) n += r;
    audio_close(a);
    stt_cfg_t c = { argv[1], argc > 3 ? atoi(argv[3]) : 4, 0, PROMPT };
    stt_t *s = stt_init(&c);
    if (!s) { fprintf(stderr, "stt_cli: can't load %s\n", argv[1]); return 1; }
    char text[512] = "";
    int rc = n > 0 ? stt_run(s, pcm, n, text, sizeof text) : -1;
    stt_free(s);
    free(pcm);
    if (rc != 0) { fprintf(stderr, "stt_cli: whisper failed\n"); return 1; }
    printf("%s\n", text);
    return 0;
}
