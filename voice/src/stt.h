/* Speech-to-text: thin wrapper over whisper.cpp (C API). */
#ifndef TARS_STT_H
#define TARS_STT_H
#include <stdint.h>

typedef struct stt stt_t;

typedef struct {
    const char *model;      /* ggml-tiny.en-q5_1.bin */
    int threads;            /* 2: leave cores for vision/YOLO */
    int audio_ctx;          /* 0 = auto (max(512, segment), ~3x faster); -1 = full 30 s window */
    const char *prompt;     /* vocabulary bias */
} stt_cfg_t;

stt_t *stt_init(const stt_cfg_t *c);
/* Transcribe 16 kHz mono samples. Returns 0 and fills text. */
int  stt_run(stt_t *s, const int16_t *pcm, int n, char *text, int len);
void stt_free(stt_t *s);

#endif
