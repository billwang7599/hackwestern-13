/* Audio input for tars_voice: 16 kHz mono signed 16-bit, from one of
 *
 *   qnx            QNX io-audio (libasound) preferred capture device   [QNX only]
 *   qnx:C,D        QNX card C, device D (see `mic_test -l`)            [QNX only]
 *   wav:FILE       a WAV file (any rate / channels / 16-bit PCM; downmixed + resampled)
 *   raw:-          raw s16le 16 kHz mono on stdin (pipe from any recorder), raw:FILE too
 *
 * On a Mac, live mic for a quick try:
 *   ffmpeg -f avfoundation -i ":0" -ac 1 -ar 16000 -f s16le - | tars_voice --input raw:-
 */
#ifndef TARS_AUDIO_H
#define TARS_AUDIO_H
#include <stdint.h>

#define AUDIO_RATE 16000

typedef struct audio_src audio_src_t;

audio_src_t *audio_open(const char *spec, char *err, int errlen);
/* Read up to n samples. Returns samples read (> 0), 0 at end of input, < 0 on error.
 * Live sources block until n samples are available. */
int  audio_read(audio_src_t *a, int16_t *buf, int n);
int  audio_is_live(const audio_src_t *a);
void audio_close(audio_src_t *a);

/* QNX: print the capture devices (card, device, name). Returns count or < 0. */
int  audio_list_devices(void);

/* Write a 16 kHz mono 16-bit WAV. Returns 0 on success. */
int  wav_write(const char *path, const int16_t *s, int n);

#endif
