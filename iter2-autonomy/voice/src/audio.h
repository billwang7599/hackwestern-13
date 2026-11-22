/* Audio input for tars_voice: 16 kHz mono signed 16-bit, from one of
 *
 *   qnx            QNX io-audio (libasound) preferred capture device   [QNX only]
 *   qnx:C,D        QNX card C, device D (see `mic_test -l`)            [QNX only]
 *   wav:FILE       a WAV file (any rate / channels / 16-bit PCM; downmixed + resampled)
 *   raw:-          raw s16le 16 kHz mono on stdin (pipe from any recorder), raw:FILE too
 *   phone          the iPhone mic stream (AirPods), TCP 127.0.0.1:5577 (iproxy to the phone)
 *   phone:PORT     same, another port;  phone:HOST:PORT  another host too
 *
 * phone: protocol "MIC1" (the phone listens, tars_voice connects, like SAY on 5573):
 *   phone -> Pi: one header line "MIC1 16000 1 s16le\n", then packets
 *                <type:1 byte> <len:2 bytes little-endian> <len bytes>
 *     'A' audio: len bytes of 16 kHz mono s16le (len even, 2..8192)
 *     'M' mute   (len 0): the phone is talking (half duplex); drop audio until 'U'
 *     'U' unmute (len 0)
 *     'K' keepalive (len 0): at least once a second when no audio flows
 *   Anything else (bad header, unknown type, odd/oversize len) = protocol error: drop the
 *   link and reconnect. No packet for PHONE_IDLE_MS = dead link: drop and reconnect.
 *   Reconnect backoff 0.5 s doubling to 5 s. The Pi never writes on this socket.
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
 * Live sources block until n samples are available, except phone:, which waits at most
 * ~100 ms and then returns
 *   AUDIO_IDLE  nothing to hear right now (link down, muted, no data yet): run housekeeping
 *   AUDIO_GAP   the stream was interrupted (mute or link lost): the samples before and after
 *               aren't continuous, so end any speech segment in progress (once per gap). */
#define AUDIO_IDLE (-2)
#define AUDIO_GAP  (-3)
#define PHONE_DEFAULT_PORT 5577
#define PHONE_IDLE_MS 3000
int  audio_read(audio_src_t *a, int16_t *buf, int n);
int  audio_is_live(const audio_src_t *a);
void audio_close(audio_src_t *a);
/* phone: one-line state for logs/status ("up", "muted", "down"); "" for other sources */
const char *audio_state(const audio_src_t *a);

/* QNX: print the capture devices (card, device, name). Returns count or < 0. */
int  audio_list_devices(void);

/* Write a 16 kHz mono 16-bit WAV. Returns 0 on success. */
int  wav_write(const char *path, const int16_t *s, int n);

#endif
