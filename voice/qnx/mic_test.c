/* mic_test: 10-minute check that the USB mic works under QNX io-audio, BEFORE
 * anything else in voice/ (PLAN.md hardware check 5).
 *
 *   ./mic_test -l                 list sound cards / PCM devices
 *   ./mic_test                    record 3 s from the preferred capture device -> mic.wav
 *   ./mic_test -d 1,0 -s 5 -o x.wav   card 1 device 0, 5 seconds
 *
 * Prints the level every 0.5 s and a verdict. Copy mic.wav to the laptop and
 * listen: scp qnxuser@<pi-ip>:voice/mic.wav . && afplay mic.wav
 * Same capture code (src/audio.c) as tars_voice, so if this works, voice input works.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "audio.h"

int main(int argc, char **argv)
{
    const char *dev = NULL, *out = "mic.wav";
    double secs = 3.0;
    int opt;
    while ((opt = getopt(argc, argv, "ld:s:o:h")) != -1) {
        switch (opt) {
        case 'l': return audio_list_devices() < 0 ? 1 : 0;
        case 'd': dev = optarg; break;
        case 's': secs = atof(optarg); break;
        case 'o': out = optarg; break;
        default:
            fprintf(stderr, "usage: mic_test [-l] [-d card,dev] [-s seconds] [-o file.wav]\n");
            return opt == 'h' ? 0 : 2;
        }
    }
    char spec[64], err[256] = "";
    snprintf(spec, sizeof spec, dev ? "qnx:%s" : "qnx", dev);
    audio_src_t *a = audio_open(spec, err, sizeof err);
    if (!a) {
        fprintf(stderr, "FAIL: can't open the mic: %s\n"
                        "  - is the mic plugged in? is io-audio running? (pidin | grep io-audio)\n"
                        "  - list devices: ./mic_test -l, then ./mic_test -d <card>,<dev>\n", err);
        return 1;
    }
    int n = (int)(secs * AUDIO_RATE);
    int16_t *buf = malloc((size_t)n * sizeof *buf);
    if (!buf) return 1;
    printf("recording %.1f s at 16 kHz mono... speak now (say \"TARS, turn left\")\n", secs);
    int got = 0, chunk = AUDIO_RATE / 2;
    double peak_all = 0, rms_min = 1e9, rms_max = 0;
    while (got < n) {
        int want = n - got < chunk ? n - got : chunk;
        int r = audio_read(a, buf + got, want);
        if (r <= 0) { fprintf(stderr, "FAIL: read error after %d samples\n", got); break; }
        double e = 0, pk = 0;
        for (int i = 0; i < r; i++) {
            double x = buf[got + i] / 32768.0;
            e += x * x;
            if (fabs(x) > pk) pk = fabs(x);
        }
        double rms = sqrt(e / r), db = 20 * log10(rms + 1e-9);
        if (rms < rms_min) rms_min = rms;
        if (rms > rms_max) rms_max = rms;
        if (pk > peak_all) peak_all = pk;
        int bars = db < -60 ? 0 : (int)((db + 60) / 2);
        printf("  %5.1fs  %6.1f dBFS  %.*s\n", (got + r) / (double)AUDIO_RATE, db, bars,
               "##############################");
        fflush(stdout);
        got += r;
    }
    audio_close(a);
    if (got == 0) return 1;
    if (wav_write(out, buf, got) != 0) { fprintf(stderr, "FAIL: can't write %s\n", out); return 1; }
    double lo = 20 * log10(rms_min + 1e-9), hi = 20 * log10(rms_max + 1e-9);
    printf("wrote %s (%d samples). quietest %.1f dBFS, loudest %.1f dBFS, peak %.2f\n", out, got, lo, hi, peak_all);
    if (peak_all < 1e-4) printf("VERDICT: FAIL - pure silence. Wrong device (-l), muted, or no mic.\n");
    else if (hi - lo < 10) printf("VERDICT: WEAK - speech is < 10 dB above the background. Speak closer / louder, check gain.\n");
    else if (peak_all > 0.99) printf("VERDICT: OK but CLIPPING - too loud, back off a little.\n");
    else printf("VERDICT: OK - speech %.0f dB above background. Listen to %s to be sure.\n", hi - lo, out);
    free(buf);
    return 0;
}
