/* intent_cli: run the C intent pipeline on stdin lines (tests, parity, timing).
 *
 *   intent_cli [--any]            one line per input:
 *                                 action<TAB>intent<TAB>prob<TAB>wake<TAB>why<TAB>line | line
 *   intent_cli --bench N          time N passes over the input, print us/sentence
 *   intent_cli --probs            also print all class probabilities
 * --any = don't require the wake word.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "intent.h"

static const char *act_name[] = { "ignore", "stop", "command", "reject" };

int main(int argc, char **argv)
{
    int require_wake = 1, bench = 0, probs = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--any")) require_wake = 0;
        else if (!strcmp(argv[i], "--bench") && i + 1 < argc) bench = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--probs")) probs = 1;
    }
    static char lines[20000][256];
    int n = 0;
    char buf[1024];
    while (n < 20000 && fgets(buf, sizeof buf, stdin)) {
        buf[strcspn(buf, "\r\n")] = '\0';
        snprintf(lines[n++], sizeof lines[0], "%s", buf);
    }
    intent_decision_t d;
    if (bench > 0) {
        struct timespec a, b;
        clock_gettime(CLOCK_MONOTONIC, &a);
        volatile int sink = 0;
        for (int r = 0; r < bench; r++)
            for (int i = 0; i < n; i++) { intent_decide(lines[i], require_wake, &d); sink += d.action; }
        clock_gettime(CLOCK_MONOTONIC, &b);
        double us = ((b.tv_sec - a.tv_sec) * 1e6 + (b.tv_nsec - a.tv_nsec) / 1e3) / ((double)bench * n);
        printf("%.2f us/sentence over %d sentences x %d passes\n", us, n, bench);
        return 0;
    }
    for (int i = 0; i < n; i++) {
        intent_decide(lines[i], require_wake, &d);
        printf("%s\t%s\t%.6f\t%d\t%s\t", act_name[d.action], d.intent, d.prob, d.wake, d.why ? d.why : "");
        for (int j = 0; j < d.n_lines; j++) printf("%s%s", j ? " | " : "", d.lines[j]);
        if (probs) {
            char nb[512];
            char *toks[64];
            int nt = intent_normalize(lines[i], nb, sizeof nb, toks, 64);
            float p[32];
            intent_probs(toks, nt, p);     /* note: includes the wake word; debug only */
            printf("\t");
            for (int k = 0; k < intent_count(); k++) printf("%s=%.3f ", intent_name(k), p[k]);
        }
        printf("\n");
    }
    return 0;
}
