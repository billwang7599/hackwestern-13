/* Separate per-process audit files. Called outside the motor control lock.
 * TARS_ACTION_DIR overrides the QNX default ($HOME); host tests opt in. */
#ifndef TARS_ACTION_LOG_H
#define TARS_ACTION_LOG_H
#include <ctype.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static void action_log(const char *source, int client, const char *action, const char *result)
{
    static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    static FILE *file;
    const char *dir = getenv("TARS_ACTION_DIR");
#ifdef __QNXNTO__
    if (!dir) dir = getenv("HOME");
#endif
    if (!dir || !*dir) return;
    char path[1024], old[1030];
    if (snprintf(path, sizeof path, "%s/tars-%s-actions.log", dir, source) >= (int)sizeof path) return;
    pthread_mutex_lock(&mutex);
    if (!file) file = fopen(path, "a");
    if (file && ftell(file) >= 8*1024*1024) {
        fclose(file);
        snprintf(old, sizeof old, "%s.1", path);
        rename(path, old);
        file = fopen(path, "a");
    }
    if (file) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        struct tm utc;
        gmtime_r(&ts.tv_sec, &utc);
        char stamp[32];
        strftime(stamp, sizeof stamp, "%Y-%m-%dT%H:%M:%S", &utc);
        fprintf(file, "%s.%03ldZ %s client=%d ", stamp, ts.tv_nsec/1000000, source, client);
        const char *fields[] = {action, result};
        for (int i=0; i<2; ++i) {
            if (i) fputs(" -> ", file);
            for (const unsigned char *p=(const unsigned char *)fields[i]; *p && *p!='\n'; ++p)
                fputc(*p>=32 && *p!=127 ? *p : ' ', file);
        }
        fputc('\n', file);
        fflush(file);
    }
    pthread_mutex_unlock(&mutex);
}
#endif
