/* Stub backend: no hardware. Logs what would be sent to the Uno.
 *
 * M is sent 200x/s, so it's printed only when the duty changes by more than
 * STUB_M_DELTA or every STUB_M_PERIOD_MS. Everything else is printed as-is.
 *
 * Test hook: inject() queues a fake Uno line ("HIT 900", "READY", ...) that the
 * next poll_event() returns. The TCP command `SIMHIT <mag>` uses it, so tests
 * can fake an accelerometer hit without an Uno. */
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "goalie.h"

#define STUB_M_DELTA      0.05f
#define STUB_M_PERIOD_MS  500
#define STUB_QUEUE        8

static float last_printed = 1e9f;
static struct timespec last_print;
static int stopped_printed;

static pthread_mutex_t qlock = PTHREAD_MUTEX_INITIALIZER;
static char queue[STUB_QUEUE][64];
static int q_head, q_count;

static long ms_since(const struct timespec *t)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - t->tv_sec) * 1000L + (now.tv_nsec - t->tv_nsec) / 1000000L;
}

static int stub_init(const char *dev)
{
    fprintf(stderr, "[stub] no hardware (dev '%s' ignored)\n", dev);
    clock_gettime(CLOCK_MONOTONIC, &last_print);
    return 0;
}

static int stub_motor(float duty)
{
    stopped_printed = 0;
    if (fabsf(duty - last_printed) > STUB_M_DELTA || ms_since(&last_print) >= STUB_M_PERIOD_MS) {
        printf("[stub] M %.3f\n", duty);
        fflush(stdout);
        last_printed = duty;
        clock_gettime(CLOCK_MONOTONIC, &last_print);
    }
    return 0;
}

static int stub_stop(void)
{
    if (!stopped_printed) {
        printf("[stub] S\n");
        fflush(stdout);
        stopped_printed = 1;
        last_printed = 1e9f;   /* print the next M */
    }
    return 0;
}

static int stub_needle(int steps)   { printf("[stub] N %d\n", steps); fflush(stdout); return 0; }
static int stub_celebrate(int ms)   { printf("[stub] C %d\n", ms);    fflush(stdout); return 0; }
static int stub_zero(void)          { printf("[stub] Z\n");           fflush(stdout); return 0; }

static int stub_inject(const char *line)
{
    pthread_mutex_lock(&qlock);
    int ok = q_count < STUB_QUEUE;
    if (ok) {
        snprintf(queue[(q_head + q_count) % STUB_QUEUE], sizeof queue[0], "%s", line);
        q_count++;
    }
    pthread_mutex_unlock(&qlock);
    return ok ? 0 : -1;
}

static int stub_poll(char *buf, size_t n)
{
    int got = 0;
    pthread_mutex_lock(&qlock);
    if (q_count > 0 && n > 0) {
        snprintf(buf, n, "%s", queue[q_head]);
        q_head = (q_head + 1) % STUB_QUEUE;
        q_count--;
        got = 1;
    }
    pthread_mutex_unlock(&qlock);
    return got;
}

static void stub_shutdown(void) { printf("[stub] S (shutdown)\n"); fflush(stdout); }

const goalie_backend_t goalie_stub = {
    .name = "stub",
    .init = stub_init,
    .motor = stub_motor,
    .stop = stub_stop,
    .needle = stub_needle,
    .celebrate = stub_celebrate,
    .zero_needle = stub_zero,
    .poll_event = stub_poll,
    .inject = stub_inject,
    .shutdown = stub_shutdown,
};
