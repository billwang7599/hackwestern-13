/* GPIO backend: the Pi drives the L298N directly (no Uno). Wiring = the
 * "Pi to L298N Wiring" guide:
 *
 *   left  (A): ENA GPIO12 (pin 32), IN1 GPIO16 (pin 36), IN2 GPIO20 (pin 38)
 *   right (B): ENB GPIO13 (pin 33), IN3 GPIO21 (pin 40), IN4 GPIO26 (pin 37)
 *   10 kOhm ENA->GND and ENB->GND keep the wheels off while the Pi boots.
 *
 * Pins go through QNX's rpi_gpio resource manager text nodes (/dev/gpio/N:
 * "out", "on", "off"). Speed is software PWM on ENA/ENB from one thread at
 * 100 Hz; that thread is the only one that touches the pins.
 *
 * -d options (comma separated, all optional): invl, invr (flip a side's
 * direction), swap (left<->right), min=<0..1> (duty for the smallest non-zero
 * drive, default 0.35: below that the SM11 motors just hum), mm_s=, deg_s=
 * (speeds at D 1, default 150 / 120, the SM11 kit numbers).
 *
 * MOVE/TURN are timed open loop like the Uno's DC firmware: MOVE at full
 * duty, TURN at full (speed >= 0.99) or slow (0.40) duty, for amount / rate.
 * Reported to tars_motor as WHEELS DC + DCSLOW, so its pose and move-time
 * estimates match. A non-zero drive() aborts a running move.
 *
 * Fail safe: if drive() isn't called for 300 ms (control thread stuck) the PWM
 * thread turns both sides off. shutdown() leaves every pin low. A SIGKILL or a
 * crash can leave the last pin state: the 10k pull-downs only help while the
 * pins are inputs, so keep the battery switch in reach. */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "motor.h"

#define PWM_PERIOD_NS 10000000L
#define FEED_TIMEOUT_NS 300000000LL

enum { ENA, IN1, IN2, ENB, IN3, IN4, NPIN };
static const int pin_gpio[NPIN] = { 12, 16, 20, 13, 21, 26 };
static int pin_fd[NPIN] = { -1, -1, -1, -1, -1, -1 };
static int pin_on[NPIN];

static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_t pwm_thread;
static volatile int pwm_running;
static float want_l, want_r;          /* protected by mu */
static long long fed_ns;              /* protected by mu */
static int inv_l, inv_r, swap_lr;
static float min_duty = 0.35f;
static int write_errors;
static float rate_mm_s = 150.0f, rate_deg_s = 120.0f;
#define SLOW_TURN 0.40f

/* timed exact move, protected by mu */
static char mv_kind;
static float mv_amount, mv_l, mv_r;
static long long mv_start_ns, mv_dur_ns;
#define EQ 8
static motor_event_t evq[EQ];
static int evq_n, wheels_sent;

static long long mono_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void pin_set(int p, int on)
{
    if (pin_fd[p] < 0 || pin_on[p] == on) return;
    const char *v = on ? "on" : "off";
    lseek(pin_fd[p], 0, SEEK_SET);
    if (write(pin_fd[p], v, strlen(v)) < 0) {
        if (write_errors++ < 5)
            fprintf(stderr, "[gpio] GPIO%d %s: %s\n", pin_gpio[p], v, strerror(errno));
        return;
    }
    pin_on[p] = on;
}

static void all_off(void)
{
    pin_set(ENA, 0); pin_set(ENB, 0);
    for (int p = 0; p < NPIN; p++) pin_set(p, 0);
}

static float duty_of(float v)
{
    float a = fabsf(v);
    if (a < 0.03f) return 0.0f;
    if (a > 1.0f) a = 1.0f;
    return min_duty + (1.0f - min_duty) * a;
}

/* direction pins first, with that side's enable off while it flips */
static void set_dir(int en, int ina, int inb, float v)
{
    int fwd = v > 0.0f, rev = v < 0.0f;
    if ((pin_on[ina] != fwd || pin_on[inb] != rev)) pin_set(en, 0);
    pin_set(ina, fwd);
    pin_set(inb, rev);
}

static void sleep_until(long long t_ns)
{
#ifdef TIMER_ABSTIME
    struct timespec ts = { .tv_sec = t_ns / 1000000000LL, .tv_nsec = t_ns % 1000000000LL };
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {}
#else   /* macOS host build: relative sleep is close enough */
    long long d = t_ns - mono_ns();
    if (d <= 0) return;
    struct timespec ts = { .tv_sec = d / 1000000000LL, .tv_nsec = d % 1000000000LL };
    nanosleep(&ts, NULL);
#endif
}

static void *pwm_main(void *arg)
{
    (void)arg;
    long long start = mono_ns();
    while (pwm_running) {
        pthread_mutex_lock(&mu);
        float l = mv_kind ? mv_l : want_l, r = mv_kind ? mv_r : want_r;
        int stale = mono_ns() - fed_ns > FEED_TIMEOUT_NS;
        pthread_mutex_unlock(&mu);
        if (stale) l = r = 0.0f;
        if (swap_lr) { float t = l; l = r; r = t; }
        if (inv_l) l = -l;
        if (inv_r) r = -r;

        set_dir(ENA, IN1, IN2, l);
        set_dir(ENB, IN3, IN4, r);
        float dl = duty_of(l), dr = duty_of(r);
        pin_set(ENA, dl > 0.0f);
        pin_set(ENB, dr > 0.0f);

        /* turn each enable off at its duty point, earlier one first */
        long long end = start + PWM_PERIOD_NS;
        long long off_l = dl >= 0.99f ? end : start + (long long)(dl * PWM_PERIOD_NS);
        long long off_r = dr >= 0.99f ? end : start + (long long)(dr * PWM_PERIOD_NS);
        int first = off_l <= off_r ? ENA : ENB;
        long long t1 = off_l <= off_r ? off_l : off_r;
        long long t2 = off_l <= off_r ? off_r : off_l;
        if (t1 < end) { sleep_until(t1); pin_set(first, 0); }
        if (t2 < end) { sleep_until(t2); pin_set(first == ENA ? ENB : ENA, 0); }
        sleep_until(end);
        start = end;
        long long now = mono_ns();
        if (now - start > 5 * PWM_PERIOD_NS) start = now;   /* fell behind: resync */
    }
    all_off();
    return NULL;
}

static void parse_opts(const char *dev)
{
    if (!dev) return;
    char buf[128];
    snprintf(buf, sizeof buf, "%s", dev);
    for (char *save = NULL, *tok = strtok_r(buf, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
        if (!strcmp(tok, "invl")) inv_l = 1;
        else if (!strcmp(tok, "invr")) inv_r = 1;
        else if (!strcmp(tok, "swap")) swap_lr = 1;
        else if (!strncmp(tok, "min=", 4)) min_duty = fminf(fmaxf(strtof(tok + 4, NULL), 0.0f), 1.0f);
        else if (!strncmp(tok, "mm_s=", 5) && strtof(tok + 5, NULL) > 0) rate_mm_s = strtof(tok + 5, NULL);
        else if (!strncmp(tok, "deg_s=", 6) && strtof(tok + 6, NULL) > 0) rate_deg_s = strtof(tok + 6, NULL);
        else if (*tok) fprintf(stderr, "[gpio] ignoring option '%s'\n", tok);
    }
}

static int gpio_init(const char *dev)
{
    parse_opts(dev);
    for (int p = 0; p < NPIN; p++) {
        char path[32];
        snprintf(path, sizeof path, "/dev/gpio/%d", pin_gpio[p]);
        pin_fd[p] = open(path, O_WRONLY);
        if (pin_fd[p] < 0 || write(pin_fd[p], "out", 3) != 3) {
            fprintf(stderr, "[gpio] %s: %s (rpi_gpio running? user in group gpio?)\n", path, strerror(errno));
            for (int q = 0; q <= p; q++) if (pin_fd[q] >= 0) { close(pin_fd[q]); pin_fd[q] = -1; }
            return -1;
        }
        pin_on[p] = 1;          /* force the first write */
        pin_set(p, 0);
    }
    fed_ns = mono_ns();
    pwm_running = 1;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    struct sched_param sp = { .sched_priority = 25 };
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    pthread_attr_setschedparam(&attr, &sp);
    if (pthread_create(&pwm_thread, &attr, pwm_main, NULL) != 0 &&
        pthread_create(&pwm_thread, NULL, pwm_main, NULL) != 0) {   /* no RT rights: normal prio */
        pwm_running = 0;
        all_off();
        return -1;
    }
    pthread_attr_destroy(&attr);
    fprintf(stderr, "[gpio] L298N on GPIO 12/16/20 (left) 13/21/26 (right), 100 Hz PWM, min duty %.2f%s%s%s\n",
            min_duty, inv_l ? ", left inverted" : "", inv_r ? ", right inverted" : "", swap_lr ? ", swapped" : "");
    return 0;
}

static void push_locked(motor_event_t ev) { if (evq_n < EQ) evq[evq_n++] = ev; }

/* end the running move (mu held): DONE with the full amount, or ABORT with the part that ran */
static void mv_end_locked(int ok, const char *why)
{
    if (!mv_kind) return;
    motor_event_t ev;
    memset(&ev, 0, sizeof ev);
    ev.kind = ok ? MOTOR_EV_MOVE_DONE : MOTOR_EV_MOVE_ABORT;
    ev.what = mv_kind;
    double f = ok ? 1.0 : (double)(mono_ns() - mv_start_ns) / (double)mv_dur_ns;
    ev.amount = mv_amount * (float)fmin(fmax(f, 0.0), 1.0);
    snprintf(ev.why, sizeof ev.why, "%s", ok ? "" : why);
    mv_kind = 0;
    push_locked(ev);
}

static int gpio_move(char kind, float amount, float speed)
{
    float v = kind == 'R' && speed < 0.99f ? SLOW_TURN : 1.0f;
    float rate = (kind == 'M' ? rate_mm_s : rate_deg_s) * duty_of(v);
    if ((kind != 'M' && kind != 'R') || rate <= 0) return -1;
    pthread_mutex_lock(&mu);
    mv_end_locked(0, "replaced");
    float sign = amount < 0 ? -1.0f : 1.0f;
    mv_l = sign * v;                    /* + TURN = right (clockwise): left fwd, right back */
    mv_r = kind == 'M' ? sign * v : -sign * v;
    mv_kind = kind;
    mv_amount = amount;
    mv_start_ns = mono_ns();
    mv_dur_ns = (long long)(fabsf(amount) / rate * 1e9);
    fed_ns = mv_start_ns;
    pthread_mutex_unlock(&mu);
    return 0;
}

static int gpio_move_abort(void)
{
    pthread_mutex_lock(&mu);
    if (mv_kind) mv_end_locked(0, "stop");
    else {   /* nothing running: still answer, like the Uno */
        motor_event_t ev;
        memset(&ev, 0, sizeof ev);
        ev.kind = MOTOR_EV_MOVE_ABORT;
        snprintf(ev.why, sizeof ev.why, "idle");
        push_locked(ev);
    }
    pthread_mutex_unlock(&mu);
    return 0;
}

static int gpio_poll(motor_event_t *ev)
{
    int got = 0;
    pthread_mutex_lock(&mu);
    if (wheels_sent < 2) {
        memset(ev, 0, sizeof *ev);
        ev->kind = MOTOR_EV_WHEELS;
        if (wheels_sent++ == 0) { ev->what = 'D'; ev->amount = rate_mm_s; ev->amount2 = rate_deg_s; }
        else { ev->what = 'S'; ev->amount = rate_deg_s * duty_of(SLOW_TURN); }
        pthread_mutex_unlock(&mu);
        return 1;
    }
    if (mv_kind && mono_ns() - mv_start_ns >= mv_dur_ns) mv_end_locked(1, "");
    if (evq_n) {
        *ev = evq[0];
        memmove(evq, evq + 1, (size_t)(--evq_n) * sizeof *evq);
        got = 1;
    }
    pthread_mutex_unlock(&mu);
    return got;
}

static int gpio_drive(float left, float right)
{
    pthread_mutex_lock(&mu);
    if (mv_kind && (fabsf(left) >= 0.03f || fabsf(right) >= 0.03f)) mv_end_locked(0, "drive");
    want_l = left;
    want_r = right;
    fed_ns = mono_ns();
    pthread_mutex_unlock(&mu);
    return 0;
}

static void gpio_shutdown(void)
{
    pthread_mutex_lock(&mu);
    mv_kind = 0;
    pthread_mutex_unlock(&mu);
    gpio_drive(0, 0);
    if (pwm_running) {
        pwm_running = 0;
        pthread_join(pwm_thread, NULL);
    }
    all_off();
    for (int p = 0; p < NPIN; p++) if (pin_fd[p] >= 0) { close(pin_fd[p]); pin_fd[p] = -1; }
}

const motor_backend_t motor_gpio = {
    .name = "gpio",
    .default_dev = "",
    .init = gpio_init,
    .drive = gpio_drive,
    .shutdown = gpio_shutdown,
    .move = gpio_move,
    .move_abort = gpio_move_abort,
    .poll = gpio_poll,
};
