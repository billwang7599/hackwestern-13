/* Stub backend: prints commands instead of moving anything.
 * Use it to test networking, watchdog, gestures, exact moves and tip/bump
 * handling with no hardware attached (inject Uno events with SIM, stub only).
 *
 * Exact moves (MOVE/TURN) are simulated with the same trapezoid profile the
 * Uno's stepper wheels use (wheels.h), so DONE arrives after the real time and
 * an abort reports how far it got. Like the Uno: a non-zero drive aborts a move,
 * drive(0, 0) is a keepalive. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "motor.h"
#include "wheels.h"

static float last_l = 99.0f, last_r = 99.0f;

static char  mv_kind;                  /* 0 = no move */
static float mv_amount, mv_speed;
static long  mv_steps;
static long long mv_start;
static float cal_d = WHEEL_DIAM_MM, cal_t = WHEEL_TRACK_MM;
#define SQ 8
static motor_event_t q[SQ];
static int qn;

static long long now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void push(motor_event_t ev) { if (qn < SQ) q[qn++] = ev; }

/* mm or deg moved so far, signed like the request */
static float mv_done_amount(void)
{
    long s = wheel_steps_at(mv_steps, mv_speed, (int)(now_ms() - mv_start));
    if (!mv_steps) return 0;
    return mv_amount * (float)s / (float)mv_steps;
}

static void mv_finish(int ok, const char *why)
{
    if (!mv_kind) return;
    motor_event_t ev;
    memset(&ev, 0, sizeof ev);
    ev.kind = ok ? MOTOR_EV_MOVE_DONE : MOTOR_EV_MOVE_ABORT;
    ev.what = mv_kind;
    ev.amount = ok ? mv_amount : mv_done_amount();
    snprintf(ev.why, sizeof ev.why, "%s", ok ? "" : why);
    fprintf(stderr, "[stub] %s %c %.1f %s\n", ok ? "DONE" : "ABORT", mv_kind, ev.amount, ev.why);
    mv_kind = 0;
    push(ev);
}

static int stub_init(const char *dev)
{
    (void)dev;
    fprintf(stderr, "[stub] init\n");
    return 0;
}

static int stub_drive(float l, float r)
{
    if (mv_kind && (fabsf(l) >= 0.03f || fabsf(r) >= 0.03f)) mv_finish(0, "drive");
    /* Only log changes, or the 50 Hz loop spams the terminal. */
    if (fabsf(l - last_l) > 0.01f || fabsf(r - last_r) > 0.01f) {
        fprintf(stderr, "[stub] drive L=%+.2f R=%+.2f\n", l, r);
        last_l = l; last_r = r;
    }
    return 0;
}

static int stub_move(char kind, float amount, float speed)
{
    mv_finish(0, "replaced");
    mv_kind = kind;
    mv_amount = amount;
    mv_speed = speed;
    mv_steps = wheel_move_steps(kind, amount, cal_d, cal_t);
    mv_start = now_ms();
    fprintf(stderr, "[stub] move %c %.1f speed %.2f (%ld steps, ~%d ms)\n", kind, amount, speed,
            mv_steps, wheel_move_ms(mv_steps, speed));
    if (!mv_steps) mv_finish(1, "");
    return 0;
}

static int stub_move_abort(void) { mv_finish(0, "stop"); return 0; }

static int stub_wheel_cal(float d, float t)
{
    cal_d = d; cal_t = t;
    fprintf(stderr, "[stub] wheel cal diam %.1f track %.1f\n", d, t);
    return 0;
}

static int stub_led(int p)  { fprintf(stderr, "[stub] led %d\n", p); return 0; }
static int stub_beep(int n) { fprintf(stderr, "[stub] beep %d\n", n); return 0; }

static int stub_poll(motor_event_t *ev)
{
    if (mv_kind && now_ms() - mv_start >= wheel_move_ms(mv_steps, mv_speed)) mv_finish(1, "");
    if (!qn) return 0;
    *ev = q[0];
    memmove(q, q + 1, (size_t)(--qn) * sizeof *q);
    return 1;
}

static int arms_halted;

static int stub_arm(int arm, float deg, float dps)
{
    arms_halted = 0;
    fprintf(stderr, "[stub] arm %d -> %.1f deg @ %.0f dps\n", arm, deg, dps);
    return 0;
}

static int stub_arm_halt(void) {
    if (!arms_halted) fprintf(stderr, "[stub] arms halt\n");
    arms_halted = 1;
    return 0;
}
static int stub_arm_zero(void) { fprintf(stderr, "[stub] arms zero\n"); return 0; }
static int stub_level(void)    { fprintf(stderr, "[stub] level (upright recaptured)\n"); return 0; }

static void stub_shutdown(void)
{
    mv_kind = 0;
    fprintf(stderr, "[stub] shutdown (motors stopped)\n");
}

const motor_backend_t motor_stub = {
    .name = "stub",
    .default_dev = "-",
    .init = stub_init,
    .drive = stub_drive,
    .gesture = NULL,              /* gestures run on the Pi as arm keyframes */
    .shutdown = stub_shutdown,
    .arm = stub_arm,
    .arm_halt = stub_arm_halt,
    .arm_zero = stub_arm_zero,
    .level = stub_level,
    .move = stub_move,
    .move_abort = stub_move_abort,
    .wheel_cal = stub_wheel_cal,
    .led = stub_led,
    .beep = stub_beep,
    .poll = stub_poll,            /* move timing; other events come from SIM in main.c */
};
