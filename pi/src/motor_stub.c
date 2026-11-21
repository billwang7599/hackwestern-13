/* Stub backend: prints commands instead of moving anything.
 * Use it to test networking, watchdog, gestures and tip/bump handling with no
 * hardware attached (inject Uno events with the SIM command, stub only). */
#include <stdio.h>
#include <math.h>
#include "motor.h"

static float last_l = 99.0f, last_r = 99.0f;

static int stub_init(const char *dev)
{
    (void)dev;
    fprintf(stderr, "[stub] init\n");
    return 0;
}

static int stub_drive(float l, float r)
{
    /* Only log changes, or the 50 Hz loop spams the terminal. */
    if (fabsf(l - last_l) > 0.01f || fabsf(r - last_r) > 0.01f) {
        fprintf(stderr, "[stub] drive L=%+.2f R=%+.2f\n", l, r);
        last_l = l; last_r = r;
    }
    return 0;
}

static int stub_arm(int arm, float deg, float dps)
{
    fprintf(stderr, "[stub] arm %d -> %.1f deg @ %.0f dps\n", arm, deg, dps);
    return 0;
}

static int stub_arm_halt(void) { fprintf(stderr, "[stub] arms halt\n"); return 0; }
static int stub_arm_zero(void) { fprintf(stderr, "[stub] arms zero\n"); return 0; }
static int stub_level(void)    { fprintf(stderr, "[stub] level (upright recaptured)\n"); return 0; }

static void stub_shutdown(void)
{
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
    .poll = NULL,                 /* events come from SIM in main.c */
};
