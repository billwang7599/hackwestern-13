/* Stub backend: prints commands instead of moving anything.
 * Use it to test networking/watchdog with no hardware attached.
 * Also the starting point for a direct-GPIO backend on the Pi 5 (see TODO). */
#include <stdio.h>
#include <math.h>
#include "motor.h"

static float last_l = 99.0f, last_r = 99.0f;

static int stub_init(const char *dev)
{
    (void)dev;
    fprintf(stderr, "[stub] init\n");
    /* TODO(direct-gpio): open the QNX Pi GPIO resource manager here and set
     * the TB6612FNG pins as outputs. Pi 5 GPIO goes through the RP1 chip; check
     * the QNX RPi5 BSP docs / sample apps for the GPIO + PWM interface. */
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

static int stub_gesture(const char *name)
{
    fprintf(stderr, "[stub] gesture %s\n", name);
    return 0;
}

static void stub_shutdown(void)
{
    fprintf(stderr, "[stub] shutdown (motors stopped)\n");
}

const motor_backend_t motor_stub = {
    "stub", stub_init, stub_drive, stub_gesture, stub_shutdown,
};
