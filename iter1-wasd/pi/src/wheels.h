/* Stepper-wheel geometry and motion profile. Defaults MIRROR uno/tars/tars.ino
 * (WHEEL_* constants); keep them in sync. The server uses them to estimate how
 * long MOVE/TURN take (reply + timeout), the stub backend to simulate it. */
#ifndef TARS_WHEELS_H
#define TARS_WHEELS_H
#include <math.h>

#define WHEEL_STEPS_PER_REV  4075.7728f  /* 28BYJ-48: 64 half-steps x 63.684:1 */
#define WHEEL_DIAM_MM        65.0f
#define WHEEL_TRACK_MM       100.0f
#define WHEEL_MAX_RPM        12.0f
#define WHEEL_ACCEL_SPS2     3000.0f
#define WHEEL_MIN_SPS        100.0f
#define MOVE_MAX_MM          1000.0f
#define TURN_MAX_DEG         360.0f

static inline float wheel_max_sps(void) { return WHEEL_MAX_RPM * WHEEL_STEPS_PER_REV / 60.0f; }

/* half-steps each wheel turns for a move ('M' mm or 'R' deg) */
static inline long wheel_move_steps(char kind, float amount, float diam, float track)
{
    float mm = kind == 'M' ? amount : 3.14159265f * track * amount / 360.0f;
    return lroundf(fabsf(mm) * WHEEL_STEPS_PER_REV / (3.14159265f * diam));
}

/* trapezoid (or triangle) profile duration in ms */
static inline int wheel_move_ms(long steps, float speed)
{
    if (steps <= 0) return 0;
    float v = wheel_max_sps() * (speed > 0 && speed <= 1 ? speed : 1), a = WHEEL_ACCEL_SPS2;
    float ramp = v * v / a;                       /* steps spent accelerating + braking */
    float t = steps >= ramp ? 2 * v / a + (steps - ramp) / v : 2 * sqrtf(steps / a);
    return (int)(t * 1000.0f + 0.5f);
}

/* steps done after t_ms of the same profile (the stub uses it for ABORT amounts) */
static inline long wheel_steps_at(long steps, float speed, int t_ms)
{
    int total = wheel_move_ms(steps, speed);
    if (t_ms >= total) return steps;
    if (t_ms <= 0) return 0;
    float v = wheel_max_sps() * (speed > 0 && speed <= 1 ? speed : 1), a = WHEEL_ACCEL_SPS2;
    float t = t_ms / 1000.0f, T = total / 1000.0f;
    float vp = fminf(v, a * T / 2);               /* peak speed */
    float ta = vp / a;
    float s;
    if (t < ta) s = 0.5f * a * t * t;
    else if (t < T - ta) s = 0.5f * a * ta * ta + vp * (t - ta);
    else { float r = T - t; s = steps - 0.5f * a * r * r; }
    return lroundf(fminf(fmaxf(s, 0), (float)steps));
}
#endif
