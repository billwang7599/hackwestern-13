/* Arm gestures as keyframe tables. The control thread plays them (main.c):
 * each keyframe sends A commands for the arms it names, waits until the arm
 * model says they've arrived, holds, then moves to the next one.
 *
 * Angles: arm 0 = left, 1 = right, degrees, 0 = slab hanging flush with the
 * body, + = top of the slab swings forward. The Uno mirrors arm 1 so + means
 * forward on both sides. */
#ifndef TARS_GESTURE_H
#define TARS_GESTURE_H

#define ARM_KEEP        1000.0f   /* "leave this arm where it is" */
#define ARM_LIMIT_DEG   135.0f    /* same clamp as the Uno */
#define ARM_MAX_DPS     60.0f     /* 28BYJ-48 half-step limit at 5 V (~700 steps/s) */
#define ARM_DEFAULT_DPS 45.0f
/* Playback timing: real steppers run a few % slower than the model. */
#define FRAME_MARGIN    1.05f
#define FRAME_PAD_MS    40

typedef struct {
    float deg[2];     /* target per arm, or ARM_KEEP */
    float dps;        /* speed for this move */
    int   hold_ms;    /* pause after arriving */
} keyframe_t;

typedef struct {
    const char *name;
    const char *what;          /* one-line description (for GESTURES / docs) */
    const keyframe_t *frames;
    int n;
} gesture_t;

const gesture_t *gesture_find(const char *name);   /* NULL if unknown */
const gesture_t *gesture_at(int i);                /* NULL past the end */
/* Expected duration in ms when starting from arm angles start[2]. */
int gesture_duration_ms(const gesture_t *g, const float start[2]);

#endif
