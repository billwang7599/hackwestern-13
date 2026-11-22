#include <math.h>
#include <stddef.h>
#include <strings.h>
#include "gesture.h"

#define K ARM_KEEP

/* Right arm swings up, waggles three times, comes back. ~5.5 s */
static const keyframe_t wave[] = {
    {{K, 70}, 60, 0},
    {{K, 40}, 60, 0}, {{K, 75}, 60, 0},
    {{K, 40}, 60, 0}, {{K, 75}, 60, 0},
    {{K, 40}, 60, 0}, {{K, 75}, 60, 0},
    {{K, 0},  60, 0},
};
/* Right arm straight out front, hold, back. ~4.5 s */
static const keyframe_t point[] = {
    {{K, 90}, 60, 1500},
    {{K, 0},  45, 0},
};
/* Both arms flick forward a little and drop. ~1.3 s */
static const keyframe_t shrug[] = {
    {{25, 25}, 60, 400},
    {{0, 0},   60, 0},
};
/* Small double dip, like a nod. ~1.4 s */
static const keyframe_t nod[] = {
    {{15, 15}, 60, 100},
    {{0, 0},   60, 100},
    {{15, 15}, 60, 100},
    {{0, 0},   60, 0},
};
/* The QNXplorer "standing" pose: outer slabs split fore/aft. Stays there. */
static const keyframe_t stance[] = {
    {{30, -30}, 45, 0},
};
/* Everything flush: the closed monolith. */
static const keyframe_t fold[] = {
    {{0, 0}, 45, 0},
};

#define G(n, w, f) { n, w, f, (int)(sizeof f / sizeof *f) }
static const gesture_t gestures[] = {
    G("wave",   "right arm up, waggle 3x, back down", wave),
    G("point",  "right arm straight forward for 1.5 s", point),
    G("shrug",  "both arms flick forward 25 deg", shrug),
    G("nod",    "small double dip of both arms", nod),
    G("stance", "split outer slabs fore/aft (QNXplorer pose), stays", stance),
    G("fold",   "all slabs flush (monolith), stays", fold),
};

const gesture_t *gesture_at(int i)
{
    if (i < 0 || i >= (int)(sizeof gestures / sizeof *gestures)) return NULL;
    return &gestures[i];
}

const gesture_t *gesture_find(const char *name)
{
    for (const gesture_t *g = gestures; g < gestures + sizeof gestures / sizeof *gestures; g++)
        if (!strcasecmp(g->name, name)) return g;
    return NULL;
}

int gesture_duration_ms(const gesture_t *g, const float start[2])
{
    float pos[2] = { start[0], start[1] };
    float ms = 0;
    for (int i = 0; i < g->n; i++) {
        const keyframe_t *f = &g->frames[i];
        float travel = 0;
        for (int a = 0; a < 2; a++) {
            if (f->deg[a] >= ARM_KEEP) continue;
            float d = fabsf(f->deg[a] - pos[a]) / f->dps;
            if (d > travel) travel = d;
            pos[a] = f->deg[a];
        }
        ms += travel * 1000.0f * FRAME_MARGIN + FRAME_PAD_MS + (float)f->hold_ms;
    }
    return (int)ms;
}
