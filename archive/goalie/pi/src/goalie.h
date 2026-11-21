/* Pluggable actuator backend for goalie_motor (pick with -b).
 *
 * Wire protocol to the Uno (goalie/docs/architecture.md, "Link 2"):
 *   Pi -> Uno:  "M <duty>"  "S"  "N <steps>"  "C <ms>"  "Z"
 *   Uno -> Pi:  "READY"  "HIT <mag_mg>"  "A <mg>"
 *
 * All calls except init/shutdown come from the real-time control thread, so
 * they must never block: drop a line rather than wait (the next tick resends M).
 */
#ifndef GOALIE_BACKEND_H
#define GOALIE_BACKEND_H

#include <stddef.h>

typedef struct {
    const char *name;
    int  (*init)(const char *dev);
    int  (*motor)(float duty);         /* M, duty in [-1, 1], + moves paddle to +x */
    int  (*stop)(void);                /* S: all motors off */
    int  (*needle)(int steps);         /* N: scoreboard needle absolute position */
    int  (*celebrate)(int ms);         /* C: flag motor for ms */
    int  (*zero_needle)(void);         /* Z: needle's current position = 0 */
    /* Non-blocking: copy one complete line from the Uno (no '\n') into buf.
     * Returns 1 if a line was copied, 0 if none is ready. */
    int  (*poll_event)(char *buf, size_t n);
    /* Test hook: queue a fake Uno line (e.g. "HIT 900"). NULL = unsupported
     * (only the stub has it; the TCP command SIMHIT uses it). */
    int  (*inject)(const char *line);
    void (*shutdown)(void);
} goalie_backend_t;

extern const goalie_backend_t goalie_arduino;
extern const goalie_backend_t goalie_stub;

#endif
