/* Pluggable motor backend. Pick one at runtime with `-b <name>`.
 * Add a new backend: implement the struct below in motor_<name>.c, add it to
 * motor_backends[] in main.c and to SRCS in the Makefile.
 *
 * Every function is called ONLY from the real-time control thread, so a
 * backend never needs its own locking. Keep them fast and non-blocking. */
#ifndef TARS_MOTOR_H
#define TARS_MOTOR_H

typedef enum {
    MOTOR_EV_NONE = 0,
    MOTOR_EV_READY,   /* co-processor (re)booted: arm zero is lost */
    MOTOR_EV_TIP,     /* tipped over; value = tilt in degrees */
    MOTOR_EV_BUMP,    /* bump; value = peak deviation in milli-g */
} motor_event_kind_t;

typedef struct {
    motor_event_kind_t kind;
    int value;
} motor_event_t;

typedef struct motor_backend {
    const char *name;
    /* used when -d isn't given */
    const char *default_dev;
    /* dev = backend-specific string (serial path for pico/uno, ignored for stub). 0 = ok */
    int  (*init)(const char *dev);
    /* left/right in [-1, 1]. Called every tick (also keeps the co-processor's
     * own watchdog fed). 0 = ok */
    int  (*drive)(float left, float right);
    /* Device-side named gesture (the Pico's servo). NULL if the backend has
     * arms: then the Pi runs gestures itself as arm keyframes (gesture.c). */
    int  (*gesture)(const char *name);
    void (*shutdown)(void);   /* must leave motors stopped */

    /* ---- arms (two steppers). All NULL if the backend has no arms. ---- */
    /* arm 0 = left, 1 = right; deg absolute (+ = swing forward); dps > 0 */
    int  (*arm)(int arm, float deg, float dps);
    int  (*arm_halt)(void);   /* stop both arms where they are */
    int  (*arm_zero)(void);   /* current position := 0 deg */
    int  (*level)(void);      /* "robot is upright now": recapture tilt reference */

    /* ---- events from the co-processor. NULL = none. 1 = *ev filled. ---- */
    int  (*poll)(motor_event_t *ev);
} motor_backend_t;

extern const motor_backend_t motor_stub;
extern const motor_backend_t motor_pico_uart;
extern const motor_backend_t motor_uno_serial;

#endif
