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
    MOTOR_EV_MOVE_DONE,   /* exact move finished: what = 'M'|'R', amount = mm|deg moved */
    MOTOR_EV_MOVE_ABORT,  /* exact move aborted: same + why */
    MOTOR_EV_BTN,         /* on-robot button: value = MOTOR_BTN_TEACH | MOTOR_BTN_FORGET */
    MOTOR_EV_WHEELS,      /* DC wheels (no encoders): amount = mm/s and amount2 = deg/s at D 1 */
} motor_event_kind_t;

#define MOTOR_BTN_TEACH  1
#define MOTOR_BTN_FORGET 2

typedef struct {
    motor_event_kind_t kind;
    int value;
    char what;          /* MOVE_*: 'M' or 'R' */
    float amount;       /* MOVE_*: mm or deg actually moved (signed) */
    float amount2;      /* WHEELS: spin deg/s */
    char why[16];       /* MOVE_ABORT: stop|drive|replaced|tipped|link|... */
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

    /* ---- exact moves (stepper wheels: counted steps; DC wheels: timed). NULL if unsupported. ----
     * kind 'M' = straight, amount mm (+ fwd); 'R' = spin, amount deg (+ = right/cw).
     * speed in (0, 1]. The device answers with a MOVE_DONE / MOVE_ABORT event.
     * While a move runs, drive(0, 0) is a keepalive; non-zero drive aborts it. */
    int  (*move)(char kind, float amount, float speed);
    int  (*move_abort)(void);                      /* answered by MOVE_ABORT */
    int  (*wheel_cal)(float diam_mm, float track_mm);
    /* DC wheels: save the speeds measured at D 1 (0, 0 = just ask). The device
     * answers with a WHEELS event; stepper firmware ignores it. NULL if unsupported. */
    int  (*speed_cal)(float mm_s, float deg_s);

    /* ---- on-robot UI. NULL if unsupported. ---- */
    int  (*led)(int pattern);   /* 0 off, 1 on, 2 slow blink, 3 fast blink */
    int  (*beep)(int n);        /* n short beeps, 0 = silence */

    /* ---- events from the co-processor. NULL = none. 1 = *ev filled. ---- */
    int  (*poll)(motor_event_t *ev);
} motor_backend_t;

extern const motor_backend_t motor_stub;
extern const motor_backend_t motor_pico_uart;
extern const motor_backend_t motor_uno_serial;

#endif
