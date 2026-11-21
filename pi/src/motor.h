/* Pluggable motor backend. Pick one at runtime with `-b <name>`.
 * Add a new backend: implement the struct below in motor_<name>.c, add it to
 * motor_backends[] in main.c and to SRCS in the Makefile. */
#ifndef TARS_MOTOR_H
#define TARS_MOTOR_H

typedef struct motor_backend {
    const char *name;
    /* dev = backend-specific string (UART path for pico, ignored for stub). 0 = ok */
    int  (*init)(const char *dev);
    /* left/right in [-1, 1]. Called from the real-time control thread every tick,
     * so keep it fast and non-blocking. 0 = ok */
    int  (*drive)(float left, float right);
    /* named gesture ("wave", "nod", ...). May be NULL. 0 = ok */
    int  (*gesture)(const char *name);
    void (*shutdown)(void);   /* must leave motors stopped */
} motor_backend_t;

extern const motor_backend_t motor_stub;
extern const motor_backend_t motor_pico_uart;

#endif
