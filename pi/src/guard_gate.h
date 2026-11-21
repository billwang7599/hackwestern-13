#ifndef TARS_GUARD_GATE_H
#define TARS_GUARD_GATE_H
/* Same-host LiDAR heartbeat gate. All calls are control-thread-only except init. */
int guard_gate_init(int port); /* 0 disables (legacy bench mode); service enables it. */
int guard_gate_poll(void);     /* bounded, nonblocking, true only for fresh CLEAR */
long guard_gate_age_ms(void);
const char *guard_gate_reason(void);
void guard_gate_close(void);
#endif
