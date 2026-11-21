/* Arduino Uno backend: text lines over USB serial (or the header UART).
 * Firmware + protocol: uno/tars/tars.ino.
 *
 *   Pi -> Uno   "D <l> <r>"  every tick (also feeds the Uno's 200 ms watchdog)
 *               "S"  "A <arm> <deg> <dps>"  "X" (halt arms)  "Z"  "U"
 *   Uno -> Pi   "READY"  "T <deg>"  "B <mg>"  "# ..." (diagnostics, logged only)
 *
 * Fully non-blocking: output goes through a small buffer so a partial write
 * never splits a line; input is split into lines and parsed into events that
 * the control thread drains with poll(). If the device disappears (USB
 * unplugged, Uno reset), it is reopened every second. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "motor.h"

#define UNO_BAUD     B115200
#define OUT_CAP      1024
#define IN_CAP       128
#define EV_CAP       16
#define REOPEN_MS    1000

static char dev_path[128];
static int  fd = -1;
static char outbuf[OUT_CAP];
static size_t out_len;
static char inbuf[IN_CAP];
static size_t in_len;
static int  in_overflow;
static motor_event_t evq[EV_CAP];
static int  ev_head, ev_tail;
static long long last_open_try;
static unsigned dropped;

static long long now_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static int uno_open(void)
{
    last_open_try = now_ms();
    fd = open(dev_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        cfsetispeed(&t, UNO_BAUD);
        cfsetospeed(&t, UNO_BAUD);
        t.c_cflag |= CLOCAL | CREAD;
        t.c_cflag &= ~(CSTOPB | PARENB);
        tcsetattr(fd, TCSANOW, &t);
        tcflush(fd, TCIOFLUSH);
    }
    out_len = in_len = 0;
    in_overflow = 0;
    /* Note: opening the Uno's USB port pulses DTR, which reboots it. It
     * answers READY ~1.5 s later; until then our lines are simply lost. */
    fprintf(stderr, "[uno] opened %s @115200\n", dev_path);
    return 0;
}

static void uno_lost(const char *why)
{
    fprintf(stderr, "[uno] lost %s (%s), retrying every %d ms\n", dev_path, why, REOPEN_MS);
    close(fd);
    fd = -1;
}

static void ev_push(motor_event_kind_t k, int v)
{
    int next = (ev_head + 1) % EV_CAP;
    if (next == ev_tail) return;          /* full: drop (they repeat anyway) */
    evq[ev_head].kind = k;
    evq[ev_head].value = v;
    ev_head = next;
}

static void parse_line(char *s)
{
    if (s[0] == '#') {                     /* diagnostics: log, never act on */
        fprintf(stderr, "[uno] %s\n", s);
        return;
    }
    char *end;
    if (!strcmp(s, "READY")) {
        ev_push(MOTOR_EV_READY, 0);
    } else if ((s[0] == 'T' || s[0] == 'B') && s[1] == ' ') {
        long v = strtol(s + 2, &end, 10);
        if (end == s + 2 || *end) return;  /* malformed */
        ev_push(s[0] == 'T' ? MOTOR_EV_TIP : MOTOR_EV_BUMP, (int)v);
    }
    /* anything else: ignore */
}

static void read_input(void)
{
    char buf[128];
    for (;;) {
        ssize_t n = read(fd, buf, sizeof buf);
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) uno_lost(strerror(errno));
            return;
        }
        if (n == 0) return;
        for (ssize_t i = 0; i < n; i++) {
            char c = buf[i];
            if (c == '\n' || c == '\r') {
                if (!in_overflow && in_len) {
                    inbuf[in_len] = '\0';
                    parse_line(inbuf);
                }
                in_len = 0;
                in_overflow = 0;
            } else if (in_len >= IN_CAP - 1) {
                in_overflow = 1;
            } else if (!in_overflow) {
                inbuf[in_len++] = c;
            }
        }
    }
}

static void flush_out(void)
{
    while (fd >= 0 && out_len) {
        ssize_t w = write(fd, outbuf, out_len);
        if (w < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) uno_lost(strerror(errno));
            return;
        }
        if (w == 0) return;
        memmove(outbuf, outbuf + w, out_len - (size_t)w);
        out_len -= (size_t)w;
    }
}

static int send_line(const char *line)
{
    if (fd < 0) {
        if (now_ms() - last_open_try >= REOPEN_MS) uno_open();
        if (fd < 0) return -1;
    }
    size_t n = strlen(line);
    if (out_len + n > OUT_CAP) {
        if (dropped++ % 100 == 0) fprintf(stderr, "[uno] TX buffer full, dropping lines\n");
        return -1;
    }
    memcpy(outbuf + out_len, line, n);
    out_len += n;
    flush_out();
    return 0;
}

static int uno_init(const char *dev)
{
    snprintf(dev_path, sizeof dev_path, "%s", dev);
    if (uno_open() != 0) {
        fprintf(stderr, "[uno] open %s: %s\n", dev, strerror(errno));
        return -1;
    }
    return send_line("S\n");
}

static int uno_drive(float l, float r)
{
    char line[48];
    snprintf(line, sizeof line, "D %.3f %.3f\n", l, r);
    int rc = send_line(line);
    if (fd >= 0) read_input();
    return rc;
}

static int uno_arm(int arm, float deg, float dps)
{
    char line[48];
    snprintf(line, sizeof line, "A %d %.1f %.0f\n", arm, deg, dps);
    return send_line(line);
}

static int uno_arm_halt(void) { return send_line("X\n"); }
static int uno_arm_zero(void) { return send_line("Z\n"); }
static int uno_level(void)    { return send_line("U\n"); }

static int uno_poll(motor_event_t *ev)
{
    if (ev_tail == ev_head) return 0;
    *ev = evq[ev_tail];
    ev_tail = (ev_tail + 1) % EV_CAP;
    return 1;
}

static void uno_shutdown(void)
{
    if (fd < 0) return;
    out_len = 0;                       /* stop wins over anything still queued */
    send_line("S\n");
    /* Bounded drain: tcdrain() can block forever if nothing reads the port. */
    for (int i = 0; i < 20 && fd >= 0 && out_len; i++) {
        struct timespec ts = { 0, 10 * 1000000L };
        nanosleep(&ts, NULL);
        flush_out();
    }
    if (fd < 0) return;
    close(fd);
    fd = -1;
}

const motor_backend_t motor_uno_serial = {
    .name = "uno",
    .default_dev = "/dev/serusb1",     /* TODO: verify on the Pi 5 QNX image */
    .init = uno_init,
    .drive = uno_drive,
    .gesture = NULL,
    .shutdown = uno_shutdown,
    .arm = uno_arm,
    .arm_halt = uno_arm_halt,
    .arm_zero = uno_arm_zero,
    .level = uno_level,
    .poll = uno_poll,
};
