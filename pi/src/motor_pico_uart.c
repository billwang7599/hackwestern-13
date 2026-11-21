/* Pico co-processor backend: sends text lines over UART, the Pico does PWM.
 *   "D <l> <r>\n"   drive, floats in [-1, 1]
 *   "S\n"           stop
 *   "G <name>\n"    gesture (servo)
 * The Pico also has its own watchdog: if it hears nothing for ~300 ms it stops.
 * We resend D every control tick, which keeps it alive. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "motor.h"

#define PICO_BAUD B115200

static int fd = -1;

static int uart_write_line(const char *s)
{
    if (fd < 0) return -1;
    size_t n = strlen(s);
    ssize_t w = write(fd, s, n);
    if (w != (ssize_t)n) {
        /* Non-blocking: drop the line if the buffer is full; the next tick resends. */
        return -1;
    }
    return 0;
}

static void uart_drain_input(void)
{
    /* The Pico may print debug/acks; discard so the RX buffer never fills.
     * TODO: parse lines like "R <cm>" here if the Pico reads a distance sensor. */
    char buf[128];
    while (fd >= 0 && read(fd, buf, sizeof buf) > 0) {}
}

static int pico_init(const char *dev)
{
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[pico] open %s: %s\n", dev, strerror(errno));
        return -1;
    }
    struct termios t;
    if (tcgetattr(fd, &t) < 0) {
        fprintf(stderr, "[pico] tcgetattr: %s\n", strerror(errno));
        close(fd); fd = -1;
        return -1;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, PICO_BAUD);
    cfsetospeed(&t, PICO_BAUD);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~(CSTOPB | PARENB);
    tcsetattr(fd, TCSANOW, &t);
    tcflush(fd, TCIOFLUSH);
    fprintf(stderr, "[pico] opened %s @115200\n", dev);
    return uart_write_line("S\n");
}

static int pico_drive(float l, float r)
{
    char line[48];
    snprintf(line, sizeof line, "D %.3f %.3f\n", l, r);
    uart_drain_input();
    return uart_write_line(line);
}

static int pico_gesture(const char *name)
{
    char line[64];
    snprintf(line, sizeof line, "G %.40s\n", name);
    return uart_write_line(line);
}

static void pico_shutdown(void)
{
    if (fd < 0) return;
    uart_write_line("S\n");
    tcdrain(fd);
    close(fd);
    fd = -1;
}

/* Legacy: the Pico has one gesture servo and no arms or IMU. Kept compiling in
 * case a Pico turns up; the default backend is now "uno". */
const motor_backend_t motor_pico_uart = {
    .name = "pico",
    .default_dev = "/dev/ser1",
    .init = pico_init,
    .drive = pico_drive,
    .gesture = pico_gesture,
    .shutdown = pico_shutdown,
};
