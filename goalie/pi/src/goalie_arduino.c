/* Arduino Uno backend: text lines over serial (115200 8N1), the Uno does PWM,
 * the stepper and the accelerometer. Protocol: goalie.h / docs/architecture.md.
 *
 * Device path (-d):
 *   /dev/serusb1  default: Uno on USB.
 *                 TODO(on-device): verify the QNX 8.0 Pi 5 image enumerates the
 *                 Uno's USB serial (ATmega16U2, CDC-ACM) and what it's called.
 *                 Check `ls /dev/ser*` with and without the Uno plugged in;
 *                 io-usb-otg + devc-serusb must be running.
 *   /dev/ser1     fallback: Uno TX/RX on Pi GPIO14/15 UART (pins 8/10). The Uno
 *                 TX is 5 V; divide it to 3.3 V before the Pi RX pin.
 *
 * Opening the USB port usually resets the Uno (DTR); it boots in ~1.5 s and
 * then sends READY. Lines sent before that are lost, which is harmless because
 * M is resent every tick. */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "goalie.h"

#define UNO_BAUD B115200

static int fd = -1;
static char rx[256];
static size_t rx_used;

static int write_line(const char *s)
{
    if (fd < 0) return -1;
    size_t n = strlen(s);
    ssize_t w = write(fd, s, n);
    /* Non-blocking: if the TX buffer is full, drop it; M is resent next tick. */
    return w == (ssize_t)n ? 0 : -1;
}

static int ard_init(const char *dev)
{
    fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[uno] open %s: %s\n", dev, strerror(errno));
        return -1;
    }
    struct termios t;
    if (tcgetattr(fd, &t) < 0) {
        fprintf(stderr, "[uno] tcgetattr: %s\n", strerror(errno));
        close(fd); fd = -1;
        return -1;
    }
    cfmakeraw(&t);
    cfsetispeed(&t, UNO_BAUD);
    cfsetospeed(&t, UNO_BAUD);
    t.c_cflag |= CLOCAL | CREAD | CS8;
    t.c_cflag &= ~(CSTOPB | PARENB);
    tcsetattr(fd, TCSANOW, &t);
    tcflush(fd, TCIOFLUSH);
    rx_used = 0;
    fprintf(stderr, "[uno] opened %s @115200 (waiting for READY is not required)\n", dev);
    return write_line("S\n");
}

static int ard_motor(float duty)
{
    char line[24];
    snprintf(line, sizeof line, "M %.3f\n", duty);
    return write_line(line);
}

static int ard_stop(void) { return write_line("S\n"); }
static int ard_zero(void) { return write_line("Z\n"); }

static int ard_needle(int steps)
{
    char line[24];
    snprintf(line, sizeof line, "N %d\n", steps);
    return write_line(line);
}

static int ard_celebrate(int ms)
{
    char line[24];
    snprintf(line, sizeof line, "C %d\n", ms);
    return write_line(line);
}

static int ard_poll(char *buf, size_t n)
{
    if (fd < 0 || n == 0) return 0;
    for (;;) {
        /* A complete line already buffered? */
        char *nl = memchr(rx, '\n', rx_used);
        if (nl) {
            size_t len = (size_t)(nl - rx);
            size_t cp = len < n - 1 ? len : n - 1;
            memcpy(buf, rx, cp);
            buf[cp] = '\0';
            if (cp > 0 && buf[cp - 1] == '\r') buf[cp - 1] = '\0';
            rx_used -= len + 1;
            memmove(rx, nl + 1, rx_used);
            return 1;
        }
        if (rx_used == sizeof rx) rx_used = 0;          /* garbage, no newline: drop */
        ssize_t r = read(fd, rx + rx_used, sizeof rx - rx_used);
        if (r <= 0) return 0;                           /* EAGAIN or nothing */
        rx_used += (size_t)r;
    }
}

static void ard_shutdown(void)
{
    if (fd < 0) return;
    write_line("S\n");
    tcdrain(fd);
    close(fd);
    fd = -1;
}

const goalie_backend_t goalie_arduino = {
    .name = "arduino",
    .init = ard_init,
    .motor = ard_motor,
    .stop = ard_stop,
    .needle = ard_needle,
    .celebrate = ard_celebrate,
    .zero_needle = ard_zero,
    .poll_event = ard_poll,
    .inject = NULL,
    .shutdown = ard_shutdown,
};
