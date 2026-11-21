/* Standalone bench test for uno/tars/tars.ino. Use a serial port with no other
 * writer. Build on QNX or macOS: cc -Wall -Wextra -O2 serial_move.c -o serial_move
 * Run: ./serial_move /path/to/serial-port  (wheels lifted for observation)
 */
#include "../../shared/action_log.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t interrupted;
static void cancel(int sig) { (void)sig; interrupted = 1; }

static long long milliseconds(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static int send_line(int fd, const char *line)
{
    const char *commands = line;
    size_t left = strlen(line);
    while (left) {
        ssize_t n = write(fd, line, left);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) { action_log("motor", -1, commands + strspn(commands, "\n"), "serial write failed"); perror("serial write"); return -1; }
        left -= (size_t)n;
        line += n;
    }
    while (*commands) {
        commands += strspn(commands, "\n");
        if (!*commands) break;
        action_log("motor", -1, commands, "serial TX");
        commands += strcspn(commands, "\n");
    }
    return 0;
}

/* Read complete lines up to a single monotonic deadline. */
static int wait_reply(int fd, int moving, int timeout_ms)
{
    long long end = milliseconds() + timeout_ms;
    char line[256];
    size_t used = 0;
    while (!interrupted) {
        int remaining = (int)(end - milliseconds());
        if (remaining <= 0) break;
        struct pollfd port = {fd, POLLIN, 0};
        int rc = poll(&port, 1, remaining);
        if (rc < 0 && errno == EINTR) continue;
        if (!rc) break;
        if (rc < 0 || !(port.revents & POLLIN)) { fputs("Serial connection lost\n", stderr); return -1; }
        char ch;
        if (read(fd, &ch, 1) != 1) { perror("serial read"); return -1; }
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (used + 1 >= sizeof line) { fputs("Unexpected serial line length\n", stderr); return -1; }
            line[used++] = ch;
            continue;
        }
        line[used] = 0;
        printf("Uno: %s\n", line);
        used = 0;
        if (!moving && !strncmp(line, "# arms ", 7) && strstr(line, " wheels ")) return 0;
        if (moving && !strncmp(line, "ABORT M ", 8)) return -1;
        if (moving && !strncmp(line, "DONE M ", 7)) return 0;
    }
    fprintf(stderr, "%s waiting for %s\n", interrupted ? "Interrupted" : "Timed out",
            moving ? "Uno move completion" : "Uno firmware status");
    return -1;
}

enum pattern { PULSES, PULSES60, PULSES100, ALTERNATE, FULL };

static int pulse_wheels(int fd, enum pattern pattern)
{
    const long long start = milliseconds();
    const int seconds = (pattern == PULSES60 || pattern == PULSES100) ? 20 : 10;
    for (int phase = 0; phase < seconds; ++phase) {
        const long long deadline = start + (phase + 1) * 1000;
        const char *command = phase % 2 == 0 ? "D 0.50 0.50\n" : NULL;
        const char *label = command ? "both forward" : "stop";
        if (pattern == PULSES60 && command) {
            command = "D 0.60 0.60\n";
            label = "both forward (60%)";
        } else if (pattern == FULL || (pattern == PULSES100 && command)) {
            command = "D 1.00 1.00\n";
            label = "full (100%)";
        } else if (pattern == ALTERNATE) {
            command = phase % 2 == 0 ? "D 0.12 0.12\n" : "D 0.60 0.60\n";
            label = phase % 2 == 0 ? "low (12%)" : "high (60%)";
        }
        if (interrupted || milliseconds() >= deadline) return -1;
        printf("Second %d: %s\n", phase + 1, label);
        if (!command && send_line(fd, "S\n") < 0) return -1;
        while (!interrupted) {
            long long remaining = deadline - milliseconds();
            if (remaining <= 0) break;
            if (command && send_line(fd, command) < 0) return -1;
            /* Refresh before the Uno's 200 ms drive watchdog expires. */
            int delay = remaining > 50 ? 50 : (int)remaining;
            if (poll(NULL, 0, delay) < 0 && errno != EINTR) return -1;
        }
    }
    if (interrupted || send_line(fd, "S\n?\n") < 0) return -1;
    return wait_reply(fd, 0, 2000);
}

/* Input records come from the Pi's keyboard_hid helper at 50 Hz. */
static int keyboard_wheels(int fd)
{
    char line[128];
    int armed = 0, active = 0;
    long long started = 0, previous_frame = 0;
    puts("W/S forward/reverse, A/D pivot, Space stop, F return to dashboard");
    fflush(stdout);
    while (!interrupted && fgets(line, sizeof line, stdin)) {
        long long stamp;
        int attached, mask;
        char extra;
        if (sscanf(line, "%lld %d %d %c", &stamp, &attached, &mask, &extra) != 3)
            return -1;
        long long now = milliseconds();
        if (now - previous_frame >= 200) armed = 0;
        previous_frame = now;
        long long age = now - stamp / 1000;
        const char *command = NULL;
        if (attached == 1 && age >= 0 && age < 300) {
            if (mask == 32) return 0;
            if (!mask) armed = 1;
            else if (armed) {
                switch (mask) {
                case 1: command = "D 0.60 0.60\n"; break;
                case 2: command = "D -0.60 -0.60\n"; break;
                case 4: command = "D -0.60 0.60\n"; break;
                case 8: command = "D 0.60 -0.60\n"; break;
                }
                if (!active) started = now;
                if (now - started >= 2000 || (active && active != mask)) command = NULL;
                if (!command) armed = 0;
            }
        } else armed = 0;
        if (command || active) {
            if (send_line(fd, command ? command : "S\n") < 0) return -1;
            if (active != (command ? mask : 0)) {
                printf("Keyboard: %s", command ? command : "stop\n");
                fflush(stdout);
            }
        }
        active = command ? mask : 0;
    }
    return interrupted ? 0 : -1;
}

int main(int argc, char **argv)
{
    int pulses = argc == 3 && !strcmp(argv[2], "--pulses");
    int pulses60 = argc == 3 && !strcmp(argv[2], "--pulses60");
    int pulses100 = argc == 3 && !strcmp(argv[2], "--pulses100");
    int alternate = argc == 3 && !strcmp(argv[2], "--alternate");
    int wasd = argc == 3 && !strcmp(argv[2], "--wasd");
    int full = argc == 3 && !strcmp(argv[2], "--full");
    if (argc != 2 && !pulses && !pulses60 && !pulses100 && !alternate && !full && !wasd) { fprintf(stderr, "Usage: %s SERIAL_PORT [--pulses|--pulses60|--pulses100|--alternate|--full|--wasd]\n", argv[0]); return 2; }
    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(argv[1]); return 1; }
    struct termios t;
    if (tcgetattr(fd, &t) < 0) { perror("tcgetattr"); close(fd); return 1; }
    cfmakeraw(&t);
    t.c_cflag |= CLOCAL | CREAD;
    t.c_cflag &= ~CSTOPB;
    if (cfsetispeed(&t, B115200) || cfsetospeed(&t, B115200) || tcsetattr(fd, TCSANOW, &t)) {
        perror("serial setup"); close(fd); return 1;
    }
    signal(SIGINT, cancel);
    signal(SIGTERM, cancel);
    sleep(2);  /* Opening a hardware port may reset the Uno. */
    int result = 1;
    if (!interrupted && tcflush(fd, TCIFLUSH) == 0 &&
        send_line(fd, "\nS\n?\n") == 0 && wait_reply(fd, 0, 2000) == 0 && !interrupted) {
        if (wasd) {
            result = keyboard_wheels(fd) == 0 ? 0 : 1;
        } else if (pulses || pulses60 || pulses100 || alternate || full) {
            result = pulse_wheels(fd, full ? FULL : alternate ? ALTERNATE : pulses100 ? PULSES100 : pulses60 ? PULSES60 : PULSES) == 0 ? 0 : 1;
        } else {
            puts("Sending M 20 0.25 (20 mm forward at 25% speed)");
            if (send_line(fd, "M 20 0.25\n") == 0 && wait_reply(fd, 1, 5000) == 0)
                result = 0;
        }
    }
    if (send_line(fd, "\nS\n") < 0) result = 1;
    close(fd);
    return result;
}
