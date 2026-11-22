"""Power-loss logger: one fsync'd line per second in state/powerlog.log, so after a sudden reboot
the last line says when the Pi died and what the motors were doing.

Line: <UTC time> up=<s since this boot> <tars_motor STATUS reply, or 'motor=down'>.
On start it appends a BOOT line naming the previous run's last line to state/power-events.log
(`cat state/power-events.log` = one line per unexpected restart). Read-only on 5555 (STATUS only).
"""
import fcntl
import os
import socket
import time
from pathlib import Path

STATE = Path(__file__).with_name('state')
LOG = STATE / 'powerlog.log'
EVENTS = STATE / 'power-events.log'
MAX = 5_000_000


def stamp():
    return time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())


def last_line(path):
    try:
        with open(path, 'rb') as f:
            f.seek(max(0, f.seek(0, 2) - 2048))
            lines = f.read().decode(errors='replace').strip().splitlines()
            return lines[-1] if lines else ''
    except OSError:
        return ''


def append(path, line):
    with open(path, 'a') as f:
        f.write(line + '\n')
        f.flush()
        os.fsync(f.fileno())


def main():
    STATE.mkdir(exist_ok=True)
    lock = open(STATE / 'powerlog.lock', 'w')
    try:
        fcntl.lockf(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)   # one logger only
    except OSError:
        return
    append(EVENTS, f'{stamp()} BOOT; previous run last alive: {last_line(LOG) or "(no log)"}')
    start, sock = time.monotonic(), None
    while True:
        try:
            if sock is None:
                sock = socket.create_connection(('127.0.0.1', 5555), timeout=0.5)
                sock.sendall(b'WHO powerlog\n')
                sock.recv(256)
            sock.sendall(b'STATUS\n')
            status = sock.recv(4096).decode(errors='replace').strip().replace('\n', ' ')[:400]
        except OSError:
            if sock:
                sock.close()
            sock, status = None, 'motor=down'
        try:
            if LOG.exists() and LOG.stat().st_size > MAX:
                LOG.replace(LOG.with_suffix('.log.1'))
            append(LOG, f'{stamp()} up={time.monotonic() - start:.0f} {status}')
        except OSError:
            pass
        time.sleep(1)


if __name__ == '__main__':
    main()
