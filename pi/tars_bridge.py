#!/usr/bin/env python3
"""tars_bridge.py: BRING-UP / DEBUG TOOL ONLY. Not the demo path.

The robot runs the native QNX C server tars_motor (pi/src, built with qcc).
Running natively on QNX is the point of the project. This file exists only to
test the motors and wiring over Wi-Fi BEFORE the QNX toolchain works, and for
debugging later. Don't build features on it.

A stdlib-only Python stand-in for tars_motor (pi/src/main.c).

Same TCP protocol on port 5555, same Uno serial lines, same safety rules, so
laptop/teleop.py, laptop/wheel_cal.py, laptop/tars_client.py and the vision app
work unchanged. Use it when the QNX cross-compile (Docker + SDP 8.0) isn't set
up yet: QNX 8.0 ships Python 3, so you just copy this one file to the Pi.

    python3 tars_bridge.py                      # Uno, serial port auto-detected
    python3 tars_bridge.py -d /dev/serusb1      # Uno on a given port
    python3 tars_bridge.py --stub               # no hardware, prints what it would do
    python3 tars_bridge.py -h

Works on a Mac too (Uno plugged into the Mac, then `python laptop/teleop.py localhost`).

Port of main.c + motor_uno_serial.c + motor_stub.c + gesture.c + wheels.h.
Keep it in sync with them. Behaviour that matters for safety:
  * 500 ms watchdog: no D/S/PING/MOVE/... from any client -> wheels 0.
    No client at boot = wheels held at 0.
  * S from any client stops the wheels on the next tick (no slew) and aborts MOVE/TURN.
  * A client disconnecting stops the wheels and aborts a running MOVE/TURN.
  * Uno "T <deg>" = tipped: wheels latched at 0 until RESET. "B <mg>" = bump:
    driving refused for 1 s.
  * "D 0 0" during a MOVE/TURN is a keepalive; a non-zero D takes over (aborts it).
  * D is resent to the Uno every 20 ms tick (feeds the Uno's own 200 ms watchdog).
  * Missing / unplugged Uno: retried every second, the server keeps running.

Not real-time: the C server's control thread runs SCHED_RR; this one is a plain
Python thread (-r is accepted and ignored). Fine for testing and demos.

Usage: tars_bridge.py [-b uno|stub] [--stub] [-d dev] [-p port] [-w watchdog_ms]
                      [-r rt_prio (ignored)] [-k bump_hold_ms]
"""
from __future__ import annotations

import argparse
import errno
import glob
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

try:
    import termios
except ImportError:          # some minimal builds; we fall back to stty
    termios = None
if os.environ.get("TARS_BRIDGE_NO_TERMIOS"):      # force the stty fallback (tests / odd QNX images)
    termios = None

# ------------------------------------------------------------------ constants (main.c, gesture.h, wheels.h)
TICK_HZ = 50
SLEW_PER_TICK = 0.08
OBSTACLE_STOP_M = 0.25
TIP_CLEAR_MS = 1000

ARM_KEEP = 1000.0
ARM_LIMIT_DEG = 135.0
ARM_MAX_DPS = 60.0
ARM_DEFAULT_DPS = 45.0
FRAME_MARGIN = 1.05
FRAME_PAD_MS = 40

WHEEL_STEPS_PER_REV = 4075.7728
WHEEL_DIAM_MM = 65.0
WHEEL_TRACK_MM = 100.0
WHEEL_MAX_RPM = 12.0
WHEEL_ACCEL_SPS2 = 3000.0
MOVE_MAX_MM = 1000.0
TURN_MAX_DEG = 360.0

REOPEN_MS = 1000
IN_CAP = 128
OUT_CAP = 1024

# Ports a USB Uno shows up as. QNX: devc-serusb -> /dev/serusb*. Linux: ttyACM/ttyUSB.
# macOS: cu.usbmodem (genuine Uno) / cu.usbserial / cu.wchusbserial (CH340 clones).
# /dev/ser1 (the Pi's header UART) is NOT auto-picked: pass -d /dev/ser1 for that.
AUTO_PATTERNS = ["/dev/serusb*", "/dev/ttyACM*", "/dev/ttyUSB*",
                 "/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.wchusbserial*"]
if os.environ.get("TARS_SERIAL_GLOB"):            # e.g. "/dev/serusb*:/dev/ser2" (':'-separated)
    AUTO_PATTERNS = os.environ["TARS_SERIAL_GLOB"].split(":")


def log(msg: str) -> None:
    sys.stderr.write(msg + "\n")
    sys.stderr.flush()


def now_ms() -> int:
    return int(time.monotonic() * 1000)


def clampf(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def lround(x: float) -> int:
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


# ------------------------------------------------------------------ wheels.h
def wheel_max_sps() -> float:
    return WHEEL_MAX_RPM * WHEEL_STEPS_PER_REV / 60.0


def wheel_move_steps(kind: str, amount: float, diam: float, track: float) -> int:
    mm = amount if kind == "M" else math.pi * track * amount / 360.0
    return lround(abs(mm) * WHEEL_STEPS_PER_REV / (math.pi * diam))


def _speed(speed: float) -> float:
    return wheel_max_sps() * (speed if 0 < speed <= 1 else 1)


def wheel_move_ms(steps: int, speed: float) -> int:
    if steps <= 0:
        return 0
    v, a = _speed(speed), WHEEL_ACCEL_SPS2
    ramp = v * v / a
    t = 2 * v / a + (steps - ramp) / v if steps >= ramp else 2 * math.sqrt(steps / a)
    return int(t * 1000.0 + 0.5)


def wheel_steps_at(steps: int, speed: float, t_ms: int) -> int:
    total = wheel_move_ms(steps, speed)
    if t_ms >= total:
        return steps
    if t_ms <= 0:
        return 0
    v, a = _speed(speed), WHEEL_ACCEL_SPS2
    t, T = t_ms / 1000.0, total / 1000.0
    vp = min(v, a * T / 2)
    ta = vp / a
    if t < ta:
        s = 0.5 * a * t * t
    elif t < T - ta:
        s = 0.5 * a * ta * ta + vp * (t - ta)
    else:
        r = T - t
        s = steps - 0.5 * a * r * r
    return lround(min(max(s, 0.0), float(steps)))


# ------------------------------------------------------------------ gesture.c
K = ARM_KEEP
GESTURES = [  # name, frames: ((deg_left, deg_right), dps, hold_ms)
    ("wave", [((K, 70), 60, 0), ((K, 40), 60, 0), ((K, 75), 60, 0), ((K, 40), 60, 0),
              ((K, 75), 60, 0), ((K, 40), 60, 0), ((K, 75), 60, 0), ((K, 0), 60, 0)]),
    ("point", [((K, 90), 60, 1500), ((K, 0), 45, 0)]),
    ("shrug", [((25, 25), 60, 400), ((0, 0), 60, 0)]),
    ("nod", [((15, 15), 60, 100), ((0, 0), 60, 100), ((15, 15), 60, 100), ((0, 0), 60, 0)]),
    ("stance", [((30, -30), 45, 0)]),
    ("fold", [((0, 0), 45, 0)]),
]


def gesture_find(name: str):
    for g in GESTURES:
        if g[0].lower() == name.lower():
            return g
    return None


def gesture_duration_ms(g, start) -> int:
    pos = list(start)
    ms = 0.0
    for deg, dps, hold in g[1]:
        travel = 0.0
        for a in range(2):
            if deg[a] >= ARM_KEEP:
                continue
            d = abs(deg[a] - pos[a]) / dps
            travel = max(travel, d)
            pos[a] = deg[a]
        ms += travel * 1000.0 * FRAME_MARGIN + FRAME_PAD_MS + hold
    return int(ms)


# ------------------------------------------------------------------ events
class Ev:
    READY, TIP, BUMP, DONE, ABORT, BTN = range(6)
    BTN_TEACH, BTN_FORGET = 1, 2

    def __init__(self, kind: int, value: int = 0, what: str = "", amount: float = 0.0, why: str = ""):
        self.kind, self.value, self.what, self.amount, self.why = kind, value, what, amount, why


# ------------------------------------------------------------------ stub backend (motor_stub.c)
class StubBackend:
    name = "stub"

    def __init__(self):
        self.last_l = self.last_r = 99.0
        self.mv_kind = ""
        self.mv_amount = self.mv_speed = 0.0
        self.mv_steps = 0
        self.mv_start = 0
        self.cal_d, self.cal_t = WHEEL_DIAM_MM, WHEEL_TRACK_MM
        self.q: list[Ev] = []

    def describe_dev(self) -> str:
        return "-"

    def init(self) -> int:
        log("[stub] init")
        return 0

    def _done_amount(self) -> float:
        s = wheel_steps_at(self.mv_steps, self.mv_speed, now_ms() - self.mv_start)
        return self.mv_amount * s / self.mv_steps if self.mv_steps else 0.0

    def _finish(self, ok: bool, why: str) -> None:
        if not self.mv_kind:
            return
        amt = self.mv_amount if ok else self._done_amount()
        why = "" if ok else why
        log("[stub] %s %s %.1f %s" % ("DONE" if ok else "ABORT", self.mv_kind, amt, why))
        if len(self.q) < 8:
            self.q.append(Ev(Ev.DONE if ok else Ev.ABORT, what=self.mv_kind, amount=amt, why=why))
        self.mv_kind = ""

    def drive(self, l: float, r: float) -> None:
        if self.mv_kind and (abs(l) >= 0.03 or abs(r) >= 0.03):
            self._finish(False, "drive")
        if abs(l - self.last_l) > 0.01 or abs(r - self.last_r) > 0.01:
            log("[stub] drive L=%+.2f R=%+.2f" % (l, r))
            self.last_l, self.last_r = l, r

    def move(self, kind: str, amount: float, speed: float) -> None:
        self._finish(False, "replaced")
        self.mv_kind, self.mv_amount, self.mv_speed = kind, amount, speed
        self.mv_steps = wheel_move_steps(kind, amount, self.cal_d, self.cal_t)
        self.mv_start = now_ms()
        log("[stub] move %s %.1f speed %.2f (%d steps, ~%d ms)" % (
            kind, amount, speed, self.mv_steps, wheel_move_ms(self.mv_steps, speed)))
        if not self.mv_steps:
            self._finish(True, "")

    def move_abort(self) -> None:
        self._finish(False, "stop")

    def wheel_cal(self, d: float, t: float) -> None:
        self.cal_d, self.cal_t = d, t
        log("[stub] wheel cal diam %.1f track %.1f" % (d, t))

    def led(self, p: int) -> None:
        log("[stub] led %d" % p)

    def beep(self, n: int) -> None:
        log("[stub] beep %d" % n)

    def arm(self, i: int, deg: float, dps: float) -> None:
        log("[stub] arm %d -> %.1f deg @ %.0f dps" % (i, deg, dps))

    def arm_halt(self) -> None:
        log("[stub] arms halt")

    def arm_zero(self) -> None:
        log("[stub] arms zero")

    def level(self) -> None:
        log("[stub] level (upright recaptured)")

    def poll(self):
        if self.mv_kind and now_ms() - self.mv_start >= wheel_move_ms(self.mv_steps, self.mv_speed):
            self._finish(True, "")
        return self.q.pop(0) if self.q else None

    def shutdown(self) -> None:
        self.mv_kind = ""
        log("[stub] shutdown (motors stopped)")


# ------------------------------------------------------------------ serial setup
def _termios_raw(fd: int) -> None:
    """cfmakeraw + 115200 8N1, CLOCAL|CREAD, like motor_uno_serial.c."""
    t = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, _isp, _osp, cc = t
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
               | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
    # VMIN=1 like cfmakeraw: with VMIN=0 a non-blocking read of an empty port
    # returns 0 bytes (looks like a hang-up) instead of EAGAIN.
    cc = list(cc)
    cc[termios.VMIN] = 1 if isinstance(cc[termios.VMIN], int) else b"\x01"
    cc[termios.VTIME] = 0 if isinstance(cc[termios.VTIME], int) else b"\x00"
    speed = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
    termios.tcflush(fd, termios.TCIOFLUSH)


# Tried in order if termios is missing or refuses. All read the port on stdin.
STTY_VARIANTS = [
    ["stty", "115200", "raw", "-echo", "clocal", "cs8", "-parenb", "-cstopb"],      # POSIX / Linux / macOS
    ["stty", "baud=115200", "bits=8", "par=none", "stopb=1", "-edit", "-echo",       # QNX style
     "-ihflow", "-ohflow", "-isflow", "-osflow"],
]


def _stty_raw(path: str) -> bool:
    for argv in STTY_VARIANTS:
        try:
            with open(path, "rb", buffering=0) as port:
                rc = subprocess.run(argv, stdin=port, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, timeout=3).returncode
            if rc == 0:
                log("[uno] configured with: %s" % " ".join(argv))
                return True
        except Exception:
            continue
    return False


def configure_serial(fd: int, path: str) -> None:
    if termios is not None:
        try:
            _termios_raw(fd)
            return
        except Exception as e:  # noqa: BLE001
            log("[uno] termios setup failed (%s), trying stty" % e)
    if not _stty_raw(path):
        log("[uno] WARNING: could not set 115200 raw on %s; set it by hand, e.g. "
            "`stty baud=115200 -edit < %s` (QNX) and restart" % (path, path))


# ------------------------------------------------------------------ uno backend (motor_uno_serial.c)
class UnoBackend:
    name = "uno"

    def __init__(self, dev):
        self.dev = dev                 # None = auto-detect
        self.path = dev or ""
        self.fd = -1
        self.out = b""
        self.inbuf = b""
        self.in_overflow = False
        self.evq: list[Ev] = []
        self.last_open_try = 0
        self.dropped = 0
        self.waiting_logged = 0

    def describe_dev(self) -> str:
        return self.dev or ("auto(%s)" % (self.path or "none yet"))

    def _find(self):
        if self.dev:
            return self.dev
        for pat in AUTO_PATTERNS:
            hits = sorted(glob.glob(pat))
            if hits:
                return hits[0]
        return None

    def _open(self) -> int:
        self.last_open_try = now_ms()
        path = self._find()
        if path is None:
            if not self.waiting_logged:
                log("[uno] no Uno serial port yet (looked for %s), retrying every %d ms"
                    % (" ".join(AUTO_PATTERNS), REOPEN_MS))
            self.waiting_logged += 1
            return -1
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            if not self.waiting_logged:
                log("[uno] %s not there yet (%s), retrying every %d ms" % (path, e.strerror, REOPEN_MS))
            self.waiting_logged += 1
            return -1
        self.waiting_logged = 0
        self.path = path
        self.fd = fd
        configure_serial(fd, path)
        self.out = b""
        self.inbuf = b""
        self.in_overflow = False
        # Opening the Uno's USB port pulses DTR, which reboots it: READY ~1.5 s later.
        log("[uno] opened %s @115200" % path)
        return 0

    def _lost(self, why: str) -> None:
        log("[uno] lost %s (%s), retrying every %d ms" % (self.path, why, REOPEN_MS))
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.fd = -1

    def _push(self, ev: Ev) -> None:
        if len(self.evq) < 15:
            self.evq.append(ev)

    def _parse(self, s: str) -> None:
        if s.startswith("#"):
            log("[uno] %s" % s)
            return
        if s == "READY":
            self._push(Ev(Ev.READY))
        elif s == "BTN TEACH":
            self._push(Ev(Ev.BTN, Ev.BTN_TEACH))
        elif s == "BTN FORGET":
            self._push(Ev(Ev.BTN, Ev.BTN_FORGET))
        elif s.startswith("DONE "):
            m = re.fullmatch(r"DONE ([MR]) +(\S+)", s)
            if m:
                try:
                    self._push(Ev(Ev.DONE, what=m.group(1), amount=float(m.group(2))))
                except ValueError:
                    pass
        elif s.startswith("ABORT "):
            m = re.fullmatch(r"ABORT ([MR]) +(\S+) +(\S{1,15})", s)
            if m:
                try:
                    self._push(Ev(Ev.ABORT, what=m.group(1), amount=float(m.group(2)), why=m.group(3)))
                except ValueError:
                    pass
        elif len(s) >= 2 and s[0] in "TB" and s[1] == " ":
            try:
                v = int(s[2:])
            except ValueError:
                return
            self._push(Ev(Ev.TIP if s[0] == "T" else Ev.BUMP, v))

    def _read(self) -> None:
        while self.fd >= 0:
            try:
                data = os.read(self.fd, 128)
            except BlockingIOError:
                return
            except InterruptedError:
                continue
            except OSError as e:
                self._lost(e.strerror or str(e))
                return
            if not data:
                self._lost("hang-up")
                return
            for c in data:
                if c in (10, 13):
                    if not self.in_overflow and self.inbuf:
                        self._parse(self.inbuf.decode("ascii", "replace"))
                    self.inbuf = b""
                    self.in_overflow = False
                elif len(self.inbuf) >= IN_CAP - 1:
                    self.in_overflow = True
                elif not self.in_overflow:
                    self.inbuf += bytes((c,))

    def _flush(self) -> None:
        while self.fd >= 0 and self.out:
            try:
                w = os.write(self.fd, self.out)
            except BlockingIOError:
                return
            except InterruptedError:
                continue
            except OSError as e:
                self._lost(e.strerror or str(e))
                return
            if w == 0:
                return
            self.out = self.out[w:]

    def _send(self, line: str) -> int:
        if self.fd < 0:
            if now_ms() - self.last_open_try >= REOPEN_MS:
                self._open()
            if self.fd < 0:
                return -1
        b = line.encode("ascii")
        if len(self.out) + len(b) > OUT_CAP:
            if self.dropped % 100 == 0:
                log("[uno] TX buffer full, dropping lines")
            self.dropped += 1
            return -1
        self.out += b
        self._flush()
        return 0

    def init(self) -> int:
        if self._open() == 0:
            self._send("S\n")
        return 0          # a missing Uno is not fatal: retried every second

    def drive(self, l: float, r: float) -> None:
        self._send("D %.3f %.3f\n" % (l, r))
        if self.fd >= 0:
            self._read()

    def arm(self, i: int, deg: float, dps: float) -> None:
        self._send("A %d %.1f %.0f\n" % (i, deg, dps))

    def move(self, kind: str, amount: float, speed: float) -> None:
        self._send("%s %.1f %.2f\n" % (kind, amount, speed))

    def wheel_cal(self, d: float, t: float) -> None:
        self._send("K %.1f %.1f\n" % (d, t))

    def led(self, p: int) -> None:
        self._send("L %d\n" % p)

    def beep(self, n: int) -> None:
        self._send("B %d\n" % n)

    def move_abort(self) -> None:
        self._send("S\n")

    def arm_halt(self) -> None:
        self._send("X\n")

    def arm_zero(self) -> None:
        self._send("Z\n")

    def level(self) -> None:
        self._send("U\n")

    def poll(self):
        return self.evq.pop(0) if self.evq else None

    def shutdown(self) -> None:
        if self.fd < 0:
            return
        self.out = b""                 # stop wins over anything still queued
        self._send("S\n")
        for _ in range(20):
            if self.fd < 0 or not self.out:
                break
            time.sleep(0.01)
            self._flush()
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


class PicoBackend:
    """Not ported: the Pico build needs the C tars_motor. Kept so -b pico fails clearly."""
    name = "pico"

    def __init__(self, dev):
        self.dev = dev or "/dev/ser1"

    def describe_dev(self) -> str:
        return self.dev

    def init(self) -> int:
        try:
            os.close(os.open(self.dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK))
            log("[pico] open %s: ok, but the Pico backend isn't in tars_bridge.py; use the C tars_motor"
                % self.dev)
        except OSError as e:
            log("[pico] open %s: %s" % (self.dev, e.strerror))
        return -1


# ------------------------------------------------------------------ the server (main.c)
MV_NAMES = ("idle", "running", "done", "aborted")
MV_IDLE, MV_RUNNING, MV_DONE, MV_ABORTED = range(4)
LED_NAMES = {"off": 0, "teach": 2, "rec": 1, "ok": 1, "no": 3, "forget": 3}


def _f(tok: str):
    """C %f: a float, or None. nan/inf allowed through (callers check nan)."""
    try:
        return float(tok)
    except ValueError:
        return None


class Server:
    def __init__(self, motor, wd_ms: int, bump_ms: int):
        self.motor = motor
        self.wd_ms = wd_ms
        self.bump_ms = bump_ms
        self.lock = threading.RLock()
        self.running = True
        # shared state
        self.target_l = self.target_r = 0.0
        self.out_l = self.out_r = 0.0
        self.last_dist = -1.0
        self.estop = 0
        self.estop_why = "none"
        self.last_cmd = time.monotonic() - 10.0      # start in watchdog-stop state
        self.arm_req = [None, None]
        self.req_halt = self.req_zero = self.req_level = self.req_stop_gesture = self.req_reset = False
        self.req_hard_stop = False
        self.stop_rx_ms = 0
        self.stop_ms = -1
        self.vis_mode, self.vis_see = "-", "-"
        self.vis_fps, self.vis_cam_ms, self.vis_stop_ms, self.vis_at_ms = 0.0, -1, -1, 0
        self.req_gesture = None
        self.simq: list[Ev] = []
        self.arm_pos = [0.0, 0.0]
        self.arm_tgt = [0.0, 0.0]
        self.gesture_name = "none"
        self.tipped = 0
        self.tilt_deg = 0
        self.bumps = 0
        self.last_tip_ms = 0
        self.bump_until_ms = 0
        self.ready_count = 0
        self.mv_state = MV_IDLE
        self.mv_id = self.mv_est_ms = 0
        self.mv_kind = ""
        self.mv_req = self.mv_done = self.mv_speed = 0.0
        self.mv_why = "-"
        self.mv_deadline_ms = 0
        self.req_move = False
        self.req_move_abort = None
        self.wheel_d, self.wheel_t = WHEEL_DIAM_MM, WHEEL_TRACK_MM
        self.req_wheelcal = False
        self.led_pattern, self.req_led, self.req_beep = -1, -1, -1
        self.btn_teach = self.btn_forget = 0
        self.uno_pending = 0
        # control-thread only
        self.ctl_pos = [0.0, 0.0]
        self.ctl_tgt = [0.0, 0.0]
        self.ctl_dps = [ARM_DEFAULT_DPS, ARM_DEFAULT_DPS]
        self.ctl_g = None
        self.ctl_g_idx = 0
        self.ctl_g_next = 0

    # ---------------------------------------------------------------- control
    def ctl_arm(self, i: int, deg: float, dps: float) -> None:
        deg = clampf(deg, -ARM_LIMIT_DEG, ARM_LIMIT_DEG)
        dps = clampf(dps, 1.0, ARM_MAX_DPS)
        self.ctl_tgt[i] = deg
        self.ctl_dps[i] = dps
        self.motor.arm(i, deg, dps)

    def ctl_halt_arms(self) -> None:
        self.ctl_tgt[0], self.ctl_tgt[1] = self.ctl_pos[0], self.ctl_pos[1]
        self.motor.arm_halt()

    def ctl_gesture_stop(self, why: str) -> None:
        if not self.ctl_g:
            return
        log("[ctl] gesture %s %s" % (self.ctl_g[0], why))
        self.ctl_g = None

    def ctl_gesture_start(self, g) -> None:
        self.ctl_gesture_stop("cancelled")
        self.ctl_g, self.ctl_g_idx, self.ctl_g_next = g, -1, 0
        log("[ctl] gesture %s start (~%d ms)" % (g[0], gesture_duration_ms(g, self.ctl_pos)))

    def ctl_gesture_tick(self, now: int) -> None:
        if not self.ctl_g or now < self.ctl_g_next:
            return
        self.ctl_g_idx += 1
        if self.ctl_g_idx >= len(self.ctl_g[1]):
            self.ctl_gesture_stop("done")
            return
        deg, dps, hold = self.ctl_g[1][self.ctl_g_idx]
        travel = 0.0
        for a in range(2):
            if deg[a] >= ARM_KEEP:
                continue
            travel = max(travel, abs(deg[a] - self.ctl_pos[a]) / clampf(dps, 1.0, ARM_MAX_DPS))
            self.ctl_arm(a, deg[a], dps)
        self.ctl_g_next = now + int(travel * 1000.0 * FRAME_MARGIN) + FRAME_PAD_MS + hold

    def ctl_arm_model(self, dt: float) -> None:
        for i in range(2):
            step, d = self.ctl_dps[i] * dt, self.ctl_tgt[i] - self.ctl_pos[i]
            self.ctl_pos[i] = self.ctl_tgt[i] if abs(d) <= step else self.ctl_pos[i] + (step if d > 0 else -step)

    def ctl_move_abort(self, why: str) -> None:
        if self.mv_state != MV_RUNNING:
            return
        self.mv_state = MV_ABORTED
        self.mv_why = why
        self.motor.move_abort()
        log("[ctl] move %d abort (%s)" % (self.mv_id, why))

    def ctl_event(self, ev: Ev, now: int) -> None:
        if ev.kind == Ev.TIP:
            was = self.tipped
            self.tipped = 1
            self.tilt_deg = ev.value
            self.last_tip_ms = now
            self.target_l = self.target_r = 0.0
            self.ctl_move_abort("tipped")
            if not was:
                log("[ctl] TIPPED (%d deg) -> stop, latched until RESET" % ev.value)
                self.ctl_gesture_stop("cancelled (tipped)")
                self.ctl_halt_arms()
        elif ev.kind == Ev.BUMP:
            self.bumps += 1
            self.bump_until_ms = now + self.bump_ms
            self.target_l = self.target_r = 0.0
            self.ctl_move_abort("bump")
            log("[ctl] BUMP (%d mg) -> stop, drive refused for %d ms" % (ev.value, self.bump_ms))
        elif ev.kind == Ev.READY:
            self.ready_count += 1
            if self.mv_state == MV_RUNNING:            # a rebooted Uno forgot the move
                self.mv_state = MV_ABORTED
                self.mv_why = "reboot"
            self.uno_pending = 0
            if self.led_pattern >= 0:
                self.motor.led(self.led_pattern)       # restore the LED it forgot
            if self.ready_count > 1:
                log("[ctl] co-processor READY again (it rebooted): arm angles reset to 0, "
                    "re-zero the arms by hand if they were not at 0")
            else:
                log("[ctl] co-processor READY")
            self.ctl_gesture_stop("cancelled (co-processor reboot)")
            self.ctl_pos = [0.0, 0.0]
            self.ctl_tgt = [0.0, 0.0]
        elif ev.kind in (Ev.DONE, Ev.ABORT):
            ok = ev.kind == Ev.DONE
            word = "DONE" if ok else "ABORT"
            if self.uno_pending <= 0:
                log("[ctl] stray %s %s %.1f ignored" % (word, ev.what, ev.amount))
                return
            self.uno_pending -= 1
            if self.uno_pending > 0:
                log("[ctl] %s %s %.1f (older move)" % (word, ev.what, ev.amount))
                return
            self.mv_state = MV_DONE if ok else MV_ABORTED
            self.mv_done = ev.amount
            if ok:
                self.mv_why = "-"
            elif self.mv_why == "-":
                self.mv_why = ev.why or "?"
            log("[ctl] move %d %s: %s %.1f %s" % (self.mv_id, "done" if ok else "aborted", ev.what,
                                                  ev.amount, "" if ok else ev.why))
        elif ev.kind == Ev.BTN:
            if ev.value == Ev.BTN_TEACH:
                self.btn_teach += 1
            elif ev.value == Ev.BTN_FORGET:
                self.btn_forget += 1
            log("[ctl] button %s" % ("TEACH" if ev.value == Ev.BTN_TEACH else "FORGET"))

    def ms_since_cmd(self) -> int:
        return int((time.monotonic() - self.last_cmd) * 1000)

    def tick(self) -> None:
        now = now_ms()
        m = self.motor
        with self.lock:
            # 1. events: from the co-processor, then injected ones (SIM)
            while True:
                ev = m.poll()
                if ev is None:
                    break
                self.ctl_event(ev, now)
            sims, self.simq = self.simq, []
            for ev in sims:
                self.ctl_event(ev, now)

            # 2. client requests
            dist = -1.0                         # no obstacle sensor yet (obstacle.c)
            areq, self.arm_req = self.arm_req, [None, None]
            halt, zero, level = self.req_halt, self.req_zero, self.req_level
            stop_g, reset, hard = self.req_stop_gesture, self.req_reset, self.req_hard_stop
            hard_rx = self.stop_rx_ms
            start_g = self.req_gesture
            self.req_halt = self.req_zero = self.req_level = self.req_stop_gesture = self.req_reset = False
            self.req_hard_stop = False
            self.req_gesture = None

            if self.req_move_abort:
                self.ctl_move_abort(self.req_move_abort)
                self.req_move_abort = None
            if self.req_move:
                m.move(self.mv_kind, self.mv_req, self.mv_speed)
                self.uno_pending += 1
                self.mv_deadline_ms = now + 2 * self.mv_est_ms + 2000
                log("[ctl] move %d: %s %.1f speed %.2f (~%d ms)" % (
                    self.mv_id, "MOVE" if self.mv_kind == "M" else "TURN", self.mv_req, self.mv_speed,
                    self.mv_est_ms))
            self.req_move = False
            if self.mv_state == MV_RUNNING and now > self.mv_deadline_ms:
                self.ctl_move_abort("timeout")      # device never answered
                self.uno_pending = 0
            moving = self.mv_state == MV_RUNNING
            if self.req_wheelcal:
                m.wheel_cal(self.wheel_d, self.wheel_t)
            self.req_wheelcal = False
            led, beep = self.req_led, self.req_beep
            self.req_led = self.req_beep = -1

            # 3. wheels: watchdog, obstacle, tip latch, bump hold
            tl, tr = self.target_l, self.target_r
            wd = self.ms_since_cmd() > self.wd_ms
            if wd:
                tl = tr = 0.0
                self.target_l = self.target_r = 0.0
            blocked = 0 <= dist < OBSTACLE_STOP_M and (tl + tr) > 0
            if moving and 0 <= dist < OBSTACLE_STOP_M and self.mv_kind == "M" and self.mv_req > 0:
                self.ctl_move_abort("obstacle")
            bumped = now < self.bump_until_ms
            if blocked or self.tipped or bumped:
                tl = tr = 0.0
            self.estop = int(bool(wd or blocked or self.tipped or bumped))
            was_why = self.estop_why
            self.estop_why = ("tipped" if self.tipped else "bump" if bumped else "obstacle" if blocked
                              else "watchdog" if wd else "none")
            self.last_dist = dist
            if self.estop or hard:
                self.out_l, self.out_r = tl, tr
            else:
                self.out_l = self.out_l + clampf(tl - self.out_l, -SLEW_PER_TICK, SLEW_PER_TICK)
                self.out_r = self.out_r + clampf(tr - self.out_r, -SLEW_PER_TICK, SLEW_PER_TICK)
            l, r = self.out_l, self.out_r
            if self.estop_why != was_why and self.estop_why != "none":
                log("[ctl] STOP (%s)" % self.estop_why)

            # 4. arms + gestures
            if reset:
                log("[ctl] RESET: tip latch cleared")
            if stop_g:
                self.ctl_gesture_stop("cancelled")
            if halt:
                self.ctl_halt_arms()
            if zero:
                self.ctl_gesture_stop("cancelled")
                self.ctl_pos = [0.0, 0.0]
                self.ctl_tgt = [0.0, 0.0]
                m.arm_zero()
            if level:
                m.level()
            if not self.tipped:
                for i in range(2):
                    if areq[i] is not None:
                        self.ctl_arm(i, areq[i][0], areq[i][1])
                if start_g:
                    self.ctl_gesture_start(start_g)
                self.ctl_gesture_tick(now)
            self.ctl_arm_model(1.0 / TICK_HZ)

            # during a move D 0 0 is a keepalive; a non-zero D aborts it on the device
            m.drive(l, r)
            if led >= 0:
                m.led(led)
            if beep >= 0:
                m.beep(beep)
            if hard:
                self.stop_ms = now_ms() - hard_rx
            self.arm_pos = list(self.ctl_pos)
            self.arm_tgt = list(self.ctl_tgt)
            self.gesture_name = self.ctl_g[0] if self.ctl_g else "none"

    def control_loop(self) -> None:
        period = 1.0 / TICK_HZ
        nxt = time.monotonic()
        while self.running:
            nxt += period
            try:
                self.tick()
            except Exception as e:  # noqa: BLE001  never let the loop die silently
                log("[ctl] tick error: %r" % e)
            delay = nxt - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.5:
                nxt = time.monotonic()          # fell far behind: don't burst
        with self.lock:
            if self.mv_state == MV_RUNNING:
                self.motor.move_abort()
            self.motor.drive(0.0, 0.0)
            self.motor.arm_halt()

    # ---------------------------------------------------------------- network
    def feed(self) -> None:
        self.last_cmd = time.monotonic()

    def refused(self):
        return "tipped over: stand TARS up, then send RESET" if self.tipped else None

    def handle_line(self, line: str) -> str:
        toks = line.split()
        if not toks:
            return ""
        cmd = toks[0][:15].upper()
        args = toks[1:]
        with self.lock:
            return self._handle(cmd, args, line)

    def _handle(self, cmd: str, args: list, line: str) -> str:
        if cmd == "D":
            a = _f(args[0]) if len(args) > 0 else None
            b = _f(args[1]) if len(args) > 1 else None
            if a is None or b is None or math.isnan(a) or math.isnan(b):
                return "ERR usage: D <left> <right>\n"
            self.feed()
            left = self.bump_until_ms - now_ms()
            why = self.refused()
            if why:
                self.target_l = self.target_r = 0.0
                return "ERR %s\n" % why
            if left > 0:
                self.target_l = self.target_r = 0.0
                return "ERR bumped: driving paused %d ms\n" % left
            self.target_l, self.target_r = clampf(a, -1, 1), clampf(b, -1, 1)
            if self.mv_state == MV_RUNNING and (abs(self.target_l) >= 0.03 or abs(self.target_r) >= 0.03):
                self.mv_state = MV_ABORTED          # the device aborts itself on the non-zero D
                self.mv_why = "drive"
            return "OK\n"

        if cmd == "S":
            self.target_l = self.target_r = 0.0
            self.req_stop_gesture = self.req_halt = True
            self.req_hard_stop = True
            self.stop_rx_ms = now_ms()
            if self.mv_state == MV_RUNNING:
                self.req_move_abort = "stop"
            self.req_gesture = None
            self.arm_req = [None, None]
            self.feed()
            return "OK\n"

        if cmd == "G":
            if not args:
                return "ERR usage: G <name>\n"
            name = args[0][:47]
            g = gesture_find(name)
            self.feed()
            why = self.refused()
            if why:
                return "ERR %s\n" % why
            if not g:
                return "ERR unknown gesture '%s' (try GESTURES)\n" % name
            self.req_gesture = g
            self.arm_req = [None, None]
            return "OK %d\n" % gesture_duration_ms(g, self.arm_pos)

        if cmd == "A":
            arm = None
            if args:
                try:
                    arm = int(args[0])
                except ValueError:
                    arm = None
            deg = _f(args[1]) if len(args) > 1 else None
            dps = _f(args[2]) if len(args) > 2 else None
            if (arm not in (0, 1) or deg is None or math.isnan(deg)
                    or (len(args) > 2 and (dps is None or not dps > 0))):
                return "ERR usage: A <0|1> <deg> [dps>0]\n"
            deg = clampf(deg, -ARM_LIMIT_DEG, ARM_LIMIT_DEG)
            dps = clampf(dps, 1.0, ARM_MAX_DPS) if dps is not None else ARM_DEFAULT_DPS
            self.feed()
            why = self.refused()
            if why:
                return "ERR %s\n" % why
            self.req_stop_gesture = True
            self.req_gesture = None
            self.arm_req[arm] = (deg, dps)
            return "OK %.1f\n" % deg

        if cmd == "HOME":
            self.feed()
            why = self.refused()
            if why:
                return "ERR %s\n" % why
            self.req_stop_gesture = True
            self.req_gesture = None
            self.arm_req = [(0.0, ARM_DEFAULT_DPS), (0.0, ARM_DEFAULT_DPS)]
            return "OK\n"

        if cmd in ("ZERO", "LEVEL"):
            if cmd == "ZERO":
                self.req_zero = True
                self.req_gesture = None
            else:
                self.req_level = True
            return "OK\n"

        if cmd == "RESET":
            ago = now_ms() - self.last_tip_ms
            if self.tipped and ago < TIP_CLEAR_MS:
                return "ERR still tipped (last report %d ms ago)\n" % ago
            if self.tipped:
                self.req_reset = True
            self.tipped = 0
            self.bump_until_ms = 0
            self.target_l = self.target_r = 0.0
            return "OK\n"

        if cmd in ("MOVE", "TURN"):
            kind = "M" if cmd == "MOVE" else "R"
            a = _f(args[0]) if args else None
            c = _f(args[1]) if len(args) > 1 else None
            n = 0 if a is None else (2 if c is not None else 1)
            if n < 1 or math.isnan(a) or (n == 2 and not (0 < c <= 1)):
                return "ERR usage: %s <%s> [speed 0-1]\n" % (cmd, "mm" if kind == "M" else "deg")
            lim = MOVE_MAX_MM if kind == "M" else TURN_MAX_DEG
            amt, spd = clampf(a, -lim, lim), (c if n == 2 else 1.0)
            self.feed()
            left = self.bump_until_ms - now_ms()
            why = self.refused()
            if why:
                return "ERR %s\n" % why
            if left > 0:
                return "ERR bumped: driving paused %d ms\n" % left
            self.target_l = self.target_r = 0.0       # the D stream becomes keepalives
            self.mv_id += 1
            self.mv_state = MV_RUNNING
            self.mv_kind, self.mv_req, self.mv_speed, self.mv_done = kind, amt, spd, 0.0
            self.mv_why = "-"
            self.mv_est_ms = wheel_move_ms(wheel_move_steps(kind, amt, self.wheel_d, self.wheel_t), spd)
            self.req_move = True
            self.req_move_abort = None                # the device replaces a running move itself
            return "OK %d %d\n" % (self.mv_id, self.mv_est_ms)

        if cmd == "WHEELCAL":
            vals = []
            for t in args[:2]:
                v = _f(t)
                if v is None:
                    break
                vals.append(v)
            if len(vals) == 2 and 10 <= vals[0] <= 300 and 20 <= vals[1] <= 500:
                self.wheel_d, self.wheel_t = vals
                self.req_wheelcal = True
            elif vals:
                return "ERR usage: WHEELCAL [diam_mm 10-300 track_mm 20-500]\n"
            return "OK %.1f %.1f\n" % (self.wheel_d, self.wheel_t)

        if cmd in ("LED", "BEEP"):
            led = cmd == "LED"
            v = -1
            if len(args) == 1:
                w = args[0]
                if led and not w[0].isdigit():
                    v = LED_NAMES.get(w.lower()[:15], -1)
                else:
                    try:
                        v = int(w)
                    except ValueError:
                        v = -1
            if v < 0 or v > (3 if led else 9):
                return ("ERR usage: LED <off|teach|rec|ok|no|forget|0-3>\n" if led
                        else "ERR usage: BEEP <0-9>\n")
            if led:
                self.led_pattern = self.req_led = v
            else:
                self.req_beep = v
            return "OK\n"

        if cmd == "GESTURES":
            return "OK " + " ".join(g[0] for g in GESTURES) + "\n"

        if cmd == "PING":
            self.feed()
            return "OK PONG\n"

        if cmd == "STATUS":
            n = now_ms()
            bl = self.bump_until_ms - n
            return ("OK %.2f %.2f %d %.2f %d why=%s arm0=%.1f arm1=%.1f arm0_tgt=%.1f arm1_tgt=%.1f "
                    "gesture=%s tipped=%d tilt=%d bump_ms=%d bumps=%d ready=%d backend=%s "
                    "stop_ms=%d vis=%s vis_fps=%.1f vis_cam_ms=%d vis_stop_ms=%d vis_see=%s "
                    "vis_age_ms=%d move=%s move_id=%d move_kind=%s move_req=%.1f move_done=%.1f "
                    "move_why=%s move_est_ms=%d wheel_d=%.1f wheel_t=%.1f btn_teach=%d btn_forget=%d "
                    "led=%d\n") % (
                self.out_l, self.out_r, self.ms_since_cmd(), self.last_dist, self.estop, self.estop_why,
                self.arm_pos[0], self.arm_pos[1], self.arm_tgt[0], self.arm_tgt[1], self.gesture_name,
                self.tipped, self.tilt_deg, bl if bl > 0 else 0, self.bumps, self.ready_count,
                self.motor.name, self.stop_ms, self.vis_mode, self.vis_fps, self.vis_cam_ms,
                self.vis_stop_ms, self.vis_see, (n - self.vis_at_ms) if self.vis_at_ms else -1,
                MV_NAMES[self.mv_state], self.mv_id,
                "MOVE" if self.mv_kind == "M" else "TURN" if self.mv_kind == "R" else "-",
                self.mv_req, self.mv_done, self.mv_why, self.mv_est_ms, self.wheel_d, self.wheel_t,
                self.btn_teach, self.btn_forget, self.led_pattern)

        if cmd == "VIS":
            if len(args) < 5:
                return "ERR usage: VIS <mode> <fps> <cam_ms> <stop_ms> <label>\n"
            try:
                fps, cam, stp = float(args[1]), int(args[2]), int(args[3])
            except ValueError:
                return "ERR usage: VIS <mode> <fps> <cam_ms> <stop_ms> <label>\n"
            self.vis_mode, self.vis_see = args[0][:15], args[4][:23]
            self.vis_fps, self.vis_cam_ms, self.vis_stop_ms = fps, cam, stp
            self.vis_at_ms = now_ms()
            return "OK\n"

        if cmd == "SIM":
            if not isinstance(self.motor, StubBackend):
                return "ERR SIM is only for -b stub\n"
            if not args:
                return "ERR usage: SIM T|B|READY [n]\n"
            kind = args[0].upper()
            try:
                v = int(args[1]) if len(args) > 1 else 0
            except ValueError:
                v = 0
            if kind == "T":
                ev = Ev(Ev.TIP, v)
            elif kind == "B":
                ev = Ev(Ev.BUMP, v)
            elif kind == "READY":
                ev = Ev(Ev.READY)
            elif kind == "BTN":
                b2 = args[1].upper() if len(args) > 1 else ""
                bv = Ev.BTN_TEACH if b2 == "TEACH" else Ev.BTN_FORGET if b2 == "FORGET" else 0
                if not bv:
                    return "ERR usage: SIM BTN TEACH|FORGET\n"
                ev = Ev(Ev.BTN, bv)
            else:
                return "ERR usage: SIM T|B|READY [n] | SIM BTN TEACH|FORGET\n"
            if len(self.simq) < 8:
                self.simq.append(ev)
            return "OK\n"

        return "ERR unknown command '%s'\n" % toks0(line)

    def client_loop(self, conn: socket.socket, peer) -> None:
        log("[net] client connected (%s)" % peer[0])
        buf = b""
        try:
            while True:
                data = conn.recv(512)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    reply = self.handle_line(raw.decode("ascii", "replace").rstrip("\r"))
                    if reply:
                        conn.sendall(reply.encode("ascii", "replace"))
                if len(buf) >= 511:
                    buf = b""                       # line too long: drop it
        except OSError:
            pass
        # a dropped client means stop the wheels now, don't wait for the watchdog
        with self.lock:
            self.target_l = self.target_r = 0.0
            if self.mv_state == MV_RUNNING:
                self.req_move_abort = "disconnect"
        log("[net] client disconnected -> stop")
        try:
            conn.close()
        except OSError:
            pass


def toks0(line: str) -> str:
    t = line.split()
    return t[0][:15] if t else ""


def local_ips():
    """Best-effort list of this machine's LAN addresses (for the startup banner)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))          # no packet is sent
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(i for i in ips if not i.startswith("127."))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Python drop-in for tars_motor (TCP :5555 -> Uno serial).")
    ap.add_argument("-b", dest="backend", default="uno", choices=["uno", "stub", "pico"])
    ap.add_argument("--stub", action="store_true", help="same as -b stub (no hardware)")
    ap.add_argument("-d", dest="dev", default=None, help="serial device (default: auto-detect)")
    ap.add_argument("-p", dest="port", type=int, default=5555)
    ap.add_argument("-w", dest="wd_ms", type=int, default=500, help="watchdog ms")
    ap.add_argument("-r", dest="rt_prio", type=int, default=30, help="ignored (C server only)")
    ap.add_argument("-k", dest="bump_ms", type=int, default=1000, help="bump hold ms")
    args = ap.parse_args(argv)
    backend = "stub" if args.stub else args.backend

    motor = {"uno": lambda: UnoBackend(args.dev), "stub": StubBackend,
             "pico": lambda: PicoBackend(args.dev)}[backend]()
    if motor.init() != 0:
        log("backend init failed")
        return 1

    srv = Server(motor, args.wd_ms, args.bump_ms)

    def on_signal(_sig, _frm):
        srv.running = False
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)   # started from ssh: keep running

    ctl = threading.Thread(target=srv.control_loop, name="control", daemon=True)
    ctl.start()

    lsock = None
    try:
        lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lsock.bind(("0.0.0.0", args.port))
        lsock.listen(4)
        lsock.settimeout(0.2)
        log("tars_motor: backend=%s dev=%s port=%d watchdog=%dms prio=- (python bridge)"
            % (motor.name, motor.describe_dev(), args.port, args.wd_ms))
        log("tars_bridge: BRING-UP/DEBUG ONLY. The robot's real server is the native QNX tars_motor (pi/src).")
        ips = local_ips()
        if ips:
            log("tars_bridge: laptop -> python laptop/teleop.py %s" % ips[0])
    except OSError as e:
        log("socket/bind/listen: %s" % e)
        srv.running = False

    while srv.running:
        try:
            conn, peer = lsock.accept()
        except socket.timeout:
            continue
        except OSError as e:
            if e.errno == errno.EINTR:
                continue
            log("accept: %s" % e)
            break
        conn.settimeout(None)
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=srv.client_loop, args=(conn, peer), daemon=True).start()

    srv.running = False
    ctl.join(2)
    with srv.lock:
        motor.shutdown()
    if lsock:
        lsock.close()
    log("tars_motor: bye")
    return 0


if __name__ == "__main__":
    sys.exit(main())
