"""TCP client for the tars_motor server on the Pi.

    from tars_client import Tars
    with Tars("tars.local") as t:
        t.drive(0.5, 0.5)     # must be refreshed < 500 ms or the watchdog stops
        t.gesture("wave")     # wave point shrug nod stance fold
        t.arm(0, 45)          # left arm swings 45 deg forward
        t.home()
        t.stop()
        t.move_mm(100)        # stepper wheels: exactly 10 cm forward (blocks until done)
        t.turn_deg(30)        # exactly 30 deg to the right (+) / left (-)

Set TARS_HOST / TARS_PORT env vars to change the default address.
"""
from __future__ import annotations

import os
import socket
import threading
import time

DEFAULT_HOST = os.environ.get("TARS_HOST", "tars.local")
DEFAULT_PORT = int(os.environ.get("TARS_PORT", "5555"))

GESTURES = ["wave", "point", "shrug", "nod", "stance", "fold"]
LEFT, RIGHT = 0, 1


class TarsError(RuntimeError):
    pass


class Tars:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: float = 2.0):
        self.addr = (host, port)
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._file = None
        self._lock = threading.Lock()

    # -- connection -------------------------------------------------------
    def connect(self) -> "Tars":
        self._sock = socket.create_connection(self.addr, timeout=self.timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._file = self._sock.makefile("rw", encoding="ascii", newline="\n")
        return self

    def close(self) -> None:
        if self._sock:
            try:
                self.stop()
            except Exception:
                pass
            if self._file:
                self._file.close()
            self._sock.close()
        self._sock = None
        self._file = None

    def __enter__(self) -> "Tars":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()

    # -- raw protocol -----------------------------------------------------
    def send(self, line: str) -> str:
        """Send one command line, return the reply (without 'OK'). Raises on ERR."""
        if not self._file:
            self.connect()
        with self._lock:
            self._file.write(line.strip() + "\n")
            self._file.flush()
            reply = self._file.readline().strip()
        if not reply:
            raise TarsError("connection closed")
        if reply.startswith("ERR"):
            raise TarsError(reply[3:].strip())
        return reply[2:].strip()

    # -- wheels -----------------------------------------------------------
    def drive(self, left: float, right: float) -> None:
        """Raises TarsError while tipped over ('tipped ...') or after a bump ('bumped ...')."""
        self.send(f"D {left:.3f} {right:.3f}")

    def stop(self) -> None:
        """Stop the wheels, cancel any gesture, halt both arms."""
        self.send("S")

    # -- arms + gestures --------------------------------------------------
    def gesture(self, name: str) -> float:
        """Start a gesture (non-blocking). Returns its expected duration in seconds."""
        r = self.send(f"G {name}")
        return int(r) / 1000.0 if r.isdigit() else 0.0

    def gesture_wait(self, name: str) -> None:
        """Start a gesture and block until the Pi reports it finished."""
        end = time.monotonic() + self.gesture(name) + 1.0   # timeout, not the normal exit
        time.sleep(0.1)
        while time.monotonic() < end and self.status().get("gesture") == name:
            time.sleep(0.1)

    def arm(self, which: int, deg: float, dps: float | None = None) -> float:
        """Arm 0=left 1=right to deg (+ = forward, clamped to +-135). Returns the clamped angle."""
        cmd = f"A {which} {deg:.1f}" + (f" {dps:.0f}" if dps else "")
        return float(self.send(cmd))

    def home(self) -> None:
        self.send("HOME")

    def zero(self) -> None:
        """Declare the arms' current position as 0 (after straightening them by hand)."""
        self.send("ZERO")

    def level(self) -> None:
        """Declare the robot's current orientation as upright (tilt reference)."""
        self.send("LEVEL")

    def reset(self) -> None:
        """Clear the tip-over latch once QNXplorer is standing again."""
        self.send("RESET")

    def gestures(self) -> list[str]:
        return self.send("GESTURES").split()

    # -- MOVE / TURN (stepper wheels: exact; DC wheels: timed) -----------------
    def _exact(self, verb: str, amount: float, speed: float | None, wait: bool,
               timeout: float | None) -> dict:
        r = self.send(f"{verb} {amount:.1f}" + (f" {speed:.2f}" if speed else "")).split()
        move_id, est_ms = int(r[0]), int(r[1])
        info = {"id": move_id, "est_s": est_ms / 1000.0, "move": "running"}
        if not wait:
            return info
        end = time.monotonic() + (timeout if timeout is not None else est_ms / 1000.0 * 2 + 3.0)
        time.sleep(min(0.05, est_ms / 1000.0))
        while time.monotonic() < end:
            self.send("PING")          # MOVE/TURN need a keepalive within tars_motor's 500 ms deadman
            st = self.status()
            if int(st.get("move_id", -1)) != move_id:
                info["move"] = "replaced"
                return info
            if st.get("move") != "running":
                info.update(move=st["move"], done=float(st.get("move_done", 0)), why=st.get("move_why"))
                return info
            time.sleep(0.05)
        info["move"] = "timeout"
        return info

    def move_mm(self, mm: float, speed: float | None = None, wait: bool = True,
                timeout: float | None = None) -> dict:
        """Drive exactly mm (+ forward). speed = fraction of max RPM. With wait=True
        blocks until done and returns {'move': 'done'|'aborted'|..., 'done': mm, 'why': ...}.
        Without wait returns at once with {'id', 'est_s'}; S (stop()) aborts it."""
        return self._exact("MOVE", mm, speed, wait, timeout)

    def turn_deg(self, deg: float, speed: float | None = None, wait: bool = True,
                 timeout: float | None = None) -> dict:
        """Spin in place exactly deg: + = right (clockwise from above), - = left."""
        return self._exact("TURN", deg, speed, wait, timeout)

    def wheel_cal(self, diam_mm: float | None = None, track_mm: float | None = None) -> tuple[float, float]:
        """Set (saved in the Uno's EEPROM) or read the wheel diameter and track width."""
        r = self.send("WHEELCAL" + (f" {diam_mm:.1f} {track_mm:.1f}" if diam_mm else "")).split()
        return float(r[0]), float(r[1])

    def speed_cal(self, mm_s: float | None = None, deg_s: float | None = None,
                  deg_s_slow: float | None = None) -> tuple[float, float] | None:
        """DC wheels: read, or set (saved in the Uno's EEPROM), the speeds at D 1 and,
        optionally, the slow-turn rate (TURN speed < 1; None = the Uno keeps its own).
        Returns (mm_s, deg_s), or None for stepper wheels. A set is applied once the Uno
        confirms (WHEELS DC); this waits up to 1 s for that."""
        if mm_s is None:
            r = self.send("SPEEDCAL").split()
            return (float(r[1]), float(r[2])) if r[0] == "dc" else None
        slow = f" {deg_s_slow:.1f}" if deg_s_slow else ""
        self.send(f"SPEEDCAL {mm_s:.1f} {deg_s:.1f}{slow}")
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            cur = self.speed_cal()
            sl = self.dc_slow_rate()
            if (cur and abs(cur[0] - mm_s) < 0.05 and abs(cur[1] - deg_s) < 0.05
                    and (not deg_s_slow or (sl is not None and abs(sl - deg_s_slow) < 0.05))):
                return cur
            time.sleep(0.05)
        raise TarsError("the Uno didn't confirm the new speeds (DC firmware flashed? Bluetooth unplugged?)")

    def dc_slow_rate(self) -> float | None:
        """DC wheels: the slow-turn rate in deg/s (TURN speed < 1, e.g. goto's 0.40 and the
        guard_blocked pivot), or None (stepper wheels, or an Uno too old to report it)."""
        r = self.send("SPEEDCAL").split()
        return float(r[3]) if r[0] == "dc" and len(r) > 3 else None

    # -- on-robot UI ----------------------------------------------------------
    def led(self, pattern: int) -> None:
        """0 off, 1 on, 2 slow blink, 3 fast blink."""
        self.send(f"LED {pattern}")

    def beep(self, n: int = 1) -> None:
        self.send(f"BEEP {n}")

    def buttons(self) -> tuple[int, int]:
        """(teach, forget) press counters; they only count up, compare with the last read."""
        st = self.status()
        return int(st.get("btn_teach", 0)), int(st.get("btn_forget", 0))

    # -- info ---------------------------------------------------------------
    def ping(self) -> float:
        t0 = time.monotonic()
        self.send("PING")
        return (time.monotonic() - t0) * 1000.0

    def status(self) -> dict:
        parts = self.send("STATUS").split()
        l, r, age, dist, estop = parts[:5]
        st: dict = {"left": float(l), "right": float(r), "ms_since_cmd": int(age),
                    "dist_m": float(dist), "estop": estop == "1"}
        for kv in parts[5:]:
            k, _, v = kv.partition("=")
            try:
                st[k] = float(v) if "." in v else int(v)
            except ValueError:
                st[k] = v
        st["tipped"] = bool(st.get("tipped", 0))
        return st

    # -- convenience: timed moves (feeds the watchdog for you) -------------
    def move_for(self, left: float, right: float, seconds: float, hz: float = 10) -> None:
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end:
                self.drive(left, right)
                time.sleep(1.0 / hz)
        finally:
            self.stop()

    def forward(self, seconds: float = 1.0, speed: float = 0.5) -> None:
        self.move_for(speed, speed, seconds)

    def turn(self, seconds: float = 0.6, speed: float = 0.5) -> None:
        """Positive speed = turn right (spin in place)."""
        self.move_for(speed, -speed, seconds)


if __name__ == "__main__":
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    with Tars(host) as t:
        print(f"ping {t.ping():.1f} ms")
        print(t.status())
