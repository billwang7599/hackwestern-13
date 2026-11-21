"""TCP client for the tars_motor server on the Pi.

    from tars_client import Tars
    with Tars("tars.local") as t:
        t.drive(0.5, 0.5)     # must be refreshed < 500 ms or the watchdog stops
        t.gesture("wave")     # wave point shrug nod stance fold
        t.arm(0, 45)          # left arm swings 45 deg forward
        t.home()
        t.stop()

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
        """Clear the tip-over latch once TARS is standing again."""
        self.send("RESET")

    def gestures(self) -> list[str]:
        return self.send("GESTURES").split()

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
