"""TCP client for the tars_motor server on the Pi.

    from tars_client import Tars
    with Tars("tars.local") as t:
        t.drive(0.5, 0.5)     # must be refreshed < 500 ms or the watchdog stops
        t.gesture("wave")
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


class TarsError(RuntimeError):
    pass


class Tars:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: float = 2.0):
        self.addr = (host, port)
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._file = None
        self._lock = threading.Lock()
        self._hold: tuple[float, float] | None = None
        self._hold_thread: threading.Thread | None = None

    # -- connection -------------------------------------------------------
    def connect(self) -> "Tars":
        self._sock = socket.create_connection(self.addr, timeout=self.timeout)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._file = self._sock.makefile("rw", encoding="ascii", newline="\n")
        return self

    def close(self) -> None:
        self._hold = None
        if self._sock:
            try:
                self.stop()
            except Exception:
                pass
            self._sock.close()
        self._sock = None

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
            raise TarsError(reply)
        return reply[2:].strip()

    # -- commands ---------------------------------------------------------
    def drive(self, left: float, right: float) -> None:
        self.send(f"D {left:.3f} {right:.3f}")

    def stop(self) -> None:
        self._hold = None
        self.send("S")

    def gesture(self, name: str) -> None:
        self.send(f"G {name}")

    def ping(self) -> float:
        t0 = time.monotonic()
        self.send("PING")
        return (time.monotonic() - t0) * 1000.0

    def status(self) -> dict:
        l, r, age, dist, estop = self.send("STATUS").split()
        return {"left": float(l), "right": float(r), "ms_since_cmd": int(age),
                "dist_m": float(dist), "estop": estop == "1"}

    # -- convenience: timed moves (feeds the watchdog for you) -------------
    def move_for(self, left: float, right: float, seconds: float, hz: float = 10) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.drive(left, right)
            time.sleep(1.0 / hz)
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
