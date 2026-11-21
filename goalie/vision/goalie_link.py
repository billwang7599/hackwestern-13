"""TCP client for the goalie_motor server on the Pi (protocol: docs/architecture.md).

    from goalie_link import GoalieLink
    link = GoalieLink("goalie.local")
    target_x, duty = link.frame(t_ms, ball=(bx, by), paddle=px)   # every camera frame
    link.mode("LINUX"); link.status()["jit_max_us"]

Built for a camera loop: it never blocks for long.
  - short connect / reply timeouts (default 0.3 s / 0.15 s)
  - when the server is down, calls fail fast with LinkDown and the link
    retries in the background of your calls with exponential backoff
    (0.25 s .. 2 s); it does not sleep.
  - a reply timeout drops the connection (so replies can never get out of
    step with commands) and the next call reconnects.

Set GOALIE_HOST / GOALIE_PORT to change the default address.
"""
from __future__ import annotations

import os
import socket
import threading
import time

from field import DEFAULT_PORT as _PORT

DEFAULT_HOST = os.environ.get("GOALIE_HOST", "goalie.local")
DEFAULT_PORT = int(os.environ.get("GOALIE_PORT", str(_PORT)))


class GoalieError(RuntimeError):
    """Server replied ERR ..."""


class LinkDown(ConnectionError):
    """Not connected (server down, timed out, or waiting for the reconnect backoff)."""


def _num(v: str):
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


def parse_status(reply: str) -> dict:
    """'mode=QNX saves=3 loop_hz=199.8 foo' -> {'mode': 'QNX', 'saves': 3, 'loop_hz': 199.8}.
    Tokens without '=' and unknown keys are kept/ignored harmlessly."""
    out = {}
    for tok in reply.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = _num(v)
    return out


class GoalieLink:
    BACKOFF_MIN = 0.25
    BACKOFF_MAX = 2.0

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT,
                 timeout: float = 0.15, connect_timeout: float = 0.3):
        self.addr = (host, port)
        self.timeout = timeout
        self.connect_timeout = connect_timeout
        self._sock: socket.socket | None = None
        self._buf = b""
        self._lock = threading.Lock()
        self._backoff = self.BACKOFF_MIN
        self._next_try = 0.0
        self.reconnects = 0          # successful (re)connections, for the UI/tests
        self.last_error = ""

    # -- connection -------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._sock is not None

    def _connect(self) -> None:
        now = time.monotonic()
        if now < self._next_try:
            raise LinkDown(f"reconnect in {self._next_try - now:.2f}s ({self.last_error})")
        try:
            s = socket.create_connection(self.addr, timeout=self.connect_timeout)
        except OSError as e:
            self.last_error = str(e) or type(e).__name__
            self._next_try = now + self._backoff
            self._backoff = min(self._backoff * 2, self.BACKOFF_MAX)
            raise LinkDown(self.last_error) from None
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        self._sock, self._buf = s, b""
        self._backoff = self.BACKOFF_MIN
        self.reconnects += 1

    def _drop(self, why: str) -> None:
        self.last_error = why
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._next_try = time.monotonic() + self._backoff

    def close(self) -> None:
        with self._lock:
            self._drop("closed")

    def __enter__(self) -> "GoalieLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def _readline(self) -> bytes:
        while b"\n" not in self._buf:
            chunk = self._sock.recv(4096)
            if not chunk:
                raise ConnectionResetError("server closed connection")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line

    # -- raw protocol -----------------------------------------------------
    def send(self, line: str) -> str:
        """Send one command line, return the reply without the leading 'OK'.
        Raises GoalieError on ERR, LinkDown when not connected / timed out."""
        with self._lock:
            if self._sock is None:
                self._connect()
            try:
                self._sock.sendall((line.strip() + "\n").encode("ascii"))
                reply = self._readline().decode("ascii", "replace").strip()
            except (OSError, socket.timeout) as e:
                self._drop(str(e) or type(e).__name__)
                raise LinkDown(self.last_error) from None
        if reply.startswith("ERR"):
            raise GoalieError(reply)
        if not reply.startswith("OK"):
            raise GoalieError(f"bad reply: {reply!r}")
        return reply[2:].strip()

    # -- commands ---------------------------------------------------------
    def frame(self, t_ms: int, ball: tuple[float, float] | None,
              paddle: float | None) -> tuple[float, float]:
        """One camera frame. Returns (target_x, duty) from the Pi."""
        bs, bx, by = (1, ball[0], ball[1]) if ball is not None else (0, 0.0, 0.0)
        ps, px = (1, paddle) if paddle is not None else (0, 0.0)
        r = self.send(f"F {int(t_ms)} {bs} {bx:.4f} {by:.4f} {ps} {px:.4f}").split()
        try:
            return float(r[0]), float(r[1])
        except (IndexError, ValueError):
            raise GoalieError(f"bad F reply: {r!r}") from None

    def mode(self, m: str) -> str:
        return self.send(f"MODE {m.upper()}")

    def human(self, x: float) -> None:
        self.send(f"HUMAN {x:.4f}")

    def auto(self) -> None:
        self.send("AUTO")

    def goal(self) -> None:
        self.send("GOAL")

    def reset(self) -> None:
        self.send("RESET")

    def ping(self) -> float:
        t0 = time.monotonic()
        self.send("PING")
        return (time.monotonic() - t0) * 1000.0

    def status(self) -> dict:
        return parse_status(self.send("STATUS"))


class DryLink:
    """--no-pi stand-in: prints what would be sent, answers like a parked goalie."""

    connected = True
    reconnects = 0
    last_error = ""

    def __init__(self, verbose_every: int = 30):
        self.n = 0
        self.every = max(1, verbose_every)
        self.state = {"mode": "QNX", "ctl": "AUTO", "saves": 0, "goals": 0, "wd": 0,
                      "jit_avg_us": 0, "jit_max_us": 0, "loop_hz": 0.0}

    def send(self, line: str) -> str:
        self.n += 1
        if not line.startswith("F") or self.n % self.every == 0:
            print(f"[no-pi] -> {line}")
        return "PONG" if line == "PING" else ""

    def frame(self, t_ms, ball, paddle):
        bs, bx, by = (1, *ball) if ball is not None else (0, 0.0, 0.0)
        ps, px = (1, paddle) if paddle is not None else (0, 0.0)
        self.send(f"F {int(t_ms)} {bs} {bx:.4f} {by:.4f} {ps} {px:.4f}")
        return 0.0, 0.0

    def mode(self, m):
        self.state["mode"] = m.upper()
        self.send(f"MODE {m.upper()}")
        return m.upper()

    def human(self, x):
        self.state["ctl"] = "HUMAN"
        self.send(f"HUMAN {x:.4f}")

    def auto(self):
        self.state["ctl"] = "AUTO"
        self.send("AUTO")

    def goal(self):
        self.state["goals"] += 1
        self.send("GOAL")

    def reset(self):
        self.state.update(saves=0, goals=0)
        self.send("RESET")

    def ping(self):
        return 0.0

    def status(self):
        return dict(self.state)

    def close(self):
        pass


if __name__ == "__main__":
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    link = GoalieLink(host, port, timeout=1.0, connect_timeout=2.0)
    print(f"ping {link.ping():.1f} ms")
    print(link.status())
