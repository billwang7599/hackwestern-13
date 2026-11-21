"""Line-protocol link to tars_motor (TCP :5555), or a dry-run printer.

Thread-safe: the frame loop and the 20 ms tick timer both send. If the server
is unreachable, commands are dropped (the robot's 500 ms watchdog has already
stopped the wheels) and we retry the connection every second.
"""
from __future__ import annotations

import socket
import sys
import threading
import time


class MotorLink:
    def __init__(self, host: str | None, port: int = 5555, verbose: bool = False):
        self.host, self.port = host, port
        self.verbose = verbose
        self.sock: socket.socket | None = None
        self.rfile = None
        self.lock = threading.Lock()
        self.next_try = 0.0
        self.last_rtt_ms = 0.0
        self.errors = 0
        self.last_err = ""

    @property
    def dry(self) -> bool:
        return not self.host

    def _connect(self) -> bool:
        if self.sock:
            return True
        now = time.monotonic()
        if now < self.next_try:
            return False
        self.next_try = now + 1.0
        try:
            s = socket.create_connection((self.host, self.port), timeout=1.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(1.0)
            self.sock, self.rfile = s, s.makefile("r", encoding="ascii", newline="\n")
            print(f"[link] connected to {self.host}:{self.port}", file=sys.stderr, flush=True)
            return True
        except OSError as e:
            print(f"[link] {self.host}:{self.port} unreachable ({e}); retrying",
                  file=sys.stderr, flush=True)
            return False

    def _drop(self, why: str) -> None:
        print(f"[link] lost connection: {why}", file=sys.stderr, flush=True)
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        self.sock = self.rfile = None

    def send(self, line: str) -> str | None:
        """Send one command, wait for its reply. Returns the reply (or None)."""
        if self.dry:
            if self.verbose or not line.startswith("D"):
                print(f"[dry] {line}", flush=True)
            return "OK"
        with self.lock:
            if not self._connect():
                return None
            t0 = time.monotonic()
            try:
                self.sock.sendall((line + "\n").encode("ascii"))
                reply = self.rfile.readline().strip()
            except OSError as e:
                self._drop(str(e))
                return None
            if not reply:
                self._drop("server closed")
                return None
            self.last_rtt_ms = (time.monotonic() - t0) * 1000.0
            if reply.startswith("ERR"):
                self.errors += 1
                if reply != self.last_err:
                    print(f"[link] {line!r} -> {reply}", file=sys.stderr, flush=True)
                self.last_err = reply
            elif self.verbose:
                print(f"[link] {line} -> {reply} ({self.last_rtt_ms:.1f} ms)", flush=True)
            return reply

    def close(self) -> None:
        if self.sock:
            self.send("S")
            self._drop("closing")
