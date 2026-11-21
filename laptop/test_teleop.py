#!/usr/bin/env python3
"""Tests for the hold-to-drive teleop (laptop/teleop.py). stdlib only, except
part 3, which needs pygame(-ce) and is skipped without it.

    python3 laptop/test_teleop.py        # needs pi/build/tars_motor_host (make -C pi host)

1. HoldDrive: scripted keydown/keyup -> exact D/S stream (mixing, arcs, cancel,
   20 Hz resend, release = S, focus loss, speed caps).
2. The same script through the real tars_client against the stub tars_motor:
   the server sees the D/S stream and its wheels follow (hard stop on release).
3. The real pygame window (SDL dummy video driver) with posted key events.
"""
from __future__ import annotations

import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import teleop  # noqa: E402
from teleop import HoldDrive  # noqa: E402

BIN = os.path.join(HERE, "..", "pi", "build", "tars_motor_host")
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def test_logic() -> None:
    print("HoldDrive: mixing")
    h = HoldDrive(speed=0.5)
    check(h.key_down("w", 0.0) == ["D 0.500 0.500"], "W forward")
    check(h.key_up("w", 0.1) == ["S"], "release W -> S")
    check(h.key_down("s", 0.2) == ["D -0.500 -0.500"], "S back")
    check(h.key_up("s", 0.3) == ["S"], "release S")
    check(h.key_down("a", 0.4) == ["D -0.500 0.500"], "A spins left: left back, right forward")
    check(h.key_up("a", 0.5) == ["S"], "release A")
    check(h.key_down("d", 0.6) == ["D 0.500 -0.500"], "D spins right")
    check(h.key_down("w", 0.7) == ["D 0.500 0.200"], "W+D arc right: right (inner) wheel slower")
    check(h.key_up("d", 0.8) == ["D 0.500 0.500"], "back to straight")
    check(h.key_down("a", 0.9) == ["D 0.200 0.500"], "W+A arc left")
    check(h.key_up("w", 1.0) == ["D -0.500 0.500"], "A alone again = spin")
    check(h.key_down("s", 1.1) == ["D -0.200 -0.500"], "S+A arc backwards, left inner")
    check(h.key_up("a", 1.2) == ["D -0.500 -0.500"], "S alone")
    check(h.key_down("w", 1.3) == ["S"], "W+S cancel -> stop")
    check(h.key_up("s", 1.4) == ["D 0.500 0.500"], "W alone again")
    check(h.key_down("a", 1.5) == ["D 0.200 0.500"] and h.key_down("d", 1.6) == ["D 0.500 0.500"],
          "W+A+D: A and D cancel -> straight")
    check(h.stop(1.7) == ["S"] and not h.held and h.wheels() == (0.0, 0.0), "space: stop + forget keys")
    check(h.key_up("w", 1.8) == [] and h.key_up("a", 1.8) == [], "key-ups after space are silent")
    check(h.key_down("q", 1.9) == [] and h.key_down("w", 2.0) == ["D 0.500 0.500"]
          and h.key_down("w", 2.01) == [], "non-drive keys + OS repeat keydowns ignored")
    h.stop(2.1)

    print("HoldDrive: 20 Hz resend, release, focus loss")
    h = HoldDrive(speed=1.0)
    out, t = [], 0.0
    out += h.key_down("w", t)
    while t < 1.0 - 1e-9:                     # 100 fps frame loop for 1 s
        t = round(t + 0.01, 2)
        out += h.tick(t)
    check(len(out) == 21 and all(x == "D 1.000 1.000" for x in out), f"20 Hz while held: {len(out)}")
    check(h.key_up("w", 1.0) == ["S"], "release -> S at once")
    check(all(h.tick(1.0 + i * 0.05) == [] for i in range(1, 20)), "silence after release (watchdog not needed)")
    h.key_down("d", 2.0)
    check(h.stop(2.1) == ["S"] and h.tick(2.2) == [], "focus lost -> S, no more D")

    print("HoldDrive: speed")
    h = HoldDrive(speed=0.9)
    h.key_down("w", 0)
    check(h.change_speed(+0.1, 0.01) == ["D 1.000 1.000"], "+ while moving resends")
    check(h.change_speed(+0.1, 0.02) == ["D 1.000 1.000"] and h.speed == 1.0, "capped at 1.0 = max RPM")
    h.stop(0.03)
    for _ in range(20):
        h.change_speed(-0.1, 0.04)
    check(h.speed == 0.1 and h.change_speed(-0.1, 0.05) == [], "floor 0.1, silent when stopped")
    check(teleop.WHEEL_MAX_RPM == 12.0, "HUD RPM matches the Uno's WHEEL_MAX_RPM")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Server:
    def __init__(self):
        self.port = free_port()
        self.proc = subprocess.Popen([BIN, "-b", "stub", "-p", str(self.port), "-r", "1"],
                                     stderr=subprocess.PIPE, text=True)
        self.log: list[str] = []
        threading.Thread(target=self._read, daemon=True).start()
        for _ in range(100):
            if any("tars_motor:" in l for l in self.log):
                break
            time.sleep(0.02)

    def _read(self) -> None:
        for line in self.proc.stderr:
            self.log.append(line.rstrip("\n"))

    def drives(self) -> list[tuple[float, float]]:
        return [(float(m[1]), float(m[2])) for l in self.log
                if (m := re.search(r"drive L=([+-][\d.]+) R=([+-][\d.]+)", l))]

    def stop(self) -> None:
        self.proc.send_signal(signal.SIGINT)
        self.proc.wait(3)


class Spy(teleop.Link):
    """The real Link -> tars_client -> TCP, recording every line it sends."""
    def __init__(self, host: str, port: int):
        os.environ["TARS_PORT"] = str(port)
        super().__init__(host)
        self.port = port
        self.sent: list[tuple[float, str]] = []

    def ensure(self) -> bool:
        if self.t:
            return True
        from tars_client import Tars
        self.t = Tars(self.host, self.port, timeout=1.0).connect()
        self.status = "connected"
        return True

    def send(self, line: str) -> None:
        self.sent.append((time.monotonic(), line))
        super().send(line)


def test_stub_stream() -> None:
    print("scripted keys -> tars_client -> stub tars_motor")
    srv = Server()
    link = Spy("127.0.0.1", srv.port)
    h = HoldDrive(speed=0.8)
    script = [(0.00, "down", "w"), (0.60, "down", "d"), (0.90, "up", "d"), (1.20, "up", "w"),
              (1.50, "down", "a"), (1.90, "up", "a"), (2.20, "end", "")]
    t0 = time.monotonic()
    status_moving = status_after = None
    i = 0
    while i < len(script):
        now = time.monotonic() - t0
        if now >= script[i][0]:
            _, kind, k = script[i]
            if kind == "down":
                for line in h.key_down(k, now): link.send(line)
            elif kind == "up":
                for line in h.key_up(k, now): link.send(line)
                if k == "w":
                    time.sleep(0.05)
                    status_after = link.t.status()
            i += 1
            continue
        for line in h.tick(now):
            link.send(line)
        if 1.0 < now < 1.05 and status_moving is None:
            status_moving = link.t.status()
        time.sleep(0.005)
    lines = [l for _, l in link.sent]
    check(lines[0] == "D 0.800 0.800", f"first line {lines[0]}")
    check("D 0.800 0.320" in lines, "W+D arc reached the wire")
    stops = [n for n, l in enumerate(lines) if l == "S"]
    check(len(stops) == 2, f"exactly one S per release-to-idle: {len(stops)}")
    ds = [t for t, l in link.sent[:stops[0]] if l.startswith("D")]
    gaps = [b - a for a, b in zip(ds, ds[1:])]
    check(len(ds) >= 22 and max(gaps) < 0.075, f"D every ~50 ms while held ({len(ds)} lines, max gap {max(gaps):.3f})")
    check(status_moving and status_moving["estop"] is False and status_moving["left"] > 0.7,
          f"server driving while held {status_moving}")
    check(status_after and status_after["left"] == 0.0 and status_after["right"] == 0.0,
          f"release = wheels 0 on the server within 50 ms {status_after}")
    check(any(l == -0.8 or abs(l + 0.8) < 0.01 for l, _ in srv.drives()), "spin-left wheel speeds reached the backend")
    time.sleep(0.7)
    check(link.t.status()["why"] == "watchdog", "idle afterwards: watchdog-stopped, nothing more sent")
    link.close()
    srv.stop()
    check("[stub] shutdown" in "\n".join(srv.log), "server clean")


def test_window() -> None:
    try:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
        import pygame
    except ImportError:
        print("pygame window: SKIPPED (pygame-ce not installed)")
        return
    print("pygame window (dummy video) with posted key events")
    srv = Server()
    os.environ["TARS_PORT"] = str(srv.port)
    import importlib
    import tars_client
    importlib.reload(tars_client)
    importlib.reload(teleop)

    def script() -> None:
        time.sleep(0.8)
        ev = lambda t, k: pygame.event.post(pygame.event.Event(t, key=k, mod=0, unicode="", scancode=0))
        ev(pygame.KEYDOWN, pygame.K_w); time.sleep(0.5)
        ev(pygame.KEYUP, pygame.K_w); time.sleep(0.3)
        ev(pygame.KEYDOWN, pygame.K_a); time.sleep(0.3)
        pygame.event.post(pygame.event.Event(pygame.WINDOWFOCUSLOST)); time.sleep(0.3)
        ev(pygame.KEYDOWN, pygame.K_ESCAPE)

    th = threading.Thread(target=script, daemon=True)
    th.start()
    t0 = time.monotonic()
    teleop.run_window("127.0.0.1", 0.8)
    th.join(1)
    took = time.monotonic() - t0
    d = srv.drives()
    check(took < 4, f"Esc quits ({took:.1f} s)")
    check((0.8, 0.8) in d or any(abs(l - 0.8) < 0.01 and abs(r - 0.8) < 0.01 for l, r in d), f"W drove {d}")
    check(any(l < -0.5 and r > 0.5 for l, r in d), f"A spun left {d}")
    check(d and d[-1] == (0.0, 0.0), f"stopped at the end {d[-1:]}")
    srv.stop()


if __name__ == "__main__":
    if not os.path.exists(BIN):
        sys.exit(f"build the server first: make -C pi host   ({BIN} missing)")
    test_logic()
    test_stub_stream()
    test_window()
    print(f"{checks - failures}/{checks} checks passed")
    sys.exit(1 if failures else 0)
