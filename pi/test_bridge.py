#!/usr/bin/env python3
"""Bridge-only tests for pi/tars_bridge.py. stdlib only.

    python3 pi/test_bridge.py          # or: make -C pi test-bridge (also runs test_server.py on it)

The full tars_motor suite (test_server.py) also runs against the bridge; this
file adds what the bridge does differently or what matters for the Wi-Fi MVP:
  1. laptop teleop keys -> real tars_client -> bridge -> fake Uno on a pty:
     the EXACT serial lines (slew ramp, arc, spin, release = instant 0, S).
  2. Two clients at once: one drives, the other's S wins; a disconnect stops.
  3. Watchdog on the wire: the client goes quiet -> D 0.000 within ~0.55 s.
  4. tars_client.move_mm / turn_deg (what wheel_cal.py uses) -> M/R and DONE.
  5. Serial auto-detect (no -d), the --stub flag, and the startup banner.
"""
from __future__ import annotations

import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "laptop"))
BRIDGE = os.path.join(HERE, "tars_bridge.py")

import test_server as ts  # noqa: E402
ts.BIN = BRIDGE
from test_server import Client, FakeUno, Server, check  # noqa: E402
from tars_client import Tars  # noqa: E402
from teleop import HoldDrive  # noqa: E402


def d_values(lines: list[str]) -> list[str]:
    """Collapse the 50 Hz D stream to its distinct consecutive values."""
    out: list[str] = []
    for l in lines:
        if l.startswith("D ") and (not out or out[-1] != l):
            out.append(l)
    return out


def ramp(a: float, b: float) -> list[float]:
    """The bridge's slew: 0.08 per 20 ms tick from a to b."""
    vals, v = [], a
    while abs(b - v) > 1e-9:
        v = v + max(-0.08, min(0.08, b - v))
        vals.append(round(v, 3))
    return vals


def test_teleop_on_the_wire() -> None:
    print("teleop keys -> tars_client -> bridge -> fake Uno (exact serial lines)")
    uno = FakeUno()
    srv = Server("-b", "uno", "-d", uno.path)
    t = Tars("127.0.0.1", srv.port, timeout=1.0).connect()
    try:
        uno.pump(0.3)
        uno.send("READY\r\n")
        check(srv.wait_log(r"co-processor READY"), "READY")
        h = HoldDrive(speed=0.8)
        script = [(0.00, "down", "w"), (0.50, "down", "d"), (0.90, "up", "d"), (1.30, "up", "w"),
                  (1.60, "down", "a"), (2.10, "up", "a"), (2.40, "end", "")]
        uno.lines.clear()
        t0, i = time.monotonic(), 0
        marks: dict[str, int] = {}
        while i < len(script):
            now = time.monotonic() - t0
            if now >= script[i][0]:
                _, kind, k = script[i]
                lines = h.key_down(k, now) if kind == "down" else h.key_up(k, now) if kind == "up" else []
                for line in lines:
                    t.send(line)
                marks[f"{kind}{k}"] = len(uno.lines)
                i += 1
                continue
            for line in h.tick(now):
                t.send(line)
            time.sleep(0.005)
        uno.pump(0.1)
        L = uno.lines
        non_d = [l for l in L if not l.startswith("D ")]
        # S from the client also halts the arms ("X"), exactly like the C server
        check(all(l.startswith("D ") or l == "X" for l in L), f"only D/X on the wire: {non_d[:5]}")

        w = d_values(L[:marks["downd"]])
        exp = [f"D {v:.3f} {v:.3f}" for v in ramp(0, 0.8)]
        check(w == exp, f"W: slewed ramp 0.08/tick up to 0.800\n    got {w}\n    exp {exp}")
        arc = d_values(L[marks["downd"]:marks["upd"]])
        check(arc and arc[-1] == "D 0.800 0.320", f"W+D: arc, inner (right) wheel 0.32: {arc[-3:]}")
        check(arc == [f"D 0.800 {v:.3f}" for v in ramp(0.8, 0.32)], f"arc slews the inner wheel down: {arc}")
        back = d_values(L[marks["upd"]:marks["upw"]])
        check(back and back[-1] == "D 0.800 0.800", f"release D: back to straight {back[-2:]}")
        rel = L[marks["upw"]:marks["downa"]]
        rd = [l for l in rel if l.startswith("D ")]
        check(rd and all(l == "D 0.000 0.000" for l in rd), f"release W (S): next D is 0 at once, no ramp {rd[:3]}")
        spin = d_values(L[marks["downa"]:marks["upa"]])
        check(spin and spin[-1] == "D -0.800 0.800", f"A: spin left, left wheel back {spin[-2:]}")
        after = [l for l in L[marks["upa"]:] if l.startswith("D ")]
        check(after and all(l == "D 0.000 0.000" for l in after), "release A: wheels 0 at once")
        check("S" not in L, "teleop S is handled by the bridge (no S to the Uno unless a move runs)")
        st = t.status()
        check(st["left"] == 0.0 and st["backend"] == "uno" and st["ready"] == 1, f"STATUS after {st}")
    finally:
        t.close()
        srv.stop()
        uno.close()


def test_two_clients_and_watchdog() -> None:
    print("two clients: S from either wins; disconnect stops; watchdog on the wire")
    uno = FakeUno()
    srv = Server("-b", "uno", "-d", uno.path, "-w", "500")
    a, b = Client(srv.port), Client(srv.port)
    try:
        uno.pump(0.2)
        for _ in range(10):
            a.cmd("D 0.5 0.5")
            time.sleep(0.04)
        check(a.status()["l"] == 0.5, "client A drives")
        uno.lines.clear()
        check(b.cmd("S") == "OK", "client B sends S")
        uno.pump(0.05)
        ds = [l for l in uno.lines if l.startswith("D ")]
        check(ds and ds[0] == "D 0.000 0.000", f"S from B: next D on the wire is 0 {ds[:2]}")
        check(b.status()["l"] == 0.0, "stopped")

        for _ in range(10):
            a.cmd("D 0.4 0.4")
            time.sleep(0.04)
        uno.lines.clear()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            if uno.lines and uno.lines[-1] == "D 0.000 0.000":
                break
            time.sleep(0.005)
        took = time.monotonic() - t0
        check(0.40 < took < 0.62, f"client silent -> watchdog zero on the wire after {took:.2f} s (500 ms)")
        check(b.status()["why"] == "watchdog", "why=watchdog")

        for _ in range(5):
            a.cmd("D 0.3 0.3")
            time.sleep(0.04)
        a.close()
        check(srv.wait_log(r"client disconnected -> stop"), "disconnect logged")
        uno.pump(0.1)      # like the C server: target 0, slewed down in <= 4 ticks
        ds = [l for l in uno.lines if l.startswith("D ")]
        check(ds and ds[-1] == "D 0.000 0.000", f"disconnect -> zero within 100 ms, not after the watchdog {ds[-5:]}")
        check(b.cmd("PING") == "OK PONG", "other client unaffected")
    finally:
        b.close()
        srv.stop()
        uno.close()


def test_exact_moves_via_client() -> None:
    print("tars_client move_mm / turn_deg (wheel_cal.py path) through the bridge")
    uno = FakeUno()
    srv = Server("-b", "uno", "-d", uno.path)
    t = Tars("127.0.0.1", srv.port, timeout=1.0).connect()
    try:
        uno.pump(0.2)
        uno.lines.clear()

        def answer_after(line: str, reply: str) -> None:
            import threading

            def run() -> None:
                end = time.monotonic() + 3
                while time.monotonic() < end and line not in uno.lines:
                    time.sleep(0.01)
                time.sleep(0.2)
                uno.send(reply)
            threading.Thread(target=run, daemon=True).start()

        answer_after("M 100.0 1.00", "DONE M 100.0\r\n")
        r = t.move_mm(100, wait=True)
        check("M 100.0 1.00" in uno.lines, "M on the wire")
        check(r["move"] == "done" and r["done"] == 100.0, f"move_mm done: {r}")
        answer_after("R -30.0 1.00", "DONE R -30.0\r\n")
        r = t.turn_deg(-30, wait=True)
        check("R -30.0 1.00" in uno.lines and r["move"] == "done" and r["done"] == -30.0, f"turn_deg -30 {r}")
        check(t.wheel_cal(70, 120) == (70.0, 120.0), "wheel_cal set")
        uno.pump(0.05)
        check("K 70.0 120.0" in uno.lines, "K on the wire")
    finally:
        t.close()
        srv.stop()
        uno.close()


def test_autodetect_and_flags() -> None:
    print("serial auto-detect, --stub, banner")
    d = tempfile.mkdtemp()
    os.environ["TARS_SERIAL_GLOB"] = os.path.join(d, "serusb*")
    try:
        srv = Server("-b", "uno", "-w", "300")              # no -d
        check(srv.wait_log(r"no Uno serial port yet"), "waits when no port matches")
        uno = FakeUno()
        os.symlink(uno.path, os.path.join(d, "serusb1"))
        uno.pump(1.3)
        check(srv.wait_log(r"\[uno\] opened .*serusb1 @115200"), "found the port once it appeared")
        check(any(l.startswith("D ") for l in uno.lines), "D stream started")
        srv.stop()
        uno.close()
    finally:
        del os.environ["TARS_SERIAL_GLOB"]
    os.environ["TARS_BRIDGE_NO_TERMIOS"] = "1"
    try:
        uno = FakeUno()
        srv = Server("-d", uno.path)
        check(srv.wait_log(r"\[uno\] configured with: stty"), "no termios -> stty fallback configured the port")
        uno.pump(0.3)
        uno.send("READY\r\n")
        check(srv.wait_log(r"co-processor READY"), "READY parsed after stty setup")
        check(uno.lines[:1] == ["S"] and "D 0.000 0.000" in uno.lines, f"lines flow {uno.lines[:2]}")
        srv.stop()
        uno.close()
    finally:
        del os.environ["TARS_BRIDGE_NO_TERMIOS"]
    srv = Server("--stub")
    c = Client(srv.port)
    check(c.status()["backend"] == "stub", "--stub = -b stub")
    check(srv.wait_log(r"tars_motor: backend=stub .*\(python bridge\)"), "banner says python bridge")
    c.close()
    check(srv.stop() == 0, "clean exit")


if __name__ == "__main__":
    t0 = time.monotonic()
    test_teleop_on_the_wire()
    test_two_clients_and_watchdog()
    test_exact_moves_via_client()
    test_autodetect_and_flags()
    print(f"{ts.checks - ts.failures}/{ts.checks} checks passed ({time.monotonic() - t0:.1f} s)")
    sys.exit(1 if ts.failures else 0)
