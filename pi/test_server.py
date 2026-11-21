#!/usr/bin/env python3
"""Tests for tars_motor (host build). Standard library only.

    make -C pi test            # builds build/tars_motor_host, then runs this

Part 1 drives `-b stub` over TCP: protocol, watchdog, disconnect stop, arms,
gestures (wave/point/shrug/nod/stance/fold), tip-over latch + RESET, bump hold,
co-processor reboot, ZERO/LEVEL, Ctrl-C.
Part 2 runs the REAL uno backend (`-b uno`) against a fake Uno on a
pseudo-terminal and checks the exact serial lines both ways.
"""
from __future__ import annotations

import os
import re
import select
import signal
import socket
import subprocess
import sys
import threading
import time
import tty

BIN = sys.argv[1] if len(sys.argv) > 1 else "build/tars_motor_host"
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Server:
    def __init__(self, *args: str):
        self.port = free_port()
        self.proc = subprocess.Popen([BIN, "-p", str(self.port), "-r", "1", *args],
                                     stderr=subprocess.PIPE, text=True)
        self.log: list[str] = []
        self._t = threading.Thread(target=self._read, daemon=True)
        self._t.start()
        for _ in range(100):
            if any("tars_motor:" in l for l in self.log):
                break
            time.sleep(0.02)

    def _read(self) -> None:
        for line in self.proc.stderr:
            self.log.append(line.rstrip("\n"))

    def text(self) -> str:
        return "\n".join(self.log)

    def wait_log(self, pat: str, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(re.search(pat, l) for l in self.log):
                return True
            time.sleep(0.02)
        return False

    def stop(self) -> int:
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self._t.join(1)
        return self.proc.returncode


class Client:
    def __init__(self, port: int):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=2)
        self.f = self.s.makefile("rw", encoding="ascii", newline="\n")

    def cmd(self, line: str) -> str:
        self.f.write(line + "\n")
        self.f.flush()
        return self.f.readline().strip()

    def status(self) -> dict:
        r = self.cmd("STATUS").split()
        assert r[0] == "OK", r
        d = {"l": float(r[1]), "r": float(r[2]), "age": int(r[3]), "dist": float(r[4]),
             "estop": int(r[5])}
        for kv in r[6:]:
            k, v = kv.split("=", 1)
            d[k] = v
        return d

    def close(self) -> None:
        self.f.close()          # the makefile holds the socket open otherwise
        self.s.close()


def drive_for(c: Client, l: float, r: float, seconds: float) -> list[str]:
    replies, end = [], time.monotonic() + seconds
    while time.monotonic() < end:
        replies.append(c.cmd(f"D {l} {r}"))
        time.sleep(0.05)
    return replies


# ------------------------------------------------------------------ part 1: stub
def test_stub() -> None:
    srv = Server("-b", "stub", "-w", "300", "-k", "800")
    c = Client(srv.port)
    try:
        print("basics")
        check(c.cmd("PING") == "OK PONG", "PING")
        st = c.status()
        check(st["backend"] == "stub" and st["gesture"] == "none" and st["tipped"] == "0", f"status {st}")
        check(c.cmd("FOO").startswith("ERR unknown"), "unknown command")
        check(c.cmd("D 1").startswith("ERR usage"), "D needs two args")
        check(c.cmd("D nan 0").startswith("ERR"), "D nan")
        check(c.cmd("GESTURES") == "OK wave point shrug nod stance fold", "GESTURES")
        for bad in ("A", "A 0", "A 2 10", "A -1 10", "A 0 10 -5", "A 0 10 0", "A x 10"):
            check(c.cmd(bad).startswith("ERR usage"), f"{bad!r} rejected")
        check(c.cmd("G dance").startswith("ERR unknown gesture"), "unknown gesture")
        check(c.cmd("G").startswith("ERR usage"), "G with no name")
        check(c.cmd("SIM X").startswith("ERR usage"), "SIM bad kind")
        c.f.write("\n\r\n")       # blank lines get no reply...
        check(c.cmd("PING") == "OK PONG", "...so replies stay in step")

        print("drive + slew")
        replies = drive_for(c, 0.5, 0.5, 0.5)
        check(all(r == "OK" for r in replies), "D replies OK")
        st = c.status()
        check(st["l"] == 0.5 and st["r"] == 0.5 and st["estop"] == 0 and st["why"] == "none", f"driving {st}")
        check(srv.wait_log(r"\[stub\] drive L=\+0\.08 R=\+0\.08"), "slewed first step +0.08")
        check(srv.wait_log(r"\[stub\] drive L=\+0\.50 R=\+0\.50"), "reached 0.50")
        c.cmd("D 2 -3")
        time.sleep(0.6)
        st = c.status()
        check(st["l"] == 0.0 and st["why"] == "watchdog", f"clamped then watchdog {st}")

        print("watchdog")
        drive_for(c, 0.4, 0.4, 0.3)
        time.sleep(0.5)
        st = c.status()
        check(st["estop"] == 1 and st["why"] == "watchdog" and st["l"] == 0.0, f"watchdog stop {st}")
        check(srv.wait_log(r"STOP \(watchdog\)"), "watchdog logged")

        print("disconnect -> stop")
        c2 = Client(srv.port)
        for _ in range(5):
            c2.cmd("D 0.3 0.3")
            time.sleep(0.05)
        c2.close()
        check(srv.wait_log(r"client disconnected -> stop"), "disconnect logged")
        time.sleep(0.1)
        check(c.status()["l"] == 0.0, "stopped after disconnect")

        print("arms")
        check(c.cmd("A 0 30 60") == "OK 30.0", "A 0 30 60")
        check(srv.wait_log(r"\[stub\] arm 0 -> 30\.0 deg @ 60 dps"), "arm cmd sent to backend")
        time.sleep(0.3)
        st = c.status()
        check(0 < float(st["arm0"]) < 30 and st["arm0_tgt"] == "30.0", f"arm moving {st}")
        time.sleep(0.4)
        check(c.status()["arm0"] == "30.0", "arm0 arrived (model)")
        check(c.cmd("A 1 500 999") == "OK 135.0", "clamped angle")
        check(srv.wait_log(r"arm 1 -> 135\.0 deg @ 60 dps"), "clamped speed to 60 dps")
        check(c.cmd("HOME") == "OK", "HOME")
        check(srv.wait_log(r"arm 1 -> 0\.0 deg @ 45 dps"), "HOME sends 0 at default speed")
        time.sleep(1.0)
        st = c.status()
        check(st["arm0"] == "0.0" and st["arm1"] == "0.0", f"homed {st}")

        print("gesture shrug")
        r = c.cmd("G shrug")
        check(r.startswith("OK "), f"G shrug -> {r}")
        ms = int(r.split()[1]) if r.startswith("OK ") else 0
        check(1100 < ms < 1500, f"shrug duration {ms}")
        time.sleep(0.15)
        check(c.status()["gesture"] == "shrug", "gesture running")
        check(srv.wait_log(r"arm 0 -> 25\.0") and srv.wait_log(r"arm 1 -> 25\.0"), "both arms 25")
        check(srv.wait_log(r"gesture shrug done", ms / 1000 + 1.0), "shrug done")
        st = c.status()
        check(st["gesture"] == "none" and st["arm0"] == "0.0" and st["arm1"] == "0.0", f"after shrug {st}")

        print("gesture wave (keyframe order)")
        n0 = len(srv.log)
        r = c.cmd("G wave")
        ms = int(r.split()[1])
        check(srv.wait_log(r"gesture wave done", ms / 1000 + 1.5), "wave done")
        seq = [float(m.group(1)) for l in srv.log[n0:] if (m := re.search(r"arm 1 -> (-?[\d.]+)", l))]
        check(seq == [70, 40, 75, 40, 75, 40, 75, 0], f"wave sequence {seq}")
        check(not any("arm 0 ->" in l for l in srv.log[n0:]), "wave leaves arm 0 alone")

        print("gesture nod / stance / fold")
        r = c.cmd("G nod")
        check(srv.wait_log(r"gesture nod done", int(r.split()[1]) / 1000 + 1.0), "nod done")
        c.cmd("G stance")
        check(srv.wait_log(r"gesture stance done", 2.0), "stance done")
        st = c.status()
        check(st["arm0"] == "30.0" and st["arm1"] == "-30.0", f"stance holds pose {st}")
        c.cmd("G fold")
        check(srv.wait_log(r"gesture fold done", 2.0), "fold done")
        st = c.status()
        check(st["arm0"] == "0.0" and st["arm1"] == "0.0", f"folded {st}")

        print("cancel: S, A, and a new gesture")
        c.cmd("G point")
        time.sleep(0.4)
        check(c.cmd("S") == "OK", "S")
        check(srv.wait_log(r"gesture point cancelled"), "S cancels gesture")
        check(srv.wait_log(r"\[stub\] arms halt"), "S halts arms")
        time.sleep(0.1)
        st = c.status()
        a1 = float(st["arm1"])
        check(st["gesture"] == "none" and 5 < a1 < 60 and st["arm1_tgt"] == st["arm1"], f"halted mid-way {st}")
        time.sleep(0.3)
        check(float(c.status()["arm1"]) == a1, "arm stays halted")
        c.cmd("G wave")
        time.sleep(0.3)
        c.cmd("A 0 10")
        check(srv.wait_log(r"gesture wave cancelled"), "A cancels gesture")
        c.cmd("G point")
        time.sleep(0.2)
        n0 = len(srv.log)
        c.cmd("G shrug")
        check(srv.wait_log(r"gesture point cancelled") and srv.wait_log(r"gesture shrug start"), "new gesture replaces old")
        check(srv.wait_log(r"gesture shrug done", 3.0), "replacement finished")
        c.cmd("HOME")
        time.sleep(1.5)

        print("tip-over latch")
        drive_for(c, 0.5, 0.5, 0.4)
        c.cmd("G wave")
        time.sleep(0.2)
        check(c.cmd("SIM T 63") == "OK", "SIM T")
        check(srv.wait_log(r"TIPPED \(63 deg\)"), "tip logged")
        check(srv.wait_log(r"gesture wave cancelled \(tipped\)"), "tip cancels gesture")
        time.sleep(0.1)
        st = c.status()
        check(st["tipped"] == "1" and st["why"] == "tipped" and st["estop"] == 1 and st["l"] == 0.0
              and st["tilt"] == "63", f"tipped status {st}")
        check(srv.wait_log(r"drive L=\+0\.00 R=\+0\.00"), "wheels hard-stopped")
        check(c.cmd("D 0.5 0.5").startswith("ERR tipped"), "D refused while tipped")
        check(c.cmd("G wave").startswith("ERR tipped"), "G refused while tipped")
        check(c.cmd("A 0 10").startswith("ERR tipped"), "A refused while tipped")
        check(c.cmd("HOME").startswith("ERR tipped"), "HOME refused while tipped")
        check(c.cmd("RESET").startswith("ERR still tipped"), "RESET refused right after a T")
        c.cmd("SIM T 61")          # the Uno repeats T every 500 ms while tipped
        time.sleep(0.6)
        check(c.cmd("RESET").startswith("ERR still tipped"), "still refused 600 ms later")
        time.sleep(0.5)
        check(c.cmd("RESET") == "OK", "RESET once the T reports stop")
        check(srv.wait_log(r"RESET: tip latch cleared"), "reset logged")
        drive_for(c, 0.3, 0.3, 0.4)
        st = c.status()
        check(st["tipped"] == "0" and st["l"] == 0.3, f"drives again {st}")

        print("bump hold")
        c.cmd("SIM B 1500")
        time.sleep(0.1)
        r = c.cmd("D 0.3 0.3")
        check(r.startswith("ERR bumped"), f"D refused during bump hold: {r}")
        st = c.status()
        check(st["why"] == "bump" and st["bumps"] == "1" and st["l"] == 0.0 and int(st["bump_ms"]) > 0, f"bump {st}")
        check(srv.wait_log(r"BUMP \(1500 mg\)"), "bump logged")
        time.sleep(0.8)
        check(c.cmd("D 0.3 0.3") == "OK", "drives again after hold")
        c.cmd("SIM B 1300")
        time.sleep(0.1)
        check(c.cmd("RESET") == "OK" and c.cmd("D 0.2 0.2") == "OK", "RESET also clears a bump hold")

        print("co-processor READY / ZERO / LEVEL")
        c.cmd("A 0 20 60")
        time.sleep(0.5)
        c.cmd("SIM READY")
        check(srv.wait_log(r"co-processor READY$"), "first READY")
        c.cmd("A 0 20 60")
        time.sleep(0.5)
        c.cmd("SIM READY")
        check(srv.wait_log(r"READY again \(it rebooted\)"), "second READY = reboot warning")
        time.sleep(0.1)
        st = c.status()
        check(st["arm0"] == "0.0" and st["ready"] == "2", f"arm model reset on reboot {st}")
        c.cmd("A 1 15 60")
        time.sleep(0.5)
        check(c.cmd("ZERO") == "OK" and srv.wait_log(r"\[stub\] arms zero"), "ZERO")
        time.sleep(0.1)
        check(c.status()["arm1"] == "0.0", "ZERO resets model")
        check(c.cmd("LEVEL") == "OK" and srv.wait_log(r"\[stub\] level"), "LEVEL")
    finally:
        c.close()

    print("Ctrl-C")
    rc = srv.stop()
    check(rc == 0, f"exit code {rc}")
    check("[stub] shutdown (motors stopped)" in srv.text() and "tars_motor: bye" in srv.text(), "clean shutdown")


# ------------------------------------------------------------------ part 2: uno over a pty
class FakeUno:
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        self.path = os.ttyname(self.slave)
        self.buf = b""
        self.lines: list[str] = []
        self._run = True
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self) -> None:
        # Drain continuously, like a real Uno, so the pty buffer never fills.
        while self._run:
            r, _, _ = select.select([self.master], [], [], 0.02)
            if r:
                try:
                    self.buf += os.read(self.master, 4096)
                except OSError:
                    return
                *done, self.buf = self.buf.split(b"\n")
                self.lines += [d.decode() for d in done]

    def pump(self, seconds: float = 0.0) -> None:
        time.sleep(seconds + 0.03)

    def send(self, s: str) -> None:
        os.write(self.master, s.encode())

    def close(self) -> None:
        self._run = False
        self._t.join(1)
        os.close(self.master)
        os.close(self.slave)


def test_uno() -> None:
    print("uno backend over a pseudo-terminal")
    uno = FakeUno()
    srv = Server("-b", "uno", "-d", uno.path, "-w", "300")
    c = Client(srv.port)
    try:
        uno.pump(1.0)
        check(uno.lines and uno.lines[0] == "S", f"first line is S: {uno.lines[:2]}")
        ds = [l for l in uno.lines if l.startswith("D ")]
        check(40 <= len(ds) <= 55, f"D sent every tick (~50/s): {len(ds)}")
        check(all(l == "D 0.000 0.000" for l in ds), "idle D lines are zero")
        check(srv.wait_log(r"\[uno\] opened"), "opened log")

        uno.send("# mpu6050 at 0x68\r\nREADY\r\n")
        check(srv.wait_log(r"\[uno\] # mpu6050 at 0x68"), "# line logged, not acted on")
        check(srv.wait_log(r"co-processor READY$"), "READY parsed (CRLF)")
        check(c.status()["ready"] == "1", "ready=1")

        uno.lines.clear()
        drive_for(c, 0.5, -0.5, 0.5)
        uno.pump(0.1)
        check("D 0.500 -0.500" in uno.lines, "drive reaches the wire")

        uno.lines.clear()
        c.cmd("G wave")
        c.cmd("PING")
        uno.pump(0.3)
        check("A 1 70.0 60" in uno.lines, f"gesture keyframe on the wire: {[l for l in uno.lines if l[0] != 'D']}")
        c.cmd("S")
        uno.pump(0.2)
        check("X" in uno.lines, "S -> X (halt arms)")
        uno.lines.clear()
        c.cmd("A 0 -20 30")
        time.sleep(0.05)
        c.cmd("ZERO")
        time.sleep(0.05)
        c.cmd("LEVEL")
        uno.pump(0.2)
        non_d = [l for l in uno.lines if not l.startswith("D ")]
        check(non_d == ["A 0 -20.0 30", "Z", "U"], f"A/ZERO/LEVEL lines {non_d}")

        check(c.cmd("SIM T 50").startswith("ERR SIM is only"), "SIM refused on real backend")

        print("uno: junk, split lines, tip, bump")
        uno.send("T abc\nHIT 5\nB\n# T 99\nREADX\n")
        uno.pump(0.2)
        check(c.status()["tipped"] == "0" and c.status()["bumps"] == "0", "junk ignored")
        drive_for(c, 0.4, 0.4, 0.3)
        uno.send("T")
        uno.pump(0.1)
        uno.send(" 66\n")
        check(srv.wait_log(r"TIPPED \(66 deg\)"), "split T line parsed")
        uno.pump(0.05)
        uno.lines.clear()
        uno.pump(0.2)
        check(c.status()["tipped"] == "1", "tipped")
        check(uno.lines and all(l == "D 0.000 0.000" for l in uno.lines if l.startswith("D ")), "zero drive after tip")
        check(c.cmd("D 0.4 0.4").startswith("ERR tipped"), "D refused")
        time.sleep(1.05)
        check(c.cmd("RESET") == "OK", "RESET after T stops")
        uno.send("B 1700\r\n")
        check(srv.wait_log(r"BUMP \(1700 mg\)"), "bump parsed")
        check(c.cmd("D 0.2 0.2").startswith("ERR bumped"), "bump hold")
    finally:
        c.close()
    uno.lines.clear()
    rc = srv.stop()
    uno.pump(0.2)
    check(rc == 0, f"uno exit code {rc}")
    check(uno.lines and uno.lines[-1] == "S", f"last line on shutdown is S: {uno.lines[-2:]}")
    uno.close()


def test_pico_still_builds() -> None:
    print("pico backend still selectable")
    srv = Server("-b", "pico", "-d", "/nonexistent")
    srv.proc.wait(3)
    check(srv.proc.returncode == 1 and srv.wait_log(r"\[pico\] open /nonexistent"), "pico backend present")


if __name__ == "__main__":
    t0 = time.monotonic()
    test_stub()
    test_uno()
    test_pico_still_builds()
    print(f"{checks - failures}/{checks} checks passed ({time.monotonic() - t0:.1f} s)")
    sys.exit(1 if failures else 0)
