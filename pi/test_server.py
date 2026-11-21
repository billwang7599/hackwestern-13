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
import atexit
import tempfile
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
        atexit.register(self.stop)
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


def keepalive_for(c: Client, seconds: float) -> None:
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        c.cmd("PING")
        time.sleep(min(0.05, max(0, end - time.monotonic())))


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

        print("vision hook: S is a hard stop (no slew), VIS shows in STATUS")
        drive_for(c, 1, 1, 0.4)               # slewed up to full (resending D)
        st = c.status()
        check(st["l"] > 0.9, f"drove up to full speed: {st}")
        check(c.cmd("S") == "OK", "S")
        time.sleep(0.05)                      # <= 2 ticks at 50 Hz
        st = c.status()
        check(st["l"] == 0.0 and st["r"] == 0.0, f"S zeroes wheels within one tick: {st['l']} {st['r']}")
        check(0 <= int(st["stop_ms"]) <= 25, f"stop_ms = S received -> wheels 0 ({st['stop_ms']})")
        check(st["vis"] == "-" and st["vis_age_ms"] == "-1", "no VIS yet")
        check(c.cmd("VIS follow 14.5 52 61 fist") == "OK", "VIS accepted")
        st = c.status()
        check(st["vis"] == "follow" and st["vis_fps"] == "14.5" and st["vis_cam_ms"] == "52"
              and st["vis_stop_ms"] == "61" and st["vis_see"] == "fist", f"VIS fields {st}")
        check(c.cmd("VIS bad").startswith("ERR usage"), "VIS usage error")
        age0 = int(c.status()["age"])
        time.sleep(0.2)
        c.cmd("VIS gesture 10 1 -1 no_hand")
        check(int(c.status()["age"]) >= age0 + 150, "VIS does not feed the watchdog")

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
        # Never give a reconnecting backend a recyclable /dev/ttys path directly.
        # Removing this unique alias on cleanup prevents an orphan from opening
        # a later SSH/terminal PTY that reuses the same device number.
        self.directory = tempfile.mkdtemp(prefix="tars-fake-uno-")
        self.path = os.path.join(self.directory, "serial")
        os.symlink(os.ttyname(self.slave), self.path)
        self.buf = b""
        self.lines: list[str] = []
        self._run = True
        atexit.register(self.close)
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
                self.lines += [d.decode() for d in done if d]  # firmware ignores empty resync lines

    def pump(self, seconds: float = 0.0) -> None:
        time.sleep(seconds + 0.03)

    def send(self, s: str) -> None:
        os.write(self.master, s.encode())

    def close(self) -> None:
        if not self._run:
            return
        os.unlink(self.path)
        os.rmdir(self.directory)
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


# ------------------------------------------------------------------ stepper moves, IO, buttons (stub)
def wait_move(c: Client, timeout: float) -> dict:
    end = time.monotonic() + timeout
    st = c.status()
    while st["move"] == "running" and time.monotonic() < end:
        c.cmd("PING")  # exact moves now also require the native deadman lease
        time.sleep(0.02)
        st = c.status()
    return st


def test_stub_moves() -> None:
    srv = Server("-b", "stub", "-w", "300", "-k", "600")
    c = Client(srv.port)
    try:
        print("stub: boot with no client = stopped, watchdog armed")
        time.sleep(0.4)
        st = c.status()
        check(st["l"] == 0.0 and st["estop"] == 1 and st["why"] == "watchdog", f"idle at boot {st}")
        check(st["move"] == "idle" and st["move_id"] == "0" and st["btn_teach"] == "0"
              and st["btn_forget"] == "0" and st["led"] == "-1", f"boot status {st}")
        check(not any(re.search(r"drive L=[+-]0\.[1-9]", l) for l in srv.log), "never drove without a client")

        print("stub: MOVE / TURN with simulated step timing")
        r = c.cmd("MOVE 100")
        check(re.fullmatch(r"OK 1 \d+", r) is not None, f"MOVE reply {r}")
        est = int(r.split()[2])
        check(2600 < est < 2850, f"MOVE 100 est {est} ms (12 RPM, 65 mm wheels)")
        st = c.status()
        check(st["move"] == "running" and st["move_kind"] == "MOVE" and st["move_req"] == "100.0", f"running {st}")
        t0 = time.monotonic()
        st = wait_move(c, 5)
        took = (time.monotonic() - t0) * 1000
        check(st["move"] == "done" and st["move_done"] == "100.0" and st["move_why"] == "-", f"done {st}")
        check(est - 150 < took < est + 250, f"took {took:.0f} ms vs est {est}")
        r = c.cmd("TURN 30")
        est = int(r.split()[2])
        check(r.startswith("OK 2 ") and 850 < est < 980, f"TURN 30 -> {r}")
        st = wait_move(c, 3)
        check(st["move"] == "done" and st["move_kind"] == "TURN" and st["move_done"] == "30.0", f"turn {st}")
        r = c.cmd("TURN -90 0.5")
        check(int(r.split()[2]) > 2 * 1000, f"half speed is slower: {r}")
        check(srv.wait_log(r"move R -90\.0 speed 0\.50"), "speed passed to backend")
        c.cmd("S")

        print("stub: aborts")
        c.cmd("MOVE 200")
        keepalive_for(c, 1.0)
        check(c.cmd("S") == "OK", "S")
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "stop" and 0 < float(st["move_done"]) < 200,
              f"S aborts {st}")
        c.cmd("MOVE 200")
        keepalive_for(c, 0.3)
        for _ in range(3):
            c.cmd("D 0 0")                    # zero D = keepalive, doesn't abort
            time.sleep(0.05)
        check(c.status()["move"] == "running", "D 0 0 doesn't abort a move")
        check(c.cmd("D 0.3 0.3") == "OK", "D takes over")
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "drive", f"non-zero D aborts {st}")
        check(srv.wait_log(r"ABORT M [\d.]+ drive"), "backend saw the abort")
        c.cmd("S")
        r1 = c.cmd("MOVE 200")
        r2 = c.cmd("MOVE 50")
        check(r1.split()[1] != r2.split()[1], "new id")
        st = wait_move(c, 3)
        check(st["move"] == "done" and st["move_done"] == "50.0" and st["move_id"] == r2.split()[1],
              f"replaced move: the newer one is reported {st}")
        check(srv.wait_log(r"older move"), "older move's ABORT ignored")
        c.cmd("MOVE 200")
        time.sleep(0.2)
        c.cmd("SIM T 60")
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "tipped", f"tip aborts {st}")
        check(c.cmd("MOVE 10").startswith("ERR tipped") and c.cmd("TURN 10").startswith("ERR tipped"), "refused while tipped")
        time.sleep(1.05)
        check(c.cmd("RESET") == "OK", "RESET")
        c.cmd("MOVE 200")
        time.sleep(0.2)
        c.cmd("SIM B 1500")
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "bump", f"bump aborts {st}")
        check(c.cmd("MOVE 10").startswith("ERR bumped"), "refused during bump hold")
        time.sleep(0.7)
        c2 = Client(srv.port)
        c2.cmd("MOVE 200")
        time.sleep(0.2)
        c2.close()
        time.sleep(0.2)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "disconnect", f"disconnect aborts {st}")

        print("stub: exact moves also expire without command keepalive")
        c.cmd("MOVE 200")
        time.sleep(0.45)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "watchdog", f"move deadman {st}")

        print("stub: MOVE/TURN usage + clamps")
        for bad in ("MOVE", "MOVE x", "MOVE 10 0", "MOVE 10 2", "MOVE nan", "TURN", "TURN 10 -1"):
            check(c.cmd(bad).startswith("ERR usage"), f"{bad!r} rejected")
        c.cmd("MOVE 5000")
        check(c.status()["move_req"] == "1000.0", "MOVE clamped to 1000 mm")
        c.cmd("TURN -9999")
        check(c.status()["move_req"] == "-360.0", "TURN clamped to 360 deg")
        c.cmd("S")

        print("stub: WHEELCAL")
        check(c.cmd("WHEELCAL") == "OK 65.0 100.0", "WHEELCAL query")
        check(c.cmd("WHEELCAL 70 120") == "OK 70.0 120.0", "WHEELCAL set")
        check(srv.wait_log(r"wheel cal diam 70\.0 track 120\.0"), "cal reaches backend")
        check(c.cmd("WHEELCAL 5 5").startswith("ERR usage") and c.cmd("WHEELCAL 70").startswith("ERR usage"), "bad cal")
        st = c.status()
        check(st["wheel_d"] == "70.0" and st["wheel_t"] == "120.0", f"cal in STATUS {st}")
        r = c.cmd("TURN 30")
        check(950 < int(r.split()[2]) < 1050, f"wider track -> longer turn: {r}")
        c.cmd("S")
        c.cmd("WHEELCAL 65 100")

        print("stub: LED / BEEP / buttons")
        check(c.cmd("LED 2") == "OK" and srv.wait_log(r"\[stub\] led 2"), "LED 2")
        check(c.status()["led"] == "2", "led in STATUS")
        check(c.cmd("BEEP 3") == "OK" and srv.wait_log(r"\[stub\] beep 3"), "BEEP 3")
        for name, pat in (("teach", 2), ("rec", 1), ("ok", 1), ("no", 3), ("forget", 3), ("off", 0), ("TEACH", 2)):
            n0 = len(srv.log)
            check(c.cmd(f"LED {name}") == "OK", f"LED {name}")
            check(srv.wait_log(rf"\[stub\] led {pat}$"), f"LED {name} -> pattern {pat}")
            check(c.status()["led"] == str(pat), f"LED {name} in STATUS")
        for n in (1, 2, 3, 4):
            check(c.cmd(f"BEEP {n}") == "OK" and srv.wait_log(rf"\[stub\] beep {n}$"), f"BEEP {n}")
        drive_before = c.status()
        for bad in ("LED", "LED 4", "LED -1", "LED x", "LED 1 2", "LED teach now", "LED blink",
                    "BEEP", "BEEP 10", "BEEP 2x", "BEEP ok"):
            check(c.cmd(bad).startswith("ERR usage"), f"{bad!r} rejected")
        check(c.cmd("PING") == "OK PONG", "still in step after errors")
        st = c.status()
        check(st["l"] == drive_before["l"] and st["move"] == drive_before["move"]
              and st["led"] == drive_before["led"], "rejected LED/BEEP change nothing")
        for b in ("TEACH", "TEACH", "FORGET"):
            check(c.cmd(f"SIM BTN {b}") == "OK", f"SIM BTN {b}")
        time.sleep(0.1)
        st = c.status()
        check(st["btn_teach"] == "2" and st["btn_forget"] == "1", f"button counters {st}")
        check(c.cmd("SIM BTN X").startswith("ERR usage"), "SIM BTN bad")
        check(srv.wait_log(r"button TEACH") and srv.wait_log(r"button FORGET"), "buttons logged")
    finally:
        c.close()
    check(srv.stop() == 0, "stub exit 0")


def test_uno_moves() -> None:
    print("uno: MOVE/TURN/K/L/B/BTN on the wire")
    uno = FakeUno()
    srv = Server("-b", "uno", "-d", uno.path, "-w", "300")
    c = Client(srv.port)
    nd = lambda: [l for l in uno.lines if not l.startswith("D ")]
    try:
        uno.pump(0.3)
        uno.send("READY\r\n")
        check(srv.wait_log(r"co-processor READY"), "READY first")
        uno.lines.clear()
        r = c.cmd("MOVE 100")
        uno.pump(0.1)
        check(nd() == ["M 100.0 1.00"], f"M on the wire {nd()}")
        check(all(l == "D 0.000 0.000" for l in uno.lines if l.startswith("D ")), "keepalive D 0 0 during the move")
        check(c.status()["move"] == "running", "running until the Uno answers")
        uno.send("DONE M 100.0\r\n")
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "done" and st["move_done"] == "100.0", f"DONE parsed {st}")

        uno.lines.clear()
        c.cmd("TURN -30 0.5")
        uno.pump(0.1)
        check(nd() == ["R -30.0 0.50"], f"R on the wire {nd()}")
        c.cmd("S")
        uno.pump(0.1)
        check("S" in nd(), "S aborts on the wire")
        check(c.status()["move"] == "aborted", "aborted at once")
        uno.send("ABORT R -12.3 stop\r\n")
        time.sleep(0.1)
        st = c.status()
        check(st["move_done"] == "-12.3" and st["move_why"] == "stop", f"ABORT amount {st}")

        uno.lines.clear()
        c.cmd("MOVE 100")
        time.sleep(0.1)
        c.cmd("MOVE 50")
        uno.pump(0.1)
        check(nd() == ["M 100.0 1.00", "M 50.0 1.00"], f"both moves on the wire {nd()}")
        uno.send("ABORT M 5.0 replaced\r\n")
        time.sleep(0.1)
        check(c.status()["move"] == "running", "older ABORT doesn't end the new move")
        uno.send("DONE M 50.0\r\n")
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "done" and st["move_done"] == "50.0", f"replace accounting {st}")
        uno.send("DONE M 7.0\r\nDONE X 1\r\nABORT M\r\n")
        time.sleep(0.1)
        check(c.status()["move_done"] == "50.0" and srv.wait_log(r"stray DONE"), "stray/junk DONE ignored")

        c.cmd("MOVE 100")
        time.sleep(0.05)
        uno.send("ABORT M 0.0 nosteppers\r\n")       # the DC-wheel firmware
        time.sleep(0.1)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "nosteppers", f"DC firmware refusal {st}")

        uno.lines.clear()
        c.cmd("WHEELCAL 70 120")
        c.cmd("LED 3")
        c.cmd("BEEP 2")
        uno.pump(0.1)
        check(nd() == ["K 70.0 120.0", "L 3", "B 2"], f"K/L/B on the wire {nd()}")
        uno.lines.clear()
        for name in ("teach", "rec", "ok", "no", "forget", "off"):
            c.cmd(f"LED {name}")
            time.sleep(0.03)
        for n in (1, 2, 3, 4):
            c.cmd(f"BEEP {n}")
            time.sleep(0.03)
        uno.pump(0.1)
        check(nd() == ["L 2", "L 1", "L 1", "L 3", "L 3", "L 0", "B 1", "B 2", "B 3", "B 4"],
              f"LED names / BEEP 1-4 on the wire {nd()}")
        c.cmd("LED 3")

        uno.send("BTN TEACH\r\nBTN TEACH\nBTN FORGET\r\nBTN X\r\nBTN\r\n")
        time.sleep(0.15)
        st = c.status()
        check(st["btn_teach"] == "2" and st["btn_forget"] == "1" and st["bumps"] == "0", f"BTN parsed {st}")

        uno.lines.clear()
        c.cmd("MOVE 300")
        time.sleep(0.05)
        uno.send("READY\r\n")                         # Uno rebooted mid-move
        uno.pump(0.1)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "reboot", f"reboot aborts {st}")
        check("L 3" in nd(), f"LED pattern restored after reboot {nd()}")

        uno.lines.clear()
        r = c.cmd("TURN 1")                           # never answered -> timeout
        est = int(r.split()[2])
        keepalive_for(c, (2 * est + 2000) / 1000 + 0.3)
        st = c.status()
        check(st["move"] == "aborted" and st["move_why"] == "timeout", f"timeout {st}")
        check("S" in nd(), "timeout sends S")
    finally:
        c.close()
    srv.stop()
    uno.close()


def test_uno_reconnect() -> None:
    print("uno: missing at boot, unplug + replug")
    import tempfile
    d = tempfile.mkdtemp()
    link = os.path.join(d, "uno")

    def point(target: str) -> None:
        tmp = link + ".tmp"
        os.symlink(target, tmp)
        os.replace(tmp, link)

    srv = Server("-b", "uno", "-d", link, "-w", "300")
    try:
        time.sleep(0.3)
        check(srv.proc.poll() is None, "server stays up with no Uno")
        check(srv.wait_log(r"not there yet"), "logs that it is waiting")
        c = Client(srv.port)
        check(c.cmd("PING") == "OK PONG", "TCP works without the Uno")
        st = c.status()
        check(st["l"] == 0.0 and st["why"] == "watchdog", f"stopped {st}")
        time.sleep(1.2)
        check(sum("not there yet" in l for l in srv.log) == 1, "doesn't spam the log")

        u1 = FakeUno()
        point(u1.path)
        u1.pump(1.3)
        ds = [l for l in u1.lines if l.startswith("D ")]
        check(len(ds) > 5, f"found the Uno once it appeared ({len(ds)} D lines)")
        check(srv.wait_log(r"\[uno\] opened"), "opened")

        u1.close()                                    # unplugged
        check(srv.wait_log(r"\[uno\] lost", 1.0), "loss detected")
        u2 = FakeUno()
        point(u2.path)
        u2.pump(1.5)
        ds = [l for l in u2.lines if l.startswith("D ")]
        check(len(ds) > 5, f"reconnected after replug ({len(ds)} D lines)")
        check(sum("[uno] opened" in l for l in srv.log) == 2, "opened twice")
        u2.send("READY\r\nBTN TEACH\r\n")
        time.sleep(0.1)
        st = c.status()
        check(st["ready"] == "1" and st["btn_teach"] == "1", f"events flow after replug {st}")
        check(c.cmd("PING") == "OK PONG" and srv.proc.poll() is None, "server never went down")
        c.close()
        u2.close()
    finally:
        rc = srv.stop()
    check(rc == 0, f"clean exit {rc}")


def test_pico_still_builds() -> None:
    print("pico backend still selectable")
    srv = Server("-b", "pico", "-d", "/nonexistent")
    srv.proc.wait(3)
    check(srv.proc.returncode == 1 and srv.wait_log(r"\[pico\] open /nonexistent"), "pico backend present")


if __name__ == "__main__":
    t0 = time.monotonic()
    test_stub()
    test_uno()
    test_stub_moves()
    test_uno_moves()
    test_uno_reconnect()
    test_pico_still_builds()
    print(f"{checks - failures}/{checks} checks passed ({time.monotonic() - t0:.1f} s)")
    sys.exit(1 if failures else 0)
