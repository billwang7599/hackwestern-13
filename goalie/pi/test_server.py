#!/usr/bin/env python3
"""End-to-end test for goalie_motor (host build, stub backend). Stdlib only.

    make test          # builds build/goalie_motor_host, then runs this

Starts the server on a free port, drives it over TCP like the laptop would,
and checks the protocol from goalie/docs/architecture.md: PING/STATUS,
prediction convergence, SIMHIT saves (debounced), reversal saves, GOAL, HUMAN,
watchdog park, LINUX-mode jitter emulation, RESET, and a clean SIGINT stop.
"""
from __future__ import annotations

import os
import random
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.path.join(HERE, "build", "goalie_motor_host")

REQUIRED_KEYS = {"mode", "ctl", "saves", "goals", "target", "paddle", "duty", "wd",
                 "ms_since_frame", "jit_avg_us", "jit_max_us", "loop_hz", "hits"}

FPS = 60
DT = 1.0 / FPS


class Client:
    def __init__(self, port: int):
        deadline = time.time() + 5
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=2)
                break
            except OSError:
                if time.time() > deadline:
                    raise
                time.sleep(0.05)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.f = self.sock.makefile("r")
        self.t0 = time.monotonic()

    def cmd(self, line: str) -> str:
        self.sock.sendall((line + "\n").encode())
        reply = self.f.readline().strip()
        if not reply:
            raise RuntimeError(f"no reply to {line!r}")
        return reply

    def status(self) -> dict:
        r = self.cmd("STATUS")
        assert r.startswith("OK "), r
        return dict(kv.split("=", 1) for kv in r[3:].split())

    def t_ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000)

    def frame(self, ball, paddle_x=0.0) -> tuple[float, float]:
        if ball is None:
            line = f"F {self.t_ms()} 0 0 0 1 {paddle_x:.4f}"
        else:
            line = f"F {self.t_ms()} 1 {ball[0]:.4f} {ball[1]:.4f} 1 {paddle_x:.4f}"
        r = self.cmd(line).split()
        assert r[0] == "OK" and len(r) == 3, r
        return float(r[1]), float(r[2])

    def roll(self, x0, y0, vx, vy, until, noise=0.001, rng=None, on_frame=None):
        """Stream real-time frames of a straight roll until until(x, y) is true."""
        rng = rng or random.Random(1)
        x, y = x0, y0
        last = None
        while not until(x, y):
            last = self.frame((x + rng.gauss(0, noise), y + rng.gauss(0, noise)))
            if on_frame:
                on_frame(x, y)
            time.sleep(DT)
            x += vx * DT
            y += vy * DT
        return last


passed = 0


def check(cond, msg):
    global passed
    if not cond:
        raise AssertionError(msg)
    passed += 1
    print(f"  ok  {msg}")


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main() -> int:
    if not os.path.exists(BIN):
        print(f"missing {BIN}; run `make host` first", file=sys.stderr)
        return 2
    port = free_port()
    log = tempfile.NamedTemporaryFile("w+", prefix="goalie_motor_", suffix=".log", delete=False)
    proc = subprocess.Popen([BIN, "-b", "stub", "-p", str(port), "-w", "300"],
                            stdout=log, stderr=subprocess.STDOUT)
    ok = False
    try:
        c = Client(port)
        print(f"goalie_motor_host on :{port} (log {log.name})")

        print("protocol")
        check(c.cmd("PING") == "OK PONG", "PING -> OK PONG")
        st = c.status()
        check(REQUIRED_KEYS <= st.keys(), f"STATUS has all contract keys ({len(st)} keys)")
        check(st["mode"] == "QNX" and st["ctl"] == "AUTO", "starts in QNX/AUTO")
        check(c.cmd("FOO").startswith("ERR"), "unknown command -> ERR")
        check(c.cmd("F 1 2 0 0 1 0").startswith("ERR"), "bad F (seen=2) -> ERR")
        check(c.cmd("F 1 1 0.1").startswith("ERR"), "short F -> ERR")
        check(c.cmd("MODE BSD").startswith("ERR"), "bad MODE -> ERR")

        print("prediction: ball rolls from (0, 0.55) to cross at x=0.08, 1.0 m/s")
        x_cross, speed, y0 = 0.08, 1.0, 0.55
        vy = -speed
        vx = x_cross / (y0 / speed)
        tgt, duty = c.roll(0.0, y0, vx, vy, until=lambda x, y: y < 0.15)
        st = c.status()
        check(abs(tgt - x_cross) < 0.01, f"F reply target {tgt:.4f} ~ {x_cross} (true crossing)")
        check(st["pred"] == "INCOMING", "STATUS pred=INCOMING")
        check(duty > 0.0, f"duty {duty:+.3f} > 0 (paddle at 0, target to the right)")

        print("save by accelerometer HIT (stub SIMHIT), debounced")
        hits_sent = []

        def maybe_hit(x, y):
            if y < 0.10 and len(hits_sent) < 2:
                hits_sent.append(c.cmd("SIMHIT 900"))
        c.roll(0.0 + vx * (y0 - 0.15) / speed, 0.15, vx, vy,
               until=lambda x, y: y < 0.03, on_frame=maybe_hit)
        time.sleep(0.05)
        st = c.status()
        check(all(h.startswith("OK SIMHIT") for h in hits_sent) and len(hits_sent) == 2,
              "two SIMHITs accepted")
        check(st["hits"] == "2", "hits=2")
        check(st["saves"] == "1", "saves=1 (one save per shot)")

        print("watchdog")
        time.sleep(0.5)
        st = c.status()
        check(st["wd"] == "1", "no frames for 500 ms -> wd=1")
        check(abs(float(st["target"])) < 1e-9, f"target parked at 0 (target={st['target']})")

        print("save by reversal near the goal line")
        time.sleep(0.7)   # ball unseen > 1 s -> re-armed
        check(c.status()["armed"] == "1", "re-armed after ball unseen 1 s")
        c.roll(-0.05, 0.50, 0.0, -1.0, until=lambda x, y: y < 0.04)
        c.roll(-0.05, 0.04, 0.0, +0.8, until=lambda x, y: y > 0.20, rng=random.Random(2))
        check(c.status()["saves"] == "2", "ball bounced back from y<0.10 -> saves=2")

        print("goal")
        check(c.cmd("GOAL") == "OK GOAL 1", "GOAL -> OK GOAL 1")
        check(c.status()["goals"] == "1", "goals=1")

        print("human mode")
        check(c.cmd("HUMAN -0.06").startswith("OK HUMAN"), "HUMAN accepted")
        time.sleep(0.05)
        st = c.status()
        check(st["ctl"] == "HUMAN" and abs(float(st["target"]) + 0.06) < 1e-6,
              f"ctl=HUMAN target=-0.06 (got {st['target']})")
        check(c.cmd("HUMAN 5").startswith("OK HUMAN 0.11"), "HUMAN clamps to the goal")
        check(c.cmd("AUTO") == "OK AUTO" and c.status()["ctl"] == "AUTO", "AUTO")

        print("QNX vs LINUX (emulated) jitter")
        check(c.cmd("MODE QNX") == "OK QNX", "MODE QNX")
        c.cmd("RESET")
        time.sleep(1.2)
        q = c.status()
        check(c.cmd("MODE LINUX") == "OK LINUX", "MODE LINUX")
        time.sleep(1.5)
        lx = c.status()
        q_max, l_max = float(q["jit_max_us"]), float(lx["jit_max_us"])
        print(f"      QNX   jit_avg={q['jit_avg_us']}us jit_max={q['jit_max_us']}us "
              f"loop_hz={q['loop_hz']} sched={q.get('sched')}")
        print(f"      LINUX jit_avg={lx['jit_avg_us']}us jit_max={lx['jit_max_us']}us "
              f"loop_hz={lx['loop_hz']} sched={lx.get('sched')}")
        check(abs(float(q["loop_hz"]) - 200) < 20, f"QNX loop_hz ~200 ({q['loop_hz']})")
        check(lx["mode"] == "LINUX" and lx.get("emulated") == "1", "STATUS mode=LINUX emulated=1")
        check(l_max > 5000 and l_max > 3 * q_max,
              f"LINUX jit_max {l_max:.0f}us >> QNX jit_max {q_max:.0f}us")
        check(c.cmd("MODE QNX") == "OK QNX", "back to MODE QNX")

        print("reset")
        check(c.cmd("RESET") == "OK RESET", "RESET")
        time.sleep(0.02)
        st = c.status()
        check(st["saves"] == st["goals"] == st["hits"] == "0", "saves=goals=hits=0")
        check(float(st["jit_max_us"]) < l_max, "jitter stats reset")

        print("shutdown")
        c.sock.close()
        proc.send_signal(signal.SIGINT)
        rc = proc.wait(timeout=5)
        log.flush()
        out = open(log.name).read()
        check(rc == 0, "SIGINT -> exit 0")
        check("[stub] S (shutdown)" in out, "motor stopped on exit")
        check("[stub] C 1500" in out, "celebrate (C) sent on save")
        check("[stub] N 256" in out, "needle (N) moved to +2 points after two saves")
        ok = True
    except Exception as e:  # noqa: BLE001
        print(f"FAIL: {e}")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        log.flush()
        if not ok:
            print("---- server log (tail) ----")
            print("".join(open(log.name).readlines()[-40:]))
    print(f"{'PASS' if ok else 'FAIL'}: {passed} checks")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
