#!/usr/bin/env python3
"""Voice manual moves vs the REAL go-to controller and the REAL tars_motor (no fake listeners).

    make -C iter2-autonomy/voice test-goto-link      (also part of `make test`)

A voice MOVE/TURN used to go "CANCEL to goto, then MOVE/TURN to tars_motor". goto answers
CANCEL with S to tars_motor even when idle. If goto was slower than goto_reply_ms, its S landed
after the TURN and aborted it, and the late "OK" stayed on the link, so every later voice move
read a stale OK, went out too early and was aborted too. Checks, by tars_motor's own STATUS
(move=done, not aborted) and logs:
  1. idle goto: the TURN runs to the end, and goto sends no S
  2. goto stalled (SIGSTOP 0.8 s > goto_reply_ms): the TURN still runs to the end
  3. the next move after a stall runs to the end (no stale-reply desync)
  4. goto busy (GOTO trash can, searching): voice cancels it, then its TURN runs to the end,
     and goto is idle afterwards
  5. stop is unchanged: S to tars_motor first, then CANCEL to goto
needs: iter2-autonomy/voice/build/tars_voice_host, iter1-wasd/pi/build/tars_motor_host,
iter2-autonomy/goto/build/goto_host, the whisper model (loaded, not used: text input only).
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
import test_e2e as e2e  # noqa: E402  (free_port, paths)

REPO = os.path.abspath(os.path.join(e2e.VOICE, "..", ".."))
GOTO_DIR = os.path.join(REPO, "iter2-autonomy", "goto")
GOTO_BIN = os.path.join(GOTO_DIR, "build", "goto_host")
LOGS = os.path.join(e2e.VOICE, "build")
VOICE_BIN = os.environ.get("TARS_VOICE_BIN", e2e.VOICE_BIN)   # override: check an older build fails
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    print(f"  {'ok  ' if cond else 'FAIL'} {what}")
    if not cond:
        failures += 1


def line_cmd(port: int, line: str) -> str:
    with socket.create_connection(("127.0.0.1", port), timeout=5) as s:
        s.sendall((line + "\n").encode())
        return s.makefile().readline().strip()


def kv(reply: str) -> dict:
    return dict(x.split("=", 1) for x in reply.split() if "=" in x)


def read(path: str) -> str:
    with open(path, errors="replace") as f:
        return f.read()


class Stack:
    def __init__(self):
        self.mport, self.gport, self.dport, self.nport = (e2e.free_port() for _ in range(4))
        self.mlog, self.glog, self.vlog = (os.path.join(LOGS, f"test_goto_link_{n}.log") for n in ("motor", "goto", "voice"))
        self.motor = subprocess.Popen([e2e.MOTOR_BIN, "-b", "stub", "-p", str(self.mport), "-r", "0"],
                                      stdout=open(self.mlog, "w"), stderr=subprocess.STDOUT)
        time.sleep(0.3)
        self.goto = subprocess.Popen([GOTO_BIN, "-m", f"127.0.0.1:{self.mport}", "-p", str(self.gport),
                                      "-u", str(self.dport), "-n", "0", "-M", "-", "-r", "0", "-v",
                                      "-c", os.path.join(GOTO_DIR, "goto.conf"), "-o", os.path.join(GOTO_DIR, "objects.txt")],
                                     stdout=open(self.glog, "w"), stderr=subprocess.STDOUT)
        self.voice = subprocess.Popen([VOICE_BIN, "--input", "raw:-", "--model", e2e.MODEL, "--port", str(self.mport),
                                       "--goto-port", str(self.gport), "--net-port", str(self.nport), "--say-port", "0"],
                                      stdin=subprocess.PIPE, stdout=open(self.vlog, "w"), stderr=subprocess.STDOUT)
        for port in (self.gport, self.nport):
            for _ in range(150):
                try:
                    socket.create_connection(("127.0.0.1", port), timeout=1).close()
                    break
                except OSError:
                    time.sleep(0.1)
        time.sleep(0.5)

    def motor_status(self) -> dict:
        return kv(line_cmd(self.mport, "STATUS"))

    def goto_state(self) -> str:
        return kv(line_cmd(self.gport, "STATUS")).get("state", "?")

    def hear(self, text: str) -> str:
        return line_cmd(self.nport, f"HEAR {text}")

    def last_move_id(self, verb: str) -> int | None:
        """id tars_motor gave voice's last MOVE/TURN ("[motor] TURN -30 -> OK <id> <ms>")."""
        ids = re.findall(rf"\[motor\] {verb} \S+\s+-> OK (\d+) ", read(self.vlog))
        return int(ids[-1]) if ids else None

    def wait_move(self, mid: int, timeout: float = 12.0) -> str:
        """final state of tars_motor move mid: done | aborted:<why> | running (timeout)"""
        end = time.time() + timeout
        while time.time() < end:
            st = self.motor_status()
            if st.get("move_id") == str(mid) and st.get("move") != "running":
                m = re.search(rf"move {mid} (done|aborted)[^\n]*", read(self.mlog))
                return m.group(0) if m else st.get("move", "?")
            if st.get("move_id", "0").isdigit() and int(st.get("move_id", "0")) > mid:
                m = re.search(rf"move {mid} (done|aborted)[^\n]*", read(self.mlog))
                return m.group(0) if m else "replaced"
            time.sleep(0.05)
        return "running"

    def close(self):
        try:
            os.kill(self.goto.pid, signal.SIGCONT)
        except OSError:
            pass
        self.voice.stdin.close()
        for p in (self.voice, self.goto, self.motor):
            p.terminate()
        for p in (self.voice, self.goto, self.motor):
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()


def move_case(s: Stack, text: str, verb: str, what: str, stall: float = 0.0) -> None:
    n_before = s.last_move_id(verb) or 0
    if stall:
        os.kill(s.goto.pid, signal.SIGSTOP)              # goto busy/stalled longer than goto_reply_ms
        threading.Timer(stall, lambda: os.kill(s.goto.pid, signal.SIGCONT)).start()
    r = s.hear(text)
    mid, end = None, time.time() + 4                 # the HEAR reply can come before the send is logged
    while time.time() < end:
        mid = s.last_move_id(verb)
        if mid is not None and mid != n_before:
            break
        time.sleep(0.05)
    check(r.startswith("OK heard command") and mid is not None and mid != n_before, f"{what}: voice sent {verb} ({r})")
    if mid is None:
        return
    res = s.wait_move(mid)
    check(res.startswith(f"move {mid} done"), f"{what}: tars_motor ran {verb} to the end ({res})")


def main() -> int:
    for p in (VOICE_BIN, e2e.MOTOR_BIN, GOTO_BIN, e2e.MODEL):
        if not os.path.exists(p):
            sys.exit(f"missing {p}")
    s = Stack()
    try:
        print("1. idle go-to")
        g0 = read(s.glog).count("-> S")
        move_case(s, "tars turn left", "TURN", "idle")
        check(read(s.glog).count("-> S") == g0, "idle: goto sent no S to tars_motor")
        move_case(s, "tars move forward a bit", "MOVE", "idle, MOVE")

        print("2. go-to stalled 0.8 s (goto_reply_ms is 300)")
        move_case(s, "tars turn right", "TURN", "stalled", stall=0.8)
        move_case(s, "tars turn left ninety", "TURN", "stalled again", stall=0.8)

        print("3. right after the stalls")
        move_case(s, "tars turn right", "TURN", "after stall")
        move_case(s, "tars turn left", "TURN", "after stall, again")

        print("4. go-to busy")
        r = line_cmd(s.gport, "GOTO trash can")
        time.sleep(0.6)
        st = s.goto_state()
        check(r.startswith("OK") and st != "idle", f"goto busy after GOTO ({r}; state={st})")
        stops0 = int(s.motor_status().get("stops", "0"))
        move_case(s, "tars turn right", "TURN", "busy")
        check(int(s.motor_status().get("stops", "0")) > stops0, "busy: goto's cancel stopped its own motion (stops= went up)")
        check(s.goto_state() == "idle", "busy: goto idle after the voice move")

        print("5. stop unchanged: S first, then CANCEL")
        s.hear("tars move forward")
        time.sleep(0.2)
        v0 = len(read(s.vlog))
        r = s.hear("tars stop")
        check(r.startswith("OK heard stop"), f"stop heard ({r})")
        time.sleep(0.5)
        tail = read(s.vlog)[v0:]
        i_s, i_c = tail.find("[motor] S "), tail.find("[goto] CANCEL")
        check(0 <= i_s < i_c, "stop: S to tars_motor before CANCEL to goto")
        time.sleep(0.3)
        check(s.motor_status().get("move") != "running", "stop: tars_motor not moving")
    finally:
        s.close()
    print(f"{checks - failures}/{checks} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
