#!/usr/bin/env python3
"""tars_voice --input phone:HOST:PORT (the iPhone on the car streams the AirPods mic).

    make -C iter2-autonomy/voice test-phone        (also part of `make test`)

A fake "phone" listens on a free port and speaks the MIC1 protocol (audio.h): header line
"MIC1 16000 1 s16le", then packets <type><len16 LE><payload>: 'A' audio, 'M' mute, 'U' unmute,
'K' keepalive. The REAL tars_voice host build connects to it, exactly as it connects to the
phone through iproxy on the Pi, and drives the REAL tars_motor host build (-b stub) and a fake
go-to controller. Speech is macOS `say` -> 16 kHz WAV (tests/clips cache, shared with test_e2e).

Checks:
  - hands-free: "Tars, go to the trash can." -> GOTO trash can; wake word required
    (no "Tars" -> nothing sent); "Tars, stop." -> S first, then CANCEL, stops= +1.
  - mute: audio sent between 'M' and 'U' is never heard (half duplex: TARS talking).
    Speech before the mute is still heard (segment flushed at the mute).
  - the phone hanging up mid-sentence -> tars_voice reconnects and the next command works.
  - phone app gone for a while (port closed) -> reconnects when it comes back.
  - silent link (header, then nothing) -> dropped after ~3 s and redialled.
  - garbage: wrong header, unknown packet type, odd/oversize length, binary noise ->
    the link is dropped and redialled, tars_voice keeps running, the next command works.
  - the net port (typed "stop") stays responsive while the phone link is down.
needs: macOS say/afconvert, numpy, build/tars_voice_host, iter1-wasd/pi/build/tars_motor_host, the model.
"""
from __future__ import annotations

import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_e2e as e2e  # noqa: E402  (synth, FakeGoto, free_port, paths)
from test_net import Line, motor_status  # noqa: E402

SR = 16000
HEADER = b"MIC1 16000 1 s16le\n"
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        failures += 1


def pcm(x: np.ndarray) -> bytes:
    return (np.clip(x, -1, 1) * 32767).astype(np.int16).tobytes()


def speech(text: str) -> bytes:
    c = e2e.synth(text, e2e.available_voices()[0])
    return pcm(c / (np.abs(c).max() + 1e-9) * 0.5)


def silence(sec: float) -> bytes:
    # a little noise, like a real mic (keeps the VAD's noise floor sane)
    rng = np.random.default_rng(1)
    return pcm(rng.normal(0, 0.001, int(sec * SR)).astype(np.float32))


def pkt(kind: bytes, payload: bytes = b"") -> bytes:
    return kind + struct.pack("<H", len(payload)) + payload


class FakePhone:
    """The phone app's mic stream listener (MIC1). One connection at a time, like the app."""
    def __init__(self, port: int | None = None):
        self.port = port or e2e.free_port()
        self.accepts = 0
        self.conn: socket.socket | None = None
        self.cv = threading.Condition()
        self.srv: socket.socket | None = None
        self.open()

    def open(self):
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(2)
        threading.Thread(target=self._run, args=(self.srv,), daemon=True).start()

    def _run(self, srv):
        while True:
            try:
                c, _ = srv.accept()
            except OSError:
                return
            c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.cv:
                if self.conn:
                    try:
                        self.conn.close()
                    except OSError:
                        pass
                self.conn = c
                self.accepts += 1
                self.cv.notify_all()

    def wait_conn(self, n: int, timeout: float = 10) -> bool:
        """wait until the n-th accept"""
        end = time.monotonic() + timeout
        with self.cv:
            while self.accepts < n:
                left = end - time.monotonic()
                if left <= 0:
                    return False
                self.cv.wait(left)
        return True

    def send(self, data: bytes) -> bool:
        try:
            self.conn.sendall(data)
            return True
        except OSError:
            return False

    def stream(self, audio: bytes, speed: float = 4.0, chunk: int = 640) -> bool:
        """send audio as 'A' packets of 20 ms, `speed` x real time"""
        for i in range(0, len(audio), chunk):
            if not self.send(pkt(b"A", audio[i:i + chunk])):
                return False
            time.sleep(chunk / 2 / SR / speed)
        return True

    def hang_up(self):
        with self.cv:
            if self.conn:
                self.conn.close()
                self.conn = None

    def close_listener(self):
        self.srv.close()
        self.hang_up()


def peer_closed(s: socket.socket | None, timeout: float) -> bool:
    """True if the other side (tars_voice) closed this connection within timeout"""
    if s is None:
        return True
    s.settimeout(timeout)
    try:
        return s.recv(1) == b""
    except socket.timeout:
        return False
    except OSError:
        return True


def main() -> int:
    for p in (e2e.VOICE_BIN, e2e.MOTOR_BIN, e2e.MODEL):
        if not os.path.exists(p):
            sys.exit(f"missing {p}")
    mport, nport, go, phone = e2e.free_port(), e2e.free_port(), e2e.FakeGoto(), FakePhone()
    motor = subprocess.Popen([e2e.MOTOR_BIN, "-b", "stub", "-p", str(mport), "-r", "1", "-w", "5000"],
                             stderr=subprocess.DEVNULL)
    time.sleep(0.3)
    logp = os.path.join(e2e.VOICE, "build", "test_phone_voice.log")
    log = open(logp, "w")
    v = subprocess.Popen([e2e.VOICE_BIN, "--input", f"phone:127.0.0.1:{phone.port}", "--model", e2e.MODEL,
                          "--port", str(mport), "--goto-port", str(go.port), "--net-port", str(nport),
                          "--say-port", "0", "--button-talk", "0"],
                         stdout=log, stderr=subprocess.STDOUT)
    w = None
    try:
        for _ in range(200):
            try:
                w = Line(nport)
                break
            except OSError:
                time.sleep(0.05)
        check(w is not None, "voice net port open")
        w.s.settimeout(None)                     # quiet stretches (mute, link down) are > 5 s
        n0 = len(w.lines)
        w.send(b"WATCH\n")
        w.reply(n0)

        def heard(pred, timeout=15):
            return w.wait(lambda l: '"type":"heard"' in l and '"src":"mic"' in l and pred(l), timeout)

        def sent_lines(start: int) -> list[str]:
            evs = [json.loads(l) for _, l in w.lines[start:] if l.startswith("{")]
            return [e.get("line", "") for e in evs if e.get("type") == "sent"]

        print("connect + hands-free command")
        check(phone.wait_conn(1), "tars_voice dialled the phone")
        phone.send(HEADER + pkt(b"U"))
        go_n = len(go.lines)
        phone.stream(silence(1.0) + speech("Tars, go to the trash can.") + silence(1.2))
        ev = heard(lambda l: "trash" in l.lower())
        check(ev is not None and '"intent":"goto"' in ev[1], f"heard via phone -> goto ({ev and ev[1][:140]})")
        time.sleep(0.3)
        check("GOTO trash can" in go.lines[go_n:], f"go-to got GOTO trash can ({go.lines[go_n:]})")

        print("wake word required (hands-free)")
        go_n, w_n = len(go.lines), len(w.lines)
        phone.stream(speech("Go to the door.") + silence(1.2))
        ev = heard(lambda l: "door" in l.lower(), 10)
        check(ev is not None and '"action":"ignore"' in ev[1], f"no 'Tars' -> ignored ({ev and ev[1][:140]})")
        check(not any(l.startswith("GOTO") for l in go.lines[go_n:]), "nothing sent to go-to without the wake word")

        print("stop through the phone mic")
        stops0 = int(motor_status(mport)["stops"])
        w_n = len(w.lines)
        phone.stream(speech("Tars, stop.") + silence(1.2))
        ev = heard(lambda l: '"action":"stop"' in l)
        check(ev is not None, f"'Tars, stop.' -> stop ({ev and ev[1][:140]})")
        time.sleep(0.3)
        sl = sent_lines(w_n)
        check("S" in sl and "CANCEL" in sl and sl.index("S") < sl.index("CANCEL"), f"S then CANCEL ({sl})")
        check(int(motor_status(mport)["stops"]) == stops0 + 1, "tars_motor stops= +1")

        print("mute (phone talking): audio between M and U is never heard")
        w_n = len(w.lines)
        phone.send(pkt(b"M"))
        phone.stream(speech("Tars, turn left.") + silence(0.6))     # TARS's own voice leaking in
        phone.send(pkt(b"K"))
        phone.send(pkt(b"U"))
        phone.stream(silence(1.0))
        time.sleep(1.5)
        lefts = [l for _, l in w.lines[w_n:] if '"src":"mic"' in l and "left" in l.lower()]
        check(not lefts, f"muted speech not heard ({lefts[:1]})")
        phone.stream(speech("Tars, turn right.") + silence(1.2))
        ev = heard(lambda l: "right" in l.lower())
        check(ev is not None and "TURN 30" in ev[1], f"unmuted -> TURN 30 ({ev and ev[1][:140]})")

        print("speech right before a mute is still heard (segment flushed)")
        w_n = len(w.lines)
        phone.stream(silence(0.5) + speech("Tars, turn left."))
        phone.send(pkt(b"M"))                                        # no trailing silence
        ev = heard(lambda l: "left" in l.lower())
        check(ev is not None and "TURN -30" in ev[1], f"pre-mute speech acted ({ev and ev[1][:140]})")
        phone.send(pkt(b"U"))

        print("phone hangs up mid-sentence -> reconnect")
        n_acc = phone.accepts
        half = speech("Tars, go to the chair please.")
        phone.stream(silence(0.5) + half[: len(half) // 2])
        phone.hang_up()
        check(phone.wait_conn(n_acc + 1, 10), "tars_voice redialled after the hang-up")
        phone.send(HEADER)
        go_n = len(go.lines)
        phone.stream(silence(1.0) + speech("Tars, go to the backpack.") + silence(1.2))
        heard(lambda l: "backpack" in l.lower())
        time.sleep(0.3)
        check("GOTO backpack" in go.lines[go_n:], f"command after reconnect ({go.lines[go_n:]})")

        print("phone app gone for a while (port closed), net input still answers")
        n_acc = phone.accepts
        phone.close_listener()
        time.sleep(2.5)
        c = Line(nport)
        n0 = len(c.lines)
        t0 = time.monotonic()
        c.send(b"HEAR stop\n")
        r = c.reply(n0, 5)
        check(r is not None and r[1].startswith("OK heard stop") and r[0] - t0 < 0.5,
              f"typed stop answered while the phone is gone ({r and r[1]}, {(r[0] - t0) * 1000 if r else -1:.0f} ms)")
        c.close()
        check(v.poll() is None, "tars_voice still running")
        phone.open()
        check(phone.wait_conn(n_acc + 1, 10), "redialled when the phone came back (<= 5 s backoff)")
        phone.send(HEADER)
        go_n = len(go.lines)
        phone.stream(silence(1.0) + speech("Tars, go to the trash can.") + silence(1.2))
        heard(lambda l: "trash" in l.lower())
        time.sleep(0.3)
        check("GOTO trash can" in go.lines[go_n:], f"command after the phone came back ({go.lines[go_n:]})")

        print("silent link (no keepalive) is dropped and redialled")
        n_acc = phone.accepts
        conn = phone.conn
        t0 = time.monotonic()
        closed = peer_closed(conn, 6)
        dt = time.monotonic() - t0
        check(closed and 2.0 < dt < 5.0, f"dropped after {dt:.1f} s of silence")
        check(phone.wait_conn(n_acc + 1, 10), "redialled after the silent link")
        phone.send(HEADER)
        conn = phone.conn
        for _ in range(4):                       # keepalives keep a quiet link alive
            phone.send(pkt(b"K"))
            time.sleep(1.0)
        check(not peer_closed(conn, 0.2) and phone.accepts == n_acc + 1, "keepalives keep the link up for 4 s")

        print("garbage from the phone")
        bad = [
            ("wrong header", b"HELLO 44100 2 f32\n"),
            ("binary noise", bytes(np.random.default_rng(7).integers(0, 256, 3000, dtype=np.uint8))),
            ("unknown packet type", HEADER + pkt(b"Z", b"abcd")),
            ("odd audio length", HEADER + pkt(b"A", b"abc")),
            ("oversize audio packet", HEADER + b"A" + struct.pack("<H", 60000) + b"\x00" * 100),
            ("mute with a payload", HEADER + pkt(b"M", b"xx")),
            ("header line too long", b"M" * 200),
        ]
        for name, data in bad:
            n_acc = phone.accepts
            conn = phone.conn
            phone.send(data)
            closed = peer_closed(conn, 4)
            check(closed and v.poll() is None, f"{name}: link dropped, tars_voice alive")
            check(phone.wait_conn(n_acc + 1, 10), f"{name}: redialled")
        phone.send(HEADER)
        go_n = len(go.lines)
        # split packets across sends at awkward byte boundaries
        audio = silence(1.0) + speech("Tars, go to the cup.") + silence(1.2)
        stream = b"".join(pkt(b"A", audio[i:i + 1000]) for i in range(0, len(audio), 1000))
        for i in range(0, len(stream), 777):
            phone.send(stream[i:i + 777])
            time.sleep(777 / 2 / SR / 4)
        heard(lambda l: "cup" in l.lower())
        time.sleep(0.3)
        check("GOTO cup" in go.lines[go_n:], f"command after the garbage, odd TCP splits ({go.lines[go_n:]})")

        print("CRLF header (e.g. typed by hand with nc) is accepted")
        n_acc = phone.accepts
        phone.hang_up()
        check(phone.wait_conn(n_acc + 1, 10), "redialled")
        phone.send(b"MIC1 16000 1 s16le\r\n")
        w_n = len(w.lines)
        phone.stream(silence(1.0) + speech("Tars, turn left.") + silence(1.2))
        ev = heard(lambda l: "left" in l.lower())
        check(ev is not None and "TURN -30" in ev[1], f"CRLF header stream -> TURN -30 ({ev and ev[1][:120]})")
    finally:
        if w:
            w.close()
        v.terminate()
        try:
            v.wait(10)
        except subprocess.TimeoutExpired:
            v.kill()
        log.close()
        motor.terminate()
        motor.wait(5)
        go.close()
        try:
            phone.close_listener()
        except OSError:
            pass
    print(f"{checks - failures}/{checks} checks passed (voice log: {logp})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
