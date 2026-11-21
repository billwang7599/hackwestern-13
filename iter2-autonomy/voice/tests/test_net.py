#!/usr/bin/env python3
"""tars_voice network input (mission control): HEAR <text>, AUDIO <bytes> clips, WATCH events.

    make -C iter2-autonomy/voice test-net        (also part of `make test`)

Runs the REAL tars_voice host build against the REAL tars_motor host build (-b stub) and a
fake go-to controller. The mic input is raw s16le on stdin, so the "robot mic" can talk at
the same time as the laptop. Checks:
  - HEAR goes through the same classifier: "tars turn left" -> TURN -30, GOTO -> go-to.
  - a stop word acts first: S to tars_motor, then CANCEL to go-to, before the heard event,
    BEEP or speech; tars_motor's stops= goes up.
  - AUDIO (WAV or raw PCM) goes through whisper + the same path; bad formats -> ERR.
  - mic and net at the same time both act.
  - a typed stop is not held up by a clip being transcribed.
needs: macOS `say`/`afconvert`, numpy, iter2-autonomy/voice/build/tars_voice_host,
iter1-wasd/pi/build/tars_motor_host, the whisper model.
"""
from __future__ import annotations

import io
import json
import os
import socket
import subprocess
import sys
import threading
import time
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_e2e as e2e  # noqa: E402  (synth, FakeGoto, free_port, paths)

SR = 16000
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        failures += 1


def pcm16(x: np.ndarray) -> bytes:
    return (np.clip(x, -1, 1) * 32767).astype(np.int16).tobytes()


def wav_bytes(x: np.ndarray, rate: int = SR) -> bytes:
    b = io.BytesIO()
    with wave.open(b, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
        w.writeframes(pcm16(x))
    return b.getvalue()


def clip(text: str) -> np.ndarray:
    c = e2e.synth(text, e2e.available_voices()[0])
    pad = np.zeros(int(0.3 * SR), np.float32)
    return np.concatenate([pad, c / (np.abs(c).max() + 1e-9) * 0.5, pad])


class Line:
    """One TCP connection to the voice net port, line reader on a thread."""
    def __init__(self, port: int):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.lines: list[tuple[float, str]] = []
        self.cv = threading.Condition()
        threading.Thread(target=self._rd, daemon=True).start()

    def _rd(self):
        buf = b""
        while True:
            try:
                d = self.s.recv(65536)
            except OSError:
                return
            if not d:
                return
            buf += d
            while b"\n" in buf:
                l, buf = buf.split(b"\n", 1)
                with self.cv:
                    self.lines.append((time.monotonic(), l.decode(errors="replace")))
                    self.cv.notify_all()

    def send(self, data: bytes) -> None:
        self.s.sendall(data)

    def wait(self, pred, timeout: float = 10.0):
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                for t, l in self.lines:
                    if pred(l):
                        return t, l
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self.cv.wait(left)

    def reply(self, start: int, timeout: float = 15.0):
        """first OK/ERR line after index start"""
        end = time.monotonic() + timeout
        with self.cv:
            while True:
                for t, l in self.lines[start:]:
                    if l.startswith(("OK", "ERR")):
                        return t, l
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self.cv.wait(left)

    def events(self) -> list[dict]:
        out = []
        for _, l in self.lines:
            if l.startswith("{"):
                out.append(json.loads(l))
        return out

    def close(self):
        self.s.close()


def motor_status(port: int) -> dict:
    with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
        s.sendall(b"STATUS\n")
        r = s.makefile().readline()
    return dict(kv.split("=", 1) for kv in r.split() if "=" in kv)


def main() -> int:
    for p in (e2e.VOICE_BIN, e2e.MOTOR_BIN, e2e.MODEL):
        if not os.path.exists(p):
            sys.exit(f"missing {p}")
    mport, nport, go = e2e.free_port(), e2e.free_port(), e2e.FakeGoto()
    motor = subprocess.Popen([e2e.MOTOR_BIN, "-b", "stub", "-p", str(mport), "-r", "1", "-w", "5000"],
                             stderr=subprocess.DEVNULL)
    time.sleep(0.3)
    logp = os.path.join(e2e.VOICE, "build", "test_net_voice.log")
    log = open(logp, "w")
    v = subprocess.Popen([e2e.VOICE_BIN, "--input", "raw:-", "--model", e2e.MODEL, "--port", str(mport),
                          "--goto-port", str(go.port), "--net-port", str(nport), "--say-port", "0"],
                         stdin=subprocess.PIPE, stdout=log, stderr=subprocess.STDOUT)
    try:
        w = None
        for _ in range(200):
            try:
                w = Line(nport)
                break
            except OSError:
                time.sleep(0.05)
        check(w is not None, "net port open")
        n0 = len(w.lines)
        w.send(b"WATCH\n")
        check(w.reply(n0) is not None and w.reply(n0)[1] == "OK watch", "WATCH")
        c = Line(nport)

        print("HEAR text")
        n0 = len(c.lines)
        c.send(b"PING\n")
        check(c.reply(n0)[1] == "OK PONG", "PING")
        n0 = len(c.lines)
        c.send(b"HEAR tars turn left\n")
        r = c.reply(n0)
        check(r is not None and r[1].startswith("OK heard command left"), f"HEAR turn left -> {r and r[1]}")
        ev = w.wait(lambda l: '"type":"sent"' in l and '"line":"TURN -30"' in l)
        check(ev is not None and '"reply":"OK' in ev[1], "event: TURN -30 sent to tars_motor, accepted")
        n0 = len(c.lines)
        c.send(b"HEAR go to the chair\n")              # no wake word needed on push-to-talk
        r = c.reply(n0)
        check(r is not None and "command goto" in r[1], f"HEAR go to the chair -> {r and r[1]}")
        check(any(l == "GOTO chair" for l in go.lines), f"go-to got GOTO chair ({go.lines})")
        n0 = len(c.lines)
        c.send(b"HEAR tell me a joke\n")
        r = c.reply(n0)
        check(r is not None and r[1].startswith("OK heard reject"), f"nonsense -> reject ({r and r[1]})")
        n0 = len(c.lines)
        c.send(b"BOGUS\n")
        check(c.reply(n0)[1].startswith("ERR"), "unknown command -> ERR")

        print("stop acts first")
        stops0 = int(motor_status(mport)["stops"])
        w_n0 = len(w.lines)
        n0 = len(c.lines)
        c.send(b"HEAR uh please stop now\n")
        r = c.reply(n0)
        check(r is not None and r[1].startswith("OK heard stop"), f"'please stop now' -> stop ({r and r[1]})")
        w.wait(lambda l: '"type":"say"' in l and "Stopping" in l, 3)
        evs = [json.loads(l) for _, l in w.lines[w_n0:] if l.startswith("{")]
        kinds = [(e["type"], e.get("to"), e.get("line")) for e in evs]
        try:
            i_s = kinds.index(("sent", "motor", "S"))
            i_c = kinds.index(("sent", "goto", "CANCEL"))
            i_h = [k[0] for k in kinds].index("heard")
            i_b = kinds.index(("sent", "motor", "BEEP 1"))
            order_ok = i_s < i_c < i_h < i_b
        except ValueError:
            order_ok = False
        check(order_ok, f"S, then CANCEL, then the heard event, then BEEP ({kinds})")
        check(int(motor_status(mport)["stops"]) == stops0 + 1, "tars_motor stops= went up by one")
        check(go.lines and go.lines[-1] == "CANCEL", "go-to got CANCEL")
        heard = [e for e in evs if e["type"] == "heard"]
        check(bool(heard) and heard[0]["src"] == "net-text" and heard[0]["why"] == "safety",
              f"heard event: src=net-text why=safety ({heard[:1]})")

        print("AUDIO clips (whisper on the robot)")
        n0 = len(c.lines)
        data = wav_bytes(clip("Stop!"))
        c.send(f"AUDIO {len(data)}\n".encode() + data)
        r = c.reply(n0, 20)
        check(r is not None and r[1].startswith("OK heard stop"), f"WAV 'Stop!' -> stop ({r and r[1]})")
        n0 = len(c.lines)
        go_n = len(go.lines)
        data = pcm16(clip("Tars, go to the trash can."))
        c.send(f"AUDIO {len(data)}\n".encode() + data)
        r = c.reply(n0, 20)
        check(r is not None and "command goto" in r[1], f"raw PCM 'go to the trash can' -> goto ({r and r[1]})")
        check("GOTO trash can" in go.lines[go_n:], f"go-to got GOTO trash can ({go.lines[go_n:]})")
        ev = w.wait(lambda l: '"src":"net-audio"' in l and "trash" in l)
        check(ev is not None and json.loads(ev[1])["stt_ms"] > 0, "heard event src=net-audio with stt_ms")
        n0 = len(c.lines)
        data = wav_bytes(clip("Stop!")[::2], rate=8000)
        c.send(f"AUDIO {len(data)}\n".encode() + data)
        r = c.reply(n0)
        check(r is not None and r[1].startswith("ERR audio must be 16 kHz"), f"8 kHz WAV -> ERR ({r and r[1]})")
        n0 = len(c.lines)
        c.send(b"AUDIO 99999999\n")
        r = c.reply(n0)
        check(r is not None and r[1].startswith("ERR usage: AUDIO"), "oversized AUDIO -> ERR")
        n0 = len(c.lines)
        c.send(b"PING\n")
        check(c.reply(n0)[1] == "OK PONG", "connection still usable after errors")

        print("typed stop while a clip is being transcribed")
        long = np.concatenate([clip("Tars, go to the chair and then wait there for a while please.")] * 3)
        data = pcm16(long)
        a = Line(nport)
        an0 = len(a.lines)
        a.send(f"AUDIO {len(data)}\n".encode() + data)
        time.sleep(0.05)
        n0 = len(c.lines)
        t0 = time.monotonic()
        c.send(b"HEAR stop\n")
        rs = c.reply(n0)
        ra = a.reply(an0, 30)
        lat = rs[0] - t0 if rs else 99
        clip_ev = [e for e in w.events() if e.get("src") == "net-audio" and "chair" in e.get("text", "").lower()]
        stt_ms = clip_ev[-1]["stt_ms"] if clip_ev else -1
        check(rs is not None and rs[1].startswith("OK heard stop") and lat < 0.25,
              f"typed stop answered in {lat * 1000:.0f} ms (clip's whisper took {stt_ms} ms)")
        check(ra is not None and rs is not None and (stt_ms < 300 or rs[0] < ra[0]),
              "the stop didn't wait for the clip")
        a.close()

        print("robot mic and laptop at the same time")
        mic_clip = clip("Tars, turn right.")

        def play():
            sil = pcm16(np.zeros(int(0.6 * SR), np.float32))
            for chunk_src in (sil, pcm16(mic_clip), sil, sil):
                for i in range(0, len(chunk_src), 640):
                    v.stdin.write(chunk_src[i:i + 640]); v.stdin.flush()
                    time.sleep(0.02)
        th = threading.Thread(target=play)
        th.start()
        time.sleep(0.9)                                # while the mic clip is being spoken
        n0 = len(c.lines)
        c.send(b"HEAR tars go to the door\n")
        r = c.reply(n0)
        th.join()
        mic = w.wait(lambda l: '"src":"mic"' in l, 10)
        check(r is not None and "command goto" in r[1], f"laptop HEAR acted during mic speech ({r and r[1]})")
        check(mic is not None and "TURN 30" in mic[1], f"mic segment acted too ({mic and mic[1][:120]})")
        c.close()
        w.close()
    finally:
        try:
            v.stdin.close()
        except OSError:
            pass
        v.terminate()
        try:
            v.wait(10)
        except subprocess.TimeoutExpired:
            v.kill()
        log.close()
        motor.terminate()
        motor.wait(5)
        go.close()
    print(f"{checks - failures}/{checks} checks passed (voice log: {logp})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
