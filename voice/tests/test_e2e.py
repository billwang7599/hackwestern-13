#!/usr/bin/env python3
"""End-to-end voice test on a Mac: speech audio -> VAD -> whisper.cpp (tiny.en-q5_1)
-> intent -> the REAL tars_motor (host build, -b stub) over TCP, and a fake
go-to controller (records GOTO/CANCEL lines, like goto/ will on TCP 5570).

    make -C voice e2e

Clips are synthesised with macOS `say` (two voices), converted to 16 kHz with
`afconvert`, and joined into one stream with gaps, like someone talking to the
robot. The stream is run clean, with white noise at 20 dB SNR, and with
"crowd" (other speech + noise) at 10 dB SNR. tars_voice reads it as a WAV
(as fast as it can; VAD timing is by sample count, so results are repeatable).

Checks: every stop clip ends in S (+ CANCEL to go-to) in clean and white-noise
runs; no WRONG motion in clean runs; clean accuracy >= 85%; tars_motor
accepted every command (OK replies); the go-to controller got exactly the
GOTO lines. Crowd noise at 10 dB SNR is reported, not required. Prints latency.
needs: macOS (say, afconvert), numpy, voice/build/tars_voice_host, pi/build/tars_motor_host.
"""
from __future__ import annotations

import os
import re
import socket
import subprocess
import sys
import threading
import time
import wave

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
VOICE = os.path.dirname(HERE)
REPO = os.path.dirname(VOICE)
CLIPS = os.path.join(HERE, "clips")
VOICE_BIN = os.path.join(VOICE, "build", "tars_voice_host")
MOTOR_BIN = os.path.join(REPO, "pi", "build", "tars_motor_host")
MODEL = os.path.join(VOICE, "models", "ggml-tiny.en-q5_1.bin")
SR = 16000

# (spoken text, expected first command line; None = nothing; "REJECT" = LED no)
CASES = [
    ("Tars, go forward.", "MOVE 300"),
    ("Tars, turn left.", "TURN -30"),
    ("Tars, turn right ninety degrees.", "TURN 90"),
    ("Tars, go back half a meter.", "MOVE -500"),
    ("Tars, turn around.", "TURN 180"),
    ("Tars, stop!", "S"),
    ("Stop!", "S"),
    ("Tars, go to the trash can.", "GOTO trash can"),
    ("Tars, drive to the chair.", "GOTO chair"),
    ("Tars, find the door.", "GOTO door"),
    ("Tars, head over to that bin.", "GOTO trash can"),
    ("Tars, do a spin.", "TURN 360"),
    ("Tars, status report.", "STATUS"),
    ("Tars, move forward two steps.", "MOVE 200"),
    ("Tars, turn left forty five degrees.", "TURN -45"),
    ("Freeze!", "S"),
    ("Tars, hang a right.", "TURN 30"),
    ("Tars, back up.", "MOVE -300"),
    ("Nice robot. What does it do?", None),
    ("Turn left.", None),                       # no wake word
    ("Tars, tell me a joke.", "REJECT"),
]
VOICES = ["Samantha", "Daniel"]
CROWD = ["I think the demo is over there.", "Did you get any pizza yet?",
         "This hackathon is so much fun.", "Where is the hardware table?"]
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def available_voices() -> list[str]:
    out = subprocess.run(["say", "-v", "?"], capture_output=True, text=True).stdout
    have = {l.split()[0] for l in out.splitlines() if l.strip()}
    v = [x for x in VOICES if x in have]
    return v or [""]


def synth(text: str, voice: str) -> np.ndarray:
    os.makedirs(CLIPS, exist_ok=True)
    key = re.sub(r"[^a-z0-9]+", "_", f"{voice}_{text}".lower()).strip("_")[:60]
    wav = os.path.join(CLIPS, key + ".wav")
    if not os.path.exists(wav):
        aiff = os.path.join(CLIPS, key + ".aiff")
        cmd = ["say", "-o", aiff] + (["-v", voice] if voice else []) + [text]
        subprocess.run(cmd, check=True)
        subprocess.run(["afconvert", "-f", "WAVE", "-d", "LEI16@16000", "-c", "1", aiff, wav], check=True)
        os.remove(aiff)
    with wave.open(wav) as w:
        assert w.getframerate() == SR and w.getnchannels() == 1
        return np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768


def write_wav(path: str, x: np.ndarray) -> None:
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(SR)
        w.writeframes((np.clip(x, -1, 1) * 32767).astype(np.int16).tobytes())


def build_stream(voice: str, noise: str, snr_db: float, rng) -> tuple[np.ndarray, list]:
    """Clips with 1.2 s gaps. Returns audio and [(start_s, end_s, expected)]."""
    parts, marks, t = [np.zeros(int(1.0 * SR), np.float32)], [], 1.0
    for text, want in CASES:
        c = synth(text, voice)
        c = c / (np.abs(c).max() + 1e-9) * 0.5
        marks.append((t, t + len(c) / SR, text, want))
        parts += [c, np.zeros(int(1.2 * SR), np.float32)]
        t += len(c) / SR + 1.2
    x = np.concatenate(parts)
    if noise != "clean":
        speech_p = np.mean(np.concatenate([synth(tx, voice) for tx, _ in CASES[:3]]) ** 2) * (0.5 / 0.3) ** 2
        n = rng.normal(0, 1, len(x)).astype(np.float32)
        if noise == "crowd":
            crowd = np.concatenate([synth(s, VOICES[-1] if voice == VOICES[0] else VOICES[0]) for s in CROWD])
            crowd = np.resize(crowd, len(x)) * 0.6
            n = 0.5 * n * np.sqrt(np.mean(crowd ** 2)) + crowd
        n *= np.sqrt(speech_p / (np.mean(n ** 2) + 1e-12) / 10 ** (snr_db / 10))
        x = x + n
    return x, marks


def free_port() -> int:
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


class FakeGoto:
    """Stands in for the go-to controller: accepts one client, records lines, replies OK."""
    def __init__(self):
        self.port = free_port()
        self.lines: list[str] = []
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(2)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            f = c.makefile("rw", encoding="ascii", newline="\n")
            for line in f:
                self.lines.append(line.strip())
                f.write("OK\n"); f.flush()

    def close(self):
        self.srv.close()


def run_condition(name: str, wav: str, marks) -> dict:
    port = free_port()
    go = FakeGoto()
    motor = subprocess.Popen([MOTOR_BIN, "-b", "stub", "-p", str(port), "-r", "1", "-w", "5000"],
                             stderr=subprocess.PIPE, text=True)
    time.sleep(0.3)
    t0 = time.monotonic()
    r = subprocess.run([VOICE_BIN, "--input", f"wav:{wav}", "--model", MODEL, "--port", str(port),
                        "--goto-port", str(go.port), "--threads", "2"], capture_output=True, text=True, timeout=600)
    time.sleep(0.1)
    go.close()
    wall = time.monotonic() - t0
    motor.terminate()
    mlog = motor.communicate(timeout=5)[1]
    out = r.stdout
    heard = []                                   # (at_s, len_s, stt_ms, text, lines, block_of_cmds)
    cur = None
    for line in out.splitlines():
        m = re.match(r'\[heard\] "(.*)" at=([\d.]+)s len=([\d.]+)s stt=(\d+)ms -> (\w+) (\S+) ([\d.]+) \((\w*)\)(?:: (.*))?$', line)
        if m:
            cur = {"text": m[1], "at": float(m[2]), "len": float(m[3]), "stt": int(m[4]), "action": m[5],
                   "lines": (m[9] or "").split(" | ") if m[9] else [], "replies": []}
            heard.append(cur)
        m = re.match(r"\[(motor|goto)\] (.*?)\s+-> (.*) \(\d+ ms\)$", line)
        if m and cur is not None:
            cur["replies"].append((m[1], m[2], m[3]))
    good = wrong_motion = missed = 0
    rows = []
    def cmd_of(lines):
        for l in lines:
            if l != "@goto CANCEL":
                return l[6:] if l.startswith("@goto ") else l
        return None

    for start, end, text, want in marks:
        segs = [h for h in heard if start - 0.1 <= h["at"] - h["len"] + 0.3 and h["at"] <= end + 1.2]
        cmds = [cmd_of(s["lines"]) for s in segs if cmd_of(s["lines"])]
        got = cmds[0] if cmds else None
        acts = [s["action"] for s in segs]
        if want == "REJECT":
            ok = got in ("LED no", None) and not any(c.startswith(("MOVE", "TURN", "GOTO")) for c in cmds)
        else:
            ok = got == want
        motion = [c for c in cmds if c.startswith(("MOVE", "TURN", "GOTO"))]
        if not ok and motion and (want is None or want == "REJECT" or motion[0] != want):
            wrong_motion += 1
        if want == "S" and got != "S" and "crowd" not in name:
            check(False, f"[{name}] stop clip {text!r} did not stop (heard {[s['text'] for s in segs]})")
        if want == "S" and got == "S":
            check(any("@goto CANCEL" in s["lines"] for s in segs), f"[{name}] stop also cancels go-to")
        good += ok
        if not ok and want not in (None, "REJECT") and not motion:
            missed += 1
        rows.append((text, want, got, [s["text"] for s in segs], [s["stt"] for s in segs], ok))
    replies = [rp for h in heard for rp in h["replies"]]
    bad_replies = [rp for rp in replies if not rp[2].startswith("OK")]
    goto_sent = [rp[1] for rp in replies if rp[0] == "goto"]
    stt = [h["stt"] for h in heard]
    audio_s = sum(h["len"] for h in heard)
    return {"name": name, "rows": rows, "good": good, "n": len(marks), "wrong_motion": wrong_motion,
            "missed": missed, "bad_replies": bad_replies, "replies": len(replies), "stt": stt,
            "audio_s": audio_s, "wall": wall, "rc": r.returncode, "stderr": r.stderr, "mlog": mlog,
            "goto_got": list(go.lines), "goto_sent": goto_sent}


def test_talk_button(voice: str) -> None:
    """Real-time stream on stdin (raw:-): no wake word -> ignored; Uno button
    press (SIM BTN TEACH on the stub) -> the next command works without it."""
    print("talk button: real-time raw stdin, SIM BTN TEACH")
    port, go = free_port(), FakeGoto()
    motor = subprocess.Popen([MOTOR_BIN, "-b", "stub", "-p", str(port), "-r", "1", "-w", "5000"],
                             stderr=subprocess.PIPE, text=True)
    time.sleep(0.3)
    v = subprocess.Popen([VOICE_BIN, "--input", "raw:-", "--model", MODEL, "--port", str(port),
                          "--goto-port", str(go.port)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.DEVNULL, text=False)
    clip = synth("Turn left.", voice)
    clip = clip / (np.abs(clip).max() + 1e-9) * 0.5

    def play(x):                       # real-time pacing, 20 ms chunks
        pcm = (np.clip(x, -1, 1) * 32767).astype(np.int16).tobytes()
        for i in range(0, len(pcm), 640):
            v.stdin.write(pcm[i:i + 640]); v.stdin.flush()
            time.sleep(0.02)
    sil = np.zeros(int(1.0 * SR), np.float32)
    play(sil); play(clip); play(sil)                     # no wake word: must be ignored
    c = socket.create_connection(("127.0.0.1", port))
    c.sendall(b"SIM BTN TEACH\n"); c.recv(100)
    play(np.zeros(int(0.6 * SR), np.float32))            # > 250 ms poll
    play(clip); play(sil)
    v.stdin.close()
    out = v.stdout.read().decode()
    v.wait(30); c.close(); motor.terminate(); motor.wait(5); go.close()
    heard = [l for l in out.splitlines() if l.startswith("[heard]")]
    check(len(heard) == 2, f"two segments heard: {heard}")
    check(bool(heard) and "no_wake" in heard[0], f"without button: ignored ({heard[:1]})")
    check("[talk] button pressed" in out, "button press seen via STATUS btn_teach")
    check(len(heard) == 2 and "TURN -30" in heard[1], f"after button: TURN -30 ({heard[1:]})")


def main() -> int:
    for p in (VOICE_BIN, MOTOR_BIN, MODEL):
        if not os.path.exists(p):
            sys.exit(f"missing {p} (make -C voice host, make -C pi host, voice/get_model.sh)")
    rng = np.random.default_rng(7)
    voices = available_voices()
    results = []
    for voice in voices:
        for noise, snr in (("clean", 0), ("white", 20), ("crowd", 15), ("crowd", 10)):
            x, marks = build_stream(voice, noise, snr, rng)
            wav = os.path.join(CLIPS, f"stream_{voice or 'default'}_{noise}.wav")
            write_wav(wav, x)
            res = run_condition(f"{voice or 'default'}/{noise}" + (f"@{snr}dB" if noise != "clean" else ""), wav, marks)
            results.append(res)
            check(res["rc"] == 0, f"[{res['name']}] tars_voice exit {res['rc']}: {res['stderr'][-300:]}")
            print(f"{res['name']:<22} {res['good']}/{res['n']} correct, wrong motion {res['wrong_motion']}, "
                  f"missed {res['missed']}, tars_motor replies OK {res['replies'] - len(res['bad_replies'])}/{res['replies']}, "
                  f"whisper mean {np.mean(res['stt']):.0f} ms max {max(res['stt'])} ms over {len(res['stt'])} segments")
            for text, want, got, heard, stt, ok in res["rows"]:
                if not ok:
                    print(f"    miss: {text!r} want {want} got {got}; heard {heard}")
            if res["name"].endswith("clean"):
                check(res["wrong_motion"] == 0, f"[{res['name']}] no wrong motion")
            check(res["goto_got"] == res["goto_sent"], f"[{res['name']}] go-to controller got {res['goto_got']} vs sent {res['goto_sent']}")
            check(not res["bad_replies"], f"[{res['name']}] tars_motor rejected {res['bad_replies']}")
            if res["name"].endswith("clean"):
                check(res["good"] >= 0.85 * res["n"], f"[{res['name']}] clean accuracy >= 85% ({res['good']}/{res['n']})")
            check(re.search(r"client \d+ is 'voice'", res["mlog"]) is not None, f"[{res['name']}] voice identified itself to tars_motor")
    test_talk_button(voices[0])
    stt = [s for r in results for s in r["stt"]]
    seg = [r["audio_s"] / max(1, len(r["stt"])) for r in results]
    print(f"\nwhisper tiny.en-q5_1, 2 threads, audio_ctx auto (512), this Mac: mean {np.mean(stt):.0f} ms, "
          f"p90 {np.percentile(stt, 90):.0f} ms, max {max(stt)} ms per segment (avg segment {np.mean(seg):.2f} s)")
    print(f"{checks - failures}/{checks} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
