"""Voice lab: text or laptop-mic speech through the robot's REAL whisper + C intent
classifier, run on the laptop, showing what it decided and the exact lines it would send.

Laptop only (bench/README.md "Voice lab"). Nothing here runs on the robot.

Where each number comes from (no robot code is copied or changed):
  decision      iter2-autonomy/voice/build/intent_cli   (the C classifier, same intent.c as QNX)
  top-3, slots  iter2-autonomy/voice/intent/intent.py   (Python reference that parses the same
                intent_model.h; README: 2863/2863 parity with C). Its top-1 prob is checked
                against intent_cli's and a mismatch is shown.
  exact lines   a "shadow" tars_voice_host (same main.c as the robot) whose tars_motor, go-to
                and phone-speech links point at fake listeners on free ports. We send it
                "HEAR <text>" on its own net port and record what reaches the fakes, in order.
                The fakes reply "OK" (MOVE/TURN: "OK 0 -1", so no keepalive PINGs).
  transcript    iter2-autonomy/voice/build/stt_cli MODEL clip.wav (the mission --laptop-stt path)

Send switch (off by default = dry run): when on, text goes to the target's tars_voice as
"HEAR <text>" and audio as "AUDIO <n>" + bytes, through the robot plugin (the only owner of
robot connections). If the robot plugin can't send binary data, audio falls back to laptop
whisper + HEAR <transcript> (like mission --laptop-stt) and says so.
"""
from __future__ import annotations

import array
import hashlib
import io
import json
import math
import os
import random
import re
import shutil
import socket
import statistics
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import wave

SR = 16000
ACTS = ("ignore", "stop", "command", "reject")
GOTO_PREFIX = "@goto "

_bench = None
_paths: dict = {}
_state = {"send": False}
_shadow = None
_shadow_lock = threading.Lock()
_ref = None            # (intent module, model) or None
_ref_err = ""
_batch = {"thread": None, "cancel": False}
_cache_dir = os.path.join(tempfile.gettempdir(), "tars-bench-voice")


# ------------------------------------------------------------------ paths and model info
def _setup_paths(repo: str) -> None:
    v = os.path.join(repo, "iter2-autonomy", "voice")
    _paths.update(
        voice=v,
        intent_dir=os.path.join(v, "intent"),
        intent_cli=os.path.join(v, "build", "intent_cli"),
        stt_cli=os.path.join(v, "build", "stt_cli"),
        voice_bin=os.path.join(v, "build", "tars_voice_host"),
        model=os.environ.get("BENCH_WHISPER_MODEL", os.path.join(v, "models", "ggml-tiny.en-q5_1.bin")),
        header=os.path.join(v, "intent", "intent_model.h"),
        phrases=os.path.join(v, "intent", "phrases.txt"),
        realistic=os.path.join(v, "intent", "data", "realistic.tsv"),
        e2e=os.path.join(v, "tests", "test_e2e.py"),
    )


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


_info_cache: dict = {}


def info() -> dict:
    """Which binaries and model files are loaded (shown in the panel header)."""
    if not _info_cache:
        out = {}
        m = _paths["model"]
        out["whisper_model"] = {"file": os.path.basename(m), "path": m, "present": os.path.exists(m)}
        if os.path.exists(m):
            out["whisper_model"].update(bytes=os.path.getsize(m), sha256=_sha256(m)[:16])
        h = _paths["header"]
        head = {"file": "intent_model.h", "present": os.path.exists(h)}
        if os.path.exists(h):
            with open(h) as f:
                src = f.read()
            head["sha256"] = hashlib.sha256(src.encode()).hexdigest()[:16]
            head["comment"] = " ".join(l.strip(" /*") for l in src.split("*/")[0].splitlines()[1:]).strip()
            t = re.search(r"#define INTENT_THRESHOLD ([\d.]+)f", src)
            st = re.search(r"#define INTENT_STOP_THRESHOLD ([\d.]+)f", src)
            head["threshold"] = float(t.group(1)) if t else None
            head["stop_threshold"] = float(st.group(1)) if st else None
            cli = _paths["intent_cli"]
            head["intent_cli_stale"] = os.path.exists(cli) and os.path.getmtime(cli) < os.path.getmtime(h)
        out["classifier"] = head
        out["bins"] = {k: os.path.exists(_paths[k]) for k in ("intent_cli", "stt_cli", "voice_bin")}
        out["reference"] = "intent.py loaded" if _ref else f"intent.py unavailable: {_ref_err}"
        _info_cache.update(out)
    d = dict(_info_cache)
    d["send"] = _state["send"]
    d["robot_plugin"] = _robot() is not None
    d["target"] = _bench.target if _bench else {}
    d["shadow"] = _shadow.describe() if _shadow else "not started (starts on first use)"
    d["say"] = shutil.which("say") is not None and shutil.which("afconvert") is not None
    return d


# ------------------------------------------------------------------ the C classifier
def cli_decide(texts: list[str], require_wake: bool) -> list[dict]:
    """Run the real intent_cli on many lines at once. One dict per line."""
    if not texts:
        return []
    clean = [t.replace("\n", " ").replace("\r", " ") for t in texts]
    args = [_paths["intent_cli"]] + ([] if require_wake else ["--any"])
    p = subprocess.run(args, input="\n".join(clean) + "\n", capture_output=True, text=True, timeout=60)
    if p.returncode != 0:
        raise RuntimeError(f"intent_cli failed: {p.stderr.strip()[-300:]}")
    out = []
    for line in p.stdout.splitlines():
        f = line.split("\t")
        while len(f) < 6:
            f.append("")
        lines = [x for x in f[5].split(" | ") if x] if f[5] else []
        out.append({"action": f[0], "intent": f[1], "prob": float(f[2] or 0), "wake": f[3] == "1",
                    "why": f[4], "lines": lines})
    if len(out) != len(clean):
        raise RuntimeError(f"intent_cli gave {len(out)} lines for {len(clean)} inputs")
    return out


def _load_ref() -> None:
    global _ref, _ref_err
    try:
        sys.path.insert(0, _paths["intent_dir"])
        import intent as I  # noqa: E402  (the reference spec next to the C code)
        _ref = (I, I.load_model(_paths["header"]))
    except Exception as e:  # the C path still works without it
        _ref, _ref_err = None, f"{type(e).__name__}: {e}"
    finally:
        if sys.path and sys.path[0] == _paths["intent_dir"]:
            sys.path.pop(0)


def detail(text: str) -> dict:
    """Top-3, slots and wake handling from the Python reference (same weights as C)."""
    if not _ref:
        return {"error": _ref_err}
    I, m = _ref
    toks = I.normalize(text)
    wake, rest = I.strip_wake(toks)
    d = {"tokens": toks, "wake_word": wake, "rest": rest, "safety_stop": I.safety_stop(rest, wake)}
    if rest:
        p = I.predict(m, rest)
        order = sorted(range(len(p)), key=lambda k: -p[k])
        d["top3"] = [[I.INTENTS[k], round(p[k], 4)] for k in order[:3]]
        d["features"] = " ".join(I.feature_tokens(rest))
    val, unit = I.parse_amount(rest)
    d["amount"] = None if val is None else {"value": val, "unit": unit,
                                            "move_mm": I.move_mm(val, unit), "turn_deg": I.turn_deg(val, unit)}
    d["object"] = I.find_object(m.objects, rest)
    d["threshold"] = m.threshold
    return d


# ------------------------------------------------------------------ shadow tars_voice
class _Fake:
    """A listener standing in for tars_motor / go-to / phone speech. Records every line
    (with a shared sequence number, so the order across the three is kept) and replies."""

    def __init__(self, name: str, rec, reply):
        self.name, self.rec, self.reply = name, rec, reply
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(8)
        self.port = self.srv.getsockname()[1]
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            threading.Thread(target=self._client, args=(c,), daemon=True).start()

    def _client(self, c):
        buf = b""
        with c:
            while True:
                try:
                    d = c.recv(4096)
                except OSError:
                    return
                if not d:
                    return
                buf += d
                while b"\n" in buf:
                    l, buf = buf.split(b"\n", 1)
                    line = l.decode(errors="replace").rstrip("\r")
                    self.rec(self.name, line)
                    r = self.reply(line)
                    if r is not None:
                        try:
                            c.sendall((r + "\n").encode())
                        except OSError:
                            return

    def close(self):
        try:
            self.srv.close()
        except OSError:
            pass


def _motor_reply(line: str):
    if line.startswith(("MOVE", "TURN")):
        return "OK 0 -1"             # id 0, est -1: tars_voice's keepalive stays off
    if line == "STATUS":
        return "OK shadow=1 (fake tars_motor: no real state)"
    return "OK"


def _goto_reply(line: str):
    if line == "MEMORY SAY":
        return "OK n=0 (shadow: the dry run has no object memory)"
    return "OK"


class Shadow:
    """The real tars_voice host build, wired to fakes. HEAR -> the lines it really sends."""

    def __init__(self):
        self.lines: list[dict] = []
        self.cv = threading.Condition()
        self.seq = 0
        rec = self._rec
        self.motor = _Fake("motor", rec, _motor_reply)
        self.goto = _Fake("goto", rec, _goto_reply)
        self.say = _Fake("say", rec, lambda l: None)
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        self.net_port = s.getsockname()[1]
        s.close()
        self.log_path = os.path.join(_cache_dir, "shadow_voice.log")
        os.makedirs(_cache_dir, exist_ok=True)
        self.log = open(self.log_path, "w")
        self.proc = subprocess.Popen(
            [_paths["voice_bin"], "--input", "raw:-", "--model", _paths["model"],
             "--host", "127.0.0.1", "--port", str(self.motor.port),
             "--goto-port", str(self.goto.port), "--say-port", str(self.say.port),
             "--net-host", "127.0.0.1", "--net-port", str(self.net_port), "--threads", "2"],
            stdin=subprocess.PIPE, stdout=self.log, stderr=subprocess.STDOUT)
        self.sock = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15:
            if self.proc.poll() is not None:
                raise RuntimeError(f"shadow tars_voice exited {self.proc.returncode} (log {self.log_path})")
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.net_port), timeout=10)
                break
            except OSError:
                time.sleep(0.05)
        if not self.sock:
            self.close()
            raise RuntimeError("shadow tars_voice net port never opened")
        self.rf = self.sock.makefile("rb")
        self._cmd("PING")
        self._settle(0.3)                       # "WHO voice" on connect

    def _rec(self, dest: str, line: str):
        with self.cv:
            self.seq += 1
            self.lines.append({"seq": self.seq, "to": dest, "line": line, "t": time.monotonic()})
            self.cv.notify_all()

    def _cmd(self, line: str, data: bytes = b"") -> str:
        self.sock.sendall((line + "\n").encode() + data)
        r = self.rf.readline()
        if not r:
            raise RuntimeError("shadow tars_voice closed the connection")
        return r.decode(errors="replace").strip()

    def _settle(self, quiet: float, cap: float = 1.0):
        """wait until no new line arrived for `quiet` s (best-effort say / beep lines)"""
        t0 = time.monotonic()
        with self.cv:
            while True:
                now = time.monotonic()
                last = max(t0, self.lines[-1]["t"] if self.lines else 0)
                idle = now - last
                if idle >= quiet or now - t0 >= cap:
                    return
                self.cv.wait(quiet - idle)

    def alive(self) -> bool:
        return self.proc.poll() is None

    def hear(self, text: str) -> dict:
        with self.cv:
            start = len(self.lines)
        t0 = time.perf_counter()
        reply = self._cmd("HEAR " + text.replace("\n", " "))
        ms = (time.perf_counter() - t0) * 1000
        self._settle(0.08)
        with self.cv:
            got = [{"to": x["to"], "line": x["line"]} for x in self.lines[start:]]
        return {"reply": reply, "sent": got, "ms": round(ms, 2)}

    def describe(self) -> str:
        return (f"tars_voice_host pid {self.proc.pid} net {self.net_port}, fakes motor {self.motor.port} "
                f"goto {self.goto.port} say {self.say.port}" if self.alive() else
                f"exited ({self.proc.returncode}), log {self.log_path}")

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        for f in (self.motor, self.goto, self.say):
            f.close()
        self.log.close()


def shadow_hear(text: str) -> dict:
    global _shadow
    with _shadow_lock:
        try:
            if _shadow is None or not _shadow.alive():
                if _shadow:
                    _shadow.close()
                _shadow = Shadow()
            return _shadow.hear(text)
        except Exception as e:
            if _shadow:
                _shadow.close()
            _shadow = None
            return {"error": f"{type(e).__name__}: {e}"}


# ------------------------------------------------------------------ one decision, fully shown
def _split_dest(lines: list[str]) -> list[dict]:
    return [{"to": "goto", "line": l[len(GOTO_PREFIX):]} if l.startswith(GOTO_PREFIX) else
            {"to": "motor", "line": l} for l in lines]


def analyse(text: str, mode: str = "ptt", shadow: bool = True) -> dict:
    """mode 'ptt' = push-to-talk / HEAR / AUDIO (no wake word needed, as the robot's net input);
    'mic' = the robot's own mic (wake word required; wake carry-over is not modelled)."""
    require_wake = mode == "mic"
    t0 = time.perf_counter()
    c = cli_decide([text], require_wake)[0]
    classify_ms = (time.perf_counter() - t0) * 1000
    r = {"text": text, "mode": mode, "decision": c, "classify_ms": round(classify_ms, 2),
         "classify_note": "intent_cli process incl. spawn; the decision itself is ~2 us",
         "detail": detail(text), "notes": []}
    det = r["detail"]
    if "top3" in det and c["why"] in ("classifier", "low_conf", "unknown", "no_object"):
        if abs(det["top3"][0][1] - c["prob"]) > 1e-3:
            r["notes"].append(f"PARITY: intent.py top-1 {det['top3'][0]} != intent_cli {c['intent']} {c['prob']:.4f}")
    if c["action"] == "ignore":
        r["sent"] = []
        r["sent_source"] = f"nothing: ignored ({c['why']})"
        if c["why"] == "no_wake":
            r["notes"].append("no wake word: the robot mic ignores this; push-to-talk would act on it")
        return r
    if shadow:
        s = shadow_hear(text)
        if "error" in s:
            r["sent"] = _split_dest(c["lines"])
            r["sent_source"] = ("MIRRORED from intent_cli lines (shadow tars_voice failed: "
                                f"{s['error']}); BEEP/SAY extras not shown")
        else:
            r["sent"] = s["sent"]
            r["sent_source"] = "shadow tars_voice_host (real main.c) -> fake listeners"
            r["shadow_reply"] = s["reply"]
            want = f"OK heard {c['action']} {c['intent']}"
            if not s["reply"].startswith(want):
                r["notes"].append(f"MISMATCH: tars_voice said '{s['reply']}', intent_cli said '{want}'")
    else:
        r["sent"] = _split_dest(c["lines"])
        r["sent_source"] = "intent_cli lines (batch: shadow skipped for speed)"
    stops = [x for x in r["sent"] if x["to"] == "motor" and x["line"] == "S"]
    if c["action"] == "stop" and (not r["sent"] or r["sent"][0] != {"to": "motor", "line": "S"}):
        r["notes"].append("STOP ORDER: the first line sent was not S to tars_motor")
    r["stop_first"] = bool(stops) and r["sent"][0]["line"] == "S"
    return r


# ------------------------------------------------------------------ audio
def wav_bytes(samples, rate: int = SR) -> bytes:
    b = io.BytesIO()
    with wave.open(b, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(samples.tobytes() if hasattr(samples, "tobytes") else samples)
    return b.getvalue()


def read_wav(data: bytes) -> "array.array":
    """16 kHz mono 16-bit WAV (or raw s16le) -> array('h')."""
    if data[:4] == b"RIFF":
        try:
            with wave.open(io.BytesIO(data)) as w:
                if w.getsampwidth() != 2 or w.getnchannels() != 1 or w.getframerate() != SR:
                    raise ValueError("audio must be 16 kHz mono 16-bit")
                raw = w.readframes(w.getnframes())
        except (wave.Error, EOFError) as e:
            raise ValueError(f"bad WAV: {e}") from None
    else:
        raw = data[: len(data) // 2 * 2]
    a = array.array("h")
    a.frombytes(raw)
    if sys.byteorder != "little":
        a.byteswap()
    return a


def stt(samples: "array.array") -> tuple[str, float]:
    """Transcript via the real stt_cli; returns (text, ms incl. process + model load)."""
    os.makedirs(_cache_dir, exist_ok=True)
    fd, path = tempfile.mkstemp(suffix=".wav", dir=_cache_dir)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(wav_bytes(samples))
        t0 = time.perf_counter()
        p = subprocess.run([_paths["stt_cli"], _paths["model"], path, "2"], capture_output=True,
                           text=True, timeout=60)
        ms = (time.perf_counter() - t0) * 1000
    finally:
        os.unlink(path)
    if p.returncode != 0:
        raise RuntimeError(f"stt_cli failed: {p.stderr.strip()[-300:]}")
    return p.stdout.strip(), ms


def synth(text: str, voice: str = "") -> "array.array":
    """macOS `say` -> 16 kHz mono (cached), like iter2-autonomy/voice/tests/test_e2e.py."""
    os.makedirs(_cache_dir, exist_ok=True)
    key = hashlib.sha1(f"{voice}|{text}".encode()).hexdigest()[:16]
    wav = os.path.join(_cache_dir, f"say_{key}.wav")
    if not os.path.exists(wav):
        aiff = wav[:-4] + ".aiff"
        subprocess.run(["say", "-o", aiff] + (["-v", voice] if voice else []) + [text], check=True, timeout=30)
        subprocess.run(["afconvert", "-f", "WAVE", "-d", "LEI16@16000", "-c", "1", aiff, wav],
                       check=True, timeout=30)
        os.remove(aiff)
    with open(wav, "rb") as f:
        return read_wav(f.read())


def _rms(a) -> float:
    return math.sqrt(sum(x * x for x in a) / max(1, len(a)))


def mix_noise(a: "array.array", kind: str, snr_db: float, rng: random.Random,
              babble: "array.array | None" = None) -> "array.array":
    """Add white noise or babble (other speech) at snr_db relative to the clip's speech level."""
    if kind in ("", "none", None):
        return a
    pad = array.array("h", [0]) * int(0.3 * SR)
    x = pad + a + pad
    sp = _rms(a) or 1.0
    if kind == "white":
        n = [rng.gauss(0, 1) for _ in range(len(x))]
    else:
        src = babble if babble else array.array("h", [0])
        off = rng.randrange(max(1, len(src)))
        n = [float(src[(off + i) % len(src)]) for i in range(len(x))]
        # a little white floor, as the voice e2e "crowd" condition does
        nr = _rms(n) or 1.0
        n = [v + 0.5 * nr * rng.gauss(0, 1) for v in n]
    nr = _rms(n) or 1.0
    g = sp / nr / (10 ** (snr_db / 20))
    out = array.array("h", (max(-32768, min(32767, int(s + g * v))) for s, v in zip(x, n)))
    return out


CROWD = ["I think the demo is over there.", "Did you get any pizza yet?",
         "This hackathon is so much fun.", "Where is the hardware table?"]


def babble_source(path: str = "") -> "array.array":
    if path:
        with open(path, "rb") as f:
            return read_wav(f.read())
    voice = "Daniel" if "Daniel" in _voices() else ""
    out = array.array("h")
    for s in CROWD:
        out += synth(s, voice)
    return out


_voices_cache: list = []


def _voices() -> list:
    if not _voices_cache and shutil.which("say"):
        o = subprocess.run(["say", "-v", "?"], capture_output=True, text=True).stdout
        _voices_cache.extend(l.split()[0] for l in o.splitlines() if l.strip())
    return _voices_cache


def analyse_audio(data: bytes, mode: str = "ptt") -> dict:
    samples = read_wav(data)
    if len(samples) < SR // 10:
        raise ValueError("audio shorter than 0.1 s")
    if len(samples) > 15 * SR:
        raise ValueError("audio longer than 15 s (the robot's AUDIO limit)")
    text, stt_ms = stt(samples)
    r = analyse(text, mode)
    r["stt_ms"] = round(stt_ms, 1)
    r["stt_note"] = "stt_cli process incl. model load (~30 ms); the robot keeps the model loaded"
    r["audio_s"] = round(len(samples) / SR, 2)
    r["src"] = "audio"
    return r


# ------------------------------------------------------------------ send to the robot
def _robot():
    return _bench.plugins.get("robot") if _bench else None


def robot_send_text(text: str) -> dict:
    rb = _robot()
    if not rb:
        return {"error": "robot plugin not loaded: nothing sent"}
    try:
        return {"line": "HEAR " + text, "reply": rb.send("voice", "HEAR " + text)}
    except Exception as e:
        return {"line": "HEAR " + text, "error": f"{type(e).__name__}: {e}"}


def robot_send_audio(data: bytes, transcript: str) -> dict:
    rb = _robot()
    if not rb:
        return {"error": "robot plugin not loaded: nothing sent"}
    line = f"AUDIO {len(data)}"
    try:
        return {"line": line, "reply": rb.send("voice", line, data=data)}
    except TypeError:
        # robot.send has no binary payload: do what mission --laptop-stt does
        r = robot_send_text(transcript)
        r["note"] = "robot.send(dest, line, data=) not available: sent the LAPTOP transcript as HEAR"
        return r
    except Exception as e:
        return {"line": line, "error": f"{type(e).__name__}: {e}"}


# ------------------------------------------------------------------ batch
def _fill(t: str) -> str:
    """Deterministic placeholder fill for phrases.txt ({n} turn, {d} distance, {o} object)."""
    return t.replace("{n}", "ninety degrees").replace("{d}", "half a meter").replace("{o}", "trash can")


def load_phrases() -> list[dict]:
    out = []
    with open(_paths["phrases"]) as f:
        for line in f:
            line = line.split("#", 1)[0].rstrip("\n")
            if "\t" not in line:
                continue
            lab, ph = line.split("\t", 1)
            ph = ph.strip()
            if ph:
                out.append({"text": "tars " + _fill(ph), "want": lab.strip(), "cmd": "-"})
    return out


def load_realistic() -> list[dict]:
    out = []
    with open(_paths["realistic"]) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            t, w, c = (line.rstrip("\n").split("\t") + ["-", "-"])[:3]
            out.append({"text": t, "want": w, "cmd": c})
    return out


def load_wavdir(path: str) -> list[dict]:
    """*.wav in a folder. Labels from labels.tsv (file<TAB>intent[<TAB>first line]) if present,
    else the file name before '__' (e.g. goto__bin_far.wav), else unlabelled."""
    labels = {}
    lp = os.path.join(path, "labels.tsv")
    if os.path.exists(lp):
        with open(lp) as f:
            for line in f:
                p = line.rstrip("\n").split("\t")
                if len(p) >= 2 and not line.startswith("#"):
                    labels[p[0]] = (p[1], p[2] if len(p) > 2 else "-")
    out = []
    for fn in sorted(os.listdir(path)):
        if not fn.lower().endswith(".wav"):
            continue
        w, c = labels.get(fn, (fn.split("__", 1)[0] if "__" in fn else "?", "-"))
        out.append({"wav": os.path.join(path, fn), "text": fn, "want": w, "cmd": c})
    return out


def _got(dec: dict) -> str:
    return dec["intent"] if dec["action"] in ("command", "stop") else dec["action"]


def _ok(row: dict, dec: dict) -> bool:
    """Same scoring as iter2-autonomy/voice/tests/test_intent.py test_realistic."""
    got, want = _got(dec), row["want"]
    # goto STATUS/CANCEL come before a manual move (voice asks if goto is busy, then cancels): not the command
    first = next((l for l in dec["lines"] if l not in (GOTO_PREFIX + "CANCEL", GOTO_PREFIX + "STATUS")), "-")
    if first.startswith(GOTO_PREFIX):
        first = first[len(GOTO_PREFIX):]
    want_lines = row["cmd"]
    # phrases.txt label "unknown" = the robot should reject it (LED no), as realistic's "reject"
    return (got == want or (want == "reject" and dec["action"] in ("reject", "ignore")) or
            (want == "unknown" and dec["action"] == "reject")) and (want_lines in ("-", "") or first == want_lines)


def _moves(dec: dict) -> bool:
    return any(l.startswith(("MOVE", "TURN", GOTO_PREFIX + "GOTO", GOTO_PREFIX + "RECALL")) for l in dec["lines"])


def _pct(xs: list[float], q: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    return s[min(len(s) - 1, int(round(q * (len(s) - 1))))]


def run_batch(opts: dict, publish=None) -> dict:
    """opts: source phrases|realistic|wavdir, path, audio (bool: synthesise with say), voice,
    noise none|white|babble, snr, babble_file, mode ptt|mic, limit, seed."""
    src = opts.get("source", "realistic")
    rows = {"phrases": load_phrases, "realistic": load_realistic}.get(src)
    rows = rows() if rows else load_wavdir(opts.get("path", ""))
    if opts.get("limit"):
        rows = rows[: int(opts["limit"])]
    mode = opts.get("mode", "mic" if src == "realistic" else "ptt")
    audio = bool(opts.get("audio")) or src == "wavdir"
    noise, snr = opts.get("noise", "none"), float(opts.get("snr", 15))
    rng = random.Random(int(opts.get("seed", 1)))
    babble = babble_source(opts.get("babble_file", "")) if audio and noise == "babble" else None
    voice = opts.get("voice", "")
    t_start = time.time()
    results, stt_ms, cls_ms = [], [], []
    if not audio:
        t0 = time.perf_counter()
        decs = cli_decide([r["text"] for r in rows], mode == "mic")
        per = (time.perf_counter() - t0) * 1000 / max(1, len(rows))
        for r, d in zip(rows, decs):
            results.append({"text": r["text"], "heard": r["text"], "want": r["want"], "cmd": r["cmd"], "dec": d})
        cls_ms = [per]
    else:
        for i, r in enumerate(rows):
            if _batch["cancel"]:
                break
            if "wav" in r:
                with open(r["wav"], "rb") as f:
                    a = read_wav(f.read())
            else:
                a = synth(r["text"], voice)
            a = mix_noise(a, noise, snr, rng, babble)
            try:
                heard, ms = stt(a)
            except Exception as e:
                heard, ms = f"(stt failed: {e})", 0.0
            stt_ms.append(ms)
            t0 = time.perf_counter()
            d = cli_decide([heard], mode == "mic")[0]
            cls_ms.append((time.perf_counter() - t0) * 1000)
            results.append({"text": r["text"], "heard": heard, "want": r["want"], "cmd": r["cmd"], "dec": d})
            if publish and (i % 5 == 4 or i == len(rows) - 1):
                publish({"kind": "batch", "running": True, "done": i + 1, "total": len(rows)})
    labelled = [x for x in results if x["want"] != "?"]
    good = [x for x in labelled if _ok(x, x["dec"])]
    wrong_motion = [x for x in labelled if not _ok(x, x["dec"]) and _moves(x["dec"])]
    # confusion: want -> got
    conf: dict = {}
    for x in labelled:
        g = _got(x["dec"])
        conf.setdefault(x["want"], {}).setdefault(g, 0)
        conf[x["want"]][g] += 1
    misses = [x for x in labelled if not _ok(x, x["dec"])]
    misses.sort(key=lambda x: (not _moves(x["dec"]), -x["dec"]["prob"]))  # wrong motions first, then confident
    stops = [x for x in labelled if x["want"] == "stop"]
    out = {
        "kind": "batch", "running": False, "source": src, "mode": mode, "audio": audio,
        "noise": noise if audio else "none", "snr": snr if audio and noise != "none" else None,
        "voice": voice or "(default)", "n": len(results), "labelled": len(labelled), "correct": len(good),
        "accuracy": round(len(good) / len(labelled), 4) if labelled else None,
        "wrong_motion": len(wrong_motion),
        "stop_recall": f"{sum(1 for x in stops if x['dec']['action'] == 'stop')}/{len(stops)}",
        "confusion": conf,
        "worst": [{"text": x["text"], "heard": x["heard"], "want": x["want"], "cmd": x["cmd"],
                   "got": _got(x["dec"]), "prob": round(x["dec"]["prob"], 3), "lines": x["dec"]["lines"],
                   "moved": _moves(x["dec"])} for x in misses[:15]],
        "latency_ms": {"stt_median": round(statistics.median(stt_ms), 1) if stt_ms else None,
                       "stt_p95": round(_pct(stt_ms, 0.95), 1) if stt_ms else None,
                       "classify_median": round(statistics.median(cls_ms), 2) if cls_ms else None,
                       "classify_p95": round(_pct(cls_ms, 0.95), 2) if cls_ms else None,
                       "note": ("stt = stt_cli process incl. model load; classify = intent_cli spawn"
                                if audio else "classify = one intent_cli run over the whole set, per line")},
        "load": os.getloadavg()[0] if hasattr(os, "getloadavg") else None,
        "seconds": round(time.time() - t_start, 1),
        "cancelled": bool(_batch["cancel"]),
        "rows": [{"text": x["text"], "heard": x["heard"], "want": x["want"], "got": _got(x["dec"]),
                  "action": x["dec"]["action"], "prob": round(x["dec"]["prob"], 4),
                  "lines": x["dec"]["lines"], "ok": x["want"] == "?" or _ok(x, x["dec"])} for x in results],
    }
    return out


# ------------------------------------------------------------------ routes
def _json(body: bytes) -> dict:
    return json.loads(body or b"{}")


def _r_info(q, body):
    return 200, info()


def _r_text(q, body):
    b = _json(body)
    text = (b.get("text") or "").strip()
    if not text:
        return 400, {"error": "empty text"}
    mode = b.get("mode", "ptt")
    r = analyse(text, mode)
    r["src"] = "text"
    if _state["send"]:
        r["robot"] = robot_send_text(text)
    _publish_result(r)
    return 200, r


def _r_audio(q, body):
    mode = q.get("mode", "ptt")
    try:
        r = analyse_audio(body, mode)
    except ValueError as e:
        return 400, {"error": str(e)}
    if _state["send"]:
        r["robot"] = robot_send_audio(body if body[:4] == b"RIFF" else wav_bytes(read_wav(body)), r["text"])
    _publish_result(r)
    return 200, r


def _r_send(q, body):
    b = _json(body)
    on = bool(b.get("on"))
    if on and not _robot():
        _state["send"] = False
        return 409, {"error": "robot plugin not loaded: the send switch stays off", "send": False}
    _state["send"] = on
    _bench.log("voice", "send switch " + ("ON: voice goes to the robot's tars_voice" if on else "off (dry run)"))
    _bench.publish("voice", {"kind": "send", "send": on})
    return 200, {"send": on}


def _r_batch(q, body):
    opts = _json(body)
    t = _batch["thread"]
    if t and t.is_alive():
        return 409, {"error": "a batch is already running"}
    _batch["cancel"] = False
    if not (opts.get("audio") or opts.get("source") == "wavdir"):
        r = run_batch(opts)
        _bench.publish("voice", {k: v for k, v in r.items() if k != "rows"})
        return 200, r

    def work():
        try:
            r = run_batch(opts, publish=lambda d: _bench.publish("voice", d))
        except Exception as e:
            traceback.print_exc()
            r = {"kind": "batch", "running": False, "error": f"{type(e).__name__}: {e}"}
        _batch["last"] = r
        _bench.publish("voice", r)
        _bench.log("voice", f"batch done: {r.get('correct')}/{r.get('labelled')} ({r.get('source')}, "
                            f"noise {r.get('noise')} {r.get('snr') or ''})")
    _batch["thread"] = threading.Thread(target=work, daemon=True)
    _batch["thread"].start()
    return 202, {"started": True}


def _r_batch_cancel(q, body):
    _batch["cancel"] = True
    return 200, {"cancelling": True}


def _r_batch_last(q, body):
    return 200, _batch.get("last") or {}


def _publish_result(r: dict) -> None:
    d = r["decision"]
    _bench.publish("voice", {"kind": "result", "src": r.get("src"), "text": r["text"], "action": d["action"],
                             "intent": d["intent"], "prob": d["prob"], "sent": r.get("sent", []),
                             "robot": r.get("robot")}, sticky=False)


def setup(bench) -> None:
    global _bench
    _bench = bench
    _setup_paths(bench.repo)
    _load_ref()
    missing = [k for k in ("intent_cli", "stt_cli", "voice_bin") if not os.path.exists(_paths[k])]
    if missing:
        bench.log("voice", f"voice lab: missing {missing}: run make -C iter2-autonomy/voice host")
    bench.route("GET", "/api/voicelab/info", _r_info)
    bench.route("POST", "/api/voicelab/text", _r_text)
    bench.route("POST", "/api/voicelab/audio", _r_audio)
    bench.route("POST", "/api/voicelab/send", _r_send)
    bench.route("POST", "/api/voicelab/batch", _r_batch)
    bench.route("POST", "/api/voicelab/batch_cancel", _r_batch_cancel)
    bench.route("GET", "/api/voicelab/batch_last", _r_batch_last)


def shutdown() -> None:
    global _shadow
    _batch["cancel"] = True
    with _shadow_lock:
        if _shadow:
            _shadow.close()
            _shadow = None
