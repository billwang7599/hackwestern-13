#!/usr/bin/env python3
"""Voice lab tests: the bench server with only the voicelab plugin, on a free port.

    python3 bench/tests/test_voice.py          (needs make -C iter2-autonomy/voice host + get_model.sh;
                                                macOS say/afconvert for the audio check)
Checks:
  1. the panel's decision (HTTP) == intent_cli on every line of phrases.txt (both wake modes),
     and the Python reference top-1 matches intent_cli's prob (the top-3 shown is trustworthy)
  2. dry-run lines come from the shadow tars_voice_host and match tars_voice's own e2e
     expectations (iter2-autonomy/voice/tests/test_e2e.py CASES), incl. the 4 named phrases;
     a stop sends S to tars_motor first, then CANCEL to go-to
  3. a `say`-synthesised WAV round-trips through whisper to the right intent
  4. send switch off: nothing reaches a fake tars_voice; on: HEAR / AUDIO reach it
"""
import ast
import http.client
import json
import os
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import bench as B  # noqa: E402

fails = total = 0


def check(ok, what):
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class FakeVoice:
    """Stands in for the robot's tars_voice: counts connections, records lines and AUDIO bytes."""
    def __init__(self):
        self.srv = socket.socket()
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.conns = 0
        self.got: list = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            self.conns += 1
            f = c.makefile("rb")
            while True:
                line = f.readline()
                if not line:
                    break
                line = line.decode().strip()
                if line.startswith("AUDIO "):
                    data = f.read(int(line.split()[1]))
                    self.got.append((line, len(data)))
                    c.sendall(b"OK heard command left 0.99 turn left\n")
                else:
                    self.got.append((line, 0))
                    c.sendall(b"OK heard command left 0.99 x\n")
            c.close()


class FakeRobot:
    """What the robot plugin offers other plugins: send(dest, line[, data])."""
    def __init__(self, port):
        self.port = port

    def send(self, dest, line, data=b""):
        assert dest == "voice"
        with socket.create_connection(("127.0.0.1", self.port), timeout=5) as s:
            s.sendall((line + "\n").encode() + (data or b""))
            return s.makefile().readline().strip()


def main() -> int:
    port = free_port()
    bench = B.Bench(B.parse_args(["--port", str(port), "--plugins", "voicelab"]))
    B.load_plugins(bench, ["voicelab"])
    vl = bench.plugins.get("voicelab")
    check(vl is not None, "voicelab plugin loads")
    if not vl:
        return 1
    srv = B.ThreadingHTTPServer(("127.0.0.1", port), B.make_handler(bench))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def api(path, body=None, raw=None, ctype="application/json"):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=120)
        if raw is not None:
            c.request("POST", "/api/voicelab/" + path, body=raw, headers={"Content-Type": ctype})
        elif body is None:
            c.request("GET", "/api/voicelab/" + path)
        else:
            c.request("POST", "/api/voicelab/" + path, body=json.dumps(body))
        r = c.getresponse()
        return r.status, json.loads(r.read() or b"{}")

    st, info = api("info")
    check(st == 200 and all(info["bins"].values()) and info["whisper_model"]["present"],
          f"info: binaries + model present ({info.get('bins')})")
    check(info["send"] is False and info["robot_plugin"] is False, "send switch off by default, no robot plugin")

    # ---- 1. classifier parity on every phrases.txt line
    rows = vl.load_phrases()
    texts = [r["text"] for r in rows]
    for mode, flags in (("ptt", ["--any"]), ("mic", [])):
        st, b = api("batch", {"source": "phrases", "mode": mode})
        p = subprocess.run([vl._paths["intent_cli"]] + flags, input="\n".join(texts) + "\n",
                           capture_output=True, text=True)
        direct = [l.split("\t") for l in p.stdout.splitlines()]
        same = st == 200 and len(b["rows"]) == len(direct) == len(texts)
        bad = []
        for row, d in zip(b.get("rows", []), direct):
            want_lines = [x for x in d[5].split(" | ") if x]
            if (row["action"], abs(row["prob"] - float(d[2])) < 1e-4, row["lines"]) != (d[0], True, want_lines):
                bad.append((row["text"], row["action"], d[0]))
        check(same and not bad, f"panel == intent_cli on all {len(texts)} phrases.txt lines, mode {mode} "
                                f"({len(bad)} differ{': ' + str(bad[:3]) if bad else ''})")
        print(f"       phrases.txt ({mode}): {b.get('correct')}/{b.get('labelled')} vs labels, "
              f"wrong motion {b.get('wrong_motion')}, stop recall {b.get('stop_recall')}")
    par_bad = 0
    decs = vl.cli_decide(texts, False)
    for t, d in zip(texts, decs):
        if d["why"] in ("classifier", "low_conf", "unknown", "no_object"):
            top = vl.detail(t)["top3"][0]
            if top[0] != d["intent"] or abs(top[1] - d["prob"]) > 1e-3:
                par_bad += 1
    check(par_bad == 0, f"top-3 (intent.py) top-1 == intent_cli intent and prob on every classified line ({par_bad} differ)")
    st, b = api("batch", {"source": "realistic"})
    print(f"       realistic.tsv (mic, text): {b['correct']}/{b['labelled']}, wrong motion {b['wrong_motion']}, "
          f"stop recall {b['stop_recall']}")
    check(b["wrong_motion"] == 0 and b["correct"] >= 0.9 * b["labelled"], "realistic.tsv: >= 90%, no wrong motion")

    # ---- 2. dry-run lines from the shadow tars_voice vs its own e2e expectations
    src = open(vl._paths["e2e"]).read()
    cases = next(ast.literal_eval(n.value) for n in ast.parse(src).body
                 if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") == "CASES")
    e2e_bad = []
    for text, want in cases:
        st, r = api("text", {"text": text, "mode": "mic"})     # e2e runs the robot mic (wake required)
        real = [s for s in r["sent"] if not (s["to"] == "goto" and s["line"] == "CANCEL")]
        first = real[0]["line"] if real else None
        exp = "LED no" if want == "REJECT" else want
        if first != exp or (r["sent"] and "shadow" not in r["sent_source"]):
            e2e_bad.append((text, want, first, r["sent_source"]))
    check(not e2e_bad, f"all {len(cases)} test_e2e CASES: shadow tars_voice first line == e2e expectation "
                       f"({e2e_bad[:3]})")
    named = [("tars stop", [("motor", "S"), ("goto", "CANCEL"), ("motor", "BEEP 1")]),
             ("move forward half a meter", [("goto", "CANCEL"), ("motor", "MOVE 500"), ("motor", "BEEP 1")]),
             ("turn left", [("goto", "CANCEL"), ("motor", "TURN -30"), ("motor", "BEEP 1")]),
             ("tars go to the trash can", [("goto", "GOTO trash can"), ("motor", "BEEP 1")])]
    for text, want in named:
        st, r = api("text", {"text": text, "mode": "ptt"})
        got = [(s["to"], s["line"]) for s in r["sent"] if s["to"] != "say"]
        check(got == want and "shadow tars_voice" in r["sent_source"] and not r["notes"],
              f"dry run '{text}' -> {got}")
    st, r = api("text", {"text": "tars stop", "mode": "ptt"})
    check(r["sent"][0] == {"to": "motor", "line": "S"} and r["stop_first"], "stop: S to tars_motor is the very first line")
    st, r = api("text", {"text": "turn left", "mode": "mic"})
    check(r["sent"] == [] and r["decision"]["why"] == "no_wake", "robot mic mode: no wake word -> nothing sent")

    # ---- 3. say -> WAV -> whisper -> intent
    if vl.shutil.which("say") and vl.shutil.which("afconvert"):
        wav = vl.wav_bytes(vl.synth("Tars, turn left."))
        st, r = api("audio?mode=mic", raw=wav, ctype="audio/wav")
        lines = [s["line"] for s in r.get("sent", [])]
        check(st == 200 and r["decision"]["intent"] == "left" and "TURN -30" in lines,
              f"say 'Tars, turn left.' -> whisper '{r.get('text')}' -> {r.get('decision', {}).get('intent')} {lines} "
              f"(stt {r.get('stt_ms')} ms, load {os.getloadavg()[0]:.2f})")
        st, r = api("audio", raw=b"RIFF" + b"\0" * 40, ctype="audio/wav")
        check(st == 400, f"a bad WAV is refused with 400 ({r.get('error')})")
    else:
        print("  skip say/afconvert not available: audio round trip not tested")

    # ---- 4. send switch
    fv = FakeVoice()
    st, r = api("send", {"on": True})
    check(st == 409 and r["send"] is False, "send switch can't turn on without the robot plugin")
    bench.plugins["robot"] = FakeRobot(fv.port)
    api("text", {"text": "tars turn left"})
    if vl.shutil.which("say"):
        api("audio", raw=vl.wav_bytes(vl.synth("Tars, turn left.")), ctype="audio/wav")
    time.sleep(0.3)
    check(fv.conns == 0 and fv.got == [], f"send switch OFF: nothing reached the fake tars_voice ({fv.got})")
    st, r = api("send", {"on": True})
    check(st == 200 and r["send"] is True, "send switch on (robot plugin present)")
    st, r = api("text", {"text": "tars turn left"})
    check(("HEAR tars turn left", 0) in fv.got and r["robot"]["reply"].startswith("OK heard"),
          f"send ON: HEAR reached tars_voice, reply shown ({r.get('robot')})")
    if vl.shutil.which("say"):
        wav = vl.wav_bytes(vl.synth("Tars, turn left."))
        st, r = api("audio", raw=wav, ctype="audio/wav")
        check((f"AUDIO {len(wav)}", len(wav)) in fv.got, f"send ON: AUDIO {len(wav)} + bytes reached tars_voice")
    api("send", {"on": False})

    vl.shutdown()
    srv.shutdown()
    print(f"{total - fails}/{total} checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
