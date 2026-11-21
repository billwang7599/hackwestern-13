#!/usr/bin/env python3
"""Laptop mission control (laptop/mission.py) end to end on a Mac. stdlib only.

    python3 tests/test_laptop.py        (make -C iter2-autonomy/mission test-laptop)

Stack A: REAL tars_motor_host (-b stub, watchdog 2 s so the app's own stops are visible),
REAL goto_host, mission_relay host build, fake tars_voice + frame broker, and mission.py
driven through its HTTP API like the browser does.
Stack B: the same tars_motor with a fake goto that checks, when CANCEL arrives, whether
tars_motor has already seen the S (STOP order).
Checks: page + state stream; STOP = S then CANCEL; WASD drives and stops on release; the
browser going quiet stops the drive; killing the app stops ITS drive at once and leaves
another client's move alone; goto buttons; MEMORY SAY via voice; typed text and push-to-talk
reach tars_voice; camera MJPEG passthrough; stale data shows as stale; --laptop-stt sends
only text (if the whisper build + model + macOS say are there).
"""
from __future__ import annotations

import hashlib
import http.client
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_relay as tr  # noqa: E402  (fakes, Relay, free_port, wait_for, motor_status)
from test_relay import check, free_port, wait_for, motor_status, port_open  # noqa: E402

MISSION = tr.MISSION
REPO = tr.REPO
APP = os.path.join(MISSION, "laptop", "mission.py")
RELAY_BIN = os.path.join(MISSION, "build", "mission_relay_host")
GOTO_BIN = os.path.join(REPO, "iter2-autonomy", "goto", "build", "goto_host")
VOICE_DIR = os.path.join(REPO, "iter2-autonomy", "voice")


class App:
    def __init__(self, relay_port: int, extra=()):
        self.http = free_port()
        os.makedirs(tr.SCRATCH, exist_ok=True)
        self.log = open(os.path.join(tr.SCRATCH, f"app_{self.http}.log"), "w")
        self.p = subprocess.Popen([sys.executable, APP, "--relay", f"127.0.0.1:{relay_port}",
                                   "--http-port", str(self.http), "--key", tr.KEY, *extra], stdout=self.log, stderr=subprocess.STDOUT)
        assert wait_for(self._up, 5), "mission.py didn't start"

    def _up(self):
        try:
            self.get("/")
            return True
        except OSError:
            return False

    def get(self, path: str) -> bytes:
        c = http.client.HTTPConnection("127.0.0.1", self.http, timeout=3)
        c.request("GET", path)
        r = c.getresponse().read()
        c.close()
        return r

    def post(self, path: str, obj=None, raw: bytes | None = None) -> dict:
        c = http.client.HTTPConnection("127.0.0.1", self.http, timeout=25)
        body = raw if raw is not None else json.dumps(obj or {}).encode()
        c.request("POST", path, body=body, headers={"Content-Type": "application/octet-stream" if raw is not None else "application/json"})
        r = json.loads(c.getresponse().read() or b"{}")
        c.close()
        return r

    def state(self, timeout=3.0) -> dict:
        """one event from the /state stream (what the browser renders)"""
        c = http.client.HTTPConnection("127.0.0.1", self.http, timeout=timeout)
        c.request("GET", "/state")
        r = c.getresponse()
        buf = b""
        while b"\n\n" not in buf:
            buf += r.read1(65536)
        c.close()
        return json.loads(buf.split(b"\n\n", 1)[0][6:])

    def kill(self):
        self.p.send_signal(signal.SIGKILL)
        self.p.wait(5)
        self.log.close()

    def close(self):
        if self.p.poll() is None:
            self.p.terminate()
            try:
                self.p.wait(5)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.log.close()


class Heartbeat:
    """The browser holding a key: POST /keys every 100 ms."""
    def __init__(self, app: App, keys=("w",)):
        self.app, self.keys = app, list(keys)
        self.stop_ev = threading.Event()
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        while not self.stop_ev.is_set():
            try:
                self.app.post("/keys", {"held": self.keys})
            except OSError:
                return
            time.sleep(0.1)

    def stop(self):
        self.stop_ev.set()
        self.th.join()


def stack_a(mport):
    print("stack A: real tars_motor + real goto + relay + mission.py")
    gport = free_port()
    mem = os.path.join(tr.SCRATCH, f"mem_{gport}.txt")
    if os.path.exists(mem):
        os.remove(mem)
    glog = open(os.path.join(tr.SCRATCH, f"goto_{gport}.log"), "w")
    goto = subprocess.Popen([GOTO_BIN, "-m", f"127.0.0.1:{mport}", "-p", str(gport), "-u", str(free_port()),
                             "-n", "0", "-r", "0", "-M", mem,
                             "-o", os.path.join(REPO, "iter2-autonomy", "goto", "objects.txt")], stderr=glog)
    voice, broker = tr.FakeVoice(), tr.FakeBroker()
    rl = tr.Relay(RELAY_BIN, mport, gport, voice.port, broker.port)
    app = App(rl.port)
    try:
        page = app.get("/").decode()
        check('id="stop"' in page and "/cam.mjpg" in page, "page served with the STOP button and camera")
        ok = wait_for(lambda: all(v["up"] for v in app.state()["links"].values()), 5, 0.2)
        st = app.state()
        check(ok, f"all five relay links up ({ {k: v['up'] for k, v in st['links'].items()} })")
        # the key: mission.py won't start without one; a wrong one is refused by the relay
        env = {k: v for k, v in os.environ.items() if k != "MISSION_KEY"}
        r = subprocess.run([sys.executable, APP, "--relay", f"127.0.0.1:{rl.port}", "--http-port", str(free_port())],
                           env=env, capture_output=True, text=True, timeout=10)
        check(r.returncode != 0 and "relay key" in r.stderr, f"no key -> mission.py refuses to start ({r.returncode})")
        bad = App(rl.port, extra=("--key", "wrong"))
        try:
            time.sleep(1.0)
            links = bad.state()["links"]
            check(not any(v["up"] for v in links.values()) and all("ERR key" in v["error"] for v in links.values()),
                  f"wrong key -> every link refused with ERR key ({ {k: v['error'] for k, v in links.items()} })")
        finally:
            bad.kill()
        ok = wait_for(lambda: "out_l" in app.state().get("motor", {}) and "state" in app.state().get("goto", {}), 4, 0.2)
        st = app.state()
        check(ok and st.get("motor_age", 9) < 1 and st.get("goto_age", 9) < 1,
              f"tars_motor + goto status live (ages {st.get('motor_age')}, {st.get('goto_age')})")
        check("trash can" in st.get("objects_list", []), "object list from goto OBJECTS")

        print("goto buttons")
        r = app.post("/goto", {"line": "GOTO trash can"})["reply"]
        check(r == "OK goto trash can", f"GOTO trash can -> {r}")
        check(wait_for(lambda: app.state()["goto"].get("state") not in ("idle", None), 2, 0.1),
              f"goto busy shown ({app.state()['goto'].get('state')})")
        print("WASD while goto is busy: CANCEL first, then drive (after tars_motor's 1.5 s hold)")
        hb = Heartbeat(app)
        ok = wait_for(lambda: motor_status(mport)["l"] > 0.3, 3.0, 0.05)
        check(ok, f"laptop drives after cancelling goto (l={motor_status(mport)['l']})")
        check(app.state()["goto"].get("state") == "idle", "goto went idle (CANCEL sent before the first D)")
        hb.stop()
        app.post("/keys", {"held": []})
        check(wait_for(lambda: motor_status(mport)["l"] == 0.0, 0.5, 0.02), "key released -> S -> wheels 0")
        r = app.post("/goto", {"line": "BOGUS stuff"})["reply"]
        check(r.startswith("ERR not allowed"), "unlisted commands refused by the app")
        r = app.post("/goto", {"line": "FORGET all"})["reply"]
        check(r.startswith("OK forgot"), f"FORGET all -> {r}")
        r = app.post("/goto", {"line": "SCAN"})["reply"]
        check(r == "OK scan", f"SCAN -> {r}")
        r = app.post("/goto", {"line": "CANCEL"})["reply"]
        check(r == "OK" and wait_for(lambda: app.state()["goto"].get("state") == "idle", 2, 0.1), "CANCEL -> idle")
        ev = [e["text"] for e in app.state()["events"]]
        check(any("GOTO trash can -> OK" in e for e in ev), "goto commands appear in the event log")

        print("voice through the laptop")
        r = app.post("/voice/text", {"text": "tars turn left"})["reply"]
        check(r.startswith("OK heard"), f"typed text -> HEAR ({r})")
        r = app.post("/memory_say")["reply"]
        check(r.startswith("OK"), f"MEMORY SAY -> tars_voice 'what do you remember' ({r})")
        clip = os.urandom(32000)
        r = app.post("/voice/audio", raw=clip)["reply"]
        check(r == "OK got 32000" and voice.audio[-1][1] == hashlib.sha1(clip).hexdigest(),
              f"push-to-talk clip reaches tars_voice intact ({r})")
        vl = app.state()["voice"]
        check(any(e.get("src") == "push-to-talk" for e in vl) and any(e.get("src") == "typed" for e in vl),
              "voice log shows the laptop's typed text and clip")

        print("camera")
        c = http.client.HTTPConnection("127.0.0.1", app.http, timeout=5)
        c.request("GET", "/cam.mjpg")
        resp = c.getresponse()
        buf = b""
        while buf.count(b"--frame") < 2 and len(buf) < 2_000_000:
            buf += resp.read1(65536)
        c.close()
        part = buf.split(b"\r\n\r\n", 1)[1]
        jpeg = part[:part.index(b"\r\n--frame")]
        check(jpeg[:2] == b"\xff\xd8" and any(p[12:] == jpeg for p in broker.sent.values()),
              "MJPEG frame = the broker's JPEG, untouched")
        check(app.state()["cam"]["age"] < 1.0, "camera age shown")

        print("drive safety")
        time.sleep(1.6)                                    # goto's S (from CANCEL above) holds our D for 1.5 s
        app.post("/keys", {"held": ["w"]})                 # one press, then the browser goes quiet
        check(wait_for(lambda: motor_status(mport)["l"] > 0.3, 1, 0.02), "single key press drives")
        t0 = time.monotonic()
        ok = wait_for(lambda: motor_status(mport)["l"] == 0.0, 1.5, 0.02)
        dt = time.monotonic() - t0
        check(ok and dt < 0.8, f"browser quiet -> app stops the drive in {dt * 1000:.0f} ms (watchdog is 2 s here)")
        app.post("/keys", {"held": []})
        hb = Heartbeat(app)
        check(wait_for(lambda: motor_status(mport)["l"] > 0.3, 2, 0.02), "driving again (key held)")
        t0 = time.monotonic()
        app.kill()
        hb.stop()
        ok = wait_for(lambda: motor_status(mport)["l"] == 0.0, 1.5, 0.01)
        dt = time.monotonic() - t0
        check(ok and dt < 0.4, f"app killed (-9) while driving -> wheels 0 in {dt * 1000:.0f} ms (not the 2 s watchdog)")
        app = App(rl.port)
        wait_for(lambda: app.state()["links"]["MOTOR"]["up"], 5, 0.1)
        other = socket.create_connection(("127.0.0.1", mport))
        of = other.makefile("rw")
        of.write("WHO other\nMOVE 1000\n"); of.flush()
        of.readline(); mv = of.readline().strip()
        keep = threading.Event()

        def ping():
            while not keep.is_set():
                of.write("PING\n"); of.flush(); of.readline()
                time.sleep(0.1)
        tp = threading.Thread(target=ping, daemon=True)
        tp.start()
        time.sleep(0.3)
        app.kill()
        time.sleep(0.5)
        st = motor_status(mport)
        check(mv.startswith("OK") and st.get("move") == "running",
              f"app killed while idle -> another client's MOVE keeps running (move={st.get('move')})")
        keep.set(); tp.join()
        of.write("S\n"); of.flush(); of.readline()
        other.close()
        app = App(rl.port)

        if os.path.exists(os.path.join(VOICE_DIR, "build", "stt_cli")) and \
                os.path.exists(os.path.join(VOICE_DIR, "models", "ggml-tiny.en-q5_1.bin")):
            print("--laptop-stt: whisper on the laptop, only text to the robot")
            sys.path.insert(0, os.path.join(VOICE_DIR, "tests"))
            try:
                import numpy as np
                import test_e2e as e2e
                x = e2e.synth("Tars, turn left.", e2e.available_voices()[0])
                pcm = (np.clip(x / (abs(x).max() + 1e-9) * 0.5, -1, 1) * 32767).astype(np.int16).tobytes()
                app2 = App(rl.port, extra=("--laptop-stt",))
                wait_for(lambda: app2.state()["links"]["VOICE"]["up"], 5, 0.1)
                n_audio = len(voice.audio)
                r = app2.post("/voice/audio", raw=pcm)["reply"]
                check(r.startswith("OK heard") and "turn left" in r.lower() and len(voice.audio) == n_audio,
                      f"laptop transcribed, robot got HEAR text only ({r})")
                app2.close()
            except Exception as e:                       # noqa: BLE001
                check(False, f"--laptop-stt test crashed: {e}")
        else:
            print("  skip --laptop-stt (no voice/build/stt_cli or model)")

        print("stale data")
        goto.terminate(); goto.wait(5)
        ok = wait_for(lambda: app.state().get("goto_age", 0) > 1.0, 3, 0.2)
        st = app.state()
        check(ok and st.get("relay", {}).get("goto") == "0",
              f"goto gone -> its data ages ({st.get('goto_age')} s) and the relay reports goto=0")
        check(st.get("motor_age", 9) < 1, "tars_motor data stays fresh")
    finally:
        app.close()
        rl.close()
        if goto.poll() is None:
            goto.terminate(); goto.wait(5)
        glog.close()
        voice.srv.close(); broker.close()


def stack_b(mport):
    print("stack B: STOP order (fake goto checks tars_motor when CANCEL arrives)")
    seen = {}

    def handle(srv, c, line, _rest):
        if line == "CANCEL":
            seen.setdefault("cancel_stops", []).append(int(motor_status(mport)["stops"]))
            seen.setdefault("cancel_l", []).append(motor_status(mport)["l"])
            return "OK"
        if line == "STATUS":
            return f"OK state={seen.get('state', 'idle')} target=- result=-"
        if line == "OBJECTS":
            return "OK trash can,chair"
        return "OK"
    goto = tr.LineServer(handle)
    voice, broker = tr.FakeVoice(), tr.FakeBroker()
    rl = tr.Relay(RELAY_BIN, mport, goto.port, voice.port, broker.port)
    app = App(rl.port)
    try:
        wait_for(lambda: app.state()["links"]["MOTOR"]["up"] and app.state()["links"]["GOTO"]["up"], 5, 0.1)
        stops0 = int(motor_status(mport)["stops"])
        r = app.post("/stop")
        ok = wait_for(lambda: bool(seen.get("cancel_stops")), 2)
        check(r.get("S") and r.get("CANCEL"), f"STOP sent both ({r})")
        check(ok and seen["cancel_stops"][0] == stops0 + 1,
              f"S reached tars_motor before CANCEL reached goto (stops {stops0} -> {seen.get('cancel_stops')})")
        seen["state"] = "follow_path"                   # goto busy -> WASD must CANCEL before the first D
        seen.pop("cancel_l", None)
        time.sleep(0.5)
        hb = Heartbeat(app)
        ok = wait_for(lambda: motor_status(mport)["l"] > 0.3, 3, 0.05)
        hb.stop()
        app.post("/keys", {"held": []})
        check(ok and seen.get("cancel_l") and seen["cancel_l"][0] == 0.0,
              f"busy goto: CANCEL arrived before the first D moved the wheels ({seen.get('cancel_l')})")
        seen["state"] = "idle"
        time.sleep(0.5)
        n = len(seen.get("cancel_l", []))
        hb = Heartbeat(app)
        wait_for(lambda: motor_status(mport)["l"] > 0.3, 2, 0.05)
        hb.stop()
        app.post("/keys", {"held": []})
        check(len(seen.get("cancel_l", [])) == n, "idle goto: no CANCEL (no needless 1.5 s hold)")
    finally:
        app.close()
        rl.close()
        goto.close(); voice.srv.close(); broker.close()


def main() -> int:
    for p in (RELAY_BIN, tr.MOTOR_BIN, GOTO_BIN):
        if not os.path.exists(p):
            sys.exit(f"missing {p} (make -C iter2-autonomy/mission host; make -C iter1-wasd/pi host; make -C iter2-autonomy/goto host)")
    mport = free_port()
    motor = subprocess.Popen([tr.MOTOR_BIN, "-b", "stub", "-p", str(mport), "-r", "0", "-w", "2000"],
                             stderr=subprocess.DEVNULL)
    try:
        assert wait_for(lambda: port_open(mport), 3), "tars_motor didn't start"
        stack_a(mport)
        stack_b(mport)
    finally:
        motor.terminate()
        motor.wait(5)
    print(f"{tr.checks - tr.failures}/{tr.checks} checks passed")
    return 1 if tr.failures else 0


if __name__ == "__main__":
    sys.exit(main())
