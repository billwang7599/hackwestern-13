#!/usr/bin/env python3
"""The bench as mission control: every mission.py feature, through the bench's HTTP API, against
mission's own fake Pi (iter2-autonomy/mission/tests/demo_stack.py pieces): the REAL tars_motor
(-b stub), REAL goto, REAL tars_voice host build (FakeVoice if not built), REAL mission_relay
with a key, a fake frame broker (edge-ai/models/dog.jpg), one LDR1 room scan into goto's nav
feed, fake YOLO detections and a fake phone status tee. Free ports only.

    make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host && make -C iter2-autonomy/mission host
    make -C iter2-autonomy/voice host      # optional: real tars_voice instead of the fake
    python3 bench/tests/test_mission.py

SSH (journals, room-view tunnel) uses a fake `ssh` (BENCH_SSH) that runs the remote command
locally, so Andrew's follow_actions.py really runs against journal files in a temp dir.
Not covered here: the real Pi, the relay over Wi-Fi, the robot's camera, real ssh.
"""
from __future__ import annotations

import hashlib
import http.client
import json
import os
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import math

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
MT = os.path.join(REPO, "iter2-autonomy", "mission", "tests")
sys.path.insert(0, MT)
import test_relay as tr  # noqa: E402
import demo_stack as ds  # noqa: E402  (ldr1_room, JpegBroker)

RELAY_BIN = os.path.join(REPO, "iter2-autonomy", "mission", "build", "mission_relay_host")
fails = total = 0
OUT = tempfile.mkdtemp(prefix="bench-mission-")


def check(ok, what):
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)
    return ok


FAKE_SSH = r'''
import os, sys, time
a = sys.argv[1:]
with open(os.environ["FAKE_SSH_LOG"], "a") as f:
    f.write(" ".join(a) + "\n")
i, host, cmd = 0, None, []
while i < len(a):
    if a[i] in ("-o", "-L", "-p", "-i"):
        i += 2
        continue
    if a[i].startswith("-"):
        i += 1
        continue
    host, cmd = a[i], a[i + 1:]
    break
if host == "bad@host":
    sys.stderr.write("ssh: connect to host bad port 22: Connection refused\n")
    sys.exit(255)
if cmd:
    os.execvp("sh", ["sh", "-c", " ".join(cmd)])
while True:
    time.sleep(1)
'''


class BenchProc:
    def __init__(self, args, env=None, name="bench"):
        self.port = tr.free_port()
        self.logp = os.path.join(OUT, f"{name}_{self.port}.log")
        self.log = open(self.logp, "w")
        self.p = subprocess.Popen([sys.executable, os.path.join(BENCH, "bench.py"), "--port", str(self.port), *args],
                                  stdout=self.log, stderr=subprocess.STDOUT, env=env)
        assert tr.wait_for(lambda: tr.port_open(self.port) or self.p.poll() is not None, 8), "bench didn't start"
        assert self.p.poll() is None, f"bench exited: {open(self.logp).read()[-500:]}"

    def req(self, method, path, body=None, raw=None, timeout=25):
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
        data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
        c.request(method, path, body=data, headers={"Content-Type": "application/octet-stream" if raw is not None else "application/json"})
        r = c.getresponse()
        out = r.read()
        c.close()
        ct = r.getheader("Content-Type") or ""
        return r.status, (json.loads(out) if "json" in ct else out)

    def get(self, path):
        return self.req("GET", path)[1]

    def post(self, path, body=None, raw=None):
        return self.req("POST", path, body if raw is None else None, raw)[1]

    def state(self):
        return self.get("/api/mission/state")

    def events(self, topics, timeout=4.0) -> dict:
        """read /events until every topic in `topics` was seen once; {topic: data}"""
        s = socket.create_connection(("127.0.0.1", self.port), timeout=timeout)
        s.sendall(b"GET /events HTTP/1.1\r\nHost: x\r\n\r\n")
        f = s.makefile("rb")
        got, cur, end = {}, None, time.monotonic() + timeout
        try:
            while time.monotonic() < end and not all(t in got for t in topics):
                line = f.readline().decode(errors="replace").rstrip("\r\n")
                if line.startswith("event: "):
                    cur = line[7:]
                elif line.startswith("data: ") and cur:
                    if cur in topics and cur not in got:
                        got[cur] = json.loads(line[6:])
                    cur = None
        except OSError:
            pass
        s.close()
        return got

    def kill(self):
        if self.p.poll() is None:
            self.p.send_signal(signal.SIGKILL)
            self.p.wait(3)

    def close(self):
        if self.p.poll() is None:
            self.p.terminate()
            try:
                self.p.wait(5)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.log.close()


class Holder:
    """the browser holding keys: POST /api/robot/keys every 100 ms"""
    def __init__(self, b: BenchProc, keys=("w",)):
        self.b, self.keys, self.stop_ev = b, list(keys), threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.stop_ev.is_set():
            try:
                self.b.post("/api/robot/keys", {"held": self.keys, "speed": 0.6})
            except OSError:
                return
            self.stop_ev.wait(0.1)

    def stop(self):
        self.stop_ev.set()


def main() -> int:
    for exe in (tr.MOTOR_BIN, ds.GOTO_BIN, RELAY_BIN):
        if not os.path.exists(exe):
            print(f"missing {exe}: make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host && make -C iter2-autonomy/mission host")
            return 2
    print(f"load {os.getloadavg()[0]:.1f} {os.getloadavg()[1]:.1f}   logs {OUT}")
    real_voice = os.path.exists(ds.VOICE_BIN) and os.path.exists(ds.MODEL)
    mport, gport, dport, nport, vport = (tr.free_port() for _ in range(5))
    procs, stop = [], threading.Event()
    benches: list[BenchProc] = []
    os.makedirs(tr.SCRATCH, exist_ok=True)
    fake_voice = None
    try:
        # ---------------------------------------------------------------- the fake Pi (demo_stack.py)
        procs.append(subprocess.Popen([tr.MOTOR_BIN, "-b", "stub", "-p", str(mport), "-r", "0", "-w", "2000"],
                                      stderr=subprocess.DEVNULL))
        assert tr.wait_for(lambda: tr.port_open(mport), 3)
        mem = os.path.join(OUT, "memory.txt")
        goto = subprocess.Popen([ds.GOTO_BIN, "-m", f"127.0.0.1:{mport}", "-p", str(gport), "-u", str(dport), "-n", str(nport),
                                 "-r", "0", "-M", mem, "-o", os.path.join(REPO, "iter2-autonomy", "goto", "objects.txt")],
                                stderr=open(os.path.join(OUT, "goto.log"), "w"))
        procs.append(goto)
        assert tr.wait_for(lambda: tr.port_open(gport), 3)
        if real_voice:
            procs.append(subprocess.Popen([ds.VOICE_BIN, "--input", "raw:-", "--model", ds.MODEL, "--port", str(mport),
                                           "--goto-port", str(gport), "--net-port", str(vport), "--say-port", "0"],
                                          stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=open(os.path.join(OUT, "voice.log"), "w")))
            tr.wait_for(lambda: tr.port_open(vport), 8)
        else:
            fake_voice = tr.FakeVoice()
            vport = fake_voice.port
        print(f"tars_voice: {'REAL host build' if real_voice else 'FAKE (voice/build/tars_voice_host not built)'}")
        broker = ds.JpegBroker()
        rl = tr.Relay(RELAY_BIN, mport, gport, vport, broker.port)
        nav = socket.create_connection(("127.0.0.1", nport))
        room = ds.ldr1_room()
        nav.sendall(struct.pack("!I", len(room)) + room)
        phone_on = threading.Event()
        phone_on.set()

        def fake_phone():                       # demo_stack.py's fake_phone, with an off switch
            det = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tee, k = None, 0
            while not stop.wait(0.2):
                k += 1
                x = 200 + 60 * math.sin(k / 10)
                det.sendto(json.dumps({"w": 640, "h": 480, "dets": [{"label": "chair", "conf": 0.82, "box": [x, 180, x + 140, 400]}]}).encode(),
                           ("127.0.0.1", dport))
                if k % 5 == 0 and phone_on.is_set():
                    try:
                        tee = tee or socket.create_connection(("127.0.0.1", rl.tee))
                        tee.sendall((json.dumps({
                            "type": "status", "streaming": True, "motion_hz": 99.8, "camera_fps": 10.1,
                            "safety": {"verdict": "CLEAR", "permit": True, "live": True, "stop_source": "lidar", "nearest_obstacle_m": 1.4},
                            "ai": {"live": True, "inference_ms": 148, "objects": [{"label": "dog", "confidence": 0.91, "bbox": [0.16, 0.38, 0.27, 0.55]}]},
                            "depth": {"live": False, "error": "MiDaS off"},
                            "surroundings": {"tracking": "normal", "floor_ready": True, "voxels": 4665, "frames": k, "scan_degrees": 180,
                                             "message": "demo", "pose": [0, 0, 0, 0.3], "objects": [{"label": "chair", "position": [1.0, 0, 2.0]}]},
                        }) + "\n").encode())
                    except OSError:
                        tee = None
        threading.Thread(target=fake_phone, daemon=True).start()

        # ---------------------------------------------------------------- fake ssh + journals
        jdir = os.path.join(OUT, "journals")
        os.makedirs(jdir)
        with open(os.path.join(jdir, "tars-motor-actions.log"), "w") as f:
            f.write("2026-11-21T22:00:00Z motor c3 D 0.5 0.5 -> OK\n")
        ssh_py = os.path.join(OUT, "fake_ssh.py")
        with open(ssh_py, "w") as f:
            f.write(FAKE_SSH)
        ssh_log = os.path.join(OUT, "ssh.log")
        env = dict(os.environ, BENCH_SSH=shlex.join([sys.executable, ssh_py]), FAKE_SSH_LOG=ssh_log, TARS_ACTION_DIR=jdir)
        env.pop("MISSION_KEY", None)
        env.pop("TARS_SSH", None)

        # ---------------------------------------------------------------- launcher rules
        print("launcher")
        r = subprocess.run([sys.executable, os.path.join(BENCH, "bench.py"), "--target", "pi", "--relay", f"127.0.0.1:{rl.port}",
                            "--port", str(tr.free_port())], capture_output=True, text=True, env=env, timeout=10)
        check(r.returncode != 0 and "relay key" in r.stderr, f"--target pi without a key -> bench refuses to start (exit {r.returncode})")
        b = BenchProc(["--target", "pi", "--relay", f"127.0.0.1:{rl.port}", "--plugins", "robot,mission"],
                      env=dict(env, MISSION_KEY=tr.KEY), name="pi")
        benches.append(b)
        page = b.get("/").decode()
        check('id="stop"' in page and 'id="badge"' in page, "page has the STOP button and the target badge in the shared header")
        pj = b.get("/panels.json")
        check(pj["target"]["name"] == "pi" and pj["real"] is True, "panels.json: target pi, real=true (badge says REAL ROBOT)")
        check("mission.js" in pj["panels"] and "car.js" not in pj["panels"] and "world2d.js" not in pj["panels"]
              and set(pj["hidden"]) == {"car.js", "world2d.js"}, f"pi: sim-only panels hidden ({pj['hidden']}), mission panels on")
        check(all(p in pj["panels"] for p in ("drive.js", "status.js", "voice.js", "map3d.js")), "pi: drive, status, voice, map panels kept")
        shell = b.get("/shell.js").decode()
        check("REAL ROBOT (Pi)" in shell and "'SIM'" in shell, "shell.js labels the badge SIM / REAL ROBOT")

        # ---------------------------------------------------------------- links + state stream
        print("relay links + state (WATCH through the bench)")
        ok = tr.wait_for(lambda: all(v["up"] for v in b.state()["links"].values()), 6, 0.2)
        st = b.state()
        check(ok and set(st["links"]) >= {"motor", "goto", "voice", "watch", "cam"},
              f"relay channels MOTOR GOTO VOICE WATCH CAM up ({ {k: v['up'] for k, v in st['links'].items()} })")
        ok = tr.wait_for(lambda: all(k in b.state() for k in ("relay", "memory", "nav", "dets", "grid", "phone", "motor", "goto")), 6, 0.2)
        st = b.state()
        check(ok, f"WATCH streams parsed by mission.State: {sorted(k for k in st if not k.endswith('_age'))}")
        check(st.get("relay", {}).get("motor") == "1" and st["relay"].get("goto") == "1" and st["relay"].get("voice") == "1",
              f"relay R line: services up ({st.get('relay')})")
        check(any(d.get("label") == "chair" for d in (st.get("dets") or {}).get("frame", {}).get("dets", [])), "goto DETS (Y): the chair")
        check(bool((st.get("grid") or {}).get("rle")), "goto NAV GRID (K) from the LDR1 room scan")
        check(((st.get("phone") or {}).get("status") or {}).get("safety", {}).get("verdict") == "CLEAR", "phone tee (P): verdict CLEAR")
        check("trash can" in st.get("objects_list", []), "object list from goto OBJECTS")
        check(st.get("target") == "pi" and st.get("speed") == 0.5, f"state carries target + WASD speed ({st.get('speed')})")
        ev = b.events(["mission", "mission_grid", "mission_phone", "status", "goto"])
        check(set(ev) == {"mission", "mission_grid", "mission_phone", "status", "goto"}, f"SSE topics to the browser: {sorted(ev)}")
        check("grid" not in ev.get("mission", {"grid": 1}) and "phone" not in ev.get("mission", {"phone": 1}),
              "big items (grid, phone) only on their own sticky topics")

        # ---------------------------------------------------------------- camera
        print("camera (relay CAM)")
        ok = tr.wait_for(lambda: (b.state().get("cam") or {}).get("id", 0) > 0, 4, 0.1)
        code, jpeg = b.req("GET", "/api/mission/cam.jpg")
        with open(ds.JPEG, "rb") as f:
            want = f.read()
        check(ok and code == 200 and jpeg == want, f"cam.jpg = the broker's frame byte for byte ({len(jpeg) if isinstance(jpeg, bytes) else jpeg})")
        check((b.state()["cam"]["age"] or 9) < 1.0, "camera age shown")

        # ---------------------------------------------------------------- STOP, WASD
        print("STOP + WASD (through the relay)")
        stops0 = int(tr.motor_status(mport)["stops"])
        r = b.post("/api/mission/goto", {"line": "GOTO chair"})["reply"]
        check(r.startswith("OK"), f"GOTO chair -> {r}")
        busy = tr.wait_for(lambda: (b.get("/api/robot/state").get("goto") or {}).get("state") not in ("idle", None), 2, 0.1)
        r = b.post("/api/robot/stop", {})
        ok = tr.wait_for(lambda: int(tr.motor_status(mport)["stops"]) >= stops0 + 1, 2, 0.05)   # goto answers CANCEL with S too
        idle = tr.wait_for(lambda: (b.get("/api/robot/state").get("goto") or {}).get("state") == "idle", 2, 0.1)
        check(r.get("S") and r.get("CANCEL") and ok and busy and idle,
              f"STOP: S reached tars_motor (stops {stops0}->{tr.motor_status(mport)['stops']}), goto busy -> idle")
        h = Holder(b)
        ok = tr.wait_for(lambda: tr.motor_status(mport)["l"] > 0.3, 3, 0.05)
        check(ok, f"hold W -> drives (l={tr.motor_status(mport)['l']})")
        h.keys = []
        check(tr.wait_for(lambda: tr.motor_status(mport)["l"] == 0.0, 0.8, 0.02), "release -> S -> wheels 0")
        h.keys = ["w"]
        tr.wait_for(lambda: tr.motor_status(mport)["l"] > 0.3, 2, 0.05)
        h.stop()
        t0 = time.monotonic()
        ok = tr.wait_for(lambda: tr.motor_status(mport)["l"] == 0.0, 1.5, 0.02)
        dt = time.monotonic() - t0
        check(ok and dt < 0.9, f"browser goes quiet while driving -> bench stops in {dt * 1000:.0f} ms (watchdog 2 s)")
        h = Holder(b)
        tr.wait_for(lambda: tr.motor_status(mport)["l"] > 0.3, 2, 0.05)
        b.post("/api/robot/stop", {})
        h.stop()
        check(tr.wait_for(lambda: tr.motor_status(mport)["l"] == 0.0, 0.6, 0.02), "STOP while a key is held -> wheels 0")

        # ---------------------------------------------------------------- goto buttons
        print("goto buttons (mission.Mission.goto_cmd via the bench)")
        time.sleep(1.6)                                         # S hold from the stop above
        replies = {}
        for line in ("RECALL chair", "SCAN", "CANCEL", "FORGET chair", "FORGET all"):
            replies[line] = b.post("/api/mission/goto", {"line": line})["reply"]
        check(all(v.startswith("OK") for v in replies.values()), f"RECALL/SCAN/CANCEL/FORGET/FORGET all -> {replies}")
        r = b.post("/api/mission/goto", {"line": "MOVE 100"})["reply"]
        check(r.startswith("ERR not allowed"), f"unlisted goto command refused ({r})")
        evs = b.state()["events"]
        check(any("SCAN -> OK" in e["text"] for e in evs), "goto commands appear in the mission event log")
        check(any(e["kind"] == "goto" and "->" not in e["text"] for e in evs),
              f"goto EVENTs from relay WATCH (E) in the log ({[e['text'] for e in evs if '->' not in e['text']][-2:]})")

        # ---------------------------------------------------------------- voice
        print("voice (text, MEMORY SAY, push-to-talk)")
        r = b.post("/api/mission/text", {"text": "tars turn left"})["reply"]
        check(r.startswith("OK heard"), f"typed text -> HEAR -> tars_voice ({r[:70]})")
        r = b.post("/api/mission/memory_say", {})["reply"]
        check(r.startswith("OK"), f"MEMORY SAY -> 'what do you remember' ({r[:70]})")
        clip = (struct.pack("<h", 0) * 16000)
        r = b.post("/api/mission/audio", raw=clip)["reply"]
        if real_voice:
            check(r.startswith(("OK", "ERR")) and "link down" not in r and r != "ERR no reply", f"1 s clip -> AUDIO 32000 -> tars_voice ({r[:70]})")
        else:
            check(r == "OK got 32000" and fake_voice.audio[-1][1] == hashlib.sha1(clip).hexdigest(), f"clip byte-exact to tars_voice ({r})")
        r = b.post("/api/mission/audio", raw=b"\0" * 100)["reply"]
        check(r == "ERR clip too short", f"short clip refused ({r})")
        st = b.state()
        vl = st.get("voice", [])
        check(any(e.get("src") == "typed" for e in vl) and any(e.get("src") == "push-to-talk" for e in vl),
              "voice log has the typed line and the clip")
        if real_voice:
            check(tr.wait_for(lambda: any(e.get("type") == "heard" for e in b.state().get("voice", [])), 3, 0.2),
                  "tars_voice WATCH events (heard) via relay WATCH V")
        time.sleep(1.6)                                          # 'turn left' may have moved; let it settle

        # ---------------------------------------------------------------- tip reset
        r = b.post("/api/mission/reset", {})["reply"]
        check(r.startswith(("OK", "ERR")) and "link down" not in r and "not allowed" not in r, f"RESET tip -> tars_motor answered ({r})")

        # ---------------------------------------------------------------- journals + room view (fake ssh)
        print("action journals + room view over (fake) ssh")
        j = b.get("/api/mission/journal")
        check(j["host"] == "qnxuser@172.23.43.185", f"default ssh host read from Andrew's actions.command ({j['host']})")
        j = b.post("/api/mission/journal", {"on": True, "host": "qnxuser@pi.test"})
        ok = tr.wait_for(lambda: any("D 0.5 0.5" in l for l in b.get("/api/mission/journal")["lines"]), 5, 0.2)
        check(j.get("running") and ok, "follow_actions.py (Andrew's, unchanged) streams the existing journal")
        with open(os.path.join(jdir, "tars-nav-actions.log"), "w") as f:
            f.write("2026-11-21T22:00:05Z nav EVENT arrived chair\n")
        got = b.events(["journal"], timeout=4)
        ok = tr.wait_for(lambda: any("arrived chair" in l for l in b.get("/api/mission/journal")["lines"]), 4, 0.2)
        check(ok and "journal" in got, "a new journal file shows up live (topic `journal`)")
        with open(ssh_log) as f:
            sl = f.read()
        check("qnxuser@pi.test python3 -u -" in sl and "BatchMode=yes" in sl, "ssh ran 'python3 -u -' on the chosen host, BatchMode")
        b.post("/api/mission/journal", {"on": False})
        check(not b.get("/api/mission/journal")["running"], "journal follower stopped")
        j = b.post("/api/mission/journal", {"on": True, "host": "bad@host"})
        ok = tr.wait_for(lambda: "Connection refused" in b.get("/api/mission/journal")["error"], 3, 0.1)
        check(ok and not b.get("/api/mission/journal")["running"], "ssh failure shown, not hung")
        check("error" in b.post("/api/mission/journal", {"on": True, "host": "x; rm -rf /"}), "bad host string refused")
        rv = b.post("/api/mission/roomview", {"on": True, "host": "qnxuser@pi.test"})
        time.sleep(0.3)
        rv = b.get("/api/mission/roomview")
        with open(ssh_log) as f:
            sl = f.read()
        check(rv["running"] and rv["url"] == "http://127.0.0.1:5580/" and "-L 127.0.0.1:5580:127.0.0.1:5580" in sl,
              "room view: ssh -N -L 5580 tunnel (room-view.command) running, URL given")
        b.post("/api/mission/roomview", {"on": False})
        check(not b.get("/api/mission/roomview")["running"], "room view tunnel closed")

        # ---------------------------------------------------------------- stale data
        print("stale data")
        phone_on.clear()
        ok = tr.wait_for(lambda: b.state().get("phone_age", 0) > 2.5, 5, 0.2)
        check(ok, f"phone tee stops -> phone_age grows ({b.state().get('phone_age')}) (panel greys it out)")

        # ---------------------------------------------------------------- --laptop-stt
        # the laptop only sends and receives: on the real robot the Pi transcribes, so the bench refuses it
        r = subprocess.run([sys.executable, os.path.join(BENCH, "bench.py"), "--target", "pi", "--relay",
                            f"127.0.0.1:{rl.port}", "--key", tr.KEY, "--laptop-stt"],
                           capture_output=True, text=True, timeout=20)
        check(r.returncode != 0 and "--laptop-stt is not allowed" in r.stderr,
              f"--target pi --laptop-stt -> bench refuses to start (exit {r.returncode})")

        # ---------------------------------------------------------------- kill -9 the bench while driving
        print("bench killed while driving")
        time.sleep(1.6)
        h = Holder(b)
        ok = tr.wait_for(lambda: tr.motor_status(mport)["l"] > 0.3, 3, 0.05)
        h.stop()
        b.kill()
        t0 = time.monotonic()
        ok2 = tr.wait_for(lambda: tr.motor_status(mport)["l"] == 0.0, 1.5, 0.01)
        dt = time.monotonic() - t0
        check(ok and ok2 and dt < 0.4, f"bench killed (-9) -> relay closes its MOTOR pipe -> wheels 0 in {dt * 1000:.0f} ms")

        # ---------------------------------------------------------------- sim target: same panels, laptop polling
        print("--target sim: mission panels from direct goto polling + tars_voice WATCH")
        b3 = BenchProc(["--target", "sim", "--motor", f"127.0.0.1:{mport}", "--goto", f"127.0.0.1:{gport}",
                        "--voice", f"127.0.0.1:{vport}", "--plugins", "robot,mission"], env=env, name="sim")
        benches.append(b3)
        pj = b3.get("/panels.json")
        check(pj["real"] is False and "car.js" in pj["panels"] and "world2d.js" in pj["panels"] and pj["hidden"] == [],
              "sim: every panel shown, badge SIM")
        ok = tr.wait_for(lambda: all(k in b3.state() for k in ("memory", "nav", "dets", "grid", "route")), 5, 0.2)
        st = b3.state()
        check(ok, f"sim: DETS/MEMORY/NAV/ROUTE/GRID polled from goto ({sorted(k for k in st if not k.endswith('_age'))})")
        check("cam" in st and not st["cam"]["id"] and b3.req("GET", "/api/mission/cam.jpg")[0] == 404, "sim: no camera, says so")
        r = b3.post("/api/mission/goto", {"line": "SCAN"})["reply"]
        check(r.startswith("OK"), f"sim: goto buttons work ({r})")
        b3.post("/api/mission/goto", {"line": "CANCEL"})
        time.sleep(1.6)
        r = b3.post("/api/mission/text", {"text": "what do you remember"})["reply"]
        check(r.startswith("OK"), f"sim: text -> tars_voice ({r[:60]})")
        if real_voice:
            check(tr.wait_for(lambda: any(e.get("type") in ("heard", "memory", "say") for e in b3.state().get("voice", [])), 3, 0.2),
                  "sim: tars_voice WATCH events reach the voice log")
        r = b3.post("/api/robot/stop", {})
        check(r.get("S") and r.get("CANCEL"), "sim: STOP sends S and CANCEL")
        b4 = BenchProc(["--target", "usb", "--motor", f"127.0.0.1:{mport}", "--goto", f"127.0.0.1:{tr.free_port()}",
                        "--voice", f"127.0.0.1:{tr.free_port()}", "--plugins", "robot,mission"], env=env, name="usb")
        benches.append(b4)
        pj = b4.get("/panels.json")
        check(pj["real"] is True and "car.js" not in pj["panels"], "usb: REAL ROBOT badge, sim-only panels hidden")
        r = b4.post("/api/robot/stop", {})
        check(r.get("S") is True, f"usb: STOP works with goto absent ({r})")
        b4b = BenchProc(["--target", "usb", "--all-panels", "--plugins", "none"], env=env, name="usball")
        benches.append(b4b)
        check("car.js" in b4b.get("/panels.json")["panels"], "--all-panels shows sim-only panels on a real target")
    finally:
        stop.set()
        for b in benches:
            b.close()
        for p in procs:
            p.terminate()
        try:
            rl.close()
            broker.close()
        except Exception:
            pass
    print(f"\n{total - fails}/{total} passed   load {os.getloadavg()[0]:.1f}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
