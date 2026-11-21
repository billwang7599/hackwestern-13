#!/usr/bin/env python3
"""Host tests for mission_relay (iter2-autonomy/mission/relay.c). stdlib only.

    python3 tests/test_relay.py build/mission_relay_host      (make -C iter2-autonomy/mission test-relay)

The REAL tars_motor host build (-b stub) plus fakes for goto, tars_voice, the frame broker
and the phone tee. Checks:
  - MOTOR pipe: commands pass; a laptop that disconnects stops ITS drive at once (before the
    watchdog) and leaves another client's exact move alone
  - WATCH: every stream arrives tagged; status 5 Hz, map/phone <= 1 Hz (rate limits hold)
  - CAM: <= 5 fps, frames passed through byte for byte, broker asked <= 5 times a second
  - a laptop that never reads (WATCH + CAM) doesn't block the relay, the fast laptop, or goto
  - disconnects are clean: no leaked service connections; pollers only while someone watches
  - VOICE pipe carries a binary AUDIO clip intact; key; service down -> ERR; HELLO timeout
"""
from __future__ import annotations

import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
MISSION = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(MISSION))
MOTOR_BIN = os.path.join(REPO, "iter1-wasd", "pi", "build", "tars_motor_host")
SCRATCH = os.path.join(MISSION, "build", "scratch")

checks = failures = 0
_port_lock = threading.Lock()
_handed: set[int] = set()


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    print(("  ok   " if cond else "  FAIL ") + what)
    if not cond:
        failures += 1


def free_port() -> int:
    with _port_lock:
        while True:
            s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
            if p not in _handed:
                _handed.add(p)
                return p


def port_open(port: int) -> bool:
    try:
        socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
        return True
    except OSError:
        return False


def wait_for(pred, timeout: float, step: float = 0.02) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(step)
    return pred()


# ------------------------------------------------------------------ fakes
class LineServer:
    """Accepts many clients; handle(conn, line) -> reply or None. Tracks open connections."""
    def __init__(self, handle):
        self.port = free_port()
        self.handle = handle
        self.open = 0
        self.max_send_s = 0.0
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(16)
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            with self.lock:
                self.open += 1
                self.clients.append(c)
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def send(self, c: socket.socket, data: bytes) -> None:
        t = time.monotonic()
        c.sendall(data)
        self.max_send_s = max(self.max_send_s, time.monotonic() - t)

    def _serve(self, c: socket.socket):
        buf = b""
        try:
            while True:
                d = c.recv(65536)
                if not d:
                    break
                buf += d
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    r = self.handle(self, c, line.decode(errors="replace").strip(), buf)
                    if isinstance(r, tuple):            # (reply, bytes consumed from buf)
                        r, used = r
                        buf = buf[used:]
                    if r is not None:
                        self.send(c, (r + "\n").encode())
        except OSError:
            pass
        finally:
            with self.lock:
                self.open -= 1
                if c in self.clients:
                    self.clients.remove(c)
            c.close()

    def broadcast(self, line: str) -> None:
        with self.lock:
            cs = list(self.clients)
        for c in cs:
            try:
                self.send(c, (line + "\n").encode())
            except OSError:
                pass

    def close(self):
        self.srv.close()


class FakeGoto:
    def __init__(self, grid_bytes: int = 0):
        self.counts: dict[str, int] = {}
        self.grid = "OK grid v=3 size=200 res=0.10 origin=-10.00,-10.00 box=0,0,1,1 rle=1x1"
        if grid_bytes:
            self.grid = "OK grid v=3 size=200 res=0.10 origin=-10.00,-10.00 box=0,0,200,200 rle=" + \
                        ",".join(["1x1,2x1"] * (grid_bytes // 8))
        self.srv = LineServer(self.handle)
        self.port = self.srv.port

    def handle(self, srv, c, line, _rest):
        self.counts[line] = self.counts.get(line, 0) + 1
        return {
            "STATUS": "OK state=idle target=- result=- seen=0 motor=1 memory=0 pose=-",
            "DETS": 'OK age=0.10 {"w":640,"h":480,"dets":[{"label":"chair","conf":0.8,"box":[1,2,3,4]}]}',
            "MEMORY": "OK n=1 pose=0,0,0.0; chair#1 fresh x=1000 y=0 map=1.00,2.00",
            "NAV": "OK pose=arkit,0.10,0.20,5.0 free=1 occ=0 version=3 packets=1 bad=0 feed_age=0.05",
            "NAV ROUTE": "OK route=0.00,0.00;1.00,2.00 i=1",
            "NAV GRID": self.grid,
            "WATCH": "OK",
            "CANCEL": "OK",
        }.get(line, "OK " + line)


class FakeVoice:
    def __init__(self):
        self.srv = LineServer(self.handle)
        self.port = self.srv.port
        self.audio: list[tuple[int, str]] = []

    def handle(self, srv, c, line, rest):
        if line == "WATCH":
            return "OK watch"
        if line.startswith("HEAR "):
            return "OK heard command left 0.99 " + line[5:]
        if line.startswith("AUDIO "):
            n = int(line[6:])
            data = rest
            while len(data) < n:
                data += c.recv(65536)
            self.audio.append((n, hashlib.sha1(data[:n]).hexdigest()))
            return ("OK got %d" % n, n)
        return "ERR"


class FakeBroker:
    """iter2-autonomy/edge-ai/rgb_broker.py's GET protocol: 4-byte length + RGB1 packet."""
    def __init__(self):
        self.port = free_port()
        self.gets = 0
        self.frame_id = 1000
        self.sent: dict[int, bytes] = {}
        self.size = 20000
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", self.port))
        self.srv.listen(8)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            try:
                c.settimeout(1)
                if c.recv(4) == b"GET\n":
                    self.gets += 1
                    self.frame_id += 1
                    jpeg = b"\xff\xd8" + os.urandom(self.size) + b"\xff\xd9"
                    pkt = b"RGB1" + struct.pack("!Q", self.frame_id) + jpeg
                    self.sent[self.frame_id] = pkt
                    c.sendall(struct.pack("!I", len(pkt)) + pkt)
            except OSError:
                pass
            c.close()

    def close(self):
        self.srv.close()


# ------------------------------------------------------------------ relay + laptop clients
KEY = "test-key"                 # the relay refuses to start without one


class Relay:
    def __init__(self, exe, motor, goto, voice, rgb, extra=(), key=KEY):
        self.port, self.tee = free_port(), free_port()
        auth = ["-k", key] if key else ["--no-key"]
        os.makedirs(SCRATCH, exist_ok=True)
        self.logp = os.path.join(SCRATCH, f"relay_{self.port}.log")
        self.log = open(self.logp, "w")
        self.p = subprocess.Popen([exe, "-p", str(self.port), "-b", "127.0.0.1", "--motor", str(motor),
                                   "--goto", str(goto), "--voice", str(voice), "--rgb", str(rgb),
                                   "--tee", str(self.tee), "-v", *auth, *extra], stderr=self.log)
        assert wait_for(self._up, 3), "relay didn't open its port"

    def _up(self):
        try:
            socket.create_connection(("127.0.0.1", self.port), timeout=0.2).close()
            return True
        except OSError:
            return False

    def close(self):
        self.p.terminate()
        try:
            self.p.wait(3)
        except subprocess.TimeoutExpired:
            self.p.kill()
        self.log.close()


class Lap:
    """A laptop connection: HELLO <ch>, then lines (or raw frames for CAM)."""
    def __init__(self, port, ch, key=KEY, rcvbuf=0, read=True):
        self.s = socket.socket()
        if rcvbuf:
            self.s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        self.s.connect(("127.0.0.1", port))
        self.s.settimeout(5)
        self.s.sendall(f"HELLO {ch} {key}".strip().encode() + b"\n")
        self.buf = b""
        self.hello = self._line()
        self.lines: list[tuple[float, str]] = []
        self.frames: list[bytes] = []
        self.closed = False
        if read and self.hello.startswith("OK"):
            threading.Thread(target=self._rd, args=(ch,), daemon=True).start()

    def _line(self) -> str:
        while b"\n" not in self.buf:
            d = self.s.recv(65536)
            if not d:
                self.closed = True
                return ""
            self.buf += d
        l, self.buf = self.buf.split(b"\n", 1)
        return l.decode(errors="replace")

    def _rd(self, ch):
        try:
            while True:
                if ch == "CAM":
                    while len(self.buf) < 4:
                        d = self.s.recv(1 << 20)
                        if not d:
                            raise OSError
                        self.buf += d
                    n = struct.unpack("!I", self.buf[:4])[0]
                    while len(self.buf) < 4 + n:
                        d = self.s.recv(1 << 20)
                        if not d:
                            raise OSError
                        self.buf += d
                    self.frames.append(self.buf[4:4 + n])
                    self.buf = self.buf[4 + n:]
                else:
                    l = self._line()
                    if self.closed:
                        return
                    self.lines.append((time.monotonic(), l))
        except (OSError, socket.timeout):
            self.closed = True

    def send(self, line: str) -> None:
        self.s.sendall(line.encode() + b"\n")

    def reply(self, timeout=3.0) -> str | None:
        n0 = len(self.lines)
        if not wait_for(lambda: len(self.lines) > n0, timeout):
            return None
        return self.lines[n0][1]

    def tags(self, since=0.0) -> dict[str, int]:
        out: dict[str, int] = {}
        for t, l in list(self.lines):
            if t >= since and len(l) > 1:
                out[l[0]] = out.get(l[0], 0) + 1
        return out

    def last(self, tag: str) -> str | None:
        for _, l in reversed(list(self.lines)):
            if l.startswith(tag + " "):
                return l[2:]
        return None

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


def motor_status(port) -> dict:
    with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
        s.sendall(b"STATUS\n")
        r = s.makefile().readline().split()
    d = dict(kv.split("=", 1) for kv in r if "=" in kv)
    d["l"], d["r"] = float(r[1]), float(r[2])
    return d


# ------------------------------------------------------------------ tests
def test_motor_pipe(exe, mport, env):
    print("MOTOR pipe: a laptop disconnect stops only its own drive")
    rl = Relay(exe, mport, *env)
    try:
        lap = Lap(rl.port, "MOTOR")
        check(lap.hello == "OK relay MOTOR", f"HELLO MOTOR -> {lap.hello}")
        lap.send("WHO laptop")
        check((lap.reply() or "").startswith("OK"), "WHO laptop passes through")
        lap.send("PING")
        check(lap.reply() == "OK PONG", "PING -> OK PONG from tars_motor")
        stop = threading.Event()

        def feed():                                       # keep the watchdog fed like teleop does
            while not stop.is_set():
                try:
                    lap.send("D 0.5 0.5")
                except OSError:
                    return
                time.sleep(0.05)
        th = threading.Thread(target=feed, daemon=True)
        th.start()
        check(wait_for(lambda: motor_status(mport)["l"] > 0.3, 2), "laptop D 0.5 0.5 drives the wheels")
        stop.set(); th.join()
        t0 = time.monotonic()
        lap.close()
        ok = wait_for(lambda: motor_status(mport)["l"] == 0.0, 1.0, 0.01)
        dt = time.monotonic() - t0
        check(ok and dt < 0.3, f"laptop gone -> wheels 0 in {dt * 1000:.0f} ms (watchdog would be 500 ms)")
        # another client's exact move is left alone when an idle laptop leaves
        other = socket.create_connection(("127.0.0.1", mport))
        of = other.makefile("rw")
        of.write("MOVE 1000\n"); of.flush()
        mv = of.readline().strip()
        keep = threading.Event()

        def ping():
            while not keep.is_set():
                of.write("PING\n"); of.flush(); of.readline()
                time.sleep(0.1)
        tp = threading.Thread(target=ping, daemon=True)
        tp.start()
        lap2 = Lap(rl.port, "MOTOR")
        lap2.send("PING"); lap2.reply()
        time.sleep(0.2)
        lap2.close()
        time.sleep(0.4)
        st = motor_status(mport)
        check(mv.startswith("OK") and st.get("move") == "running",
              f"other client's MOVE still running after an idle laptop left (move={st.get('move')} why={st.get('move_why')})")
        keep.set(); tp.join()
        of.write("S\n"); of.flush(); of.readline()
        other.close()
    finally:
        rl.close()


def test_watch(exe, mport, env, goto, voice):
    print("WATCH: tagged streams and rate limits")
    rl = Relay(exe, mport, *env)
    try:
        tee = socket.create_connection(("127.0.0.1", rl.tee))
        stop = threading.Event()

        def tee_feed():                                    # the phone tee at 20 Hz
            k = 0
            while not stop.is_set():
                k += 1
                try:
                    tee.sendall((json.dumps({"type": "status", "k": k, "surroundings": {"grid": "0" * 1000}}) + "\n").encode())
                except OSError:
                    return
                time.sleep(0.05)
        threading.Thread(target=tee_feed, daemon=True).start()
        w = Lap(rl.port, "WATCH")
        check(w.hello == "OK relay WATCH", f"HELLO WATCH -> {w.hello}")
        time.sleep(1.0)
        goto.srv.broadcast("EVENT arrived chair")
        voice.srv.broadcast('{"type":"heard","src":"mic","text":"tars stop"}')
        t0 = time.monotonic()
        time.sleep(3.0)
        n = w.tags(t0)
        check(13 <= n.get("M", 0) <= 17, f"tars_motor STATUS at 5 Hz ({n.get('M', 0)} in 3 s)")
        check(13 <= n.get("G", 0) <= 17 and 13 <= n.get("Y", 0) <= 17, f"goto STATUS + DETS at 5 Hz ({n.get('G', 0)}, {n.get('Y', 0)})")
        check(2 <= n.get("K", 0) <= 4, f"map (NAV GRID) <= 1 Hz ({n.get('K', 0)} in 3 s)")
        check(2 <= n.get("P", 0) <= 4, f"phone status <= 1 Hz although the tee sends 20 Hz ({n.get('P', 0)} in 3 s)")
        check(all(2 <= n.get(t, 0) <= 4 for t in "ONTR"), f"memory / nav / route / relay status at 1 Hz ({[n.get(t, 0) for t in 'ONTR']})")
        check(w.last("E") == "EVENT arrived chair", "goto EVENT forwarded (E)")
        check((w.last("V") or "").startswith('{"type":"heard"'), "voice event forwarded (V)")
        check((w.last("M") or "").startswith("OK ") and "stops=" in (w.last("M") or ""), "M is a real tars_motor STATUS")
        p = json.loads(w.last("P") or "{}")
        check(p.get("type") == "status" and p.get("k", 0) > 40, f"P is the newest phone status line (k={p.get('k')})")
        r = w.last("R") or ""
        check("motor=1" in r and "goto=1" in r and "voice=1" in r and "phone_tee=1" in r, f"R reports upstreams up ({r[:70]})")
        check(goto.counts.get("WATCH", 0) >= 1, "relay subscribed to goto events")
        st = motor_status(mport)
        check("why=watchdog" in " ".join(f"{k}={v}" for k, v in st.items()) or st["l"] == 0.0,
              "the relay's own motor link never drives (wheels 0)")
        stop.set()
        tee.close()
        w.close()
        ok = wait_for(lambda: goto.srv.open == 0 and voice.srv.open == 0, 2)
        check(ok, f"nobody watching -> relay drops its goto/voice links ({goto.srv.open}, {voice.srv.open})")
    finally:
        rl.close()


def test_cam(exe, mport, env, broker):
    print("CAM: <= 5 fps passthrough")
    rl = Relay(exe, mport, *env)
    try:
        g0 = broker.gets
        c = Lap(rl.port, "CAM")
        check(c.hello == "OK relay CAM", f"HELLO CAM -> {c.hello}")
        time.sleep(3.0)
        frames, gets = len(c.frames), broker.gets - g0
        check(12 <= frames <= 16, f"{frames} frames in 3 s (cap 5 fps)")
        check(gets <= 16, f"broker asked {gets} times in 3 s")
        same = all(broker.sent.get(struct.unpack("!Q", f[4:12])[0]) == f for f in c.frames)
        check(same and frames > 0, "frames are the broker's RGB1 packets byte for byte")
        c.close()
    finally:
        rl.close()


def test_slow_laptop(exe, mport, env, goto, broker):
    print("a laptop that never reads doesn't block anyone")
    broker.size = 400000                     # big frames + a 1 MB watch queue: the slow laptop backs up fast
    rl = Relay(exe, mport, *env, extra=("--watch-lim", "600000"))
    try:
        slow_w = Lap(rl.port, "WATCH", rcvbuf=4096, read=False)
        slow_c = Lap(rl.port, "CAM", rcvbuf=4096, read=False)
        check(slow_w.hello.startswith("OK") and slow_c.hello.startswith("OK"), "slow laptop connected (WATCH + CAM)")
        fast = Lap(rl.port, "WATCH")
        fast_c = Lap(rl.port, "CAM")
        goto.max_send_reset = goto.srv.max_send_s = 0.0
        time.sleep(1.0)
        t0 = time.monotonic()
        time.sleep(6.0)
        n = fast.tags(t0)
        check(28 <= n.get("M", 0) <= 32, f"fast laptop still gets 5 Hz status ({n.get('M', 0)} in 6 s)")
        check(n.get("K", 0) >= 3, f"fast laptop still gets the big map line ({n.get('K', 0)} x ~300 KB)")
        check(len(fast_c.frames) >= 12, f"fast camera laptop still gets frames ({len(fast_c.frames)} in ~7 s)")
        check(goto.srv.max_send_s < 0.05, f"goto's replies never blocked (max send {goto.srv.max_send_s * 1000:.1f} ms)")
        r = fast.last("R") or ""
        kv = dict(x.split("=", 1) for x in r.split() if "=" in x)
        check(int(kv.get("drops", 0)) > 0 and int(kv.get("cam_skipped", 0)) > 0,
              f"slow laptop's lines/frames dropped, not queued (drops={kv.get('drops')} cam_skipped={kv.get('cam_skipped')})")
        q = int(kv.get("queued", -1))
        check(0 <= q <= 2 * 600000 + 2 * 400100, f"relay's queued bytes stay bounded ({q} B for 4 laptops)")
        for l in (slow_w, slow_c, fast, fast_c):
            l.close()
    finally:
        rl.close()
        broker.size = 20000


def test_clean_disconnects(exe, mport, env, goto, voice):
    print("clean disconnects")
    rl = Relay(exe, mport, *env)
    try:
        base = int(motor_status(mport)["clients"])
        for i in range(30):
            a = Lap(rl.port, ["MOTOR", "GOTO", "VOICE", "WATCH", "CAM"][i % 5])
            if i % 2:
                a.send("PING")
            a.close()
        ok = wait_for(lambda: int(motor_status(mport)["clients"]) == base and goto.srv.open == 0 and voice.srv.open == 0, 3)
        check(ok, f"30 connect/close cycles leave no service connections (motor clients {motor_status(mport)['clients']} vs {base}, "
                  f"goto {goto.srv.open}, voice {voice.srv.open})")
        a = Lap(rl.port, "MOTOR")
        a.send("PING")
        check(a.reply() == "OK PONG" and rl.p.poll() is None, "relay still serving afterwards")
        a.close()
        s = socket.create_connection(("127.0.0.1", rl.port))
        s.settimeout(5)
        t0 = time.monotonic()
        got = s.recv(10)
        check(got == b"" and 2.5 < time.monotonic() - t0 < 4.5, "no HELLO -> closed after ~3 s")
        s.close()
        s = socket.create_connection(("127.0.0.1", rl.port))
        s.sendall(f"HELLO BOGUS {KEY}\n".encode())
        check(s.makefile().readline().startswith("ERR unknown channel"), "unknown channel -> ERR")
        s.close()
    finally:
        rl.close()


def test_voice_pipe_key_down(exe, mport, env, voice):
    print("VOICE pipe (binary AUDIO), key, service down")
    rl = Relay(exe, mport, *env, key="s3cret")
    try:
        bad = Lap(rl.port, "VOICE", key="nope")
        check(bad.hello == "ERR key", f"wrong key -> {bad.hello}")
        check(bad.s.recv(16) == b"", "wrong key -> connection closed")
        bad.close()
        for ch in ("MOTOR", "WATCH"):
            bad = Lap(rl.port, ch, key="")
            check(bad.hello == "ERR key", f"{ch} with no key -> {bad.hello}")
            check(bad.s.recv(16) == b"", f"{ch} with no key -> connection closed")
            bad.close()
        v = Lap(rl.port, "VOICE", key="s3cret")
        check(v.hello == "OK relay VOICE", "right key -> OK")
        v.send("HEAR tars turn left")
        check((v.reply() or "").startswith("OK heard"), "HEAR passes through")
        clip = os.urandom(200000)
        v.s.sendall(f"AUDIO {len(clip)}\n".encode() + clip)
        r = v.reply(5)
        check(r == f"OK got {len(clip)}" and voice.audio and voice.audio[-1][1] == hashlib.sha1(clip).hexdigest(),
              "a 200 KB AUDIO clip arrives intact")
        v.close()
    finally:
        rl.close()
    dead = free_port()
    rl = Relay(exe, mport, env[0], dead, *env[2:])
    try:
        v = Lap(rl.port, "VOICE")
        check(v.hello == "ERR relay: VOICE unavailable", f"tars_voice not running -> {v.hello}")
        w = Lap(rl.port, "WATCH")
        time.sleep(1.5)
        r = w.last("R") or ""
        check("voice=0" in r and "motor=1" in r, f"WATCH still works and says voice=0 ({r[:60]})")
        w.close()
    finally:
        rl.close()


def test_headless_tee(exe, mport, env):
    """Andrew's headless.py MissionTee (opt-in, TARS_MISSION_TEE_PORT) feeding the relay."""
    print("phone status tee from headless.py")
    sys.path.insert(0, os.path.join(REPO, "iter2-autonomy", "dashboard"))
    from headless import MissionTee                     # noqa: E402

    class FakeLink:                                      # ControlLink's lock + latest
        lock = threading.Lock()
        latest = {"type": "status", "mode": "headless", "safety": {"verdict": "CLEAR"},
                  "surroundings": {"grid": "0" * 40000, "pose": [0.1, 0, 0.2, 0.5]}}
    rl = Relay(exe, mport, *env)
    tee = MissionTee(FakeLink, rl.tee, period=0.2)
    threading.Thread(target=tee.run, daemon=True).start()
    try:
        w = Lap(rl.port, "WATCH")
        ok = wait_for(lambda: w.last("P") is not None, 3)
        p = json.loads(w.last("P") or "{}")
        check(ok and p.get("safety", {}).get("verdict") == "CLEAR" and len(p["surroundings"]["grid"]) == 40000,
              "MissionTee's status (40 KB map included) reaches the laptop as P")
        w.close()
        port = rl.tee
        rl.close()
        time.sleep(1.0)                                   # one send may still land in the dead socket
        sent = tee.sent
        time.sleep(1.5)
        check(tee.sent == sent, "relay gone: the tee stops sending (backs off), nothing else happens")
        rl = Relay(exe, mport, *env, extra=("--tee", str(port)))   # same tee port (later flag wins)
        w = Lap(rl.port, "WATCH")
        check(wait_for(lambda: w.last("P") is not None, 5), "relay back: the tee reconnects by itself")
        w.close()
    finally:
        tee.stop.set()
        rl.close()


def test_key_required(exe, mport, env):
    """The relay refuses to start keyless; MISSION_KEY works; --no-key is an explicit opt-out."""
    print("key required: no key -> won't start; MISSION_KEY; --no-key opt-out")
    base = [exe, "-p", str(free_port()), "-b", "127.0.0.1", "--motor", str(mport), "--goto", str(env[0]),
            "--voice", str(env[1]), "--rgb", str(env[2])]
    clean = {k: v for k, v in os.environ.items() if k != "MISSION_KEY"}
    r = subprocess.run(base, env=clean, capture_output=True, text=True, timeout=5)
    check(r.returncode == 2 and "refusing to start without a key" in r.stderr,
          f"no -k, no MISSION_KEY -> exits 2 ({r.returncode}, {r.stderr.strip()[:60]!r})")
    r = subprocess.run(base + ["-k", "a", "--no-key"], env=clean, capture_output=True, text=True, timeout=5)
    check(r.returncode == 2, f"-k and --no-key together -> exits 2 ({r.returncode})")
    r = subprocess.run(base + ["-k", "two words"], env=clean, capture_output=True, text=True, timeout=5)
    check(r.returncode == 2, f"a key with a space -> exits 2 ({r.returncode})")
    # MISSION_KEY from the environment
    port = free_port()
    p = subprocess.Popen(base[:2] + [str(port)] + base[3:], env={**clean, "MISSION_KEY": "envkey"},
                         stderr=subprocess.PIPE, text=True)
    try:
        check(wait_for(lambda: port_open(port), 3), "MISSION_KEY -> relay starts")
        bad = Lap(port, "WATCH", key="test-key")
        check(bad.hello == "ERR key", f"MISSION_KEY: other key refused ({bad.hello})")
        bad.close()
        good = Lap(port, "WATCH", key="envkey")
        check(good.hello == "OK relay WATCH", f"MISSION_KEY: right key works ({good.hello})")
        good.close()
    finally:
        p.terminate(); p.wait(3)
    # --no-key: starts, loud warning, keyless HELLO works
    port = free_port()
    p = subprocess.Popen(base[:2] + [str(port)] + base[3:] + ["--no-key"], env=clean,
                         stderr=subprocess.PIPE, text=True)
    try:
        check(wait_for(lambda: port_open(port), 3), "--no-key -> relay starts")
        lap = Lap(port, "WATCH", key="")
        check(lap.hello == "OK relay WATCH", f"--no-key: keyless HELLO works ({lap.hello})")
        lap.close()
    finally:
        p.terminate(); p.wait(3)
        err = p.stderr.read()
    check("WARNING --no-key" in err and "ANY client" in err, "--no-key logs a loud warning")


def main() -> int:
    exe = sys.argv[1] if len(sys.argv) > 1 else os.path.join(MISSION, "build", "mission_relay_host")
    for p in (exe, MOTOR_BIN):
        if not os.path.exists(p):
            sys.exit(f"missing {p} (make -C iter2-autonomy/mission host; make -C iter1-wasd/pi host)")
    mport = free_port()
    motor = subprocess.Popen([MOTOR_BIN, "-b", "stub", "-p", str(mport), "-r", "0"], stderr=subprocess.DEVNULL)
    goto, voice, broker = FakeGoto(grid_bytes=300000), FakeVoice(), FakeBroker()
    try:
        assert wait_for(lambda: port_open(mport), 3), "tars_motor didn't start"
        env = (goto.port, voice.port, broker.port)
        test_motor_pipe(exe, mport, env)
        test_watch(exe, mport, env, goto, voice)
        test_cam(exe, mport, env, broker)
        test_slow_laptop(exe, mport, env, goto, broker)
        test_clean_disconnects(exe, mport, env, goto, voice)
        test_voice_pipe_key_down(exe, mport, env, voice)
        test_key_required(exe, mport, env)
        test_headless_tee(exe, mport, env)
    finally:
        motor.terminate()
        motor.wait(5)
        goto.srv.close(); voice.srv.close(); broker.close()
    print(f"{checks - failures}/{checks} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
