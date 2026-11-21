"""bench plugin `mission`: mission control's features (iter2-autonomy/mission/laptop/mission.py) in
the bench, so one laptop page drives the sim, the Uno on USB, or the real robot.

Andrew's code is IMPORTED, not copied:
  - mission.State parses every WATCH stream (DETS Y, MEMORY O, NAV N, NAV ROUTE T, NAV GRID K,
    goto EVENT E, tars_voice V, phone tee P, relay R, motor M, goto G) and the CAM frames.
  - mission.Mission.goto_cmd / memory_say / hear / audio run unchanged, called with a small
    adapter (_Adapter) whose .goto/.voice/.state look like Mission's, but whose .cmd() goes
    through the robot plugin (the bench's only robot connection).
  - follow_actions.py and the two .command files' ssh lines (journals, room-view tunnel) are
    run as Andrew wrote them; the default user@host is read from his actions.command.

Where the data comes from:
  pi       mission_relay WATCH + CAM (robot plugin's links), exactly what mission.py sees.
  sim/usb  the bench polls goto itself at the relay's rates (DETS 5 Hz; MEMORY, NAV, NAV ROUTE,
           NAV GRID 1 Hz) and feeds the replies to State with the relay's tags; tars_voice
           events come from a WATCH connection the robot plugin keeps. No camera.

Routes (/api/mission/...):
  GET  state          snapshot (State.snapshot + objects_list, speed, laptop_stt, target)
  GET  cam.jpg        latest camera JPEG (404 when none). The panel polls it (≤ 5 fps source)
  POST goto {line}    GOTO/RECALL/SCAN/FORGET/CANCEL/MEMORY/OBJECTS/STATUS/PING only (mission's list)
  POST memory_say     "what do you remember" to tars_voice (goto MEMORY SAY if voice is down)
  POST text {text}    HEAR <text> to tars_voice
  POST audio (raw)    16 kHz mono s16le clip: AUDIO <n> + bytes (or laptop whisper + HEAR with
                      --laptop-stt)
  POST reset          RESET to tars_motor (clears the tip latch)
  GET  journal        {running, host, error, lines}; POST journal {on, host}
  GET  roomview       {running, host, url, error};   POST roomview {on, host}
Topics: `mission` (5 Hz, sticky; no grid/phone), `mission_grid` (sticky, on change),
`mission_phone` (sticky, on change), `journal` (new journal lines, not sticky).
"""
from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import sys
import threading
import time

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_LAPTOP = os.path.join(_REPO, "iter2-autonomy", "mission", "laptop")
sys.path.insert(0, _LAPTOP)
import mission as _mission  # noqa: E402  (State, Mission, GOTO_ALLOWED, REPO paths)

PUBLISH_HZ = 5.0
JOURNAL_KEPT = 400
ROOM_VIEW_PORT = 5580          # Andrew's room-view.command forwards the Pi's 127.0.0.1:5580 here
_bench = None
_state: "_mission.State | None" = None
_adapter: "_Adapter | None" = None
_stop = threading.Event()
_objects: list[str] = []
_journal: "_Ssh | None" = None
_roomview: "_Ssh | None" = None


def _robot():
    return _bench.plugins.get("robot") if _bench else None


def _target() -> str:
    return _bench.target["name"] if _bench else "?"


# ------------------------------------------------------------------ adapter for Mission's methods
class _Chan:
    """Looks like a mission.RelayLink to Mission's methods (.cmd, .connected); goes through the
    robot plugin so the bench keeps a single connection per service."""

    def __init__(self, dest: str):
        self.dest = dest

    @property
    def connected(self) -> bool:
        r = _robot()
        link = r.link() if r else None
        return bool(link and link.chans[self.dest].connected)

    def cmd(self, line: str, timeout: float = 3.0, data: bytes = b"") -> str:
        r = _robot()
        if not r:
            return "ERR robot plugin not loaded"
        return r.send(self.dest, line, data=data, timeout=timeout, src="mission")


class _Adapter:
    """The attributes Mission.goto_cmd / memory_say / hear / audio read from `self`."""

    def __init__(self, state, laptop_stt: bool, stt_cli: str, model: str):
        self.state = state
        self.goto, self.voice, self.motor = _Chan("goto"), _Chan("voice"), _Chan("motor")
        self.laptop_stt, self.stt_cli, self.model = laptop_stt, stt_cli, model

    # Mission.audio calls self.hear(...) when the laptop transcribes
    def hear(self, text: str, src: str = "typed") -> str:
        return _mission.Mission.hear(self, text, src)

    def goto_cmd(self, line: str) -> str:
        return _mission.Mission.goto_cmd(self, line)

    def memory_say(self) -> str:
        return _mission.Mission.memory_say(self)

    def audio(self, pcm: bytes) -> str:
        return _mission.Mission.audio(self, pcm)


# ------------------------------------------------------------------ ssh helpers (Andrew's scripts)
def _default_ssh_host() -> str:
    """The user@host in Andrew's actions.command (`${1:-qnxuser@…}`), unless TARS_SSH is set."""
    if os.environ.get("TARS_SSH"):
        return os.environ["TARS_SSH"]
    try:
        with open(os.path.join(_LAPTOP, "actions.command")) as f:
            m = re.search(r"\$\{1:-([^}]+)\}", f.read())
            if m:
                return m.group(1)
    except OSError:
        pass
    return "qnxuser@tars.local"


def _ssh_bin() -> list[str]:
    return shlex.split(os.environ.get("BENCH_SSH", "ssh"))   # tests swap in a fake ssh


class _Ssh:
    """One ssh child: the action-journal follower (lines → topic `journal`) or the room-view tunnel.
    BatchMode: it never waits for a password (use an ssh key; the panel shows ssh's error)."""

    def __init__(self, kind: str):
        self.kind = kind
        self.p: subprocess.Popen | None = None
        self.host = ""
        self.error = ""
        self.lines: list[str] = []
        self.lock = threading.Lock()

    def running(self) -> bool:
        return bool(self.p and self.p.poll() is None)

    def start(self, host: str) -> dict:
        self.stop()
        host = (host or "").strip() or _default_ssh_host()
        if not re.fullmatch(r"[A-Za-z0-9_.@:\-\[\]]+", host):
            return {"error": "bad host (user@host)"}
        self.host, self.error = host, ""
        common = ["-o", "BatchMode=yes", "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3"]
        try:
            if self.kind == "journal":
                # actions.command: ssh … host 'python3 -u -' < follow_actions.py
                stdin = open(os.path.join(_LAPTOP, "follow_actions.py"), "rb")
                argv = _ssh_bin() + ["-o", "ControlMaster=auto", "-o", "ControlPath=/tmp/hw13-actions-%C",
                                     *common, host, "python3 -u -"]
                self.p = subprocess.Popen(argv, stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                stdin.close()
            else:
                # room-view.command: ssh -N -L 127.0.0.1:5580:127.0.0.1:5580 host
                fwd = f"127.0.0.1:{ROOM_VIEW_PORT}:127.0.0.1:{ROOM_VIEW_PORT}"
                argv = _ssh_bin() + ["-N", "-o", "ExitOnForwardFailure=yes", *common, "-L", fwd, host]
                self.p = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                          stderr=subprocess.PIPE)
        except OSError as e:
            self.error = f"can't run ssh: {e}"
            return self.info()
        p = self.p
        threading.Thread(target=self._read_err, args=(p,), daemon=True).start()
        if self.kind == "journal":
            threading.Thread(target=self._read_out, args=(p,), daemon=True).start()
        _bench.log("mission", f"{self.kind}: ssh {host} started")
        return self.info()

    def _read_out(self, p) -> None:
        for raw in p.stdout:
            line = raw.decode(errors="replace").rstrip("\r\n")
            with self.lock:
                self.lines.append(line)
                del self.lines[:-JOURNAL_KEPT]
            _bench.publish("journal", {"host": self.host, "line": line}, sticky=False)

    def _read_err(self, p) -> None:
        for raw in p.stderr:
            t = raw.decode(errors="replace").strip()
            if t:
                self.error = t[-300:]
        rc = p.wait()
        if p is self.p:
            _bench.log("mission", f"{self.kind}: ssh ended (exit {rc}){': ' + self.error if self.error else ''}")

    def stop(self) -> None:
        p, self.p = self.p, None
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(2)
            except subprocess.TimeoutExpired:
                p.kill()

    def info(self) -> dict:
        d = {"running": self.running(), "host": self.host or _default_ssh_host(), "error": self.error}
        if self.kind == "roomview":
            d["url"] = f"http://127.0.0.1:{ROOM_VIEW_PORT}/"
        else:
            with self.lock:
                d["lines"] = list(self.lines[-200:])
        return d


# ------------------------------------------------------------------ data
def _links() -> dict:
    """{name: obj with .connected/.error} for State.snapshot (the robot plugin's links)."""
    r = _robot()
    link = r.link() if r else None
    return dict(link.links) if link else {}


def _poll_direct(n: int) -> None:
    """sim/usb: what mission_relay's WATCH poller does on the Pi, from the laptop (same tags)."""
    r = _robot()
    link = r.link() if r else None
    if not link or not link.chans["goto"].connected:
        return
    polls = [("Y", "DETS")]
    if n % 5 == 0:
        polls += [("O", "MEMORY"), ("N", "NAV"), ("T", "NAV ROUTE"), ("K", "NAV GRID")]
    for tag, line in polls:
        rep = r.send("goto", line, timeout=0.6, src="poll")   # src=poll: not a `cmd` event
        if rep.startswith(("OK", "ERR")) and "link down" not in rep and rep != "ERR no reply":
            _state.on_watch(f"{tag} {rep}")


def _objects_list() -> None:
    global _objects
    if _objects:
        return
    r = _robot()
    link = r.link() if r else None
    if link and link.chans["goto"].connected:
        rep = r.send("goto", "OBJECTS", timeout=1.0, src="poll")
        if rep.startswith("OK "):
            _objects = [o for o in rep[3:].split(",") if o]


def snapshot(sent: dict | None = None) -> dict:
    sent = {} if sent is None else sent
    snap = _state.snapshot(_links(), sent)
    r = _robot()
    link = r.link() if r else None
    snap["objects_list"] = _objects
    snap["speed"] = link.drive.speed if link else None
    snap["laptop_stt"] = _adapter.laptop_stt
    snap["target"] = _target()
    snap["camera"] = "relay CAM (phone frames via the Pi)" if _target() == "pi" else "none (sim/usb: no robot camera)"
    snap["journal"] = {k: v for k, v in _journal.info().items() if k != "lines"}
    snap["roomview"] = _roomview.info()
    return snap


def _loop() -> None:
    n = 0
    sent: dict = {}
    last_obj = 0.0
    attached = False
    while not _stop.wait(1.0 / PUBLISH_HZ):
        n += 1
        try:
            if not attached and _robot() and _robot().link():
                _robot().listen("watch", _state.on_watch)
                _robot().listen("cam", _state.on_frame)
                attached = True
            elif not attached and n == 25:
                print("mission: no robot plugin after 5 s: mission panels have no data", file=sys.stderr)
            if _target() != "pi":
                _poll_direct(n)
            if time.monotonic() - last_obj > 2.0:
                last_obj = time.monotonic()
                _objects_list()
            r = _robot()
            link = r.link() if r else None
            if link and link.paused():          # recorder replay: don't publish live data
                continue
            snap = snapshot(sent=sent)
            g, p = snap.pop("grid", None), snap.pop("phone", None)
            if g is not None and not g.get("same"):
                _bench.publish("mission_grid", g)
            if p is not None and not p.get("same"):
                _bench.publish("mission_phone", p)
            _bench.publish("mission", snap)
        except Exception as ex:                 # one bad tick must not end the loop
            print(f"mission: tick failed: {type(ex).__name__}: {ex}", file=sys.stderr)


# ------------------------------------------------------------------ plugin glue
def _j(body: bytes) -> dict:
    try:
        return json.loads(body or b"{}")
    except ValueError:
        return {}


def setup(bench) -> None:
    global _bench, _state, _adapter, _journal, _roomview
    _bench = bench
    _state = _mission.State()
    a = bench.args
    stt_cli = getattr(a, "stt_cli", "") or os.path.join(_REPO, "iter2-autonomy", "voice", "build", "stt_cli")
    model = getattr(a, "model", "") or os.path.join(_REPO, "iter2-autonomy", "voice", "models", "ggml-tiny.en-q5_1.bin")
    _adapter = _Adapter(_state, bool(getattr(a, "laptop_stt", False)), stt_cli, model)
    _journal, _roomview = _Ssh("journal"), _Ssh("roomview")
    # plugins load in name order (mission before robot): the loop attaches to robot once it exists

    def r_state(q, body):
        return 200, snapshot()

    def r_cam(q, body):
        with _state.cam_cv:
            jpeg = _state.cam_jpeg
        if not jpeg:
            return 404, {"error": "no camera frame" + ("" if _target() == "pi" else " (sim/usb: no robot camera)")}
        return 200, (jpeg, "image/jpeg")

    def r_goto(q, body):
        return 200, {"reply": _adapter.goto_cmd(str(_j(body).get("line", "")))}

    def r_memsay(q, body):
        return 200, {"reply": _adapter.memory_say()}

    def r_text(q, body):
        return 200, {"reply": _adapter.hear(str(_j(body).get("text", "")))}

    def r_audio(q, body):
        return 200, {"reply": _adapter.audio(body)}

    def r_reset(q, body):
        rep = _adapter.motor.cmd("RESET")
        _state.note(f"RESET (tip latch) -> {rep}", "motor")
        return 200, {"reply": rep}

    def ssh_route(obj):
        def get(q, body):
            return 200, obj.info()

        def post(q, body):
            d = _j(body)
            if d.get("on", True):
                return 200, obj.start(str(d.get("host", "")))
            obj.stop()
            return 200, obj.info()
        return get, post

    bench.route("GET", "/api/mission/state", r_state)
    bench.route("GET", "/api/mission/cam.jpg", r_cam)
    bench.route("POST", "/api/mission/goto", r_goto)
    bench.route("POST", "/api/mission/memory_say", r_memsay)
    bench.route("POST", "/api/mission/text", r_text)
    bench.route("POST", "/api/mission/audio", r_audio)
    bench.route("POST", "/api/mission/reset", r_reset)
    for name, obj in (("journal", _journal), ("roomview", _roomview)):
        g, p = ssh_route(obj)
        bench.route("GET", f"/api/mission/{name}", g)
        bench.route("POST", f"/api/mission/{name}", p)
    threading.Thread(target=_loop, daemon=True, name="mission").start()


def shutdown() -> None:
    _stop.set()
    for s in (_journal, _roomview):
        if s:
            s.stop()
