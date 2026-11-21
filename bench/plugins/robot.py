"""bench plugin `robot`: the ONLY thing in the bench that talks to the robot stack.

Targets (bench.target["name"]):
  sim / usb  direct TCP to tars_motor (WHO bench), goto's control port (+ a second WATCH
             connection for EVENTs) and tars_voice's laptop port.
  pi         everything through mission_relay (channels MOTOR, GOTO, VOICE, WATCH) with the
             key, using iter2-autonomy/mission/laptop/mission.py's RelayLink (imported).

API for other plugins (bench.plugins.get("robot")):
  send(dest, line, data=b"", timeout=None, src="plugin") -> reply
                                                         dest: "motor" | "goto" | "voice"; data =
                                                         raw bytes written right after the line
                                                         (AUDIO <n> + n bytes of 16 kHz PCM/WAV, as
                                                         mission.py does; the relay's VOICE channel
                                                         is a byte pipe, <= 1 MB up). timeout None =
                                                         3 s, 20 s when data is given (whisper).
  send_nowait(dest, line, src) -> bool                   reply still published as `cmd`
  stop(why) -> {"S": bool, "CANCEL": bool}               S to tars_motor FIRST, then CANCEL to goto
  status() -> dict | None                                latest parsed tars_motor STATUS
  goto_status() -> dict | None
  listen(dest, fn)                                       fn(line) for unsolicited lines
                                                         (goto WATCH EVENTs, tars_voice JSON)
  stop_count() -> int                                    bumps on every stop() (scenarios abort)

Events: `status` (10 Hz), `goto` (STATUS ~4 Hz + EVENTs), `cmd` (every line sent + its reply,
except the 10 Hz STATUS poll and move keepalive PINGs, which `status` already covers).

Drive (POST /api/robot/keys): iter1-wasd/laptop/teleop.py's HoldDrive on the server, D at 20 Hz,
S on release; if the tab stops refreshing held keys for KEY_HEARTBEAT_S (0.35 s, mission.py's
rule) the server sends S. Manual beats go-to: CANCEL to goto before the first D if goto is busy.
"""
from __future__ import annotations

import collections
import json
import os
import socket
import sys
import threading
import time

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO, "iter2-autonomy", "mission", "laptop"))
import mission as _mission  # noqa: E402  (RelayLink, KEY_HEARTBEAT_S; also puts teleop on sys.path)
from teleop import HoldDrive  # noqa: E402

KEY_HEARTBEAT_S = _mission.KEY_HEARTBEAT_S
POLL_HZ = 10.0
GOTO_EVERY = 3                     # goto STATUS every 3rd poll (~3.3 Hz)
EVENTS_KEPT = 60
DESTS = ("motor", "goto", "voice")

_bench = None
_link: "RobotLink | None" = None


# ------------------------------------------------------------------ parsing
def parse_status(reply: str) -> dict | None:
    """tars_motor 'OK <l> <r> <ms_since_cmd> <dist_m> <estop> k=v ...' -> dict (numbers as numbers)."""
    parts = reply.split()
    if len(parts) < 6 or parts[0] != "OK":
        return None
    try:
        st: dict = {"out_l": float(parts[1]), "out_r": float(parts[2]), "ms_since_cmd": int(parts[3]),
                    "dist_m": float(parts[4]), "estop": int(parts[5])}
    except ValueError:
        return None
    for tok in parts[6:]:
        k, eq, v = tok.partition("=")
        if not eq:
            continue
        st[k] = _num(v)
    pose = st.get("pose")
    if isinstance(pose, str) and pose.count(",") == 2:
        try:
            x, y, h = (float(p) for p in pose.split(","))
            st["pose_x_mm"], st["pose_y_mm"], st["pose_hdg_deg"] = x, y, h
        except ValueError:
            pass
    return st


def parse_kv(reply: str) -> dict:
    out = {}
    for tok in reply.split():
        k, eq, v = tok.partition("=")
        if eq:
            out[k] = _num(v)
    return out


def _num(v: str):
    try:
        return int(v)
    except ValueError:
        try:
            return float(v)
        except ValueError:
            return v


# ------------------------------------------------------------------ transports
class DirectLink:
    """Plain TCP line link with the same shape as mission.RelayLink (send/connected/error/close,
    on_line + on_connect callbacks, reconnects by itself)."""

    def __init__(self, host: str, port: int, name: str, on_line=None, on_connect=None, start: bool = True):
        self.host, self.port, self.ch = host, port, name
        self.on_line, self.on_connect = on_line, on_connect
        self.sock: socket.socket | None = None
        self.connected = False
        self.error = "not connected yet"
        self.wlock = threading.Lock()
        self.stop = threading.Event()
        if start:
            self.start()

    def start(self) -> None:
        threading.Thread(target=self._run, daemon=True, name=f"link-{self.ch}").start()

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=1.5)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                with self.wlock:
                    self.sock, self.connected, self.error = s, True, ""
                if self.on_connect:
                    self.on_connect()
                for raw in s.makefile("rb"):
                    if self.on_line:
                        self.on_line(raw.decode(errors="replace").rstrip("\r\n"))
                raise OSError("closed by peer")
            except OSError as e:
                self.error = str(e) or type(e).__name__
            finally:
                with self.wlock:
                    if self.sock:
                        try:
                            self.sock.close()
                        except OSError:
                            pass
                    self.sock, self.connected = None, False
            self.stop.wait(0.5)

    def send(self, data: bytes | str) -> bool:
        if isinstance(data, str):
            data = data.encode() + b"\n"
        with self.wlock:
            if not self.sock:
                return False
            try:
                self.sock.settimeout(2.0)
                self.sock.sendall(data)
                self.sock.settimeout(None)
                return True
            except OSError as e:
                self.error = str(e)
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                return False

    def close(self) -> None:
        self.stop.set()
        with self.wlock:
            if self.sock:
                try:
                    self.sock.shutdown(socket.SHUT_RDWR)
                    self.sock.close()
                except OSError:
                    pass


class Chan:
    """One request/reply channel. tars_motor, goto and tars_voice answer every line with exactly
    one OK/ERR line, in order, so replies are matched FIFO. Anything else (tars_voice's JSON
    events) goes to listeners. A waiter that times out stays queued so a late reply can't be
    taken for the next command's; a reconnect clears the queue."""

    def __init__(self, dest: str, publish, hello: str | None = None):
        self.dest, self.publish, self.hello = dest, publish, hello
        self.pending: collections.deque = collections.deque()
        self.lock = threading.Lock()
        self.listeners: list = []
        self.link = None  # set by RobotLink

    def on_connect(self) -> None:
        for _ in range(100):           # RelayLink starts connecting inside its own __init__
            if self.link is not None:
                break
            time.sleep(0.01)
        with self.lock:
            old, self.pending = self.pending, collections.deque()
        for e in old:
            e["reply"] = "ERR link reconnected"
            e["ev"].set()
        if self.hello:
            self.send_nowait(self.hello, src="link")

    def on_line(self, line: str) -> None:
        if line.startswith(("OK", "ERR")):
            with self.lock:
                e = self.pending.popleft() if self.pending else None
            if e is None:
                self._unsolicited(line)
                return
            e["reply"] = line
            e["ev"].set()
            if e["src"] not in ("poll", "keepalive"):
                ev = {"dest": self.dest, "line": e["line"], "reply": line, "src": e["src"],
                      "ms": round((time.monotonic() - e["t0"]) * 1000, 1)}
                if e["bytes"]:
                    ev["bytes"] = e["bytes"]  # the byte count, never the payload
                self.publish("cmd", ev, sticky=False)
        else:
            self._unsolicited(line)

    def _unsolicited(self, line: str) -> None:
        for fn in list(self.listeners):
            try:
                fn(line)
            except Exception as ex:  # a listener bug must not kill the reader
                print(f"robot: listener on {self.dest} failed: {ex}", file=sys.stderr)

    def _enqueue(self, line: str, src: str, data: bytes = b"") -> dict | None:
        e = {"line": line, "src": src, "t0": time.monotonic(), "ev": threading.Event(), "reply": None,
             "bytes": len(data)}
        with self.lock:  # queue + write under one lock so the FIFO matches the wire order
            if not self.link or not self.link.send(line.encode() + b"\n" + bytes(data)):
                return None
            self.pending.append(e)
        return e

    def send_nowait(self, line: str, src: str = "plugin") -> bool:
        e = self._enqueue(line, src)
        if e is None:
            self._down(line, src)
            return False
        return True

    def request(self, line: str, timeout: float = 3.0, src: str = "plugin", data: bytes = b"") -> str:
        e = self._enqueue(line, src, data)
        if e is None:
            return self._down(line, src, len(data))
        if not e["ev"].wait(timeout):
            r = "ERR no reply"
            if src not in ("poll", "keepalive"):
                ev = {"dest": self.dest, "line": line, "reply": r, "src": src, "ms": timeout * 1000}
                if data:
                    ev["bytes"] = len(data)
                self.publish("cmd", ev, sticky=False)
            return r
        return e["reply"]

    def _down(self, line: str, src: str, nbytes: int = 0) -> str:
        err = getattr(self.link, "error", "") if self.link else "no link"
        r = f"ERR {self.dest} link down ({err})"
        if src not in ("poll", "keepalive"):
            ev = {"dest": self.dest, "line": line, "reply": r, "src": src, "ms": 0}
            if nbytes:
                ev["bytes"] = nbytes
            self.publish("cmd", ev, sticky=False)
        return r

    @property
    def connected(self) -> bool:
        return bool(self.link and self.link.connected)


# ------------------------------------------------------------------ the link
class RobotLink:
    def __init__(self, bench):
        self.bench = bench
        t = bench.target
        self.name = t["name"]
        pub = bench.publish
        self.chans = {"motor": Chan("motor", pub, hello="WHO bench"), "goto": Chan("goto", pub),
                      "voice": Chan("voice", pub)}
        self.links: dict[str, object] = {}
        self.goto_events: list[dict] = []
        self.glock = threading.Lock()
        # listen("watch", fn): fn(tagged line) in mission_relay's WATCH format ("V {json}", "Y OK …").
        # pi: every relay WATCH line. sim/usb: tars_voice's WATCH events as "V <json>" (the mission
        # plugin polls goto itself there). listen("cam", fn): fn(RGB1 packet), pi only (relay CAM).
        self.watch_listeners: list = []
        self.frame_listeners: list = []
        if self.name == "pi":
            host, _, port = (t.get("relay") or "").partition(":")
            if not host:
                raise RuntimeError("--target pi needs --relay <pi-ip>[:5575]")
            port = int(port or 5575)
            key = getattr(bench.args, "key", "") or ""
            for d, ch in (("motor", "MOTOR"), ("goto", "GOTO"), ("voice", "VOICE")):
                c = self.chans[d]
                c.link = _mission.RelayLink(host, port, ch, key, on_line=c.on_line, on_connect=c.on_connect)
                self.links[d] = c.link
            self.links["watch"] = _mission.RelayLink(host, port, "WATCH", key, on_line=self._relay_watch)
            self.links["cam"] = _mission.RelayLink(host, port, "CAM", key, on_frame=self._frame)
        else:
            for d in DESTS:
                h, _, p = t[d].rpartition(":")
                c = self.chans[d]
                c.link = DirectLink(h or "127.0.0.1", int(p), d, on_line=c.on_line, on_connect=c.on_connect,
                                    start=False)
                self.links[d] = c.link
            h, _, p = t["goto"].rpartition(":")
            self.links["watch"] = DirectLink(h or "127.0.0.1", int(p), "watch", on_line=self._direct_watch,
                                             on_connect=lambda: self.links["watch"].send("WATCH"), start=False)
            h, _, p = t["voice"].rpartition(":")    # tars_voice events (heard/sent/say), read-only
            self.links["voicewatch"] = DirectLink(h or "127.0.0.1", int(p), "voicewatch",
                                                  on_line=self._voice_watch,
                                                  on_connect=lambda: self.links["voicewatch"].send("WATCH"),
                                                  start=False)
            for l in self.links.values():    # wired up first, then connect (on_connect uses them)
                l.start()
        self.drive = HoldDrive(speed=float(getattr(bench.args, "speed", None) or 0.5))
        self.dlock = threading.Lock()
        self.key_seen = 0.0
        self.last_d = (0.0, 0.0)
        self.my_moves: set[int] = set()
        self._status: dict | None = None
        self._status_t = 0.0
        self._goto: dict | None = None
        self._goto_t = 0.0
        self._stops = 0
        self.paused = lambda: False           # the recorder sets this during replay
        self.stop_ev = threading.Event()
        threading.Thread(target=self._drive_loop, daemon=True, name="bench-drive").start()
        threading.Thread(target=self._poll_loop, daemon=True, name="bench-poll").start()

    # ---- watch feeds
    def _direct_watch(self, line: str) -> None:
        if line.startswith("EVENT "):
            self._goto_event(line[6:])

    def _relay_watch(self, line: str) -> None:
        if line.startswith("E "):
            self._goto_event(line[2:].removeprefix("EVENT "))
        self._to_listeners(self.watch_listeners, line)

    def _voice_watch(self, line: str) -> None:
        if line.startswith("{"):
            self._to_listeners(self.watch_listeners, "V " + line)

    def _frame(self, pkt: bytes) -> None:
        self._to_listeners(self.frame_listeners, pkt)

    @staticmethod
    def _to_listeners(fns: list, arg) -> None:
        for fn in list(fns):
            try:
                fn(arg)
            except Exception as ex:          # a listener bug must not kill the reader
                print(f"robot: watch listener failed: {type(ex).__name__}: {ex}", file=sys.stderr)

    def _goto_event(self, text: str) -> None:
        ev = {"t": time.time(), "text": text}
        with self.glock:
            self.goto_events.append(ev)
            del self.goto_events[:-EVENTS_KEPT]
        for fn in list(self.chans["goto"].listeners):
            try:
                fn("EVENT " + text)
            except Exception as ex:
                print(f"robot: goto listener failed: {ex}", file=sys.stderr)
        self.bench.log("goto", "EVENT " + text)
        self._publish_goto()

    def _publish_goto(self) -> None:
        if self.paused():
            return
        with self.glock:
            evs = list(self.goto_events[-20:])
        self.bench.publish("goto", {"t": time.time(), "ok": self._goto is not None and time.monotonic() - self._goto_t < 2,
                                    "status": self._goto, "events": evs, "link": self.chans["goto"].connected})

    # ---- public
    def send(self, dest: str, line: str, data: bytes = b"", timeout: float | None = None,
             src: str = "plugin") -> str:
        if dest not in self.chans:
            return f"ERR unknown dest {dest}"
        line = " ".join(str(line).split())
        if not line:
            return "ERR empty"
        data = bytes(data or b"")
        if timeout is None:
            timeout = 20.0 if data else 3.0
        r = self.chans[dest].request(line, timeout, src, data)
        self._note_move(dest, line, r)
        return r

    def send_nowait(self, dest: str, line: str, src: str = "plugin") -> bool:
        return self.chans[dest].send_nowait(" ".join(line.split()), src)

    def _note_move(self, dest: str, line: str, reply: str) -> None:
        if dest == "motor" and line.split()[0].upper() in ("MOVE", "TURN") and reply.startswith("OK "):
            try:
                self.my_moves.add(int(reply.split()[1]))
            except (IndexError, ValueError):
                pass

    def stop(self, why: str = "STOP") -> dict:
        """S to tars_motor first, then CANCEL to goto. Never waits for a reply first."""
        with self.dlock:
            self.drive.held.clear()
            self.drive.moving = False
            self.last_d = (0.0, 0.0)
        self._stops += 1
        s_ok = self.chans["motor"].send_nowait("S", src="stop")
        c_ok = self.chans["goto"].send_nowait("CANCEL", src="stop")
        self.bench.log("stop", f"{why}: S {'sent' if s_ok else 'NOT SENT (motor link down)'}, "
                               f"CANCEL {'sent' if c_ok else 'not sent (goto link down)'}")
        return {"S": s_ok, "CANCEL": c_ok}

    def status(self) -> dict | None:
        return self._status if time.monotonic() - self._status_t < 1.0 else None

    def goto_status(self) -> dict | None:
        return self._goto if time.monotonic() - self._goto_t < 2.0 else None

    def link_state(self) -> dict:
        out = {}
        for k, l in self.links.items():
            out[k] = {"connected": bool(l.connected), "error": "" if l.connected else str(getattr(l, "error", ""))}
        return out

    # ---- drive
    def keys(self, held: list[str], speed: float | None) -> dict:
        now = time.monotonic()
        lines: list[str] = []
        with self.dlock:
            self.key_seen = now
            was_moving = self.drive.moving
            if speed is not None:
                self.drive.speed = round(min(1.0, max(0.1, float(speed))), 2)
            new = {k for k in held if k in ("w", "a", "s", "d")}
            for k in sorted(self.drive.held - new):
                lines += self.drive.key_up(k, now)
            for k in sorted(new - self.drive.held):
                lines += self.drive.key_down(k, now)
            if speed is not None and self.drive.moving and not lines:
                lines += self.drive._emit(now)   # speed changed while held: send it now
            starting = not was_moving and self.drive.moving
        if starting and self._goto_busy():
            self.chans["goto"].send_nowait("CANCEL", src="drive")
            self.bench.log("drive", "WASD while goto was busy: CANCEL (tars_motor may hold the drive 1.5 s)")
        self._send_drive(lines)
        return {"lines": lines, "moving": self.drive.moving}

    def cancel_goto_if_busy(self, src: str) -> None:
        """Before a manual MOVE/TURN. goto answers CANCEL by sending S to tars_motor, which would
        abort a MOVE sent right after it, so: only when goto is (or may be) busy, and then wait
        for the reply and let that S land first (mission.py's rule for WASD)."""
        if not self.chans["goto"].connected or not self._goto_busy():
            return
        self.chans["goto"].request("CANCEL", 1.0, src)
        time.sleep(0.15)

    def _goto_busy(self) -> bool:
        g = self.goto_status()
        return g is None or g.get("state", "idle") != "idle"

    def _send_drive(self, lines: list[str]) -> None:
        for l in lines:
            if l == "S":
                self.last_d = (0.0, 0.0)
            else:
                p = l.split()
                self.last_d = (float(p[1]), float(p[2]))
            self.chans["motor"].send_nowait(l, src="drive")

    def _drive_loop(self) -> None:
        while not self.stop_ev.wait(0.02):
            now = time.monotonic()
            with self.dlock:
                if self.drive.moving and now - self.key_seen > KEY_HEARTBEAT_S:
                    self.drive.held.clear()
                    self.drive.moving = False
                    lines = ["S"]
                    self.bench.log("stop", "browser went quiet while driving: S")
                else:
                    lines = self.drive.tick(now)
            self._send_drive(lines)

    # ---- polling
    def _poll_loop(self) -> None:
        n = 0
        while not self.stop_ev.wait(1.0 / POLL_HZ):
            n += 1
            try:
                self._poll_once(n)
            except Exception as ex:          # one bad tick must not end polling for good
                print(f"robot: poll failed: {type(ex).__name__}: {ex}", file=sys.stderr)

    def _poll_once(self, n: int) -> None:
        m = self.chans["motor"]
        st = None
        raw = ""
        if m.connected:
            raw = m.request("STATUS", 0.5, src="poll")
            st = parse_status(raw)
            if st:
                self._status, self._status_t = st, time.monotonic()
                # our own MOVE/TURN needs a keepalive inside tars_motor's watchdog (STATUS isn't one)
                if st.get("move") == "running" and st.get("move_id") in self.my_moves:
                    m.send_nowait("PING", src="keepalive")
        if not self.paused():
            self.bench.publish("status", {
                "t": time.time(), "ok": st is not None, "fields": st, "raw": raw if not st else "",
                "links": self.link_state(), "target": self.name,
                "drive": {"cmd_l": self.last_d[0], "cmd_r": self.last_d[1], "speed": self.drive.speed,
                          "held": sorted(self.drive.held)}})
        if n % GOTO_EVERY == 0:
            g = self.chans["goto"]
            if g.connected:
                r = g.request("STATUS", 0.5, src="poll")
                if r.startswith("OK"):
                    self._goto, self._goto_t = parse_kv(r), time.monotonic()
            self._publish_goto()

    def close(self) -> None:
        self.stop_ev.set()
        for l in self.links.values():
            l.close()


# ------------------------------------------------------------------ plugin glue
def _j(body: bytes) -> dict:
    try:
        return json.loads(body or b"{}")
    except ValueError:
        return {}


def setup(bench) -> None:
    global _bench, _link
    _bench = bench
    _link = RobotLink(bench)

    def r_stop(q, body):
        return 200, _link.stop(_j(body).get("why", "STOP button"))

    def r_send(q, body):
        d = _j(body)
        dest, line = d.get("dest", "motor"), str(d.get("line", ""))
        if dest not in DESTS:
            return 400, {"error": f"dest must be one of {DESTS}"}
        if dest == "motor" and line.split()[:1] and line.split()[0].upper() in ("MOVE", "TURN"):
            _link.cancel_goto_if_busy("console")  # manual MOVE/TURN beats go-to
        r = _link.send(dest, line, src="console")
        bench.log("console", f"{dest}> {line} -> {r}")
        return 200, {"reply": r}

    def r_keys(q, body):
        d = _j(body)
        sp = d.get("speed")
        return 200, _link.keys(list(d.get("held") or []), float(sp) if sp is not None else None)

    def r_move(q, body):
        d = _j(body)
        kind = str(d.get("kind", "")).upper()
        if kind not in ("MOVE", "TURN"):
            return 400, {"error": "kind must be MOVE or TURN"}
        amt = float(d.get("amount", 0))
        sp = d.get("speed")
        line = f"{kind} {amt:.1f}" + (f" {float(sp):.2f}" if sp else "")
        _link.cancel_goto_if_busy("drive")
        r = _link.send("motor", line, src="drive")
        bench.log("drive", f"{line} -> {r}")
        return 200, {"reply": r}

    def r_state(q, body):
        return 200, {"target": _link.name, "links": _link.link_state(), "status": _link.status(),
                     "goto": _link.goto_status()}

    bench.route("POST", "/api/robot/stop", r_stop)
    bench.route("POST", "/api/robot/send", r_send)
    bench.route("POST", "/api/robot/keys", r_keys)
    bench.route("POST", "/api/robot/move", r_move)
    bench.route("GET", "/api/robot/state", r_state)


def shutdown() -> None:
    if _link:
        try:
            _link.stop("bench closing")
        finally:
            _link.close()


# module-level API (bench.plugins["robot"].send(...))
def send(dest: str, line: str, data: bytes = b"", timeout: float | None = None, src: str = "plugin") -> str:
    return _link.send(dest, line, data, timeout, src) if _link else "ERR robot plugin not set up"


def send_nowait(dest: str, line: str, src: str = "plugin") -> bool:
    return _link.send_nowait(dest, line, src) if _link else False


def stop(why: str = "STOP") -> dict:
    return _link.stop(why) if _link else {"S": False, "CANCEL": False}


def status() -> dict | None:
    return _link.status() if _link else None


def goto_status() -> dict | None:
    return _link.goto_status() if _link else None


def listen(dest: str, fn) -> None:
    """dest motor/goto/voice: unsolicited lines on that channel. "watch": relay WATCH lines (pi)
    or "V <json>" tars_voice events (sim/usb). "cam": RGB1 frame packets (pi only)."""
    if not _link:
        return
    if dest == "watch":
        _link.watch_listeners.append(fn)
    elif dest == "cam":
        _link.frame_listeners.append(fn)
    elif dest in _link.chans:
        _link.chans[dest].listeners.append(fn)


def stop_count() -> int:
    return _link._stops if _link else 0


def link() -> RobotLink | None:
    return _link
