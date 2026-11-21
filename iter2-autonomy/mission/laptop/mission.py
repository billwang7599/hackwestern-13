#!/usr/bin/env python3
"""TARS mission control: a laptop-only dashboard + controls in your browser.

    python3 iter2-autonomy/mission/laptop/mission.py --relay <pi-ip>     # then open http://localhost:8765
    python3 .../mission.py --relay <pi-ip> --laptop-stt                  # whisper on the laptop, send text only

Everything goes through mission_relay on the Pi (TCP 5575). The robot never needs this:
close the laptop and it carries on exactly as before. This process does no robot work:
no perception, mapping, planning, intent classification or decisions. It shows what the
Pi computed, passes your commands to tars_motor / goto / tars_voice, and stops.

  STOP / Space        S to tars_motor, then CANCEL to goto (the same order as a spoken stop)
  hold W A S D        D l r at 20 Hz (iter1-wasd/laptop/teleop.py's HoldDrive), S on release;
                      CANCEL to goto before the first D if goto is busy (manual beats go-to)
  push-to-talk        16 kHz mono clip -> tars_voice on the Pi (whisper + classifier there);
                      with --laptop-stt the laptop transcribes and sends only HEAR <text>
  text box            HEAR <text> -> the Pi's classifier (stop words first)
  GOTO/RECALL/SCAN/FORGET buttons -> goto; MEMORY SAY -> "what do you remember" to tars_voice

If this process dies, its relay connections close and tars_motor stops only the drive it
started. If the browser tab goes quiet while a key is held, the drive stops in 0.35 s.
stdlib only.
"""
from __future__ import annotations

import argparse
import http.server
import json
import os
import signal
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time
import wave
from urllib.parse import urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
MISSION = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(MISSION))
sys.path.insert(0, os.path.join(REPO, "iter1-wasd", "laptop"))
from teleop import HoldDrive  # noqa: E402  (pure key -> D/S logic, no pygame needed)

KEY_HEARTBEAT_S = 0.35           # browser must refresh held keys this often, else stop
GOTO_ALLOWED = ("GOTO", "RECALL", "SCAN", "FORGET", "CANCEL", "MEMORY", "OBJECTS", "STATUS", "PING")
EVENTS_KEPT = 120


# ------------------------------------------------------------------ relay links
class RelayLink:
    """One relay channel (HELLO <ch>); reconnects by itself; lines or frames to a callback."""

    def __init__(self, host: str, port: int, ch: str, key: str, on_line=None, on_frame=None,
                 on_connect=None):
        self.host, self.port, self.ch, self.key = host, port, ch, key
        self.on_line, self.on_frame, self.on_connect = on_line, on_frame, on_connect
        self.sock: socket.socket | None = None
        self.connected = False
        self.error = "not connected yet"
        self.wlock = threading.Lock()
        self.stop = threading.Event()
        self.replies: list[tuple[int, str]] = []   # pipes: (seq, line), last 50
        self.seq = 0
        self.rcv = threading.Condition()
        self.cmd_lock = threading.Lock()      # one request/reply at a time (send() never waits on it)
        threading.Thread(target=self._run, daemon=True, name=f"relay-{ch}").start()

    def _run(self) -> None:
        while not self.stop.is_set():
            try:
                s = socket.create_connection((self.host, self.port), timeout=2)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(None)
                s.sendall(f"HELLO {self.ch} {self.key}".strip().encode() + b"\n")
                f = s.makefile("rb")
                hello = f.readline().decode(errors="replace").strip()
                if not hello.startswith("OK"):
                    raise OSError(hello or "relay closed")
                s.settimeout(None)
                with self.wlock:
                    self.sock = s
                    self.connected = True
                    self.error = ""
                if self.on_connect:
                    threading.Thread(target=self.on_connect, daemon=True).start()
                if self.ch == "CAM":
                    while True:
                        hdr = f.read(4)
                        if len(hdr) < 4:
                            raise OSError("relay closed")
                        n = struct.unpack("!I", hdr)[0]
                        pkt = f.read(n)
                        if len(pkt) < n:
                            raise OSError("relay closed")
                        if self.on_frame:
                            self.on_frame(pkt)
                else:
                    for raw in f:
                        line = raw.decode(errors="replace").rstrip("\r\n")
                        if self.on_line:
                            self.on_line(line)
                        with self.rcv:
                            self.seq += 1
                            self.replies.append((self.seq, line))
                            del self.replies[:-50]
                            self.rcv.notify_all()
                    raise OSError("relay closed")
            except OSError as e:
                self.error = str(e)
            finally:
                with self.wlock:
                    if self.sock:
                        try:
                            self.sock.close()
                        except OSError:
                            pass
                    self.sock = None
                    self.connected = False
            self.stop.wait(1.0)

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

    def cmd(self, line: str, timeout: float = 3.0, data: bytes = b"") -> str:
        """send one line (+ payload), wait for the next OK/ERR reply"""
        with self.cmd_lock:
            with self.rcv:
                seq0 = self.seq
            if not self.send(line.encode() + b"\n" + data):
                return f"ERR {self.ch} link down ({self.error})"
            end = time.monotonic() + timeout
            with self.rcv:
                while True:
                    for q, r in self.replies:
                        if q > seq0 and r.startswith(("OK", "ERR")):
                            return r
                    left = end - time.monotonic()
                    if left <= 0:
                        return "ERR no reply"
                    self.rcv.wait(left)

    def close(self) -> None:
        self.stop.set()
        with self.wlock:
            if self.sock:
                try:
                    self.sock.close()
                except OSError:
                    pass


# ------------------------------------------------------------------ state (display only)
def kv(text: str) -> dict:
    out = {}
    for tok in text.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


class State:
    """Latest line of every stream + its receipt time. Parsing here is for display only."""

    def __init__(self):
        self.lock = threading.Lock()
        self.items: dict[str, tuple[float, object]] = {}
        self.events: list[dict] = []          # goto EVENTs + laptop actions
        self.voice: list[dict] = []           # tars_voice events + laptop voice actions
        self.grid_seq = self.phone_seq = 0
        self.cam_id = 0
        self.cam_t = 0.0
        self.cam_jpeg = b""
        self.cam_cv = threading.Condition()

    def put(self, key: str, value) -> None:
        with self.lock:
            self.items[key] = (time.monotonic(), value)

    def note(self, text: str, kind: str = "laptop") -> None:
        with self.lock:
            self.events.append({"t": time.time(), "kind": kind, "text": text})
            del self.events[:-EVENTS_KEPT]

    def voice_note(self, ev: dict) -> None:
        ev = dict(ev, t=time.time())
        with self.lock:
            self.voice.append(ev)
            del self.voice[:-EVENTS_KEPT]

    def on_watch(self, line: str) -> None:
        if len(line) < 2 or line[1] != " ":
            return
        tag, rest = line[0], line[2:]
        if tag == "M":                                     # tars_motor STATUS
            parts = rest.split()
            d = kv(rest)
            if len(parts) >= 6 and parts[0] == "OK":
                d.update(out_l=parts[1], out_r=parts[2], cmd_age_ms=parts[3], estop=parts[5])
            self.put("motor", d)
        elif tag == "G":
            self.put("goto", kv(rest))
        elif tag == "Y":                                   # "OK age=0.10 {json}"
            try:
                age = float(rest.split()[1][4:])
                self.put("dets", {"age": age, "frame": json.loads(rest.split(" ", 2)[2])})
            except (ValueError, IndexError):
                pass
        elif tag == "O":                                   # MEMORY
            body = rest[3:] if rest.startswith("OK ") else rest
            parts = [p.strip() for p in body.split(";")]
            head = kv(parts[0]) if parts else {}
            objs = []
            for p in parts[1:]:
                bits = p.split()
                if len(bits) >= 2:
                    name_end = next((i for i, b in enumerate(bits) if b in ("fresh", "stale", "gone")), 1)
                    objs.append({"name": " ".join(bits[:name_end]), "status": bits[name_end] if name_end < len(bits) else "?",
                                 **kv(" ".join(bits[name_end + 1:]))})
            self.put("memory", {"head": head, "objects": objs})
        elif tag == "N":
            self.put("nav", kv(rest))
        elif tag == "T":                                   # NAV ROUTE
            d = kv(rest)
            pts = []
            if d.get("route", "-") != "-":
                for p in d["route"].split(";"):
                    try:
                        x, z = p.split(",")
                        pts.append([float(x), float(z)])
                    except ValueError:
                        pass
            self.put("route", {"points": pts, "i": int(d.get("i", 0) or 0)})
        elif tag == "K":                                   # NAV GRID, RLE kept as is (browser draws it)
            with self.lock:
                old = self.items.get("grid", (0, {}))[1]
            d = kv(rest)
            if not isinstance(old, dict) or old.get("v") != d.get("v"):
                self.grid_seq += 1
            d["seq"] = self.grid_seq
            self.put("grid", d)
        elif tag == "E":
            with self.lock:
                self.events.append({"t": time.time(), "kind": "goto", "text": rest})
                del self.events[:-EVENTS_KEPT]
        elif tag == "V":
            try:
                self.voice_note(json.loads(rest))
            except ValueError:
                pass
        elif tag == "P":
            try:
                p = json.loads(rest)
            except ValueError:
                return
            self.phone_seq += 1
            self.put("phone", {"seq": self.phone_seq, "status": p})
        elif tag == "R":
            self.put("relay", kv(rest))

    def on_frame(self, pkt: bytes) -> None:
        if len(pkt) < 14 or pkt[:4] != b"RGB1":
            return
        with self.cam_cv:
            self.cam_id = struct.unpack("!Q", pkt[4:12])[0]
            self.cam_jpeg = pkt[12:]
            self.cam_t = time.monotonic()
            self.cam_cv.notify_all()

    def snapshot(self, links: dict, sent: dict) -> dict:
        now = time.monotonic()
        with self.lock:
            out = {}
            for k, (t, v) in self.items.items():
                if k == "grid" and sent.get("grid") == v.get("seq"):
                    out[k] = {"seq": v["seq"], "same": True}
                elif k == "phone" and sent.get("phone") == v.get("seq"):
                    out[k] = {"seq": v["seq"], "same": True}
                else:
                    out[k] = v
                out[k + "_age"] = round(now - t, 2)
            out["events"] = self.events[-60:]
            out["voice"] = self.voice[-60:]
        if "grid" in out and not out["grid"].get("same"):
            sent["grid"] = out["grid"].get("seq")
        if "phone" in out and not out["phone"].get("same"):
            sent["phone"] = out["phone"].get("seq")
        with self.cam_cv:
            out["cam"] = {"id": self.cam_id, "age": round(now - self.cam_t, 2) if self.cam_t else None}
        out["links"] = {k: {"up": l.connected, "error": l.error} for k, l in links.items()}
        return out


# ------------------------------------------------------------------ mission control
class Mission:
    def __init__(self, host: str, port: int, key: str, laptop_stt: bool, stt_cli: str, model: str,
                 speed: float):
        self.state = State()
        self.laptop_stt, self.stt_cli, self.model = laptop_stt, stt_cli, model
        self.motor = RelayLink(host, port, "MOTOR", key, on_line=self._motor_reply,
                               on_connect=lambda: self.motor.send("WHO mission-laptop"))
        self.goto = RelayLink(host, port, "GOTO", key, on_connect=self._goto_hello)
        self.voice = RelayLink(host, port, "VOICE", key)
        self.watch = RelayLink(host, port, "WATCH", key, on_line=self.state.on_watch)
        self.cam = RelayLink(host, port, "CAM", key, on_frame=self.state.on_frame)
        self.links = {"MOTOR": self.motor, "GOTO": self.goto, "VOICE": self.voice, "WATCH": self.watch, "CAM": self.cam}
        self.drive = HoldDrive(speed=speed)
        self.dlock = threading.Lock()
        self.key_seen = 0.0
        self.objects: list[str] = []
        self.stop_ev = threading.Event()
        threading.Thread(target=self._drive_loop, daemon=True, name="drive").start()

    # ---- links
    def _motor_reply(self, line: str) -> None:
        if line.startswith("ERR"):
            self.state.note(f"tars_motor: {line}", "motor")
        self.state.put("motor_reply", line)

    def _goto_hello(self) -> None:
        r = self.goto.cmd("OBJECTS")
        if r.startswith("OK "):
            self.objects = [o for o in r[3:].split(",") if o]

    # ---- stop + drive
    def stop(self, why: str = "STOP button") -> dict:
        """S to tars_motor first, then CANCEL to goto."""
        with self.dlock:
            self.drive.held.clear()
            self.drive.moving = False
        s_ok = self.motor.send("S")
        c_ok = self.goto.send("CANCEL")
        self.state.note(f"{why}: S {'sent' if s_ok else 'NOT SENT (motor link down)'}, "
                        f"CANCEL {'sent' if c_ok else 'not sent (goto link down)'}", "stop")
        return {"S": s_ok, "CANCEL": c_ok}

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
            starting = not was_moving and self.drive.moving
        if starting and self.goto_busy():
            # Manual beats go-to: CANCEL before the first D. Only when goto is (or may be) busy:
            # goto answers CANCEL with S, and tars_motor then holds our D for 1.5 s ("ERR held").
            self.goto.send("CANCEL")
            self.state.note("WASD while goto was busy: CANCEL (tars_motor may hold the drive 1.5 s)", "goto")
        for l in lines:
            self.motor.send(l)
        return {"lines": lines}

    def goto_busy(self) -> bool:
        """goto's last STATUS says it's doing something, or we don't know (no fresh STATUS)."""
        with self.state.lock:
            t, g = self.state.items.get("goto", (0.0, {}))
        return time.monotonic() - t > 1.5 or g.get("state", "idle") != "idle"

    def _drive_loop(self) -> None:
        while not self.stop_ev.wait(0.02):
            now = time.monotonic()
            lines: list[str] = []
            with self.dlock:
                if self.drive.moving and now - self.key_seen > KEY_HEARTBEAT_S:
                    self.drive.held.clear()
                    self.drive.moving = False
                    lines = ["S"]
                    self.state.note("browser went quiet while driving: S", "stop")
                else:
                    lines = self.drive.tick(now)
            for l in lines:
                self.motor.send(l)

    # ---- goto / voice
    def goto_cmd(self, line: str) -> str:
        line = " ".join(line.split())[:200]
        if not line or line.split()[0].upper() not in GOTO_ALLOWED:
            return "ERR not allowed from mission control"
        r = self.goto.cmd(line)
        self.state.note(f"{line} -> {r}", "goto")
        return r

    def memory_say(self) -> str:
        """Ask the robot itself (voice prints, beeps and speaks it on the phone)."""
        if self.voice.connected:
            r = self.voice.cmd("HEAR what do you remember", timeout=4)
            self.state.voice_note({"type": "laptop", "text": "MEMORY SAY via tars_voice", "reply": r})
            return r
        r = self.goto.cmd("MEMORY SAY")
        self.state.note(f"MEMORY SAY -> {r}", "goto")
        return r

    def hear(self, text: str, src: str = "typed") -> str:
        text = " ".join(text.split())[:300]
        if not text:
            return "ERR empty"
        r = self.voice.cmd("HEAR " + text, timeout=4)
        self.state.voice_note({"type": "laptop", "src": src, "text": text, "reply": r})
        return r

    def audio(self, pcm: bytes) -> str:
        if len(pcm) < 3200:
            return "ERR clip too short"
        pcm = pcm[: 15 * 16000 * 2]
        if not self.laptop_stt:
            r = self.voice.cmd(f"AUDIO {len(pcm)}", timeout=20, data=pcm)
            self.state.voice_note({"type": "laptop", "src": "push-to-talk", "text": f"{len(pcm) / 32000:.1f} s clip -> Pi", "reply": r})
            return r
        scratch = os.path.join(MISSION, "build", "scratch")
        os.makedirs(scratch, exist_ok=True)
        wav = os.path.join(scratch, f"ptt_{os.getpid()}.wav")
        with wave.open(wav, "wb") as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(16000)
            w.writeframes(pcm)
        try:
            p = subprocess.run([self.stt_cli, self.model, wav], capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired) as e:
            return f"ERR laptop stt: {e}"
        text = p.stdout.strip()
        if p.returncode != 0 or not text:
            return f"ERR laptop stt failed: {p.stderr.strip()[-200:]}"
        return self.hear(text, src="laptop-stt")

    def close(self) -> None:
        self.stop_ev.set()
        for l in self.links.values():
            l.close()


# ------------------------------------------------------------------ HTTP (localhost only)
class Handler(http.server.BaseHTTPRequestHandler):
    mission: Mission = None  # type: ignore
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):            # quiet
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _body(self) -> bytes:
        n = int(self.headers.get("Content-Length", 0) or 0)
        return self.rfile.read(min(n, 2_000_000)) if n > 0 else b""

    def do_GET(self):
        path = urlparse(self.path).path
        m = self.mission
        if path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/state":                                   # server-sent events, 5 Hz
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            sent: dict = {}          # big items (grid, phone) only when they change, per browser
            try:
                while not m.stop_ev.is_set():
                    snap = m.state.snapshot(m.links, sent)
                    snap["objects_list"] = m.objects
                    snap["speed"] = m.drive.speed
                    snap["laptop_stt"] = m.laptop_stt
                    self.wfile.write(b"data: " + json.dumps(snap).encode() + b"\n\n")
                    self.wfile.flush()
                    time.sleep(0.2)
            except (OSError, ValueError):
                pass
            self.close_connection = True
        elif path == "/cam.mjpg":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            last = -1
            try:
                while not m.stop_ev.is_set():
                    with m.state.cam_cv:
                        if m.state.cam_id == last:
                            m.state.cam_cv.wait(1.0)
                        if m.state.cam_id == last or not m.state.cam_jpeg:
                            continue
                        last, jpeg = m.state.cam_id, m.state.cam_jpeg
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                    self.wfile.flush()
            except (OSError, ValueError):
                pass
            self.close_connection = True
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        path = urlparse(self.path).path
        m = self.mission
        if path == "/stop":
            self._json(m.stop())
            return
        if path == "/voice/audio":
            self._json({"reply": m.audio(self._body())})
            return
        try:
            req = json.loads(self._body() or b"{}")
        except ValueError:
            self._json({"error": "bad json"}, 400)
            return
        if path == "/keys":
            self._json(m.keys(list(req.get("held", [])), req.get("speed")))
        elif path == "/goto":
            self._json({"reply": m.goto_cmd(str(req.get("line", "")))})
        elif path == "/memory_say":
            self._json({"reply": m.memory_say()})
        elif path == "/voice/text":
            self._json({"reply": m.hear(str(req.get("text", "")))})
        elif path == "/motor/reset":                     # clear the tip latch (teleop's R key)
            self._json({"reply": m.motor.send("RESET")})
        else:
            self._json({"error": "not found"}, 404)


class Server(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main() -> int:
    ap = argparse.ArgumentParser(description="TARS laptop mission control (browser UI on localhost)")
    ap.add_argument("--relay", default=os.environ.get("TARS_HOST", "tars.local"),
                    help="Pi address (host or host:port, relay port 5575)")
    ap.add_argument("--key", default=os.environ.get("TARS_RELAY_KEY", ""), help="relay key (mission_relay -k)")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--speed", type=float, default=0.6, help="WASD start speed (fraction of max)")
    ap.add_argument("--laptop-stt", action="store_true",
                    help="cheat: run whisper on the laptop (voice/build/stt_cli) and send only HEAR <text>")
    ap.add_argument("--stt-cli", default=os.path.join(REPO, "iter2-autonomy", "voice", "build", "stt_cli"))
    ap.add_argument("--model", default=os.path.join(REPO, "iter2-autonomy", "voice", "models", "ggml-tiny.en-q5_1.bin"))
    a = ap.parse_args()
    host, _, port = a.relay.partition(":")
    if a.laptop_stt and not (os.path.exists(a.stt_cli) and os.path.exists(a.model)):
        sys.exit(f"--laptop-stt needs {a.stt_cli} (make -C iter2-autonomy/voice host) and {a.model}")
    m = Mission(host, int(port or 5575), a.key, a.laptop_stt, a.stt_cli, a.model, a.speed)
    Handler.mission = m
    srv = Server(("127.0.0.1", a.http_port), Handler)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"mission control: http://localhost:{a.http_port}  (relay {host}:{port or 5575}"
          f"{', laptop speech-to-text' if a.laptop_stt else ''})", flush=True)
    done.wait()
    # Exiting closes the relay links: tars_motor stops only a drive we started.
    # No S here on purpose: S would also stop another client's motion.
    m.close()
    srv.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
