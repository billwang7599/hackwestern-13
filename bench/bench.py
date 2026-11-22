#!/usr/bin/env python3
"""TARS digital test bench: a browser app (http://127.0.0.1:8770).

Normally on the laptop against the sim. `--target robot` runs it ON the Pi itself against the real stack,
and the 2D room view then shows the real robot (plugins/live.py); open it from a laptop with
`--bind 0.0.0.0` (a token is required then: --token or BENCH_TOKEN, open http://<pi-ip>:8770/?token=...).

The robot never needs this. It is a dev tool for driving, debugging and
experimenting with the robot stack against a simulated car/room or the real
hardware. Python stdlib only. See bench/README.md for the contract.

Plugins: every bench/plugins/<name>.py with a setup(bench) function is loaded
at startup. A plugin adds routes under /api/<name>/... and publishes
server-sent events. Panels: every bench/web/panels/<name>.js is loaded by the
browser shell; a panel calls Bench.register({...}). Adding a part = adding
one plugin file and one panel file, no shared file edits.
"""
from __future__ import annotations

import argparse
import hmac
import importlib.util
import json
import os
import queue
import sys
import threading
import traceback
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.join(HERE, "web")
PLUGINS = os.path.join(HERE, "plugins")
REPO = os.path.dirname(HERE)

# Panels that only mean something with the simulated car/room. Hidden on the real targets
# (usb = real Uno + motors on this Mac, pi = the real robot) unless --all-panels.
SIM_ONLY_PANELS = {"car.js", "world2d.js", "gotomem.js"}
# On the real targets the live plugin feeds world2d with the real robot, so it stays.
LIVE_PANELS = {"world2d.js"}
REAL_TARGETS = ("usb", "pi", "robot")

CTYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
          ".css": "text/css; charset=utf-8", ".json": "application/json",
          ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg"}


class Bench:
    """What a plugin gets in setup(bench)."""

    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.repo = REPO
        # Where the robot stack is. sim: host builds on this Mac (bench/run_sim.py).
        # usb: real Uno on this Mac's USB, host tars_motor. pi: the real Pi via mission_relay.
        # robot: this bench runs ON the Pi, direct TCP to the local stack (like sim/usb).
        self.target = {"name": args.target, "motor": args.motor, "goto": args.goto,
                       "voice": args.voice, "relay": args.relay}
        self.plugins: dict[str, object] = {}
        self._routes: dict[tuple[str, str], object] = {}
        self._subs: list[queue.Queue] = []
        self._taps: list = []
        self._lock = threading.Lock()
        self._last: dict[str, bytes] = {}  # last event per topic, replayed to new browser tabs

    def route(self, method: str, path: str, fn) -> None:
        """fn(query: dict[str, str], body: bytes) -> (status, payload).
        payload: dict/list -> JSON, str -> text, bytes -> octet-stream,
        or a (bytes, content_type) tuple."""
        self._routes[(method.upper(), path)] = fn

    def publish(self, topic: str, data, sticky: bool = True) -> None:
        """Send an event to every open browser tab (EventSource on /events).
        sticky: a tab that connects later gets the latest value of this topic."""
        msg = f"event: {topic}\ndata: {json.dumps(data, separators=(',', ':'))}\n\n".encode()
        with self._lock:
            if sticky:
                self._last[topic] = msg
            subs = list(self._subs)
            taps = list(self._taps)
        for fn in taps:
            try:
                fn(topic, data)
            except Exception:
                traceback.print_exc()
        for q in subs:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass  # slow tab: drop, the next event carries fresh data

    def tap(self, fn) -> None:
        """Server-side listener: fn(topic, data) for every publish (the recorder uses it).
        Called on the publisher's thread; keep it quick."""
        with self._lock:
            self._taps.append(fn)

    def log(self, source: str, text: str) -> None:
        """A line for the shared log panel (and the recorder)."""
        self.publish("log", {"src": source, "text": text}, sticky=False)

    # -- internals --
    def _subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=256)
        with self._lock:
            for msg in self._last.values():
                q.put_nowait(msg)
            self._subs.append(q)
        return q

    def _unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            if q in self._subs:
                self._subs.remove(q)


def load_plugins(bench: Bench, only: list[str] | None) -> None:
    if not os.path.isdir(PLUGINS):
        return
    for fn in sorted(os.listdir(PLUGINS)):
        if not fn.endswith(".py") or fn.startswith("_"):
            continue
        name = fn[:-3]
        if only and name not in only:
            continue
        try:
            spec = importlib.util.spec_from_file_location(f"bench_plugin_{name}", os.path.join(PLUGINS, fn))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            if hasattr(mod, "setup"):
                mod.setup(bench)
                bench.plugins[name] = mod
                print(f"bench: plugin {name} loaded", file=sys.stderr)
        except Exception:
            # One broken part must not take the whole bench down.
            print(f"bench: plugin {name} FAILED to load:\n{traceback.format_exc()}", file=sys.stderr)


def make_handler(bench: Bench):
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *_):
            pass

        def _authorised(self, query: dict) -> bool:
            """--token set: the page, the API and /events need it, once as ?token= (then a cookie)."""
            token = getattr(bench.args, "token", "")
            if not token:
                return True
            if hmac.compare_digest(query.get("token", ""), token):
                self._set_cookie = f"bench_token={token}; Path=/; HttpOnly; SameSite=Strict"
                return True
            for part in (self.headers.get("Cookie") or "").split(";"):
                k, _, v = part.strip().partition("=")
                if k == "bench_token" and hmac.compare_digest(v, token):
                    return True
            return False

        def _send(self, status: int, payload, ctype: str | None = None) -> None:
            if isinstance(payload, tuple):
                payload, ctype = payload
            if isinstance(payload, (dict, list)):
                body, ctype = json.dumps(payload).encode(), "application/json"
            elif isinstance(payload, str):
                body, ctype = payload.encode(), ctype or "text/plain; charset=utf-8"
            else:
                body, ctype = payload or b"", ctype or "application/octet-stream"
            self.send_response(status)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            if getattr(self, "_set_cookie", ""):
                self.send_header("Set-Cookie", self._set_cookie)
            self.end_headers()
            self.wfile.write(body)

        def _dispatch(self, method: str) -> None:
            u = urllib.parse.urlparse(self.path)
            path = u.path
            query = {k: v[-1] for k, v in urllib.parse.parse_qs(u.query).items()}
            self._set_cookie = ""
            if not self._authorised(query):
                return self._send(401, "TARS bench: add ?token=<the bench token> to the address "
                                       "(bench.py --token / BENCH_TOKEN on the robot)")
            if method == "GET" and path == "/events":
                return self._events()
            if method == "GET" and path == "/panels.json":
                pdir = os.path.join(WEB, "panels")
                names = sorted(f for f in os.listdir(pdir) if f.endswith(".js")) if os.path.isdir(pdir) else []
                hidden = []
                if bench.target["name"] != "sim" and not getattr(bench.args, "all_panels", False):
                    live = "live" in bench.plugins
                    drop = {n for n in SIM_ONLY_PANELS if not (live and n in LIVE_PANELS)}
                    hidden = [n for n in names if n in drop]
                    names = [n for n in names if n not in drop]
                return self._send(200, {"panels": names, "hidden": hidden, "plugins": sorted(bench.plugins),
                                        "target": bench.target, "real": bench.target["name"] != "sim",
                                        "live": "live" in bench.plugins})
            fn = bench._routes.get((method, path))
            if fn:
                n = int(self.headers.get("Content-Length") or 0)
                body = self.rfile.read(n) if n else b""
                try:
                    status, payload = fn(query, body)
                except Exception as e:
                    traceback.print_exc()
                    status, payload = 500, {"error": f"{type(e).__name__}: {e}"}
                return self._send(status, payload)
            if method == "GET":
                return self._static(path)
            self._send(404, {"error": "no such route"})

        def _static(self, path: str) -> None:
            rel = "index.html" if path in ("", "/") else path.lstrip("/")
            full = os.path.realpath(os.path.join(WEB, rel))
            if not full.startswith(os.path.realpath(WEB) + os.sep) or not os.path.isfile(full):
                return self._send(404, "not found")
            with open(full, "rb") as f:
                data = f.read()
            self._send(200, data, CTYPES.get(os.path.splitext(full)[1], "application/octet-stream"))

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            q = bench._subscribe()
            try:
                while True:
                    try:
                        msg = q.get(timeout=15)
                    except queue.Empty:
                        msg = b": keepalive\n\n"
                    self.wfile.write(msg)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                bench._unsubscribe(q)
                self.close_connection = True

        def do_GET(self):
            self._dispatch("GET")

        def do_POST(self):
            self._dispatch("POST")

    return H


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="TARS digital test bench (laptop, or --target robot on the Pi)")
    ap.add_argument("--port", type=int, default=8770, help="browser port (default 8770)")
    ap.add_argument("--bind", default="127.0.0.1", help="listen address (default localhost only; "
                    "anything else needs --token)")
    ap.add_argument("--target", choices=["sim", "usb", "pi", "robot"], default="sim",
                    help="robot = this bench runs ON the Pi, next to the real stack")
    ap.add_argument("--token", default=os.environ.get("BENCH_TOKEN", ""),
                    help="required to open the page when --bind is not localhost (env BENCH_TOKEN)")
    ap.add_argument("--motor", default="127.0.0.1:5555", help="tars_motor host:port (sim/usb)")
    ap.add_argument("--goto", default="127.0.0.1:5570", help="goto host:port (sim/usb)")
    ap.add_argument("--voice", default="127.0.0.1:5574", help="tars_voice laptop port (sim/usb)")
    ap.add_argument("--relay", default="", help="pi target: <pi-ip>[:5575] of mission_relay "
                    "(default $TARS_HOST or tars.local)")
    ap.add_argument("--key", default=os.environ.get("MISSION_KEY", ""), help="pi target: relay key (env MISSION_KEY)")
    ap.add_argument("--plugins", default="", help="comma list to load only some plugins")
    # mission control's flags (iter2-autonomy/mission/laptop/mission.py)
    ap.add_argument("--speed", type=float, default=0.5, help="WASD start speed (fraction of max)")
    ap.add_argument("--laptop-stt", action="store_true",
                    help="mission panel push-to-talk: whisper on the laptop, send only HEAR <text>")
    ap.add_argument("--stt-cli", default=os.path.join(REPO, "iter2-autonomy", "voice", "build", "stt_cli"))
    ap.add_argument("--model", default=os.path.join(REPO, "iter2-autonomy", "voice", "models", "ggml-tiny.en-q5_1.bin"))
    ap.add_argument("--all-panels", action="store_true", help="show sim-only panels on usb/pi targets too")
    a = ap.parse_args(argv)
    if a.target == "pi":
        a.relay = a.relay or os.environ.get("TARS_HOST", "tars.local")
        if not a.key:
            ap.error("--target pi needs the relay key: --key <key> or MISSION_KEY=<key> "
                     "(the same one mission_relay was started with)")
        if a.laptop_stt:
            # the laptop only sends and receives: speech-to-text runs on the Pi (tars_voice AUDIO)
            ap.error("--laptop-stt is not allowed with --target pi: the Pi transcribes the raw AUDIO itself")
    if a.bind not in ("127.0.0.1", "localhost", "::1") and len(a.token) < 12:
        # the page can drive the robot: never open it to the network without a key
        ap.error(f"--bind {a.bind} needs --token <at least 12 characters> (or BENCH_TOKEN)")
    if a.target == "robot" and a.laptop_stt:
        ap.error("--laptop-stt makes no sense with --target robot: tars_voice transcribes on the Pi")
    if a.laptop_stt and not (os.path.exists(a.stt_cli) and os.path.exists(a.model)):
        ap.error(f"--laptop-stt needs {a.stt_cli} (make -C iter2-autonomy/voice host) and {a.model}")
    return a


def main(argv=None) -> int:
    args = parse_args(argv)
    bench = Bench(args)
    load_plugins(bench, [p for p in args.plugins.split(",") if p] or None)
    srv = ThreadingHTTPServer((args.bind, args.port), make_handler(bench))
    srv.daemon_threads = True
    print(f"bench: http://{args.bind}:{args.port}/{'?token=<token>' if args.token else ''}  target={args.target}",
          file=sys.stderr)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    for mod in bench.plugins.values():
        stop = getattr(mod, "shutdown", None)
        if stop:
            try:
                stop()
            except Exception:
                traceback.print_exc()
    return 0


if __name__ == "__main__":
    sys.exit(main())
