"""The phone's scene wire format, as a server, plus recording and replay.

Format (Andrew's MotionUSB SceneCapture -> dashboard/mapping.py, port 5561): the phone
LISTENS; the receiver connects. Each scene is a 4-byte big-endian length + UTF-8 JSON
(version 1, ≤ 512 KiB); the receiver ACKs each with byte 0x01 and the phone keeps only one
scene outstanding. `SceneServer` plays the phone's side so mapping.SceneFeed can read a
recording or the sim exactly as it reads the real phone.

Recording file (.jsonl): line 1 = {"bench_phone_recording": 1, "source": ..., "created": ...},
then one {"t": seconds since first scene, "scene": {...}} per scene, as received.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
import time


def encode(scene: dict) -> bytes:
    body = json.dumps(scene, separators=(",", ":"), allow_nan=False).encode()
    return struct.pack("!I", len(body)) + body


class SceneServer:
    """Serve scenes from `source()` (an iterator of (delay_s, scene)) to one receiver at a
    time on 127.0.0.1:<port>, waiting for each ACK like the phone does."""

    def __init__(self, source_factory, port: int = 0, name: str = "scene"):
        self.factory, self.name = source_factory, name
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(1)
        self.port = self.srv.getsockname()[1]
        self.sent = 0
        self.state = "waiting for receiver"
        self.finished = threading.Event()  # source exhausted (replay done)
        self.closed = threading.Event()
        threading.Thread(target=self._run, daemon=True, name=f"{name}-server").start()

    def _run(self):
        while not self.closed.is_set():
            try:
                conn, _ = self.srv.accept()
            except OSError:
                return
            self.state = "streaming"
            try:
                with conn:
                    conn.settimeout(3)
                    for delay, scene in self.factory():
                        if self.closed.wait(max(0.0, delay)):
                            return
                        conn.sendall(encode(scene))
                        if conn.recv(1) != b"\x01":
                            raise ConnectionError("receiver did not ACK")
                        self.sent += 1
                    # Done: stop listening, so the receiver's reconnect can't restart the
                    # replay (its 1 s read timeout drops an idle link).
                    self.state = f"finished ({self.sent} scenes)"
                    self.finished.set()
                    self.srv.close()
                    return
            except OSError as e:
                self.state = f"receiver dropped: {e}"

    def close(self):
        self.closed.set()
        try:
            self.srv.close()
        except OSError:
            pass


class Recorder:
    def __init__(self, path: str, source: str):
        self.path = path
        self.f = open(path, "w")
        self.f.write(json.dumps({"bench_phone_recording": 1, "source": source,
                                 "created": time.strftime("%Y-%m-%dT%H:%M:%S")}) + "\n")
        self.t0 = None
        self.count = 0
        self.lock = threading.Lock()

    def add(self, scene: dict):
        with self.lock:
            if self.f is None:
                return
            now = time.monotonic()
            self.t0 = now if self.t0 is None else self.t0
            self.f.write(json.dumps({"t": round(now - self.t0, 4), "scene": scene},
                                    separators=(",", ":")) + "\n")
            self.count += 1

    def close(self):
        with self.lock:
            if self.f is not None:
                self.f.close()
                self.f = None


def read_recording(path: str) -> tuple[dict, list[tuple[float, dict]]]:
    with open(path) as f:
        header = json.loads(f.readline())
        if header.get("bench_phone_recording") != 1:
            raise ValueError(f"{path}: not a bench phone recording")
        rows = [json.loads(line) for line in f if line.strip()]
    return header, [(float(r["t"]), r["scene"]) for r in rows]


def replay_factory(path: str, speed: float = 1.0, loop: bool = False):
    """Scenes with original spacing (speed 2 = twice as fast, 0 = as fast as ACKs allow).
    Each loop gets a new session id so the mapper starts clean instead of rejecting the
    repeated timestamps."""
    _, rows = read_recording(path)

    def gen():
        n = 0
        while True:
            prev = None
            for t, scene in rows:
                delay = 0.0 if prev is None or speed <= 0 else (t - prev) / speed
                prev = t
                if n:
                    scene = dict(scene, session=f"{scene['session']}-loop{n}")
                yield delay, scene
            n += 1
            if not loop or not rows:
                return
    return gen
