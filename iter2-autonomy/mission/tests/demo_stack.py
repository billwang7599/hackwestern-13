#!/usr/bin/env python3
"""Try mission control on a Mac with no robot: a fake Pi on localhost.

    make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host && make -C iter2-autonomy/mission host
    python3 iter2-autonomy/mission/tests/demo_stack.py          # then open http://localhost:8765

Runs the REAL tars_motor (-b stub), goto and (if built) tars_voice host builds, mission_relay,
and fakes for the phone: one LDR1 room scan into goto's nav feed, YOLO detections of a chair,
camera frames (edge-ai/models/dog.jpg), a phone status tee. Then mission.py. Ctrl-C stops all.
"""
from __future__ import annotations

import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_relay as tr  # noqa: E402

REPO = tr.REPO
GOTO_BIN = os.path.join(REPO, "iter2-autonomy", "goto", "build", "goto_host")
VOICE_BIN = os.path.join(REPO, "iter2-autonomy", "voice", "build", "tars_voice_host")
MODEL = os.path.join(REPO, "iter2-autonomy", "voice", "models", "ggml-tiny.en-q5_1.bin")
JPEG = os.path.join(REPO, "iter2-autonomy", "edge-ai", "models", "dog.jpg")


def ldr1_room() -> bytes:
    """a 4 x 3 m room around the phone: floor everywhere inside, walls at the edges, a box"""
    pts = []
    for i in range(-18, 19):
        for j in range(-12, 26):
            pts.append((i * 0.1, -0.3, j * 0.1))
    pts = pts[::2][:700]
    for i in range(-20, 21):
        pts += [(i * 0.1, 0.2, 2.6), (i * 0.1, 0.2, -1.4)]
    for j in range(-14, 27):
        pts += [(-2.0, 0.2, j * 0.1), (2.0, 0.2, j * 0.1)]
    pts += [(0.6 + dx * 0.1, 0.1, 1.2 + dz * 0.1) for dx in range(3) for dz in range(3)]
    pts = pts[:1024]
    body = b"LDR1" + struct.pack("!IQII", 1, int(time.monotonic() * 1e6), 7, len(pts)) + struct.pack("!5f", 0, 0, 0, 0.3, -0.3)
    return body + b"".join(struct.pack("!3f", *p) for p in pts)


class JpegBroker(tr.FakeBroker):
    def __init__(self):
        with open(JPEG, "rb") as f:
            self.jpeg = f.read()
        super().__init__()

    def _run(self):
        while True:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            try:
                c.settimeout(1)
                if c.recv(4) == b"GET\n":
                    self.frame_id += 1
                    pkt = b"RGB1" + struct.pack("!Q", self.frame_id) + self.jpeg
                    c.sendall(struct.pack("!I", len(pkt)) + pkt)
            except OSError:
                pass
            c.close()


def main() -> int:
    mport, gport, dport, nport, vport = (tr.free_port() for _ in range(5))
    procs = []
    os.makedirs(tr.SCRATCH, exist_ok=True)
    procs.append(subprocess.Popen([tr.MOTOR_BIN, "-b", "stub", "-p", str(mport), "-r", "0"], stderr=subprocess.DEVNULL))
    assert tr.wait_for(lambda: tr.port_open(mport), 3)
    procs.append(subprocess.Popen([GOTO_BIN, "-m", f"127.0.0.1:{mport}", "-p", str(gport), "-u", str(dport), "-n", str(nport),
                                   "-r", "0", "-M", os.path.join(tr.SCRATCH, "demo_memory.txt"),
                                   "-o", os.path.join(REPO, "iter2-autonomy", "goto", "objects.txt")], stderr=subprocess.DEVNULL))
    assert tr.wait_for(lambda: tr.port_open(gport), 3)
    if os.path.exists(VOICE_BIN) and os.path.exists(MODEL):
        procs.append(subprocess.Popen([VOICE_BIN, "--input", "raw:-", "--model", MODEL, "--port", str(mport),
                                       "--goto-port", str(gport), "--net-port", str(vport), "--say-port", "0"],
                                      stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    else:
        print("tars_voice host build not found: voice panel will say voice is down")
    broker = JpegBroker()
    rl = tr.Relay(os.path.join(tr.MISSION, "build", "mission_relay_host"), mport, gport, vport, broker.port)
    nav = socket.create_connection(("127.0.0.1", nport))
    room = ldr1_room()
    nav.sendall(struct.pack("!I", len(room)) + room)
    stop = threading.Event()

    def fake_phone():
        det = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tee = None
        k = 0
        while not stop.wait(0.2):
            k += 1
            x = 200 + 60 * math.sin(k / 10)
            det.sendto(json.dumps({"w": 640, "h": 480, "dets": [{"label": "chair", "conf": 0.82, "box": [x, 180, x + 140, 400]}]}).encode(),
                       ("127.0.0.1", dport))
            if k % 5 == 0:
                try:
                    tee = tee or socket.create_connection(("127.0.0.1", rl.tee))
                    tee.sendall((json.dumps({
                        "type": "status", "mode": "headless", "streaming": True, "motion_hz": 99.8, "camera_fps": 10.1,
                        "safety": {"verdict": "CLEAR", "permit": True, "live": True, "stop_source": "lidar", "nearest_obstacle_m": 1.4},
                        "ai": {"live": True, "inference_ms": 148, "objects": [{"label": "dog", "confidence": 0.91, "bbox": [0.16, 0.38, 0.27, 0.55]}]},
                        "depth": {"live": False, "error": "MiDaS off"},
                        "surroundings": {"tracking": "normal", "floor_ready": True, "voxels": 4665, "frames": k, "scan_degrees": 180,
                                         "message": "demo", "pose": [0, 0, 0, 0.3], "objects": [{"label": "chair", "position": [1.0, 0, 2.0]}]},
                    }) + "\n").encode())
                except OSError:
                    tee = None
    threading.Thread(target=fake_phone, daemon=True).start()
    app = subprocess.Popen([sys.executable, os.path.join(tr.MISSION, "laptop", "mission.py"), "--relay", f"127.0.0.1:{rl.port}"])
    try:
        app.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        app.terminate()
        rl.close()
        for p in procs:
            p.terminate()
        broker.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
