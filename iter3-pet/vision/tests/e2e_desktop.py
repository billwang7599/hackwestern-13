"""End-to-end on the Mac, no webcam needed:

  real hand photos (MediaPipe's public test images) -> scripted video ->
  desktop/gesture_drive.py --video (real MediaPipe) -> TCP -> pi tars_motor_host -b stub

Checks the full chain: forward step, peace -> FOLLOW, follow D stream,
palm STOP with a latency line, hand-lost STOP, and the server's STATUS showing
the VIS report. Needs iter3-pet/vision/.venv (mediapipe, opencv) and `make -C iter1-wasd/pi host`.

    iter3-pet/vision/.venv/bin/python iter3-pet/vision/tests/e2e_desktop.py
"""
from __future__ import annotations

import os
import re
import socket
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
VISION = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(VISION))
ASSETS = os.path.join(HERE, "assets")
BUILD = os.path.join(HERE, "build")
SERVER = os.path.join(REPO, "iter1-wasd", "pi", "build", "tars_motor_host")
URL = "https://storage.googleapis.com/mediapipe-assets/"
IMAGES = ["thumb_up.jpg", "victory.jpg", "fist.jpg", "left_hands.jpg"]

import cv2  # noqa: E402
import numpy as np  # noqa: E402


def fetch() -> None:
    os.makedirs(ASSETS, exist_ok=True)
    for f in IMAGES:
        p = os.path.join(ASSETS, f)
        if not os.path.exists(p):
            print(f"downloading {f}")
            urllib.request.urlretrieve(URL + f, p)


def canvas(img, cx=0.5, scale=1.0):
    """Place img on a 640x480 white canvas centred at x = cx * width."""
    out = np.full((480, 640, 3), 255, np.uint8)
    h = int(300 * scale)
    w = int(img.shape[1] * h / img.shape[0])
    im = cv2.resize(img, (w, h))
    x0 = int(cx * 640 - w / 2)
    y0 = (480 - h) // 2
    xs, xe = max(0, x0), min(640, x0 + w)
    out[y0:y0 + h, xs:xe] = im[:, xs - x0:xe - x0]
    return out


def make_video(path: str) -> list[str]:
    ims = {f: cv2.imread(os.path.join(ASSETS, f)) for f in IMAGES}
    palm = ims["left_hands.jpg"][:, :360]                 # the upright open palm
    blank = np.full((480, 640, 3), 255, np.uint8)
    seq = []
    seq += [("blank", blank)] * 10
    seq += [("thumbs_up", canvas(ims["thumb_up.jpg"]))] * 8          # -> forward step
    seq += [("blank", blank)] * 30                                     # -> step ends / lost
    seq += [("peace", canvas(ims["victory.jpg"]))] * 10               # -> FOLLOW
    # small fist on the left of the raw camera image = the rover's LEFT
    # (the camera looks where the rover looks), far away -> forward + turn left
    seq += [("fist", canvas(ims["fist.jpg"], cx=0.25, scale=0.5))] * 30
    seq += [("palm", canvas(palm))] * 6                               # -> STOP (palm)
    seq += [("fist", canvas(ims["fist.jpg"], cx=0.5, scale=0.5))] * 15
    seq += [("blank", blank)] * 15                                     # -> lost STOP
    vw = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"MJPG"), 30, (640, 480))
    for _, fr in seq:
        vw.write(fr)
    vw.release()
    return [n for n, _ in seq]


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def status(port: int) -> dict:
    with socket.create_connection(("127.0.0.1", port)) as s:
        s.sendall(b"STATUS\n")
        r = s.makefile().readline().split()
    return dict(kv.split("=", 1) for kv in r[6:])


def main() -> int:
    if not os.path.exists(SERVER):
        subprocess.run(["make", "-C", os.path.join(REPO, "iter1-wasd", "pi"), "host"], check=True)
    fetch()
    os.makedirs(BUILD, exist_ok=True)
    video = os.path.join(BUILD, "e2e.avi")
    make_video(video)

    port = free_port()
    srv = subprocess.Popen([SERVER, "-b", "stub", "-p", str(port)], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True)
    time.sleep(0.4)
    fails = 0
    try:
        app = subprocess.run(
            [sys.executable, os.path.join(VISION, "desktop", "gesture_drive.py"),
             "--video", video, "--headless", "--realtime", "--learned", "", "--host", "127.0.0.1", "--port", str(port)],
            capture_output=True, text=True, timeout=120)
        out = app.stdout
        st = status(port)
    finally:
        srv.terminate()
        log = srv.communicate(timeout=5)[0]

    cmd_lines = [ln for ln in out.splitlines() if ln.startswith(("[cmd]", "[lat]", "[see]", "[status]", "[end]"))]
    print("\n".join(cmd_lines))

    def check(cond, what):
        nonlocal fails
        print(("  ok   " if cond else "  FAIL ") + what)
        fails += 0 if cond else 1

    print("\nchecks:")
    check(app.returncode == 0, f"app exit 0 (got {app.returncode})")
    check(re.search(r"\[cmd\] D 0\.50 0\.50\s+forward", out) is not None, "thumbs up -> forward step")
    check(re.search(r"\[cmd\] S\s+(done|lost)", out) is not None, "step ended with S")
    check(re.search(r"\[cmd\] S\s+mode_follow", out) is not None, "peace -> FOLLOW mode")
    check("mode=follow" in out, "status line reports mode=follow")
    m = re.search(r"\[lat\] PALM -> STOP: camera frame -> S acked in (\d+) ms", out)
    check(m is not None, "palm -> STOP with latency line")
    check(re.search(r"\[cmd\] S\s+lost", out.split("PALM -> STOP")[-1]) is not None,
          "hand lost after palm/follow -> S lost")
    # server side: follow stream arrived and turned left (right wheel faster)
    drives = [(float(a), float(b)) for a, b in re.findall(r"drive L=([+-][\d.]+) R=([+-][\d.]+)", log)]
    check(any(r > l + 0.05 and r > 0 for l, r in drives), "server saw follow: forward + turning left toward the hand")
    check(st.get("vis") in ("follow", "gesture") and st.get("vis_see") is not None,
          f"server STATUS has VIS: vis={st.get('vis')} fps={st.get('vis_fps')} "
          f"cam_ms={st.get('vis_cam_ms')} stop_ms={st.get('vis_stop_ms')} server_stop_ms={st.get('stop_ms')}")
    check(int(st.get("stop_ms", "-1")) >= 0, "server measured S -> wheels 0")
    if m:
        print(f"\nlatency (Mac, real MediaPipe, localhost): camera frame -> S acked = {m.group(1)} ms; "
              f"server S -> wheels 0 = {st.get('stop_ms')} ms")
    print(f"\n{'E2E OK' if not fails else f'E2E FAILED ({fails})'}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
