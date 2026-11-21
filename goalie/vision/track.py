"""Beat the Goalie: laptop main app. Webcam -> ball + paddle (metres) -> Pi.

    python track.py                         # camera from calib.json, Pi at goalie.local:5556
    python track.py --host 192.168.1.50
    python track.py --fake --host 127.0.0.1 # synthetic ball, no camera (e.g. with mock_pi.py)
    python track.py --no-pi                 # no server: prints what would be sent
    python track.py --fake --headless --frames 300 --out out/smoke.png   # smoke test

Keys (in the window):
    m  toggle MODE QNX / LINUX (emulated jitter)     h  human goalie on/off
    a/d  move the human target (if no Switch controller)
    r  RESET scores      t  taunts on/off      q / Esc  quit
The Switch controller's A/B button also toggles human mode.

Every camera frame sends `F <t_ms> <ball_seen> <bx> <by> <paddle_seen> <px>`;
the reply's target_x is drawn as the predicted crossing. A goal (ball seen
past the goal line inside the goal) sends GOAL, debounced. STATUS is polled
at ~5 Hz for the scoreboard, mode banner and watchdog light.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

from cvutil import Calib, hsv_mask, largest_blob, px_to_field
from field import GOAL_HALF_W, in_goal
from goalie_link import DEFAULT_HOST, DEFAULT_PORT, DryLink, GoalieError, GoalieLink, LinkDown
from ui import UIState, render

STATUS_PERIOD = 0.2
GOAL_REARM_Y = 0.15         # ball must come back out past this before the next goal counts
GOAL_DEBOUNCE_S = 1.5
WIN = "Beat the Goalie"


class Detector:
    """HSV blobs -> pixel centres -> field metres via the calibration homography."""

    def __init__(self, calib: Calib):
        self.c = calib
        self.H = calib.homography()

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dets = {"polys": [self.c.corners_px]}
        out = []
        for name, rng, min_a in (("ball", self.c.ball, self.c.ball_min_area),
                                 ("paddle", self.c.paddle, self.c.paddle_min_area)):
            blob = largest_blob(hsv_mask(hsv, rng), min_a)
            if blob is None:
                out.append(None)
                continue
            dets[name] = blob[:2]
            out.append(px_to_field(self.H, blob[0], blob[1]))
        ball, paddle = out
        return ball, (paddle[0] if paddle else None), dets


class GoalDetector:
    """Fires once when the ball is seen past the goal line (y < 0) inside the goal."""

    def __init__(self):
        self.armed = True
        self.last = -1e9

    def update(self, ball, now: float) -> bool:
        if ball is None:
            return False
        x, y = ball
        if y > GOAL_REARM_Y:
            self.armed = True
        if self.armed and y < 0 and in_goal(x) and now - self.last > GOAL_DEBOUNCE_S:
            self.armed, self.last = False, now
            return True
        return False


class CameraSource:
    def __init__(self, calib: Calib, index: int | None):
        self.cap = cv2.VideoCapture(calib.camera if index is None else index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)          # we want the newest frame, not a queue
        if not self.cap.isOpened():
            raise RuntimeError("camera not available (macOS: allow camera access for your terminal)")
        self.det = Detector(calib)

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            return None, None, None, None
        ball, paddle, dets = self.det.detect(frame)
        return ball, paddle, frame, dets

    def command(self, target, duty):
        pass


class FakeSource:
    def __init__(self, args):
        from fake_ball import FakeCamera, config_from_args
        self.cam = FakeCamera(config_from_args(args))
        self.period = 1.0 / self.cam.fps
        self.next = time.monotonic()

    def read(self):
        self.next += self.period
        d = self.next - time.monotonic()
        if d > 0:
            time.sleep(d)
        else:
            self.next = time.monotonic()
        ball, paddle = self.cam.read()
        return ball, paddle, None, None

    def command(self, target, duty):
        self.cam.command(target, duty)


def main(argv=None) -> int:
    from fake_ball import add_fake_args
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--calib", default=None, help="calib.json (default: calib.json, else calib.example.json)")
    ap.add_argument("--camera", type=int, default=None, help="override the camera index")
    ap.add_argument("--fake", action="store_true", help="synthetic ball instead of the camera")
    ap.add_argument("--no-pi", action="store_true", help="no server: print what would be sent")
    ap.add_argument("--no-taunt", action="store_true")
    ap.add_argument("--no-switch", action="store_true", help="don't look for a Switch controller")
    ap.add_argument("--headless", action="store_true", help="no window (writes --out at the end)")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run until q)")
    ap.add_argument("--out", default="out/track.png", help="overlay PNG written in headless mode")
    ap.add_argument("--mode", choices=["QNX", "LINUX"], default=None, help="send MODE at start")
    add_fake_args(ap)
    a = ap.parse_args(argv)

    calib = Calib.load(a.calib)
    try:
        src = FakeSource(a) if a.fake else CameraSource(calib, a.camera)
    except RuntimeError as e:
        print(f"!! {e}", file=sys.stderr)
        return 1
    link = DryLink() if a.no_pi else GoalieLink(a.host, a.port)

    taunter = None
    if not a.no_taunt:
        from taunt import Taunter
        taunter = Taunter()
    sw = None
    if not a.no_switch and not a.headless:
        from switch_input import SwitchInput
        sw = SwitchInput()

    ui = UIState(taunt_on=taunter is not None, field_w=calib.field_w)
    goals = GoalDetector()
    counts = {"frames": 0, "link_down": 0, "goals_sent": 0, "status_polls": 0}

    def safe(fn, *args):
        try:
            r = fn(*args)
            ui.link_up = True
            return r
        except LinkDown:
            ui.link_up = False
            counts["link_down"] += 1
        except GoalieError as e:
            print(f"[pi] {e}")
        return None

    if a.mode:
        safe(link.mode, a.mode)
    last_status = 0.0
    fps_t = time.monotonic()
    img = None
    try:
        while True:
            ball, paddle, frame, dets = src.read()
            now = time.monotonic()
            if ball is None and paddle is None and frame is None and not a.fake:
                continue                               # camera hiccup
            counts["frames"] += 1
            dt = now - fps_t
            fps_t = now
            if dt > 0:
                ui.fps = 0.9 * ui.fps + 0.1 / dt if ui.fps else 1 / dt

            # controller / human mode
            if sw is not None:
                sw.poll()
                ui.controller = sw.name if sw.available else ""
                if sw.toggled:
                    ui.human = not ui.human
                    if not ui.human:
                        safe(link.auto)
                if sw.available:
                    ui.human_x = sw.x
            if ui.human:
                safe(link.human, ui.human_x)

            # the frame itself
            ui.push_ball(ball)
            ui.paddle = paddle
            r = safe(link.frame, int(now * 1000), ball, paddle)
            if r is not None:
                ui.target_x, ui.duty = r
                src.command(*r)
            else:
                src.command(0.0, 0.0)

            if goals.update(ball, now):
                counts["goals_sent"] += 1
                safe(link.goal)
                ui.set_flash("GOAL!")

            # status at ~5 Hz
            if now - last_status >= STATUS_PERIOD:
                last_status = now
                st = safe(link.status)
                if st is not None:
                    counts["status_polls"] += 1
                    old = ui.status
                    if isinstance(old.get("saves"), int) and st.get("saves", 0) > old["saves"]:
                        ui.set_flash("SAVE!")
                    ui.status = st
                    if taunter:
                        taunter.update(st.get("saves"), st.get("goals"))
                        ui.taunt_text, ui.taunt_t = taunter.last_text, taunter.last_t

            img = render(ui, frame, dets)
            if a.headless:
                if a.frames and counts["frames"] >= a.frames:
                    break
                continue
            cv2.imshow(WIN, img)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27) or (a.frames and counts["frames"] >= a.frames):
                break
            if k == ord("m"):
                new = "LINUX" if ui.status.get("mode", "QNX") == "QNX" else "QNX"
                if safe(link.mode, new) is not None:
                    ui.status["mode"] = new
            elif k == ord("h"):
                ui.human = not ui.human
                if not ui.human:
                    safe(link.auto)
            elif k == ord("a"):
                ui.human_x = max(-GOAL_HALF_W, ui.human_x - 0.02)
            elif k == ord("d"):
                ui.human_x = min(GOAL_HALF_W, ui.human_x + 0.02)
            elif k == ord("r"):
                safe(link.reset)
                goals.armed = True
            elif k == ord("t") and taunter:
                taunter.enabled = ui.taunt_on = not taunter.enabled
    except KeyboardInterrupt:
        pass
    finally:
        if img is not None and a.headless:
            out = Path(a.out)
            out.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out), img)
            print(f"wrote {out}")
        if not a.headless:
            cv2.destroyAllWindows()
        link.close()
    st = ui.status
    print(f"frames={counts['frames']} link_down={counts['link_down']} goals_sent={counts['goals_sent']} "
          f"status_polls={counts['status_polls']} mode={st.get('mode')} saves={st.get('saves')} "
          f"goals={st.get('goals')} jit_max_us={st.get('jit_max_us')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
