"""Calibrate the overhead camera: field corners -> homography, plus HSV colour
ranges for the ball and the paddle. Writes calib.json (gitignored).

    python calibrate.py                     # camera 0
    python calibrate.py --camera 1 --field-w 0.40
    python calibrate.py --image snap.png    # from a still

1. Click the 4 field corners in this order (the goal is at the BOTTOM of the image):
       1 far-left   2 far-right   3 goal-right   4 goal-left
   (u = undo last corner). Corners 3/4 must lie ON THE GOAL LINE: that is y=0.
2. Press b, click the ball. Press p, click the paddle blade (put the coloured
   tape on the blade tip only: its centre is what the Pi controls).
   Tweak the six sliders until the mask window shows just that object.
3. s = save calib.json, q = quit.  Space = freeze/unfreeze the live image.
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

from cvutil import DEFAULT_CALIB, Calib, HSVRange, field_to_px, hsv_mask, largest_blob, range_from_patch
from field import GOAL_HALF_W

WIN, TUNE = "calibrate", "tune (ball/paddle HSV)"
LABELS = ["1 far-left", "2 far-right", "3 goal-right", "4 goal-left"]
SLIDERS = [("H lo", "h_lo", 179), ("H hi", "h_hi", 179), ("S lo", "s_lo", 255),
           ("S hi", "s_hi", 255), ("V lo", "v_lo", 255), ("V hi", "v_hi", 255)]


class App:
    def __init__(self, calib: Calib, fresh_corners: bool):
        self.c = calib
        self.corners = [] if fresh_corners else [list(p) for p in calib.corners_px]
        self.pick = None          # None | "ball" | "paddle"
        self.edit = "ball"        # which range the sliders edit
        self.hsv = None

    def on_mouse(self, ev, u, v, *_):
        if ev != cv2.EVENT_LBUTTONDOWN:
            return
        if self.pick and self.hsv is not None:
            setattr(self.c, self.pick, range_from_patch(self.hsv, u, v))
            print(f"{self.pick}: {getattr(self.c, self.pick)}")
            self.edit, self.pick = self.pick, None
            self.push_sliders()
        elif len(self.corners) < 4:
            self.corners.append([u, v])
            print(f"corner {LABELS[len(self.corners) - 1]}: ({u}, {v})")

    def push_sliders(self):
        r = getattr(self.c, self.edit)
        for label, attr, _ in SLIDERS:
            cv2.setTrackbarPos(label, TUNE, getattr(r, attr))

    def pull_sliders(self):
        r = getattr(self.c, self.edit)
        for label, attr, _ in SLIDERS:
            setattr(r, attr, cv2.getTrackbarPos(label, TUNE))

    def draw(self, img):
        out = img.copy()
        for i, p in enumerate(self.corners):
            cv2.circle(out, tuple(p), 5, (0, 255, 255), -1)
            cv2.putText(out, LABELS[i], (p[0] + 6, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        if len(self.corners) == 4:
            self.c.corners_px = self.corners
            H = self.c.homography()
            cv2.polylines(out, [np.array(self.corners, np.int32)], True, (0, 255, 255), 1)
            a, b = field_to_px(H, -GOAL_HALF_W, 0), field_to_px(H, GOAL_HALF_W, 0)
            cv2.line(out, a, b, (0, 0, 255), 3)                  # the goal mouth
            for name, r, col in (("ball", self.c.ball, (0, 140, 255)), ("paddle", self.c.paddle, (0, 255, 0))):
                blob = largest_blob(hsv_mask(self.hsv, r), 20)
                if blob:
                    cv2.circle(out, (int(blob[0]), int(blob[1])), 10, col, 2)
                    cv2.putText(out, name, (int(blob[0]) + 12, int(blob[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        msg = (f"click {LABELS[len(self.corners)]}" if len(self.corners) < 4 else
               f"click the {self.pick}" if self.pick else
               f"b=pick ball  p=pick paddle  s=save  q=quit  (sliders edit: {self.edit})")
        cv2.putText(out, msg, (8, out.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--image", help="calibrate from a still image instead of the camera")
    ap.add_argument("--field-w", type=float, default=None, help="measured wall-to-wall width, m")
    ap.add_argument("--out", default=str(DEFAULT_CALIB))
    ap.add_argument("--keep-corners", action="store_true", help="start from the saved corners")
    a = ap.parse_args()

    calib = Calib.load() if DEFAULT_CALIB.exists() else Calib()
    if a.field_w:
        calib.field_w = a.field_w
    if a.camera is not None:
        calib.camera = a.camera
    cap, still = None, None
    if a.image:
        still = cv2.imread(a.image)
        if still is None:
            print(f"!! can't read {a.image}", file=sys.stderr)
            return 1
    else:
        cap = cv2.VideoCapture(calib.camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, calib.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, calib.height)
        if not cap.isOpened():
            print(f"!! camera {calib.camera} not available (macOS: allow camera access for your terminal)",
                  file=sys.stderr)
            return 1

    app = App(calib, fresh_corners=not a.keep_corners)
    cv2.namedWindow(WIN)
    cv2.namedWindow(TUNE)
    cv2.setMouseCallback(WIN, app.on_mouse)
    for label, _, mx in SLIDERS:
        cv2.createTrackbar(label, TUNE, 0, mx, lambda _v: None)
    app.push_sliders()
    frozen = None
    print(__doc__)
    while True:
        if still is not None:
            img = still
        elif frozen is not None:
            img = frozen
        else:
            ok, img = cap.read()
            if not ok:
                continue
        app.hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        app.pull_sliders()
        cv2.imshow(WIN, app.draw(img))
        cv2.imshow(TUNE, hsv_mask(app.hsv, getattr(calib, app.edit)))
        k = cv2.waitKey(15) & 0xFF
        if k == ord("q") or k == 27:
            break
        if k == ord("u") and app.corners:
            app.corners.pop()
        elif k == ord("b"):
            app.pick = "ball"
        elif k == ord("p"):
            app.pick = "paddle"
        elif k == ord("e"):                     # switch which range the sliders edit
            app.edit = "paddle" if app.edit == "ball" else "ball"
            app.push_sliders()
        elif k == ord(" ") and still is None:
            frozen = None if frozen is not None else img.copy()
        elif k == ord("s"):
            if len(app.corners) != 4:
                print("!! click all 4 corners first")
                continue
            calib.corners_px = app.corners
            calib.width, calib.height = img.shape[1], img.shape[0]
            calib.save(a.out)
            print(f"saved {a.out}")
    if cap:
        cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
