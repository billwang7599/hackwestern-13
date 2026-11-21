"""Calibration tools; everything is written to camera.json (see config.py).

  python calib.py nominal --hfov 55                 # intrinsics from an FOV guess
  python calib.py intrinsics --device 0 --board 9x6 --square 0.024
        # live: move a printed chessboard around; SPACE grabs a view (or 'a'
        # toggles auto-grab), ENTER computes. Or: --images 'shots/*.png'
  python calib.py pose --height 0.35 --back 0.25 --pitch 8 [--yaw 0 --x 0]
        # simple extrinsics: camera behind the goal, tilted down
  python calib.py pose-board --device 0 --board 9x6 --square 0.024 --origin 0 1.0
        # chessboard flat on the floor, rows running across the goal (+X), first
        # inner corner at world (X=origin[0], Y=0, Z=origin[1]); solvePnP.
        # SPACE accepts once the drawn axes look right (red X right, green Z out,
        # blue up).
  python calib.py hsv --device 0                    # click the ball; sliders; s saves
  python calib.py ball tennis | --diam 0.067
  python calib.py ballcheck --device 0 --distance 1.00
        # hold the ball on the optical axis at a measured distance; prints the
        # estimated distance and a suggested size-scale correction
  python calib.py show
"""
from __future__ import annotations

import argparse
import glob
import math
import sys
import time

import cv2
import numpy as np

from config import BALLS, DEFAULT_CONFIG, Camera, Config
from detect import BallDetector
from geometry import backproject
from live import open_camera


def _cam(dev):
    cap, info = open_camera(dev, fps=30)
    if cap is None:
        sys.exit(f"camera {dev} did not open (Camera permission for the terminal?)")
    print("capture:", info)
    for _ in range(40):          # C270 auto-exposure ramps up after open
        cap.read()
    return cap


def board_points(board, square):
    cols, rows = board
    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
    return obj


def parse_board(s):
    c, r = s.lower().split("x")
    return int(c), int(r)


def find_board(gray, board):
    ok, corners = cv2.findChessboardCorners(gray, board, cv2.CALIB_CB_ADAPTIVE_THRESH
                                            + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ok:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
    return ok, corners


def cmd_nominal(a, cfg):
    pose = dict(R=cfg.camera.R, C=cfg.camera.C)
    cfg.camera = Camera.nominal(a.w, a.h, a.hfov)
    cfg.camera.R, cfg.camera.C = pose["R"], pose["C"]
    print(f"f = {cfg.camera.fx:.1f} px for HFOV {a.hfov} deg at {a.w}x{a.h}")


def cmd_intrinsics(a, cfg):
    board = parse_board(a.board)
    obj = board_points(board, a.square)
    objs, imgs, size = [], [], None
    if a.images:
        for p in sorted(glob.glob(a.images)):
            g = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
            ok, c = find_board(g, board)
            print(p, "found" if ok else "no board")
            if ok:
                objs.append(obj), imgs.append(c)
                size = g.shape[::-1]
    else:
        cap = _cam(a.device)
        auto, last = False, 0.0
        while True:
            ok, f = cap.read()
            if not ok:
                continue
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            found, c = find_board(g, board)
            vis = f.copy()
            if found:
                cv2.drawChessboardCorners(vis, board, c, found)
            cv2.putText(vis, f"views {len(imgs)}  SPACE grab  a auto({auto})  ENTER solve  q quit",
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("intrinsics", vis)
            k = cv2.waitKey(1) & 0xFF
            if found and (k == 32 or (auto and time.time() - last > 1.0)):
                objs.append(obj), imgs.append(c)
                size, last = g.shape[::-1], time.time()
            if k in (13, 10) and len(imgs) >= 8:
                break
            if k == ord("q"):
                sys.exit("aborted")
        cap.release()
        cv2.destroyAllWindows()
    if len(imgs) < 5:
        sys.exit("need >= 5 board views (8-20 recommended, tilted and in the corners)")
    rms, K, dist, _, _ = cv2.calibrateCamera(objs, imgs, size, None, None)
    c = cfg.camera
    c.w, c.h = size
    c.fx, c.fy, c.cx, c.cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    c.dist = dist.ravel()[:5].tolist()
    hfov = math.degrees(2 * math.atan(c.w / 2 / c.fx))
    print(f"rms reprojection {rms:.3f} px; fx {c.fx:.1f} fy {c.fy:.1f} -> HFOV {hfov:.1f} deg; dist {np.round(c.dist, 4)}")


def cmd_pose(a, cfg):
    cfg.camera.set_pose(a.height, a.back, a.x, a.pitch, a.yaw)
    print("camera at", np.round(cfg.camera.C, 3), "pitch", a.pitch, "yaw", a.yaw)


def pose_from_board(rvec, tvec, origin_xz):
    """Board frame (x along rows -> world +X, y -> world +Z, z -> world +Y)."""
    A = np.array([[1, 0, 0], [0, 0, 1], [0, 1, 0]], float)   # columns: images of board x,y,z
    Rb, _ = cv2.Rodrigues(rvec)
    b = np.array([origin_xz[0], 0.0, origin_xz[1]])
    R = Rb @ A.T
    C = b - A @ Rb.T @ tvec.ravel()
    return R, C


def cmd_pose_board(a, cfg):
    board = parse_board(a.board)
    obj = board_points(board, a.square)
    cam = cfg.camera
    K, dist = cam.K, np.asarray(cam.dist, float)
    cap = None if a.image else _cam(a.device)
    while True:
        f = cv2.imread(a.image) if a.image else cap.read()[1]
        if f is None:
            continue
        if (f.shape[1], f.shape[0]) != (cam.w, cam.h):
            cam = cam.scaled(f.shape[1], f.shape[0])
            K = cam.K
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        found, c = find_board(g, board)
        vis = f.copy()
        best = None
        if found:
            # the detected corner order can be rotated 180 deg: keep the
            # solution with the camera above the floor looking out (+Z)
            for cc in (c, c[::-1].copy()):
                ok, rv, tv = cv2.solvePnP(obj, cc, K, dist)
                R, C = pose_from_board(rv, tv, a.origin)
                if C[1] > 0 and R[2, 2] > 0:
                    best = (R, C, rv, tv, cc)
                    break
            if best:
                R, C, rv, tv, cc = best
                cv2.drawFrameAxes(vis, K, dist, rv, tv, 3 * a.square)
                pitch = math.degrees(math.asin(-R[2, 1]))
                cv2.putText(vis, f"cam at X{C[0]:+.2f} Y{C[1]:.2f} Z{C[2]:+.2f} pitch {pitch:.1f}deg",
                            (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if a.image:
            if not best:
                sys.exit("board not found / no valid pose in image")
            break
        cv2.imshow("pose-board (SPACE accept, q quit)", vis)
        k = cv2.waitKey(1) & 0xFF
        if k == 32 and best:
            break
        if k == ord("q"):
            sys.exit("aborted")
    R, C = best[0], best[1]
    cfg.camera.R, cfg.camera.C = R.tolist(), C.tolist()
    print("camera centre (world):", np.round(C, 3))
    if cap:
        cap.release()
        cv2.destroyAllWindows()


def cmd_hsv(a, cfg):
    cap = _cam(a.device)
    d = cfg.detect
    win = "hsv picker: click ball, s save, q quit"
    cv2.namedWindow(win)
    names = ["H lo", "S lo", "V lo", "H hi", "S hi", "V hi"]
    vals = d.hsv_lo + d.hsv_hi
    for n, v, mx in zip(names, vals, [179, 255, 255, 179, 255, 255]):
        cv2.createTrackbar(n, win, int(v), mx, lambda _: None)
    state = {"frame": None}

    def on_click(ev, x, y, *_):
        if ev == cv2.EVENT_LBUTTONDOWN and state["frame"] is not None:
            hsv = cv2.cvtColor(state["frame"], cv2.COLOR_BGR2HSV)
            patch = hsv[max(0, y - 3):y + 4, max(0, x - 3):x + 4].reshape(-1, 3)
            h, s, v = np.median(patch, axis=0)
            lo = [int(h - 10) % 180, int(max(60, s - 70)), int(max(50, v - 90))]
            hi = [int(h + 10) % 180, 255, 255]
            for n, val in zip(names, lo + hi):
                cv2.setTrackbarPos(n, win, val)
            print(f"picked HSV ({h:.0f},{s:.0f},{v:.0f}) -> lo {lo} hi {hi}")

    cv2.setMouseCallback(win, on_click)
    det = BallDetector(d)
    d.static_reject = False
    while True:
        ok, f = cap.read()
        if not ok:
            continue
        state["frame"] = f
        v = [cv2.getTrackbarPos(n, win) for n in names]
        d.hsv_lo, d.hsv_hi = v[:3], v[3:]
        r = det.detect(f)
        vis = f.copy()
        m = cv2.cvtColor(det.mask, cv2.COLOR_GRAY2BGR)
        if r:
            cv2.circle(vis, (int(r.u), int(r.v)), int(r.diam / 2), (0, 255, 0), 1)
        cv2.imshow(win, np.hstack([vis, m]))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("s"):
            print("saved hsv", d.hsv_lo, d.hsv_hi)
            d.static_reject = True
            break
        if k == ord("q"):
            sys.exit("not saved")
    cap.release()
    cv2.destroyAllWindows()


def cmd_ball(a, cfg):
    cfg.set_ball(a.name if a.name else a.diam)
    print(f"ball diameter {cfg.ball_diam*1000:.0f} mm, quadratic drag {cfg.drag_quad}")


def cmd_ballcheck(a, cfg):
    cap = _cam(a.device)
    cfg.detect.static_reject = False
    det = BallDetector(cfg.detect)
    est = []
    t_end = time.time() + a.seconds
    while time.time() < t_end:
        ok, f = cap.read()
        if not ok:
            continue
        cam = cfg.camera if (f.shape[1], f.shape[0]) == (cfg.camera.w, cfg.camera.h) \
            else cfg.camera.scaled(f.shape[1], f.shape[0])
        r = det.detect(f)
        if r and not r.clipped:
            m = backproject(cam, r.u, r.v, r.diam, cfg.ball_diam)
            est.append((m.depth, r.diam, r.diam_ellipse))
    cap.release()
    if not est:
        sys.exit("ball not detected; run `calib.py hsv` first")
    D = np.array(est)
    med = np.median(D[:, 0])
    print(f"{len(D)} frames: estimated camera->ball distance {med:.3f} m (sd {D[:, 0].std()*100:.1f} cm), "
          f"true {a.distance:.3f} m; diameter {np.median(D[:, 1]):.2f} px (ellipse {np.median(D[:, 2]):.2f})")
    scale = med / a.distance
    print(f"error {100*(med/a.distance-1):+.1f}%.  If consistent across distances, either the ball "
          f"diameter or the focal length is off by that factor: e.g. `calib.py ball --diam "
          f"{cfg.ball_diam/scale:.4f}` (effective size), or run `calib.py intrinsics`.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(DEFAULT_CONFIG))
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("nominal"); p.add_argument("--hfov", type=float, default=55); p.add_argument("--w", type=int, default=640); p.add_argument("--h", type=int, default=480)
    p = sub.add_parser("intrinsics"); p.add_argument("--device", type=int, default=0); p.add_argument("--board", default="9x6"); p.add_argument("--square", type=float, default=0.024); p.add_argument("--images")
    p = sub.add_parser("pose"); p.add_argument("--height", type=float, required=True); p.add_argument("--back", type=float, default=0.25); p.add_argument("--pitch", type=float, default=0); p.add_argument("--yaw", type=float, default=0); p.add_argument("--x", type=float, default=0)
    p = sub.add_parser("pose-board"); p.add_argument("--device", type=int, default=0); p.add_argument("--board", default="9x6"); p.add_argument("--square", type=float, default=0.024); p.add_argument("--origin", type=float, nargs=2, default=[0.0, 1.0], metavar=("X", "Z")); p.add_argument("--image")
    p = sub.add_parser("hsv"); p.add_argument("--device", type=int, default=0)
    p = sub.add_parser("ball"); p.add_argument("name", nargs="?", choices=list(BALLS)); p.add_argument("--diam", type=float)
    p = sub.add_parser("ballcheck"); p.add_argument("--device", type=int, default=0); p.add_argument("--distance", type=float, required=True); p.add_argument("--seconds", type=float, default=3)
    sub.add_parser("show")
    a = ap.parse_args()
    cfg = Config.load(a.config)
    if a.cmd == "show":
        import json
        from dataclasses import asdict
        print(json.dumps(asdict(cfg), indent=2))
        return
    {"nominal": cmd_nominal, "intrinsics": cmd_intrinsics, "pose": cmd_pose,
     "pose-board": cmd_pose_board, "hsv": cmd_hsv, "ball": cmd_ball,
     "ballcheck": cmd_ballcheck}[a.cmd](a, cfg)
    cfg.save(a.config)
    print("wrote", a.config)


if __name__ == "__main__":
    main()
