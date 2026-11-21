"""Board-pose math and config round trip."""
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from calib import board_points, pose_from_board  # noqa: E402
from config import Camera, Config  # noqa: E402


def test_pose_from_board_recovers_camera():
    cam = Camera.nominal(640, 480, 55)
    cam.set_pose(height=0.45, back=0.3, x=0.05, pitch_deg=20, yaw_deg=-4)
    board, sq, origin = (9, 6), 0.03, (-0.12, 0.8)
    obj = board_points(board, sq)
    # board x -> world +X, board y -> world +Z, floor Y = 0
    Pw = np.stack([origin[0] + obj[:, 0], np.zeros(len(obj)), origin[1] + obj[:, 1]], 1)
    pc = (cam.Rm @ (Pw - cam.Cv).T).T
    px = np.stack([cam.fx * pc[:, 0] / pc[:, 2] + cam.cx, cam.fy * pc[:, 1] / pc[:, 2] + cam.cy], 1)
    ok, rv, tv = cv2.solvePnP(obj, px.astype(np.float32), cam.K, np.zeros(5))
    R, C = pose_from_board(rv, tv, origin)
    assert np.allclose(C, cam.Cv, atol=1e-4), C
    assert np.allclose(R, cam.Rm, atol=1e-4)


def test_config_roundtrip(tmp_path):
    cfg = Config()
    cfg.set_ball("pingpong")
    cfg.camera.set_pose(0.5, 0.2, 0.0, 12, 3)
    p = tmp_path / "c.json"
    cfg.save(p)
    c2 = Config.load(p)
    assert c2.ball_diam == 0.04 and np.allclose(c2.camera.Rm, cfg.camera.Rm)


def test_calib_cli(tmp_path):
    p = str(tmp_path / "c.json")
    py = sys.executable
    for args in (["nominal", "--hfov", "60"], ["pose", "--height", "0.4", "--pitch", "10"],
                 ["ball", "tennis"], ["show"]):
        r = subprocess.run([py, "calib.py", "--config", p, *args], cwd=HERE, capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
    c = Config.load(p)
    assert abs(c.camera.fx - 320 / np.tan(np.radians(30))) < 1e-6 and abs(c.camera.C[1] - 0.4) < 1e-9
