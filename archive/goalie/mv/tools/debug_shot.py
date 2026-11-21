"""Print per-frame pipeline state for one synthetic shot (debugging aid).
usage: python tools/debug_shot.py BALL DIST SPEED FPS SEED"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from config import Config, DRAG_QUAD
from synth import Renderer, default_camera, make_shot, truth_labels
from tracker import Tracker

ball, dist, speed, fps, seed = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
rng = np.random.default_rng(seed)
cfg = Config(); cfg.camera = default_camera(); cfg.set_ball(ball)
shot = make_shot(rng, dist, speed, cfg.ball_diam / 2, DRAG_QUAD[ball], cfg.goal_w, cfg.goal_h)
print(f"truth x={shot.x_cross:+.3f} y={shot.y_cross:.3f} t={shot.t_cross:.3f} apex={shot.apex:.2f}")
rend = Renderer(cfg.camera, cfg.ball_diam, rng, fps=fps)
trk = Tracker(cfg)
from synth import Shot
empty = Shot(shot.t, shot.p + np.array([0, 0, 100.0]), shot.v, 0, 0, 0, 0)
for _ in range(25):
    trk.step(rend.frame(empty, 0.0), -10.0)
trk.clear()
for t in np.arange(0.01, shot.t_cross, 1 / fps):
    st = trk.step(rend.frame(shot, t), t)
    gt = truth_labels(shot, t)
    d = st.det
    m = st.meas
    c = st.cross
    print(f"t={t:.3f} gt=({gt.pos[0]:+.2f},{gt.pos[1]:.2f},{gt.pos[2]:.2f}) "
          f"det={'-' if d is None else f'({d.u:.0f},{d.v:.0f}) d={d.diam:.1f} clip={d.clipped}'} "
          f"meas={'-' if m is None else np.round(m.p, 2)} n={st.n_track} "
          f"cross={'-' if not c.coming else f'x{c.x:+.2f} y{c.y:.2f} t{c.t_cross:.3f}'}")
