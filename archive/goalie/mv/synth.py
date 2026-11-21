"""Synthetic renderer + accuracy sweep for the MV pipeline (no hardware).

Truth: ball of diameter D launched from Z = dist toward a random point in the
goal mouth, integrated with gravity + realistic QUADRATIC air drag (DRAG_QUAD
per ball) and a floor bounce.  Each camera frame is rendered exactly: a pixel
(2x2 supersampled) is "ball" if its ray lies inside the sphere's cone.  The
exposure is averaged over several sub-frames (motion blur), then mild optical
blur and sensor noise; optional static same-colour clutter.  The REAL
detect -> geometry -> tracker/fit pipeline runs on those frames and the
crossing prediction is scored against the true crossing.

  python synth.py --demo --stage predict --ball tennis --dist 3 --speed 6
        -> out/demo_*.png (+ out/demo.mp4) with live-identical labels and GT
  python synth.py --sweep            -> out/sweep.md, sweep.csv, sweep_*.png
  python synth.py --sweep --quick    (fewer shots, for a smoke test)

Primary metric: error of the crossing prediction available LEAD s before the
ball crosses (default 0.15 s: what a goalie needs to move), and "final" (last
frame before crossing).  "ok3" = fraction of shots with 2D error <= 3 cm.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import math
import os
import sys
import time
from dataclasses import dataclass
from multiprocessing import Pool
from pathlib import Path

import cv2
import numpy as np

from config import BALLS, DRAG_QUAD, Camera, Config
from tracker import Labels, Tracker, labels_from

OUT = Path(__file__).resolve().parent / "out"
BALL_BGR = np.array([30, 120, 240], np.float32)   # orange


def default_camera(w=640, h=480):
    cam = Camera.nominal(w, h, 55)
    cam.set_pose(height=0.35, back=0.25, pitch_deg=8)
    return cam


# ----------------------------------------------------------------- truth ---
@dataclass
class Shot:
    t: np.ndarray        # dense times
    p: np.ndarray        # dense positions (N,3)
    v: np.ndarray
    t_cross: float
    x_cross: float
    y_cross: float
    apex: float


def simulate(p0, v0, r, kq, g=9.81, e=0.75, rolling=False, dt=5e-4, t_max=4.0):
    p, v = np.array(p0, float), np.array(v0, float)
    ts, ps, vs = [0.0], [p.copy()], [v.copy()]
    t = 0.0
    gv = np.array([0, -g, 0.0]) if not rolling else np.zeros(3)

    def acc(vv):
        return gv - kq * np.linalg.norm(vv) * vv

    while t < t_max and p[2] > -0.3:
        k1 = acc(v)
        k2 = acc(v + 0.5 * dt * k1)
        k3 = acc(v + 0.5 * dt * k2)
        k4 = acc(v + dt * k3)
        p = p + dt * (v + dt / 6 * (k1 + k2 + k3))
        v = v + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if not rolling and p[1] < r and v[1] < 0:
            p[1] = 2 * r - p[1]
            v[1] = -e * v[1]
        t += dt
        ts.append(t), ps.append(p.copy()), vs.append(v.copy())
    T, P, V = np.array(ts), np.array(ps), np.array(vs)
    i = int(np.argmax(P[:, 2] <= 0)) if (P[:, 2] <= 0).any() else -1
    if i <= 0:
        return None
    a = P[i - 1, 2] / (P[i - 1, 2] - P[i, 2])
    tc = T[i - 1] + a * dt
    pc = P[i - 1] + a * (P[i] - P[i - 1])
    return Shot(T, P, V, tc, pc[0], pc[1], float(P[:i, 1].max()))


def make_shot(rng, dist, speed, r, kq, goal_w, goal_h, rolling=False):
    """Aim at a random point of the goal mouth; pick v0 by shooting (2 Newton
    passes on the launch velocity so drag is accounted for)."""
    x0 = rng.uniform(-0.4, 0.4)
    y0 = r if rolling else rng.uniform(0.15, 0.5)
    xt = rng.uniform(-goal_w / 2 + r, goal_w / 2 - r)
    yt = r if rolling else rng.uniform(r, goal_h - r)
    p0 = np.array([x0, y0, dist])
    tgt = np.array([xt, yt, 0.0])
    hor = math.hypot(xt - x0, dist)
    T = hor / speed
    a = np.array([0, 0 if rolling else -9.81, 0])
    v0 = (tgt - p0 - 0.5 * a * T * T) / T
    for _ in range(3):
        s = simulate(p0, v0, r, kq, rolling=rolling, t_max=3 * T + 0.5)
        if s is None:
            return None
        miss = np.array([s.x_cross - xt, 0 if rolling else s.y_cross - yt])
        v0[0] -= miss[0] / s.t_cross
        if not rolling:
            v0[1] -= miss[1] / s.t_cross
        if hypot2(miss) < 1e-3:
            break
    s = simulate(p0, v0, r, kq, rolling=rolling, t_max=3 * T + 0.5)
    return s


def hypot2(m):
    return float(np.hypot(*m))


# -------------------------------------------------------------- renderer ---
class Renderer:
    def __init__(self, cam: Camera, ball_diam, rng, noise=3.0, clutter=False, subframes=6,
                 exposure=None, fps=30):
        self.cam, self.r, self.rng = cam, ball_diam / 2, rng
        self.noise, self.sub, self.fps = noise, subframes, fps
        self.exposure = (1.0 / fps) if exposure is None else exposure
        W, H = cam.w, cam.h
        # background: smooth grey-green texture + a "floor" band
        base = np.empty((H, W, 3), np.float32)
        base[:] = (105, 118, 108)
        lum = cv2.resize(rng.normal(0, 1, (H // 16, W // 16)).astype(np.float32), (W, H),
                         interpolation=cv2.INTER_CUBIC)
        chroma = cv2.resize(rng.normal(0, 1, (H // 16, W // 16, 3)).astype(np.float32), (W, H),
                            interpolation=cv2.INTER_CUBIC)
        base += 12 * lum[..., None] + 3 * chroma
        base[int(H * 0.62):] += (-10, -4, 8)
        if clutter:  # static orange-ish clutter + other coloured blobs
            for _ in range(3):
                c = (int(rng.uniform(40, W - 40)), int(rng.uniform(40, H - 40)))
                cv2.circle(base, c, int(rng.uniform(8, 20)), tuple(float(x) for x in BALL_BGR * rng.uniform(0.85, 1.05)), -1)
            for _ in range(6):
                c = (int(rng.uniform(0, W)), int(rng.uniform(0, H)))
                col = tuple(float(x) for x in rng.uniform(30, 230, 3))
                cv2.rectangle(base, c, (c[0] + int(rng.uniform(10, 60)), c[1] + int(rng.uniform(10, 60))), col, -1)
        self.bg = base
        # per-pixel camera-frame rays at 2x2 sub-pixel offsets
        self.rays = []
        for oy, ox in ((-0.25, -0.25), (-0.25, 0.25), (0.25, -0.25), (0.25, 0.25)):
            yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
            d = np.stack([(xx + ox - cam.cx) / cam.fx, (yy + oy - cam.cy) / cam.fy,
                          np.ones_like(xx)], -1)
            d /= np.linalg.norm(d, axis=-1, keepdims=True)
            self.rays.append(d)
        self.noise_pool = [rng.normal(0, noise, (H, W, 3)).astype(np.float32) for _ in range(6)]

    def frame(self, shot: Shot, t: float):
        cam, r = self.cam, self.r
        W, H = cam.w, cam.h
        img = self.bg.copy()
        ts = t - self.exposure / 2 + self.exposure * (np.arange(self.sub) + 0.5) / self.sub
        cov = np.zeros((H, W), np.float32)
        centers = []
        for tt in ts:
            p = np.array([np.interp(tt, shot.t, shot.p[:, k]) for k in range(3)])
            pc = cam.Rm @ (p - cam.Cv)
            D = np.linalg.norm(pc)
            if pc[2] <= r or D <= r * 1.05:
                continue
            centers.append((pc, D))
        if centers:
            # ROI from projected extents
            us, vs = [], []
            for pc, D in centers:
                rad = cam.f * r / math.sqrt(max(D * D - r * r, 1e-9)) * (1 + abs(pc[0] / pc[2]) + abs(pc[1] / pc[2])) + 3
                u = cam.cx + cam.fx * pc[0] / pc[2]
                v = cam.cy + cam.fy * pc[1] / pc[2]
                us += [u - rad, u + rad]
                vs += [v - rad, v + rad]
            x0, x1 = max(0, int(min(us))), min(W, int(max(us)) + 2)
            y0, y1 = max(0, int(min(vs))), min(H, int(max(vs)) + 2)
            if x1 > x0 and y1 > y0:
                acc = np.zeros((y1 - y0, x1 - x0), np.float32)
                for pc, D in centers:
                    dirc = (pc / D).astype(np.float32)
                    cosa = math.sqrt(1 - (r / D) ** 2)
                    for R in self.rays:
                        acc += (R[y0:y1, x0:x1] @ dirc >= cosa)
                cov[y0:y1, x0:x1] = acc / (4 * len(ts))
        if cov.any():
            shade = BALL_BGR[None, None, :]
            img = img * (1 - cov[..., None]) + shade * cov[..., None]
        img = cv2.GaussianBlur(img, (0, 0), 0.7)
        img += self.noise_pool[int(self.rng.integers(len(self.noise_pool)))]
        return np.clip(img, 0, 255).astype(np.uint8)


def truth_labels(shot: Shot, t: float) -> Labels:
    p = np.array([np.interp(t, shot.t, shot.p[:, k]) for k in range(3)])
    v = np.array([np.interp(t, shot.t, shot.v[:, k]) for k in range(3)])
    return labels_from(p, v)


# ---------------------------------------------------------------- runner ---
def run_shot(cfg: Config, shot: Shot, rng, fps=30, noise=3.0, clutter=False, lead=0.15,
             exposure=None, keep_frames=False, stage="predict", true_diam=None):
    cam = cfg.camera
    rend = Renderer(cam, true_diam or cfg.ball_diam, rng, noise, clutter, fps=fps, exposure=exposure)
    trk = Tracker(cfg)
    t0 = rng.uniform(0, 1 / fps)
    times = np.arange(t0, shot.t_cross + 0.5 / fps, 1 / fps)
    if cfg.detect.use_bg or cfg.detect.static_reject:  # warm up on empty frames
        empty = Shot(shot.t, shot.p + np.array([0, 0, 100.0]), shot.v, 0, 0, 0, 0)
        for _ in range(25):
            trk.step(rend.frame(empty, 0.0), -10.0)
        trk.clear()
    recs, frames = [], []
    for t in times:
        img = rend.frame(shot, t)
        st = trk.step(img, t)
        recs.append((t, st))
        if keep_frames:
            from overlay import render
            gt = truth_labels(shot, t)
            out = render(img, st, cfg, stage, trk.det.mask,
                         hud=[f"SYNTH t={t * 1000:.0f}ms  cross in {(shot.t_cross - t) * 1000:.0f}ms",
                              "grey GT = ground truth"], gt=gt,
                         gt_cross=(shot.x_cross, shot.y_cross))
            frames.append(out)
    n_det = sum(1 for _, s in recs if s.meas is not None)

    def at(tcut):
        cand = [(t, s) for t, s in recs if t <= tcut and s.cross.coming]
        return cand[-1] if cand else (None, None)

    res = {"n_frames": len(recs), "n_det": n_det}
    for tag, tcut in (("lead", shot.t_cross - lead), ("final", shot.t_cross)):
        t, s = at(tcut)
        if s is None:
            res[tag] = None
            continue
        c, kc = s.cross, s.kf_cross
        res[tag] = dict(ex=c.x - shot.x_cross, ey=c.y - shot.y_cross,
                        et=c.t_cross - shot.t_cross,
                        kex=(kc.x - shot.x_cross) if kc.coming else float("nan"),
                        key=(kc.y - shot.y_cross) if kc.coming else float("nan"),
                        sig=math.sqrt(np.trace(c.cov_xy)) if c.cov_xy is not None else float("nan"))
        if s.labels.speed is not None:
            gt = truth_labels(shot, t)
            res[tag]["e_speed"] = s.labels.speed / gt.speed - 1
            res[tag]["e_head"] = (s.labels.heading - gt.heading) if s.labels.heading is not None else float("nan")
    return res, frames


def cell_job(args):
    ball, dist, speed, fps, n, seed, mode, lead, clutter, use_bg, exposure, size_method, size_bias = args
    rng = np.random.default_rng(seed)
    cfg = Config()
    cfg.camera = default_camera()
    cfg.set_ball(ball)
    cfg.rolling = mode == "roll"
    cfg.detect.use_bg = use_bg
    cfg.detect.size_method = size_method
    r = cfg.ball_diam / 2
    out = []
    lobs = 0
    for i in range(n):
        shot = make_shot(rng, dist, speed, r, DRAG_QUAD[ball], cfg.goal_w, cfg.goal_h, cfg.rolling)
        if shot is None or (not cfg.rolling and shot.apex > 1.2):
            lobs += 1
            continue
        true_d = cfg.ball_diam
        cfg.ball_diam = true_d * (1 + size_bias)   # pipeline believes a wrong size
        res, _ = run_shot(cfg, shot, rng, fps, clutter=clutter, lead=lead, exposure=exposure,
                          true_diam=true_d)
        cfg.ball_diam = true_d
        out.append(res)
    return (ball, dist, speed, fps, mode), out, lobs, n


def summarize(results, tag):
    rows = [r[tag] for r in results if r.get(tag)]
    if not rows:
        return None
    cover = len(rows) / max(1, len(results))
    e2 = np.array([math.hypot(r["ex"], r["ey"]) for r in rows])
    ke = np.array([math.hypot(r["kex"], r["key"]) for r in rows])
    es = np.array([abs(r.get("e_speed", np.nan)) for r in rows])
    return dict(n=len(rows), cover=cover, med=float(np.median(e2)), p90=float(np.percentile(e2, 90)),
                ok3=float((e2 <= 0.03).mean()), ok10=float((e2 <= 0.10).mean()),
                medx=float(np.median(np.abs([r["ex"] for r in rows]))),
                medy=float(np.median(np.abs([r["ey"] for r in rows]))),
                medt=float(np.median(np.abs([r["et"] for r in rows]))),
                kf_med=float(np.nanmedian(ke)) if np.isfinite(ke).any() else float("nan"),
                spd=float(np.nanmedian(es)) if np.isfinite(es).any() else float("nan"))


def sweep(a):
    OUT.mkdir(exist_ok=True)
    balls = a.balls.split(",")
    dists = [float(x) for x in a.dists.split(",")]
    speeds = [float(x) for x in a.speeds.split(",")]
    fpss = [int(x) for x in a.fpss.split(",")]
    jobs = []
    seed = 1000
    for mode in a.modes.split(","):
        for ball, d, s, f in itertools.product(balls, dists, speeds, fpss):
            seed += 1
            jobs.append((ball, d, s, f, a.shots, seed, mode, a.lead, a.clutter, a.bg,
                         a.exposure, a.size_method, a.size_bias))
    t0 = time.time()
    with Pool(a.jobs) as pool:
        res = pool.map(cell_job, jobs, chunksize=1)
    print(f"sweep: {len(jobs)} cells x {a.shots} shots in {time.time() - t0:.0f}s", file=sys.stderr)
    lines = [f"# MV synthetic sweep\n",
             f"camera 640x480 HFOV55 at (0,0.35,-0.25) pitch 8deg; exposure="
             f"{'1/fps' if a.exposure is None else f'{a.exposure*1000:.0f}ms'}; noise 3; "
             f"clutter={a.clutter}; bg={a.bg}; size={a.size_method}; size_bias={a.size_bias:+.0%}; truth has quadratic drag; "
             f"prediction scored {a.lead*1000:.0f} ms before crossing ('lead') and at the last frame ('final').\n",
             "err = 2D crossing error (X, height) in cm; ok3 = share within 3 cm; spd = median speed-label error.\n",
             "| mode | ball | dist m | speed m/s | fps | n | pred% | lead med cm | lead p90 cm | lead ok3 | lead ok10 | KF lead med | final med cm | final ok3 | |t| err ms | spd err |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    rows_csv = []
    for (ball, d, s, f, mode), out, lobs, n in res:
        L, F = summarize(out, "lead"), summarize(out, "final")
        key = f"| {mode} | {ball} | {d:g} | {s:g} | {f} |"
        if L is None:
            flight = d / s
            why = ("lob (apex>1.2m)" if lobs > n / 2 else
                   f"flight {flight*1000:.0f}ms < lead+fit" if flight < a.lead + 6 / f else "no prediction")
            fin = f"final med {F['med']*100:.1f} cm" if F else ""
            lines.append(f"{key} {len(out)} | 0% | {why} | | | | | {fin} | | | |")
            rows_csv.append(dict(mode=mode, ball=ball, dist=d, speed=s, fps=f, n=len(out), note=why))
            continue
        def g(D, k, sc=100, fmt=".1f"):
            return "--" if D is None or not math.isfinite(D[k]) else f"{D[k] * sc:{fmt}}"
        lines.append(f"{key} {len(out)} | {g(L,'cover',100,'.0f')}% | {g(L,'med')} | {g(L,'p90')} | {g(L,'ok3',100,'.0f')}% | "
                     f"{g(L,'ok10',100,'.0f')}% | {g(L,'kf_med')} | {g(F,'med')} | {g(F,'ok3',100,'.0f')}% | "
                     f"{g(L,'medt',1000,'.0f')} | {g(L,'spd',100,'.0f')}% |")
        rows_csv.append(dict(mode=mode, ball=ball, dist=d, speed=s, fps=f, n=len(out),
                             **{f"lead_{k}": v for k, v in (L or {}).items()},
                             **{f"final_{k}": v for k, v in (F or {}).items()}))
    tag = a.tag
    (OUT / f"sweep{tag}.md").write_text("\n".join(lines) + "\n")
    keys = sorted({k for r in rows_csv for k in r})
    with open(OUT / f"sweep{tag}.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows_csv)
    print("\n".join(lines))
    plot_sweep(rows_csv, tag)


def plot_sweep(rows, tag):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    for mode in sorted({r["mode"] for r in rows}):
        rs = [r for r in rows if r["mode"] == mode]
        balls = sorted({r["ball"] for r in rs}, key=lambda b: BALLS[b])
        fpss = sorted({r["fps"] for r in rs})
        dists = sorted({r["dist"] for r in rs})
        speeds = sorted({r["speed"] for r in rs})
        fig, axs = plt.subplots(len(fpss), len(balls), figsize=(4 * len(balls), 3.4 * len(fpss)),
                                squeeze=False, constrained_layout=True)
        for i, f in enumerate(fpss):
            for j, b in enumerate(balls):
                M = np.full((len(speeds), len(dists)), np.nan)
                why = {}
                for r in rs:
                    if r["fps"] == f and r["ball"] == b:
                        ij = (speeds.index(r["speed"]), dists.index(r["dist"]))
                        if "lead_med" in r:
                            M[ij] = r["lead_med"] * 100
                        else:
                            note = r.get("note", "")
                            why[ij] = "lob" if note.startswith("lob") else ("too\nfast" if "flight" in note else "none")
                ax = axs[i][j]
                im = ax.imshow(M, origin="lower", cmap="RdYlGn_r", vmin=0, vmax=15, aspect="auto")
                for (yy, xx), val in np.ndenumerate(M):
                    ax.text(xx, yy, why.get((yy, xx), "n/a") if np.isnan(val) else f"{val:.1f}",
                            ha="center", va="center", fontsize=9)
                ax.set_xticks(range(len(dists)), [f"{d:g}" for d in dists])
                ax.set_yticks(range(len(speeds)), [f"{s:g}" for s in speeds])
                ax.set_xlabel("shot distance m")
                ax.set_ylabel("speed m/s")
                ax.set_title(f"{b} {BALLS[b]*1000:.0f}mm @{f}fps")
        fig.suptitle(f"{mode}: median crossing error (cm) of the prediction 150 ms before the goal\n"
                     "lob = arc too high to be a shot; too fast = flight shorter than 150 ms + 5 frames")
        fig.colorbar(im, ax=axs, shrink=0.8, label="cm")
        fig.savefig(OUT / f"sweep{tag}_{mode}.png", dpi=110)
        plt.close(fig)


def demo(a):
    OUT.mkdir(exist_ok=True)
    rng = np.random.default_rng(a.seed)
    cfg = Config()
    cfg.camera = default_camera()
    cfg.set_ball(a.ball)
    cfg.rolling = a.mode == "roll"
    cfg.detect.use_bg = a.bg
    shot = make_shot(rng, a.dist, a.speed, cfg.ball_diam / 2, DRAG_QUAD[a.ball], cfg.goal_w,
                     cfg.goal_h, cfg.rolling)
    res, frames = run_shot(cfg, shot, rng, a.fps, clutter=a.clutter, keep_frames=True,
                           stage=a.stage, exposure=a.exposure)
    print(f"truth crossing x={shot.x_cross:+.3f} h={shot.y_cross:.3f} t={shot.t_cross*1000:.0f}ms "
          f"apex={shot.apex:.2f}m; frames={res['n_frames']} detected={res['n_det']}")
    for tag in ("lead", "final"):
        r = res[tag]
        if r:
            print(f"  {tag:5s}: err x {r['ex']*100:+.1f}cm h {r['ey']*100:+.1f}cm t {r['et']*1000:+.0f}ms "
                  f"(1-sigma {r['sig']*100:.1f}cm) KF x {r['kex']*100:+.1f} h {r['key']*100:+.1f}cm "
                  f"speed-label err {r.get('e_speed', float('nan'))*100:+.0f}%")
        else:
            print(f"  {tag:5s}: no prediction")
    idx = sorted({len(frames) // 3, 2 * len(frames) // 3, len(frames) - 2})
    for k in idx:
        cv2.imwrite(str(OUT / f"demo_{a.stage}_{k:03d}.png"), frames[k])
    # raw-ish video for live.py --video replay tests is written separately
    vw = cv2.VideoWriter(str(OUT / f"demo_{a.stage}.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), a.fps,
                         (frames[0].shape[1], frames[0].shape[0]))
    for f in frames:
        vw.write(f)
    vw.release()
    print("wrote", ", ".join(f"out/demo_{a.stage}_{k:03d}.png" for k in idx), f"and out/demo_{a.stage}.mp4")


def write_raw_clip(a):
    """Unannotated synthetic clip for testing live.py --video."""
    OUT.mkdir(exist_ok=True)
    rng = np.random.default_rng(a.seed)
    cam = default_camera()
    r = BALLS[a.ball] / 2
    rend = Renderer(cam, BALLS[a.ball], rng, fps=a.fps, clutter=a.clutter)
    vw = cv2.VideoWriter(str(OUT / "synth_raw.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (cam.w, cam.h))
    for k in range(3):
        shot = make_shot(rng, a.dist, a.speed, r, DRAG_QUAD[a.ball], 0.8, 0.5)
        for t in np.arange(0, shot.t_cross + 0.4, 1 / a.fps):
            vw.write(rend.frame(shot, t))
    vw.release()
    print("wrote out/synth_raw.mp4")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--demo", action="store_true")
    ap.add_argument("--raw-clip", action="store_true", help="write out/synth_raw.mp4 for live.py --video")
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--balls", default="pingpong,tennis,soccer")
    ap.add_argument("--dists", default="1,2,3,4")
    ap.add_argument("--speeds", default="2,5,10")
    ap.add_argument("--fpss", default="30,60")
    ap.add_argument("--modes", default="air")
    ap.add_argument("--shots", type=int, default=24)
    ap.add_argument("--lead", type=float, default=0.15)
    ap.add_argument("--clutter", action="store_true")
    ap.add_argument("--bg", action="store_true", help="MOG2 motion mask")
    ap.add_argument("--exposure", type=float, default=None, help="s (default = 1/fps)")
    ap.add_argument("--size-bias", type=float, default=0.0,
                    help="pipeline assumes ball diameter x(1+bias) (systematic-error test)")
    ap.add_argument("--size-method", default="moments", choices=["moments", "ellipse"])
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) - 2))
    ap.add_argument("--tag", default="")
    ap.add_argument("--ball", default="tennis", choices=list(BALLS))
    ap.add_argument("--dist", type=float, default=3.0)
    ap.add_argument("--speed", type=float, default=6.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--mode", default="air", choices=["air", "roll"])
    ap.add_argument("--stage", default="predict", choices=["detect", "track", "predict"])
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    if a.quick:
        a.shots = 4
    if a.sweep:
        sweep(a)
    if a.demo:
        demo(a)
    if a.raw_clip:
        write_raw_clip(a)
    if not (a.sweep or a.demo or a.raw_clip):
        ap.print_help()


if __name__ == "__main__":
    main()
