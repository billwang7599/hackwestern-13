"""Beat the Goalie simulator: is the DC gearmotor fast enough?

    python run.py                      # default scenario: 60 casual 1.0 m/s rolls, exit 0 = pass
    python run.py --feasibility        # fastest stoppable shot, RPM 100/150/200, camera table
    python run.py --feasibility --sweep-rpm   # + minimum RPM for 1.0 / 1.5 m/s, pivot and rail
    python run.py --live               # animated top-down view; keys below
    python run.py --rpm 90 --latency-ms 60 --fps 30 --geometry rail --paddle-len 0.08
    python run.py --gains kp=8,kd=0.3 --params my_bench.json

Measured your motor?  `--rpm <no-load RPM at your supply voltage>` (docs/rpm_test.md).
The Pi's motor model is given the same RPM unless you pass --model-rpm.

Live keys: space = shoot, left/right = aim, up/down = speed, click = move the
ball's start spot, a = auto-shooter on/off, r = reset score, q = quit.
Headless (MPLBACKEND=Agg) --live plays one scripted shot and saves out/live_frame.png.

Pass/fail: the default scenario passes when the goalie saves >= 90% of
on-target shots rolled at 1.0 m/s from 60 cm, aimed uniformly across the goal.
1.0 m/s is the middle of a casual underhand roll (0.8-1.5 m/s: 60 cm in
0.4-0.75 s); the demo needs to beat casual players, the fast end is reported
by --feasibility.
"""
from __future__ import annotations

import argparse
import copy
import math
import os
import sys
import tempfile

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "goalie-mpl"))

from params import CASUAL_SHOT, DEG, SAVE_TARGET, SimParams  # noqa: E402
from simulate import Sim, run_shot, save_rate, shot_set, tuned  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

SPEEDS = [round(0.5 + 0.25 * i, 2) for i in range(19)]           # 0.5 .. 5.0 m/s
RPM_GRID = [5, 8, 10, 12, 15, 20, 25, 30, 40, 50, 60, 75, 100, 125, 150, 200, 250, 300]


# ---------------------------------------------------------------- params
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--feasibility", action="store_true", help="speed sweep + verdict")
    ap.add_argument("--sweep-rpm", action="store_true", help="with --feasibility: min RPM tables")
    ap.add_argument("--live", action="store_true", help="animated top-down view")
    ap.add_argument("--geometry", choices=["pivot", "rail"])
    ap.add_argument("--rpm", type=float, help="no-load output RPM (default 150)")
    ap.add_argument("--rpm-model", choices=["gear", "fixed"],
                    help="how torque scales with --rpm (default gear: same motor, other gearbox)")
    ap.add_argument("--stall", type=float, help="stall torque N*m at the reference RPM (default 0.10)")
    ap.add_argument("--paddle-len", type=float, help="pivot arm length / rail blade width, m (0.12)")
    ap.add_argument("--paddle-w", type=float, help="pivot blade width, m (0.05)")
    ap.add_argument("--spool-r", type=float, help="rail spool radius, m (0.015)")
    ap.add_argument("--goal-w", type=float, help="goal width, m (0.22)")
    ap.add_argument("--fps", type=float, help="camera fps (30)")
    ap.add_argument("--latency-ms", type=float, help="camera capture->Pi latency (50)")
    ap.add_argument("--noise-mm", type=float, help="detection noise 1-sigma (2)")
    ap.add_argument("--drop", type=float, help="probability a detection is missed (0)")
    ap.add_argument("--backlash-deg", type=float, help="gearbox backlash (3)")
    ap.add_argument("--deadband", type=float, help="motor deadband in duty (0.12)")
    ap.add_argument("--friction", type=float, help="gearbox friction as a fraction of stall torque (0.10)")
    ap.add_argument("--walls", action="store_true", help="bank shots off the side walls")
    ap.add_argument("--model-rpm", type=float, help="Pi's belief of the RPM (default = --rpm)")
    ap.add_argument("--latency-est-ms", type=float, help="Pi's assumed latency (default = --latency-ms)")
    ap.add_argument("--gains", help="kp=6,kd=0.25,db_comp=0.12,obs_l1=0.5,obs_l2=4 (ControlParams)")
    ap.add_argument("--params", help="JSON overrides, e.g. {\"motor\": {\"rpm\": 120}}")
    ap.add_argument("--speed", type=float, default=CASUAL_SHOT, help="scenario shot speed m/s")
    ap.add_argument("--shots", type=int, default=60, help="shots per data point")
    ap.add_argument("--aim", choices=["uniform", "corners"], default="uniform")
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--frames", type=int, default=0, help="--live headless: frames to render")
    return ap.parse_args(argv)


def build(a, **over) -> SimParams:
    """CLI args (+ overrides with the same names) -> SimParams with a matched Pi model."""
    o = dict(vars(a))
    o.update(over)
    p = SimParams.from_json(o["params"])
    if o["geometry"]: p.goalie.geometry = o["geometry"]
    if o["rpm"]: p.motor.rpm = o["rpm"]
    if o["rpm_model"]: p.motor.rpm_model = o["rpm_model"]
    if o["stall"]: p.motor.ref_stall_torque = o["stall"]
    if o["paddle_len"]: p.goalie.paddle_len = o["paddle_len"]
    if o["paddle_w"]: p.goalie.paddle_w = o["paddle_w"]
    if o["spool_r"]: p.goalie.spool_radius = o["spool_r"]
    if o["goal_w"]: p.field.goal_half_w = o["goal_w"] / 2
    if o["fps"]: p.camera.fps = o["fps"]
    if o["latency_ms"] is not None: p.camera.latency = o["latency_ms"] / 1000
    if o["noise_mm"] is not None: p.camera.noise = o["noise_mm"] / 1000
    if o["drop"] is not None: p.camera.drop = o["drop"]
    if o["backlash_deg"] is not None: p.motor.backlash = o["backlash_deg"] * DEG
    if o["deadband"] is not None: p.motor.deadband = o["deadband"]
    if o["friction"] is not None: p.motor.coulomb_frac = o["friction"]
    if o["walls"]: p.field.walls = True
    p = tuned(p)                                      # Pi model = real motor (RPM was measured)
    p.control.latency_est = p.camera.latency
    if o["model_rpm"]: p.control.model_rpm = o["model_rpm"]
    if o["latency_est_ms"] is not None: p.control.latency_est = o["latency_est_ms"] / 1000
    if o["gains"]:
        for kv in o["gains"].split(","):
            k, v = kv.split("=")
            if not hasattr(p.control, k.strip()):
                sys.exit(f"unknown gain {k}")
            setattr(p.control, k.strip(), float(v))
    if o.get("static"):
        p.motor.deadband = 9.0                        # motor never moves: baseline
    return p


def describe(p: SimParams) -> str:
    g, m, c = p.goalie, p.motor, p.camera
    geo = (f"pivot arm {g.paddle_len*100:.0f} cm, blade {g.paddle_w*100:.0f} cm" if g.geometry == "pivot"
           else f"rail, blade {g.paddle_len*100:.0f} cm, spool r {g.spool_radius*1000:.0f} mm")
    return (f"{geo}; motor {m.rpm:.0f} RPM ({m.rpm_model}, stall {m.stall_torque:.2f} N*m, "
            f"tau {p.control.model_tau*1000:.0f} ms, backlash {m.backlash/DEG:.0f} deg); "
            f"goal {p.field.goal_half_w*200:.0f} cm; camera {c.fps:.0f} fps, "
            f"{c.latency*1000:.0f} ms, noise {c.noise*1000:.0f} mm")


# ---------------------------------------------------------------- metrics
def rate(p, v, a, aim=None):
    return save_rate(p, v, a.shots, a.seed, aim or a.aim)[0]


def fastest(p, a, aim=None, speeds=SPEEDS) -> float:
    """Highest speed in the grid such that it and every slower speed are saved >= 90%."""
    best = 0.0
    for v in speeds:
        if rate(p, v, a, aim) < SAVE_TARGET:
            break
        best = v
    return best


def min_rpm(a, geometry, v, aim) -> float | None:
    """Smallest grid RPM from which every faster motor saves >= 90% at speed v."""
    ok = None
    for rpm in reversed(RPM_GRID):
        if rate(build(a, geometry=geometry, rpm=rpm), v, a, aim) >= SAVE_TARGET:
            ok = rpm
        else:
            break
    return ok


def fmt_v(v):
    return "< 0.5" if v == 0 else (f">= {v:.2f}" if v >= SPEEDS[-1] else f"{v:.2f}")


def fmt_rpm(r):
    return "none <= 300" if r is None else (f"<= {r}" if r == RPM_GRID[0] else f"{r}")


# ---------------------------------------------------------------- modes
def scenario(a) -> int:
    p = build(a)
    print(f"Scenario: {a.shots} shots at {a.speed:.2f} m/s from {p.field.field_len*100:.0f} cm, "
          f"aim {a.aim} across the goal")
    print("  " + describe(p))
    sr, res = save_rate(p, a.speed, a.shots, a.seed, a.aim)
    sr_c = rate(p, a.speed, a, "corners")
    sr_0 = rate(build(a, static=True), a.speed, a)
    on = [r for r in res if r.outcome in ("save", "goal")]
    miss = [abs(r.x_cross - r.paddle_x) for r in on]
    print(f"  saved {sum(r.outcome == 'save' for r in on)}/{len(on)} on target = {sr*100:.0f}%   "
          f"(corner shots {sr_c*100:.0f}%, a goalie that never moves {sr_0*100:.0f}%)")
    print(f"  |ball x - paddle x| at the line: mean {1000*sum(miss)/len(miss):.0f} mm, "
          f"max {1000*max(miss):.0f} mm")
    ok = sr >= SAVE_TARGET
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "scenario.png")
    plot_scenario(p, a, res, path)
    print(f"  plot: {path}")
    print(f"{'PASS' if ok else 'FAIL'}: {sr*100:.0f}% saved, need {SAVE_TARGET*100:.0f}%")
    return 0 if ok else 1


def feasibility(a) -> int:
    import matplotlib
    matplotlib.use(os.environ.get("MPLBACKEND", "Agg"))
    import matplotlib.pyplot as plt

    p = build(a)
    geo = p.goalie.geometry
    print(f"Feasibility ({a.shots} shots per point, pass = {SAVE_TARGET*100:.0f}% of on-target shots saved)")
    print("  " + describe(p))
    print("\n  speed m/s   saved   corners   never-moves")
    curve = {"uniform": [], "corners": [], "static": []}
    ps = build(a, static=True)
    for v in SPEEDS:
        u, c, s = rate(p, v, a, "uniform"), rate(p, v, a, "corners"), rate(ps, v, a, "uniform")
        curve["uniform"].append(u); curve["corners"].append(c); curve["static"].append(s)
        print(f"  {v:8.2f}   {u*100:5.0f}%   {c*100:6.0f}%   {s*100:8.0f}%")
    best = fastest(p, a)
    best_c = fastest(p, a, "corners")

    print(f"\n  Fastest stoppable shot (>= {SAVE_TARGET*100:.0f}%), m/s:")
    print("    RPM    pivot (all / corners)    rail (all / corners)")
    for rpm in (30, 50, 100, 150, 200):
        cells = []
        for g in ("pivot", "rail"):
            q = build(a, rpm=rpm, geometry=g)
            cells.append(f"{fmt_v(fastest(q, a)):>7} / {fmt_v(fastest(q, a, 'corners')):<7}")
        print(f"    {rpm:4d}   {cells[0]:<22}   {cells[1]}")

    print(f"\n  Camera (pivot/rail at {p.motor.rpm:.0f} RPM and at 30 RPM): fastest stoppable, corner shots, m/s")
    print("    fps  latency   pivot@rpm   rail@rpm   pivot@30   rail@30")
    for fps in (30, 60):
        for lat in (30, 60):
            vals = [fastest(build(a, fps=fps, latency_ms=lat, geometry=g, rpm=r), a, "corners")
                    for r in (p.motor.rpm, 30) for g in ("pivot", "rail")]
            print(f"    {fps:3d}  {lat:4d} ms   " + "   ".join(f"{fmt_v(v):>8}" for v in vals))

    print(f"\n  Robustness ({geo}, {p.motor.rpm:.0f} RPM): fastest stoppable, corner shots, m/s")
    rob = [("as configured", {}),
           ("Pi thinks RPM is 30% higher", dict(model_rpm=p.motor.rpm * 1.3)),
           ("Pi thinks RPM is 30% lower", dict(model_rpm=p.motor.rpm * 0.7)),
           ("Pi latency guess 30 ms off", dict(latency_est_ms=p.camera.latency * 1000 + 30)),
           ("backlash 8 deg", dict(backlash_deg=8)),
           ("10% missed detections", dict(drop=0.10)),
           ("5 mm detection noise", dict(noise_mm=5)),
           ("deadband 0.25 (Pi compensates 0.12)", dict(deadband=0.25)),
           ("gearbox friction doubled (0.20 of stall)", dict(friction=0.20)),
           ("weak motor: 1/3 of the stall torque", dict(stall=p.motor.ref_stall_torque / 3))]
    for name, ov in rob:
        print(f"    {name:<38} {fmt_v(fastest(build(a, **ov), a, 'corners')):>7}")

    rpm_curves = None
    if a.sweep_rpm:
        print("\n  Minimum no-load RPM to save >= 90% (RPM grid 5..300):")
        print("    geometry   1.0 m/s all   1.0 corners   1.5 m/s all   1.5 corners")
        for g, extra in (("pivot", {}), ("rail", {}), ("pivot*", dict(rpm_model="fixed"))):
            aa = argparse.Namespace(**{**vars(a), **extra})
            cells = [fmt_rpm(min_rpm(aa, g.rstrip("*"), v, aim)) for v in (1.0, 1.5)
                     for aim in ("uniform", "corners")]
            print(f"    {g:<8}   " + "   ".join(f"{c:>11}" for c in cells))
        print("    * pivot with stall torque and inertia NOT rescaled with RPM (--rpm-model fixed):\n"
              "      checks the answer doesn't hinge on how a slower motor is assumed to scale")
        rpm_curves = {g: [fastest(build(a, geometry=g, rpm=r), a, "corners") for r in RPM_GRID]
                      for g in ("pivot", "rail")}

    os.makedirs(OUT, exist_ok=True)
    n = 2 if rpm_curves else 1
    fig, axs = plt.subplots(1, n, figsize=(6.5 * n, 4.5), squeeze=False)
    ax = axs[0][0]
    ax.plot(SPEEDS, [100 * v for v in curve["uniform"]], "o-", label="aimed anywhere")
    ax.plot(SPEEDS, [100 * v for v in curve["corners"]], "s-", label="aimed at the corners")
    ax.plot(SPEEDS, [100 * v for v in curve["static"]], "--", color="gray", label="goalie never moves")
    ax.axhline(SAVE_TARGET * 100, color="k", lw=0.8)
    ax.axvspan(0.8, 1.5, color="green", alpha=0.08, label="casual hand roll")
    ax.set(xlabel="shot speed (m/s)", ylabel="saved (%)", ylim=(0, 102),
           title=f"{geo}, {p.motor.rpm:.0f} RPM, {p.camera.fps:.0f} fps, {p.camera.latency*1000:.0f} ms")
    ax.legend(loc="lower left", fontsize=8); ax.grid(alpha=0.3)
    if rpm_curves:
        ax = axs[0][1]
        for g, vs in rpm_curves.items():
            ax.plot(RPM_GRID, vs, "o-", label=g)
        ax.axhspan(0.8, 1.5, color="green", alpha=0.08, label="casual hand roll")
        ax.set(xscale="log", xlabel="motor no-load RPM", ylabel="fastest stoppable corner shot (m/s)",
               title="motor speed needed")
        ax.legend(); ax.grid(alpha=0.3, which="both")
    fig.tight_layout()
    path = os.path.join(OUT, "feasibility.png")
    fig.savefig(path, dpi=110)
    print(f"\n  plot: {path}")

    s0 = curve["static"][SPEEDS.index(1.0)]
    print(f"\nVERDICT: At {p.motor.rpm:.0f} RPM a {geo} goalie stops shots up to {fmt_v(best)} m/s "
          f"(corner shots up to {fmt_v(best_c)} m/s); a casual hand roll is ~0.8-1.5 m/s. "
          f"A goalie that never moves saves {s0*100:.0f}% at 1.0 m/s.")
    n_frames = p.field.field_len / max(best_c, 0.5) * p.camera.fps
    print(f"  Caveat: at {fmt_v(best_c)} m/s the ball crosses the field in ~{n_frames:.0f} frames and moves "
          f"{100 * max(best_c, 0.5) / p.camera.fps:.0f} cm per frame; motion blur and missed detections "
          "above ~2 m/s are not modelled beyond --drop, so trust the low-speed end more.")
    return 0 if best >= CASUAL_SHOT else 1


# ---------------------------------------------------------------- plots
def plot_scenario(p, a, res, path):
    import matplotlib
    matplotlib.use(os.environ.get("MPLBACKEND", "Agg"))
    import matplotlib.pyplot as plt

    gh = p.field.goal_half_w
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 4.8))
    # one corner shot, time series
    x0, aim, sd = shot_set(1, p, a.seed + 7, "corners")[0]
    aim = 0.85 * (gh - p.ball.radius)
    x0 = -0.12
    log = []
    r = run_shot(p, a.speed, x0, aim, sd, log)
    t = [row[0] - p.pre_roll for row in log]
    a1.plot(t, [row[1] for row in log], color="tab:orange", label="ball x")
    a1.plot(t, [row[3] for row in log], "--", color="k", lw=1, label="target (prediction)")
    a1.plot(t, [row[4] for row in log], color="tab:purple", lw=1, label="Pi model x_est")
    a1.plot(t, [row[5] for row in log], color="tab:blue", lw=2, label="paddle x (true)")
    a1.set(xlabel="time since release (s)", ylabel="x (m)", xlim=(-0.2, r.t_flight + 0.1),
           title=f"corner shot at {a.speed:.1f} m/s -> {r.outcome}")
    a1.axvline(r.t_flight, color="gray", lw=0.8)
    a1.grid(alpha=0.3)
    ax2 = a1.twinx()
    ax2.plot(t, [row[6] for row in log], color="tab:green", lw=0.7, alpha=0.6)
    ax2.set_ylabel("duty", color="tab:green"); ax2.set_ylim(-1.1, 1.1)
    a1.legend(loc="lower left", fontsize=8)
    # all shots
    for o, c in (("save", "tab:green"), ("goal", "tab:red")):
        rs = [r for r in res if r.outcome == o]
        a2.scatter([r.x_cross for r in rs], [r.paddle_x for r in rs], c=c, s=18, label=o)
    a2.plot([-gh, gh], [-gh, gh], color="gray", lw=0.8)
    a2.set(xlabel="ball x at goal line (m)", ylabel="paddle x then (m)",
           title=f"{len(res)} shots at {a.speed:.1f} m/s, {p.goalie.geometry}, {p.motor.rpm:.0f} RPM")
    a2.legend(); a2.grid(alpha=0.3); a2.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------- live
def live(a) -> int:
    import random

    import matplotlib
    headless = os.environ.get("MPLBACKEND", "").lower() == "agg" or not (
        sys.platform == "darwin" or os.environ.get("DISPLAY"))
    if headless:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Rectangle

    p = build(a)
    f, g, br = p.field, p.goalie, p.ball.radius
    sim = Sim(p, a.seed)
    sim.place(0.0)
    st = {"aim": 0.06, "speed": a.speed, "auto": False, "next_auto": 1.5, "msg": "",
          "rng": random.Random(a.seed)}
    FRAME = 1 / 30

    fig = plt.figure(figsize=(9, 7.5))
    ax = fig.add_axes([0.06, 0.06, 0.6, 0.88])
    axf = fig.add_axes([0.71, 0.55, 0.27, 0.3])
    axt = fig.add_axes([0.71, 0.08, 0.27, 0.4]); axt.axis("off")
    W = f.field_half_w
    ax.set(xlim=(-W - 0.02, W + 0.02), ylim=(-0.08, f.field_len + 0.05), aspect="equal",
           title="Beat the Goalie (top view, camera's eye)")
    ax.add_patch(Rectangle((-W, 0), 2 * W, f.field_len, fc="#e8f2e0", ec="k"))
    ax.add_patch(Rectangle((-f.goal_half_w, -0.05), 2 * f.goal_half_w, 0.05, fc="#ddd", ec="k"))
    ball = Circle((0, 0), br, fc="orange", ec="k", zorder=5)
    ax.add_patch(ball)
    (paddle,) = ax.plot([], [], color="tab:blue", lw=6, solid_capstyle="butt", zorder=4)
    (est,) = ax.plot([], [], "|", color="tab:purple", ms=18, mew=2)
    (tgt,) = ax.plot([], [], "v", color="k", ms=8)
    (trail,) = ax.plot([], [], ".", color="orange", ms=3, alpha=0.5)
    (path,) = ax.plot([], [], ":", color="gray")
    (aimln,) = ax.plot([], [], "-", color="red", alpha=0.3)
    # front view (looking at the goal from the field)
    axf.set(xlim=(-0.16, 0.16), ylim=(-0.01, 0.18), aspect="equal", title="front view", xticks=[], yticks=[])
    axf.axhline(0, color="k")
    for s in (-1, 1):
        axf.axvline(s * f.goal_half_w, color="gray", lw=3)
    (arm,) = axf.plot([], [], color="tab:blue", lw=5, solid_capstyle="butt")
    fball = Circle((0, br), br, fc="orange", ec="k")
    axf.add_patch(fball)
    txt = axt.text(0, 1, "", va="top", family="monospace", fontsize=9)

    def shoot():
        if not sim.shot:
            sim.shoot(st["speed"], st["aim"])

    def on_key(e):
        k = e.key
        if k == " ":
            shoot()
        elif k == "left":
            st["aim"] = max(-f.goal_half_w, st["aim"] - 0.01)
        elif k == "right":
            st["aim"] = min(f.goal_half_w, st["aim"] + 0.01)
        elif k == "up":
            st["speed"] = min(5.0, st["speed"] + 0.25)
        elif k == "down":
            st["speed"] = max(0.25, st["speed"] - 0.25)
        elif k == "a":
            st["auto"] = not st["auto"]
        elif k == "r":
            sim.saves = sim.goals = 0
        elif k == "q":
            plt.close(fig)

    def on_click(e):
        if e.inaxes is ax and e.xdata is not None and not sim.shot and e.ydata > 0.15:
            sim.place(e.xdata, e.ydata)

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("button_press_event", on_click)

    def update(_):
        t_end = sim.t + FRAME
        while sim.t < t_end:
            r = sim.step()
            if r:
                st["msg"] = {"save": "SAVED!", "goal": "GOAL!", "wide": "wide", "stopped": "stopped"}[r.outcome]
                st["reset_at"] = sim.t + 0.8
        if not sim.shot and st.get("reset_at") and sim.t > st["reset_at"]:
            st["reset_at"] = None
            sim.place(st["rng"].uniform(-0.15, 0.15) if st["auto"] else sim.ball.x if sim.ball.y > 0.15 else 0.0)
            st["next_auto"] = sim.t + 0.6
        if st["auto"] and not sim.shot and not st.get("reset_at") and sim.t > st["next_auto"]:
            gh = f.goal_half_w - br
            st["aim"] = st["rng"].uniform(-gh, gh)
            shoot()
        b = sim.ball
        ball.center = (b.x, b.y)
        px, q = sim.goalie.x, sim.goalie.q
        if g.geometry == "pivot":
            # blade projected on the table: tip +- half width across
            hw = g.paddle_w / 2 * math.cos(q)
            paddle.set_data([px - hw, px + hw], [0, 0])
            pz = g.paddle_len + g.pivot_gap
            tipx, tipz = g.paddle_len * math.sin(q), pz - g.paddle_len * math.cos(q)
            arm.set_data([0, tipx], [pz, tipz])
            arm.set_linewidth(max(2, g.paddle_w * 200))
        else:
            paddle.set_data([px - g.paddle_len / 2, px + g.paddle_len / 2], [0, 0])
            arm.set_data([px - g.paddle_len / 2, px + g.paddle_len / 2], [0.02, 0.02])
            arm.set_linewidth(20)
        fball.center = (b.x, br)
        est.set_data([sim.ctl.x_est], [0])
        tgt.set_data([sim.target], [0.03])
        tr = list(sim.trail)
        trail.set_data([x for x, _ in tr], [y for _, y in tr])
        if sim.pred.incoming and sim.shot:
            path.set_data([b.x, sim.pred.x_cross], [b.y, 0])
        else:
            path.set_data([], [])
        aimln.set_data(([b.x, st["aim"]], [b.y, 0]) if not sim.shot else ([], []))
        n = sim.saves + sim.goals
        txt.set_text(
            f"{st['msg']:<8}\n\nsaves {sim.saves}   goals {sim.goals}\n"
            f"save rate {100 * sim.saves / n if n else 0:.0f}%\n\n"
            f"shot speed {st['speed']:.2f} m/s\naim x {st['aim']*100:+.0f} cm\n"
            f"auto shooter {'ON' if st['auto'] else 'off'}\n\n"
            f"{p.goalie.geometry}, {p.motor.rpm:.0f} RPM\n{p.camera.fps:.0f} fps, "
            f"{p.camera.latency*1000:.0f} ms\nduty {sim.duty:+.2f}\n\n"
            "space shoot  <- -> aim\nup/down speed  a auto\nclick: move ball\nr reset  q quit")
        return ()

    if headless:
        # scripted: ball from the left, 1.5 m/s at the right corner; stop just before the line
        sim.place(-0.12)
        st["speed"], st["aim"] = max(a.speed, 1.5), 0.075
        for _ in range(int(0.5 / FRAME)):
            update(0)
        shoot()
        frames = a.frames or 10 ** 6
        for i in range(frames):
            update(0)
            if not a.frames and sim.ball.y < 0.06:
                break
        os.makedirs(OUT, exist_ok=True)
        out = os.path.join(OUT, "live_frame.png")
        fig.savefig(out, dpi=100)
        print(f"headless live: rendered {i + 1} frames after the shot, saved {out}")
        return 0
    anim = FuncAnimation(fig, update, interval=FRAME * 1000, blit=False, cache_frame_data=False)  # noqa: F841
    plt.show()
    return 0


def main(argv=None) -> int:
    a = parse_args(argv)
    if a.live:
        return live(a)
    if a.feasibility:
        return feasibility(a)
    return scenario(a)


if __name__ == "__main__":
    sys.exit(main())
