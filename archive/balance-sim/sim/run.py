"""Run the balance simulator.

    python run.py                         # headless, NEMA17, pushes +/-0.25 N*s, saves out/nema17.png
    python run.py --motor tt              # the small yellow DC gearmotors
    python run.py --compare               # verdict: which motors can balance this robot?
    python run.py --compare --ablate      # + why the TT motors fail (torque vs backlash), ~1 min
    python run.py --live                  # animated window; arrow keys / a,d push it, r = reset
    python run.py --push 2:0.4 --push 5:-0.6 --duration 9
    python run.py --gains kp=80,kd=20 --backend py --fused --slip --no-outer
    python run.py --params my_robot.json  # override RobotParams fields

Gains: C defaults (balance_default_cfg) <- gains.json[motor] (from tune.py --save)
<- --gains. Exit code is 0 if the scenario passes.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

from params import DEG, MOTOR_PRESETS, RobotParams, Scenario, robot_for_motor
from simulate import Sim, fmt_metrics, max_push, run_scenario
from tune import load_gains

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")

COMPARE = ["nema17", "tt", "dc25", "pololu37d"]
PUSH_OK = 0.20   # N*s both ways = survives a real shove at the top of the slab


def parse_gains(s):
    if not s:
        return {}
    if os.path.exists(s):
        with open(s) as f:
            return json.load(f)
    return {k: float(v) for k, v in (kv.split("=") for kv in s.split(","))}


def build(a, motor=None):
    p = RobotParams.from_json(a.params)
    p = robot_for_motor(motor or a.motor, p)
    if a.fused:
        p.fused_imu = True
    if a.slip:
        p.wheel_slip = True
    if a.seed is not None:
        p.seed = a.seed
    gains = load_gains(motor or a.motor)
    if p.fused_imu:
        gains["use_fused_pitch"] = 1
    if a.no_outer:
        gains.update(speed_kp=0.0, speed_ki=0.0)
    gains.update(parse_gains(a.gains))
    sc = Scenario()
    if a.push:
        sc.pushes = [tuple(float(v) for v in x.split(":")) for x in a.push]
    if a.duration:
        sc.duration = a.duration
    return p, gains, sc


def plot(sim, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    L = sim.log
    t = L["t"]
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, [v / DEG for v in L["pitch"]], label="true pitch")
    ax[0].plot(t, [v / DEG for v in L["pitch_est"]], label="estimated", alpha=0.7)
    ax[0].plot(t, [v / DEG for v in L["pitch_target"]], label="target (outer loop)", ls="--")
    ax[0].set_ylabel("pitch [deg]")
    ax[1].plot(t, L["xd"], label="true speed")
    ax[1].plot(t, L["v_cmd"], label="wheel cmd", alpha=0.7)
    ax[1].set_ylabel("speed [m/s]")
    if sim.p.drive == "dc":
        ax[2].plot(t, L["duty"], label="PWM duty")
        ax[2].set_ylabel("duty")
    else:
        ax[2].plot(t, L["accel"], label="accel cmd")
        ax[2].set_ylabel("accel [m/s^2]")
    ax2b = ax[2].twinx()
    ax2b.plot(t, L["tau"], color="C3", alpha=0.4, label="wheel torque")
    ax2b.set_ylabel("torque [N*m]", color="C3")
    ax[3].plot(t, L["x"], label="position")
    ax[3].set_ylabel("x [m]")
    ax[3].set_xlabel("time [s]")
    for t0, t1, F in sim.forces:
        for a_ in ax:
            a_.axvspan(t0, t1 + 0.02, color="red" if F > 0 else "blue", alpha=0.15)
    if sim.fell_at:
        for a_ in ax:
            a_.axvline(sim.fell_at, color="k", ls=":")
    for a_ in ax:
        a_.grid(alpha=0.3)
        a_.legend(loc="upper right", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def headless(a):
    p, gains, sc = build(a)
    m, sim = run_scenario(p, gains, sc, a.backend)
    print(fmt_metrics(a.motor, m))
    mp, stands = max_push(p, gains, a.backend)
    print(f"  max recoverable push (both directions): {mp:.2f} N*s" if stands
          else "  cannot even stand still")
    path = a.out or os.path.join(OUT, f"{a.motor}.png")
    plot(sim, path, f"{a.motor}: {'PASS' if m['passed'] else 'FAIL'}, max push {mp:.2f} N*s")
    print(f"  plot: {path}")
    return 0 if m["passed"] else 1


def verdict(m, mp, stands):
    if not stands or m["fell"]:
        return "NO - falls over"
    if m["passed"] and mp >= PUSH_OK:
        return "YES - balances and survives pushes"
    return "MARGINAL - stands, but a light push knocks it over"


def compare(a):
    rows = []
    for motor in COMPARE:
        p, gains, sc = build(a, motor)
        m, sim = run_scenario(p, gains, sc, a.backend)
        mp, stands = max_push(p, gains, a.backend)
        rows.append((motor, m, mp, stands))
        plot(sim, os.path.join(OUT, f"{motor}.png"), f"{motor}: max push {mp:.2f} N*s")
    if a.ablate:
        # What limits the TT motors? Re-tune each variant (fair comparison), ~15 s each.
        from tune import search
        for label, over in [("tt, no backlash/deadband", dict(dc_backlash=0.0, dc_deadband=0.0,
                                                             dc_deadband_comp=0.0)),
                            ("tt, 3x torque (e.g. 12V/metal)", dict(dc_stall_torque=0.24)),
                            ("tt, both", dict(dc_backlash=0.0, dc_deadband=0.0,
                                              dc_deadband_comp=0.0, dc_stall_torque=0.24))]:
            g, _ = search("tt", over=over, n=150, push=0.15, center=load_gains("tt"),
                          log=lambda *_: None)
            p, _g, sc = build(a, "tt")
            p.update(over)
            m, _s = run_scenario(p, g, sc, a.backend)
            mp, stands = max_push(p, g, a.backend)
            rows.append((label, m, mp, stands))
    print(f"{'motor':<34}{'result':<9}{'max tilt':>9}{'max push':>10}  verdict")
    for label, m, mp, stands in rows:
        print(f"{label:<34}{('PASS' if m['passed'] else 'FAIL'):<9}"
              f"{m['max_tilt_deg']:>7.1f}deg{mp:>8.2f}Ns  {verdict(m, mp, stands)}")
    print()
    print("scenario: hold upright -> arm -> pushes", ", ".join(f"{i:+.2f} N*s" for _, i in Scenario().pushes),
          "at the top of the slab; 'max push' = largest impulse recovered in BOTH directions.")
    print("gains per motor from gains.json (tune.py --save), else C defaults.")
    by = {r[0]: r for r in rows}
    st, tt = by["nema17"], by["tt"]
    print()
    print("VERDICT for the motors you have (TT-style plastic gearmotors, no encoders):")
    print(f"  {verdict(tt[1], tt[2], tt[3])}  (max push {tt[2]:.2f} N*s vs {st[2]:.2f} N*s for NEMA17)")
    if not (tt[1]["passed"] and tt[2] >= PUSH_OK):
        print("  -> order motors: 2x NEMA17 + TMC2209 (recommended) or 2x Pololu 37D w/ encoders.")
    print(f"plots in {OUT}/")
    return 0


def live(a):
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle, Polygon
    import math
    import time

    p, gains, sc = build(a)
    sc.pushes = []   # you are the disturbance
    state = {"sim": Sim(p, gains, a.backend, sc), "wall": time.time()}
    WIN = 6.0

    fig = plt.figure(figsize=(11, 6))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.6])
    axr = fig.add_subplot(gs[:, 0])
    axp = fig.add_subplot(gs[0, 1]); axs = fig.add_subplot(gs[1, 1], sharex=axp)
    axc = fig.add_subplot(gs[2, 1], sharex=axp)
    axr.set_aspect("equal"); axr.set_ylim(-0.05, 0.6); axr.set_xlim(-0.4, 0.4)
    axr.axhline(0, color="k", lw=1)
    slab = Polygon([[0, 0]] * 4, closed=True, fc="#9aa0a6", ec="k")
    wheel = Circle((0, p.wheel_radius), p.wheel_radius, fc="#333", ec="k")
    spoke, = axr.plot([], [], color="w", lw=2)
    axr.add_patch(slab); axr.add_patch(wheel)
    txt = axr.text(0.02, 0.98, "", transform=axr.transAxes, va="top", family="monospace", fontsize=8)
    axr.set_title("TARS sim - arrows/a,d: push  r: reset  [ ]: push size")
    lp, = axp.plot([], [], label="pitch"); lt, = axp.plot([], [], "--", label="target")
    ls_, = axs.plot([], [], label="speed"); lv, = axs.plot([], [], alpha=0.7, label="wheel cmd")
    lc, = axc.plot([], [], label="duty" if p.drive == "dc" else "accel cmd")
    axp.set_ylabel("deg"); axs.set_ylabel("m/s"); axc.set_ylabel("duty" if p.drive == "dc" else "m/s^2")
    for ax_ in (axp, axs, axc):
        ax_.legend(loc="upper left", fontsize=8); ax_.grid(alpha=0.3)
    push = {"size": 0.25}

    def on_key(ev):
        s = state["sim"]
        if ev.key in ("right", "d"):
            s.add_push(+push["size"])
        elif ev.key in ("left", "a"):
            s.add_push(-push["size"])
        elif ev.key == "]":
            push["size"] = round(push["size"] + 0.05, 2)
        elif ev.key == "[":
            push["size"] = max(0.05, round(push["size"] - 0.05, 2))
        elif ev.key == "r":
            state["sim"] = Sim(p, gains, a.backend, Scenario(pushes=[]))
            state["wall"] = time.time()
    fig.canvas.mpl_connect("key_press_event", on_key)

    def frame(_):
        s = state["sim"]
        now = time.time()
        target = s.t + min(now - state["wall"], 0.1)   # real time, but never > 0.1 s per frame
        state["wall"] = now
        while s.t < target:
            s.step()
        x, th = s.s[0], s.s[2]
        r, H, W = p.wheel_radius, p.body_height, p.body_width
        ax_, ay_ = x, r
        c, sn = math.cos(th), math.sin(th)
        pts = [(-W / 2, -0.01), (W / 2, -0.01), (W / 2, H), (-W / 2, H)]
        slab.set_xy([(ax_ + px * c + py * sn, ay_ - px * sn + py * c) for px, py in pts])
        wheel.center = (ax_, ay_)
        phi = -x / r
        spoke.set_data([ax_, ax_ + r * math.sin(phi)], [ay_, ay_ + r * math.cos(phi)])
        axr.set_xlim(x - 0.4, x + 0.4)
        F = s.plant.F
        st = ["DISABLED", "WAIT_UPRIGHT", "RUNNING", "FALLEN"][s.cmd[5]]
        txt.set_text(f"t={s.t:5.1f}s  {st}\npitch {th/DEG:+5.1f} deg\nspeed {s.s[1]:+.2f} m/s\n"
                     f"push size {push['size']:.2f} N*s" + ("\nPUSH!" if F else "")
                     + ("\nFELL - press r" if s.fell_at else ""))
        L = s.log
        n = int(WIN / s.dt)
        tt = L["t"][-n:]
        lp.set_data(tt, [v / DEG for v in L["pitch"][-n:]])
        lt.set_data(tt, [v / DEG for v in L["pitch_target"][-n:]])
        ls_.set_data(tt, L["xd"][-n:]); lv.set_data(tt, L["v_cmd"][-n:])
        lc.set_data(tt, (L["duty"] if p.drive == "dc" else L["accel"])[-n:])
        if tt:
            axp.set_xlim(max(0, tt[-1] - WIN), max(WIN, tt[-1]))
            for ax_, ys in ((axp, lp.get_ydata()), (axs, ls_.get_ydata()), (axc, lc.get_ydata())):
                lo, hi = min(ys), max(ys)
                pad = 0.1 * (hi - lo) + 0.05
                ax_.set_ylim(lo - pad, hi + pad)
        return slab, wheel, spoke, txt, lp, lt, ls_, lv, lc

    _anim = FuncAnimation(fig, frame, interval=33, blit=False, cache_frame_data=False)
    print(f"live sim ({p.drive}, backend {state['sim'].ctrl.backend}). Close the window to quit.")
    plt.tight_layout()
    plt.show()
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--motor", default="nema17", choices=sorted(MOTOR_PRESETS))
    ap.add_argument("--compare", action="store_true", help="motor verdict table")
    ap.add_argument("--ablate", action="store_true",
                    help="with --compare: re-tune TT variants to see what limits them")
    ap.add_argument("--live", action="store_true", help="interactive window")
    ap.add_argument("--backend", default="auto", choices=["auto", "c", "py"])
    ap.add_argument("--params", help="JSON file overriding RobotParams")
    ap.add_argument("--gains", help="k=v,k=v or a JSON file of balance_cfg_t fields")
    ap.add_argument("--push", action="append", help="time:impulse (N*s), repeatable")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--out", help="PNG path for the headless plot (default out/<motor>.png)")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--fused", action="store_true", help="BNO085-style fused-angle IMU")
    ap.add_argument("--slip", action="store_true", help="limit traction (stepper)")
    ap.add_argument("--no-outer", action="store_true", help="disable the speed loop")
    a = ap.parse_args()
    if a.live:
        return live(a)
    if a.compare:
        return compare(a)
    return headless(a)


if __name__ == "__main__":
    sys.exit(main())
