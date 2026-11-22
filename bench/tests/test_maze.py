#!/usr/bin/env python3
"""pocket_maze: GOTO trash can through the maze Bill drew live on pocket_mouth (bench/world/rooms/pocket_maze.json).
Mac only, SIM: real tars_motor/goto host builds, the virtual SM11, the world sim. Nothing here runs on the robot.

    python3 bench/tests/test_maze.py [--only fresh1,fresh4,switch] [--runs 3] [--foot N] [--baseline] [--room R]
                                     [--no-route-sticky] [-v]

  fresh1   fresh run_sim at 1x, GOTO trash can from the start spot
  fresh4   the same at 4x
  switch   run_sim at 1x with the bench; GOTO; after --switch-after s (wall) POST /api/world/speed 4
           (cancels the GOTO, restarts goto -T 4 with memory kept); then GOTO trash can again

Each run must end 'arrived' within 1.0 m of the bin. Reported per run: result, wall and sim time,
distance driven (car odometer), replans, blocked:map events (and whether the replan cap tripped),
heading fixes, guard refusals/aborts.

The maze's passages are 0.78-0.83 m wide. With goto.conf's default footprint (ceil(0.27 / 0.1) = 3
cells each side) they're closed once LiDAR has drawn the walls, so goto gives up 'unreachable'. The
runs therefore pass --goto-conf nav_foot_cells=<--foot, default 2> to run_sim (sim only).
--baseline runs with goto.conf's default instead and only reports (no checks).
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_speed as T  # noqa: E402  (Line, kv, http helpers)

BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
VERBOSE = "-v" in sys.argv
passed = failed = 0


def arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


ONLY = [x for x in arg("--only", "").split(",") if x]
RUNS = int(arg("--runs", "3"))
FOOT = int(arg("--foot", "2"))
BASELINE = "--baseline" in sys.argv
SWITCH_AFTER = float(arg("--switch-after", "25"))
ROOM = arg("--room", "pocket_maze")


def check(ok, what):
    global passed, failed
    if BASELINE:
        print("  info " + what, flush=True)
        return
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


class Sim(T.Sim):
    """test_speed.Sim with extra run_sim arguments"""

    def __init__(self, room, speed, bench=False, extra=()):
        scratch = os.path.join(REPO, ".scratch")
        os.makedirs(scratch, exist_ok=True)
        self.pf = os.path.join(scratch, f"test_maze_{os.getpid()}_{time.monotonic_ns()}.json")
        argv = [sys.executable, os.path.join(BENCH, "run_sim.py"), "--free-ports", "--no-voice", "--car", "virtual",
                "--room", room, "--speed", f"{speed:g}", "--ports-file", self.pf, *extra]
        if not bench:
            argv.append("--no-bench")
        self.p = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.out = []
        threading.Thread(target=self._read, daemon=True).start()
        end = time.monotonic() + 180
        while not os.path.exists(self.pf):
            if self.p.poll() is not None or time.monotonic() > end:
                raise RuntimeError("run_sim didn't come up:\n" + "".join(self.out[-30:]))
            time.sleep(0.2)
        info = json.load(open(self.pf))
        self.ports, self.pids, self.run_dir = info["ports"], info["pids"], info["run_dir"]
        self.truth = None
        self.lock = threading.Lock()
        self.run = True
        if not bench:
            import socket
            self.ts = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ts.bind(("127.0.0.1", self.ports["truth_copy"]))
            self.ts.settimeout(0.3)
            threading.Thread(target=self._truth, daemon=True).start()

    def truth_now(self):
        t = self.last()
        if t is not None:
            return t
        return self.car_ctl({"get": 1})["truth"]


NO_STICKY = "--no-route-sticky" in sys.argv   # A/B: goto's heading-aware / sticky routes off (as on the robot)


def extra_args():
    return ([] if BASELINE else ["--goto-conf", f"nav_foot_cells={FOOT}"]) + (["--no-route-sticky"] if NO_STICKY else [])


def wait_idle(sim, g, cap_wall):
    w0 = time.monotonic()
    st = {}
    while time.monotonic() - w0 < cap_wall:
        st = T.kv(g.cmd("STATUS"))
        if st.get("state") == "idle":
            break
        time.sleep(0.1)
    return st, time.monotonic() - w0


def summarise(sim, st, wall, t0, t1, bin_, log, speed):
    return {
        "speed": speed, "result": st.get("result"), "wall_s": wall,
        "sim_s": (t1.get("t", 0) - t0.get("t", 0)) / 1000.0 if "t" in t0 else None,
        "drove_m": (t1.get("odo", 0) - t0.get("odo", 0)) if "odo" in t0 else None,
        "bin_m": math.hypot(t1["x"] - bin_["x"], t1["y"] - bin_["y"]),
        "replans": int(st.get("replans", -1)), "blocked_map": log.count("blocked:map"),
        "replan_cap": "replan_cap" in log or "replan cap" in log, "heading_fixes": int(st.get("heading_fixes", -1)),
        "guard": int(st.get("guard_refusals", 0)) + int(st.get("guard_aborts", 0)),
        "unreachable": log.count("unreachable"), "timeout": "failed:timeout" in log,
        "routes": log.count("event: route "), "route_kept": log.count("route_kept"),
        "route_repair": log.count("route_repair"), "route_switch": log.count("route_switch"),
        "guard_stops": log.count("blocked:lidar_guard") + log.count("blocked:guard"), "run_dir": sim.run_dir,
        "sticky": "nav_sticky = 1" in open(os.path.join(sim.run_dir, "goto.conf")).read(),
    }


def fmt(r):
    s = f"{r['sim_s']:.0f}" if r["sim_s"] is not None else "?"
    d = f"{r['drove_m']:.1f}" if r["drove_m"] is not None else "?"
    return (f"{r['result']}: wall {r['wall_s']:.0f} s, sim {s} s, drove {d} m, ends {r['bin_m']:.2f} m from bin, "
            f"replans {r['replans']}, blocked:map {r['blocked_map']}{' (REPLAN CAP)' if r['replan_cap'] else ''}, "
            f"heading_fixes {r['heading_fixes']}, guard refusals+aborts {r['guard']}")


def bin_of(sim):
    rm = sim.wctl({"op": "get_room"})["room"]
    return next(o for o in rm["objects"] if o.get("label") == "trash can")


def fresh(speed):
    sim = Sim(ROOM, speed, extra=extra_args())
    try:
        g = T.Line(sim.ports["goto"])
        end = time.monotonic() + 15
        while sim.last() is None and time.monotonic() < end:
            time.sleep(0.05)
        time.sleep(1.5 / speed)
        bin_ = bin_of(sim)
        t0 = sim.last()
        r = g.cmd("GOTO trash can")
        st, wall = wait_idle(sim, g, 900 / speed + 60)
        time.sleep(0.3 / speed)
        out = summarise(sim, st, wall, t0, sim.last(), bin_, sim.goto_log(), speed)
        out["accepted"] = r.startswith("OK")
        g.close()
        return out
    finally:
        sim.stop()


def switch():
    """1x with the bench; GOTO; switch to 4x mid-run (cancels, restarts goto, memory kept); GOTO again"""
    sim = Sim(ROOM, 1.0, bench=True, extra=extra_args())
    try:
        g = T.Line(sim.ports["goto"])
        time.sleep(2.5)
        bin_ = bin_of(sim)
        t0 = sim.car_ctl({"get": 1})["truth"]
        r1 = g.cmd("GOTO trash can")
        time.sleep(SWITCH_AFTER)
        mid = T.kv(g.cmd("STATUS"))
        g.close()
        c, rr = sim.http("POST", "/api/world/speed", {"speed": 4})
        ok_switch = c == 200 and rr.get("ok")
        time.sleep(1.5)
        g = T.Line(sim.ports["goto"])
        r2 = g.cmd("GOTO trash can")
        st, wall = wait_idle(sim, g, 900 / 4 + 60)
        time.sleep(0.2)
        t1 = sim.car_ctl({"get": 1})["truth"]
        out = summarise(sim, st, wall, t0, t1, bin_, sim.goto_log(), 4.0)
        out.update(accepted=r1.startswith("OK") and r2.startswith("OK"), switched=ok_switch,
                   mid_state=mid.get("state"), goto_T4="T=4.0" in sim.goto_log())
        g.close()
        return out
    finally:
        sim.stop()


def main():
    t0 = time.monotonic()
    mode = "goto.conf default footprint (report only)" if BASELINE else f"--goto-conf nav_foot_cells={FOOT}"
    print(f"test_maze: {ROOM}, {RUNS} run(s) each, {mode}, routes {'plain (--no-route-sticky)' if NO_STICKY else 'heading-aware + sticky'}")
    scen = [("fresh1", lambda: fresh(1.0)), ("fresh4", lambda: fresh(4.0)), ("switch", switch)]
    for name, fn in scen:
        if ONLY and name not in ONLY:
            continue
        for k in range(RUNS):
            try:
                r = fn()
            except Exception as e:  # noqa: BLE001
                check(False, f"{name} run {k + 1}: crashed: {e}")
                continue
            print(f"    {name} run {k + 1}: {fmt(r)}", flush=True)
            print("    RESULT " + json.dumps({"scenario": name, "run": k + 1, **r}), flush=True)
            check(r["accepted"], f"{name} run {k + 1}: GOTO accepted")
            if name == "switch":
                check(r["switched"] and r["goto_T4"], f"switch run {k + 1}: speed switch to 4x took (goto -T 4)")
            check(r["result"] == "arrived" and r["bin_m"] <= 1.0,
                  f"{name} run {k + 1}: arrived at the bin ({r['result']}, {r['bin_m']:.2f} m)")
            check(not r["replan_cap"], f"{name} run {k + 1}: replan cap not hit ({r['blocked_map']} blocked:map)")
    print(f"\ntest_maze: {passed}/{passed + failed} passed in {time.monotonic() - t0:.0f} s")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
