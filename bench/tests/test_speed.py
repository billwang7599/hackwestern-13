#!/usr/bin/env python3
"""Demo sim speed (bench/run_sim.py --speed N) end to end. Mac only, SIM: real tars_motor/goto host
builds, the virtual SM11 (real firmware on a pty), the world sim. Nothing here runs on the robot.

    python3 bench/tests/test_speed.py [--only open_bin,pocket_mouth,live] [--speeds 1,4] [--runs 1] [-v]

  <room>   for each room (open_bin, pocket_mouth) the same GOTO trash can at every --speeds value,
           each in a fresh run_sim (--free-ports --no-voice --no-bench, the bench's 0.40 x 0.30 m car).
           Recorded per run: result, wall time and sim time (car truth clock) from GOTO to idle,
           distance driven (the car's odometer), final pose and distance to the bin, goto's
           replans / guard_refusals / guard_aborts / backtracks, and blocked:map events in goto.log.
           Checks, fast vs 1x: same result; stop distance to the bin within 0.2 m; sim clock >= 0.85 N x the wall clock; wall time >= 0.5 N x lower; drive distance within 40 % (+0.5 m); guard refusals + aborts within 2.
  live     run_sim --speed 1 with the bench: POST /api/world/speed {4} -> world, car and goto (-T 4)
           all at 4x, memory kept; a MOVE 1000 at 4x lands like at 1x in ~1/4 the wall time;
           S mid-MOVE stops the wheels within 150 ms of wall time; back to 1x.
"""
from __future__ import annotations

import http.client
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)

VERBOSE = "-v" in sys.argv
passed = failed = 0


def arg(name, default):
    if name in sys.argv:
        return sys.argv[sys.argv.index(name) + 1]
    return default


ONLY = [x for x in arg("--only", "").split(",") if x]
SPEEDS = [float(x) for x in arg("--speeds", "1,4").split(",")]
RUNS = int(arg("--runs", "1"))


def check(ok, what):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


class Line:
    def __init__(self, port, hello=None, timeout=5):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.f = self.s.makefile("rw", encoding="utf-8", newline="\n")
        if hello:
            self.cmd(hello)

    def cmd(self, line):
        self.f.write(line + "\n")
        self.f.flush()
        while True:
            r = self.f.readline().strip()
            if not r.startswith("EVENT"):
                return r

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


def kv(r):
    return dict(x.split("=", 1) for x in r.split()[1:] if "=" in x)


class Sim:
    """run_sim in the background; the car truth (50 Hz sim) on our own UDP port"""

    def __init__(self, room, speed, bench=False):
        scratch = os.path.join(REPO, ".scratch")
        os.makedirs(scratch, exist_ok=True)
        self.pf = os.path.join(scratch, f"test_speed_{os.getpid()}_{room}_{speed:g}.json")
        argv = [sys.executable, os.path.join(BENCH, "run_sim.py"), "--free-ports", "--no-voice", "--car", "virtual",
                "--room", room, "--speed", f"{speed:g}", "--ports-file", self.pf]
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
        # with --no-bench nobody listens on the truth copy: we do (the car's full truth incl. its odometer)
        if not bench:
            self.ts = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ts.bind(("127.0.0.1", self.ports["truth_copy"]))
            self.ts.settimeout(0.3)
            threading.Thread(target=self._truth, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            self.out.append(line)
            if VERBOSE:
                print("    | " + line, end="")

    def _truth(self):
        while self.run:
            try:
                t = json.loads(self.ts.recv(8192))
            except (OSError, ValueError):
                continue
            with self.lock:
                self.truth = t

    def last(self):
        with self.lock:
            return dict(self.truth) if self.truth else None

    def wctl(self, msg):
        w = Line(self.ports["world_ctl"])
        try:
            return json.loads(w.cmd(json.dumps(msg)))
        finally:
            w.close()

    def car_ctl(self, msg):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1)
        try:
            s.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.ports["car_ctl"]))
            return json.loads(s.recv(65536))
        finally:
            s.close()

    def http(self, method, path, body=None):
        c = http.client.HTTPConnection("127.0.0.1", self.ports["bench"], timeout=30)
        c.request(method, path, body=json.dumps(body) if body is not None else None)
        r = c.getresponse()
        return r.status, json.loads(r.read() or b"{}")

    def goto_log(self):
        try:
            return open(os.path.join(self.run_dir, "goto.log")).read()
        except OSError:
            return ""

    def stop(self):
        self.run = False
        self.p.send_signal(signal.SIGINT)
        try:
            self.p.wait(20)
        except subprocess.TimeoutExpired:
            self.p.kill()
        try:
            os.remove(self.pf)
        except OSError:
            pass


def one_goto(room, speed):
    sim = Sim(room, speed)
    try:
        g = Line(sim.ports["goto"])
        # the phone's map needs a moment of frames before the first plan (same in sim time at every speed)
        end = time.monotonic() + 15
        while sim.last() is None and time.monotonic() < end:
            time.sleep(0.05)
        time.sleep(1.5 / speed)
        rm = sim.wctl({"op": "get_room"})["room"]
        bin_ = next(o for o in rm["objects"] if o.get("label") == "trash can")
        t0 = sim.last()
        w0 = time.monotonic()
        r = g.cmd("GOTO trash can")
        st = {}
        while time.monotonic() - w0 < 400 / speed + 30:
            st = kv(g.cmd("STATUS"))
            if st.get("state") == "idle":
                break
            time.sleep(0.05)
        wall = time.monotonic() - w0
        time.sleep(0.3 / speed)
        t1 = sim.last()
        log = sim.goto_log()
        res = {
            "room": room, "speed": speed, "accepted": r.startswith("OK"), "result": st.get("result"),
            "wall_s": wall, "sim_s": (t1["t"] - t0["t"]) / 1000.0, "drove_m": t1["odo"] - t0["odo"],
            "x": t1["x"], "y": t1["y"], "bin_m": math.hypot(t1["x"] - bin_["x"], t1["y"] - bin_["y"]),
            "replans": int(st.get("replans", -1)), "guard_refusals": int(st.get("guard_refusals", -1)),
            "guard_aborts": int(st.get("guard_aborts", -1)), "backtracks": int(st.get("backtracks", -1)),
            "blocked_map": log.count("blocked:map"), "goto_T": f"T={speed:.1f}" in log,
            "car_speed": sim.car_ctl({"get": 1}).get("speed"), "car_slow_ms": sim.car_ctl({"get": 1}).get("slow_ms"),
            "world": sim.wctl({"op": "get_state"})["state"].get("counts", {}),
        }
        g.close()
        return res
    finally:
        sim.stop()


def fmt(r):
    return (f"{r['speed']:g}x {r['result']}: wall {r['wall_s']:.1f} s, sim {r['sim_s']:.1f} s, drove {r['drove_m']:.2f} m, "
            f"end ({r['x']:.2f},{r['y']:.2f}) {r['bin_m']:.2f} m from bin, replans {r['replans']}, "
            f"blocked:map {r['blocked_map']}, guard refused {r['guard_refusals']} aborted {r['guard_aborts']}, "
            f"backtracks {r['backtracks']}, car slow_ms {r['car_slow_ms']}")


def sc_room(room):
    print(f"speed: GOTO trash can in {room} at {', '.join(f'{s:g}x' for s in SPEEDS)} ({RUNS} run(s) each)")
    rows = {}
    for sp in SPEEDS:
        for k in range(RUNS):
            r = one_goto(room, sp)
            rows.setdefault(sp, []).append(r)
            print(f"    {room} run {k + 1}: {fmt(r)}", flush=True)
            w = r["world"]
            if sp != 1:
                print(f"      world sim: ldr1 {w.get('ldr1')} (dropped {w.get('ldr1_dropped', 0)}), "
                      f"GAT1 {w.get('gat')} max gap {w.get('gat_max_gap_ms')} ms, sim_guard_late {w.get('sim_guard_late', 0)}, "
                      f"sim_stale_ticks {w.get('sim_stale_ticks', 0)}")
            check(r["accepted"], f"{room} {sp:g}x: GOTO accepted")
            if sp != 1:
                check(r["goto_T"] and r["car_speed"] == sp, f"{room} {sp:g}x: goto -T {sp:g} and the car at {r['car_speed']}x")
    base = rows.get(1.0)
    if not base:
        return
    b = base[0]
    for sp in SPEEDS:
        if sp == 1:
            continue
        for r in rows[sp]:
            check(r["result"] == b["result"], f"{room} {sp:g}x: same result as 1x ({r['result']} vs {b['result']})")
            # the clock: sim seconds per wall second during the go-to (host stalls fall back to 1x)
            clock = r["sim_s"] / max(0.01, r["wall_s"])
            check(clock >= 0.85 * sp, f"{room} {sp:g}x: sim clock ran {clock:.2f}x the wall clock")
            # end to end: also depends on the route goto picked, which varies run to run (pocket_mouth)
            ratio = b["wall_s"] / max(0.01, r["wall_s"])
            check(ratio >= 0.5 * sp, f"{room} {sp:g}x: wall time {r['wall_s']:.1f} s vs {b['wall_s']:.1f} s ({ratio:.1f}x faster)")
            # where it docks: the distance to the bin (pocket_mouth may dock on either side of it,
            # at 1x too); the spot itself is reported
            d = math.hypot(r["x"] - b["x"], r["y"] - b["y"])
            check(abs(r["bin_m"] - b["bin_m"]) <= 0.2,
                  f"{room} {sp:g}x: stops {r['bin_m']:.2f} m from the bin vs {b['bin_m']:.2f} m at 1x (spots {d:.2f} m apart)")
            check(abs(r["drove_m"] - b["drove_m"]) <= 0.4 * b["drove_m"] + 0.5,
                  f"{room} {sp:g}x: drove {r['drove_m']:.2f} m vs {b['drove_m']:.2f} m")
            ga = r["guard_refusals"] + r["guard_aborts"]
            gb = b["guard_refusals"] + b["guard_aborts"]
            check(abs(ga - gb) <= 2, f"{room} {sp:g}x: guard refusals+aborts {ga} vs {gb}")


def wait(fn, t):
    end = time.monotonic() + t
    while time.monotonic() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.02)
    return None


def sc_live():
    print("speed: live switch 1x -> 4x through the bench, MOVE timing, S wins, back to 1x")
    sim = Sim("open_bin", 1.0, bench=True)
    try:
        info = sim.wctl({"op": "speed"})
        check(info.get("speed") == 1.0, f"world starts at 1x ({info})")
        c, r = sim.http("POST", "/api/world/speed", {"speed": 4})
        check(c == 200 and r.get("ok"), f"POST /api/world/speed 4 ({c} {str(r)[:160]})")
        check(sim.wctl({"op": "speed"}).get("speed") == 4.0, "world sim at 4x")
        check(sim.car_ctl({"get": 1}).get("speed") == 4, "virtual car at 4x")
        check("T=4.0" in sim.goto_log(), "goto restarted with -T 4")
        c, r = sim.http("GET", "/api/world/speed")
        check(c == 200 and r.get("speed") == 4.0, f"GET /api/world/speed says 4 ({r})")
        c, r = sim.http("POST", "/api/world/speed", {"speed": 99})
        check(c == 400 and not r.get("ok"), f"speed 99 refused ({c})")
        m = Line(sim.ports["motor"], "WHO test_speed")
        wait(lambda: "ready=1" in m.cmd("STATUS"), 10)
        time.sleep(1.6)          # run_sim's S holds other clients' D for 1.5 s
        car = lambda: sim.car_ctl({"get": 1})["truth"]     # noqa: E731
        a = car()
        w0 = time.monotonic()
        r = m.cmd("MOVE 1000")
        done = wait(lambda: (m.cmd("PING"), "move=done" in m.cmd("STATUS"))[1], 20)
        wall = time.monotonic() - w0
        b = car()
        d = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        print(f"    MOVE 1000 at 4x: {r}, {d * 1000:.0f} mm in {wall:.2f} s wall (1x takes ~6.7 s)")
        check(bool(done) and 850 < d * 1000 < 1050 and wall < 3.0, f"MOVE 1000 at 4x: {d * 1000:.0f} mm in {wall:.2f} s")
        # stop wins: S mid-MOVE
        r = m.cmd("MOVE 1000")
        time.sleep(0.25)
        w0 = time.monotonic()
        m.cmd("S")
        stopped = wait(lambda: abs(car()["v"]) < 0.01, 2)
        lat = time.monotonic() - w0
        check(bool(stopped) and lat < 0.15, f"S mid-MOVE at 4x: wheels stopped {1000 * lat:.0f} ms (wall) after S")
        m.close()
        c, r = sim.http("POST", "/api/world/speed", {"speed": 1})
        check(c == 200 and r.get("ok") and sim.car_ctl({"get": 1}).get("speed") == 1, "back to 1x")
    finally:
        sim.stop()


def main():
    t0 = time.monotonic()
    for room in ("open_bin", "pocket_mouth"):
        if not ONLY or room in ONLY:
            sc_room(room)
    if not ONLY or "live" in ONLY:
        sc_live()
    print(f"\ntest_speed: {passed}/{passed + failed} passed in {time.monotonic() - t0:.0f} s")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
