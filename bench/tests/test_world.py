#!/usr/bin/env python3
"""World sim + run_sim end to end (Mac only, SIM: real tars_motor/goto host builds, simulated sensors).

    python3 bench/tests/test_world.py [--only name,name] [-v]

Starts bench/run_sim.py --free-ports (stub car unless bench/car/launch.py exists), then:
  goto       GOTO trash can in the default room (bin behind a partition, remembered): arrives,
             never through the wall
  moved      the bin is moved live (world control); GOTO again finds it at the new spot
  dropbox    room demo_detour loaded through the bench (/api/world/load -> goto restarted with
             memory); a box is placed on the route mid-GOTO (/api/world/ctl): the car never
             enters it, and goto ends arrived or failed (not stuck)
  resets     🚗 car_to_start keeps a placed box, a drawn wall and goto (same connection, map pose at the
             start); 🗺 reset_map restores the room, restarts goto, keeps the car where it is and goto's
             map and memory poses agree with it
  silent     guard goes silent: tars_motor reports guard_stale and doesn't drive
  person     a person steps in front: world guard BLOCKED and tars_motor guard_blocked
and finally Ctrl-C (SIGINT) to run_sim: every child gone.
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
sys.path.insert(0, os.path.join(BENCH, "world"))
import common as C  # noqa: E402

VERBOSE = "-v" in sys.argv
passed = failed = 0


def check(ok, what):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


class Line:
    def __init__(self, port, hello=None):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=5)
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


def kv(r):
    return dict(x.split("=", 1) for x in r.split()[1:] if "=" in x)


class Sim:
    def __init__(self):
        scratch = os.path.join(REPO, ".scratch")
        os.makedirs(scratch, exist_ok=True)
        self.pf = os.path.join(scratch, f"test_world_{os.getpid()}.json")
        self.p = subprocess.Popen([sys.executable, os.path.join(BENCH, "run_sim.py"), "--free-ports", "--no-voice",
                                   "--ports-file", self.pf], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.out = []
        threading.Thread(target=self._read, daemon=True).start()
        end = time.monotonic() + 120
        while not os.path.exists(self.pf):
            if self.p.poll() is not None or time.monotonic() > end:
                raise RuntimeError("run_sim didn't come up:\n" + "".join(self.out[-30:]))
            time.sleep(0.2)
        info = json.load(open(self.pf))
        self.ports, self.pids, self.car = info["ports"], info["pids"], info["car"]
        self.world = Line(self.ports["world_ctl"])
        self.truths = []
        self.run = True
        self.wlock = threading.Lock()
        threading.Thread(target=self._poll, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            self.out.append(line)
            if VERBOSE:
                print("    | " + line, end="")

    def wctl(self, msg):
        with self.wlock:
            return json.loads(self.world.cmd(json.dumps(msg)))

    def state(self):
        return self.wctl({"op": "get_state"})["state"]

    def _poll(self):
        w = Line(self.ports["world_ctl"])
        while self.run:
            try:
                st = json.loads(w.cmd('{"op":"get_state"}'))["state"]
            except (OSError, ValueError):
                return
            if st.get("truth"):
                self.truths.append((time.monotonic(), st["truth"], st["guard"]["verdict"]))
            time.sleep(0.1)

    def http(self, method, path, body=None):
        c = http.client.HTTPConnection("127.0.0.1", self.ports["bench"], timeout=30)
        c.request(method, path, body=json.dumps(body) if body is not None else None)
        r = c.getresponse()
        return r.status, json.loads(r.read() or b"{}")

    def stop(self):
        self.run = False
        self.p.send_signal(signal.SIGINT)
        try:
            self.p.wait(20)
        except subprocess.TimeoutExpired:
            self.p.kill()
        time.sleep(0.5)
        alive = []
        for n, pid in self.pids.items():
            try:
                os.kill(pid, 0)
                alive.append(n)
            except ProcessLookupError:
                pass
            except PermissionError:
                alive.append(n)
        try:
            os.remove(self.pf)
        except OSError:
            pass
        return alive


def wait_goto(g: Line, timeout, hook=None):
    end = time.monotonic() + timeout
    st = kv(g.cmd("STATUS"))
    while time.monotonic() < end:
        st = kv(g.cmd("STATUS"))
        if hook:
            hook(st)
        if st.get("state") == "idle":
            return st
        time.sleep(0.2)
    return st


def sc_goto(sim):
    print("world: GOTO trash can in the default room (bin behind the partition, remembered)")
    g = Line(sim.ports["goto"])
    n0 = len(sim.truths)
    r = g.cmd("GOTO trash can")
    check(r.startswith("OK"), f"GOTO accepted ({r})")
    st = wait_goto(g, 180)
    tr = [t for _, t, _ in sim.truths[n0:]]
    last = tr[-1] if tr else {"x": 0, "y": 0}
    d = math.hypot(last["x"] - 2.4, last["y"])
    print(f"    goto: {st.get('result')} replans={st.get('replans')}, car at {last['x']:.2f},{last['y']:.2f} ({d:.2f} m from the bin)")
    check(st.get("result") == "arrived", f"arrived ({st.get('result')})")
    check(d < 0.95, f"stopped next to the bin ({d:.2f} m centre to centre)")
    through = [t for t in tr if 1.2 < t["x"] < 1.4 and abs(t["y"]) < 0.6]
    check(not through, "never drove through the partition")
    return g


def sc_moved(sim, g):
    print("world: the bin is moved live; GOTO finds it at the new spot")
    st = sim.state()["truth"]
    bx, by = 2.4, -1.3
    r = sim.wctl({"op": "update", "id": "bin", "x": bx, "y": by})
    check(r.get("ok"), f"world control moved the bin ({r})")
    room = sim.wctl({"op": "get_room"})["room"]
    b = next(o for o in room["objects"] if o["id"] == "bin")
    check((b["x"], b["y"]) == (bx, by), "room shows the bin at the new spot")
    r = g.cmd("GOTO trash can")
    st = wait_goto(g, 200)
    t = sim.state()["truth"]
    d = math.hypot(t["x"] - bx, t["y"] - by)
    mem = g.cmd("MEMORY")
    print(f"    goto: {st.get('result')}, {d:.2f} m from the new spot; memory: {mem[:160]}")
    check(st.get("result") == "arrived" and d < 1.0, f"found it at the new spot and docked ({st.get('result')}, {d:.2f} m)")


def in_box(x, y, box, margin):
    c, s = math.cos(-box.get("yaw", 0)), math.sin(-box.get("yaw", 0))
    dx, dy = x - box["x"], y - box["y"]
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return abs(lx) < box["w"] / 2 + margin and abs(ly) < box["d"] / 2 + margin


def sc_dropbox(sim):
    print("world: demo_detour, a box dropped on the route mid-GOTO")
    code, r = sim.http("POST", "/api/world/load", {"name": "demo_detour"})
    check(code == 200 and r.get("ok") and (r.get("goto_restart") or {}).get("ok"),
          f"bench loaded the room and restarted goto with its memory ({code}, {str(r)[:120]})")
    hw = os.environ.get("TEST_WORLD_GUARD_HALF_W")      # compare guard geometries (lidar_guard: 0.45)
    if hw:
        print(f"    guard_half_w = {hw} (TEST_WORLD_GUARD_HALF_W)")
        sim.wctl({"op": "set", "params": {"guard_half_w": float(hw)}})
    time.sleep(2.0)
    g = Line(sim.ports["goto"])
    mem = g.cmd("MEMORY")
    check("trash can#1" in mem, f"goto remembers the bin ({mem[:80]})")
    t0 = sim.state()["truth"]
    check(abs(t0["x"]) < 0.05 and abs(t0["y"]) < 0.05, f"car back at the start ({t0['x']:.2f}, {t0['y']:.2f})")
    n0 = len(sim.truths)
    g.cmd("GOTO trash can")
    placed = {}

    def hook(st):
        if placed or st.get("state") not in ("follow_path", "look", "approach"):
            return
        t = sim.truths[-1][1]
        if t["x"] > 0.4:
            box = {"label": "", "x": t["x"] + 0.95 * math.cos(t["th"]), "y": t["y"] + 0.95 * math.sin(t["th"]),
                   "w": 0.35, "d": 0.9, "h": 0.5, "yaw": t["th"]}
            rep = sim.http("POST", "/api/world/ctl", {"op": "add", "obj": box})[1]
            placed.update(box, id=rep.get("id"), at=(t["x"], t["y"]), ok=rep.get("ok"))
    st = wait_goto(g, 200, hook)
    check(placed.get("ok"), f"box placed through the bench mid-route ({placed.get('id')} at {placed.get('x', 0):.2f},{placed.get('y', 0):.2f})")
    tr = sim.truths[n0:]
    inside = [t for _, t, _ in tr if placed and in_box(t["x"], t["y"], placed, 0.0)]
    contact = any(t.get("contact") for _, t, _ in tr)
    blocked = sum(1 for _, _, v in tr if v == "BLOCKED")
    end = tr[-1][1] if tr else {"x": 0, "y": 0}
    d = math.hypot(end["x"] - 3.2, end["y"])
    cnt = sim.state()["counts"]
    print(f"    sim health: guard sends max gap {cnt.get('gat_max_gap_ms')} ms, sim-stale guard ticks "
          f"{cnt.get("sim_stale_ticks", 0)}, late sends (>=100 ms) {cnt.get("sim_guard_late", 0)}, LDR1 sent {cnt.get('ldr1')} dropped {cnt.get('ldr1_dropped', 0)}")
    print(f"    goto: {st.get('result')} replans={st.get('replans')} guard BLOCKED samples={blocked} contact={contact} "
          f"end {end['x']:.2f},{end['y']:.2f} ({d:.2f} m from the bin)")
    check(not inside, "the car never entered the box")
    check(not contact, "the car never touched the box (no wall contact)")
    check(blocked > 0 or int(st.get("replans", "0") or 0) > 0, "it reacted: guard BLOCKED and/or goto re-planned")
    check(st.get("state") == "idle" and (st.get("result") == "arrived" or str(st.get("result", "")).startswith("failed")),
          f"goto ended arrived or failed, not stuck ({st.get('state')}, {st.get('result')})")
    if st.get("result") == "arrived":
        check(d < 0.95, f"arrived at the bin after the detour ({d:.2f} m)")


def sc_guard(sim):
    print("world: guard faults (empty room)")
    code, r = sim.http("POST", "/api/world/load", {"name": "empty"})
    check(code == 200 and r.get("ok"), f"empty room loaded ({code}, {str(r)[:100]})")
    time.sleep(1.8)          # the load sent S (from run_sim): other clients' D is held 1.5 s
    m = Line(sim.ports["motor"], "WHO test")
    end = time.monotonic() + 5
    while "guard_reason=clear" not in m.cmd("STATUS") and time.monotonic() < end:
        time.sleep(0.05)
    r = m.cmd("D 0.4 0.4")
    check(r.startswith("OK"), f"D accepted with the guard CLEAR ({r})")
    m.cmd("S")
    time.sleep(1.7)          # S holds other clients' D for 1.5 s; ours is the only client driving
    sim.wctl({"op": "fault", "name": "guard_silent"})
    time.sleep(0.4)
    st = kv(m.cmd("STATUS"))
    r = m.cmd("D 0.4 0.4")
    time.sleep(0.1)
    st2 = m.cmd("STATUS")
    out = [float(v) for v in st2.split()[1:3]]
    print(f"    silent guard: reason={st.get('guard_reason')} permit={st.get('guard_permit')} D -> {r} out={out}")
    check(st.get("guard_reason") in ("guard_stale", "guard_missing") and st.get("guard_permit") == "0",
          f"tars_motor sees the guard go stale ({st.get('guard_reason')})")
    check(not r.startswith("OK") or out == [0.0, 0.0], f"and refuses to drive forward ({r}, out={out})")
    m.cmd("S")
    sim.wctl({"op": "fault", "name": "clear"})
    time.sleep(0.5)
    r = sim.wctl({"op": "fault", "name": "person"})
    check(r.get("ok"), f"person steps in front ({r})")
    time.sleep(0.5)
    ws = sim.state()["guard"]
    st = kv(m.cmd("STATUS"))
    room = sim.wctl({"op": "get_room"})["room"]
    print(f"    person: world guard {ws['verdict']} nearest {ws.get('nearest')}; tars_motor {st.get('guard_reason')}")
    check(ws["verdict"] == "BLOCKED", f"world guard BLOCKED ({ws['verdict']}, nearest {ws.get('nearest')} m)")
    check(st.get("guard_reason") == "guard_blocked", f"tars_motor guard_blocked ({st.get('guard_reason')})")
    check(any(o["label"] == "person" for o in room["objects"]), "the person is in the room (sensors + 3D map see it)")
    r = m.cmd("D 0.4 0.4")
    check(not r.startswith("OK") or kv(m.cmd("STATUS")).get("guard_permit") == "0", f"no forward drive into the person ({r})")
    m.cmd("S")
    sim.wctl({"op": "fault", "name": "clear"})


def _nav_pose(g: Line):
    """goto NAV -> (source, x_ark, z_ark)"""
    r = g.cmd("NAV")
    src, x, z, _ = kv(r).get("pose", "none,0,0,0").split(",")
    return src, float(x), float(z)


def _mem_pose(g: Line):
    """goto MEMORY -> its memory-frame pose (motor mm, mm, deg)"""
    r = g.cmd("MEMORY")
    v = r.split("pose=", 1)[1].split(";")[0].split()[0].split(",")
    return float(v[0]), float(v[1]), float(v[2])


def _wait_truth(sim, x, y, timeout=4.0):
    end = time.monotonic() + timeout
    t = None
    while time.monotonic() < end:
        t = sim.state().get("truth")
        if t and math.hypot(t["x"] - x, t["y"] - y) < 0.05:
            return t
        time.sleep(0.1)
    return t


def _wait_nav(g: Line, xa, za, timeout=6.0):
    end = time.monotonic() + timeout
    p = None
    while time.monotonic() < end:
        try:
            p = _nav_pose(g)
        except (OSError, ValueError, IndexError):
            p = None
        if p and p[0] == "arkit" and math.hypot(p[1] - xa, p[2] - za) < 0.1:
            return p
        time.sleep(0.2)
    return p


def sc_resets(sim):
    print("world: 🚗 Car to start keeps the room and goto; 🗺 Reset map restores the room, car stays put")
    code, r = sim.http("POST", "/api/world/load", {"name": "empty"})
    check(code == 200 and r.get("ok"), f"empty room loaded ({code})")
    time.sleep(1.5)
    room0 = sim.wctl({"op": "get_room"})["room"]
    start = room0["start"]
    code, a = sim.http("POST", "/api/world/ctl", {"op": "add", "obj": {"type": "box", "x": -1.5, "y": 1.5}})
    code2, w = sim.http("POST", "/api/world/ctl", {"op": "add_wall", "seg": [-2.0, -1.5, -1.0, -1.5]})
    check(a.get("ok") and w.get("ok"), f"placed a box ({a.get('id')}) and drew a wall ({w.get('id')})")
    sim.wctl({"op": "teleport", "x": 1.0, "y": -0.5, "th": 0.6})
    t = _wait_truth(sim, 1.0, -0.5)
    check(t and math.hypot(t["x"] - 1.0, t["y"] + 0.5) < 0.05, "car moved away from the start (1.0, -0.5)")
    g = Line(sim.ports["goto"])           # stays open across car-to-start: a goto restart would drop it
    _wait_nav(g, -0.5, 1.0)

    # ---- 🚗 Car to start
    stops0 = int(kv(Line(sim.ports["motor"]).cmd("STATUS")).get("stops", "0") or 0)
    code, r = sim.http("POST", "/api/world/car_to_start", {})
    check(code == 200 and r.get("ok"), f"car_to_start accepted ({code}, {str(r)[:80]})")
    check((r.get("stopped") or {}).get("S") and (r.get("stopped") or {}).get("CANCEL"), f"S + CANCEL sent first ({r.get('stopped')})")
    t = _wait_truth(sim, start["x"], start["y"])
    check(t and math.hypot(t["x"] - start["x"], t["y"] - start["y"]) < 0.05,
          f"car back at the start ({t and round(t['x'], 2)}, {t and round(t['y'], 2)})")
    room = sim.wctl({"op": "get_room"})["room"]
    check(any(o["id"] == a.get("id") for o in room["objects"]), "the placed box is still there")
    check([-2.0, -1.5, -1.0, -1.5] in room["walls"], "the drawn wall is still there")
    stops1 = int(kv(Line(sim.ports["motor"]).cmd("STATUS")).get("stops", "0") or 0)
    check(stops1 > stops0, f"tars_motor got the S (stops {stops0} -> {stops1})")
    xa, za, _ = C.world_to_ark(start, start["x"], start["y"])
    p = _wait_nav(g, xa, za)
    check(p and p[0] == "arkit" and math.hypot(p[1] - xa, p[2] - za) < 0.1,
          f"goto NOT restarted (same connection) and its map pose is at the start ({p})")
    try:
        mp = _mem_pose(g)
        print(f"  info goto memory-frame pose after car-to-start: {mp} (dead reckoning; doesn't see the jump)")
    except (OSError, ValueError, IndexError) as e:
        print(f"  info goto MEMORY: {e}")

    # ---- 🗺 Reset map
    sim.wctl({"op": "teleport", "x": 1.2, "y": 0.8, "th": -0.4})
    t = _wait_truth(sim, 1.2, 0.8)
    code, r = sim.http("POST", "/api/world/reset_map", {})
    check(code == 200 and r.get("ok") and (r.get("goto_restart") or {}).get("ok"),
          f"reset_map accepted, goto restarted ({code}, {str(r.get('goto_restart'))[:80]})")
    room = sim.wctl({"op": "get_room"})["room"]
    check(room["objects"] == room0["objects"] and room["walls"] == room0["walls"],
          f"room back as loaded ({len(room['objects'])} objects, {len(room['walls'])} walls)")
    time.sleep(1.0)
    t = sim.state().get("truth")
    check(t and math.hypot(t["x"] - 1.2, t["y"] - 0.8) < 0.05, f"car stayed where it was ({t and round(t['x'], 2)}, {t and round(t['y'], 2)})")
    g2 = Line(sim.ports["goto"])
    xa, za, _ = C.world_to_ark(start, 1.2, 0.8)
    p = _wait_nav(g2, xa, za)
    check(p and p[0] == "arkit" and math.hypot(p[1] - xa, p[2] - za) < 0.1, f"new goto's map pose is at the car ({p}, want {xa:.2f},{za:.2f})")
    mx, my, mh = C.world_to_motor(start, 1.2, 0.8, -0.4)
    mp = _mem_pose(g2)
    check(math.hypot(mp[0] - mx, mp[1] - my) < 60 and abs(((mp[2] - mh) + 180) % 360 - 180) < 5,
          f"new goto's memory frame agrees with the car ({mp} vs {mx:.0f},{my:.0f},{mh:.1f})")
    grid = g2.cmd("NAV")
    print(f"  info NAV after reset_map: {grid[:120]}")


def main():
    only = sys.argv[sys.argv.index("--only") + 1].split(",") if "--only" in sys.argv else None
    want = lambda n: only is None or n in only  # noqa: E731
    t0 = time.monotonic()
    try:
        load = os.getloadavg()[0]
    except OSError:
        load = -1
    sim = Sim()
    print(f"run_sim up: car={sim.car}, bench :{sim.ports['bench']}, load {load:.1f}")
    try:
        g = None
        if want("goto") or want("moved"):
            g = sc_goto(sim)
        if want("moved"):
            sc_moved(sim, g)
        if want("dropbox"):
            sc_dropbox(sim)
        if want("resets"):
            sc_resets(sim)
        if want("silent") or want("person"):
            sc_guard(sim)
    except Exception:
        import traceback
        traceback.print_exc()
        check(False, "a scenario crashed")
    finally:
        alive = sim.stop()
        tail = "".join(sim.out[-3:])
        check(not alive, f"Ctrl-C: every child stopped ({alive or 'none left'})")
        check("S, CANCEL" in "".join(sim.out), "run_sim sent S + CANCEL before stopping")
        if VERBOSE:
            print(tail)
    print(f"{passed}/{passed + failed} checks passed ({time.monotonic() - t0:.0f} s, load {os.getloadavg()[0]:.1f})")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
