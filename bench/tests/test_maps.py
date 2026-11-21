#!/usr/bin/env python3
"""Map demo modes (bench/maps, plugins/maps.py), Mac only, SIMULATION.

    python3 bench/tests/test_maps.py [--unit] [-v]

unit (no stack): grid run-length code, occupied cells -> walls, goto MEMORY parsing, seed points,
    the frontier planner, the phone sim's splat surfaces through Andrew's validate_scene + Map
    (-> surface_splats), a phone3d recording -> prescan -> room.
end to end (bench/run_sim.py --free-ports: the virtual SM11 if built, real tars_motor + goto host
builds, the world sim, the bench with every plugin; prescans go to a temp dir):
  A1 fresh   default room reloaded with no memory: goto's map + memory empty
  A2 scan    one goto SCAN at the start (sim pose noise off from A1 to here): goto's map gains cells, every one
             in the sim LiDAR's line of sight from where it scanned (none behind the partition, none outside)
  A3 explore SCAN / frontier hops until goto remembers the trash can, then GOTO: arrived, near the bin
  A4 save    the result saved as a prescan (grid + objects + splats)
  B1 load    the prescan becomes the room (walls rebuilt from its cells, the bin where it was seen),
             car dropped somewhere else, goto restarted and its map seeded (counts match)
  B2 walls   the virtual car pushed hard at an imported wall stops at it (contact), it doesn't pass
  B3 goto    GOTO trash can from the dropped pose: arrived, near the bin
"""
from __future__ import annotations

import http.client
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
sys.path.insert(0, BENCH)
sys.path.insert(0, os.path.join(BENCH, "world"))
sys.path.insert(0, os.path.join(BENCH, "phone"))
import common as C  # noqa: E402
from maps import explore as E  # noqa: E402
from maps import prescan as PS  # noqa: E402

VERBOSE = "-v" in sys.argv
passed = failed = 0


def check(ok, what):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


# ------------------------------------------------------------------ unit
def unit():
    print("unit")
    g = bytearray(PS.SIZE * PS.SIZE)
    for i in range(len(g)):
        xa, za = PS.cell_center(i % PS.SIZE, i // PS.SIZE)
        if -1.5 < xa < 1.5 and -1.0 < za < 2.0:
            g[i] = PS.FREE
        if abs(za - 1.35) < 0.05 and -0.6 < xa < 0.6:
            g[i] = PS.OCC
    enc = PS.encode_rle(g)
    check(PS.decode_rle(enc["box"], enc["rle"]) == g, f"run-length code round trip ({len(enc['rle'])} chars)")
    rep = f"OK grid v=9 size=200 res=0.10 origin=-10.00,-10.00 box={','.join(map(str, enc['box']))} rle={enc['rle']}"
    check(PS.parse_nav_grid(rep) == g, "goto NAV GRID reply parsed")
    walls, dropped = PS.grid_to_walls(g)
    check(walls == [[1.35, -0.6, 1.35, 0.6]] and dropped == 0, f"a row of occupied cells -> one wall {walls}")
    mem = PS.parse_memory("OK n=2 pose=0,0,0.0; trash can#1 fresh x=2400 y=0 rel_bearing=0 rel_dist=2400 "
                          "est_dist=2400 age=3 seen=4 conf=0.81 map=0.01,2.39; chair#2 stale x=1 y=2 rel_bearing=0 "
                          "rel_dist=1 est_dist=1 age=3 seen=1 conf=0.50")
    check(len(mem) == 1 and mem[0]["label"] == "trash can" and mem[0]["z_ark"] == 2.39,
          "goto MEMORY: entries with a map position (DR-only ones skipped)")
    pts = PS.seed_points(g)
    check(len(pts) == g.count(1) + g.count(2) and all(p[1] in (0.0, 0.3) for p in pts), "one seed point per mapped cell")
    plan = E.plan_frontier(g, 0.0, 0.0)
    ok = plan is not None and all(abs(z - 1.35) > 0.3 or abs(x) > 0.6 for x, z in plan[1])
    check(ok, f"frontier planner: a reachable frontier, path clear of the wall ({plan and plan[0]})")
    # phone sim splats -> Andrew's Map -> surface_splats
    sys.path.insert(0, os.path.join(REPO, "iter2-autonomy", "dashboard"))
    import mapping
    import sim
    ph = sim.SimPhone()
    sc = ph.scene(1000.0, 0.0, 0.0, 0.0)
    try:
        mapping.validate_scene(sc)
        valid = True
    except ValueError as e:
        valid = str(e)
    m = mapping.Map(save_path=os.path.join(tempfile.gettempdir(), "tars-test-maps-map.json"), native=None)
    m.update(sc)
    snap = m.snapshot()
    check(valid is True and len(sc["surfaces"]) > 50, f"sim scene surfaces pass validate_scene ({len(sc['surfaces'])} rows)")
    check(len(snap["surface_splats"]) > 1000, f"Andrew's Map makes surface_splats from them ({len(m.surfaces)} patches)")
    colours = {tuple(r[6:9]) for r in sc["surfaces"]}
    check(len(colours) > 20, "splats are coloured by what was hit")
    rec = os.path.join(BENCH, "phone", "recordings", "sample-sim-room.jsonl")
    if os.path.exists(rec):
        p = PS.from_recording(rec, "unit_rec")
        room, info = PS.to_room(p)
        check(info["free"] > 100 and info["walls"] > 3 and len(json.dumps(room)) < 9000,
              f"recording -> prescan -> room ({info}, {len(json.dumps(room))} bytes)")
        check(True, f"(that sample predates sim splats: {len(p['surfaces'])} surfaces)")
    # a fresh recording (the bin in sim.DEFAULT_ROOM is behind a wall from here: only the chair is seen; header line + {"t", "scene"} lines, the phone3d recorder's format) with splats
    tmp = os.path.join(tempfile.mkdtemp(prefix="tars-test-maps-rec-"), "spin.jsonl")
    ph2 = sim.SimPhone(session="unit-spin")
    with open(tmp, "w") as f:
        f.write(json.dumps({"format": "unit test"}) + "\n")
        for k in range(36):
            f.write(json.dumps({"t": k * 0.1, "scene": ph2.scene(1000.0 + k * 0.1, 0.2, 0.0, k * math.pi / 18)}) + "\n")
    p2 = PS.from_recording(tmp, "unit_spin")
    room2, info2 = PS.to_room(p2)
    check(len(p2["surfaces"]) > 300 and info2["occ"] > 50 and [o["label"] for o in p2["objects"]] == ["chair"],
          f"spin recording -> prescan with splats ({len(p2['surfaces'])}), cells {info2}, objects "
          f"{[o['label'] for o in p2['objects']]}")
    shutil.rmtree(os.path.dirname(tmp), ignore_errors=True)


# ------------------------------------------------------------------ the stack
def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Stack:
    def __init__(self, prescans: str):
        self.pf = os.path.join(prescans, "ports.json")
        env = dict(os.environ, BENCH_PRESCANS=prescans, TARS_BENCH_PHONE_ROOM_PORT=str(free_port()),
                   TARS_BENCH_PHONE_SCENE_PORT=str(free_port()))
        self.p = subprocess.Popen([sys.executable, os.path.join(BENCH, "run_sim.py"), "--free-ports", "--no-voice",
                                   "--ports-file", self.pf], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  text=True, env=env)
        self.out = []
        threading.Thread(target=self._read, daemon=True).start()
        end = time.monotonic() + 180
        while not os.path.exists(self.pf):
            if self.p.poll() is not None or time.monotonic() > end:
                raise RuntimeError("run_sim didn't come up:\n" + "".join(self.out[-30:]))
            time.sleep(0.2)
        info = json.load(open(self.pf))
        self.P, self.car, self.run_dir = info["ports"], info["car"], info["run_dir"]
        self._wait_bench()

    def _read(self):
        for line in self.p.stdout:
            self.out.append(line)
            if VERBOSE:
                print("    | " + line, end="")

    def _wait_bench(self):
        end = time.monotonic() + 20
        while time.monotonic() < end:
            try:
                if self.http("GET", "/api/maps/info")[0] == 200 and self.goto("PING").startswith("OK"):
                    return
            except OSError:
                pass
            time.sleep(0.3)

    def http(self, method, path, body=None, timeout=90):
        c = http.client.HTTPConnection("127.0.0.1", self.P["bench"], timeout=timeout)
        c.request(method, path, body=json.dumps(body) if body is not None else None)
        r = c.getresponse()
        return r.status, json.loads(r.read() or b"{}")

    def line(self, port, msg):
        with socket.create_connection(("127.0.0.1", port), timeout=5) as s:
            f = s.makefile("rw", encoding="utf-8", newline="\n")
            f.write(msg + "\n")
            f.flush()
            return f.readline().strip()

    def goto(self, msg):
        return self.line(self.P["goto"], msg)

    def world(self, msg: dict):
        return json.loads(self.line(self.P["world_ctl"], json.dumps(msg)))

    def truth(self):
        return self.world({"op": "get_state"})["state"]["truth"]

    def goto_kv(self):
        return dict(t.split("=", 1) for t in self.goto("STATUS").split()[1:] if "=" in t)

    def wait_goto_idle(self, timeout):
        end = time.monotonic() + timeout
        time.sleep(1.0)
        while time.monotonic() < end:
            kv = self.goto_kv()
            if kv.get("state") == "idle":
                return kv
            time.sleep(0.5)
        return self.goto_kv()

    def stop(self):
        try:
            self.p.send_signal(2)
            self.p.wait(25)
        except subprocess.TimeoutExpired:
            self.p.kill()


def nav_counts(st):
    p = E.parse_nav(st.goto("NAV"))
    return {"free": p[4], "occ": p[5]} if p else None


def los_violations(g, room, origins):
    """known cells that NO scan origin could see (a wall in between); (bad, total, behind, outside)"""
    scene = C.Scene(room)
    bad = total = behind = outside = 0
    bad_cells = []
    for i, v in enumerate(g):
        if not v:
            continue
        total += 1
        xa, za = PS.cell_center(i % PS.SIZE, i // PS.SIZE)
        x, y, _ = C.ark_to_world(room["start"], xa, za)
        if 1.5 < x < 3.5 and abs(y) < 0.45:
            behind += 1
        if not (-1.15 < x < 5.15 and -2.15 < y < 2.15):
            outside += 1
        seen = False
        # any part of the 0.1 m cell (centre or a point near a corner): cells on a shadow's edge are
        # partly visible, and the phone's floor hits land in that part
        for ox, oy in origins:
            for px, py in ((x, y), (x - .045, y - .045), (x + .045, y - .045), (x - .045, y + .045), (x + .045, y + .045)):
                d = math.hypot(px - ox, py - oy)
                t, _, _ = scene.ray(ox, oy, math.atan2(py - oy, px - ox), d)
                if t >= d - 0.12:
                    seen = True
                    break
            if seen:
                break
        bad += not seen
        if not seen:
            t, hid, _ = scene.ray(origins[0][0], origins[0][1], math.atan2(y - origins[0][1], x - origins[0][0]), 9)
            bad_cells.append((round(x, 2), round(y, 2), "occ" if v == 2 else "free", hid))
    return bad, total, behind, outside, bad_cells


def e2e():
    tmp = tempfile.mkdtemp(prefix="tars-test-maps-")
    st = Stack(tmp)
    print(f"end to end: car = {st.car} (logs {st.run_dir})")
    try:
        # A1 (sim pose noise off until A2's map is checked: the sim's noisy ARKit pose (1 cm, 0.5 deg sigma)
        # smears points a few cm into shadows, as on the robot; the line-of-sight check is exact)
        defaults = st.world({"op": "get_room"})["room"]["params"]
        st.world({"op": "set", "params": {"pose_noise_m": 0.0, "pose_noise_deg": 0.0}})
        s, r = st.http("POST", "/api/maps/fresh", {})
        n0 = nav_counts(st)
        mem = st.goto("MEMORY")
        check(s == 200 and r.get("ok") and r.get("nav_live"), f"A1 fresh: goto restarted, nav feed live ({r.get('goto_restart')})")
        check(mem.startswith("OK n=0"), f"A1 goto remembers nothing ({mem[:40]})")
        room = st.world({"op": "get_room"})["room"]
        # A2: one SCAN where the car stands
        t0 = st.truth()
        trail, done = [], threading.Event()

        def follow():                 # where the LiDAR looked from: the car's centre slips a few cm while it spins
            while not done.is_set():
                try:
                    t = st.truth()
                    trail.append((t["x"], t["y"]))
                except (OSError, ValueError, KeyError, TypeError):
                    pass
                time.sleep(0.1)
        ft = threading.Thread(target=follow, daemon=True)
        ft.start()
        s, r = st.http("POST", "/api/maps/scan", {})
        kv = st.wait_goto_idle(150)
        done.set()
        ft.join()
        t1 = st.truth()
        g = PS.parse_nav_grid(st.goto("NAV GRID"))
        st.world({"op": "set", "params": {k: defaults[k] for k in ("pose_noise_m", "pose_noise_deg")}})
        cnt = PS.counts(g)
        moved = math.hypot(t1["x"] - t0["x"], t1["y"] - t0["y"])
        check(r.get("ok") and str(kv.get("result", "")).startswith("scanned"), f"A2 SCAN ran: {kv.get('result')}")
        check(cnt["free"] > 3 * max(1, n0["free"]) and cnt["occ"] > 20,
              f"A2 the scan filled goto's map: free {n0['free']} -> {cnt['free']}, occ {n0['occ']} -> {cnt['occ']}")
        spread = max(math.hypot(x - t0["x"], y - t0["y"]) for x, y in trail) if trail else 0.0
        bad, total, behind, outside, bad_cells = los_violations(g, room, [(t0["x"], t0["y"])] + trail[::2])
        check(behind == 0 and outside == 0, f"A2 nothing mapped behind the partition ({behind}) or outside the room ({outside})")
        check(bad == 0, f"A2 every mapped cell is in line of sight of the scan ({bad} of {total} not; the car's centre wandered {spread:.2f} m while spinning) {bad_cells[:8]}")
        # A3: explore until the bin is remembered, then GOTO
        s, r = st.http("POST", "/api/maps/explore", {"target": "trash can", "max_hops": 6, "then_goto": True})
        end = time.monotonic() + 480
        ex = {}
        while time.monotonic() < end:
            ex = st.http("GET", "/api/maps/info")[1]["explore"] or {}
            if not ex.get("running", True):
                break
            time.sleep(1)
        check(str(ex.get("result", "")).startswith("found"),
              f"A3 explore found the trash can: {ex.get('result')} ({ex.get('hops')} hops, {ex.get('scans')} scans)")
        kv = st.wait_goto_idle(180)
        tr = st.truth()
        b = next(o for o in room["objects"] if o["label"] == "trash can")
        d = math.hypot(tr["x"] - b["x"], tr["y"] - b["y"])
        check(kv.get("result") == "arrived" and d < 1.1, f"A3 GOTO trash can on the map it built: {kv.get('result')}, {d:.2f} m from the bin")
        # A4
        s, r = st.http("POST", "/api/maps/save_prescan", {"name": "test_scan"})
        check(s == 200 and r.get("ok") and "trash can" in r.get("objects", []) and r.get("surfaces", 0) > 100,
              f"A4 prescan saved ({ {k: r.get(k) for k in ('free', 'occ', 'objects', 'surfaces', 'bytes')} })")
        p = PS.load_prescan(os.path.join(tmp, "test_scan.json"))
        # B1: load it, drop the car somewhere else
        pg = PS.grid_from_prescan(p)
        bin_ark = next(o for o in p["objects"] if o["label"] == "trash can")
        blk = E._blocked(pg, 0.4)
        drop = None
        best = -1
        for i, v in enumerate(pg):
            if v != PS.FREE or blk[i]:
                continue
            xa, za = PS.cell_center(i % PS.SIZE, i // PS.SIZE)
            x, y = PS.ark_to_world(xa, za)
            db = math.hypot(xa - bin_ark["x_ark"], za - bin_ark["z_ark"])
            # the main room (x < 3.4): the 0.8 m doorway to the second room is narrower than goto's
            # planner allows (0.35 m robot radius, observed free floor), so goto can't route out of there
            if x > 3.2 or db < 1.2 or math.hypot(x - tr["x"], y - tr["y"]) < 1.0:
                continue
            score = math.hypot(x, y) - 0.5 * db
            if score > best:
                best, drop = score, {"x": round(x, 2), "y": round(y, 2), "th": round(math.atan2(-y, -x), 2)}
        s, r = st.http("POST", "/api/maps/load_prescan", {"name": "test_scan", "drop": drop}, timeout=120)
        room2 = st.world({"op": "get_room"})["room"]
        n = nav_counts(st)
        want = PS.counts(pg)
        check(s == 200 and r.get("ok"), f"B1 prescan loaded, car dropped at {drop}: seed {r.get('seed')}")
        check(room2["name"].startswith("prescan_") and len(room2["walls"]) > 5 and
              any(o["label"] == "trash can" for o in room2["objects"]),
              f"B1 the sim room is rebuilt from the scan: {len(room2['walls'])} walls, objects "
              f"{[o['label'] for o in room2['objects']]}")
        check(n and n["free"] >= 0.9 * want["free"] and n["occ"] >= 0.9 * want["occ"],
              f"B1 goto's map seeded before the car moves: {n} (prescan {want})")
        tr = st.truth()
        check(math.hypot(tr["x"] - drop["x"], tr["y"] - drop["y"]) < 0.1, "B1 the car is at the drop pose")
        # B2: push the car at an imported wall (the longest one), 0.35 m away, side on
        w = max(room2["walls"], key=lambda w: math.hypot(w[2] - w[0], w[3] - w[1]))
        mx, my = (w[0] + w[2]) / 2, (w[1] + w[3]) / 2
        ex_, ey_ = w[2] - w[0], w[3] - w[1]
        L = math.hypot(ex_, ey_)
        nx, ny = -ey_ / L, ex_ / L
        if nx * (tr["x"] - mx) + ny * (tr["y"] - my) < 0:
            nx, ny = -nx, -ny                    # the side of the wall the car is on
        cx, cy = mx + 0.35 * nx, my + 0.35 * ny
        th = math.atan2(ey_, ex_)                 # parallel to the wall
        st.world({"op": "teleport", "x": cx, "y": cy, "th": th})
        time.sleep(0.5)
        # body y = left; push toward the wall: -n expressed in the body frame
        vy = -(-math.sin(th) * nx + math.cos(th) * ny) * 4.0
        vx = -(math.cos(th) * nx + math.sin(th) * ny) * 4.0
        contacts = []
        stop_poll = threading.Event()

        def poll():
            while not stop_poll.is_set():
                try:
                    contacts.append(bool(st.truth().get("contact")))
                except (OSError, ValueError, KeyError):
                    pass
                time.sleep(0.02)
        th_ = threading.Thread(target=poll, daemon=True)
        th_.start()
        st.world({"op": "car", "msg": {"fault": "push", "vx": vx, "vy": vy, "g": 0.5}})
        time.sleep(1.2)
        stop_poll.set()
        th_.join()
        t2 = st.truth()
        side = nx * (t2["x"] - mx) + ny * (t2["y"] - my)
        near = min(C.dist_point_seg(t2["x"], t2["y"], *s_) for s_ in room2["walls"])
        moved = math.hypot(t2["x"] - cx, t2["y"] - cy)
        # free, the push carries the car ~0.6 m (4 m/s, 0.15 s decay); against a wall it ends at the car's
        # radius (car_r_m 0.12) from the nearest segment (walls a cell thick can be two segments 0.1 m apart)
        check(side > 0 and near < 0.16 and moved < 0.4,
              f"B2 car pushed at an imported wall stops on it: {near:.2f} m from the nearest wall, moved {moved:.2f} m "
              f"(free: ~0.6), didn't cross (contact seen {any(contacts)})")
        # B3: back to the drop pose, GOTO from there
        s, r = st.http("POST", "/api/maps/drop", drop, timeout=120)
        check(s == 200 and r.get("ok") and (r.get("seed") or {}).get("ok"), f"B3 dropped again, map re-seeded: {r.get('seed')}")
        print("    GOTO trash can from", drop, flush=True)
        st.goto("GOTO trash can")
        kv = st.wait_goto_idle(240)
        tr = st.truth()
        b2 = next(o for o in room2["objects"] if o["label"] == "trash can")
        d = math.hypot(tr["x"] - b2["x"], tr["y"] - b2["y"])
        check(kv.get("result") == "arrived" and d < 1.1,
              f"B3 GOTO trash can from the drop pose on the prescan map: {kv.get('result')} (map={kv.get('map')}, "
              f"replans {kv.get('replans')}), {d:.2f} m from the bin")
    finally:
        st.stop()
        if failed and not VERBOSE:
            print("".join(st.out[-25:]))
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    t0 = time.monotonic()
    unit()
    if "--unit" not in sys.argv:
        e2e()
    load = os.getloadavg()
    print(f"\n{passed} passed, {failed} failed ({time.monotonic() - t0:.0f} s, load {load[0]:.1f} {load[1]:.1f} {load[2]:.1f})")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
