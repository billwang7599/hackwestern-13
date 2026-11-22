#!/usr/bin/env python3
"""Memory edit API end to end (Mac only, SIM: real tars_motor/goto host builds, simulated sensors).

    python3 bench/tests/test_memory.py [-v]

Starts bench/run_sim.py (test_world's harness, bench on :8777), loads pocket_mouth (bin remembered
behind the wall), drives the car away from the start (real MOVE/TURN), then through
GET/POST /api/world/memory: add (stale chair on the map), add {object}, update (move + state),
forget one (no restart), to_room + reset_map (goto restarts with it), to_room write (temp room file),
set, forget all, and an edit during a GOTO (S + CANCEL first: the car stops, goto idle).
After every edit goto's own MEMORY must agree with where the car really is (rel_dist/rel_bearing
against the sim's truth), i.e. the memory frame is right even away from the start.
"""
from __future__ import annotations

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_world as TW  # noqa: E402

C = TW.C
passed = failed = 0


def check(ok, what):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


class Sim(TW.Sim):
    """test_world's harness with the bench on :8777 (the user's demo is on :8771)"""
    def __init__(self):
        real = TW.subprocess.Popen

        def popen(argv, *a, **k):
            if any(str(x).endswith("run_sim.py") for x in argv):
                argv = list(argv) + ["--bench-port", "8777"]
            return real(argv, *a, **k)
        TW.subprocess.Popen = popen
        try:
            super().__init__()
        finally:
            TW.subprocess.Popen = real


def mem(sim):
    code, r = sim.http("GET", "/api/world/memory")
    return r if code == 200 and r.get("ok") else {"entries": [], "error": r.get("error")}


def edit(sim, body):
    code, r = sim.http("POST", "/api/world/memory", body)
    return code, r


def find(entries, label):
    return [e for e in entries if e["label"] == label]


def near(e, x, y, tol=0.06):
    return e is not None and math.hypot(e["x"] - x, e["y"] - y) < tol


def goto_agrees(sim, g, label, x, y, what):
    """goto's own MEMORY: rel_dist/rel_bearing of <label> from the car match the sim's truth"""
    t = sim.state()["truth"]
    r = TW.Line(sim.ports["goto"]).cmd("MEMORY")    # fresh connection: edits restart goto
    ent = [p.strip() for p in r.split(";")[1:] if p.strip().startswith(label + "#")]
    if not ent:
        return check(False, f"{what}: goto MEMORY has a {label} ({r[:160]})")
    kv = dict(x.split("=", 1) for x in ent[-1].split() if "=" in x)
    dist = math.hypot(x - t["x"], y - t["y"])
    brg = math.degrees(C.wrap(t["th"] - math.atan2(y - t["y"], x - t["x"])))   # + = right (clockwise)
    dd = abs(float(kv["rel_dist"]) / 1000.0 - dist)
    db = abs((float(kv["rel_bearing"]) - brg + 180) % 360 - 180)
    check(dd < 0.12 and db < 8, f"{what}: goto's {label} is {kv['rel_dist']} mm at {kv['rel_bearing']}° from the car "
                                f"(truth {dist * 1000:.0f} mm at {brg:.0f}°)")


def main():
    sim = Sim()
    print(f"run_sim up: car={sim.car}, bench :{sim.ports['bench']}")
    g = TW.Line(sim.ports["goto"])
    m = TW.Line(sim.ports["motor"], "WHO test_memory")
    tmp_room = os.path.join(C.ROOMS, "test_memory_tmp.json")
    try:
        print("memory: pocket_mouth seeds the bin behind the wall")
        code, r = sim.http("POST", "/api/world/load", {"name": "pocket_mouth"})
        check(code == 200 and r.get("ok"), f"pocket_mouth loaded ({code})")
        time.sleep(2.0)
        bin_obj = next(o for o in sim.wctl({"op": "get_room"})["room"]["objects"] if o["id"] == "bin")
        e = find(mem(sim)["entries"], "trash can")
        check(len(e) == 1 and near(e[0], bin_obj["x"], bin_obj["y"]), f"GET memory: trash can at the bin ({e})")

        print("memory: drive the car away from the start (TURN 90 right, MOVE 500: open floor)")
        time.sleep(1.6)    # the load's S holds other clients 1.5 s
        m = TW.Line(sim.ports["motor"], "WHO test_memory")
        def settle(timeout=10.0):
            """until the car stops moving (truth unchanged for 0.6 s)"""
            end, last, still = time.monotonic() + timeout, None, 0
            while time.monotonic() < end and still < 3:
                t = sim.state()["truth"]
                still = still + 1 if last and math.hypot(t["x"] - last["x"], t["y"] - last["y"]) < 0.005 and \
                    abs(C.wrap(t["th"] - last["th"])) < 0.01 else 0
                last = t
                time.sleep(0.2)
        r1 = m.cmd("TURN 90")
        time.sleep(0.5)
        settle()
        time.sleep(1.0)
        r2 = m.cmd("MOVE 500")
        time.sleep(0.5)
        settle()
        t = sim.state()["truth"]
        s = sim.wctl({"op": "get_room"})["room"]["start"]
        moved = math.hypot(t["x"] - s["x"], t["y"] - s["y"])
        check(moved > 0.05 or abs(math.degrees(C.wrap(t["th"] - s["th"]))) > 20, f"car away from the start ({r1}, {r2}; {moved:.2f} m, th {math.degrees(t['th']):.0f}°)")
        # (not a check: before any edit goto's memory frame is tars_motor's dead reckoning, which drifts
        # when the sim car under-turns; every edit re-anchors it on the car's real pose)
        tr = sim.state()["truth"]
        print(f"  note before edits: goto MEMORY {TW.Line(sim.ports['goto']).cmd('MEMORY')[:140]} | truth car "
              f"({tr['x']:.2f}, {tr['y']:.2f}, {math.degrees(tr['th']):.0f}°)")

        print("memory: add a stale chair at (1.0, -1.5)")
        code, r = edit(sim, {"op": "add", "label": "chair", "x": 1.0, "y": -1.5, "state": "stale"})
        ch = find(r.get("entries", []), "chair")
        check(code == 200 and r.get("ok") and r.get("goto_restart", {}).get("ok"), f"add: OK, goto restarted ({code} {r.get('error')})")
        check(len(ch) == 1 and near(ch[0], 1.0, -1.5) and ch[0]["state"] == "stale", f"add: chair stale at (1.0, -1.5) ({ch})")
        check(len(find(r.get("entries", []), "trash can")) == 1, "add: the bin is still remembered")
        goto_agrees(sim, g, "chair", 1.0, -1.5, "after add")
        goto_agrees(sim, g, "trash can", bin_obj["x"], bin_obj["y"], "after add")

        print("memory: update moves the bin to (0.5, 1.5) and marks it stale")
        key = find(r.get("entries", []), "trash can")[0]["key"] if find(r.get("entries", []), "trash can") else "trash can#1"
        code, r = edit(sim, {"op": "update", "key": key, "x": 0.5, "y": 1.5, "state": "stale"})
        b = find(r.get("entries", []), "trash can")
        check(code == 200 and len(b) == 1 and near(b[0], 0.5, 1.5) and b[0]["state"] == "stale", f"update: bin stale at (0.5, 1.5) ({b})")
        goto_agrees(sim, g, "trash can", 0.5, 1.5, "after update")
        code, r = edit(sim, {"op": "update", "key": "nothing#9", "x": 0, "y": 0})
        check(code == 404, f"update of a missing key -> 404 ({code})")
        code, r = edit(sim, {"op": "add", "label": "chair", "x": 0, "y": 0, "state": "wet"})
        check(code == 400, f"bad state -> 400 ({code})")

        print("memory: add {object} remembers the bottle where it really is")
        bottle = next(o for o in sim.wctl({"op": "get_room"})["room"]["objects"] if o["id"] == "bottle")
        code, r = edit(sim, {"op": "add", "object": "bottle"})
        wb = find(r.get("entries", []), "water bottle")
        # the camera may have seen the real bottle already (goto keeps its own entry too): any entry at it
        check(code == 200 and any(near(x, bottle["x"], bottle["y"], 0.15) for x in wb), f"add object: a water bottle at ({bottle['x']}, {bottle['y']}) ({wb})")

        print("memory: forget one (goto FORGET, no restart)")
        g = TW.Line(sim.ports["goto"])
        g_before = g.cmd("PING")
        chair_key = find(r.get("entries", []), "chair")[0]["key"]
        code, r = edit(sim, {"op": "forget", "key": chair_key})
        check(code == 200 and r.get("ok") and "goto_restart" not in r and not find(r.get("entries", []), "chair"),
              f"forget {chair_key}: gone, no restart ({r.get('reply')})")
        check(g.cmd("PING").startswith("OK") and g_before.startswith("OK"), "same goto connection still answers (not restarted)")

        print("memory: to_room, then Reset map restarts goto with it")
        code, r = edit(sim, {"op": "to_room"})
        rm = sim.wctl({"op": "get_room"})["room"]["memory"]
        check(code == 200 and r.get("ok") and len(rm) == len(r.get("memory", [])) >= 2, f"to_room: room memory = the live entries ({rm})")
        check(any(x.get("label") == "trash can" and abs(x["x"] - 0.5) < 0.06 and x.get("state") == "stale" for x in rm),
              "to_room: the moved, stale bin is in the room memory")
        code, r = sim.http("POST", "/api/world/reset_map", {})
        check(code == 200 and r.get("ok"), f"reset_map ({code})")
        g = TW.Line(sim.ports["goto"])    # goto restarted
        time.sleep(2.5)
        e = mem(sim)["entries"]
        b = find(e, "trash can")
        check(len(b) == 1 and near(b[0], 0.5, 1.5) and b[0]["state"] == "stale",
              f"after reset_map goto remembers the room memory ({[(x['key'], x['x'], x['y'], x['state']) for x in e]})")

        print("memory: to_room write saves the room file (temp room)")
        code, r = sim.http("POST", "/api/world/save", {"name": "test_memory_tmp", "overwrite": True})
        code, r = edit(sim, {"op": "to_room", "write": True})
        disk = json.load(open(tmp_room)) if os.path.exists(tmp_room) else {}
        check(code == 200 and r.get("ok") and len(disk.get("memory", [])) >= 2, f"to_room write: test_memory_tmp.json has the memory ({disk.get('memory')})")

        print("memory: set replaces everything")
        code, r = edit(sim, {"op": "set", "entries": [{"label": "trash can", "x": 2.5, "y": -0.5}, {"label": "box", "x": 0.0, "y": 1.0, "state": "gone"}]})
        e = r.get("entries", [])
        box = find(e, "box")
        check(code == 200 and len(find(e, "trash can")) == 1 and near(find(e, "trash can")[0], 2.5, -0.5)
              and len(box) == 1 and box[0]["state"] == "gone" and not find(e, "chair"),
              f"set: bin at (2.5, -0.5), box gone, old entries dropped (camera sightings may add more) ({[(x['key'], x['x'], x['y'], x['state']) for x in e]})")

        print("memory: an edit during a GOTO stops the car first (stop wins)")
        time.sleep(1.6)
        g = TW.Line(sim.ports["goto"])
        rep = g.cmd("GOTO trash can")
        p0, moving = sim.state()["truth"], False
        end = time.monotonic() + 15
        while time.monotonic() < end and not moving:
            t = sim.state()["truth"]
            moving = math.hypot(t["x"] - p0["x"], t["y"] - p0["y"]) > 0.05 or abs(C.wrap(t["th"] - p0["th"])) > 0.1
            time.sleep(0.2)
        code, r = edit(sim, {"op": "add", "label": "chair", "x": -1.0, "y": 1.0})
        check(code == 200 and moving and (r.get("stopped") or {}).get("S"), f"edit mid-GOTO ({rep}, car moving={moving}): S sent first ({r.get('stopped')})")
        p0 = sim.state()["truth"]
        time.sleep(1.0)
        p1 = sim.state()["truth"]
        check(math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"]) < 0.03, "car stands still after the edit")
        g = TW.Line(sim.ports["goto"])
        check(TW.kv(g.cmd("STATUS")).get("state") == "idle", "goto is idle (the GOTO was cancelled with the restart)")

        print("memory: forget all")
        code, r = edit(sim, {"op": "forget"})
        check(code == 200 and r.get("ok") and r.get("entries") == [], f"forget all: empty ({r.get('reply')})")
    finally:
        try:
            os.remove(tmp_room)
        except OSError:
            pass
        alive = sim.stop()
        check(not alive, f"run_sim stopped, children gone ({alive})")
    print(f"{passed}/{passed + failed} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
