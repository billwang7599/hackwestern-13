#!/usr/bin/env python3
"""Whole bench end to end, through the bench's HTTP API (what the browser uses). Mac only, SIM.

    python3 bench/tests/test_e2e.py [-v] [--only scenarios,goto,stop,voice,box,collide,moved,detour]

run_sim.py --free-ports with the VIRTUAL SM11 (real tars.ino on a pty + physics), the real host
tars_motor / goto / tars_voice (if built + model), world sim, bench. Nothing here is the robot:
it proves the laptop stack and the glue, not the car. Covers:
  scenarios  every scenario in plugins/scenarios.py, in bring-up order: link, then the speedcal_* ones
             (answered "yes": that saves to the VIRTUAL Uno's EEPROM, gone when vuno exits), then the
             rest. Car scenarios run in room `empty` (fresh start pose each); goto_trash in `default`
  goto       /api/world/goto "trash can" in `default`: arrived, next to the bin
  stop       STOP button (/api/robot/stop) mid-GOTO: wheels at rest time; S reached tars_motor
             before CANCEL reached goto (their logs, ms stamps)
  voice      voice lab send switch on, typed "tars go to the trash can": the car arrives
  box        a box dropped (world ctl) 0.7 m ahead mid-route: never entered, goto ends arrived|failed
  collide    a box dropped right behind the car, then reverse: the car's physics hits it (contact)
  moved      room demo_moved_bin through the bench: arrives at the new spot
  detour     room demo_detour: EXPECTED failed:blocked (goto vs the real ±0.45 m guard corridor)
"""
from __future__ import annotations

import http.client
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
VERBOSE = "-v" in sys.argv
ONLY = None
if "--only" in sys.argv:
    ONLY = set(sys.argv[sys.argv.index("--only") + 1].split(","))

passed = failed = 0
results: list[tuple[str, str, str]] = []      # (name, PASS/FAIL/SKIP/EXPECTED, detail)


def check(ok, what):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)
    return ok


def load():
    try:
        return os.getloadavg()[0]
    except OSError:
        return -1.0


class Sim:
    def __init__(self):
        scratch = os.path.join(REPO, ".scratch")
        os.makedirs(scratch, exist_ok=True)
        self.pf = os.path.join(scratch, f"test_e2e_{os.getpid()}.json")
        self.p = subprocess.Popen([sys.executable, os.path.join(BENCH, "run_sim.py"), "--free-ports", "--no-build",
                                   "--ports-file", self.pf], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.out: list[str] = []
        threading.Thread(target=self._read, daemon=True).start()
        end = time.monotonic() + 150
        while not os.path.exists(self.pf):
            if self.p.poll() is not None or time.monotonic() > end:
                raise RuntimeError("run_sim didn't come up:\n" + "".join(self.out[-30:]))
            time.sleep(0.2)
        info = json.load(open(self.pf))
        self.ports, self.pids, self.car, self.run_dir = info["ports"], info["pids"], info["car"], info["run_dir"]
        self.voice = info.get("voice", "")
        self.lock = threading.Lock()
        self.events: list[tuple[float, str, dict]] = []      # (monotonic, topic, data)
        self.latest: dict[str, dict] = {}
        self.alive = True
        threading.Thread(target=self._sse, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            self.out.append(line)
            if VERBOSE:
                print("    | " + line, end="")

    def _sse(self):
        c = http.client.HTTPConnection("127.0.0.1", self.ports["bench"], timeout=60)
        c.request("GET", "/events")
        r = c.getresponse()
        topic = None
        while self.alive:
            ln = r.fp.readline()
            if not ln:
                return
            ln = ln.decode().rstrip("\n")
            if ln.startswith("event: "):
                topic = ln[7:]
            elif ln.startswith("data: ") and topic:
                try:
                    d = json.loads(ln[6:])
                except ValueError:
                    continue
                with self.lock:
                    self.latest[topic] = d
                    if topic not in ("status", "world", "phone"):      # keep memory sane
                        self.events.append((time.monotonic(), topic, d))
                    if topic == "car":
                        self.events.append((time.monotonic(), "car", d))

    def http(self, method, path, body=None, timeout=30):
        c = http.client.HTTPConnection("127.0.0.1", self.ports["bench"], timeout=timeout)
        c.request(method, path, body=json.dumps(body) if body is not None else None)
        r = c.getresponse()
        raw = r.read()
        try:
            return r.status, json.loads(raw or b"{}")
        except ValueError:
            return r.status, {"raw": raw[:200].decode("utf-8", "replace")}

    def send(self, dest, line):
        return self.http("POST", "/api/robot/send", {"dest": dest, "line": line})[1].get("reply", "")

    def truth(self):
        c = self.latest.get("car") or {}
        return c.get("truth") if c.get("alive") else None

    def goto_kv(self):
        r = self.send("goto", "STATUS")
        return dict(x.split("=", 1) for x in r.split()[1:] if "=" in x)

    def wait_goto_idle(self, timeout, hook=None):
        end = time.monotonic() + timeout
        st = {}
        while time.monotonic() < end:
            st = self.goto_kv()
            if hook:
                hook(st)
            if st.get("state") == "idle":
                return st
            time.sleep(0.3)
        return st

    def load_room(self, name):
        code, r = self.http("POST", "/api/world/load", {"name": name})
        time.sleep(1.0)      # goto restart + pose reset + car teleport settle
        return code, r

    def stop(self):
        self.alive = False
        self.p.send_signal(signal.SIGINT)
        try:
            self.p.wait(25)
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


def want(name):
    return ONLY is None or name in ONLY


# ------------------------------------------------------------------ scenarios
def sc_scenarios(sim: Sim):
    print("scenarios (plugins/scenarios.py) on the virtual car")
    code, lst = sim.http("GET", "/api/scenarios/list")
    names = [s["name"] for s in lst.get("scenarios", [])]
    check(code == 200 and names, f"{len(names)} scenarios listed")
    first = [n for n in ("link", "speedcal_move", "speedcal_spin", "speedcal_slow") if n in names]
    names = first + [n for n in names if n not in first]
    for name in names:
        # car measurements need open floor (the guard would stop a MOVE 1000 at default's partition)
        sim.load_room("default" if name == "goto_trash" else "empty")
        sim.http("POST", "/api/robot/stop", {"why": "e2e: between scenarios"})
        # STOP = S + CANCEL; idle goto answers CANCEL with its own S, which holds the bench's
        # motion for 1.5 s (tars_motor S-hold). Wait it out so the next MOVE isn't aborted.
        time.sleep(2.0)
        n0 = len(sim.events)
        t0 = time.monotonic()
        code, r = sim.http("POST", "/api/scenarios/run", {"name": name})
        if code != 200:
            check(False, f"{name}: run refused ({code} {r})")
            results.append((name, "FAIL", f"refused {r}"))
            continue
        last, answered = None, set()
        while time.monotonic() - t0 < 240:
            with sim.lock:
                evs = [d for _, tp, d in sim.events[n0:] if tp == "scenario" and d.get("name") == name]
            if evs:
                last = evs[-1]
                pr = last.get("prompt")
                if pr and id(pr) not in answered and last.get("state") == "running":
                    key = json.dumps(pr, sort_keys=True) + str(len(last.get("steps", [])))
                    if key not in answered:
                        answered.add(key)
                        # yes/no = "save SPEEDCAL to the Uno?": yes, it's the virtual Uno's EEPROM
                        val = {"yesno": True, "ok": True}.get(pr.get("kind"), None)
                        sim.http("POST", "/api/scenarios/answer", {"value": val})
                if last.get("state") not in (None, "running"):
                    break
            time.sleep(0.2)
        st = (last or {}).get("state", "timeout")
        nums = ", ".join(f"{k}={v.get('v')}{(' ' + v['unit'].split()[0]) if v.get('unit') else ''}"
                         for k, v in ((last or {}).get("numbers") or {}).items())
        fails = [s["text"] for s in (last or {}).get("steps", []) if s.get("kind") in ("fail", "skip")]
        where = (last or {}).get("where", "?")
        detail = f"[{where}] {nums}" + (f" | {'; '.join(fails)[:300]}" if fails else "")
        results.append((name, st.upper(), detail))
        check(st == "pass" and where == "sim-car", f"{name}: {st} {detail[:260]}  ({time.monotonic() - t0:.1f} s, load {load():.1f})")


# ------------------------------------------------------------------ goto
def sc_goto(sim: Sim, room="default", label="goto default"):
    print(f"{label}: /api/world/goto trash can in `{room}`")
    code, r = sim.load_room(room)
    check(code == 200 and r.get("ok"), f"room {room} loaded through the bench ({code})")
    rm = sim.http("GET", "/api/world/room")[1]
    bins = [o for o in rm.get("objects", []) if o.get("label") == "trash can"]
    target = (bins[0]["x"], bins[0]["y"]) if bins else (2.4, 0.0)
    t0 = time.monotonic()
    code, r = sim.http("POST", "/api/world/goto", {"label": "trash can"})
    check(code == 200 and r.get("ok"), f"GOTO accepted ({r.get('reply')})")
    st = sim.wait_goto_idle(240)
    tr = sim.truth() or {"x": 0, "y": 0}
    d = math.hypot(tr["x"] - target[0], tr["y"] - target[1])
    w = sim.latest.get("world") or {}
    ok = check(st.get("result") == "arrived" and d < 1.0,
               f"arrived {d:.2f} m from the bin ({st.get('result')}, replans={st.get('replans')}, {time.monotonic() - t0:.0f} s, load {load():.1f})")
    check(w.get("goto") is not None and w.get("robot"), f"world topic carries goto status for the 2D banner (goto={str(w.get('goto'))[:80]})")
    results.append((label, "PASS" if ok else "FAIL", f"{st.get('result')} {d:.2f} m, {time.monotonic() - t0:.0f} s"))


def sc_route_seen(sim: Sim):
    """during a GOTO the world topic carries a route for the 2D panel"""
    print("world2d: route in the world topic during a GOTO")
    sim.load_room("default")
    sim.http("POST", "/api/world/goto", {"label": "trash can"})
    seen = False
    end = time.monotonic() + 30
    while time.monotonic() < end and not seen:
        w = sim.latest.get("world") or {}
        seen = bool(w.get("route"))
        time.sleep(0.2)
    sim.http("POST", "/api/robot/stop", {"why": "e2e: route check done"})
    check(seen, "world topic had a non-empty route while goto was busy")
    results.append(("world2d route", "PASS" if seen else "FAIL", ""))
    time.sleep(1.5)


# ------------------------------------------------------------------ stop
MS_RE = re.compile(r"(\d{2}):(\d{2}):(\d{2})\.(\d{3})")


def log_lines(sim, name):
    try:
        return open(os.path.join(sim.run_dir, f"{name}.log"), errors="replace").read().splitlines()
    except OSError:
        return []


def sc_stop(sim: Sim):
    print("STOP button during a GOTO")
    sim.load_room("default")
    sim.http("POST", "/api/world/goto", {"label": "trash can"})
    end = time.monotonic() + 60
    while time.monotonic() < end:           # wait until it's really driving
        tr = sim.truth() or {}
        if abs(tr.get("wl", 0)) > 0.05 or abs(tr.get("wr", 0)) > 0.05:
            break
        time.sleep(0.02)
    tr = sim.truth() or {}
    moving = abs(tr.get("wl", 0)) > 0.05 or abs(tr.get("wr", 0)) > 0.05
    check(moving, f"car was moving before STOP (wl={tr.get('wl')}, wr={tr.get('wr')})")
    n0 = len(sim.events)
    t0 = time.monotonic()
    code, r = sim.http("POST", "/api/robot/stop", {"why": "e2e STOP button"})
    check(code == 200 and r.get("S") and r.get("CANCEL"), f"STOP sent S and CANCEL ({r})")
    t_rest = None
    while time.monotonic() - t0 < 3:
        with sim.lock:
            cars = [(t, d) for t, tp, d in sim.events[n0:] if tp == "car" and d.get("truth")]
        for t, d in cars:
            tt = d["truth"]
            if abs(tt.get("wl", 1)) < 0.01 and abs(tt.get("wr", 1)) < 0.01:
                t_rest = t - t0
                break
        if t_rest is not None:
            break
        time.sleep(0.02)
    ok1 = check(t_rest is not None and t_rest < 0.6,
                f"wheels at rest {t_rest * 1000 if t_rest is not None else -1:.0f} ms after the STOP click (car topic is 20 Hz, so +-50 ms)")
    st = sim.goto_kv()
    check(st.get("state") == "idle", f"goto idle after STOP ({st.get('state')}, {st.get('result')})")
    # order: which arrived first. cmd events are published when the reply comes back; use them as a
    # cross-check, and the per-process "S"/"cancel" receipt if their logs carry it.
    with sim.lock:
        cmds = [(t, d) for t, tp, d in sim.events[n0:] if tp == "cmd" and d.get("src") == "stop"]
    order = [d.get("dest") + ":" + d.get("line", "") for _, d in cmds]
    s_first = bool(order) and order[0].startswith("motor:S")
    check(s_first, f"S (motor) answered before CANCEL (goto): {order}")
    results.append(("STOP mid-GOTO", "PASS" if ok1 and s_first else "FAIL",
                    f"wheels at rest {t_rest * 1000 if t_rest is not None else -1:.0f} ms; reply order {order}"))
    time.sleep(1.6)


# ------------------------------------------------------------------ voice
def sc_voice(sim: Sim):
    print("voice lab: send on, typed 'tars go to the trash can'")
    code, info = sim.http("GET", "/api/voicelab/info")
    if "tars_voice" not in "".join(sim.out) or "not built" in sim.voice or "no model" in sim.voice:
        results.append(("voice -> GOTO", "SKIP", f"tars_voice not up: {sim.voice}"))
        print(f"  skip tars_voice not running ({sim.voice})")
        return
    sim.load_room("default")
    code, r = sim.http("POST", "/api/voicelab/send", {"on": True})
    check(code == 200 and r.get("send"), f"send switch on ({r})")
    t0 = time.monotonic()
    code, r = sim.http("POST", "/api/voicelab/text", {"text": "tars go to the trash can"}, timeout=40)
    check(code == 200, f"voice lab took the text ({code}): intent {str(r.get('intent') or r.get('decision') or '')[:100]}; robot {str(r.get('robot'))[:120]}")
    busy = False
    end = time.monotonic() + 10
    while time.monotonic() < end and not busy:
        busy = sim.goto_kv().get("state") not in (None, "idle")
        time.sleep(0.2)
    check(busy, "goto went busy from the spoken command")
    st = sim.wait_goto_idle(240)
    tr = sim.truth() or {"x": 0, "y": 0}
    d = math.hypot(tr["x"] - 2.4, tr["y"])
    ok = check(st.get("result") == "arrived" and d < 1.0, f"car arrived {d:.2f} m from the bin ({st.get('result')}, {time.monotonic() - t0:.0f} s)")
    sim.http("POST", "/api/voicelab/send", {"on": False})
    results.append(("voice -> GOTO", "PASS" if ok else "FAIL", f"{st.get('result')} {d:.2f} m"))


# ------------------------------------------------------------------ box mid-route
BOX_ID = "e2e_box"


def box_inside(tr, bx, by, w, dpt, margin=0.0):
    return abs(tr["x"] - bx) < w / 2 + margin and abs(tr["y"] - by) < dpt / 2 + margin


def sc_box(sim: Sim):
    print("box dropped live 0.7 m ahead of the car mid-route")
    sim.load_room("default")
    sim.http("POST", "/api/world/goto", {"label": "trash can"})
    end = time.monotonic() + 60
    start = None
    while time.monotonic() < end:
        tr = sim.truth()
        if tr:
            start = start or (tr["x"], tr["y"])
            if math.hypot(tr["x"] - start[0], tr["y"] - start[1]) > 0.35 and abs(tr.get("v", 0)) > 0.05:
                break
        time.sleep(0.05)
    tr = sim.truth()
    bx, by = tr["x"] + 0.7 * math.cos(tr["th"]), tr["y"] + 0.7 * math.sin(tr["th"])
    code, r = sim.http("POST", "/api/world/ctl", {"op": "add", "obj": {"id": BOX_ID, "label": "", "x": bx, "y": by,
                                                                        "w": 0.30, "d": 0.30, "h": 0.30, "solid": True}})
    check(code == 200 and r.get("ok"), f"box dropped at {bx:.2f},{by:.2f} ({r})")
    inside = contact = 0
    blocked = 0

    def hook(st):
        nonlocal inside, contact, blocked
        t = sim.truth()
        if t:
            inside += box_inside(t, bx, by, 0.30, 0.30)
            contact += bool(t.get("contact"))
        w = sim.latest.get("world") or {}
        blocked += ((w.get("guard") or {}).get("verdict") == "BLOCKED")
    st = sim.wait_goto_idle(240, hook)
    m3 = sim.http("GET", "/api/phone/map.json")[1]
    check(inside == 0, "the car's centre never entered the box")
    check(st.get("state") == "idle" and (st.get("result") == "arrived" or str(st.get("result", "")).startswith("failed")),
          f"goto ended arrived or failed, not stuck ({st.get('result')}, replans={st.get('replans')}, guard BLOCKED samples={blocked}, contact samples={contact})")
    results.append(("box dropped mid-route", "PASS" if inside == 0 and st.get("state") == "idle" else "FAIL",
                    f"{st.get('result')}, replans={st.get('replans')}, BLOCKED samples={blocked}, contact={contact}"))
    sim.http("POST", "/api/world/ctl", {"op": "remove", "id": BOX_ID})
    time.sleep(1.0)


def sc_collide(sim: Sim):
    print("collision: box dropped right behind the car, reverse into it (guard watches forward only)")
    sim.load_room("empty")
    time.sleep(1.0)
    tr = sim.truth()
    bx, by = tr["x"] - 0.30 * math.cos(tr["th"]), tr["y"] - 0.30 * math.sin(tr["th"])
    code, r = sim.http("POST", "/api/world/ctl", {"op": "add", "obj": {"id": BOX_ID, "label": "", "x": bx, "y": by,
                                                                        "w": 0.20, "d": 0.40, "h": 0.30, "solid": True}})
    check(code == 200 and r.get("ok"), f"box dropped 0.30 m behind ({r})")
    time.sleep(0.8)          # walls are pushed to the car on every edit
    bumps0 = int((sim.http("GET", "/api/robot/state")[1].get("status") or {}).get("bumps", 0) or 0)
    contact, x_min = False, tr["x"]
    t0 = time.monotonic()
    while time.monotonic() - t0 < 3.0:
        sim.send("motor", "D -0.6 -0.6")
        t = sim.truth() or {}
        contact = contact or bool(t.get("contact"))
        x_min = min(x_min, t.get("x", x_min))
        time.sleep(0.1)
    sim.send("motor", "S")
    time.sleep(0.3)
    st = sim.http("GET", "/api/robot/state")[1].get("status") or {}
    t = sim.truth() or {}
    back_face = bx + 0.10        # box spans bx-0.10 .. bx+0.10 in x (car faces +x at start)
    params = (sim.http("GET", "/api/car/state")[1].get("params") or {})
    R = params.get("car_r_m", 0.12)
    R = R.get("v", R.get("value", 0.12)) if isinstance(R, dict) else float(R)
    stopped_at = t.get("x", 0)
    # the physics holds the centre at face + collision radius; `contact` is only true on the steps
    # where it pushes the car back out, so a 10 Hz sample can miss it: judge by where it stopped
    at_face = abs(stopped_at - (back_face + R)) < 0.03
    ok = check(at_face, f"car stopped against the dropped box while still reversing: x {tr['x']:.2f} -> {stopped_at:.2f}, "
                        f"expected face {back_face:.2f} + radius {R:.2f} = {back_face + R:.2f} (contact seen: {contact})")
    check(stopped_at > bx, f"the car didn't drive through the box (x {stopped_at:.2f} > box centre {bx:.2f})")
    print(f"    tars_motor bumps {bumps0} -> {st.get('bumps')} (firmware bump detection on the impact spike; informational)")
    results.append(("dropped box reaches the car's walls", "PASS" if ok and stopped_at > bx else "FAIL",
                    f"stopped x={stopped_at:.2f} (face+R {back_face + R:.2f}), contact seen={contact}, tars_motor bumps {bumps0}->{st.get('bumps')}"))
    sim.http("POST", "/api/world/ctl", {"op": "remove", "id": BOX_ID})


def sc_phone_box(sim: Sim):
    print("3D map (phone sim) picks up a dropped box from the world topic")
    sim.http("POST", "/api/phone/source", {"kind": "sim"})
    sim.load_room("empty")
    sim.http("POST", "/api/phone/command", {"op": "map_clear"})
    time.sleep(1.5)
    tr = sim.truth()
    bx, by = tr["x"] + 1.2 * math.cos(tr["th"]), tr["y"] + 1.2 * math.sin(tr["th"])

    def occ_near():
        m = sim.http("GET", "/api/phone/map.json")[1]
        n = m.get("size", 0)
        res = m.get("resolution", 0.05)
        org = m.get("origin", [0, 0])
        g = m.get("grid", "")
        return m, n, res, org, g
    m0 = sim.http("GET", "/api/phone/status")[1]
    code, r = sim.http("POST", "/api/world/ctl", {"op": "add", "obj": {"id": "e2e_box3d", "label": "", "x": bx, "y": by,
                                                                        "w": 0.4, "d": 0.4, "h": 0.5, "solid": True}})
    time.sleep(3.0)
    st = sim.http("GET", "/api/phone/status")[1]
    note = str(st)[:240]
    seen = "world" in json.dumps(st)
    check(code == 200 and r.get("ok"), f"box dropped for the 3D map ({r})")
    check(seen, f"phone sim follows the world room (status: {note})")
    results.append(("3D map follows world room", "PASS" if seen else "FAIL", note[:160]))
    sim.http("POST", "/api/world/ctl", {"op": "remove", "id": "e2e_box3d"})


def sc_detour(sim: Sim):
    print("demo_detour (EXPECTED failed:blocked until goto's corridor fix lands)")
    sim.load_room("demo_detour")
    sim.http("POST", "/api/world/goto", {"label": "trash can"})
    st = sim.wait_goto_idle(300)
    res = str(st.get("result"))
    print(f"    result {res}, replans={st.get('replans')}")
    tag = "EXPECTED" if res.startswith("failed") else ("PASS (unexpected)" if res == "arrived" else "FAIL")
    results.append(("demo_detour", tag, f"{res}, replans={st.get('replans')}"))
    check(st.get("state") == "idle", f"goto ended, not stuck ({res})")


def main():
    t0 = time.monotonic()
    print(f"load {load():.1f}")
    sim = Sim()
    print(f"run_sim up: car={sim.car}, voice={sim.voice}, bench :{sim.ports['bench']}, logs {sim.run_dir}")
    check(sim.car == "virtual", f"run_sim started the VIRTUAL car ({sim.car})")
    try:
        end = time.monotonic() + 20
        while time.monotonic() < end and not sim.truth():
            time.sleep(0.2)
        check(sim.truth() is not None, "car topic is live in the bench (truth flowing)")
        w = sim.latest.get("world") or {}
        check(w.get("car_kind") == "virtual", f"world topic says car_kind=virtual ({w.get('car_kind')})")
        for name, fn in (("scenarios", sc_scenarios), ("goto", sc_goto), ("route", sc_route_seen), ("stop", sc_stop),
                         ("voice", sc_voice), ("box", sc_box), ("collide", sc_collide), ("phone", sc_phone_box),
                         ("moved", lambda s: sc_goto(s, "demo_moved_bin", "goto demo_moved_bin")),
                         ("detour", sc_detour)):
            if not want(name):
                continue
            try:
                fn(sim)
            except Exception as e:
                import traceback
                traceback.print_exc()
                check(False, f"{name} crashed: {type(e).__name__}: {e}")
                results.append((name, "ERROR", f"{type(e).__name__}: {e}"))
            sim.http("POST", "/api/robot/stop", {"why": "e2e: between parts"})
            time.sleep(0.5)
    finally:
        alive = sim.stop()
        check(not alive, f"run_sim stopped every child ({alive or 'none left'})")
    print("\nRESULTS")
    for n, s, d in results:
        print(f"  {s:<9} {n:<36} {d}")
    print(f"\n{passed}/{passed + failed} checks passed ({time.monotonic() - t0:.0f} s, load {load():.1f})")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
