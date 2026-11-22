#!/usr/bin/env python3
"""Navbar pages + the two "remove an object" paths, through the bench's HTTP API (no browser).

Static: every panel is on a page the shell knows (PAGES in web/shell.js, or a legacy drawer id), each
page in PAGES gets at least one panel, index.html has the navbar and no old drawer, `node --check` on
every web file (if node is installed).

Live (run_sim.py --free-ports --no-build --car stub --room open_bin --no-voice):
- Room page table: 🗑 sends world/ctl {op: remove, id}; Undo re-adds the saved object ({op: add, obj}) or
  wall ({op: add_wall, seg}). Checks the object/wall is gone from /api/world/room, then back as it was.
- Memory page: POST /api/mission/goto {line: "FORGET <label>#<id>"} for one row, "FORGET all" for the
  button. Checks goto answers "OK forgot n" and that the MEMORY list (topic mission / /api/mission/state)
  loses the forgotten entry (or re-learns it as a new sighting, since the camera still sees the bin).

Not covered: clicks in a real browser (see bench/README.md, "Pages"), the real robot.
"""
from __future__ import annotations

import http.client
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

BENCH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(BENCH)
WEB = os.path.join(BENCH, "web")
passed = failed = 0


def check(ok, what):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)
    return ok


# ------------------------------------------------------------------ static
def static_checks():
    print("static: pages and panels")
    shell = open(os.path.join(WEB, "shell.js")).read()
    m = re.search(r"const PAGES = \[(.*?)\n  \];", shell, re.S)
    pages = re.findall(r"\[\s*'([a-z0-9-]+)'", m.group(1)) if m else []
    legacy = dict(re.findall(r"(\w+): '(\w+)'", re.search(r"const LEGACY = \{(.*?)\};", shell).group(1)))
    check(pages == ["drive", "room", "memory", "voice", "car", "tests", "mission"], f"PAGES in navbar order: {pages}")
    used: dict[str, list[str]] = {p: [] for p in pages}
    used["tests"].append("log")                         # the shell's own log panel
    bad = []
    for f in sorted(os.listdir(os.path.join(WEB, "panels"))):
        if not f.endswith(".js"):
            continue
        src = open(os.path.join(WEB, "panels", f)).read()
        for pid, area in re.findall(r"id: '([\w-]+)',[^\n]*?area: '([\w-]+)'", src):
            if area in ("view", "header"):
                continue
            page = legacy.get(area, area)
            if page not in used:
                bad.append(f"{f}:{pid} area {area}")
            else:
                used[page].append(pid)
    check(not bad, "every panel is on a known page" + (f": {bad}" if bad else ""))
    empty = [p for p, ids in used.items() if not ids]
    check(not empty, "every page has a panel: " + ", ".join(f"{p}={'/'.join(ids)}" for p, ids in used.items()))
    check("room-objects" in used["room"] and "mc-memory" in used["memory"], "Room has the objects table, Memory has the memory table")
    html = open(os.path.join(WEB, "index.html")).read()
    check('id="navbar"' in html and 'id="drawer"' not in html and 'id="stop"' in html, "index.html: navbar + STOP, no old drawer")
    w2d = open(os.path.join(WEB, "panels", "world2d.js")).read()
    check("'contextmenu'" in w2d and "op: 'remove'" in w2d and "op: 'add', obj: u.obj" in w2d, "world2d: right-click menu, remove, undo re-add")
    mis = open(os.path.join(WEB, "panels", "mission.js")).read()
    check("FORGET ${what}" in mis and "'FORGET all'" in mis and "confirm(" in mis, "memory: Forget per row, Forget all behind a confirm")
    node = shutil.which("node")
    if node:
        files = [os.path.join(WEB, "shell.js")] + [os.path.join(WEB, "panels", f) for f in os.listdir(os.path.join(WEB, "panels")) if f.endswith(".js")]
        errs = [f for f in files if subprocess.run([node, "--check", f], capture_output=True).returncode]
        check(not errs, f"node --check {len(files)} web files" + (f": {errs}" if errs else ""))
    else:
        print("  skip node --check (no node)")


# ------------------------------------------------------------------ live
class Sim:
    def __init__(self):
        scratch = os.path.join(REPO, ".scratch")
        os.makedirs(scratch, exist_ok=True)
        self.pf = os.path.join(scratch, f"test_pages_{os.getpid()}.json")
        self.p = subprocess.Popen([sys.executable, os.path.join(BENCH, "run_sim.py"), "--free-ports", "--no-build",
                                   "--car", "stub", "--room", "open_bin", "--no-voice", "--ports-file", self.pf],
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.out: list[str] = []
        threading.Thread(target=lambda: [self.out.append(l) for l in self.p.stdout], daemon=True).start()
        end = time.monotonic() + 150
        while not os.path.exists(self.pf):
            if self.p.poll() is not None or time.monotonic() > end:
                raise RuntimeError("run_sim didn't come up:\n" + "".join(self.out[-30:]))
            time.sleep(0.2)
        time.sleep(0.3)
        self.ports = json.load(open(self.pf))["ports"]

    def http(self, method, path, body=None, timeout=30):
        c = http.client.HTTPConnection("127.0.0.1", self.ports["bench"], timeout=timeout)
        c.request(method, path, body=json.dumps(body) if body is not None else None)
        r = c.getresponse()
        raw = r.read()
        try:
            return r.status, json.loads(raw or b"{}")
        except ValueError:
            return r.status, {"raw": raw[:200].decode("utf-8", "replace")}

    def stop(self):
        self.p.send_signal(signal.SIGINT)
        try:
            self.p.wait(25)
        except subprocess.TimeoutExpired:
            self.p.kill()
        try:
            os.remove(self.pf)
        except OSError:
            pass


def wait_for(fn, secs, step=0.3):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return fn()


def room(sim):
    st, r = sim.http("GET", "/api/world/room")
    return (r.get("room") or r) if st == 200 else {}


def live_checks(sim):
    print("Room page: delete from the table, then Undo")
    r = wait_for(lambda: room(sim).get("objects") and room(sim), 30)
    objs = {o["id"]: o for o in r.get("objects", [])}
    check({"bin", "bottle"} <= set(objs), f"open_bin has bin and bottle: {sorted(objs)}")
    saved = objs.get("bottle")
    st, rep = sim.http("POST", "/api/world/ctl", {"op": "remove", "id": "bottle"})        # the row's 🗑
    check(st == 200 and rep.get("ok"), f"remove bottle -> {rep}")
    gone = wait_for(lambda: "bottle" not in {o["id"] for o in room(sim).get("objects", [])}, 5)
    check(gone, "bottle gone from the room")
    st, rep = sim.http("POST", "/api/world/ctl", {"op": "remove", "id": "bottle"})
    check(not rep.get("ok") and "no object" in str(rep.get("error")), f"second remove is refused, not silent: {rep}")
    st, rep = sim.http("POST", "/api/world/ctl", {"op": "add", "obj": saved})              # Undo
    back = wait_for(lambda: next((o for o in room(sim).get("objects", []) if o["id"] == "bottle"), None), 5)
    check(rep.get("ok") and back and all(abs(back[k] - saved[k]) < 1e-6 for k in ("x", "y", "h")) and back.get("label") == saved.get("label"),
          f"undo restores bottle as it was: {back}")
    walls = room(sim).get("walls", [])
    w0 = walls[0] if walls else None
    st, rep = sim.http("POST", "/api/world/ctl", {"op": "remove", "id": "w0"})
    check(rep.get("ok") and wait_for(lambda: len(room(sim).get("walls", [])) == len(walls) - 1, 5), "remove wall w0")
    st, rep = sim.http("POST", "/api/world/ctl", {"op": "add_wall", "seg": w0})
    check(rep.get("ok") and wait_for(lambda: any(all(abs(a - b) < 1e-6 for a, b in zip(w, w0)) for w in room(sim).get("walls", [])), 5),
          "undo re-adds the wall")

    print("Memory page: Forget one, Forget all")
    def mem():
        st, s = sim.http("GET", "/api/mission/state")
        return ((s.get("memory") or {}).get("objects") or []) if st == 200 else []
    first = wait_for(lambda: [o for o in mem() if o["name"].startswith("trash")], 40)
    check(bool(first), f"goto remembers the bin (camera sees it): {[o['name'] for o in mem()]}")
    if first:
        name = first[0]["name"].replace("_", " ")
        seen0 = int(first[0].get("seen", 0))
        st, rep = sim.http("POST", "/api/mission/goto", {"line": f"FORGET {name}"})          # the row's Forget
        check(st == 200 and str(rep.get("reply", "")).startswith("OK forgot 1"), f"FORGET {name} -> {rep}")
        # it is either gone, or re-learned from scratch (seen count restarts), never the old entry
        def fresh():
            m = [o for o in mem() if o["name"].replace("_", " ") == name]
            return not m or int(m[0].get("seen", 0)) < seen0
        check(wait_for(fresh, 5), "the forgotten entry left goto's memory (re-learned at most as a new sighting)")
    st, rep = sim.http("POST", "/api/mission/goto", {"line": "FORGET all"})                   # Forget all…
    check(st == 200 and re.match(r"OK forgot \d+", str(rep.get("reply", ""))), f"FORGET all -> {rep}")
    st, rep = sim.http("POST", "/api/mission/goto", {"line": "FORGET no such thing"})
    check(st == 200 and str(rep.get("reply", "")).startswith("OK forgot 0"), f"FORGET of something unknown forgets 0: {rep}")


def main():
    static_checks()
    if "--static" not in sys.argv:
        sim = Sim()
        try:
            live_checks(sim)
        finally:
            sim.stop()
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
