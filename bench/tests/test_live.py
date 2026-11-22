#!/usr/bin/env python3
"""The live view (bench --target robot + plugins/live.py) against the sim stack, where the truth is known.

    python3 bench/tests/test_live.py        (needs make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host)

run_sim starts the real tars_motor + goto host builds, the world sim and the stub car (no bench). A second
bench is then started the way it runs ON the Pi (`--target robot`, direct TCP to tars_motor / goto) with
only the robot, maps, live and world plugins: no world sim plugin, so everything it draws comes from goto.
Checks:
  * topic world / GET /api/live/state is live: goto's pose == the sim car's truth (position AND heading,
    which pins down the NAV yaw convention), standing still and while a GOTO drives
  * the route goto is following, and its memory (the room's bin) at the right place
  * panels.json keeps world2d on the real target (live: true), the sim-only world plugin routes are gone
  * the token gate: --bind 0.0.0.0 needs --token; no token -> 401; ?token -> 200 + cookie; cookie -> 200
"""
from __future__ import annotations

import json
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
ok = fail = 0


def check(c, m):
    global ok, fail
    print(("ok   " if c else "FAIL ") + m, flush=True)
    ok += bool(c)
    fail += (not c)


def free_port() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def get(url, headers=None, timeout=3):
    req = urllib.request.Request(url, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)


def post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def world_state(port):
    with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
        f = s.makefile("rw")
        f.write(json.dumps({"op": "get_state"}) + "\n")
        f.flush()
        return json.loads(f.readline()).get("state") or {}


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def main() -> int:
    tmp = tempfile.mkdtemp(prefix="tars-live-")
    ports_file = os.path.join(tmp, "ports.json")
    sim = subprocess.Popen([sys.executable, os.path.join(BENCH, "run_sim.py"), "--car", "stub", "--free-ports",
                            "--no-bench", "--no-voice", "--no-build", "--room", "default", "--ports-file", ports_file],
                           stdout=open(os.path.join(tmp, "run_sim.log"), "w"), stderr=subprocess.STDOUT,
                           start_new_session=True)
    procs = [sim]
    try:
        end = time.monotonic() + 60
        while not os.path.exists(ports_file) and time.monotonic() < end and sim.poll() is None:
            time.sleep(0.2)
        if not os.path.exists(ports_file):
            print(open(os.path.join(tmp, "run_sim.log")).read()[-3000:])
            check(False, "run_sim came up (ports file)")
            return 1
        time.sleep(0.5)
        P = json.load(open(ports_file))
        ports = P.get("ports", P)
        check(all(k in ports for k in ("motor", "goto", "world_ctl")), f"run_sim up: motor {ports.get('motor')} goto {ports.get('goto')}")

        bp = free_port()
        bench = subprocess.Popen([sys.executable, os.path.join(BENCH, "bench.py"), "--target", "robot", "--port", str(bp),
                                  "--motor", f"127.0.0.1:{ports['motor']}", "--goto", f"127.0.0.1:{ports['goto']}",
                                  "--voice", "127.0.0.1:1", "--plugins", "robot,maps,live,world"],
                                 stdout=open(os.path.join(tmp, "bench.log"), "w"), stderr=subprocess.STDOUT)
        procs.append(bench)
        base = f"http://127.0.0.1:{bp}"
        live = {}
        end = time.monotonic() + 30
        while time.monotonic() < end:
            try:
                live = json.loads(get(base + "/api/live/state")[1])
                if live.get("alive") and live.get("truth"):
                    break
            except Exception:
                pass
            time.sleep(0.3)
        check(live.get("live") and live.get("alive") and live.get("truth"),
              f"live view up with a pose from goto NAV ({(live.get('truth') or {}).get('src')})")

        pj = json.loads(get(base + "/panels.json")[1])
        check(pj.get("live") is True and "world2d.js" in pj["panels"] and "car.js" not in pj["panels"],
              f"panels.json: live, world2d kept, sim-only car panel hidden ({pj.get('hidden')})")
        code, _, _ = get(base + "/api/world/room")
        check(code == 404, f"no sim room on the real target (/api/world/room -> {code})")

        def compare(tag, tol_m, tol_deg):
            lv = json.loads(get(base + "/api/live/state")[1]).get("truth")
            tr = world_state(ports["world_ctl"]).get("truth")
            if not lv or not tr:
                check(False, f"{tag}: poses available (live {bool(lv)}, truth {bool(tr)})")
                return
            d = math.hypot(lv["x"] - tr["x"], lv["y"] - tr["y"])
            a = math.degrees(angle_diff(lv["th"], tr["th"]))
            check(d < tol_m and a < tol_deg,
                  f"{tag}: live {lv['x']:.2f},{lv['y']:.2f} {math.degrees(lv['th']):.0f}° vs truth "
                  f"{tr['x']:.2f},{tr['y']:.2f} {math.degrees(tr['th']):.0f}° (off {d:.2f} m, {a:.0f}°)")

        compare("standing still", 0.15, 8)

        # memory: the room seeds goto with the bin where it is (2.4, 0.0)
        end = time.monotonic() + 10
        objs = []
        while time.monotonic() < end:
            objs = json.loads(get(base + "/api/live/state")[1]).get("room", {}).get("objects", [])
            if objs:
                break
            time.sleep(0.5)
        bin_ = next((o for o in objs if o["label"] == "trash can"), None)
        check(bin_ is not None and math.hypot(bin_["x"] - 2.4, bin_["y"] - 0.0) < 0.3,
              f"goto's memory on the view: trash can at {bin_ and (bin_['x'], bin_['y'])} (room: 2.4, 0.0)")

        # drive: GOTO from the view, then poses while moving and the route
        rep = post(base + "/api/world/goto", {"label": "trash can"})
        check(rep.get("ok"), f"GOTO from the live view: {rep.get('reply')}")
        saw_route, moved, errs = False, False, []
        start_truth = world_state(ports["world_ctl"]).get("truth") or {}
        end = time.monotonic() + 60
        while time.monotonic() < end:
            st = json.loads(get(base + "/api/live/state")[1])
            tr = world_state(ports["world_ctl"]).get("truth") or {}
            lv = st.get("truth")
            if st.get("route"):
                saw_route = True
            if tr and start_truth and math.hypot(tr["x"] - start_truth["x"], tr["y"] - start_truth["y"]) > 0.3:
                moved = True
            if lv and tr:
                errs.append((math.hypot(lv["x"] - tr["x"], lv["y"] - tr["y"]),
                             math.degrees(angle_diff(lv["th"], tr["th"]))))
            g = st.get("goto") or {}
            if g.get("state") == "idle" and moved:
                break
            time.sleep(0.25)
        check(moved, "the robot drove (sim truth moved > 0.3 m)")
        check(saw_route, "the view got goto's route while it drove")
        # The stub car jumps a whole TURN at once and the live state is up to 0.25 s old, so a sample taken
        # across a jump can be 90° off by timing alone: judge the median, and print the worst for the record.
        dm = sorted(e[0] for e in errs)[len(errs) // 2] if errs else 9.0
        am = sorted(e[1] for e in errs)[len(errs) // 2] if errs else 999.0
        check(len(errs) > 10 and dm < 0.15 and am < 8,
              f"while driving: live pose follows the truth (median off {dm:.2f} m, {am:.0f}° over {len(errs)} samples; "
              f"worst {max((e[0] for e in errs), default=0):.2f} m, {max((e[1] for e in errs), default=0):.0f}° "
              f"= sampled across a stub TURN jump)")
        compare("after the GOTO", 0.2, 10)

        # token gate
        r = subprocess.run([sys.executable, os.path.join(BENCH, "bench.py"), "--target", "robot", "--bind", "0.0.0.0",
                            "--port", str(free_port())], capture_output=True, text=True, timeout=20)
        check(r.returncode == 2 and "--token" in r.stderr, "--bind 0.0.0.0 without --token refuses to start")
        tp = free_port()
        tb = subprocess.Popen([sys.executable, os.path.join(BENCH, "bench.py"), "--target", "robot", "--bind", "0.0.0.0",
                               "--port", str(tp), "--token", "tars-demo-token-123", "--plugins", "live",
                               "--motor", f"127.0.0.1:{ports['motor']}", "--goto", f"127.0.0.1:{ports['goto']}"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(tb)
        tbase = f"http://127.0.0.1:{tp}"
        end = time.monotonic() + 15
        code = 0
        while time.monotonic() < end:
            try:
                code = get(tbase + "/")[0]
                break
            except Exception:
                time.sleep(0.2)
        check(code == 401, f"no token -> {code}")
        check(get(tbase + "/panels.json")[0] == 401, "no token: the API is closed too")
        code, _, hdr = get(tbase + "/?token=tars-demo-token-123")
        cookie = (hdr.get("Set-Cookie") or "").split(";")[0]
        check(code == 200 and cookie.startswith("bench_token="), f"?token -> {code}, cookie set")
        check(get(tbase + "/panels.json", {"Cookie": cookie})[0] == 200, "cookie -> the API opens")
        check(get(tbase + "/?token=wrong-token-xxxx")[0] == 401, "wrong token -> 401")
    finally:
        for p in reversed(procs):
            try:
                if p is sim:
                    os.killpg(p.pid, 2)
                else:
                    p.terminate()
            except (OSError, ProcessLookupError):
                pass
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
    print(f"{ok}/{ok + fail} checks passed  (logs: {tmp})")
    return 1 if fail else 0


if __name__ == "__main__":
    sys.exit(main())
