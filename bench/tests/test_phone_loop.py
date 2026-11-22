#!/usr/bin/env python3
"""Phone in the loop, with a FAKE phone (no iPhone needed): bench/run_sim.py --phone tcp:127.0.0.1.

The fake (bench/phone/fake_phone.py) speaks MotionUSB's LDR1 (5562) and RGB1 (5563) wire formats. Through
the real pieces, built for the Mac: bench/phone/link.py -> lidar_guard -> tars_motor's guard gate, the
link's LDR1 tee -> goto's nav port (TCP), rgb_broker -> detect (if built with ORT + model) -> goto.
Checks:
  * goto NAV: pose=arkit at the fake phone's pose, packets counting, no bad packets, obstacle cells mapped,
    and the link passed full 1024-point frames (12332 B, over macOS's UDP datagram cap)
  * the pose follows when the phone "is carried" (fake pose moved)
  * tars_motor STATUS: lidar_guard's verdicts arrive (guard_age_ms small, reason not missing/stale)
  * rgb_broker serves the phone's latest frame; detect's frames reach goto (STATUS fps > 0) [skipped,
    and said so, if detect_host_phone or the model isn't there]
  * bench target phone: badge data (real=false, live=true), virtual car panel kept, /api/live/state alive
  * GOTO: goto's search TURN reaches the virtual car (the phone itself doesn't move)
  * STOP: /api/robot/stop sends S + CANCEL; the phone losing tracking -> goto pose leaves arkit
  * Ctrl-C (SIGINT) on run_sim: S + CANCEL, everything exits
"""
from __future__ import annotations

import json
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
sys.path.insert(0, os.path.join(BENCH, "phone"))
import fake_phone  # noqa: E402

fails = 0
skips = []


def check(c, m):
    global fails
    print(("  ok   " if c else "  FAIL ") + m, flush=True)
    fails += 0 if c else 1


def line(port, cmd, timeout=2.0) -> str:
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
        f = s.makefile("rw", encoding="ascii", newline="\n")
        f.write(cmd + "\n")
        f.flush()
        return f.readline().strip()


def kv(rep: str) -> dict:
    return dict(p.split("=", 1) for p in rep.split() if "=" in p)


def get(url, timeout=3):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read())


def post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def wait(fn, timeout, step=0.2):
    end = time.monotonic() + timeout
    v = None
    while time.monotonic() < end:
        try:
            v = fn()
            if v:
                return v
        except Exception:  # noqa: BLE001
            pass
        time.sleep(step)
    return v


def nav(ports) -> dict:
    rep = line(ports["goto"], "NAV")
    d = kv(rep)
    src, x, z, yaw = (d.get("pose", "none,0,0,0").split(",") + ["0"] * 4)[:4]
    d.update(src=src, x=float(x or 0), z=float(z or 0), yaw=float(yaw or 0))
    return d


def main() -> int:
    for exe in ("iter2-autonomy/goto/build/goto_host", "iter1-wasd/pi/build/tars_motor_host"):
        if not os.path.exists(os.path.join(REPO, exe)):
            print(f"build first: {exe} (make -C iter2-autonomy/goto host; make -C iter1-wasd/pi host)")
            return 1
    ph = fake_phone.FakePhone(points=1024)
    ph.pose = [0.40, -0.20, 0.0]
    tmp = tempfile.mkdtemp(prefix="tars-phone-")
    ports_file = os.path.join(tmp, "ports.json")
    log = open(os.path.join(tmp, "run_sim.log"), "w")
    sim = subprocess.Popen([sys.executable, os.path.join(BENCH, "run_sim.py"), "--phone", "tcp:127.0.0.1",
                            "--phone-ports", f"{ph.ldr_port},{ph.rgb_port}", "--free-ports", "--no-voice",
                            "--ports-file", ports_file], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        if not wait(lambda: os.path.exists(ports_file) or sim.poll() is not None, 90) or not os.path.exists(ports_file):
            print(open(log.name).read()[-3000:])
            check(False, "run_sim --phone came up")
            return 1
        info = json.load(open(ports_file))
        P, run_dir = info["ports"], info["run_dir"]
        check(all(k in info["pids"] for k in ("phone_link", "lidar_guard", "rgb_broker", "goto", "tars_motor", "bench"))
              and "world" not in info["pids"], f"run_sim --phone: link, lidar_guard, rgb_broker, goto, tars_motor, bench; "
              f"no world sim ({sorted(info['pids'])})")

        # --- LDR1 -> goto map + pose
        n = wait(lambda: (lambda d: d if d["src"] == "arkit" and int(d.get("packets", 0)) > 20 else None)(nav(P)), 20)
        check(bool(n) and abs(n["x"] - 0.40) < 0.02 and abs(n["z"] + 0.20) < 0.02,
              f"goto NAV pose=arkit at the phone's pose (got {n and (n['src'], n['x'], n['z'])}, want 0.40,-0.20)")
        n = nav(P)
        check(int(n.get("bad", 1)) == 0 and int(n.get("occ", 0)) > 0 and int(n.get("free", 0)) > 0,
              f"goto map from the phone's points: occ={n.get('occ')} free={n.get('free')} bad={n.get('bad')}")
        st = wait(lambda: (lambda d: d if d["nav_sent"] > 20 else None)(json.load(open(os.path.join(run_dir, "phone_link.json")))), 5)
        check(st["ldr_bytes_max"] == 4 + 44 + 1024 * 12 and st["nav_sent"] > 20,
              f"link passed full 1024-point frames ({st['ldr_bytes_max']} B) and teed {st['nav_sent']} to goto over TCP")
        ph.pose = [1.10, -0.60, 0.5]       # "carry the phone"
        n = wait(lambda: (lambda d: d if abs(d["x"] - 1.10) < 0.02 and abs(d["z"] + 0.60) < 0.02 else None)(nav(P)), 5)
        check(bool(n), f"pose follows the carried phone ({n and (n['x'], n['z'], n['yaw'])})")

        # --- lidar_guard -> tars_motor
        s = wait(lambda: (lambda d: d if d.get("guard_reason") not in (None, "missing", "stale")
                          and int(d.get("guard_age_ms", 9999)) < 500 else None)(kv(line(P["motor"], "STATUS"))), 10)
        check(bool(s), f"tars_motor gets lidar_guard verdicts (guard_reason={s and s.get('guard_reason')} "
                       f"age={s and s.get('guard_age_ms')} ms permit={s and s.get('guard_permit')})")

        # --- camera: broker, detect -> goto
        with socket.create_connection(("127.0.0.1", P["broker"]), timeout=2) as b:
            b.sendall(b"GET\n")
            hdr = b.recv(4)
            size = struct.unpack("!I", hdr)[0] if len(hdr) == 4 else 0
        check(size == 12 + len(ph.jpeg), f"rgb_broker serves the phone's latest RGB1 frame ({size} B)")
        if "detect" in info["pids"]:
            g = wait(lambda: (lambda d: d if float(d.get("fps", 0)) > 0 else None)(kv(line(P["goto"], "STATUS"))), 20)
            check(bool(g), f"detect (YOLO-World on ORT) frames reach goto: STATUS fps={g and g.get('fps')}")
        else:
            skips.append("detect not built / no model: camera -> goto not checked (make -C iter2-autonomy/goto host-detect-phone)")

        # --- bench
        base = f"http://127.0.0.1:{P['bench']}"
        pj = get(base + "/panels.json")
        check(pj["target"]["name"] == "phone" and pj["real"] is False and pj["live"] is True
              and "car.js" in pj["panels"] and "world2d.js" in pj["panels"],
              f"bench target phone: real=false, live=true, car + 2D view kept (hidden {pj.get('hidden')})")
        lv = wait(lambda: (lambda d: d if d.get("alive") and d.get("truth") else None)(get(base + "/api/live/state")), 10)
        check(bool(lv) and lv["truth"].get("src") == "arkit" and "iPhone" in lv["room"]["name"],
              f"2D view: goto's pose from the phone ({lv and lv['truth']}) in '{lv and lv['room']['name']}'")

        # --- goto drives the VIRTUAL car (the phone stands still: it's a search spin, so no pose check)
        def car_th():
            t = (get(base + "/api/car/state") or {}).get("truth") or {}
            return t.get("th", t.get("h"))
        th0 = wait(car_th, 5)
        rep = line(P["goto"], "GOTO trash can")
        moved = wait(lambda: (lambda th: th if th is not None and th0 is not None and abs(th - th0) > 0.05 else None)(car_th()), 15)
        check(rep.startswith("OK") and moved is not None,
              f"GOTO trash can ({rep}) -> goto's TURN reaches the virtual car (heading {th0} -> {moved})")

        # --- stop wins
        stops0 = int(kv(line(P["motor"], "STATUS")).get("stops", 0))
        r = post(base + "/api/robot/stop", {})
        stops1 = int(kv(line(P["motor"], "STATUS")).get("stops", 0))
        check(bool(r.get("S")) and bool(r.get("CANCEL")) and stops1 > stops0,
              f"STOP button: S + CANCEL (tars_motor stops {stops0} -> {stops1})")
        ph.tracking = False
        n = wait(lambda: (lambda d: d if d["src"] != "arkit" else None)(nav(P)), 5)
        check(bool(n), f"phone loses tracking -> goto stops using its pose (pose={n and n['src']})")
        ph.tracking = True
    finally:
        if sim.poll() is None:
            sim.send_signal(signal.SIGINT)
            try:
                sim.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(sim.pid, signal.SIGKILL)
        ph.close()
    out = open(log.name).read()
    ml = ""
    try:
        ml = open(os.path.join(run_dir, "tars_motor.log")).read()
    except (OSError, NameError):
        pass
    # run_sim's stop(): a 'run_sim' client sends S to tars_motor (then CANCEL to goto) before anything is killed
    check(sim.returncode == 0 and "S, CANCEL, stopping everything" in out and "is 'run_sim'" in ml[-3000:],
          f"Ctrl-C: S (+ CANCEL) from run_sim before shutdown, run_sim exited {sim.returncode}")
    for s in skips:
        print("  SKIP " + s)
    print(f"{'all checks passed' if not fails else f'{fails} FAILED'} (logs {tmp}, run dir {locals().get('run_dir')})")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
