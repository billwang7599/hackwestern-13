#!/usr/bin/env python3
"""plugins/car.py against a running virtual car (free ports), plus the bench shell
loading it. No tars_motor needed: this only checks the bridge and routes."""
from __future__ import annotations

import http.client
import importlib
import json
import os
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(BENCH, "car"))
sys.path.insert(0, BENCH)
import launch as car  # noqa: E402

fails = total = 0


def check(ok, what):
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


def free_port(kind=socket.SOCK_STREAM):
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def main() -> int:
    ctl = free_port(socket.SOCK_DGRAM)
    os.environ["BENCH_CAR_CTL"] = str(ctl)
    import bench as B
    port = free_port()
    bench = B.Bench(B.parse_args(["--port", str(port), "--plugins", "car"]))
    events = []
    bench.tap(lambda t, d: events.append((t, d)) if t == "car" else None)
    B.load_plugins(bench, ["car"])
    check("car" in bench.plugins, "bench loads plugins/car.py")
    srv = B.ThreadingHTTPServer(("127.0.0.1", port), B.make_handler(bench))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def req(method, path, body=None):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
        c.request(method, path, body=json.dumps(body) if body is not None else None)
        r = c.getresponse()
        return r.status, json.loads(r.read() or b"null")

    st, body = req("POST", "/api/car/fault", {"fault": "tip"})
    check(st == 503 and "no virtual car" in body.get("error", ""), "no car running: routes answer 503 with a hint")
    time.sleep(0.2)
    check(events and events[-1][1]["alive"] is False, "topic car says alive=false with no car")

    proc, dev = car.start("sm11", 0, ctl, 3, log=subprocess.DEVNULL)   # truth only via {"sub"}
    try:
        t0 = time.time()
        while time.time() - t0 < 4 and not (events and events[-1][1]["alive"] and events[-1][1]["params"]):
            time.sleep(0.05)
        ok = bool(events) and events[-1][1]["alive"]
        check(ok and events[-1][1]["params"]["gain_l"] == 1.0, f"subscribed: topic car alive with params in {time.time() - t0:.2f} s")
        n0 = len(events)
        time.sleep(1.0)
        rate = len(events) - n0
        check(12 <= rate <= 25, f"topic car at ~20 Hz ({rate}/s)")
        st, body = req("POST", "/api/car/set", {"gain_r": 0.9, "batt_v": 7.0})
        time.sleep(0.2)
        p = req("GET", "/api/car/state")[1]["params"]
        check(st == 200 and p["gain_r"] == 0.9 and p["batt_v"] == 7.0, "POST /api/car/set changes the live profile")
        st, body = req("POST", "/api/car/set", {"bogus": 1})
        check(st == 400 and "unknown key" in body.get("error", ""), "unknown profile key -> 400")
        req("POST", "/api/car/fault", {"fault": "tip"})
        time.sleep(0.3)
        t = req("GET", "/api/car/state")[1]["truth"]
        check(t["faults"]["tip"] and t["roll"] > 60, "POST /api/car/fault tip -> truth shows it")
        req("POST", "/api/car/fault", {"fault": "clear"})
        req("POST", "/api/car/pose", {"x": 1.5, "y": -0.5, "th": 1.0})
        time.sleep(0.3)
        t = req("GET", "/api/car/state")[1]["truth"]
        check(not t["faults"]["tip"] and abs(t["x"] - 1.5) < 1e-3 and abs(t["th"] - 1.0) < 1e-3, "clear + pose teleport")
        req("POST", "/api/car/button", {"button": "down"})
        time.sleep(0.2)
        t = req("GET", "/api/car/state")[1]["truth"]
        req("POST", "/api/car/button", {"button": "up"})
        check(t["btn"] == 1, "KEY button down reaches the firmware pin")
    finally:
        car.stop()
        mod = bench.plugins.get("car")
        if mod and hasattr(mod, "shutdown"):
            mod.shutdown()
        srv.shutdown()
    js = os.path.join(BENCH, "web", "panels", "car.js")
    try:
        r = subprocess.run(["node", "--check", js], capture_output=True, text=True)
        check(r.returncode == 0, "panels/car.js parses (node --check)" + (": " + r.stderr[:200] if r.returncode else ""))
    except FileNotFoundError:
        print("  ..   node not installed: panels/car.js syntax not checked")
    print(f"{total - fails}/{total} checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
