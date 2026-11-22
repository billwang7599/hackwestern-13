#!/usr/bin/env python3
"""Shell smoke test: static files, /panels.json, a plugin route, an SSE event."""
import http.client
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import bench as B  # noqa: E402

fails = 0
total = 0


def check(ok, what):
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what)


def main() -> int:
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    bench = B.Bench(B.parse_args(["--port", str(port), "--plugins", "none"]))
    bench.route("POST", "/api/t/echo", lambda q, body: (200, {"q": q, "n": len(body)}))
    bench.route("GET", "/api/t/boom", lambda q, body: 1 / 0)
    srv = B.ThreadingHTTPServer(("127.0.0.1", port), B.make_handler(bench))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def req(method, path, body=None):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
        c.request(method, path, body=body)
        r = c.getresponse()
        return r.status, r.read()

    st, body = req("GET", "/")
    check(st == 200 and b"shell.js" in body and b'id="stop"' in body, "index.html served, has STOP")
    st, body = req("GET", "/panels.json")
    check(st == 200 and b'"target"' in body, "/panels.json")
    # the live plugin's module loads on every target; only real/phone targets make the page LIVE
    sim = B.Bench(B.parse_args(["--port", "0", "--plugins", "live"]))
    B.load_plugins(sim, ["live"])
    check("live" in sim.plugins and B.bench_live(sim) is False, "sim target: live plugin loaded but panels.json live=false")
    st, body = req("POST", "/api/t/echo?a=1", b"xyz")
    check(st == 200 and b'"n": 3' in body and b'"a": "1"' in body, "plugin route gets query and body")
    st, body = req("GET", "/api/t/boom")
    check(st == 500 and b"ZeroDivisionError" in body, "a crashing route answers 500, server lives")
    st, _ = req("GET", "/../bench.py")
    check(st == 404, "no path traversal out of web/")
    st, _ = req("GET", "/api/nope")
    check(st == 404, "unknown route 404")

    bench.publish("status", {"x": 1})  # sticky: a tab that connects later gets it
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    c.request("GET", "/events")
    r = c.getresponse()
    got = b""
    t0 = time.time()
    threading.Timer(0.3, lambda: bench.publish("log", {"src": "t", "text": "hi"}, sticky=False)).start()
    while time.time() - t0 < 3 and b"hi" not in got:
        got += r.fp.read1(4096)
    check(b'event: status\ndata: {"x":1}' in got, "sticky event replayed to a new tab")
    check(b"event: log" in got and b'"hi"' in got, "live event delivered")
    srv.shutdown()
    print(f"{total - fails}/{total} checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
