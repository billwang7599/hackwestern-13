#!/usr/bin/env python3
"""robot link + drive + scenarios + recorder, against the real host builds on free ports:
tars_motor -b stub, goto, and mission_relay (for --target pi). Fakes only where a check needs
to see the wire (STOP order).

    make -C iter1-wasd/pi host && make -C iter2-autonomy/goto host && make -C iter2-autonomy/mission host
    python3 bench/tests/test_ui.py
"""
from __future__ import annotations

import hashlib
import http.client
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
sys.path.insert(0, BENCH)
sys.path.insert(0, os.path.join(REPO, "iter2-autonomy", "mission", "tests"))
import bench as B  # noqa: E402
import test_relay as tr  # noqa: E402  (free_port, wait_for, LineServer, Relay, FakeBroker)

MOTOR_BIN = os.path.join(REPO, "iter1-wasd", "pi", "build", "tars_motor_host")
GOTO_BIN = os.path.join(REPO, "iter2-autonomy", "goto", "build", "goto_host")
RELAY_BIN = os.path.join(REPO, "iter2-autonomy", "mission", "build", "mission_relay_host")

fails = total = 0


def check(ok, what):
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)
    return ok


def load(name: str, bench):
    spec = importlib.util.spec_from_file_location(f"bench_plugin_{name}", os.path.join(BENCH, "plugins", name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Tap:
    def __init__(self, bench):
        self.ev: list[tuple[float, str, object]] = []
        self.lock = threading.Lock()
        bench.tap(self)

    def __call__(self, topic, data):
        with self.lock:
            self.ev.append((time.monotonic(), topic, data))

    def of(self, topic, since=0.0):
        with self.lock:
            return [(t, d) for t, tp, d in self.ev if tp == topic and t >= since]


def stack(tmp):
    mp, gp, dp, np_ = (tr.free_port() for _ in range(4))
    logs = open(os.path.join(tmp, "stack.log"), "w")
    procs = [subprocess.Popen([MOTOR_BIN, "-b", "stub", "-p", str(mp), "-r", "0"], stderr=logs)]
    assert tr.wait_for(lambda: tr.port_open(mp), 3), "tars_motor didn't start"
    procs.append(subprocess.Popen([GOTO_BIN, "-m", f"127.0.0.1:{mp}", "-p", str(gp), "-u", str(dp), "-n", str(np_),
                                   "-r", "0", "-M", "-", "-o", os.path.join(REPO, "iter2-autonomy", "goto", "objects.txt")],
                                  stderr=logs))
    assert tr.wait_for(lambda: tr.port_open(gp), 3), "goto didn't start"
    return mp, gp, procs


def http_req(port, method, path, body=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    c.request(method, path, body=json.dumps(body) if body is not None else None)
    r = c.getresponse()
    return r.status, json.loads(r.read() or b"null")


def main() -> int:
    for b in (MOTOR_BIN, GOTO_BIN, RELAY_BIN):
        if not os.path.exists(b):
            print(f"missing {b}: build the hosts first (see the docstring)")
            return 2
    print(f"load: {os.getloadavg()[0]:.2f}")
    tmp = tempfile.mkdtemp(prefix="bench_ui_")
    procs = []
    try:
        mp, gp, procs = stack(tmp)
        vport = tr.free_port()      # nothing listens: voice must show as down, not break anything
        args = B.parse_args(["--motor", f"127.0.0.1:{mp}", "--goto", f"127.0.0.1:{gp}", "--voice", f"127.0.0.1:{vport}"])
        bench = B.Bench(args)
        tap = Tap(bench)
        rec = load("recorder", bench)
        rec.DIR = os.path.join(tmp, "recordings")
        robot = load("robot", bench)
        sc = load("scenarios", bench)
        for n, m in (("robot", robot), ("recorder", rec), ("scenarios", sc)):
            bench.plugins[n] = m
            m.setup(bench)
        hport = tr.free_port()
        srv = B.ThreadingHTTPServer(("127.0.0.1", hport), B.make_handler(bench))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        L = robot.link()

        print("-- link")
        check(tr.wait_for(lambda: L.chans["motor"].connected and L.chans["goto"].connected, 3), "motor + goto links up")
        check(tr.wait_for(lambda: robot.status() is not None, 2), "STATUS polled and parsed")
        st = robot.status() or {}
        check(st.get("backend") == "stub" and "move" in st and "guard_reason" in st, f"STATUS fields (backend={st.get('backend')})")
        check(st.get("clients", 0) >= 2, f"tars_motor sees bench + goto as clients ({st.get('clients')})")
        t0 = time.monotonic()
        check(robot.send("motor", "PING") == "OK PONG", "send(motor, PING) -> OK PONG")
        check(robot.send("goto", "PING").startswith("OK"), "send(goto, PING)")
        check(robot.send("voice", "PING").startswith("ERR voice link down"), "voice down -> clear ERR, nothing hangs")
        cmds = [d for _, d in tap.of("cmd", t0)]
        check(any(d["line"] == "PING" and d["reply"] == "OK PONG" and d["dest"] == "motor" for d in cmds), "cmd event has line + reply")
        check(not any(d["line"] == "STATUS" and d["src"] == "poll" for d in cmds), "10 Hz STATUS polls are not cmd events")
        s, r = http_req(hport, "POST", "/api/robot/send", {"dest": "motor", "line": "WHEELCAL"})
        check(s == 200 and r["reply"].startswith("OK 65.0"), f"POST /api/robot/send ({r})")
        check(tr.wait_for(lambda: any((d.get("status") or {}).get("state") == "idle" for _, d in tap.of("goto")), 2),
              "goto STATUS published")
        t1 = time.monotonic()
        time.sleep(1.0)
        n = len(tap.of("status", t1))
        check(7 <= n <= 12, f"status at ~10 Hz ({n} in 1 s)")

        print("-- WASD")
        t0 = time.monotonic()
        L.keys(["w"], 0.6)
        tr.wait_for(lambda: False, 0.25)
        L.keys(["w"], 0.6)                   # one heartbeat, then the tab "goes quiet"
        t_last = time.monotonic()
        got_s = tr.wait_for(lambda: any(d["line"] == "S" and d["src"] == "drive" for _, d in tap.of("cmd", t0)), 2)
        ds = [(t, d) for t, d in tap.of("cmd", t0) if d["src"] == "drive"]
        n_d = sum(1 for _, d in ds if d["line"].startswith("D "))
        t_s = next((t for t, d in ds if d["line"] == "S"), None)
        check(got_s, "quiet tab while W held -> server sends S")
        if t_s:
            # the S cmd event lands at reply time; the send is <= 20 ms after the 0.35 s mark
            check(0.33 <= t_s - t_last <= 0.6, f"S {1000 * (t_s - t_last):.0f} ms after the last key refresh (0.35 s rule)")
        check(n_d >= 8, f"D resent at ~20 Hz while held ({n_d} D in ~0.6 s)")
        check(all(d["line"] == "D 0.600 0.600" for _, d in ds if d["line"].startswith("D ")), "W at speed 0.6 -> D 0.600 0.600")
        check(tr.wait_for(lambda: (robot.status() or {}).get("out_l", 1) == 0, 1), "wheels at zero after the S")
        t0 = time.monotonic()
        L.keys(["a"], 0.5)
        L.keys([], 0.5)
        ds = [d["line"] for _, d in tap.of("cmd", t0) if d["src"] == "drive"]
        tr.wait_for(lambda: len([1 for _, d in tap.of("cmd", t0) if d["src"] == "drive"]) >= 2, 1)
        ds = [d["line"] for _, d in tap.of("cmd", t0) if d["src"] == "drive"]
        check(ds[:2] == ["D -0.500 0.500", "S"], f"A = spin left, release = S ({ds[:2]})")

        time.sleep(1.6)  # past tars_motor's 1.5 s hold after the S
        s, r = http_req(hport, "POST", "/api/robot/move", {"kind": "TURN", "amount": -90})
        check(s == 200 and r["reply"].startswith("OK "), f"POST /api/robot/move TURN -90 ({r})")
        mid = int(r["reply"].split()[1]) if r["reply"].startswith("OK ") else -1
        time.sleep(0.6)
        st = robot.status() or {}
        check(st.get("move_id") == mid and st.get("move") == "running",
              f"manual TURN still running 0.6 s later: idle goto not CANCELled into an S, keepalive PINGs ({st.get('move')} {st.get('move_why')})")
        robot.send("motor", "S", src="test")

        print("-- scenarios on the stub")
        want = {"link": "pass", "stop_latency": "pass", "watchdog": "pass", "stop_mid_move": "pass", "tip": "pass",
                "bump": "pass", "move_1000": "skip", "turn_90": "skip", "speedcal_move": "skip", "goto_trash": "skip",
                "wheel_direction": "skip"}
        for name, exp in want.items():
            time.sleep(0.6)  # tars_motor holds D for 1.5 s after an S from another client: keep apart
            res = sc.run(name, wait=True, timeout=30)
            nums = ", ".join(f"{k}={v['v']}" for k, v in (res.get("numbers") or {}).items())
            if not check(res.get("state") == exp, f"scenario {name}: {res.get('state')} (want {exp}) {nums}"):
                for s in res.get("steps", []):
                    print("        ", s["kind"], s["text"])
        s, r = http_req(hport, "GET", "/api/scenarios/list")
        check(s == 200 and len(r["scenarios"]) >= 12, f"/api/scenarios/list ({len(r['scenarios'])})")

        print("-- prompt + STOP aborts a scenario")
        # a scenario that asks: wheel_direction on 'real' prompts; force where=real with a tiny scenario
        @sc.scenario("t_prompt", "test: prompt then wait")
        def _t(c):
            v = c.ask("number please", "number")
            c.num("v", v)
            c.ask("now wait (STOP me)", "ok")
        sc.run("t_prompt")
        check(tr.wait_for(lambda: any((d.get("prompt") or {}).get("kind") == "number" for _, d in tap.of("scenario")), 2), "prompt published")
        http_req(hport, "POST", "/api/scenarios/answer", {"value": "42"})
        tr.wait_for(lambda: any((d.get("prompt") or {}).get("q", "").startswith("now wait") for _, d in tap.of("scenario")), 2)
        http_req(hport, "POST", "/api/robot/stop", {})
        tr.wait_for(lambda: sc._run.get("last") is not None, 3)
        last = sc._run.get("last") or {}
        check(last.get("state") == "aborted" and last["numbers"].get("v", {}).get("v") == 42.0, f"answer used, STOP aborted it ({last.get('state')})")

        print("-- recorder round trip")
        rec_path = rec._rec["path"]
        time.sleep(0.3)
        robot.send("motor", "BEEP 1", src="test")
        time.sleep(0.3)
        rec._rec["f"].flush()
        lines = [json.loads(l) for l in open(rec_path)]
        topics = {l["topic"] for l in lines}
        check({"status", "cmd", "goto", "scenario"} <= topics, f"recording has status/cmd/goto/scenario ({sorted(topics)})")
        check(any(l["topic"] == "cmd" and l["data"]["line"] == "BEEP 1" for l in lines), "commands are recorded")
        # take a short slice with commands in it, replay it and watch the wire
        sl = [l for l in lines if l["topic"] in ("cmd", "status")][-40:]
        t_base = sl[0]["t"]
        small = os.path.join(rec.DIR, "slice.jsonl")
        with open(small, "w") as f:
            for l in sl:
                f.write(json.dumps(dict(l, t=l["t"] - t_base)) + "\n")
        dur = sl[-1]["t"] - t_base
        sent: list[str] = []
        orig = robot.Chan._enqueue

        def spy(self, line, src, data=b""):
            sent.append(f"{self.dest}:{src}:{line}")
            return orig(self, line, src, data)
        robot.Chan._enqueue = spy
        n_before = len(lines)
        n_rec_before = rec._rec["n"]
        t0 = time.monotonic()
        s, r = http_req(hport, "POST", "/api/recorder/replay", {"name": "slice.jsonl", "speed": 2})
        check(s == 200, f"replay started ({r})")
        tr.wait_for(lambda: rec.replaying(), 1)
        live_during = []
        tr.wait_for(lambda: not rec.replaying(), dur / 2 + 3)
        robot.Chan._enqueue = orig
        replayed = [(t, tp, d) for t, tp, d in tap.ev if t >= t0 and isinstance(d, dict) and d.get("_replay")]
        live_status = [d for t, d in tap.of("status", t0) if not d.get("_replay")]
        check(len(replayed) >= len(sl) - 1, f"replay re-published {len(replayed)}/{len(sl)} events")
        check(any(tp == "cmd" for _, tp, _ in replayed), "replayed cmd events reach the UI")
        check(any(":poll:STATUS" in s for s in sent), f"the spy saw the live STATUS polls ({len(sent)} sends)")
        bad = [s for s in sent if not (s.endswith(":STATUS") and ":poll:" in s) and ":keepalive:" not in s]
        check(not bad, f"nothing but the STATUS poll went to the robot during replay ({bad[:5]})")
        check(len(live_status) <= 2, f"live status paused during replay ({len(live_status)} live)")
        rec._rec["f"].flush()
        new = [json.loads(l) for l in open(rec_path)][n_before:]
        check(not any(isinstance(l["data"], dict) and l["data"].get("_replay") for l in new), "replayed events not re-recorded")
        check(rec._rec["n"] >= n_rec_before, "recording continued")
        s, r = http_req(hport, "GET", "/api/recorder/file?name=../../bench.py")
        check(s == 404, "recorder file route refuses path traversal")

        print("-- STOP order (fake servers see the wire)")
        seen: list[tuple[float, str]] = []

        def fake(srv, c, line, buf):
            seen.append((time.monotonic(), ("M " if srv is fm else "G ") + line))
            if line == "STATUS" and srv is fm:
                return "OK 0.00 0.00 0 -1.00 0 why=none backend=fake move=idle"
            return "OK state=idle" if line == "STATUS" else "OK"
        fm = tr.LineServer(fake)
        fg = tr.LineServer(fake)
        b2 = B.Bench(B.parse_args(["--motor", f"127.0.0.1:{fm.port}", "--goto", f"127.0.0.1:{fg.port}",
                                   "--voice", f"127.0.0.1:{tr.free_port()}"]))
        L2 = robot.RobotLink(b2)
        tr.wait_for(lambda: L2.chans["motor"].connected and L2.chans["goto"].connected, 3)
        time.sleep(0.2)
        t0 = time.monotonic()
        res = L2.stop("test")
        tr.wait_for(lambda: any(l == "G CANCEL" for t, l in seen if t >= t0), 2)
        order = [l for t, l in seen if t >= t0 and l in ("M S", "G CANCEL")]
        check(res == {"S": True, "CANCEL": True}, f"stop() sent both ({res})")
        check(order[:2] == ["M S", "G CANCEL"], f"S reaches tars_motor before CANCEL reaches goto ({order})")
        check(any(l == "M WHO bench" for _, l in seen), "WHO bench on connect")
        L2.close()
        fm.close(); fg.close()

        print("-- send(dest, line, data): AUDIO <n> + bytes, direct TCP")
        clip = bytes(range(256)) * 1250 + b"\n\nOK\n"     # 320 KB, binary, with newlines inside
        sha = hashlib.sha1(clip).hexdigest()
        fv = tr.FakeVoice()
        b5 = B.Bench(B.parse_args(["--motor", f"127.0.0.1:{tr.free_port()}", "--goto", f"127.0.0.1:{tr.free_port()}",
                                   "--voice", f"127.0.0.1:{fv.port}"]))
        tap5 = Tap(b5)
        L5 = robot.RobotLink(b5)
        tr.wait_for(lambda: L5.chans["voice"].connected, 3)
        r = L5.send("voice", f"AUDIO {len(clip)}", clip)
        check(r == f"OK got {len(clip)}", f"AUDIO clip -> {r}")
        check(fv.audio[-1:] == [(len(clip), sha)], "clip arrived byte for byte")
        check(L5.send("voice", "HEAR go left").startswith("OK heard"), "next command still lines up after the clip")
        ev = [d for _, d in tap5.of("cmd") if d["line"].startswith("AUDIO")]
        check(ev and ev[-1].get("bytes") == len(clip) and len(json.dumps(ev[-1])) < 300,
              f"cmd event has the byte count, not the bytes ({ev[-1] if ev else None})")
        L5.close()

        print("-- --target pi through the real mission_relay")
        broker = tr.FakeBroker()
        rl = tr.Relay(RELAY_BIN, mp, gp, fv.port, broker.port)
        b3 = B.Bench(B.parse_args(["--target", "pi", "--relay", f"127.0.0.1:{rl.port}", "--key", tr.KEY]))
        L3 = robot.RobotLink(b3)
        ok = tr.wait_for(lambda: L3.chans["motor"].connected and L3.chans["goto"].connected, 4)
        check(ok, f"relay MOTOR + GOTO channels up ({L3.link_state()})")
        check(L3.send("motor", "PING") == "OK PONG", "PING through the relay")
        check(tr.wait_for(lambda: L3.status() is not None, 2), "STATUS polled through the relay")
        check(L3.send("goto", "STATUS").startswith("OK state="), "goto STATUS through the relay")
        tr.wait_for(lambda: L3.chans["voice"].connected, 3)
        r = L3.send("voice", f"AUDIO {len(clip)}", clip)
        check(r == f"OK got {len(clip)}" and fv.audio[-1:] == [(len(clip), sha)],
              f"AUDIO clip through the relay's VOICE pipe, intact ({r})")
        check(L3.send("voice", "HEAR stop").startswith("OK heard"), "relay: next command lines up after the clip")
        r = L3.stop("relay test")
        check(r["S"] and r["CANCEL"], f"stop through the relay ({r})")
        b4 = B.Bench(B.parse_args(["--target", "pi", "--relay", f"127.0.0.1:{rl.port}", "--key", "wrong"]))
        L4 = robot.RobotLink(b4)
        time.sleep(1.0)
        check(not L4.chans["motor"].connected and "key" in str(L4.links["motor"].error).lower(),
              f"wrong key: link stays down and says why ({L4.links['motor'].error})")
        L3.close(); L4.close(); rl.close(); broker.close(); fv.srv.close()

        robot.shutdown()
        rec.shutdown()
        srv.shutdown()
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()
        shutil.rmtree(tmp, ignore_errors=True)
    print(f"load: {os.getloadavg()[0]:.2f}")
    print(f"{total - fails}/{total} checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
