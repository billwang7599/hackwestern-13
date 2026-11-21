#!/usr/bin/env python3
"""Bring up the whole simulated robot on this Mac, then the bench. Laptop only.

    python3 bench/run_sim.py                    # real port numbers on 127.0.0.1, room "default"
    python3 bench/run_sim.py --room demo_detour --free-ports
    python3 bench/run_sim.py --car stub         # force the stub car

  car          virtual SM11 (bench/car/launch.py, real firmware on a pty) if present,
               else the STUB CAR (tars_motor -b stub + bench/world/stub_car.py, no physics)
  tars_motor   iter1-wasd/pi host build, -b uno -d <pty> (or -b stub), guard gate on -g
  goto         iter2-autonomy/goto host build: control, detections, nav feed, memory (seeded
               from the room's "memory")
  tars_voice   iter2-autonomy/voice host build, laptop text/push-to-talk port, mic off
               (only if built and the whisper model exists; see the table)
  mission_relay  only with --relay
  world sim    bench/world/worldsim.py: phone pose + LiDAR guard + nav feed + YOLO, room edits
  bench        bench/bench.py --target sim, with every port

Ctrl-C: S to tars_motor and CANCEL to goto first, then every child is stopped.
Everything is a child in its own session; logs go to a temp run directory (printed).
"""
from __future__ import annotations

import argparse
import json
import os
import secrets
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(HERE, "world"))
import common as C  # noqa: E402

PI = os.path.join(REPO, "iter1-wasd", "pi")
GOTO_DIR = os.path.join(REPO, "iter2-autonomy", "goto")
VOICE_DIR = os.path.join(REPO, "iter2-autonomy", "voice")
MISSION = os.path.join(REPO, "iter2-autonomy", "mission")
MOTOR_BIN = os.path.join(PI, "build", "tars_motor_host")
GOTO_BIN = os.path.join(GOTO_DIR, "build", "goto_host")
VOICE_BIN = os.path.join(VOICE_DIR, "build", "tars_voice_host")
RELAY_BIN = os.path.join(MISSION, "build", "mission_relay_host")
VOICE_MODEL = os.path.join(VOICE_DIR, "models", "ggml-tiny.en-q5_1.bin")

REAL_PORTS = {"motor": 5555, "goto": 5570, "det": 5571, "nav": 5572, "voice": 5574, "relay": 5575,
              "guard": 5568, "truth": 5590, "car_ctl": 5591, "world_ctl": 5592, "world_state": 5593,
              "bench": 8770, "relay_tee": 5576, "phone_room": 5594, "phone_scene": 5595}


def free_ports(names) -> dict:
    """ephemeral ports free for both TCP and UDP, all different"""
    out, held = {}, []
    for n in names:
        while True:
            s = socket.socket()
            s.bind(("127.0.0.1", 0))
            p = s.getsockname()[1]
            u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                u.bind(("127.0.0.1", p))
            except OSError:
                s.close()
                continue
            held += [s, u]
            out[n] = p
            break
    for s in held:
        s.close()
    return out


def port_open(p: int) -> bool:
    try:
        socket.create_connection(("127.0.0.1", p), timeout=0.3).close()
        return True
    except OSError:
        return False


def wait_for(fn, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if fn():
            return True
        time.sleep(0.05)
    return False


def line_cmd(port: int, lines: list[str], timeout=2.0) -> list[str]:
    """one short connection: send lines, read one reply line each"""
    out = []
    with socket.create_connection(("127.0.0.1", port), timeout=timeout) as s:
        f = s.makefile("rw", encoding="ascii", newline="\n")
        for ln in lines:
            f.write(ln + "\n")
            f.flush()
            out.append(f.readline().strip())
    return out


class Stack:
    def __init__(self, a):
        self.a = a
        self.procs: list[tuple[str, subprocess.Popen]] = []
        self.rows: list[tuple[str, str, str]] = []
        self.run_dir = tempfile.mkdtemp(prefix="tars-bench-")
        self.ports = free_ports(REAL_PORTS) if a.free_ports else dict(REAL_PORTS)
        if a.bench_port:
            self.ports["bench"] = a.bench_port
        self.room = C.load_room_file(a.room)
        self.mem_path = os.path.join(self.run_dir, "goto_memory.txt")
        self.car_kind = "none"
        self.voice_note = ""
        self.lock = threading.Lock()
        self.goto_proc = None

    # ------------------------------------------------------------ helpers
    def spawn(self, name: str, argv: list[str], **kw) -> subprocess.Popen:
        log = open(os.path.join(self.run_dir, f"{name}.log"), "a")   # append: goto restarts on room loads
        p = subprocess.Popen(argv, stdout=kw.pop("stdout", log), stderr=log, start_new_session=True,
                             stdin=kw.pop("stdin", subprocess.DEVNULL), **kw)
        self.procs.append((name, p))
        return p

    def must(self, ok: bool, what: str, logname: str):
        if not ok:
            path = os.path.join(self.run_dir, f"{logname}.log")
            tail = ""
            try:
                tail = "".join(open(path).readlines()[-8:])
            except OSError:
                pass
            raise RuntimeError(f"{what} (log {path}):\n{tail}")

    def build(self):
        if self.a.no_build:
            return
        jobs = [["make", "-s", "-C", PI, "host"], ["make", "-s", "-C", GOTO_DIR, "host"]]
        if self.a.relay:
            jobs.append(["make", "-s", "-C", MISSION, "host"])
        for j in jobs:
            r = subprocess.run(j, capture_output=True, text=True)
            if r.returncode != 0:
                raise RuntimeError(f"build failed: {' '.join(j)}\n{r.stdout[-1500:]}{r.stderr[-1500:]}")

    # ------------------------------------------------------------ bring-up
    def start(self):
        P = self.ports
        self.build()
        # 1. car: the virtual SM11 if its launcher exists, else the stub
        pty = None
        if self.a.car in ("auto", "virtual"):
            launch = os.path.join(HERE, "car", "launch.py")
            if os.path.exists(launch):
                try:
                    sys.path.insert(0, os.path.join(HERE, "car"))
                    import launch as car_launch  # bench/car/launch.py (virtual car part)
                    # interface (lead, kothar/bench-car f6b29d0): start(profile, truth, ctl, seed) -> (proc, dev);
                    # dev is a symlink that survives a simulated unplug, so tars_motor gets the link
                    carlog = open(os.path.join(self.run_dir, "car.log"), "w")
                    proc, pty = car_launch.start(self.a.profile, P["truth"], P["car_ctl"], self.a.seed, log=carlog)
                    self.car_launch = car_launch
                    self.procs.append(("car", proc))
                    self.car_kind = "virtual"
                    self.rows.append(("virtual SM11 car", f"pty {pty}", f"truth UDP {P['truth']}, ctl UDP {P['car_ctl']}"))
                except Exception as e:
                    if self.a.car == "virtual":
                        raise
                    print(f"run_sim: virtual car failed ({e!r}); using the STUB CAR", file=sys.stderr)
            elif self.a.car == "virtual":
                raise RuntimeError("--car virtual: bench/car/launch.py not in this tree")
        if pty is None:
            self.car_kind = "stub"
        # 2. tars_motor
        backend = ["-b", "uno", "-d", pty] if pty else ["-b", "stub"]
        self.spawn("tars_motor", [MOTOR_BIN, *backend, "-p", str(P["motor"]), "-r", "0", "-l", "-g", str(P["guard"])])
        self.must(wait_for(lambda: port_open(P["motor"]), 8), "tars_motor didn't open its port", "tars_motor")
        if pty:     # the firmware says READY ~1.6 s after the port opens
            ok = wait_for(lambda: "ready=1" in " ".join(line_cmd(P["motor"], ["STATUS"])), 10)
            self.must(ok, "tars_motor never saw READY from the virtual Uno", "tars_motor")
        self.rows.append(("tars_motor", f"TCP {P['motor']}", f"-b {'uno ' + pty if pty else 'stub'}, guard GAT1 UDP {P['guard']}"))
        if self.car_kind == "stub":
            if not self.a.slow_stub:     # the stub's stepper timing is ~41 mm/s; this is ~163 mm/s, about the SM11 cap
                line_cmd(P["motor"], ["WHO run_sim", "WHEELCAL 260 400"])
            self.spawn("stub_car", [sys.executable, os.path.join(HERE, "world", "stub_car.py"), "--motor", str(P["motor"]),
                                    "--truth", str(P["truth"]), "--ctl", str(P["car_ctl"]), "--room", self.a.room])
            self.rows.append(("STUB CAR (no physics)", f"UDP {P['car_ctl']}", f"tars_motor pose -> truth UDP {P['truth']}"
                              + ("" if self.a.slow_stub else ", WHEELCAL 260 400")))
        # 3. goto
        self.start_goto(C.goto_memory_text(self.room))
        # 4. world sim
        truth_copy = free_ports(["x"])["x"]
        self.ports["truth_copy"] = truth_copy
        self.spawn("world", [sys.executable, os.path.join(HERE, "world", "worldsim.py"), "--room", self.a.room,
                             "--truth", str(P["truth"]), "--truth-fwd", f"127.0.0.1:{truth_copy}",
                             "--car-ctl", str(P["car_ctl"]), "--ctl", str(P["world_ctl"]),
                             "--state-to", f"127.0.0.1:{P['world_state']}", "--guard", str(P["guard"]),
                             "--nav", str(P["nav"]), "--det", str(P["det"]), "--seed", str(self.a.seed)])
        self.must(wait_for(lambda: port_open(P["world_ctl"]), 8), "world sim didn't open its control port", "world")
        self.rows.append(("world sim", f"TCP+UDP {P['world_ctl']}", f"room {self.room['name']}, state UDP {P['world_state']}"))
        # 5. voice (optional: needs the host build + model; building fetches whisper.cpp)
        model = self.a.voice_model or os.environ.get("TARS_VOICE_MODEL") or VOICE_MODEL
        if self.a.no_voice:
            self.voice_note = "off (--no-voice)"
        elif not os.path.exists(VOICE_BIN):
            self.voice_note = "not built (make -C iter2-autonomy/voice host; it fetches whisper.cpp)"
        elif not os.path.exists(model):
            self.voice_note = f"no model at {os.path.relpath(model, REPO)} (iter2-autonomy/voice/get_model.sh)"
        else:
            self.spawn("tars_voice", [VOICE_BIN, "--input", "raw:-", "--model", model, "--port", str(P["motor"]),
                                      "--goto-port", str(P["goto"]), "--net-port", str(P["voice"]), "--say-port", "0"],
                       stdin=subprocess.PIPE)
            if wait_for(lambda: port_open(P["voice"]), 30):
                self.rows.append(("tars_voice", f"TCP {P['voice']}", "laptop text / push-to-talk, mic off"))
            else:
                self.voice_note = "started but its net port never opened (see log)"
        if self.voice_note:
            self.rows.append(("tars_voice", "-", self.voice_note))
        # 6. mission_relay (optional)
        self.key = ""
        if self.a.relay:
            self.key = secrets.token_hex(16)
            self.spawn("mission_relay", [RELAY_BIN, "-p", str(P["relay"]), "-b", "127.0.0.1", "-k", self.key,
                                         "--motor", str(P["motor"]), "--goto", str(P["goto"]), "--voice", str(P["voice"]),
                                         "--rgb", "0", "--tee", str(P["relay_tee"]), "-v"])
            self.must(wait_for(lambda: port_open(P["relay"]), 8), "mission_relay didn't open its port", "mission_relay")
            self.rows.append(("mission_relay", f"TCP {P['relay']}", f"key {self.key}"))
        # 7. supervisor (the world plugin asks it to restart goto when a room with memory loads)
        self.sup = socket.socket()
        self.sup.bind(("127.0.0.1", 0))
        self.sup.listen(4)
        self.ports["supervisor"] = self.sup.getsockname()[1]
        threading.Thread(target=self._sup_loop, daemon=True).start()
        # 8. bench
        if not self.a.no_bench:
            env = dict(os.environ, BENCH_WORLD_CTL=str(P["world_ctl"]), BENCH_WORLD_STATE=str(P["world_state"]),
                       BENCH_SUPERVISOR=str(self.ports["supervisor"]), BENCH_CAR_CTL=str(P["car_ctl"]),
                       BENCH_CAR_TRUTH=str(truth_copy), BENCH_CAR_KIND=self.car_kind,
                       TARS_BENCH_PHONE_ROOM_PORT=str(P["phone_room"]), TARS_BENCH_PHONE_SCENE_PORT=str(P["phone_scene"]))
            argv = [sys.executable, os.path.join(HERE, "bench.py"), "--port", str(P["bench"]), "--target", "sim",
                    "--motor", f"127.0.0.1:{P['motor']}", "--goto", f"127.0.0.1:{P['goto']}",
                    "--voice", f"127.0.0.1:{P['voice']}"]
            self.spawn("bench", argv, env=env)
            self.must(wait_for(lambda: port_open(P["bench"]), 8), "bench didn't open its port", "bench")
            self.rows.append(("bench", f"http://127.0.0.1:{P['bench']}/", "target sim"))
            self.rows.append(("iPhone 3D map (bench)", f"HTTP {P['phone_room']}", f"room view; scene source TCP {P['phone_scene']}"))

    def start_goto(self, mem_text: str | None):
        P = self.ports
        if mem_text:
            with open(self.mem_path, "w") as f:
                f.write(mem_text)
        elif os.path.exists(self.mem_path):
            os.remove(self.mem_path)
        p = self.spawn("goto", [GOTO_BIN, "-m", f"127.0.0.1:{P['motor']}", "-p", str(P["goto"]), "-u", str(P["det"]),
                                "-n", str(P["nav"]), "-r", "0", "-M", self.mem_path, "-v",
                                "-o", os.path.join(GOTO_DIR, "objects.txt"), "-c", os.path.join(GOTO_DIR, "goto.conf")])
        self.goto_proc = p
        self.must(wait_for(lambda: port_open(P["goto"]), 8), "goto didn't open its port", "goto")
        if not any(r[0] == "goto" for r in self.rows):
            self.rows.append(("goto", f"TCP {P['goto']}", f"detections UDP {P['det']}, nav TCP {P['nav']}, memory {self.mem_path}"))

    def restart_goto(self, mem_text: str | None) -> dict:
        """new room: S first, a fresh goto (memory = the room's, empty map), tars_motor pose reset"""
        P = self.ports
        with self.lock:
            try:
                line_cmd(P["motor"], ["WHO run_sim", "S"])
            except OSError:
                pass
            old = self.goto_proc
            self.procs = [(n, p) for n, p in self.procs if p is not old]
            if old:
                old.terminate()
                try:
                    old.wait(3)
                except subprocess.TimeoutExpired:
                    old.kill()
            wait_for(lambda: not port_open(P["goto"]), 3)
            self.start_goto(mem_text)
            try:
                rep = line_cmd(P["motor"], ["WHO run_sim", "POSE RESET"])[-1]
            except OSError as e:
                rep = f"ERR {e}"
        return {"ok": True, "pose_reset": rep, "memory": bool(mem_text)}

    def _sup_loop(self):
        while True:
            try:
                c, _ = self.sup.accept()
            except OSError:
                return
            try:
                f = c.makefile("rw", encoding="utf-8", newline="\n")
                msg = json.loads(f.readline() or "{}")
                if msg.get("op") == "restart_goto":
                    rep = self.restart_goto(msg.get("memory"))
                elif msg.get("op") == "info":
                    rep = {"ok": True, "ports": self.ports, "car": self.car_kind, "run_dir": self.run_dir}
                else:
                    rep = {"ok": False, "error": "ops: restart_goto, info"}
            except Exception as e:
                rep = {"ok": False, "error": f"{type(e).__name__}: {e}"}
            try:
                f.write(json.dumps(rep) + "\n")
                f.flush()
            except OSError:
                pass
            c.close()

    # ------------------------------------------------------------ shutdown
    def stop(self):
        P = self.ports
        try:                                         # stop always wins: wheels first
            line_cmd(P["motor"], ["WHO run_sim", "S"], timeout=1)
        except OSError:
            pass
        try:
            line_cmd(P["goto"], ["CANCEL"], timeout=1)
        except OSError:
            pass
        for name, p in reversed(self.procs):
            if p.poll() is None:
                try:
                    p.terminate()
                except OSError:
                    pass
        end = time.monotonic() + 4
        for name, p in reversed(self.procs):
            try:
                p.wait(max(0.1, end - time.monotonic()))
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(2)
        cl = getattr(self, "car_launch", None)
        if cl is not None:
            try:
                cl.stop()
            except Exception:
                pass
        try:
            self.sup.close()
        except (OSError, AttributeError):
            pass

    def table(self) -> str:
        w = max(len(r[0]) for r in self.rows)
        w2 = max(len(r[1]) for r in self.rows)
        car = "STUB CAR: tars_motor -b stub, no firmware, no physics" if self.car_kind == "stub" else "virtual SM11 (real firmware)"
        out = [f"TARS sim stack (SIM, not the robot), car = {car}", ""]
        out += [f"  {a:<{w}}  {b:<{w2}}  {c}" for a, b, c in self.rows]
        out += ["", f"  logs: {self.run_dir}", "  Ctrl-C: S + CANCEL, then everything stops"]
        return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="TARS sim stack + bench (laptop only)")
    ap.add_argument("--room", default="default", help=f"rooms: {', '.join(C.list_rooms())}")
    ap.add_argument("--car", choices=["auto", "virtual", "stub"], default="auto")
    ap.add_argument("--profile", default="sm11_2wd",
                    help="virtual car profile (bench/car/profiles/<name>.json): sm11_2wd = the demo default (2 motors), sm11 = the 4WD kit")
    ap.add_argument("--free-ports", action="store_true", help="ephemeral ports (tests; several stacks at once)")
    ap.add_argument("--bench-port", type=int, default=0)
    ap.add_argument("--no-bench", action="store_true")
    ap.add_argument("--no-voice", action="store_true")
    ap.add_argument("--voice-model", default="")
    ap.add_argument("--relay", action="store_true", help="also run mission_relay (random key, printed)")
    ap.add_argument("--slow-stub", action="store_true", help="stub car at the stepper's real ~41 mm/s")
    ap.add_argument("--no-build", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--ports-file", default="", help="write ports + pids as JSON here once everything is up")
    a = ap.parse_args(argv)

    st = Stack(a)
    done = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: done.set())
    signal.signal(signal.SIGINT, lambda *_: done.set())
    try:
        st.start()
    except Exception as e:
        print(f"run_sim: {e}", file=sys.stderr)
        st.stop()
        return 1
    print(st.table(), flush=True)
    if a.ports_file:
        with open(a.ports_file + ".tmp", "w") as f:
            json.dump({"ports": st.ports, "car": st.car_kind, "run_dir": st.run_dir,
                       "pids": {n: p.pid for n, p in st.procs}, "voice": st.voice_note or "up"}, f)
        os.replace(a.ports_file + ".tmp", a.ports_file)
    try:
        while not done.is_set():
            dead = []
            if st.lock.acquire(blocking=False):          # not mid goto restart
                dead = [(n, p.returncode) for n, p in st.procs if p.poll() is not None]
                st.lock.release()
            if dead:
                print(f"run_sim: {dead[0][0]} exited ({dead[0][1]}): stopping (logs {st.run_dir})", file=sys.stderr)
                break
            done.wait(0.5)
    finally:
        print("run_sim: S, CANCEL, stopping everything", file=sys.stderr, flush=True)
        st.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
