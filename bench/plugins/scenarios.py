"""bench plugin `scenarios`: scripted checks against the robot, with pass/fail and numbers.

Adding a scenario = one decorated function in this file:

    @scenario("my_check", "What it checks", needs="any"|"sim"|"car")
    def my_check(c: Ctx):
        r = c.send("motor", "PING")
        c.expect(r == "OK PONG", f"PING -> {r}")
        c.num("ping_ms", 1.2, "ms")

Ctx gives: send(dest, line), status() (latest tars_motor STATUS, dict), wait(pred, timeout),
wait_move(id) (our MOVE/TURN to finish), truth() (virtual car truth from the `car` topic, or None),
car_control(msg) (UDP car control, contract port 5591 / the car plugin), ask(question, kind)
(prompt the user in the panel: kind "yesno" | "number" | "ok"), note(text), num(name, v, unit),
expect(ok, text), skip(why). Every step is published live as topic `scenario`.

Where it runs (c.where): "sim-car" (the virtual SM11: car truth is flowing), "stub" (tars_motor -b
stub: commands and safety paths only, the stub never finishes a MOVE), "real" (a real Uno /
car: measurements are prompts). Scenarios that need what isn't there SKIP and say why; nothing
here pretends a stub or a sim proves what the car does.

STOP (button or Space) aborts the running scenario: the robot plugin's stop() runs as usual and
the scenario sees it at its next step.
"""
from __future__ import annotations

import json
import math
import socket
import threading
import time
import traceback

_bench = None
SCENARIOS: dict[str, dict] = {}
_run: dict = {"name": None, "thread": None}
_answer = {"ev": threading.Event(), "value": None}
_truth = {"t": 0.0, "d": None}
CAR_CONTROL_PORT = 5591


def scenario(name: str, title: str, needs: str = "any", psec: str = ""):
    """needs: any (stub works), car (sim-car or real: the stub can't do it), sim (sim-car only).
    psec: the docs/PI_TEST.md check it maps to."""
    def deco(fn):
        SCENARIOS[name] = {"fn": fn, "title": title, "needs": needs, "pi_test": psec, "doc": (fn.__doc__ or "").strip()}
        return fn
    return deco


class Aborted(Exception):
    pass


class Skip(Exception):
    pass


class Ctx:
    def __init__(self, bench, name: str):
        self.bench, self.name = bench, name
        self.robot = bench.plugins.get("robot")
        self.steps: list[dict] = []
        self.numbers: dict[str, dict] = {}
        self.ok = True
        self.stops0 = self.robot.stop_count() if self.robot else 0
        self.t0 = time.time()
        st = self.status(wait=1.0)
        self.backend = (st or {}).get("backend", "?")
        if self.truth(max_age=1.0) is not None:
            self.where = "sim-car"
        elif self.backend == "stub":
            self.where = "stub"
        else:
            self.where = "real"

    # ---- reporting
    def _pub(self, state: str = "running", prompt: dict | None = None) -> None:
        self.bench.publish("scenario", {"name": self.name, "state": state, "where": self.where,
                                        "backend": self.backend, "steps": self.steps[-60:], "numbers": self.numbers,
                                        "prompt": prompt, "elapsed": round(time.time() - self.t0, 1)})

    def note(self, text: str) -> None:
        self.steps.append({"t": round(time.time() - self.t0, 2), "kind": "note", "text": text})
        self._pub()

    def expect(self, ok: bool, text: str) -> bool:
        self.steps.append({"t": round(time.time() - self.t0, 2), "kind": "pass" if ok else "fail", "text": text})
        self.ok = self.ok and bool(ok)
        self._pub()
        return bool(ok)

    def num(self, name: str, value, unit: str = "") -> None:
        self.numbers[name] = {"v": round(value, 3) if isinstance(value, float) else value, "unit": unit}
        self._pub()

    def skip(self, why: str):
        raise Skip(why)

    def check_abort(self) -> None:
        if self.robot and self.robot.stop_count() != self.stops0:
            raise Aborted("STOP pressed")
        if _run.get("cancel"):
            raise Aborted("cancelled")

    # ---- robot
    def send(self, dest: str, line: str, timeout: float = 3.0) -> str:
        self.check_abort()
        if not self.robot:
            raise Skip("robot plugin not loaded")
        r = self.robot.send(dest, line, timeout=timeout, src=f"scenario:{self.name}")
        self.steps.append({"t": round(time.time() - self.t0, 2), "kind": "cmd", "text": f"{dest}> {line} -> {r[:160]}"})
        self._pub()
        return r

    def status(self, wait: float = 0.0) -> dict | None:
        end = time.monotonic() + wait
        while True:
            st = self.robot.status() if self.robot else None
            if st is not None or time.monotonic() >= end:
                return st
            time.sleep(0.05)

    def fresh_status(self) -> dict | None:
        """STATUS asked right now (not the 10 Hz poll's copy)."""
        from_robot = self.send("motor", "STATUS", 1.0) if self.robot else ""
        parse = getattr(self.robot, "parse_status", None)
        return parse(from_robot) if parse else None

    def wait(self, pred, timeout: float, every: float = 0.02):
        """poll pred() until truthy (returns its value) or timeout (returns None)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check_abort()
            v = pred()
            if v:
                return v
            time.sleep(every)
        return None

    def wait_move(self, move_id: int, timeout: float) -> dict:
        """until STATUS says our move is no longer running. The robot plugin keeps it alive (PING)."""
        def done():
            st = self.robot.status()
            if st and (st.get("move_id") != move_id or st.get("move") != "running"):
                return st
            return None
        st = self.wait(done, timeout, every=0.05)
        return st or {"move": "timeout"}

    def start_move(self, line: str) -> tuple[int, int]:
        r = self.send("motor", line)
        if not r.startswith("OK "):
            raise Skip(f"{line} refused: {r}")
        p = r.split()
        return int(p[1]), int(p[2])

    # ---- virtual car
    def truth(self, max_age: float = 0.5) -> dict | None:
        d = _truth["d"]
        if d is None or time.monotonic() - _truth["t"] > max_age:
            return None
        return d

    def car_control(self, msg: dict) -> bool:
        car = self.bench.plugins.get("car")
        for fn_name in ("control", "send_control"):
            fn = getattr(car, fn_name, None) if car else None
            if callable(fn):
                fn(msg)
                return True
        port = getattr(car, "CONTROL_PORT", None) or getattr(self.bench.args, "car_control", None) or CAR_CONTROL_PORT
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(json.dumps(msg).encode(), ("127.0.0.1", int(port)))
            s.close()
            return True
        except OSError:
            return False

    # ---- the user
    def ask(self, question: str, kind: str = "ok", timeout: float = 600.0):
        """Prompt in the panel; blocks until answered. kind: ok | yesno | number."""
        _answer["ev"].clear()
        _answer["value"] = None
        self.steps.append({"t": round(time.time() - self.t0, 2), "kind": "ask", "text": question})
        self._pub(prompt={"q": question, "kind": kind})
        end = time.monotonic() + timeout
        while not _answer["ev"].wait(0.1):
            self.check_abort()
            if time.monotonic() > end:
                raise Aborted("no answer")
        v = _answer["value"]
        self.steps.append({"t": round(time.time() - self.t0, 2), "kind": "note", "text": f"answer: {v}"})
        self._pub()
        if kind == "number":
            try:
                return float(v)
            except (TypeError, ValueError):
                raise Aborted(f"not a number: {v}")
        if kind == "yesno":
            return str(v).lower() in ("y", "yes", "true", "1")
        return v


def _truth_dist(a: dict, b: dict) -> float:
    return math.hypot(b["x"] - a["x"], b["y"] - a["y"]) * 1000.0


def _wrap_deg(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def _measure_move(c: Ctx, line: str, kind: str, asked: float) -> dict:
    """run a MOVE/TURN and measure it: truth on the sim car, the user's tape on the real one."""
    a = c.truth()
    mid, est = c.start_move(line)
    c.num("est_ms", est, "ms")
    t0 = time.monotonic()
    st = c.wait_move(mid, est / 1000.0 * 2 + 5)
    c.num("took_ms", round((time.monotonic() - t0) * 1000), "ms")
    c.expect(st.get("move") == "done", f"move ended: {st.get('move')} why={st.get('move_why')}")
    c.num("reported", st.get("move_done", float("nan")), "mm" if kind == "MOVE" else "deg")
    out = {"st": st}
    if c.where == "sim-car":
        time.sleep(0.3)  # let it coast to rest
        b = c.truth()
        if kind == "MOVE":
            true = _truth_dist(a, b) * (1 if asked >= 0 else -1)
            # sideways drift relative to the start heading
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            lat = (-math.sin(a["th"]) * dx + math.cos(a["th"]) * dy) * 1000
            c.num("lateral_mm", lat, "mm (+ = left)")
        else:
            true = -_wrap_deg(math.degrees(b["th"] - a["th"]))  # TURN + = right = clockwise = -th
        out["true"] = true
    elif c.where == "real":
        out["true"] = c.ask(f"{line}: how far did it REALLY go? ({'mm, tape' if kind == 'MOVE' else 'deg'})", "number")
    else:
        raise Skip("needs the car or the sim car (the stub never finishes a MOVE)")
    c.num("true", out["true"], "mm" if kind == "MOVE" else "deg")
    err = (out["true"] - asked) / abs(asked) * 100.0
    c.num("error_pct", err, "%")
    out["err"] = err
    return out


# ------------------------------------------------------------------ scenarios
@scenario("link", "PING + STATUS sanity (link, backend, wheels)", psec="2b step 6")
def s_link(c: Ctx):
    t0 = time.monotonic()
    r = c.send("motor", "PING")
    c.num("ping_ms", (time.monotonic() - t0) * 1000, "ms")
    c.expect(r == "OK PONG", f"PING -> {r}")
    st = c.status(wait=1.0)
    c.expect(st is not None, "STATUS parses")
    if st:
        c.num("backend", st.get("backend"))
        c.num("wheels", st.get("wheels"))
        c.num("clients", st.get("clients"))
        if c.where != "stub":
            c.expect(st.get("wheels") == "dc", "STATUS wheels=dc (SM11 firmware reported WHEELS DC)")
    g = c.send("goto", "PING", 1.0)
    c.note(f"goto: {g}")


@scenario("move_1000", "MOVE 1000: reported vs true distance, sideways curve", needs="car", psec="2b step 4-5, Step 5 dc-move")
def s_move(c: Ctx):
    o = _measure_move(c, "MOVE 1000", "MOVE", 1000.0)
    c.expect(abs(o["err"]) <= 15.0, f"distance within 15% (got {o['err']:+.1f}%; calibrate with SPEEDCAL if not)")
    lat = c.numbers.get("lateral_mm", {}).get("v")
    if lat is not None:
        c.expect(abs(lat) <= 150, f"curve over 1 m <= 150 mm (got {lat:+.0f}; trim DC_LEFT/RIGHT_GAIN)")


@scenario("turn_90", "TURN 90 (right): reported vs true angle", needs="car", psec="2b step 4")
def s_turn90(c: Ctx):
    o = _measure_move(c, "TURN 90", "TURN", 90.0)
    c.expect(abs(o["err"]) <= 15.0, f"angle within 15% (got {o['err']:+.1f}%; goto's search needs < ~15%)")


@scenario("turn_-90", "TURN -90 (left): reported vs true angle", needs="car", psec="2b step 4")
def s_turn_m90(c: Ctx):
    o = _measure_move(c, "TURN -90", "TURN", -90.0)
    c.expect(abs(o["err"]) <= 15.0, f"angle within 15% (got {o['err']:+.1f}%)")


@scenario("turn_slow", "TURN 90 at 0.40 (goto's turns, the guard pivot): must actually turn", needs="car", psec="2b step 4 slow turn")
def s_turn_slow(c: Ctx):
    o = _measure_move(c, "TURN 90 0.40", "TURN", 90.0)
    c.expect(o["true"] > 20, "it turned (didn't stall: else raise DC_SLOW_TURN_DUTY)")
    c.expect(abs(o["err"]) <= 15.0, f"angle within 15% (got {o['err']:+.1f}%; dc-spin --slow)")


@scenario("stop_latency", "drive 1 s at D 0.6, then S: time until the wheels read zero", psec="2b step 3")
def s_stop(c: Ctx):
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        r = c.robot.send("motor", "D 0.600 0.600", timeout=1.0, src=f"scenario:{c.name}")
        if not r.startswith("OK"):
            raise Skip(f"D refused: {r}")
        c.check_abort()
        time.sleep(0.05)
    st = c.status()
    c.expect(st is not None and st["out_l"] > 0.1, f"wheels driving before S (out_l={st and st['out_l']})")
    t0 = time.monotonic()
    c.send("motor", "S")
    t_s = (time.monotonic() - t0) * 1000
    c.num("S_reply_ms", t_s, "ms")
    z = c.wait(lambda: (lambda s: s if s and s["out_l"] == 0 and s["out_r"] == 0 else None)(c.fresh_status_quiet()), 2.0, 0.005)
    lat = (time.monotonic() - t0) * 1000
    c.expect(z is not None, "tars_motor output reads zero after S")
    c.num("status_zero_ms", lat, "ms")
    c.expect(lat < 150, f"zero within 150 ms (got {lat:.0f})")
    if c.where == "sim-car":
        w = c.wait(lambda: (lambda t: t if t and abs(t.get("wl", 1)) < 0.005 and abs(t.get("wr", 1)) < 0.005 else None)(c.truth()), 2.0)
        c.num("wheels_stopped_ms", (time.monotonic() - t0) * 1000, "ms (car truth)")
        c.expect(w is not None, "virtual car's wheels stopped")
    elif c.where == "real":
        c.expect(c.ask("Did the wheels stop dead at once (brake)?", "yesno"), "user: stopped dead")


def _fresh_status_quiet(self):
    """STATUS right now, without a step line per call (used in tight loops)."""
    r = self.robot.send("motor", "STATUS", timeout=1.0, src="poll")
    from_robot = self.bench.plugins.get("robot")
    return from_robot.parse_status(r)


Ctx.fresh_status_quiet = _fresh_status_quiet


@scenario("watchdog", "one D, then silence: tars_motor's watchdog stops the wheels", psec="Step 1 watchdog")
def s_watchdog(c: Ctx):
    r = c.send("motor", "D 0.500 0.500")
    t0 = time.monotonic()
    c.expect(r.startswith("OK"), f"D accepted ({r})")
    # STATUS (unlike PING) doesn't feed the watchdog, so asking it fast is fine
    moving = c.wait(lambda: (lambda s: s if s and s["out_l"] > 0 and s.get("why") != "watchdog" else None)(
        c.fresh_status_quiet()), 0.4, 0.01)
    c.expect(moving is not None, "wheels running after the one D")
    st = c.wait(lambda: (lambda s: s if s and s["out_l"] == 0 and s.get("why") == "watchdog" else None)(
        c.fresh_status_quiet()), 3.0, 0.01)
    ms = (time.monotonic() - t0) * 1000
    c.expect(st is not None, "wheels stopped with why=watchdog")
    c.num("stop_after_ms", ms, "ms (tars_motor -w, default 500)")
    c.expect(300 <= ms <= 1000, f"stopped between 300 and 1000 ms (got {ms:.0f})")


@scenario("stop_mid_move", "MOVE 300, then S after 0.5 s: the move aborts (why=stop)", psec="2b step 4 S mid-move")
def s_stop_mid_move(c: Ctx):
    mid, est = c.start_move("MOVE 300")
    time.sleep(0.5)
    c.send("motor", "S")
    st = c.wait(lambda: (lambda s: s if s and s.get("move_id") == mid and s.get("move") != "running" else None)(c.robot.status()), 2.0)
    c.expect(st is not None and st.get("move") == "aborted", f"move aborted ({st and st.get('move')}, why={st and st.get('move_why')})")
    if st:
        c.num("done_at_abort", st.get("move_done"), "mm")
    if c.where == "sim-car":
        t = c.wait(lambda: (lambda t: t if t and abs(t.get("v", 1)) < 0.005 else None)(c.truth()), 2.0)
        c.expect(t is not None, "virtual car stopped")


@scenario("wheel_direction", "D 0.5 0.5 = both forward; D 0.5 0 = left only", needs="car", psec="2b step 2")
def s_dir(c: Ctx):
    def drive(l, r, secs=0.8):
        end = time.monotonic() + secs
        peak = {"wl": 0.0, "wr": 0.0}
        while time.monotonic() < end:
            c.robot.send("motor", f"D {l:.3f} {r:.3f}", timeout=1.0, src=f"scenario:{c.name}")
            t = c.truth()
            if t:
                for k in peak:
                    if abs(t.get(k, 0)) > abs(peak[k]):
                        peak[k] = t.get(k, 0)
            c.check_abort()
            time.sleep(0.05)
        c.send("motor", "S")
        return peak
    if c.where == "sim-car":
        p = drive(0.5, 0.5)
        c.num("both_wl", p["wl"], "m/s"); c.num("both_wr", p["wr"], "m/s")
        c.expect(p["wl"] > 0.02 and p["wr"] > 0.02, "both wheels forward")
        time.sleep(0.5)
        p = drive(0.5, 0.0)
        c.num("left_wl", p["wl"], "m/s"); c.num("left_wr", p["wr"], "m/s")
        c.expect(p["wl"] > 0.02 and abs(p["wr"]) < 0.01, "only the left wheel")
    elif c.where == "real":
        c.ask("Wheels in the air? Press OK, then watch: D 0.5 0.5 for 0.8 s.", "ok")
        drive(0.5, 0.5)
        c.expect(c.ask("Did BOTH wheels spin FORWARD? (no: flip LEFT_INVERT / RIGHT_INVERT)", "yesno"), "both forward")
        drive(0.5, 0.0)
        c.expect(c.ask("Did ONLY the LEFT wheel spin? (no: swap the L and R pin groups)", "yesno"), "left only")
    else:
        raise Skip("needs the car or the sim car")


def _speedcal(c: Ctx, kind: str, slow: bool = False):
    import wheel_cal  # iter1-wasd/laptop (on sys.path via the robot plugin)
    r = c.send("motor", "SPEEDCAL")
    p = r.split()
    if not r.startswith("OK") or p[1] != "dc":
        raise Skip(f"not DC wheels: SPEEDCAL -> {r} (stepper build: use wheel_cal.py move/turns)")
    mm_s, deg_s = float(p[2]), float(p[3])
    sl = float(p[4]) if len(p) > 4 else None
    c.num("old_mm_s", mm_s, "mm/s"); c.num("old_deg_s", deg_s, "deg/s")
    if sl:
        c.num("old_deg_s_slow", sl, "deg/s")
    if slow and not sl:
        raise Skip("the Uno doesn't report a slow-turn rate (flash the current tars.ino)")
    if kind == "MOVE":
        o = _measure_move(c, "MOVE 1000", "MOVE", 1000.0)
        asked, new = 1000.0, None
        new_mm_s = wheel_cal.scaled(mm_s, asked, o["true"])
        new = (new_mm_s, deg_s, sl)
    else:
        asked = 180.0 if slow else 360.0
        o = _measure_move(c, f"TURN {asked:.0f}" + (" 0.40" if slow else ""), "TURN", asked)
        if slow:
            new = (mm_s, deg_s, wheel_cal.scaled(sl, asked, o["true"]))
        else:
            new = (mm_s, wheel_cal.scaled(deg_s, asked, o["true"]), sl)
    c.num("new_mm_s", new[0], "mm/s"); c.num("new_deg_s", new[1], "deg/s")
    if new[2]:
        c.num("new_deg_s_slow", new[2], "deg/s")
    if not 100 <= new[0] <= 200:
        c.note("mm_s outside 100-200: change DC_MAX_DUTY in tars.ino rather than living with it")
    if not c.ask(f"Save SPEEDCAL {new[0]:.1f} {new[1]:.1f}" + (f" {new[2]:.1f}" if new[2] else "") + " to the Uno's EEPROM?", "yesno"):
        c.note("not saved")
        return
    line = f"SPEEDCAL {new[0]:.1f} {new[1]:.1f}" + (f" {new[2]:.1f}" if new[2] else "")
    c.expect(c.send("motor", line).startswith("OK"), "SPEEDCAL set accepted")
    ok = c.wait(lambda: (lambda r: r if r.startswith("OK dc") and abs(float(r.split()[2]) - new[0]) < 0.05
                         and abs(float(r.split()[3]) - new[1]) < 0.05 else None)(
        c.robot.send("motor", "SPEEDCAL", timeout=1.0, src="poll")), 1.5, 0.1)
    c.expect(ok is not None, "the Uno confirmed the new speeds (WHEELS DC)")


@scenario("speedcal_move", "wheel_cal dc-move: MOVE 1000, measure, new mm_s, save", needs="car", psec="Step 5 DC dc-move")
def s_cal_move(c: Ctx):
    _speedcal(c, "MOVE")


@scenario("speedcal_spin", "wheel_cal dc-spin: TURN 360, measure, new deg_s, save", needs="car", psec="Step 5 DC dc-spin")
def s_cal_spin(c: Ctx):
    _speedcal(c, "TURN")


@scenario("speedcal_slow", "wheel_cal dc-spin --slow: TURN 180 0.40, new deg_s_slow, save", needs="car", psec="Step 5 DC dc-spin --slow")
def s_cal_slow(c: Ctx):
    _speedcal(c, "TURN", slow=True)


def _safety(c: Ctx, what: str):
    """tip / bump: inject (sim car fault, or the stub's SIM line), check D is refused, clear."""
    st0 = c.status(wait=1.0) or {}
    if c.where == "sim-car":
        c.car_control({"fault": what})
        c.note(f"virtual car fault: {what}")
    elif c.where == "stub":
        c.note("stub backend: injecting the co-processor line (tests tars_motor, NOT the firmware)")
        c.send("motor", "SIM T 60" if what == "tip" else "SIM B 900")
    else:
        c.ask("Tilt the car past ~45° and hold it" if what == "tip" else "Give the car a sharp knock (bump)", "ok")
    if what == "tip":
        st = c.wait(lambda: (lambda s: s if s and s.get("tipped") == 1 else None)(c.robot.status()), 3.0)
        c.expect(st is not None, "STATUS tipped=1")
    else:
        st = c.wait(lambda: (lambda s: s if s and s.get("bumps", 0) > st0.get("bumps", 0) else None)(c.robot.status()), 3.0)
        c.expect(st is not None, f"STATUS bumps went up (was {st0.get('bumps')})")
    if st:
        c.num("estop_why", st.get("why"))
        c.num("tilt", st.get("tilt"), "deg")
    r = c.send("motor", "D 0.300 0.300")
    st2 = c.fresh_status_quiet()
    c.expect(r.startswith("ERR") or (st2 and st2["out_l"] == 0), f"drive refused or held at zero while {what} ({r})")
    c.send("motor", "S")
    if c.where == "sim-car":
        c.car_control({"fault": "clear"})
    elif c.where == "real" and what == "tip":
        c.ask("Stand it back up, then OK", "ok")
    if what == "tip":
        # tipped latches until RESET; RESET is refused while tip reports are recent (TIP_CLEAR_MS)
        r = c.send("motor", "RESET")
        c.expect(r.startswith("ERR still tipped") or r.startswith("OK"), f"RESET right away: {r}")
        t0 = time.monotonic()
        r = c.wait(lambda: (lambda r: r if r.startswith("OK") else None)(
            c.robot.send("motor", "RESET", timeout=1.0, src=f"scenario:{c.name}")), 6.0, 0.25)
        c.num("reset_ok_after_ms", (time.monotonic() - t0) * 1000, "ms")
        c.expect(r is not None, "RESET accepted once the tip reports stopped")
    else:
        bump_ms = (st or {}).get("bump_ms", 0) or 0
        c.wait(lambda: c.robot.status() and (c.robot.status().get("bump_ms") or 0) == 0, bump_ms / 1000 + 3)
    r = c.send("motor", "D 0.300 0.300")
    c.expect(r.startswith("OK"), f"drive accepted again ({r})")
    c.send("motor", "S")


@scenario("tip", "tip-over: stops, refuses D, RESET once upright", psec="Step 1/2 tip")
def s_tip(c: Ctx):
    _safety(c, "tip")


@scenario("bump", "bump: stops, holds D for bump_ms, then drives again", psec="Step 1/2 bump")
def s_bump(c: Ctx):
    _safety(c, "bump")


@scenario("goto_trash", "GOTO trash can in the sim room: EVENT arrived", needs="sim", psec="(sim only)")
def s_goto(c: Ctx):
    if c.where != "sim-car" or "world" not in c.bench.plugins:
        raise Skip("needs sim (the world plugin + virtual car feeding goto); on the real car, say it")
    events: list[str] = []
    c.robot.listen("goto", lambda line: events.append(line))
    r = c.send("goto", "GOTO trash can")
    c.expect(r.startswith("OK"), f"goto accepted ({r})")
    t0 = time.monotonic()
    end_ev = c.wait(lambda: next((e for e in events if e.split()[1:2] and
                                  (e.split()[1] in ("arrived", "lost", "cancelled") or e.split()[1].startswith("failed"))), None),
                    240.0, 0.2)
    c.num("took_s", time.monotonic() - t0, "s")
    for e in events[-12:]:
        c.note(e)
    c.expect(end_ev is not None and " arrived " in end_ev + " ", f"arrived ({end_ev})")


# ------------------------------------------------------------------ runner + routes
def _runner(name: str) -> None:
    sc = SCENARIOS[name]
    c = Ctx(_bench, name)
    c._pub()
    state = "pass"
    try:
        if sc["needs"] in ("car", "sim") and c.where == "stub":
            raise Skip("needs the car or the sim car (this is the stub backend)")
        if sc["needs"] == "sim" and c.where == "real":
            raise Skip("needs sim")
        sc["fn"](c)
        state = "pass" if c.ok else "fail"
    except Skip as e:
        state = "skip"
        c.steps.append({"t": round(time.time() - c.t0, 2), "kind": "skip", "text": str(e)})
    except Aborted as e:
        state = "aborted"
        c.steps.append({"t": round(time.time() - c.t0, 2), "kind": "fail", "text": f"aborted: {e}"})
    except Exception as e:
        state = "error"
        c.steps.append({"t": round(time.time() - c.t0, 2), "kind": "fail", "text": f"{type(e).__name__}: {e}"})
        traceback.print_exc()
    finally:
        # leave the robot stopped, whatever happened (a stop is always safe)
        if c.robot:
            c.robot.send_nowait("motor", "S", src=f"scenario:{name}")
        _run["name"] = None
        c._pub(state)
        _bench.log("scenario", f"{name}: {state.upper()} ({c.where}) " +
                   ", ".join(f"{k}={v['v']}{v['unit'] and ' ' + v['unit'].split()[0]}" for k, v in c.numbers.items()))
        _run["last"] = {"name": name, "state": state, "where": c.where, "numbers": c.numbers, "steps": c.steps}


def run(name: str, wait: bool = False, timeout: float = 300.0) -> dict:
    """start a scenario (tests/other plugins). wait=True blocks and returns the result."""
    if name not in SCENARIOS:
        return {"error": f"no scenario {name}"}
    if _run["name"]:
        return {"error": f"{_run['name']} is running"}
    _run.update(name=name, cancel=False, last=None)
    th = threading.Thread(target=_runner, args=(name,), daemon=True, name=f"scenario-{name}")
    _run["thread"] = th
    th.start()
    if wait:
        th.join(timeout)
        return _run.get("last") or {"error": "timeout"}
    return {"started": name}


def answer(value) -> None:
    _answer["value"] = value
    _answer["ev"].set()


def setup(bench) -> None:
    global _bench
    _bench = bench

    def tap(topic, data):
        if topic == "car" and isinstance(data, dict) and not data.get("_replay"):
            d = data.get("truth", data) if isinstance(data.get("truth", data), dict) else None
            if d and "x" in d and "th" in d:
                _truth["d"], _truth["t"] = d, time.monotonic()
    bench.tap(tap)

    def r_list(q, body):
        return 200, {"running": _run["name"], "robot": "robot" in bench.plugins, "car": "car" in bench.plugins,
                     "world": "world" in bench.plugins,
                     "scenarios": [{"name": k, "title": v["title"], "needs": v["needs"], "pi_test": v["pi_test"]}
                                   for k, v in SCENARIOS.items()]}

    def r_run(q, body):
        name = (json.loads(body or b"{}") or {}).get("name", "")
        if "robot" not in bench.plugins:
            return 409, {"error": "robot plugin not loaded"}
        r = run(name)
        return (409 if "error" in r else 200), r

    def r_answer(q, body):
        answer((json.loads(body or b"{}") or {}).get("value"))
        return 200, {"ok": True}

    def r_cancel(q, body):
        _run["cancel"] = True
        answer(None)
        return 200, {"ok": True}

    bench.route("GET", "/api/scenarios/list", r_list)
    bench.route("POST", "/api/scenarios/run", r_run)
    bench.route("POST", "/api/scenarios/answer", r_answer)
    bench.route("POST", "/api/scenarios/cancel", r_cancel)
