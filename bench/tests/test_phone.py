#!/usr/bin/env python3
"""bench/phone tests: sim feed -> Andrew's mapping, record/replay, endpoints, usbmux errors,
live world changes. Every port is free/ephemeral. No iPhone is used."""
import http.client
import json
import math
import os
import plistlib
import socket
import struct
import sys
import tempfile
import threading
import time

BENCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, BENCH)
from phone import pipeline as P, sim, usbmux, scene_io  # noqa: E402
import bench as B  # noqa: E402

fails = total = 0


def check(ok, what):
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait(cond, timeout=30.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.05)
    return False


def run_to_end(pipe):
    """Wait until the finite source has sent everything and the mapper took it all."""
    ok = wait(lambda: pipe.server is not None and pipe.server.finished.is_set())
    ok = ok and wait(lambda: pipe.feed.count >= pipe.server.sent, 5)
    return ok


def cell_index(wx, wy):
    c = P.mapping.Map.cell(*sim.to_arkit(wx, wy, 0)[0::2])
    return c[1] * 200 + c[0]


def seg_dist(px, py, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    u = max(0.0, min(1.0, ((px - x1) * ex + (py - y1) * ey) / (ex * ex + ey * ey)))
    return math.hypot(px - x1 - u * ex, py - y1 - u * ey)


def geometry_dist(room, wx, wy):
    """Distance from a world point to the nearest wall/box/cylinder outline."""
    d = min(seg_dist(wx, wy, *w) for w in room["walls"])
    for b in room["boxes"]:
        dx = max(abs(wx - b["x"]) - b["w"] / 2, 0)
        dy = max(abs(wy - b["y"]) - b["d"] / 2, 0)
        d = min(d, math.hypot(dx, dy))
    for c in room["cylinders"]:
        d = min(d, max(0.0, math.hypot(wx - c["x"], wy - c["y"]) - c["r"]))
    return d


def occupied_world(grid):
    out = []
    for i, v in enumerate(grid):
        if v == 2:
            ax, az = P.mapping.Map.world((i % 200, i // 200))
            out.append((az, ax))  # world x = ARKit z, world y = ARKit x
    return out


def test_sim_map_and_replay(tmp):
    print("sim feed -> Andrew's Map (over his SceneFeed/TCP), then record -> replay")
    pipe = P.PhonePipeline(room_port=free_port(), scene_port=free_port(), state_dir=tmp)
    try:
        rec = pipe.record(True, "t.jsonl", directory=tmp)
        st = pipe.set_source("sim", pose="script", realtime=False, duration=28.0)
        check(st["source"] == "sim" and not st["error"], f"sim source started ({st['error'] or 'ok'})")
        check(run_to_end(pipe), f"sim stream finished ({pipe.server.sent} scenes, link: {pipe.feed.status})")
        saved = pipe.record(False)
        check(saved.get("scenes") == pipe.server.sent, f"recorded every scene ({saved.get('scenes')})")
        room = sim.room_from_json(sim.DEFAULT_ROOM)
        grid_a = bytes(pipe.mapper.grid)
        vox_a = list(pipe.mapper.voxels)
        objs_a = sorted((o["label"], tuple(o["position"])) for o in pipe.mapper.objects.values())
        occ = occupied_world(grid_a)
        near = sum(geometry_dist(room, x, y) <= 0.2 for x, y in occ)
        check(len(occ) > 150, f"walls mapped as occupied ({len(occ)} cells)")
        check(near / max(1, len(occ)) >= 0.97, f"occupied cells sit on walls/objects ({near}/{len(occ)})")
        for name, (wx, wy) in {"left wall y=2": (0.5, 1.95), "right wall y=-2": (0.5, -1.95),
                               "back wall x=-1": (-0.95, 0.0), "inner wall x=2": (2.0, -1.0)}.items():
            idx = [cell_index(wx + dx, wy + dy) for dx in (-0.1, 0, 0.1) for dy in (-0.1, 0, 0.1)]
            check(any(grid_a[i] == 2 for i in idx), f"occupied at the {name}")
        open_free = [grid_a[cell_index(x / 10, y / 10)] == 1 for x in range(3, 18) for y in range(-8, 9)]
        check(sum(open_free) / len(open_free) > 0.5, f"open floor mostly free ({sum(open_free)}/{len(open_free)} cells)")
        check(not any(grid_a[cell_index(x / 10, y / 10)] == 2 for x in range(4, 17) for y in range(-5, 6)),
              "no occupied cells in the open middle")
        check(any(lbl == "chair" for lbl, _ in objs_a) and not any(lbl == "trash can" for lbl, _ in objs_a),
              f"chair detected, bin behind the wall not detected ({[l for l, _ in objs_a]})")

        st = pipe.set_source("replay", path=saved["saved"], speed=0)
        check(st["source"] == "replay", "replay source started")
        check(run_to_end(pipe), f"replay finished ({pipe.server.sent} scenes)")
        grid_b = bytes(pipe.mapper.grid)
        objs_b = sorted((o["label"], tuple(o["position"])) for o in pipe.mapper.objects.values())
        check(grid_a == grid_b, "replay gives the same occupancy grid")
        check(vox_a == list(pipe.mapper.voxels), f"replay gives the same voxels ({len(vox_a)})")
        check(objs_a == objs_b, "replay gives the same objects")
        size = os.path.getsize(saved["saved"])
        print(f"    recording size {size / 1e6:.2f} MB")
    finally:
        pipe.close()


def test_world_live(tmp):
    print("sim feed follows live world changes (topic `world`)")
    pipe = P.PhonePipeline(room_port=free_port(), scene_port=free_port(), state_dir=tmp)
    try:
        # World-sim style room (bench/world/rooms/*.json shape), robot parked facing +x.
        world = {"room": {"wall_h": 2.0, "walls": [[-1, -2, 5, -2], [5, -2, 5, 2], [5, 2, -1, 2], [-1, 2, -1, -2]],
                          "objects": [{"id": "door", "label": "door", "x": 3.0, "y": 0, "w": .04, "d": .8,
                                       "h": 2.0, "solid": False}]},
                 "car": {"x": 0.0, "y": 0.0, "th": 0.0}}
        check(pipe.set_world(world) is False, "world before sim: stored, nothing to re-cast yet")
        pipe.set_source("sim", pose="auto")
        check(pipe.status()["room_source"] == "world", "sim starts from the world's room")

        def occ_near(x, y, r=0.25):
            g = bytes(pipe.mapper.grid)
            return sum(g[cell_index(x + dx / 10, y + dy / 10)] == 2
                       for dx in range(-3, 4) for dy in range(-3, 4) if math.hypot(dx, dy) / 10 <= r)

        def refresh_pose():
            pipe.set_world(world)  # keeps the world pose fresh, like a 5593 stream would
            return False
        wait(lambda: refresh_pose() or pipe.mapper.frames >= 5, 10)
        check(pipe.mapper.frames >= 5, f"sim scenes flowing ({pipe.mapper.frames})")
        check(occ_near(1.5, 0.0) == 0, "nothing at (1.5, 0) yet; the non-solid door isn't geometry")
        world["room"]["objects"].append({"id": "p1", "label": "person", "x": 1.5, "y": 0.0, "r": 0.2, "h": 1.7})
        check(pipe.set_world(world) is True, "adding a person changes the geometry")
        ok = wait(lambda: refresh_pose() or occ_near(1.5, 0.0) >= 3, 10)
        check(ok, f"the added person appears as occupied cells ({occ_near(1.5, 0.0)})")
        world["room"]["objects"].append({"id": "b", "label": "box", "x": 1.8, "y": -0.6, "w": .4, "d": .3, "h": .3})
        pipe.set_world(world)
        ok = wait(lambda: refresh_pose() or occ_near(1.8, -0.6) >= 2, 10)
        check(ok, f"a dropped box (in view) appears too ({occ_near(1.8, -0.6)})")
        check(pipe.set_world({"sensors": {"guard": "CLEAR"}}) is False, "a world message without a room is ignored")
        pipe.set_source("off")
    finally:
        pipe.close()


def test_endpoints(tmp):
    print("bench plugin endpoints + Andrew's room view server")
    port, room_port = free_port(), free_port()
    os.environ.update(TARS_BENCH_PHONE_ROOM_PORT=str(room_port), TARS_BENCH_PHONE_SCENE_PORT=str(free_port()),
                      TARS_BENCH_PHONE_SOURCE="sim")
    bench = B.Bench(B.parse_args(["--port", str(port), "--plugins", "phone"]))
    B.load_plugins(bench, ["phone"])
    check("phone" in bench.plugins, "phone plugin loads")
    srv = B.ThreadingHTTPServer(("127.0.0.1", port), B.make_handler(bench))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def get(p, path, method="GET", body=None):
        c = http.client.HTTPConnection("127.0.0.1", p, timeout=5)
        c.request(method, path, body=body)
        r = c.getresponse()
        return r.status, r.read()
    try:
        mod = bench.plugins["phone"]
        wait(lambda: mod.pipe.mapper.frames >= 3, 10)
        st, body = get(port, "/api/phone/map.json")
        room = json.loads(body)
        check(st == 200 and room["size"] == 200 and len(room["grid"]) == 40000 and set(room["grid"]) <= set("0123"),
              "/api/phone/map.json is valid room JSON (200x200 grid)")
        check(len(room["pose"]) == 4 and isinstance(room["points3d"], list) and room["frames"] >= 3,
              f"room JSON has pose, points3d, frames ({room['frames']}, {len(room['points3d'])} points)")
        def room_frames():
        try:
            return json.loads(get(room_port, "/map.json")[1] or b"{}").get("frames", 0)
        except OSError:
            return 0
    wait(lambda: room_frames() >= 1, 5)
        st, body = get(room_port, "/")
        check(st == 200 and b"Room view" in body, "Andrew's room_view.html served on the room port")
        st, body = get(room_port, "/map.json")
        check(st == 200 and json.loads(body).get("frames", 0) >= 1, "his /map.json serves the bench map")
        st, body = get(port, "/api/phone/status")
        s = json.loads(body)
        check(st == 200 and s["source"] == "sim" and s["room_url"].endswith(f":{room_port}/"), "status endpoint")
        st, body = get(port, "/api/phone/source", "POST", b'{"kind":"bogus"}')
        check(st == 400, "bad source refused")
        st, body = get(port, "/api/phone/command", "POST", b'{"op":"map_clear"}')
        check(st == 200 and json.loads(body)["ok"], "Andrew's map_clear through the bench")
        bench.publish("car", {"x": 0.5, "y": 0.2, "th": 0.1})
        wait(lambda: any(o.get("id") == "truth" for o in json.loads(get(port, "/api/phone/map.json")[1])["objects"]), 3)
        objs = json.loads(get(port, "/api/phone/map.json")[1])["objects"]
        check(any(o.get("id") == "truth" for o in objs), "car topic -> sim truth overlay")
        check(mod.parse_route("OK route=1.0,2.0;1.5,2.5 i=1") == [[1.0, 2.0], [1.5, 2.5]]
              and mod.parse_route("OK route=- i=0") is None, "goto NAV ROUTE parsing")
    finally:
        mod.shutdown()
        srv.shutdown()
        for k in ("TARS_BENCH_PHONE_ROOM_PORT", "TARS_BENCH_PHONE_SCENE_PORT", "TARS_BENCH_PHONE_SOURCE"):
            os.environ.pop(k, None)


def fake_usbmuxd(path, replies):
    """A unix-socket usbmuxd answering each request with the next plist in `replies`."""
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(path)
    srv.listen(4)

    def serve():
        while True:
            try:
                c, _ = srv.accept()
            except OSError:
                return
            with c:
                hdr = c.recv(16)
                if len(hdr) < 16:
                    continue
                n = struct.unpack("<I", hdr[:4])[0]
                req = b""
                while len(req) < n - 16:
                    req += c.recv(n - 16 - len(req))
                msg = plistlib.loads(req)
                reply = replies.get(msg["MessageType"], {"MessageType": "Result", "Number": 1})
                body = plistlib.dumps(reply)
                c.sendall(struct.pack("<IIII", 16 + len(body), 1, 8, 1) + body)
    threading.Thread(target=serve, daemon=True).start()
    return srv


def test_usbmux(tmp):
    print("usbmux client")
    sock_dir = tempfile.mkdtemp(prefix="mux")  # unix socket paths must stay short (<104)
    try:
        missing = os.path.join(sock_dir, "nope")
        try:
            usbmux.list_devices(missing)
            check(False, "missing usbmuxd raises")
        except usbmux.UsbmuxError as e:
            check("not reachable" in str(e), "missing usbmuxd -> readable UsbmuxError")
        empty = os.path.join(sock_dir, "empty")
        srv = fake_usbmuxd(empty, {"ListDevices": {"DeviceList": []}})
        check(usbmux.list_devices(empty) == [], "no device -> empty list")
        try:
            usbmux.connect_phone(5561, empty)
            check(False, "no device raises NoDevice")
        except usbmux.NoDevice as e:
            check("no iPhone on USB" in str(e), f"no device -> NoDevice ({e})")
        fwd = usbmux.Forwarder(0, 5561, empty)
        c = socket.create_connection(("127.0.0.1", fwd.port), timeout=2)
        c.settimeout(2)
        check(c.recv(1) == b"", "forwarder closes the client when there is no phone")
        wait(lambda: fwd.state.startswith("error"), 2)
        check("no iPhone" in fwd.state, f"forwarder state says why ({fwd.state})")
        fwd.close()
        srv.close()
        refused = os.path.join(sock_dir, "refused")
        srv2 = fake_usbmuxd(refused, {"ListDevices": {"DeviceList": [{"DeviceID": 7, "Properties": {
            "ConnectionType": "USB", "SerialNumber": "abc", "DeviceID": 7}}]},
            "Connect": {"MessageType": "Result", "Number": 3}})
        try:
            usbmux.connect_phone(5561, refused)
            check(False, "refused raises")
        except usbmux.UsbmuxError as e:
            check("MotionUSB" in str(e), f"app not running -> readable error ({e})")
        srv2.close()
        # The real daemon on this Mac: whatever it says, the client must not crash.
        try:
            devs = usbmux.list_devices()
            print(f"    real usbmuxd: {len(devs)} device(s)")
        except usbmux.UsbmuxError as e:
            print(f"    real usbmuxd: {e}")
    finally:
        for f in os.listdir(sock_dir):
            os.unlink(os.path.join(sock_dir, f))
        os.rmdir(sock_dir)


def test_room_format():
    print("room parsing")
    r = sim.room_from_json({"wall_h": 2.0, "walls": [[0, 0, 1, 0]], "objects": [
        {"label": "trash can", "x": 1, "y": 2, "r": .15, "h": .4},
        {"label": "chair", "pos": [3, 4], "size": [.4, .5], "h": .8},
        {"label": "door", "x": 0, "y": 0, "w": .04, "d": .8, "h": 2, "solid": False}]})
    check(r["wall_height"] == 2.0 and len(r["cylinders"]) == 1 and len(r["boxes"]) == 2
          and r["boxes"][1]["solid"] is False, "world-room format (r cylinders, w/d boxes, solid:false)")
    check(sim.find_room({"state": {"walls": []}}) == {"walls": []} and sim.find_room({"x": 1}) is None,
          "room found nested in a world message")
    ph = sim.SimPhone(room={"walls": [], "objects": []}, dropout=0, range_noise_m=0)
    s = ph.scene(1.0, 0, 0, 0)
    P.mapping.validate_scene(s)
    check(all(abs(p[1]) < 1e-6 for p in s["points"]) and s["points"], "empty room: only floor points; scene passes Andrew's validator")


def main():
    tmp = tempfile.mkdtemp(prefix="bench-phone-")
    print(f"load: {os.getloadavg()[0]:.2f}")
    test_room_format()
    test_usbmux(tmp)
    test_sim_map_and_replay(tmp)
    test_world_live(tmp)
    test_endpoints(tmp)
    print(f"{total - fails}/{total} checks passed (no iPhone used)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
