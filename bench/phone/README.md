# bench/phone: iPhone LiDAR → laptop 3D map

Andrew's phone pipeline, run on the laptop, fed by the real iPhone, a recording, or a
ray-cast sim room. Everything after the source is **Andrew's code, imported unchanged**:
`dashboard/mapping.SceneFeed` + `Map`, and `dashboard/room_view.serve_room` (his
`room_view.html` + `room_controls.js`). The bench panel (`web/panels/map3d.js`) shows his room
view in an iframe and adds a source picker, link state and rates.

```
source on 127.0.0.1:5595 ──► mapping.SceneFeed ──► mapping.Map(native=None) ──► room_view on 127.0.0.1:5594
  live:   usbmux.Forwarder 5595 → phone 5561 (stdlib, no iproxy needed)
  iproxy: you run `iproxy -l -s 127.0.0.1 5595:5561`
  replay: scene_io.SceneServer playing recordings/*.jsonl
  sim:    scene_io.SceneServer playing sim.SimPhone (ray-cast room)
```

## How the phone's data reaches the Pi, and the Mac

- MotionUSB on the phone **listens** (NWListener): 5557 motion (JSON lines, ~100 Hz),
  5558 video (4-byte length + JPEG, ACK 0x01), 5559 status/commands (JSON lines),
  **5561 scenes** (4-byte BE length + JSON v1, ≤ 512 KiB, ACK 0x01, one in flight),
  5562 safety, 5563 RGB.
- On the Pi: a patched usbmuxd + `iproxy -l -s 127.0.0.1 P:P` per port (`service.py`), and
  `headless.py` connects to 127.0.0.1 as a client.
- On the Mac, macOS's own usbmuxd (`/var/run/usbmuxd`) does the USB part. `usbmux.py` is a
  ~150-line stdlib client (plist ListDevices / Connect), so no install is needed. `iproxy` from
  brew works too if it's installed.
- Only one host can own the phone: **plugged into the Mac means not on the Pi.** The phone accepts
  one client per stream.

## Live checklist (not yet run with a real phone)

1. Plug the iPhone into this Mac with the cable, unlock it, and tap **Trust** if asked.
2. Open **MotionUSB** on the phone, press **Start**, and keep it in the foreground.
3. Run `python3 bench/bench.py` and open http://127.0.0.1:8770. In **iPhone 3D map**, pick
   *live iPhone (USB, this Mac)* and press **apply**. The line under the picker lists the
   phone's serial.
4. The state line should say `Receiving ARKit scene`, with scenes/s above 0 and motion near 100 Hz.
   If it says `connection refused … MotionUSB` instead, the app isn't started.
5. Turn the phone slowly around the room. Once the floor is calibrated, voxels and free/occupied
   counts rise, and Andrew's 3D view and floor map fill in. **● record** saves the stream to
   `recordings/` so you can replay it later.

## Recording and replay

`.jsonl`: a header line, then `{"t": s, "scene": {…}}` per scene (the validated scene as
received). Replay keeps the original spacing (`speed`, 0 = as fast as ACKs allow) and gives
each loop a new session id so Map starts clean. `recordings/sample-sim-room.jsonl` (0.8 MB)
is **synthetic** (`make_sample.py`, every 10th sim scene), not a phone capture. A full
28 s sim recording is about 7.7 MB: real recordings get large, so don't commit them.

## Sim feed

`sim.SimPhone` ray-casts a 48×30 depth grid (64°×50°, 0.2–4 m, 1 cm range noise, 2 %
dropout), with the camera 0.138 m above the floor (Andrew's mount estimate). Detections are
labelled objects in the field of view, within 4 m and not occluded by walls or other objects.
Frames: ARKit (x, y, z) = (world y, world z, world x), yaw = world th, floor_y = 0.
- **Room**: the sim world's room format (`walls`, `wall_h`, `objects` with `r` = cylinder or
  `w`/`d` = box, `solid: false` = no geometry). It comes live from the **`world` topic**: the
  room at the top level or under `room`/`world`/`state`. Each geometry change swaps the caster,
  so dropped, moved or deleted obstacles are in the next scene. If no world is loaded, it uses
  the built-in `DEFAULT_ROOM`.
- **Pose**: `auto` = sim car truth (topic `car`, else a `car`/`robot`/`pose` in the world
  message) when fresh, otherwise a scripted scan path. `car` waits for truth; `script` always
  follows the script.
- **Surfaces** (Andrew's splat rows, `SceneCapture.surfaceSamples` format): one per 4th depth
  sample, ≤ 512 per scene, with the hit's normal and a colour from **what was hit** (labels as
  in the 2D editor, walls grey, floor dark), not from a camera image. Andrew's `Map` keeps them
  and his room view draws them as Gaussian patches (**Surface** button).
- Not modelled: the confidence map, real RGB, inference/persistent points, and the real phone's
  frame drops.

## Things to know

- Occupancy is Andrew's **Python reference grid** (`Map(native=None)`). On the robot the grid
  comes from goto's native map (`NativeGrid` → goto 5570), but on the bench only the robot plugin
  talks to goto. His Map keeps obstacles **until cleared**, so a moved obstacle leaves its old
  cells (his documented behaviour). Use **clear map**.
- Overlays added on top of his snapshot: the sim car's truth appears as an object called
  "sim car truth" (only with the sim source), and goto's `NAV ROUTE` fills `path` when his
  preview path is empty and the robot plugin is loaded. That route is in goto's nav frame.
- `serve_room` binds the module global `room_view.PORT`, which the pipeline sets (5594 by default).

Tests: `python3 bench/tests/test_phone.py` (no phone needed). Andrew's own:
`python3 -m unittest discover -s iter2-autonomy/dashboard -p 'test_*.py'` (the native-map tests
need `make -C iter2-autonomy/goto host` first).

## Phone in the loop: `python3 bench/run_sim.py --phone usb`

The real phone replaces the world sim; the code it feeds is the Pi's, built for the Mac. The virtual car takes
the MOVE/TURN, and nothing moves the phone, so carry it at car height (~14 cm) along the route goto draws.

```
phone 5562 LDR1 ─usbmuxd─► link.py 127.0.0.1:5562 ─► lidar_guard ─GAT1/GOB1 UDP─► tars_motor -g (guard gate)
                                  └─ tee, TCP ────────────────────────────────────► goto nav port (map + ARKit pose)
phone 5563 RGB1 ─usbmuxd─► link.py 127.0.0.1:5563 ─► rgb_broker :5566 ─► detect (YOLO-World, ORT) ─UDP─► goto 5571
goto ─MOVE/TURN─► tars_motor ─► virtual SM11 (real firmware on a pty)        bench --target phone: live 2D view
```

On the Pi it's the same except `iproxy` instead of `link.py`, and `lidar_guard --nav-port` tees over UDP. A full
LDR1 frame (1024 points) is 12332 B, over macOS's 9216 B UDP datagram cap, so on the Mac link.py tees to goto
over TCP (goto takes both). The bytes to lidar_guard are passed through unchanged.

Needs: `make -C iter2-autonomy/goto host-detect-phone` (ONNX Runtime + jpeg-turbo; `ORT_PHONE=<ORT SDK dir>`
if ORT isn't from brew), `iter2-autonomy/goto/models/yolo_world.onnx` + `labels.txt` (get_model.sh). Without
detect it still runs (map, pose, guard) and GOTO reports no camera frames. `lidar_guard` builds itself.
Not set for the phone yet: `hfov_deg` (goto.conf keeps the C270's 60; the upright phone frame is narrower,
unmeasured) and `--detect-rotate` (270, as on the Pi; check with detect `--save-frame`).
Rehearse without a phone: `python3 bench/phone/fake_phone.py`, then `run_sim.py --phone tcp:127.0.0.1
--phone-ports <ldr>,<rgb>`. Test: `python3 bench/tests/test_phone_loop.py`.
