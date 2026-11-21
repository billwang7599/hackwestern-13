# Detection interface (any detector → `goto`)

`goto` doesn't care which detector produces the boxes. Our `detect` (C++, ONNX Runtime) is one option; the teammate's YOLO is another. Anything that can send **one JSON object per camera frame** can drive the robot.

## Format

One line per frame, UTF-8, no newlines inside:

```json
{"t": 1732170000.123, "w": 640, "h": 480,
 "dets": [{"label": "trash can", "conf": 0.71, "box": [x1, y1, x2, y2]}]}
```

| key | type | meaning |
|---|---|---|
| `w`, `h` | int | **required.** Frame size in pixels, which is what `box` is measured in. |
| `dets` | array | **required.** Send it even when it's empty (`[]`). Empty frames are how `goto` knows the camera is alive but sees nothing. |
| `label` | string | Class name. It goes through the alias table in `iter2-autonomy/goto/objects.txt`, so COCO's `"dining table"` or `"bottle"` work. Upper or lower case doesn't matter. |
| `conf` | number | 0..1. `goto` ignores anything below `min_conf` (0.30). Defaults to 1 if missing. |
| `box` | 4 numbers | `x1, y1, x2, y2` in pixels, top-left origin, **in the full frame, not the letterboxed network input**. |
| `t` | number | Capture time in any units. It's informational only, because `goto` uses arrival time. |

Other keys (`"track_id"`, `"cls"`, nested objects) are allowed and ignored. `"score"` is accepted for `conf`, `"name"` for `label` and `"xyxy"` for `box`.

**Send every processed frame**, including empty ones, at whatever rate you can (5–10 fps is plenty for a 4 cm/s robot). If no frames arrive for 3 s, `goto` stops and reports `no camera frames`.

## Transport (pick one; all go to the same parser)

| How | Address | Use when |
|---|---|---|
| **UDP** (recommended) | `127.0.0.1:5571`, one datagram per frame | Default. The detector can crash or restart without breaking anything. |
| TCP | `127.0.0.1:5571`, newline-separated stream | Your code already has a socket writer. |
| stdin | `detector \| goto -i` | Quick pipes and testing. |

The ports are bound to localhost only, so the detector runs on the Pi next to `goto`.

## Plugging in the teammate's YOLO

**C/C++ (on the robot, QNX native).** Copy `to_json()` from `iter2-autonomy/goto/src/yolo_post.hpp`, which is header-only with no dependencies, or write the string yourself:

```cpp
#include "yolo_post.hpp"                    // iter2-autonomy/goto/src
// dets: std::vector<yolo::Det>{cls, conf, x1, y1, x2, y2} in frame pixels
std::string line = yolo::to_json(t, frame_w, frame_h, dets, class_names);
sendto(udp_fd, line.data(), line.size(), 0, (sockaddr*)&goto_addr, sizeof goto_addr);  // 127.0.0.1:5571
```

If your network outputs raw YOLOv8/YOLO11 tensors, `yolo::letterbox()` and `yolo::decode()` in the same header do the pre- and post-processing, including NMS and undoing the letterbox.

**Python (laptop testing only; the robot runs C/C++).** Three lines on top of an ultralytics loop:

```python
import json, socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for r in model.predict(source=0, stream=True, imgsz=320, verbose=False):
    h, w = r.orig_shape
    dets = [{"label": r.names[int(c)], "conf": float(p), "box": [float(v) for v in b]}
            for b, c, p in zip(r.boxes.xyxy.tolist(), r.boxes.cls.tolist(), r.boxes.conf.tolist())]
    sock.sendto(json.dumps({"t": 0, "w": w, "h": h, "dets": dets}).encode(), ("127.0.0.1", 5571))
```

To drive the robot from a laptop detector, run `goto` on the laptop as well (`iter2-autonomy/goto/build/goto_host -m <pi-ip>:5555`). It's a dev-only setup: Wi-Fi latency makes steering worse.

## Checking your output

```sh
echo '{"w":640,"h":480,"dets":[{"label":"bin","conf":0.9,"box":[300,200,340,300]}]}' \
  | nc -u -w0 127.0.0.1 5571          # goto STATUS frames= should go up during a GOTO
printf 'STATUS\n' | nc 127.0.0.1 5570
```
