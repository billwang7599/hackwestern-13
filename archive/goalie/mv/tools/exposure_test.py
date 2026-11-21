"""Does the capture backend honour exposure / fps requests?  Prints the
readback and mean brightness for several settings.  usage: exposure_test.py DEV"""
import sys, time
import cv2
dev = int(sys.argv[1]) if len(sys.argv) > 1 else 0

def measure(cap, n=30):
    for _ in range(10): cap.read()
    t = time.time(); m = []; k = 0
    for _ in range(n):
        ok, f = cap.read()
        if ok: m.append(f.mean()); k += 1
    return sum(m) / max(1, len(m)), k / (time.time() - t)

for label, props in [("default", {}),
                     ("fps=60", {cv2.CAP_PROP_FPS: 60}),
                     ("auto_exp=0.25 exp=-7", {cv2.CAP_PROP_AUTO_EXPOSURE: 0.25, cv2.CAP_PROP_EXPOSURE: -7}),
                     ("auto_exp=1 exp=-10", {cv2.CAP_PROP_AUTO_EXPOSURE: 1, cv2.CAP_PROP_EXPOSURE: -10}),
                     ("exp=0.002", {cv2.CAP_PROP_EXPOSURE: 0.002}),
                     ("1280x720", {cv2.CAP_PROP_FRAME_WIDTH: 1280, cv2.CAP_PROP_FRAME_HEIGHT: 720})]:
    cap = cv2.VideoCapture(dev, cv2.CAP_AVFOUNDATION)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    rets = {int(k): cap.set(k, v) for k, v in props.items()}
    mean, fps = measure(cap)
    print(f"{label:24s} set_ok={rets} -> {int(cap.get(3))}x{int(cap.get(4))} fps_prop={cap.get(5):.1f} "
          f"measured={fps:.1f} exp_prop={cap.get(cv2.CAP_PROP_EXPOSURE)} mean={mean:.1f}")
    cap.release()
