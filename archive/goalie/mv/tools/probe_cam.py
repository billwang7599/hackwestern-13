"""Probe OpenCV/AVFoundation camera indices: open, read, measure fps, mean brightness."""
import time
import cv2

for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
    if not cap.isOpened():
        print(i, "not opened"); continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, f = cap.read()
    t = time.time(); n = 0; f2 = f
    for _ in range(20):
        ok2, fr = cap.read()
        if ok2: n += 1; f2 = fr
    dt = time.time() - t
    print(i, "opened read_ok=", ok, None if f is None else f.shape,
          "mean=", None if f2 is None else round(float(f2.mean()), 1),
          "fps_prop=", cap.get(cv2.CAP_PROP_FPS), "measured_fps=", round(n / dt, 1))
    cap.release()
