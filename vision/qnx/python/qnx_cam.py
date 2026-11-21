"""QNX camera frames for the gesture app, via QNX's own codelab helper.

We reuse `qnx_camera_helper.py` from the official QNX codelab "Run MediaPipe face
detection from a QNX camera with Python" (downloaded by setup_pi.sh into
vendor/, not vendored in git). That helper only accepts NV12 (the Pi Camera
Module 3 format). A USB UVC webcam like the Logitech C270 typically delivers
packed YUV 4:2:2 instead, so this subclass also accepts packed frames:

  * frametype == NV12 (1)        -> the helper's NV12 path
  * any other frametype          -> treat framedesc as {height, width, stride}
                                    (the layout of the packed QNX frame
                                    descriptors) and pick the pixel format from
                                    bytes-per-pixel: 2 = YUYV (or UYVY with
                                    packed_order="uyvy"), 3 = RGB888, 4 = RGBX/BGRX

THIS FILE HAS NOT RUN ON QNX YET. The frame type number is logged on the first
frame, so if colours look wrong, or the stride guess fails, the log says what
the camera is really sending. The C++ app (vision/qnx/cpp) uses QNX's
qnx_defs.cc, which handles YCBYCR/CBYCRY/RGB888/RGB8888/BGR8888 with the real
header constants.
"""
from __future__ import annotations

import ctypes as c
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "vendor"))
try:
    import qnx_camera_helper as qh  # noqa: E402
except ImportError as e:  # pragma: no cover - only on a box without setup_pi.sh
    raise SystemExit("vendor/qnx_camera_helper.py missing: run vision/qnx/setup_pi.sh on the Pi") from e

EOK = 0


class PackedDesc(c.Structure):
    _fields_ = [("height", c.c_uint32), ("width", c.c_uint32), ("stride", c.c_uint32)]


def list_units() -> list[int]:
    lib = c.CDLL("libcamapi.so.1")
    lib.camera_get_supported_cameras.restype = c.c_int
    lib.camera_get_supported_cameras.argtypes = [c.c_uint, c.POINTER(c.c_uint), c.c_void_p]
    n = c.c_uint(0)
    if lib.camera_get_supported_cameras(0, c.byref(n), None) != EOK or n.value == 0:
        return []
    arr = (c.c_int * n.value)()
    if lib.camera_get_supported_cameras(n.value, c.byref(n), arr) != EOK:
        return []
    return list(arr[: n.value])


class GestureCamera(qh.QnxCamera):
    def __init__(self, unit: int, packed_order: str = "yuyv", rgb4_order: str = "bgrx"):
        super().__init__(unit)
        self.packed_order = packed_order
        self.rgb4_order = rgb4_order
        self._announced = False

    def _announce(self, b, extra: str) -> None:
        if not self._announced:
            print(f"[cam] unit {self.unit}: frametype={b.frametype} {extra}", flush=True)
            self._announced = True

    def read_rgb(self, timeout_seconds: float = 1.0) -> np.ndarray:
        """Next frame as HxWx3 RGB. Raises TimeoutError if none arrives in time."""
        k = self.cam
        deadline = time.monotonic() + timeout_seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("camera frame timeout")
            if self._receive_pulse_timeout(max(1, int(remaining * 1e9))) == qh.PULSE_TIMED_OUT:
                raise TimeoutError("camera frame timeout")
            if self._pulse.code != qh.PULSE_CODE_DATA:
                continue
            if k.camera_get_viewfinder_buffers(self.handle, self.key, c.byref(self._buffer), None) != EOK:
                continue
            b = self._buffer
            try:
                if not b.framebuf:
                    raise RuntimeError("camera returned a null frame buffer")
                if b.frametype == qh.FRAMETYPE_NV12:
                    d = b.framedesc.nv12
                    w, h, stride, uvo, uvs = int(d.width), int(d.height), int(d.stride), int(d.uv_offset), int(d.uv_stride)
                    self._announce(b, f"NV12 {w}x{h}")
                    size = self._nv12_buffer_size(w, h, stride, uvo, uvs)
                    raw = np.frombuffer(c.string_at(b.framebuf, size), np.uint8)
                    y = raw[: stride * h].reshape(h, stride)[:, :w]
                    uv = raw[uvo:size].reshape(h // 2, uvs)[:, :w].reshape(h // 2, w // 2, 2)
                    return cv2.cvtColorTwoPlane(np.ascontiguousarray(y), np.ascontiguousarray(uv),
                                                cv2.COLOR_YUV2RGB_NV12)
                d = c.cast(c.addressof(b.framedesc), c.POINTER(PackedDesc)).contents
                w, h, stride = int(d.width), int(d.height), int(d.stride)
                if w <= 0 or h <= 0 or stride < w or stride * h > qh.MAX_FRAME_BYTES:
                    raise RuntimeError(f"frametype {b.frametype}: unexpected descriptor "
                                       f"w={w} h={h} stride={stride}; try the C++ app")
                bpp = stride // w
                raw = np.frombuffer(c.string_at(b.framebuf, stride * h), np.uint8).reshape(h, stride)
                if bpp == 2:
                    self._announce(b, f"packed 4:2:2 ({self.packed_order}) {w}x{h}")
                    code = cv2.COLOR_YUV2RGB_YUYV if self.packed_order == "yuyv" else cv2.COLOR_YUV2RGB_UYVY
                    return cv2.cvtColor(np.ascontiguousarray(raw[:, : w * 2].reshape(h, w, 2)), code)
                if bpp == 3:
                    self._announce(b, f"RGB888 {w}x{h}")
                    return np.ascontiguousarray(raw[:, : w * 3].reshape(h, w, 3))
                if bpp >= 4:
                    self._announce(b, f"32-bit ({self.rgb4_order}) {w}x{h}")
                    px = raw[:, : w * 4].reshape(h, w, 4)
                    code = cv2.COLOR_BGRA2RGB if self.rgb4_order == "bgrx" else cv2.COLOR_RGBA2RGB
                    return cv2.cvtColor(np.ascontiguousarray(px), code)
                raise RuntimeError(f"frametype {b.frametype}: {bpp} bytes/pixel unsupported")
            finally:
                k.camera_return_buffer(self.handle, c.byref(b))
