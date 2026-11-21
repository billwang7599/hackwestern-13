#!/usr/bin/env python3
"""Fetch reproducible official YOLOX-Nano, MiDaS-small and validation image; no model export."""
from hashlib import sha256
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parent
ARTIFACTS = {
    'LICENSE-MiDaS': (
        'https://raw.githubusercontent.com/isl-org/MiDaS/v2_1/LICENSE',
        '99ec0b9f9bcc9234b649787b8f03b07dbece95764b7879e1e72fb76cb0f96876'),
    'LICENSE-YOLOX': (
        'https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/main/LICENSE',
        '0ec3668d3274bcf29e8a29e9576d5a2cd96fc78d3c5bec4387355a796e5d9088'),
    'midas-small.onnx': (
        'https://github.com/isl-org/MiDaS/releases/download/v2_1/model-small.onnx',
        '2d8c6cb8f415229daf1eb041024208e2608c9f98e17c81cc7c6ecb449c56fd58'),
    'yolox_tiny.onnx': (
        'https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_tiny.onnx',
        '427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7'),
    'yolox_nano.onnx': (
        'https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_nano.onnx',
        'c789161ed43c8269fcd4e67c67eeeb4e80c622da2eb296a20bc6007bd18a0b7d'),
    'dog.jpg': (
        'https://raw.githubusercontent.com/Megvii-BaseDetection/YOLOX/main/assets/dog.jpg',
        '5a9522051c3cec2bbd2f6323fccba32e8fbf3ddcc2b3e2fd46b04c720bc6f866'),
}
for name, (url, expected) in ARTIFACTS.items():
    destination = ROOT / name
    data = destination.read_bytes() if destination.exists() else urlopen(url, timeout=60).read(80_000_001)
    actual = sha256(data).hexdigest()
    if len(data) > 80_000_000 or actual != expected:
        raise SystemExit(f'{name}: checksum mismatch; expected {expected}, got {actual}; file not replaced')
    if not destination.exists():
        temporary = destination.with_suffix(destination.suffix + '.tmp')
        temporary.write_bytes(data)
        temporary.replace(destination)
    print(f'Verified {name}: {actual}')
