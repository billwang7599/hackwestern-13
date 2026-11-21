"""Recreate the pinned Apple-distributed Core ML detector in the local repo."""
import hashlib
from pathlib import Path
import urllib.request

URL = 'https://ml-assets.apple.com/coreml/models/Image/ObjectDetection/YOLOv3Tiny/YOLOv3TinyFP16.mlmodel'
SHA256 = '73406178d0f5793d0d5d1e38274acd146a744c2245c9b63a11998a5015925dda'
DEST = Path(__file__).with_name('YOLOv3TinyFP16.mlmodel')

if __name__ == '__main__':
    data = DEST.read_bytes() if DEST.exists() else urllib.request.urlopen(URL, timeout=60).read()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise SystemExit('Model hash mismatch; leaving the local file unchanged')
    if not DEST.exists():
        DEST.write_bytes(data)
    print(f'Verified {DEST.name}: {SHA256}')
