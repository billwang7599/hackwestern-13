#!/usr/bin/env python3
"""Validate the actual native MiDaS result contract; not metric-depth accuracy."""
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
COMMAND = [str(ROOT / 'build/ai_depth'), '--model', str(ROOT / 'models/midas-small.onnx'), '--threads', '1']
JPEG = (ROOT / 'models/dog.jpg').read_bytes()


def job(identifier, payload):
    return struct.pack('!II', identifier, len(payload)) + payload


class DepthTests(unittest.TestCase):
    def invoke(self, data):
        # This QNX ORT package emits a large startup diagnostic burst. A file
        # keeps stderr from filling a pipe while the parent writes a JPEG job.
        with tempfile.TemporaryFile() as diagnostics:
            result = subprocess.run(COMMAND, input=data, stdout=subprocess.PIPE,
                                    stderr=diagnostics, timeout=60)
            diagnostics.seek(0)
            result.stderr = diagnostics.read()
            return result

    def test_real_model_returns_finite_varied_relative_depth(self):
        result = self.invoke(job(17, JPEG))
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        self.assertLess(len(result.stdout), 60000)
        value = json.loads(result.stdout)
        self.assertEqual(value['id'], 17)
        self.assertEqual(value['kind'], 'relative_inverse_depth')
        self.assertEqual((value['depth_width'], value['depth_height']), (64, 64))
        depth = value['inverse_depth']
        self.assertEqual(len(depth), 4096)
        self.assertTrue(all(math.isfinite(v) and v >= 0 for v in depth))
        self.assertGreater(max(depth) - min(depth), 1e-3)
        self.assertGreater(value['inference_ms'], 0)

    def test_bad_jpeg_recovers_before_valid_job(self):
        result = self.invoke(job(1, b'bad jpeg') + job(2, JPEG))
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        values = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([v['id'] for v in values], [1, 2])
        self.assertIn('error', values[0])
        self.assertEqual(values[0]['inverse_depth'], [])
        self.assertEqual(len(values[1]['inverse_depth']), 4096)

    def test_truncation_is_rejected(self):
        result = self.invoke(struct.pack('!II', 1, 100) + b'1234')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, b'')


if __name__ == '__main__':
    unittest.main()
