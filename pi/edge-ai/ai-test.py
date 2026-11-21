#!/usr/bin/env python3
"""Run against the actual compiled native detector, including real model inference."""
import json
import math
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent
COMMAND = [str(ROOT / 'build/ai_detector'), '--model', str(ROOT / 'models/yolox_nano.onnx'), '--threads', '1']
JPEG = (ROOT / 'models/dog.jpg').read_bytes()


def job(identifier, payload):
    return struct.pack('!II', identifier, len(payload)) + payload


class DetectorTests(unittest.TestCase):
    def invoke(self, data):
        # This QNX ORT package emits a large startup diagnostic burst. A file
        # keeps stderr from filling a pipe while the parent writes a JPEG job.
        with tempfile.TemporaryFile() as diagnostics:
            result = subprocess.run(COMMAND, input=data, stdout=subprocess.PIPE,
                                    stderr=diagnostics, timeout=30)
            diagnostics.seek(0)
            result.stderr = diagnostics.read()
            return result

    def test_real_model_detects_upstream_dog_image(self):
        result = self.invoke(job(91, JPEG))
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        lines = result.stdout.splitlines()
        self.assertEqual(len(lines), 1)
        value = json.loads(lines[0])
        self.assertEqual(value['id'], 91)
        self.assertEqual(value['model'], 'YOLOX-Nano')
        self.assertIn('dog', [obj['label'] for obj in value['objects']])
        self.assertLessEqual(len(value['objects']), 20)
        for metric in ('processing_ms', 'inference_ms'):
            self.assertTrue(math.isfinite(value[metric]) and value[metric] >= 0)
        for obj in value['objects']:
            self.assertTrue(0.35 <= obj['confidence'] <= 1)
            x, y, width, height = obj['bbox']
            self.assertTrue(all(math.isfinite(v) for v in obj['bbox']))
            self.assertGreater(width, 0)
            self.assertGreater(height, 0)
            self.assertGreaterEqual(x, 0)
            self.assertGreaterEqual(y, 0)
            self.assertLessEqual(x + width, 1.000002)
            self.assertLessEqual(y + height, 1.000002)

    def test_bad_jpeg_does_not_desynchronize_next_job(self):
        result = self.invoke(job(3, b'bad jpeg') + job(4, JPEG))
        self.assertEqual(result.returncode, 0, result.stderr.decode())
        values = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertEqual([v['id'] for v in values], [3, 4])
        self.assertIn('error', values[0])
        self.assertNotIn('error', values[1])
        self.assertIn('dog', [obj['label'] for obj in values[1]['objects']])

    def test_oversized_packet_is_rejected_without_waiting_for_body(self):
        result = self.invoke(struct.pack('!II', 5, 2_000_001))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, b'')
        self.assertIn(b'length', result.stderr)

    def test_truncated_body_terminates_worker(self):
        result = self.invoke(struct.pack('!II', 6, 100) + b'1234')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, b'')
        self.assertIn(b'Truncated', result.stderr)


if __name__ == '__main__':
    unittest.main()
