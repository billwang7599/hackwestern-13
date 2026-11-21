"""Run: python3 -m unittest discover -s laptop -p 'test_joycon_relay.py'."""
import struct
import unittest
from joycon_relay import samples


class ReportTests(unittest.TestCase):
    def test_three_signed_samples(self):
        values = (-32768, 32767, -1, 0, 42, -42)
        report = bytes([0x30]) + bytes(12) + struct.pack('<6h', *values) * 3
        self.assertEqual(samples(report), [values] * 3)

    def test_ignore_other_or_truncated_reports(self):
        for report in (b'', bytes([0x30]) * 48, bytes([0x21]) * 49):
            self.assertEqual(samples(report), [])
