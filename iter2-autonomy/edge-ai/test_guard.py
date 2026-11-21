"""Socket-level guard tests. Portable; no phone, AI model or motor is required."""
import json
import math
import os
from pathlib import Path
import socket
import struct
import subprocess
import time
import unittest

HERE = Path(__file__).resolve().parent
BINARY = Path(os.environ.get('GUARD_BINARY', str(HERE / 'build/lidar_guard')))


def floor_points():
    return [[x, 0.0, z] for x in (-0.3, 0.0, 0.3) for z in (0.25, 0.4, 0.6, 0.8, 1.0)]


def packet(seq=1, source_us=None, flags=7, points=None, pose=(0, 0.5, 0, 0), floor=0):
    if source_us is None:
        source_us = int(time.monotonic() * 1_000_000)
    if points is None:
        points = floor_points()
    body = struct.pack('!4sIQII5f', b'LDR1', seq, source_us, flags, len(points), *pose, floor)
    body += b''.join(struct.pack('!3f', *point) for point in points)
    return struct.pack('!I', len(body)) + body


class GuardFixture(unittest.TestCase):
    fusion_enabled = False
    require_clear = False
    @classmethod
    def setUpClass(cls):
        if 'GUARD_BINARY' not in os.environ:
            subprocess.run(['sh', str(HERE / 'build-guard.sh')], check=True)

    def setUp(self):
        self.server = socket.socket()
        self.server.bind(('127.0.0.1', 0))
        self.server.listen(2)
        self.server.settimeout(2)
        self.status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status.bind(('127.0.0.1', 0))
        self.status.settimeout(0.1)
        self.fusion = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as unused:
            unused.bind(('127.0.0.1', 0))
            self.fusion_port = unused.getsockname()[1]
        self.proc = subprocess.Popen([str(BINARY), '--port', str(self.server.getsockname()[1]),
                                      '--status-port', str(self.status.getsockname()[1]),
                                      '--fusion-port', str(self.fusion_port if self.fusion_enabled else 0),
                                      '--require-clear', '1' if self.require_clear else '0'],
                                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        self.client, _ = self.server.accept()
        self.client.settimeout(1)

    def tearDown(self):
        self.client.close()
        self.server.close()
        self.proc.terminate()
        try:
            self.proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
            self.fail('Guard did not terminate promptly')
        self.status.close()
        self.fusion.close()

    def wait_status(self, predicate, timeout=1):
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            try:
                data = self.status.recv(4096)
            except socket.timeout:
                continue
            last = json.loads(data)
            if predicate(last):
                return last
        self.fail(f'No matching guard status; last={last}')

    def send_packet(self, value):
        self.client.sendall(value)
        self.assertEqual(self.client.recv(1), b'\x01')

class GuardTests(GuardFixture):
    def test_clear_then_stale_watchdog_keeps_ticking(self):
        self.send_packet(packet())
        clear = self.wait_status(lambda s: s['permit'])
        self.assertEqual(clear['verdict'], 'CLEAR')
        self.assertEqual(clear['floor_hits'], [5, 5, 5])
        stale = self.wait_status(lambda s: s['verdict'] == 'STALE' and s['frames'] == 1)
        self.assertFalse(stale['permit'])
        self.assertEqual(stale['reason'], 'frame_late')
        self.assertGreaterEqual(stale['sample_age_ms'], 150)
        self.assertLess(stale['sample_age_ms'], 270)
        self.assertGreater(stale['ticks'], clear['ticks'])

    def test_blocked_takes_precedence_over_floor_coverage(self):
        self.send_packet(packet(points=floor_points() + [[0, .2, .35]]))
        result = self.wait_status(lambda s: s.get('frames') == 1)
        self.assertEqual(result['verdict'], 'BLOCKED')
        self.assertFalse(result['permit'])
        self.assertAlmostEqual(result['nearest_m'], .35, places=3)

    def test_unknown_when_coverage_or_flags_missing(self):
        for seq, (flags, reason) in enumerate(((0, 'tracking_limited'), (5, 'set_floor'),
                                               (3, 'aim_forward'), (7, 'no_depth_points')), start=1):
            self.send_packet(packet(seq=seq, flags=flags, points=[] if flags == 7 else floor_points()))
            result = self.wait_status(lambda s: s.get('frames') == seq)
            self.assertEqual(result['verdict'], 'UNKNOWN')
            self.assertEqual(result['reason'], reason)
            self.assertFalse(result['permit'])
    def test_tracking_and_sparse_coverage_policy(self):
        for seq, (flags, points, verdict, permit) in enumerate(((3, floor_points(), 'UNKNOWN', False),
                (7, [[0, 0, .4]] * 15, 'CAUTION', False if self.require_clear else True),
                (7, [], 'UNKNOWN', False)), start=1):
            self.send_packet(packet(seq=seq, flags=flags, points=points))
            result = self.wait_status(lambda s: s.get('frames') == seq)
            self.assertEqual(result['verdict'], verdict)
            self.assertEqual(result['permit'], permit)

    def test_partial_coverage_permitted(self):
        self.send_packet(packet(points=[[0, 0, .4]]))
        result = self.wait_status(lambda s: s.get('frames') == 1)
        self.assertEqual(result['verdict'], 'CAUTION')
        self.assertEqual(result['permit'], not self.require_clear)

    def test_missing_calibration_does_not_publish_obstacle_distance(self):
        self.send_packet(packet(flags=3, points=floor_points() + [[0, .2, .35]]))
        result = self.wait_status(lambda s: s.get('frames') == 1)
        self.assertEqual(result['verdict'], 'UNKNOWN')
        self.assertIsNone(result['nearest_obstacle_m'])
        self.assertIsNone(result['nearest_m'])

    def test_world_pose_rotation_and_height(self):
        # Camera at (2, .5, 3), looking +x: forward maps to world x,
        # side maps to -world z. Ground coordinates are translated accordingly.
        points = [[2 + z, 1, 3 - x] for x, _, z in floor_points()]
        points.append([2.25, 1.3, 3])
        self.send_packet(packet(points=points, pose=(2, 1.5, 3, math.pi / 2), floor=1))
        result = self.wait_status(lambda s: s.get('frames') == 1)
        self.assertEqual(result['verdict'], 'BLOCKED')
        self.assertEqual(result['floor_hits'], [5, 5, 5])
        self.assertAlmostEqual(result['nearest_m'], .25, places=3)

    def test_disconnect_immediately_revokes_permission(self):
        self.send_packet(packet())
        self.wait_status(lambda s: s['permit'])
        self.client.close()
        result = self.wait_status(lambda s: not s['connected'] and s.get('frames') == 1)
        self.assertEqual(result['verdict'], 'STALE')
        self.assertFalse(result['permit'])
        self.assertLess(result['sample_age_ms'], 150)

    def test_nonfinite_packet_closes_without_ack(self):
        self.client.sendall(packet(points=[[float('nan'), 0, .3]]))
        self.assertEqual(self.client.recv(1), b'')
        result = self.wait_status(lambda s: s['invalid_packets'] == 1)
        self.assertEqual(result['frames'], 0)
        self.assertFalse(result['permit'])

    def test_oversized_packet_rejected_before_body(self):
        self.client.sendall(struct.pack('!I', 12333))
        self.assertEqual(self.client.recv(1), b'')
        result = self.wait_status(lambda s: s['invalid_packets'] == 1)
        self.assertEqual(result['verdict'], 'STALE')

    def test_duplicate_timestamp_revokes_permission(self):
        stamp = int(time.monotonic() * 1_000_000)
        self.send_packet(packet(source_us=stamp))
        self.wait_status(lambda s: s['permit'])
        self.client.sendall(packet(seq=2, source_us=stamp))
        self.assertEqual(self.client.recv(1), b'')
        result = self.wait_status(lambda s: s['invalid_packets'] == 1)
        self.assertEqual(result['verdict'], 'STALE')
        self.assertEqual(result['frames'], 1)

    def test_fragmented_payload_acknowledged_only_after_complete(self):
        data = packet()
        self.client.sendall(data[:7])
        self.client.settimeout(.03)
        with self.assertRaises(socket.timeout):
            self.client.recv(1)
        self.client.settimeout(1)
        self.client.sendall(data[7:])
        self.assertEqual(self.client.recv(1), b'\x01')
        self.assertTrue(self.wait_status(lambda s: s.get('frames') == 1)['permit'])

    def test_growth_in_source_backlog_prevents_clearance(self):
        stamp = int(time.monotonic() * 1_000_000)
        self.send_packet(packet(source_us=stamp))
        self.wait_status(lambda s: s['permit'])
        time.sleep(.12)
        # Source only advanced 1 ms while receipt advanced >=120 ms.
        self.send_packet(packet(seq=2, source_us=stamp + 1000))
        result = self.wait_status(lambda s: s.get('frames') == 2)
        self.assertGreater(result['backlog_ms'], 100)
        self.assertEqual(result['verdict'], 'STALE')

    def test_stalled_external_worker_does_not_stop_guard_ticks(self):
        worker = subprocess.Popen(['python3', '-c', 'import time; time.sleep(10)'])
        try:
            first = self.wait_status(lambda s: s['ticks'] >= 2)
            last = self.wait_status(lambda s: s['ticks'] >= first['ticks'] + 20)
            self.assertEqual(last['verdict'], 'STALE')
            self.assertGreater(last['uptime_ms'], first['uptime_ms'])
            self.assertIn('schedule_error', last)
            self.assertIn('affinity_error', last)
        finally:
            worker.terminate()
            worker.wait(timeout=2)


class FusionTests(GuardFixture):
    fusion_enabled = True

    def fusion_packet(self, nearest=.4, ttl_us=400000, sent=None, expires=None):
        sent = time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000 if sent is None else sent
        return struct.pack('!4sQQf', b'FUS1', sent, sent + ttl_us if expires is None else expires, nearest)

    def send_fusion(self, value=None):
        value = self.fusion_packet() if value is None else value
        self.fusion.sendto(value, ('127.0.0.1', self.fusion_port))
        return value

    def test_inferred_veto_and_distance_warning(self):
        self.send_packet(packet(points=floor_points() + [[0, .2, 1.4]]))
        self.wait_status(lambda s: s['permit'])
        self.send_fusion(self.fusion_packet(.9))
        warning = self.wait_status(lambda s: s['inferred_live'])
        self.assertTrue(warning['permit'])
        self.assertAlmostEqual(warning['nearest_obstacle_m'], .9, places=3)
        self.assertAlmostEqual(warning['lidar_nearest_m'], 1.4, places=3)
        self.send_packet(packet(seq=2, points=floor_points() + [[0, .2, 1.4]]))
        self.send_fusion(self.fusion_packet(.6))
        blocked = self.wait_status(lambda s: s['stop_source'] == 'inferred')
        self.assertEqual(blocked['verdict'], 'BLOCKED')
        self.assertEqual(blocked['lidar_verdict'], 'CLEAR')
        self.assertFalse(blocked['permit'])

    def test_exact_stop_boundary_stays_blocked_across_heartbeats(self):
        self.send_packet(packet())
        self.send_fusion(self.fusion_packet(.6, ttl_us=400000))
        previous = self.wait_status(lambda s: s['stop_source'] == 'inferred')
        # Reuse one F32 0.6 packet. Its rounded value must veto continuously,
        # not just in the receive tick's observed_stop branch.
        for seq in (2, 3):
            self.send_packet(packet(seq=seq))
            result = self.wait_status(lambda s: s['ticks'] >= previous['ticks'] + 6)
            self.assertEqual(result['inferred_packets'], 1)
            self.assertEqual(result['lidar_verdict'], 'CLEAR')
            self.assertEqual(result['verdict'], 'BLOCKED')
            self.assertEqual(result['stop_source'], 'inferred')
            self.assertFalse(result['permit'])
            previous = result

    def test_inference_never_overrides_lidar_denial(self):
        self.send_fusion()
        stale = self.wait_status(lambda s: s['inferred_live'])
        self.assertEqual(stale['verdict'], 'STALE')
        self.assertFalse(stale['permit'])
        self.assertEqual(stale['stop_source'], 'lidar')
        self.send_packet(packet(flags=3))
        unknown = self.wait_status(lambda s: s.get('frames') == 1)
        self.assertEqual(unknown['verdict'], 'UNKNOWN')
        self.assertFalse(unknown['permit'])
        self.send_fusion(self.fusion_packet(-1))
        self.send_packet(packet(seq=2, points=floor_points() + [[0, .2, .3]]))
        blocked = self.wait_status(lambda s: s.get('frames') == 2)
        self.assertEqual(blocked['verdict'], 'BLOCKED')
        self.assertEqual(blocked['stop_source'], 'lidar')

    def test_expiry_and_replay_fall_back_to_measured_lidar(self):
        self.send_packet(packet())
        replay = self.send_fusion(self.fusion_packet(.4, ttl_us=90000))
        self.wait_status(lambda s: s['stop_source'] == 'inferred')
        time.sleep(.075)
        self.send_packet(packet(seq=2))
        self.send_fusion(replay)
        result = self.wait_status(lambda s: not s['inferred_live'] and s['inferred_invalid'] >= 1)
        self.assertEqual(result['verdict'], 'CLEAR')
        self.assertTrue(result['permit'])
        self.assertIsNone(result['inferred_nearest_m'])
        self.assertIsNone(result['nearest_obstacle_m'])

    def test_explicit_reset_distinct_from_valid_empty_estimate(self):
        self.send_packet(packet())
        self.send_fusion()
        self.wait_status(lambda s: s['stop_source'] == 'inferred')
        self.send_fusion(self.fusion_packet(-1, expires=0))
        result = self.wait_status(lambda s: s['inferred_packets'] == 2)
        self.assertTrue(result['permit'])
        self.assertFalse(result['inferred_live'])
        self.send_packet(packet(seq=2))
        self.send_fusion(self.fusion_packet(-1))
        result = self.wait_status(lambda s: s['inferred_packets'] == 3)
        self.assertTrue(result['permit'])
        self.assertTrue(result['inferred_live'])
        self.assertIsNone(result['inferred_nearest_m'])

    def test_invalid_future_old_overlong_nonfinite_packets_are_ignored(self):
        self.send_packet(packet())
        now = time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000
        invalid = [b'x', b'FUS1' + bytes(29), self.fusion_packet(float('nan')),
                   self.fusion_packet(float('inf')), self.fusion_packet(-2), self.fusion_packet(-.5),
                   self.fusion_packet(sent=now+500000),
                   self.fusion_packet(sent=now-600000),
                   self.fusion_packet(ttl_us=600000), self.fusion_packet(.3, expires=0)]
        for value in invalid: self.send_fusion(value)
        result = self.wait_status(lambda s: s['inferred_invalid'] >= len(invalid))
        self.assertEqual(result['inferred_packets'], 0)
        self.assertFalse(result['inferred_live'])
        self.assertTrue(result['permit'])

    def test_malformed_packet_does_not_erase_existing_veto(self):
        self.send_packet(packet())
        self.send_fusion()
        self.wait_status(lambda s: s['stop_source'] == 'inferred')
        self.send_fusion(b'invalid')
        result = self.wait_status(lambda s: s['inferred_invalid'] >= 1)
        self.assertEqual(result['stop_source'], 'inferred')
        self.assertFalse(result['permit'])


class StrictCoverageTests(GuardTests):
    # All guard/disconnect/invalid-data checks also run under autonomous policy.
    require_clear = True


if __name__ == '__main__':
    unittest.main()
