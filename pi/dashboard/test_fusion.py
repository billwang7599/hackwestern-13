"""Conservative metric-depth fusion: estimates veto, measured LiDAR wins."""
import json
import socket
import struct
import unittest
from unittest.mock import patch

from fusion import FusionPublisher
from mapping import Map
from test_mapping import Clock, floor_points, frame, target


def fused(points=None, observed=1, **changes):
    result = frame(**changes)
    result['estimated_points'] = points if points is not None else [[.05, .3, .45]]
    result['depth_calibration'] = dict(scale=1.1, offset=.02, samples=40, heldout_p90_m=.1,
                                     observation_timestamp=observed,
                                     usage='conservative_obstacle_veto')
    return result


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.mapper = Map(clock=self.clock)

    def test_inference_can_only_add_blocked_space(self):
        self.mapper.update(fused())
        snapshot = self.mapper.snapshot()
        self.assertEqual(set(self.mapper.grid), {0})
        self.assertEqual(set(snapshot['grid']), {'0', '3'})
        self.assertEqual(snapshot['inferred_obstacle_cells'], 1)
        self.assertAlmostEqual(self.mapper.inference_veto()['nearest_m'], .4)
        self.assertEqual(set(self.mapper._saved()['grid']), {'0'})
        self.assertNotIn('estimated_points', self.mapper._saved())

    def test_visualization_only_metadata_never_vetoes(self):
        value = fused()
        del value['depth_calibration']['usage']
        value['depth_calibration']['visualization_only'] = True
        self.mapper.update(value)
        state = self.mapper.snapshot()
        self.assertEqual(len(state['estimated3d']), 1)
        self.assertEqual(state['inferred_obstacle_cells'], 0)
        self.assertEqual(self.mapper.inference_veto()['expires'], 0)

    def test_cached_source_never_refreshes_expiry_or_reinserts_overridden_points(self):
        self.mapper.update(fused())
        deadline = self.mapper.inference_deadline
        self.clock.advance(.6)
        value = fused(timestamp=1.6)
        value['points'] = [[.05, .3, .45]]
        self.mapper.update(value)
        self.assertEqual(self.mapper.inference_deadline, deadline)
        self.assertEqual(self.mapper.inference_veto()['nearest_m'], -1)
        self.clock.advance(.6)
        self.mapper.update(fused(timestamp=2.2))
        self.assertEqual(self.mapper.inference_deadline, deadline)
        self.assertEqual(self.mapper.inference_veto()['nearest_m'], -1)
        self.assertEqual(self.mapper.snapshot()['inferred_obstacle_cells'], 0)

    def test_original_observation_age_reduces_first_lease(self):
        self.mapper.update(fused(observed=.1, timestamp=1.9))
        self.assertAlmostEqual(self.mapper.inference_deadline, self.clock.now + .2)
        self.assertAlmostEqual(self.mapper.inference_veto()['expires'], self.clock.now + .2)
        self.clock.advance(.21)
        self.assertEqual(self.mapper.inference_veto()['expires'], 0)
        self.assertEqual(self.mapper.snapshot()['inferred_obstacle_cells'], 0)

    def test_invalid_calibration_withholds_estimates_but_keeps_measured_map(self):
        for key, value in [('scale', 0), ('scale', -1), ('scale', float('nan')),
                           ('offset', float('inf')), ('samples', 39), ('samples', True),
                           ('heldout_p90_m', .251), ('heldout_p90_m', -1),
                           ('observation_timestamp', 2), ('observation_timestamp', -1)]:
            with self.subTest(key=key, value=value):
                mapper = Map(clock=self.clock)
                scene = fused()
                scene['points'] = [[2, .3, 2]]
                scene['depth_calibration'][key] = value
                mapper.update(scene)
                self.assertEqual(mapper.inference_veto()['expires'], 0)
                self.assertEqual(mapper.snapshot()['inferred_obstacle_cells'], 0)
                self.assertEqual(len(mapper.voxels), 1)

    def test_unmeasured_floor_tracking_loss_disconnect_and_reset_clear_veto(self):
        for change in ('floor', 'tracking', 'disconnect', 'reset'):
            with self.subTest(change=change):
                mapper = Map(clock=self.clock)
                mapper.update(fused())
                self.assertGreater(mapper.inference_veto()['expires'], 0)
                if change == 'floor':
                    mapper.update(fused(timestamp=1.1, floor_y=None))
                elif change == 'tracking':
                    mapper.update(fused(timestamp=1.1, tracking='limited'))
                elif change == 'disconnect':
                    mapper.disconnect()
                else:
                    mapper.command({'op': 'map_clear'})
                self.assertEqual(mapper.inference_veto()['expires'], 0)
                self.assertEqual(mapper.snapshot()['inferred_obstacle_cells'], 0)

    def test_measured_endpoint_and_same_ray_free_volume_override_inference(self):
        for endpoint in ([0, .4, 1], [0, .4, 2]):
            with self.subTest(endpoint=endpoint):
                mapper = Map(clock=self.clock)
                mapper.update(fused([[0, .4, 1]], pose=[0, .4, 0, 0]))
                self.assertEqual(mapper.snapshot()['inferred_obstacle_cells'], 1)
                value = fused([[0, .4, 1]], timestamp=1.2, pose=[0, .4, 0, 0])
                value['points'] = [endpoint]
                mapper.update(value)
                self.assertEqual(mapper.snapshot()['inferred_obstacle_cells'], 0)
                self.assertEqual(mapper.inference_veto()['nearest_m'], -1)

    def test_ground_return_does_not_clear_whole_obstacle_column(self):
        self.mapper.update(fused([[0, .4, 1]], pose=[0, .5, 0, 0]))
        value = fused([[0, .4, 1]], timestamp=1.1, pose=[0, .5, 0, 0])
        value['points'] = [[0, 0, 1]]
        self.mapper.update(value)
        self.assertEqual(self.mapper.snapshot()['inferred_obstacle_cells'], 1)

    def test_measured_obstacle_is_never_removed_by_inference_expiry(self):
        value = fused()
        value['points'] = [[2, .3, 2]]
        self.mapper.update(value)
        before = self.mapper.grid.count(2)
        self.clock.advance(2.1)
        self.mapper.update(frame(timestamp=3.1))
        self.assertEqual(self.mapper.grid.count(2), before)
        self.assertEqual(self.mapper.snapshot()['inferred_obstacle_cells'], 0)

    def test_inferred_obstacle_blocks_preview_then_expires_to_lidar_only(self):
        value = fused([[.05, .4, .05]], objects=[target()])
        value['points'] = floor_points()
        self.mapper.update(value)
        object_id = self.mapper.snapshot()['objects'][0]['id']
        self.mapper.command({'op': 'select_target', 'id': object_id})
        self.assertEqual(self.mapper.snapshot()['path'], [])
        self.clock.advance(2.1)
        self.mapper.update(frame(timestamp=3.1))
        self.assertTrue(self.mapper.snapshot()['path'])

    def test_clear_calibrated_result_differs_from_missing_inference(self):
        self.mapper.update(fused([]))
        veto = self.mapper.inference_veto()
        self.assertEqual(veto['nearest_m'], -1)
        self.assertGreater(veto['expires'], 0)
        self.mapper.update(frame(timestamp=1.2))
        self.assertEqual(self.mapper.inference_veto()['expires'], 0)

    def test_udp_packet_preserves_expiry_and_explicit_invalidation(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(.5)
            publisher = FusionPublisher(receiver.getsockname()[1], clock=self.clock)
            try:
                publisher.publish({'nearest_m': .3, 'expires': self.clock.now + .15})
                magic, sent, expires, near = struct.unpack('!4sQQf', receiver.recv(100))
                self.assertEqual(magic, b'FUS1')
                self.assertEqual(sent, 10_000_000)
                self.assertEqual(expires, 10_150_000)
                self.assertAlmostEqual(near, .3, places=6)
                publisher.clear()
                _, _, expires, near = struct.unpack('!4sQQf', receiver.recv(100))
                self.assertEqual((expires, near), (0, -1))
            finally:
                publisher.close()

    def test_bounded_inferred_storage(self):
        with patch('mapping.MAX_INFERRED_VOXELS', 7):
            self.mapper.update(fused([[i*.2, .3, .4] for i in range(30)]))
        self.assertEqual(len(self.mapper.estimated_voxels), 7)
        self.assertEqual(len(self.mapper.inferred_obstacles), 7)
        json.dumps(self.mapper.snapshot(), allow_nan=False)

    def test_publisher_is_disabled_unless_explicitly_configured(self):
        with patch('fusion.socket.socket', side_effect=AssertionError('Unexpected socket')):
            publisher = FusionPublisher()
            publisher.publish({'nearest_m': .3, 'expires': 100})
            publisher.close()


if __name__ == '__main__':
    unittest.main()
