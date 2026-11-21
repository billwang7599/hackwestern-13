"""Uncertain remembered geometry is bounded, ages, and never controls motion."""
import copy
import unittest
from unittest.mock import patch

from mapping import Map
from test_mapping import Clock, floor_points, frame, target


def remembered(points=None, observed=1, mode='direct', confidence=.5, **changes):
    value = frame(**changes)
    value['persistent_points'] = points if points is not None else [[0, .4, 8]]
    value['persistent_calibration'] = dict(
        usage='persistent_map', visualization_only=True, observation_timestamp=observed,
        calibration_timestamp=observed, mode=mode, confidence=confidence, anchor_age_s=0,
        scale=1, offset=.01, heldout_p90_m=.1, heldout_median_m=.05, samples=40)
    return value


class PersistentMemoryTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.mapper = Map(clock=self.clock)

    def test_visual_memory_never_changes_safety_occupancy_or_saved_geometry(self):
        self.mapper.update(remembered([[.05, .3, .45], [0, 2, 18]]))
        state = self.mapper.snapshot()
        self.assertEqual(len(state['distant3d']), 2)
        self.assertEqual(set(state['grid']), {'0'})
        self.assertEqual(state['points3d'], [])
        self.assertEqual(state['estimated3d'], [])
        self.assertEqual(state['inferred_obstacle_cells'], 0)
        self.assertEqual(self.mapper.inference_veto(), dict(nearest_m=-1, expires=0))
        saved = self.mapper._saved()
        self.assertEqual(saved['voxels'], [])
        self.assertNotIn('distant3d', saved)
        self.assertNotIn('persistent_voxels', saved)

    def test_visual_memory_does_not_block_valid_path(self):
        value = remembered([[.05, .4, .05]], objects=[target()])
        value['points'] = floor_points()
        self.mapper.update(value)
        object_id = self.mapper.snapshot()['objects'][0]['id']
        self.mapper.command({'op': 'select_target', 'id': object_id})
        self.assertTrue(self.mapper.snapshot()['path'])
        self.assertEqual(self.mapper.snapshot()['inferred_obstacle_cells'], 0)

    def test_duplicate_source_and_duplicate_voxels_do_not_strengthen_or_refresh(self):
        self.mapper.update(remembered([[0, .4, 8]] * 10))
        item = copy.deepcopy(next(iter(self.mapper.persistent_voxels.values())))
        self.assertEqual(item['observations'], 1)
        self.clock.advance(5)
        self.mapper.update(remembered(timestamp=6))
        self.assertEqual(next(iter(self.mapper.persistent_voxels.values())), item)
        self.assertLess(self.mapper.snapshot()['distant_confidence'][0], item['confidence'])

    def test_distinct_validated_observations_strengthen_and_refresh(self):
        self.mapper.update(remembered())
        first = self.mapper.snapshot()['distant_confidence'][0]
        self.clock.advance(1)
        self.mapper.update(remembered(observed=2, timestamp=2, mode='anchors'))
        self.assertGreater(self.mapper.snapshot()['distant_confidence'][0], first)
        item = next(iter(self.mapper.persistent_voxels.values()))
        self.assertEqual(item['observations'], 2)
        self.assertEqual(item['seen'], self.clock.now)

    def test_cached_calibration_does_not_amplify_confidence(self):
        first = remembered(mode='cached', confidence=.3)
        self.mapper.update(first)
        for n in (1.5, 2, 2.5, 3):
            self.clock.advance(.5)
            value = remembered(observed=n, timestamp=n, mode='cached', confidence=.3)
            value['persistent_calibration']['calibration_timestamp'] = 1
            self.mapper.update(value)
        item = next(iter(self.mapper.persistent_voxels.values()))
        self.assertLessEqual(item['confidence'], .3)
        self.assertEqual(item['observations'], 0)

    def test_confidence_decays_and_original_age_controls_expiration(self):
        self.mapper.update(remembered(observed=1, timestamp=61))
        self.assertAlmostEqual(self.mapper.snapshot()['distant_memory']['oldest_age_s'], 60)
        self.clock.advance(59.9)
        self.assertEqual(len(self.mapper.snapshot()['distant3d']), 1)
        self.clock.advance(.1)
        self.assertEqual(self.mapper.snapshot()['distant3d'], [])

    def test_expired_observation_cannot_be_reintroduced(self):
        self.mapper.update(remembered())
        self.clock.advance(121)
        self.mapper.update(remembered(timestamp=122))
        self.assertEqual(self.mapper.snapshot()['distant3d'], [])

    def test_reset_new_session_and_floor_recalibration_clear_memory(self):
        for operation in ('reset', 'session', 'floor'):
            with self.subTest(operation=operation):
                mapper = Map(clock=self.clock)
                mapper.update(remembered())
                if operation == 'reset':
                    mapper.command({'op': 'map_clear'})
                elif operation == 'session':
                    mapper.update(frame(session='new-session'))
                else:
                    mapper.update(frame(timestamp=2, floor_y=-.4))
                self.assertEqual(mapper.snapshot()['distant3d'], [])

    def test_tracking_loss_retains_aging_history_but_accepts_no_new_geometry(self):
        self.mapper.update(remembered())
        self.clock.advance(1)
        self.mapper.update(remembered([[0, 1, 10]], observed=2, timestamp=2, tracking='limited'))
        self.assertEqual(self.mapper.snapshot()['distant3d'], [[0, .4, 8]])
        self.assertFalse(self.mapper.snapshot()['tracking_live'])
        self.assertEqual(self.mapper.inference_veto()['expires'], 0)

    def test_lidar_endpoint_and_free_ray_remove_memory_and_cache_cannot_restore(self):
        for endpoint in ([0, .4, 1], [0, .4, 2]):
            with self.subTest(endpoint=endpoint):
                mapper = Map(clock=self.clock)
                mapper.update(remembered([[0, .4, 1]], pose=[0, .4, 0, 0]))
                value = remembered([[0, .4, 1]], timestamp=1.1, pose=[0, .4, 0, 0])
                value['points'] = [endpoint]
                mapper.update(value)
                self.assertEqual(mapper.snapshot()['distant3d'], [])
                mapper.update(remembered([[0, .4, 1]], timestamp=1.2, pose=[0, .4, 0, 0]))
                self.assertEqual(mapper.snapshot()['distant3d'], [])

    def test_floor_return_does_not_erase_an_entire_column(self):
        self.mapper.update(remembered([[0, .4, 1]], pose=[0, .5, 0, 0]))
        value = frame(timestamp=2, pose=[0, .5, 0, 0], points=[[0, 0, 1]])
        self.mapper.update(value)
        self.assertEqual(len(self.mapper.snapshot()['distant3d']), 1)

    def test_bad_metadata_drops_memory_without_corrupting_measured_data(self):
        cases = [('usage', 'conservative_obstacle_veto'), ('visualization_only', False),
                 ('mode', 'invented'), ('confidence', float('nan')), ('confidence', 1.1),
                 ('observation_timestamp', 2), ('anchor_age_s', 31), ('scale', -1),
                 ('offset', float('inf')), ('samples', 39), ('heldout_p90_m', .3),
                 ('calibration_timestamp', 2), ('heldout_median_m', .5)]
        for key, bad in cases:
            with self.subTest(key=key):
                value = remembered()
                value['points'] = [[2, .3, 2]]
                value['persistent_calibration'][key] = bad
                mapper = Map(clock=self.clock)
                mapper.update(value)
                self.assertEqual(mapper.snapshot()['distant3d'], [])
                self.assertEqual(len(mapper.voxels), 1)

    def test_cached_fit_age_is_bounded_and_points_have_20m_range(self):
        value = remembered(mode='cached', observed=5, timestamp=5)
        value['persistent_calibration']['calibration_timestamp'] = 1
        self.mapper.update(value)
        self.assertEqual(self.mapper.snapshot()['distant3d'], [])
        self.mapper.update(remembered([[0, .5, 19], [0, .5, 21]], observed=6, timestamp=6))
        self.assertEqual(self.mapper.snapshot()['distant3d'], [[0, .5, 19]])

    def test_storage_and_wire_sampling_are_bounded(self):
        with patch('mapping.MAX_PERSISTENT_VOXELS', 7):
            self.mapper.update(remembered([[i*.2, .4, 8] for i in range(30)]))
        self.assertEqual(len(self.mapper.persistent_voxels), 7)
        self.assertEqual(len(self.mapper.snapshot()['distant3d']), 7)
        for n in range(20):
            points = [[(i % 32) * .11, .4 + (i // 32) * .11, 2 + n*.11] for i in range(256)]
            self.mapper.update(remembered(points, observed=n+2, timestamp=n+2))
        state = self.mapper.snapshot()
        self.assertLessEqual(len(self.mapper.persistent_voxels), 5000)
        self.assertLessEqual(len(state['distant3d']), 1500)
        self.assertEqual(len(state['distant3d']), len(state['distant_confidence']))

    def test_packet_point_limit_and_nonfinite_points(self):
        for points in ([[0, 1, 1]] * 257, [[0, float('nan'), 1]]):
            with self.assertRaises(ValueError):
                self.mapper.update(remembered(points))


if __name__ == '__main__':
    unittest.main()
