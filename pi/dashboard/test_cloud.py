import unittest
from mapping import Map
from test_mapping import frame


class CloudTests(unittest.TestCase):
    def test_inferred_points_do_not_change_occupancy(self):
        mapper = Map()
        value = frame()
        value['estimated_points'] = [[1, .5, 2], [2, 0, 2]]
        value['depth_calibration'] = dict(scale=1, offset=0, samples=40, heldout_p90_m=.1,
                                          observation_timestamp=1, visualization_only=True)
        mapper.update(value)
        result = mapper.snapshot()
        self.assertEqual(set(result['grid']), {'0'})
        self.assertEqual(len(result['estimated3d']), 2)
        self.assertEqual(result['voxels'], 0)

    def test_measured_returns_replace_nearby_estimates(self):
        mapper = Map()
        value = frame()
        value['estimated_points'] = [[1, .5, 2]]
        value['depth_calibration'] = dict(scale=1, offset=0, samples=40, heldout_p90_m=.1,
                                          observation_timestamp=1, visualization_only=True)
        mapper.update(value)
        value = frame(timestamp=2, points=[[1.01, .51, 2.01]])
        value['estimated_points'] = [[1, .5, 2]]
        mapper.update(value)
        result = mapper.snapshot()
        self.assertEqual(result['estimated3d'], [])
        self.assertEqual(len(result['points3d']), 1)

    def test_measured_cloud_visible_before_floor_calibration(self):
        mapper = Map()
        value = frame(points=[[1, 1, 1]])
        value['floor_y'] = None
        mapper.update(value)
        result = mapper.snapshot()
        self.assertEqual(len(result['points3d']), 1)
        self.assertEqual(set(result['grid']), {'0'})
        self.assertFalse(result['floor_ready'])


if __name__ == '__main__':
    unittest.main()
