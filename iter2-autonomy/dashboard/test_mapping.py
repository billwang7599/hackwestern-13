"""Deterministic mapping, preview, persistence and framing tests; no hardware."""
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import mapping


class Clock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now

    def advance(self, seconds=0.6):
        self.now += seconds


def frame(points=None, objects=None, **changes):
    result = dict(version=1, session='session-a', timestamp=1.0,
                  tracking='normal', pose=[0.05, 0.5, 0.05, 0.0], floor_y=0.0,
                  points=points or [], objects=objects or [])
    result.update(changes)
    return result


def target(x=2.55, z=0.05, label='chair'):
    return dict(label=label, confidence=0.9, position=[x, 0.5, z])


def floor_points():
    return [[round(x / 10 + 0.05, 3), 0.0, round(z / 10 + 0.05, 3)]
            for x in range(-10, 30) for z in range(-15, 15)]


class FragmentedSocket:
    def __init__(self, data):
        self.data = data

    def recv(self, size):
        result, self.data = self.data[:min(size, 7)], self.data[min(size, 7):]
        return result


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.mapper = mapping.Map(clock=self.clock)

    def cell_value(self, x, z):
        cell = self.mapper.cell(x, z)
        return self.mapper.snapshot()['grid'][cell[1] * mapping.SIZE + cell[0]]

    def select(self):
        object_id = self.mapper.snapshot()['objects'][0]['id']
        result = self.mapper.command({'op': 'select_target', 'id': object_id})
        self.assertTrue(result['ok'])
        return self.mapper.snapshot()

    def test_world_geometry_and_height_projection(self):
        self.mapper.update(frame(points=[[2.05, -1.0, -2.05], [2.15, -0.6, -2.05],
                                         [2.25, 0.4, -2.05], [2.35, -1.4, -2.05]],
                                 floor_y=-1.0, pose=[4, 0, -3, math.pi]))
        self.assertEqual(self.cell_value(2.05, -2.05), '1')
        self.assertEqual(self.cell_value(2.15, -2.05), '2')
        self.assertEqual(self.cell_value(2.25, -2.05), '0')  # Above obstacle band.
        self.assertEqual(self.cell_value(2.35, -2.05), '0')  # Below floor.
        # World points are independent of the camera's changing pose/yaw.
        self.mapper.update(frame(points=[[2.05, -1.0, -2.05]], timestamp=2,
                                 floor_y=-1.0, pose=[-3, 1, 4, -math.pi / 2]))
        self.assertEqual(self.cell_value(2.05, -2.05), '1')
        self.assertEqual(self.mapper.snapshot()['voxels'], 4)

    def test_obstacle_cannot_be_erased_by_ground_return(self):
        self.mapper.update(frame(points=[[0.05, 0.3, 0.05], [0.05, 0, 0.05]]))
        self.mapper.update(frame(points=[[0.05, 0, 0.05]], timestamp=2))
        self.assertEqual(self.cell_value(0.05, 0.05), '2')

    def test_path_detours_obstacle_with_clearance_and_standoff(self):
        obstacles = [[1.05, 0.5, round(z / 10 + 0.05, 3)] for z in range(-5, 6)]
        self.mapper.update(frame(points=floor_points() + obstacles, objects=[target()]))
        state = self.select()
        self.assertTrue(state['path'], state['plan_status'])
        self.assertLessEqual(len(state['path']), 256)
        self.assertTrue(any(abs(z - 0.05) > 0.85 for _, z in state['path']))
        final = state['path'][-1]
        self.assertGreaterEqual(math.dist(final, [2.55, 0.05]), 0.6 - 1e-8)
        self.assertLessEqual(math.dist(final, [2.55, 0.05]), 0.85 + 1e-8)
        # Sample every segment, including compressed straight sections.
        for first, last in zip(state['path'], state['path'][1:]):
            steps = max(1, round(math.dist(first, last) / 0.025))
            for step in range(steps + 1):
                p = [first[i] + (last[i] - first[i]) * step / steps for i in (0, 1)]
                self.assertEqual(self.cell_value(*p), '1')
                self.assertGreater(min(math.dist(p, [o[0], o[2]]) for o in obstacles), 0.35)
        self.assertIn('Preview only', state['plan_status'])

    def test_unknown_gap_blocks_route(self):
        points = [p for p in floor_points() if p[0] < 0.6 or p[0] > 1.4]
        self.mapper.update(frame(points=points, objects=[target()]))
        state = self.select()
        self.assertEqual(state['path'], [])
        self.assertIn('No route', state['plan_status'])

    def test_footprint_cannot_overlap_unknown_along_narrow_corridor(self):
        points = [p for p in floor_points() if abs(p[2] - 0.05) < 0.2]
        self.mapper.update(frame(points=points, objects=[target()]))
        state = self.select()
        self.assertEqual(state['path'], [])
        self.assertIn('Observe clear floor', state['plan_status'])

    def test_tracking_floor_session_and_stale_gates(self):
        self.mapper.update(frame(points=floor_points(), objects=[target()]))
        self.assertTrue(self.select()['path'])
        self.clock.advance(1.01)
        stale = self.mapper.snapshot()
        self.assertFalse(stale['live'])
        self.assertEqual(stale['path'], [])
        self.mapper.update(frame(timestamp=2, tracking='limited', points=[[3, 0, 3]]))
        self.assertFalse(self.mapper.snapshot()['live'])
        self.assertEqual(self.cell_value(3, 3), '0')
        self.mapper.update(frame(timestamp=3, floor_y=None))
        self.assertFalse(self.mapper.snapshot()['floor_ready'])
        self.assertEqual(self.mapper.snapshot()['voxels'], 0)
        self.mapper.update(frame(timestamp=4, session='session-b'))
        state = self.mapper.snapshot()
        self.assertEqual(state['session'], 'session-b')
        self.assertEqual(state['objects'], [])
        self.assertEqual(state['frames'], 1)
        self.assertEqual(set(state['grid']), {'0'})

    def test_floor_recalibration_discards_old_geometry(self):
        self.mapper.update(frame(points=[[0, 0, 0]], objects=[target()]))
        self.mapper.update(frame(timestamp=2, floor_y=-0.5))
        self.assertEqual(self.mapper.snapshot()['voxels'], 0)
        self.assertEqual(self.mapper.snapshot()['objects'], [])

    def test_stale_object_memory_is_not_a_target(self):
        self.mapper.update(frame(points=floor_points(), objects=[target()]))
        self.clock.advance(5.1)
        self.mapper.update(frame(timestamp=2))
        state = self.select()
        self.assertTrue(state['live'])
        self.assertEqual(len(state['objects']), 1)
        self.assertEqual(state['path'], [])
        self.assertIn('Target stale', state['plan_status'])

    def test_objects_before_floor_are_remembered_without_authorizing_paths(self):
        self.mapper.update(frame(objects=[target()], floor_y=None))
        state = self.select()
        self.assertEqual(len(state['objects']), 1)
        self.assertTrue(state['tracking_live'])
        self.assertTrue(state['objects'][0]['currently_seen'])
        self.assertFalse(state['floor_ready'])
        self.assertTrue(state['floor_assumed'])
        self.assertAlmostEqual(state['floor_y'], state['pose'][1] - 0.13803)
        self.assertIsNone(self.mapper.floor_y)
        self.assertFalse(state['live'])
        self.assertEqual(state['path'], [])
        self.assertEqual(set(state['grid']), {'0'})

    def test_tilted_mount_heights_and_native_floor_authority(self):
        theta = math.radians(12)
        rot = [[1,0,0,0], [0,math.cos(theta),math.sin(theta),0],
               [0,-math.sin(theta),math.cos(theta),0], [0,.5,0,1]]
        from native_map import NativeGrid
        self.mapper.native = NativeGrid()
        self.mapper.update(frame(camera_transform_columns=rot, floor_y=None))
        state = self.mapper.snapshot()
        self.assertAlmostEqual(state['camera_height_m'], .1524 - .01437*math.cos(theta))
        self.assertAlmostEqual(state['lidar_height_m'], .1524 - .06416*math.cos(theta))
        self.assertTrue(state['floor_assumed'])
        self.assertFalse(state['floor_ready'])
        self.assertEqual(set(state['grid']), {'0'})
        self.assertAlmostEqual(state['floor_y'], .5-state['camera_height_m'])
        roll = [[0,1,0,0], [-1,0,0,0], [0,0,1,0], [0,0,0,1]]
        self.assertAlmostEqual(mapping.mount_heights(roll)[0], .1524+(.03361-.16343/2))

    def test_assumed_floor_is_replaced_by_measured_level(self):
        self.mapper.update(frame(floor_y=None))
        self.assertTrue(self.mapper.snapshot()['floor_assumed'])
        self.mapper.update(frame(timestamp=2, floor_y=-0.2))
        state = self.mapper.snapshot()
        self.assertTrue(state['floor_ready'])
        self.assertFalse(state['floor_assumed'])
        self.assertEqual(state['floor_y'], -0.2)

    def test_cached_inference_does_not_refresh_object_observation_age(self):
        detection = dict(target(), observation_timestamp=0.5)
        self.mapper.update(frame(objects=[detection]))
        self.assertEqual(self.mapper.snapshot()['objects'][0]['age_s'], 0.5)
        self.clock.advance(1)
        self.mapper.update(frame(timestamp=2, objects=[detection]))
        state = self.mapper.snapshot()['objects'][0]
        self.assertEqual(state['age_s'], 1.5)
        self.assertEqual(state['last_seen_age_s'], 1.5)
        self.clock.advance(4)
        self.mapper.update(frame(timestamp=6))
        state = self.mapper.snapshot()['objects'][0]
        self.assertEqual(state['age_s'], 5.5)
        self.assertFalse(state['currently_seen'])
        self.assertTrue(state['stale'])
        self.assertEqual(self.select()['path'], [])

    def test_new_observation_refreshes_memory_but_disconnect_marks_it_stale(self):
        self.mapper.update(frame(objects=[target()]))
        self.clock.advance(6)
        self.mapper.update(frame(timestamp=7, objects=[target(x=2.6)]))
        state = self.mapper.snapshot()['objects'][0]
        self.assertFalse(state['stale'])
        self.assertTrue(state['currently_seen'])
        self.assertEqual(state['age_s'], 0)
        self.mapper.disconnect()
        state = self.mapper.snapshot()['objects'][0]
        self.assertTrue(state['stale'])
        self.assertFalse(state['currently_seen'])

    def test_old_or_invalid_source_observations_cannot_become_fresh_targets(self):
        self.mapper.update(frame(timestamp=10, objects=[dict(target(), observation_timestamp=1)]))
        self.assertEqual(self.mapper.snapshot()['objects'], [])
        for observed in (True, float('nan'), -1, 12):
            with self.subTest(observed=observed), self.assertRaises(ValueError):
                self.mapper.update(frame(timestamp=11, objects=[dict(target(), observation_timestamp=observed)]))

    def test_stable_nearest_label_object_ids(self):
        self.mapper.update(frame(objects=[target(), target(x=-2)]))
        ids = [o['id'] for o in self.mapper.snapshot()['objects']]
        self.mapper.update(frame(timestamp=2, objects=[target(x=-1.9), target(x=2.6)]))
        self.assertEqual([o['id'] for o in self.mapper.snapshot()['objects']], list(reversed(ids)))
        self.mapper.update(frame(timestamp=3, objects=[target(x=-1.8, label='bottle')]))
        self.assertEqual(len(self.mapper.snapshot()['objects']), 3)

    def test_disconnect_is_immediate_and_old_feed_cannot_disconnect_new(self):
        old, new = object(), object()
        self.mapper.update(frame(points=floor_points(), objects=[target()]), source=old)
        self.assertTrue(self.select()['path'])
        self.mapper.update(frame(timestamp=2), source=new)
        self.mapper.disconnect(source=old)
        self.assertTrue(self.mapper.snapshot()['live'])
        self.mapper.disconnect(source=new)
        state = self.mapper.snapshot()
        self.assertFalse(state['live'])
        self.assertEqual(state['path'], [])

    def test_new_geometry_invalidates_throttled_preview(self):
        self.mapper.update(frame(points=floor_points(), objects=[target()]))
        self.assertTrue(self.select()['path'])
        self.mapper.update(frame(timestamp=2, points=[[0.05, 0.4, 0.05]]))
        self.assertEqual(self.mapper.snapshot()['path'], [])
        self.clock.advance()
        self.assertIn('Observe clear floor', self.mapper.snapshot()['plan_status'])

    def test_bounded_voxels_objects_and_snapshot(self):
        with patch.object(mapping, 'MAX_VOXELS', 10), patch.object(mapping, 'MAX_OBJECTS', 3):
            self.mapper.update(frame(points=[[i, 0, 0] for i in range(20)],
                                     objects=[target(x=i * 2) for i in range(10)]))
        state = self.mapper.snapshot()
        self.assertEqual(state['voxels'], 10)
        self.assertEqual(len(state['objects']), 3)
        self.assertEqual(len(state['grid']), 40000)
        self.assertLess(len(json.dumps(state, allow_nan=False).encode()), 70000)

    def test_invalid_and_out_of_order_input_does_not_mutate(self):
        self.mapper.update(frame(points=[[0, 0, 0]]))
        before = self.mapper.snapshot()
        invalid = [frame(version=True), frame(timestamp=True), frame(timestamp=10 ** 300),
                   frame(pose=[0, 0, float('nan'), 0]), frame(points=[[0, float('inf'), 0]]),
                   frame(points=[[0, 0, 0]] * 1501), frame(objects=[target()] * 21),
                   frame(objects=[dict(target(), confidence=-0.1)]),
                   frame(objects=[dict(target(), label='a' * 65)]),
                   frame(session=''), frame(tracking='unknown'), frame(floor_y=True),
                   frame(timestamp=1)]
        for packet in invalid:
            with self.subTest(packet=str(packet)[:80]):
                with self.assertRaises(ValueError):
                    self.mapper.update(packet)
                self.assertEqual(self.mapper.snapshot(), before)

    def test_map_persistence_waits_for_matching_session_and_fresh_objects(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'state' / 'map.json'
            self.mapper.save_path = path
            self.mapper.update(frame(points=floor_points(), objects=[target()]))
            self.assertTrue(self.mapper.command({'op': 'map_save'})['ok'])
            loaded = mapping.Map(save_path=path, clock=self.clock)
            self.assertTrue(loaded.command({'op': 'map_load'})['ok'])
            loaded.update(frame(session='different'))
            self.assertFalse(loaded.snapshot()['live'])
            self.assertEqual(loaded.snapshot()['voxels'], 0)
            loaded.update(frame(session='session-a', tracking='limited'))
            self.assertEqual(loaded.snapshot()['voxels'], 0)
            loaded.update(frame(session='session-a', timestamp=2))
            state = loaded.snapshot()
            self.assertTrue(state['live'])
            self.assertEqual(state['grid'], self.mapper.snapshot()['grid'])
            self.assertIsNone(state['objects'][0]['age_s'])
            self.assertTrue(state['objects'][0]['restored'])
            self.assertTrue(state['objects'][0]['stale'])
            self.assertFalse(state['objects'][0]['currently_seen'])
            loaded.command({'op': 'select_target', 'id': state['objects'][0]['id']})
            self.assertEqual(loaded.snapshot()['path'], [])
            self.clock.advance()
            loaded.update(frame(timestamp=3, objects=[target()]))
            self.assertTrue(loaded.snapshot()['path'])
            self.assertFalse(loaded.snapshot()['objects'][0]['restored'])
            self.assertTrue(loaded.command({'op': 'map_clear'})['ok'])
            self.assertEqual(loaded.snapshot()['voxels'], 0)
            self.assertEqual(loaded.snapshot()['objects'], [])

    def test_surface_color_fusion_is_visual_only_and_round_trips(self):
        row = [12, 30, -40, 0, 0, 127, 200, 20, 40, 8]
        self.mapper.update(frame(surfaces=[row]))
        updated = row[:]
        updated[6] = 100
        self.mapper.update(frame(timestamp=2, surfaces=[updated]))
        state = self.mapper.snapshot()
        packed = mapping.base64.b64decode(state['surface_splats'])
        self.assertEqual(mapping.SURFACE_RECORD.unpack(packed)[6], 175)
        self.assertEqual(state['voxels'], 0)
        self.assertEqual(set(state['grid']), {'0'})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'map.json'
            self.mapper.save_path = path
            self.assertTrue(self.mapper.command({'op': 'map_save'})['ok'])
            loaded = mapping.Map(save_path=path, clock=self.clock)
            self.assertTrue(loaded.command({'op': 'map_load'})['ok'])
            loaded.update(frame(tracking='limited'))
            self.assertEqual(loaded.snapshot()['surface_splats'], '')
            loaded.update(frame(timestamp=2))
            self.assertEqual(loaded.snapshot()['surface_splats'], state['surface_splats'])
            loaded.update(frame(session='new'))
            self.assertEqual(loaded.snapshot()['surface_splats'], '')

    def test_invalid_saved_map_rejected_without_destroying_current_map(self):
        with tempfile.TemporaryDirectory() as directory:
            self.mapper.save_path = Path(directory) / 'map.json'
            self.mapper.update(frame(points=[[0, 0, 0]]))
            self.mapper.save_path.write_text('{"version":1,"grid":"2"}')
            self.assertFalse(self.mapper.command({'op': 'map_load'})['ok'])
            self.assertEqual(self.mapper.snapshot()['voxels'], 1)
            self.mapper.disconnect()
            self.assertFalse(self.mapper.command({'op': 'map_save'})['ok'])

    def test_new_session_cancels_pending_restore(self):
        with tempfile.TemporaryDirectory() as directory:
            self.mapper.save_path = Path(directory) / 'map.json'
            self.mapper.update(frame(points=[[0, 0, 0]]))
            self.assertTrue(self.mapper.command({'op': 'map_save'})['ok'])
            self.mapper.update(frame(session='session-b'))
            self.assertTrue(self.mapper.command({'op': 'map_load'})['ok'])
            self.mapper.update(frame(session='session-b', timestamp=2))
            self.assertFalse(self.mapper.snapshot()['live'])
            self.mapper.update(frame(session='session-c'))
            self.assertTrue(self.mapper.snapshot()['live'])
            self.assertIsNone(self.mapper.pending_restore)
            self.assertEqual(self.mapper.snapshot()['voxels'], 0)

    def test_packet_framing_and_limits(self):
        payload = json.dumps(frame()).encode()
        packet = struct.pack('!I', len(payload)) + payload
        self.assertEqual(mapping.read_scene(FragmentedSocket(packet)), frame())
        for data in [struct.pack('!I', mapping.MAX_PACKET + 1), struct.pack('!I', 0),
                     struct.pack('!I', 2) + b'[]', struct.pack('!I', 2) + b'xx',
                     struct.pack('!I', 2) + b'\xff\xff']:
            with self.subTest(data=data):
                with self.assertRaises(ValueError):
                    mapping.read_scene(FragmentedSocket(data))
        with self.assertRaises(ConnectionError):
            mapping.read_scene(FragmentedSocket(packet[:-1]))

    def test_scene_feed_reuses_mapper(self):
        feed = mapping.SceneFeed(mapper=self.mapper)
        self.assertIs(feed.mapper, self.mapper)
        self.assertEqual(feed.port, 5561)
        self.assertFalse(feed.stop.is_set())

    def test_scene_feed_acknowledges_only_valid_processed_frames(self):
        for valid in (True, False):
            feed = mapping.SceneFeed(mapper=self.mapper)
            payload = json.dumps(frame(version=1 if valid else 9)).encode()

            class Connection(FragmentedSocket):
                def __init__(self):
                    super().__init__(struct.pack('!I', len(payload)) + payload)
                    self.sent = []

                def __enter__(self):
                    return self

                def __exit__(self, *args):
                    feed.stop.set()

                def settimeout(self, value):
                    pass

                def sendall(self, data):
                    self.sent.append(data)
                    feed.stop.set()

            connection = Connection()
            with patch.object(mapping.socket, 'create_connection', return_value=connection):
                feed.run()
            self.assertEqual(connection.sent, [b'\x01'] if valid else [])
            self.assertFalse(feed.snapshot()['live'])


if __name__ == '__main__':
    unittest.main()
