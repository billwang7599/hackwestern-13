"""Synthetic timing check; does not measure phone/USB latency or drive motors."""
import json
import statistics
import time
from mapping import Map
from test_mapping import frame, target

mapper = Map()
points = [[x / 10 + .05, 0, z / 10 + .05] for x in range(-10, 40) for z in range(-15, 15)]
mapper.update(frame(points=points, objects=[target()]))
object_id = mapper.snapshot()['objects'][0]['id']
rows = [[], [], []]
for i in range(50):
    start = time.perf_counter()
    mapper.update(frame(points=points, objects=[target()], timestamp=i + 2))
    rows[0].append(1000 * (time.perf_counter() - start))
    start = time.perf_counter()
    mapper.command({'op': 'select_target', 'id': object_id})
    rows[1].append(1000 * (time.perf_counter() - start))
    start = time.perf_counter()
    snapshot = mapper.snapshot()
    rows[2].append(1000 * (time.perf_counter() - start))
for name, values in zip(('ingest1500', 'forced plan', 'snapshot'), rows):
    print(name, 'median_ms', round(statistics.median(values), 2), 'max_ms', round(max(values), 2))
print('snapshot_bytes', len(json.dumps(snapshot).encode()))
