#!/usr/bin/env python3
"""Parity: goto/src/nav.c planner vs pi/dashboard/mapping.py Map._plan.

Mac test harness only (never runs on the robot). Builds shared grid fixtures,
runs Andrew's Python planner on each (imported, not copied), runs the C planner
through build/nav_cli, and requires the same result: the same error, or the
same corner list cell for cell.

    python3 tests/nav_parity.py build/nav_cli [--random N] [--seed S]
"""
import os
import random
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'dashboard'))
import mapping  # noqa: E402

SIZE = mapping.SIZE
ERRORS = {
    'Observe clear floor around the robot before planning': -1,
    'Target outside the 20 m map': -2,
    'No observed clear approach point near target': -3,
    'No route through observed clear floor': -4,
    'Route exceeds preview complexity limit': -6,
}


def f32(v):
    return struct.unpack('<f', struct.pack('<f', v))[0]


def python_plan(grid, radius, sx, sz, tx, tz):
    now = 1000.0
    m = mapping.Map(save_path=os.devnull, clock=lambda: now)
    m.grid = bytearray(grid)
    m.pose = [sx, 0.0, sz, 0.0]
    m.connected, m.received, m.tracking, m.floor_y = True, now, 'normal', 0.0
    m.objects['1'] = {'id': '1', 'label': 'target', 'position': [tx, 0.0, tz],
                      'confidence': 0.9, 'seen': now, 'observation_timestamp': now}
    m.selected = '1'
    old = mapping.ROBOT_RADIUS
    mapping.ROBOT_RADIUS = radius
    try:
        m._plan(now, force=True)
    finally:
        mapping.ROBOT_RADIUS = old
    if m.path:
        return ('OK', [tuple(p) for p in m.path])
    return ('ERR', ERRORS.get(m.plan_status, m.plan_status))


# ---- fixtures -----------------------------------------------------------

def cell_centre(i):
    return f32(-10.0 + (i + 0.5) * 0.1)


def blank():
    return [0] * (SIZE * SIZE)


def fill(g, x0, z0, x1, z1, v):
    for z in range(max(0, z0), min(SIZE, z1)):
        for x in range(max(0, x0), min(SIZE, x1)):
            g[z * SIZE + x] = v


def room(x0=60, z0=60, x1=140, z1=140):
    g = blank()
    fill(g, x0, z0, x1, z1, 1)
    return g


def fixtures(n_random, seed):
    out = []
    c = cell_centre
    g = room(); out.append(('open room', g, 0.35, c(70), c(70), c(130), c(125)))
    g = room(); fill(g, 99, 60, 101, 125, 2)
    out.append(('route around a wall', g, 0.35, c(75), c(100), c(125), c(100)))
    g = room(); fill(g, 99, 60, 101, 140, 2); fill(g, 99, 95, 101, 106, 1)
    out.append(('wall with a door', g, 0.35, c(75), c(80), c(125), c(120)))
    g = room(); fill(g, 85, 85, 115, 88, 2); fill(g, 85, 85, 88, 115, 2); fill(g, 112, 85, 115, 115, 2)
    out.append(('U trap', g, 0.35, c(100), c(95), c(100), c(130)))
    g = room(); fill(g, 90, 70, 110, 130, 0)
    out.append(('unknown blocks', g, 0.35, c(75), c(100), c(125), c(100)))
    g = room(); fill(g, 60, 99, 140, 101, 2)
    out.append(('no path', g, 0.35, c(100), c(75), c(100), c(125)))
    g = room(); fill(g, 70, 60, 72, 140, 2)
    out.append(('start blocked', g, 0.35, c(74), c(100), c(120), c(100)))
    g = room()
    out.append(('target outside', g, 0.35, c(100), c(100), f32(10.5), f32(0.0)))
    g = room(); fill(g, 110, 90, 130, 110, 2); fill(g, 115, 95, 125, 105, 1)
    out.append(('no goal', g, 0.35, c(75), c(100), c(120), c(100)))
    g = room(); out.append(('already on ring', g, 0.35, c(100), c(100), c(107), c(100)))
    g = room(); out.append(('small robot', g, 0.15, c(65), c(65), c(135), c(135)))
    g = room(); fill(g, 80, 60, 81, 100, 2); fill(g, 81, 100, 120, 101, 2)
    out.append(('radius 0.2', g, 0.2, c(70), c(70), c(110), c(90)))
    rng = random.Random(seed)
    for i in range(n_random):
        x0, z0, x1, z1 = rng.randint(30, 70), rng.randint(30, 70), rng.randint(130, 170), rng.randint(130, 170)
        g = room(x0, z0, x1, z1)
        for _ in range(rng.randint(3, 25)):
            x, z = rng.randint(30, 170), rng.randint(30, 170)
            fill(g, x, z, x + rng.randint(1, 20), z + rng.randint(1, 20), rng.choice((2, 2, 2, 0)))
        r = rng.choice((0.15, 0.2, 0.25, 0.35))
        # off-centre float32 positions inside the room (sometimes anywhere)
        lo_x, hi_x = -10 + x0 * 0.1 + 0.5, -10 + x1 * 0.1 - 0.5
        lo_z, hi_z = -10 + z0 * 0.1 + 0.5, -10 + z1 * 0.1 - 0.5
        if rng.random() < 0.1:
            lo_x = lo_z = -11.0
            hi_x = hi_z = 11.0
        pts = [f32(rng.uniform(lo_x, hi_x)), f32(rng.uniform(lo_z, hi_z)),
               f32(rng.uniform(lo_x, hi_x)), f32(rng.uniform(lo_z, hi_z))]
        out.append(('random %d' % i, g, r, *pts))
    return out


def main():
    cli = sys.argv[1]
    n_random = int(sys.argv[sys.argv.index('--random') + 1]) if '--random' in sys.argv else 60
    seed = int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 13
    cases = fixtures(n_random, seed)
    lines = []
    for _, g, r, sx, sz, tx, tz in cases:
        lines.append(''.join(map(str, g)) + ' ' + ' '.join(float(v).hex() for v in (f32(r), sx, sz, tx, tz)))
    res = subprocess.run([cli], input='\n'.join(lines) + '\n', capture_output=True, text=True, check=True)
    c_out = res.stdout.splitlines()
    assert len(c_out) == len(cases), (len(c_out), len(cases), res.stderr)
    passed = failed = routed = 0
    for (name, g, r, sx, sz, tx, tz), line in zip(cases, c_out):
        kind, py = python_plan(g, r, sx, sz, tx, tz)
        parts = line.split()
        if parts[0] == 'ERR':
            c_res = ('ERR', int(parts[1]))
        else:
            vals = [float.fromhex(v) for v in parts[2:]]
            c_res = ('OK', [(vals[i], vals[i + 1]) for i in range(0, len(vals), 2)])
        if kind == 'ERR':
            want = ('ERR', py)
            if c_res == ('ERR', -5) and py == -1 and 1 not in g:
                want = c_res            # no free cells: C says NO_MAP, Python says start
        else:
            # Python's path starts with the start cell; C excludes it. C's are float32.
            # A start already on the ring is Python [start] and C 0 waypoints.
            want = ('OK', [(f32(x), f32(z)) for x, z in py[1:]])
        if c_res == want:
            passed += 1
            routed += c_res[0] == 'OK'
        else:
            failed += 1
            print('FAIL %s:\n  python %s\n  c      %s' % (name, want, c_res))
    print('nav parity: %d/%d cases identical (%d routed, %d errors), seed %d' %
          (passed, passed + failed, routed, passed - routed, seed))
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
