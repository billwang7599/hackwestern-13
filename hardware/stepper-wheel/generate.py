"""Generate manifold printable wheel STLs using only Python's standard library.

Run: python3 hardware/stepper-wheel/generate.py --diameter 127 --clearance 0.15
All dimensions are millimetres; clearance is added to BOTH total bore dimensions.
"""
import argparse
from collections import Counter
import math
from pathlib import Path
import struct

ROOT = Path(__file__).resolve().parent
N = 240


def angle(i, radius):
    # Six constant-width 6 mm spokes; extra samples follow each cutout boundary.
    sector, j = divmod(i, 40)
    half = min(math.asin(min(1, 3 / radius)), math.radians(20))
    local = -half + 2 * half * j / 8 if j <= 8 else half + (math.pi / 3 - 2 * half) * (j - 8) / 32
    return sector * math.pi / 3 + local


def make_mesh(diameter=127, clearance=0.15, coupon=False):
    vertices, lookup, faces = [], {}, {}

    def vertex(p):
        key = tuple(round(v, 9) for v in p)
        if key not in lookup:
            lookup[key] = len(vertices)
            vertices.append(key)
        return lookup[key]

    def face(ids):
        ids = list(dict.fromkeys(ids))
        if len(ids) < 3:
            return
        key = tuple(sorted(ids))
        if key in faces:
            del faces[key]
        else:
            faces[key] = ids

    levels = [0, 1, 6.6, 7] if coupon else [0, 1, 1.8, 4, 4.2, 5, 5.8, 8, 8.2, 9, 10, 13.6, 14]
    height = levels[-1]
    floor = height - 6
    rings = []
    for z in levels:
        entry = max(0, z - (height - 0.4))
        bore_r = (5 + clearance) / 2 + entry
        bore_half_flat = (3 + clearance) / 2 + entry
        if coupon:
            radii = [0, bore_r, 6]
        else:
            groove = 0.8 if 1.8 <= z <= 4.2 or 5.8 <= z <= 8.2 else 0
            radii = [0, bore_r, 10, diameter / 2 - 6, diameter / 2 - groove]
        layer = []
        for k, r in enumerate(radii):
            points = []
            for i in range(N):
                a = angle(i, max(r, 3))
                rr = min(r, bore_half_flat / max(abs(math.sin(a)), 1e-12)) if k == 1 else r
                points.append(vertex((rr * math.cos(a), rr * math.sin(a), z)))
            layer.append(points)
        rings.append(layer)

    for layer in range(len(levels) - 1):
        z = levels[layer]
        for band in range(len(rings[layer]) - 1):
            for i in range(N):
                if band == 0 and z >= floor:
                    continue
                if not coupon:
                    if band == 2 and (z >= 4 or i % 40 >= 8):
                        continue
                    if band == 3 and z >= 10:
                        continue
                j = (i + 1) % N
                a, b = rings[layer][band][i], rings[layer][band + 1][i]
                c, d = rings[layer][band + 1][j], rings[layer][band][j]
                e, f = rings[layer + 1][band][i], rings[layer + 1][band + 1][i]
                g, h = rings[layer + 1][band + 1][j], rings[layer + 1][band][j]
                for q in [(d, c, b, a), (e, f, g, h), (a, b, f, e), (b, c, g, f), (c, d, h, g), (d, a, e, h)]:
                    face(q)

    triangles = []
    for polygon in faces.values():
        for j in range(1, len(polygon) - 1):
            triangles.append((polygon[0], polygon[j], polygon[j + 1]))
    return vertices, triangles


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def write_stl(path, mesh):
    vertices, triangles = mesh
    edges, directions = Counter(), Counter()
    volume = 0
    payload = bytearray(b'28BYJ48 wheel; millimetres'.ljust(80, b'\0'))
    payload.extend(struct.pack('<I', len(triangles)))
    for ids in triangles:
        a, b, c = [vertices[i] for i in ids]
        normal = cross(tuple(b[k]-a[k] for k in range(3)), tuple(c[k]-a[k] for k in range(3)))
        norm = math.sqrt(sum(v*v for v in normal))
        assert norm > 1e-10, 'Degenerate triangle'
        normal = tuple(v/norm for v in normal)
        payload.extend(struct.pack('<12fH', *normal, *a, *b, *c, 0))
        volume += sum(a[k]*cross(b, c)[k] for k in range(3))/6
        for u, v in zip(ids, ids[1:] + ids[:1]):
            edges[tuple(sorted((u, v)))] += 1
            directions[(u, v)] += 1
    assert all(n == 2 for n in edges.values()), 'Mesh is not closed'
    assert all(directions[(u, v)] == directions[(v, u)] for u, v in edges), 'Inconsistent normals'
    assert volume > 0
    path.write_bytes(payload)
    print(f'{path.name}: {len(triangles)} triangles, closed manifold, volume {volume/1000:.2f} cm^3')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diameter', type=float, default=127)
    parser.add_argument('--clearance', type=float, default=0.15)
    args = parser.parse_args()
    if not 40 <= args.diameter <= 200 or not 0 <= args.clearance <= 0.6:
        parser.error('Use diameter 40–200 mm and total bore clearance 0–0.6 mm.')
    write_stl(ROOT / f'wheel-{args.diameter:g}mm.stl', make_mesh(args.diameter, args.clearance))
    for clearance in [0, 0.15, 0.3]:
        write_stl(ROOT / f'fit-test-{clearance:.2f}mm.stl', make_mesh(clearance=clearance, coupon=True))
