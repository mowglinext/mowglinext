#!/usr/bin/env python3
"""
Write deterministic, realistic coverage-planner benchmark fixtures (replay schema 1).

Lawns of 200-2000 m2 with concave recorded outlines (recorded every ~0.25 m, as a
driven perimeter is), 0-20 drawn obstacles (trees and beds), and the three headland
ring modes (5 passes, AUTO = 0, rings off = -1). Shipped obstacle/pivot margins.
No randomness: every coordinate is a closed-form function of the case name, so the
files are byte-identical on every machine and Python version.

    python3 make_bench_fixtures.py OUTPUT_DIR
"""

import json
import math
import pathlib
import sys

SHIPPED = {
    'operation_width': 0.16,
    'headland_width': 0.18,
    'num_headland_passes': 5,
    'chassis_safety_inset': 0.0,
    'mow_angle_rad': -1.0,
    'min_swath_length': 0.15,
    'ring_direction': 0,
    'min_turn_radius': 0.20,
    'turn_radius': 0.20,
    'step': 0.03,
    'obstacle_margin': 0.389,
    'pivot_sweep_radius': 0.597,
    'boundary_margin': 0.597,
    'perpendicular': False,
    'connector_max_headland_passes': 0,
    'start_hint': None,
}

RECORD_SPACING_M = 0.25


def area(ring):
    return 0.5 * sum(
        ring[i][0] * ring[(i + 1) % len(ring)][1] - ring[(i + 1) % len(ring)][0] * ring[i][1]
        for i in range(len(ring))
    )


def inside(x, y, ring):
    hit = False
    j = len(ring) - 1
    for i in range(len(ring)):
        (xi, yi), (xj, yj) = ring[i], ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def distance_to_ring(x, y, ring):
    best = math.inf
    for i in range(len(ring)):
        (ax, ay), (bx, by) = ring[i - 1], ring[i]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
    return best


def resample(closed, spacing):
    """Walk a dense closed polyline and drop a point every `spacing` metres."""
    out, carry = [], 0.0
    for i in range(len(closed)):
        (ax, ay), (bx, by) = closed[i], closed[(i + 1) % len(closed)]
        seg = math.hypot(bx - ax, by - ay)
        t = carry
        while t < seg:
            out.append([round(ax + (bx - ax) * t / seg, 4), round(ay + (by - ay) * t / seg, 4)])
            t += spacing
        carry = t - seg
    return out


def blob(target_area):
    """Smooth concave lawn: r(t) = s (1 + 0.22 sin(3t + 0.4) + 0.08 cos(5t))."""
    dense = [
        (
            (1 + 0.22 * math.sin(3 * t + 0.4) + 0.08 * math.cos(5 * t)) * math.cos(t),
            (1 + 0.22 * math.sin(3 * t + 0.4) + 0.08 * math.cos(5 * t)) * math.sin(t) * 0.8,
        )
        for t in (2 * math.pi * k / 4000 for k in range(4000))
    ]
    s = math.sqrt(target_area / abs(area(dense)))
    return resample([(x * s, y * s) for x, y in dense], RECORD_SPACING_M)


def ushape(target_area):
    """U-shaped lawn (a house bite out of one side) with rounded outer corners."""
    w, h, bite_w, bite_h = 1.0, 0.75, 0.4, 0.45
    corners = [
        (0, 0), (w, 0), (w, h), ((w + bite_w) / 2, h), ((w + bite_w) / 2, h - bite_h),
        ((w - bite_w) / 2, h - bite_h), ((w - bite_w) / 2, h), (0, h),
    ]
    s = math.sqrt(target_area / abs(area(corners)))
    pts = [(x * s, y * s) for x, y in corners]
    # Round each corner with a 1.5 m fillet so the recorded line looks driven.
    dense = []
    for i in range(len(pts)):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
        a = math.atan2(p0[1] - p1[1], p0[0] - p1[0])
        b = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        r = 1.5
        q0 = (p1[0] + r * math.cos(a), p1[1] + r * math.sin(a))
        q2 = (p1[0] + r * math.cos(b), p1[1] + r * math.sin(b))
        for k in range(9):
            u = k / 8
            # Quadratic Bezier through the corner.
            dense.append(
                (
                    (1 - u) ** 2 * q0[0] + 2 * (1 - u) * u * p1[0] + u * u * q2[0],
                    (1 - u) ** 2 * q0[1] + 2 * (1 - u) * u * p1[1] + u * u * q2[1],
                )
            )
    return resample(dense, RECORD_SPACING_M)


def obstacles(outer, count):
    """`count` trees (16-gon) and beds (rectangles), >= 2.5 m inside and apart."""
    if count == 0:
        return []
    xs = [p[0] for p in outer]
    ys = [p[1] for p in outer]
    golden = (math.sqrt(5) - 1) / 2
    placed, k = [], 0
    while len(placed) < count and k < 20000:
        k += 1
        x = min(xs) + (max(xs) - min(xs)) * ((k * golden) % 1.0)
        y = min(ys) + (max(ys) - min(ys)) * ((k * golden * golden * 1.7) % 1.0)
        radius = 0.35 + 0.5 * ((k * 0.618) % 1.0)
        if not inside(x, y, outer) or distance_to_ring(x, y, outer) < 2.5 + radius:
            continue
        if any(math.hypot(x - px, y - py) < 2.5 + radius + pr for px, py, pr in placed):
            continue
        placed.append((x, y, radius))
    rings = []
    for i, (x, y, radius) in enumerate(placed):
        if i % 3 == 2:  # a bed: rotated rectangle
            a, hw, hh = 0.3 * i, radius * 1.6, radius * 0.7
            corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
            rings.append(
                [
                    [round(x + cx * math.cos(a) - cy * math.sin(a), 4),
                     round(y + cx * math.sin(a) + cy * math.cos(a), 4)]
                    for cx, cy in corners
                ]
            )
        else:  # a tree
            rings.append(
                [
                    [round(x + radius * math.cos(2 * math.pi * j / 16), 4),
                     round(y + radius * math.sin(2 * math.pi * j / 16), 4)]
                    for j in range(16)
                ]
            )
    if len(rings) != count:
        raise SystemExit(f'could only place {len(rings)}/{count} obstacles')
    return rings


CASES = [
    # name, shape, area m2, obstacles, num_headland_passes
    ('blob200_o0_r5', blob, 200, 0, 5),
    ('blob200_o5_r5', blob, 200, 5, 5),
    ('blob200_o5_rm1', blob, 200, 5, -1),
    ('blob600_o10_r5', blob, 600, 10, 5),
    ('blob600_o10_r0', blob, 600, 10, 0),
    ('blob600_o10_rm1', blob, 600, 10, -1),
    ('ushape1200_o5_r5', ushape, 1200, 5, 5),
    ('ushape1200_o20_rm1', ushape, 1200, 20, -1),
    ('blob2000_o0_r5', blob, 2000, 0, 5),
    ('blob2000_o20_r5', blob, 2000, 20, 5),
    ('blob2000_o20_r0', blob, 2000, 20, 0),
    ('blob2000_o20_rm1', blob, 2000, 20, -1),
]


def build(name):
    """Return the schema-1 fixture of one named case."""
    _, shape, target, count, passes = next(c for c in CASES if c[0] == name)
    outer = shape(target)
    return {
        'schema': 1,
        'precision': 'float32',
        'polygons': [[outer] + obstacles(outer, count)],
        'parameters': {**SHIPPED, 'num_headland_passes': passes},
    }


def main():
    out = pathlib.Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=True)
    for name, *_ in CASES:
        fixture = build(name)
        rings = fixture['polygons'][0]
        (out / f'{name}.json').write_text(json.dumps(fixture, indent=1) + '\n')
        print(f'{name}: {len(rings[0])} boundary points, {abs(area(rings[0])):.0f} m2, '
              f'{len(rings) - 1} obstacles')


if __name__ == '__main__':
    main()
