#!/usr/bin/env python3
"""
Randomised differential test: old vs new coverage_replay on seeded random lawns.

    fuzz_compare.py --base BASE_BIN --cand CAND_BIN [--cases 200] [--seed 1] [--cpu 7]
                    [--keep DIR]

Every case is a random lawn (star, comb with narrow fingers, L, rotated rectangle,
thin neck; recorded every 0.05-0.5 m; 0-15 obstacles, some touching each other or
the edge) with random planner parameters (rings -1/0/1/2/5, op width, turn
radius, fixed or AUTO angle, winding, cross-hatch, turn-envelope limit, start
hint, pivot sweep on/off, obstacle margin, float32/float64). Both binaries run it
with the planning trace on; the stable record (every traced stage and the executed
sub-paths as IEEE-754 bits) must be identical, or both must fail with the same
message. Deterministic for a seed. Exit 1 on the first difference (the fixture is
kept for replay).
"""

import argparse
import json
import math
import pathlib
import random
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'replay'))
import run_replay  # noqa: E402


def ring_area(ring):
    return 0.5 * sum(
        ring[i][0] * ring[(i + 1) % len(ring)][1] - ring[(i + 1) % len(ring)][0] * ring[i][1]
        for i in range(len(ring))
    )


def inside(x, y, ring):
    hit, j = False, len(ring) - 1
    for i in range(len(ring)):
        (xi, yi), (xj, yj) = ring[i], ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def edge_distance(x, y, ring):
    best = math.inf
    for i in range(len(ring)):
        (ax, ay), (bx, by) = ring[i - 1], ring[i]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / ((dx * dx + dy * dy) or 1.0)))
        best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
    return best


def resample(corners, spacing):
    out = []
    for i in range(len(corners)):
        (ax, ay), (bx, by) = corners[i], corners[(i + 1) % len(corners)]
        steps = max(1, int(math.hypot(bx - ax, by - ay) / spacing))
        out += [[ax + (bx - ax) * k / steps, ay + (by - ay) * k / steps] for k in range(steps)]
    return out


def outline(rng, size):
    kind = rng.choice(['star', 'comb', 'ell', 'rect', 'neck'])
    if kind == 'star':
        lobes, depth = rng.randint(2, 6), rng.uniform(0.05, 0.35)
        phase = rng.uniform(0, math.tau)
        corners = [
            (size * (1 + depth * math.sin(lobes * t + phase)) * math.cos(t),
             size * 0.8 * (1 + depth * math.sin(lobes * t + phase)) * math.sin(t))
            for t in (math.tau * k / 400 for k in range(400))
        ]
    elif kind == 'comb':  # fingers with narrow gaps: many concave turns
        fingers, w, h = rng.randint(2, 5), 2 * size, 1.6 * size
        gap, depth = rng.uniform(0.6, 2.5), rng.uniform(0.3, 0.7) * h
        pitch = w / fingers
        corners = [(0, 0), (w, 0), (w, h)]
        for f in range(fingers - 1, 0, -1):
            x = f * pitch
            corners += [(x + gap / 2, h), (x + gap / 2, h - depth), (x - gap / 2, h - depth),
                        (x - gap / 2, h)]
        corners.append((0, h))
    elif kind == 'ell':
        a, b = 2 * size, rng.uniform(0.3, 0.7) * 2 * size
        corners = [(0, 0), (a, 0), (a, b), (b, b), (b, a), (0, a)]
    elif kind == 'neck':  # two rooms joined by a corridor barely wider than the robot
        w, h, n = size, 1.4 * size, rng.uniform(0.6, 2.0)
        corners = [(0, 0), (w, 0), (w, h / 2 - n / 2), (2 * w, h / 2 - n / 2), (2 * w, 0),
                   (3 * w, 0), (3 * w, h), (2 * w, h), (2 * w, h / 2 + n / 2),
                   (w, h / 2 + n / 2), (w, h), (0, h)]
    else:
        corners = [(0, 0), (2 * size, 0), (2 * size, 1.3 * size), (0, 1.3 * size)]
    a = rng.uniform(0, math.pi)
    ox, oy = rng.uniform(-50, 50), rng.uniform(-50, 50)
    corners = [(ox + x * math.cos(a) - y * math.sin(a), oy + x * math.sin(a) + y * math.cos(a))
               for x, y in corners]
    if ring_area(corners) < 0 and rng.random() < 0.5:
        corners.reverse()
    return kind, resample(corners, rng.uniform(0.05, 0.5))


def obstacles(rng, outer, count):
    xs, ys = [p[0] for p in outer], [p[1] for p in outer]
    placed, tries = [], 0
    while len(placed) < count and tries < 4000:
        tries += 1
        x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
        r = rng.uniform(0.15, 1.2)
        if not inside(x, y, outer) or edge_distance(x, y, outer) < r + rng.uniform(0.2, 2.0):
            continue
        # Mostly apart, sometimes overlapping a neighbour (a merged bed + tree).
        if rng.random() > 0.15 and any(math.hypot(x - a, y - b) < r + c + 0.3
                                       for a, b, c in placed):
            continue
        placed.append((x, y, r))
    rings = []
    for x, y, r in placed:
        if rng.random() < 0.6:
            n = rng.choice([6, 8, 16, 24])
            rings.append([[x + r * math.cos(math.tau * k / n), y + r * math.sin(math.tau * k / n)]
                          for k in range(n)])
        else:
            a, hw, hh = rng.uniform(0, math.pi), r * rng.uniform(0.8, 2.0), r * 0.6
            rings.append([[x + cx * math.cos(a) - cy * math.sin(a),
                           y + cx * math.sin(a) + cy * math.cos(a)]
                          for cx, cy in ((-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh))])
    return rings


def case(rng):
    size = rng.uniform(3.0, 16.0)
    kind, outer = outline(rng, size)
    holes = obstacles(rng, outer, rng.choice([0, 0, 1, 3, 6, 10, 15]))
    pivot = rng.random() < 0.6
    params = {
        'operation_width': rng.choice([0.14, 0.16, 0.16, 0.2, 0.3]),
        'headland_width': rng.choice([0.18, 0.4]),
        'num_headland_passes': rng.choice([-1, 0, 1, 2, 5, 5]),
        'chassis_safety_inset': rng.choice([0.0, 0.0, 0.1]),
        'mow_angle_rad': rng.choice([-1.0, -1.0, rng.uniform(0, math.pi)]),
        'min_swath_length': 0.15,
        'ring_direction': rng.choice([0, 1, 2]),
        'min_turn_radius': 0.20,
        'turn_radius': rng.choice([0.20, 0.20, 0.30, 0.45]),
        'step': 0.03,
        'obstacle_margin': rng.choice([0.0, 0.2, 0.389]),
        'pivot_sweep_radius': 0.597 if pivot else 0.0,
        'boundary_margin': 0.597 if pivot else 0.0,
        'perpendicular': rng.random() < 0.15,
        'connector_max_headland_passes': rng.choice([0, 0, 2]),
        'start_hint': None,
    }
    if rng.random() < 0.25:
        p = rng.choice(outer)
        params['start_hint'] = [p[0] + rng.uniform(-1, 1), p[1] + rng.uniform(-1, 1)]
    fixture = {'schema': 1, 'precision': rng.choice(['float32', 'float32', 'float64']),
               'polygons': [[outer] + holes], 'parameters': params}
    label = (f'{kind} {abs(ring_area(outer)):.0f}m2 {len(outer)}pts {len(holes)}obst '
             f'rings={params["num_headland_passes"]}')
    return label, fixture


def run(binary, path, cpu):
    out = subprocess.run(['taskset', '-c', str(cpu), str(binary), str(path), '1'],
                         capture_output=True, text=True, timeout=1800)
    if out.returncode:
        return ('error', out.stderr.strip())
    return ('ok', run_replay.stable(json.loads(out.stdout.splitlines()[0])))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=pathlib.Path, required=True)
    parser.add_argument('--cand', type=pathlib.Path, required=True)
    parser.add_argument('--cases', type=int, default=200)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--cpu', type=int, default=7)
    parser.add_argument('--keep', type=pathlib.Path)
    args = parser.parse_args()
    rng = random.Random(args.seed)
    work = args.keep or pathlib.Path(tempfile.mkdtemp())
    work.mkdir(parents=True, exist_ok=True)
    planned = failed = 0
    for n in range(args.cases):
        label, fixture = case(rng)
        path = work / f'fuzz_{args.seed}_{n}.json'
        path.write_text(json.dumps(fixture) + '\n')
        a, b = run(args.base, path, args.cpu), run(args.cand, path, args.cpu)
        if a != b:
            where = (run_replay.first_difference(a[1], b[1])
                     if a[0] == b[0] == 'ok' else f'{a[0]} vs {b[0]}')
            print(f'case {n} ({label}): DIFFERENT at {where}; fixture {path}', flush=True)
            raise SystemExit(1)
        planned += a[0] == 'ok'
        failed += a[0] == 'error'
        detail = (f'{a[1]["metrics"]["poses"]} poses, {a[1]["metrics"]["subpaths"]} sub-paths'
                  if a[0] == 'ok' else 'both failed alike: ' + a[1][-60:])
        print(f'case {n} ({label}): identical, {detail}', flush=True)
        if not args.keep:
            path.unlink()
    print(f'{args.cases} cases identical ({planned} planned, {failed} rejected alike)')


if __name__ == '__main__':
    main()
