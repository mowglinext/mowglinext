#!/usr/bin/env python3
"""
Exact-output check and interleaved A/B timing of two coverage_replay builds.

    ab_compare.py --base BASE_BIN --cand CAND_BIN --reps 21 [--cpu 7] FIXTURE.json...

1. Equality: each binary runs every fixture ONCE with the planning trace on; the
   stable part of the record (every traced stage, boundaries, kept swaths, the
   executed sub-paths as IEEE-754 bit patterns, quaternions, metrics, FollowStrip
   fingerprint) must be identical. Any difference aborts before timing.
2. Timing: --reps fresh processes per binary, interleaved base/cand/base/...,
   pinned with taskset to one core, trace off. Reports the in-process planner +
   connector time (median, IQR, and a 95 % bootstrap CI of the median ratio).
"""

import argparse
import json
import pathlib
import random
import statistics
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'replay'))
import run_replay  # noqa: E402  (stable(): the geometry part of a record)


def run(binary, fixture, cpu, trace):
    command = ['taskset', '-c', str(cpu), str(binary), str(fixture), '1']
    if not trace:
        command.append('--no-trace')
    out = subprocess.run(command, capture_output=True, text=True, check=True, timeout=3600)
    return json.loads(out.stdout.splitlines()[0])


def quartiles(values):
    q = statistics.quantiles(values, n=4)
    return q[0], statistics.median(values), q[2]


def bootstrap_ratio(base, cand, rounds=2000, seed=1):
    rng = random.Random(seed)
    ratios = []
    for _ in range(rounds):
        b = statistics.median(rng.choices(base, k=len(base)))
        c = statistics.median(rng.choices(cand, k=len(cand)))
        ratios.append(b / c)
    ratios.sort()
    return ratios[int(0.025 * rounds)], ratios[int(0.975 * rounds)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=pathlib.Path, required=True)
    parser.add_argument('--cand', type=pathlib.Path, required=True)
    parser.add_argument('--reps', type=int, default=21)
    parser.add_argument('--cpu', type=int, default=7)
    parser.add_argument('--skip-equality', action='store_true')
    parser.add_argument('fixtures', type=pathlib.Path, nargs='+')
    args = parser.parse_args()

    if not args.skip_equality:
        for fixture in args.fixtures:
            a = run_replay.stable(run(args.base, fixture, args.cpu, True))
            b = run_replay.stable(run(args.cand, fixture, args.cpu, True))
            if a != b:
                print(f'{fixture.stem}: OUTPUT DIFFERS at {run_replay.first_difference(a, b)}')
                raise SystemExit(1)
            print(f'{fixture.stem}: identical ({a["metrics"]["poses"]} poses, '
                  f'{a["metrics"]["subpaths"]} sub-paths, output {run_replay.digest(a)[:12]})',
                  flush=True)
    if args.reps < 1:
        return
    print(f'{"fixture":22s} {"base ms (q1/med/q3)":>26s} {"cand ms (q1/med/q3)":>26s} '
          f'{"speedup":>8s} {"95% CI":>14s}')
    for fixture in args.fixtures:
        times = {'base': [], 'cand': []}
        for _ in range(args.reps):
            for name, binary in (('base', args.base), ('cand', args.cand)):
                t = run(binary, fixture, args.cpu, False)['timing']
                times[name].append(t['planning_ms'] + t['connectors_ms'])
        b, c = quartiles(times['base']), quartiles(times['cand'])
        lo, hi = bootstrap_ratio(times['base'], times['cand'])
        print(f'{fixture.stem:22s} {b[0]:8.1f}/{b[1]:8.1f}/{b[2]:8.1f} '
              f'{c[0]:8.1f}/{c[1]:8.1f}/{c[2]:8.1f} {b[1] / c[1]:7.2f}x '
              f'[{lo:5.2f},{hi:5.2f}]', flush=True)


if __name__ == '__main__':
    main()
