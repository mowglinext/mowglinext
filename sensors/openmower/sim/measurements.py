# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Measurements shared by the simulator and its ROS probe (no ROS imports)."""

from bisect import bisect_left
from collections import deque
import math
import re
import time


class ClockEpoch:
    """Detect a changed system/monotonic mapping using bracketed clock reads.

    The system read happened between the two monotonic reads. Their bounds
    describe measurement uncertainty, without an adjustable timing tolerance.
    """

    def __init__(self):
        self.bounds = None

    def observe(self, before_ns, host_ns, after_ns):
        bounds = (host_ns - after_ns, host_ns - before_ns)
        changed = False
        if self.bounds is not None:
            lower = max(bounds[0], self.bounds[0])
            upper = min(bounds[1], self.bounds[1])
            changed = lower > upper
            if not changed:
                bounds = (lower, upper)
        self.bounds = bounds
        return changed

    def sample(self):
        before = time.monotonic_ns()
        host = time.time_ns()
        after = time.monotonic_ns()
        changed = self.observe(before, host, after)
        # Physical integration uses an actual monotonic endpoint, never the
        # bracket midpoint or an estimated system timestamp.
        return after / 1e9, host / 1e9, changed

    def to_monotonic(self, system_stamp):
        if self.bounds is None or not math.isfinite(system_stamp):
            raise ValueError('no valid system/monotonic clock mapping')
        return system_stamp - (self.bounds[0] + self.bounds[1]) / 2e9


class MotionHistory:
    """Ground distance/yaw on the monotonic physical-integration clock.

    Values are cumulative; callers map ROS stamps through independently read
    clock bounds before querying. Refining those bounds cannot reorder history.
    """

    def __init__(self, capacity=60000):
        self.rows = deque(maxlen=capacity)

    def append(self, stamp, distance, yaw, ticks):
        if self.rows and stamp <= self.rows[-1][0]:
            raise ValueError('ground clock did not advance')
        self.rows.append((stamp, distance, yaw, tuple(ticks)))

    def at(self, stamp):
        rows = list(self.rows)
        if not rows or not rows[0][0] <= stamp <= rows[-1][0]:
            raise ValueError('message interval is outside recorded ground history')
        i = bisect_left(rows, stamp, key=lambda row: row[0])
        if rows[i][0] == stamp:
            return rows[i][1:3]
        a, b = rows[i - 1], rows[i]
        weight = (stamp - a[0]) / (b[0] - a[0])
        return tuple(a[j] + weight * (b[j] - a[j]) for j in (1, 2))

    def mean(self, start, end):
        if end <= start:
            raise ValueError('empty ground interval')
        a, b = self.at(start), self.at(end)
        return tuple((y - x) / (end - start) for x, y in zip(a, b))

    def stationary_for(self, stamp):
        """Time since the last observed integer wheel-tick change, at stamp."""
        rows = [row for row in self.rows if row[0] <= stamp]
        if not rows or stamp > self.rows[-1][0]:
            raise ValueError('calibration event is outside recorded ground history')
        for a, b in zip(reversed(rows[:-1]), reversed(rows[1:])):
            if a[3] != b[3]:
                return stamp - b[0]
        return stamp - rows[0][0]


def odometry_mean(rows):
    """Receive time, vx, yaw rate, ROS stamp -> means and exact stamp interval."""
    if len(rows) < 2:
        raise ValueError('fewer than two odometry messages')
    if any(not all(math.isfinite(v) for v in row) for row in rows):
        raise ValueError('nonfinite odometry sample')
    if any(b[3] <= a[3] for a, b in zip(rows, rows[1:])):
        raise ValueError('odometry stamps did not advance')
    start, end = rows[0][3], rows[-1][3]
    means = tuple(sum(b[j] * (b[3] - a[3]) for a, b in zip(rows, rows[1:])) / (end - start)
                  for j in (1, 2))
    return means, start, end


def run_scenario(scenario, report, *args, fixture=None):
    """A failed prerequisite keeps the run red and stops dependent scenarios."""
    failures = len(report.failed)
    clock_steps = len(fixture.clock_steps) if fixture is not None else 0
    try:
        scenario(*args)
    except Exception as exc:
        report.check(scenario.__name__, 'scenario raised', False, repr(exc))
    if fixture is not None and len(fixture.clock_steps) > clock_steps:
        report.check(scenario.__name__, 'fixture host clock stayed in one epoch', False,
                     repr(fixture.clock_steps[clock_steps:]))
    return len(report.failed) == failures


def drives_running(effective):
    """A stop-latency test needs both drives active at its starting instant."""
    return all(math.isfinite(effective[name]) and abs(effective[name]) > 0.05
               for name in ('left', 'right'))


def calibration_attempt(lines, since):
    """Pair completion with its actual docked start, excluding aborted attempts."""
    start = None
    for line in lines:
        match = re.search(r'\[(?:INFO|WARN)\] \[([0-9.]+)\].*IMU bias (.*)', line)
        if not match:
            continue
        stamp, event = float(match[1]), match[2]
        if stamp < since:
            continue
        if event.startswith('calibration started (docked'):
            start = stamp
        elif event.startswith('calibration aborted'):
            start = None
        elif event.startswith('ready:') and start is not None:
            return start, stamp
    raise ValueError('no completed docked calibration attempt')
