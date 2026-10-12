# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

from pathlib import Path
import math

import yaml


def _areas():
    path = Path(__file__).resolve().parents[1] / 'config_webots' / 'mowgli_garden.yaml'
    return yaml.safe_load(path.read_text())['areas']


def _bounds(area):
    points = area['polygon']
    return (
        min(p[0] for p in points), min(p[1] for p in points),
        max(p[0] for p in points), max(p[1] for p in points),
    )


def test_full_size_lawn_does_not_intersect_synthetic_charging_disk():
    lawn = _areas()[0]
    assert lawn['name'] == 'main_mow'
    assert not lawn['is_navigation']
    x0, y0, x1, y1 = _bounds(lawn)
    points = lawn['polygon']
    area = abs(sum(
        a[0] * b[1] - a[1] * b[0]
        for a, b in zip(points, points[1:] + points[:1])
    )) / 2.0
    assert math.isclose(area, 54.0)
    closest_x = max(x0, min(0.0, x1))
    closest_y = max(y0, min(0.0, y1))
    assert math.hypot(closest_x, closest_y) > 0.3
    # Preserve the real world's obstacle and existing E2E obstacle fixture.
    assert x0 < 3.0 < x1 and y0 < 1.5 < y1


def test_dock_transit_connects_to_lawn_without_a_polygon_gap():
    lawn, transit = _areas()
    assert transit['is_navigation']
    a = _bounds(lawn)
    b = _bounds(transit)
    assert min(a[2], b[2]) > max(a[0], b[0])
    assert min(a[3], b[3]) > max(a[1], b[1])


def test_dock_and_staging_have_complete_chassis_rotation_room():
    _, transit = _areas()
    x0, y0, x1, y1 = _bounds(transit)
    # Shipped complete chassis plus the existing 0.05 m Nav2 clearance.
    radius = math.hypot(0.53, 0.275) + 0.05
    for x, y in ((0.0, 0.0), (-1.5, 0.0), (-1.5, 1.0)):
        assert x0 < x - radius and x + radius < x1
        assert y0 < y - radius and y + radius < y1
