# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

from collections import defaultdict
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace


def _load_e2e_module():
    path = Path(__file__).resolve().parents[2] / 'e2e_test.py'
    spec = importlib.util.spec_from_file_location('mowgli_e2e_ground_truth', path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class _Logger:

    def info(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


def _area(module):
    points = [
        SimpleNamespace(x=0.0, y=0.0),
        SimpleNamespace(x=1.0, y=0.0),
        SimpleNamespace(x=1.0, y=1.0),
        SimpleNamespace(x=0.0, y=1.0),
    ]
    obstacle = SimpleNamespace(
        points=[
            SimpleNamespace(x=0.4, y=0.4),
            SimpleNamespace(x=0.6, y=0.4),
            SimpleNamespace(x=0.6, y=0.6),
            SimpleNamespace(x=0.4, y=0.6),
        ]
    )
    return SimpleNamespace(
        name='main',
        is_navigation_area=False,
        area=SimpleNamespace(points=points),
        obstacles=[obstacle],
    )


def test_immutable_target_excludes_obstacle_cells() -> None:
    module = _load_e2e_module()
    node = module.E2ETestNode.__new__(module.E2ETestNode)
    node.metrics = module.Metrics()
    node.get_logger = lambda: _Logger()

    assert node._set_mowing_area_target(_area(module), True)
    assert node.metrics.coverage_target_ready
    assert (1, 1) in node.metrics.mow_area_cells
    assert (10, 10) not in node.metrics.mow_area_cells


def test_blade_sweep_marks_footprint_not_only_robot_center() -> None:
    module = _load_e2e_module()
    node = module.E2ETestNode.__new__(module.E2ETestNode)
    node.metrics = module.Metrics()
    node.metrics.mow_area_cells = {
        (x, y)
        for x in range(-10, 11)
        for y in range(-10, 11)
    }
    node.metrics.covered_cell_visits = defaultdict(int)

    node._mark_blade_sweep(0.0, 0.0, 0.2, 0.0, 1.0)

    assert len(node.metrics.covered_cells) > 4
    assert any(y != 0 for _, y in node.metrics.covered_cells)


def test_completion_counters_are_read_before_state_transition() -> None:
    source = (
        Path(__file__).resolve().parents[2] / 'e2e_test.py'
    ).read_text()
    callback = source.split('def _on_bt_status', 1)[1].split(
        'def _track_phase_transition', 1
    )[0]

    assert callback.index('self.metrics.swath_count =') < callback.index(
        'self._track_phase_transition('
    )


def test_obstacle_detection_reads_filtered_beams_toward_world_target() -> None:
    module = _load_e2e_module()
    node = module.E2ETestNode.__new__(module.E2ETestNode)
    node.ground_truth_pose = (0.0, 0.0, 0.0)
    ranges = [float('inf')] * 9
    ranges[0] = 2.75
    node._latest_collision_scan = SimpleNamespace(
        angle_min=-math.pi,
        angle_increment=math.pi / 4.0,
        range_min=0.05,
        range_max=10.0,
        ranges=ranges,
    )

    assert node._collision_scan_range_toward(3.0, 0.0) == 2.75
    assert math.isinf(node._collision_scan_range_toward(0.0, 3.0))


def test_collision_metrics_use_filtered_safety_scan() -> None:
    module = _load_e2e_module()
    node = module.E2ETestNode.__new__(module.E2ETestNode)
    node.metrics = module.Metrics()
    scan = SimpleNamespace(
        range_min=0.05,
        range_max=10.0,
        ranges=[float('inf'), 0.12, 0.40],
    )

    node._on_collision_scan(scan)

    assert node._latest_collision_scan is scan
    assert node.metrics.min_obstacle_dist[0][1] == 0.12


def test_obstacle_validation_has_no_gazebo_service_fallback() -> None:
    source = (
        Path(__file__).resolve().parents[2] / 'e2e_test.py'
    ).read_text()

    assert 'WEBOTS_TEST_OBSTACLE_X = 3.0' in source
    assert 'WEBOTS_LIDAR_YAW_IN_BASE_RAD = math.pi' in source
    assert '/scan_collision' in source
    assert 'LaserScan, "/scan",' not in source
    assert 'gz service' not in source
