# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""The Webots world's static test_obstacle is in the map exactly when LiDAR is off.

Without LiDAR nothing senses the box, so a GPS-only run would plan through it and
sit against it until the timeout. With LiDAR the runtime obstacle tracker must
still meet it (e2e_test.py checks that), so the map must stay empty there.
"""

import importlib.util
from pathlib import Path
import re

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, LogWarning
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

PACKAGES = Path(__file__).resolve().parents[2]
LAUNCH = PACKAGES / 'mowgli_bringup' / 'launch' / 'sim_full_system.launch.py'
WORLD = PACKAGES / 'mowgli_simulation' / 'worlds_webots' / 'mowgli_garden.wbt'


def _map_server_params(monkeypatch, use_lidar, world=None):
    spec = importlib.util.spec_from_file_location('sim_world_obstacle_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGES / name))
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['use_lidar'] = use_lidar
    if world is not None:
        context.launch_configurations['world'] = world
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    node = next(e for e in description.entities
                if isinstance(e, Node) and e.node_executable == 'map_server_node')
    groups = evaluate_parameters(context, node._Node__parameters)
    return {key: value for group in groups if isinstance(group, dict) for key, value in group.items()}


def _world_obstacle_polygon():
    text = WORLD.read_text()
    block = re.search(r'Solid\s*\{\s*translation\s+(\S+)\s+(\S+)\s+\S+(.*?)name\s+"test_obstacle"',
                      text, re.S)
    assert block, 'test_obstacle not found in the world file'
    x, y = float(block.group(1)), float(block.group(2))
    size = re.search(r'boundingObject\s+Box\s*\{\s*size\s+(\S+)\s+(\S+)', text[block.start():])
    sx, sy = float(size.group(1)), float(size.group(2))
    return [(x - sx / 2, y - sy / 2), (x + sx / 2, y - sy / 2),
            (x + sx / 2, y + sy / 2), (x - sx / 2, y + sy / 2)]


def _parse(polygon):
    return [tuple(float(v) for v in pt.split(',')) for pt in polygon.split(';')]


def test_map_obstacle_matches_the_world_box_without_lidar(monkeypatch):
    params = _map_server_params(monkeypatch, 'false')
    assert len(params['area_obstacles']) == 1
    got = _parse(params['area_obstacles'][0])
    want = _world_obstacle_polygon()
    assert got == pytest.approx(want), 'SIM_WORLD_OBSTACLE drifted from mowgli_garden.wbt'


@pytest.mark.parametrize('use_lidar', ['true', '1'])
def test_map_stays_empty_with_lidar(monkeypatch, use_lidar):
    params = _map_server_params(monkeypatch, use_lidar)
    assert params['area_obstacles'] == ['']



def test_custom_world_does_not_get_the_garden_obstacle(monkeypatch):
    params = _map_server_params(monkeypatch, 'false', world='my_yard.wbt')
    assert params['area_obstacles'] == ['']


def test_garden_seed_warns_when_the_saved_map_takes_precedence(tmp_path):
    spec = importlib.util.spec_from_file_location('sim_world_obstacle_warning', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    areas = tmp_path / 'areas.dat'
    map_params = tmp_path / 'map_server.yaml'
    map_params.write_text(
        f'map_server_node:\n  ros__parameters:\n    areas_file_path: {areas}\n'
    )
    context = LaunchContext()
    assert module._warn_if_saved_map_overrides_obstacle(context, 'true', map_params) is None
    areas.touch()
    actions = module._warn_if_saved_map_overrides_obstacle(context, 'true', map_params)
    assert len(actions) == 1
    assert isinstance(actions[0], LogWarning)
    assert module._warn_if_saved_map_overrides_obstacle(context, 'false', map_params) is None
