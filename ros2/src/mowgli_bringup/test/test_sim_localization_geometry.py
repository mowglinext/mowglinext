# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Execute the simulator -> navigation -> fusion launch chain without ROS nodes."""

import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile

import pytest
from test_foxglove_launch_config import (
    _build, _resolve, Action, Configuration, Declaration, Description, Include, Node, Parameter,
)
import yaml

ROOT = Path(__file__).resolve().parents[2]


class Opaque(Action):
    pass


def _execute(path, context, monkeypatch, tmp_path, robot_geometry=None):
    monkeypatch.setattr(Configuration, 'perform',
                        lambda self, context: context[self.name], raising=False)
    spec = importlib.util.spec_from_file_location(
        'geometry_robot_config', ROOT / 'mowgli_bringup/launch/robot_config_util.py'
    )
    util = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(util)
    robot_params = util.load_robot_params(
        str(ROOT / 'mowgli_bringup'), str(tmp_path / 'absent.yaml'))
    robot_params.update(robot_geometry or {})
    namespace = {name: getattr(util, name) for name in dir(util) if not name.startswith('__')}
    namespace.update(
        os=os, yaml=yaml, json=json,
        get_package_share_directory=lambda package: str(ROOT / package),
        get_package_prefix=lambda package: str(tmp_path / package),
        load_robot_params=lambda *args: robot_params,
        LaunchDescription=Description, DeclareLaunchArgument=Declaration,
        IncludeLaunchDescription=Include, LaunchConfiguration=Configuration,
        OpaqueFunction=Opaque, Node=Node, ParameterValue=Parameter,
    )
    for name in ('ExecuteProcess', 'GroupAction', 'RegisterEventHandler', 'IfCondition',
                 'UnlessCondition', 'OnProcessExit', 'PythonLaunchDescriptionSource',
                 'PythonExpression', 'SetParameter', 'RewrittenYaml'):
        namespace[name] = Action
    helper_path = ROOT / 'fusion_graph/launch/fusion_graph_simulation_geometry.py'
    if helper_path.exists():
        helper_spec = importlib.util.spec_from_file_location('sim_geometry_test', helper_path)
        helper = importlib.util.module_from_spec(helper_spec)
        helper_spec.loader.exec_module(helper)
        namespace['resolve_simulation_geometry'] = helper.resolve_simulation_geometry
    functions = [item for item in ast.parse(path.read_text()).body
                 if isinstance(item, ast.FunctionDef)]
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
    if '_read_robot_config' in namespace:
        monkeypatch.setitem(namespace, '_read_robot_config', lambda: robot_params)
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    description = namespace['generate_launch_description']()
    for entity in description.entities:
        if isinstance(entity, Declaration):
            context.setdefault(entity.args[0], entity.options['default_value'])
    expanded = []
    for entity in description.entities:
        if isinstance(entity, Opaque):
            expanded.extend(entity.options['function'](context))
        else:
            expanded.append(entity)
    return Description(expanded), context


@pytest.mark.parametrize('use_lidar', ['false', 'true'])
def test_scenario_geometry_reaches_cog_and_fusion(use_lidar, monkeypatch, tmp_path):
    sim, sim_context = _build('sim_full_system.launch.py', use_lidar=use_lidar)
    nav = next(item for item in sim.entities if isinstance(item, Include)
               and item.args[0].args[0].endswith('navigation.launch.py'))
    nav_context = {key: _resolve(value, sim_context)
                   for key, value in nav.options['launch_arguments']}
    navigation, nav_context = _execute(
        ROOT / 'mowgli_bringup/launch/navigation.launch.py', nav_context, monkeypatch, tmp_path)
    cog = next(item for item in navigation.entities if isinstance(item, Node)
               and item.options.get('name') == 'cog_to_imu')
    include = next(item for item in navigation.entities if isinstance(item, Include)
                   and item.args[0].args[0].endswith('fusion_graph.launch.py'))
    fusion_context = {key: _resolve(value, nav_context)
                      for key, value in include.options['launch_arguments']}
    fusion, fusion_context = _execute(
        ROOT / 'fusion_graph/launch/fusion_graph.launch.py', fusion_context, monkeypatch, tmp_path)
    nodes = [item for item in fusion.entities if isinstance(item, Node)]
    expected = {'datum_lat': 48.137154, 'datum_lon': 11.576124,
                'lever_arm_x': 0.3, 'lever_arm_y': 0.0}
    for node, context in [(cog, nav_context)] + [(item, fusion_context) for item in nodes]:
        parameters = {}
        for layer in node.options['parameters']:
            if isinstance(layer, dict):
                parameters.update(_resolve(layer, context))
        assert {key: parameters[key] for key in expected} == expected
        for yaw in (0.0, math.pi / 2, -math.pi / 2):
            x, y = 4.49, 6.69
            antenna_x, antenna_y = x + 0.3 * math.cos(yaw), y + 0.3 * math.sin(yaw)
            latitude = expected['datum_lat'] + antenna_y / (6378137 * math.pi / 180)
            longitude = expected['datum_lon'] + antenna_x / (
                6378137 * math.pi / 180 * math.cos(math.radians(expected['datum_lat'])))
            map_x = (longitude - parameters['datum_lon']) * 6378137 * math.pi / 180 * math.cos(
                math.radians(parameters['datum_lat']))
            map_y = (latitude - parameters['datum_lat']) * 6378137 * math.pi / 180
            assert map_x - parameters['lever_arm_x'] * math.cos(yaw) == pytest.approx(x, abs=1e-8)
            assert map_y - parameters['lever_arm_x'] * math.sin(yaw) == pytest.approx(y, abs=1e-8)


@pytest.mark.parametrize('filename', [
    'mowgli_bringup/launch/navigation.launch.py',
    'fusion_graph/launch/fusion_graph.launch.py',
])
@pytest.mark.parametrize('robot_geometry', [
    {}, {'datum_lat': 59.3, 'datum_lon': 18.0, 'gps_x': 0.17, 'gps_y': -0.12},
])
def test_default_hardware_launch_preserves_robot_geometry(
        filename, robot_geometry, monkeypatch, tmp_path):
    description, context = _execute(
        ROOT / filename, {}, monkeypatch, tmp_path, robot_geometry)
    nodes = [item for item in description.entities if isinstance(item, Node)
             and item.options.get('name') in ('cog_to_imu', 'fusion_graph_node')]
    assert nodes
    for node in nodes:
        parameters = {}
        for layer in node.options['parameters']:
            if isinstance(layer, dict):
                parameters.update(_resolve(layer, context))
        assert parameters['datum_lat'] == robot_geometry.get('datum_lat', 0.0)
        assert parameters['datum_lon'] == robot_geometry.get('datum_lon', 0.0)
        assert parameters['lever_arm_x'] == robot_geometry.get('gps_x', 0.3)
        assert parameters['lever_arm_y'] == robot_geometry.get('gps_y', 0.0)


@pytest.mark.parametrize('filename', [
    'mowgli_bringup/launch/navigation.launch.py',
    'fusion_graph/launch/fusion_graph.launch.py',
])
def test_simulation_bundle_cannot_override_hardware_time(filename, monkeypatch, tmp_path):
    bundle = json.dumps({'datum_lat': 48.137154, 'datum_lon': 11.576124,
                         'lever_arm_x': 0.3, 'lever_arm_y': 0.0})
    with pytest.raises(ValueError, match='requires use_sim_time=true'):
        _execute(ROOT / filename, {'simulation_geometry': bundle}, monkeypatch, tmp_path)


@pytest.mark.parametrize('bundle', [
    {}, {'datum_lat': 48.0},
    {'datum_lat': 48.0, 'datum_lon': 11.0, 'lever_arm_x': float('nan'), 'lever_arm_y': 0.0},
    {'datum_lat': 0.0, 'datum_lon': 0.0, 'lever_arm_x': 0.3, 'lever_arm_y': 0.0},
    {'datum_lat': 91.0, 'datum_lon': 11.0, 'lever_arm_x': 0.3, 'lever_arm_y': 0.0},
])
def test_partial_or_invalid_geometry_fails_before_localization(bundle, monkeypatch, tmp_path):
    with pytest.raises(ValueError):
        _execute(ROOT / 'fusion_graph/launch/fusion_graph.launch.py',
                 {'use_sim_time': 'true', 'simulation_geometry': json.dumps(bundle)},
                 monkeypatch, tmp_path)
