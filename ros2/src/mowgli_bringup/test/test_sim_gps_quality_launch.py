# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Evaluate the actual full-sim launch builder with real launch substitutions."""

import importlib.util
from pathlib import Path
import shlex
import subprocess
import sys

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown, TimerAction,
)
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest
from ros2launch.api.api import parse_launch_arguments


@pytest.mark.parametrize('use_lidar', ['false', 'true'])
@pytest.mark.parametrize('pattern', [None, '', '1,RTK_FIXED;2,RTK_FLOAT;3,NO_FIX'])
def test_quality_pattern_reaches_single_relay(monkeypatch, use_lidar, pattern):
    packages = Path(__file__).resolve().parents[2]
    path = packages / 'mowgli_bringup' / 'launch' / 'sim_full_system.launch.py'
    spec = importlib.util.spec_from_file_location('sim_quality_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, 'get_package_share_directory',
        lambda package: str(packages / package),
    )
    description = module.generate_launch_description()
    context = LaunchContext()
    context.launch_configurations['use_lidar'] = use_lidar
    if pattern is not None:
        context.launch_configurations['gps_quality_pattern'] = pattern
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    assert context.launch_configurations['gps_quality_pattern'] == (pattern or '')
    relays = [entity for entity in description.entities
              if isinstance(entity, Node)
              and entity.node_package == 'mowgli_simulation'
              and entity.node_executable == 'sim_navsat_rtk_fix.py']
    assert len(relays) == 1
    parameters = evaluate_parameters(context, relays[0]._Node__parameters)
    merged = {key: value for group in parameters for key, value in group.items()}
    assert merged['quality_pattern'] == (pattern or '')
    assert isinstance(merged['quality_pattern'], str)
    guards = [action for action in relays[0].get_sub_entities()
              if isinstance(action, OpaqueFunction)]
    assert len(guards) == 1
    with pytest.raises(RuntimeError, match='GNSS quality relay exited unexpectedly'):
        guards[0].execute(context)


def _relay_exit_actions(monkeypatch):
    packages = Path(__file__).resolve().parents[2]
    path = packages / 'mowgli_bringup' / 'launch' / 'sim_full_system.launch.py'
    spec = importlib.util.spec_from_file_location('sim_quality_exit', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module, 'get_package_share_directory', lambda name: str(packages / name),
    )
    description = module.generate_launch_description()
    relay = next(action for action in description.entities
                 if isinstance(action, Node)
                 and action.node_executable == 'sim_navsat_rtk_fix.py')
    return relay.get_sub_entities()


@pytest.mark.parametrize('status', [0, 1, 2])
def test_unexpected_required_relay_exit_fails_launch(monkeypatch, tmp_path, status):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    process = ExecuteProcess(
        cmd=[sys.executable, '-c', f'raise SystemExit({status})'],
        on_exit=_relay_exit_actions(monkeypatch),
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([process]))
    assert service.run() != 0


def test_requested_shutdown_still_succeeds(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    process = ExecuteProcess(
        cmd=[sys.executable, '-c', 'import time; time.sleep(10)'],
        on_exit=_relay_exit_actions(monkeypatch),
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        process,
        TimerAction(period=0.1, actions=[Shutdown(reason='requested cleanup')]),
    ]))
    assert service.run() == 0


@pytest.mark.parametrize('target', ['sim', 'e2e-test', 'e2e-test-no-lidar'])
@pytest.mark.parametrize('pattern', ['', ' ', '1,RTK_FIXED;2,NO_FIX'])
def test_make_quality_arguments_are_accepted_by_ros_cli(tmp_path, target, pattern):
    ros2 = Path(__file__).resolve().parents[3]
    makefile = tmp_path / 'Makefile'
    # GNU make executes recursive $(MAKE) recipe lines even with -n. Remove
    # that special treatment in the external dry-run copy; run no recipes.
    makefile.write_text((ros2 / 'Makefile').read_text().replace('$(MAKE)', 'make'))
    rendered = subprocess.run(
        ['make', '--no-print-directory', '-n', '-f', str(makefile), target,
         f'GPS_QUALITY_PATTERN={pattern}'],
        cwd=ros2, check=True, capture_output=True, text=True, timeout=5,
    ).stdout
    arguments = [token for token in shlex.split(rendered.replace('\\\n', ' '))
                 if token.startswith('gps_quality_pattern:=')]
    parsed = list(parse_launch_arguments(arguments))
    assert parsed == ([('gps_quality_pattern', pattern)] if pattern.strip() else [])
