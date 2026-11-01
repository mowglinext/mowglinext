# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Evaluate the actual full-sim launch builder with real launch substitutions."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest


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
    shutdowns = [action for action in relays[0].get_sub_entities()
                 if isinstance(action, Shutdown)]
    assert len(shutdowns) == 1
    events = []
    monkeypatch.setattr(context, 'emit_event_sync', events.append)
    shutdowns[0].execute(context)
    assert events[0].reason == 'GNSS quality relay exited'
