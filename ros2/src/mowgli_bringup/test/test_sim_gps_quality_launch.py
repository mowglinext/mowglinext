# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""Exercise the production simulator's GNSS-quality parameter wiring."""

import pytest

from test_foxglove_launch_config import Node, _build, _parameters


def _gps(description):
    nodes = [entity for entity in description.entities
             if isinstance(entity, Node)
             and entity.options.get('name') == 'sim_navsat_rtk_fix']
    assert len(nodes) == 1
    return nodes[0]


def test_default_preserves_always_fixed_baseline():
    description, context = _build('sim_full_system.launch.py')
    assert context['gps_quality_pattern'] == ''
    assert _parameters(_gps(description), context)['quality_pattern'] == ''


@pytest.mark.parametrize('use_lidar', ['false', 'true'])
@pytest.mark.parametrize('pattern', ['', '90,RTK_FIXED;30,RTK_FLOAT;10,NO_FIX'])
def test_requested_quality_pattern_reaches_single_gps_relay(use_lidar, pattern):
    description, context = _build(
        'sim_full_system.launch.py', use_lidar=use_lidar, gps_quality_pattern=pattern
    )
    parameters = _parameters(_gps(description), context)
    assert parameters['quality_pattern'] == pattern
    assert isinstance(parameters['quality_pattern'], str)
