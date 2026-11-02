# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

"""Webots must expose the same cutting-tool frame as the physical URDF."""

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml


_ROS_SRC = Path(__file__).resolve().parents[2]
_SIM_URDF = _ROS_SRC / 'mowgli_simulation/urdf_webots/mowgli_webots.urdf'
_BRINGUP = _ROS_SRC / 'mowgli_bringup'


def test_cutting_tool_frame_matches_physical_urdf():
    sim = ET.parse(_SIM_URDF).getroot()
    assert sim.find("./link[@name='blade_link']") is not None
    joint = sim.find("./joint[@name='blade_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'blade_link'

    # Use the existing simulation wheel radius, not the physical template's
    # newer radius: the blade remains 5 cm above base_footprint in either model.
    params = yaml.safe_load((_BRINGUP / 'config/mowgli_robot.yaml').read_text())
    params = params['mowgli']['ros__parameters']
    base_origin = sim.find("./joint[@name='base_footprint_to_base_link']/origin")
    wheel_radius = float(base_origin.attrib['xyz'].split()[2])
    keys = ('chassis_center_x', 'blade_x_offset', 'blade_y_offset')
    result = subprocess.run(
        [
            'xacro',
            str(_BRINGUP / 'urdf/mowgli.urdf.xacro'),
            f'wheel_radius:={wheel_radius}',
            *[f'{key}:={params[key]}' for key in keys],
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    physical = ET.fromstring(result.stdout)
    expected = physical.find("./joint[@name='blade_joint']/origin")
    actual = joint.find('origin')
    for attribute in ('xyz', 'rpy'):
        assert [float(v) for v in actual.attrib[attribute].split()] == pytest.approx(
            [float(v) for v in expected.attrib[attribute].split()]
        )
