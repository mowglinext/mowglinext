# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

import os
from pathlib import Path
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import pytest


def _tool_offset(robot):
    assert robot.find("./link[@name='blade_link']") is not None
    joint = robot.find("./joint[@name='blade_joint']")
    assert joint is not None and joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'blade_link'
    assert [float(v) for v in joint.find('origin').attrib['rpy'].split()] == [0, 0, 0]
    return tuple(float(v) for v in joint.find('origin').attrib['xyz'].split())


def test_webots_tool_frame_matches_production_construction_for_sim_geometry():
    source = Path(__file__).resolve().parents[2]
    simulated = ET.parse(
        source / 'mowgli_simulation/urdf_webots/mowgli_webots.urdf'
    ).getroot()
    # Use this simulator's existing chassis centre and wheel geometry, rather
    # than xacro's independent fallback dimensions for a physical robot.
    imu_origin = simulated.find("./joint[@name='base_link_to_imu']/origin")
    wheel = simulated.find("./link[@name='left_wheel']/visual/geometry/cylinder")
    centre = float(imu_origin.attrib['xyz'].split()[0])
    radius = float(wheel.attrib['radius'])
    production = subprocess.run(
        ['xacro', str(source / 'mowgli_bringup/urdf/mowgli.urdf.xacro'),
         f'chassis_center_x:={centre}', f'wheel_radius:={radius}'],
        check=True, capture_output=True, text=True,
    )
    assert _tool_offset(simulated) == pytest.approx(
        _tool_offset(ET.fromstring(production.stdout))
    )
    parent = simulated.find("./joint[@name='base_footprint_to_base_link']")
    base_height = float(parent.find('origin').attrib['xyz'].split()[2])
    assert base_height + _tool_offset(simulated)[2] == pytest.approx(0.05)


def test_real_robot_state_publisher_exposes_cutting_tool_transform(tmp_path):
    from ament_index_python.packages import get_package_prefix
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    import yaml

    source = Path(__file__).resolve().parents[1]
    description = (source / 'urdf_webots/mowgli_webots.urdf').read_text()
    parameters = tmp_path / 'rsp.yaml'
    parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'robot_description': description,
    }}}))
    private = '/tool_frame_' + uuid.uuid4().hex
    remaps = ['--ros-args', '-r', '/tf_static:=' + private + '/tf_static',
              '-r', '/tf:=' + private + '/tf']
    context = rclpy.Context()
    domain_id = int(os.environ.get('ROS_DOMAIN_ID', '186'))
    rclpy.init(context=context, domain_id=domain_id)
    node = rclpy.create_node('cutting_tool_frame_test', context=context, cli_args=remaps)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    executable = (
        Path(get_package_prefix('robot_state_publisher'))
        / 'lib/robot_state_publisher/robot_state_publisher'
    )
    with (tmp_path / 'rsp.log').open('w') as log:
        process = subprocess.Popen(
            [str(executable), *remaps, '--params-file', str(parameters)],
            env=dict(os.environ, ROS_DOMAIN_ID=str(domain_id)),
            stdout=log, stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if buffer.can_transform('base_footprint', 'blade_link', Time()):
                    break
            assert process.poll() is None
            assert buffer.can_transform('base_footprint', 'blade_link', Time())
            transform = buffer.lookup_transform('base_footprint', 'blade_link', Time()).transform
            assert (transform.translation.x, transform.translation.y,
                    transform.translation.z) == pytest.approx((0.18, 0.0, 0.05))
            assert (transform.rotation.x, transform.rotation.y,
                    transform.rotation.z, transform.rotation.w) == pytest.approx((0, 0, 0, 1))
        finally:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
            listener.unregister()
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)
