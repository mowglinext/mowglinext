# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

from pathlib import Path

import yaml


def test_kinematic_drive_publishes_simulator_pose_truth() -> None:
    source = (
        Path(__file__).resolve().parents[1]
        / 'mowgli_simulation'
        / 'kinematic_drive.py'
    ).read_text()

    assert "PoseStamped, '/sim/ground_truth_pose'" in source
    assert 'self.__gt_pose_pub.publish(gt)' in source
    assert "gt.header.frame_id = 'map'" in source


def test_chassis_observes_actual_wheels_without_another_actuation_model() -> None:
    package = Path(__file__).resolve().parents[1]
    source = (package / 'mowgli_simulation' / 'kinematic_drive.py').read_text()
    actuation = (package / 'src' / 'sim_actuation_node.cpp').read_text()

    assert 'step_firmware_wheel_model(' in actuation
    assert 'step_firmware_wheel_model(' not in source
    assert "('left_wheel_sensor', 'right_wheel_sensor')" in source
    assert 'motor.setVelocity(' not in source
    assert 'motor.setPosition(' not in source


def test_minimal_webots_launch_owns_shared_actuation_node() -> None:
    package = Path(__file__).resolve().parents[1]
    launch = (package / 'launch' / 'webots_minimal.launch.py').read_text()

    assert launch.count("executable='sim_actuation_node'") == 1
    assert 'sim_actuation_node,' in launch


def test_full_sim_does_not_duplicate_shared_actuation_node() -> None:
    source_root = Path(__file__).resolve().parents[2]
    launch = (
        source_root
        / 'mowgli_bringup'
        / 'launch'
        / 'sim_full_system.launch.py'
    ).read_text()

    assert 'executable="sim_actuation_node"' not in launch
    assert 'sim_actuation_node,' not in launch


def test_full_sim_gyro_uses_authoritative_achievable_twist() -> None:
    source_root = Path(__file__).resolve().parents[2]
    launch = (
        source_root
        / 'mowgli_bringup'
        / 'launch'
        / 'sim_full_system.launch.py'
    ).read_text()

    assert '"synthesize_from_applied_twist": True' in launch
    assert '"applied_twist_topic": "/sim/applied_twist"' in launch


def test_full_sim_reads_datum_and_lever_arm_from_scenario_config() -> None:
    source_root = Path(__file__).resolve().parents[2]
    package = Path(__file__).resolve().parents[1]
    launch = (
        source_root
        / 'mowgli_bringup'
        / 'launch'
        / 'sim_full_system.launch.py'
    ).read_text()
    scenario_path = (
        package / 'config_webots' / 'mowgli_garden.yaml'
    )
    scenario = yaml.safe_load(scenario_path.read_text())

    assert 'scenario_config' in launch
    assert '"datum_lat": datum_lat' in launch
    assert '"datum_lon": datum_lon' in launch
    assert '"lever_arm_x": lever_arm_x' in launch
    assert '"lever_arm_y": lever_arm_y' in launch
    assert scenario['gps_reference']['latitude'] == 48.137154
    assert scenario['gps_reference']['longitude'] == 11.576124
    assert scenario['antenna_lever_arm'] == {'x': 0.30, 'y': 0.0}


def test_step_observations_use_native_epoch_and_actual_applied_twist(monkeypatch):
    import math
    from builtin_interfaces.msg import Time
    from types import SimpleNamespace
    from mowgli_simulation import kinematic_drive as module

    drive = module.KinematicDrive()
    poses, twists = [], []
    native = SimpleNamespace(time=1.22)
    wheels = [SimpleNamespace(position=0.0), SimpleNamespace(position=0.0)]
    values = {
        'node': SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1_793_000_000)))),
        'supervisor': SimpleNamespace(getTime=lambda: native.time),
        'wheel_sensors': [SimpleNamespace(getValue=lambda wheel=wheel: wheel.position)
                          for wheel in wheels],
        'wheel_baseline': None, 'wheel_fault': False,
        'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'z': 0.1,
        'translation_field': SimpleNamespace(setSFVec3f=lambda value: None),
        'rotation_field': SimpleNamespace(setSFRotation=lambda value: None),
        'gt_pose_pub': SimpleNamespace(publish=poses.append),
        'applied_twist_pub': SimpleNamespace(publish=twists.append),
        'robot_node': SimpleNamespace(setVelocity=lambda value: None),
    }
    for name, value in values.items():
        setattr(drive, '_KinematicDrive__' + name, value)
    drive.step()  # First finite sensor acquisition establishes the offset.
    assert poses == twists == []
    left = (0.15 - 0.3 * 0.325 / 2) * 0.02
    right = (0.15 + 0.3 * 0.325 / 2) * 0.02
    wheels[0].position, wheels[1].position = left / 0.093, right / 0.093
    native.time = 1.24
    drive.step()
    distance = (wheels[0].position * 0.093 + wheels[1].position * 0.093) / 2
    turn = (wheels[1].position * 0.093 - wheels[0].position * 0.093) / 0.325
    half = turn / 2
    assert poses[0].header.stamp.sec == 1
    assert poses[0].header.stamp.nanosec == 240_000_000
    assert twists[0].header.stamp == poses[0].header.stamp
    assert twists[0].header.frame_id == 'base_footprint'
    assert twists[0].twist.linear.x == distance / 0.02
    assert twists[0].twist.angular.z == turn / 0.02
    assert poses[0].pose.position.x == distance * (math.sin(half) / half) * math.cos(half)
    assert poses[0].pose.position.y == distance * (math.sin(half) / half) * math.sin(half)
    # After the controller stops the actual wheels, a NEW native stationary
    # interval reports zero twist and unchanged pose. Pending/stale commands
    # cannot replace an already published native acquisition's payload.
    drive.step()
    assert len(poses) == len(twists) == 1
    native.time = 1.26
    drive.step()
    assert twists[1].twist.linear.x == twists[1].twist.angular.z == 0.0
    assert poses[1].pose == poses[0].pose

    previous_yaw = turn
    for step, yaw_rate in enumerate((0.3, -0.6, 0.0, 0.6), start=1):
        native.time = 1.26 + step * 0.02
        wheels[0].position += (0.15 - yaw_rate * 0.325 / 2) * 0.02 / 0.093
        wheels[1].position += (0.15 + yaw_rate * 0.325 / 2) * 0.02 / 0.093
        drive.step()
        pose, applied = poses[-1], twists[-1]
        yaw = 2.0 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        assert abs(yaw - previous_yaw - applied.twist.angular.z * 0.02) < 1e-15
        assert applied.header.stamp == pose.header.stamp
        previous_yaw = yaw
