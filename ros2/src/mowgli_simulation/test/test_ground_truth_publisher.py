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


def test_chassis_and_wheel_odom_share_one_actuation_model() -> None:
    package = Path(__file__).resolve().parents[1]
    source = (package / 'mowgli_simulation' / 'kinematic_drive.py').read_text()
    urdf = (package / 'urdf_webots' / 'mowgli_webots.urdf').read_text()

    assert '<cmdVelTopic>/cmd_vel_wheels</cmdVelTopic>' in urdf
    assert '<applyFirmwareModel>false</applyFirmwareModel>' in urdf
    assert "properties.get('applyFirmwareModel', 'true')" in source
    assert 'if self.__apply_firmware_model:' in source
    assert 'vx, wz = cmd_vx, cmd_wz' in source


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

    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *args, **kwargs: None)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 5.0)
    drive = module.KinematicDrive()
    poses, twists = [], []
    values = {
        'node': SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1_793_000_000)))),
        'supervisor': SimpleNamespace(getTime=lambda: 1.24),
        'last_cmd_time_s': 5.0, 'target_vx': 0.15, 'target_wz': 0.3,
        'apply_firmware_model': False, 'dt_s': 0.02,
        'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'z': 0.1,
        'translation_field': SimpleNamespace(setSFVec3f=lambda value: None),
        'rotation_field': SimpleNamespace(setSFRotation=lambda value: None),
        'gt_pose_pub': SimpleNamespace(publish=poses.append),
        'applied_twist_pub': SimpleNamespace(publish=twists.append),
        'robot_node': SimpleNamespace(setVelocity=lambda value: None),
    }
    for name, value in values.items():
        setattr(drive, '_KinematicDrive__' + name, value)
    drive.step()
    assert poses[0].header.stamp.sec == 1
    assert poses[0].header.stamp.nanosec == 240_000_000
    assert twists[0].header.stamp == poses[0].header.stamp
    assert twists[0].header.frame_id == 'base_footprint'
    assert twists[0].twist.linear.x == 0.15
    assert twists[0].twist.angular.z == 0.3
    assert poses[0].pose.position.x == 0.003 * math.cos(0.003)
    assert poses[0].pose.position.y == 0.003 * math.sin(0.003)
    # A timed-out command must report the actual zero applied at this step.
    drive._KinematicDrive__last_cmd_time_s = 4.0
    drive.step()
    assert twists[1].twist.linear.x == twists[1].twist.angular.z == 0.0
    assert poses[1].pose == poses[0].pose

    previous_yaw = 0.006
    for step, yaw_rate in enumerate((0.3, -0.6, 0.0, 0.6), start=1):
        drive._KinematicDrive__supervisor.getTime = lambda: 1.24 + step * 0.02
        drive._KinematicDrive__last_cmd_time_s = 5.0
        drive._KinematicDrive__target_wz = yaw_rate
        drive.step()
        pose, applied = poses[-1], twists[-1]
        yaw = 2.0 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        assert abs(yaw - previous_yaw - applied.twist.angular.z * 0.02) < 1e-15
        assert applied.header.stamp == pose.header.stamp
        previous_yaw = yaw
