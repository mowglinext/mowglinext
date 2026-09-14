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

    assert '"synthesize_from_cmd_vel": True' in launch
    assert '"cmd_vel_topic": "/cmd_vel_wheels"' in launch


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
