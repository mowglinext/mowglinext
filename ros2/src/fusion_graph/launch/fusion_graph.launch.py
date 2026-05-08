# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
fusion_graph.launch.py

Launches the factor-graph localizer (fusion_graph_node). Reads the GPS
datum and antenna lever-arm from install/config/mowgli/mowgli_robot.yaml
(falls back to the in-package template) and forwards them as parameters.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _read_robot_config() -> dict:
    """Load mowgli_robot.yaml and return the inner mower section."""
    runtime = "/ros2_ws/config/mowgli_robot.yaml"
    fallback = os.path.join(
        get_package_share_directory("mowgli_bringup"),
        "config", "mowgli_robot.yaml",
    )
    path = runtime if os.path.exists(runtime) else fallback
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    # Schema: mowgli.<model>.<field>
    mower = data.get("mowgli", {})
    if not isinstance(mower, dict):
        return {}
    # Pick the first model section (single-mower deployments).
    for _, fields in mower.items():
        if isinstance(fields, dict):
            return fields
    return {}


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock when true.",
    )
    use_magnetometer_arg = DeclareLaunchArgument(
        "use_magnetometer", default_value="false",
        description="Subscribe to /imu/mag_yaw and feed it into the graph (Huber-robustified). OFF by default; mag is corrupted by motor magnetic field on this chassis.",
    )
    use_scan_matching_arg = DeclareLaunchArgument(
        "use_scan_matching", default_value="false",
        description="Per-tick ICP between consecutive node scans → BetweenFactor. Required for GPS-loss tolerance.",
    )
    use_loop_closure_arg = DeclareLaunchArgument(
        "use_loop_closure", default_value="false",
        description="Loop-closure search against earlier nodes; resets accumulated drift on revisits.",
    )
    primary_mode_arg = DeclareLaunchArgument(
        "primary_mode", default_value="true",
        description="True: fusion_graph owns the map→odom TF and /odometry/filtered_map (replaces ekf_map_node). False: observer mode — output is remapped to /fusion_graph/odometry, no TF broadcast (ekf_map_node keeps owning the map frame). Set by navigation.launch.py based on whether a persisted graph file already exists.",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_magnetometer = LaunchConfiguration("use_magnetometer")
    use_scan_matching = LaunchConfiguration("use_scan_matching")
    use_loop_closure = LaunchConfiguration("use_loop_closure")
    primary_mode = LaunchConfiguration("primary_mode")

    cfg = _read_robot_config()
    datum_lat = float(cfg.get("datum_lat", 0.0) or 0.0)
    datum_lon = float(cfg.get("datum_lon", 0.0) or 0.0)
    lever_x = float(cfg.get("gps_x", 0.0) or 0.0)
    lever_y = float(cfg.get("gps_y", 0.0) or 0.0)

    params_file = os.path.join(
        get_package_share_directory("fusion_graph"),
        "config", "fusion_graph.yaml",
    )

    common_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "datum_lat": datum_lat,
            "datum_lon": datum_lon,
            "lever_arm_x": lever_x,
            "lever_arm_y": lever_y,
            "use_magnetometer": use_magnetometer,
            "use_scan_matching": use_scan_matching,
            "use_loop_closure": use_loop_closure,
            "dock_pose_yaw": float(cfg.get("dock_pose_yaw", 0.0) or 0.0),
        },
    ]

    # Two mutually-exclusive Node actions gated on primary_mode. The
    # split is purely about output topic + TF: primary publishes to
    # /odometry/filtered_map and broadcasts map→odom; observer
    # publishes the same payload to /fusion_graph/odometry and skips
    # the TF broadcast (gated in C++ by primary_mode_).
    fusion_graph_primary = Node(
        condition=IfCondition(primary_mode),
        package="fusion_graph",
        executable="fusion_graph_node",
        name="fusion_graph_node",
        output="screen",
        parameters=common_params + [{"primary_mode": True}],
    )

    fusion_graph_observer = Node(
        condition=UnlessCondition(primary_mode),
        package="fusion_graph",
        executable="fusion_graph_node",
        name="fusion_graph_node",
        output="screen",
        parameters=common_params + [{"primary_mode": False}],
        remappings=[("/odometry/filtered_map", "/fusion_graph/odometry")],
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_magnetometer_arg,
        use_scan_matching_arg,
        use_loop_closure_arg,
        primary_mode_arg,
        fusion_graph_primary,
        fusion_graph_observer,
    ])
