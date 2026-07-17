# Copyright 2026 Mowgli Project
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
mowgli.launch.py

Main bringup launch file for the Mowgli robot mower (physical hardware).

Brings up:
  1. robot_state_publisher  – processes URDF/xacro and publishes /robot_description
                              plus static TF from URDF fixed joints.
  2. hardware_bridge        – serial bridge to the Mowgli firmware board.
  3. twist_mux              – priority-based cmd_vel multiplexer.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Shared robot-config loader (sibling module installed alongside this launch
# file). Deep-merges the SPARSE installed mowgli_robot.yaml over the in-package
# template defaults, so a missing key falls through to its versioned default.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_config_util import load_robot_params  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true.",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/mowgli",
        description="Serial port connected to the Mowgli firmware board.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")

    # ------------------------------------------------------------------
    # Robot config (mowgli_robot.yaml)
    # ------------------------------------------------------------------
    # Merged params = in-package template defaults with the installed sparse
    # config layered on top (robot_config_util.load_robot_params). A key the
    # installed config omits falls through to its versioned template default;
    # a missing installed file yields the pure template. Single source of truth.
    robot_params = load_robot_params(bringup_dir, "/ros2_ws/config/mowgli_robot.yaml")

    # ------------------------------------------------------------------
    # URDF / xacro
    # ------------------------------------------------------------------
    xacro_file = os.path.join(bringup_dir, "urdf", "mowgli.urdf.xacro")

    # Robot shape from config (all passed to URDF xacro)
    chassis_length   = float(robot_params.get("chassis_length", 0.54))
    chassis_width    = float(robot_params.get("chassis_width", 0.40))
    chassis_height   = float(robot_params.get("chassis_height", 0.19))
    chassis_center_x = float(robot_params.get("chassis_center_x", 0.18))
    wheel_radius     = float(robot_params.get("wheel_radius", 0.093))
    wheel_width      = float(robot_params.get("wheel_width", 0.04))
    wheel_track      = float(robot_params.get("wheel_track", 0.325))
    wheel_x_offset   = float(robot_params.get("wheel_x_offset", 0.0))
    caster_radius    = float(robot_params.get("caster_radius", 0.03))
    caster_track     = float(robot_params.get("caster_track", 0.36))
    blade_radius     = float(robot_params.get("blade_radius", 0.09))

    # Sensor positions from config
    lidar_x   = str(robot_params.get("lidar_x", 0.38))
    lidar_y   = str(robot_params.get("lidar_y", 0.0))
    lidar_z   = str(robot_params.get("lidar_z", 0.22))
    lidar_yaw = str(robot_params.get("lidar_yaw", 0.0))
    imu_x     = str(robot_params.get("imu_x", 0.18))
    imu_y     = str(robot_params.get("imu_y", 0.0))
    imu_z     = str(robot_params.get("imu_z", 0.095))
    imu_roll  = str(robot_params.get("imu_roll", 0.0))
    imu_pitch = str(robot_params.get("imu_pitch", 0.0))
    imu_yaw   = str(robot_params.get("imu_yaw", 0.0))
    gps_x     = str(robot_params.get("gps_x", 0.3))
    gps_y     = str(robot_params.get("gps_y", 0.0))
    gps_z     = str(robot_params.get("gps_z", 0.20))

    # Compute Nav2 footprint from chassis shape
    fp_front = chassis_center_x + chassis_length / 2.0
    fp_rear  = chassis_center_x - chassis_length / 2.0
    fp_half_w = chassis_width / 2.0
    footprint = (
        f"[[{fp_front:.3f}, {fp_half_w:.3f}], "
        f"[{fp_front:.3f}, {-fp_half_w:.3f}], "
        f"[{fp_rear:.3f}, {-fp_half_w:.3f}], "
        f"[{fp_rear:.3f}, {fp_half_w:.3f}]]"
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " chassis_length:=", str(chassis_length),
            " chassis_width:=", str(chassis_width),
            " chassis_height:=", str(chassis_height),
            " chassis_center_x:=", str(chassis_center_x),
            " wheel_radius:=", str(wheel_radius),
            " wheel_width:=", str(wheel_width),
            " wheel_track:=", str(wheel_track),
            " wheel_x_offset:=", str(wheel_x_offset),
            " caster_radius:=", str(caster_radius),
            " caster_track:=", str(caster_track),
            " blade_radius:=", str(blade_radius),
            " lidar_x:=", lidar_x,
            " lidar_y:=", lidar_y,
            " lidar_z:=", lidar_z,
            " lidar_yaw:=", lidar_yaw,
            " imu_x:=", imu_x,
            " imu_y:=", imu_y,
            " imu_z:=", imu_z,
            " imu_roll:=", imu_roll,
            " imu_pitch:=", imu_pitch,
            " imu_yaw:=", imu_yaw,
            " gps_x:=", gps_x,
            " gps_y:=", gps_y,
            " gps_z:=", gps_z,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 2. hardware_bridge
    hardware_bridge_params = os.path.join(
        bringup_dir, "config", "hardware_bridge.yaml"
    )

    hardware_bridge_node = Node(
        package="mowgli_hardware",
        executable="hardware_bridge_node",
        name="hardware_bridge",
        output="screen",
        parameters=[
            hardware_bridge_params,
            # Allow command-line override of the serial port.
            {"serial_port": serial_port},
            {"use_sim_time": use_sim_time},
            # Pass dock pose from robot config for dock position anchoring
            {"dock_pose_x": float(robot_params.get("dock_pose_x", 0.0))},
            {"dock_pose_y": float(robot_params.get("dock_pose_y", 0.0))},
            {"dock_pose_yaw": float(robot_params.get("dock_pose_yaw", 0.0))},
            {"imu_yaw": float(robot_params.get("imu_yaw", 0.0))},
            # Wheel odometry kinematics — single source of truth in
            # mowgli_robot.yaml. hardware_bridge uses ticks_per_meter for
            # host-side odometry and also re-sends it to the STM32 so the
            # wheel PI / firmware odom leave board.h TICKS_PER_M as a
            # startup-only fallback.
            {"wheel_track": float(robot_params.get("wheel_track", 0.325))},
            {"ticks_per_meter": float(robot_params.get("ticks_per_meter", 300.0))},
            # Drive-motor wheel-velocity PID + feedforward, pushed to the STM32
            # firmware so the GUI can retune the per-wheel loop without
            # reflashing. Defaults mirror the firmware compile-time fallback.
            {"wheel_pid_kp": float(robot_params.get("wheel_pid_kp", 30.0))},
            {"wheel_pid_ki": float(robot_params.get("wheel_pid_ki", 5000.0))},
            {"wheel_pid_kd": float(robot_params.get("wheel_pid_kd", 0.0))},
            {"wheel_pid_integral_limit": float(robot_params.get(
                "wheel_pid_integral_limit", 100.0))},
            {"wheel_pid_pwm_per_mps": float(robot_params.get(
                "wheel_pid_pwm_per_mps", 300.0))},
            # IMU calibration tuning (operator-tunable via the GUI).
            {"imu_cal_samples": int(robot_params.get("imu_cal_samples", 200))},
            {"imu_cal_persist_path": str(robot_params.get(
                "imu_cal_persist_path", "/ros2_ws/maps/imu_calibration.txt"))},
            {"imu_cal_auto_rest_sec": float(robot_params.get(
                "imu_cal_auto_rest_sec", 15.0))},
            {"imu_cal_periodic_recal_sec": float(robot_params.get(
                "imu_cal_periodic_recal_sec", 600.0))},
            # Lift / blade safety tuning.
            {"lift_recovery_mode": bool(robot_params.get("lift_recovery_mode", False))},
            {"lift_blade_resume_delay_sec": float(robot_params.get(
                "lift_blade_resume_delay_sec", 1.0))},
            # 2026-07-17 Option C (task #34): the host-side gyro angular-rate
            # loop (angular_rate_loop_enabled/_kp/_ki/_kff, Option B task #24)
            # is REMOVED — the yaw-rate loop now runs in firmware (task #33),
            # closing on the same gyro without the host's USB round-trip
            # latency. hardware_bridge now sends wz straight through; these
            # tune the firmware loop instead, pushed via PACKET_ID_LL_SET_YAW_PID
            # (a separate packet from SET_DRIVE_PID — Firmware-2's #33 report).
            # Defaults match the firmware's own power-on fallback.
            {"yaw_kp": float(robot_params.get("yaw_kp", 0.30))},
            {"yaw_ki": float(robot_params.get("yaw_ki", 0.40))},
            {"yaw_trim_limit_mps": float(robot_params.get("yaw_trim_limit_mps", 0.15))},
            {"yaw_loop_enabled": bool(robot_params.get("yaw_loop_enabled", True))},
            # gyro_sign is the field sign-check remedy for the physical IMU
            # mounting (UNVALIDATED default +1) — see send_yaw_pid().
            {"yaw_gyro_sign": int(robot_params.get("yaw_gyro_sign", 1))},
        ],
        # The node publishes on ~/topic (e.g. /hardware_bridge/wheel_odom).
        # behavior_tree_node subscribes to /hardware_bridge/status etc.
        remappings=[
            ("~/imu/data_raw", "/imu/data"),
            ("~/imu/mag_raw", "/imu/mag_raw"),
            ("~/wheel_odom", "/wheel_odom"),
            ("~/wheel_ticks", "/wheel_ticks"),
            ("~/emergency", "/hardware_bridge/emergency"),
            ("~/power", "/hardware_bridge/power"),
            ("~/status", "/hardware_bridge/status"),
            ("~/cmd_vel", "/cmd_vel"),
            ("~/dock_heading", "/gnss/heading"),
        ],
    )

    # 3. twist_mux
    twist_mux_params = os.path.join(bringup_dir, "config", "twist_mux.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {"use_sim_time": use_sim_time},
        ],
        # Mux output goes directly to hardware_bridge's /cmd_vel.
        # Collision_monitor sits upstream on the Nav2 path only.
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            serial_port_arg,
            robot_state_publisher_node,
            hardware_bridge_node,
            twist_mux_node,
        ]
    )
