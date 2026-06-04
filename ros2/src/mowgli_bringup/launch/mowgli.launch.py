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
  2. controller_manager     – ros2_control stack: MowgliSystemInterface plugin
                              (owns the STM32 serial link + embeds the comms
                              core) driven by chained diff_drive + per-wheel pid
                              controllers (replaces the old hardware_bridge_node
                              + hand-rolled wheel-PI / gyro loop).
  3. twist_mux              – priority-based cmd_vel multiplexer.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    # Try the Docker-mounted config first, fall back to the in-package default.
    robot_config_path = "/ros2_ws/config/mowgli_robot.yaml"
    if not os.path.isfile(robot_config_path):
        robot_config_path = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")

    with open(robot_config_path, "r") as f:
        robot_config_yaml = yaml.safe_load(f) or {}

    robot_params = robot_config_yaml.get("mowgli", {}).get("ros__parameters", {})

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
            # --- ros2_control hardware interface (real STM32 drive base) ---
            " use_ros2_control:=true",
            " serial_port:=", serial_port,
            " baud_rate:=", str(robot_params.get("baud_rate", 115200)),
            " ticks_per_meter:=", str(robot_params.get("ticks_per_meter", 220.0)),
            " wheel_meas_filter_tau:=", str(robot_params.get("wheel_meas_filter_tau", 0.12)),
            " pwm_per_mps:=", str(robot_params.get("pwm_per_mps", 300.0)),
            " deadband_pwm:=", str(robot_params.get("deadband_pwm", 40.0)),
            " max_pwm:=", str(robot_params.get("max_pwm", 255.0)),
            " dock_pose_x:=", str(robot_params.get("dock_pose_x", 0.0)),
            " dock_pose_y:=", str(robot_params.get("dock_pose_y", 0.0)),
            " dock_pose_yaw:=", str(robot_params.get("dock_pose_yaw", 0.0)),
            " imu_cal_samples:=", str(robot_params.get("imu_cal_samples", 1000)),
            " lift_recovery_mode:=",
            "true" if bool(robot_params.get("lift_recovery_mode", False)) else "false",
            " use_motor_speed_velocity:=",
            "true" if bool(robot_params.get("use_motor_speed_velocity", False)) else "false",
            " motor_speed_scale_alpha:=", str(robot_params.get("motor_speed_scale_alpha", 0.02)),
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

    # 2. ros2_control controller_manager (owns the STM32 serial link via the
    #    MowgliSystemInterface plugin embedded in the robot_description, plus the
    #    chained diff_drive + per-wheel pid controllers). This REPLACES the old
    #    standalone hardware_bridge_node + its hand-rolled wheel-PI / gyro loop.
    #    The comms core (IMU, status, emergency, blade, dock-heading, /wheel_odom,
    #    …) still runs — it is embedded inside the plugin and applies the same
    #    topic remaps it used standalone (set in MowgliSystemInterface).
    controllers_yaml = os.path.join(bringup_dir, "config", "mowgli_controllers.yaml")

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": use_sim_time},
            # diff_drive_controller's wheel_radius MUST equal the plugin's (the
            # radian value cancels in the odom/velocity path only if they agree;
            # ticks_per_meter sets the true ground scale). Inject both from the
            # SAME mowgli_robot.yaml values the URDF/plugin use so they can never
            # diverge — overrides the placeholders in mowgli_controllers.yaml.
            {"diff_drive_controller.wheel_radius": wheel_radius},
            {"diff_drive_controller.wheel_separation": wheel_track},
        ],
    )

    # Spawners. joint_state_broadcaster + the two per-wheel PID controllers come
    # up first; diff_drive_controller is chained ON TOP of the PID controllers'
    # reference interfaces, so it must be activated only after they exist.
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    pid_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_controller_left_wheel", "-c", "/controller_manager"],
        output="screen",
    )
    pid_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_controller_right_wheel", "-c", "/controller_manager"],
        output="screen",
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "-c", "/controller_manager",
            # Consume twist_mux's /cmd_vel (TwistStamped).
            "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel",
        ],
        output="screen",
    )
    # Activate diff_drive only after BOTH per-wheel PID controllers are up, so
    # its chained reference interfaces are available.
    diff_drive_after_pids = RegisterEventHandler(
        OnProcessExit(target_action=pid_right_spawner, on_exit=[diff_drive_spawner])
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
            controller_manager_node,
            jsb_spawner,
            pid_left_spawner,
            pid_right_spawner,
            diff_drive_after_pids,
            twist_mux_node,
        ]
    )
