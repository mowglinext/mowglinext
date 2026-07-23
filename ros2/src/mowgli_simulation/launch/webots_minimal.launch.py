#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
#
# Licensed under the GNU GPL v3.

r"""
Minimal Webots launch file — Phase 1 of the Webots migration.

Boots:
  * Webots simulator with the Mowgli garden world (Xvfb headless)
  * webots_ros2_driver to bridge devices (LiDAR, IMU, GPS, motors)
  * controller_manager + diff_drive_controller + joint_state_broadcaster

Does NOT boot the full ROS stack (Nav2, fusion_graph, BT, hardware
bridge, etc.) — this slice exists to verify the cmd_vel → wheel
response is honest before layering anything on top.

Usage:
  ros2 launch mowgli_simulation webots_minimal.launch.py

Then in another shell:
  ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/TwistStamped \
    '{twist: {linear: {x: 0.0}, angular: {z: 0.8}}}'
  ros2 topic echo /wheel_odom_raw --field twist.twist.angular.z
  # Expect: ~0.8 rad/s.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    pkg_share = get_package_share_directory('mowgli_simulation')

    # ── Args ──────────────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Webots execution mode: realtime | fast | pause',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='mowgli_garden.wbt',
        description='World filename inside worlds_webots/',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ── Webots ────────────────────────────────────────────────────────────────
    webots = WebotsLauncher(
        world=PathJoinSubstitution([pkg_share, 'worlds_webots', LaunchConfiguration('world')]),
        mode=LaunchConfiguration('mode'),
        ros2_supervisor=True,
    )

    # Webots external controller node — wires the URDF to the running sim
    # and brings up the device plugins (LiDAR, IMU, GPS) + ros2_control
    # bridge to controller_manager.
    robot_description_path = os.path.join(pkg_share, 'urdf_webots', 'mowgli_webots.urdf')
    ros2_control_params = os.path.join(pkg_share, 'config_webots', 'ros2_control.yaml')

    # controller_manager subscribes to /robot_description (std_msgs/String)
    # to discover the joints declared in the URDF's <ros2_control> block.
    # WebotsController itself does not publish that topic — it only passes
    # the path to its own internal driver. We need a separate
    # robot_state_publisher to make the URDF content visible on the topic.
    with open(robot_description_path, 'r') as f:
        urdf_content = f.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': urdf_content, 'use_sim_time': use_sim_time},
        ],
    )

    mowgli_driver = WebotsController(
        robot_name='MowgliMower',
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                # We run our own robot_state_publisher above with the same
                # URDF content, so disable the driver-internal republish to
                # avoid a duplicate publisher on /robot_description (which
                # confuses controller_manager's transient_local subscriber
                # — it can latch the wrong content).
                'set_robot_state_publisher': False,
            },
            ros2_control_params,
        ],
        # Controller defaults: /diffdrive_controller/cmd_vel and
        # /diffdrive_controller/odom. Map them onto the topics the rest
        # of the Mowgli stack expects.
        remappings=[
            # /cmd_vel_wheels (not /cmd_vel) so the sim_actuation node can insert
            # the firmware deadband + angular-rate PI between the nav command
            # (/cmd_vel) and the wheels — reproducing the real actuation limit
            # cycle. sim_actuation republishes /cmd_vel → /cmd_vel_wheels.
            ('/diffdrive_controller/cmd_vel', '/cmd_vel_wheels'),
            ('/diffdrive_controller/odom', '/wheel_odom_raw'),
        ],
        # respawn=True: webots-controller has a hardcoded 30 s connect
        # timeout to the simulator. On boot, Webots takes 20-40 s to
        # finish loading the world (textures, ODE setup, etc.) and the
        # `<extern>` controller slot only opens at the end. If the 30 s
        # timeout fires before Webots is ready, the controller "Gives
        # up" and dies — taking the entire stack down because
        # WaitForControllerConnection then never starts the ros2_control
        # spawners. Allowing respawn re-attempts the connect; by the
        # second attempt Webots is ready and the connection succeeds.
        # (Phase 1 dev disabled this to surface other crashes; with the
        # Phase 2.2 fixes in place, the race is the only crash mode.)
        respawn=True,
    )

    # ── ros2_control spawners ────────────────────────────────────────────────
    # 90 s timeout: gives the Webots IPC connection + URDF parse + hardware
    # interface registration plenty of slack on first boot, especially
    # under Xvfb where startup is slower than on a real display.
    controller_manager_timeout = ['--controller-manager-timeout', '90']

    diffdrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    # Wait for the Webots driver to register before spawning controllers,
    # otherwise the spawners time out trying to find the controller_manager.
    waiting = WaitForControllerConnection(
        target_driver=mowgli_driver,
        nodes_to_start=[diffdrive_spawner, joint_state_spawner],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            mode_arg,
            world_arg,
            webots,
            webots._supervisor,
            rsp_node,
            mowgli_driver,
            waiting,
        ]
    )
