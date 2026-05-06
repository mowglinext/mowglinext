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
sim_full_system.launch.py

Sim equivalent of full_system.launch.py.

Replaces the hardware-only pieces (mowgli.launch.py — serial bridge, real
twist_mux, MQTT) with simulator pieces (Gazebo world + spawned robot,
fake_hardware_bridge, sim-specific noise/relay shims) and otherwise mirrors
full_system's structure node-for-node so sim and prod behave the same.

Brings up:
  1. mowgli_simulation/launch/simulation.launch.py — Gazebo world + spawned robot
  1b. Topic relays (sim → /mowgli/hardware namespace expected by EKF config)
  2. navigation.launch.py                          — robot_localization (dual EKF) + Nav2
  3. behavior_tree_node                            — mowgli_behavior
  4. map_server_node                               — mowgli_map (coverage + keepout)
  5. obstacle_tracker_node                         — persistent LiDAR obstacle tracker
  6. navsat_to_absolute_pose_node                  — GPS → /gps/pose_cov for the EKF
  7. localization_monitor_node                     — pose-quality diagnostics
  8. calibrate_imu_yaw_node                        — on-demand yaw calibration service
  9. diagnostics_node                              — mowgli_monitoring
  10. foxglove_bridge                              — WebSocket bridge for Foxglove Studio
  11. Sim-only shims:
        - fake_hardware_bridge_node (stubs the firmware bridge: status, power,
          battery_state, charging detection)
        - twist_mux + cmd_vel_unstamper (TwistStamped → Twist for gz)
        - sim_navsat_rtk_fix       (Gazebo NavSatFix → /gps/fix with
                                    GBAS_FIX status + RTK-Fixed covariance)
        - sim_wheel_slip           (periodic encoder/GPS divergence)
        - sim_imu_noise            (gyro/accel bias-walk + white noise)
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")
    simulation_dir = get_package_share_directory("mowgli_simulation")
    behavior_dir = get_package_share_directory("mowgli_behavior")
    map_dir = get_package_share_directory("mowgli_map")
    monitoring_dir = get_package_share_directory("mowgli_monitoring")

    # ------------------------------------------------------------------
    # Pre-read mowgli_robot.yaml so operator settings (use_lidar, datum,
    # tick_rate, bt_debug_logging, …) can come from the runtime config in
    # the same way full_system.launch.py reads them. CLI override on
    # the launch command still wins because DeclareLaunchArgument's
    # default applies only when nothing was passed.
    # ------------------------------------------------------------------
    robot_config = "/ros2_ws/config/mowgli_robot.yaml"
    if not os.path.isfile(robot_config):
        robot_config = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")
    robot_params = {}
    if os.path.isfile(robot_config):
        with open(robot_config, "r") as f:
            cfg = yaml.safe_load(f) or {}
        robot_params = cfg.get("mowgli", {}).get("ros__parameters", {})

    early_use_lidar = "true"
    if "use_lidar" in robot_params:
        early_use_lidar = "true" if bool(robot_params["use_lidar"]) else "false"
    elif "lidar_enabled" in robot_params:
        early_use_lidar = "true" if bool(robot_params["lidar_enabled"]) else "false"

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="garden",
        description="Gazebo world name (garden, empty_garden, small_garden) or path to SDF.",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo in headless mode (no GUI).",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value=early_use_lidar,
        description="Enable LiDAR-dependent nodes (fusion_graph scan-matching, obstacle_tracker, "
                    "collision_monitor scan source). Default read from mowgli_robot.yaml.use_lidar; "
                    "CLI override wins.",
    )

    use_obstacle_tracker_arg = DeclareLaunchArgument(
        "use_obstacle_tracker",
        default_value="true",
        description="Enable persistent obstacle tracking from /global_costmap into "
                    "/obstacle_tracker/obstacles. Set to false to disable for debugging.",
    )

    enable_foxglove_arg = DeclareLaunchArgument(
        "enable_foxglove",
        default_value="true",
        description="Launch foxglove_bridge for the GUI when true.",
    )

    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="Port number for the Foxglove Bridge WebSocket server.",
    )

    # use_sim_time is always true here — sim has no other meaning. We don't
    # expose it as an argument because flipping it false would break every
    # downstream node that assumes /clock.
    sim_time = {"use_sim_time": True}

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    use_lidar = LaunchConfiguration("use_lidar")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    behavior_params = os.path.join(behavior_dir, "config", "behavior_tree.yaml")
    map_params = os.path.join(map_dir, "config", "map_server.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    localization_params = os.path.join(bringup_dir, "config", "robot_localization.yaml")
    obstacle_tracker_params = os.path.join(map_dir, "config", "obstacle_tracker.yaml")
    twist_mux_params = os.path.join(bringup_dir, "config", "twist_mux.yaml")

    # ------------------------------------------------------------------
    # 1. Gazebo simulation — world + spawned robot + robot_state_publisher
    #
    # Spawn yaw is forced to dock_pose_yaw from mowgli_robot.yaml so the
    # robot's physical orientation in Gazebo matches what the rest of
    # the stack thinks the dock is facing. With spawn yaw=0 (the default
    # in simulation.launch.py) and dock_pose_yaw=4.17 in mowgli_robot.yaml,
    # the IMU yaw seeded on dock and the actual yaw disagree by 4.17 rad,
    # which poisons the GPS lever-arm correction (rotated by EKF yaw)
    # and pushes /odometry/filtered_map ~0.30 m off ground truth — exactly
    # the offset that trips BoundaryGuard at strip-end U-turns.
    # ------------------------------------------------------------------
    dock_pose_yaw_str = str(float(robot_params.get("dock_pose_yaw", 0.0)))
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": world,
            "headless": headless,
            "use_rviz": use_rviz,
            "spawn_yaw": dock_pose_yaw_str,
        }.items(),
    )

    # 1b. Topic relays: simulation topics → hardware namespace.
    # The EKF config (robot_localization.yaml) expects /mowgli/hardware/* topics
    # that come from hardware_bridge on real hardware. In sim, the gz bridge
    # publishes /wheel_odom and /imu/data directly, so we relay them in.
    relay_wheel_odom = Node(
        package="topic_tools",
        executable="relay",
        name="relay_wheel_odom",
        output="screen",
        parameters=[sim_time],
        arguments=["/wheel_odom", "/mowgli/hardware/wheel_odom"],
    )

    relay_imu = Node(
        package="topic_tools",
        executable="relay",
        name="relay_imu",
        output="screen",
        parameters=[sim_time],
        arguments=["/imu/data", "/mowgli/hardware/imu"],
    )

    # ------------------------------------------------------------------
    # 2. Navigation stack — robot_localization (dual EKF) + Nav2
    #    ekf_odom_node publishes odom→base_footprint; ekf_map_node publishes
    #    map→odom. fusion_graph (LiDAR scan-matching factor graph) is opt-in
    #    via use_fusion_graph in mowgli_robot.yaml.
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_lidar": use_lidar,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 3. Behavior tree node
    # ------------------------------------------------------------------
    behavior_tree_node = Node(
        package="mowgli_behavior",
        executable="behavior_tree_node",
        name="behavior_tree_node",
        output="screen",
        parameters=[
            behavior_params,
            sim_time,
            # Operator-tunable BT knobs sourced from mowgli_robot.yaml
            # so they appear on the GUI Settings page.
            {"tick_rate": float(robot_params.get("tick_rate", 10.0))},
            {"bt_debug_logging": bool(robot_params.get("bt_debug_logging", False))},
        ],
    )

    # ------------------------------------------------------------------
    # 4. Map server (coverage cells + keepout/speed masks)
    # The dock pose lives in mowgli_robot.yaml and gates the dock-exclusion
    # polygon: when (x|y|yaw) != 0, map_server marks the dock + approach
    # corridor and returns it as an F2C polygon hole on GetMowingArea —
    # without that, F2C plans strips through (0,0) and MPPI orbits the dock.
    # ------------------------------------------------------------------
    map_server_node = Node(
        package="mowgli_map",
        executable="map_server_node",
        name="map_server_node",
        output="screen",
        parameters=[
            map_params,
            sim_time,
            {
                "dock_pose_x": float(robot_params.get("dock_pose_x", 0.0)),
                "dock_pose_y": float(robot_params.get("dock_pose_y", 0.0)),
                "dock_pose_yaw": float(robot_params.get("dock_pose_yaw", 0.0)),
            },
        ],
    )

    # ------------------------------------------------------------------
    # 5. Obstacle tracker — persistent LiDAR obstacle detection.
    # Subscribes to /global_costmap/costmap, clusters LETHAL cells inside the
    # mowing polygon, promotes stable clusters to PERSISTENT after
    # persistence_threshold (10 s), and feeds them back to map_server which
    # marks the cells OBSTACLE_PERMANENT and republishes the keepout mask.
    # ------------------------------------------------------------------
    obstacle_tracker_node = Node(
        condition=IfCondition(LaunchConfiguration("use_obstacle_tracker")),
        package="mowgli_map",
        executable="obstacle_tracker_node",
        name="obstacle_tracker",
        output="screen",
        parameters=[obstacle_tracker_params, sim_time],
    )

    # ------------------------------------------------------------------
    # 6. NavSat → AbsolutePose converter.
    # Reads /gps/fix (NavSatFix), converts to ENU using mowgli_robot.yaml's
    # datum, applies the antenna lever-arm with the current EKF yaw, and
    # publishes /gps/pose_cov for the EKF + /gps/absolute_pose for GUI/BT.
    #
    # In sim we MUST pin the datum to the Gazebo world's <spherical_
    # coordinates> origin (Munich, 48.137154 / 11.576124 in garden.sdf)
    # — otherwise the auto-seed in navsat_to_absolute_pose_node fires on
    # the first /gps/fix arrival and seeds the datum at the *antenna's*
    # lat/lon (gz publishes /gps/fix at the antenna position, 0.30 m
    # forward of base_link). That misseats the map frame origin by
    # 0.30 m relative to Gazebo, so dock_pose=(0,0) in map ends up at
    # Gazebo (0.30, 0), the polygon (-3..3) lives at Gazebo (-2.7..3.3),
    # and the F2C plan's strip-end U-turns end up flush with the
    # polygon edge from the EKF's perspective even though the robot
    # is physically inside.
    #
    # The mowgli_robot.yaml datum is left at 0/0 because real-hardware
    # deployments set their own datum per-site — this is sim-only.
    datum_lat = float(robot_params.get("datum_lat", 0.0)) or 48.137154
    datum_lon = float(robot_params.get("datum_lon", 0.0)) or 11.576124
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            localization_params,
            {"datum_lat": datum_lat, "datum_lon": datum_lon},
            sim_time,
        ],
    )

    # ------------------------------------------------------------------
    # 7. Localization monitor — publishes pose-quality diagnostics
    # ------------------------------------------------------------------
    localization_monitor_node = Node(
        package="mowgli_localization",
        executable="localization_monitor_node",
        name="localization_monitor_node",
        output="screen",
        parameters=[localization_params, sim_time],
    )

    # ------------------------------------------------------------------
    # 8. IMU yaw calibration node (on-demand)
    # Idle until /calibrate_imu_yaw_node/calibrate is called.
    # ------------------------------------------------------------------
    calibrate_imu_yaw_node = Node(
        package="mowgli_localization",
        executable="calibrate_imu_yaw_node",
        name="calibrate_imu_yaw_node",
        output="screen",
        parameters=[sim_time],
    )

    # ------------------------------------------------------------------
    # 9. Diagnostics aggregator
    # ------------------------------------------------------------------
    diagnostics_node = Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[monitoring_params, sim_time],
    )

    # ------------------------------------------------------------------
    # 10. Foxglove Bridge — WebSocket bridge for Foxglove Studio + GUI.
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        condition=IfCondition(enable_foxglove),
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": foxglove_port,
                "address": "0.0.0.0",
                "use_sim_time": True,
                "send_buffer_limit": 10000000,
                "num_threads": 0,
                "capabilities": [
                    "clientPublish",
                    "services",
                    "connectionGraph",
                ],
            },
        ],
    )

    # NOTE: docking_server is launched and lifecycle-managed by Nav2's
    # navigation_launch.py (in the lifecycle_nodes list). Do NOT add it here
    # — duplicating it exhausts DDS participants and causes lifecycle
    # conflicts.

    # ==================================================================
    # 11. Sim-only shims (no real-hardware equivalent)
    # ==================================================================

    # 11a. fake_hardware_bridge — stubs status/power/emergency/battery.
    # Subscribes to /odometry/filtered_map (map frame, same frame as
    # dock_pose) so dock-proximity → charging detection fires at the right
    # physical location regardless of map→odom drift after EKF set_pose.
    fake_hardware_bridge_node = Node(
        package="mowgli_simulation",
        executable="fake_hardware_bridge_node",
        name="fake_hardware_bridge",
        output="screen",
        parameters=[
            sim_time,
            {
                "dock_x": float(robot_params.get("dock_pose_x", 0.0)),
                "dock_y": float(robot_params.get("dock_pose_y", 0.0)),
                "dock_pose_yaw": float(robot_params.get("dock_pose_yaw", 0.0)),
                "dock_proximity": 0.3,
            },
        ],
    )

    # 11b. twist_mux + cmd_vel unstamper.
    # Real hardware:  twist_mux → /cmd_vel (TwistStamped)   → hardware_bridge
    # Sim:            twist_mux → /cmd_vel_stamped → unstamper → /cmd_vel
    #                                                          (Twist) → gz
    # Nav2 Kilted speaks TwistStamped end-to-end; gz.msgs.Twist has no
    # stamped variant, so we strip the header right before the bridge.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params, sim_time],
        remappings=[("cmd_vel_out", "/cmd_vel_stamped")],
    )

    cmd_vel_unstamper_node = Node(
        package="mowgli_simulation",
        executable="sim_cmd_vel_unstamp.py",
        name="sim_cmd_vel_unstamp",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/cmd_vel_stamped",
                "output_topic": "/cmd_vel",
            }
        ],
    )

    # 11c. sim_navsat_rtk_fix — promotes Gazebo's STATUS_FIX (0) to
    # STATUS_GBAS_FIX (2) and stamps a realistic RTK-Fixed covariance
    # (sigma ~3 mm) on /gps/fix. quality_pattern is empty here — set
    # to e.g. "90,RTK_FIXED;30,RTK_FLOAT;10,NO_FIX" when validating
    # degraded-RTK behaviour.
    sim_navsat_rtk_fix_node = Node(
        package="mowgli_simulation",
        executable="sim_navsat_rtk_fix.py",
        name="sim_navsat_rtk_fix",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/gps/fix_raw",
                "output_topic": "/gps/fix",
                "quality_pattern": "",
                "noise_seed": 42,
            }
        ],
    )

    # 11d. sim_wheel_slip — relays /wheel_odom_raw → /wheel_odom and
    # injects modest wheel slip events (5 cm/s for 1 s every 30 s)
    # to model real-world encoder/GPS divergence on grass. The EKF
    # only fuses /wheel_odom twist, so the slip surfaces as a brief
    # encoder vs GPS / gyro disagreement that the EKF must reconcile.
    # Re-enabled after the SDF base_link audit fix (commit dd779b2e):
    # without realistic wheel-odom noise the EKF over-trusts wheel
    # integration and drifts ~16 cm under motion (sim 44 measurement).
    sim_wheel_slip_node = Node(
        package="mowgli_simulation",
        executable="sim_wheel_slip.py",
        name="sim_wheel_slip",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/wheel_odom_raw",
                "output_topic": "/wheel_odom",
                "slip_period_s": 30.0,
                "slip_duration_s": 1.0,
                "slip_vx_bias": 0.05,
            }
        ],
    )

    # 11e. sim_imu_noise — adds bias-random-walk + white noise to the
    # otherwise-perfect Gazebo IMU stream so the EKF/fusion_graph see
    # realistic MEMS noise. Tuned for ~0.8°/min yaw drift at 60 s.
    # σ_bias(t) = walk_std × √t, so at gyro_bias_walk_std=3e-5 rad/s/√s
    # the bias is 2.32e-4 rad/s after 60 s ≈ 0.013°/s ≈ 0.8°/min.
    sim_imu_noise_node = Node(
        package="mowgli_simulation",
        executable="sim_imu_noise.py",
        name="sim_imu_noise",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/imu/data_gz",
                "output_topic": "/imu/data",
                "gyro_white_std": 5.0e-4,
                "gyro_bias_walk_std": 3.0e-5,
                "gyro_bias_init_std": 5.0e-5,
                "accel_white_std": 0.01,
                "accel_bias_walk_std": 1.0e-4,
                "accel_bias_init_std": 0.005,
                "noise_seed": 42,
            }
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            world_arg,
            headless_arg,
            use_rviz_arg,
            use_lidar_arg,
            use_obstacle_tracker_arg,
            enable_foxglove_arg,
            foxglove_port_arg,
            # Sim → hardware namespace relays
            relay_wheel_odom,
            relay_imu,
            # Subsystem includes
            simulation_launch,
            navigation_launch,
            # Production-shape nodes (mirroring full_system.launch.py order)
            behavior_tree_node,
            map_server_node,
            obstacle_tracker_node,
            navsat_converter_node,
            localization_monitor_node,
            calibrate_imu_yaw_node,
            diagnostics_node,
            foxglove_bridge_node,
            # Sim-only shims
            fake_hardware_bridge_node,
            twist_mux_node,
            cmd_vel_unstamper_node,
            sim_navsat_rtk_fix_node,
            sim_wheel_slip_node,
            sim_imu_noise_node,
        ]
    )
