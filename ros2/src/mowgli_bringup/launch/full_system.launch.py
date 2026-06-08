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
full_system.launch.py

Complete Mowgli robot mower system launch.

Brings up all subsystems:
  1. mowgli.launch.py        — hardware bridge, RSP, twist_mux
  2. navigation.launch.py    — robot_localization (dual EKF), Nav2
  3. Behavior tree node       — mowgli_behavior
  4. Map server               — mowgli_map
  5. Wheel odometry            — mowgli_localization
  6. NavSat converter          — mowgli_localization (/gps/absolute_pose, /gps/status, /gps/pose_cov)
  7. Localization monitor      — mowgli_localization
  8. Diagnostics               — mowgli_monitoring
  9. MQTT bridge (optional)   — mowgli_monitoring
  10. foxglove_bridge — WebSocket bridge for GUI and Foxglove Studio
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
    behavior_dir = get_package_share_directory("mowgli_behavior")
    map_dir = get_package_share_directory("mowgli_map")
    monitoring_dir = get_package_share_directory("mowgli_monitoring")

    # ------------------------------------------------------------------
    # Pre-read mowgli_robot.yaml for launch-arg defaults so operator
    # toggles set in the runtime config (or via the GUI) take effect
    # without having to also touch .env / compose. CLI/compose override
    # (use_lidar:=false) still wins because DeclareLaunchArgument applies
    # its default only when no value is passed.
    # ------------------------------------------------------------------
    _runtime_cfg_path = "/ros2_ws/config/mowgli_robot.yaml"
    _early_use_lidar = "true"
    if os.path.isfile(_runtime_cfg_path):
        try:
            with open(_runtime_cfg_path, "r") as _f:
                _cfg = yaml.safe_load(_f) or {}
            _rp = _cfg.get("mowgli", {}).get("ros__parameters", {})
            if "lidar_enabled" in _rp:
                _early_use_lidar = "true" if bool(_rp["lidar_enabled"]) else "false"
        except yaml.YAMLError:
            pass

    # LIDAR_ENABLED env var overrides the yaml (back-compat with the
    # installer's .env workflow).
    _env_lidar = os.environ.get("LIDAR_ENABLED", "").strip().lower()
    if _env_lidar in ("false", "0", "no"):
        _early_use_lidar = "false"
    elif _env_lidar in ("true", "1", "yes"):
        _early_use_lidar = "true"

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
        description="Serial port for the hardware bridge.",
    )

    enable_mqtt_arg = DeclareLaunchArgument(
        "enable_mqtt",
        default_value="false",
        description="Launch the MQTT bridge node when true.",
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

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value=_early_use_lidar,
        description="Enable LiDAR-dependent nodes (fusion_graph scan-matching, obstacle layer, collision monitor scan). Default read from mowgli_robot.yaml.use_lidar (or .lidar_enabled); CLI/compose override wins. Set to false for GPS-only operation without a LiDAR.",
    )

    use_obstacle_tracker_arg = DeclareLaunchArgument(
        "use_obstacle_tracker",
        default_value="true",
        description="Enable persistent obstacle tracking from /scan into the mow_progress map. Promotes static clusters to OBSTACLE_PERMANENT after 60 s, triggers replanning around them. Set to false if the tracker is misbehaving on grass-heavy terrain.",
    )

    # use_fusion_graph and use_magnetometer are NOT declared here —
    # navigation.launch.py reads them from mowgli_robot.yaml directly
    # so the operator flips them via the runtime config (and a
    # container restart picks the change up). CLI override on the
    # top-level launch (`... use_fusion_graph:=true`) still works
    # because the arg is declared in navigation.launch.py and CLI
    # values propagate to all included files.

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    enable_mqtt = LaunchConfiguration("enable_mqtt")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    use_lidar = LaunchConfiguration("use_lidar")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    behavior_params = os.path.join(behavior_dir, "config", "behavior_tree.yaml")
    map_params = os.path.join(map_dir, "config", "map_server.yaml")
    nav2_params_file = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    mqtt_params = os.path.join(monitoring_dir, "config", "mqtt_bridge.yaml")
    # Robot-specific config (bind-mounted from mowgli-docker/config/mowgli/)
    robot_config = "/ros2_ws/config/mowgli_robot.yaml"

    # Load robot config to extract mowgli parameters for nodes that need
    # explicit values (e.g. navsat_to_absolute_pose needs datum from mowgli).
    robot_params = {}
    if os.path.isfile(robot_config):
        with open(robot_config, "r") as f:
            robot_config_yaml = yaml.safe_load(f) or {}
        robot_params = robot_config_yaml.get("mowgli", {}).get("ros__parameters", {})

    # ------------------------------------------------------------------
    # 1. mowgli.launch.py — hardware bridge, RSP, twist_mux
    # ------------------------------------------------------------------
    mowgli_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "mowgli.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "serial_port": serial_port,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. navigation.launch.py — robot_localization (dual EKF) + Nav2
    #                           (+ optional fusion_graph)
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
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
            {"use_sim_time": use_sim_time},
            # Operator-tunable BT knobs sourced from mowgli_robot.yaml
            # so they appear on the GUI Settings page.
            {"tick_rate": float(robot_params.get("tick_rate", 10.0))},
            {"bt_debug_logging": bool(robot_params.get("bt_debug_logging", False))},
            # undock_speed / undock_distance are consumed by the BackUp BT
            # instances via {undock_speed} / {undock_distance} blackboard
            # references in main_tree.xml. See issue #191.
            {"undock_speed": float(robot_params.get("undock_speed", 0.15))},
            {"undock_distance": float(robot_params.get("undock_distance", 1.0))},
            # transit_speed / mowing_speed flow into SetNavMode, which sets
            # them on the live controllers (FollowPath.desired_linear_vel for
            # the RPP transit controller, FollowCoveragePath.speed_fast for the
            # FTC coverage controller) per nav mode. Without these the BT used
            # hardcoded 0.5/0.25 and the configured speeds never took effect.
            {"transit_speed": float(robot_params.get("transit_speed", 0.25))},
            {"mowing_speed": float(robot_params.get("mowing_speed", 0.2))},
        ],
    )

    # ------------------------------------------------------------------
    # 4. Map server
    # ------------------------------------------------------------------
    map_server_node = Node(
        package="mowgli_map",
        executable="map_server_node",
        name="map_server_node",
        output="screen",
        parameters=[
            map_params,
            {"use_sim_time": use_sim_time},
            # Dock pose + body geometry from mowgli_robot.yaml. Without
            # these the map_server uses C++ defaults (0,0,0) and builds
            # an axis-aligned polygon at the map origin — wrong unless
            # the dock happens to be exactly there. The hardware_bridge
            # already receives the same dock_pose_*; here we forward to
            # the planner / keepout-mask path too.
            {"dock_pose_x": float(robot_params.get("dock_pose_x", 0.0))},
            {"dock_pose_y": float(robot_params.get("dock_pose_y", 0.0))},
            {"dock_pose_yaw": float(robot_params.get("dock_pose_yaw", 0.0))},
            {"dock_body_length_m": float(robot_params.get("dock_body_length_m", 0.80))},
            {"dock_body_width_m": float(robot_params.get("dock_body_width_m", 0.55))},
            # Bypass-arc planner geometry — lifted from physical/operational
            # sections of mowgli_robot.yaml so the cleaning-robot detour
            # around discrete obstacles uses the correct robot footprint
            # and the wall-vs-obstacle threshold operators tune per site.
            {"chassis_width": float(robot_params.get("chassis_width", 0.40))},
            {"max_obstacle_avoidance_distance":
                float(robot_params.get("max_obstacle_avoidance_distance", 2.0))},
        ],
    )

    # ------------------------------------------------------------------
    # Wheel odometry is produced directly by hardware_bridge on
    # /wheel_odom (from the firmware's odom packet). The old
    # mowgli_localization/wheel_odometry_node subscribed to /wheel_ticks
    # and re-published /wheel_odom — but /wheel_ticks has no publisher
    # on this branch, so that node was dead weight and a duplicate
    # publisher for /wheel_odom. Keep the source in the package for now
    # (disabled) and rely on hardware_bridge alone.
    # ------------------------------------------------------------------
    # 7a. NavSat adapter for legacy AbsolutePose + typed GNSS status
    # navsat_transform_node takes /gps/fix directly for the EKF pipeline;
    # this node publishes /gps/status through the shared GNSS adapter,
    # keeps /gps/absolute_pose for legacy consumers, and emits /gps/pose_cov
    # for ekf_map_node fusion.
    # ------------------------------------------------------------------
    datum_lat = float(robot_params.get("datum_lat", 0.0))
    datum_lon = float(robot_params.get("datum_lon", 0.0))
    gnss_backend = os.environ.get("GNSS_BACKEND", "gps")
    gps_protocol = os.environ.get("GPS_PROTOCOL", "UBX")
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            {
                "datum_lat": datum_lat,
                "datum_lon": datum_lon,
                "gnss_backend": gnss_backend,
                "gps_protocol": gps_protocol,
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 8. Localization monitor
    # ------------------------------------------------------------------
    localization_monitor_node = Node(
        package="mowgli_localization",
        executable="localization_monitor_node",
        name="localization_monitor_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 8b. IMU yaw calibration node (on-demand)
    # Exposes /calibrate_imu_yaw_node/calibrate — idle until called.
    # ------------------------------------------------------------------
    calibrate_imu_yaw_node = Node(
        package="mowgli_localization",
        executable="calibrate_imu_yaw_node",
        name="calibrate_imu_yaw_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"undock_distance": float(robot_params.get("undock_distance", 2.0))},
            {"undock_speed": float(robot_params.get("undock_speed", 0.15))},
        ],
    )

    # ------------------------------------------------------------------
    # 9. Diagnostics
    # ------------------------------------------------------------------
    diagnostics_node = Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[
            monitoring_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 10. MQTT bridge (optional)
    # ------------------------------------------------------------------
    mqtt_bridge_node = Node(
        condition=IfCondition(enable_mqtt),
        package="mowgli_monitoring",
        executable="mqtt_bridge_node",
        name="mqtt_bridge_node",
        output="screen",
        parameters=[
            mqtt_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 11. Foxglove Bridge — WebSocket bridge for GUI and Foxglove Studio
    # ------------------------------------------------------------------
    # No topic/service whitelists — all topics are available for Foxglove
    # Studio debugging. The GUI backend throttles subscriptions on its side.
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
                "send_buffer_limit": 1000000,
                "num_threads": 2,
                "capabilities": [
                    "clientPublish",
                    "services",
                    "connectionGraph",
                ],
            },
        ],
    )

    # NOTE: docking_server is launched and lifecycle-managed by Nav2's
    # navigation_launch.py (in the lifecycle_nodes list). Do NOT launch
    # it here — duplicating it exhausts DDS participants and causes
    # lifecycle conflicts.

    # ------------------------------------------------------------------
    # 12. cmd_vel WebSocket relay — low-latency manual mowing control
    # ------------------------------------------------------------------
    # Accepts TwistStamped JSON on port 8766 and publishes directly to
    # /cmd_vel_teleop via rclpy, bypassing foxglove_bridge's JSON→CDR
    # conversion overhead and the shared-connection head-of-line blocking
    # with subscription data. The Go GUI's PublisherRoute connects here
    # instead of going through foxglove_bridge for manual mowing.
    cmd_vel_relay_node = Node(
        package="mowgli_bringup",
        executable="cmd_vel_ws_relay.py",
        name="cmd_vel_ws_relay",
        output="screen",
    )

    # ------------------------------------------------------------------
    # 13. Obstacle tracker — persistent LiDAR obstacle detection
    # ------------------------------------------------------------------
    obstacle_tracker_params = os.path.join(
        map_dir, "config", "obstacle_tracker.yaml"
    )

    # Obstacle tracker — DBSCAN-clusters LiDAR obstacle returns from the
    # global costmap, promotes clusters to PERSISTENT after
    # `persistence_threshold` seconds (see obstacle_tracker.yaml) of
    # stable observation, and publishes them on
    # /obstacle_tracker/obstacles. map_server_node consumes those,
    # marks the impacted cells OBSTACLE_PERMANENT in the
    # classification layer, republishes the keepout mask so the global
    # costmap routes around them, and triggers a replan so the BT
    # picks up new strips that avoid the obstacle. Transient obstacles
    # (a person walking by) expire after transient_timeout (5 s) and
    # don't permanently shape the map.
    #
    # Was disabled in launch in an earlier iteration because the
    # tracker promoted too aggressively and produced large persistent
    # obstacles from grass/ground returns. Re-enabled with the
    # currently-shipping tuning in obstacle_tracker.yaml
    # (persistence_threshold 10 s; tightening to ≥ 30 s reduces the
    # bystander-permanently-shapes-the-map effect at the cost of slower
    # adaptation to real new obstacles). Toggle off via the
    # use_obstacle_tracker launch arg if it misbehaves on real grass.
    obstacle_tracker_node = Node(
        condition=IfCondition(LaunchConfiguration("use_obstacle_tracker")),
        package="mowgli_map",
        executable="obstacle_tracker_node",
        name="obstacle_tracker",
        output="screen",
        parameters=[
            obstacle_tracker_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            # Arguments
            use_sim_time_arg,
            serial_port_arg,
            enable_mqtt_arg,
            enable_foxglove_arg,
            foxglove_port_arg,
            use_lidar_arg,
            use_obstacle_tracker_arg,
            # Subsystem includes
            mowgli_launch,
            navigation_launch,
            # Individual nodes
            behavior_tree_node,
            map_server_node,
            obstacle_tracker_node,
            navsat_converter_node,  # publishes /gps/absolute_pose for GUI + BT
            localization_monitor_node,
            calibrate_imu_yaw_node,
            diagnostics_node,
            mqtt_bridge_node,
            foxglove_bridge_node,
            cmd_vel_relay_node,
            # Dock heading is published by hardware_bridge at 1 Hz while
            # charging (~/dock_heading → /gnss/heading via mowgli.launch.py
            # remapping). No separate launch action needed.
        ]
    )
