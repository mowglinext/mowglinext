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
  3. Behavior tree node      — mowgli_behavior
  4. Map server              — mowgli_map
  5. Wheel odometry          — mowgli_localization
  6. NavSat converter        — mowgli_localization (/gps/absolute_pose, /gps/pose_cov)
  7. Localization monitor    — mowgli_localization
  8. Diagnostics             — mowgli_monitoring
  9. MQTT bridge (optional)  — mowgli_monitoring
  10. foxglove_bridge        — WebSocket bridge for GUI and Foxglove Studio
  11. LED status ring (opt)  — mowgli_leds
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Shared robot-config loader (sibling module installed alongside this launch
# file). Deep-merges the SPARSE installed mowgli_robot.yaml over the in-package
# template defaults, so a missing key falls through to its versioned default.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_config_util import (  # noqa: E402
    DEFAULT_TOOL_WIDTH_M,
    load_robot_params,
    resolve_lidar_enabled,
    warn_lidar_key_absent,
)


def generate_launch_description() -> LaunchDescription:
    # Keep sidecar-internal GNSS transport/status channels out of Foxglove.
    # This covers the current hidden topic prefix plus older visible names.
    internal_gnss_topic_whitelist = (
        r"^(?!/(?:_gps_internal|gps_internal|universal_gnss)(?:/.*)?$).*"
    )

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
    #
    # LiDAR presence comes from the CONFIG ONLY. The LIDAR_ENABLED env var is
    # deliberately not read here — see robot_config_util's "LiDAR presence"
    # block for why the old env fallback was removed and why an absent key
    # resolves to DEFAULT_LIDAR_ENABLED with a loud warning.
    # ------------------------------------------------------------------
    _runtime_cfg_path = "/ros2_ws/config/mowgli_robot.yaml"
    _rp = load_robot_params(bringup_dir, _runtime_cfg_path)
    _lidar_enabled, _lidar_explicit = resolve_lidar_enabled(_rp)
    _early_use_lidar = "true" if _lidar_enabled else "false"
    if not _lidar_explicit:
        warn_lidar_key_absent(_runtime_cfg_path)

    # Same pre-read for the optional WS2812 status ring. Defaults FALSE in the
    # in-package template (Invariant 15), so an existing robot never starts
    # writing to a /dev/spidev it may not own until the operator opts in.
    _early_led_enabled = "true" if bool(_rp.get("led_enabled", False)) else "false"

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
        description="Enable LiDAR-dependent nodes (fusion_graph scan-matching, obstacle layer, collision monitor scan). Default read from mowgli_robot.yaml.lidar_enabled ONLY (the LIDAR_ENABLED env var is not consulted); CLI/compose override wins. Set to false for GPS-only operation without a LiDAR.",
    )

    use_obstacle_tracker_arg = DeclareLaunchArgument(
        "use_obstacle_tracker",
        default_value="true",
        description="Enable persistent obstacle tracking from /scan into the mow_progress map. Promotes static clusters to OBSTACLE_PERMANENT after 60 s, triggers replanning around them. Set to false if the tracker is misbehaving on grass-heavy terrain.",
    )

    led_enabled_arg = DeclareLaunchArgument(
        "led_enabled",
        default_value=_early_led_enabled,
        description="Launch the WS2812 status ring node (mowgli_leds). Default read from mowgli_robot.yaml.led_enabled; CLI/compose override wins. Requires the SPI4-M0 device-tree overlay (see mowgli_leds/README.md) -- the node warns once and stands down when /dev/spidev is absent.",
    )

    # use_fusion_graph and use_magnetometer are NOT declared here —
    # navigation.launch.py reads them from mowgli_robot.yaml directly
    # so the operator flips them via the runtime config (and a
    # container restart picks the change up). There is no CLI override
    # for use_fusion_graph (the launch arg was removed).

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
    # (Nav2 params are owned by navigation.launch.py, which deep-merges
    # nav2_params_base.yaml + the lidar/no-lidar overlay; nothing here loads them.)
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")
    mqtt_params = os.path.join(monitoring_dir, "config", "mqtt_bridge.yaml")

    # Robot parameters for nodes that need explicit values (e.g.
    # navsat_to_absolute_pose needs the datum). Merged params = in-package
    # template defaults with the installed sparse config layered on top, so a
    # key the installed config omits falls through to its versioned template
    # default (single source of truth).
    robot_params = load_robot_params(bringup_dir, _runtime_cfg_path)

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
            # idle_nav2_suspend: PAUSE the Nav2 lifecycle stack while parked on
            # the dock to cut idle CPU/thermal load (costmaps stop looping).
            # Default off — a deliberate per-site opt-in. RESUME is guaranteed
            # before motion by the root Nav2ResumeGuard + Nav2ReadyPoll.
            {"idle_nav2_suspend":
                bool(robot_params.get("idle_nav2_suspend", False))},
            # transit_speed / mowing_speed flow into SetNavMode, which sets
            # them on the live controllers (FollowPath.desired_linear_vel for
            # the RPP transit controller, FollowCoveragePath.speed_fast for the
            # FTC coverage controller) per nav mode. Without these the BT used
            # hardcoded 0.5/0.25 and the configured speeds never took effect.
            {"transit_speed": float(robot_params.get("transit_speed", 0.25))},
            {"mowing_speed": float(robot_params.get("mowing_speed", 0.2))},
            # mow_angle_deg: operator swath direction. -1 (negative) = AUTO
            # (coverage server picks the swath-count-minimising angle); 0..179 =
            # fixed swath angle in degrees. Read by PlanCoverageArea::buildGoal
            # off the BT blackboard into the plan_coverage action goal.
            {"mow_angle_deg": float(robot_params.get("mow_angle_deg", -1.0))},
            # Area-recording boundary resolution. Both were hardcoded in
            # main_tree.xml (0.2 m Douglas-Peucker tolerance, 2 Hz sampling),
            # which cost a field recording all but 24 vertices of a 38 m
            # perimeter. The tolerance must stay well under tool_width so a
            # boundary error can never exceed one cutting swath, and the rate
            # must be fast enough that RecordArea's 0.05 m minimum-spacing gate
            # — not the sample rate — is the binding limiter. 10 Hz covers
            # max_mps (0.5 m/s) and is also the BT tick_rate ceiling, above
            # which a higher value has no effect.
            {
                "area_simplification_tolerance": float(
                    robot_params.get("area_simplification_tolerance", 0.05)
                )
            },
            {"area_record_rate_hz": float(robot_params.get("area_record_rate_hz", 10.0))},
            # LocalizationGuard thresholds. These were previously listed in
            # mowgli_robot.yaml but NEVER forwarded here, so the node silently
            # ran its compiled defaults and the GUI's reset-to-default had
            # nothing to act on. The guard keys on GNSS solution quality from
            # /gps/status — see mowgli_behavior/localization_health.hpp for why
            # the fused covariance is the wrong signal.
            {"loc_gnss_acc_pause_m": float(robot_params.get("loc_gnss_acc_pause_m", 0.30))},
            {"loc_gnss_acc_resume_m": float(robot_params.get("loc_gnss_acc_resume_m", 0.15))},
            {"loc_gnss_stale_s": float(robot_params.get("loc_gnss_stale_s", 5.0))},
            {
                "loc_sigma_pause_persist_s": float(
                    robot_params.get("loc_sigma_pause_persist_s", 3.0)
                )
            },
            {
                "loc_sigma_resume_persist_s": float(
                    robot_params.get("loc_sigma_resume_persist_s", 2.0)
                )
            },
            # Divergence backstop — MUST stay above fusion_graph's own
            # pivot/slip inflation ceiling (~2.35 m) or the 2026-08-20
            # livelock returns. 0 disables it.
            {"loc_sigma_pause_m": float(robot_params.get("loc_sigma_pause_m", 5.0))},
            {"loc_sigma_resume_m": float(robot_params.get("loc_sigma_resume_m", 2.0))},
            {
                "loc_sigma_backstop_persist_s": float(
                    robot_params.get("loc_sigma_backstop_persist_s", 10.0)
                )
            },
            # Battery thresholds — operator-tunable in mowgli_robot.yaml and
            # surfaced on the GUI Settings page. Forwarded here under the C++
            # parameter names the behavior node declares (behavior_tree.yaml
            # uses *_pct aliases that do NOT match those names, so without
            # this passthrough the node silently runs its compiled defaults).
            # battery_critical_recovery_percent is the HYSTERESIS upper bound
            # the critical-battery handler uses to return to IDLE after a
            # recharge; it must exceed battery_critical_percent (the node
            # clamps it if not).
            # Start-pose escape (issue #487) — SAFETY-CRITICAL: this is the
            # only new PHYSICAL MOTION in the start-occupied workstream. A
            # short, bounded, open-loop nudge OPPOSITE the last commanded
            # motion, fired only on a confirmed START_OCCUPIED-with-zero-
            # progress coverage pass, with the blade verified off, routed
            # through collision_monitor and the lowest twist_mux lane.
            # Defaults live in the in-package template mowgli_robot.yaml
            # (invariant 15); the fallbacks here must match it.
            {
                "start_blocked_escape_enabled": bool(
                    robot_params.get("start_blocked_escape_enabled", True)
                )
            },
            {
                "start_blocked_escape_speed": float(
                    robot_params.get("start_blocked_escape_speed", 0.10)
                )
            },
            {
                "start_blocked_escape_distance": float(
                    robot_params.get("start_blocked_escape_distance", 0.40)
                )
            },
            {
                "start_blocked_escape_timeout_s": float(
                    robot_params.get("start_blocked_escape_timeout_s", 6.0)
                )
            },
            {
                "start_blocked_escape_min_signal_speed": float(
                    robot_params.get("start_blocked_escape_min_signal_speed", 0.03)
                )
            },
            {
                "start_blocked_escape_signal_max_age_s": float(
                    robot_params.get("start_blocked_escape_signal_max_age_s", 90.0)
                )
            },
            {"battery_full_voltage": float(robot_params.get("battery_full_voltage", 28.0))},
            {"battery_empty_voltage": float(robot_params.get("battery_empty_voltage", 24.0))},
            {"battery_critical_voltage": float(robot_params.get("battery_critical_voltage", 0.0))},
            {"battery_low_percent": float(robot_params.get("battery_low_percent", 20.0))},
            {"battery_critical_percent": float(robot_params.get("battery_critical_percent", 10.0))},
            {"battery_full_percent": float(robot_params.get("battery_full_percent", 95.0))},
            {
                "battery_critical_recovery_percent": float(
                    robot_params.get("battery_critical_recovery_percent", 30.0)
                )
            },
        ],
    )

    # ------------------------------------------------------------------
    # 4. Map server
    # ------------------------------------------------------------------
    # WGS84 datum — shared by navsat_to_absolute_pose (GPS→map projection)
    # and map_server (areas.dat datum stamp + datum-change migration of the
    # persisted map + dock pose, issue #216). Extracted once so both nodes
    # are guaranteed the same anchor.
    datum_lat = float(robot_params.get("datum_lat", 0.000000000))
    datum_lon = float(robot_params.get("datum_lon", 0.000000000))

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
            # Mirror the chassis_safety_inset coverage_server gets from
            # navigation.launch.py (operator override, else chassis_width/2).
            # The BT no longer pre-gates polygon size (the coverage server
            # reports too-small fields itself), but other BT consumers (bypass
            # arcs) still read this.
            {"chassis_safety_inset": float(
                robot_params.get(
                    "chassis_safety_inset",
                    float(robot_params.get("chassis_width", 0.40)) / 2.0))},
            {"max_obstacle_avoidance_distance":
                float(robot_params.get("max_obstacle_avoidance_distance", 2.0))},
            # Extra LETHAL margin grown around drawn obstacle polygons in the
            # keepout mask — mirrors coverage_server.obstacle_margin (injected
            # by navigation.launch.py) so the transit planner and the swath
            # planner keep the same distance from a drawn tree/root zone.
            {"obstacle_margin": min(1.0, max(0.0, float(
                robot_params.get("obstacle_margin", 0.15))))},
            # Hard area-boundary enforcement (operator intent: "lethal area
            # where there is no navigation or mowing area"). When true (default)
            # the keepout mask marks every cell outside the union of all areas
            # as LETHAL so the planner never routes out and MPPI never steers
            # out. Operator-tunable from mowgli_robot.yaml / GUI; set false only
            # if the dock/transit corridor is NOT covered by a navigation area
            # (otherwise the hard boundary would strand docking). The free slack
            # left outside each edge for RTK drift is enforce_boundary_margin_m.
            {"lethal_outside_areas": bool(
                robot_params.get("lethal_outside_areas", True))},
            {"enforce_boundary_margin_m": float(
                robot_params.get("enforce_boundary_margin_m", 0.40))},
            # tool_width is the SINGLE source of truth (mowgli_robot.yaml) for
            # both the mark_cells_mowed stamp radius / sliver detection here AND
            # coverage_server.operation_width (injected by navigation.launch.py).
            # Inject it AFTER map_params so the operator value overrides the
            # static map_server.yaml default — otherwise an operator who changes
            # tool_width moves the F2C swath spacing while this stamp radius
            # stays frozen, re-opening the un-mowed-strip / under-coverage gap.
            {"tool_width": float(robot_params.get("tool_width", DEFAULT_TOOL_WIDTH_M))},
            # Datum anchor for the persisted map (issue #216): map_server
            # stamps areas.dat with the datum its metre coordinates were
            # recorded against, and on a datum change re-projects the whole
            # map + dock pose into the new frame instead of letting them
            # shift across the garden. MUST match navsat_to_absolute_pose's
            # datum (same robot_params read).
            {"datum_lat": datum_lat},
            {"datum_lon": datum_lon},
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
    # 6a. NavSat adapter for legacy AbsolutePose-compatible consumers.
    # navsat_transform_node takes /gps/fix directly for the EKF pipeline;
    # this node keeps /gps/absolute_pose for legacy consumers and emits
    # /gps/pose_cov for ekf_map_node fusion. Universal GNSS owns /gps/status.
    # ------------------------------------------------------------------
    # (datum_lat / datum_lon extracted above, next to map_server — the two
    # nodes must share the same anchor.)
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            {
                "datum_lat": datum_lat,
                "datum_lon": datum_lon,
            },
            {"use_sim_time": use_sim_time},
        ],
    )

    # ------------------------------------------------------------------
    # 7. Localization monitor
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
    # 7b. IMU yaw calibration node (on-demand)
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
            # One-click dock calibration (CalibrateDock action). Defaults come
            # from the mowgli_bringup template (Invariant 15) via robot_params.
            {"dock_calib_reverse_distance_m": float(
                robot_params.get("dock_calib_reverse_distance_m", 2.0))},
            {"dock_calib_reverse_speed_ms": float(
                robot_params.get("dock_calib_reverse_speed_ms", 0.15))},
            {"dock_calib_redock_overshoot_m": float(
                robot_params.get("dock_calib_redock_overshoot_m", 0.30))},
            {"dock_calib_rtk_wait_timeout_s": float(
                robot_params.get("dock_calib_rtk_wait_timeout_s", 10.0))},
            {"dock_calib_redock_charge_timeout_s": float(
                robot_params.get("dock_calib_redock_charge_timeout_s", 30.0))},
            {"dock_calib_cog_min_samples": int(
                robot_params.get("dock_calib_cog_min_samples", 8))},
            {"dock_calib_cog_std_max_rad": float(
                robot_params.get("dock_calib_cog_std_max_rad", 0.0524))},
            {"dock_calib_cog_bearing_match_max_rad": float(
                robot_params.get("dock_calib_cog_bearing_match_max_rad", 0.1047))},
            {"dock_calib_min_baseline_displacement_m": float(
                robot_params.get("dock_calib_min_baseline_displacement_m", 0.5))},
        ],
    )

    # ------------------------------------------------------------------
    # 8. Diagnostics
    # ------------------------------------------------------------------
    diagnostics_node = Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[
            monitoring_params,
            {
                "use_sim_time": use_sim_time,
                # Follow the same authoritative LiDAR flag as the Nav stack so the
                # health check reports "disabled" instead of a false "no scan" error.
                "lidar_enabled": ParameterValue(use_lidar, value_type=bool),
            },
        ],
    )

    # ------------------------------------------------------------------
    # 9. MQTT bridge (optional)
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
    # 10. Foxglove Bridge — WebSocket bridge for GUI and Foxglove Studio
    # ------------------------------------------------------------------
    # Expose the public graph broadly, but keep sidecar-internal GNSS
    # transport/status topics out of Foxglove so they do not leak into the
    # GUI contract or trigger schema-resolution noise.
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
                "send_buffer_limit": 10000000,
                "num_threads": 2,
                "topic_whitelist": [internal_gnss_topic_whitelist],
                "client_topic_whitelist": [internal_gnss_topic_whitelist],
                "capabilities": [
                    "clientPublish",
                    "services",
                    "connectionGraph",
                    # Allow Foxglove Studio to read AND set ROS parameters live
                    # (e.g. tuning controller_server / coverage critics in the
                    # field). param_whitelist (default '.*') gates which params.
                    "parameters",
                    "parametersSubscribe",
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
    # LiDAR-only in practice: the tracker clusters LiDAR obstacle returns, so
    # without a scanner it only burns CPU (~30 % on a Pi 4, measured while
    # mowing with lidar_enabled: false) waiting on inputs that never come —
    # gate it on use_lidar as well.
    obstacle_tracker_node = Node(
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_obstacle_tracker"),
                    "' == 'true' and '",
                    LaunchConfiguration("use_lidar"),
                    "' == 'true'",
                ]
            )
        ),
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
    # 11. WS2812 status ring (optional)
    # ------------------------------------------------------------------
    # Read-only status display: subscribes to /behavior_tree_node/
    # high_level_status, /gps/status and /hardware_bridge/power and writes the
    # rendered frame to /dev/spidev. It commands nothing and is NOT part of the
    # blade-safety path (the STM32 keeps its own status LED and remains the sole
    # blade safety authority). Every default lives in the in-package
    # mowgli_robot.yaml template (Invariant 15), so the GUI can reset any of
    # them; only genuine overrides land in the sparse installed config.
    led_ring_node = Node(
        condition=IfCondition(LaunchConfiguration("led_enabled")),
        package="mowgli_leds",
        executable="led_ring_node",
        name="led_ring_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                # The node re-checks led_enabled itself so `ros2 run` outside
                # this launch file also stands down by default.
                "led_enabled": ParameterValue(
                    LaunchConfiguration("led_enabled"), value_type=bool
                ),
                "led_count": int(robot_params.get("led_count", 16)),
                "led_spi_device": str(
                    robot_params.get("led_spi_device", "/dev/spidev4.1")
                ),
                "led_brightness": float(robot_params.get("led_brightness", 0.6)),
                "led_refresh_hz": float(robot_params.get("led_refresh_hz", 20.0)),
                "led_keepalive_s": float(robot_params.get("led_keepalive_s", 2.0)),
                "led_status_timeout_s": float(
                    robot_params.get("led_status_timeout_s", 5.0)
                ),
                "led_device_retry_s": float(
                    robot_params.get("led_device_retry_s", 30.0)
                ),
                "led_low_battery_percent": float(
                    robot_params.get("led_low_battery_percent", 20.0)
                ),
                "led_charge_full_percent": float(
                    robot_params.get("led_charge_full_percent", 99.0)
                ),
                "led_idle_scale": float(robot_params.get("led_idle_scale", 0.10)),
                "led_spi_speed_hz": int(
                    robot_params.get("led_spi_speed_hz", 2400000)
                ),
            },
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
            led_enabled_arg,
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
            led_ring_node,
            cmd_vel_relay_node,
            # Dock heading is published by hardware_bridge at 1 Hz while
            # charging (~/dock_heading → /gnss/heading via mowgli.launch.py
            # remapping). No separate launch action needed.
        ]
    )
