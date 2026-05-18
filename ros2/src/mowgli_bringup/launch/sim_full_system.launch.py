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

Simulation full-system launch for the Mowgli robot mower (Webots backend).

Combines the Webots simulation environment with the full navigation and
behavior stack, using simulated time throughout.

Brings up:
  1. mowgli_simulation/launch/webots_minimal.launch.py — Webots world +
     spawned robot + ros2_control diff_drive_controller.
  2. navigation.launch.py                              — robot_localization
     (dual EKF), Nav2.
  3. Behavior tree node                                 — mowgli_behavior.
  4. Map server                                         — mowgli_map.
  5. Diagnostics                                        — mowgli_monitoring.
  6. Foxglove bridge                                    — GUI bridge.
  7. Simulation helpers (IMU noise, NavSat RTK status promotion, wheel slip,
     fake hardware bridge, navsat→absolute_pose).

NOTE: This file is in transition from the old Gazebo Ignition pipeline to
Webots. Phase 1 of the Webots migration boots the simulator + diff-drive
controller (see ``webots_minimal.launch.py``). Topic remapping for the
Webots driver outputs (e.g. ``/wheel_odom_raw``, ``/imu``, ``/scan``,
``/gps/fix_raw``) into the namespaces the rest of the stack expects is
Phase 2 work — until that lands the helpers below may not see live data.
"""

import os

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
    # Declared arguments
    # ------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="mowgli_garden.wbt",
        description="Webots world filename inside worlds_webots/.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Launch RViz2.",
    )

    # Back-compat shim: ``headless`` was a Gazebo-era flag controlling
    # the embedded Xvfb. Webots is launched in ``fast`` mode (no GUI
    # window) by default via webots_minimal.launch.py, so the flag is
    # accepted for CLI/Makefile compatibility but currently ignored.
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="(Deprecated, Gazebo-era) ignored — Webots runs in 'fast' mode with no GUI by default.",
    )

    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Enable LiDAR-dependent nodes (obstacle tracker, fusion_graph scan-matching). Set to false for GPS-only.",
    )

    # Webots execution mode. ``realtime`` is the default — sim time
    # advances at wall-clock rate (1×). Required because controller_server
    # at 20 Hz sim time gets CPU-starved under fast mode (sim runs ~5×
    # wall on this hardware → controller dt clamps to 0.5 s → PRE_ROTATE
    # PID can't close large heading errors before goal_timeout). The
    # kinematic_drive plugin handles pacing fine in either mode; the
    # bottleneck is the Nav2 controller loop. Override with ``mode:=fast``
    # for E2E test runtime when the timing budget allows.
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="realtime",
        description="Webots execution mode: realtime | fast | pause.",
    )

    # use_fusion_graph + use_magnetometer come from
    # mowgli_robot.yaml via navigation.launch.py — no need to declare
    # them here. CLI override still propagates.

    # ------------------------------------------------------------------
    # Resolved substitutions
    # use_sim_time is always true in simulation — no argument needed.
    # ------------------------------------------------------------------
    world = LaunchConfiguration("world")
    use_rviz = LaunchConfiguration("use_rviz")
    use_lidar = LaunchConfiguration("use_lidar")
    mode = LaunchConfiguration("mode")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    behavior_params = os.path.join(behavior_dir, "config", "behavior_tree.yaml")
    map_params = os.path.join(map_dir, "config", "map_server.yaml")
    monitoring_params = os.path.join(monitoring_dir, "config", "diagnostics.yaml")

    # ------------------------------------------------------------------
    # 1. Webots simulation — world + spawned robot + diff_drive_controller
    #    (Phase 1 slice — does not yet bring up sensor topic remaps to
    #    /mowgli/hardware/*.)
    # ------------------------------------------------------------------
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulation_dir, "launch", "webots_minimal.launch.py")
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": "true",
            # See the comment on ``mode_arg`` above. Defaults to ``fast``
            # which is what the E2E test commands and the human-in-the-
            # loop iteration loop both use.
            "mode": mode,
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. Navigation stack — fusion_graph (single localizer for both
    #    map→odom AND odom→base_footprint) + Nav2.
    # ------------------------------------------------------------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_ekf": "True",
            "use_lidar": use_lidar,
            # cog_to_imu defaults are fine in sim now that the node
            # self-gates the stationary anchor on |wheel_omega| (won't
            # republish a stale forward-motion yaw while the robot is
            # pivoting in place).
            #
            # Sim-only TF / cadence overrides. Hardware defaults
            # (declared in navigation.launch.py / fusion_graph.launch.py)
            # are 0.0 forward-stamp + 25 Hz factor-graph because the
            # 100 ms lead costs ~5° yaw error per pivot at 0.5 rad/s on
            # real hardware. Under sim_time, the publish/lookup phase
            # offset routinely throws ExtrapolationException without the
            # lead, and the controller queries align poorly with the
            # 25 Hz TF cadence — restore the sim-tested values here.
            # fusion_graph_tf_lead_s now also applies to the
            # odom→base_footprint TF (ekf_odom_node was removed
            # 2026-05-18; fusion_graph owns both transforms).
            "fusion_graph_tf_lead_s": "0.1",
            "fusion_graph_node_period_s": "0.02",
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
            {"use_sim_time": True},
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
            {"use_sim_time": True},
            # Inject the simulation test field. The default map_server.yaml
            # ships with empty area_* arrays so a fresh real-robot install
            # never tries to mow a phantom polygon at the GPS datum (see
            # commit history for BLOCKER #1). The sim needs a polygon to
            # exercise coverage, so override here. 9×6 m rectangle, no
            # pre-defined obstacles — runtime obstacle tracker handles
            # whatever appears in the world file.
            {
                "area_names": ["main_mow"],
                "area_polygons": ["-7.0,-3.0;2.0,-3.0;2.0,3.0;-7.0,3.0"],
                "area_is_navigation": [False],
                "area_obstacles": [""],
            },
        ],
    )

    # ------------------------------------------------------------------
    # 5. Diagnostics
    # ------------------------------------------------------------------
    diagnostics_node = Node(
        package="mowgli_monitoring",
        executable="diagnostics_node",
        name="diagnostics_node",
        output="screen",
        parameters=[
            monitoring_params,
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 6. Foxglove Bridge — binary WebSocket bridge for Foxglove Studio
    #    Connect via: ws://localhost:8765 (Foxglove WebSocket protocol)
    # ------------------------------------------------------------------
    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "use_sim_time": True,
                "send_buffer_limit": 10000000,
                "num_threads": 0,
            },
        ],
    )

    # NOTE: docking_server is launched and lifecycle-managed by Nav2's
    # navigation_launch.py (in the lifecycle_nodes list). Do NOT launch
    # it here — duplicating it exhausts DDS participants and causes
    # lifecycle conflicts.

    # ------------------------------------------------------------------
    # 7. Obstacle tracker — persistent LiDAR obstacle detection
    # ------------------------------------------------------------------
    obstacle_tracker_params = os.path.join(map_dir, "config", "obstacle_tracker.yaml")

    obstacle_tracker_node = Node(
        condition=IfCondition(use_lidar),
        package="mowgli_map",
        executable="obstacle_tracker_node",
        name="obstacle_tracker",
        output="screen",
        parameters=[
            obstacle_tracker_params,
            {"use_sim_time": True},
        ],
    )

    # ------------------------------------------------------------------
    # 8. Fake hardware bridge — stub services/topics for simulation
    # ------------------------------------------------------------------
    fake_hardware_bridge_node = Node(
        package="mowgli_simulation",
        executable="fake_hardware_bridge_node",
        name="fake_hardware_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 9. Sim NavSat RTK status promoter
    #     Production code (navsat_to_absolute_pose_node) requires
    #     STATUS_GBAS_FIX (2) for the GPS path. The sim GPS source
    #     publishes on /gps/fix_raw with default STATUS_FIX (0); this
    #     relay rewrites status -> GBAS_FIX and republishes on /gps/fix
    #     with a realistic RTK-Fixed covariance (sigma ~3 mm).
    # ------------------------------------------------------------------
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
                # Realistic mowing scenario: 90 s RTK-Fixed (open sky),
                # 30 s RTK-Float (light tree cover), 10 s no-fix (dense
                # canopy / multipath). Empty pattern → always RTK_FIXED
                # (σ=3 mm, no Python noise injection — sensor only sees
                # the simulator GPS plugin's intrinsic ~2 cm noise). Bias
                # disabled while debugging fusion_graph; restore the
                # cycle pattern once the baseline is clean.
                "quality_pattern": "",
                "noise_seed": 42,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 10. NavSat -> AbsolutePose converter (production node, but
    #     full_system.launch.py launches it directly rather than via
    #     navigation.launch.py so the sim path needs its own copy).
    #     Reads /gps/fix and publishes /gps/pose_cov
    #     (PoseWithCovarianceStamped in map frame) which ekf_map_node
    #     fuses as pose0. Without this, no GPS reaches the EKF in sim
    #     and the BT cannot transition out of IDLE.
    #
    #     Datum matches the simulator world; if you change the sim
    #     world's lat/lon, change these too.
    # ------------------------------------------------------------------
    navsat_converter_node = Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "datum_lat": 48.137154,
                "datum_lon": 11.576124,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 11. twist_mux
    #
    # In production, mowgli.launch.py runs twist_mux with output remapped
    # to /cmd_vel (TwistStamped) directly into hardware_bridge. The sim
    # path skips mowgli.launch.py and the Webots diff_drive_controller
    # consumes TwistStamped natively (use_stamped_vel: true), so the
    # mux output goes straight to /cmd_vel.
    # ------------------------------------------------------------------
    twist_mux_params = os.path.join(bringup_dir, "config", "twist_mux.yaml")
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_params,
            {"use_sim_time": True},
        ],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    # ------------------------------------------------------------------
    # 12. Sim wheel-odom adapter — relays /wheel_odom_raw → /wheel_odom
    #     and stamps the production-grade twist covariance that the real
    #     hardware_bridge_node sets (vy variance = 1e-4 enforces the
    #     non-holonomic constraint, vx σ ≈ 0.1 m/s, wz σ ≈ 0.03 rad/s).
    #     The sim diff-drive controller publishes default zero
    #     covariance, which lets GPS lateral noise leak into the EKF as
    #     apparent sideways drift and broke strip tracking in sim runs
    #     (boundary violations, COVERAGE_FAILED loops). Also injects
    #     modest periodic slip events so EKF/fusion_graph see realistic
    #     encoder/GPS divergence.
    # ------------------------------------------------------------------
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
                # Bias temporarily zeroed for fusion_graph debugging —
                # we want to isolate algorithm behaviour from sensor
                # noise. Reset to 0.05 once the baseline is clean.
                "slip_vx_bias": 0.0,
            }
        ],
    )

    # ------------------------------------------------------------------
    # 13. Sim IMU noise injector
    #     Adds gyro/accel bias-random-walk + white noise to the
    #     simulator's perfect IMU stream and republishes on /imu/data
    #     with realistic MEMS noise. Set all *_white_std and *_walk_std
    #     parameters to 0 for a noiseless A/B baseline.
    # ------------------------------------------------------------------
    sim_imu_noise_node = Node(
        package="mowgli_simulation",
        executable="sim_imu_noise.py",
        name="sim_imu_noise",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                # Webots IMU plugin (Ros2IMU in mowgli_webots.urdf) emits
                # /imu/data_sim. The previous /imu/data_raw matched the
                # Gazebo-era topic name and silently produced nothing in
                # Webots — robot_localization saw zero IMU input and the
                # map→odom TF never converged.
                "input_topic": "/imu/data_sim",
                "output_topic": "/imu/data",
                # All bias + white noise temporarily zeroed for
                # fusion_graph debugging — gives a perfect-sensor sim so
                # we can isolate algorithm behaviour from sensor noise.
                # Restore the MPU-9250 / LIS6DSL defaults once the
                # baseline is clean (gyro_white_std=0.005, walk=1e-4,
                # init=1e-3; accel_white_std=0.05, walk=1e-3,
                # init=0.05).
                "gyro_white_std": 0.0,
                "gyro_bias_walk_std": 0.0,
                "gyro_bias_init_std": 0.0,
                "accel_white_std": 0.0,
                "accel_bias_walk_std": 0.0,
                "accel_bias_init_std": 0.0,
                "noise_seed": 42,
                # Perfect-IMU sim mode: bypass Webots gyro/accel entirely
                # and synthesize from /cmd_vel. The Webots IMU is gyro-
                # noisy by construction (ODE physics leaks angular drift
                # between kinematic-drive teleport ticks → ~0.03 rad/s
                # phantom yaw rate even when stationary, which the EKF
                # accumulates into a 5°/s map-yaw drift). Setting
                # white/walk noise to 0 was not enough because that
                # noise comes from /imu/data_sim, not from this node.
                "synthesize_from_cmd_vel": True,
                "cmd_vel_topic": "/cmd_vel",
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
            use_rviz_arg,
            headless_arg,
            use_lidar_arg,
            mode_arg,
            # Subsystem includes
            simulation_launch,
            navigation_launch,
            # Individual nodes
            fake_hardware_bridge_node,
            sim_navsat_rtk_fix_node,
            navsat_converter_node,
            twist_mux_node,
            sim_wheel_slip_node,
            sim_imu_noise_node,
            behavior_tree_node,
            map_server_node,
            obstacle_tracker_node,
            diagnostics_node,
            foxglove_bridge_node,
        ]
    )
