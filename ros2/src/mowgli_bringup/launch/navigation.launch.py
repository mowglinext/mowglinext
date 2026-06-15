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
navigation.launch.py

Navigation stack launch file for the Mowgli robot mower.

Brings up:
  1. Localization — fusion_graph_node (GTSAM iSAM2 factor graph) owns
     both map→odom AND odom→base_footprint. Map-frame inputs: wheel +
     IMU + GPS + COG (+ optional LiDAR scan-matching and loop-closure).
     Local-frame: wheel vx + gyro_z integrated at IMU rate (replaces
     the standalone robot_localization ekf_odom_node).
  2. Two helper nodes — cog_to_imu (GPS COG as a continuous absolute-
     yaw observation with adaptive covariance) and mag_yaw_publisher
     (tilt-compensated LIS3MDL magnetometer yaw, gated on
     /ros2_ws/maps/mag_calibration.yaml existing).
  3. Nav2 bringup — full navigation stack (controllers, planners,
     recoveries, BT navigator, costmaps, lifecycle).

Architecture (REP-105):
  map → odom → base_footprint → base_link → sensors
  fusion_graph_node owns both map→odom AND odom→base_footprint.
  It runs WITHOUT LiDAR when use_scan_matching=false AND
  use_loop_closure=false — the graph is then just a Pose2 backbone
  with wheel between-factors, gyro between-factors, and GNSS
  lever-arm + COG / mag unaries; local-frame DR is still produced
  from the same wheel + gyro stream.
  ekf_map_node, ekf_odom_node, slam_toolbox, Kinematic-ICP, and
  FusionCore have all been removed — see CLAUDE.md "What NOT to Do"
  for deprecated paths.
"""

import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    # ------------------------------------------------------------------
    # Package directories
    # ------------------------------------------------------------------
    bringup_dir = get_package_share_directory("mowgli_bringup")

    # ------------------------------------------------------------------
    # Pre-read mowgli_robot.yaml for launch-arg defaults.
    # Operator-facing toggles (use_fusion_graph, use_magnetometer) live
    # in the runtime config so they survive container restarts and the
    # GUI can flip them without editing launch files. CLI override
    # (foo:=true) still wins because DeclareLaunchArgument applies its
    # default only when no CLI value is set.
    # ------------------------------------------------------------------
    _runtime_cfg_path = "/ros2_ws/config/mowgli_robot.yaml"
    _early_use_lidar = "true"
    _lidar_from_yaml = False
    _early_use_magnetometer = "false"
    _early_use_scan_matching = "false"
    _early_use_loop_closure = "false"
    _early_fusion_graph_period = "0.04"
    # GPS-derived dock detection: approach the cradle off RTK-Fixed
    # /gps/absolute_pose instead of the corruptible map→odom factor-graph TF
    # (a graph that reloads corrupted on dock arrival otherwise sends the
    # robot ~2 m off and the docking action times out). Default ON; operator
    # can disable per-site (e.g. cradles where GPS only Floats) to fall back
    # to the legacy graph-TF approach.
    _early_use_gps_dock_detection = "true"
    if os.path.isfile(_runtime_cfg_path):
        try:
            with open(_runtime_cfg_path, "r") as _f:
                _cfg = yaml.safe_load(_f) or {}
            _rp = _cfg.get("mowgli", {}).get("ros__parameters", {})
            # The yaml key is `lidar_enabled` (matches install
            # template + GUI). The launch CLI arg is still
            # `use_lidar:=true|false` so existing CI / dev scripts
            # don't break.
            if "lidar_enabled" in _rp:
                _early_use_lidar = "true" if bool(_rp["lidar_enabled"]) else "false"
                _lidar_from_yaml = True
            _early_use_magnetometer = "true" if bool(
                _rp.get("use_magnetometer", False)) else "false"
            _early_use_scan_matching = "true" if bool(
                _rp.get("use_scan_matching", False)) else "false"
            _early_use_loop_closure = "true" if bool(
                _rp.get("use_loop_closure", False)) else "false"
            _early_fusion_graph_period = str(
                float(_rp.get("fusion_graph_node_period_s", 0.04)))
            _early_use_gps_dock_detection = "true" if bool(
                _rp.get("use_gps_dock_detection", True)) else "false"
        except yaml.YAMLError:
            pass

    # LIDAR_ENABLED env var is a FALLBACK ONLY — it applies only when the yaml
    # does NOT set lidar_enabled. The GUI-managed yaml is authoritative when
    # present (a stale installer .env must never override the user's config).
    if not _lidar_from_yaml:
        _env_lidar = os.environ.get("LIDAR_ENABLED", "").strip().lower()
        if _env_lidar in ("false", "0", "no"):
            _early_use_lidar = "false"
        elif _env_lidar in ("true", "1", "yes"):
            _early_use_lidar = "true"

    # ------------------------------------------------------------------
    # Loop-closure gating
    # ------------------------------------------------------------------
    # Loop closure (when use_loop_closure=true) is force-OFF on the
    # very first boot — there is no persisted graph for the iSAM2
    # backend to close against. fusion_graph_node auto-saves on dock
    # arrival, so the next boot honours the operator yaml flag.
    _graph_file = "/ros2_ws/maps/fusion_graph.graph"
    _graph_exists = os.path.isfile(_graph_file)
    _effective_use_loop_closure = (
        _early_use_loop_closure if _graph_exists else "false"
    )

    # ------------------------------------------------------------------
    # Declared arguments
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock when true.",
    )


    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value=_early_use_lidar,
        description="When false, use nav2_params_no_lidar.yaml (no obstacle layer, collision monitor pass-through). Default read from mowgli_robot.yaml.lidar_enabled; CLI/compose override wins.",
    )

    use_magnetometer_arg = DeclareLaunchArgument(
        "use_magnetometer",
        default_value=_early_use_magnetometer,
        description="Enable magnetometer yaw fusion. Default read from mowgli_robot.yaml.use_magnetometer; CLI override wins. OFF on chassis without motor-isolated mag.",
    )

    use_scan_matching_arg = DeclareLaunchArgument(
        "use_scan_matching",
        default_value=_early_use_scan_matching,
        description="LiDAR scan-matching between consecutive nodes (fusion_graph). Default read from mowgli_robot.yaml.",
    )

    use_loop_closure_arg = DeclareLaunchArgument(
        "use_loop_closure",
        default_value=_effective_use_loop_closure,
        description="Loop-closure search against earlier graph nodes (fusion_graph). Default read from mowgli_robot.yaml AND gated on a persisted graph file existing on disk — first session can't loop-close against itself.",
    )

    use_gps_dock_detection_arg = DeclareLaunchArgument(
        "use_gps_dock_detection",
        default_value=_early_use_gps_dock_detection,
        description="Approach the dock off RTK-Fixed /gps/absolute_pose (via opennav_docking external detection) instead of the corruptible map→odom factor-graph TF. Launches gps_dock_detection_node and sets simple_charging_dock.use_external_detection_pose=true. Default read from mowgli_robot.yaml.use_gps_dock_detection (default true). Set false to use the legacy graph-TF approach (e.g. cradles where GPS only Floats).",
    )

    cog_stationary_seed_rate_hz_arg = DeclareLaunchArgument(
        "cog_stationary_seed_rate_hz",
        default_value="2.0",
        description="cog_to_imu stationary anchor rate (Hz). Real hardware: 2.0 (anchors fusion_graph). Sim with kinematic teleport: set to 0.0 — the stale anchor pins ekf_map yaw against gyro integration during PRE_ROTATE (issue #200).",
    )

    # ------------------------------------------------------------------
    # TF forward-stamp / fusion_graph cadence — sim vs hardware.
    # Defaults are the HARDWARE-correct values (no forward extrapolation,
    # 25 Hz factor-graph). sim_full_system.launch.py overrides these to
    # the sim-friendly values (0.1 s lead, 50 Hz) where the sim_time
    # phase offset between publish and lookup forces ExtrapolationException
    # at lower rates / no lead. On real hardware, forward-stamping the
    # map TF by 100 ms costs 5° of yaw error per pivot at 0.5 rad/s
    # — visible on Foxglove and pushed into FTC's heading PID.
    # fusion_graph_tf_lead_s is shared by map→odom AND odom→base
    # publishers inside fusion_graph_node now that ekf_odom is gone.
    # ------------------------------------------------------------------
    fusion_graph_tf_lead_arg = DeclareLaunchArgument(
        "fusion_graph_tf_lead_s",
        default_value="0.05",
        description="fusion_graph TF forward-stamp (seconds), applied to both map→odom and odom→base_footprint. Hardware default 0.05: forward-stamping the TF ~50 ms brackets the RotationShimController's 10 Hz clock and stops the ExtrapolationException that froze cmd_vel in PRE_ROTATE on real hardware (#283). Sim should set 0.1.",
    )
    fusion_graph_node_period_arg = DeclareLaunchArgument(
        "fusion_graph_node_period_s",
        default_value=_early_fusion_graph_period,
        description="fusion_graph factor-graph node cadence (seconds). Default read from mowgli_robot.yaml; hardware fallback 0.04 = 25 Hz, recommended 0.1 = 10 Hz on Pi. Sim default 0.02 = 50 Hz.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_lidar = LaunchConfiguration("use_lidar")
    use_magnetometer = LaunchConfiguration("use_magnetometer")
    use_scan_matching = LaunchConfiguration("use_scan_matching")
    use_loop_closure = LaunchConfiguration("use_loop_closure")
    use_gps_dock_detection = LaunchConfiguration("use_gps_dock_detection")
    fusion_graph_tf_lead_s = LaunchConfiguration("fusion_graph_tf_lead_s")
    fusion_graph_node_period_s = LaunchConfiguration("fusion_graph_node_period_s")

    # ------------------------------------------------------------------
    # Config paths — one shared base + thin lidar/no-lidar overlays, deep-
    # merged in _inject_dock_pose_and_speeds below. The overlays carry ONLY
    # what genuinely differs (costmap obstacle vs static layers, scan-based
    # vs pass-through collision_monitor, FollowPath collision detection); the
    # base holds everything else so the two variants cannot silently drift.
    # ------------------------------------------------------------------
    nav2_params_base = os.path.join(bringup_dir, "config", "nav2_params_base.yaml")
    nav2_params_lidar = os.path.join(bringup_dir, "config", "nav2_params_lidar.yaml")
    nav2_params_no_lidar = os.path.join(bringup_dir, "config", "nav2_params_no_lidar.yaml")

    # Compute robot footprint from mowgli_robot.yaml so Nav2 costmaps
    # match the actual chassis shape regardless of mower model. Prefer
    # the runtime config (install/, mounted at /ros2_ws/config) which
    # reflects the operator-calibrated chassis values; fall back to the
    # in-package template only when the runtime mount is unavailable
    # (e.g. running outside the production container). Earlier versions
    # of this launch always read the package template, which silently
    # diverged from the URDF (mowgli.launch.py uses the runtime path)
    # and gave Nav2 a footprint that did not match the actual robot.
    runtime_config = "/ros2_ws/config/mowgli_robot.yaml"
    template_config = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")
    robot_config_file = (
        runtime_config if os.path.isfile(runtime_config) else template_config
    )
    footprint_str = ""
    # Physical chassis width default — overwritten from the robot config below
    # when present. Hoisted here so it is always defined for the chassis_safety_inset
    # fallback AND the coverage_server.robot_width injection (both read it via the
    # _inject_dock_pose_and_speeds closure), even on a fresh checkout with no config.
    cw = 0.40
    # LIDAR mount geometry for the costmap_scan_filter ground filter.
    # lidar_height = lidar_z (above base_link); lidar_mount_yaw rotates a
    # beam's index angle into the IMU/base frame before the gravity
    # projection (the LIDAR is ~π-mounted on this chassis, so omitting it
    # inverts the front/back ground-filter sign on a slope). imu_yaw is
    # subtracted because the gravity "up" vector is expressed in the IMU
    # frame; it is 0 on this stack but kept general.
    lidar_height_m = 0.22
    lidar_mount_yaw = 0.0
    if os.path.isfile(robot_config_file):
        with open(robot_config_file, "r") as f:
            rcfg = yaml.safe_load(f) or {}
        rp = rcfg.get("mowgli", {}).get("ros__parameters", {})
        lidar_height_m = float(rp.get("lidar_z", lidar_height_m))
        lidar_mount_yaw = float(rp.get("lidar_yaw", 0.0)) - float(rp.get("imu_yaw", 0.0))
        cl = float(rp.get("chassis_length", 0.54))
        cw = float(rp.get("chassis_width", 0.40))
        ccx = float(rp.get("chassis_center_x", 0.18))
        # Add 5cm margin to chassis footprint for costmap planning clearance
        margin = 0.05
        fp_f = ccx + cl / 2.0 + margin
        fp_r = ccx - cl / 2.0 - margin
        fp_hw = cw / 2.0 + margin
        footprint_str = (
            f"[[{fp_f:.3f}, {fp_hw:.3f}], "
            f"[{fp_f:.3f}, {-fp_hw:.3f}], "
            f"[{fp_r:.3f}, {-fp_hw:.3f}], "
            f"[{fp_r:.3f}, {fp_hw:.3f}]]"
        )

    # Read dock pose and Nav2 speed knobs from the runtime config. Dock
    # pose feeds docking_server's home_dock.pose below. The WGS84 datum
    # is read by full_system.launch.py and passed to navsat_to_absolute_pose_node
    # directly — not needed here.
    dock_pose_x = 0.0
    dock_pose_y = 0.0
    dock_pose_yaw = 0.0
    # Speeds are operator-facing knobs in mowgli_robot.yaml. Nothing read
    # them before — they were orphan params — so editing them looked like
    # it should do something but didn't. Load here and inject into the
    # Nav2 YAMLs (controller + docking) alongside the dock pose.
    #   transit_speed    → FollowPath.desired_linear_vel (RPP)
    #   mowing_speed     → FollowCoveragePath.vx_max (MPPI)
    #   undock_speed     → behavior_tree_node param of the same name,
    #                      pushed onto the BT blackboard at startup and
    #                      read by undock-flow BackUp instances via
    #                      backup_speed="{undock_speed}" in main_tree.xml.
    #                      Wired in full_system.launch.py (Node parameters
    #                      list). See issue #191.
    transit_speed = 0.3
    mowing_speed = 0.25
    datum_lat = 0.0
    datum_lon = 0.0
    # Nav2 goal/progress tolerances exposed on the GUI's Settings →
    # Navigation page. Same orphan-param story as the speeds: the YAML
    # values were being shadowed by hardcoded constants in
    # nav2_params.yaml, so editing the sliders did nothing. Inject them
    # into the rewritten Nav2 yaml below alongside the speeds.
    xy_goal_tolerance = 0.30
    yaw_goal_tolerance = 0.5
    # coverage_xy_tolerance → coverage_goal_checker.xy_goal_tolerance.
    # Per-swath DISCONTINUOUS model (2026-06-12): each swath is its own
    # follow_path goal, and PathProgressGoalChecker only fires after the robot
    # has tracked >= progress_threshold (0.95) of that swath's poses — so a LOOSE
    # xy gate is safe (progress, not proximity, prevents early firing). The exact
    # swath-end xy is irrelevant (RotationShim re-aligns at the next swath start);
    # too tight (0.05/0.15) made each swath-end goal never SUCCEED → the next
    # swath never dispatched (the hang that retired the per-swath model). 0.25 m
    # lets every swath complete and advance.
    coverage_xy_tolerance = 0.25
    # Single source of truth for blade cutting width — flowed from
    # mowgli_robot.yaml.tool_width into both map_server (param
    # tool_width, used by mark_cells_mowed stamp + sliver detection)
    # and the coverage_server (param operation_width, which becomes
    # F2C Robot::setCovWidth, controlling swath spacing). The two
    # used to be separately configured (mower_width=0.18 + statically
    # operation_width=0.20), which made map_server's stamp radius
    # narrower than F2C's swath spacing — every gap between adjacent
    # swaths had a strip of cells that map_server never marked as
    # mowed. Sharing the one number fixes that by construction.
    tool_width = 0.18
    # F2C v2 coverage tuning. Operator-tunable via the GUI's Mowing
    # section; injected into coverage_server's parameters at launch
    # so changes via mowgli_robot.yaml take effect on next bringup.
    headland_width = 0.35
    # NOTE: the coverage planner no longer plans turns (the diff-drive pivots
    # in place between explicit segments), so there is no min_turning_radius
    # knob for coverage anymore. The mowgli_robot.yaml key of that name, if
    # present, is simply ignored here.
    progress_timeout_sec = 300.0
    # num_headland_passes: 0 = auto (ceil(headland_width / tool_width)),
    # >0 forces exactly that many concentric perimeter rings.
    num_headland_passes = 0
    # swath_overlap: how much narrower F2C's swath spacing is than the physical
    # cut width. F2C's operation_width (Robot::setCovWidth) = tool_width −
    # swath_overlap, so adjacent swaths OVERLAP by this much. tool_width itself
    # (map_server stamp radius) is unchanged. Without overlap, the headland↔
    # first-swath seam and BruteForce's edge margins leave thin un-mowed bands
    # (~95% on the 9×6 m field); 0.02 m (≈11% of the 0.18 m cut) takes the
    # harness-measured coverage to 100% with no other cost. Operator-tunable via
    # mowgli_robot.yaml for sites that want more/less overlap.
    swath_overlap = 0.02
    # chassis_safety_inset: how far INSIDE the operator polygon the F2C
    # planning field is pre-shrunk before any swath/headland computation.
    # Default = chassis_width / 2 (computed below) so the chassis edge
    # cannot cross the polygon boundary under perfect tracking; tracking
    # error then has to overshoot by half the chassis to escape, which is
    # well outside the <10 mm lateral spec on coverage swaths. An explicit
    # override in mowgli_robot.yaml wins over the default.
    # TRADE-OFF (deliberate): at chassis_width/2 = 0.20 m the outermost mown
    # ring sits ~0.29 m inside the boundary, leaving a ~0.20 m uncut perimeter
    # border. That is the price of guaranteeing the chassis never leaves the
    # zone — it is what fixed the 2026-06 boundary excursion. Now that
    # coverage_server also clips commanded poses to the boundary
    # (clip_path_to_boundary), an operator who wants a narrower uncut border can
    # lower this toward ~0.10 m and lean on the clip + tracking — but that is a
    # SAFETY decision (smaller margin for tracking overshoot), so it is left to
    # an explicit mowgli_robot.yaml override, not reduced by default.
    chassis_safety_inset = None
    # Dock approach distance: how far behind the dock the opennav_docking
    # staging pose sits. Edited as `dock_approach_distance` in the GUI
    # (positive metres), injected below as the negative-X
    # `simple_charging_dock.staging_x_offset` consumed by the dock plugin.
    # (Until 2026-06 it was injected into `home_dock.staging_x_offset`,
    # the dock-instance namespace the plugin never reads, so the slider was
    # orphan and the static -1.5 m governed. See issue #192.)
    dock_approach_distance = 1.5
    # Extra inward shift of the home-dock pose (metres). MUST have a module-
    # level default: it is read unconditionally in _inject_dock_pose_and_speeds,
    # but was previously assigned only inside the `if runtime yaml exists` block
    # below — so a fresh checkout / CI run with no
    # /ros2_ws/config/mowgli_robot.yaml raised NameError and aborted the whole
    # navigation launch.
    dock_approach_overshoot = 0.05
    # SimpleChargingDock charging-current threshold (amps). 0.3 is the
    # production default (see nav2_params.yaml for the "0.1 stops too
    # early, 0.5 over-presses" rationale). Operator-overridable via
    # mowgli_robot.yaml so sites with different chargers can tune.
    dock_charging_threshold = 0.3
    # Phantom-tuning knobs surfaced through mowgli_robot.yaml so the GUI
    # can edit them without an SSH session. Defaults match the C++ node
    # defaults; override on the Settings page.
    dock_pose_yaw_sigma_rad = 0.035
    enable_mag_cal = False
    mag_cal_path = "/ros2_ws/maps/mag_calibration.yaml"
    declination_deg = 1.5
    min_horizontal_uT = 5.0
    mag_yaw_variance = 0.0027
    runtime_robot_config = "/ros2_ws/config/mowgli_robot.yaml"
    if os.path.isfile(runtime_robot_config):
        with open(runtime_robot_config, "r") as f:
            rt_cfg = yaml.safe_load(f) or {}
        rt_rp = rt_cfg.get("mowgli", {}).get("ros__parameters", {})
        dock_pose_x = float(rt_rp.get("dock_pose_x", 0.0))
        dock_pose_y = float(rt_rp.get("dock_pose_y", 0.0))
        dock_pose_yaw = float(rt_rp.get("dock_pose_yaw", 0.0))
        transit_speed = float(rt_rp.get("transit_speed", transit_speed))
        mowing_speed = float(rt_rp.get("mowing_speed", mowing_speed))
        datum_lat = float(rt_rp.get("datum_lat", 0.0))
        datum_lon = float(rt_rp.get("datum_lon", 0.0))
        xy_goal_tolerance = float(
            rt_rp.get("xy_goal_tolerance", xy_goal_tolerance))
        yaw_goal_tolerance = float(
            rt_rp.get("yaw_goal_tolerance", yaw_goal_tolerance))
        coverage_xy_tolerance = float(
            rt_rp.get("coverage_xy_tolerance", coverage_xy_tolerance))
        dock_approach_distance = float(
            rt_rp.get("dock_approach_distance", dock_approach_distance))
        dock_approach_overshoot = float(
            rt_rp.get("dock_approach_overshoot", 0.05))
        dock_charging_threshold = float(
            rt_rp.get("dock_charging_threshold", dock_charging_threshold))
        # Defensive clip: a stale per-site mowgli_robot.yaml can carry
        # the legacy 0.5 m default that breaks cell-based mowing (the
        # SimpleGoalChecker fired on tick 1 — but the coverage slot uses
        # PathProgressGoalChecker, which gates on monotonic path progress
        # (>= 0.95), so it CANNOT latch mid-swath regardless of xy tolerance.
        # The clip only guards against an absurd value; cap at 0.25 m, the
        # per-swath ceiling (must let each swath-end goal SUCCEED so the next
        # swath dispatches — see coverage_xy_tolerance comment above).
        if coverage_xy_tolerance > 0.25:
            print(
                "WARN: coverage_xy_tolerance={} m exceeds the 0.25 m ceiling. "
                "Clipping to 0.25. Update "
                "mowgli_robot.yaml.coverage_xy_tolerance to silence.".format(
                    coverage_xy_tolerance))
            coverage_xy_tolerance = 0.25
        progress_timeout_sec = float(
            rt_rp.get("progress_timeout_sec", progress_timeout_sec))
        dock_pose_yaw_sigma_rad = float(rt_rp.get(
            "dock_pose_yaw_sigma_rad", dock_pose_yaw_sigma_rad))
        enable_mag_cal = bool(rt_rp.get("enable_mag_cal", enable_mag_cal))
        mag_cal_path = str(rt_rp.get("mag_calibration_path", mag_cal_path))
        declination_deg = float(rt_rp.get("declination_deg", declination_deg))
        min_horizontal_uT = float(rt_rp.get("min_horizontal_uT", min_horizontal_uT))
        mag_yaw_variance = float(rt_rp.get("mag_yaw_variance", mag_yaw_variance))
        tool_width = float(rt_rp.get("tool_width", tool_width))
        headland_width = float(rt_rp.get("headland_width", headland_width))
        num_headland_passes = int(rt_rp.get(
            "num_headland_passes", num_headland_passes))
        swath_overlap = float(rt_rp.get("swath_overlap", swath_overlap))
        # Operator override wins; otherwise fall back to chassis_width/2
        # (cw was already read above from the same runtime config).
        if "chassis_safety_inset" in rt_rp:
            chassis_safety_inset = float(rt_rp["chassis_safety_inset"])
    if chassis_safety_inset is None:
        # cw is the chassis width read from the same runtime config a few
        # lines above; default the inset to half of it.
        chassis_safety_inset = cw / 2.0

    # Compute BT XML paths from installed package shares (not hardcoded).
    bt_nav_to_pose_xml = os.path.join(
        get_package_share_directory("mowgli_behavior"),
        "trees", "navigate_to_pose.xml",
    )
    bt_nav_through_poses_xml = os.path.join(
        get_package_share_directory("nav2_bt_navigator"),
        "behavior_trees", "navigate_through_poses_w_replanning_and_recovery.xml",
    )

    # opennav_docking declares home_dock.pose as PARAMETER_DOUBLE_ARRAY (see
    # opennav_docking/utils.hpp::parseDockParams). Nav2's RewrittenYaml can
    # only substitute scalar values; passing a stringified list "[x, y, yaw]"
    # ends up as a STRING parameter and the node rejects it with
    # "Dock home_dock has no valid 'pose'".
    #
    # So we preprocess both nav2 yaml files here — load with yaml.safe_load,
    # write the dock pose as a native list, dump to a tmp file — and hand
    # those tmp files to RewrittenYaml as its sources. RewrittenYaml then
    # handles the remaining scalar rewrites (use_sim_time, footprint, BT XML
    # paths) without touching the pose list.
    def _deep_merge(base, overlay):
        """Recursively merge overlay into base: nested dicts merge key-by-key,
        lists and scalars replace wholesale. Used to compose the shared
        nav2_params_base.yaml with the selected lidar/no-lidar overlay."""
        import copy
        out = copy.deepcopy(base)
        for k, v in overlay.items():
            if k in out and isinstance(out[k], dict) and isinstance(v, dict):
                out[k] = _deep_merge(out[k], v)
            else:
                out[k] = copy.deepcopy(v)
        return out

    def _inject_dock_pose_and_speeds(overlay_path: str) -> str:
        """Merge nav2_params_base.yaml with the given variant overlay, write
        mowgli_robot.yaml-derived values into the result, and return the temp
        file path.

        RewrittenYaml only handles scalar substitutions, so we use this
        path for anything that needs the YAML parser (lists, or when we'd
        have to guess at the dotted-path root key). Speed params are
        scalars and could technically go through RewrittenYaml, but
        doing them here keeps all robot-yaml → nav2-yaml wiring in one
        place — easier to find when tuning later.
        """
        import tempfile
        with open(nav2_params_base, "r") as fh:
            base_doc = yaml.safe_load(fh) or {}
        with open(overlay_path, "r") as fh:
            overlay_doc = yaml.safe_load(fh) or {}
        doc = _deep_merge(base_doc, overlay_doc)
        # home_dock.pose must be a YAML list (PARAMETER_DOUBLE_ARRAY).
        home_dock = (doc.setdefault("docking_server", {})
                        .setdefault("ros__parameters", {})
                        .setdefault("home_dock", {}))
        # Apply dock_approach_overshoot in the body forward direction.
        # opennav_docking's graceful_controller will drive toward this
        # shifted target and stop at docking_threshold (5 cm) before it,
        # putting the robot physically at the calibrated dock_pose with
        # firm contact on the charging cradle — instead of stopping
        # 5 cm short like the un-offset configuration did 2026-05-17.
        # The overshoot is yaml-tunable (dock_approach_overshoot in
        # mowgli_robot.yaml); 0 disables the shift cleanly.
        import math as _math
        _cos_yaw = _math.cos(dock_pose_yaw)
        _sin_yaw = _math.sin(dock_pose_yaw)
        home_dock["pose"] = [
            dock_pose_x + dock_approach_overshoot * _cos_yaw,
            dock_pose_y + dock_approach_overshoot * _sin_yaw,
            dock_pose_yaw,
        ]
        # SimpleChargingDock plugin params — charging-current threshold
        # is operator-tunable so the static nav2_params.yaml value can be
        # overridden per-site from mowgli_robot.yaml + GUI.
        scd = (doc.setdefault("docking_server", {})
                  .setdefault("ros__parameters", {})
                  .setdefault("simple_charging_dock", {}))
        scd["charging_threshold"] = dock_charging_threshold
        # GPS-derived dock detection. When enabled, SimpleChargingDock pulls
        # the live dock target from the `detected_dock_pose` topic
        # (gps_dock_detection_node, fed by RTK-Fixed /gps/absolute_pose) every
        # control loop instead of the one-shot map→odom snapshot it takes at
        # dockRobot() start — so a corrupt fusion_graph map→odom cannot send
        # the robot to the wrong place. gps_dock_detection_node already emits
        # the dock CONTACT pose in odom (the same frame fixed_frame uses, so
        # getRefinedPose skips its internal TF), so the plugin's own
        # detection-to-contact translation/rotation offsets must be ZEROED
        # (the upstream defaults assume a sensor-frame marker detection with a
        # ~0.20 m contact standoff + a pitch/roll re-frame — both wrong for an
        # already-map-aligned, contact-point pose). This is gated on the same
        # _early_use_gps_dock_detection that launches the node, so the param
        # and the publisher are always consistent on the yaml/default path.
        if _early_use_gps_dock_detection == "true":
            scd["use_external_detection_pose"] = True
            # Detection is already the dock contact point in odom: no standoff,
            # no re-frame rotation.
            scd["external_detection_translation_x"] = 0.0
            scd["external_detection_translation_y"] = 0.0
            scd["external_detection_rotation_yaw"] = 0.0
            scd["external_detection_rotation_pitch"] = 0.0
            scd["external_detection_rotation_roll"] = 0.0
            # Drop a detection that goes stale (gps_dock_detection_node stops
            # publishing only when it has NEVER had a Fixed sample; through
            # Float it republishes the last good odom-anchored detection). 2 s
            # > the 1 s default tolerates brief publisher hiccups without
            # failing the approach.
            scd["external_detection_timeout"] = 2.0
            # Light low-pass on the detection (opennav PoseFilter). The source
            # is already cm-stable RTK-Fixed, so keep the default light coef.
            scd["filter_coef"] = 0.1
        # Staging pose offset along the dock's X axis (negative = behind
        # the dock, the side the robot approaches from). yaml exposes
        # dock_approach_distance as a positive metres knob in the GUI;
        # opennav_docking expects the same value negative. This MUST live
        # under the simple_charging_dock plugin namespace — the plugin reads
        # `<plugin_name>.staging_x_offset`; writing it under `home_dock`
        # (the dock-instance namespace, which only carries type/frame/pose)
        # was silently ignored, leaving the static nav2_params.yaml value
        # to govern and orphaning the GUI knob. See issue #192.
        scd["staging_x_offset"] = -float(dock_approach_distance)

        # FollowPath (transit controller = RPP via RotationShim).
        fp = (doc.setdefault("controller_server", {})
                 .setdefault("ros__parameters", {})
                 .setdefault("FollowPath", {}))
        fp["desired_linear_vel"] = transit_speed

        # FollowCoveragePath (coverage controller = MPPI via RotationShim).
        # MPPI's forward-speed cap is vx_max; mowing_speed overrides it. (The
        # old FTC knob was speed_fast — injecting that now would warn
        # "cannot be set" and the operator's mowing_speed would never reach the
        # controller.)
        fcp = (doc.setdefault("controller_server", {})
                  .setdefault("ros__parameters", {})
                  .setdefault("FollowCoveragePath", {}))
        fcp["vx_max"] = mowing_speed

        # Goal-checker tolerances. Two checkers live under
        # controller_server: stopped_goal_checker (used by FollowPath /
        # transit) and coverage_goal_checker (used by FollowCoveragePath
        # / mowing). The transit XY/yaw tolerances and the coverage XY
        # tolerance are operator-facing, so route them through here.
        cs_params = (doc.setdefault("controller_server", {})
                        .setdefault("ros__parameters", {}))
        sgc = cs_params.setdefault("stopped_goal_checker", {})
        sgc["xy_goal_tolerance"] = xy_goal_tolerance
        sgc["yaw_goal_tolerance"] = yaw_goal_tolerance
        cgc = cs_params.setdefault("coverage_goal_checker", {})
        cgc["xy_goal_tolerance"] = coverage_xy_tolerance

        # Progress checker timeout: how long Nav2 waits for the robot to
        # achieve required_movement_radius before declaring no-progress.
        pc = cs_params.setdefault("progress_checker", {})
        pc["movement_time_allowance"] = progress_timeout_sec

        # coverage_server (mowgli_coverage / Fields2Cover v3): the F2C
        # operation_width is the swath SPACING (Robot::setCovWidth). It must not
        # be WIDER than the blade cut (that leaves un-mowed strips — the 54 %
        # coverage seen 2026-05-12 with static 0.20 vs blade 0.18). We go one
        # step further and make it slightly NARROWER than the cut by
        # swath_overlap, so adjacent swaths overlap and the headland↔swath seam +
        # BruteForce edge margins are covered (harness: 95 % → 100 % on the 9×6 m
        # field). map_server's stamp radius stays tool_width/2 (the physical
        # cut), so the two consumers are now intentionally decoupled by the
        # overlap. Clamp ≥ 0.05 so a silly override can't collapse the spacing.
        cov_params = (doc.setdefault("coverage_server", {})
                          .setdefault("ros__parameters", {}))
        cov_params["operation_width"] = max(0.05, tool_width - swath_overlap)
        # robot_width = the PHYSICAL chassis width (cw, read from the same robot
        # config above). F2C's geometry here is driven by operation_width + the
        # explicit insets, so this is semantic-only today — but tying it to the
        # real chassis keeps Robot::getWidth() honest (the static nav2_params.yaml
        # default was 0.20, half the actual 0.40 m chassis).
        cov_params["robot_width"] = cw
        cov_params["default_headland_width"] = headland_width
        cov_params["num_headland_passes"] = num_headland_passes
        cov_params["chassis_safety_inset"] = chassis_safety_inset

        tmp = tempfile.NamedTemporaryFile(
            mode="w", prefix="mowgli_nav2_", suffix=".yaml", delete=False)
        yaml.safe_dump(doc, tmp, default_flow_style=False, sort_keys=False)
        tmp.close()
        return tmp.name

    nav2_params_lidar = _inject_dock_pose_and_speeds(nav2_params_lidar)
    nav2_params_no_lidar = _inject_dock_pose_and_speeds(nav2_params_no_lidar)
    nav2_params_file = PythonExpression([
        "'", nav2_params_lidar, "' if '",
        use_lidar, "'.lower() in ('true', '1') else '",
        nav2_params_no_lidar, "'",
    ])

    # Rewrite use_sim_time, footprint, and BT XML paths throughout nav2_params.yaml.
    # (home_dock.pose is NOT in this dict — it's injected as a proper YAML
    # list by _inject_dock_pose above; RewrittenYaml can only do scalar
    # substitutions.)
    param_rewrites = {
        "use_sim_time": use_sim_time,
        "default_nav_to_pose_bt_xml": bt_nav_to_pose_xml,
        "default_nav_through_poses_bt_xml": bt_nav_through_poses_xml,
    }
    if footprint_str:
        param_rewrites["footprint"] = footprint_str

    nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key="",
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    # ------------------------------------------------------------------
    # 1. Nav2 navigation (controllers, planners, behaviors, BT navigator)
    # ------------------------------------------------------------------
    # Gate Nav2 startup on the map→odom TF being available.
    wait_for_tf_script = os.path.join(
        get_package_prefix("mowgli_bringup"),
        "lib", "mowgli_bringup", "wait_for_tf.py"
    )

    wait_for_map_odom_tf = ExecuteProcess(
        cmd=[
            "python3", wait_for_tf_script,
            "--parent", "map",
            "--child", "odom",
            "--timeout", "120",
        ],
        name="wait_for_map_odom_tf",
        output="screen",
    )

    nav2_navigation_group = GroupAction(
        actions=[
            SetParameter("bond_timeout", 10.0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        bringup_dir, "launch", "nav2_navigation_launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )

    # Launch Nav2 only after the map→odom TF is available
    nav2_after_tf = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_map_odom_tf,
            on_exit=[nav2_navigation_group],
        )
    )

    # No-lidar global_costmap needs an always-current static_layer to keep
    # the costmap reporting current_=true under Nav2 Kilted's KeepoutFilter
    # (otherwise every plan aborts with "Costmap timed out waiting for
    # update"). Publishes a single empty OccupancyGrid (latched).
    empty_static_map_pub = Node(
        package="mowgli_bringup",
        executable="empty_static_map_pub.py",
        name="empty_static_map_pub",
        output="screen",
        condition=UnlessCondition(use_lidar),
    )

    # ------------------------------------------------------------------
    # gps_link → gps static alias.
    # ------------------------------------------------------------------
    # Historical: some GNSS producers publish NavSatFix in frame_id=gps while
    # the URDF names the antenna frame gps_link. navsat_transform was removed
    # 2026-04-26, but keeping the alias is still cheap insurance for third-
    # party tools that walk the frame tree from gps.
    static_gps_link_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_gps_link_to_gps_alias",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "gps_link",
            "--child-frame-id", "gps",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # fusion_graph_node — GTSAM iSAM2 factor-graph localizer. Always
    # primary (no fallback to ekf_map_node, which was removed alongside
    # the use_fusion_graph flag in this refactor). Works WITHOUT LiDAR
    # when use_scan_matching=false AND use_loop_closure=false (default).
    # Reads datum + lever-arm from mowgli_robot.yaml inside the include.
    fusion_graph_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fusion_graph"),
                "launch", "fusion_graph.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_magnetometer": use_magnetometer,
            "use_scan_matching": use_scan_matching,
            "use_loop_closure": use_loop_closure,
            "primary_mode": "true",
            "tf_publish_lead_s": fusion_graph_tf_lead_s,
            "node_period_s": fusion_graph_node_period_s,
        }.items(),
    )

    # dock_yaw_to_set_pose was inlined into fusion_graph_node
    # (FusionGraphNode::SeedFromDockPose) on 2026-05-18 — the dock
    # pose anchor is now applied directly by the localizer on
    # is_charging rising edge, without a separate node + topic round-trip.

    # Publishes GPS course-over-ground as a synthetic sensor_msgs/Imu on
    # /imu/cog_heading so ekf_map_node can fuse it as an absolute-yaw
    # observation. Once the session is seeded and the robot is driving
    # forward faster than min_speed_ms with RTK-Fixed, this node corrects
    # gyro drift every /gps/absolute_pose sample.
    # cog_to_imu publishes a stationary "anchor" yaw at
    # stationary_seed_rate_hz Hz when GPS COG cannot be derived
    # (robot not moving forward). On real hardware this anchors the
    # fusion_graph yaw across long stationary periods. In sim with
    # KinematicDrive (which teleports without forward GPS motion),
    # the anchor pins ekf_map_node's yaw to a stale value and fights
    # the gyro integration, so a robot in PRE_ROTATE never closes
    # large heading errors (issue #200). Default 2.0 Hz, overridden
    # to 0.0 in sim_full_system.launch.py.
    cog_stationary_rate = LaunchConfiguration("cog_stationary_seed_rate_hz")
    cog_to_imu = Node(
        package="mowgli_localization",
        executable="cog_to_imu",
        name="cog_to_imu",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "datum_lat": datum_lat,
             "datum_lon": datum_lon,
             "enable_mag_cal": enable_mag_cal,
             "mag_calibration_path": mag_cal_path,
             "stationary_seed_rate_hz": cog_stationary_rate,
             # Stationary-yaw aging penalty. The republish_latched path
             # adds (rate · age)² to the variance to model the chance
             # that the latched yaw has gone stale (manual rotation,
             # gyro bias accumulation) since the last forward-motion
             # measurement. Upstream default is 0.005 rad/s ≈ 0.29 °/s,
             # which inflates σ to ~45° after 2-3 min stationary —
             # aggressive enough that fusion_graph effectively ignores
             # the seed during normal idle windows. On this chassis the
             # post-calibration gyro bias drift is closer to 0.01-0.03 °/s,
             # so 0.001 rad/s (= 0.057 °/s, ~3.4 °/min) is a much closer
             # match and keeps σ ≈ 9° after 2-3 min — still penalises
             # manual pushes but lets the EKF actually use the seed.
             "stationary_yaw_drift_rate": 0.001},
        ],
    )

    # Publishes tilt-compensated magnetic heading as a synthetic
    # sensor_msgs/Imu on /imu/mag_yaw. Gated on use_magnetometer:=true
    # AND the presence of mag_calibration.yaml. Default OFF: on the
    # current chassis the motor field induces a heading-dependent bias
    # the static cal cannot remove, so feeding mag yaw into the EKF or
    # the factor graph poisons the map-frame yaw. Only launch if the
    # operator has explicitly opted in (e.g. on a motor-isolated mag).
    mag_cal_path = "/ros2_ws/maps/mag_calibration.yaml"
    mag_cal_present = "true" if os.path.isfile(mag_cal_path) else "false"
    mag_yaw_publisher = Node(
        condition=IfCondition(PythonExpression(
            ["'", use_magnetometer, "' == 'true' and ",
             "'", mag_cal_present, "' == 'true'"])),
        package="mowgli_localization",
        executable="mag_yaw_publisher",
        name="mag_yaw_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "calibration_path": mag_cal_path,
             "declination_deg": declination_deg,
             "min_horizontal_uT": min_horizontal_uT,
             "yaw_variance": mag_yaw_variance},
        ],
    )

    # Conditional radial-blank filter for the local_costmap obstacle_layer.
    # Republishes /scan as /scan_costmap, masking returns < 0.70 m only
    # while is_charging or for 5 s after charging drops — closes the
    # 0.10–0.65 m blind ring during mowing while keeping the dock
    # invisible to BackUp's collision check (behavior_server reads
    # local_costmap/costmap_raw). collision_monitor still polls /scan
    # unfiltered and stops the robot on real-time contact.
    # Motion-compensates the sequential LaserScan rays so a 360° scan
    # acquired while rotating doesn't appear smeared by ω×scan_period in
    # the map frame. Output /scan_deskewed feeds the rest of the pipeline.
    scan_deskew = Node(
        package="mowgli_localization",
        executable="scan_deskew_node",
        name="scan_deskew",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "input_topic": "/scan",
             "output_topic": "/scan_deskewed",
             "imu_topic": "/imu/data",
             "reference": "end",
             "imu_max_age_s": 0.5},
        ],
    )

    costmap_scan_filter = Node(
        package="mowgli_localization",
        executable="costmap_scan_filter_node",
        name="costmap_scan_filter",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "input_topic": "/scan_deskewed",
             "output_topic": "/scan_costmap",
             "status_topic": "/hardware_bridge/status",
             # Always-on chassis self-return blank. YardForce 500 chassis
             # corner reach from LiDAR (mounted at body 0,0.024 above
             # base_link, chassis 0.60×0.40 centred at +0.18 X):
             #   front-left corner  (0.48, 0.20) → 0.51 m
             #   front-right corner (0.48,-0.20) → 0.53 m
             #   rear-left corner  (-0.12, 0.20) → 0.21 m
             #   rear-right corner (-0.12,-0.20) → 0.25 m
             # 0.55 m blanks all four corners + some safety. We lose
             # real-obstacle detection within 55 cm of the LiDAR, but
             # collision_monitor PolygonStop (forward extent 0.55 m)
             # already protects that zone using a polygon-shaped check
             # downstream. For a 0.3 m/s mower this is acceptable;
             # tighten if real-obstacle sensitivity is critical.
             "chassis_blank_range": 0.55,
             "dock_blank_range": 0.70,
             "post_undock_blank_sec": 5.0,
             # Ground-filter geometry from mowgli_robot.yaml. lidar_mount_yaw
             # (~π on the 180°-rotated mount) is essential — without it the
             # gravity projection's front/back sign inverts on a slope and
             # forward ground returns survive as phantom obstacles.
             "lidar_height_m": lidar_height_m,
             "lidar_mount_yaw": lidar_mount_yaw,
             # Ground-filter floor raised 0.08 → 0.15 m (2026-06-12). At
             # 0.08 a phantom ground strike only needs ~1.5° of IMU tilt
             # error at 3 m range to pass as an "obstacle" — on a bumpy
             # lawn (2D LiDAR, non-flat ground) those leaked continuously
             # and walled the robot in with phantom obstacles ("collision
             # ahead" everywhere, spin aborts). At 0.15 the required error
             # doubles, while every real obstacle that matters (dock 0.5 m,
             # legs, trunks) still returns at the 0.22 m scan-plane height
             # and passes. Sub-15 cm objects are not reliably detectable
             # with a 2D LiDAR on this terrain anyway.
             "min_obstacle_z_m": 0.15},
        ],
    )

    # GPS-derived dock detection. Publishes the true dock contact pose
    # (computed from RTK-Fixed /gps/absolute_pose + the calibrated map-frame
    # dock_pose, expressed in odom via the CONTINUOUS odom→base_footprint DR
    # — never via the corruptible map→odom) on /detected_dock_pose.
    # opennav_docking's SimpleChargingDock (use_external_detection_pose=true,
    # injected above) reads it as the live dock target, so a corrupt
    # fusion_graph map→odom cannot send the robot to the wrong place. Gated on
    # use_gps_dock_detection; when off, the legacy graph-TF approach runs and
    # this node is not launched (and use_external_detection_pose stays false).
    gps_dock_detection = Node(
        condition=IfCondition(use_gps_dock_detection),
        package="mowgli_localization",
        executable="gps_dock_detection_node",
        name="gps_dock_detection",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "dock_pose_x": dock_pose_x,
             "dock_pose_y": dock_pose_y,
             "dock_pose_yaw": dock_pose_yaw,
             # MUST match docking_server.fixed_frame / .base_frame so the
             # detection lands in the frame getRefinedPose() expects (and skips
             # its internal TF) and isDocked() compares the right frames.
             "fixed_frame": "odom",
             "base_frame": "base_footprint",
             "publish_rate_hz": 10.0,
             # Only RTK-Fixed drives a fresh detection; Float republishes the
             # last good one. Set false to also accept Float (not recommended —
             # disable the whole feature instead at Float-only cradles).
             "require_rtk_fixed": True},
        ],
        # SimpleChargingDock subscribes to the relative name "detected_dock_pose"
        # which resolves against the docking_server's NAMESPACE ("/"), not its
        # node name -> /detected_dock_pose. Publish there. (A previous remap to
        # /docking_server/detected_dock_pose left the server with 0 subscribers,
        # so docking aborted with error 904 "Failed initial dock detection".)
        remappings=[("detected_dock_pose", "/detected_dock_pose")],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_lidar_arg,
            use_magnetometer_arg,
            use_scan_matching_arg,
            use_loop_closure_arg,
            use_gps_dock_detection_arg,
            cog_stationary_seed_rate_hz_arg,
            fusion_graph_tf_lead_arg,
            fusion_graph_node_period_arg,
            # Localization helpers + fusion_graph_node (single localizer
            # for both map→odom AND odom→base_footprint; ekf_odom_node
            # was removed 2026-05-18).
            static_gps_link_alias,
            fusion_graph_launch,
            cog_to_imu,
            mag_yaw_publisher,
            scan_deskew,
            costmap_scan_filter,
            gps_dock_detection,
            wait_for_map_odom_tf,
            nav2_after_tf,
            empty_static_map_pub,
        ]
    )
