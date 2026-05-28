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
    _early_use_magnetometer = "false"
    _early_use_scan_matching = "false"
    _early_use_loop_closure = "false"
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
            _early_use_magnetometer = "true" if bool(
                _rp.get("use_magnetometer", False)) else "false"
            _early_use_scan_matching = "true" if bool(
                _rp.get("use_scan_matching", False)) else "false"
            _early_use_loop_closure = "true" if bool(
                _rp.get("use_loop_closure", False)) else "false"
        except yaml.YAMLError:
            pass

    # LIDAR_ENABLED env var overrides the yaml (back-compat with the
    # installer's .env workflow). Recognised values for "off": false,
    # 0, no. Anything else (including unset) keeps the yaml-derived
    # default.
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

    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Run the robot_localization dual EKF. Set to False in simulation where Gazebo provides the odom TF directly.",
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
        default_value="0.0",
        description="fusion_graph TF forward-stamp (seconds), applied to both map→odom and odom→base_footprint. Hardware default 0.0. Sim should set 0.1.",
    )
    fusion_graph_node_period_arg = DeclareLaunchArgument(
        "fusion_graph_node_period_s",
        default_value="0.04",
        description="fusion_graph factor-graph node cadence (seconds). Hardware default 0.04 = 25 Hz (5x faster than 5 Hz controller queries, sustainable on Pi). Sim default 0.02 = 50 Hz to absorb sim_time TF gaps.",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ekf = LaunchConfiguration("use_ekf")
    use_lidar = LaunchConfiguration("use_lidar")
    use_magnetometer = LaunchConfiguration("use_magnetometer")
    use_scan_matching = LaunchConfiguration("use_scan_matching")
    use_loop_closure = LaunchConfiguration("use_loop_closure")
    fusion_graph_tf_lead_s = LaunchConfiguration("fusion_graph_tf_lead_s")
    fusion_graph_node_period_s = LaunchConfiguration("fusion_graph_node_period_s")

    # ------------------------------------------------------------------
    # Config paths
    # ------------------------------------------------------------------
    nav2_params_lidar = os.path.join(bringup_dir, "config", "nav2_params.yaml")
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
    if os.path.isfile(robot_config_file):
        with open(robot_config_file, "r") as f:
            rcfg = yaml.safe_load(f) or {}
        rp = rcfg.get("mowgli", {}).get("ros__parameters", {})
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
    #   transit_speed    → FollowPath.desired_linear_vel + max_speed_xy
    #   mowing_speed     → FollowCoveragePath.max_speed_xy
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
    # coverage_xy_tolerance: must equal coverage_goal_checker.xy_goal_tolerance.
    # 0.10 m to match the StoppedGoalChecker tolerance — completion is
    # gated by robot velocity (must be ≈ stopped), so the xy tolerance
    # only needs to be loose enough for FTC's PID to converge.
    coverage_xy_tolerance = 0.10
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
    min_turning_radius = 0.05
    progress_timeout_sec = 300.0
    # num_headland_passes: 0 = auto (ceil(headland_width / tool_width)),
    # >0 forces exactly that many concentric perimeter rings.
    num_headland_passes = 0
    # chassis_safety_inset: how far INSIDE the operator polygon the F2C
    # planning field is pre-shrunk before any swath/headland computation.
    # Default = chassis_width / 2 (computed below) so the chassis edge
    # cannot cross the polygon boundary under perfect FTC tracking;
    # tracking error then has to overshoot by half the chassis to escape,
    # which is well outside FTC's <10 mm lateral spec on coverage swaths.
    # An explicit override in mowgli_robot.yaml wins over the default.
    chassis_safety_inset = None
    # Dock approach distance: how far behind the dock the opennav_docking
    # staging pose sits. Edited as `dock_approach_distance` in the GUI
    # (positive metres), injected here as the negative-X
    # `home_dock.staging_x_offset` consumed by docking_server. The yaml
    # value was orphan before this — editing the slider produced no
    # operational change because docking_server kept its hardcoded
    # -1.5 m default. See issue #192.
    dock_approach_distance = 1.5
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
        # SimpleGoalChecker fires on tick 1 because the strip end is
        # within tolerance of the robot's current pose, FTC reports
        # SUCCEEDED before publishing any cmd_vel, BT loops forever).
        # Cap at 0.15 m here — comfortably below tool_width (0.18 m)
        # so a forgotten YAML field can't reproduce the field bug on a
        # fresh deploy, and below typical strip length so the goal
        # checker can't latch mid-strip.
        if coverage_xy_tolerance > 0.15:
            print(
                "WARN: coverage_xy_tolerance={} m exceeds the 0.15 m safe ceiling "
                "(robot may declare success too early on the final approach, or "
                "latch mid-strip). Clipping to 0.15. Update "
                "mowgli_robot.yaml.coverage_xy_tolerance to silence.".format(
                    coverage_xy_tolerance))
            coverage_xy_tolerance = 0.15
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
        min_turning_radius = float(rt_rp.get(
            "min_turning_radius", min_turning_radius))
        num_headland_passes = int(rt_rp.get(
            "num_headland_passes", num_headland_passes))
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
    def _inject_dock_pose_and_speeds(src_path: str) -> str:
        """Write mowgli_robot.yaml-derived values into the Nav2 params YAML
        and return the temp file path.

        RewrittenYaml only handles scalar substitutions, so we use this
        path for anything that needs the YAML parser (lists, or when we'd
        have to guess at the dotted-path root key). Speed params are
        scalars and could technically go through RewrittenYaml, but
        doing them here keeps all robot-yaml → nav2-yaml wiring in one
        place — easier to find when tuning later.
        """
        import tempfile
        with open(src_path, "r") as fh:
            doc = yaml.safe_load(fh) or {}
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
        # Staging pose offset along the dock's X axis (negative = behind
        # the dock, the side the robot approaches from). yaml exposes
        # dock_approach_distance as a positive metres knob in the GUI;
        # opennav_docking expects the same value negative. Wiring the
        # two replaces the previously-orphan dock_approach_distance —
        # the GUI slider now drives the actual staging point. See
        # issue #192.
        home_dock["staging_x_offset"] = -float(dock_approach_distance)

        # SimpleChargingDock plugin params — charging-current threshold
        # is operator-tunable so the static nav2_params.yaml value can be
        # overridden per-site from mowgli_robot.yaml + GUI.
        scd = (doc.setdefault("docking_server", {})
                  .setdefault("ros__parameters", {})
                  .setdefault("simple_charging_dock", {}))
        scd["charging_threshold"] = dock_charging_threshold

        # FollowPath (transit controller = RPP via RotationShim).
        fp = (doc.setdefault("controller_server", {})
                 .setdefault("ros__parameters", {})
                 .setdefault("FollowPath", {}))
        fp["desired_linear_vel"] = transit_speed

        # FollowCoveragePath (FTC: coverage strip controller). Its speed
        # knob is speed_fast; mowing_speed overrides it.
        fcp = (doc.setdefault("controller_server", {})
                  .setdefault("ros__parameters", {})
                  .setdefault("FollowCoveragePath", {}))
        fcp["speed_fast"] = mowing_speed

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

        # coverage_server (mowgli_coverage / Fields2Cover v2): the F2C
        # operation_width is the swath spacing (Robot::setCovWidth). Tie
        # it directly to the blade's effective cutting width — without
        # the alignment, F2C plans swaths spaced wider than the blade
        # can cover, leaving thin un-mowed strips between every pair of
        # adjacent swaths (the previous static 0.20 m vs blade 0.18 m
        # gave a 2 cm gap × tracking error → the 54 % coverage seen in
        # 2026-05-12). One value, two consumers (map_server's stamp
        # radius is also tool_width / 2 — perfectly tiles).
        cov_params = (doc.setdefault("coverage_server", {})
                          .setdefault("ros__parameters", {}))
        cov_params["operation_width"] = tool_width
        cov_params["default_headland_width"] = headland_width
        cov_params["min_turning_radius"] = min_turning_radius
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
    # Historical: navsat_transform_node used to look up a TF from
    # base_footprint to the NavSatFix header frame. Our URDF calls
    # that frame gps_link; the ublox_dgnss driver publishes frame_id=gps.
    # navsat_transform was removed 2026-04-26, but the alias is still
    # cheap insurance for any third-party tool that walks the frame
    # tree from gps.
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
             "post_undock_blank_sec": 5.0},
        ],
    )

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_ekf_arg,
            use_lidar_arg,
            use_magnetometer_arg,
            use_scan_matching_arg,
            use_loop_closure_arg,
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
            wait_for_map_odom_tf,
            nav2_after_tf,
            empty_static_map_pub,
        ]
    )
