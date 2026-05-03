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
  1. robot_localization dual-EKF — ekf_odom (wheels + gyro → odom→base_footprint
     TF, continuous) and ekf_map (+ GPS via navsat_transform → map→odom
     correction). Sub-cm σ_xy under RTK-Fixed.
  2. Three helper nodes — dock_yaw_to_set_pose (seeds both EKFs with the dock
     heading on the is_charging rising edge), cog_to_imu (GPS COG as a
     continuous absolute-yaw observation with adaptive covariance), and
     mag_yaw_publisher (tilt-compensated LIS3MDL magnetometer yaw, gated on
     /ros2_ws/maps/mag_calibration.yaml existing).
  3. Nav2 bringup — full navigation stack (controllers, planners, recoveries,
     BT navigator, costmaps, lifecycle).

Architecture (REP-105):
  map → (ekf_map) → odom → (ekf_odom) → base_footprint → base_link → sensors
  ekf_map fuses /gps/pose_cov (from navsat_to_absolute_pose_node, datum +
  lever-arm corrected) as pose0. The slam_toolbox RTK fallback that
  previously fed /map_pose has been removed; fusion_graph (planned, see
  docs/HANDOFF_FUSION_GRAPH.md) will replace ekf_map_node end-to-end.
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
    _early_use_fusion_graph = "false"
    _early_use_magnetometer = "false"
    _early_use_scan_matching = "false"
    _early_use_loop_closure = "false"
    if os.path.isfile(_runtime_cfg_path):
        try:
            with open(_runtime_cfg_path, "r") as _f:
                _cfg = yaml.safe_load(_f) or {}
            _rp = _cfg.get("mowgli", {}).get("ros__parameters", {})
            # use_lidar accepts either key for back-compat with installs
            # that wrote the env-style name into mowgli_robot.yaml.
            if "use_lidar" in _rp:
                _early_use_lidar = "true" if bool(_rp["use_lidar"]) else "false"
            elif "lidar_enabled" in _rp:
                _early_use_lidar = "true" if bool(_rp["lidar_enabled"]) else "false"
            _early_use_fusion_graph = "true" if bool(
                _rp.get("use_fusion_graph", False)) else "false"
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
    # Auto-graduation rule for fusion_graph
    # ------------------------------------------------------------------
    # When the operator opts in (`use_fusion_graph: true` in
    # mowgli_robot.yaml), we still don't blindly hand over map→odom
    # to fusion_graph_node — a polluted/half-built graph could spin
    # Nav2 in a wrong frame. Instead:
    #
    #   - If a persisted graph exists on disk → fusion_graph runs as
    #     PRIMARY (owns map→odom + /odometry/filtered_map). ekf_map
    #     is skipped. Loop closure honours the operator yaml flag.
    #   - If no graph file → fusion_graph runs in OBSERVER mode
    #     (publishes to /fusion_graph/odometry, no TF). ekf_map keeps
    #     driving Nav2. Loop closure is force-OFF (nothing meaningful
    #     to close against in a bootstrapping graph). On dock arrival
    #     fusion_graph auto-saves; the next boot picks up the file
    #     and graduates to PRIMARY.
    #
    # This way the operator never has to time the use_fusion_graph
    # flip — the system bootstraps itself.
    _graph_file = "/ros2_ws/maps/fusion_graph.graph"
    _graph_exists = os.path.isfile(_graph_file)
    _early_primary_mode = (
        "true" if _early_use_fusion_graph == "true" and _graph_exists
        else "false"
    )
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
        description="When false, use nav2_params_no_lidar.yaml (no obstacle layer, collision monitor pass-through). Default read from mowgli_robot.yaml.use_lidar (or .lidar_enabled); CLI/compose override wins.",
    )

    use_fusion_graph_arg = DeclareLaunchArgument(
        "use_fusion_graph",
        default_value=_early_use_fusion_graph,
        description="Replace ekf_map_node with fusion_graph_node (GTSAM). Default read from mowgli_robot.yaml.use_fusion_graph; CLI override wins. ekf_odom_node keeps publishing odom->base_footprint either way.",
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

    primary_mode_arg = DeclareLaunchArgument(
        "primary_mode",
        default_value=_early_primary_mode,
        description="Auto-set: true when use_fusion_graph is yes AND a persisted graph exists on disk (fusion_graph drives Nav2). False otherwise (fusion_graph runs in observer mode building a graph for next session, ekf_map_node drives Nav2).",
    )

    # ------------------------------------------------------------------
    # Resolved substitutions
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ekf = LaunchConfiguration("use_ekf")
    use_lidar = LaunchConfiguration("use_lidar")
    use_fusion_graph = LaunchConfiguration("use_fusion_graph")
    use_magnetometer = LaunchConfiguration("use_magnetometer")
    use_scan_matching = LaunchConfiguration("use_scan_matching")
    use_loop_closure = LaunchConfiguration("use_loop_closure")
    primary_mode = LaunchConfiguration("primary_mode")

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
    #   undock_speed     → BackUp backup_speed attribute in main_tree.xml
    #                      (hardcoded in the XML so not injected here, but
    #                      the comment keeps the three values together
    #                      for discoverability).
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
    coverage_xy_tolerance = 0.5
    progress_timeout_sec = 300.0
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
        progress_timeout_sec = float(
            rt_rp.get("progress_timeout_sec", progress_timeout_sec))
        dock_pose_yaw_sigma_rad = float(rt_rp.get(
            "dock_pose_yaw_sigma_rad", dock_pose_yaw_sigma_rad))
        enable_mag_cal = bool(rt_rp.get("enable_mag_cal", enable_mag_cal))
        mag_cal_path = str(rt_rp.get("mag_calibration_path", mag_cal_path))
        declination_deg = float(rt_rp.get("declination_deg", declination_deg))
        min_horizontal_uT = float(rt_rp.get("min_horizontal_uT", min_horizontal_uT))
        mag_yaw_variance = float(rt_rp.get("mag_yaw_variance", mag_yaw_variance))

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
        (doc.setdefault("docking_server", {})
            .setdefault("ros__parameters", {})
            .setdefault("home_dock", {}))["pose"] = [
                dock_pose_x, dock_pose_y, dock_pose_yaw]

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
    # Alternative localization backend: robot_localization
    # ------------------------------------------------------------------
    # Three nodes. Active only when localization_backend == "robot_localization".
    # ekf_odom_node         : wheels + IMU gyro → odom → base_footprint TF
    # navsat_transform_node : /gps/fix + /odometry/filtered → /odometry/gps
    # ekf_map_node          : wheels + IMU + /odometry/gps → map → odom TF
    robot_localization_params = os.path.join(
        bringup_dir, "config", "robot_localization.yaml"
    )

    # navsat_transform_node looks up a TF from base_footprint to the frame
    # named in the NavSatFix header. Our URDF calls that frame gps_link,
    # but the ublox_dgnss driver publishes frame_id=gps. Alias gps_link →
    # gps as a static identity so the lookup finds a chain.
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

    ekf_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom_node",
        output="screen",
        parameters=[
            robot_localization_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    # Datum triple [lat, lon, yaw]. yaw stays 0 — our IMU has no
    # magnetometer so yaw can't be anchored to true north at boot;
    # robot yaw will align to GPS track after the first straight
    # motion, and to dock_pose_yaw at dock reset. Datum lat/lon must
    # match what was used to save areas / dock pose, otherwise saved
    # coordinates shift.
    # navsat_transform_node removed 2026-04-26 — its only output (/odometry/gps)
    # had no subscribers in the active fusion path. /gps/pose_cov from the
    # custom navsat_to_absolute_pose_node (which applies the lever-arm
    # correction with map yaw) is what ekf_map actually fuses.
    # /gps/filtered (foxglove visualisation) is replaced by /gps/absolute_pose.

    # ekf_map_node — runs whenever fusion_graph is NOT in primary mode.
    # That covers: use_fusion_graph=false (operator opted out) AND
    # use_fusion_graph=true but no persisted graph yet (bootstrap
    # session — fusion_graph builds the graph in observer mode while
    # ekf_map keeps driving Nav2).
    ekf_map_node = Node(
        condition=UnlessCondition(primary_mode),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_map_node",
        output="screen",
        parameters=[
            robot_localization_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered_map"),
            # robot_localization defaults to a global /set_pose topic shared
            # by both EKFs. Remap ekf_map's subscription to a node-unique
            # name so seeding ekf_map does not also reset ekf_odom.
            ("set_pose", "/ekf_map_node/set_pose"),
        ],
    )

    # fusion_graph_node — GTSAM iSAM2 factor-graph localizer (planned
    # replacement for ekf_map_node). Mutually exclusive with the EKF
    # above. Reads datum + lever-arm from mowgli_robot.yaml inside the
    # fusion_graph launch include.
    fusion_graph_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fusion_graph"),
                "launch", "fusion_graph.launch.py",
            )
        ),
        # Run whenever the operator wants fusion_graph at all — the
        # primary_mode flag inside the include decides whether it
        # owns map→odom or just observes.
        condition=IfCondition(use_fusion_graph),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_magnetometer": use_magnetometer,
            "use_scan_matching": use_scan_matching,
            "use_loop_closure": use_loop_closure,
            "primary_mode": primary_mode,
        }.items(),
    )

    # Seeds ekf_map with the dock heading on rising edges of is_charging.
    # Fires once per docking event plus once at boot if the robot is
    # already docked.
    dock_yaw_to_set_pose = Node(
        package="mowgli_localization",
        executable="dock_yaw_to_set_pose",
        name="dock_yaw_to_set_pose",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "dock_pose_yaw": dock_pose_yaw,
             "dock_pose_yaw_sigma_rad": dock_pose_yaw_sigma_rad},
        ],
    )

    # Publishes GPS course-over-ground as a synthetic sensor_msgs/Imu on
    # /imu/cog_heading so ekf_map_node can fuse it as an absolute-yaw
    # observation. Once the session is seeded and the robot is driving
    # forward faster than min_speed_ms with RTK-Fixed, this node corrects
    # gyro drift every /gps/absolute_pose sample.
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
             "mag_calibration_path": mag_cal_path},
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
    costmap_scan_filter = Node(
        package="mowgli_localization",
        executable="costmap_scan_filter_node",
        name="costmap_scan_filter",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time,
             "input_topic": "/scan",
             "output_topic": "/scan_costmap",
             "status_topic": "/hardware_bridge/status",
             "dock_blank_range": 0.70,
             "post_undock_blank_sec": 5.0},
        ],
    )

    # ekf_odom_node subscribes to /wheel_odom directly (see
    # robot_localization.yaml). ekf_map_node fuses /gps/pose_cov directly
    # as pose0 (published by navsat_to_absolute_pose_node).

    # ------------------------------------------------------------------
    # LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            use_sim_time_arg,
            use_ekf_arg,
            use_lidar_arg,
            use_fusion_graph_arg,
            use_magnetometer_arg,
            use_scan_matching_arg,
            use_loop_closure_arg,
            primary_mode_arg,
            # robot_localization dual EKF + helpers
            static_gps_link_alias,
            ekf_odom_node,
            ekf_map_node,
            fusion_graph_launch,
            dock_yaw_to_set_pose,
            cog_to_imu,
            mag_yaw_publisher,
            costmap_scan_filter,
            wait_for_map_odom_tf,
            nav2_after_tf,
            empty_static_map_pub,
        ]
    )
