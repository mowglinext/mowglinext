# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
#
# Regression tests for nav2_params.yaml configuration. These guard
# against the class of bug where a goal_checker tolerance is set so
# loose that the controller silently reports SUCCEEDED on the first
# tick, the BT loops GetNextSegment forever, and the robot never moves.
# See: 2026-05-08 field incident — coverage_goal_checker.xy_goal_tolerance
# was 0.5 m, which made every <0.5 m strip "already done" the moment
# FTC started.
"""Regression tests for the nav2_params.yaml goal-checker tolerances."""
import os
import re

import pytest
import yaml


def _config_path(name: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "config", name)


def _load_yaml(name: str) -> dict:
    with open(_config_path(name), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def _deep_merge(base: dict, overlay: dict) -> dict:
    """Mirror of navigation.launch.py _deep_merge: nested dicts merge key-by-key,
    lists/scalars replace. The runtime Nav2 config is base ⊕ variant overlay, so
    the tests must validate the MERGED result, not the standalone files."""
    out = dict(base)
    for k, v in overlay.items():
        if k in out and isinstance(out[k], dict) and isinstance(v, dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def _load_params() -> dict:
    """LiDAR variant = nav2_params_base.yaml ⊕ nav2_params_lidar.yaml."""
    return _deep_merge(_load_yaml("nav2_params_base.yaml"),
                       _load_yaml("nav2_params_lidar.yaml"))


def _controller_section(params: dict) -> dict:
    return params["controller_server"]["ros__parameters"]


# The coverage goal-checker was migrated off SimpleGoalChecker/StoppedGoalChecker
# (commit 4bae0567) to PathProgressGoalChecker. The old "xy_goal_tolerance must
# be <= mower_width" and "stateful must be true" guards belonged to the
# SimpleGoalChecker era and no longer apply: PathProgressGoalChecker gates
# completion on monotonic path progress (it cannot fire on the first tick before
# the robot moves), and it has no `stateful` field. CLAUDE.md invariant: do NOT
# use Simple/Stopped here.
COVERAGE_GOAL_CHECKER_PLUGIN = "mowgli_nav2_plugins/PathProgressGoalChecker"


def _coverage_goal_checker(cfg: dict) -> dict:
    return cfg["coverage_goal_checker"]


def test_coverage_goal_checker_is_path_progress() -> None:
    """The coverage goal-checker MUST be PathProgressGoalChecker.

    SimpleGoalChecker/StoppedGoalChecker fire on the final pose (or on
    velocity stoppage) without requiring the robot to actually traverse
    the path: the F2C path's last pose can sit 25-50 cm from where FTC's
    PID converges (Dubins connector geometry), and StoppedGoalChecker
    matches FTC's mid-traversal PRE_ROTATE pivots — both complete the
    coverage action at near-zero coverage. PathProgressGoalChecker only
    fires after monotonic progress >= progress_threshold of the path
    poses (CLAUDE.md invariant).
    """
    cfg = _controller_section(_load_params())
    assert _coverage_goal_checker(cfg)["plugin"] == COVERAGE_GOAL_CHECKER_PLUGIN


def test_coverage_goal_checker_progress_threshold_is_high() -> None:
    """progress_threshold gates completion on monotonic path traversal.
    A low value would let the action succeed before the strip is mowed —
    pin it high so the robot has to actually cover the swath.
    """
    cfg = _controller_section(_load_params())
    gc = _coverage_goal_checker(cfg)
    assert "progress_threshold" in gc, (
        "PathProgressGoalChecker should pin progress_threshold explicitly "
        "rather than relying on the plugin default."
    )
    assert 0.90 <= gc["progress_threshold"] <= 1.0, (
        f"coverage_goal_checker.progress_threshold={gc['progress_threshold']} "
        "is out of the sane [0.90, 1.0] range; coverage would complete before "
        "the swath is mowed."
    )


def test_stopped_goal_checker_velocity_threshold_is_set() -> None:
    """StoppedGoalChecker without trans_stopped_velocity defaults to
    a permissive threshold; pin a value so the check is deterministic.
    """
    cfg = _controller_section(_load_params())
    assert "trans_stopped_velocity" in cfg["stopped_goal_checker"]
    assert cfg["stopped_goal_checker"]["trans_stopped_velocity"] > 0.0


def test_followpath_uses_rotation_shim() -> None:
    """The transit/dock controller wraps RPP in RotationShimController so
    big heading errors get an in-place rotate before driving. If this
    pin slips, RPP starts driving an arc immediately and the robot
    spirals away from its first carrot.
    """
    cfg = _controller_section(_load_params())
    assert (
        cfg["FollowPath"]["plugin"]
        == "nav2_rotation_shim_controller::RotationShimController"
    )


def test_followcoveragepath_uses_ftc() -> None:
    """Coverage is FTCController (restored 2026-06-19, reverting the MPPI
    experiment in a8fbe5fd).

    MPPI cut the path and swerved wide at swath U-turns/corners, and sharpening
    corners made it weave on straights (the wz_std/wz_max tug-of-war). FTC is a
    deterministic carrot-follower that tracks the continuous F2C route tightly,
    pivots in place at the path start (PRE_ROTATE), and skirts obstacles by
    lateral path deviation. The LiDAR variant must keep obstacle avoidance ON
    (there IS a local obstacle_layer to read). Guard the plugin and the obstacle
    flags against a silent revert to MPPI.
    """
    fcp = _controller_section(_load_params())["FollowCoveragePath"]
    assert fcp["plugin"] == "mowgli_nav2_plugins/FTCController"
    assert "primary_controller" not in fcp, (
        "FTC is a standalone controller — it must NOT be wrapped in RotationShim"
    )
    assert fcp["check_obstacles"] is True, (
        "LiDAR variant must check obstacles (the obstacle_layer exists)"
    )
    assert fcp["enable_obstacle_deviation"] is True


# ── 2026-05-08 field bug: launch override + per-site yaml + no-lidar variant
# all default coverage_xy_tolerance back to 0.5 m, silently re-breaking
# coverage. The nav2_params.yaml fix above is necessary but not sufficient
# — these tests guard the other three paths.

def _read_text(rel_path: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "..", rel_path), "r", encoding="utf-8") as fh:
        return fh.read()


def _base_ftc_max_goal_distance_error() -> float:
    """FTC's parking distance from nav2_params_base.yaml — the robot stops
    translating up to this far short of the final pose."""
    cfg = _load_yaml("nav2_params_base.yaml")
    fcp = cfg["controller_server"]["ros__parameters"]["FollowCoveragePath"]
    return float(fcp["max_goal_distance_error"])


def test_navigation_launch_default_coverage_tolerance_matches_ftc_park() -> None:
    """navigation.launch.py picks the runtime coverage_xy_tolerance from
    mowgli_robot.yaml, falling back to a hardcoded default. FTC zeroes
    linear.x once it leaves FOLLOWING, parking up to max_goal_distance_error
    (0.50 m) short of the goal — so the coverage goal-checker XY gate must be
    >= that, or the area never completes and re-mows. Pin the launch default
    >= FTC's parking distance (the 2026-06-25 regression set it to 0.25).
    """
    src = _read_text("launch/navigation.launch.py")
    m = re.search(r"coverage_xy_tolerance\s*=\s*([\d\.]+)", src)
    assert m, "Could not find coverage_xy_tolerance default in navigation.launch.py"
    default = float(m.group(1))
    park = _base_ftc_max_goal_distance_error()
    assert default >= park, (
        f"navigation.launch.py default coverage_xy_tolerance={default} m is tighter "
        f"than FTC max_goal_distance_error={park} m — the area-end goal would never "
        "be accepted (progress timeout → full re-mow)."
    )


def test_navigation_launch_floors_coverage_tolerance_at_ftc_park() -> None:
    """A stale per-site mowgli_robot.yaml might carry the old 0.25 m value.
    The launch script must FLOOR the injected coverage_xy_tolerance at FTC's
    max_goal_distance_error (not cap it at 0.25 — that was the regression),
    so the goal-checker gate can never be tighter than FTC can park.
    """
    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"coverage_xy_tolerance\s*<\s*ftc_park_dist", src), (
        "Expected a floor on coverage_xy_tolerance in navigation.launch.py tied to "
        "FTC's max_goal_distance_error (e.g. `if coverage_xy_tolerance < "
        "ftc_park_dist: coverage_xy_tolerance = ftc_park_dist`). Without it, a stale "
        "per-site YAML with 0.25 m re-breaks coverage completion."
    )


def test_mowgli_robot_yaml_default_coverage_tolerance_matches_ftc_park() -> None:
    """The shipped mowgli_robot.yaml is the template per-site config gets
    seeded from. If the shipped value is tighter than FTC's parking distance,
    every fresh install inherits the area-end stall.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "config", "mowgli_robot.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    tol = cfg["mowgli"]["ros__parameters"]["coverage_xy_tolerance"]
    park = _base_ftc_max_goal_distance_error()
    assert tol >= park, (
        f"mowgli_robot.yaml ships coverage_xy_tolerance={tol} m, tighter than FTC "
        f"max_goal_distance_error={park} m — the area never completes and re-mows."
    )


def test_base_ftc_clamp_admits_speed_fast() -> None:
    """FTC hard-clamps its output to ±max_cmd_vel_speed, so a max_cmd_vel_speed
    below speed_fast silently caps the mow speed. The shipped base.yaml must keep
    the clamp >= the carrot speed."""
    cfg = _load_yaml("nav2_params_base.yaml")
    fcp = cfg["controller_server"]["ros__parameters"]["FollowCoveragePath"]
    assert float(fcp["max_cmd_vel_speed"]) >= float(fcp["speed_fast"]), (
        f"base.yaml FTC max_cmd_vel_speed={fcp['max_cmd_vel_speed']} < "
        f"speed_fast={fcp['speed_fast']} — the mow speed is silently capped."
    )


def test_navigation_launch_raises_ftc_clamp_with_mowing_speed() -> None:
    """When the launch overrides speed_fast with the operator's mowing_speed it
    must also raise max_cmd_vel_speed, or a mowing_speed above 0.30 is silently
    clamped by FTC."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"max_cmd_vel_speed\W+\W*=\W*mowing_speed", src) or re.search(
        r"fcp\[.max_cmd_vel_speed.\]\s*=\s*mowing_speed", src), (
        "navigation.launch.py overrides speed_fast=mowing_speed but does not raise "
        "FollowCoveragePath.max_cmd_vel_speed — a mowing_speed > 0.30 is silently "
        "capped by FTC's output clamp."
    )


def test_base_coverage_goal_checker_xy_ge_ftc_park() -> None:
    """The static base.yaml relationship that the launch floor preserves:
    coverage_goal_checker.xy_goal_tolerance must be >= FTC's
    max_goal_distance_error so a parked FTC can satisfy the goal."""
    cfg = _load_yaml("nav2_params_base.yaml")
    gc = cfg["controller_server"]["ros__parameters"]["coverage_goal_checker"]
    park = _base_ftc_max_goal_distance_error()
    assert float(gc["xy_goal_tolerance"]) >= park, (
        f"base.yaml coverage_goal_checker.xy_goal_tolerance={gc['xy_goal_tolerance']} "
        f"< FTC max_goal_distance_error={park} — area-end stall."
    )


def test_no_lidar_variant_uses_path_progress_goal_checker() -> None:
    """nav2_params_no_lidar.yaml is the GPS-only variant — it must use the
    same PathProgressGoalChecker plugin with a high progress threshold and
    sane finite tolerances as the LiDAR variant. (The old '<= mower_width
    xy tolerance' guard was for SimpleGoalChecker, which is gone.)
    """
    cfg = _load_no_lidar_params()
    gc = cfg["controller_server"]["ros__parameters"]["coverage_goal_checker"]
    assert gc["plugin"] == COVERAGE_GOAL_CHECKER_PLUGIN
    assert 0.90 <= gc["progress_threshold"] <= 1.0
    assert 0.0 < gc["xy_goal_tolerance"] <= 1.0
    assert 0.0 < gc["yaw_goal_tolerance"] <= 3.1416


def _load_no_lidar_params() -> dict:
    """GPS-only variant = nav2_params_base.yaml ⊕ nav2_params_no_lidar.yaml."""
    return _deep_merge(_load_yaml("nav2_params_base.yaml"),
                       _load_yaml("nav2_params_no_lidar.yaml"))


def test_no_lidar_followcoveragepath_uses_ftc_without_obstacle_checks() -> None:
    """The GPS-only variant runs the SAME FTC controller as the LiDAR variant,
    but with obstacle checking/deviation OFF — there is no local obstacle_layer
    to read without LiDAR, so FTC would query an empty/static costmap. Pin both.
    """
    fcp = _controller_section(_load_no_lidar_params())["FollowCoveragePath"]
    assert fcp["plugin"] == "mowgli_nav2_plugins/FTCController"
    assert fcp["check_obstacles"] is False, (
        "no-LiDAR variant must NOT check obstacles (no obstacle_layer)"
    )
    assert fcp["enable_obstacle_deviation"] is False


def test_coverage_controller_aligned_across_variants() -> None:
    """FollowCoveragePath's tracking config (plugin + PID + speeds) MUST be
    identical between the LiDAR and no-LiDAR configs — only the obstacle flags
    (check_obstacles, enable_obstacle_deviation) legitimately differ, because the
    no-LiDAR variant has no obstacle_layer. Guards every PID/speed tuning change
    touching one file from forgetting the other (the bug class that once left
    no_lidar on a different controller).
    """
    lidar = dict(_controller_section(_load_params())["FollowCoveragePath"])
    no_lidar = dict(_controller_section(_load_no_lidar_params())["FollowCoveragePath"])
    obstacle_flags = ("check_obstacles", "enable_obstacle_deviation")
    for k in obstacle_flags:
        lidar.pop(k, None)
        no_lidar.pop(k, None)
    assert lidar == no_lidar, (
        "FollowCoveragePath tracking config differs between the merged LiDAR and "
        "no-LiDAR configs beyond the obstacle flags — the PID/speed params must "
        "come entirely from nav2_params_base.yaml so the two stay in lockstep."
    )


def _docking_controller(params: dict) -> dict:
    d = params["docking_server"]["ros__parameters"]
    return {k: v for k, v in d.items() if k.startswith("controller.")}


def test_docking_controller_aligned_across_variants() -> None:
    """The dock graceful-controller gains AND velocity limits MUST be identical
    across the LiDAR and no-LiDAR configs — docking geometry doesn't depend on
    the lidar. 2026-06-08: v_linear_min/max had silently drifted (lidar 0.10/0.15
    vs no_lidar 0.16/0.25), and 0.10 sits below the 0.15 firmware deadband so the
    final crawl was zeroed. This guard makes the two move in lockstep.
    """
    lp = _docking_controller(_load_params())
    np_ = _docking_controller(_load_no_lidar_params())
    assert lp == np_, (
        f"docking controller.* differs across variants:\n  lidar={lp}\n  no_lidar={np_}"
    )


def test_docking_v_linear_min_above_firmware_deadband() -> None:
    """hardware_bridge zeros |vx| < 0.15, so the dock crawl floor must EXCEED it
    or the final approach is zeroed and the robot stops short of the cradle.
    Also enforce v_linear_max > v_linear_min.
    """
    for loader in (_load_params, _load_no_lidar_params):
        d = loader()["docking_server"]["ros__parameters"]
        vmin = d["controller.v_linear_min"]
        vmax = d["controller.v_linear_max"]
        assert vmin >= 0.15, (
            f"docking controller.v_linear_min={vmin} is at/below the 0.15 firmware "
            "deadband — the crawl-in gets zeroed and the robot never seats."
        )
        assert vmax > vmin, f"v_linear_max={vmax} must exceed v_linear_min={vmin}"


def test_coverage_server_geometry_aligned_across_variants() -> None:
    """coverage_server geometry (swath spacing, headland, insets) MUST match
    across variants — planning doesn't depend on the lidar.
    """
    def cov(params):
        return params["coverage_server"]["ros__parameters"]
    lp = cov(_load_params())
    np_ = cov(_load_no_lidar_params())
    for k in (
        "operation_width",
        "default_headland_width",
        "num_headland_passes",
        "min_swath_length",
    ):
        assert lp.get(k) == np_.get(k), (
            f"coverage_server.{k} differs across variants: "
            f"lidar={lp.get(k)} vs no_lidar={np_.get(k)}"
        )


def test_coverage_server_has_no_turn_planning_knobs() -> None:
    """The coverage planner emits EXPLICIT segments (headland rings + straight
    serpentine swaths) and plans NO turns — the diff-drive pivots in place
    between segments (RotationShim). Re-introducing a turn-planner knob here
    (min_turning_radius / continuity / turn_type / route planner selection)
    means someone re-wired F2C path planning, which this chassis cannot track
    (Ω-loops, reverse cusps — field 2026-06-12). Guard both variants.
    """
    for loader in (_load_params, _load_no_lidar_params):
        cov = loader()["coverage_server"]["ros__parameters"]
        for forbidden in (
            "min_turning_radius",
            "default_path_continuity_type",
            "turn_type",
            "use_native_route_planning",
            "redirect_swaths",
            "linear_curv_change",
        ):
            assert forbidden not in cov, (
                f"coverage_server.{forbidden} re-introduces turn/route planning — "
                "the segment model owns turns via in-place pivots"
            )


# ── base/overlay structure guards (the refactor that killed lidar drift) ──

def test_base_has_no_costmap_layers_or_polygons() -> None:
    """nav2_params_base.yaml is the SHARED core; the lidar/no-lidar specifics
    (costmap obstacle/static layers + plugins, collision_monitor polygons +
    observation_sources) live ONLY in the overlays. If base pinned them, an
    overlay could not cleanly select the variant and drift would creep back.
    """
    base = _load_yaml("nav2_params_base.yaml")
    for cm in ("local_costmap", "global_costmap"):
        params = base[cm][cm]["ros__parameters"]
        assert "plugins" not in params, f"base {cm} must not pin plugins (overlay owns it)"
        assert "obstacle_layer" not in params and "static_layer" not in params, (
            f"base {cm} must not carry a source layer — overlays do"
        )
    cmon = base["collision_monitor"]["ros__parameters"]
    assert "polygons" not in cmon and "observation_sources" not in cmon, (
        "base collision_monitor must not pin polygons/observation_sources — overlays do"
    )


def test_overlays_select_disjoint_costmap_layers() -> None:
    """A merged costmap must reference exactly one source layer (never both
    obstacle_layer and static_layer), and its plugins list must only name layers
    that exist after the merge. Guards the base+overlay composition itself.
    """
    base = _load_yaml("nav2_params_base.yaml")
    for variant in ("nav2_params_lidar.yaml", "nav2_params_no_lidar.yaml"):
        merged = _deep_merge(base, _load_yaml(variant))
        for cm in ("local_costmap", "global_costmap"):
            params = merged[cm][cm]["ros__parameters"]
            assert not ("obstacle_layer" in params and "static_layer" in params), (
                f"{variant}: {cm} merged config has BOTH obstacle_layer and static_layer"
            )
            for layer in params["plugins"]:
                assert layer in params, (
                    f"{variant}: {cm} plugins references undefined layer '{layer}'"
                )


def test_coverage_is_ftc_transit_is_not() -> None:
    """Wiring guard (restored 2026-06-19): the COVERAGE slot (FollowCoveragePath)
    MUST be FTCController, and the TRANSIT slot (FollowPath) MUST NOT be — transit
    stays RotationShim+RPP. FTC is a standalone controller, so it is never a
    RotationShim primary_controller. Guards both variants against a silent swap of
    the coverage controller back to MPPI or onto the transit slot."""
    for loader in (_load_params, _load_no_lidar_params):
        cs = loader()["controller_server"]["ros__parameters"]
        assert cs["FollowCoveragePath"].get("plugin") == "mowgli_nav2_plugins/FTCController", (
            "FollowCoveragePath must be FTCController"
        )
        assert "primary_controller" not in cs["FollowCoveragePath"], (
            "FTC is standalone — FollowCoveragePath must not wrap a primary_controller"
        )
        fp = cs["FollowPath"]
        assert "FTCController" not in fp.get("plugin", ""), "FollowPath (transit) must not be FTC"
        assert "FTCController" not in fp.get("primary_controller", ""), (
            "FollowPath (transit) must not wrap FTC"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
