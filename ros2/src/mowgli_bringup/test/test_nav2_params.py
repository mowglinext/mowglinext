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


def test_followcoveragepath_uses_rotation_shim_mppi() -> None:
    """Coverage is RotationShim wrapping MPPI on the CONTINUOUS full path.

    RotationShim does ONE crisp in-place pivot to the path-start heading after
    transit, then hands the whole CC-Dubins route to MPPI (it doesn't re-engage
    at the U-turn arcs). CRITICAL: closed_loop MUST be false — closed_loop=true
    reads ~0 odom angular velocity during the sub-deadband pivot on this chassis
    and pins the ramp at one accel step (0.25 rad/s), so the pivot never clears
    the firmware deadband and every coverage goal stalled. Guard both the wrap
    and closed_loop=false against regression.
    """
    fcp = _controller_section(_load_params())["FollowCoveragePath"]
    assert fcp["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert fcp["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert fcp["closed_loop"] is False, (
        "closed_loop=true stalls the RotationShim pivot on this deadband chassis"
    )


# ── 2026-05-08 field bug: launch override + per-site yaml + no-lidar variant
# all default coverage_xy_tolerance back to 0.5 m, silently re-breaking
# coverage. The nav2_params.yaml fix above is necessary but not sufficient
# — these tests guard the other three paths.

def _read_text(rel_path: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "..", rel_path), "r", encoding="utf-8") as fh:
        return fh.read()


def test_navigation_launch_default_coverage_tolerance_is_tight() -> None:
    """navigation.launch.py picks the runtime coverage_xy_tolerance
    from mowgli_robot.yaml, falling back to a hardcoded default. A
    stale or missing mowgli_robot.yaml field then determines whether
    mowing works. Pin the launch default <= mower_width.
    """
    src = _read_text("launch/navigation.launch.py")
    m = re.search(r"coverage_xy_tolerance\s*=\s*([\d\.]+)", src)
    assert m, "Could not find coverage_xy_tolerance default in navigation.launch.py"
    default = float(m.group(1))
    # Per-swath DISCONTINUOUS model: the coverage slot uses PathProgressGoalChecker
    # (fires only after >= 95% path progress), so a loose xy can't latch early —
    # the floor was a SimpleGoalChecker-era concern. The ceiling now just guards
    # against an absurd value; 0.25 m is the per-swath ceiling (each swath-end
    # goal must SUCCEED so the next swath dispatches).
    assert default <= 0.25, (
        f"navigation.launch.py default coverage_xy_tolerance={default} m exceeds "
        "the 0.25 m per-swath ceiling."
    )


def test_navigation_launch_clips_runaway_coverage_tolerance() -> None:
    """A stale per-site mowgli_robot.yaml might still carry the legacy
    0.5 m value. The launch script must clip — anything above ~0.15 m
    silently regresses to the field-broken state.
    """
    src = _read_text("launch/navigation.launch.py")
    # Look for an explicit clip in the launch script's coverage_xy_tolerance handling.
    assert re.search(r"coverage_xy_tolerance\s*>\s*0\.\d+", src), (
        "Expected a clip on coverage_xy_tolerance in navigation.launch.py "
        "(e.g. `if coverage_xy_tolerance > 0.15: coverage_xy_tolerance = 0.15`). "
        "Without it, a stale per-site YAML with 0.5 m re-breaks coverage."
    )


def test_mowgli_robot_yaml_default_coverage_tolerance_is_tight() -> None:
    """The shipped mowgli_robot.yaml is the template per-site config gets
    seeded from. If the shipped value is loose, every fresh install
    inherits the bug.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "config", "mowgli_robot.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    tol = cfg["mowgli"]["ros__parameters"]["coverage_xy_tolerance"]
    assert tol <= 0.25, (
        f"mowgli_robot.yaml ships coverage_xy_tolerance={tol} m exceeds the 0.25 m "
        "per-swath ceiling (progress-gated goal checker makes a loose xy safe)."
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


def test_no_lidar_followcoveragepath_uses_rotation_shim_mppi() -> None:
    """The GPS-only variant must run the SAME coverage controller as the LiDAR
    variant (RotationShim+MPPI, closed_loop=false). Pins it so the two can't
    diverge again.
    """
    fcp = _controller_section(_load_no_lidar_params())["FollowCoveragePath"]
    assert fcp["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert fcp["primary_controller"] == "nav2_mppi_controller::MPPIController"
    assert fcp["closed_loop"] is False


def test_coverage_controller_aligned_across_variants() -> None:
    """FollowCoveragePath MUST be value-for-value identical between the LiDAR
    and no-LiDAR configs (CLAUDE.md single-source rule). MPPI's CostCritic is
    harmless on the no-LiDAR empty costmap, so there is no reason to diverge —
    and a divergence is exactly the bug class that left no_lidar on FTC. Guards
    every tuning change touching one file from forgetting the other.
    """
    lidar = _controller_section(_load_params())["FollowCoveragePath"]
    no_lidar = _controller_section(_load_no_lidar_params())["FollowCoveragePath"]
    assert lidar == no_lidar, (
        "FollowCoveragePath differs between the merged LiDAR and no-LiDAR "
        "configs — it must come entirely from nav2_params_base.yaml so the two "
        "stay in lockstep (the transit FollowPath block may differ for "
        "use_collision_detection, but the coverage controller must not)."
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


def test_no_ftc_controller_anywhere() -> None:
    """FTC is retired as a CONTROLLER (the user dropped it): no controller slot —
    directly or as a RotationShim primary_controller — may be FTCController, and
    no leftover disabled `_FollowCoveragePath_FTC*` block may linger in the merged
    config. (Historical *comments* mentioning FTC are fine — they explain why it
    was abandoned; this guards the live wiring, in both variants.)"""
    for loader in (_load_params, _load_no_lidar_params):
        cs = loader()["controller_server"]["ros__parameters"]
        assert not any(k.startswith("_FollowCoveragePath_FTC") for k in cs), (
            "leftover disabled FTC block in controller_server"
        )
        for slot in cs["controller_plugins"]:
            plug = cs[slot].get("plugin", "")
            prim = cs[slot].get("primary_controller", "")
            assert "FTCController" not in plug, f"{slot}.plugin is FTCController"
            assert "FTCController" not in prim, f"{slot}.primary_controller is FTCController"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
