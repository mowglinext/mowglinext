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
import math
import os
import re
import sys

import pytest
import yaml

# Shared recursive dict-merge (deep-copies throughout, so the merged result
# shares no mutable nested dict with either input). Imported rather than
# duplicated so these tests validate the exact same merge navigation.launch.py
# actually performs, instead of a parallel (and previously shallow-copy,
# aliasing-prone) reimplementation.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "launch"))
from robot_config_util import (  # noqa: E402
    GLOBAL_COSTMAP_RESOLUTION_M,
    chassis_circumscribed_radius as _chassis_circumscribed_radius,
    chassis_footprint as _chassis_footprint,
    deep_merge as _deep_merge,
    global_inflation_radius as _global_inflation_radius,
)


def _config_path(name: str) -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(here, "..", "config", name)


def _load_yaml(name: str) -> dict:
    with open(_config_path(name), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


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


def test_transit_goal_checker_ignores_final_heading() -> None:
    """Coverage transits (navigate_to_pose_transit.xml) must not demand a final
    heading: RPP cannot pivot at the end of a path and the 1 Hz replan loop made
    a 0.40 m transit spin for 164 s on 2026-09-12. Docking keeps
    stopped_goal_checker (yaw 0.10) untouched."""
    for params in (_load_params(), _load_no_lidar_params()):
        cfg = _controller_section(params)
        assert "transit_goal_checker" in cfg["goal_checker_plugins"]
        gc = cfg["transit_goal_checker"]
        assert gc["plugin"] == "nav2_controller::SimpleGoalChecker"
        assert gc["xy_goal_tolerance"] <= 0.5
        assert gc["yaw_goal_tolerance"] >= 3.1
        assert cfg["stopped_goal_checker"]["yaw_goal_tolerance"] <= 0.2


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
    # The floor: a comparison against ftc_park_dist that raises the injected
    # tolerance up to it. The injected value is held in a local (cov_xy_tol) —
    # NOT the enclosing coverage_xy_tolerance, which must not be rebound inside
    # the nested _inject fn or it becomes function-local and the earlier read
    # raises UnboundLocalError (the 2026-06-26 launch crash). Match the floor
    # by its effect (`< ftc_park_dist` … `= ftc_park_dist`), name-agnostic.
    assert re.search(r"<\s*ftc_park_dist", src) and re.search(r"=\s*ftc_park_dist", src), (
        "Expected a floor on the injected coverage tolerance in navigation.launch.py "
        "tied to FTC's max_goal_distance_error (e.g. `if cov_xy_tol < ftc_park_dist: "
        "cov_xy_tol = ftc_park_dist`). Without it, a stale per-site YAML with 0.25 m "
        "re-breaks coverage completion."
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


# ---------------------------------------------------------------------------
# Obstacle-avoidance knob injection (GUI: Settings → Obstacles)
# ---------------------------------------------------------------------------


def _template_robot_params() -> dict:
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "config", "mowgli_robot.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)["mowgli"]["ros__parameters"]


def test_navigation_launch_injects_ftc_max_lateral_deviation() -> None:
    """max_obstacle_avoidance_distance must drive FTC's max_lateral_deviation.
    Before 2026-07 it only reached map_server.bypass_max_length — the GUI
    'Max Obstacle Detour' slider silently did nothing for coverage-time
    skirting (FTC stayed pinned at the static base.yaml 1.5 m)."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(
        r"fcp\[.max_lateral_deviation.\]\s*=.*max_obstacle_avoidance_distance",
        src, re.DOTALL), (
        "navigation.launch.py must inject max_obstacle_avoidance_distance into "
        "FollowCoveragePath.max_lateral_deviation — otherwise the GUI knob is an "
        "orphan and FTC keeps the static base.yaml deviation limit."
    )


def test_navigation_launch_injects_ftc_clearance_margin() -> None:
    """obstacle_clearance_margin must reach FollowCoveragePath.

    This is the ONLY operator knob that widens the pass-by gap when skirting:
    obstacle_inflation_radius cannot do it (FTC's deviation checks threshold at
    cost 253, whose band is sized by the footprint inscribed radius, not by
    inflation_radius — raising inflation only grows the 1..252 decay region the
    checks ignore). If this injection is dropped the GUI slider goes inert and
    the robot silently returns to grazing past obstacles."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(
        r"fcp\[.obstacle_clearance_margin.\]\s*=.*obstacle_clearance_margin",
        src, re.DOTALL), (
        "navigation.launch.py must inject obstacle_clearance_margin into "
        "FollowCoveragePath.obstacle_clearance_margin."
    )


def test_navigation_launch_injects_ftc_wait_timeout() -> None:
    """obstacle_wait_timeout_s must reach FollowCoveragePath.

    It shipped in the GUI param catalog with full i18n but was never injected,
    so the operator-facing value was inert and the static base.yaml 5.0 always
    won. Guard against that regressing."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(
        r"fcp\[.obstacle_wait_timeout_s.\]\s*=.*obstacle_wait_timeout_s",
        src, re.DOTALL), (
        "navigation.launch.py must inject obstacle_wait_timeout_s into "
        "FollowCoveragePath.obstacle_wait_timeout_s."
    )


def test_clearance_margin_is_distinct_from_body_half_width() -> None:
    """The clearance margin must NOT be folded into obstacle_body_half_width.

    That parameter does double duty as detection reach; widening it to buy
    pass-by margin re-opens the over-reach that caused ~15x
    'deviation > max -> hold 5 s' stalls per mow (2026-06-25 field note in
    nav2_params_base.yaml). Both keys must exist independently."""
    for loader in (_load_params, _load_no_lidar_params):
        fcp = _controller_section(loader())["FollowCoveragePath"]
        assert "obstacle_body_half_width" in fcp
        assert "obstacle_clearance_margin" in fcp
        assert fcp["obstacle_body_half_width"] == 0.12, (
            "detection half-width must stay at the tuned 0.12 — raise "
            "obstacle_clearance_margin for pass-by margin instead"
        )


def test_navigation_launch_injects_local_inflation_with_floor() -> None:
    """obstacle_inflation_radius reaches ONLY the local costmap, floored at the
    chassis circumscribed radius. Below that radius the inflation layer loses
    its inscribed band, so footprint-cost semantics degrade and FTC's
    threshold-253 deviation detector has nothing to read.

    The floor is DERIVED from the live chassis parameters, never a literal.
    Chassis dimensions are operator-editable in the GUI, so a literal goes
    stale the moment someone edits them — which is exactly what happened: the
    old hardcoded 0.58 quoted a ~0.572 m radius taken from chassis_length 0.54
    and was already below the real radius at the then-current chassis_width
    0.40, let alone the measured 0.45."""
    src = _read_text("launch/navigation.launch.py")
    m = re.search(
        r"lc_infl\[.inflation_radius.\]\s*=\s*min\(\s*([\d.]+),\s*"
        r"max\(\s*infl_floor,\s*obstacle_inflation_radius", src)
    assert m, (
        "navigation.launch.py must clamp-inject obstacle_inflation_radius into the "
        "local costmap inflation_layer (lc_infl['inflation_radius'] = min(cap, "
        "max(infl_floor, obstacle_inflation_radius)))."
    )
    assert float(m.group(1)) == 1.50, (
        f"inflation cap {m.group(1)} — the upper clamp is pinned at 1.50 m."
    )
    assert re.search(r"infl_floor\s*=\s*chassis_circumscribed_radius\(", src), (
        "the inflation floor must be DERIVED via "
        "robot_config_util.chassis_circumscribed_radius(), not hardcoded — a "
        "literal goes stale when the operator edits the chassis in the GUI."
    )
    # And that derivation, on the shipped template, must still clear the old
    # hand-tuned 0.58 floor. If a future chassis default makes it smaller, the
    # inflation layer stops covering the footprint and this must be revisited
    # deliberately rather than silently.
    template = _load_yaml("mowgli_robot.yaml")["mowgli"]["ros__parameters"]
    derived = _chassis_circumscribed_radius(template)
    assert derived >= 0.58, (
        f"template chassis derives an inflation floor of {derived:.3f} m, below "
        "the 0.58 m the local costmap was tuned against — footprint-cost "
        "semantics degrade below the circumscribed radius."
    )
    # Exactly two writers: the local chain above and the derived global radius
    # (test_global_inflation_gives_the_transit_planner_berth_without_blocking).
    writers = re.findall(r"(\w+)\[.inflation_radius.\]\s*=", src)
    assert sorted(writers) == ["gc_infl", "lc_infl"], (
        f"inflation_radius writers in navigation.launch.py: {writers} — only the "
        "local chain (lc_infl) and the derived global radius (gc_infl) may write it."
    )


def test_global_inflation_gives_the_transit_planner_berth_without_blocking() -> None:
    """Field 2026-09-21: with a 0.20 m global inflation the cost gradient around
    a LiDAR obstacle was 3 cm wide, SmacPlanner2D planned the transit 0.2 m
    from it, the body (0.275 m half-width) overlapped it and RPP refused the
    path 442 times over 97 s. The global radius now reaches the body's full
    reach, derived from the chassis, so cost_travel_multiplier has a gradient
    to act on.

    What must NOT change is the band Smac refuses: cost >= INSCRIBED, i.e. the
    footprint's inscribed radius. Widening THAT is what made robots standing on
    their own coverage line "start occupied" — so no custom_inscribed_radius on
    the global costmap, and the keepout mask stays un-inflated (keepout_filter
    after inflation_layer, pinned elsewhere)."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"gc_infl\[.inflation_radius.\]\s*=\s*global_inflation_radius\(rp\)", src), (
        "the GLOBAL inflation radius must be injected from "
        "robot_config_util.global_inflation_radius(rp) — derived, never a literal."
    )
    assert "gc_infl[\"custom_inscribed_radius\"]" not in src and \
        "gc_infl['custom_inscribed_radius']" not in src, (
        "the global costmap must keep the footprint's inscribed band: widening "
        "what Smac refuses re-creates 'start occupied' on the coverage line."
    )
    template = _load_yaml("mowgli_robot.yaml")["mowgli"]["ros__parameters"]
    circumscribed = _chassis_circumscribed_radius(template)
    derived = _global_inflation_radius(template)
    assert derived > circumscribed, (
        f"global inflation {derived:.3f} m must reach past the body "
        f"({circumscribed:.3f} m) or the gradient ends inside the footprint."
    )
    base = _load_yaml("nav2_params_base.yaml")
    gi = base["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]
    assert abs(float(gi["inflation_radius"]) - round(derived, 2)) <= 0.01, (
        f"base.yaml global inflation_radius {gi['inflation_radius']} should document "
        f"the shipped derived value {derived:.3f} (the launch overwrites it)."
    )
    scaling = float(gi["cost_scaling_factor"])
    front, rear, half_width = _chassis_footprint(template)
    inscribed = min(front, -rear, half_width)
    # Nav2 InflationLayer: cost = 252 * exp(-scaling * (d - inscribed)).
    cost_at_body = 252.0 * math.exp(-scaling * (circumscribed - inscribed))
    assert cost_at_body >= 5.0, (
        f"global cost_scaling_factor {scaling}: the cost left at the body's reach "
        f"({cost_at_body:.1f}) is too small for cost_travel_multiplier to buy berth — "
        "Smac hugs obstacles again (10 let the replayed transit dip to 0.29 m)."
    )

    # obstacle_tracker_node draws obstacle proposals by clustering THIS costmap
    # at an occupancy threshold, so the gradient's reach at that cost is the
    # size of every tracked obstacle. Keep it within one global cell of what it
    # was (0.20 m, the old inflation_radius, which capped it).
    tracker = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "mowgli_map", "src",
        "obstacle_tracker_node.cpp")
    with open(tracker) as fh:
        m = re.search(r"OBSTACLE_COST\s*=\s*(\d+)", fh.read())
    assert m, "obstacle_tracker_node.cpp no longer defines OBSTACLE_COST — re-check this bound"
    occupancy = int(m.group(1))
    # nav2_costmap_2d publisher: occupancy = 1 + 97 * (cost - 1) / 251 (integer).
    cost = next(c for c in range(1, 253) if 1 + (97 * (c - 1)) // 251 >= occupancy)
    tracker_reach = min(derived, inscribed + math.log(252.0 / cost) / scaling)
    assert tracker_reach <= 0.20 + GLOBAL_COSTMAP_RESOLUTION_M + 1e-9, (
        f"tracked obstacles would reach {tracker_reach:.3f} m past the LiDAR mark "
        f"(cost >= {cost}), more than one cell beyond the 0.20 m before 2026-09-21 — "
        "a lower cost_scaling_factor inflates every obstacle proposal."
    )


def test_navigation_launch_guards_polygon_slow_write() -> None:
    """PolygonSlow only exists in the LiDAR overlay's collision_monitor. The
    slowdown_ratio injection must be guarded so the no-lidar merge (pass-
    through monitor) is not given a phantom polygon block."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"if\s+.PolygonSlow.\s+in\s+", src), (
        "navigation.launch.py must guard the PolygonSlow.slowdown_ratio write "
        "with a presence check — the no-lidar variant has no PolygonSlow."
    )


def test_navigation_launch_injects_coverage_obstacle_margin() -> None:
    """obstacle_margin must reach coverage_server (F2C hole buffering). The
    keepout band is a DIFFERENT, derived number (keepout_obstacle_margin),
    injected by full_system.launch.py into map_server."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(
        r"cov_params\[.obstacle_margin.\]\s*=\s*planned_margin", src), (
        "navigation.launch.py must inject the FLOORED obstacle_margin into "
        "coverage_server."
    )
    fs_src = _read_text("launch/full_system.launch.py")
    assert re.search(
        r'\{"keepout_obstacle_margin":\s*keepout_obstacle_margin_m\}', fs_src), (
        "full_system.launch.py must inject the DERIVED keepout band into "
        "map_server."
    )
    assert re.search(
        r"keepout_obstacle_margin_m\s*=\s*keepout_obstacle_margin\(robot_params\)",
        fs_src), (
        "the keepout band must come from robot_config_util."
        "keepout_obstacle_margin(), not a literal and not obstacle_margin."
    )
    assert not re.search(r'\{"obstacle_margin":', fs_src), (
        "map_server must NOT receive coverage's obstacle_margin: one number for "
        "a centreline planner and a mask read by a point-check planner counts "
        "the body twice (START_OCCUPIED on the coverage line, 2026-09-16/17)."
    )


def test_obstacle_template_defaults_match_static_yaml() -> None:
    """The template's obstacle knobs must default to the static yaml values so
    a sparse installed file (no overrides) is behaviour-preserving."""
    rp = _template_robot_params()
    base = _load_yaml("nav2_params_base.yaml")
    lidar = _load_yaml("nav2_params_lidar.yaml")
    local_infl = (base["local_costmap"]["local_costmap"]["ros__parameters"]
                  ["inflation_layer"]["inflation_radius"])
    assert float(rp["obstacle_inflation_radius"]) == float(local_infl), (
        f"template obstacle_inflation_radius={rp['obstacle_inflation_radius']} != "
        f"base.yaml local inflation_radius={local_infl} — a fresh install would "
        "silently change avoidance behaviour."
    )
    slow = (lidar["collision_monitor"]["ros__parameters"]["PolygonSlow"]
            ["slowdown_ratio"])
    assert float(rp["obstacle_slowdown_ratio"]) == float(slow), (
        f"template obstacle_slowdown_ratio={rp['obstacle_slowdown_ratio']} != "
        f"lidar overlay PolygonSlow.slowdown_ratio={slow}."
    )
    # obstacle_margin is DERIVED, not pinned to a literal: it must be at least
    # the body half-width, which follows chassis_width (the full derived floor
    # is checked in test_boundary_inset_and_obstacle_margin_are_separate_knobs). It was a literal 0.2
    # (2026-07-23) until 2026-09-16, when the chassis had grown to 0.45 m and
    # the literal no longer covered the body. The exact-equality check lives in
    # test_boundary_inset_and_obstacle_margin_are_separate_knobs.
    from robot_config_util import chassis_half_width as _chassis_half_width

    assert float(rp["obstacle_margin"]) >= _chassis_half_width(rp), (
        f"template obstacle_margin={rp['obstacle_margin']} is inside the body "
        f"half-width {_chassis_half_width(rp):.3f} — the launch floor would "
        "silently override it and the yaml would lie to whoever reads it."
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


def test_docking_max_retries_and_battery_status_present() -> None:
    """docking_server.max_retries and simple_charging_dock.use_battery_status are
    INJECTED at launch from mowgli_robot.yaml (dock_max_retries /
    dock_use_charger_detection, issue #195). navigation.launch.py writes them
    into the MERGED doc, so both keys must exist in both merged variants — and,
    per Invariant 8, they must live in the shared BASE with NEITHER overlay
    redefining them (an overlay copy would win the deep-merge for one variant
    only, silently splitting the two configs' docking behaviour).
    """
    for name, params in (("lidar", _load_params()), ("no_lidar", _load_no_lidar_params())):
        ds = params["docking_server"]["ros__parameters"]
        assert "max_retries" in ds, f"{name}: docking_server.max_retries missing"
        assert isinstance(ds["max_retries"], int) and ds["max_retries"] >= 0, (
            f"{name}: docking_server.max_retries={ds['max_retries']!r} must be a non-negative int"
        )
        scd = ds["simple_charging_dock"]
        assert "use_battery_status" in scd, (
            f"{name}: simple_charging_dock.use_battery_status missing"
        )
        assert isinstance(scd["use_battery_status"], bool), (
            f"{name}: simple_charging_dock.use_battery_status must be a bool"
        )

    base = _load_yaml("nav2_params_base.yaml")["docking_server"]["ros__parameters"]
    assert "max_retries" in base and "use_battery_status" in base["simple_charging_dock"], (
        "both keys must be defined in the SHARED base (nav2_params_base.yaml)"
    )
    for overlay in ("nav2_params_lidar.yaml", "nav2_params_no_lidar.yaml"):
        ov = (_load_yaml(overlay).get("docking_server") or {}).get("ros__parameters") or {}
        assert "max_retries" not in ov, (
            f"{overlay} redefines docking_server.max_retries — it belongs in the base only"
        )
        assert "use_battery_status" not in (ov.get("simple_charging_dock") or {}), (
            f"{overlay} redefines simple_charging_dock.use_battery_status — base only"
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


def test_coverage_server_num_headland_passes_sentinel_range() -> None:
    """num_headland_passes is a THREE-WAY sentinel (issue #429):
    <0 = NONE (no perimeter rings — the swaths mow to the boundary),
    0 = AUTO (ceil(default_headland_width / operation_width), min 1),
    >0 = exactly that many rings.

    The value must therefore stay an int in [-1, 5]: nothing clamps it on the
    way through navigation.launch.py, so a stray large/negative value would
    silently reshape every plan.
    """
    for name, params in (("lidar", _load_params()), ("no_lidar", _load_no_lidar_params())):
        n = params["coverage_server"]["ros__parameters"]["num_headland_passes"]
        assert isinstance(n, int) and not isinstance(n, bool), (
            f"{name}: num_headland_passes must be an int, got {type(n).__name__}"
        )
        assert -1 <= n <= 5, f"{name}: num_headland_passes={n} outside the [-1, 5] sentinel range"


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


def test_controller_server_odom_topic_is_not_default() -> None:
    """CLAUDE.md 'What NOT to Do': controller_server.odom_topic must never be
    left unset — Nav2 defaults it to "odom", which NOTHING publishes on this
    robot, so MPPI/RPP/FTC get ZERO velocity feedback (MPPI circles on tight
    coverage; RPP's velocity-scaled lookahead collapses -> transit weaving).
    CLAUDE.md previously claimed this file already guarded odom_topic; it
    didn't (grep=0) — this is that guard, for both variants.
    """
    for loader in (_load_params, _load_no_lidar_params):
        odom_topic = _controller_section(loader()).get("odom_topic")
        assert odom_topic, (
            "controller_server.odom_topic is unset — Nav2 defaults to 'odom', "
            "which nothing publishes on this robot."
        )
        assert odom_topic != "odom", (
            f"controller_server.odom_topic={odom_topic!r} — the unpublished Nav2 "
            "default would leave every controller with zero velocity feedback."
        )


def test_docking_server_odom_topic_is_not_default() -> None:
    """Same guard as controller_server, for the docking graceful controller's
    nested `controller.odom_topic` (CLAUDE.md invariant 16 — the dock
    controller was missing velocity feedback entirely until this was added,
    causing a left-right hunt at the cradle).
    """
    for loader in (_load_params, _load_no_lidar_params):
        odom_topic = loader()["docking_server"]["ros__parameters"].get("controller.odom_topic")
        assert odom_topic, (
            "docking_server controller.odom_topic is unset — the dock controller "
            "runs velocity-blind and can't damp its own near-dock heading corrections."
        )
        assert odom_topic != "odom", (
            f"docking_server controller.odom_topic={odom_topic!r} — the unpublished "
            "Nav2 default 'odom' leaves the dock controller velocity-blind."
        )


def test_ftc_stall_trio_present_in_both_variants() -> None:
    """FTC's anti-wheelspin stall easing (ftc_stall.hpp / StallDecision) needs
    all three of stall_speed_ratio, stall_grace_s, stall_crawl_speed — a
    missing key silently falls back to the C++ struct default rather than
    failing loudly, which could re-open the wheelspin/turf-digging behaviour
    the feature exists to prevent. Guard both variants (both merge the same
    FollowCoveragePath from nav2_params_base.yaml today, but that's the
    invariant this test pins, not an assumption).
    """
    for loader in (_load_params, _load_no_lidar_params):
        fcp = _controller_section(loader())["FollowCoveragePath"]
        for key in ("stall_speed_ratio", "stall_grace_s", "stall_crawl_speed"):
            assert key in fcp, f"FollowCoveragePath.{key} missing from merged config"
        assert fcp["stall_speed_ratio"] > 0.0, (
            "stall_speed_ratio <= 0 disables anti-wheelspin easing entirely"
        )
        assert fcp["stall_grace_s"] > 0.0
        assert 0.0 < fcp["stall_crawl_speed"] < fcp["speed_slow"], (
            f"stall_crawl_speed={fcp['stall_crawl_speed']} must be a genuine crawl — "
            f"below speed_slow={fcp['speed_slow']} — or easing to it doesn't slow anything down"
        )


def test_ftc_turn_fallback_is_configured_and_bounded() -> None:
    """FTC's turn fallback (ftc_turn_fallback.hpp) improvises a turn the lattice
    cannot drive — reverse, pivot, straight, pivot — instead of aborting. It
    drives the bladed robot off its plan, so every knob that bounds it must be
    set explicitly (a missing key silently runs the C++ default) and stay inside
    the ranges FTCController accepts (onParameterChange rejects, declare clamps):
    the reverse within the reverse-escape's order of magnitude, a rejoin search
    of a few metres, a turn threshold that leaves straights to the WEDGED path,
    and a finite deadline. The reverse must also still be governed by
    obstacle_reverse_enabled (FTC zeroes it when that is false), so that switch
    has to be present alongside.
    """
    for loader in (_load_params, _load_no_lidar_params):
        fcp = _controller_section(loader())["FollowCoveragePath"]
        for key in ("turn_fallback_enabled", "turn_fallback_max_reverse_m",
                    "turn_fallback_max_rejoin_arc_m", "turn_fallback_min_turn_deg",
                    "turn_fallback_timeout_s", "obstacle_reverse_enabled"):
            assert key in fcp, f"FollowCoveragePath.{key} missing from merged config"
        assert isinstance(fcp["turn_fallback_enabled"], bool)
        assert 0.0 <= fcp["turn_fallback_max_reverse_m"] <= 0.5, (
            "the fallback's straight reverse must stay of the order of the "
            "reverse-escape's 0.30 m"
        )
        assert 1.0 <= fcp["turn_fallback_max_rejoin_arc_m"] <= 5.0, (
            "the rejoin search must reach past a U-turn (> 1 m) but stay a few metres"
        )
        assert 30.0 <= fcp["turn_fallback_min_turn_deg"] <= 90.0, (
            "the turn threshold must keep straights (0 deg) on the WEDGED path "
            "and still catch the 70 deg kinks of 2026-09-22"
        )
        assert 10.0 <= fcp["turn_fallback_timeout_s"] <= 120.0


def test_turn_fallback_is_unreachable_without_lidar() -> None:
    """The fallback runs only where FTC's obstacle deviation runs. The no-LiDAR
    overlay must keep both obstacle flags off, so a GPS-only robot behaves
    exactly as before (it has no local obstacle layer to plan the manoeuvre on).
    """
    fcp = _controller_section(_load_no_lidar_params())["FollowCoveragePath"]
    assert fcp["enable_obstacle_deviation"] is False
    assert fcp["check_obstacles"] is False
    assert _controller_section(_load_params())["FollowCoveragePath"]["use_offset_lattice"] is True


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
        assert "FTCController" not in fp.get("primary_controller", {}).get("plugin", ""), (
            "FollowPath (transit) must not wrap FTC"
        )


def test_lidar_collision_monitor_has_active_hard_stop() -> None:
    """The LiDAR variant MUST carry an ACTIVE pre-contact hard-stop layer.

    2026-07-21: the static PolygonStop was disabled long ago (it froze cmd_vel
    on skirted obstacles lingering in its fixed box), which left ROS2 with NO
    stop layer — the robot drove into low/undetected obstacles at >=half speed.
    It was replaced by a velocity-projected `approach` polygon (FootprintApproach)
    that keys on the current velocity vector, so a skirted obstacle now abeam is
    off the forward projection and can't re-freeze cmd_vel. Guard that the active
    set contains an enabled `approach` polygon, that it reads the published
    footprint, and that the old static PolygonStop stays disabled. (Firmware is
    still the sole e-stop authority — this is only the soft layer.)
    """
    cmon = _load_yaml("nav2_params_lidar.yaml")["collision_monitor"]["ros__parameters"]
    active = cmon["polygons"]
    approach = [
        name for name in active
        if cmon.get(name, {}).get("action_type") == "approach"
        and cmon.get(name, {}).get("enabled") is True
    ]
    assert approach, (
        "LiDAR collision_monitor has no ACTIVE approach-type (velocity-projected) "
        "hard-stop polygon — ROS2 would have no pre-contact stop layer."
    )
    fa = cmon[approach[0]]
    assert fa.get("footprint_topic"), (
        "the approach polygon must use a footprint_topic so the projection "
        "matches the real chassis outline"
    )
    assert fa.get("min_points", 0) >= 2, "approach polygon needs stray-return hysteresis"
    # The static box must NOT be re-activated — that is the regression this guards.
    if "PolygonStop" in cmon:
        assert cmon["PolygonStop"].get("enabled") is False, (
            "static PolygonStop must stay DISABLED — the velocity-projected "
            "FootprintApproach is its deliberate replacement (froze cmd_vel on "
            "skirted obstacles)."
        )
        assert "PolygonStop" not in active, (
            "static PolygonStop must not be in the active polygons list"
        )


def test_lidar_collision_monitor_has_no_active_static_stop() -> None:
    """No ACTIVE collision_monitor polygon may be a static `stop` (issue #393).

    A `stop`-type polygon zeroes the ENTIRE twist whenever ≥min_points fall
    inside it, with NO regard for the command's direction. A front-facing static
    stop box therefore freezes a REVERSE-escape command (Nav2 recovery BackUp and
    the FTC reverse-escape both route through collision_monitor via cmd_vel_nav),
    so a robot wedged nose-to-obstacle could never back out — BackUp timed out
    (err 711) even with the rear fully open. Every active gate must instead be
    velocity-projected (`approach`) or non-freezing (`slowdown`/`limit`) so it can
    only ever act in the direction of travel. This guards against re-introducing a
    static front stop (the A-C1 PolygonStopNarrow regression that caused #393).
    """
    cmon = _load_yaml("nav2_params_lidar.yaml")["collision_monitor"]["ros__parameters"]
    active = cmon["polygons"]
    static_stops = [
        name for name in active
        if cmon.get(name, {}).get("action_type") == "stop"
    ]
    assert not static_stops, (
        f"active collision_monitor polygons must not be static `stop` type "
        f"(they freeze the reverse-escape command — issue #393): {static_stops}. "
        "Use `approach` (velocity-projected) so a front box can't zero a reverse."
    )


def test_transit_planner_prefers_berth() -> None:
    """Smac transit planner must weight costmap cost enough to keep berth from
    obstacle-adjacent cells (issue #393). Raising cost_travel_multiplier biases
    the blade-off transit hop away from nose-into-pocket approaches WITHOUT
    raising global inflation_radius (a documented "Start occupied" landmine)."""
    base = _load_yaml("nav2_params_base.yaml")
    mult = base["planner_server"]["ros__parameters"]["GridBased"]["cost_travel_multiplier"]
    assert float(mult) >= 3.0, (
        f"SmacPlanner2D cost_travel_multiplier={mult} — must stay ≥3.0 so transit "
        "paths keep berth from obstacles (issue #393 transit-wedge)."
    )


def test_transit_lookahead_damps_pursuit_weave() -> None:
    """RPP transit lookahead must stay long enough to damp the slow S-weave.

    2026-07-21: with the firmware yaw-rate loop (Option C) lagging the wheel
    command, a short lookahead let RPP overshoot each carrot and set up a
    meter-scale pursuit limit cycle on transits. Raised lookahead_time 1.5->2.0
    and min_lookahead_dist 0.30->0.45. Pin the floors so a revert re-opens the
    weave loudly. (A fast 2-4 Hz buzz is the firmware loop, tuned in firmware,
    NOT here.)
    """
    fp = _controller_section(_load_params())["FollowPath"]["primary_controller"]
    assert float(fp["lookahead_time"]) >= 2.0, (
        f"FollowPath.lookahead_time={fp['lookahead_time']} < 2.0 — too short, "
        "re-opens the pursuit S-weave with the lagged firmware yaw loop."
    )
    assert float(fp["min_lookahead_dist"]) >= 0.45, (
        f"FollowPath.min_lookahead_dist={fp['min_lookahead_dist']} < 0.45 — too "
        "short at low speed, re-opens the pursuit S-weave."
    )
    assert float(fp["min_lookahead_dist"]) <= float(fp["max_lookahead_dist"]), (
        "min_lookahead_dist must not exceed max_lookahead_dist"
    )


def test_lyrical_transit_parameters_reach_primary_controller() -> None:
    for overlay, collision in (("lidar", True), ("no_lidar", False)):
        cfg = _deep_merge(_load_yaml("nav2_params_base.yaml"),
                          _load_yaml(f"nav2_params_{overlay}.yaml"))
        controller = _controller_section(cfg)
        rpp = controller["FollowPath"]["primary_controller"]
        assert rpp["plugin"] == (
            "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
        )
        assert rpp["max_linear_vel"] == 0.30
        assert rpp["use_collision_detection"] is collision
        assert "desired_linear_vel" not in rpp
        assert "max_linear_vel" not in controller["FollowPath"]
        assert "transform_tolerance" not in rpp
        assert "max_robot_pose_search_dist" not in rpp
        assert "transform_tolerance" not in controller["PathHandler"]
        assert cfg["local_costmap"]["local_costmap"]["ros__parameters"]["transform_tolerance"] == 0.5


if __name__ == "__main__":
    pytest.main([__file__, "-v"])


# ── Turn geometry / turn speed (issue #499) ──────────────────────────────────
#
# The violent swath-end turns that dig the lawn were traced to two numbers that
# must be consistent but live in different files with nothing relating them:
# coverage_server's turn radii (mowgli_robot.yaml) and FollowCoveragePath's
# speed/angular clamps (nav2_params_base.yaml). These tests pin the *relations*
# and the guard that reports them — deliberately NOT any particular radius,
# which is a field-measured trade against the connector fallback rate.


def _template_robot_params() -> dict:
    """The in-package mowgli_robot.yaml template (Invariant 15: defaults live
    here, the installed file is sparse)."""
    with open(_config_path("mowgli_robot.yaml"), "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)["mowgli"]["ros__parameters"]


def test_default_wheel_track_matches_template() -> None:
    """DEFAULT_WHEEL_TRACK_M is the last-resort floor beneath the template's
    wheel_track, not a second source of truth for it. If the two diverge, the
    half-track the turn-geometry check compares against stops describing the
    robot. Same contract as DEFAULT_TOOL_WIDTH_M."""
    from robot_config_util import DEFAULT_WHEEL_TRACK_M

    template = _template_robot_params()
    assert "wheel_track" in template, (
        "mowgli_robot.yaml template lost wheel_track — the turn-geometry check "
        "and the firmware kinematics push would fall back to a constant."
    )
    assert float(template["wheel_track"]) == pytest.approx(DEFAULT_WHEEL_TRACK_M), (
        f"template wheel_track={template['wheel_track']} != "
        f"DEFAULT_WHEEL_TRACK_M={DEFAULT_WHEEL_TRACK_M} — the fallback has drifted "
        "from the real default."
    )


def test_template_turn_speed_ratio_is_a_slowdown() -> None:
    """turn_speed_ratio scales mowing_speed into FollowCoveragePath.speed_slow.
    A default above 1.0 would ship the exact defect it was added to fix: swath-end
    turns driven FASTER than the straights."""
    template = _template_robot_params()
    assert "turn_speed_ratio" in template, (
        "mowgli_robot.yaml template is missing turn_speed_ratio — speed_slow would "
        "fall back to a static value and stop tracking the operator's mowing_speed."
    )
    ratio = float(template["turn_speed_ratio"])
    assert 0.0 < ratio <= 1.0, (
        f"turn_speed_ratio={ratio} must be in (0, 1.0]: >1 drives turns faster than "
        "straights (issue #499), <=0 stops the robot in every bend."
    )


def test_navigation_launch_injects_derived_speed_slow() -> None:
    """One load-bearing line: without this injection speed_slow falls back to the
    static base.yaml value and the operator's mowing_speed stops applying to
    swath-end turns — the issue #499 defect where turns ran FASTER than straights.

    The arithmetic itself (ratio clamp, min_speed_mps floor, mowing_speed ceiling)
    is exercised for real in test_robot_config_util.py::TestDeriveTurnSpeed; this
    only guards that the launch actually calls it and injects the result."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"fcp\[.speed_slow.\]\s*=\s*turn_speed", src), (
        "navigation.launch.py no longer injects FollowCoveragePath.speed_slow from "
        "the derived turn speed — mowing_speed stops applying to turns (issue #499)."
    )
    assert "derive_turn_speed(" in src and "turn_speed_ratio" in src, (
        "navigation.launch.py does not derive the turn speed from mowing_speed via "
        "robot_config_util.derive_turn_speed."
    )


def test_navigation_launch_runs_the_turn_geometry_check() -> None:
    """The check must run at launch and must be fed the CONFIGURED wheel track,
    never a literal — the half-track is what makes an arc undrivable forward-only.

    Its behaviour (what it flags, and that it never raises) is exercised for real
    in test_robot_config_util.py::TestCheckTurnGeometry."""
    src = _read_text("launch/navigation.launch.py")
    assert "check_turn_geometry(" in src, (
        "navigation.launch.py does not run the turn-geometry check — the issue #499 "
        "geometry defect can be reintroduced silently."
    )
    assert re.search(r"wheel_track\s*=\s*float\(rt_rp\.get\(", src), (
        "wheel_track is not read from the robot config — the turn-geometry check "
        "must never compare against a hardcoded track."
    )


def test_base_ftc_can_command_its_own_turn_speed() -> None:
    """Consistency inside the file we control: FTC's own speed_slow and
    max_cmd_vel_ang imply a tightest commandable arc of speed_slow/max_cmd_vel_ang.
    That must stay a sane, positive radius — if max_cmd_vel_ang were ever dropped
    toward zero the controller could not turn at all."""
    fcp = _controller_section(_load_params())["FollowCoveragePath"]
    wz_max = float(fcp["max_cmd_vel_ang"])
    assert wz_max > 0.0, "max_cmd_vel_ang must be positive or FTC cannot turn"
    tightest = float(fcp["speed_slow"]) / wz_max
    assert 0.0 < tightest < 1.0, (
        f"speed_slow/max_cmd_vel_ang = {tightest:.3f} m is not a plausible turn "
        "radius for this chassis — check the FollowCoveragePath speed/angular pair."
    )


def test_ftc_blade_load_keys_present_in_both_variants() -> None:
    """FTC's blade-load slowdown (ftc_blade_load.hpp / BladeLoadDecision) needs
    all five blade_load_* keys in the static config — a missing key silently
    falls back to the C++ struct default instead of failing loudly. The ramp
    must be non-empty even while the feature ships disabled, so flipping the
    GUI toggle alone is enough to engage it.
    """
    keys = ("blade_load_slowdown_enabled", "blade_load_rpm_full", "blade_load_rpm_min",
            "blade_load_min_speed_ratio", "blade_load_telemetry_max_age_s")
    for loader in (_load_params, _load_no_lidar_params):
        fcp = _controller_section(loader())["FollowCoveragePath"]
        for key in keys:
            assert key in fcp, f"FollowCoveragePath.{key} missing from merged config"
        assert fcp["blade_load_slowdown_enabled"] is False, (
            "blade_load_slowdown_enabled must ship OFF in the static yaml: the "
            "no-load blade RPM differs per motor, so a fresh install must not crawl "
            "on unverified thresholds — the operator enables it from the GUI"
        )
        assert fcp["blade_load_rpm_full"] > fcp["blade_load_rpm_min"] > 0.0
        assert 0.0 < fcp["blade_load_min_speed_ratio"] <= 1.0
        # 4 Hz blade reports: the age gate must admit at least two missed reports.
        assert fcp["blade_load_telemetry_max_age_s"] >= 0.5


def test_blade_load_template_defaults_match_static_yaml() -> None:
    """The template's blade_load_* knobs must equal the static
    nav2_params_base.yaml values so a sparse installed file (no overrides) is
    behaviour-preserving, and so the GUI's 'at default' marker tells the truth."""
    rp = _template_robot_params()
    fcp = _controller_section(_load_yaml("nav2_params_base.yaml"))["FollowCoveragePath"]
    assert bool(rp["blade_load_slowdown_enabled"]) == bool(fcp["blade_load_slowdown_enabled"])
    for key in ("blade_load_rpm_full", "blade_load_rpm_min", "blade_load_min_speed_ratio"):
        assert float(rp[key]) == float(fcp[key]), (
            f"template {key}={rp[key]} != nav2_params_base.yaml FollowCoveragePath.{key}={fcp[key]}"
        )


def test_navigation_launch_injects_ftc_blade_load() -> None:
    """All four operator blade_load_* keys must reach FollowCoveragePath.

    If this injection is dropped the GUI's Mowing → Blade load toggle goes inert
    (FTC's declare_parameter default, false, wins) and the slowdown can never be
    switched on from the settings page."""
    src = _read_text("launch/navigation.launch.py")
    for key in ("blade_load_slowdown_enabled", "blade_load_rpm_full", "blade_load_rpm_min",
                "blade_load_min_speed_ratio"):
        assert re.search(
            r"fcp\[.%s.\]\s*=\s*blade_load_params\[.%s.\]" % (key, key), src), (
            f"navigation.launch.py must inject {key} into FollowCoveragePath.{key} "
            "via derive_blade_load_params."
        )


def test_lidar_map_anchor_flag_is_plumbed_end_to_end() -> None:
    """use_lidar_map_anchor must travel template -> navigation.launch.py ->
    fusion_graph.launch.py -> node, gated on LiDAR hardware.

    Without the launch plumbing an installed override is silently inert, and
    without the template key the GUI cannot reset it (Invariant 15)."""
    nav = _read_text("launch/navigation.launch.py")
    assert re.search(r'_rp\.get\("use_lidar_map_anchor", False\)', nav), (
        "navigation.launch.py must read use_lidar_map_anchor from the robot config"
    )
    assert re.search(r'"use_lidar_map_anchor":\s*lidar_gated\(use_lidar_map_anchor\)', nav), (
        "use_lidar_map_anchor must be forwarded to fusion_graph LiDAR-gated: no scanner, no grid"
    )
    fg_launch = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "..", "..", "fusion_graph", "launch", "fusion_graph.launch.py")
    with open(fg_launch, "r", encoding="utf-8") as fh:
        fg = fh.read()
    assert re.search(r'DeclareLaunchArgument\(\s*"use_lidar_map_anchor"', fg)
    assert re.search(r'"use_lidar_map_anchor":\s*use_lidar_map_anchor', fg)
    template = _load_yaml("mowgli_robot.yaml")["mowgli"]["ros__parameters"]
    assert template.get("use_lidar_map_anchor") is False, (
        "the LiDAR map anchor ships OFF by default (2026-09-13): experimental, costly on the "
        "Pi, enabled per robot from Settings > Localization; the launch fallback must agree"
    )
    # Shadow mode rides the same path: config -> navigation.launch.py
    # (LiDAR-gated) -> fusion_graph.launch.py -> node.
    assert re.search(r'_rp\.get\("lidar_anchor_shadow_mode", False\)', nav)
    assert re.search(r'"lidar_anchor_shadow_mode":\s*lidar_gated\(lidar_anchor_shadow_mode\)', nav)
    assert re.search(r'DeclareLaunchArgument\(\s*"lidar_anchor_shadow_mode"', fg)
    assert re.search(r'"lidar_anchor_shadow_mode":\s*lidar_anchor_shadow_mode', fg)
    assert template.get("lidar_anchor_shadow_mode") is False


# ---------------------------------------------------------------------------
# ROS 2 Lyrical / Nav2 1.5.1 additions
# ---------------------------------------------------------------------------


def test_transit_dwpp_is_off_by_default_in_both_variants() -> None:
    """Dynamic Window Pure Pursuit changes how EVERY transit, dock approach and
    HOME leg is driven. It ships OFF so an upgrade never silently re-tunes the
    transit lane; the operator turns it on for a supervised field test through
    `transit_dynamic_window` in mowgli_robot.yaml."""
    for loader in (_load_params, _load_no_lidar_params):
        primary = _controller_section(loader())["FollowPath"]["primary_controller"]
        assert primary["use_dynamic_window"] is False, (
            "FollowPath.primary_controller.use_dynamic_window must ship false — "
            "enabling DWPP by default re-tunes transit without a field test."
        )


def test_transit_dwpp_window_is_forward_only_and_within_chassis_limits() -> None:
    """The DWPP window bounds must agree with the rest of the transit config:
    no reverse (allow_reversing is false — a reversing transit drags the deck
    over ground the robot cannot see), and no yaw rate above what the firmware
    yaw loop holds (FTC clamps at max_cmd_vel_ang 0.8)."""
    for loader in (_load_params, _load_no_lidar_params):
        primary = _controller_section(loader())["FollowPath"]["primary_controller"]
        assert primary["allow_reversing"] is False
        assert primary["min_linear_vel"] == 0.0, (
            "min_linear_vel must be 0.0 so the dynamic window cannot select a "
            "reverse velocity on a controller configured allow_reversing: false."
        )
        assert primary["max_angular_vel"] <= 0.8, (
            "max_angular_vel above 0.8 rad/s lets DWPP command a yaw rate the "
            "firmware yaw loop cannot hold (FTC's own clamp is 0.8)."
        )
        assert primary["min_angular_vel"] == -primary["max_angular_vel"]
        # Nav2 expects the deceleration limits NEGATIVE; a positive value here
        # makes the window collapse instead of widening.
        assert primary["max_linear_decel"] < 0.0
        assert primary["max_angular_decel"] < 0.0
        assert primary["max_linear_accel"] > 0.0


def test_navigation_launch_injects_transit_dynamic_window() -> None:
    """The template knob must actually reach RPP. An orphan knob is worse than
    no knob: the GUI shows a toggle that changes nothing (issue #192 class)."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(
        r"fp\[.primary_controller.\]\[.use_dynamic_window.\]\s*=.*transit_dynamic_window",
        src, re.DOTALL), (
        "navigation.launch.py must inject transit_dynamic_window into "
        "FollowPath.primary_controller.use_dynamic_window."
    )
    template = _template_robot_params()
    assert template["transit_dynamic_window"] is False, (
        "the template default must stay false — see the base.yaml rationale."
    )


def test_coverage_stays_on_ftc_when_dwpp_is_available() -> None:
    """DWPP is a lookahead-carrot method: it cuts the R = 0.20 m turn-around
    arcs of an F2C route exactly as any pure pursuit does. The coverage slot is
    FTCController (CLAUDE.md Invariant 8) and DWPP must never appear under it."""
    for loader in (_load_params, _load_no_lidar_params):
        fcp = _controller_section(loader())["FollowCoveragePath"]
        assert fcp["plugin"] == "mowgli_nav2_plugins/FTCController"
        assert "use_dynamic_window" not in fcp


def test_axis_goal_checker_is_declared_but_not_selected() -> None:
    """The stock Lyrical AxisGoalChecker is available for a field comparison,
    but the default stays PathProgressGoalChecker: only a progress gate survives
    a closed headland ring whose start and end coincide."""
    for loader in (_load_params, _load_no_lidar_params):
        cfg = _controller_section(loader())
        assert "coverage_axis_goal_checker" in cfg["goal_checker_plugins"]
        axis = cfg["coverage_axis_goal_checker"]
        assert axis["plugin"] == "nav2_controller::AxisGoalChecker"
        # Bounded by the FeasiblePathHandler's prune_distance: the checker only
        # ever sees the LOCAL plan, so a tolerance near 2.0 m unlocks instantly.
        prune_distance = cfg["PathHandler"].get("prune_distance", 2.0)
        assert axis["path_length_tolerance"] < prune_distance, (
            "path_length_tolerance must stay below the path handler's "
            f"prune_distance ({prune_distance} m) or the goal is 'reached' from "
            "the first tick of every segment."
        )
        # FTC parks max_goal_distance_error short of the final pose by design.
        park = _base_ftc_max_goal_distance_error()
        assert axis["along_path_tolerance"] >= park, (
            f"along_path_tolerance {axis['along_path_tolerance']} is tighter than "
            f"FTC's park distance {park} — the goal could never be reached."
        )
    template = _template_robot_params()
    assert template["coverage_goal_checker_id"] == "coverage_goal_checker"


def test_behavior_tree_gets_the_coverage_goal_checker_id() -> None:
    """The BT dispatches coverage goals with this id; it must be wired from the
    robot config, not hardcoded in the node."""
    src = _read_text("launch/full_system.launch.py")
    assert "coverage_goal_checker_id" in src, (
        "full_system.launch.py must pass coverage_goal_checker_id to "
        "behavior_tree_node."
    )


def test_custom_inscribed_radius_is_opt_in_and_becomes_the_floor() -> None:
    """local_inflation_inscribed_radius defaults to -1.0 (derive from the
    footprint, Nav2's own behaviour). When set, it MUST also become the
    inflation floor — otherwise the chassis-derived floor raises the radius
    straight back up and the override is inert."""
    template = _template_robot_params()
    assert template["local_inflation_inscribed_radius"] == -1.0
    src = _read_text("launch/navigation.launch.py")
    assert re.search(
        r"lc_infl\[.custom_inscribed_radius.\]\s*=\s*local_inflation_inscribed_radius",
        src), "navigation.launch.py must inject custom_inscribed_radius."
    assert re.search(
        r"if\s+local_inflation_inscribed_radius\s*>=\s*0\.0:.*?"
        r"infl_floor\s*=\s*local_inflation_inscribed_radius",
        src, re.DOTALL), (
        "when set, local_inflation_inscribed_radius must replace the "
        "chassis-derived inflation floor."
    )


# ---------------------------------------------------------------------------
# Chassis geometry consistency (2026-09-16 collision)
# ---------------------------------------------------------------------------
#
# The robot's width is consumed by four independent things: the Nav2 footprint,
# the inflation floor, the coverage planner's inset, and FTC's clearance model.
# On 2026-09-16 the chassis went 0.40 m -> 0.45 m; the first two followed
# (they DERIVE it), the coverage planner did not (it carried a hardcoded 0.40),
# so every plan routed the centreline 0.20 m from obstacles while the body
# reached 0.225 m. FTC tracked that plan to within 2 cm and drove 2.5 cm into a
# mapped tree. These tests exist so a chassis edit can never again move some
# consumers and not others.


def test_navigation_launch_derives_chassis_width_from_the_config() -> None:
    """`cw` feeds coverage_server.robot_width AND the safety-inset floor. It must
    come from the merged robot config, never from a literal — chassis_width is
    operator-editable in the GUI."""
    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"cw\s*=\s*float\(\s*rp\.get\(\s*[\"']chassis_width[\"']", src), (
        "navigation.launch.py must read the chassis width from the merged robot "
        "config (rp), not hardcode it — the hardcoded 0.40 is what put the "
        "coverage plan inside the robot."
    )
    assert not re.search(r"^\s*cw\s*=\s*0\.\d+\s*$", src, re.MULTILINE), (
        "a bare numeric `cw = <literal>` is the exact regression this guards."
    )
    assert re.search(r"cov_params\[[\"']robot_width[\"']\]\s*=\s*cw", src), (
        "coverage_server.robot_width must be the real chassis width."
    )


def test_obstacle_margin_is_floored_at_the_body_half_width() -> None:
    """obstacle_margin is the ONLY obstacle clearance the coverage plan has once
    chassis_safety_inset is 0. Its floor is the MAX of two derived demands:

      * controller: chassis_half_width + FTC's clamped obstacle_clearance_margin
        + FTC_TRACKING_SLACK_M — FTC's footprint clearance model refuses a line
        closer than half-width + clearance to a lethal cell, so a plan at the
        bare half-width made FTC fight its own plan along every obstacle;
      * transit plannability: chassis_half_width (the un-inflated keepout band)
        + the mask->global-costmap rasterisation slack, so a robot ON its
        coverage line is never START_OCCUPIED.
    """
    import math

    import robot_config_util as rcu

    src = _read_text("launch/navigation.launch.py")
    assert re.search(r"planned_margin\s*=\s*planning_obstacle_margin\(", src), (
        "the obstacle-margin floor must be DERIVED via "
        "robot_config_util.planning_obstacle_margin(), not hardcoded — a literal "
        "goes stale when the operator edits the chassis in the GUI."
    )
    assert re.search(
        r'"obstacle_clearance_margin":\s*clamped_clearance_margin', src), (
        "the floor must be computed from the SAME clamped clearance margin "
        "that is injected into FTC."
    )
    assert re.search(
        r'fcp\["obstacle_clearance_margin"\]\s*=\s*clamped_clearance_margin', src)

    rp = _template_robot_params()
    hw = rcu.chassis_half_width(rp)
    controller = (hw + rcu.ftc_obstacle_clearance_margin(rp)
                  + rcu.FTC_TRACKING_SLACK_M)
    transit = hw + rcu.GLOBAL_COSTMAP_RESOLUTION_M * math.sqrt(2.0)
    floor = rcu.planning_obstacle_margin_floor(rp)
    assert max(controller, transit) <= floor < max(controller, transit) + 1e-3, (
        "the floor is max(controller, transit) rounded UP to the millimetre."
    )
    # An operator value below the floor is RAISED; above it, obeyed; capped.
    assert rcu.planning_obstacle_margin({**rp, "obstacle_margin": 0.0}) == floor
    assert rcu.planning_obstacle_margin({**rp, "obstacle_margin": 0.6}) == 0.6
    assert rcu.planning_obstacle_margin({**rp, "obstacle_margin": 5.0}) == 1.0
    # It follows the chassis and FTC's clearance margin — nothing is a literal.
    wide = {**rp, "chassis_width": float(rp["chassis_width"]) + 0.10}
    assert rcu.planning_obstacle_margin_floor(wide) == pytest.approx(
        floor + 0.05, abs=1.1e-3)
    roomy = {**rp, "obstacle_clearance_margin": 0.20}
    assert rcu.planning_obstacle_margin_floor(roomy) == pytest.approx(
        hw + 0.20 + rcu.FTC_TRACKING_SLACK_M, abs=1.1e-3)
    # FTC clamps the clearance margin; the floor must use the clamped value.
    silly = {**rp, "obstacle_clearance_margin": 9.0}
    assert rcu.ftc_obstacle_clearance_margin(silly) == rcu.FTC_CLEARANCE_MARGIN_MAX_M


def test_keepout_margin_counts_the_body_exactly_once() -> None:
    """The keepout mask's consumer is SmacPlanner2D — a POINT check with no
    footprint test — and the mask is NOT inflated (plugin order pinned below).
    So the band map_server paints around a drawn obstacle must be the WHOLE body
    half-width, once: not coverage's obstacle_margin (a centreline offset that
    also carries FTC's slack), and with no inflation band on top."""
    import robot_config_util as rcu

    rp = _template_robot_params()
    hw = rcu.chassis_half_width(rp)
    keepout = rcu.keepout_obstacle_margin(rp)
    assert hw <= keepout < hw + 1e-3, (
        f"keepout band {keepout:.4f} must be the body half-width {hw:.3f} on "
        "the shipped config."
    )
    # By construction a robot ON its coverage line is plannable: the line
    # clears the band by the worst-case rasterisation slack, for ANY operator
    # obstacle_margin and any chassis.
    for override in ({}, {"obstacle_margin": 0.6}, {"obstacle_margin": 5.0},
                     {"chassis_width": 0.535}, {"chassis_width": 0.39},
                     {"obstacle_clearance_margin": 0.3}):
        params = {**rp, **override}
        line = rcu.planning_obstacle_margin(params)
        band = rcu.keepout_obstacle_margin(params)
        assert band >= rcu.chassis_half_width(params) - 1e-12, override
        assert line - band >= rcu.keepout_raster_slack() - 1e-9, (
            f"{override}: coverage line {line:.3f} is within the raster slack "
            f"of the keepout band {band:.3f} — START_OCCUPIED on the line."
        )
    # An operator-RAISED obstacle_margin (root zone) pulls the transit band up
    # with it; it is never silently coverage-only.
    raised = rcu.keepout_obstacle_margin({**rp, "obstacle_margin": 0.6})
    assert raised == pytest.approx(0.6 - rcu.keepout_raster_slack())


def test_global_costmap_inflates_before_the_keepout_filter() -> None:
    """inflation_layer BEFORE keepout_filter in BOTH variants: the keepout mask
    (recorded boundary + drawn obstacles) already carries the body, so inflating
    it counted the body twice. Hand-applied on the robot 2026-09-17 (17-minute
    mow, zero START_OCCUPIED, zero boundary e-stops); an image deploy would have
    silently discarded it. keepout_obstacle_margin's derivation ASSUMES this
    order — flipping it back makes every coverage line next to a drawn obstacle
    un-plannable again."""
    for loader, first in ((_load_params, "obstacle_layer"),
                          (_load_no_lidar_params, "static_layer")):
        gc = loader()["global_costmap"]["global_costmap"]["ros__parameters"]
        plugins = gc["plugins"]
        # Source layers first (the no-LiDAR variant also carries the fleet
        # peers layer, which must be inflated like any obstacle), then
        # inflation, and the keepout filter LAST so the mask is never inflated.
        assert plugins[0] == first, f"global costmap plugin order is {plugins}"
        assert plugins[-1] == "keepout_filter", f"global costmap plugin order is {plugins}"
        assert plugins.index("inflation_layer") == len(plugins) - 2, (
            f"inflation_layer must sit right before keepout_filter: {plugins}"
        )
        lc = loader()["local_costmap"]["local_costmap"]["ros__parameters"]
        assert "keepout_filter" not in lc["plugins"]  # Invariant 5


def test_global_costmap_resolution_matches_the_launch_constant() -> None:
    """full_system.launch.py does not load the Nav2 params, so the global
    costmap resolution used by the margin derivations is single-sourced in
    robot_config_util — and pinned here so the two cannot drift."""
    import robot_config_util as rcu

    for loader in (_load_params, _load_no_lidar_params):
        gc = loader()["global_costmap"]["global_costmap"]["ros__parameters"]
        assert float(gc["resolution"]) == rcu.GLOBAL_COSTMAP_RESOLUTION_M
    # The raster-slack bound assumes the mask is no coarser than the costmap.
    map_yaml = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "mowgli_map",
        "config", "map_server.yaml")
    with open(map_yaml) as fh:
        map_doc = yaml.safe_load(fh)
    (node_params,) = [v["ros__parameters"] for v in map_doc.values()]
    assert float(node_params["resolution"]) <= rcu.GLOBAL_COSTMAP_RESOLUTION_M


def test_enforce_boundary_margin_is_floored_at_the_circumscribed_radius() -> None:
    """The outside-slack band is the room the BODY has to overhang the recorded
    line. chassis_safety_inset is 0, so the outermost coverage pass puts the
    robot's CENTRE on that line and the footprint then reaches up to the
    CIRCUMSCRIBED radius outside it in any orientation — only 0.275 m sideways,
    but chassis_center_x + chassis_length/2 + margin = 0.53 m forward at a row
    end. That is why the 0.40 m literal (sized for a 0.40 m chassis) and the
    0.275 m half-width floor that replaced it both left the front of the chassis
    over LETHAL keepout cells."""
    src = _read_text("launch/full_system.launch.py")
    assert re.search(
        r"boundary_margin_floor\s*=\s*chassis_circumscribed_radius\(", src), (
        "the enforce_boundary_margin_m floor must be DERIVED via "
        "robot_config_util.chassis_circumscribed_radius(), not hardcoded and "
        "not the half-width — the half-width only covers the SIDEWAYS overhang, "
        "and a literal goes stale when the operator edits the chassis in the "
        "GUI."
    )
    assert not re.search(
        r"boundary_margin_floor\s*=\s*chassis_half_width\(", src), (
        "the half-width floor was a no-op (0.275 < the 0.40 fallback) and does "
        "not cover the 0.53 m forward reach of the footprint."
    )
    assert re.search(
        r"enforce_boundary_margin_m\s*<\s*boundary_margin_floor.*?"
        r"enforce_boundary_margin_m\s*=\s*boundary_margin_floor",
        src, re.DOTALL), (
        "an operator value below the floor must be RAISED to it, not obeyed."
    )
    assert re.search(
        r'\{"enforce_boundary_margin_m":\s*enforce_boundary_margin_m\}', src), (
        "map_server must receive the FLOORED value, not a fresh read of the "
        "robot config."
    )
    # The floor must actually clear the forward reach on the shipped chassis —
    # the whole point of moving off the half-width.
    from robot_config_util import chassis_footprint as _chassis_footprint

    template = _load_yaml("mowgli_robot.yaml")["mowgli"]["ros__parameters"]
    front, _rear, half_width = _chassis_footprint(template)
    floor = _chassis_circumscribed_radius(template)
    assert floor >= front > half_width, (
        "the derived floor must cover the FORWARD footprint reach, which is "
        f"what the half-width ({half_width:.3f} m) misses ({front:.3f} m)."
    )


def test_boundary_inset_and_obstacle_margin_are_separate_knobs() -> None:
    """The two used to be one value with opposite requirements, which is why no
    setting satisfied both: 0 put the ring on the recorded line but left the
    obstacle holes raw, while a body-sized value cleared obstacles and refused
    to mow a 27 cm band along every hedge."""
    from robot_config_util import planning_obstacle_margin_floor

    template = _template_robot_params()
    derived = planning_obstacle_margin_floor(template)

    # Boundary: the recorded perimeter was DRIVEN by the operator, so it is the
    # reachable limit — the outermost pass rides on it.
    assert template["chassis_safety_inset"] == 0.0, (
        "chassis_safety_inset is boundary-only; any inset is a band that never "
        "gets cut. Obstacle clearance belongs to obstacle_margin."
    )
    # Obstacles: equal to the DERIVED floor (body half-width + what FTC and the
    # transit keepout demand) so the yaml does not lie to whoever reads it (0.2
    # survived a 0.40 -> 0.45 m chassis unnoticed). The GUI schema default must
    # be the same number, or the settings backend's prune-equal-to-default
    # would diverge from the template.
    assert template["obstacle_margin"] == pytest.approx(derived, abs=1e-9), (
        f"template obstacle_margin {template['obstacle_margin']} != derived "
        f"floor {derived:.3f} for the shipped chassis/template."
    )
    schema_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..",
        "gui", "asserts", "mower_config.schema.json")
    if os.path.isfile(schema_path):  # absent in a ros2-only checkout/container
        import json

        with open(schema_path) as fh:
            schema_text = json.load(fh)

        def _find(node, key):
            if isinstance(node, dict):
                if key in node and isinstance(node[key], dict):
                    return node[key]
                for value in node.values():
                    found = _find(value, key)
                    if found is not None:
                        return found
            return None

        prop = _find(schema_text, "obstacle_margin")
        assert prop is not None
        assert prop["default"] == pytest.approx(derived, abs=1e-9)
        assert "transit planning" not in prop["description"].split(".")[0], (
            "obstacle_margin no longer drives the transit keepout band directly."
        )


def test_ftc_uses_the_real_footprint_for_clearance() -> None:
    """The line model's 0.12 m half-width is not this robot: the body reaches
    0.275 m. With the footprint model the geometry is honest and the margin is
    real slack rather than a stand-in for missing body."""
    for loader in (_load_params, _load_no_lidar_params):
        fcp = _controller_section(loader())["FollowCoveragePath"]
        assert fcp["use_footprint_clearance"] is True, (
            "FTC must sample the real chassis polygon; the line model under-states "
            "the body by 0.155 m per side and needs a compensating margin."
        )
        # Slack on top of a real body — not a body substitute. The old 0.20 m
        # value existed only to make 0.12 + margin reach the true half-width.
        assert fcp["obstacle_clearance_margin"] <= 0.10, (
            f"obstacle_clearance_margin {fcp['obstacle_clearance_margin']} is "
            "body-sized, not slack-sized: on top of the real footprint it puts "
            "the robot back half a metre from every hedge."
        )


def test_ftc_plans_avoidance_as_a_whole_profile() -> None:
    """The lattice planner needs the real chassis polygon (it falls back to the
    legacy single-offset search without one) and a horizon the LiDAR can fill."""
    fcp = _controller_section(_load_params())["FollowCoveragePath"]
    assert fcp["use_offset_lattice"] is True
    assert fcp["use_footprint_clearance"] is True, (
        "use_offset_lattice silently degrades to the legacy search when FTC has "
        "no footprint polygon to sample."
    )
    assert 0.5 <= fcp["avoidance_horizon_m"] <= 3.0, (
        "Beyond the obstacle layer's marking range the lattice plans over cells "
        "nothing can ever mark; too short and a skirt cannot ramp in."
    )
    # A lattice step per station at the steepest slope: finer lateral steps than
    # the local costmap resolution buy nothing.
    assert fcp["deviation_step"] >= 0.05
    assert fcp["avoidance_max_slope"] > 0.0


def test_collision_monitor_polygons_follow_the_chassis() -> None:
    """The collision monitor is the last line of defence; its polygons may not be
    narrower than the body. They shipped as literals sized for the 0.40 m chassis
    (PolygonStopNarrow ±0.20, PolygonStop ±0.18) against a body reaching ±0.275,
    so 7.5-9.5 cm of carriage on each side sat OUTSIDE the polygon meant to stop
    before touching. FootprintApproach is exempt: it reads
    /local_costmap/published_footprint and already follows the real geometry."""
    src = _read_text("launch/navigation.launch.py")
    for polygon in ("PolygonStopNarrow", "PolygonStop"):
        assert re.search(
            rf'cm_params\["{polygon}"\]\["points"\]\s*=\s*_poly\(', src), (
            f"{polygon}.points must be DERIVED from the chassis footprint at "
            "launch, not left as a literal in the overlay yaml."
        )
    assert re.search(r"_poly\(\s*fp_f \+ 0\.12,\s*fp_f - 0\.02,\s*fp_hw\)", src)
    assert re.search(r"_poly\(\s*fp_f \+ 0\.05,\s*fp_r - 0\.05,\s*fp_hw\)", src)
    # The footprint triple must be bound unconditionally — reading it from the
    # injection closure when only an `if rp:` branch assigned it is the
    # UnboundLocalError class that crash-looped the stack on 2026-09-16.
    assert re.search(r"^    fp_f, fp_r, fp_hw = chassis_footprint\(rp or \{\}\)", src,
                     re.MULTILINE), (
        "fp_f/fp_r/fp_hw must be bound at function scope, not only inside `if rp:`."
    )


# ── fleet peer obstacles (docs/MULTI_ROBOT.md) ──────────────────────────────

_FLEET_TOPIC = "/fleet/peer_obstacles"


def _fleet_sources(params: dict) -> list:
    """Every observation source in this costmap that reads the fleet cloud."""
    found = []
    for layer in params.get("plugins", []):
        cfg = params.get(layer, {})
        for src in str(cfg.get("observation_sources", "")).split():
            if cfg.get(src, {}).get("topic") == _FLEET_TOPIC:
                found.append((layer, src, cfg[src]))
    return found


def test_lidar_variant_marks_fleet_peers_in_the_local_costmap_only() -> None:
    """With a LiDAR the peer cloud goes on the LOCAL obstacle_layer and NOT the
    global one: FTC treats a cell that is lethal in the global costmap as
    'not an obstacle' (zone mask), which would hide a peer from the coverage
    controller. Marking only — the cloud has no sensor origin to raytrace from.
    """
    merged = _deep_merge(_load_yaml("nav2_params_base.yaml"), _load_yaml("nav2_params_lidar.yaml"))
    local = _fleet_sources(merged["local_costmap"]["local_costmap"]["ros__parameters"])
    glob = _fleet_sources(merged["global_costmap"]["global_costmap"]["ros__parameters"])
    assert len(local) == 1, "lidar local costmap must carry exactly one fleet source"
    assert glob == [], "lidar global costmap must NOT carry the fleet source (FTC zone mask)"
    layer, _, src = local[0]
    assert layer == "obstacle_layer"
    assert src["data_type"] == "PointCloud2"
    assert src["marking"] is True and src["clearing"] is False
    assert src["min_obstacle_height"] < 0.30 < src["max_obstacle_height"], (
        "fleet_peer_obstacles.py publishes its ring at z=0.30 m; it must sit inside the band"
    )


def test_no_lidar_variant_marks_fleet_peers_in_both_costmaps() -> None:
    """Without a LiDAR nothing else can see a peer, so a dedicated fleet_layer
    (an ObstacleLayer under a name CI's obstacle/static disjointness guard does
    not police) feeds BOTH costmaps."""
    merged = _deep_merge(_load_yaml("nav2_params_base.yaml"), _load_yaml("nav2_params_no_lidar.yaml"))
    for cm in ("local_costmap", "global_costmap"):
        params = merged[cm][cm]["ros__parameters"]
        found = _fleet_sources(params)
        assert len(found) == 1, f"{cm}: expected exactly one fleet source"
        layer, _, src = found[0]
        assert layer == "fleet_layer"
        assert params[layer]["plugin"] == "nav2_costmap_2d::ObstacleLayer"
        assert src["marking"] is True and src["clearing"] is False
        plugins = params["plugins"]
        assert plugins.index("fleet_layer") < plugins.index("inflation_layer"), (
            f"{cm}: fleet_layer must be inflated (listed before inflation_layer)"
        )
    local = merged["local_costmap"]["local_costmap"]["ros__parameters"]["fleet_layer"]
    glob = merged["global_costmap"]["global_costmap"]["ros__parameters"]["fleet_layer"]
    assert local == glob, "the two fleet_layer copies in nav2_params_no_lidar.yaml drifted apart"
