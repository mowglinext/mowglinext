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

import pytest
import yaml


def _load_params() -> dict:
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "config", "nav2_params.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


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


def test_followpath_uses_ftc() -> None:
    """The transit/dock controller is FTCController (feat/v2-control-cascade,
    2026-06-08), mirroring the ROS1 "v2" stack which used Follow-The-Carrot
    for transit AND docking. RotationShim+RPP was removed because RPP could
    not fine-pivot on this robot's deadband-limited motors and left a
    yaw-alignment gap at the dock staging pose. FTC's PRE_ROTATE/POST_ROTATE
    align heading in place. Pin it so a revert is a conscious decision.
    """
    cfg = _controller_section(_load_params())
    assert cfg["FollowPath"]["plugin"] == "mowgli_nav2_plugins/FTCController"


def test_followpath_goal_tolerance_tighter_than_checker() -> None:
    """FTC's internal goal acceptance MUST be tighter than the
    stopped_goal_checker bound to FollowPath, or FTC stops at its own
    tolerance while the external checker keeps waiting and the action hangs.
    """
    cfg = _controller_section(_load_params())
    fp = cfg["FollowPath"]
    checker = cfg["stopped_goal_checker"]
    # max_goal_angle_error is in DEGREES; yaw_goal_tolerance in RADIANS.
    assert math.radians(fp["max_goal_angle_error"]) <= checker["yaw_goal_tolerance"]
    assert fp["max_goal_distance_error"] <= checker["xy_goal_tolerance"]


def test_followcoveragepath_uses_ftc() -> None:
    """The coverage controller MUST be FTCController. Switching back to
    RPP/MPPI breaks adjacent-row spacing accuracy (CLAUDE.md invariant
    #8) — pin it explicitly.
    """
    cfg = _controller_section(_load_params())
    assert cfg["FollowCoveragePath"]["plugin"] == "mowgli_nav2_plugins/FTCController"


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
    assert default <= 0.20, (
        f"navigation.launch.py default coverage_xy_tolerance={default} m. "
        "If a per-site mowgli_robot.yaml omits this key, the launch falls back "
        "to this default and FTC will declare every short strip 'reached' on "
        "the first tick. Tighten to <= mower_width."
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
    assert tol <= 0.20, (
        f"mowgli_robot.yaml ships coverage_xy_tolerance={tol} m. "
        "Fresh installs would copy this and silently break coverage."
    )


def test_no_lidar_variant_uses_path_progress_goal_checker() -> None:
    """nav2_params_no_lidar.yaml is the GPS-only variant — it must use the
    same PathProgressGoalChecker plugin with a high progress threshold and
    sane finite tolerances as the LiDAR variant. (The old '<= mower_width
    xy tolerance' guard was for SimpleGoalChecker, which is gone.)
    """
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "..", "config", "nav2_params_no_lidar.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    gc = cfg["controller_server"]["ros__parameters"]["coverage_goal_checker"]
    assert gc["plugin"] == COVERAGE_GOAL_CHECKER_PLUGIN
    assert 0.90 <= gc["progress_threshold"] <= 1.0
    assert 0.0 < gc["xy_goal_tolerance"] <= 1.0
    assert 0.0 < gc["yaw_goal_tolerance"] <= 3.1416


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
