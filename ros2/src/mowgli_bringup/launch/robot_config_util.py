# Copyright (C) 2026 Cedric <cedric@mowgli.dev>
#
# Shared robot-config loader for the MowgliNext launch files.
#
# The INSTALLED mowgli_robot.yaml (at /ros2_ws/config/mowgli_robot.yaml) is
# SPARSE: it holds only what is decided at install time or calibrated per site
# (datum, NTRIP credentials, dock pose, hardware selections, calibration
# paths). Every other parameter's DEFAULT lives in the in-package template
# config/mowgli_robot.yaml, which is versioned and ships with the software.
#
# load_robot_params() DEEP-MERGES the installed sparse config OVER the package
# template, so:
#   * defaults come from the template — a maintainer bumping a default
#     propagates to every robot whose installed config does not override it;
#   * a key ABSENT from the installed config falls through to the template
#     default — this is exactly how the GUI's "reset to default" works
#     (it deletes the key from the installed file);
#   * an install/site decision or an explicit GUI override still wins.
#
# Nodes therefore always receive a COMPLETE parameter set, exactly as before
# the split — only the on-disk installed file is now allowed to be sparse.
# Older FULL installed configs keep working unchanged (every key overrides its
# identical template default; a no-op merge).

import copy
import os
import sys

import yaml

DEFAULT_RUNTIME_PATH = "/ros2_ws/config/mowgli_robot.yaml"

# Fallback used ONLY if load_robot_params() itself can't find a "tool_width"
# key at all (missing/unreadable template — load_robot_params() otherwise
# always returns a complete parameter set per its docstring above, so this
# should never actually fire in a working install). Single-sourced here so
# full_system.launch.py (map_server.tool_width — mark_cells_mowed stamp
# radius / sliver detection) and navigation.launch.py (feeds
# coverage_server.operation_width = tool_width - swath_overlap, F2C's swath
# spacing) can't silently diverge by each hardcoding their own literal — that
# divergence (mower_width=0.18 vs a separately-hardcoded operation_width=0.20)
# is what caused the 54% coverage regression (see CLAUDE.md Invariant 6).
# The LIVE default in normal operation is still mowgli_bringup/config/
# mowgli_robot.yaml's tool_width (Invariant 15) — this constant is the
# last-resort floor beneath that, not a second source of truth for it, which
# is why it's expected to match the template's value (see
# test_tool_width_single_source.py).
DEFAULT_TOOL_WIDTH_M = 0.18

# Wheel track (distance between wheel centres, m). Fallback used ONLY if the
# resolved robot config carries no "wheel_track" key — the LIVE default is
# mowgli_bringup/config/mowgli_robot.yaml's wheel_track (Invariant 15).
#
# Single-sourced here for the same reason as DEFAULT_TOOL_WIDTH_M: more than one
# consumer needs it and they must not each hardcode their own literal. It MUST
# equal the firmware's WHEEL_BASE (firmware/stm32/ros_usbnode/include/board.h)
# and hardware_bridge_node's wheel_track default — the firmware does the
# differential-drive IK (left = vx - wz*track/2) with ITS copy, so a host value
# that disagrees makes every derived turn geometry a lie.
#
# navigation.launch.py uses it to check the planned turn radii against the
# HALF-track: an arc of radius <= track/2 needs the inner wheel to stop or
# reverse, which is the swath-end carving in issue #499.
DEFAULT_WHEEL_TRACK_M = 0.325


def deep_merge(base, override):
    """Recursively merge ``override`` into a copy of ``base`` (override wins).

    Only dict-vs-dict collisions recurse; any scalar/list in ``override``
    replaces the base value wholesale (robot params are flat scalars, so this
    is just the nested ros__parameters block being merged key-by-key).

    Deep-copies throughout (not just ``dict(base)``) so the result shares no
    mutable nested dict with either input — a shallow ``dict(base)`` copy
    still aliases any nested dict ``override`` doesn't touch, so mutating the
    merged result in place would silently corrupt the source ``base`` (e.g.
    the in-package template). This is the single canonical implementation —
    also imported by navigation.launch.py, compute_nav2_params.py and
    test_nav2_params.py, which each used to carry their own (drifted) copy.
    """
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_robot_config(bringup_dir, runtime_path=DEFAULT_RUNTIME_PATH):
    """Return the merged full mowgli_robot config dict (template <- runtime).

    ``bringup_dir`` is the mowgli_bringup share directory
    (get_package_share_directory("mowgli_bringup")).
    """
    template_path = os.path.join(bringup_dir, "config", "mowgli_robot.yaml")
    template = {}
    if os.path.isfile(template_path):
        with open(template_path, "r") as handle:
            template = yaml.safe_load(handle) or {}
    runtime = {}
    if os.path.isfile(runtime_path):
        with open(runtime_path, "r") as handle:
            runtime = yaml.safe_load(handle) or {}
    return deep_merge(template, runtime)


def load_robot_params(bringup_dir, runtime_path=DEFAULT_RUNTIME_PATH):
    """Return the merged mowgli.ros__parameters dict the launch files inject.

    Always complete: every template key is present, with installed-config
    values layered on top. Missing runtime file -> pure template defaults.
    """
    cfg = load_robot_config(bringup_dir, runtime_path)
    return cfg.get("mowgli", {}).get("ros__parameters", {})


# ---------------------------------------------------------------------------
# LiDAR presence: mowgli_robot.yaml ONLY (no LIDAR_ENABLED env var)
# ---------------------------------------------------------------------------
#
# `lidar_enabled` in the robot config is the ONE source of truth for whether
# the LiDAR-dependent half of the stack comes up. The `LIDAR_ENABLED`
# environment variable is NOT consulted any more (removed 2026-08-31): it used
# to be a "fallback when the yaml is silent", and on a live robot that fallback
# was exactly the failure mode --- an installed config with no `lidar_enabled`
# key plus a stale `docker/.env` saying `LIDAR_ENABLED=false` ran the whole
# stack GPS-only while the operator toggled LiDAR on in the GUI and saw nothing
# change. An ambient env var is not an operator decision; a CLI/compose
# `use_lidar:=` override still wins, because typing it is one.
#
# `LIDAR_ENABLED` legitimately survives in `docker/.env` for what it always
# really controlled: whether the `mowgli-lidar` CONTAINER is started. The two
# can now disagree in the other direction (config on, container absent), which
# is why scan_deskew_node warns when `use_lidar` is true and no scan ever
# arrives (see mowgli_localization/src/scan_deskew_node.cpp).

LIDAR_ENABLED_KEY = "lidar_enabled"

# Launch args and .env values are text; these are the tokens that read as true.
TRUE_TOKENS = ("true", "1", "yes", "on")

# Absent-key default: LiDAR OFF.
#
# `lidar_enabled` is an INSTALL-DECIDED key (Invariant 15) that is deliberately
# ABSENT from the in-package template, so --- unlike every other parameter ---
# it has no template default to fall through to and its absence carries
# meaning: no LiDAR was ever recorded for this robot. Three reasons that
# resolves to False rather than the historical True:
#
#   1. A robot whose config never mentions a LiDAR most likely has none. The
#      installer always writes the key (install/lib/config.sh) and the shipped
#      sparse seed carries it, so absence means the installer never ran ---
#      a hand-rolled docker-compose, which is precisely the no-LiDAR-hardware
#      case.
#   2. The failure modes are asymmetric. Wrongly OFF gives the fully coherent,
#      supported GPS-only stack (nav2_params_no_lidar.yaml, pass-through
#      collision_monitor, no scan factors) --- degraded but consistent. Wrongly
#      ON gives the broken half-state: an obstacle_layer with no observation
#      source, a scan-based collision_monitor with a dead source, and
#      fusion_graph subscribing to a topic nothing publishes while reporting
#      success-shaped scan-matching diagnostics.
#   3. It must equal the GUI's JSON-schema default (gui/asserts/
#      mower_config.schema.json), because the settings backend PRUNES any key
#      whose value equals its schema default (sparsifyFlat, Invariant 15). With
#      the schema default True, an operator switching LiDAR ON writes `true`,
#      the backend prunes it as "same as default", the key vanishes, and the
#      toggle is inert in the ON direction forever. False makes the ON write a
#      genuine override that persists --- and it already matches what the GUI
#      renders for an absent key (`values.lidar_enabled ?? false`).
DEFAULT_LIDAR_ENABLED = False


def is_truthy(text):
    """True when a launch-arg / config text value reads as true.

    Accepts real bools unchanged so callers can pass either a yaml value or the
    textual form a launch argument carries.
    """
    if isinstance(text, bool):
        return text
    return str(text).strip().lower() in TRUE_TOKENS


def resolve_lidar_enabled(params):
    """Resolve LiDAR presence from merged robot params.

    Returns ``(enabled, explicit)``: ``explicit`` is False when the key is
    absent and ``DEFAULT_LIDAR_ENABLED`` was used, which is the case callers
    must announce loudly (see :func:`lidar_absent_warning`).
    """
    if LIDAR_ENABLED_KEY in params:
        return bool(params[LIDAR_ENABLED_KEY]), True
    return DEFAULT_LIDAR_ENABLED, False


def lidar_absent_warning(runtime_path):
    """The startup line printed when `lidar_enabled` is absent.

    Names the file, the key and the resolved mode, because the silence is what
    made the original diagnosis take an investigation.
    """
    mode = "true" if DEFAULT_LIDAR_ENABLED else "false"
    state = "ON" if DEFAULT_LIDAR_ENABLED else "OFF"
    return (
        "LIDAR CONFIG: '{key}' is ABSENT from {path} -- resolving use_lidar={mode} "
        "(LiDAR {state}). The LIDAR_ENABLED environment variable is NO LONGER read by "
        "the ROS2 stack; {path} is the only source. Robots installed with "
        "mowglinext.sh always carry this key -- a hand-rolled docker-compose can "
        "miss it. To run WITH a LiDAR, add '{key}: true' under mowgli.ros__parameters "
        "in {path} (or flip Settings -> Sensors -> LiDAR in the GUI) and restart the "
        "stack.".format(key=LIDAR_ENABLED_KEY, path=runtime_path, mode=mode, state=state)
    )


# Paths already warned about in this process. full_system.launch.py and
# navigation.launch.py both resolve LiDAR presence, and an include loads the
# second description in the SAME process, so without this the operator sees the
# identical multi-line warning twice and learns to skim it.
_LIDAR_WARNED_PATHS = set()


def warn_lidar_key_absent(runtime_path, logger=None):
    """Emit :func:`lidar_absent_warning` once per process, per config path.

    Returns the message when it was emitted, None when suppressed as a repeat,
    so callers/tests can assert on it. ``logger`` is injectable for tests; by
    default this uses launch's own screen logger (imported lazily so this
    module stays importable without a sourced ROS2 install).
    """
    if runtime_path in _LIDAR_WARNED_PATHS:
        return None
    _LIDAR_WARNED_PATHS.add(runtime_path)
    message = lidar_absent_warning(runtime_path)
    if logger is None:
        try:
            import launch.logging

            logger = launch.logging.get_logger("mowgli_bringup")
        except ImportError:
            logger = None
    if logger is None:
        print("[WARN] " + message, file=sys.stderr)
    else:
        logger.warning(message)
    return message


# ---------------------------------------------------------------------------
# Coverage turn geometry / turn speed (issue #499)
# ---------------------------------------------------------------------------
#
# The swath-end turns that carve the lawn come from two numbers that must be
# consistent but live in different files with nothing relating them:
#
#   * coverage_server's turn radii  — mowgli_robot.yaml (min_turning_radius,
#     connector_turn_radius), the floor and nominal of buildConnector's
#     radius-shrink search;
#   * FollowCoveragePath's clamps   — nav2_params_base.yaml (speed_slow,
#     max_cmd_vel_ang), which bound what the controller can actually command.
#
# These two helpers are PURE so the arithmetic is unit-testable without a ROS2
# runtime (navigation.launch.py cannot be imported outside a sourced install).
# navigation.launch.py owns the printing; these own the decisions.


def derive_turn_speed(mowing_speed, turn_speed_ratio, min_speed_mps):
    """Turn speed (FollowCoveragePath.speed_slow) derived from mowing_speed.

    Returns ``(speed, warnings)`` — ``warnings`` is a list of human-readable
    strings the caller prints; an empty list means nothing was clamped.

    speed_slow used to be a STATIC 0.16 in nav2_params_base.yaml while
    speed_fast tracked the operator's mowing_speed, so an operator who lowered
    mowing_speed to 0.15 ended up driving the swath-end TURNS 7 % FASTER than
    the straights — backwards everywhere, and worst exactly where the robot
    carves (peak wheel speed in a bend is v * (1 + track/2R), so turn speed
    multiplies the whole problem).

    Two clamps, both encoding a real limit rather than taste:
      * ceiling ``mowing_speed`` — a turn must never be faster than a straight.
        That is the defect being fixed, and it is also what a ratio > 1 means.
      * floor ``min_speed_mps``  — FTC floors its own longitudinal output there,
        so a lower target is a value the controller would silently ignore (and
        below the firmware PWM deadband the wheels simply stall). Say so rather
        than inject a fiction.
    """
    warnings = []
    ratio = min(1.0, max(0.01, float(turn_speed_ratio)))
    if ratio != float(turn_speed_ratio):
        warnings.append(
            "WARN: turn_speed_ratio={} is outside (0, 1.0] — clamped to {}. A ratio "
            "above 1.0 would drive swath-end turns FASTER than the straights, which "
            "is the issue #499 defect.".format(turn_speed_ratio, ratio))
    speed = ratio * float(mowing_speed)
    if speed < float(min_speed_mps):
        warnings.append(
            "WARN: turn_speed_ratio={} x mowing_speed={} = {:.3f} m/s is below "
            "FollowCoveragePath.min_speed_mps={} m/s, which FTC would floor anyway "
            "— using {} m/s. Lower min_speed_mps (and mind the ~0.13 m/s firmware "
            "PWM deadband) if you need slower turns.".format(
                ratio, mowing_speed, speed, min_speed_mps, min_speed_mps))
        speed = float(min_speed_mps)
    # Never above the straight-line speed — the defect this replaces.
    return min(speed, float(mowing_speed)), warnings


def check_turn_geometry(min_turn_radius, connector_turn_radius, wheel_track,
                        turn_speed, max_cmd_vel_ang):
    """Report ways the PLANNED coverage turn radii are undrivable as planned.

    Returns a list of human-readable WARN strings (empty = geometry is sane).

    WARNINGS ONLY — never an exception — and that is load-bearing:

      1. the CURRENTLY SHIPPED defaults trip check A (min_turning_radius 0.15 <=
         half-track 0.1625), so raising would refuse to start navigation on every
         existing robot, turning a lawn-quality defect into a total outage;
      2. neither condition is a safety hazard at launch. The firmware remains the
         sole blade-safety authority and still owns the e-stop; what these degrade
         is mowing QUALITY, and a robot that refuses to mow is strictly worse than
         one that mows with poor turns while the operator reads the warning;
      3. the genuinely nonsensical inputs (radius <= 0, connector radius below the
         floor) are already contained by the clamps the caller applies before
         injecting, so no unrecoverable input survives to reach here.

    The bar for a hard failure is "cannot be made safe"; neither clears it. They
    ARE loud and they print the numbers, because the whole point is that the
    offending values live in different files and nothing previously related them.
    """
    warnings = []
    half_track = 0.5 * float(wheel_track)
    r_floor = float(min_turn_radius)
    if r_floor <= half_track:
        # Forward-only geometry: an arc of radius R needs the inner wheel at
        # v*(1 - half_track/R). At R == half_track that is exactly zero; below it
        # the inner wheel must REVERSE for the chassis to trace the arc. The
        # coverage path is built as cusp-free FORWARD turn-around arcs and FTC
        # runs forward_only, so nothing in the stack expects that — the wheel
        # reverses anyway, in the soil, and carves.
        v_out = turn_speed * (1.0 + half_track / max(r_floor, 1e-6))
        v_in = turn_speed * (1.0 - half_track / max(r_floor, 1e-6))
        warnings.append(
            "WARN: min_turning_radius={} m is at/below the half-track ({} / 2 = "
            "{:.4f} m). Every arc planned at that floor needs the INNER wheel at "
            "{:+.3f} m/s while the outer runs {:.3f} m/s — a reversing inner wheel "
            "carves the lawn at swath ends (issue #499). Raise "
            "mowgli_robot.yaml.min_turning_radius above {:.4f} m — but note that "
            "at the shipped headland apron the coverage server fits an arc at only "
            "~1 join in 32 anyway ('PlanCoverage connectors:'), so this alone will "
            "not change the swath-end turns.".format(
                r_floor, wheel_track, half_track, v_in, v_out, half_track))
    # Planner/controller consistency: the tightest arc FTC can COMMAND at the turn
    # speed is turn_speed / max_cmd_vel_ang. An arc tighter than that saturates the
    # angular command for its whole length, so the controller falls behind, the
    # heading error grows, and the chassis curls INSIDE the planned radius —
    # measured at 0.09-0.14 m against a 0.15-0.18 m plan.
    r_commandable = float(turn_speed) / max(float(max_cmd_vel_ang), 1e-6)
    planned_tightest = min(r_floor, float(connector_turn_radius))
    if planned_tightest < r_commandable:
        warnings.append(
            "WARN: coverage plans turn arcs down to {:.3f} m (min_turning_radius={}, "
            "connector_turn_radius={}) but the tightest arc FollowCoveragePath can "
            "command is speed_slow/max_cmd_vel_ang = {:.3f}/{} = {:.3f} m. Every "
            "swath-end turn will saturate the angular command for its full length "
            "and track inside the planned radius (issue #499). Raise the radii in "
            "mowgli_robot.yaml, or lower turn_speed_ratio.".format(
                planned_tightest, r_floor, connector_turn_radius, turn_speed,
                max_cmd_vel_ang, r_commandable))
    return warnings
