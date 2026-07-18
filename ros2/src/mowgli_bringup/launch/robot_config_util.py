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
