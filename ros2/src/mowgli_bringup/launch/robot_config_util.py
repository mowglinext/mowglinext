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

import os

import yaml

DEFAULT_RUNTIME_PATH = "/ros2_ws/config/mowgli_robot.yaml"


def _deep_merge(base, override):
    """Recursively merge ``override`` into a copy of ``base`` (override wins).

    Only dict-vs-dict collisions recurse; any scalar/list in ``override``
    replaces the base value wholesale (robot params are flat scalars, so this
    is just the nested ros__parameters block being merged key-by-key).
    """
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
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
    return _deep_merge(template, runtime)


def load_robot_params(bringup_dir, runtime_path=DEFAULT_RUNTIME_PATH):
    """Return the merged mowgli.ros__parameters dict the launch files inject.

    Always complete: every template key is present, with installed-config
    values layered on top. Missing runtime file -> pure template defaults.
    """
    cfg = load_robot_config(bringup_dir, runtime_path)
    return cfg.get("mowgli", {}).get("ros__parameters", {})
