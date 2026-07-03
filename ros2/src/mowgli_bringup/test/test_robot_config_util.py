# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
#
# Unit tests for robot_config_util.load_robot_params — the deep-merge that
# lets the INSTALLED mowgli_robot.yaml be sparse (install choices + calibration
# outputs only) while every other default falls through to the in-package
# template. These run without any ROS deps — only PyYAML is required.

import importlib.util
import os
import sys
import tempfile
from pathlib import Path

import yaml

# mowgli_bringup package root: test/ -> package dir; the template lives at
# <pkg>/config/mowgli_robot.yaml and the helper at <pkg>/launch/.
_PKG_DIR = Path(__file__).resolve().parent.parent
_LAUNCH_DIR = _PKG_DIR / "launch"
_TEMPLATE_PATH = _PKG_DIR / "config" / "mowgli_robot.yaml"


def _load_helper():
    """Import robot_config_util.py directly from the launch dir (no ROS)."""
    sys.path.insert(0, str(_LAUNCH_DIR))
    spec = importlib.util.spec_from_file_location(
        "robot_config_util", str(_LAUNCH_DIR / "robot_config_util.py"))
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_util = _load_helper()
load_robot_params = _util.load_robot_params


def _template_params() -> dict:
    with open(_TEMPLATE_PATH, "r") as handle:
        doc = yaml.safe_load(handle) or {}
    return doc.get("mowgli", {}).get("ros__parameters", {})


def _write_sparse(params: dict) -> str:
    """Write a sparse runtime config to a temp file, return its path."""
    tmp = tempfile.NamedTemporaryFile(
        mode="w", prefix="test_robot_cfg_", suffix=".yaml", delete=False)
    yaml.safe_dump({"mowgli": {"ros__parameters": params}}, tmp)
    tmp.close()
    return tmp.name


def test_template_only_returns_full_defaults():
    """A nonexistent runtime path yields the pure template defaults."""
    template = _template_params()
    assert template, "template must be non-empty for this test to mean anything"

    merged = load_robot_params(
        str(_PKG_DIR), runtime_path="/nonexistent/mowgli_robot.yaml")
    assert merged == template


def test_sparse_overrides_and_falls_through():
    """Sparse runtime overrides its own keys; absent keys use the template."""
    template = _template_params()
    # Pick a key present in the template to override.
    assert "ticks_per_meter" in template
    original = template["ticks_per_meter"]
    override_val = float(original) + 111.0

    path = _write_sparse({
        "ticks_per_meter": override_val,
        "datum_lat": 48.123456,
        # An install-decided key that is ABSENT from the template — it must
        # still surface (and its presence is what launch files test for).
        "lidar_enabled": False,
    })
    try:
        merged = load_robot_params(str(_PKG_DIR), runtime_path=path)
    finally:
        os.unlink(path)

    # Overridden keys win.
    assert merged["ticks_per_meter"] == override_val
    assert merged["datum_lat"] == 48.123456
    # Install-decided key surfaces even though the template lacks it.
    assert "lidar_enabled" in merged
    assert merged["lidar_enabled"] is False
    # Untouched keys fall through to the template default.
    for key, value in template.items():
        if key in ("ticks_per_meter", "datum_lat"):
            continue
        assert merged[key] == value


def test_removing_key_reverts_to_template_default():
    """Reset-to-default: dropping a key from the sparse runtime reverts it."""
    template = _template_params()
    assert "ticks_per_meter" in template
    default_val = template["ticks_per_meter"]

    # Runtime that DOES override the key.
    with_key = _write_sparse({"ticks_per_meter": float(default_val) + 55.0})
    # Runtime that omits it entirely (simulates the GUI deleting the key).
    without_key = _write_sparse({"datum_lat": 1.0})
    try:
        merged_with = load_robot_params(str(_PKG_DIR), runtime_path=with_key)
        merged_without = load_robot_params(
            str(_PKG_DIR), runtime_path=without_key)
    finally:
        os.unlink(with_key)
        os.unlink(without_key)

    assert merged_with["ticks_per_meter"] == float(default_val) + 55.0
    # Key absent from runtime -> template default restored.
    assert merged_without["ticks_per_meter"] == default_val


def test_full_runtime_merges_to_itself():
    """Backward compat: a FULL runtime config (every template key) is a no-op
    merge — the merged result equals that full runtime for every template key."""
    template = _template_params()
    # Build a full runtime = template with every value bumped, so we can prove
    # each one wins over the identical-key template default.
    full = {}
    for key, value in template.items():
        if isinstance(value, bool):
            full[key] = not value
        elif isinstance(value, (int, float)):
            full[key] = value + 1
        else:
            full[key] = value  # strings/lists kept as-is

    path = _write_sparse(full)
    try:
        merged = load_robot_params(str(_PKG_DIR), runtime_path=path)
    finally:
        os.unlink(path)

    for key in template:
        assert merged[key] == full[key]
