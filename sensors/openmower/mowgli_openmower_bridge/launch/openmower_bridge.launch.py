# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""
openmower_bridge.launch.py

Launches the OpenMower hardware bridge under the node name `hardware_bridge`
with the SAME topic remaps mowgli.launch.py applies to the STM32 bridge, so
the rest of the stack cannot tell the two apart.

Parameter layering (later wins):
  1. config/openmower_bridge.yaml            — package defaults
  2. OPENMOWER_* container environment       — ports, ESC type (installer)
  3. /config/mowgli_robot.yaml (sparse)      — per-robot kinematics, blade
     inhibit, charge limits: the operator's file, read directly since this
     container does not ship the mowgli_bringup template. Only keys that are
     present are forwarded; absent keys keep the package default.
"""

import os
from typing import Any, Dict

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_CONFIG_PATH = os.environ.get("OPENMOWER_ROBOT_CONFIG", "/config/mowgli_robot.yaml")

# mowgli_robot.yaml key -> (node parameter, caster)
ROBOT_CONFIG_KEYS = {
    "ticks_per_meter": ("ticks_per_meter", float),
    "wheel_track": ("wheel_track", float),
    "max_mps": ("max_mps", float),
    "mowing_enabled": ("mowing_enabled", bool),
    "max_charge_voltage": ("max_charge_voltage", float),
    "max_charge_current": ("max_charge_current", float),
    "battery_full_voltage": ("battery_full_voltage", float),
    "battery_empty_voltage": ("battery_empty_voltage", float),
    "one_wheel_lift_emergency_ms": ("one_wheel_lift_emergency_ms", int),
    "both_wheels_lift_emergency_ms": ("both_wheels_lift_emergency_ms", int),
    "imu_cal_samples": ("imu_cal_samples", int),
    "imu_cal_auto_rest_sec": ("imu_cal_auto_rest_sec", float),
    "openmower_wheel_loop_enabled": ("wheel_loop_enabled", bool),
    "openmower_wheel_duty_per_mps": ("wheel_duty_per_mps", float),
    "openmower_wheel_kp": ("wheel_kp", float),
    "openmower_wheel_ki": ("wheel_ki", float),
    "openmower_blade_duty": ("blade_duty", float),
    "openmower_emergency_input_config": ("emergency_input_config", str),
}


def _robot_config_overrides(path: str) -> Dict[str, Any]:
    """Sparse mowgli_robot.yaml -> node parameters, only for keys present."""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
    except FileNotFoundError:
        return {}
    except (OSError, yaml.YAMLError) as exc:  # noqa: PERF203
        print(f"[openmower_bridge] cannot read {path}: {exc}; using defaults")
        return {}
    params = raw.get("mowgli", {}).get("ros__parameters", {}) if isinstance(raw, dict) else {}
    if not isinstance(params, dict):
        return {}
    out: Dict[str, Any] = {}
    for key, (param, cast) in ROBOT_CONFIG_KEYS.items():
        if key not in params or params[key] is None:
            continue
        try:
            out[param] = cast(params[key])
        except (TypeError, ValueError):
            print(f"[openmower_bridge] ignoring non-{cast.__name__} {key}={params[key]!r}")
    return out


def _env_overrides() -> Dict[str, Any]:
    """Installer-owned device selection from the container environment."""
    out: Dict[str, Any] = {}
    mapping = {
        "OPENMOWER_LL_PORT": ("ll_serial_port", str),
        "OPENMOWER_XESC_TYPE": ("xesc_type", str),
        "OPENMOWER_XESC_LEFT_PORT": ("left_xesc_port", str),
        "OPENMOWER_XESC_RIGHT_PORT": ("right_xesc_port", str),
        "OPENMOWER_XESC_MOW_PORT": ("mow_xesc_port", str),
    }
    for env, (param, cast) in mapping.items():
        value = os.environ.get(env, "").strip()
        if value:
            out[param] = cast(value)
    mow_enabled = os.environ.get("OPENMOWER_MOW_XESC_ENABLED", "").strip().lower()
    if mow_enabled in ("true", "false"):
        out["mow_xesc_enabled"] = mow_enabled == "true"
    return out


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("mowgli_openmower_bridge")
    defaults = os.path.join(share, "config", "openmower_bridge.yaml")

    node = Node(
        package="mowgli_openmower_bridge",
        executable="openmower_bridge_node",
        name="hardware_bridge",
        output="screen",
        parameters=[
            defaults,
            _env_overrides(),
            _robot_config_overrides(ROBOT_CONFIG_PATH),
        ],
        # Identical to mowgli.launch.py's hardware_bridge remaps.
        remappings=[
            ("~/imu/data_raw", "/imu/data"),
            ("~/imu/mag_raw", "/imu/mag_raw"),
            ("~/wheel_odom", "/wheel_odom"),
            ("~/wheel_ticks", "/wheel_ticks"),
            ("~/emergency", "/hardware_bridge/emergency"),
            ("~/power", "/hardware_bridge/power"),
            ("~/status", "/hardware_bridge/status"),
            ("~/cmd_vel", "/cmd_vel"),
        ],
    )
    return LaunchDescription([node])
