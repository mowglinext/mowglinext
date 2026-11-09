# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""
openmower_bridge.launch.py

Launches the OpenMower hardware bridge under the node name `hardware_bridge`
with the SAME topic remaps mowgli.launch.py applies to the STM32 bridge, so
the rest of the stack cannot tell the two apart.

Parameter layering (later wins):
  1. config/openmower_bridge.yaml            — package defaults (internal knobs)
  2. the mowgli_bringup robot TEMPLATE       — the same defaults the GUI and
     mowgli-ros2 use (Invariant 15), so "reset to default" in the GUI means
     the same value here
  3. config/backends/openmower.yaml          — OpenMower's own defaults for the
     shared keys that differ (ticks, the board's lift/tilt/charge values),
     layered exactly as robot_config_util does for mowgli-ros2
  4. OPENMOWER_* container environment       — ports, ESC type (installer)
  5. /config/mowgli_robot.yaml (sparse)      — the operator's file, written by
     the GUI. Only keys that are present are forwarded.
Without the template (or a key) a LowLevel board field stays -1 / "" (send
"undefined"): the board then keeps its own value.
"""

import os
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_CONFIG_PATH = os.environ.get("OPENMOWER_ROBOT_CONFIG", "/config/mowgli_robot.yaml")
# Copied into the image from ros2/src/mowgli_bringup/config/mowgli_robot.yaml.
ROBOT_TEMPLATE_PATH = os.environ.get(
    "OPENMOWER_ROBOT_TEMPLATE", "/opt/openmower_bridge/mowgli_robot.template.yaml"
)
# Copied from ros2/src/mowgli_bringup/config/backends/openmower.yaml.
BACKEND_DEFAULTS_PATH = os.environ.get(
    "OPENMOWER_BACKEND_DEFAULTS", "/opt/openmower_bridge/backend_defaults.yaml"
)
XESC_TYPES = ("xesc_mini", "xesc_2040")

# mowgli_robot.yaml key -> (node parameter, caster)
#
# A setting MowgliNext already has keeps its MowgliNext key: the GUI edits one
# field whatever the backend. The LowLevel board's own configuration is fed
# from those same keys; an OpenMower install seeds OpenMower's factory values
# for them (install/lib/config.sh, gui/web/src/constants/hardwareBackends.ts),
# and lowlevel_config.hpp refuses anything outside the STM32 firmware's
# envelope, since the Pico saves what it receives without clamping. Only what
# has no MowgliNext equivalent gets an openmower_* key.
ROBOT_CONFIG_KEYS = {
    # wiring (the installer's docker/.env sits between template and operator)
    "openmower_ll_port": ("ll_serial_port", str),
    "openmower_xesc_type": ("xesc_type", str),
    "openmower_xesc_left_port": ("left_xesc_port", str),
    "openmower_xesc_right_port": ("right_xesc_port", str),
    "openmower_xesc_mow_port": ("mow_xesc_port", str),
    # host side
    "ticks_per_meter": ("ticks_per_meter", float),
    "wheel_track": ("wheel_track", float),
    "max_mps": ("max_mps", float),
    "mowing_enabled": ("mowing_enabled", bool),
    "imu_cal_samples": ("imu_cal_samples", int),
    "imu_cal_auto_rest_sec": ("imu_cal_auto_rest_sec", float),
    # host-side wheel loop: the xESC takes a duty cycle, so the STM32's PWM
    # gains (wheel_pid_*) do not transfer
    "openmower_wheel_loop_enabled": ("wheel_loop_enabled", bool),
    "openmower_wheel_duty_per_mps": ("wheel_duty_per_mps", float),
    "openmower_wheel_kp": ("wheel_kp", float),
    "openmower_wheel_ki": ("wheel_ki", float),
    # LowLevel board configuration, from the shared MowgliNext settings
    "max_charge_current": ("ll_i_charge_cutoff", float),
    "max_charge_voltage": ("ll_v_battery_cutoff", float),
    "battery_empty_voltage": ("ll_v_battery_empty", float),
    "battery_full_voltage": ("ll_v_battery_full", float),
    "both_wheels_lift_emergency_ms": ("ll_lift_period_ms", int),
    "one_wheel_lift_emergency_ms": ("ll_tilt_period_ms", int),
    "openmower_emergency_input_config": ("emergency_input_config", str),
}


def _read_robot_params(path: str) -> Dict[str, Any]:
    """mowgli.ros__parameters of a robot config file ({} if absent/unreadable)."""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
    except FileNotFoundError:
        return {}
    except (OSError, yaml.YAMLError) as exc:  # noqa: PERF203
        print(f"[openmower_bridge] cannot read {path}: {exc}; using defaults")
        return {}
    params = raw.get("mowgli", {}).get("ros__parameters", {}) if isinstance(raw, dict) else {}
    return params if isinstance(params, dict) else {}


def _cast_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.strip().lower() in ("true", "false"):
        return value.strip().lower() == "true"
    raise ValueError(value)


def robot_config_overrides(params: Dict[str, Any], source: str) -> Dict[str, Any]:
    """Robot-config keys -> node parameters, only for keys present and valid."""
    out: Dict[str, Any] = {}
    for key, (param, cast) in ROBOT_CONFIG_KEYS.items():
        if key not in params or params[key] is None:
            continue
        caster = _cast_bool if cast is bool else cast
        try:
            value = caster(params[key])
        except (TypeError, ValueError):
            print(f"[openmower_bridge] {source}: ignoring non-{cast.__name__} {key}={params[key]!r}")
            continue
        if param == "xesc_type" and value not in XESC_TYPES:
            print(f"[openmower_bridge] {source}: ignoring {key}={value!r} (not {'|'.join(XESC_TYPES)})")
            continue
        if cast is str and param.endswith("_port") and not value.strip():
            continue
        out[param] = value
    return out


def env_overrides() -> Dict[str, Any]:
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
        if not value:
            continue
        if param == "xesc_type" and value not in XESC_TYPES:
            print(f"[openmower_bridge] ignoring {env}={value!r} (not {'|'.join(XESC_TYPES)})")
            continue
        out[param] = cast(value)
    mow_enabled = os.environ.get("OPENMOWER_MOW_XESC_ENABLED", "").strip().lower()
    if mow_enabled in ("true", "false"):
        out["mow_xesc_enabled"] = mow_enabled == "true"
    return out


def layered_parameters(defaults: str, template_path: str, backend_path: str,
                       config_path: str) -> List[Any]:
    """The node's parameter layers, later wins (see the module docstring)."""
    for what, path in (("robot template", template_path), ("backend defaults", backend_path)):
        if not os.path.isfile(path):
            print(f"[openmower_bridge] {what} {path} missing; "
                  "those defaults fall back to the package defaults")
    return [
        defaults,
        robot_config_overrides(_read_robot_params(template_path), "template"),
        robot_config_overrides(_read_robot_params(backend_path), "backend defaults"),
        env_overrides(),
        robot_config_overrides(_read_robot_params(config_path), config_path),
    ]


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("mowgli_openmower_bridge")
    defaults = os.path.join(share, "config", "openmower_bridge.yaml")

    node = Node(
        package="mowgli_openmower_bridge",
        executable="openmower_bridge_node",
        name="hardware_bridge",
        output="screen",
        parameters=layered_parameters(
            defaults, ROBOT_TEMPLATE_PATH, BACKEND_DEFAULTS_PATH, ROBOT_CONFIG_PATH
        ),
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
