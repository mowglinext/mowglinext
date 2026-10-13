# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
"""
airseekers_bridge.launch.py

Launches the Airseekers Tron hardware bridge under the node name
`hardware_bridge` with the SAME topic remaps mowgli.launch.py applies to the
STM32 bridge, plus (unless the Universal GNSS sidecar owns the receiver) the
UM960 GNSS node publishing /gps/fix and /gps/status.

Parameter layering (later wins), as in the OpenMower bridge:
  1. config/airseekers_bridge.yaml           — package defaults (internal knobs)
  2. the mowgli_bringup robot TEMPLATE       — the defaults the GUI and
     mowgli-ros2 use (Invariant 15)
  3. config/backends/airseekers.yaml         — the Tron's defaults for shared
     keys (wheel_track, max_mps, battery voltages, GNSS port, ...)
  4. AIRSEEKERS_* container environment      — device paths (installer)
  5. /config/mowgli_robot.yaml (sparse)      — the operator's file (GUI)
"""

import os
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_CONFIG_PATH = os.environ.get("AIRSEEKERS_ROBOT_CONFIG", "/config/mowgli_robot.yaml")
ROBOT_TEMPLATE_PATH = os.environ.get(
    "AIRSEEKERS_ROBOT_TEMPLATE", "/opt/airseekers_bridge/mowgli_robot.template.yaml")
BACKEND_DEFAULTS_PATH = os.environ.get(
    "AIRSEEKERS_BACKEND_DEFAULTS", "/opt/airseekers_bridge/backend_defaults.yaml")

# mowgli_robot.yaml key -> (bridge node parameter, caster). A setting
# MowgliNext already has keeps its key; only what has no equivalent gets an
# airseekers_* key.
BRIDGE_KEYS = {
    "airseekers_mcu_port": ("mcu_port", str),
    "airseekers_imu_port": ("imu_port", str),
    "airseekers_keys_device": ("keys_device", str),
    "airseekers_max_angular_radps": ("max_angular_radps", float),
    "airseekers_cutter_height_mm": ("cutter_height_mm", int),
    "airseekers_angular_trim_radps": ("angular_trim_radps", float),
    "airseekers_angular_deadband_radps": ("angular_deadband_radps", float),
    "airseekers_heading_hold": ("heading_hold", bool),
    "airseekers_heading_hold_kp": ("heading_hold_kp", float),
    "airseekers_heading_hold_kd": ("heading_hold_kd", float),
    "airseekers_heading_hold_max_radps": ("heading_hold_max_radps", float),
    "airseekers_imu_roll_offset_deg": ("forward_imu_roll_offset_deg", float),
    "airseekers_imu_pitch_offset_deg": ("forward_imu_pitch_offset_deg", float),
    "wheel_track": ("wheel_track", float),
    "ticks_per_meter": ("ticks_per_meter", float),
    "max_mps": ("max_mps", float),
    "mowing_enabled": ("mowing_enabled", bool),
    "imu_cal_samples": ("imu_cal_samples", int),
    "imu_cal_auto_rest_sec": ("imu_cal_auto_rest_sec", float),
    "tilt_emergency_ms": ("tilt_emergency_ms", int),
}

# mowgli_robot.yaml key -> (um960 node parameter, caster)
GNSS_KEYS = {
    "gnss_serial_device": ("port", str),
    "gnss_serial_baud": ("baud", int),
    "gnss_frame_id": ("frame_id", str),
    "ntrip_host": ("ntrip_host", str),
    "ntrip_port": ("ntrip_port", int),
    "ntrip_user": ("ntrip_user", str),
    "ntrip_password": ("ntrip_password", str),
    "ntrip_mountpoint": ("ntrip_mountpoint", str),
}

ENV_KEYS = {
    "AIRSEEKERS_MCU_PORT": ("mcu_port", str),
    "AIRSEEKERS_IMU_PORT": ("imu_port", str),
}


def read_robot_params(path: str) -> Dict[str, Any]:
    """mowgli.ros__parameters of a robot config file ({} if absent/unreadable)."""
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh) or {}
    except FileNotFoundError:
        return {}
    except (OSError, yaml.YAMLError) as exc:
        print(f"[airseekers_bridge] cannot read {path}: {exc}; using defaults")
        return {}
    params = raw.get("mowgli", {}).get("ros__parameters", {}) if isinstance(raw, dict) else {}
    return params if isinstance(params, dict) else {}


def _cast_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.strip().lower() in ("true", "false"):
        return value.strip().lower() == "true"
    raise ValueError(value)


def map_keys(params: Dict[str, Any], table: Dict[str, Any], source: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for key, (param, cast) in table.items():
        if key not in params or params[key] is None:
            continue
        caster = _cast_bool if cast is bool else cast
        try:
            value = caster(params[key])
        except (TypeError, ValueError):
            print(f"[airseekers_bridge] {source}: ignoring non-{cast.__name__} {key}={params[key]!r}")
            continue
        if cast is str and (param.endswith("port") or param == "port") and not value.strip():
            continue
        out[param] = value
    return out


def env_overrides(environ=None) -> Dict[str, Any]:
    environ = os.environ if environ is None else environ
    return {param: cast(environ[env].strip()) for env, (param, cast) in ENV_KEYS.items()
            if environ.get(env, "").strip()}


def merged(template_path: str, backend_path: str, config_path: str) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    for path in (template_path, backend_path, config_path):
        out.update(read_robot_params(path))
    return out


def bridge_parameters(defaults: str, template_path: str, backend_path: str, config_path: str,
                      environ=None) -> List[Any]:
    for what, path in (("robot template", template_path), ("backend defaults", backend_path)):
        if not os.path.isfile(path):
            print(f"[airseekers_bridge] {what} {path} missing; package defaults apply")
    return [
        defaults,
        map_keys(read_robot_params(template_path), BRIDGE_KEYS, "template"),
        map_keys(read_robot_params(backend_path), BRIDGE_KEYS, "backend defaults"),
        env_overrides(environ),
        map_keys(read_robot_params(config_path), BRIDGE_KEYS, config_path),
    ]


def gnss_enabled(cfg: Dict[str, Any]) -> bool:
    """The sidecar owns the receiver unless the Universal GNSS sidecar does
    (gnss_stack "universal"); the Airseekers backend defaults to "disabled",
    the template's value for "a backend that publishes /gps/* itself"."""
    return str(cfg.get("gnss_stack", "universal")).strip().lower() != "universal"


def gnss_parameters(cfg: Dict[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "fix_topic": "/gps/fix",
        "status_topic": "/gps/status",
        "fix_status_topic": "/gps/fix_status",
        # A single-antenna UM960: never publish a heading.
        "publish_rtk_heading": False,
        "heading_topic": "~/heading",
        "heading_valid_topic": "~/heading_valid",
        "vel_topic": "~/vel",
        "nmea_topic": "~/nmea",
        "ntrip_status_topic": "~/ntrip_status",
        "corrections_topic": "/gps/corrections",
        # The robot config is the source of truth; nothing is persisted here.
        "ntrip_settings_file": "",
        "ntrip_config_file": "",
    }
    out.update(map_keys(cfg, GNSS_KEYS, "robot config"))
    try:
        ntrip = _cast_bool(cfg.get("ntrip_enabled", False))
    except ValueError:
        ntrip = False
    # The GUI NTRIP switch: on = network RTK through the host client, off =
    # the vendor LoRa base station (the rover board handles it alone).
    out["correction_source"] = "ntrip" if ntrip else "lora"
    out["ntrip_enabled"] = ntrip
    return out


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("mowgli_airseekers_bridge")
    defaults = os.path.join(share, "config", "airseekers_bridge.yaml")
    bridge = Node(
        package="mowgli_airseekers_bridge",
        executable="airseekers_bridge_node",
        name="hardware_bridge",
        output="screen",
        parameters=bridge_parameters(defaults, ROBOT_TEMPLATE_PATH, BACKEND_DEFAULTS_PATH,
                                     ROBOT_CONFIG_PATH),
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
    actions = [bridge]
    cfg = merged(ROBOT_TEMPLATE_PATH, BACKEND_DEFAULTS_PATH, ROBOT_CONFIG_PATH)
    if gnss_enabled(cfg):
        actions.append(Node(
            package="um960_gps_driver",
            executable="um960_node",
            name="um960_gps_node",
            output="screen",
            parameters=[gnss_parameters(cfg)],
        ))
    else:
        print("[airseekers_bridge] gnss_stack=universal: the Universal GNSS sidecar owns the "
              "receiver, the UM960 node is not started")
    return LaunchDescription(actions)
