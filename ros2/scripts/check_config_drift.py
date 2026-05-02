#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""check_config_drift.py — detect divergence between mowgli_robot.yaml copies.

Background
----------
The repo carries (at least) two mowgli_robot.yaml files:

  ros2/src/mowgli_bringup/config/mowgli_robot.yaml  — committed template
  install/config/mowgli/mowgli_robot.yaml           — runtime (mounted into
                                                       the production container,
                                                       managed by the GUI)

The runtime file is what actually drives the robot at deploy time. The
template is read by `navigation.launch.py` to compute the Nav2 footprint
and by simulation/test harnesses. Drift between the two on STRUCTURAL
fields (chassis dimensions, wheel kinematics, sensor mounts) silently
gives Nav2 the wrong footprint, the URDF the wrong sensor offsets, etc.
That kind of inconsistency is invisible at runtime and bites at the worst
moment — during a coverage path, the inflation may not match the chassis
the robot is actually presenting to the world.

What this script does
---------------------
- Loads both files.
- For each STRUCTURAL field, asserts they match.
- For each USER_OVERRIDE field, expects them to differ (or only exist
  in the runtime file) and is silent.
- Prints a diff and exits non-zero if any structural field has drifted.

User-tunable fields (datum, dock pose, IMU mounting calibration, GPS
ports, NTRIP creds…) are intentionally allowed to differ — those *should*
only live in the runtime file.

Run as part of CI; or manually before / after editing either yaml.
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[2]
TEMPLATE = REPO / "ros2/src/mowgli_bringup/config/mowgli_robot.yaml"
RUNTIME = REPO / "install/config/mowgli/mowgli_robot.yaml"

# Fields that describe the physical robot — must match across both files.
# These are read by URDF generation, hardware bridge, Nav2 footprint, and
# coverage planning, so any divergence will give different parts of the
# stack different mental models of the robot.
STRUCTURAL = {
    # Drivetrain
    "wheel_radius", "wheel_width", "wheel_track", "wheel_x_offset",
    "ticks_per_meter", "ticks_per_revolution",
    # Chassis
    "chassis_length", "chassis_width", "chassis_height",
    "chassis_center_x", "chassis_mass_kg",
    # Casters
    "caster_radius", "caster_track",
    # Mowing tool
    "blade_radius", "tool_width",
    # LiDAR mount (overrides URDF defaults)
    "lidar_x", "lidar_y", "lidar_z", "lidar_yaw",
    # Mower model identifier
    "mower_model",
}

# Fields the operator legitimately tunes per-deployment. We do NOT compare.
USER_OVERRIDE = {
    "datum_lat", "datum_lon", "datum_alt",
    "dock_pose_x", "dock_pose_y", "dock_pose_yaw",
    "imu_pitch", "imu_roll", "imu_yaw", "imu_x", "imu_y", "imu_z",
    "gps_antenna_x", "gps_antenna_y", "gps_x", "gps_y", "gps_z",
    "gps_port", "gps_baudrate", "gps_protocol", "gps_timeout_sec",
    "gps_wait_after_undock_sec",
    "ntrip_enabled", "ntrip_host", "ntrip_port",
    "ntrip_user", "ntrip_password", "ntrip_mountpoint",
    "transit_speed", "mowing_speed", "undock_speed", "undock_distance",
    "mowing_enabled", "automatic_mode", "rain_mode", "rain_debounce_sec",
    "rain_delay_minutes",
    "lift_recovery_mode", "lift_blade_resume_delay_sec",
    "emergency_stop_on_lift", "emergency_stop_on_tilt",
    "use_lidar", "lidar_enabled",
    "map_save_on_dock", "map_save_path",
    "min_turning_radius", "headland_width",
    "outline_offset", "outline_overlap", "outline_passes",
    "path_spacing", "mow_angle_offset_deg", "mow_angle_increment_deg",
    "max_obstacle_avoidance_distance", "coverage_xy_tolerance",
    "xy_goal_tolerance", "yaw_goal_tolerance",
    "progress_timeout_sec",
    "motor_temp_low_c", "motor_temp_high_c",
    "battery_full_voltage", "battery_low_voltage",
    "dock_approach_distance", "dock_max_retries", "dock_use_charger_detection",
}


def load(p: Path) -> dict:
    with p.open() as f:
        data = yaml.safe_load(f) or {}
    return data.get("mowgli", {}).get("ros__parameters", {})


def main() -> int:
    if not TEMPLATE.is_file():
        print(f"FATAL: template not found: {TEMPLATE}", file=sys.stderr)
        return 2
    if not RUNTIME.is_file():
        print(f"FATAL: runtime not found: {RUNTIME}", file=sys.stderr)
        return 2

    tpl = load(TEMPLATE)
    rt = load(RUNTIME)

    # Structural drift: this is the bug we're guarding against.
    drift = []
    for k in sorted(STRUCTURAL):
        t = tpl.get(k, "<MISSING>")
        r = rt.get(k, "<MISSING>")
        if t != r:
            drift.append((k, t, r))

    # Surface unexpected keys — fields in only one file that aren't on
    # either list. Useful to catch typos and orphans.
    unexpected_in_runtime = sorted(
        k for k in rt
        if k not in tpl and k not in STRUCTURAL and k not in USER_OVERRIDE
    )
    unexpected_in_template = sorted(
        k for k in tpl
        if k not in rt and k not in STRUCTURAL and k not in USER_OVERRIDE
    )

    print(f"Template: {TEMPLATE.relative_to(REPO)}")
    print(f"Runtime : {RUNTIME.relative_to(REPO)}")
    print()

    if drift:
        print(f"STRUCTURAL DRIFT ({len(drift)} field(s)):")
        for k, t, r in drift:
            print(f"  {k}: template={t}  runtime={r}")
        print()
    else:
        print("Structural fields: in sync.")
        print()

    if unexpected_in_runtime:
        print("Keys in runtime not categorized (treat as orphans?):")
        for k in unexpected_in_runtime:
            print(f"  {k} = {rt[k]}")
        print()

    if unexpected_in_template:
        print("Keys in template not categorized (treat as orphans?):")
        for k in unexpected_in_template:
            print(f"  {k} = {tpl[k]}")
        print()

    return 1 if drift else 0


if __name__ == "__main__":
    sys.exit(main())
