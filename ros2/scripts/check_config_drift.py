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

The installed file is SPARSE (Architecture Invariant 15): it holds only
install-time choices + calibration outputs, and `load_robot_params`
(mowgli_bringup/launch/robot_config_util.py) deep-merges it OVER the
template so any absent key inherits its template default. STRUCTURAL
fields (chassis dimensions, wheel kinematics, sensor mounts) normally
live ONLY in the template; a structural field that also appears in the
installed file with a DIFFERENT value gives Nav2 the wrong footprint, the
URDF the wrong sensor offsets, etc. That kind of inconsistency is
invisible at runtime and bites at the worst moment — during a coverage
path, the inflation may not match the chassis the robot is actually
presenting to the world.

What this script does
---------------------
- Loads both files.
- For each STRUCTURAL field, flags DRIFT only when the field is present in
  BOTH files with differing values. A structural field that lives only in
  the template (absent from the sparse installed file) is the EXPECTED,
  healthy state and is NOT flagged.
- Flags a structural field present in the installed file but with NO
  template default — it breaks GUI "reset to default" and the
  propagate-a-new-default property (Invariant 15).
- For each USER_OVERRIDE / calibration-output field, expects them to
  differ (or only exist in the installed file) and is silent.
- Prints a diff and exits non-zero if any structural field has drifted.

User-tunable fields (datum, dock pose, IMU mounting calibration, GNSS
transport, NTRIP creds…) and calibration outputs (ticks_per_meter, wheel
PID gains, magnetometer) are intentionally allowed to differ / live only
in the installed file.

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
    "gnss_receiver_family", "gnss_serial_device", "gnss_serial_baud",
    "gps_timeout_sec",
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
    "obstacle_inflation_radius", "obstacle_margin", "obstacle_slowdown_ratio",
    "xy_goal_tolerance", "yaw_goal_tolerance",
    "progress_timeout_sec",
    "motor_temp_low_c", "motor_temp_high_c",
    "battery_full_voltage", "battery_low_voltage",
    "dock_approach_distance", "dock_max_retries", "dock_use_charger_detection",
}

# Per-robot calibration outputs written back into the sparse installed file
# (Invariant 15). These legitimately live only in the installed file and are
# expected to differ from any template seed — never treated as drift or orphans.
CALIBRATION_OUTPUT = {
    "ticks_per_meter",
    "wheel_pid_kp", "wheel_pid_ki", "wheel_pid_kd",
    "wheel_pid_integral_limit", "wheel_pid_pwm_per_mps",
    "imu_yaw", "declination_deg", "enable_mag_cal", "mag_calibration_path",
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

    # Structural drift under the sparse-config model (Invariant 15). The
    # installed file inherits every absent key from the template via
    # load_robot_params' deep-merge, so a structural field that lives ONLY in
    # the template is the healthy, expected state — NOT drift. We flag:
    #   * drift        — field present in BOTH files with differing values.
    #   * no_default   — structural field present ONLY in the installed file
    #                    (no template default → breaks reset-to-default).
    # Calibration outputs (ticks_per_meter, wheel PID…) are excluded: they are
    # per-robot and legitimately diverge from any template seed.
    drift = []
    no_default = []
    for k in sorted(STRUCTURAL):
        if k in CALIBRATION_OUTPUT:
            continue
        in_tpl, in_rt = k in tpl, k in rt
        if in_tpl and in_rt and tpl[k] != rt[k]:
            drift.append((k, tpl[k], rt[k]))
        elif in_rt and not in_tpl:
            no_default.append((k, rt[k]))

    # Surface orphan keys in the installed file — not on any known list and
    # absent from the template. Likely a typo or a stray default. Keys that
    # exist only in the template are expected (sparse-config defaults), so we
    # do NOT report those.
    known = STRUCTURAL | USER_OVERRIDE | CALIBRATION_OUTPUT
    unexpected_in_runtime = sorted(
        k for k in rt if k not in tpl and k not in known
    )

    print(f"Template: {TEMPLATE.relative_to(REPO)}")
    print(f"Runtime : {RUNTIME.relative_to(REPO)}")
    print()

    if drift:
        print(f"STRUCTURAL DRIFT ({len(drift)} field(s)):")
        for k, t, r in drift:
            print(f"  {k}: template={t}  installed={r}")
        print()
    if no_default:
        print(f"STRUCTURAL FIELD WITHOUT TEMPLATE DEFAULT ({len(no_default)}):")
        for k, r in no_default:
            print(f"  {k}: installed={r}  (add a default to the template)")
        print()
    if not drift and not no_default:
        print("Structural fields: in sync.")
        print()

    if unexpected_in_runtime:
        print("Keys in installed file not categorized (treat as orphans?):")
        for k in unexpected_in_runtime:
            print(f"  {k} = {rt[k]}")
        print()

    return 1 if (drift or no_default) else 0


if __name__ == "__main__":
    sys.exit(main())
