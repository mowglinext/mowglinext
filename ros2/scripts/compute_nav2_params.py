#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""compute_nav2_params.py — derive Nav2 controller/costmap/collision-monitor/
coverage parameters from the robot's PHYSICS and the web-UI-configurable
physical inputs, instead of hand-tuned magic constants.

WHY THIS EXISTS
---------------
Most of the values in nav2_params_base.yaml are not free — they are functions
of the chassis geometry, the firmware wheel-velocity limits, the drive-motor
deadband, and the operator-chosen mowing/transit speeds. Those physical inputs
already live in ``mowgli_robot.yaml`` and are editable from the GUI. When an
operator changes ``chassis_width`` or ``mowing_speed`` the dependent Nav2
params should move WITH them — yet today they are static literals that have to
be hand-re-tuned, which is the trial-and-error loop this tool replaces.

This script encodes the kinematics and dynamics behind each derived parameter
(documented inline + in the design note ``compute_nav2_params.md``):

  1. ``--report``  : print every derived value WITH its formula + rationale.
  2. ``--yaml``    : emit a YAML fragment mirroring the nav2 params tree, so
                     the derived values can be diffed against the current
                     hand-tuned ones (or fed into the launch injector later).
  3. ``--compare`` : deep-merge nav2_params_base.yaml with the chosen
                     lidar/no_lidar overlay (same algorithm as
                     navigation.launch.py) and tabulate derived-vs-current,
                     flagging divergences and HARD physics violations.

It is read-only and idempotent: it never touches running config, the launch
files, or the nav2 YAMLs. Stdlib + PyYAML only.

PROFILES
--------
Two derivation profiles, selectable with ``--profile``:

  calm        — anti-oscillation bias. wz_max respects the per-wheel
                saturation coupling at FULL mowing speed (no silent firmware
                clipping, MPPI's DiffDrive model always matches the plant);
                exploration noise sized for clean straights.
  responsive  — faster heading correction. Accepts brief per-wheel clipping
                (bounded by --clip-fraction) during max-speed turns; wider
                exploration noise so MPPI recovers large cross-track errors
                faster. Matches the 2026-06-12 field direction.

A/B the two in the field with mow_session_monitor; neither violates hard
physics (those are clamps, not suggestions, in both profiles).

INPUTS
------
From ``mowgli_robot.yaml`` (mowgli.ros__parameters):
  mower_model                — selects the drive-motor spec row (MODEL_SPECS)
  chassis_*  , wheel_track   — footprint rectangle, diff-drive pivot rate
  chassis_mass_kg            — acceleration budget (F = m·a)
  tool_width, swath_overlap  — F2C swath spacing (operation_width)
  mowing_speed, transit_speed
  min_turning_radius, coverage_xy_tolerance, xy/yaw_goal_tolerance
  chassis_safety_inset, headland_width, num_headland_passes
  (none — the gyro yaw-rate loop moved into FIRMWARE, task #33/#34, Option C.
   It is no longer a host-toggleable ROS param; HW_ANGULAR_RATE_MAX_CMD below
   is carried as the firmware's clamp value until #33 reports its own.)

From firmware (board.h / cpp_main.cpp — NOT operator-editable):
  MAX_MPS (0.5)        — per-wheel top speed (firmware clamp)
  TICKS_PER_M (300)    — also the PWM_PER_MPS scale (PWM 300 = 1 m/s)
  PWM deadband (~40)   — PAC5210 brushed-DC static-friction deadband, i.e.
                         open-loop breakaway ≈ 40/300 ≈ 0.133 m/s per wheel

Per-model drive-motor specs (MODEL_SPECS) — sourced from official manuals,
the Mowgli firmware, and the OpenMower xESC configs; see compute_nav2_params.md
for the source table.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

try:
    import yaml
except ImportError:  # pragma: no cover
    sys.stderr.write("PyYAML required: pip install pyyaml\n")
    sys.exit(2)

# Shared recursive dict-merge (deep-copies throughout so the result shares no
# mutable nested dict with either input). Imported rather than duplicated so
# this stays in lockstep with what navigation.launch.py actually composes —
# see robot_config_util.py's deep_merge docstring for why the naive
# `dict(base)` shallow-copy variant is unsafe.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src", "mowgli_bringup", "launch"))
from robot_config_util import deep_merge  # noqa: E402


# ===========================================================================
# Hardware constants (firmware — NOT operator editable).
# These are the physical ceilings the firmware enforces; nothing derived here
# may exceed them.
# ===========================================================================
BOARD_DEFAULTS = {
    # MAX_MPS: firmware caps PER-WHEEL linear speed at 0.5 m/s (board.h).
    "max_mps": 0.5,
    # WHEEL_BASE: centre-to-centre wheel track. Overridden by the yaml
    # wheel_track when present (they MUST match — see CLAUDE.md).
    "wheel_track": 0.325,
    # TICKS_PER_M doubles as PWM_PER_MPS in the firmware: PWM 300 ≡ 1 m/s.
    "pwm_per_mps": 300.0,
    # PAC5210 brushed-DC static-friction deadband (cpp_main.cpp: "~PWM 40").
    # Open-loop breakaway speed = pwm_deadband / pwm_per_mps ≈ 0.133 m/s.
    "pwm_deadband": 40.0,
}

# hardware_bridge_node clamps (declared params, not board.h):
#   min_linear_vel  default 0.05 — |vx| below this is zeroed (sub-deadband
#                   guard; the firmware PID tracks ≥0.05 cleanly)
# 2026-07-17 Option C (task #34): the gyro rate-loop output cap used to be a
# host param (angular_rate_max_cmd, active only when angular_rate_loop_enabled
# — both removed). The loop now runs in firmware (task #33) unconditionally;
# this constant is kept as the firmware's clamp value (was the host default)
# until #33 reports its own value/param name.
HW_MIN_LINEAR_VEL = 0.05
HW_ANGULAR_RATE_MAX_CMD = 1.5


# ===========================================================================
# Per-model drive-motor specs. Keyed by mowgli_robot.yaml `mower_model`.
#
# All three platforms share the same GForce mainboard family: brushed-DC
# drive motors on a PAC5210 (PWM deadband ~40), MAX_MPS 0.5, TICKS_PER_M 300,
# WHEEL_BASE 0.325 (SA650 presumed same — firmware shares the constant).
#
# mass_kg            — official manual spec (chassis_mass_kg in the yaml wins
#                      when present; the Classic 500 measured 8.76 vs 8.8).
# drive_current_max_a— OpenMower xESC motor-current limit for the STOCK motor
#                      (community FOC tuning, the best published proxy for
#                      relative drive-motor capability across models).
# motor_force_n      — net forward force budget for BOTH wheels (Newtons),
#                      used for the accel budget a = F/m. Classic: ~9 N is
#                      the field-validated value (reaches 0.5 m/s in <1 s,
#                      a ≈ 1.0 m/s² at 8.76 kg). SA650: the 3.26× xESC
#                      current limit implies substantially more torque, but
#                      with no field data we grant a conservative 1.5× force
#                      (a ≈ 1.55 m/s²) — grass traction and ride comfort cap
#                      the useful budget long before 3×.
# No model documents drive-motor torque or the OEM gear ratio anywhere
# (manuals' "50 W rated power" is the BLADE motor) — force budgets are
# inferred and CLI-overridable (--motor-force-n).
# ===========================================================================
MODEL_SPECS = {
    "YardForce500": {
        "mass_kg": 8.8,
        "drive_current_max_a": 2.0,
        "motor_force_n": 9.0,
        "note": "Classic 500 — field-validated accel ~1.0 m/s^2",
    },
    "YardForce500B": {
        "mass_kg": 8.8,
        "drive_current_max_a": 2.0,
        "motor_force_n": 9.0,
        "note": "Classic 500B — same drive train as Classic 500",
    },
    "SA650": {
        "mass_kg": 8.5,
        "drive_current_max_a": 6.52,
        "motor_force_n": 13.5,
        "note": "SA650 ECO/B — 3.26x xESC current limit vs Classic; "
                "conservative 1.5x force budget pending field data",
    },
}
# Aliases seen in the wild.
MODEL_SPECS["SA650ECO"] = MODEL_SPECS["SA650"]
MODEL_SPECS["SA650B"] = MODEL_SPECS["SA650"]


# ===========================================================================
# Derivation profiles (anti-oscillation bias vs heading-correction bias).
# Hard physics (firmware clamps, rate-loop ceiling, deadband floors) apply
# identically to both — profiles only choose WITHIN the feasible envelope.
# ===========================================================================
PROFILES = {
    "calm": {
        # MPPI exploration noise as fractions of the velocity ranges.
        # vx 0.5: least chatter on straights (the 2026-06-07 lesson capped
        # how LOW this can go — 0.10 abs lost U-turn recovery authority, so
        # never let the absolute value fall below ~0.10).
        "vx_std_fraction": 0.5,
        "wz_std_fraction": 0.22,
        # No per-wheel clipping allowed: wz_max respects saturation at FULL
        # mowing speed so MPPI's DiffDrive model always matches the plant.
        "clip_fraction": 0.0,
        # In-place pivot wheel speed as a fraction of MAX_MPS (control
        # margin for the rate loop / firmware PID).
        "pivot_wheel_fraction": 0.5,
    },
    "responsive": {
        "vx_std_fraction": 0.75,   # 0.15 @ vx_max 0.20 — the field value
        "wz_std_fraction": 0.30,
        # Accept one wheel briefly asking 10% over MAX_MPS during a
        # max-speed turn (firmware clips it; trajectory deviates slightly
        # from MPPI's prediction for that slice).
        "clip_fraction": 0.10,
        "pivot_wheel_fraction": 0.65,
    },
}


# ===========================================================================
# Default physics knobs (the "free" inputs that have no single right value
# but a sane default + clear meaning). All overridable from the CLI.
# ===========================================================================
DEFAULT_KNOBS = {
    # Footprint safety margin added around the measured chassis rectangle
    # (matches the 0.05 m in navigation.launch.py footprint computation).
    "footprint_margin": 0.05,
    # Actuation latency for stopping-distance sizing of the collision_monitor
    # zones: cmd_vel→firmware→wheel chain + one controller tick (0.1 s) + DDS
    # jitter ≈ 0.3 s conservative.
    "reaction_time_s": 0.3,
    # Net deceleration when braking — traction-limited on grass for this
    # mass class; 1.0 m/s^2 is conservative and matches MPPI ax_min.
    "decel_max": 1.0,
    # Lateral margins on the collision_monitor zones.
    "slow_lateral_margin": 0.15,
    "stop_lateral_margin": 0.0,   # PolygonStop hugs the chassis
    # Fraction of mowing speed retained when the slow zone fires.
    "slow_speed_fraction": 0.3,
    # Reverse authority as a fraction of mowing speed (escape reverse is
    # AUTHORIZED — forward-only stalled at boundary U-turns, 2026-06).
    "reverse_fraction": 0.75,
    # Net forward force budget (N) for BOTH drive motors. None → MODEL_SPECS
    # row for mower_model. Override for a custom drivetrain.
    "motor_force_n": None,
    # Angular accel budget (rad/s^2): differential torque about the track;
    # the firmware sustains crisp pivots. 2.5 matches the field value.
    "angular_accel": 2.5,
    # MPPI prediction horizon target (s). time_steps = horizon / model_dt.
    "lookahead_horizon_s": 3.6,
    # Controller loop rate (Hz). model_dt = 1/freq. Field value 10 Hz (ARM).
    "controller_frequency": 10.0,
    # Goal-checker xy tolerance as a fraction of tool_width (coverage).
    "coverage_xy_tol_fraction": 0.83,   # 0.15 / 0.18 ≈ 0.83
    # Cost decay length for inflation cost_scaling_factor (cost falls to
    # ~1/e of lethal over this distance).
    "inflation_decay_len_m": 0.10,
    # How far OUTSIDE the recorded boundary line the lethal keepout is
    # pushed (the recorded outline is the robot's CENTRE path; footprint
    # overhang is intended).
    "keepout_tracking_margin": 0.25,
    # Safety factor applied to the open-loop breakaway speed when deriving
    # deadband-clearing floors (VelocityDeadbandCritic, approach floors).
    "deadband_margin": 1.1,
    # Profile-resolved knobs — None here means "take the profile value";
    # an explicit CLI value overrides the profile.
    "vx_std_fraction": None,
    "wz_std_fraction": None,
    "clip_fraction": None,
    "pivot_wheel_fraction": None,
}


# ===========================================================================
# Input loading
# ===========================================================================
def load_robot_yaml(path: str) -> dict:
    """Load mowgli.ros__parameters from a mowgli_robot.yaml."""
    with open(path, "r", encoding="utf-8") as fh:
        doc = yaml.safe_load(fh) or {}
    return doc.get("mowgli", {}).get("ros__parameters", {}) or {}


def resolve_profile_knobs(knobs: dict, profile: str) -> dict:
    """Fill profile-resolved knobs (None) from the chosen PROFILES row."""
    prof = PROFILES[profile]
    out = dict(knobs)
    for k, v in prof.items():
        if out.get(k) is None:
            out[k] = v
    return out


def gather_inputs(rp: dict, knobs: dict, board: dict) -> dict:
    """Resolve the physical inputs: mowgli_robot.yaml values over documented
    defaults (mirroring navigation.launch.py / mowgli.launch.py defaults),
    MODEL_SPECS for the drive-motor row."""
    cw = float(rp.get("chassis_width", 0.40))
    inset = rp.get("chassis_safety_inset", None)
    if inset is None:
        # navigation.launch.py default: chassis_width / 2 when not set.
        inset = cw / 2.0
    model = str(rp.get("mower_model", "YardForce500"))
    spec = MODEL_SPECS.get(model)
    if spec is None:
        sys.stderr.write(
            f"WARNING: unknown mower_model '{model}' — falling back to "
            f"YardForce500 drive-motor specs\n")
        spec = MODEL_SPECS["YardForce500"]
    force_n = knobs["motor_force_n"]
    if force_n is None:
        force_n = spec["motor_force_n"]
    return {
        "mower_model": model,
        "model_note": spec["note"],
        "chassis_length": float(rp.get("chassis_length", 0.60)),
        "chassis_width": cw,
        "chassis_center_x": float(rp.get("chassis_center_x", 0.18)),
        # Measured mass (yaml) wins over the manual spec.
        "chassis_mass_kg": float(rp.get("chassis_mass_kg", spec["mass_kg"])),
        "motor_force_n": float(force_n),
        "tool_width": float(rp.get("tool_width", 0.18)),
        "swath_overlap": float(rp.get("swath_overlap", 0.02)),
        # navigation.launch.py defaults: mowing 0.25, transit 0.3 (yaml wins).
        "mowing_speed": float(rp.get("mowing_speed", 0.25)),
        "transit_speed": float(rp.get("transit_speed", 0.3)),
        "wheel_track": float(rp.get("wheel_track", board["wheel_track"])),
        "min_turning_radius": float(rp.get("min_turning_radius", 0.15)),
        "coverage_xy_tolerance": float(rp.get("coverage_xy_tolerance", 0.10)),
        "xy_goal_tolerance": float(rp.get("xy_goal_tolerance", 0.30)),
        "yaw_goal_tolerance": float(rp.get("yaw_goal_tolerance", 0.10)),
        "chassis_safety_inset": float(inset),
        "headland_width": float(rp.get("headland_width", 0.18)),
        "num_headland_passes": int(rp.get("num_headland_passes", 0)),
        "max_mps": float(board["max_mps"]),
        "pwm_per_mps": float(board["pwm_per_mps"]),
        "pwm_deadband": float(board["pwm_deadband"]),
    }


# ===========================================================================
# Subsystem derivations. Each returns {param: (value, formula_str)}.
# ===========================================================================
def footprint(inp: dict, knobs: dict) -> dict:
    """Rectangular diff-drive footprint + inscribed / circumscribed radii.

    Footprint rectangle (base_footprint frame, X forward), matching
    navigation.launch.py:
        front  =  chassis_center_x + chassis_length/2 + margin
        rear   =  chassis_center_x - chassis_length/2 - margin
        half_w =  chassis_width/2 + margin

    The robot frame origin (base_footprint at the rear wheel axis) is NOT the
    rectangle centre, so the collision radii are measured from the ORIGIN:
        inscribed     = min distance origin → any edge
        circumscribed = max distance origin → any corner (REQUIRED lower
                        bound for inflation_radius — SE2 collision checking).
    """
    margin = knobs["footprint_margin"]
    cl, cw, ccx = inp["chassis_length"], inp["chassis_width"], inp["chassis_center_x"]
    fp_f = ccx + cl / 2.0 + margin
    fp_r = ccx - cl / 2.0 - margin
    half_w = cw / 2.0 + margin

    fp_str = (
        f"[[{fp_f:.3f}, {half_w:.3f}], [{fp_f:.3f}, {-half_w:.3f}], "
        f"[{fp_r:.3f}, {-half_w:.3f}], [{fp_r:.3f}, {half_w:.3f}]]"
    )
    inscribed = min(abs(fp_f), abs(fp_r), half_w)
    corners = [(fp_f, half_w), (fp_f, -half_w), (fp_r, -half_w), (fp_r, half_w)]
    circumscribed = max(math.hypot(x, y) for x, y in corners)
    return {
        "footprint": (fp_str,
                      f"rect: front={fp_f:.3f} rear={fp_r:.3f} half_w={half_w:.3f} "
                      f"(chassis {cl}x{cw}, center_x {ccx}, +{margin} margin)"),
        "_front": (fp_f, "chassis_center_x + chassis_length/2 + margin"),
        "_rear": (fp_r, "chassis_center_x - chassis_length/2 - margin"),
        "_half_width": (half_w, "chassis_width/2 + margin"),
        "_inscribed_radius": (inscribed, "min(|front|, |rear|, half_w)"),
        "_circumscribed_radius": (circumscribed,
                                  "max corner distance from base_footprint origin"),
    }


def kinematics(inp: dict, knobs: dict) -> dict:
    """Diff-drive capabilities, deadband floors, and the rate-loop ceiling.

    Per-wheel top speed v_w = MAX_MPS (firmware clamp). Diff-drive coupling:
        wheel_speed = |vx| + |wz| * track / 2
    so the per-wheel cap COUPLES vx and wz:
        in-place pivot cap : omega_pivot_max = 2 * MAX_MPS / track
        at cruise vx       : wz_sat(vx) = 2 * (MAX_MPS - vx) / track
    Commanding beyond wz_sat(vx) makes the firmware clip one wheel — the
    executed twist deviates from the commanded one (MPPI model mismatch →
    tracking oscillation). This is the constraint the 2026-06-12 wz_max=2.0
    bump violated at mowing speed (2*(0.5-0.2)/0.325 = 1.85).

    Rate-loop ceiling: the gyro yaw-rate loop (now firmware-side, Option C
    task #33/#34) clamps |wz| commands at HW_ANGULAR_RATE_MAX_CMD = 1.5 — any
    larger wz_max/pivot rate in Nav2 is silently unreachable.

    Deadband (PAC5210 brushed-DC, ~PWM 40 on the 0-300 = 1 m/s scale):
        vx_breakaway = pwm_deadband / pwm_per_mps            (≈0.133 m/s)
        wz_breakaway = 2 * vx_breakaway / track  (in-place)  (≈0.82 rad/s)
    The host bridges these (min_linear_vel guard + gyro PI / pulse
    modulation), but derived floors keep commands decisively ABOVE them.

    Linear accel budget from the per-model motor force:  a = F / m.
    """
    vw = inp["max_mps"]
    track = inp["wheel_track"]
    vx_breakaway = inp["pwm_deadband"] / inp["pwm_per_mps"]
    wz_breakaway = 2.0 * vx_breakaway / track
    omega_pivot_max = 2.0 * vw / track
    # Option C (task #34): the firmware yaw-rate loop is unconditional now
    # (no host toggle), so the clamp is always active.
    wz_ceiling = min(omega_pivot_max, HW_ANGULAR_RATE_MAX_CMD)
    ceiling_why = ("firmware rate loop: min(2*MAX_MPS/track, "
                   f"HW_ANGULAR_RATE_MAX_CMD={HW_ANGULAR_RATE_MAX_CMD})")
    wz_sat_at_mow = 2.0 * (vw - inp["mowing_speed"]) / track
    a_lin = inp["motor_force_n"] / inp["chassis_mass_kg"]
    return {
        "_vx_breakaway": (round(vx_breakaway, 3),
                          "pwm_deadband / pwm_per_mps (open-loop breakaway)"),
        "_wz_breakaway": (round(wz_breakaway, 3),
                          "2 * vx_breakaway / wheel_track (in-place)"),
        "_omega_pivot_max": (round(omega_pivot_max, 3), "2 * MAX_MPS / wheel_track"),
        "_wz_ceiling": (round(wz_ceiling, 3), ceiling_why),
        "_wz_sat_at_mowing_speed": (round(wz_sat_at_mow, 3),
                                    "2*(MAX_MPS - mowing_speed)/track "
                                    "(per-wheel saturation while cruising)"),
        "_a_lin_budget": (round(a_lin, 3),
                          f"motor_force_n({inp['motor_force_n']}) / mass "
                          f"({inp['chassis_mass_kg']}) — {inp['mower_model']}"),
        "_omega_at_mowing_minradius": (
            round(inp["mowing_speed"] / max(inp["min_turning_radius"], 1e-3), 3),
            "mowing_speed / min_turning_radius (arc-turn omega=v/r)"),
    }


def mppi(inp: dict, knobs: dict, kin: dict, fp: dict) -> dict:
    """MPPI (FollowCoveragePath primary) velocity/accel/horizon params.

    vx_max  = mowing_speed (operator).
    vx_min  = -reverse_fraction * vx_max (reverse AUTHORIZED for escapes).
    wz_max  = min(rate-loop ceiling, saturation bound). The saturation bound
              is wz_sat at vx_max for the calm profile (clip_fraction 0 — no
              silent wheel clipping ever), relaxed by clip_fraction for the
              responsive profile:
                  wz_sat = 2*((1+clip)*MAX_MPS - vx_max)/track
    vx_std/wz_std = exploration noise fractions (profile). wz_std addition-
              ally floors at 0.5*wz_breakaway when the rate loop is OFF, so
              sampled rotations actually cross the open-loop deadband instead
              of dithering inside it.
    ax_max  = F/m (per-model); ax_min = -decel_max; az_max = angular budget.
    model_dt = 1/controller_frequency; time_steps = horizon/model_dt.
    Prediction distance = time_steps*model_dt*vx_max sets the critics'
    threshold_to_consider (GoalCritic / PathFollowCritic) — tied to OUR
    horizon, not the ref 0.5 m/s defaults.
    VelocityDeadbandCritic vx = deadband_margin * vx_breakaway (penalise
    commands the motors cannot execute).
    """
    vx_max = inp["mowing_speed"]
    vx_min = -knobs["reverse_fraction"] * vx_max
    clip = knobs["clip_fraction"]
    track = inp["wheel_track"]
    wz_sat = 2.0 * ((1.0 + clip) * inp["max_mps"] - vx_max) / track
    wz_max = min(kin["_wz_ceiling"][0], wz_sat)
    vx_std = knobs["vx_std_fraction"] * vx_max
    wz_std = knobs["wz_std_fraction"] * wz_max
    # Option C (task #34): the wz_std floor for "rate loop OFF" no longer
    # applies — the firmware yaw-rate loop is unconditional, no host toggle.
    ax_max = kin["_a_lin_budget"][0]
    ax_min = -knobs["decel_max"]
    az_max = knobs["angular_accel"]
    model_dt = 1.0 / knobs["controller_frequency"]
    time_steps = int(round(knobs["lookahead_horizon_s"] / model_dt))
    pred_dist = time_steps * model_dt * vx_max
    thresh = round(pred_dist, 2)
    deadband_vx = round(knobs["deadband_margin"] * kin["_vx_breakaway"][0], 2)
    wz_std_why = f"= wz_std_fraction({knobs['wz_std_fraction']}) * wz_max"
    return {
        "vx_max": (round(vx_max, 3), "= mowing_speed"),
        "vx_min": (round(vx_min, 3), "= -reverse_fraction * vx_max (escape reverse)"),
        "vy_max": (0.0, "diff-drive: no lateral DOF"),
        "wz_max": (round(wz_max, 3),
                   f"min(wz_ceiling={kin['_wz_ceiling'][0]}, "
                   f"2*((1+{clip})*MAX_MPS - vx_max)/track={wz_sat:.3f})"),
        "vx_std": (round(vx_std, 3),
                   f"= vx_std_fraction({knobs['vx_std_fraction']}) * vx_max"),
        "vy_std": (0.0, "diff-drive: no lateral DOF"),
        "wz_std": (round(wz_std, 3), wz_std_why),
        "ax_max": (round(ax_max, 2), "= motor_force_n / chassis_mass_kg"),
        "ax_min": (round(ax_min, 2), "= -decel_max"),
        "ay_max": (0.0, "diff-drive"),
        "ay_min": (0.0, "diff-drive"),
        "az_max": (round(az_max, 2), "= angular_accel budget"),
        "model_dt": (round(model_dt, 3), "= 1 / controller_frequency"),
        "time_steps": (time_steps, "= round(lookahead_horizon_s / model_dt)"),
        "motion_model": ("DiffDrive", "skid-steer / differential"),
        "_prediction_distance": (round(pred_dist, 3),
                                 "time_steps * model_dt * vx_max"),
        "_goal_threshold_to_consider": (thresh, "= prediction distance"),
        "_path_follow_threshold_to_consider": (thresh, "= prediction distance"),
        "deadband_velocity_vx": (deadband_vx,
                                 "deadband_margin * vx_breakaway "
                                 "(VelocityDeadbandCritic, firmware breakaway)"),
    }


def rotation_shim(inp: dict, knobs: dict, kin: dict) -> dict:
    """RotationShim pivot rates (FollowPath/RPP and FollowCoveragePath/MPPI).

    The pivot rate must:
      (a) clear the rotational deadband decisively — wz_breakaway ≈ 0.82
          rad/s open-loop. The firmware rate loop (Option C, task #33/#34)
          bridges it, so 0.9*wz_breakaway is safe.
      (b) stay <= the active wz ceiling (rate-loop clamp or kinematic cap).
      (c) keep per-wheel pivot speed at a controllable fraction of MAX_MPS:
          wz_pivot = pivot_wheel_fraction * omega_pivot_max  (profile knob;
          calm 0.5 → ~1.54, responsive 0.65 → ~2.0 — the field values).
    Coverage rides (c) clamped by (b); transit is calmer: just clear of the
    deadband, clamped into [0.5, coverage rate].
    behavior_server floors (Spin recovery) also derive from the deadband:
        min_rotational_vel = deadband_margin * wz_breakaway  (≈0.85 field).
    """
    wz_break = kin["_wz_breakaway"][0]
    ceiling = kin["_wz_ceiling"][0]
    coverage_pivot = min(knobs["pivot_wheel_fraction"] * kin["_omega_pivot_max"][0],
                         ceiling)
    transit_floor = 0.9 * wz_break
    transit_pivot = min(max(transit_floor, 0.5), coverage_pivot)
    min_rot = knobs["deadband_margin"] * wz_break
    return {
        "transit_rotate_to_heading_angular_vel": (
            round(transit_pivot, 3),
            "max(0.9*wz_breakaway, 0.5), capped at coverage rate"),
        "coverage_rotate_to_heading_angular_vel": (
            round(coverage_pivot, 3),
            f"pivot_wheel_fraction({knobs['pivot_wheel_fraction']}) * "
            "omega_pivot_max, capped at wz_ceiling"),
        "max_angular_accel": (knobs["angular_accel"], "= az_max"),
        "behavior_min_rotational_vel": (
            round(min_rot, 3),
            "deadband_margin * wz_breakaway (Spin floor clears deadband)"),
        "behavior_max_rotational_vel": (
            round(ceiling, 3), "= active wz ceiling"),
    }


def costmap(inp: dict, knobs: dict, fp: dict) -> dict:
    """Inflation radius + cost scaling.

    inflation_radius MUST be >= circumscribed radius for correct SE2
    collision checking (Nav2 warns otherwise). NOTE the repo currently runs
    BELOW this on purpose (0.20/0.10) — Smac falls back to per-footprint
    checking and the tight local value frees boundary U-turns; the derived
    value is the geometric CORRECT one, the divergence is a documented trade.

    cost_scaling_factor = 1 / inflation_decay_len_m (cost falls to ~1/e of
    lethal over the decay length).
    """
    circ = fp["_circumscribed_radius"][0]
    inflation_radius = round(circ, 3)
    csf = round(1.0 / knobs["inflation_decay_len_m"], 2)
    return {
        "inflation_radius": (inflation_radius,
                             ">= circumscribed radius (SE2 collision correctness; "
                             "repo intentionally runs lower — see formula note)"),
        "cost_scaling_factor": (csf, "1 / inflation_decay_len_m"),
    }


def collision_monitor(inp: dict, knobs: dict, fp: dict, mppi_p: dict) -> dict:
    """PolygonSlow / PolygonStop zones from stopping distance.

    Stopping distance at speed v with reaction latency t_r and decel a:
        d_stop = v^2 / (2*a) + v * t_r
    Slow zone uses the MOWING speed; stop zone uses reaction-only depth.
    Polygon forward extent = footprint front + d_stop; lateral = half_width
    + margin. slowdown_ratio = slow_speed_fraction.
    (The LiDAR overlay activates PolygonSlow only — PolygonStop stays out of
    the active set per CLAUDE.md; its geometry is still derived for the day
    it returns.)
    """
    front = fp["_front"][0]
    rear = fp["_rear"][0]
    half_w = fp["_half_width"][0]
    v = inp["mowing_speed"]
    a = knobs["decel_max"]
    tr = knobs["reaction_time_s"]
    d_stop = v * v / (2.0 * a) + v * tr

    slow_f = round(front + d_stop, 3)
    slow_hw = round(half_w + knobs["slow_lateral_margin"], 3)
    slow_r = round(rear - knobs["slow_lateral_margin"], 3)
    d_react = v * tr
    stop_f = round(front + d_react, 3)
    stop_hw = round(half_w + knobs["stop_lateral_margin"], 3)
    stop_r = round(rear - knobs["stop_lateral_margin"], 3)

    def poly(f, hw, r):
        return f"[[{f}, {hw}], [{f}, {-hw}], [{r}, {-hw}], [{r}, {hw}]]"

    return {
        "_d_stop": (round(d_stop, 3), "v^2/(2*decel) + v*reaction_time"),
        "_d_react": (round(d_react, 3), "v * reaction_time"),
        "PolygonSlow.points": (poly(slow_f, slow_hw, slow_r),
                               "footprint + d_stop forward, +slow_lateral_margin"),
        "PolygonSlow.slowdown_ratio": (knobs["slow_speed_fraction"],
                                       "= slow_speed_fraction (safe-speed retain)"),
        "PolygonStop.points": (poly(stop_f, stop_hw, stop_r),
                               "footprint + d_react forward, hugs chassis "
                               "(NOT active — geometry kept current)"),
    }


def coverage(inp: dict, knobs: dict, fp: dict) -> dict:
    """F2C coverage + map_server keepout geometry.

    operation_width = tool_width - swath_overlap (navigation.launch.py
    injection: adjacent swaths OVERLAP by swath_overlap; map_server's stamp
    radius stays tool_width — single-source invariant).

    F2C headland (chassis-derived, matching coverage_nodes.cpp):
        ring_inset = 0.5 * chassis_width + footprint_margin
        headland   = ring_inset + 0.5 * operation_width

    keepout_nav_margin pushes the LETHAL zone OUTSIDE the recorded boundary
    by footprint half-width + tracking margin (robot mows TO the line, the
    recorded outline is the CENTRE path).
    """
    op_w = inp["tool_width"] - inp["swath_overlap"]
    margin = knobs["footprint_margin"]
    ring_inset = 0.5 * inp["chassis_width"] + margin
    headland = ring_inset + 0.5 * op_w
    half_w = fp["_half_width"][0]
    keepout = round(half_w + knobs["keepout_tracking_margin"], 3)
    lethal = round(keepout + 0.05, 3)
    return {
        "operation_width": (round(op_w, 3),
                            "= tool_width - swath_overlap (launch injection)"),
        "default_headland_width": (round(headland, 3),
                                   "0.5*chassis_width + margin + 0.5*op_w"),
        "min_turning_radius": (round(inp["min_turning_radius"], 3),
                               "operator ([0.10, 0.50] field envelope)"),
        "keepout_nav_margin": (keepout,
                               "footprint_half_width + keepout_tracking_margin"),
        "lethal_boundary_margin_m": (lethal, ">= keepout_nav_margin + 0.05 buffer"),
        "chassis_safety_inset": (round(inp["chassis_safety_inset"], 3),
                                 "operator (tracking-error margin; 0.15 field "
                                 "2026-06-12, lower once tracking is tight)"),
    }


def goal_checkers(inp: dict, knobs: dict) -> dict:
    """Coverage / transit goal-checker tolerances.

    Coverage xy tolerance = coverage_xy_tol_fraction * tool_width (endpoint
    within a fraction of one swath so tiling holds), clipped to the 0.15 m
    launch ceiling. Coverage yaw is loose (path-progress gates completion).
    """
    cov_xy = min(knobs["coverage_xy_tol_fraction"] * inp["tool_width"], 0.15)
    return {
        "coverage_goal_checker.xy_goal_tolerance": (
            round(cov_xy, 3),
            "coverage_xy_tol_fraction * tool_width, clipped <= 0.15"),
        "coverage_goal_checker.yaw_goal_tolerance": (
            0.30, "loose — PathProgressGoalChecker gates on path progress"),
        "coverage_goal_checker.progress_threshold": (
            0.95, "fixed: 95% monotonic path progress (CLAUDE.md invariant)"),
        "stopped_goal_checker.xy_goal_tolerance": (
            round(inp["xy_goal_tolerance"], 3), "= operator xy_goal_tolerance"),
        "stopped_goal_checker.yaw_goal_tolerance": (
            round(inp["yaw_goal_tolerance"], 3), "= operator yaw_goal_tolerance"),
    }


def follow_path(inp: dict, knobs: dict, kin: dict, rs: dict) -> dict:
    """RotationShim + RPP (FollowPath transit) params derived from physics.

    min_approach_linear_velocity must stay decisively above the firmware
    breakaway or the final approach gets zeroed/buzzes:
        = deadband_margin * vx_breakaway * 1.1  (≈ 0.16 field value).
    """
    approach = round(knobs["deadband_margin"] * kin["_vx_breakaway"][0] * 1.1, 2)
    return {
        "desired_linear_vel": (round(inp["transit_speed"], 3), "= transit_speed"),
        "rotate_to_heading_angular_vel": (
            rs["transit_rotate_to_heading_angular_vel"][0],
            "RotationShim transit pivot rate"),
        "max_angular_accel": (rs["max_angular_accel"][0], "= az_max"),
        "min_approach_linear_velocity": (
            approach,
            "deadband_margin * vx_breakaway * 1.1 (clears firmware breakaway)"),
    }


# ===========================================================================
# Orchestration
# ===========================================================================
def compute_all(rp: dict, knobs: dict, board: dict, profile: str) -> dict:
    knobs = resolve_profile_knobs(knobs, profile)
    inp = gather_inputs(rp, knobs, board)
    fp = footprint(inp, knobs)
    kin = kinematics(inp, knobs)
    mppi_p = mppi(inp, knobs, kin, fp)
    rs = rotation_shim(inp, knobs, kin)
    cm = costmap(inp, knobs, fp)
    coll = collision_monitor(inp, knobs, fp, mppi_p)
    cov = coverage(inp, knobs, fp)
    gc = goal_checkers(inp, knobs)
    fpth = follow_path(inp, knobs, kin, rs)
    return {
        "_profile": profile,
        "_inputs": inp,
        "footprint": fp,
        "kinematics": kin,
        "mppi": mppi_p,
        "rotation_shim": rs,
        "costmap": cm,
        "collision_monitor": coll,
        "coverage": cov,
        "goal_checkers": gc,
        "follow_path": fpth,
    }


def emit_report(res: dict) -> str:
    """Human-readable report: each value + formula/rationale."""
    out = []
    out.append("=" * 78)
    out.append(f"DERIVED NAV2 PARAMETERS — profile: {res['_profile']}")
    out.append("(physics-based, from mowgli_robot.yaml + firmware + model specs)")
    out.append("=" * 78)
    inp = res["_inputs"]
    out.append("\n--- Physical inputs ---")
    for k in sorted(inp):
        out.append(f"  {k:30s} = {inp[k]}")
    for section in ("footprint", "kinematics", "mppi", "rotation_shim",
                    "costmap", "collision_monitor", "coverage",
                    "goal_checkers", "follow_path"):
        out.append(f"\n--- {section} ---")
        for name, (val, why) in res[section].items():
            tag = "  (intermediate)" if name.startswith("_") else ""
            sval = str(val)
            if len(sval) > 40:
                out.append(f"  {name}{tag}")
                out.append(f"      = {sval}")
                out.append(f"      # {why}")
            else:
                out.append(f"  {name:42s} = {sval:<22s} # {why}{tag}")
    return "\n".join(out)


def emit_yaml(res: dict) -> str:
    """YAML fragment mirroring the nav2 params tree (public params only,
    no leading-underscore intermediates), diffable against the merged file."""
    def pub(section):
        return {k: v[0] for k, v in res[section].items() if not k.startswith("_")}

    mppi_p = pub("mppi")
    cm = pub("costmap")
    coll = res["collision_monitor"]
    cov = pub("coverage")
    gc = res["goal_checkers"]
    rs = res["rotation_shim"]
    fpth = pub("follow_path")

    doc = {
        "controller_server": {
            "ros__parameters": {
                "controller_frequency": 1.0 / mppi_p["model_dt"],
                "FollowPath": {
                    "desired_linear_vel": fpth["desired_linear_vel"],
                    "rotate_to_heading_angular_vel": fpth["rotate_to_heading_angular_vel"],
                    "max_angular_accel": fpth["max_angular_accel"],
                    "min_approach_linear_velocity": fpth["min_approach_linear_velocity"],
                },
                "FollowCoveragePath": {
                    "rotate_to_heading_angular_vel":
                        rs["coverage_rotate_to_heading_angular_vel"][0],
                    "max_angular_accel": rs["max_angular_accel"][0],
                    "time_steps": mppi_p["time_steps"],
                    "model_dt": mppi_p["model_dt"],
                    "vx_max": mppi_p["vx_max"],
                    "vx_min": mppi_p["vx_min"],
                    "vy_max": mppi_p["vy_max"],
                    "wz_max": mppi_p["wz_max"],
                    "vx_std": mppi_p["vx_std"],
                    "vy_std": mppi_p["vy_std"],
                    "wz_std": mppi_p["wz_std"],
                    "ax_max": mppi_p["ax_max"],
                    "ax_min": mppi_p["ax_min"],
                    "ay_max": mppi_p["ay_max"],
                    "ay_min": mppi_p["ay_min"],
                    "az_max": mppi_p["az_max"],
                    "motion_model": mppi_p["motion_model"],
                    "GoalCritic": {"threshold_to_consider":
                                   res["mppi"]["_goal_threshold_to_consider"][0]},
                    "PathFollowCritic": {"threshold_to_consider":
                                         res["mppi"]["_path_follow_threshold_to_consider"][0]},
                    "VelocityDeadbandCritic": {"deadband_velocities":
                                               [mppi_p["deadband_velocity_vx"], 0.0, 0.0]},
                },
                "coverage_goal_checker": {
                    "xy_goal_tolerance": gc["coverage_goal_checker.xy_goal_tolerance"][0],
                    "yaw_goal_tolerance": gc["coverage_goal_checker.yaw_goal_tolerance"][0],
                    "progress_threshold": gc["coverage_goal_checker.progress_threshold"][0],
                },
                "stopped_goal_checker": {
                    "xy_goal_tolerance": gc["stopped_goal_checker.xy_goal_tolerance"][0],
                    "yaw_goal_tolerance": gc["stopped_goal_checker.yaw_goal_tolerance"][0],
                },
            }
        },
        "behavior_server": {"ros__parameters": {
            "max_rotational_vel": rs["behavior_max_rotational_vel"][0],
            "min_rotational_vel": rs["behavior_min_rotational_vel"][0],
        }},
        "global_costmap": {"global_costmap": {"ros__parameters": {
            "footprint": res["footprint"]["footprint"][0],
            "inflation_layer": {
                "inflation_radius": cm["inflation_radius"],
                "cost_scaling_factor": cm["cost_scaling_factor"],
            },
        }}},
        "local_costmap": {"local_costmap": {"ros__parameters": {
            "footprint": res["footprint"]["footprint"][0],
            "inflation_layer": {
                "inflation_radius": cm["inflation_radius"],
                "cost_scaling_factor": cm["cost_scaling_factor"],
            },
        }}},
        "collision_monitor": {"ros__parameters": {
            "PolygonSlow": {
                "points": coll["PolygonSlow.points"][0],
                "slowdown_ratio": coll["PolygonSlow.slowdown_ratio"][0],
            },
            "PolygonStop": {"points": coll["PolygonStop.points"][0]},
        }},
        "coverage_server": {"ros__parameters": {
            "operation_width": cov["operation_width"],
            "default_headland_width": cov["default_headland_width"],
            "min_turning_radius": cov["min_turning_radius"],
            "chassis_safety_inset": cov["chassis_safety_inset"],
        }},
        "map_server_node": {"ros__parameters": {
            "keepout_nav_margin": cov["keepout_nav_margin"],
            "lethal_boundary_margin_m": cov["lethal_boundary_margin_m"],
            "tool_width": res["_inputs"]["tool_width"],
        }},
    }
    return yaml.safe_dump(doc, default_flow_style=False, sort_keys=False)


# ===========================================================================
# Comparison against the merged base + overlay nav2 params
# ===========================================================================
def load_merged_nav2(base_path: str, overlay_path: str | None) -> dict:
    with open(base_path, "r", encoding="utf-8") as fh:
        base = yaml.safe_load(fh) or {}
    if overlay_path:
        with open(overlay_path, "r", encoding="utf-8") as fh:
            overlay = yaml.safe_load(fh) or {}
        return deep_merge(base, overlay)
    return base


def _get(d: dict, *path):
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur


def emit_compare(res: dict, nav: dict, label: str) -> str:
    """Tabulate derived vs current merged nav2 params. Two flag levels:
    HARD = physics violation (firmware clamp / saturation / deadband — the
    current value is unreachable or causes model mismatch); diverges = the
    derived value differs but the current one is physically feasible."""
    csp = ("controller_server", "ros__parameters")
    fcp = csp + ("FollowCoveragePath",)
    fpp = csp + ("FollowPath",)
    gcp = csp + ("coverage_goal_checker",)
    sgp = csp + ("stopped_goal_checker",)
    bsp = ("behavior_server", "ros__parameters")
    gci = ("global_costmap", "global_costmap", "ros__parameters", "inflation_layer")
    lci = ("local_costmap", "local_costmap", "ros__parameters", "inflation_layer")
    cmp_ = ("collision_monitor", "ros__parameters")
    cov = ("coverage_server", "ros__parameters")

    m = res["mppi"]
    kin = res["kinematics"]
    rs = res["rotation_shim"]
    cm = res["costmap"]
    coll = res["collision_monitor"]
    cv = res["coverage"]
    gc = res["goal_checkers"]
    fp = res["follow_path"]
    inp = res["_inputs"]

    ceiling = kin["_wz_ceiling"][0]
    wz_sat_mow = kin["_wz_sat_at_mowing_speed"][0]
    vx_break = kin["_vx_breakaway"][0]

    def hard_wz(cur):
        """A current wz-type value above the ACTIVE ceiling is silently
        clamped (rate loop) or clips a wheel (kinematics) — HARD flag."""
        return isinstance(cur, (int, float)) and float(cur) > ceiling + 1e-6

    def hard_wz_moving(cur):
        """wz_max above saturation at mowing speed clips a wheel while
        cruising → MPPI model mismatch (the oscillation mechanism)."""
        return isinstance(cur, (int, float)) and (
            float(cur) > ceiling + 1e-6 or float(cur) > wz_sat_mow + 1e-6)

    def hard_vx_floor(cur):
        """A current approach/min velocity at or below firmware breakaway
        gets zeroed/buzzes — HARD flag."""
        return isinstance(cur, (int, float)) and float(cur) < vx_break - 1e-6

    # (label, derived, current, note, hard_check or None)
    rows = [
        ("FollowCoveragePath.vx_max", m["vx_max"][0], _get(nav, *fcp, "vx_max"),
         "= mowing_speed (launch-injected at runtime)", None),
        ("FollowCoveragePath.vx_min", m["vx_min"][0], _get(nav, *fcp, "vx_min"),
         "reverse escape policy", None),
        ("FollowCoveragePath.wz_max", m["wz_max"][0], _get(nav, *fcp, "wz_max"),
         f"ceiling={ceiling}, sat@mow={wz_sat_mow}", hard_wz_moving),
        ("FollowCoveragePath.vx_std", m["vx_std"][0], _get(nav, *fcp, "vx_std"),
         "exploration fraction (profile)", None),
        ("FollowCoveragePath.wz_std", m["wz_std"][0], _get(nav, *fcp, "wz_std"),
         "exploration fraction (profile)", None),
        ("FollowCoveragePath.ax_max", m["ax_max"][0], _get(nav, *fcp, "ax_max"),
         "F/m budget (per-model)", None),
        ("FollowCoveragePath.ax_min", m["ax_min"][0], _get(nav, *fcp, "ax_min"),
         "-decel_max", None),
        ("FollowCoveragePath.az_max", m["az_max"][0], _get(nav, *fcp, "az_max"),
         "angular accel budget", None),
        ("FollowCoveragePath.time_steps", m["time_steps"][0],
         _get(nav, *fcp, "time_steps"), "horizon/model_dt", None),
        ("FollowCoveragePath.model_dt", m["model_dt"][0],
         _get(nav, *fcp, "model_dt"), "1/freq", None),
        ("FollowCoveragePath.rotate_to_heading_angular_vel",
         rs["coverage_rotate_to_heading_angular_vel"][0],
         _get(nav, *fcp, "rotate_to_heading_angular_vel"),
         f"pivot rate, ceiling={ceiling}", hard_wz),
        ("FollowCoveragePath.GoalCritic.threshold_to_consider",
         m["_goal_threshold_to_consider"][0],
         _get(nav, *fcp, "GoalCritic", "threshold_to_consider"),
         "= prediction distance", None),
        ("FollowCoveragePath.PathFollowCritic.threshold_to_consider",
         m["_path_follow_threshold_to_consider"][0],
         _get(nav, *fcp, "PathFollowCritic", "threshold_to_consider"),
         "= prediction distance", None),
        ("FollowCoveragePath.VelocityDeadbandCritic.deadband_velocities[0]",
         m["deadband_velocity_vx"][0],
         (_get(nav, *fcp, "VelocityDeadbandCritic", "deadband_velocities") or [None])[0],
         "margin * vx_breakaway", None),
        ("FollowPath.desired_linear_vel", fp["desired_linear_vel"][0],
         _get(nav, *fpp, "desired_linear_vel"), "= transit_speed (launch-injected)", None),
        ("FollowPath.rotate_to_heading_angular_vel",
         fp["rotate_to_heading_angular_vel"][0],
         _get(nav, *fpp, "rotate_to_heading_angular_vel"),
         "calmer transit pivot", hard_wz),
        ("FollowPath.min_approach_linear_velocity",
         fp["min_approach_linear_velocity"][0],
         _get(nav, *fpp, "min_approach_linear_velocity"),
         f"must clear breakaway {vx_break}", hard_vx_floor),
        ("FollowPath.max_angular_accel", fp["max_angular_accel"][0],
         _get(nav, *fpp, "max_angular_accel"), "az_max", None),
        ("behavior_server.max_rotational_vel",
         rs["behavior_max_rotational_vel"][0],
         _get(nav, *bsp, "max_rotational_vel"), "= active wz ceiling", hard_wz),
        ("behavior_server.min_rotational_vel",
         rs["behavior_min_rotational_vel"][0],
         _get(nav, *bsp, "min_rotational_vel"), "deadband floor", None),
        ("coverage_goal_checker.xy_goal_tolerance",
         gc["coverage_goal_checker.xy_goal_tolerance"][0],
         _get(nav, *gcp, "xy_goal_tolerance"), "fraction of tool_width", None),
        ("stopped_goal_checker.xy_goal_tolerance",
         gc["stopped_goal_checker.xy_goal_tolerance"][0],
         _get(nav, *sgp, "xy_goal_tolerance"), "operator value", None),
        ("stopped_goal_checker.yaw_goal_tolerance",
         gc["stopped_goal_checker.yaw_goal_tolerance"][0],
         _get(nav, *sgp, "yaw_goal_tolerance"), "operator value", None),
        ("global inflation_radius", cm["inflation_radius"][0],
         _get(nav, *gci, "inflation_radius"),
         ">= circumscribed (documented trade if lower)", None),
        ("local inflation_radius", cm["inflation_radius"][0],
         _get(nav, *lci, "inflation_radius"),
         ">= circumscribed (documented trade if lower)", None),
        ("global cost_scaling_factor", cm["cost_scaling_factor"][0],
         _get(nav, *gci, "cost_scaling_factor"), "1/decay_len", None),
        ("PolygonSlow.slowdown_ratio", coll["PolygonSlow.slowdown_ratio"][0],
         _get(nav, *cmp_, "PolygonSlow", "slowdown_ratio"), "safe-speed fraction", None),
        ("PolygonSlow.points", coll["PolygonSlow.points"][0],
         _get(nav, *cmp_, "PolygonSlow", "points"), "footprint + d_stop", None),
        ("coverage_server.operation_width", cv["operation_width"][0],
         _get(nav, *cov, "operation_width"),
         "= tool_width - swath_overlap (launch-injected)", None),
        ("coverage_server.default_headland_width",
         cv["default_headland_width"][0],
         _get(nav, *cov, "default_headland_width"),
         "chassis-derived (launch-injected)", None),
        ("footprint", res["footprint"]["footprint"][0],
         _get(nav, "global_costmap", "global_costmap", "ros__parameters", "footprint"),
         "chassis rect + margin (launch-injected)", None),
    ]

    def fmt(v):
        if isinstance(v, float):
            return f"{v:g}"
        return str(v)

    def diverges(d, c):
        if c is None:
            return True
        if isinstance(d, (int, float)) and isinstance(c, (int, float)):
            return abs(float(d) - float(c)) > 1e-6
        return str(d) != str(c)

    lines = []
    lines.append("=" * 100)
    lines.append(f"DERIVED ({res['_profile']}) vs CURRENT  ({label})")
    lines.append(f"  active wz ceiling = {ceiling} (firmware rate loop, Option C); "
                 f"wheel-saturation wz @ mowing speed = {wz_sat_mow}; "
                 f"vx breakaway = {vx_break}")
    lines.append("  HARD = physics violation (clamped/clipped/zeroed at runtime); "
                 "diverges = different but feasible")
    lines.append("=" * 100)
    lines.append(f"{'param':<54}{'derived':<18}{'current':<18}flag")
    lines.append("-" * 100)
    n_hard = 0
    for lbl, d, c, _note, hard_check in rows:
        if hard_check is not None and hard_check(c):
            flag = "HARD"
            n_hard += 1
        elif diverges(d, c):
            flag = "DIVERGES"
        else:
            flag = "ok"
        ds, cs = fmt(d), fmt(c)
        if len(ds) > 16 or len(cs) > 16:
            lines.append(f"{lbl}")
            lines.append(f"  derived: {ds}")
            lines.append(f"  current: {cs}   [{flag}]")
        else:
            lines.append(f"{lbl:<54}{ds:<18}{cs:<18}{flag}")
    lines.append("-" * 100)
    lines.append(f"{n_hard} HARD physics violation(s)")
    return "\n".join(lines)


# ===========================================================================
# CLI
# ===========================================================================
def build_argparser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Derive Nav2 params from robot physics + mowgli_robot.yaml.")
    p.add_argument("--robot-yaml", default="docker/config/mowgli/mowgli_robot.yaml",
                   help="Path to mowgli_robot.yaml (physical inputs).")
    p.add_argument("--nav2-base",
                   default="ros2/src/mowgli_bringup/config/nav2_params_base.yaml",
                   help="Path to nav2_params_base.yaml for --compare.")
    p.add_argument("--overlay", default="lidar",
                   choices=["lidar", "no_lidar", "none"],
                   help="Variant overlay deep-merged onto the base for "
                        "--compare (mirrors navigation.launch.py).")
    p.add_argument("--profile", default="calm",
                   choices=["calm", "responsive", "both"],
                   help="Derivation profile. 'both' prints two reports "
                        "(--report only; --yaml/--compare need one).")
    p.add_argument("--report", action="store_true",
                   help="Print human-readable report (default if no mode given).")
    p.add_argument("--yaml", action="store_true",
                   help="Emit a YAML fragment mirroring the nav2 params tree.")
    p.add_argument("--compare", action="store_true",
                   help="Tabulate derived vs merged base+overlay nav2 params.")
    # Physics knob overrides.
    for k, v in DEFAULT_KNOBS.items():
        p.add_argument(f"--{k.replace('_', '-')}", type=float, default=v,
                       help=(f"physics knob (default {v})" if v is not None
                             else "physics knob (default: per profile/model)"))
    return p


def main(argv=None) -> int:
    args = build_argparser().parse_args(argv)
    knobs = {k: getattr(args, k) for k in DEFAULT_KNOBS}

    if not os.path.isfile(args.robot_yaml):
        sys.stderr.write(f"robot yaml not found: {args.robot_yaml}\n")
        return 2
    rp = load_robot_yaml(args.robot_yaml)

    profiles = (["calm", "responsive"] if args.profile == "both"
                else [args.profile])
    if args.profile == "both" and (args.yaml or args.compare):
        sys.stderr.write("--yaml/--compare need a single profile "
                         "(use --profile calm|responsive)\n")
        return 2

    did = False
    for prof in profiles:
        res = compute_all(rp, knobs, BOARD_DEFAULTS, prof)
        if args.yaml:
            print(emit_yaml(res)); did = True
        if args.compare:
            if not os.path.isfile(args.nav2_base):
                sys.stderr.write(f"nav2 base yaml not found: {args.nav2_base}\n")
                return 2
            overlay_path = None
            label = os.path.basename(args.nav2_base)
            if args.overlay != "none":
                overlay_path = os.path.join(
                    os.path.dirname(args.nav2_base),
                    f"nav2_params_{args.overlay}.yaml")
                if not os.path.isfile(overlay_path):
                    sys.stderr.write(f"overlay not found: {overlay_path}\n")
                    return 2
                label += f" + nav2_params_{args.overlay}.yaml"
            nav = load_merged_nav2(args.nav2_base, overlay_path)
            print(emit_compare(res, nav, label)); did = True
        if args.report or not did:
            print(emit_report(res))
            did = True
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
