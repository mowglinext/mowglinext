# Physics-derived Nav2 parameters — design note

Companion to [`compute_nav2_params.py`](compute_nav2_params.py).

> **Coverage-slot caveat.** The script was written while `FollowCoveragePath`
> was MPPI. Since 2026-06-19 that slot is `mowgli_nav2_plugins/FTCController`
> again (`nav2_params_base.yaml`), so every `FollowCoveragePath.*` MPPI row
> below — `vx_max` / `wz_max` / `vx_std` / `time_steps` / the critic
> thresholds — has **no counterpart in the live config**: `--compare` prints
> `current = None` and DIVERGES for all of them. The FollowPath (RPP),
> behavior_server, costmap, collision-monitor, coverage-server and
> goal-checker rows are still live.

## Problem

Most values in `nav2_params_base.yaml` are not free constants — they are
functions of the chassis geometry, the firmware wheel-velocity limits, the
drive-motor deadband, and the operator's mowing/transit speeds. Those physical
inputs already live in `mowgli_robot.yaml` and are editable from the GUI. When
an operator changes `chassis_width` or `mowing_speed` the dependent Nav2
params should move with them, but today most are static literals re-tuned by
hand. This tool encodes the math so the params can be *derived*, *checked*
(`--compare` flags hard physics violations), and eventually *injected at
launch*.

## Hardware model (what the math is grounded in)

### Drive motors — per-model spec table (`MODEL_SPECS`)

All supported mowers (YardForce Classic 500 / 500B / SA650) share the GForce
mainboard family: **brushed-DC drive motors on a PAC5210**, firmware per-wheel
clamp `MAX_MPS = 0.5`, `TICKS_PER_M = 300` (which doubles as PWM_PER_MPS:
PWM 300 ≡ 1 m/s), wheel track 0.325 m.

| | Classic 500 | Classic 500B | SA650 ECO/B |
|---|---|---|---|
| Official weight | 8.8 kg | 8.8 kg | 8.5 kg |
| Drive motor | brushed DC, PAC5210 | same | same IC, larger winding |
| xESC motor-current limit (stock motor) | 2.0 A | 2.0 A | 6.52 A |
| `motor_force_n` (force budget, both wheels) | 9.0 N | 9.0 N | 13.5 N |
| → accel budget a = F/m | ~1.03 m/s² | ~1.03 m/s² | ~1.59 m/s² |

Sources: YardForce manuals (ManualsLib — note the manuals' "50 W rated power"
is the **blade** motor; drive-motor torque and OEM gear ratio are documented
nowhere), Mowgli firmware (`board.h`, `cpp_main.cpp`), OpenMower xESC configs
(`YardForce_Classic_Drive_Motor.xml` vs `YardForce_SA650ECO_Drive_Motor.xml` —
the 3.26× current limit + different flux linkage confirm a different motor).
The Classic force budget is field-validated (reaches 0.5 m/s in <1 s at
8.76 kg); the SA650 gets a conservative 1.5× (not 3.26×) pending field data —
grass traction and ride comfort cap the useful budget. Override with
`--motor-force-n`.

### Deadband (the anti-oscillation core)

`cpp_main.cpp`: the PAC5210 brushed motors have a hard static-friction
deadband at **~PWM 40** on the 0–300 scale:

```
vx_breakaway = pwm_deadband / pwm_per_mps        = 40/300 ≈ 0.133 m/s  (per wheel)
wz_breakaway = 2 · vx_breakaway / wheel_track    ≈ 0.82 rad/s          (in-place)
```

Derived floors keep every commanded motion decisively above these: RPP
`min_approach_linear_velocity` ≈ 0.16 (live value 0.16); behavior_server
`min_rotational_vel` ≈ 0.90 (live value 0.85 — same math). A third floor,
`VelocityDeadbandCritic` vx = `deadband_margin (1.1) · vx_breakaway` ≈ 0.15,
is MPPI-only and has no counterpart under FTCController. `hardware_bridge`
zeroes any |vx| below `min_linear_vel` (runtime param, default 0.05) rather
than boosting it, and the firmware yaw loop bridges the rotational deadband,
but commands that *live* inside the deadband still dither.

### The rate loop clamp

2026-07-17 Option C (task #33/#34): the gyro yaw-rate loop moved from the
host into FIRMWARE and is no longer a host-toggleable `angular_rate_loop_enabled`
ROS param — it runs unconditionally, and `hardware_bridge`'s `on_cmd_vel`
forwards wz **unshaped**. The script still models a ceiling of
`HW_ANGULAR_RATE_MAX_CMD = 1.5 rad/s` and treats anything above it in Nav2
(wz_max, pivot rates, Spin max) as **silently clamped** — but that number is
carried over from the removed host default and is *not* a firmware clamp:
`cpp_main.cpp` limits the yaw loop's differential **trim**
(`YAW_TRIM_LIMIT_MPS_DEFAULT = 0.15 m/s`), not |wz|. Treat 1.5 as a modelling
assumption until the firmware reports a real ceiling.

`--compare` flags ceiling violations as **HARD** (vs mere DIVERGES).

### Wheel-saturation coupling (the oscillation mechanism)

Diff-drive: `wheel_speed = |vx| + |wz|·track/2`, firmware-clamped at MAX_MPS.
So vx and wz are coupled: at mowing speed 0.20, any `wz > 2·(0.5−0.2)/0.325 =
1.85` clips one wheel — the executed twist deviates from the commanded one and
the tracking loop oscillates. The 2026-06-12 `wz_max: 2.0` bump violated this
(and the 1.5 ceiling); it went away with MPPI, and FTCController has no
`wz_max` at all — its live angular cap is
`FollowCoveragePath.max_cmd_vel_ang` (0.8), comfortably inside the bound.

## Profiles

Hard physics applies to both; profiles choose *within* the feasible envelope.

| | `calm` | `responsive` |
|---|---|---|
| wz_max policy | no clipping ever: `min(ceiling, wz_sat @ vx_max)` | accept ≤10% per-wheel over-ask: `min(ceiling, 2·(1.1·MAX_MPS − vx_max)/track)` |
| vx_std | 0.5·vx_max (clean straights) | 0.75·vx_max (= 0.15, the MPPI-era value; 0.10 lost U-turn recovery, 2026-06-07) |
| wz_std | 0.22·wz_max | 0.30·wz_max |
| pivot wheel fraction | 0.5 (→ ~1.54 rad/s) | 0.65 (→ ~2.0 rad/s) |

Both pivot rates are then capped at the wz ceiling, so today both profiles
emit 1.5 rad/s — and FTCController has no `rotate_to_heading_angular_vel`, so
that row too has no live counterpart (only `FollowPath`'s does).

A/B in the field with `mow_session_monitor.py`; `--profile both` prints both
reports.

## The rest of the derivations

- **Footprint** — mirrors `navigation.launch.py` exactly (0.05 m margin all
  around, origin at the rear axle): inscribed/circumscribed radii from the
  ORIGIN, `inflation_radius ≥ circumscribed` for SE2 correctness (the repo
  intentionally runs lower — documented trade, flagged DIVERGES not HARD).
- **MPPI horizon** (MPPI-era, no live counterpart — see the caveat at the
  top) — `model_dt = 1/controller_frequency`, `time_steps = horizon/model_dt`;
  prediction distance `= time_steps·model_dt·vx_max` sets
  GoalCritic/PathFollowCritic `threshold_to_consider` (0.72 m at 0.20 m/s —
  the ref 1.4 assumes a 0.5 m/s robot).
- **Collision monitor** — `d_stop = v²/(2·decel) + v·t_reaction` sizes
  PolygonSlow; PolygonStop geometry is derived but stays out of the active
  set — it is `enabled: false` in both overlays since 2026-07-21, replaced by
  the velocity-projected `FootprintApproach` (plus `PolygonStopNarrow` on the
  LiDAR variant).
- **Coverage** — `operation_width = tool_width − swath_overlap` (launch
  injection), F2C headland `= 0.5·chassis_width + margin + 0.5·op_w`,
  `keepout_nav_margin = footprint_half_width + tracking_margin`.
- **Goal checkers** — coverage xy `= fraction·tool_width` clipped ≤ 0.15,
  but `navigation.launch.py` instead **floors** the live value at FTC's
  `max_goal_distance_error` (0.50 m, so a parked FTC can satisfy the goal) —
  this row therefore always DIVERGES. Transit tolerances are operator values.

MPPI **critic weights** (PathAlign / GoalCritic / CostCritic…) are NOT
derived — genuine behavioural trade-offs, hand-tuned. Only their geometric
*thresholds* are derived, and since the FTC restore neither weights nor
thresholds exist in the config any more.

## --compare and the base+overlay structure

`--compare` deep-merges `nav2_params_base.yaml` with the chosen overlay
(`--overlay lidar|no_lidar|none`) using the **same** recursive merge as the
launch path — both import `deep_merge` from
`mowgli_bringup/launch/robot_config_util.py` — then tabulates derived vs
current with two flag levels:

- **HARD** — physics violation: the current value is silently clamped
  (rate-loop ceiling), clips a wheel (saturation at mowing speed), or gets
  zeroed (below firmware breakaway). These cause model mismatch → oscillation
  or dead commands; fix them regardless of tuning taste.
- **DIVERGES** — derived differs but current is feasible (tuning choice or
  launch-injected at runtime — the static literal in the file is not what
  runs).

Run it against the DEPLOYED `mowgli_robot.yaml` (`docker/config/mowgli/`),
not the template — deployed-config drift is a recurring failure mode (e.g.
deployed `chassis_length: 0.54` vs measured 0.60 in the template).

## Usage

```bash
# Both profiles, every formula:
python3 ros2/scripts/compute_nav2_params.py --report --profile both

# Compare (calm) against base + lidar overlay, flag HARD violations:
python3 ros2/scripts/compute_nav2_params.py --compare --profile calm

# YAML fragment for the responsive profile:
python3 ros2/scripts/compute_nav2_params.py --yaml --profile responsive

# Use the in-repo template instead of the deployed config:
python3 ros2/scripts/compute_nav2_params.py --compare \
    --robot-yaml ros2/src/mowgli_bringup/config/mowgli_robot.yaml

# Override a physics knob:
python3 ros2/scripts/compute_nav2_params.py --decel-max 1.5 --report
```

Read-only and idempotent: never touches running config, launch files, or the
nav2 YAMLs.

## Future: launch injection

The plumbing exists — `navigation.launch.py::_inject_dock_pose_and_speeds()`
already rewrites mowing/transit speeds, footprint, tolerances, and coverage
geometry from `mowgli_robot.yaml` into the merged temp YAML. Extending it to
import `compute_all()` and merge the `emit_yaml()`-shaped tree would make all
of this live. Invariants the injector MUST preserve:

- `FollowCoveragePath` stays value-for-value identical across the lidar /
  no_lidar variants apart from `check_obstacles` /
  `enable_obstacle_deviation`
  (`test_nav2_params.py::test_coverage_controller_aligned_across_variants`).
- `closed_loop: false` on `FollowPath` / RotationShim (deadband chassis) —
  FTCController has no such knob.
- `coverage_goal_checker` stays `PathProgressGoalChecker`,
  `progress_threshold` 0.95.
- Coverage xy tolerance stays **≥** FTC's `max_goal_distance_error` (0.50 m).
