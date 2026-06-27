# Physics-derived Nav2 parameters — design note

Companion to [`compute_nav2_params.py`](compute_nav2_params.py).

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

Derived floors keep every commanded motion decisively above these:
`VelocityDeadbandCritic` vx = `deadband_margin (1.1) · vx_breakaway` ≈ 0.15;
RPP `min_approach_linear_velocity` ≈ 0.16; behavior_server
`min_rotational_vel` ≈ 0.90 (field value 0.85 — same math). The host bridges
the deadband (hardware_bridge `min_linear_vel` guard + gyro PI rate loop /
pulse modulation), but commands that *live* inside the deadband still dither.

### The rate loop is an INPUT, not an assumption

`angular_rate_loop_enabled` (mowgli_robot.yaml, default true per
mowgli.launch.py) decides the wz ceiling:

- **ON** (field-validated default — OFF made the robot stick/oscillate): the
  host gyro PI clamps |wz| at `angular_rate_max_cmd = 1.5 rad/s`. Anything
  above 1.5 in Nav2 (wz_max, pivot rates, Spin max) is **silently clamped**.
- **OFF**: kinematic in-place cap `2·MAX_MPS/track ≈ 3.08`, but the open-loop
  rotational deadband bites; wz_std additionally floors at `0.5·wz_breakaway`
  so MPPI samples actually cross the deadband.

`--compare` flags ceiling violations as **HARD** (vs mere DIVERGES).

### Wheel-saturation coupling (the oscillation mechanism)

Diff-drive: `wheel_speed = |vx| + |wz|·track/2`, firmware-clamped at MAX_MPS.
So vx and wz are coupled: at mowing speed 0.20, any `wz > 2·(0.5−0.2)/0.325 =
1.85` clips one wheel — the executed twist deviates from the commanded one,
MPPI's DiffDrive model mispredicts, and the correction loop oscillates. The
2026-06-12 `wz_max: 2.0` bump violates this (and the 1.5 rate-loop clamp).

## Profiles

Hard physics applies to both; profiles choose *within* the feasible envelope.

| | `calm` | `responsive` |
|---|---|---|
| wz_max policy | no clipping ever: `min(ceiling, wz_sat @ vx_max)` | accept ≤10% per-wheel over-ask: `min(ceiling, 2·(1.1·MAX_MPS − vx_max)/track)` |
| vx_std | 0.5·vx_max (clean straights) | 0.75·vx_max (= 0.15 field value; 0.10 lost U-turn recovery, 2026-06-07) |
| wz_std | 0.22·wz_max | 0.30·wz_max |
| pivot wheel fraction | 0.5 (→ ~1.54 rad/s) | 0.65 (→ ~2.0 rad/s — the field value) |

A/B in the field with `mow_session_monitor.py`; `--profile both` prints both
reports.

## The rest of the derivations

- **Footprint** — mirrors `navigation.launch.py` exactly (0.05 m margin all
  around, origin at the rear axle): inscribed/circumscribed radii from the
  ORIGIN, `inflation_radius ≥ circumscribed` for SE2 correctness (the repo
  intentionally runs lower — documented trade, flagged DIVERGES not HARD).
- **MPPI horizon** — `model_dt = 1/controller_frequency`, `time_steps =
  horizon/model_dt`; prediction distance `= time_steps·model_dt·vx_max` sets
  GoalCritic/PathFollowCritic `threshold_to_consider` (0.72 m at 0.20 m/s —
  the ref 1.4 assumes a 0.5 m/s robot).
- **Collision monitor** — `d_stop = v²/(2·decel) + v·t_reaction` sizes
  PolygonSlow; PolygonStop geometry is derived but stays out of the active
  set (CLAUDE.md invariant).
- **Coverage** — `operation_width = tool_width − swath_overlap` (launch
  injection), F2C headland `= 0.5·chassis_width + margin + 0.5·op_w`,
  `keepout_nav_margin = footprint_half_width + tracking_margin`.
- **Goal checkers** — coverage xy `= fraction·tool_width` clipped ≤ 0.15;
  transit tolerances are operator values.

MPPI **critic weights** (PathAlign / GoalCritic / CostCritic…) are NOT
derived — genuine behavioural trade-offs, hand-tuned. Only their geometric
*thresholds* are derived.

## --compare and the base+overlay structure

`--compare` deep-merges `nav2_params_base.yaml` with the chosen overlay
(`--overlay lidar|no_lidar|none`) using the **same** recursive merge as
`navigation.launch.py::_deep_merge`, then tabulates derived vs current with
two flag levels:

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
  no_lidar variants (`test_nav2_params.py` pins them in lockstep).
- `closed_loop: false` on the coverage controller (deadband chassis).
- `coverage_goal_checker` stays `PathProgressGoalChecker`,
  `progress_threshold` 0.95.
- Coverage xy tolerance stays clipped ≤ 0.15 m.
