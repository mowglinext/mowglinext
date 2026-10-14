# MAVROS odometry calibration — aligned plateau, software PASS

MowgliNext `feat/mavros-refresh`, HEAD `1a2fdae5e1db6da5f5aeb4e814618b68cd51cebb`.
MowgliMAVROS HEAD/gitlink `811eda29c1fe7097907d6687ec560fb56100d802` and Universal
GNSS gitlink `383caba3de94e16167764393d5a4ef046078b015` are unchanged by this task.
No hardware access, motion, deployment, parameter write, commit, push or staging.
Pre-existing GUI, CI, documentation and submodule work remains in place.

## Input and baseline limits

Root `rosbag_20261012T161719Z.tar.gz`, SHA256
`9e8f3662f7bc5926f546c5fff310e62fb24eda8b8fd0127db8c0ad7145bd1ac3`, contains one MCAP.
Original archive remains intact, outside the proposed patch. Extraction/decoded
positions and logs live only under `/tmp/odom-second-*`. The YAML is the user's
pasted `20261012T161738Z_ff.yaml`, not a separately retrieved file: loaded coefficient
343, report-only, requested speed 0.4 m/s, 8.5 s pass, old summary wheel ~0.051,
RTK 0.372, stall=True, empty trials and failure "observed no wheel motion".
Deployed binary/firmware image/receiver revision and robot unit are not independently
identified by that report; current repository pins do not prove its build baseline.

## Causes and focused changes

1. Final odometry metrics treated coefficient-derived underspeed as physical stall.
   With `OdometryParams` in phase `odometry`, both wheel activities and accepted
   ground-speed evidence now decide this diagnosis. Low scaled speed stays an
   explicit calibration warning. Native `DrivePidParams` behavior is unchanged.
2. The prior estimator divided whole-trial unsigned WheelTick activity (including
   ramps) by a shorter RTK distance. The new fit uses source-owned signed motor
   counters and RTK endpoints in exactly the same ROS observation window. It
   interpolates counters only between proven observations, never extrapolates.
   Both endpoints and their interpolation brackets lie inside the commanded flat
   plateau: start+ramp through planned end-ramp. Full acceleration/deceleration
   ramps and stop/hold time are excluded; existing command generation is unchanged.
3. Match GNSS quality/sequence/backend to each absolute pose's exact observation
   header. Explicitly convert observation age from ROS time into the monotonic
   trial-window domain; receipt is not acquisition. Reject missing identity,
   stale/future poses, quality loss, repeated/restarted sequence, source changes
   and gaps beyond the existing odometry timeout.
4. Consume the existing `/diagnostics` entry `mavros_esc_wheel_odometry/source`
   only for the MAVROS tuner. Use active source, raw motor revolutions, per-wheel
   sample stamps, validity, epoch and segment. Cached identical observations are
   deduplicated; changed cache, rollback, source/epoch/segment change, invalid
   observations, excessive gaps and missing brackets reject the fit. This adds
   a consumer and its `diagnostic_msgs` package dependency, no producer/message
   change. No independent RPM integration is introduced.
5. Missing/rejected RTK or a missing aligned motor window cannot produce a successful
   calibration. The failed trial is now retained in the YAML instead of disappearing
   from `trials`; previous completed passes are retained too. Reporting uses the
   aligned coefficient, not the former whole-trial quotient.

Native parameters remain encoder counts/m; MAVROS parameters remain motor
revolutions/m. MAVROS WheelTick uses a transport factor of 1000; that factor is
not a mechanical conversion and does not make native and MAVROS coefficients
interchangeable. Historical mechanical measurements are untouched. The estimator
is restricted to `OdometryParams(ticks_per_meter)`; no gains, fictitious PID
parameters, mass/URDF changes or safety threshold changes are included.
Existing live emergency/charging/lost-odom/lost-RTK checks, runaway/oscillation
handling and motion cleanup are preserved. Valid RTK stationary ground with
turning wheels is rejected as stall/slip; missing wheel activity also rejects.

## Offline result — diagnostic only, not applied

The actual callback replay and the compact translated-relative fixture give:

| Same accepted window | Result |
|---|---:|
| UTC start | 2026-10-12 16:17:43.995160983 |
| UTC end | 2026-10-12 16:17:49.517098472 |
| Duration | 5.521937489 s |
| Left / right motor delta | 98.3276387013 / 97.8468743643 rev |
| RTK endpoint distance | 2.2308872873 m |
| RTK mean on this window | 0.4040044444 m/s |
| Old coefficient-derived wheel mean (diagnostic plateau) | 0.0504521120 m/s |
| Diagnostic `(left+right)/2 / distance` | **43.9678226206 motor rev/m** |

Motor source `ardupilot_legacy`, epoch2, both segments1; GNSS `mavros_gps2`.
See [diagnostic metrics](diagnostic-metrics.json). This narrower common window
explains why its RTK mean differs from the old reported 0.372; no speed-command
ratio is used as the coefficient. Whole-trial ticks and integrated old odometry
remain diagnostic fields, explicitly not inputs to this fit.

The bag also shows RTK-position movement after the motor counters stop. Published
observation identity/quality does not prove zero physical sensor latency. This
candidate is therefore not physical acceptance, an automatic installed value or
proof of a universal factor-eight mechanical fault. No invented lag compensation
or geometry/gearbox/pole change is applied. Original trial timing is derived from
its log; replay uses the 1 s configured ramp and 8.5 s planned duration, with
stop completion at +8.75 s. Fit end uses the planned plateau boundary, not that
stop-completion assumption. Fixture coordinates have only a translated XY origin,
no geographic datum; original private positions are not copied into this patch.

## Software validation

- All `tools/motor/test`: **93 pytest cases PASS**, no skips (0.41 s).
- Isolated non-root `colcon build` of `mowgli_tools`: PASS (0.38 s).
- `colcon test` / `test-result`: **96 XML results, zero errors/failures/skips**
  (includes 93 cases and package test wrappers; not 96 independent scenarios).
- Syntax compilation and scoped diff whitespace: PASS. Changed Python lines have
  no flake8 E/F findings (130-column profile). Full-file lint still has baseline
  long lines/indentation and an unused `resolved_cmd_topic`; no full-lint PASS claim.
- Cases cover the real low-scaled-speed trial, exclusion of ramps, aligned bounds,
  identical cache, missing/invalid RTK identity and quality, stationary robot,
  wheel spin without ground movement, one wheel absent, GNSS gaps/restarts,
  counter resets/gaps/segments/epochs, stale poses, success/failure final YAML,
  report-only no parameter apply, native PID and existing live safety failures.

Run as uid1000 with ROS Lyrical and existing `/ros2_ws/install` interfaces sourced:

```sh
PYTHONPATH=tools/motor:$PYTHONPATH python3 -m pytest tools/motor/test -q
colcon --log-base /tmp/odom-second-validation/log build --base-paths tools/motor \
  --build-base /tmp/odom-second-validation/build \
  --install-base /tmp/odom-second-validation/install --packages-select mowgli_tools
colcon --log-base /tmp/odom-second-validation/log test \
  --build-base /tmp/odom-second-validation/build \
  --install-base /tmp/odom-second-validation/install \
  --packages-select mowgli_tools --return-code-on-test-failure
colcon test-result --test-result-base /tmp/odom-second-validation/build --verbose
```

Build scope is the tools package, not the whole workspace. Colcon reports using
prebuilt `mowgli_interfaces`; no interfaces rebuilt. Runtime methods in tests
use uninitialized tuner objects and mocked actuation/parameter clients: no node
publishes motion. Tests run locally, not GitHub or on the Rock5B. Temporary
logs: `/tmp/odom-second-pytest-final.log`, `/tmp/odom-second-validation-final.log`,
`/tmp/odom-second-replay.log`, `/tmp/odom-second-lint-ef.log`.

## Remaining acceptance — HARDWARE_REQUIRED

Before a separately authorized run, identify the exact Next binary/worktree,
MVR and GNSS pins, FCU/VESC image/parameters and left/right roles, receiver/driver
revision, robot unit, effective timeouts, clocks and mechanical baseline. Keep
validated geometry unchanged. Verify the deployed diagnostic entry/fields are
available with unambiguous per-wheel acquisition identity, and GNSS quality and
pose timestamps match. Independent stop must be available, blade off, clear
bounded straight area and operator supervision; the owner's unresolved total
companion-loss stop guarantee remains a separate gate. No powered test is
currently authorized.

Record command ramps, counter acquisitions and RTK acquisitions together with an
independent measured ground displacement/time reference. Compare only a bounded
constant-motion interval common to all three, without ramp observations or
interpolation brackets crossing them. Check physical GNSS latency rather than
assuming a source header removes it. Repeat safely at established speeds and
compare per-wheel/common-window coefficients with independent distance; explicitly
record measurement uncertainty and require consistency within it. Reject cached,
stale, discontinuous or rejected-RTK windows, stationary ground with wheel rotation,
missing wheel movement, or ambiguous timing. Inspect a failed YAML for its retained
trial and confirm normal stop cleanup. Only after physical timing/scale acceptance
and a separate authorization may the operator consider applying/persisting a
coefficient; this investigation has not done so. Calibration does not retune FCU
PID or fix MANUAL/GUIDED steering, source scaling or target-expiry warnings.
