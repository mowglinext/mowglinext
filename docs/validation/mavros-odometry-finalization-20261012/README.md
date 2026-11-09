# MAVROS odometry finalization — new software validation, physical acceptance open

Owner MowgliNext, `feat/mavros-refresh`, HEAD
`1a2fdae5e1db6da5f5aeb4e814618b68cd51cebb`. This follow-up starts from the existing
uncommitted [aligned-window fix](../mavros-odometry-window-20261012/README.md),
which was not reapplied. Its historical 93/96 counts are superseded for the current
worktree by the new results below. No commit, staging, push, robot access, motion,
parameter write, deployment or gitlink/submodule change.

## Follow-up contract and exact changes

- `tools/motor/mowgli_tools/drive_pid_tuner.py`: a rejected aligned motor/RTK fit
  sets `rtk_accepted=false` and clears `calibration_window`. Existing raw RTK
  distance/speed and rejection notes are retained. Under this contract,
  `rtk_accepted` on an odometry trial means eligibility of the aligned fit,
  not merely a receiver reporting RTK FIXED. Actual stalls are also ineligible.
- The second RTK validation's actual selected observations own the final bounds,
  counter interpolation and distance. All selected observations must remain
  inside common counter coverage; no fallback to pre-intersection bounds.
  Both acquisitions and interpolation brackets still exclude command ramps.
- A failed/ineligible MAVROS report keeps current/starting parameters and every
  trial, but emits empty global `proposed_params` and `recommended`. No failed
  trial emits a per-pass coefficient. Earlier successful passes remain useful
  diagnostic evidence if a later pass fails; no session coefficient is proposed.
  Existing stop and baseline-restoration behavior is unchanged.
- `gui/pkg/api/drive_tuning.go`: decode the fit window, validate positive finite
  coefficient/deltas/distance, increasing positive ROS bounds, motor-revolution
  units, recognized source and epoch/backend metadata. Verify its arithmetic
  `(left+right)/2 / RTKdistance`, the same top-level RTK distance, and optional
  per-pass coefficient. The 1e-9 relative comparison is serialization/arithmetic
  consistency, not a physical accuracy tolerance. Sanitize nonfinite window
  numbers before JSON output while keeping them ineligible.
- Remove the old-coefficient odometry-distance percentage as a MAVROS success
  criterion. A coherent estimate announces “Coefficient calculated” and explicitly
  states that physical accuracy remains unvalidated after application, even if
  `applied_live=true`. Existing `not_validated` status avoids a false green
  “Validated” GUI tag; the report-only calculation still completes successfully.
  Measurement/motion warnings retain `warning`; true stalls and failed/partial
  sessions remain visible. No frontend components/status enums are added.
- Before MAVROS YAML persistence, require all odometry passes to have eligible,
  coherent windows, no session failure and a finite proposed coefficient equal
  to their mean (the Python session aggregation). Invalid/historical reports
  cannot overwrite the installed coefficient even if a legacy process exits0.
  This is report-contract validation, not a new motor safety/stop policy.
- Native feed-forward/PID evaluators, parameter allowlists and rollback paths
  remain unchanged. No ROS producer/public message, firmware, traction parameter,
  PID, mechanical calibration or FCU/VESC setting is modified.

Two regression test files complete the four-file follow-up:
`tools/motor/test/test_mavros_odometry_tuner.py` and
`gui/pkg/api/drive_tuning_odometry_test.go`. [review.diff](review.diff) is the exact
incremental diff against the already-modified starting worktree, not the earlier
full patch. `source.sha256` binds the relevant inputs to this validation.

## New validation on this worktree — 2026-10-12

| Local check | New result |
|---|---|
| All `tools/motor/test` | **111 cases PASS**, 0 skipped, 0.60 s |
| Isolated `colcon build` of `mowgli_tools` | PASS, 1.48 s |
| `colcon test` + `test-result` | **114 XML results**, 0 errors/failures/skips; 111 cases plus wrappers |
| Expanded Go drive-tuning contracts, including native FF/PID | **21 top-level tests + 35 subtests PASS**, package 0.029 s |
| Full repository `git diff --check` | PASS |
| Touched Go `gofmt -l` | Empty/PASS |
| Python follow-up changed-line flake8 E/F, 130 columns | PASS; whole-file baseline lint debt remains |

The final Go run includes the focused odometry cases and native contracts.
No full GUI/web or whole-workspace run is claimed. All executions were local,
uid1000, ROS Lyrical/Python3.14 and the available Go toolchain, not GitHub Actions
or the robot. Colcon used existing `/ros2_ws/install/mowgli_interfaces`, did not
rebuild public interfaces, and warned about that prebuilt dependency.

Python regression coverage now runs each provenance/motion rejection through
final `run()`/YAML with and without requested apply. It asserts the retained
failed trial, false eligibility, absent fit/per-pass coefficient, empty proposals,
no live fit apply (only existing baseline restore when requested), plus preservation
of good RTK diagnostic values on counter failure. It tests a second RTK selection
that narrows bounds and verifies both motor deltas on exactly those bounds, and
rejects a selection escaping the intersection. A two-pass failure retains both
trials but no global proposal. Earlier functional/native/safety regressions remain.
Initial new pytest execution exposed two outdated mocks returning no trials;
those mocks now return a measured trial while all their apply/report assertions
remain. Final reruns pass; no assertion/test was disabled or deadline raised.

Go covers old-scale wrong/absent/zero odometry with a valid fit, rejected and
historical windows, invalid units/coefficient/counters/distance/bounds/provenance,
ratio and serialized-value mismatch, warnings/stalls, multiple-pass aggregation,
YAML decode/JSON sanitization and actual mocked persistence refusal. Both
`applied_live=false/true` remain explicitly physically unvalidated. Native
status selection, FF/PID warnings and persistence allowlists pass unchanged.

Commands from the repository root after sourcing ROS setup:

```sh
PYTHONPATH=tools/motor:$PYTHONPATH python3 -m pytest tools/motor/test -q
colcon --log-base /tmp/odom-finalize-colcon/log build --base-paths tools/motor \
  --build-base /tmp/odom-finalize-colcon/build \
  --install-base /tmp/odom-finalize-colcon/install --packages-select mowgli_tools
colcon --log-base /tmp/odom-finalize-colcon/log test \
  --build-base /tmp/odom-finalize-colcon/build \
  --install-base /tmp/odom-finalize-colcon/install --packages-select mowgli_tools \
  --return-code-on-test-failure
colcon test-result --test-result-base /tmp/odom-finalize-colcon/build --verbose
```

From `gui/`:

```sh
GOCACHE=/tmp/odom-finalize-go-cache go test ./pkg/api -count=1 -json \
  -run 'Test(MavrosOdometry|MavrosRejectedReport|Evaluate(Odometry|FeedForward|PID)Report|OdometryWindow|DriveTuningInternalTier|PidOvershoot|SanitizeDriveTuningReport|Build(FeedForward|PID)Command|PersistedParamsForMode)'
```

Logs/JSON reports/reversible starting copies remain outside VCS:
`/tmp/odom-finalize-{pytest.log,colcon.log,go-targeted.json,go-contracts.json,lint.log}`
and `/tmp/odom-finalize-before/`. Source syntax, input hashes and unrelated tracked
file hashes are checked; HEAD and pre-existing owner dirty state are preserved.

## Physical acceptance checklist — HARDWARE_REQUIRED

The earlier bag's **43.9678226206 motor rev/m** is still diagnostic only, not
applied or revalidated physically. This task does not touch the operator's
currently loaded experimental **48 motor rev/m**. The recorded GNSS movement
following stopped counters remains an unresolved sensor/timing latency concern;
a matching ROS window or RTK FIXED cannot erase it. A ratio constructed from a
window cannot itself prove physical accuracy after applying that ratio.

Before a separately authorized powered run:

- Record exact deployed Next binary/worktree, MVR/GNSS pins, FCU/VESC firmware,
  robot unit, wheel roles, receiver/driver revisions, clocks, loaded coefficient
  and timeouts. Current source pins are MVR811eda29 / GNSS383caba3; the prior
  bag's actual deployed image/receiver baseline is not fully identified.
- Preserve validated mechanics. Require blade OFF, clear bounded straight area,
  operator supervision and independent stop. Owner companion-loss stopping
  acceptance remains open. No powered procedure is authorized here.
- Observe source-owned counters and RTK acquisitions alongside independently
  measured physical displacement/time. Establish GNSS physical latency; don't
  invent a lag compensation or use cached receipt as acquisition.
- Select the same stable interval common to all sources, with ramp observations
  and interpolation brackets excluded. Check independent ground distance against
  each wheel and the fitted estimate; quantify measurement uncertainty and require
  repeated coefficients consistent within it. Reject stale/reset/source-switch/
  unaccepted RTK windows, stationary ground with wheel rotation, or missing wheels.
- Any application/persistence requires separate operator authorization. Then run
  an independent, recorded verification pass against physical ground distance.
  Application alone is not acceptance; only that evidence can support a precision
  claim. The present report/API intentionally supplies no “validated odometry”.

## Authorized software freeze — 2026-10-12

The user subsequently authorized a scoped commit and push on the existing
`feat/mavros-refresh` branch. The earlier no-commit statements describe the review
stage. A fresh pre-commit run passed111 pytest cases (0.59s), colcon114 XML results
without errors/failures/skips (package build1.56s), and21 Go parent tests plus35
subtests (0.038s). `git diff --check` passes for sources. Source SHA256 values match
the reviewed worktree. Origin/feat/mavros-refresh refreshed to1a2fdae5, identical
to the local base. No FusionGraph/robot configuration or submodule changes are
included; MVR gitlink811eda29 already matches its checkout. Original bags and
unrelated documentation changes remain local. Only code/tests, the compact
relative-coordinate fixture and the two calibration validation folders belong
to this freeze; no temporary build/log output is published. Archived diffs retain
their original bytes; their local attributes distinguish patch-context whitespace
from source whitespace checks. Hardware acceptance/loaded48/diagnostic43.9678226
limits above remain in force. Exact commit and remote SHA are in Git history and
the final handoff.
