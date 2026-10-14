# FCU parameter compatibility diagnostic

MowgliNext owns this browser tool and its software validation. Work on existing
`feat/mavros-refresh`, HEAD `35986cd0e573d5e2647679b469c70c29a72dfed1`.
MowgliMAVROS `bf5bb0db3e4e3a0784ecd486e4c4c7909c0063e7` and both parent gitlinks
are preserved. No commit, device access, parameter write or deployment.

## Scope and operator clarification

The supplied Pixhawk 5X / ArduRover 4.7.1 light export is an **inventory**, not a
known-good configuration or a template to copy. A difference from it is not an
incompatibility. Its 15,896 bytes contain 894 parameters, 893 CRLF terminators,
no final newline and no embedded firmware/board identification. SHA256:
`c4f1162d20fabcfe1c583304f9cee924c368a5ca93a2c6b6714bebd0c89e8a8e`.
The complete personal export is retained **locally only** in the Git-ignored
`local-only/` directory, together with the pre-review evidence (including the
old diff that embedded it). It must not enter a public commit without explicit
operator authorization. Public parser tests use only
`gui/web/src/utils/fixtures/synthetic-runtime.param`, generated independently:
12 synthetic parameters, BOM, comments, tabs/numeric formatting, CRLF and no
final newline. No full personal configuration is a test/build dependency.

Settings → Hardware retains Chassis and the entire FCU YAML firmware builder.
A separate runtime-parameter tab uses observed firmware identity: ArduPilot,
PX4 or generic when unknown. Import identity is independent of a connected FCU:
explicit file headers or operator identification are required. Conflicting
headers, absent versions and unsupported firmware do not activate a profile.
Initial comparison scope is **ArduRover 4.7.1 only**. This narrow software scope
is not physical certification; no PX4 profile is implemented.

## Rules and technical basis

`gui/web/src/utils/fcuParamProfile.ts` is the rule registry. Each emitted rule
carries its exact version scope, hardware/feature applicability, justification,
source and pinned backend contract link. Applicability and firmware gating precede
comparison. Editable binding inputs describe the installed setup; their initial
values are not universal recommendations. Logical SERIAL port and MAVLink stream
channel are separate inputs.

| Contract | Diagnostic behavior | Verifiable basis |
|---|---|---|
| MAVROS source/target system IDs | Always “To verify” until effective installed identities and GCS acceptance policy are confirmed; no FCU renumbering proposal | Owner `mavros_backend.launch.py`, firmware provider, ArduPilot MAVLink documentation |
| Identified serial transport | Accept MAVLink 1 and 2; propose 2 only for a non-MAVLink existing protocol on the identified link | ArduPilot MAVLink configuration documentation |
| Propulsion and blade outputs | Conditional on selected hardware/DO_SET_SERVO channel; “To verify”, no guessed output assignment | Owner firmware provider and bridge; ArduPilot DO_SET_SERVO documented admissible functions |
| VESC / DroneCAN | Only chosen VESC topology; CAN driver/protocol/output mask reviewed without changes | Owner ESC signed-feedback contract and configured actuator channels |
| Legacy RPM | Only selected VESC legacy feedback; slots differ from node IDs; raw signed motor RPM/scaling contract reviewed | Owner `wheel_tick_projection.hpp`, `MM-ESC-ODOMETRY-20261005` |
| Telemetry | Review stream delivery/acquisition freshness and link budget, no reference rate proposed | Owner telemetry tracker/projection, MAVLink stream documentation |
| ARM and safety/failsafes | Read-only review of installed policies; no presumed ARMING_REQUIRE or skip-mask correction | Owner FCU sequencer, existing safety paths and `MM-FCU-MAIN-COMMIT-20261011` |
| Navigation and sensors | Read-only review; do not infer universal receiver, EKF, compass or IMU choices | Owner GUIDED velocity/yaw-rate provider and bridge navigation inputs |
| Calibrations/preferences | Preserve values and numeric formatting; informational preservation rows only | No requirement to reproduce another robot |

Unvalidated hardware rules have **no proposed correction** and report “To verify”.
They cannot be selected individually or globally. Missing/duplicate parameters
cannot be invented or corrected: absence may mean a partial export or an omitted
light-firmware feature. Optional unrelated logging configuration is not audited.
SYSID input defaults (255/1), manually entered values and observed FCU identity
are not proof of the effective MAVROS source/target configuration or GCS
acceptance policy. SYSID rows remain read-only, including matches to defaults
and multirobot IDs; a mismatch may need a MAVROS-side fix. No confirmation
workflow for enabling SYSID corrections is introduced by these review fixes.
Safety-critical rules are excluded from bulk corrections. The exporter also
requires explicit confirmation and a nonempty justification for any future
technically established sensitive correction. That export guard creates no
robot authorization service or functional policy.

Version-name inventory reference:
[ArduPilot parameter renames](https://ardupilot.org/rover/docs/common-param-name-changes.html).
`ARMING_SKIPCHK` and old `ARMING_CHECK` have opposite bitmask meanings; there is
no mechanical conversion. Protocol/stream source:
[MAVLink configuration](https://ardupilot.org/rover/docs/common-mavlink-configuration.html).
Blade source:
[DO_SET_SERVO](https://ardupilot.org/dev/docs/mavlink-move-servo.html).

## Original storage and export

Separate localforage database `mowglinext-fcu-params`, store
`immutable_originals_v1`; the existing YAML storage is untouched. Save must
succeed before exposing a working copy. Import IDs are unique; existing IDs
cannot be overwritten through the application. Same-app writes are serialized.
“Immutable” describes application behavior, not resistance to browser storage
clearing or manual developer-tool modification. Download important originals.

The parser preserves BOM, comments, spacing, line endings, ordering and unselected
numeric spans. Export changes only existing, uniquely identified selected numeric
values. Restore does not overwrite backups. No ROS parameter/set, ARM, actuator,
firmware build, flash or upload operation is introduced.

## Validation evidence

See `results.json`, `review-tests.log`, `review-typescript.log`, `review-eslint.log`,
`source.sha256`, `preservation.json` and `review.diff`. The diff contains this task's
code/configuration changes; pre-existing documentation redistribution is excluded.
The earlier full targeted baseline passed 49 tests, TypeScript, lint and Vite
build; its logs remain available. After review, only affected parser/profile,
parameter-tool and locale suites plus TypeScript and targeted lint are rerun.
The old Hardware switcher/YAML builder and submodule code have not changed. No code in
the backend or submodule changed. Source formatting checks cover the added files;
C++ formatting is inapplicable. Vite outputs are outside the repository in `/tmp`.

## Remaining acceptance — HARDWARE_REQUIRED

A reliable parameter-only comparison cannot prove an installed control path,
electrical direction, actual acquisition freshness or companion-loss stopping.
To graduate a hardware review item to an enforceable rule, first record: Next
commit plus tested worktree hashes, owner MAVROS commit, exact firmware image/hash
and runtime version (light feature manifest included), board/robot unit, VESC
firmware/node IDs, physical output/bus wiring, configured MAVROS role indices,
original parameter export/hash and receiver/driver revisions. Those physical
baselines have not been acquired by this task.

Procedure for a separately authorized safe bench: use the owner's established
bench plan, mechanically isolate blade and traction, provide an independent
physical stop, capture actual parameter and wire telemetry, check link/identity,
configured actuator routing, legacy/common feedback, confirmed HOLD/ARM behavior
and GUIDED signs/units. Observe correct logical-to-physical roles, no unintended
motion, valid signed units and genuine acquisition freshness. Any unexpected
motion, wrong role/direction or stale physical observation is failure. Safety,
arming and failsafe choices require developers' explicit installation-specific
review and independent stopping validation; this browser tool proves none of them.

Owner's detailed open gates:
`ros2/src/external/mowglimavros/docs/architecture/fcu-session-controller.md`.
Independent total companion-loss stopping remains open there. Owner TODO/MM-702
is not modified by this task (explicit submodule read-only constraint); software
UI implementation does not establish its physical known-good reference gate.
