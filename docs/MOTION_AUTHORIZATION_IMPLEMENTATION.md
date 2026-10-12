# Motion authorization implementation and acceptance (#924)

This is a safety-critical, partial implementation of #924. It verifies FTC's
intended commands and bounds EscapeStartBlocked. It does not establish the
stack-wide autonomous-motion invariant or physical containment.

## Baselines and checkpoint

- Branch: `feat/924-motion-authorization`, isolated worktree `.motion-924`.
- Draft PR: [#950](https://github.com/mowglinext/mowglinext/pull/950), against
  `dev`; no merge, ready-for-review transition or physical deployment performed.
- Base: `dev` `9f39497f26f4759a0b26e601904710fb934beb65`.
- Original #942 regression source: `ddd49d9f59e202c265d5aca5e0d1154b79a721af`.
  Its branch and Draft PR were not modified. The original assertions and
  scenarios were retained; additional tests and real map-publication fixtures
  supply the geometry required by the production interface.
- Production checkpoint: `26926295`. Final-source native checks passed:
  navigation 22/22, behavior 46/46 and map 12/12 CTest targets. Later changes
  record test latency, exercise automatic map startup and update documentation;
  motion production is unchanged.
  Packaging checkpoint `0f66cd8c` adds Boost headers and visualization_msgs to
  the five thin sensor build dependency lists after CI exposed missing Boost.
  Test-only checkpoint `8671b3a4` verifies automatic permission publication;
  its expanded map suite passed under Fast DDS and CycloneDDS.
- #905 (`26a2515d`) and #906 (`a04c4219`) were open and unmerged when inspected.
  This imports #905-compatible raw MarkerArray publication and segment logic,
  not its transit planner or another geometry grid. Coordinate the overlapping
  publisher changes before either branch merges.

## Authorization policy

The reference point is the rear axle, `base_footprint`; `base_link` has the same
XY origin. FTC coverage commands keep that point in the recorded mowing union.
Blade-off Escape may use the recorded mowing, navigation and dock-corridor
union. A navigation corridor cannot grant FTC coverage permission. Body
extension beyond an outer boundary remains permitted by the axle policy and
bounded by the actual configured chassis. Raw forbidden holes and dock bodies
never permit overlap with the complete swept chassis. Their planner clearance
margins are not applied a second time to a complete chassis.

The shared verifier checks a constant outgoing Twist, including the entire
reference-point path and chassis sweep. Convex endpoint hulls padded by an
angular arc-to-chord bound enclose intermediate body positions. Analytic
circular-path intersections split axle arcs at polygon supporting lines;
every resulting interval is classified against the raw allowed union. Straight
paths split at polygon crossings, using exact binary-rational predicates for
ambiguous determinants and fractions. Strictly disjoint edge bounding boxes
and certified interval exclusions skip irrelevant edges without discarding
possible crossings. The point-on-boundary tolerance is 1e-8 m; it is not
localization uncertainty or permission to cross an obstacle.

Snapshots validate frame, marker purpose, topology, finite coordinates and
footprints before use. Limits are 512 markers, 32768 total points, absolute map
coordinates at most 1e6 m, and a finite 3-64-vertex chassis of radius at most
3 m. Content defines snapshot identity independently of cached publication
stamps. Empty or invalid snapshots revoke these consumers' permission. This
identity is not a producer epoch, edit sequence, map datum or action lease.

FTC's common final output gate caps and checks every nonzero return: nominal
tracking, deviation, reverse, turn fallback and rotation. It obtains a fresh
map-frame robot pose and the costmap's configured footprint. PRE/PIVOT/POST
states also validate the complete remaining directed rotation using the same
wrapped heading error as the controller. Each translational command verifies
0.1 s of intended execution, corresponding to the shipped 10 Hz controller.
The interval is not a final actuator lease or a guarantee against downstream
command retention. Zero commands bypass all geometry and pose prerequisites.
Raster costs can further reject commands; free or soft-cost cells cannot grant
polygon permission.

Escape consumes one existing fresh arming token, verifies blade-off feedback,
and moves opposite the validated last direction. Defaults remain 0.10 m/s,
0.40 m and 6 s; compiled ceilings remain 0.15 m/s, 0.60 m and 15 s. It validates
the complete straight configured envelope before starting, then checks current
geometry identity/generation, fresh pose, remaining sweep, measured/commanded
distance, 0.05 m lateral drift, 0.15 rad yaw drift and absolute monotonic timeout
every tick. A tick gap above 0.5 s ends the attempt. Each outgoing command also
certifies 2.5 s of intended held translation and tapers to remaining distance /
2.5 s. Full-speed budget accounting can end the nudge short of the configured
maximum. Recovery efficacy and firmware deadband behavior need further
integration and hardware evidence. Halt and rejection publish zero.

`motion_footprint` is an internal behavior-node parameter derived by both full
launch builders from the existing `chassis_footprint` helper, including its
existing 0.05 m navigation clearance. An ad-hoc node without the derived
footprint cannot perform Escape. No operator YAML setting is added. Existing
firmware emergency stop, blade/localization guards and collision processing
remain in their existing paths.

Map replacement revokes before fallible live mutation. Invalid reloads clear
partial replacement state and publish empty authorization; correcting the file
and reloading restores normal permission. A missing file before mutation
preserves unchanged live geometry. Rings, counts and indexed records are
validated strictly. All valid persisted and parameter holes are retained,
including coincident-centroid and duplicate rings. Orphan obstacle-array
entries reject startup; omitted trailing entries still mean no holes. Invalid
add-area requests reject without altering the live map.

## Motion-path acceptance matrix

| Producer / path | Command path and authorization in this slice | Evidence / remaining acceptance |
|---|---|---|
| FTC swaths, connectors, headlands, deviations | `computeVelocityCommands` common final gate -> Nav2 `/cmd_vel_nav` | 26 command cases, including original 12 negative and 4 positive controls; passed on the final production source |
| FTC PRE/PIVOT/POST and fallback reverse | Directed full rotation and final Twist sweep; existing costmap collision checks retained | Real-controller wheel-lag turn scenarios and complete footprint tests; passed on the final production source |
| Webots no-LiDAR full mission | Current coverage/interface/map/nav/behavior overlays plus cached remaining runtime; real launch and strict E2E harness | Attempt 15: route endpoint reached, but 0% stamped coverage and missing external dock detection caused FAIL. Simulator defects corrected and targeted checks pass; complete mowing/HOME/docking mission still unverified |
| EscapeStartBlocked | Bound action instance -> `/cmd_vel_nav`; geometry, pose, blade, arming and budgets; rejection/halt zero | 17 production-path DDS/command cases; passed on the final production source |
| Map permission production | #905-compatible `/map_server_node/transit_geometry`; `area.text=mowing|navigation`; revocation before replacement | 30 startup/reload/add/parameter cases; passed on the final production source |
| Nav2 RPP/RotationShim transit and HOME | BT `NavigateToPose` -> Nav2 -> `/cmd_vel_nav` | No new execution gate. #905 protects planning, not controller corner cutting. OPEN |
| Nav2 BackUp, BT boundary recovery and undock | BT `/backup` and recovery action paths -> Nav2 -> `/cmd_vel_nav` | Existing behavior retained; no bounded geometry exception implemented here. OPEN |
| Dock and dock calibration | `/dock_robot` and independent docking commands -> `/cmd_vel_docking` -> mux | No new execution gate; non-default approach/undock distances and charge/resume mission unverified. OPEN |
| Motion heading seed / yaw / IMU / magnetometer calibration | Independent publishers, including `SeedYawFromMotion` -> `/cmd_vel_teleop` | Sharing the teleop lane does not make an autonomous calibration manual. OPEN |
| Dig recovery | `hardware_bridge_node::dig_monitor_tick` -> `send_cmd_vel_packet`, bypassing ROS mux | Existing limits retained; no geometry execution gate. OPEN |
| Selection, shaping, final motor packets | Collision monitor -> twist_mux -> bridge slew/deadband shaping -> firmware; bridge also has direct sources | FTC/Escape validate their intended source commands. No owner/action/revision lease after shaping or final-output integration evidence. OPEN |
| Manual/operator and STOP | Existing teleop/tuning/emergency lanes; firmware emergency latch remains authoritative | No new manual geometry clamp; unconditional source STOP preserved. Final arbitration/cancel ownership unverified |

Source maps: [navigation controllers](../ros2/src/mowgli_nav2_plugins/src/ftc_controller.cpp),
[behavior navigation/recovery](../ros2/src/mowgli_behavior/src/navigation_nodes.cpp),
[calibration](../ros2/src/mowgli_behavior/src/calibration_nodes.cpp),
[hardware bridge](../ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp),
and [mux priorities/timeouts](../ros2/src/mowgli_bringup/config/twist_mux.yaml).

## Verification record

Original regression baseline was reproduced on exact dev plus unchanged #942
sources in a separate non-root ROS build: 12 FTC negative cases and the one
Escape negative case failed; four positive FTC controls passed.

Final-source navigation, behavior and map builds passed all **80 CTest targets**
(22 + 46 + 12), containing **968 GoogleTest cases** (263 + 484 + 221).
The focused suites contain 26 FTC command cases, 18 continuous
sweep cases, 17 Escape cases and 30 map startup/reload/add/parameter cases. All original
13 negative cases now pass and all four original positive FTC controls remain
green. Additional positive checks include real-FTC edge progress, 1000 legal
straight sweeps, 1000 curved headland sweeps around a hole, complete pivots,
connected polygon seams, and valid omitted trailing parameter entries.

The real FTC wheel-response fixture passed **100/100 scenario executions**:
20 repeats each of hedge U-turn recovery, fallback-disabled U-turn abort,
straight-wall abort, obstacle-during-recovery abort and no-LiDAR turn completion.
Each repeat contains two completions and three expected safe aborts, rather
than five completed missions. All executions report zero fixture lethal
samples. The unchanged dev/#942 baseline also passed all 100 executions of
those same existing fixtures. This establishes preserved fixture outcomes;
it does not measure full mission coverage or final actuator containment.

Fast DDS was used for the complete native suites. The four focused suites
(FTC, sweeps, Escape and map reload) additionally passed through CycloneDDS:
4/4 CTest targets originally containing 89 cases. The later expanded 30-case
map suite also passed under both implementations, with all 12 map-package
CTest targets rerun successfully. The two new cases exercise actual automatic
startup publication without `/map`: persisted geometry and the simulation's
parameter-defined lawn with a paused ROS clock. They neither invoke the
test-only mask rebuild nor substitute authorization evidence.

Host latency measurements below used production `26926295` and test-only
instrumentation `37af533e`. The dense-circle straight case previously averaged
226.411 ms before the certified edge exclusions, exceeding the shipped 100 ms
controller period. That comparison is to the prior implementation on the same
fixture, not an unmodified dev verifier: dev has no equivalent swept-geometry
verifier. No timing assertion is used to hide host scheduler variation.

| Geometry check | Samples | Mean | Largest observed check |
|---|---:|---:|---:|
| 32768-point circle, straight | 100 | 0.280 ms | 0.440 ms |
| Same circle, near-straight (`omega=1e-12`) | 100 | 3.85 ms | 5.17 ms |
| Same circle, curved (`omega=0.2`) | 100 | 0.845 ms | 1.393 ms |
| 32768-point rectangle, interior | 100 | 0.284 ms | 0.547 ms |
| Same rectangle, legal edge overhang | 100 | 0.225 ms | 0.350 ms |
| Complete 2-pi pivot with 49 holes | 3 | 10.45 ms | Not separately instrumented |

Repeated full-controller fixture means ranged from 0.033 ms (no-LiDAR overlay)
to 3.41 ms; the largest observed tick was 48.75 ms. These include existing
controller/costmap work. They are container/x86 host observations, not ARM
deadline guarantees or whole-stack CPU/memory overhead measurements. Across
20 runs, whole five-scenario GTest durations averaged 2280.9 ms on baseline and
2596.15 ms on the implementation. This includes setup/teardown and scheduler
variation; it is not isolated verifier CPU overhead or per-tick latency.

One additional paired process probe used `os.wait4` to obtain resource usage
for each actual test PID, avoiding inherited shell child-accounting. The five
controller scenarios passed again: baseline CPU user+system 2.441 s, peak RSS
58.6 MiB; implementation 2.959 s, peak RSS 60.1 MiB. The geometry-only 18-case
process passed with 0.824 s CPU and 32.2 MiB peak RSS. These single-process
samples include fixtures/frameworks, are sensitive to scheduling, and do not
isolate verifier overhead or predict a deployed stack's memory consumption.

Configuration/launch/URDF checks: **253 passed**, including both real full-system
launch builders with non-default chassis dimensions. These use lightweight
ROS launch adapters and execute the production builder code; they do not start
Nav2 or Webots. Firmware's 24 generated interface headers were checked, and Go
and TypeScript consumers regenerated with no drift in Linux under `LC_ALL=C`.
The additional existing OpenMower launch checks passed **11/11**. Repaired
sensor workflow YAML and all five explicit build dependency lists were checked.
Clean sensor CI at `0f66cd8c` passed GNSS (11 actual GoogleTest cases and
9 Python cases) and OpenMower (78 actual GoogleTest cases and 11 Python cases),
including newly generated interfaces and installed-consumer builds. OpenMower
amd64 and arm64 image builds and smoke tests also passed. The CI counter that
greps every XML `tests` attribute counts nested attributes twice; the case
counts here use GTest's actual executed-case output. The [full ROS workspace
Build & Test job](https://github.com/mowglinext/mowglinext/actions/runs/38014488887/job/114101679699)
also passed at test checkpoint `8671b3a4`: 14 source packages built and tested,
with colcon reporting 3107 tests, 0 errors, 0 failures and 369 skipped. That colcon total
includes framework/linter results and is not an additional GoogleTest count.
Despite the job's historical `ROS2 kilted` name, its actual setup is Lyrical.

Native builds use Docker image `be2f6e79c62e`, Ubuntu 26.04, ROS Lyrical,
`/opt/lyrical_vendor` and UID/GID 1000:1000. Generated message interfaces are
unchanged; source packages and added shared headers are rebuilt separately
against the image's cached generated-interface prefix. Full clean workspace,
coverage-planner package and physical hardware builds are not established by
those local focused checks. The separate CI workspace build covers the source
ROS packages, including coverage; it does not build firmware or prove Webots
missions or physical behavior. Local logs live in `.validation/`, excluded from Git.
Fresh final `mowgli_interfaces` CMake configuration also passed, finding
visualization_msgs and Boost 1.90.0. This validates configuration/export wiring,
not a new generated-message compilation or a clean installed-consumer build.

Independent Sol review found and prompted fixes for inward-arc rejection,
failed-reload revocation, unwrapped remaining rotation, shallow crossings,
malformed rings, count/index omissions, orphan parameter holes and unnecessary
exact arithmetic. Each has dedicated regressions. The final source review of
`26926295` found no remaining confirmed source blocker or should-fix in this
scoped FTC/Escape implementation. Its runtime conditions were checked by the
final native, transformed-seam, dense-polygon and repeated-controller runs.
The reviewer explicitly retained the full-#924 and hardware acceptance gaps.

## Simulation and resource limitations

An unchanged Webots E2E harness was executed, without a completed mission.
After Docker/WSL recovery and moving large
scratch artifacts to D:, Webots R2025a and the repository-pinned driver/control
revision `db4a77c84ee91445d39de9a631e5b57a888814e2` built and ran in a separate
bridge-networked container as UID 1000, ROS domain 170, with no robot devices,
host IPC or published ports. Xvfb, missing runtime libraries and Linux-native
Webots IPC were repaired in task-owned scratch; shared services were restored
after the user-authorized Docker restart and were not used for missions.

The current simulation launch/config/world/robot description and tested native
interfaces/navigation/behavior/map installations were overlaid on cached image
`be2f6e79c62e` (runtime provenance `df018f11`). Untouched components including
coverage and Nav2 still came from that cached runtime: this is a mixed-baseline
integration attempt, not a fully rebuilt release image. The configured
`main_mow` lawn loaded, robot-state publication initialized, and both
`joint_state_broadcaster` and `diffdrive_controller` reported active.

Earlier full-stack attempts failed readiness: docking exited with an undefined
`opennav_docking::Controller` symbol and Nav2 remained inactive. A subsequent
reliable transient-local probe delivered the real configured geometry from
the current map executable/library; `/map` is not required for this wall-timer
publication. Native startup tests establish that separately. The docking
loader failure was traced to case-insensitive Windows bind lookup: Webots'
`libController.so` shadowed ROS's `libcontroller.so`. Corrected loader order
resolves docking symbols while preserving Webots dependencies. Small Webots
controller libraries were staged in executable native `/tmp`, preserving
case-sensitive lookup and Linux IPC. Attempt 13 verified all eight navigation
lifecycle nodes, including docking, active; both drive controllers active;
and the genuine configured geometry delivered from the current map executable.
An initial CLI snapshot exhausted CycloneDDS participant indices; a later
snapshot returned all eight active states. Readiness used explicit node states,
not the generic lifecycle-manager log message.

The unchanged no-LiDAR E2E harness used its documented 3000-second budget,
accepted START, undocked and completed heading calibration. At 60 seconds it
reported approximately 1.0 m traveled, pose `(-0.96, 0.00)`, and zero coverage
path poses. The cached `df018f11` coverage executable generated a 9947-pose,
82-segment plan, then aborted with `double free or corruption (!prev)` and
exit code -6. Its missing heartbeat shut down the navigation group. No coverage
trajectory was followed, and no mowing/HOME/docking completion is claimed.
This mixed-baseline failure does not establish a regression in current
production coverage source; its cause remains unconfirmed.

The owned attempt-13 container was stopped below the 200 MB disk floor. Once
headroom recovered, current-source coverage was built on D: as UID 1000 against
the current interface overlay. All 8 focused CTest targets passed (128 GTests).
The build handoff identifies source `750eb4a1`; the runtime provenance file
does not independently record its Git SHA. The installed coverage executable
SHA-256 was `e0c4cc8409e2fd200d8029f30b9f73b08f92eecc4e2170a4f2ea48fb351fcc6b`.
Mapped-library evidence confirms the current coverage/interface/map overlays;
the schema-affected cached MQTT bridge was not launched. Other cached packages
still make this a mixed-baseline integration run.

Attempt 14h passed explicit readiness for all eight Nav2 lifecycle nodes,
coverage and both drive controllers, and received the real mowing polygon.
The unchanged no-LiDAR E2E harness ran its full default **3000-second budget**
in ROS domain 181. The mandatory session monitor started before START. The
read-only-rootfs task container disabled Docker duplicate logging and stored
the runner, session and pose logs on D:; it is stopped and removed.

Undocking and planning succeeded. Coverage produced 82 segments, 9947 poses
and a 383.8 m route. Four early FTC sweep denials aborted follow goals and
saved resume cursors 57, 118, 212 and 271; subsequent goals resumed and made
progress. The later saved cursor was 8157/9947 (82% of path poses, **not lawn
coverage**). At about 1920 seconds, charging became true during the follow
goal. ManualChargeGuard entered `MANUAL_CHARGING`, canceled motion and held
zero output while awaiting operator undocking. No override was injected.

The mission **did not complete**. At the full-budget timeout, the unchanged
harness additionally raised `AttributeError` from `RcutilsLogger.warn`, exiting
1. Its `Coverage path: 0 poses` metric was unusable despite the actual planned
and followed path; it also reported incompatible `/cmd_vel` QoS. The monitor
recorded 30421 samples over 3043.47 seconds, 319.66 m integrated wheel travel,
and final state `MANUAL_CHARGING`; the E2E harness reported about 313.7 m.
These distinct distance metrics have not been reconciled. Independent source
and receipt-time CSV assessment traced charging to the fake bridge's existing
axle-distance-to-origin rule: it asserts charging inside 0.3 m regardless of
heading or physical contact. At the rising edge, ground truth was approximately
`(-0.240,-0.150)` (0.283 m from origin), while fused pose was `(-0.548,-0.150)`.
Final truth remained about `(0.225,-0.156)` (0.274 m), preserving the charging
assertion and hold. The cached fake-bridge source has this same rule; battery
output was fixed at 28 V/full charge. This establishes a proximity-triggered
simulation hold, not a depleted battery or a new authorization failure. The
four FTC denials lack exact rejected-command evidence; full mowing is unverified.

All four last logged FTC states were FOLLOWING. Nearest receipt-time fused
poses lay 0.00075-0.00458 m inside the rectangular outer boundary, at different
edges. Their headings and preceding published commands are consistent with
perimeter overshoot, but those commands were previously accepted commands,
not the rejected Twist. The exact rejection pose, command and branch were not
logged. This cannot establish a verifier false positive or reconstruct its
precise sweep. No later authorization denial appeared. The smallest next
software investigation is one instrumented denial and deterministic replay,
not another blind full-mission run. Keep the execution invariant and existing
charging guard intact while investigating.

Ground-truth and fused-pose CSVs use different header time bases. Inspected
nearest receipt-time pairs differ by approximately 0.263-0.381 m, so raw truth,
fused map pose and wheel odometry must not be conflated. No boundary-violation
count is claimed without validating that alignment. Full attempt-14h logs
and monitor/pose data are retained under task-owned D: scratch. No physical
robot or shared-service mission was run.

Further verification uses a corrected simulation fixture: the same 54 m2 lawn
is shifted to `[-4.5,4.5] x [0.8,6.8]`, with a recorded navigation rectangle
`[-2.2,1.0] x [-1.0,1.8]` connecting origin dock/staging to the lawn. The
synthetic charging disk and every production charging/blade guard remain
unchanged. This geometry lives in the existing simulation scenario YAML;
real-robot map defaults are unaffected. First coverage poses remain on the
lawn, with blade-off Nav2 ingress rather than FTC coverage at the dock.

The no-LiDAR observer now uses the actual retained full-plan topic and stamped
post-mux command type. It requires observed undock/plan/mow/dock phases,
completed target paths without skips, actual docked charging, and the stated
median tracking threshold. `COVERAGE_INCOMPLETE` remains a failure even when
path counters later report completion. Timeout and signal termination preserve
the report's failure status. FTC rejection diagnostics record the actual
rejected pose/Twist and checked sweep; their geometry fingerprint is evidence
correlation only, not another permission identity.

Attempt 15 at `8fcb0176` executed the corrected no-LiDAR mission. Undock and
planning passed; the 9899-pose route progressed for 368.4 m, with median
tracking deviation 5.6 mm and p95 58.5 mm. Four early FTC denials were resumed.
The route reached its endpoint, but the real map reported 0% stamped coverage
and FollowStrip latched `COVERAGE_INCOMPLETE`. The harness correctly failed
mowing. Navigation to dock staging succeeded; docking then failed because no
external dock detection arrived. After over 60 s idle with zero command and no
charging, signal termination retained FAIL (exit 1). No guard was bypassed.
This is incomplete mission evidence, not preserved full-system functionality.
Logs show the simulator requested blade ON but reported an inactive blade
controller; production stamping correctly rejected it. GNSS projection also
reported an invalid-identity/provenance warning. Peak observed resource range was roughly
530-645% CPU and 1.3-1.45 GiB RSS; it is not isolated verifier overhead.

Targeted diagnosis confirmed two simulator producer defects. The fake bridge
always sent `mower_esc_status=0`, including during ON with 3000 RPM; it now
reports active only while ON. A real installed-process service/topic regression
fails against the old binary and passes OFF/ON/OFF against the correction, using
private test endpoints. Map-server verification remains unchanged.
The synthetic GNSS relay omitted `position_observation_sequence`; the current
production association explicitly rejects zero. The relay now pairs the exact
raw fix header with a nonzero sequence for each increasing raw acquisition plus
a new ground-truth generation. Duplicates, unset/future raw stamps, replay and
cached truth cannot refresh it. Both callbacks reset cached observations on ROS
clock rewind while retaining monotonic sequences.

All 60 simulator Python tests pass; 19 sweep tests include the four actual
attempt-15 rejected requests and original edge-overhang controls. The captured
requests are rejected by the real verifier; rotation-only counterparts remain
permitted. No authorization rule was relaxed. A stationary 60 s Webots probe
observed 262 fixes/261 zero-identity statuses with zero absolute poses or dock
detections before the relay correction. After correction it observed 260 fixes,
260 absolute poses and 599 real dock detections. This establishes the production
projection/detection connection, not successful physical docking or a completed
mission. Independent review found and resolved a clock-rewind regression.
Ground-truth headers currently use wall time while raw fixes use simulation
time; generation deduplication does not establish ground-truth acquisition-time
freshness. Receipt alignment and mixed cached packages remain limitations.

Available controller fixtures execute the real FTC with modeled wheel response
and simulated endpoints. They can establish command progress, expected aborts
and fixture collision checks. They do not constitute full mowing, connected
area, dock/undock/HOME/charging-resume missions or post-mux hardware-output tests.
Full mission completion is unverified; lawn coverage percentage, final-command
boundary violations and whole-stack CPU/memory overhead remain unmeasured.

The existing [OpenMower software hardware simulation](https://github.com/mowglinext/mowglinext/actions/runs/38010148929)
did execute on `0f66cd8c`: **178/179 checks passed** on its first attempt.
The failing `xesc_2040` 350 ms process-starvation scenario measured 0.224 m/s
instead of the unchanged 0.300 +/- 0.050 m/s progress requirement. Odometry
agreement, plausibility and board latch checks in that scenario passed. The
bridge/emulator/scenario sources are unchanged against the dev baseline; this
does not by itself explain the timing failure. The unchanged second attempt
passed **179/179 board/ESC checks and 10/10 stack seam checks**, with both
starvation scenes observing 0.299 m/s. The first unexplained miss is retained
as a repeatability limitation; no test, assertion or source was changed.
The preceding [dev simulation run](https://github.com/mowglinext/mowglinext/actions/runs/37965865987)
at `5733a399` passed 179/179 board/ESC checks and 10/10 stack seam checks;
the inspected bridge, simulation, hardware, message-schema and seam-launch
sources match this work's `9f39497f` dev baseline. It observed 0.300 and
0.299 m/s in the two starvation scenes. Its runner/runtime are separate evidence,
not an identical-host controlled comparison or an explanation of this failure.
The subsequent mux/bridge stack-integration script did not execute on the
first attempt because the board script returned failure. On the second attempt,
the real launch/mux/bridge moved emulated wheels through its teleop input and
stopped them after input release (970 ms observed, unchanged 1700 ms bound).
This emulates the OpenMower board/controllers
and charging/e-stop behavior; it is neither a Webots mowing mission nor evidence
for the FTC/Escape final-output invariant. No assertion was weakened.

## Unresolved safety and functionality acceptance

- Explicit producer epoch, revision and datum are absent. A map-server restart
  can leave cached permission, and depth-one DDS history can coalesce a
  revoke/restore transition. Escape generations cancel a delivered revoke,
  including restoration of identical content, but cannot detect an omitted one.
- FTC has no motion-owner/action lease. Source cancellation and geometry checks
  do not establish cancellation of a held, shaped or reselected downstream
  command. The final bridge may alter linear/angular proportions.
- Existing runtime obstacle-promotion centroid deduplication can report success
  while discarding a distinct candidate polygon. Pending-ID acceptance retains
  its polygon. This preexisting deferred gap prevents claiming protection of
  every successfully promoted forbidden region.
- Transit, docking, undocking, HOME, ingress, calibration and dig still require
  coherent bounded exceptions and final command authorization. Their normal
  missions have not been demonstrated on this implementation.
- Worst-case union construction, many intersecting rings and target ARM
  controller deadlines remain unmeasured. Host microbenchmarks cannot certify
  real-time hardware behavior.

## Physical acceptance

**HARDWARE_REQUIRED.** Record this branch's final ROS commit/image, firmware
image/hash, GNSS submodule/driver/receiver revisions, robot unit and configuration
hashes before running. No robot motion, firmware flash or physical deployment
was performed.

Remove blades, establish an exclusion zone, verify emergency stop, and use a
supervised low-speed fixture. Exercise repeated legal edge passes, pivots,
holes, blocked-start escape, connected transit, non-default dock/undock,
HOME/charge/resume and geometry revocation. Pass only if approved operations
progress and measured chassis motion stays within the declared geometry and
operation envelope. Rejected motion must stop within a separately measured
physical bound, with no blade/localization/emergency regression. Software
constant-Twist tests cannot establish slip, physical stopping distance, blade
safety or hardware reliability. Completing the open software paths is a
prerequisite to accepting the full #924 invariant.
