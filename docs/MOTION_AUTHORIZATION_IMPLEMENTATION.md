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
| Webots no-LiDAR mission cycle | Current coverage/interface/map/nav/behavior overlays plus cached remaining runtime; real launch and strict E2E harness | Attempt 17 PASS: undock, plan, mow, return-to-dock and charging; 2539 s. Three recovery cursor advances omit partial spans. Exact area coverage, explicit HOME command and charging-resume remain unverified |
| EscapeStartBlocked | Bound action instance -> `/cmd_vel_nav`; geometry, pose, blade, arming and budgets; rejection/halt zero | 17 production-path DDS/command cases; passed on the final production source |
| Map permission production | #905-compatible `/map_server_node/transit_geometry`; `area.text=mowing|navigation`; revocation before replacement | 30 startup/reload/add/parameter cases; passed on the final production source |
| Nav2 RPP/RotationShim transit and HOME | BT `NavigateToPose` -> Nav2 -> `/cmd_vel_nav` | Default staging transit completed in attempt 17. No new execution gate; #905 protects planning, not controller corner cutting. Explicit HOME interruption untested. OPEN |
| Nav2 BackUp, BT boundary recovery and undock | BT `/backup` and recovery action paths -> Nav2 -> `/cmd_vel_nav` | Existing behavior retained; no bounded geometry exception implemented here. OPEN |
| LiDAR collision catchers | Nav2 `/cmd_vel_nav` -> Collision Monitor -> `/cmd_vel_monitored`; signed forward/reverse/rotation regions with unchanged live-footprint guard | Rear-clear reverse trap reproduced and repaired; 27 isolated real-node cases and 10 registered integration tests pass. Continuous geometry authorization and complete LiDAR mission acceptance remain OPEN |
| Dock and dock calibration | `/dock_robot` and independent docking commands -> `/cmd_vel_docking` -> mux | One default simulated dock/charging cycle passed. No new execution gate; non-default distances, calibration and charge/resume remain unverified. OPEN |
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

Attempt 16 at `7b894c84` passed undock/planning and entered MOWING with verified
blade activity. It advanced 21.7 m, but 19 warnings showed a missing
`map -> blade_link` transform. It was stopped early with FAIL (exit 1), rather
than waiting for a predictable unrecorded-coverage result. Stamped area was not
measured; HOME/docking were not exercised. Two FTC denials resumed movement.
The Webots URDF now includes the fixed cutting-tool frame using the simulator's
existing 0.18 m chassis centre and 0.093 m wheel radius, matching production
xacro construction and placing the tool 0.05 m above ground. Map stamping and
blade guards are unchanged. All 62 simulator Python tests pass, including real
robot-state-publisher TF and configured xacro geometry checks; the final two
tool-frame tests also pass with `ROS_DOMAIN_ID` unset. Independent review
confirmed publisher ownership/geometry and resolved a test-domain mismatch.
Further runs must verify the live map/tool TF and installed URDF before START,
then record actual map-server stamped cells during mowing. Full mission
completion remains pending.

Attempt 17 at `929e8378` passed the actual no-LiDAR mission cycle in 2539 s
(42.3 min), exit 0: undock 11.9 s, plan 46.4 s, mow 2412.5 s, dock 46.8 s,
ending `IDLE_DOCKED` and charging. The strict harness reported 1/1 complete
paths and zero wholly skipped paths, with no `COVERAGE_INCOMPLETE`. Final global
stamped raster was 21,712 cells / 54.28 m2; this is not an area-specific coverage
percentage. The existing area plausibility guard checks only 50% interior
stamping when a usable grid exists. Three authorization aborts caused approximately 0.8 m recovery cursor
advances with blade-off transit; those partial omissions do not increment the
whole-path skip counter. The goal checker also accepted the endpoint with
0.499 m remaining. Neighboring passes may overlap these spans; actual uncovered
area has not been established. This demonstrates a progressing mission cycle,
not every planned span executed blade-on or near-total physical lawn coverage.

The harness measured 366.6 m travel and median fused-pose nearest-path error
5.3 mm (p95 57.9 mm, 55,297 samples); these are not ground-truth tracking errors
or checks against the intended segment. Three short GNSS guard pauses recovered
automatically. The pre-START monitor recorded 23,417 samples. Live spot samples
were about 1.45-1.47 GiB and 500-600% CPU, not isolated authorization overhead.
The real dock action completed and final truth was within the simulator's
0.3 m charging circle. This does not validate physical dock contact. Logs also
show dock-prior/RTK discrepancies of 0.52-0.57 m, a final 0.59 m gauge reset,
eight failed periodic graph saves and a failed dock-arrival save. These runtime
diagnostics require follow-up; a healthy covariance verdict does not resolve
position disagreement. Independent evidence review confirmed the harness PASS
and these limits. LiDAR, explicit HOME interruption, charge/resume, revocation
and additional geometries still lack full-mission evidence.

LiDAR attempt 18 at `9fdcf8c3` stopped before START with a failed readiness
verdict. All base nodes/controllers/tool TF were ready, and raw/costmap/collision
scan streams had increasing nonzero acquisition stamps. The collision-monitor
bulk parameter response contained no values, so its effective guard configuration
was not verified. A startup LiDAR-OFF warning describes the absent installed
configuration's default; source review confirms explicit CLI overrides take
precedence, so that warning does not establish the active mode. The empty
response needs isolated parameter queries, including separating unused legacy
polygons from required active guards. No mission, LiDAR obstacle response or
coverage result is claimed. A fresh task-owned graph directory was writable as
UID 1000 and live graph save/autosave settings were correct; save success was
not exercised. The next isolated attempt uses an explicit sparse LiDAR config
and verifies actual guard parameters before authorizing START.

Attempt 19 at `8573be31` passed the live LiDAR/guard readiness checks, including
all required active polygons and independently queried parameters. Three setup
parameter tests passed; unused legacy polygon and PointCloud-only height keys
no longer mask required LaserScan guard settings. The subsequent mission was
stopped by a task-observer false positive: its 30 s timer armed on the umbrella
MOWING status before planning and blade-off transit. At termination transit was
still progressing, with no recorded mowing. Genuine blade ON and verified
stamping began later during serial cleanup after observers stopped. This is not
evidence of a mower refusal or successful LiDAR mission. Harness exit was 1;
runner/container exit was 31. Sensor streams were advancing, and 355 paired
nav/monitored commands matched, but no collision response was observed.

The next task-only observer arms from current mowing/path context plus fresh
advancing direct blade telemetry. It compares blade stamps against ROS clock,
uses monotonic liveness/deadlines, observes phase changes in callbacks, excludes
intentional OFF pauses and caps credit at previous evidence expiry. The same
30 active-second progress requirement and independent 60/3000 s limits remain.
Nineteen deterministic policy/real-ROS-adapter checks pass. Cleanup now stops
simulation execution before stopping evidence recorders. Production guards and
canonical harness assertions were not changed; LiDAR mission acceptance remains
pending.

LiDAR attempt 20 at `5cc78033` passed those readiness checks and completed the
undock, plan and mowing phases: 11.7, 38.1 and 2345.4 seconds respectively.
The strict whole-cycle verdict was **FAIL**, exit 1 at the unchanged 3000-second
limit. It observed 1/1 completed paths, no wholly skipped paths, 374.4 m travel
and 20,103 global stamped cells (50.2575 m²); these are not a physical coverage
percentage. Short blade-off resume-cursor advances remain a coverage limitation.
The corrected task watchdog passed with fresh blade-active grid growth and did
not interrupt the mission. Live collision-monitor logs show slowdown and
approach responses. Seven LiDAR TF future-extrapolation errors and transient
GNSS/pose freshness rejections occurred; guards stopped or paused motion.
Eight periodic graph saves succeeded with a private writable map directory.
An auxiliary collision-state recorder expected a nonexistent message header;
its failure limits recording and does not invalidate the separate readiness
probe or establish continuous collision-state evidence.

After MOWING_COMPLETE, a localization guard halted the pending dock dispatch.
The docking client canceled only an already-polled goal handle, allowing the
late-accepted goal to survive the halt. On restart, its terminal callback wrote
into the outcome slot reused by the replacement goal. Logs show staging
preemption, an old-goal abort, a second server docking attempt, overlapping Nav2
navigation and NAV_TO_DOCK_FAILED. Source inspection confirms both lifecycle
windows. A focused DockRobot/UndockRobot repair now gives every dispatch its
own acceptance/result state, cancels late acceptance after halt or destruction,
and waits for its previous action's terminal result before replacement. A
five-second steady-clock drain timeout refuses replacement and retains the old
identity; it does not establish that cancellation succeeded. Boundary exemption
is not enabled while draining. Callbacks no longer capture the destroyed BT
node, and goal-handle operations occur outside the dispatch mutex.

Four real-ROS cancellation regressions failed against the preceding production
code; all **14 new transport cases pass**, including normal instant success,
rejection, destruction, restart, rejected cancellation and concurrent result/halt.
The focused run passed **7/7 targets**. After correcting only CRLF in the scratch
XML copy, one complete behavior run passed **47/47 CTest targets**, including
42 GoogleTest targets with 498 cases and zero failures/errors. Its first
full run was 44/47; two XML structure failures then passed, while the unchanged
manual-charge debounce test failed twice in its complete target and passed alone
before passing in the final full batch. These earlier timing failures remain
repeatability limitations; no assertion or test source was weakened.

All 96 behavior build inputs match the checkout after CRLF normalization only.
The installed behavior binary SHA-256 is
`0025fbeccb5629805cd77000130077a5e7c42905c351c06b7e5adc1d3ee41d26`.
An independent source review found no additional confirmed defect within this
per-node repair. Cross-node action ownership, a rejected cancellation's one-shot
send and indefinite transport retention for a never-answered acceptance remain
limitations. This is not a geometry exception or a final-output cancellation
lease. LiDAR attempt 21 at `1b0f50a1` used this verified installed binary and
failed the strict cycle after about 1590 seconds. Undocking passed in 11.7 s
and the independent blade-active growth check passed (0 to 352 cells in
30.0166 s). It traveled 214.8 m and stamped 12,809 global cells (32.0225 m²),
but completed 0/1 paths. Following two localization-guard pauses, FTC reverse
and Nav2 BackUp/transit failed to make progress; three controller progress
failures and four AREA_UNREACHABLE transitions exhausted the five-attempt
budget. COVERAGE_FAILED_DOCKING began at 1583.6 s. The run was stopped after
at least 135 s without distance growth; fallback docking had only just begun,
so this run neither validates nor disproves completion of the docking repair.
Six FTC denials and three boundary warnings are retained. Path-error median
15.8 mm and p95 64.0 mm measure fused nearest-path tracking, not containment.

Investigation identified two separate defects. First, the sparse task config
omitted the scenario datum: fusion/COG self-seeded from the antenna's initial
fix while map/navsat used the world reference, producing about 0.30 m of
world-X displacement. All earlier mixed-runtime mission claims remain
bounded by that frame inconsistency; a passing cycle does not validate
physical map alignment. The next isolated fixture must provide and verify
the same explicit datum for every consumer.

Second, the installed Nav2 Collision Monitor 1.5.2 reproduced a rear-clear
reverse being stopped by two returns in the extra front catcher outside the
body. `approach` checks current polygon occupancy before projecting motion,
so the static front band can also freeze reverse. The production correction
uses supported `velocity_polygon` selection: exactly-zero linear commands
select the full padded body, negative commands select that body united with
a rear catcher, and positive commands select the unchanged original forward
band. Inclusive first-match ranges cover every finite signed velocity without
an epsilon gap. Launch derives all points from the same operator chassis,
three-decimal footprint and explicit existing 0.01 m local-costmap padding.
Two-point sensitivity, projection horizon/step, source/TF rejection and the
live three-point FootprintApproach guard remain enabled.

An initial 25-case proposal used raw overlay geometry. The subsequent actual
production-helper run passed **27/27 real installed-node cases**, including
positive reverse/turn progress, forward/rear and padded-body stops, curved and
corner sweeps, custom rounding/padding, zero padding, tiny signed commands,
stale scans and missing TF. Separate real Costmap2DROS probes confirmed default,
custom and zero padding. The checked-in registered node integration test passed
**10/10 tests** (nine motion tests plus shutdown) in 30.589 seconds; its outputs
must match current input stamps. Two parameter-generation cases execute the
production launch builder using action adapters and inspect its generated YAML
and footprint-rewrite argument; they do not launch the full ROS stack.
Independent review found no confirmed blocker in this focused correction.
The complete bringup Python/configuration batch passed **274/274 cases**.
Its first complete collection required explicit non-launch file selection;
five package-index setup failures then passed after loading the workspace and
fresh bringup install. Both new CMake-registered targets passed (2/2).

The two-point subpolygons, like the preceding static catcher, are regenerated
at bringup; they do not follow live footprint/padding changes. FootprintApproach
still follows the live body at its existing three-point threshold. This adds
no geometry exception or action lease, and docking's mux lane still bypasses
Collision Monitor. Helper work occurs at launch; selector/body-sweep overhead
and target-ARM deadlines are not measured. Retained attempt-21 scans omit beam
endpoints and cannot prove
the precise triggering returns. No full LiDAR mission PASS or stack-wide #924
acceptance is established.

Datum-aligned LiDAR attempt 22 at `c8c64e13` passed all live selector and
five-consumer datum checks before START (81 stationary receipt-aligned pairs,
maximum XY difference 38.5 mm, wrapped yaw difference zero). The strict mission
**failed after 385.8 seconds**: undock 12.0 s and planning 39.1 s passed;
mowing failed after 267.8 s with 0/1 paths complete and about 1% cursor progress.
Five attempts were exhausted. It traveled 29.5 m and stamped at most 759 global
cells (1.8975 m²), not a physical coverage percentage. The watchdog did not fire.
Fallback docking **and simulated charging passed in 50.7 s**, reaching
IDLE_DOCKED. This is useful docking functionality evidence, not a mowing-cycle
PASS or physical charger-contact test.

All 181 recorded reverse requests had same-stamp monitored outputs at their
requested -0.15 m/s. Successful mowing recovery was not established. Two FTC
denials occurred during FOLLOWING through rounded outer headland corners.
One reference pose started 1.198 mm outside the recorded max-X boundary; the
other started 0.629 mm inside min-Y but its proposed 0.1 s arc crossed it.
Those requests must remain rejected. Earlier translation/steering response and
residual wheel velocity require a short closed-loop reproduction before a
controller repair can be accepted. Receipt-aligned ground truth was still
inside at both denials; fused pose/projection differences cannot be treated as
proof of physical boundary crossing or permission to ignore an estimated pose.
Later RPP collision-ahead/controller-patience and repeated transit retries also
failed. Independent transit commands remain outside this slice's execution gate.

Raw collision-beam records were retained in attempt 22; the monitor still
records only path summaries, so exact full-plan replay is unavailable.
The mixed-runtime and clock/provenance limitations remain. A complete LiDAR
mowing cycle, a measured final-command containment result, and physical
acceptance remain outstanding.

A focused comparison reproduces a headland functionality problem through
the real FTC library, with identical geometry, TF, parameters and modeled wheel
response. Its representative legal rounded rectangle is not the unrecorded
attempt-22 full plan. Original `9f39497f` dev finishes after 726 model steps but
records 417 reference samples outside the rectangle (maximum X 4.50486 m,
minimum Y 0.79522 m). This branch aborts after 266 steps with no outside sample
before the abort; extending the model with zero-command coast records 39
outside samples and a final X of approximately 4.5001 m. Neither outcome passes
the combined progress-and-containment requirement. These are modeled controller
results, not a physical stopping-distance measurement.

An isolated brake/heading-alignment experiment completes the basic unbiased fixture
without a recorded crossing and preserves all 26 FTC authorization assertions.
It is **not included in production**: its early forecast assumes the fixture's
0.15 s response, a 0.04 s future-TF probe fails progress under its finite retry
budget, and signed 3 mm pose bias can leave the estimate inside while modeled
truth crosses. Earlier unbounded probes also exposed numerical-edge behavior.
Behavioral checks are not a robustness or full-mission PASS. The real Nav2
velocity argument has no acquisition timestamp; it supplies no freshness or
physical stopping guarantee. Exact full-plan recording has separately passed
a real ROS 9,899-pose transport and startup CLI check.

Strict LiDAR attempt 23, using the same reviewed runtime and the full-plan
recorder, **failed after 293.2 s**. Undocking (11.6 s), planning (40.3 s), and
fallback docking/charging (53.9 s) passed; mowing failed after 173.3 s, with
0/1 paths completed and 29.7 m total travel. The watchdog did not trigger.
Four complete plans were retained. All 9,899 initial reference points lie
inside the configured outer rectangle; this is not a complete trajectory or
footprint certificate. The actual first corners have roughly 45-degree heading
changes on either end of 3.586 mm and 3.606 mm chamfers, without coincident
planner pivot twins. They differ materially from the representative 0.2 m
radius fixture. Two FOLLOWING denials again rejected an outside-start pose and
an outward intended arc. A progress repair must now address the actual planner
and controller corner contract, preserve the existing body/hole gates, and be
validated against the captured path. No corrected full-mission PASS is claimed.

The production coverage planner now treats a microscopic headland bevel as
one corner only when a supported tangent fillet fits across its two supporting
edges. The four-point window admits ordinary edges from 0.1 mm to 10 mm,
requires consistent turn direction and forward tangent ordering, and bounds
the virtual intersection within 10 mm of both bevel vertices. That intersection
is never emitted alone. Existing minimum-radius and sampled geometry checks
remain; GEOS additionally checks every emitted chord and tangent connector
against the complete boundary and holes. A rejected proposal keeps the original
vertices. This changes sparse headland-ring smoothing only; the final
normalization, explicit pivot twins, FTC execution gate and chassis-overhang
policy retain their existing behavior.

A real Fields2Cover production-pipeline regression uses the configured ordinary
rectangle and the observed operation/headland widths, five passes and 0.20 m
minimum radius. The preceding source fails with a 45.0579-degree maximum
heading kink; the corrected outer ring has a 5.0555-degree maximum. Summaries
of the four inner rings are unchanged. The native original plan has 9,896
poses versus the captured mission's 9,899, so this is a reproduced planning
defect, not exact regeneration of that mission. All **138 coverage GoogleTest
cases pass**, including nine new seam controls and the new real-pipeline
regression. A complete non-root ROS coverage-package build and install passes
**9/9 CTest targets**, including those six GoogleTest targets and three linters;
the overlapping counts are not additional cases. Unchanged real-controller
FTC **26/26** and Escape **17/17** assertions also pass.

Independent strong review found no supported new production blocker in this
narrow increment. It identified an incomplete rectangle predicate in the
separate modeled-controller evidence. The corrected common oracle checks all
four bounds for truth, estimates and zero-command coast, with the original
numerical tolerances. Three direct oracle controls pass, including left/top
coast rejection. Both nine-case modeled prefix batches pass again: captured
vertices passed through the corrected ring seam (175 poses), and separately
regenerated production output (174 poses), each ending after the second corner.
All 18 cases finish with zero outside samples under that complete rectangle
oracle, including 40 additional coast steps. Cases cover no-LiDAR and an empty
real LiDAR costmap, TF leads through 0.08 s, response constants 0.10/0.15/0.30 s
and signed 3 mm XY biases. This is selected wheel-response model evidence,
not actual mission LiDAR, physical stopping distance or whole-route acceptance.
The raw-path and naive-pivot progress failures remain RED. The reviewer reran all
nine seam tests and the pipeline regression, and exercised 20,000 randomized
bevel inputs. No full corrected Webots mission has run yet. A separately
constructed beveled-field probe still has unsupported later 33.75-degree and
90-degree corners before and after this change; general corner feasibility is
not established. The inherited radius-retry routine also needs configuration
validation for nonfinite or extremely large radii; shipped finite radii were
used in the reported checks.

Webots attempt 24 stopped before E2E START: the unchanged Nav2 readiness gate
failed because `is_path_valid` was unavailable after its existing 10-second
wait. Copied coverage binary hashes and package-prefix checks passed, but live
mapping and the later datum/selector gates were not reached. No mission verdict
exists for that attempt. Its 24 startup resource samples are not mission or
isolated authorization-overhead measurements.

The first two attempt-25 launches also stopped before E2E START, from test-runner
encoding and helper-order defects. A partial service probe did not establish
the Nav2 failure's cause. Lead checks found and repaired an additional syntax
defect before the next launch; shell/Python/PowerShell syntax, encoding, helper
ordering and actual checksum accept/reject checks passed. The corrected third
launch passes Nav2/LiDAR readiness and verifies the exact live coverage executable
and mapped core library. The original readiness, geometry, datum and canonical
mission assertions remain in force. The earlier service failure's cause is
undetermined; passing readiness is not a full corrected mowing-cycle PASS.

Available controller fixtures execute the real FTC with modeled wheel response
and simulated endpoints. They can establish command progress, expected aborts
and fixture collision checks. They do not constitute full mowing, connected
area, dock/undock/HOME/charging-resume missions or post-mux hardware-output tests.
One no-LiDAR mission cycle now passes; exact lawn coverage percentage,
final-command boundary violations and isolated authorization overhead remain
unmeasured.

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
  coherent bounded exceptions and final command authorization. One default
  no-LiDAR cycle demonstrates normal undock, ingress and return-to-dock;
  other variants, explicit HOME and charging-resume remain unverified.
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
