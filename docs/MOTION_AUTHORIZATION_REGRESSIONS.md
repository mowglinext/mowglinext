# Autonomous motion authorization regression baseline (#924)

This is a software acceptance baseline for [#924](https://github.com/mowglinext/mowglinext/issues/924),
before the 1.6 execution-authorization architecture. It adds tests and test registration only.
It does not repair motion behavior or establish robot/field safety.

## Baseline and related work

Fresh worktree, clean latest `dev` at `be4e0ec3ed1aff605f3d7c1ff04f23308aee90c8`.
The production code under test is unchanged from that commit.

- [#905](https://github.com/mowglinext/mowglinext/pull/905), head `9f2f047e9881ab72a93a70fa7048fb418c6b17ec`,
  handles new transit plans, polygon-union segments, smoothing, terminal seams and planning invalidation.
- [#906](https://github.com/mowglinext/mowglinext/pull/906), head `6e8da63e391518116b51093e03de32ceba6278f6`,
  includes that work and handles garden-grid extent, lifecycle and obstacle updates.
- Their related issues [#903](https://github.com/mowglinext/mowglinext/issues/903) and
  [#902](https://github.com/mowglinext/mowglinext/issues/902) were read. These planning tests are not duplicated.
  Neither pending branch was changed, rebased or merged, and their runtime behavior was not retested here.
- Adjacent pending work was inspected: #909 (FTC progress), #920 (dock stall), #923 (dock/datum restore),
  #879 (startup cancellation), #858 (blade dispatch), #761 (manual ownership), #756 (LiDAR startup).

## Reproduce

Use the repository's normal non-root ROS Lyrical workspace and pinned dependencies:

```bash
source /opt/ros/lyrical/setup.bash
source /opt/lyrical_vendor/local_setup.bash
colcon build --packages-up-to mowgli_behavior mowgli_nav2_plugins \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Intentionally nonzero until the safety implementation satisfies these tests.
colcon test --packages-select mowgli_behavior mowgli_nav2_plugins \
  --ctest-args -R 'test_(motion|escape)_authorization' --output-on-failure \
  --return-code-on-test-failure
colcon test-result --verbose

# All existing tests in the touched packages, including lint targets.
colcon test --packages-select mowgli_behavior mowgli_nav2_plugins \
  --ctest-args -E 'test_(motion|escape)_authorization' --output-on-failure \
  --return-code-on-test-failure
```

Both new executables use `ament_add_ros_isolated_gtest`. The FTC fixture configures the real controller
and local costmap, supplies fresh consistent map/odom/body TF and controlled clocks, and publishes
global grids over real DDS. A pre-existing read-only friend verifies that the production subscription
consumed each relevant grid. It never injects an authorization verdict. The two obstacle-mode flags
match LiDAR/no-LiDAR coverage; this is not a launch-stack or sensor-driver test.

The escape fixture runs the real `EscapeStartBlocked`, with arming, direction and fresh verified
blade-off evidence, and observes its real `/cmd_vel_nav` publication. Reliable delivery is established;
a RUNNING case must produce an observed command before halt; halt zero is checked separately. No geometry/pose/TF or
named recovery envelope is supplied.

## Observed software results

Non-root isolated Linux container, ROS Lyrical with the repository's pinned Nav2 dependencies,
Fast DDS (the CI middleware), current `dev` interfaces generated from source. The controller library
and behavior sources are built from this checkout, without any production edits. Clang-format
18.1.8 checks both new test files. Standalone CMake/CTest builds use the same ament test registrations
as the colcon reproduction above.

The controller target includes positive controls for both forward translation and the same pi/2
PRE_ROTATE used by the forbidden-sweep case. The negative translation checks cover both the axle
boundary and the complete chassis sweep through a hole beside an otherwise permitted axle path.
Both obstacle modes run every scenario.
The final controller run reports **16 cases: 4 passing controls, 12 failing safety assertions,
no fixture errors**. Both modes produce the same counterexamples:

| Required check | Observed output on this baseline |
|---|---|
| Missing / unknown geometry | Forward peak `0.18 m/s`, where zero is required |
| Explicit all-forbidden replacement during execution | Forward peak `0.20 m/s`, where zero is required |
| Complete intended translation segment | Axle `x=0.6883385908` to `x=0.7075047318` crosses forbidden `x=0.70` |
| Complete translating chassis sweep | Axle `x=0.060` to `x=0.075` remains permitted; the body sweep reaches `x=0.605`, intersecting the hole starting at `x=0.600` |
| Forbidden PRE_ROTATE footprint sweep | Angular peak `0.8 rad/s`, where zero is required |
| Missing recovery geometry with valid arming/blade/direction | Escape publishes `-0.10 m/s` on `/cmd_vel_nav`, where zero is required |

All **20 existing navigation-plugin CTest targets pass**, including its 17 existing gtest executables
and three lint targets. The existing Nav2-parameter, robot-geometry utility and configuration-drift
Python suites report **162 passed**. These are software checks, not motion on a robot or sensor simulation.

The escape run reports **1 case, 1 expected safety failure, no fixture errors**, with halt zero observed.
All **40 existing behavior gtest executables pass**. Its copyright, cppcheck, CMake and XML linters pass.
The cpplint CTest target exceeds its 120 s timeout on the Windows bind mount; running the same linter
on an identical package copy in native Linux storage passes in about five seconds. Three existing XML
tests initially failed because their exact closing-tag comparisons do not accept Windows CRLF;
restoring `main_tree.xml` to the committed LF bytes made them pass, with no Git/content change.
The full GPL headers in the new tests pass the copyright checker. These environment corrections are
not production patches or softened safety assertions.

Independent Sol reviews of the original tests/CMake changes and the added rotation control and
chassis-translation case have no remaining findings. Hosted CI evidence for the current PR head is
recorded in the canonical PR body. The ROS2 build/test job is expected to be red solely on the two
authorization targets, with the positive controls and existing suite passing.

## Acceptance scenarios and evidence

Each negative assertion expresses the #924 requirement. There is no expected-failure annotation,
disabled test or inverted assertion. A `ControllerException` before unsafe output is an acceptable
rejection; it cannot erase nonzero output from an earlier tick.

| Scenario / test name | Required invariant | Observation boundary |
|---|---|---|
| `MissingGeometryMustNotAuthorizeNominalMotion` | No nominal motion without geometry evidence or an explicit exception | FTC outgoing Twist, both modes |
| `UnknownGeometryMustNotAuthorizeNominalMotion` | An all-unknown grid cannot authorize motion | FTC outgoing Twist after confirmed DDS receipt, both modes |
| `RevokedGeometryMustStopAnAlreadyDispatchedPath` | Replacing a clear grid with all-forbidden geometry revokes the active path | FTC outgoing Twist; no new plan dispatch, both modes |
| `NominalCoverageMustNotCommandAcrossForbiddenBoundary` | Intended translation cannot cross a forbidden reference-point boundary | Straight commanded 0.1 s segments, kinematic TF feedback, both modes |
| `TranslationMustRejectAKeepoutInTheChassisSweep` | The complete translating chassis cannot intersect a hole even with a permitted reference-point path | Exact rectangular sweep of each straight 0.1 s command, both modes |
| `PreRotateMustRejectAForbiddenFootprintSweep` | A forbidden hole must block the complete rotation, even when both endpoint bodies fit | FTC angular output in PRE_ROTATE, both modes |
| `MissingRecoveryGeometryMustNotPublishNonzeroMotion` | Arming/blade/direction evidence alone cannot authorize a recovery envelope | Real EscapeStartBlocked `/cmd_vel_nav` output |
| `ClearGeometryControlProducesForwardMotion` | The fixture can exercise ordinary clear-space motion | Positive FTC control, both modes |
| `ClearGeometryControlProducesPreRotation` | The same rotation must produce an angular command when authorized | Same initial body/pose and pi/2 plan as the forbidden-sweep case, clear grid, both modes |

The translation fixture starts at axle `(0,0)` with the complete body inside the clear region.
The forbidden half-plane starts at `x=0.70`; the nominal path points along +X. Every emitted command
is integrated as a straight 0.1 s intended segment and the new axle TF is fed back to the real controller.
This is an intended-command counterexample, not a measured physical excursion.

The chassis-translation fixture starts with the same rear-axle body at yaw zero and follows +X.
Its forbidden hole is `[0.60,0.70] × [0.20,0.25] m`. The entire axle path at `y=0` is outside the hole,
and the initial body ends at `x=0.53`, before the hole. The hole is inside the body's lateral span
`[-0.275,0.275]`. For each straight 0.1 s command, the exact complete-body sweep is
`[min(axle endpoints)-0.17,max(axle endpoints)+0.53] × [-0.275,0.275]`.
The independent assertion rejects any positive-area intersection with the hole; it permits rejection
before intersection and does not require motion along an invalid body path. This is a hole violation,
so the approved outer-boundary overhang policy does not apply. Curved swept trajectories remain untested.

The rotational fixture uses the shipped chassis polygon relative to the rear axle:
front `0.53 m`, rear `-0.17 m`, half-width `0.275 m`, zero additional padding.
The forbidden hole is `[0.30,0.40] × [0.35,0.45] m`. The reference point and both body orientations
(0 and pi/2) are clear, as is the nominal +Y centerline and its final-heading body.
At pi/4, the hole's interior witness `(0.325,0.375)` transforms to body coordinates
`(0.495,0.035)`, strictly inside the footprint. The forbidden region is a hole, so the test does not
resolve or restrict the separate approved outer-edge overhang policy.

An old timestamp alone is deliberately not classified as invalid geometry. The revocation case uses
an explicit replacement forbidding every point. Geometry identity/revision and delivery liveness are
separate requirements; a timeout would not repair missing ownership or revocation.

## Audited motion inventory

“Static gap” below means source inspection, not a reproduced commanded excursion. Existing guard
tests describe their limited invariants; passing them does not prove the complete #924 contract.

| Motion owner / purpose | Existing verifier and guard coverage | Remaining evidence / gap |
|---|---|---|
| Coverage server: swaths, connectors, joins | Real F2C planning tests cover concavity, holes, joins, edge overhang and planner pivot sweeps (`test_coverage_planning`, `test_pivot_joins`) | Static: final residual verification logs errors then returns success. No residual-failure fixture reproduced here |
| FTC nominal coverage | Local obstacle/body checks and BT boundary monitor; clear-space forward/rotation controls in this PR | New missing/unknown/revocation/axle-boundary/full-chassis-translation acceptance tests |
| FTC lateral skirt | `test_obstacle_deviation`, `test_ftc_lattice_solver` constrain off-plan offsets | Static: global cells ≥99 block; unknown becomes free. No revision binding; nominal-line geometry is not independently checked |
| FTC PIVOT, PRE_ROTATE, POST_ROTATE, oscillation recovery | Planner pivot feasibility, local PIVOT sweep and `test_ftc_pivot`; oscillation helper tests | New PRE_ROTATE hole test only. Other runtime rotation states and outer-overhang policy remain untested here |
| FTC reverse escape / turn fallback | Existing rear-clear/budget tests and real closed-loop `test_ftc_turn_fallback_controller` | No separately verified authorized reverse envelope. Turn fallback zone/collision checks do not establish ownership/revision semantics |
| Nav2 RPP transit: first strip, inter-subpath, detour/resume, HOME staging | Global keepout planning, RPP local collision prediction; #905/#906 add complete new-plan authorization | Actual curved controller trajectories and active revocation remain untested. Plan permission does not establish command permission |
| Nav2/BT BackUp and obstacle backoff | Local collision checks, bounded ordinary 0.30 m transit recovery, blade-off structural guards; `test_transit_tree` | Complete reverse permission untested. Direct BackUp entry remains within #924 |
| Boundary recovery / NavigateInsideBoundary | Missing target/TF reject; keepout toggle acknowledgement required; existing tree/condition tests | Static: recovery disables the complete keepout filter; target offset is not independently checked for holes/overshoot |
| Dock approach / retries / HOME | Dock action lifecycle, firmware contact authority, calibrated dock geometry; #920 adds a separate stall cancellation | Static: broad HOME/docking boundary exemptions and corridor mask overriding overlapping forbidden obstacles. Corridor identity/phase/retry envelope untested |
| Undock: START and charge/rain/battery resume | Preflight Float prerequisite, bounded BackUp, post-undock Fixed wait; existing resume/startup tests | No unified configuration contract between staging offset, undock distance and map corridor. Non-default 2 m versus 1.5 m corridor untested here |
| EscapeStartBlocked | Existing pure arming/blade/direction, compiled ceiling, distance/time tests; real node observes fresh blade each tick and halts with zero | New missing-recovery-geometry test; forbidden reverse segment, cumulative attempts and geometry-change cancellation remain untested |
| Hardware bridge dig recovery | Immediate hard stop; detection/freshness and bounded reverse helper tests (`test_dig_detector`) | Static: direct motor packet reverse bypasses mux/collision monitor, checking emergency/latch/charging but no geometry. Fake-serial authorization test still needed; preserve dig skip zones and inert proposals |
| SeedYawFromMotion | Fixed GPS gate, emergency/timeout/halt zero | Static: autonomous `/cmd_vel_teleop` commands lack geometry permission and bypass collision monitor |
| IMU/dock backoff/redock and magnetometer figure-eight calibration | Emergency/cancel controls, calibration persistence/COG tests | Static: `/cmd_vel_docking` motion lacks geometry checks. Docking lane bypasses collision monitor despite stale comments |
| Direct FollowPath/NavigateToPose, BackUp, Spin, DriveOnHeading, DockRobot/UndockRobot | Local Nav2 action collision mechanisms; normal transit recovery tree excludes Spin | Controller entry is exercised here; action-server/mux/final-serial entry contracts remain untested |
| STOP/cancel/owner loss and boot/guard/rain/charge resume | Existing zero/cancel paths and resume tests; #879 addresses late BackUp acceptance | No common exception identity/revocation; cross-owner replay and current-geometry reacquisition untested |
| Startup, reload/edit/promotion/datum, invalid/off-grid/TF/pose | Existing map/TF gates; pending #905/#906 close new planning during invalidation | This PR covers absent/unknown/replaced grid only. Malformed grids, off-grid extent, wrong frame/datum, stale/missing pose/TF and final-command liveness remain untested |

No separate issue is necessary: all identified gaps are already within #924. Preserve firmware blade/e-stop
authority, approved edge coverage, legitimate dock/undock and bounded recovery while implementing them.

## Integration and first implementation PR

Keep this PR **Draft** while its acceptance tests intentionally fail. Ordinary CTest/colcon registration
makes these visible failures of the normal ROS2 gate. Do not merge it independently, disable the tests,
invert the assertions, add a broad xfail, or mark it ready to conceal the missing requirement.
The integration decision is to carry this test baseline into a focused implementation branch after
agreement on the authorized geometry/exception semantics, then advance only when its assertions pass.
This document proposes that strategy; no maintainer agreement is implied.

Recommended first implementation PR: define the shared geometry snapshot/revision and footprint/reference
policy, then enforce FTC nominal translation and PRE_ROTATE against it, including explicit invalidation
of an active path. Reuse #905/#906's geometry publication rather than create an incompatible parallel
authority. Keep recovery/docking exceptions fail-closed until their separately scoped envelopes are
available while preserving legitimate recovery/dock paths through explicitly defined permissions;
follow with escape/BackUp, dig fake-serial, dock/calibration and final mux/serial tests.

## Physical acceptance

**HARDWARE_REQUIRED.** No robot, sensor simulation or field run was performed. These tests cannot measure
stopping distance, slip, localization error or actual chassis sweeps. Before a supervised run record:
implementation commit, built ROS image, firmware image hash, all relevant submodule gitlinks, robot unit,
receiver/driver revisions and configuration hashes. Run the #924 boundary/hole/rotation/dock/recovery
fixtures at low speed with blades removed, a clear exclusion zone, verified emergency stop and an
operator ready to stop. Pass: observed swept motion stays in the declared geometry/exception envelope,
forbidden requests stop within the documented bound, and permitted coverage/dock/recovery completes.
The exact future hardware/software baseline cannot be named before the safety implementation exists.
