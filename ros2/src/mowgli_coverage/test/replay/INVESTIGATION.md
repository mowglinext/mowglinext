# Coverage replay investigation: #823 / #925

## Evidence and conclusion

The reported charge/resume incident changed a 12,782-pose plan into a
12,826-pose plan. FollowStrip fingerprints changed from `74da26e40758f472` to
`386963fd5f9c35cc`, so the saved positional cursor was rejected. That establishes
a different accepted execution plan, not the stage or cause of its divergence.

The reported field had 70 boundary points and no holes. Its rounded bounding box
(10.96 by 15.36 m), signed area (-54.38 m²) and perimeter (42.33 m) cannot recover
its vertices. The exact transported geometry, live effective parameters,
map/config revision and binary/dependency baseline have not been found in the
issue threads or repository fixtures. They have not been invented here.
The original incident is **not reproduced** and its root cause is **unconfirmed**.

The earlier 15-point replay is a different field. Float32 transport changes its
geometry and can change pose counts while rounded summaries remain identical.
Repeatability of that fixture does not disprove #823. The new harness keeps
transported and sanitized coordinates separately and exposes the earliest
differing stage instead of inferring causality from final pose counts.

References: [#823](https://github.com/mowglinext/mowglinext/issues/823),
[#925](https://github.com/mowglinext/mowglinext/issues/925).

## Pinned dependency and numerical findings

Initial investigation baseline: `9f39497f26f4759a0b26e601904710fb934beb65` on `dev`.
Fields2Cover pin: `884d895b59192882476e986ba44ea9143a06a6a9` (3.0.0).

- Brute-force AUTO costs are stored by ascending candidate index; `min_element`
  chooses the first minimum. The NSwath objective counts clipped pieces, with
  no random tie break or floating-point reduction. Identical candidate results
  therefore give identical tie selection.
- The pinned CMake `ALLOW_PARALLELIZATION` option links TBB but does not define
  the same-named C++ macro. An isolated dependency build using the project's pin
  and options had no such definition in its actual compiler flags. It evaluated
  candidates serially. This does not establish the incident binary's flags.
- A genuinely parallel build would read shared OGR geometry during clipping;
  its safety would need separate examination. No race has been demonstrated.
- Swath IDs follow clipping enumeration. The generated IDs are unique and their
  comparator is ordered; there is no demonstrated unstable-comparator defect.
  Polygon winding, cyclic vertex starts, hole order and dependency versions can
  affect geometry enumeration and must be retained as input evidence.
- F2C clone helpers round-trip through WKT. The trace reads coordinates directly
  so it does not add serialization rounding to observations.
- AUTO's selected angle is later inferred from the longest clipped swath.
  Even-lane reconstruction has floor/extent and 2 cm remainder thresholds.
  Small input or dependency differences near these thresholds could propagate
  into counts and ordering. This is a test target, not a confirmed explanation.
- Connector and subpath ordering use deterministic seeds and comparisons, but
  remain sensitive to primitive order and small geometric cost differences.

No production search, ordering, caching, strategy, connector or fingerprint
validation change is justified by this evidence. The implementation adds an
opt-in observer that delegates AUTO's argmin search once and records its return
value. Normal callers leave the observer absent. `coverage_server.cpp` is
unchanged to preserve #924 ownership.

## Historical validation (2026-10-10)

The completed x86_64 CI comparison tested PR head
`54dbcbc1fddc86d7166c92f137e3018a10c9cc7a` in synthetic merge
`6f9255585d5d531a101f29a2d3453c5290a403e3`, whose parents are baseline
`fd244fecace467d0046b28012bc29d5a4099220f` and that PR head. The workflow extracts
matching baseline source, header, tests and fixtures from its first parent.
These revisions differ from the initial investigation baseline above.

All three dedicated jobs passed: Release, Debug and RelWithDebInfo. Each tested
26 exact fixtures with 15 untraced baseline runs, 15 untraced candidate runs and
100 traced candidate runs per fixture, including repetitions within a process
and fresh processes. That is 3,380 matrix records and 3,510 field-planning calls
per configuration; the multi-polygon case has two field calls per record.
Across the three configurations: **10,140 matrix records / 10,530 field calls**.
Standalone smoke checks add 48 records / 48 field calls overall.

Within each configuration all 26 fixtures were stable, all baseline/candidate
exact outputs and coverage/connector metrics matched, and all traced/untraced
candidate execution geometry, quaternions and fingerprints matched. Changed
width, angle and geometry controls had distinct diagnostic input identities and
execution fingerprints. There was no intermittent divergence in these runs.
Pose counts, total path length, swath and blade-off transit counts, endpoint-link
lengths and connector outcomes were unchanged by this implementation. Endpoint
links are straight-line diagnostics, not measured Nav2 travel or robot safety.

Each standalone baseline passed 9/9 CTest targets and candidate passed 10/10.
The ROS package passed all 15 targets: 128 existing C++ tests, 3 trace-passivity
tests, 7 runner tests, 9 comparison tests, 5 provenance tests, the 104-record
replay matrix and lint checks. The complete ROS CI rerun passed; its first
attempt timed out in an unrelated Nav2 speed-limit test's cleanup. No Nav2 file
was changed. Formatting, cppcheck, configuration drift and both amd64/arm64
runtime image builds passed. ARM64 image construction is not ARM64 replay evidence.

Evidence: [planner jobs and retained artifacts](https://github.com/mowglinext/mowglinext/actions/runs/38092863429),
[ROS CI, successful attempt 2](https://github.com/mowglinext/mowglinext/actions/runs/38092861214).
The planner artifacts retain exact fixtures, compressed snapshots, per-fixture
metrics, hashes, build logs and metadata for 14 days. Reproduction commands and
the retained-evidence comparator are in [README.md](README.md).

### Performance and build provenance

The table reports median relative changes across the 26 per-fixture medians,
candidate versus baseline, with both untraced. Positive means more time or RSS.
RSS includes the replay process and JSON serialization. These runner measurements
are not a claimed speed improvement; no timing percentage is used as a CI gate.

| Build | Planner time | Connector time | Process time | Median process RSS |
| --- | ---: | ---: | ---: | ---: |
| Release | -0.60% | -1.37% | -0.81% | +0.008% |
| RelWithDebInfo | -0.08% | -6.88% | 0.00% | +0.008% |
| Debug | +1.58% | -1.30% | +3.13% | +0.007% |

Maximum measured peak RSS was 119,084 / 119,160 KiB (baseline / candidate) in
Release and 118,968 / 118,956 KiB in RelWithDebInfo. Full per-fixture values,
including timing outliers, are in each artifact's `comparison.json`.

All jobs used Ubuntu 26.04, ROS Lyrical, GCC 15.2.0, GDAL 3.12.2 and GEOS 3.14.1.
The actual CMake executable reported 4.4.3; the package inventory separately
reported 4.2.3. The pinned F2C library was identical across configurations:
SHA-256 `1f006c551c5b747fc594d4c69a00f7cecf5299b12aa8cb61aa7cea535c69e6e4`.
Its cache hit does not retain the original dependency compiler flags; those are
explicitly unavailable. Planner compiler commands and source/header hashes are
retained. Do not extrapolate the separately inspected upstream build's macro
flags to this cached binary or the incident binary.

### Limits of repeatability

Release and RelWithDebInfo execution hashes matched for all 26 fixtures.
Debug produced different plans for several identical fixture files, in both the
baseline and candidate. For example, `square_fixed` had 7,059 poses in Release
and 7,017 in Debug (278.9444 versus 277.6467 m); the recorded float32 field had
4,315 versus 4,321 poses. Independent artifact review localized the earliest
recorded `square_fixed` divergence to `0.rings`, first ring, point index 6, Y:
Debug bits `40032f4af56bf5c8`, Release bits `40032f4af56bf5c9` (one ULP).
Transported/normalized geometry, parameters and the preceding `safe_cell` stage
matched. This fixed-angle example differs before recorded swath/connector output;
it does not support blaming AUTO selection.

Retained planner flags were Debug `-g`, Release `-O3 -DNDEBUG` and RelWithDebInfo
`-O2 -g -DNDEBUG`. The observations establish build-dependent output in the
compiled planner stack, not the responsible floating-point operation, optimizer
behavior or F2C defect. Cached dependency flags and possible inline/header-library
interactions remain unresolved.

Repeatability within one build therefore does not establish plan identity across
build configurations. This is build sensitivity, not a reproduction or causal
explanation of the original charging incident. A changed binary/configuration
must not authorize reuse of an incompatible positional cursor.

Independent reviews found and closed replay issues in rings-dependent start
pinning, process-tree timeout cleanup, exact trace assertions, incremental and
packed-ref build provenance, required retained evidence and ordered trace-stage
comparison. Nine comparison regressions reject missing/null values, incorrect
fixture/output/input hashes, missing baseline snapshots, invalid measurements,
stage reordering/duplicates and submillimetre trace drift. None of these harness
repairs is presented as a production fix for #823.

## Review and rebase (2026-10-12)

The eight PR commits were rebased without conflicts onto fetched `dev`
`fd244fecace467d0046b28012bc29d5a4099220f`. `git range-diff` reports all eight
patches unchanged. The final diff retains the same 19-file scope; upstream's
sharp-corner pivot changes and current planner regression tests are preserved.
The new review did not establish a production correctness defect requiring an
algorithm or resume-state change.

An independent source review of rebased implementation
`b0fe1cb9e7626996f93a61d1d58ec2408a04e49d` found no confirmed blocker or
should-fix issue. It checked observer passivity, server/replay input parity,
connector/start-hint arguments, exact geometry/quaternion comparisons, retained
evidence, provenance, build configurations and documentation limits. This is
source-review evidence, separate from execution and CI evidence. The PR body
records the final validation head and run links; the measurements above retain
their original baseline and must not be relabelled as new measurements.

### Historical `test_follow_strip_dig` timeout

[Push run 38096470556](https://github.com/mowglinext/mowglinext/actions/runs/38096470556)
on documentation head `835e67efb80246cb2af860ca286e52b55388a8e1` timed out after
180.10 s. Its final test was
`FollowStripDigTest.EndSessionForgetsTheDigPointsButKeepsTheEventCounter`; the
last application log was `EndSession: clearing per-session flags`. The log does
not distinguish a hang inside that call from executor/test teardown. It does
not establish CI resource starvation or a precise deadlock cause.

The test and `status_nodes.cpp` are unchanged between that head and the fetched
dev baseline. This test ticks `EndSession` against fake action/service servers;
it never invokes the coverage planner. The same CTest target passed in 20.47 s on the
previous implementation's [push rerun 38092861214](https://github.com/mowglinext/mowglinext/actions/runs/38092861214),
20.46 s on the same documentation head's [PR run 38096473964](https://github.com/mowglinext/mowglinext/actions/runs/38096473964),
and 20.63 s on [dev run 38070842188](https://github.com/mowglinext/mowglinext/actions/runs/38070842188)
(`d4924a4b1e6dd7d56aca49b92d4a324a5196d3d6`). The individual final case took
9 ms and 21 ms in the successful push/PR runs. This supports classification
as an unrelated intermittent ROS/test-lifecycle hang, rather than a demonstrated
planner-replay regression. No test, timeout, assertion or unrelated production
code was changed to mask it. New-head required checks must still pass.


### Follow-up: controlled contraction experiment (2026-10-12)

The exact tested merge source/header were extracted from
`6f9255585d5d531a101f29a2d3453c5290a403e3`; their SHA-256 values matched the
archived CI records:
source `d8cf4623761e49d7b99f7c007da4174867d619c72346193957880749d6840b32`,
header `87572162592ef4b201f60aa80b352cf446b14f829052167fdd4dec7f80cbafdc`.
The real planner was built as UID 1000 locally, using the project's pinned F2C
recipe in Ubuntu 26.04 / GCC 15.2.0. Tests had no network or robot mounts.

The local compiler's default target was x86-64 with FMA disabled. Debug and
Release produced the same `square_fixed` plan (7,017 poses, fingerprint
`0f4d6f7779e79f5b`). Adding `-mfma` to Release produced 7,059 poses and
`bd97b10d720a3655`, matching that fixture's archived Release fingerprint.
A minimal replay of the exact straight-segment endpoints reproduced the
`40032f4af56bf5c8` versus `40032f4af56bf5c9` interpolation difference;
disassembly of the FMA variant contained `vfmadd132pd`.

Three controlled Release variants then planned each of the 26 retained fixtures
once, with byte-matching inputs and the same local dependency binary: **78 replay
records / 81 field-planning calls** (one two-polygon fixture per variant).

| Controlled local comparison | Exact stable execution matches |
| --- | ---: |
| Default target vs `-mfma -ffp-contract=off` | 26/26 |
| Default target vs `-mfma` | 0/26 |
| `-mfma -ffp-contract=off` vs `-mfma` | 0/26 |

This establishes a reproducible contraction-sensitive planner output. It does
not establish an intermittent change within one unchanged binary, the actual
ISA/contraction settings of the old CI binaries, or the original #823 cause.
[GCC 15.2 documents](https://gcc.gnu.org/onlinedocs/gcc-15.2.0/gcc/Optimize-Options.html)
that contraction can form native fused multiply-add operations and can be disabled
with `-ffp-contract=off`. No production compiler or numerical policy is changed.

The fresh local F2C library SHA-256 was
`64dd2df93d40926f3c5f68299a3f3fd7815d2c2e1273a69c764d93c846f7795a`;
the archived CI library was `1f006c55...535c69e6e4`. Despite matching source/header
and all 26 fixture hashes, each local variant matched its corresponding archived
CI execution on only 2/26 fixtures. This dependency/baseline difference prevents
attributing the remaining cross-archive differences to contraction. It is not
evidence of a random Fields2Cover search.

To make the next CI comparison auditable, the standalone GNU build records cc1
switches. Artifacts retain replay/planner/dependency binaries, hashes, disassembly,
CPU information and linked libraries. Replay build metadata includes requested
global/build-type flag hashes and compiler FMA/AVX2/fast-math/evaluation macros.
Those are diagnostics; they do not replace full dependency provenance or authorize
a saved resume cursor.

### Follow-up: timeout-test regression

A later duplicate ROS check on `835e67ef`
([run 38096470556](https://github.com/mowglinext/mowglinext/actions/runs/38096470556))
failed the Python wrapper/child test: its immediate procfs sample read `R`
instead of `Z`. It did not load the planner or F2C. That job also had an unrelated
`mowgli_behavior/test_follow_strip_dig` timeout; no behavior file is changed here.
The duplicate ROS job and all three replay/image jobs passed.

The test now waits up to one second for the original child's exit state, keyed
by PID plus procfs start time. A still-live matching identity fails at the
deadline; disappearance or PID reuse cannot be confused with a leaked child.
Four deterministic cases cover running-to-zombie, reaping, PID reuse and a live
child at the deadline. Production `run_command` kill/timeout behavior is unchanged.

The repaired 11-test runner suite passed 20/20 local Linux repetitions (220 test
cases). All 10 standalone CTest targets passed in 49.39 seconds, including the
existing planner suites, trace tests, runner/comparison/provenance tests and smoke
replay. The final lint and additional binary/metadata assertions did not run:
the test image lacked the flake8 CLI, then host command execution stalled.
Updated CI remains required; no green result for these new changes is claimed.
Independent source review found no blocker or should-fix issue in the continuation.

## Integration and resume risks

#718 can capture exact schema-1 goals and live effective parameters, with the
map/config and image/repository/dependency revisions, at server ingestion.
Any server integration must be coordinated with #924; this branch does not
modify its motion authorization or coverage-server work.

The production position-only, millimetre-quantized FollowStrip fingerprint is
mirrored for diagnosis and never weakened. The additional exact output hash and
input identity are offline diagnostics, not an accepted resume authorization.
A genuinely different execution plan must continue to reject stale cursors.
Charge/restart persistence, accepted-plan retention, reset/remapping and semantic
completion in #925/#931/#932 remain outside this change. See the README's
`HARDWARE_REQUIRED` procedure for physical resume acceptance.

## Changed files

The implementation branch changes these 19 files; no coverage-server, Nav2,
behavior, hardware, map, monitoring or GUI file is changed:

```text
.github/workflows/coverage-replay.yml
ros2/src/mowgli_coverage/CMakeLists.txt
ros2/src/mowgli_coverage/include/mowgli_coverage/coverage_planning.hpp
ros2/src/mowgli_coverage/package.xml
ros2/src/mowgli_coverage/src/coverage_planning.cpp
ros2/src/mowgli_coverage/test/replay/.gitignore
ros2/src/mowgli_coverage/test/replay/CMakeLists.txt
ros2/src/mowgli_coverage/test/replay/Dockerfile
ros2/src/mowgli_coverage/test/replay/INVESTIGATION.md
ros2/src/mowgli_coverage/test/replay/README.md
ros2/src/mowgli_coverage/test/replay/compare_replay.py
ros2/src/mowgli_coverage/test/replay/configure_replay.cmake
ros2/src/mowgli_coverage/test/replay/replay.cpp
ros2/src/mowgli_coverage/test/replay/run_replay.py
ros2/src/mowgli_coverage/test/replay/test_comparison.py
ros2/src/mowgli_coverage/test/replay/test_provenance.py
ros2/src/mowgli_coverage/test/replay/test_runner.py
ros2/src/mowgli_coverage/test/replay/test_trace.cpp
ros2/src/mowgli_coverage/test/replay/write_provenance.cmake
```
