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

Source baseline: `9f39497f26f4759a0b26e601904710fb934beb65` on `dev`.
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

## Validation status

Local Windows validation: seven Python unit tests ran; six passed and the POSIX
process-group test was skipped. These test comparison and timeout behavior,
not Fields2Cover. Clang-format 18 is required for all changed C++ lines.

The candidate C++ build, five existing planner suites, trace-passivity tests,
104-call registered replay matrix and Release/Debug/RelWithDebInfo comparisons
are pending Linux execution. No passing result is claimed for these checks.
The isolated pinned dependency alone was built successfully; that does not
validate the candidate planner or replay executable.

Baseline versus candidate pose count, path length, swath/transit counts,
connector outcomes, runtime and peak RSS are **unmeasured**. The harness emits
these metrics and supports the unchanged baseline source/header via
`REPLAY_BASELINE`; use the README commands and compare both with `--no-trace`.

Independent source reviews examined Fields2Cover determinism and the replay
implementation. Confirmed replay fixes include the server's rings-dependent
start-hint pin condition, process-tree timeout cleanup and bitwise trace parity
tests for holes, multiple cells, perpendicular swaths and pivot joins.

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
