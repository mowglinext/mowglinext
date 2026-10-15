# GUI Lot 18 — FusionGraph and Behavior Tree diagnostic types

## Baseline and scope

Branch `fix/gui-warnings-errors`, HEAD `16add50b` (Lot 17 committed).
Preflight: empty index, only the preexisting modified MAVROS submodule in the
worktree. Baseline: **127 ESLint warnings, zero errors**, 2,003 Vitest tests.

Only `hooks/useFusionGraphDiagnostics.ts`, `hooks/useBTLog.ts`, their directly
associated tests and this report change. No backend, ROS, dependency, generated
source, GPS display, fix classification, severity threshold, settings-loading,
LiDAR or MAVROS change. Nothing staged, committed or pushed.

## Contracts verified

- `gui/pkg/providers/ros.go` maps `fusionDiag` to
  `/fusion_graph/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) and `btLog`
  to `/behavior_tree_log` (`nav2_msgs/msg/BehaviorTreeLog`).
- `gui/pkg/api/mowglinext.go` decodes the ROS JSON and sends MessagePack envelopes
  through the multiplex route. `hooks/multiplexedSocket.ts` unpacks the envelope
  and supplies its decoded `data` as unknown. `useWS` subscribe URIs use that
  multiplex path: these callbacks receive objects, not JSON strings or browser
  MessageEvents. The old `useWS<string>` followed by `as any` was incorrect.
- `ros2/src/fusion_graph/src/fusion_graph_node_setup_comms.cpp` publishes one
  DiagnosticStatus every second: `name="fusion_graph"`, ROS header stamp,
  diagnostic level, message (`running` / `waiting init`), string KeyValue pairs.
  `hardware_id` is left at the ROS string default. Installed Lyrical message
  definitions specify diagnostic levels 0–3, signed int32 seconds and normalized
  nanoseconds in [0, 1,000,000,000).
- Installed Lyrical `nav2_behavior_tree/ros_topic_logger.hpp` publishes Nav2 BT
  changes with node name, uint16 UID, previous/current status strings and event
  timestamp, then clears its batch on flush. BehaviorTree.CPP also supports
  SKIPPED; its status string must not be incorrectly restricted to the four
  values listed in the ROS message comment. No BT enum or status conversion is
  introduced here.
- Consumers inspected: DiagnosticsPage, Onboarding readinessChecks/ReadinessStep,
  nav2Recovery and lidarAnchor. UI calculations, RUNNING comparisons, stale
  thresholds and diagnostic number parsing remain unchanged.

The four warnings came from two identical causes: a string-typed callback was
assigned to a local interface through `any`. Type assertions provided no runtime
validation and could accept corrupt fields or throw after partial BT mutation.

## Changes and runtime behavior

Both hooks now use `useWS<unknown>` and private focused decoders built on the
existing `isJsonObject` guard. No cast, suppression or lint configuration change.

FusionGraph validates the first status only, preserving the existing selection
rule. Name/message must be strings, level an integer 0–3, and every supplied
KeyValue must have a string key and value. Numeric-looking diagnostic strings,
including `"0"`, remain strings; no measurement is fabricated or converted.
Duplicate keys still use the final value. `Object.fromEntries` also preserves
literal own keys such as `__proto__` without invoking a prototype setter.

Absent `values` remains an empty map for legacy compatibility. Absent header,
stamp or hardware ID remains supported. If these fields are explicitly supplied,
their types and ROS stamp bounds must be valid; explicit null is malformed, not
an absent field. An empty status array or a malformed first status yields no
snapshot. Later statuses are not promoted as replacements. Invalid data leaves
the complete last valid snapshot and its receipt/provenance dates unchanged.

Valid stamp formatting, source identity and duplicate-publication handling are
unchanged. The all-zero ROS stamp still supplies no source identity. Browser
wall receipt time, monotonic receipt time and first receipt of a distinct
producer/publication identity retain their separate meanings. A publication
stamp is not a physical acquisition guarantee; no source epoch or freshness
capability is invented.

BT validates all events before mutating the accumulator. Each projected event
requires only string `node_name` and `current_status`; UID, previous status and
ROS timestamp are not consumed, fabricated or claimed to have been validated.
Existing public wire interfaces remain unchanged; the projection reuses a Pick
of BTStatusChange. Empty strings and future status strings remain literal and
cannot be promoted to RUNNING/SUCCESS. A malformed event rejects the entire
frame, preventing earlier events in that same frame from partially changing the
accumulator. Missing or empty event_log still contributes no new state.

The existing final-event-wins ordering, Map accumulation, 500 ms throttle and
30-second stale threshold remain unchanged. BT updatedAt is explicitly documented
as browser receipt wall time, not the ROS event time or physical acquisition.
Disconnect callbacks still retain the last state; no new connection state,
reconnect attempt, error notification or subscription policy is added. Malformed
frames do not restart subscriptions or renew valid diagnostic timestamps.

## Tests and validation

Focused command:

```sh
yarn vitest run src/hooks/useFusionGraphDiagnostics.test.ts \
  src/hooks/useBTLog.test.ts src/utils/nav2Recovery.test.ts \
  src/components/onboarding/readinessChecks.test.ts --pool=threads --maxWorkers=2
```

**81 tests passed**, four files. Regression coverage includes valid producer
frames, zero UID/time/level and textual measurements, missing optional metadata,
empty text/arrays, malformed fields and measurements, invalid levels/stamps,
literal dictionary keys, duplicate-publication identity, first-status selection,
whole-frame BT rejection, retained last state, disconnects, stable subscriptions,
throttling and unmount cleanup. All data and transport callbacks are mocked;
no real subscription or robot action was performed.

- Full `yarn test --pool=threads --maxWorkers=2`: **2,044 tests passed**,
  187 files, plus the appearance registry test (219.90 seconds).
  This adds 41 regressions to the 2,003-test baseline.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-lot18-dist`: passed.
- Global ESLint JSON: **127 → 123 warnings**, zero errors. Both hooks and their
  tests have zero warnings/errors; no new rule or suppression.
- `yarn lint`: passed, 123 warnings and zero errors.
- `git diff --check`: passed, including separate whitespace checks for new
  files; index remains empty.

| File | Warnings before | Warnings after |
| --- | ---: | ---: |
| hooks/useFusionGraphDiagnostics.ts | 2 | 0 |
| hooks/useBTLog.ts | 2 | 0 |
| Associated tests | 0 | 0 |

| ESLint rule | Before | After |
| --- | ---: | ---: |
| @typescript-eslint/no-explicit-any | 53 | 51 |
| @typescript-eslint/no-unsafe-assignment | 26 | 24 |
| @typescript-eslint/no-unsafe-member-access | 13 | 13 |
| @typescript-eslint/no-unsafe-argument | 8 | 8 |
| @typescript-eslint/no-unsafe-call | 3 | 3 |
| @typescript-eslint/no-unsafe-return | 3 | 3 |
| react-refresh/only-export-components | 11 | 11 |
| react-hooks/set-state-in-effect | 8 | 8 |
| react-hooks/purity | 2 | 2 |
| **Total** | **127** | **123** |

## Limits and deferred topics

Validation deliberately projects only consumed fields, rather than pretending to
validate every ROS field. Future diagnostic level values require an explicit
contract review; future BT status strings retain their string representation.
Malformed frames are silently ignored consistently with the Fusion hook's prior
error policy. There is no new diagnostic health indicator for rejected frames.

Legacy producers without a publication stamp cannot establish source identity;
receipt remains delivery information. BT replayed events still update receipt
age under the existing contract, which has no event deduplication identity here.
Those are provenance limits, not solved with a new timeout in this lot.

No producer defect requiring a backend/ROS fix was established. GPS display,
fix classifications, severity policy and settings-loading anomalies are deferred
as requested. Build large-chunk notices and existing test-environment notices
remain outside this type cleanup. No browser test is needed for these hook-only
changes; transport lifecycle and existing consumers are covered by focused and
full unit tests. Logs/build artifacts remain in `/tmp`, outside version control.
