# Settings and robot geometry integrity — Batch 10

Baseline: branch `fix/gui-warnings-errors`, HEAD
`b6588bdd798dd6f9be1286e4068829d5fbe7ffe5`. Changes are uncommitted.
The existing local MAVROS submodule changes are outside this batch.

## Settings boundary and ownership

`settingsContract.ts` models recursive JSON settings. Decoded API values enter as
`unknown`; only plain objects, JSON collections and finite scalar numbers are
accepted. Undefined values, non-JSON objects, cycles, sparse arrays, accessors,
symbol keys and invisible properties are rejected rather than silently changing
settings when serialized.

`flattenConfig()` recursively flattens nonempty plain objects into dotted keys.
Null, arrays (including nested objects inside arrays), empty objects, empty
strings, zero and false remain leaves. Dotted-key collisions and empty keys are
errors. Prototype-like own JSON keys do not mutate the result's prototype.

`useSettings` loads `/settings` (the shell settings envelope) and `/settings/yaml`
independently. YAML takes precedence over shell values for file settings. Declared
DB keys belong to `/config/keys/get` regardless of response arrival order. Loading
includes the DB read, both file reads and an active save. Only YAML HTTP 404/405
is treated as an older backend without that endpoint; malformed payloads and
other failures are reported without discarding the other source's valid data.

The setter persists declared DB keys only. String-backed fields must contain
strings; booleans are serialized explicitly. It never writes YAML or claims a
YAML-only update was saved. `/config/keys/set` must return matching values for
all submitted keys before the local DB state or success notification changes.
A late initial DB read cannot overwrite a successful local DB write. Pending
saves cannot duplicate requests or emit notifications after unmounting. The
Batch 7B YAML persistence and partial-failure contract is preserved; the review
follow-up below adds scoped capture integration to the settings manager.

Map and schedule callers now narrow settings before numeric parsing or indexing.
Map coordinate parsing retains the existing decimal-string behavior and the
existing lidar display fallback; it excludes implicit boolean/array/object
coercion. These edits do not change the map projections or backend routing.

## Geometry and sensor inputs

`RobotComponentEditor` accepts untrusted setting values and emits numeric edits
or `null` for an unavailable/cleared numeric input. Geometry numbers accept
finite numbers and complete decimal strings, including zero. Malformed explicit
sensor values display an unavailable input and a warning; the preview uses the
existing robot geometry fallback. Clearing a heading no longer becomes a zero
heading. GPS, LiDAR and IMU identifiers and YAML keys are unchanged.

The REP-103 axes (X forward, Y left), radians stored in YAML, compass degrees,
SVG projection/inverse projection, yaw formulas, rounding of manual edits,
URDF dimensions, sensor defaults and drag bounds are preserved. Dragging ignores
an unavailable SVG rectangle or invalid pointer coordinates, and an empty touch
list cannot crash the handler. Mouse/touch subscriptions retain their cleanup.

## Canonical dock readback

The sole Go production change is the read-only `calibration_status.go` response.
`pose_values_valid` is independent of the historical `present` flag: it requires
readable, parseable YAML and exactly one explicit occurrence of each numeric,
finite `dock_pose_x`, `dock_pose_y` and `dock_pose_yaw` key. The explicit triplet
`(0, 0, 0)` is valid even though `present` retains its historical false value.

The three JSON coordinates are always included. Explicit zeros encode as zero;
missing, malformed, ambiguous or nonfinite values encode as `null`. Invalid
numbers cannot break dock JSON serialization. Existing valid coordinates,
`present`, errors and the other calibration sections retain their contracts.
The optional derived degree field is omitted if its conversion is nonfinite.

Capture still requires charging contact and an operator confirmation. Contact
is checked again immediately before the write. The GUI makes one
`POST /mowglinext/map/docking` with `use_gps_position=true` and `yaw_source=0`
(PRESERVE), retaining the GPS position source and independent dock yaw.
It checks the HTTP/API response explicitly, including `response.error`, then
validates the Go acknowledgement (`{}` or its explicit `ok` string variant).
It requests `/calibration/status` once and adopts only a complete, finite
triplet with `pose_values_valid=true`, retaining canonical precision and zeros.

Write rejection, unexpected acknowledgement, missing/malformed readback and
readback request failure leave local coordinates unchanged. Notifications
distinguish an unconfirmed write from an accepted write with unconfirmed stored
coordinates. There is no fused-pose fallback or automatic second write. Duplicate
confirmation callbacks cannot start concurrent captures; unmounted components
cannot apply pending responses. The success message means stored YAML values
were loaded, rather than asserting a new physical GPS acquisition.

## Validation

- Initial focused Vitest: **86 tests passed**, four files (78 added regressions
  plus eight existing configuration tests). Review follow-up: **57 tests passed**
  in the combined editor/manager run, then the added manual-edit regression
  passed separately (**1 test**). All 58 current editor/manager tests are also
  covered by the full suite.
- Full frontend test command (`yarn test`): **1,746 Vitest tests passed**, 173
  files; the separate appearance registry test also passed. Baseline: 1,655 (initial Batch 10 result: 1,733).
- Focused Go dock integrity tests passed again during the follow-up; full `go test ./...` in `gui` passed.
  Tests cover explicit zeros, nested values, missing keys, null/string/boolean/
  array values, NaN/infinity, ambiguous paths, malformed YAML and read failures.
- `yarn tsc --noEmit` passed; `yarn build --outDir /tmp/gui-batch10-dist` passed.
- ESLint full JSON inventories: **263 → 222 warnings**, zero errors before and
  after. No suppressions or generated source edits.
- Go files are gofmt-clean; `git diff --check` passed.
- Tests use mocked API calls and temporary YAML files. No robot action was run.

| ESLint rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 107 | 94 |
| `@typescript-eslint/no-unsafe-assignment` | 55 | 38 |
| `@typescript-eslint/no-unsafe-member-access` | 38 | 34 |
| `@typescript-eslint/no-unsafe-argument` | 34 | 27 |
| `@typescript-eslint/no-unsafe-call` | 4 | 4 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 |
| `react-refresh/only-export-components` | 11 | 11 |
| `react-hooks/set-state-in-effect` | 9 | 9 |
| `react-hooks/purity` | 2 | 2 |
| **Total** | **263** | **222** |

The maintained files with warning reductions are `RobotComponentEditor.tsx`
(16 → 0), `useSettings.ts` (16 → 0), `MapPage.tsx` (5 → 2), and
`SchedulePage.tsx` (5 → 0), and `SensorsSection.tsx` (1 → 0). Related helpers, tests and `useConfig` have zero
warnings. Remaining warnings elsewhere are outside this focused batch.
The build's existing large-chunk warning and Ant Design deprecation notices
remain; neither is an ESLint failure.

## Review sections and compatibility limits

Review settings first: `settingsContract.ts` and its test, `useSettings.ts`,
`useConfig.tsx` and `useConfig.test.tsx`, followed by the minimal MapPage,
SchedulePage and useMapStreams caller changes. Review geometry and capture:
`RobotComponentEditor.tsx` and its test, `robotEditorContract.ts` and its test,
and the English/French action text. Review the read-only backend extension:
`calibration_status.go`, `calibration_status_integrity_test.go`, and
`useCalibrationStatus.ts`. This document records the final scope.

Older servers without the new validity field cannot prove explicit YAML value
presence, so dock capture readback remains unconfirmed and preserves the local
form. Legacy settings producers without `/settings/yaml` remain supported.
Consumers of malformed dock metadata must accept nullable coordinates; valid
existing values retain their numeric representation. The old `present` heuristic
continues to mean initialization status, not numerical validity.

The docking service provides `stored_pose` and can return success with a message
when applying in memory succeeds but persistence fails. The existing Go write
adapter does not expose those details. This batch does not change that adapter
or ROS. A subsequent valid YAML read may therefore contain older persisted
values: there is no write correlation ID, acquisition identity or transaction
promise. The GUI shows those canonical values and deliberately limits its
success wording to that fact. DB key writes also remain nontransactional; a
failed request may follow partial backend persistence, and the GUI does not
claim success or automatically retry it.

This is software contract validation, not hardware evidence of GPS accuracy,
charging contact, capture freshness or docking performance.

## Review follow-up: capture versus regular Settings Save

The traced path is `SettingsPage → SensorsSection → RobotComponentEditor →
useSettingsManager`. The initial Batch 10 editor sent all three canonical
readback values through `onChange`, which records draft edits. That made a
successful capture dirty: regular Save could then persist those values again
and send a manual `/mowglinext/map/docking` command with
`use_gps_position=false`, `yaw_source=REQUEST`. Such a command could overwrite
a yaw recalibrated independently after capture.

Settings now supplies a dedicated `onDockPosePersisted` callback for the complete
numeric triplet. The manager's `acceptPersistedDockPose` delegates to the existing
`acceptPersistedValues` mechanism: both the displayed values and the saved
baseline adopt the canonical readback atomically, without dirty dock fields.
Unrelated draft edits remain dirty. A subsequent unrelated Save sends only those
edits; it sends no dock YAML fields and no manual dock command. Later explicit
manual dock edits still follow the existing Save behavior.

The optional callbacks also delimit capture attempts. Capture cannot start during
a settings write. Save and partial Save cannot run while a capture is awaiting
its acknowledgement/readback, preventing an older save snapshot from replacing
the new baseline. After an unconfirmed attempt, the current form is preserved,
but existing dirty dock fields are blocked from YAML and manual runtime writes
with an explicit warning. Partial dock writes are blocked too. Unrelated saves
remain available once capture ends. Ordinary revert or a tuning report cannot
clear this uncertainty: reload settings or complete a confirmed capture to
establish the baseline. No automatic readback retry or second docking write was
added. These guards belong to the current Settings manager instance; they do not
claim a distributed lock or write/acquisition identity.

Onboarding supplies none of the new optional callbacks and retains its existing
post-readback `onChange` behavior. The Go response contract and all ROS formulas,
GPS capture options and preserved-yaw behavior are unchanged in this follow-up.

Clearing a sensor number intentionally means deleting its YAML override (`null`
payload), not setting a measured zero. Regression tests drive the actual LiDAR X,
IMU yaw and GPS height inputs through clearing, Save, and local-key pruning.
The preview's existing URDF/default fallback applies when an override is absent;
applying geometry changes still follows the existing restart behavior.

The follow-up modifies the editor and its tests, SensorsSection, SettingsPage,
the settings manager and its merge tests, the helper's narrow persisted-pose type,
English/French warnings, and this report. No Go, generated, ROS or MAVROS files
were changed by the follow-up itself. Added regressions cover A→B clean capture,
Save of unrelated data, later independent yaw calibration, explicit manual dock
edits, failed/malformed readbacks, failed acknowledgements, pending capture/save
ordering, partial-save guards, and sensor-key deletion.
