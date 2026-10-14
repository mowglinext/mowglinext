# GUI Batch 11 — map display preference integrity

Starting HEAD: `60d7df0cb9ee7caf3ecd2b21d6e433f827d3c9f4`, branch
`fix/gui-warnings-errors`. Batch 10 is committed locally at this HEAD. Initial
worktree changes were confined to the existing MAVROS submodule; it was neither
modified nor staged by this batch. No commit, push, merge, reset or generated-file
workflow was performed.

## Verified baseline and inventory

Baseline validation: 222 ESLint warnings, zero errors; 1,746 Vitest tests in
173 files, plus the separate appearance registry test; TypeScript, production
build and full GUI `go test ./...` pass. JSON inventories and logs are temporary
artifacts under `/tmp/gui-batch11-*`, outside version control.

| Rule | Before | After | Removed |
| --- | ---: | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 94 | 85 | 9 |
| `@typescript-eslint/no-unsafe-argument` | 27 | 21 | 6 |
| `@typescript-eslint/no-unsafe-assignment` | 38 | 35 | 3 |
| `@typescript-eslint/no-unsafe-call` | 4 | 4 | 0 |
| `@typescript-eslint/no-unsafe-member-access` | 34 | 31 | 3 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 | 0 |
| `react-hooks/purity` | 2 | 2 | 0 |
| `react-hooks/set-state-in-effect` | 9 | 8 | 1 |
| `react-refresh/only-export-components` | 11 | 11 | 0 |
| **Total** | **222** | **200** | **22** |

Top 15 files in the initial inventory (ties follow ESLint inventory order):

| Source under `gui/web/src` | Warnings |
| --- | ---: |
| `components/DrawControl.tsx` | 17 |
| `pages/map/hooks/useMapEditing.ts` | 12 |
| `pages/map/hooks/useMapOffset.test.ts` | 12 |
| `components/settings/AdvancedSection.tsx` | 11 |
| `components/PowerMenu.tsx` | 9 |
| `pages/map/hooks/useMapFiles.ts` | 9 |
| `pages/map/hooks/useMapStreams.ts` | 9 |
| `pages/map/components/TrackedObstaclesPanel.tsx` | 8 |
| `pages/map/hooks/useMapFiles.test.ts` | 8 |
| `pages/map/hooks/useMapOffset.ts` | 7 |
| `components/settings/PositioningSection.tsx` | 6 |
| `components/utils.tsx` | 6 |
| `pages/map/components/ImportOpenMowerModal.tsx` | 6 |
| `utils/datumGps.ts` | 6 |
| `hooks/useImuYawCalibration.ts` | 5 |

Functional groups use consistent path-based classification across both snapshots:
`pages/map`, MapPage, DrawControl and modes belong to map; settings sections,
SettingsPage and settings hooks/utilities belong to settings; GNSS/IMU hooks and
datum helpers form their own group; other shared components/utilities/pages are
listed separately. Tests inherit their source area's group.

| Functional area | Before | After |
| --- | ---: | ---: |
| Fleet | 4 | 4 |
| GNSS / IMU data and calibration | 11 | 11 |
| Map display and editing (including tests) | 112 | 90 |
| Other shared UI, pages and utilities | 37 | 37 |
| Robot action / power UI | 13 | 13 |
| Settings UI and persistence (including tests) | 45 | 45 |

Generated ROS types are excluded by the existing ESLint configuration; the
API generator's existing blanket directive means its generated file reports no
warnings. These are exclusions of the baseline, not evidence that generated
code was newly audited or fixed. This batch changes only maintained code and
adds no suppressions or assertions.

## Selected scope and runtime risk

Two related consumers were selected: `useMapOffset.ts` (7 warnings) and
`useMapBearing.ts` (4), with `useMapOffset.test.ts` (12 warnings). These manage
three string-valued DB preferences, not physical telemetry. The defects were
concrete: parseFloat accepted partially numeric strings/infinity; nonfinite
edits could be persisted; overlapping writes could complete in reverse order;
old config echoes could replace a newer draft. Bearing's cleanup depended on
`flush`, whose dependencies included the freshly recreated `useConfig.setConfig`
callback. A rerender could therefore flush an edit before the debounce deadline.
Offset timers could outlive the component and report errors after navigation.

The larger map editor cluster (DrawControl/useMapEditing) needs careful polygon
and Mapbox callback contract tracing before changes. It participates in map
editing/navigation geometry. Map file import/save and tracked obstacle actions
also have persistent map effects. Power UI has command side effects. These were
not broadened into this display-only batch merely for their warning counts.
Advanced settings' unchecked callbacks/scalar coercion and the datum/calibration
helpers' unchecked response messages remain separate actionable clusters. Their
warnings indicate potential risks, not proof of every listed producer failing.

## Actual producer → decoder → state → consumer

- `gui/pkg/api/config.go`: POST `/config/keys/get` returns a string record;
  absent keys keep the empty request placeholder. POST `/config/keys/set`
  validates a single JSON object of strings, writes DB keys and echoes the body.
  DB writes are not transactional. The generated `Api.ts` client can expose
  errors in `response.error` or reject with a parsed HttpResponse.
- `hooks/useConfig.tsx`: existing runtime `decodeStringRecord` validation,
  explicit response-error checks, matching write echoes, guarded unmount state
  and late-initial-read protection. This hook was inspected and is unchanged.
- The new focused `useMapDisplayPreference` hook consumes those validated
  string records and parses only complete finite decimal strings (including
  exponent notation and whitespace). Zero remains valid. The exact keys are
  `gui.map.offset.x`, `gui.map.offset.y`, `gui.map.display.bearing`.
- `useMapOffset` and `useMapBearing` expose local display values to MapPage and
  MapOffsetPanel/MapRotationPanel; the unchanged camera hook applies the visual
  bearing to Mapbox. No ROS source, physical observation or telemetry timestamp
  is introduced by these preferences.

Missing keys retain the existing GUI default of zero. A present malformed value
is rejected, preserving the last usable display value (or the initial default
when none has been loaded). It is never written back as a repair. Prefix strings
such as `45deg` or `3m` are no longer accepted as numbers; current producers and
writers emit plain decimal strings. Empty DB placeholders remain an unavailable
stored preference, with the existing display default, not a physical zero reading.

## Implementation and async behavior

The two hooks share the same per-preference debounce/ordered-write lifetime in
`useMapDisplayPreference.ts`, rather than implementing subtly different cleanup
policies. This helper is specific to the three GUI DB keys, not a validation
framework. Offsets keep their 1,000 ms debounce; bearing keeps 400 ms. Bearing's
existing `[-180, 180)` formula, compass units, map datum, offset formula and
camera/click projections are unchanged. Existing reset/cleared GUI display inputs
still intentionally request zero; sensor null/YAML deletion is unrelated.

Writes for a preference serialize. Pending newer edits remain visible through
late initial reads and old write echoes. Callback and notification identities
can change without flushing or resetting the deadline. Only the committed latest
writer and language are used. A failed current write reports one typed error and
retains the unsaved local display draft; it does not update the DB reference,
claim success, mark any telemetry fresh or automatically retry. A superseded
failure does not report an error against a newer edit; its explicitly requested
successor still executes. Malformed or unconfirmed API echoes are rejected by the
unchanged useConfig boundary.

Leaving the component flushes an already requested, unsent edit once and clears
its timer. This preserves bearing's existing quick-rotate/navigation behavior
and makes offset navigation follow the same rule. Offset persistence now starts
on unmount instead of waiting for the old timer; no extra operation is created.
Queued requested writes can finish after unmount without local state updates or
notifications. A previously failed write is not queued again by navigation.
Separate keys and hook instances retain independent timers and queues.

| Maintained file | Before | After |
| --- | ---: | ---: |
| `pages/map/hooks/useMapOffset.ts` | 7 | 0 |
| `pages/map/hooks/useMapBearing.ts` | 4 | 0 |
| `pages/map/hooks/useMapOffset.test.ts` | 12 | 0 |
| `pages/map/hooks/useMapDisplayPreference.ts` (new) | 0 | 1 |
| `pages/map/hooks/useMapBearing.test.ts` (new) | 0 | 0 |
| `pages/map/hooks/mapDisplayPreferences.integration.test.tsx` (new) | 0 | 0 |

The remaining helper warning is `react-hooks/set-state-in-effect`: external DB
preferences may load after mount, and a guarded effect synchronizes the display
when no local draft owns it. This is intentional synchronization, not a missing
API validator. Removing the effect would lose late loads; hiding the warning or
rewriting state solely to silence it was avoided. The two original synchronization
warnings become this one shared source location. All 21 selected unsafe typing
warnings are removed.

## Regression evidence and limits

Focused tests assert observable displayed values, exact DB payloads and write
counts: complete/malformed/nonfinite strings, missing keys, numeric zero, bearing
normalisation, debounce deadlines, callback rerenders, late reads/old echoes,
ordered overlapping edits, latest and obsolete failures, unmount flushes,
independent instances and absence of automatic retries. Integration tests use the
actual useConfig decoder with mocked generated requests: HTTP/API rejection,
missing echo, malformed echo, malformed DB reads and delayed initial values.
No existing test was removed; old unsafe option assertions are replaced by
accurately typed writer/notification mocks. Camera restoration tests are unchanged.

Ordering is scoped to one preference in one hook instance. This is not a server
transaction, distributed lock, cross-tab revision system or claim that every
independent writer is ordered. An unsaved failed draft intentionally takes
precedence over later config updates while the component remains mounted; an
explicit new edit submits a new request. Remount reloads the DB. Unmount starts the flush but full browser shutdown
cannot guarantee HTTP delivery; no keepalive/beacon protocol was added. Missing/malformed
preferences do not gain a physical availability/freshness interpretation.

No GNSS, receiver actions, hardware routes, motor tuning, DB/YAML ownership,
dock capture, calibration_status contract, sensor placement or blade/emergency
behavior changed. No ROS2, firmware, backend, generated or MAVROS code was edited.
No hardware action was used for tests. Existing Vite chunk-size and Ant Design
deprecation notices remain outside the ESLint warning count.

## Final validation

- Focused Vitest: **38 tests passed**, four files (27 added regressions plus 11
  existing offset/camera tests).
- `yarn test`: **1,773 tests passed**, 175 files; separate appearance registry
  test passed. Baseline: 1,746 tests, 173 files. No tests were removed or disabled.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-batch11-dist`: passed. Baseline build passed
  separately with `/tmp/gui-batch11-baseline-dist`.
- Full ESLint JSON inventory: **222 → 200 warnings**, **0 → 0 errors**. The
  rule, area and changed-file comparisons above include the remaining helper
  synchronization warning.
- Full GUI Go suite passed during baseline verification. Backend code was not
  changed, so no additional backend test rerun was required after frontend edits.
- `git diff --check`: passed. Final HEAD remains the starting commit and branch;
  no staged files. Existing MAVROS modifications remain untouched.

Changed/new files are the six maintained source/test entries in the table above
and this report. Temporary inventories, test logs and build outputs are outside
version control. Final `git status --short` and `git diff --stat` were checked;
the latter lists tracked changes only, while status also lists the four new files
(including this report). No unrelated tracked source modifications were introduced.

## Candidates for Batch 12

- DrawControl (17) and useMapEditing (12): trace Mapbox/GeoJSON feature and mode
  callback types first, with malformed geometry and edit-history coverage; keep
  map writes and robot commands outside any purely typing change.
- AdvancedSection (11): focus on finite scalar editing and preservation of
  structured configuration values and intentional null deletion, with actual
  settings-producer contracts and existing Batch 7B/10 guards retained.
- Map import/file boundaries (useMapFiles 9, ImportOpenMowerModal 6, related tests
  8): audit endpoint acknowledgements and parsed imported geometry before
  changing state; this needs its own scoped review due to persisted map effects.
