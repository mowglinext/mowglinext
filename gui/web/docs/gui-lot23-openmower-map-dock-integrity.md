# OpenMower map/dock outcome integrity — GUI Lot 23

## Baseline and reproduction before correction

Branch `fix/gui-warnings-errors`, HEAD `72245053` (Lot 22 committed).
The index was empty and the only preexisting worktree change was the independent
MAVROS submodule modification. Baseline ESLint: **105 warnings, zero errors**;
Vitest: **2,086 tests**. MAVROS was not modified or staged.

The new `openmower_import_integrity_test.go` first reproduced the defect against
the original code, using the existing stateful `fakeMapServer` extended with
independent live/persisted dock state. Initial state: persisted map A, dock A.
Input: the existing OpenMower fixture (Front lawn, Driveway, dock B).

Both reproduction cases returned HTTP 500 **after** clear/add/save had succeeded.
The live and persisted area lists were B. There was exactly one clear, one
explicit save and one dock call; no area rollback happened after the dock error.
For explicit refusal, the fake retained dock A. For lost transport, the fake
applied and persisted dock B before returning `context.DeadlineExceeded`.
This demonstrates why a lost response cannot mean “dock unchanged”.

The initial test run failed only its new structured-result assertion:
`map_status` was missing. The assertions proving persisted B, the dock states,
HTTP 500 and absence of rollback already passed. Evidence is retained in
`/tmp/gui-lot23-go-red.log`.

Previously both failures returned only:

```json
{"error":"set docking point: …"}
```

The frontend inferred partial success from that English prefix. It could not
distinguish rejection, missing acknowledgement or a dock persistence warning.

## Result contract

Both preview/apply summaries and error bodies now carry typed `map_status` and
`dock_status`. Existing summary fields, the successful `applied` boolean,
HTTP status codes and the string `error` remain available to older clients.
Preview is still read-only; an error still uses HTTP 400/500 rather than
pretending the whole operation succeeded.

| Observed phase/result | `map_status` | `dock_status` |
| --- | --- | --- |
| Invalid request, unavailable snapshot, canceled lock wait, cancellation before clear | `unchanged` | `not_attempted` |
| Failed clear/add; previous areas re-added and save acknowledged | `restored` | `not_attempted` |
| Failed clear/add; restoration not confirmed | `unconfirmed` | `not_attempted` |
| New areas acknowledged; explicit save refused or response lost | `active_persistence_unconfirmed` | `not_attempted` |
| Area save acknowledged, context canceled before dock dispatch | `persisted` | `not_attempted` |
| Area save acknowledged, dock answers `success=false` | `persisted` | `rejected` |
| Area save acknowledged, dock transport fails | `persisted` | `unconfirmed` |
| Dock answers success with a persistence warning | `persisted` | `active_persistence_unconfirmed` |
| Area save and dock success without warning acknowledged | `persisted` | `confirmed` |
| Successful area import with no dock requested | `persisted` | `not_requested` |

When no dock was requested, `not_requested` also applies to earlier phases.
Preview uses `unchanged` and `not_attempted` or `not_requested`, with
`applied=false`. Its summary contains no write acknowledgement.

An explicit rejection now returns, for example:

```json
{"error":"set docking point: dock pose rejected by map_server: …","map_status":"persisted","dock_status":"rejected"}
```

Lost dock transport returns the same persisted map state and
`dock_status="unconfirmed"`. No dock refusal is inferred from the exception.

These statuses describe acknowledgements within one operation, not continuously
observed physical state. `restored` means the previous **areas** were re-added
and their save acknowledged; it is not a joint map/dock rollback. `persisted`
means the ROS save service acknowledged its persistence operation, not that the
GUI independently read the file or proved power-loss durability.

## Producer contracts and recovery design

Reviewed `openmower_import.go`, `map_replace.go`, relevant routes/helpers in
`mowglinext.go`, their tests, generated service models, `SetDockingPoint.srv`,
and the map-server callbacks in `area_manager.cpp`.

- `clear_map` and `save_areas` use Trigger acknowledgements; `add_area` has its
  own success flag. The shared replacement helper now checks an explicit clear
  refusal, rather than adding new areas after a refused clear.
- `add_area` can auto-save. Therefore a failed final `save_areas` proves neither
  that disk still contains A nor that B will be lost on restart. The manual
  replacement error wording was corrected from “NOT written” to “persistence
  was NOT confirmed”; its regression retains the live-map and storage-reason
  assertions.
- `SetDockingPoint` can accept a pose in memory yet fail to write YAML, reporting
  success with a nonempty message. The import treats such a message
  conservatively as unconfirmed persistence, not full success. The shared
  low-level request helper exposes the complete response. The legacy manual
  dock route retains its existing acknowledgement semantics.
- The importer still requests manual positioning with the existing yaw options;
  it does not alter GPS capture, datum services, coordinate projections or gates.
  The fake also respects the existing default yaw source `PRESERVE`.

No new rollback follows a dock failure. There is no reliably correlated snapshot
of persisted dock state to restore, and doing so could overwrite an independent
calibration. The already-committed new areas remain available. The existing
area-only restoration policy for failed clear/add is retained, with a fresh,
bounded recovery context; failed restoration is explicitly uncertain.

## Concurrency and cancellation

The channel-backed `mapReplacementLock` now covers the importer from snapshot
through the dock response. Replacement calls use the same locked core without
nested acquisition. Public GUI add, clear and dock routes also acquire this lock
after decoding, retaining their existing 10-second request contexts and response
shapes. PUT replacement retains its size-scaled budget. Waiting is cancellable;
no new unbounded lock or retry is introduced.

Import/replacement budgets remain `60 seconds + 5 seconds/area`, capped at six
minutes. The existing compensating area restoration can use a separate bounded
budget capped at six minutes. Calls honor their request/provider contexts.

Deterministic barriers pause import A at its dock call after saving its map.
Import B or manual replacement B is then observed entering the held lock. No B
service can run until A's dock call returns. A canceled waiter makes no service
call. Additional tests cover canceled public add/clear/dock routes, cancellation
before start, an expired context, after snapshot, after clear and after save.

**Important limit:** this lock coordinates requests in one GUI process. It cannot
serialize other GUI instances, direct ROS clients, recording or independent dock
calibration. Cancellation of a local wait does **not** cancel an already-sent ROS
command. After transport failure, that remote command may still finish later,
including after this lock is released. The tests prove local dispatch ordering,
not distributed linearizability. A globally atomic or correlated recovery
protocol still needs ROS-side operation identity/coordination and remains a
separate design task. Operators must verify remote state and pending activity
before another operation after an unconfirmed outcome.

## Frontend behavior and compatibility

The outcome decoder checks both enum fields together, rejects unknown or
contradictory combinations, and validates successful summaries against their
statuses and dock presence. Modern error handling uses those fields regardless
of the language or prefix of `error`. FR/EN messages distinguish not started,
restored areas, uncertain replacement, unconfirmed map persistence, explicit
dock rejection and uncertain dock persistence/outcome.

Partial, malformed, lost or unconfirmed responses preserve all local features,
map/edit/dirty flags, obstacle originals and corridors. No datum save, retry,
rollback or success notification follows them. Warnings explain that the draft
may differ from the robot and require verification before another operation.
No-dock success explicitly says that no dock update was requested.

Legacy previews remain supported. A legacy `set docking point:` error proves
only that area replacement completed; the dock is reported as unconfirmed, not
explicitly rejected. Other legacy errors establish no reliable map outcome.
A legacy HTTP 200 `applied=true` summary is recognized as an acknowledgement but
cannot expose dock persistence warnings: it is displayed as **unconfirmed**,
preserving drafts and skipping datum writes. This deliberate compatibility limit
requires an upgraded backend for full confirmed-success handling. It neither
claims the import failed nor triggers a second import.

The modal has a synchronous in-flight guard for duplicate confirmation callbacks
and Cancel, retains fields after failure, and does not let a read-only preview
erase a prior write outcome. Preview failures are shown alongside that outcome.
Closing still sends no command. A later explicit Apply remains an operator action;
the warning instructs verification before it.

## Validation

- New Go integrity coverage: **23 cases**, including live/persisted map and dock
  state, preview without ROS calls, success with/without dock, unreadable snapshot,
  successful/failed restoration, failed save, clear refusal, dock refusal/lost
  response/persistence warning, cancellation boundaries and concurrency.
- Those cases passed **20 repetitions** (460 case executions), using channels
  rather than sleeps. Full `go test ./...` passed, including the existing manual
  dock success/rejection/transport tests and map replacement tests.
- `go test -race` was attempted but could not build with the installed Go 1.24
  runtime: `runtime/race: package testmain: cannot find package`. No race-detector
  pass is claimed; this is an outstanding tooling limitation.
- Focused frontend: **109 tests passed**, hook and modal files. There are **39
  additional tests** over baseline, covering typed outcomes, malformed and legacy
  responses, draft preservation, zero datum values, no implicit datum save/retry,
  exact success conditions and single in-flight confirmation.
- Full frontend: `yarn test --pool=threads --maxWorkers=2` passed: **2,125 tests
  in 188 files**, plus the appearance registry test. The final run took 240.77
  seconds for Vitest.
- Chromium, `--retries=0 --workers=2`: **7 passed** in 15.4 seconds. Six new
  FR/EN cases exercise actual MapPage preview/confirmation, explicit refusal,
  uncertain dock outcome and confirmed success. They verify zero source datum
  values before and after failure, one apply per confirmation, no datum/robot
  writes, no implicit retry, visible translated warnings and Cancel. The existing
  map-layer regression also passed. These are targeted runs, not the full
  Playwright suite; earlier unrelated Playwright failures are not claimed fixed.
- `yarn tsc --noEmit` passed; `yarn build --outDir /tmp/gui-lot23-dist` passed.
- `yarn lint` passed; final full JSON inventory: **105 → 105 warnings, zero
  errors**. The changed frontend source/test files have no warnings. Initial
  test-harness `require-await` errors were corrected with explicit promise
  flushing, without suppressions.
- `gofmt -l` returned no files; `git diff --check` and whitespace checks of the
  three new files passed. The final index is empty. No dependency, configuration
  or generated-file edit is used.

| ESLint rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 35 | 35 |
| `@typescript-eslint/no-unsafe-assignment` | 23 | 23 |
| `@typescript-eslint/no-unsafe-member-access` | 12 | 12 |
| `@typescript-eslint/no-unsafe-argument` | 8 | 8 |
| `@typescript-eslint/no-unsafe-call` | 3 | 3 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 |
| `react-refresh/only-export-components` | 11 | 11 |
| `react-hooks/set-state-in-effect` | 8 | 8 |
| `react-hooks/purity` | 2 | 2 |
| **Total** | **105** | **105** |

The existing standard GeoJSON import and manual map-save regressions remain in
the full suite. Their hook paths, coordinate calculations and feature models
were not edited. Manual ROS replacement still uses the same normalized area
write/restore flow; only acknowledgement handling, truthful persistence wording
and request serialization changed.

Initial browser harness failures did not justify application changes: Node JSON
imports needed explicit attributes; the map fixture needed its required datum;
Ant Design icons are part of accessible names; the existing corridor-read POST
must not be counted as a write. Datum fields trigger a read-only preview on blur,
so the test now waits for re-enablement before editing the next field. jsdom does
not run the modal entrance animation; unit assertions check mounted content,
while Chromium checks actual visibility. No timeout was increased or console
warning masked. All browser APIs and WebSockets are mocked.

## Remaining risks before v1.6

- Joint map/dock atomicity, remote command cancellation/correlation and protection
  against independent writers remain unresolved protocol limitations. No automatic
  recovery is claimed after a dock timeout or response loss.
- Existing import yaw behavior is unchanged: `yaw_source=PRESERVE` retains stored
  yaw rather than adopting the source preview heading (already documented in
  `docs/IMPORT_OPENMOWER_MAP.md`). `dock_status=confirmed` confirms acceptance of
  that request, not equality with every preview coordinate/orientation.
- Static inspection also finds an existing validation mismatch: the summary
  excludes an outline beyond the coordinate bound, but the payload builder checks
  only vertex count. A source containing one valid area plus one three-vertex
  area beyond 100 km can preview fewer areas than it sends. This geometric
  validation issue is reported separately and is not changed in this outcome fix.
- Interior GeoJSON rings, GNSS classification, diagnostic severity, datum service
  behavior and the remaining ESLint debt are untouched.
- Existing Ant Design Tooltip/React compatibility notices, jsdom pseudo-element
  notices and the large-chunk build warning remain distinct from ESLint results.

No robot connection, motion, real map import or hardware configuration write was
performed. These are software-contract tests with temporary/mock state, not field
evidence of docking gates, geometry accuracy or filesystem durability.

## Review files and delivery

- `gui/pkg/api/map_replace.go`
- `gui/pkg/api/mowglinext.go`
- `gui/pkg/api/openmower_import.go`
- `gui/pkg/api/map_replace_test.go`
- `gui/pkg/api/openmower_import_integrity_test.go` (new)
- `gui/web/src/pages/map/hooks/useMapFiles.ts`
- `gui/web/src/pages/map/hooks/useMapFiles.test.ts`
- `gui/web/src/pages/map/components/ImportOpenMowerModal.tsx`
- `gui/web/src/pages/map/components/ImportOpenMowerModal.addons.test.tsx`
- `gui/web/src/i18n/locales/en.json`
- `gui/web/src/i18n/locales/fr.json`
- `gui/web/tests/e2e/openmower-import-outcomes.spec.ts` (new)
- `docs/IMPORT_OPENMOWER_MAP.md`
- `gui/web/docs/gui-lot23-openmower-map-dock-integrity.md` (new)

Final diff: `/tmp/gui-lot23-final.diff`, excluding MAVROS. No staging, commit or
push. Review and GO COMMIT remain with the operator.

## Review follow-up — saved areas, dock not attempted

The pair `map_status=persisted`, `dock_status=not_attempted` now selects the
dedicated FR/EN message `mapImportOpenMower.dockNotAttempted`. It states that the
service confirmed the area save, the dock update was not attempted, the local
draft is preserved, and the operator must verify the robot before trying again.
It also states that no further import or datum write is started automatically.
`dockUnconfirmed` remains the message for the distinct unknown dock outcome;
the existing legacy fallback is unchanged.

Only the frontend message selection, its existing parameterized assertion,
FR/EN translations, the browser matrix and this report were adjusted. No Go
contract, lock or other operational behavior changed in this follow-up.

Validation of the complementary change:

- **113 focused Vitest tests passed** in three files: map-files hook, import
  modal and locale parity. The existing not-attempted case now requires the
  dedicated message and still verifies draft preservation, no datum write,
  no implicit retry and no success notification.
- **Two Chromium tests passed**, FR/EN `not_attempted`, with `--retries=0`.
  The visible alert must contain the full dedicated translation and exclude
  `dockUnconfirmed`. Source zero values stay present, Cancel adds no request,
  only one apply occurs and no additional datum/robot write is sent. The
  browser matrix now contains eight import cases; only the two new cases were
  rerun for this follow-up.
- `yarn tsc --noEmit`, `yarn lint` and `git diff --check` passed.
  ESLint remains **105 warnings, zero errors**.
- Full Vitest/build/Go results above are the preceding Lot 23 validation;
  they were not rerun for this narrowly scoped presentation correction.

Complementary diff only: `/tmp/gui-lot23-dock-not-attempted.diff`.
The full review diff at `/tmp/gui-lot23-final.diff` is refreshed as well.
Index empty; no staging, commit or push; MAVROS preserved.
