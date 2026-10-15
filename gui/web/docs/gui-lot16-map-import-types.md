# GUI Lot 16 — Map import boundaries

## Baseline, scope and decision

Branch `fix/gui-warnings-errors`, HEAD `68441a70` (Lot 15 committed).
The initial GUI worktree/index were clean; only the independent MAVROS submodule
change was present. It remains untouched and unindexed. No backend, ROS2,
OpenMower service, Universal GNSS, generated source or dependency changes.

The user explicitly authorized frontend protections with the backend transaction
limitation documented. **Map+dock import is not atomic. This batch does not fix
that backend defect, simulate rollback or automatically retry a write.**

Modified code: useMapFiles.ts, useMapFiles.test.ts,
ImportOpenMowerModal.tsx, and its directly related addons/lifecycle test. This
report is the only additional file. Nothing is staged, committed or pushed.

## Traced contracts

- Native map.json backup/restore uses the existing `parseMapBackup` and
  `featuresFromJSON`/class model. The parser already validates area polygons,
  optional dock availability, original obstacle outlines and LiDAR corridors.
  Its schema and behavior are unchanged. Editable in-memory class instances are
  distinct from untrusted JSON.
- OpenMower POST `/api/import/openmower` accepts bare modern `areas` /
  `docking_stations`, historical bag-export fields, and an API envelope containing
  `map`, optional datum and `apply`. `parseImportRequest`, legacy conversion,
  `validateOutline`, `buildImportSummary`, reprojection and payload building are
  owned by `gui/pkg/api/openmower_import.go`. They remain unchanged. The frontend
  does not invent a second OpenMower file schema or implement projection itself.
- The Go summary contains integer counters, finite shifts, area row names/types,
  vertex/obstacle counts, navigation booleans, measured areas, string warnings and
  `applied`. Dock is omitted or nullable; its present coordinates are numbers.
  `mn_datum_configured` is a boolean from the Go producer; absence remains allowed
  for older producers and does not authorize adopting a datum.
- Editable GeoJSON exports carry Polygon workareas/navigation/obstacles and a
  Point dock, IDs, and class properties. The existing class factory expects these
  geometries; it does not support generic GeoJSON geometry collections or
  MultiPolygon editing. GeoJSON import is local preparation, not a server write.

## Root causes and changes

The warnings came from any-typed catch variables, untyped mock arguments and
partial mock objects asserted to complete API/notification types. Separately,
JSON.parse/fetch responses were asserted to summary/FeatureCollection interfaces
without runtime checks.

### Files and GeoJSON

File selection reads the typed input.files closure instead of casting a native
Event to a React ChangeEvent. GeoJSON now uses the same native file.text UTF-8
read as the other import paths. Cancellation removes the picker; read/parse
errors are surfaced instead of leaving an unhandled FileReader failure.
Existing localized parse/empty/unknown-type notification messages are retained;
read/parse failure can also include its textual reason.

Decoded JSON enters as unknown. A complete editable candidate is checked before
any editor setter: FeatureCollection/Feature objects, string or finite numeric
IDs (zero included), correct Point/Polygon roles, finite positions (zero included),
polygon ring shape and optional typed bounding box. The guard does not reproject,
close rings, simplify, reorder or repair coordinates. Three-point legacy rings
remain accepted; all rings and extra position dimensions are retained.

Known polygon metadata overrides must match the actual class consumers: present
name/color are strings; present order/index/source indices are finite numbers.
Absent overrides retain existing class defaults. Null name/order are rejected,
since spreading them over defaults would break trim()/toString()/ordering later.
Unconsumed extra metadata, including null, is retained in the validated snapshot
and polygon properties; the factory's existing treatment of dock-specific
metadata is unchanged. Null/absent dock heading retains its existing
unavailable-heading behavior. Numeric parent IDs
are normalized to string keys, just like feature IDs, so zero is retained.

Duplicate normalized IDs and `__proto__` IDs are rejected rather than overwriting
or mutating a feature dictionary. The factory runs before state mutation. If it
cannot rebuild every feature (notably an obstacle-only document without a parent),
the import fails as a whole. No subset replaces the current editable map.

### OpenMower preview and application

Initial preview and reprojection decode the actual Go summary before accepting
it. Required fields, finite measurements, numeric counters, rows, booleans and
string warnings are checked. Optional absent datum capability and null/absent
dock remain available/unavailable as before. Preview requires `applied=false`;
a successful apply acknowledgment requires `applied=true`. No fabricated datum,
capability or coordinate fallback is introduced.

A failed preview leaves the previous preview/file pair and editor state intact.
The original file text is still sent/stashed verbatim, and modern/legacy file
parsing/reprojection remain backend-owned. One dangerous non-export case is
blocked before any preview request: a picked API envelope carrying map + apply
true, including Go-compatible ASCII case variants Map/Apply. Otherwise a file
could invoke the write route before the operator's confirmation. Ordinary map
exports are unchanged; files cannot grant write authorization.

Only the existing explicit confirmation sends apply=true. An HTTP-OK but malformed
or unapplied acknowledgment is unconfirmed: local drafts are not cleared, no
success is announced, no datum request follows and no automatic retry occurs.
The error explicitly says the server may already have changed the map.

The Go error prefix `set docking point:` is emitted only after replaceMapInternal
succeeds. In that specific HTTP failure the GUI now prefixes the original status
and body with "map replaced, but setting the dock failed". It does not infer
rollback from other HTTP errors. The existing later YAML-datum failure already
says the map was imported but writing the datum failed. Modal errors remain
visible, Cancel does not write, and retry requires another explicit Apply.

### Test types and error handling

Catch paths use the existing textual errorMessage helper, preserving ordinary
Error/plain-message/string reasons and preventing arbitrary objects reaching UI
error state. Tests use a real generated Api with an in-memory customFetch and
method spies; all notification methods are present. Request assertions use the
actual business request types, without any or partial-object casts. No robot
API is contacted.

## Validation

- Focused suites (useMapFiles, mapBackup, ImportOpenMowerModal): **83 tests passed
  in three files**. Coverage includes valid modern/legacy preview payloads,
  malformed file/read/summary/geometry/metadata failures, zero ID/coordinates/order,
  absent/null dock, local map preservation, cancellation, explicit retry,
  write-bearing API envelopes, unconfirmed acknowledgments and partial dock errors.
- Full `yarn test --pool=threads --maxWorkers=2`: **1,951 Vitest tests passed in
  183 files**, 195.38 seconds, plus the appearance registry test. Baseline 1,906;
  **45 regression cases added**. A preliminary run was stopped when final null
  metadata/orphan tests were added; these numbers are from the final complete run.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-lot16-dist`: passed.
- Global ESLint JSON before/after and `yarn lint`: **166 → 143 warnings**,
  **zero errors**. No configuration changes or suppressions.
- `git diff --check`: passed; index remains empty.

| File | Warnings before | After |
| --- | ---: | ---: |
| pages/map/hooks/useMapFiles.ts | 9 | 0 |
| pages/map/hooks/useMapFiles.test.ts | 8 | 0 |
| pages/map/components/ImportOpenMowerModal.tsx | 6 | 0 |
| pages/map/components/ImportOpenMowerModal.addons.test.tsx | 0 | 0 |
| **Global** | **166** | **143** |

| Rule | Before | After |
| --- | ---: | ---: |
| no-explicit-any | 64 | 58 |
| no-unsafe-assignment | 34 | 30 |
| no-unsafe-member-access | 29 | 19 |
| no-unsafe-argument | 11 | 8 |
| no-unsafe-call | 4 | 4 |
| no-unsafe-return | 3 | 3 |
| react-refresh/only-export-components | 11 | 11 |
| react-hooks/set-state-in-effect | 8 | 8 |
| react-hooks/purity | 2 | 2 |
| **Total warnings** | **166** | **143** |
| **Errors** | **0** | **0** |

Raw inventories/logs/builds are in `/tmp/gui-lot16-*`, outside version control.

## PRIORITY BACKEND ANOMALY — map replacement followed by dock failure

Evidence baseline: unchanged Go source at HEAD 68441a70; code inspection and
mocked frontend HTTP regressions, not a robot observation or a new Go integration
test. In openmower_import.go, applyImport calls replaceMapInternal and waits for
success, then calls setDockingPointInternal. replaceMapInternal can restore the
previous map for failures inside its own replacement sequence (map_replace.go).
That protection does not include the subsequent dock write. A dock error is
returned as HTTP 500 with `set docking point: ...` after the new areas are already
saved. **HTTP failure does not mean the previous map was retained.**

Offline reproduction for the separate backend corrective batch:

1. Use a mocked ROS provider and temporary config/map data, never a live mower.
   Initialize map A and dock A; prepare a valid different map B with a dock.
2. POST `/api/import/openmower` with `{map: <valid OpenMower map B>, apply: true}`.
   Let clear/add/save calls succeed. Force only the subsequent docking service
   call to explicitly reject (or fail its transport).
3. Observe HTTP 500, `set docking point: ...`, and map B already saved. For an
   explicit rejection without side effects the dock remains A; a transport
   failure leaves dock outcome uncertain. There is no enclosing map+dock rollback.
4. A future backend regression must assert coherent map+dock results on every
   failure point, or an explicit partial-success contract. A frontend cannot
   manufacture such atomicity. That backend work is high priority and separate.

The added frontend regression feeds this exact error phase through mocked fetch:
no draft-clearing setters, no datum write, no success toast and no second request.
The modal retains the partial-failure description; Cancel does not retry/rollback.
An operator must inspect the server's actual map/dock state before retrying or
saving old drafts. The GUI deliberately does not claim they are authoritative
after an unconfirmed or partially successful remote write.

## Other limits and manual review

- Validating the response cannot undo a write already performed by the server.
  Losing its acknowledgment is also an unconfirmed outcome, not proven failure.
  Importing the datum is a separate later operation, not a transaction with map/dock.
- Native backup restoration preserves the existing independent handling of
  original-outline/corridor persistence errors: geometry can already be loaded
  locally when an auxiliary restore fails. Their separate failure messages remain;
  this batch introduces no cross-store transaction or rollback.
- Geometry checks are structural/finite, not topology, self-intersection,
  geographic-range or projection proof. Polygon holes are retained on local import;
  the existing save path consumes the outer ring. Full hole-save semantics require
  a separate model/persistence decision and are not claimed solved here.
- GeoJSON retains existing dock-dirty behavior: loading a dock feature does not
  automatically mark it for a remote dock write. Native backup restoration's
  explicit dock-dirty handling is unchanged. No new automatic hardware write.
- Existing obstacle-parent fallback to the first workarea is retained when a
  referenced parent is missing and some parent exists. No automatic geometric
  reassociation is invented. Review unfamiliar external maps before saving.
- Summary validation does not prove acquisition identity, that counts match the
  file semantically, or that preview/apply span a stable backend session. Existing
  file-pick/reprojection concurrency semantics remain; no distributed/session lock
  is claimed. No backend request-format or save-format changes.
- No browser or physical-robot run is claimed. Manual browser review should use
  mocked APIs: select/cancel/read UTF-8 files, inspect previews after changing datum,
  and exercise an explicit partial-error response before retry. Backend anomaly
  reproduction must remain offline until a separate safety-reviewed procedure.
- Existing large-chunk build and jsdom pseudo-element notices remain. The two
  older Playwright failures are outside this unit/typing batch and are not claimed
  fixed. No lint suppressions, arbitrary assertions or dependency upgrades.

## Final review follow-up — real obstacle parents and dock identity

Both edge cases are confirmed by the existing factory implementation:
`parents[parent_id]` on a `{}` dictionary can resolve inherited toString or
constructor instead of a MowingAreaFeature; the obstacle constructor previously
accepted that truthy value. Separately, DockFeatureBase always constructs ID
`dock` even when the snapshot was stored under a different key. Counting entries
alone does not detect either incoherence.

The correction stays in useMapFiles, without changing the shared factory. The
validated snapshot first identifies actual workarea IDs. Unresolved obstacle
parent references are removed from the derived parent_id link, allowing the
factory's existing first-available-area fallback. Original mowing_area metadata
is preserved. A real own workarea whose ID is toString or constructor remains a
valid explicit parent. With no real area, no obstacle can be accepted. Before
any setter, the reconstructed map must have matching storage keys/feature IDs
and every obstacle must hold a genuine MowingAreaFeature. Noncanonical dock IDs
are rejected, not silently renamed; historical GUI exports already use `dock`.
No geometry, source metadata or fallback ordering is changed.

Twelve additional regressions cover inherited parent names with no area, fallback
with an available area (including zero ID), references to navigation/nonexistent
parents, actual own workareas using prototype-like names, and foreign string or
numeric dock IDs. Rejected imports call none of the map/features/edit/dirty/dock
setters, emit no success and perform no map write. Existing canonical dock/zero
coordinate regressions remain in the targeted suite. Only useMapFiles.ts, its
test and this report change in this follow-up. Shared types/map.ts, backend and
MAVROS remain unchanged. The backend transaction anomaly above remains open.

Follow-up validation: **126 targeted tests passed in four files**; full
`yarn test --pool=threads --maxWorkers=2`: **1,963 tests passed in 183 files**,
196.70 seconds, plus the appearance registry test. TypeScript passes; global
ESLint remains **143 warnings / zero errors**, including zero in modified files.
`git diff --check` passes and the index is empty. No additional build or browser
run is claimed for this follow-up; the initial Lot 16 production build above
passed. Logs are `/tmp/gui-lot16-followup-*`. No staging, commit or push.
