# Geometry and exclusion integrity — GUI Lot 24

## Baseline and scope

Branch `fix/gui-warnings-errors`, HEAD
`fac6b5efca550ed1739468506bace3f4e39bab3f` (Lot 23 committed). The index was
empty; the only preexisting worktree change was the independent MAVROS submodule
modification. Baseline: **105 ESLint warnings, zero errors, 2,125 Vitest tests**.
MAVROS is untouched. All changes remain uncommitted and unstaged.

This report separates two reviewable changes: A, fail-closed GeoJSON/manual-save
geometry validation; B, shared OpenMower preview/payload selection. No ROS,
firmware, generated models, dependencies or lint configuration are changed.
All service calls in validation use mocks; no robot import or hardware write ran.

## A — GeoJSON holes and manual map saving

### Reproduction before correction

A new hook regression followed the real `decodeEditableGeoJSON` →
`featuresFromJSON` → editing feature map → `handleSaveMap` path. The input was
`garden-A`, named Garden A, with a closed 20 m square and two distinct closed
interior squares. Coordinates were finite, produced using the existing
`transpose` helper. All three rings survived decoding, class rehydration and
`serializeFeatures`.

Before correction, Save called `putMowglinext` once with only the outer ring and
**zero obstacles**. The assertion requiring the two exclusions failed; the log
is `/tmp/gui-lot24-frontend-red.log`. The loss occurred in `handleSaveMap`, where
both area and obstacle conversion read only `geometry.coordinates[0]`. It did
not occur in the file parser or serialization.

After choosing refusal rather than an unproved conversion, that regression now
requires import rejection. It also constructs and rehydrates an older multi-ring
draft, verifies every ring remains present, and proves Save sends no replacement
or success/dirty cleanup. The original red result is not described as a passing
hole-conversion test.

### Representation decision

`MapArea` exposes one `geometry_msgs/Polygon` outline plus separately modelled
obstacles. An obstacle has a parent, identities and editing/persistence semantics;
a GeoJSON hole has none of those separate identities. Navigation areas also use
this model. The inspected GUI models, backup representation, ROS message and
map-server persistence/area handling do not establish an invertible,
semantically identical conversion of arbitrary holes into separate obstacles.
No such conversion or new topology support is introduced.

| Representation | Result |
| --- | --- |
| Simple workarea Polygon | Accepted after source and projected-shape validation |
| Simple navigation Polygon | Same validation; existing navigation flag retained |
| Explicit simple obstacle with resolved parent | Retained separately; existing parent/fallback selection preserved |
| Explicit obstacle without a resolvable parent | Save rejected rather than dropping an exclusion |
| Polygon with one or several interior rings | Import and Save rejected, including navigation/obstacle polygons |
| Empty, degenerate, crossing or outside interior ring | Still unsupported; never repaired or silently removed |
| MultiPolygon | Rejected; never flattened into one component |
| Invalid/nonfinite/degenerate/self-intersecting outer ring | Rejected before writes |
| Existing open simple ring | Still accepted; no new closure or winding rewrite |
| Existing closed simple ring | Preserved; closing point convention unchanged |

For example, `[outer, holeA, holeB]` no longer becomes ROS `outline=outer,
obstacles=[]`. It produces a localized refusal and zero writes. A supported
GeoJSON workarea plus **two explicitly modelled obstacle features** still
produces one area with two obstacles. Its regression reconstructs the GUI
classes from that actual saved ROS-shaped payload, verifies coordinates and
parents, and retains the existing order/identity handling. It is a mocked
software readback, not a real ROS map-server or hardware run.

### Validation and earlier editing paths

The pure `mapGeometryContract.ts` boundary checks finite numeric positions,
geographic bounds, sufficient distinct vertices, nonzero area and intersections
without reordering or repairing points. Projection uses the existing
`itranspose` and consecutive-point deduplication. The resulting ring must remain
valid after that deduplication and after the float32 precision used by ROS
`Point32`; the sent numeric values are not newly rounded. Invalid datum/offset
values and projected coordinates beyond ±100,000 m are refused. This local-frame
bound matches the established OpenMower bound; it is not a measurement or robot
operating-range claim. Zero coordinates and either winding remain valid.

Save prepares **all** area and obstacle shapes and validates an edited dock
before the first map PUT or dock POST. Parent resolution also completes before
writes. A failure preserves features, edit mode, dock-dirty and unsaved flags;
there is no success notification or cleanup. FR/EN messages explain the refusal,
preserved source/draft and absence of a map replacement/dock command.

The audit also followed earlier loss opportunities:

- `setGeometry`, feature snapshots, rehydration, history and GeoJSON export carry
  complete Polygon coordinates. Regressions prove two holes survive cloning,
  undo/redo and export with properties intact. Export is deliberately still
  available for recovering an older unsupported draft.
- `DrawControl` receives and forwards the complete geometry. The test advances
  its existing synchronization timer, not a real sleep. Map rendering consumes
  these full GeoJSON geometries; no projection or rendering code was edited.
- Turf difference/union paths retain the returned Polygon, including holes.
  A newly created hole therefore remains visible in the draft and blocks Save.
- Split reconstructs from the outer ring; shrink-memory/type-conversion paths
  also have one-ring assumptions. Narrow guards now refuse multi-ring inputs
  before those operations can discard a ring or mutate the feature state.
- ROS-shaped backups contain single outlines and explicit obstacles. Their
  format and implementation were not changed. There is no claim that a backup
  can represent arbitrary GeoJSON holes.

A browser regression imports a simple local draft first, then attempts the
unsupported source. In FR and EN it verifies the full refusal message, unchanged
Save/dirty state, zero map/dock/settings/import writes, and downloads the actual
GeoJSON export to prove the original draft remains. The mobile toolbar is used
because the existing GeoJSON upload menu is there; this batch adds no desktop UI.
Separate no-hole browser cases explicitly Save once and check the intercepted
payload. The existing Edit action's server-backup request and read-only corridor
request are distinguished from map/dock/configuration writes.

## B — OpenMower preview/payload parity

### Reproduction before correction

`TestOpenMowerPreviewPayloadGeometryParity` starts with valid mowing area A,
three-vertex area B with X=100,001 m, one valid obstacle, and an obstacle whose
centroid lies in A but whose first vertex exceeds the bound. Before correction,
`buildImportSummary` reported **one area and one obstacle**; the separately
built payload contained **two areas and two obstacles**. The new test failed
both parity assertions (`/tmp/gui-lot24-go-red.log`), without any real ROS call.

### One validated selection

The route now calls `buildOpenMowerImport` once. The same validated area list,
obstacle assignment and area loop build the summary, counts, per-area obstacle
counts and replacement payload. The existing helper entry points delegate to
that builder; they no longer have independent validation/reselection rules.

Source count/finite/±100 km checks and existing outline deduplication remain.
Degenerate/self-intersecting outlines and invalid projected/float32 shapes now
produce explicit skip warnings. Invalid shapes are absent from **both** outputs.
Source input is not mutated. Request/target projection frames and the projected
dock must be finite and representable before any apply call. This validates the
import's projection inputs; it does not write or recalibrate the datum.

Inactive, draft and unknown areas retain their filtering behavior/warnings.
Mow/navigation order and flags, relevant names/metadata, existing centroid-based
first-parent matching, orphan warnings, first-active-dock choice, UTM/ENU
projection and yaw behavior remain unchanged. The valid legacy real-map fixture
still yields eight workareas, one navigation area and its explicit obstacle.

For the reproduced input, the result is consistently `areas=[A]` with only the
valid obstacle, and warnings identify B and the invalid obstacle. A direct
**Apply with no admissible mowing area returns HTTP 400 before any ROS call**,
including navigation-only, obstacle-only, inactive, draft, invalid-only and empty
maps. Preview remains read-only and can explain why nothing is admissible.
The prior map/dock state is untouched in these mocked refusal cases.

Lot 23's outcome fields, lock, command sequence, cancellation/recovery budgets,
legacy handling, no-retry policy and partial-success interpretation are not
changed. Its integrity/concurrency tests and FR/EN browser outcomes are rerun.
There is no new rollback or datum write after a partial outcome.

## Validation

- Focused frontend: **248 tests passed in nine files**, including file imports,
  manual saves, pure geometry guards, editing, history, Draw, feature models,
  backups, the OpenMower modal and locale parity.
- Full frontend: `yarn test --pool=threads --maxWorkers=2` passed: **2,175 tests
  in 189 files**, plus the appearance registry test (228.71 seconds for Vitest).
  This adds 50 frontend regressions over baseline.
- Full `go test ./...`: passed after the final Go regressions. Targeted geometry,
  import, real legacy map, replacement and Lot 23 outcome/concurrency tests passed
  **20 repetitions**. New tests cover source/projected bounds, float32 collapse,
  invalid frames/dock, filtering, contents/counts, preview without ROS and
  refusal without prior-state changes.
- Chromium: **15 tests passed**, `--retries=0 --workers=2`, 25.5 seconds: six new
  GeoJSON FR/EN cases, eight Lot 23 OpenMower outcome cases and the existing map
  layer/console regression. APIs/WebSockets are mocked throughout.
- `yarn tsc --noEmit`: passed. `yarn build --outDir /tmp/gui-lot24-dist`: passed.
- `yarn lint` and full JSON inventories: **105 → 105 warnings, zero errors**.
  No new suppressions, rule/configuration changes or generated-source edits.
- Formatting and final diff checks: `gofmt -l` returned no files;
  `git diff --check` passed.
- Race detector: the targeted OpenMower/import/replacement group, including
  Lot 23's concurrent map/dock tests, passed with `go test -race` and the
  installed Go 1.24.0 (1.126 seconds). This is a targeted pass, not a full
  repository race run. Unlike the prior Lot 23 environment, the race runtime is
  usable here.

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

The first browser attempts exposed ambiguous harness locators: the map and mobile
navigation both have a More button, and import/save success notifications coexist.
Selectors now identify the map button and the specific saved notification. No
application change, arbitrary sleep, increased timeout, retry or console masking
was used to address those harness failures. Existing Ant Design React 19,
Dropdown/Tooltip deprecation notices, jsdom pseudo-element notices and the build
large-chunk warning remain distinct from ESLint results.

## Limits and review

- Holes and MultiPolygon are deliberately **unsupported**, not converted. An
  older multi-ring draft remains recoverable/exportable but must be represented
  safely by an explicit operator decision before it can be saved. No exclusion
  geometry is invented for the operator.
- Ring intersection tests use floating-point predicates and quadratic pairwise
  checks. No new vertex-count limit is imposed; very large input can be costly.
  These checks are not a general-purpose GIS topology proof or evidence of field
  accuracy. Source altitude is not newly interpreted; the existing 2D map
  conversion still sends Z=0.
- OpenMower orphan/ambiguous-parent rules and exclusion of invalid source areas
  remain warning-based. The operator must review those warnings. No new full
  containment guarantee for attached obstacles or alternative parenting rule is
  claimed.
- Preview/payload parity applies to the same source and projection frame. The
  route does not reselect a weaker write list. Separate preview and Apply HTTP
  requests still reread configuration; an independent datum change between them
  has no revision token or distributed lock in the existing protocol. That
  external-writer race is not solved or hidden by this change.
- Lot 23's joint map/dock atomicity, remote cancellation/correlation and
  independent-writer limitations remain. After a communication failure, verify
  robot state before another operation; there is no simulated frontend rollback.
- The browser run is targeted Chromium coverage, not the complete Playwright
  suite or all browsers. Earlier unrelated failures are not claimed resolved.
  No physical mowing, navigation or dock acceptance evidence is claimed.

Changed files, grouped for review (no sub-commits created):

A:
- `gui/web/src/pages/map/utils/mapGeometryContract.ts` (new)
- `gui/web/src/pages/map/utils/mapGeometryContract.test.ts` (new)
- `gui/web/src/pages/map/hooks/useMapFiles.ts`
- `gui/web/src/pages/map/hooks/useMapFiles.test.ts`
- `gui/web/src/pages/map/hooks/useMapEditing.ts`
- `gui/web/src/pages/map/hooks/useMapEditing.test.ts`
- `gui/web/src/pages/map/hooks/useMapEditHistory.test.ts`
- `gui/web/src/components/DrawControl.test.tsx`
- `gui/web/src/i18n/locales/en.json`
- `gui/web/src/i18n/locales/fr.json`
- `gui/web/tests/e2e/geojson-geometry-integrity.spec.ts` (new)

B:
- `gui/pkg/api/openmower_import.go`
- `gui/pkg/api/openmower_geometry_integrity_test.go` (new)

Report: `gui/web/docs/gui-lot24-geometry-geojson-integrity.md` (new).
Review diff: `/tmp/gui-lot24-final.diff`, including new files and excluding
MAVROS. Logs/build/browser artifacts stay in `/tmp`. Index empty; no staging,
commit or push. Await operator review before GO COMMIT.
