# GUI Lot 20 — Remaining Settings section types

## Preflight

Branch `fix/gui-warnings-errors`, HEAD `447cb0ba` (Lot 19 committed).
`git ls-remote origin refs/heads/fix/gui-warnings-errors` confirmed the same full
commit on GitHub before editing. The sandbox's initial DNS failure was resolved
by an authorized read-only network check. Index empty; the only preexisting
worktree change was the independent MAVROS submodule modification.
Baseline: **114 ESLint warnings, zero errors; 2,044 Vitest tests**.

Scope: PositioningSection.tsx, LocalizationSection.tsx, ObstaclesSection.tsx,
one new PositioningSection.test.tsx, and this report. Nothing staged/committed.

## Contracts and exact changes

### Positioning

The generated SettingsValues document remains the sparse settings contract.
Positioning's onChange emits string, number or null (numeric fields, string
profiles/device/current-baud and numeric baud/rates). The local Expert switch
changes component state, not a persisted boolean. Select generics now express
those actual option types, including the existing GnssSelectFieldOption type
for the signal-profile labels/descriptions. No dropdown option or handler changes.

onPersistGnssSettings and its partial dictionary use SettingsValues. The existing
pickGnssActionSettings helper replaces the identical own-key allowlist loop:
only present GNSS_ACTION_SETTINGS_KEYS are copied, without filling missing keys
or converting zero, null, empty strings or other existing values. The manager's
partial-save callback and YAML contract are unchanged. SettingsPage still supplies
savePartialValues; no persistence request is added on render or ordinary edits.

The datum path was traced through:

- Generated Api.mowglinext.callCreate: JSON POST to
  `/mowglinext/call/set_datum`; HTTP errors reject as an augmented Response and
  JSON decoding errors can resolve with response.error populated.
- `gui/pkg/api/mowglinext.go`: calls
  `/navsat_to_absolute_pose/set_datum` (`std_srvs/srv/Trigger`). A ROS success=false
  becomes an HTTP error. Successful replies return `{message: res.Message}`.
- `ros2/src/mowgli_localization/src/navsat_to_absolute_pose_node.cpp`: requires a
  received fix and its existing RTK-fixed status gate, updates the node's datum
  in memory, and formats latitude/longitude with fixed nine decimal places.

No response-format ambiguity was found on this active path. However, the generic
swagger-generated callCreate return type is ApiOkResponse, which does not describe
this command's message. Generated definitions remain untouched. The actual data
is treated as unknown and validated inside the section.

The handler now requires HTTP/API acceptance and a textual message containing
exactly two complete decimal numeric tokens. Both numbers must be finite and
within latitude [-90,90] and longitude [-180,180]. Empty values, prefix-only
parseFloat successes, nonfinite values, malformed JSON and contradictory supplied
error/success fields are rejected. Numeric nonzero underflow to zero is rejected.
Representable decimal/exponent tokens are accepted; no new rounding is applied.
The producer's nine-digit precision and genuine (0,0) coordinates remain valid.
JSON null is not a valid coordinate response.

Both coordinates are validated before either onChange call. Rejection leaves the
form unchanged and emits an error, never success. Structured backend error text
is extracted with requestErrorMessage rather than reading arbitrary fields or
stringifying the Response. Existing loading/finally handling and valid-response
success wording remain. No automatic retry, second service call or YAML POST.
Malformed pairs previously could insert NaN or report success after parseFloat
accepted a numeric prefix; those are the intended runtime behavior changes.

### Localization

onChange now takes number | boolean | null, matching its switches and numeric
controls. asBool accepts unknown because dynamic indexed SettingsValues include
unknown configuration keys; its existing exact comparisons are the runtime guard.
True and the historical string `"true"` remain enabled; the other existing cases
are unchanged. No new coercion, normalization or calibration operation.

### Obstacles

onChange now takes string | number | boolean | null. Segmented uses the existing
DigSensitivity type: off/low/medium/high are strings, not numeric option indices.
Numeric clearing still emits null and legitimate numeric zero is unchanged.
Normalization, margins, distances, slowdown values, enabled flags and detection
thresholds are unchanged.

TypeScript transpileModule comparisons against HEAD produced **identical
JavaScript for LocalizationSection and ObstaclesSection** (React JSX, ES2020,
ES modules). Existing tests are retained without adding tests for erased types.

## settingsSave audit and separate issues

A repository source reference search for settingsSave/savePartialSettingsValues
found only its definition in `utils/settingsSave.ts`; there is no active import
or invocation. Its comment claiming use by the manager/schema hook is stale.
It retains five warnings (four no-explicit-any, one no-unsafe-assignment) and is
neither modified nor deleted. No YAML-persistence refactor belongs to this lot.
The older `utils/datumGps.ts` helper likewise has no active source consumers and
still contains the previous unsafe parser; it is not reused or edited here.

Read-only ROS audit found a separate producer validation gap: on_navsat_fix stores
last_fix before association/validation, and on_set_datum checks fix presence and
status but does not check finite coordinates or latitude/longitude bounds before
updating its datum. A mocked RTK-status fix containing NaN or an out-of-range
coordinate can therefore reach an accepted Trigger response after changing the
node's in-memory datum. The GUI now rejects such a response locally; it cannot
undo a service operation already performed. A lost response also does not prove
the service had no effect. No rollback or automatic retry is claimed or attempted.
A separate ROS fix should test the service with mocked invalid fixes and require
rejection before any datum mutation. This is a source finding, not a reproduced
hardware incident or a claim that upstream producers emit these fixes in service.

The reply carries no acquisition identity/freshness evidence. This lot does not
change the existing RTK gate, GPS display, fix classifications or the backend.
The Invalid settings payload issue and diagnostic severity policy are deferred.

## Tests and validation

Focused command:

```sh
yarn vitest run src/components/settings/PositioningSection.test.tsx \
  src/components/settings/LocalizationSection.test.tsx \
  src/components/settings/ObstaclesSection.test.tsx \
  src/components/settings/gnssConfig.test.ts \
  src/hooks/useSettingsManager.merge.test.tsx --pool=threads --maxWorkers=2
```

**88 passed**, five files, including **29 new Positioning regressions**. They
cover exact producer-format coordinates, nine-digit precision, zero and boundary
coordinates, malformed messages/pairs, invalid ranges, nonfinite/underflow values,
HTTP refusal and error bodies, generated-client JSON errors, transport failure,
no false success, unchanged form on rejection, null on manual clearing, the GNSS
allowlist and absence of implicit persistence/retry. Child receiver tooling is
stubbed for the allowlist integration; it does not execute a receiver action.
Existing Localization/Obstacles tests cover dependency locks, boolean toggles,
string sensitivity and unchanged safety-margin presentation. All requests are
mocked; no robot command, ROS subscription or configuration write was performed.

- Full `yarn test --pool=threads --maxWorkers=2`: **2,073 passed**, 188 files,
  plus the appearance registry test; 224.71 seconds (29 added tests).
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-lot20-dist`: passed.
- `yarn lint`: passed, 105 warnings and zero errors.
- `git diff --check`: passed; index remains empty.
- Global ESLint JSON: **114 → 105 warnings, zero errors**. Nine removed warnings:
  seven no-explicit-any, one no-unsafe-assignment, one no-unsafe-member-access.
  The two existing Obstacles react-refresh export warnings remain intentionally;
  moving its public constants/helper is outside this focused type correction.

| File | Before | After |
| --- | ---: | ---: |
| PositioningSection.tsx | 6 | 0 |
| LocalizationSection.tsx | 2 | 0 |
| ObstaclesSection.tsx | 3 | 2 |
| PositioningSection.test.tsx (new) | 0 | 0 |

| ESLint rule | Before | After |
| --- | ---: | ---: |
| @typescript-eslint/no-explicit-any | 42 | 35 |
| @typescript-eslint/no-unsafe-assignment | 24 | 23 |
| @typescript-eslint/no-unsafe-member-access | 13 | 12 |
| @typescript-eslint/no-unsafe-argument | 8 | 8 |
| @typescript-eslint/no-unsafe-call | 3 | 3 |
| @typescript-eslint/no-unsafe-return | 3 | 3 |
| react-refresh/only-export-components | 11 | 11 |
| react-hooks/set-state-in-effect | 8 | 8 |
| react-hooks/purity | 2 | 2 |
| **Total** | **114** | **105** |

Exact modified paths, all under gui/web:

- src/components/settings/PositioningSection.tsx
- src/components/settings/LocalizationSection.tsx
- src/components/settings/ObstaclesSection.tsx
- src/components/settings/PositioningSection.test.tsx
- docs/gui-lot20-settings-remaining-types.md

No backend, ROS, MAVROS, dependency, generated-file, CSS, fix-display or YAML API
change. Existing large-chunk/build and jsdom pseudo-element notices are outside
this lot. Temporary logs and builds live under `/tmp`. No staging, commit or push.
