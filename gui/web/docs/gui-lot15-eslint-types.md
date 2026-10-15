# GUI Lot 15 — Map editing callback types

## Baseline and scope

Branch `fix/gui-warnings-errors`, HEAD
`694c6e9b2c78bdae4a48670657ea18461a4573de`: mini-lot 14 is committed.
Initial GUI worktree and index were clean; the only existing change was the
independent MAVROS submodule modification. It remains untouched and unindexed.
ESLint JSON confirmed **187 warnings / zero errors** before changes.

Three code files form this batch: DrawControl.tsx, useMapEditing.ts and
DrawControl.test.tsx. This report is the fourth changed file. No dependencies,
styles, lint configuration, generated files, backend, ROS2, MAVROS or GNSS
contracts change. Nothing is staged, committed or pushed.

## Root causes and changes

DrawControl's `Record<string, (e: any) => void>` erased the event/callback pairing.
The same any then propagated into six callbacks, while event-name strings were
unnecessarily cast for Mapbox registration/removal. An explicit DrawEvents map
and mapped DrawHandlers type now retain each consumer's existing payload shape.
Public callback props and listener implementations use that one map. The empty
listener ref is represented by null; cleanup iterates an empty object when no
handlers are registered. Listener identity and committed callback refs remain
unchanged. No additional binding, disposal, delayed synchronization or mode
transition is introduced.

The installed Mapbox Draw source was inspected for create/update/combine/delete
and selection events, along with the repository's feature.open producer. These
are existing in-process events, not decoded HTTP/WebSocket data. Typing models
the fields already expected by the callbacks; it does not validate arbitrary
GeoJSON properties or replace the custom mode's internal feature representation.
The existing feature.open consumer contract is retained; validation of its
metadata is explicitly deferred below.

In useMapEditing, the existing instanceof MowingFeatureBase checks already narrow
both subtraction inputs to polygon features. Turf's featureCollection accepts
those instances directly; two any casts and their unsafe argument warning are
removed. The local MapboxDraw declaration accepts a string mode, so the existing
registered `split_line` mode needs no any cast either. Geometry, subtraction
algorithm, order, IDs, selection, return-value checks, notifications and split
behavior are unchanged. No runtime numeric/string conversion or data validation
was added. Null, zero, empty selections and absent details are not coerced.

## Validation

- Focused `vitest run` on DrawControl, useMapEditing, types/map and useMapEditHistory:
  **60 tests passed in four files**.
- Seven added forwarding cases cover all six event names plus absent optional
  details. They verify exact payload identity, current callbacks after rerender,
  no cross-dispatch/rebinding, and symmetric cleanup. Fixtures include zero
  coordinates/ID, null properties, empty arrays, update action and combine fields.
  Existing StrictMode/timer cleanup and geometry/history tests also pass.
- Full `yarn test --pool=threads --maxWorkers=2`: **1,906 tests passed in
  183 Vitest files**, 190.42 seconds, plus the separate appearance registry test.
  Baseline: 1,899; seven event forwarding regressions added. Existing jsdom
  getComputedStyle pseudo-element notices remain.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-lot15-dist`: passed; existing large-chunk notice remains.
- Global ESLint JSON: **187 → 166 warnings**, **zero errors** both times.
- `yarn lint` (including unused-disable checks): passed, 166 warnings/zero errors.
- `git diff --check`: passed.

No artificial casts, replacement any, suppressions, timeout changes or runtime
fallbacks were introduced. Existing unsafe custom-mode assignment and open-details
metadata handling were left visible rather than hidden. No hardware command or
real backend request is exercised by the tests.

## Rule inventory

| Rule | Before | After | Removed |
| --- | ---: | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 77 | 64 | 13 |
| `@typescript-eslint/no-unsafe-assignment` | 34 | 34 | 0 |
| `@typescript-eslint/no-unsafe-member-access` | 29 | 29 | 0 |
| `@typescript-eslint/no-unsafe-argument` | 19 | 11 | 8 |
| `@typescript-eslint/no-unsafe-call` | 4 | 4 | 0 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 | 0 |
| `react-refresh/only-export-components` | 11 | 11 | 0 |
| `react-hooks/set-state-in-effect` | 8 | 8 | 0 |
| `react-hooks/purity` | 2 | 2 | 0 |
| **Total** | **187** | **166** | **21** |

## Per-file inventory

All paths below are relative to `gui/web/src`. Files without warnings are omitted;
DrawControl.test.tsx has zero warnings before/after. Rule-count abbreviations:
`any` = no-explicit-any; `assign`, `member`, `arg`, `call`, `return` = the
corresponding no-unsafe-* TypeScript rules. `refresh` = only-export-components;
`effect` = set-state-in-effect; `purity` = react-hooks/purity. Changed counts are
shown before→after. Raw JSON logs are in `/tmp/gui-lot15-eslint-{before,after}.json`,
outside version control. The inventory uses `eslint src`; generated/vendored
sources retain the repository's existing exclusions and are not modified.

| File | Before | After | Warnings by rule |
| --- | ---: | ---: | --- |
| `components/DrawControl.tsx` | 17 | 1 | any 10→0, assign 1, arg 6→0 |
| `pages/map/hooks/useMapEditing.ts` | 12 | 7 | any 3→0, assign 7, arg 2→0 |
| `components/PowerMenu.tsx` | 9 | 9 | any 3, assign 3, member 3 |
| `pages/map/hooks/useMapFiles.ts` | 9 | 9 | any 3, assign 2, member 3, arg 1 |
| `pages/map/hooks/useMapStreams.ts` | 9 | 9 | any 7, effect 2 |
| `pages/map/components/TrackedObstaclesPanel.tsx` | 8 | 8 | any 3, assign 1, member 3, arg 1 |
| `pages/map/hooks/useMapFiles.test.ts` | 8 | 8 | any 1, assign 2, member 5 |
| `components/settings/PositioningSection.tsx` | 6 | 6 | any 4, assign 1, member 1 |
| `components/utils.tsx` | 6 | 6 | any 4, assign 2 |
| `pages/map/components/ImportOpenMowerModal.tsx` | 6 | 6 | any 2, member 2, arg 2 |
| `utils/datumGps.ts` | 6 | 6 | any 2, assign 1, member 2, arg 1 |
| `hooks/useImuYawCalibration.ts` | 5 | 5 | any 1, assign 2, member 2 |
| `utils/settingsSave.ts` | 5 | 5 | any 4, assign 1 |
| `components/MowerActions.tsx` | 4 | 4 | any 2, arg 1, refresh 1 |
| `pages/map/components/MapToolbarMobile.test.tsx` | 4 | 4 | any 2, assign 1, return 1 |
| `pages/map/hooks/useMapEditHistory.test.ts` | 4 | 4 | assign 3, call 1 |
| `pages/map/hooks/useResetMowingProgress.test.ts` | 4 | 4 | member 2, call 2 |
| `utils/containers.ts` | 4 | 4 | any 1, member 2, call 1 |
| `components/settings/ObstaclesSection.tsx` | 3 | 3 | any 1, refresh 2 |
| `hooks/useContainerRestart.ts` | 3 | 3 | any 1, assign 1, member 1 |
| `hooks/useNotificationCenter.tsx` | 3 | 3 | refresh 2, effect 1 |
| `pages/FleetPage.tsx` | 3 | 3 | any 1, member 1, arg 1 |
| `pages/map/components/MapToolbar.test.tsx` | 3 | 3 | any 2, assign 1 |
| `components/AppShell.tsx` | 2 | 2 | assign 1, member 1 |
| `components/settings/HardwareSection.tsx` | 2 | 2 | any 2 |
| `components/settings/LedsSection.test.tsx` | 2 | 2 | any 2 |
| `components/settings/LocalizationSection.tsx` | 2 | 2 | any 2 |
| `components/settings/UpdateChangelog.tsx` | 2 | 2 | refresh 2 |
| `hooks/useBTLog.ts` | 2 | 2 | any 1, assign 1 |
| `hooks/useFusionGraphDiagnostics.ts` | 2 | 2 | any 1, assign 1 |
| `hooks/useIOSInstallPrompt.ts` | 2 | 2 | any 1, member 1 |
| `pages/MapPage.tsx` | 2 | 2 | assign 1, arg 1 |
| `pages/map/hooks/useManualMode.test.ts` | 2 | 2 | any 1, arg 1 |
| `theme/ThemeContext.tsx` | 2 | 2 | refresh 2 |
| `components/BTStateGraph.tsx` | 1 | 1 | refresh 1 |
| `components/YearOfLawn.tsx` | 1 | 1 | purity 1 |
| `components/dashboard/DigEscalationBanner.test.tsx` | 1 | 1 | return 1 |
| `components/settings/BatterySection.tsx` | 1 | 1 | any 1 |
| `components/settings/DockingSection.tsx` | 1 | 1 | any 1 |
| `components/settings/LedsSection.tsx` | 1 | 1 | any 1 |
| `components/settings/MowingSection.tsx` | 1 | 1 | any 1 |
| `components/settings/MqttSection.tsx` | 1 | 1 | any 1 |
| `components/settings/NavigationSection.tsx` | 1 | 1 | any 1 |
| `components/settings/NtripSection.tsx` | 1 | 1 | any 1 |
| `components/settings/RainSection.tsx` | 1 | 1 | any 1 |
| `components/settings/SafetySection.tsx` | 1 | 1 | any 1 |
| `components/settings/UniversalGnssAdvancedSettings.test.tsx` | 1 | 1 | any 1 |
| `hooks/useCalibrationStatus.ts` | 1 | 1 | effect 1 |
| `hooks/useFleet.ts` | 1 | 1 | effect 1 |
| `hooks/useInstalledVersions.ts` | 1 | 1 | assign 1 |
| `hooks/useTimeFormat.tsx` | 1 | 1 | refresh 1 |
| `hooks/useValueSince.ts` | 1 | 1 | purity 1 |
| `hooks/useWS.ts` | 1 | 1 | arg 1 |
| `pages/OnboardingPage.tsx` | 1 | 1 | effect 1 |
| `pages/logBatcher.test.ts` | 1 | 1 | return 1 |
| `pages/map/hooks/useManualMode.ts` | 1 | 1 | effect 1 |
| `pages/map/hooks/useMapDisplayPreference.ts` | 1 | 1 | effect 1 |
| `types/map.ts` | 1 | 1 | arg 1 |

## Deferred warnings and next groups

- **This cluster:** DrawControl **17→1**, useMapEditing **12→7**. The remaining
  DrawControl assignment comes from the existing any-typed DirectSelectWithBoxMode
  export. Accurately typing that mode requires its state/context/library API,
  beyond this callback batch. The seven open-details assignments in useMapEditing
  come from GeoJSON properties and an asserted MowingAreaEdit. Validating them
  needs explicit decisions for IDs, missing/null metadata and malformed values;
  it is not safe to assert a property type merely to silence warnings.
- **Map file/import boundary:** useMapFiles (9), its tests (8), and
  ImportOpenMowerModal (6) share API payload/error/datum typing. A later focused
  batch should inspect actual import contracts before changing parsing.
- **Restart/error handling:** PowerMenu (9), containers (4), useContainerRestart
  (3) share catch-error and container metadata typing. Preserve confirmations,
  restart order, fire-and-forget GUI restart and reconnect state in that batch.
- **Diagnostics:** useFusionGraphDiagnostics (2) and useBTLog (2) expose decoded
  data through any. Runtime validation there must preserve unavailable fields
  and distinguish receipt from physical acquisition; no fabricated freshness.
- **Settings callbacks/formatters:** section callback any types and
  components/utils (6) can be narrowed against actual consumers in a separate
  batch. Settings persistence/YAML partial-failure behavior stays outside this one.

No browser or hardware acceptance is claimed. This is an in-process callback/type
cleanup, with geometry and lifecycle regression coverage; it does not establish
that external GeoJSON or the custom Draw modes are fully validated.
