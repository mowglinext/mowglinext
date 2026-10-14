# ESLint warning reduction and type integrity — Batch 8

Baseline: `fix/gui-warnings-errors`, `c2763103ff7b3cc1beb1d8044f96f7a4b489f5c2`. The initial worktree had only the existing local MAVROS submodule modification. No preceding batch changes were mixed into this batch.

## Result

**356 → 294 warnings (−62), zero errors before and after.** The selected drive-tuning cluster is now warning-free. No rule, warning ceiling, ignored path, generated source or suppression was changed. Raw ESLint JSON and validation logs remain under `/tmp/gui-batch8-*`, outside version control.

## Inventory by rule

| ESLint rule | Before | After | Reduction |
| --- | ---: | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 130 | 120 | 10 |
| `@typescript-eslint/no-unsafe-argument` | 36 | 35 | 1 |
| `@typescript-eslint/no-unsafe-assignment` | 83 | 57 | 26 |
| `@typescript-eslint/no-unsafe-call` | 6 | 6 | 0 |
| `@typescript-eslint/no-unsafe-member-access` | 70 | 48 | 22 |
| `@typescript-eslint/no-unsafe-return` | 9 | 6 | 3 |
| `react-hooks/purity` | 2 | 2 | 0 |
| `react-hooks/set-state-in-effect` | 9 | 9 | 0 |
| `react-refresh/only-export-components` | 11 | 11 | 0 |

## Inventory by functional subsystem

| Group | Before | After |
| --- | ---: | ---: |
| App shell and other hooks/components | 38 | 38 |
| Diagnostics and logs | 5 | 5 |
| Drive tuning | 62 | 0 |
| GNSS configuration | 46 | 46 |
| Maps and navigation | 116 | 116 |
| Robot actions and restart | 20 | 20 |
| Settings and geometry | 69 | 69 |

## Inventory by runtime correctness risk

| Group | Before | After |
| --- | ---: | ---: |
| App/display/API typing risk | 25 | 25 |
| Command/configuration typing risk | 185 | 125 |
| Development hot-reload risk | 11 | 11 |
| Diagnostic data typing risk | 4 | 4 |
| Map geometry/import typing risk | 74 | 74 |
| React lifecycle/render risk | 11 | 11 |
| Test fixtures (no direct runtime path) | 46 | 44 |

Risk groups describe the affected data flow based on subsystem/rule, not proven defects or measured hardware safety. Type warnings in tests are listed separately. A command/configuration warning deserves producer and callback review; a refresh/export warning is primarily a development concern.

## Generated versus maintained code

All **356 before / 294 after** reported warnings are in maintained code, including test fixtures. Generated `src/types/ros.generated.ts` is excluded by the existing ESLint configuration. Generated `src/api/Api.ts` has its existing generator-owned blanket suppression. Other generated files report zero warnings. This is a reported-count inventory, not an audit of excluded generated code. None was manually edited.

## Inventory by source file

Every source file with a warning in either run appears below. Rule counts are `before → after`; files with zero warnings in both runs are omitted.

| File (`gui/web/`) | Subsystem | Before | After | Rule breakdown |
| --- | --- | ---: | ---: | --- |
| `src/components/AppShell.tsx` | App shell and other hooks/components | 2 | 2 | `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-member-access` 1→1 |
| `src/components/BTStateGraph.tsx` | Diagnostics and logs | 1 | 1 | `react-refresh/only-export-components` 1→1 |
| `src/components/DrawControl.tsx` | Maps and navigation | 17 | 17 | `@typescript-eslint/no-explicit-any` 10→10, `@typescript-eslint/no-unsafe-argument` 6→6, `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/components/MowerActions.tsx` | Robot actions and restart | 4 | 4 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-argument` 1→1, `react-refresh/only-export-components` 1→1 |
| `src/components/PowerMenu.tsx` | Robot actions and restart | 9 | 9 | `@typescript-eslint/no-explicit-any` 3→3, `@typescript-eslint/no-unsafe-assignment` 3→3, `@typescript-eslint/no-unsafe-member-access` 3→3 |
| `src/components/RobotComponentEditor.tsx` | Settings and geometry | 16 | 16 | `@typescript-eslint/no-explicit-any` 3→3, `@typescript-eslint/no-unsafe-argument` 4→4, `@typescript-eslint/no-unsafe-assignment` 9→9 |
| `src/components/YearOfLawn.tsx` | App shell and other hooks/components | 1 | 1 | `react-hooks/purity` 1→1 |
| `src/components/dashboard/DigEscalationBanner.test.tsx` | App shell and other hooks/components | 1 | 1 | `@typescript-eslint/no-unsafe-return` 1→1 |
| `src/components/settings/AdvancedSection.tsx` | Settings and geometry | 11 | 11 | `@typescript-eslint/no-explicit-any` 7→7, `@typescript-eslint/no-unsafe-argument` 2→2, `@typescript-eslint/no-unsafe-member-access` 2→2 |
| `src/components/settings/BatterySection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/DockingSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/DriveMotorSection.test.tsx` | Drive tuning | 2 | 0 | `@typescript-eslint/no-explicit-any` 1→0, `@typescript-eslint/no-unsafe-assignment` 1→0 |
| `src/components/settings/DriveMotorSection.tsx` | Drive tuning | 56 | 0 | `@typescript-eslint/no-explicit-any` 7→0, `@typescript-eslint/no-unsafe-argument` 1→0, `@typescript-eslint/no-unsafe-assignment` 25→0, `@typescript-eslint/no-unsafe-member-access` 22→0, `@typescript-eslint/no-unsafe-return` 1→0 |
| `src/components/settings/GnssReceiverActionsCard.tsx` | GNSS configuration | 20 | 20 | `@typescript-eslint/no-explicit-any` 8→8, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-call` 2→2, `@typescript-eslint/no-unsafe-member-access` 7→7, `@typescript-eslint/no-unsafe-return` 2→2 |
| `src/components/settings/GnssSerialDeviceConfigField.test.tsx` | GNSS configuration | 1 | 1 | `@typescript-eslint/no-unsafe-return` 1→1 |
| `src/components/settings/HardwareSection.tsx` | Settings and geometry | 2 | 2 | `@typescript-eslint/no-explicit-any` 2→2 |
| `src/components/settings/LedsSection.test.tsx` | Settings and geometry | 2 | 2 | `@typescript-eslint/no-explicit-any` 2→2 |
| `src/components/settings/LedsSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/LocalizationSection.tsx` | Settings and geometry | 2 | 2 | `@typescript-eslint/no-explicit-any` 2→2 |
| `src/components/settings/MowingSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/MqttSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/NavigationSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/NtripSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/ObstaclesSection.tsx` | Settings and geometry | 3 | 3 | `@typescript-eslint/no-explicit-any` 1→1, `react-refresh/only-export-components` 2→2 |
| `src/components/settings/PositioningSection.tsx` | GNSS configuration | 6 | 6 | `@typescript-eslint/no-explicit-any` 4→4, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-member-access` 1→1 |
| `src/components/settings/RainSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/SafetySection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/SensorsSection.tsx` | Settings and geometry | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/UniversalGnssAdvancedSettings.test.tsx` | GNSS configuration | 2 | 2 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/components/settings/UniversalGnssAdvancedSettings.tsx` | GNSS configuration | 1 | 1 | `@typescript-eslint/no-explicit-any` 1→1 |
| `src/components/settings/UpdateChangelog.tsx` | Settings and geometry | 2 | 2 | `react-refresh/only-export-components` 2→2 |
| `src/components/settings/gnssConfig.ts` | GNSS configuration | 4 | 4 | `@typescript-eslint/no-explicit-any` 3→3, `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/components/utils.tsx` | App shell and other hooks/components | 6 | 6 | `@typescript-eslint/no-explicit-any` 4→4, `@typescript-eslint/no-unsafe-assignment` 2→2 |
| `src/hooks/useBTLog.ts` | Diagnostics and logs | 2 | 2 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/hooks/useCalibrationStatus.ts` | App shell and other hooks/components | 1 | 1 | `react-hooks/set-state-in-effect` 1→1 |
| `src/hooks/useContainerRestart.ts` | Robot actions and restart | 3 | 3 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-member-access` 1→1 |
| `src/hooks/useDriveTuning.ts` | Drive tuning | 4 | 0 | `@typescript-eslint/no-explicit-any` 2→0, `@typescript-eslint/no-unsafe-return` 2→0 |
| `src/hooks/useFleet.ts` | App shell and other hooks/components | 1 | 1 | `react-hooks/set-state-in-effect` 1→1 |
| `src/hooks/useFusionGraphDiagnostics.ts` | Diagnostics and logs | 2 | 2 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/hooks/useGnssRuntimeConfig.ts` | GNSS configuration | 6 | 6 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-argument` 1→1, `@typescript-eslint/no-unsafe-member-access` 3→3 |
| `src/hooks/useIOSInstallPrompt.ts` | App shell and other hooks/components | 2 | 2 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-member-access` 1→1 |
| `src/hooks/useImuYawCalibration.ts` | App shell and other hooks/components | 5 | 5 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 2→2, `@typescript-eslint/no-unsafe-member-access` 2→2 |
| `src/hooks/useInstalledVersions.ts` | App shell and other hooks/components | 1 | 1 | `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/hooks/useNotificationCenter.tsx` | App shell and other hooks/components | 3 | 3 | `react-hooks/set-state-in-effect` 1→1, `react-refresh/only-export-components` 2→2 |
| `src/hooks/useSettings.ts` | Settings and geometry | 16 | 16 | `@typescript-eslint/no-explicit-any` 9→9, `@typescript-eslint/no-unsafe-argument` 1→1, `@typescript-eslint/no-unsafe-assignment` 5→5, `@typescript-eslint/no-unsafe-member-access` 1→1 |
| `src/hooks/useTimeFormat.tsx` | App shell and other hooks/components | 1 | 1 | `react-refresh/only-export-components` 1→1 |
| `src/hooks/useValueSince.ts` | App shell and other hooks/components | 1 | 1 | `react-hooks/purity` 1→1 |
| `src/hooks/useWS.ts` | App shell and other hooks/components | 1 | 1 | `@typescript-eslint/no-unsafe-argument` 1→1 |
| `src/pages/FleetPage.tsx` | App shell and other hooks/components | 3 | 3 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-argument` 1→1, `@typescript-eslint/no-unsafe-member-access` 1→1 |
| `src/pages/MapPage.tsx` | Maps and navigation | 5 | 5 | `@typescript-eslint/no-unsafe-argument` 3→3, `@typescript-eslint/no-unsafe-assignment` 2→2 |
| `src/pages/OnboardingPage.tsx` | App shell and other hooks/components | 1 | 1 | `react-hooks/set-state-in-effect` 1→1 |
| `src/pages/SchedulePage.tsx` | App shell and other hooks/components | 5 | 5 | `@typescript-eslint/no-unsafe-assignment` 2→2, `@typescript-eslint/no-unsafe-member-access` 3→3 |
| `src/pages/logBatcher.test.ts` | App shell and other hooks/components | 1 | 1 | `@typescript-eslint/no-unsafe-return` 1→1 |
| `src/pages/map/components/ImportOpenMowerModal.tsx` | Maps and navigation | 6 | 6 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-argument` 2→2, `@typescript-eslint/no-unsafe-member-access` 2→2 |
| `src/pages/map/components/MapToolbar.test.tsx` | Maps and navigation | 3 | 3 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-assignment` 1→1 |
| `src/pages/map/components/MapToolbarMobile.test.tsx` | Maps and navigation | 4 | 4 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-return` 1→1 |
| `src/pages/map/components/TrackedObstaclesPanel.tsx` | Maps and navigation | 8 | 8 | `@typescript-eslint/no-explicit-any` 3→3, `@typescript-eslint/no-unsafe-argument` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-member-access` 3→3 |
| `src/pages/map/hooks/useManualMode.test.ts` | Maps and navigation | 2 | 2 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-argument` 1→1 |
| `src/pages/map/hooks/useManualMode.ts` | Maps and navigation | 1 | 1 | `react-hooks/set-state-in-effect` 1→1 |
| `src/pages/map/hooks/useMapBearing.ts` | Maps and navigation | 4 | 4 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-member-access` 1→1, `react-hooks/set-state-in-effect` 1→1 |
| `src/pages/map/hooks/useMapEditHistory.test.ts` | Maps and navigation | 4 | 4 | `@typescript-eslint/no-unsafe-assignment` 3→3, `@typescript-eslint/no-unsafe-call` 1→1 |
| `src/pages/map/hooks/useMapEditing.ts` | Maps and navigation | 12 | 12 | `@typescript-eslint/no-explicit-any` 3→3, `@typescript-eslint/no-unsafe-argument` 2→2, `@typescript-eslint/no-unsafe-assignment` 7→7 |
| `src/pages/map/hooks/useMapFiles.test.ts` | Maps and navigation | 8 | 8 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-assignment` 2→2, `@typescript-eslint/no-unsafe-member-access` 5→5 |
| `src/pages/map/hooks/useMapFiles.ts` | Maps and navigation | 9 | 9 | `@typescript-eslint/no-explicit-any` 3→3, `@typescript-eslint/no-unsafe-argument` 1→1, `@typescript-eslint/no-unsafe-assignment` 2→2, `@typescript-eslint/no-unsafe-member-access` 3→3 |
| `src/pages/map/hooks/useMapOffset.test.ts` | Maps and navigation | 12 | 12 | `@typescript-eslint/no-explicit-any` 6→6, `@typescript-eslint/no-unsafe-argument` 6→6 |
| `src/pages/map/hooks/useMapOffset.ts` | Maps and navigation | 7 | 7 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-assignment` 2→2, `@typescript-eslint/no-unsafe-member-access` 2→2, `react-hooks/set-state-in-effect` 1→1 |
| `src/pages/map/hooks/useMapStreams.ts` | Maps and navigation | 9 | 9 | `@typescript-eslint/no-explicit-any` 7→7, `react-hooks/set-state-in-effect` 2→2 |
| `src/pages/map/hooks/useResetMowingProgress.test.ts` | Maps and navigation | 4 | 4 | `@typescript-eslint/no-unsafe-call` 2→2, `@typescript-eslint/no-unsafe-member-access` 2→2 |
| `src/theme/ThemeContext.tsx` | App shell and other hooks/components | 2 | 2 | `react-refresh/only-export-components` 2→2 |
| `src/types/map.ts` | Maps and navigation | 1 | 1 | `@typescript-eslint/no-unsafe-argument` 1→1 |
| `src/utils/containers.ts` | Robot actions and restart | 4 | 4 | `@typescript-eslint/no-explicit-any` 1→1, `@typescript-eslint/no-unsafe-call` 1→1, `@typescript-eslint/no-unsafe-member-access` 2→2 |
| `src/utils/datumGps.ts` | GNSS configuration | 6 | 6 | `@typescript-eslint/no-explicit-any` 2→2, `@typescript-eslint/no-unsafe-argument` 1→1, `@typescript-eslint/no-unsafe-assignment` 1→1, `@typescript-eslint/no-unsafe-member-access` 2→2 |
| `src/utils/settingsSave.ts` | Settings and geometry | 5 | 5 | `@typescript-eslint/no-explicit-any` 4→4, `@typescript-eslint/no-unsafe-assignment` 1→1 |

## Cohesive selection and implementation

The largest actionable cluster was `DriveMotorSection.tsx` (56), `useDriveTuning.ts` (4), and `DriveMotorSection.test.tsx` (2). Form values inferred as `any` propagated through request construction and callbacks. Error handling read unchecked nested fields. Request generics and TypeScript report interfaces did not validate decoded API data.

The panel now uses precise FF/PID form types, numeric/null field callbacks, numeric persisted-value callbacks, and the existing hardware-backend union. Watch values from the third-party form boundary are treated as unknown and only explicit `true` authorizes the undock switch. An API error body cannot masquerade as a form-validation rejection. Existing backend error extraction and translations replace unchecked nested reads.

`driveTuningContract.ts` validates unknown status, report, start and rollback bodies against the current Go producer in `gui/pkg/api/drive_tuning.go`. Known run/validation states, field shapes, finite measurements, integer counters, booleans, strings, nested diagnostics, arrays and parameter maps are checked. Required job identities/report paths are nonempty. A start acknowledgement must be running and match the requested FF/PID mode. A rollback body with `success=false` is a failure, even on HTTP success.

Go nil non-omitempty report slices/maps legitimately serialize as null; only those specific collections normalize to empty. Missing mandatory collections and malformed optional fields reject the report. Absent optional measurements/flags remain absent, explicit negatives remain false, and zero remains zero. Extra fields not consumed by the GUI may be carried without implying a capability. Display labels such as trial quality/phase remain strings to preserve the producer’s extensible labels.

Outgoing FF/PID payloads retain the existing wire keys and values. Finite measurements, integer passes, booleans and the existing left/right turn enum are checked before dispatch. Invalid form values cannot be serialized to JSON null and silently trigger backend defaults. Physical ranges, motion safety checks and backend-specific authorization remain owned by the unchanged server/ROS consumers.

The hook preserves its three-second polling interval and stable callbacks. Request generations ignore older overlapping responses and completions after cleanup. Failed/malformed status responses clear current status and expose the error; failed/malformed report loads clear the previously loaded report, release loading state and reject to the caller. A valid report or rollback is returned only after decoding; the existing panel matching/report selection continues to limit MAVROS persisted values to odometry.

## Affected files

- `src/components/settings/DriveMotorSection.tsx` and `DriveMotorSection.test.tsx`.
- `src/hooks/useDriveTuning.ts` and new `useDriveTuning.test.tsx`.
- New `src/utils/driveTuningContract.ts` and `driveTuningContract.test.ts`.
- New `src/test/driveTuningFixtures.ts` (typed, shared producer-shaped fixtures).
- `src/utils/driveTuningI18n.ts` and `src/i18n/locales/{en,fr}.json` for validation error translations.
- This document. No Go, ROS, firmware, generated source, GNSS/physical telemetry, parameter authorization, YAML save or WebSocket code changed. Local MAVROS modifications remain untouched.

## Regression evidence

- Focused contract/hook/component/locale tests: **51 passed across 4 files**.
- Full `yarn test`: appearance registry passed; **1,573 Vitest tests passed across 167 files** (baseline 1,535).
- ESLint JSON: **356 → 294 warnings, zero errors**; all selected and newly added code has zero warnings.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-batch8-dist`: passed; existing large-chunk advisory remains.
- `git diff --check`: passed.
- Go tests were not rerun: this batch changes no Go code or server contract. The preceding full Go validation remains the baseline.

Tests cover malformed bodies, missing optionals, nullable Go collections, zero/false, nonfinite/wrongly typed parameters, successful native FF/PID report shapes, rejected rollback, unconfirmed/wrong-mode starts, request errors, delayed/unmounted responses, polling callback stability, out-of-order completion and report failure after a valid load. Existing full-suite tests cover GNSS masks/source authority, hardware isolation, protected parameter routes and partial YAML failure behavior. These are software tests with mocked APIs; no real motion or hardware acceptance is claimed.

## Deferred work and compatibility limits

The next large clusters are GNSS receiver actions (20 in that component), settings hook (16), geometry editor (16), and drawing/map editing (17/12). Their independent producer, persistence and lifecycle contracts need targeted review before replacing their types. The remaining 294 warnings are retained and visible; unrelated React warnings require ownership/render analysis, and hot-reload warnings do not justify unrelated module refactors.

Validation is anchored to the current Go JSON projection, including legacy reports reserialized through its structs. Older/custom servers that omit required envelope fields or send stringified measurements now fail visibly rather than being implicitly trusted. JavaScript-safe integers are required for counters/exit codes. A server may already have started a job before an invalid acknowledgement is received; the UI reports lack of confirmation, not proof that no motion began. Request order is delivery ordering, not a physical observation identity or freshness guarantee. A network failure can hide completed backend work; no transaction, hardware timestamp or runtime capability guarantee is introduced.

No commit, push, merge, reset or cleanup was performed.
