# React hooks lifecycle and stability investigation

## Baseline and final result

2026-10-14, branch `fix/gui-warnings-errors`, base commit
`97a65997525c0bed98a2beb154160cb3ac499d32`. This continues
[the asynchronous promise safety investigation](async-promise-safety.md).
The working frontend was initially clean; the pre-existing modified
`ros2/src/external/mowglimavros` checkout was left untouched.

Counts were confirmed against the actual working tree with ESLint JSON output.

| Rule | Initial | Final |
| --- | ---: | ---: |
| `react-hooks/exhaustive-deps` | 35 | 0 |
| `react-hooks/refs` | 18 | 0 |
| `react-hooks/set-state-in-effect` | 8 | 9 |
| `react-hooks/purity` | 2 | 2 |
| `react-hooks/preserve-manual-memoization` | 2 | 0 |
| `react-hooks/immutability` | 1 | 0 |
| All targeted hook warnings | 66 | 11 |
| Other warnings | 387 | 380 |
| All warnings | 453 | 391 |
| Errors | 0 | 0 |
| Floating/misused promises | 0 / 0 | 0 / 0 |

58 original hook findings were resolved. Fixing compiler-blocking ref/dependency
issues made three existing synchronization effects newly reportable; none of
those state-update bodies was introduced by this task. Thus the net hook
reduction is 55. Seven incidental typing warnings disappeared from configuration
key-map/error-report extraction. No other warning category increased.

The 387 total-warning target was not forced: the remaining eleven hook findings
need deliberate synchronization/clock review. No lint rule was disabled and no suppression was added. The warning
ceiling, package dependencies, generated source and CI configuration were unchanged.
One existing view-stream dependency suppression was removed after declaring its
true stable resource and primitive datum dependencies.

## Phase 1: individual dependency audit

Line numbers refer to the initial checkout. Each of the 35 findings was inspected
individually against its resource lifetime and cleanup.

| Initial site | Root cause and correction |
| --- | --- |
| `src/components/FlashBoardComponent.tsx:252` | Stable Formily form/API dependencies; error Effect Event reads current notification/translation without re-seeding the form on language changes. |
| `src/components/RobotComponentEditor.tsx:479` | Unnecessary constant SCALE dependency removed; actual coordinate converter remains a dependency. |
| `src/components/settings/BatterySection.tsx:110` | Voltage-position helper moved inside memo; include its actual scalar geometry/range inputs. |
| `src/components/settings/DriveMotorSection.tsx:245` | Backend-message translator memoized against t; alert memo depends on that stable helper. |
| `src/components/settings/NtripSection.tsx:70` | Include the application singleton API in station-fetch callback dependencies. |
| `src/hooks/useBTLog.ts:63` | Use stable start/stop methods; reset the cleared throttle ref so StrictMode replay can schedule its next update. |
| `src/hooks/useCalibrationStatus.ts:67` | Include singleton API; also fixes inferred/manual memoization mismatch. |
| `src/hooks/useConfig.tsx:48` | Explicit mount-time key snapshot for the two fixed-key callers; API dependency and fresh error Effect Event prevent literal-array refetch loops. |
| `src/hooks/useDiagnostics.ts:70` | Use stable start/stop methods; clear the throttle ref on cleanup for StrictMode replay. |
| `src/hooks/useDiagnosticsSnapshot.ts:67` | Include singleton API; polling cadence and request ordering unchanged. |
| `src/hooks/useEnv.tsx:27` | Depend on stable config API namespace; error Effect Event prevents language changes from refetching configuration. |
| `src/hooks/useFirmwareDebugLogs.ts:128` | Include stable start/stop methods; retain cancellation flag, container resolution and stream teardown. |
| `src/hooks/useFusionGraphDiagnostics.ts:82` | Include stable start/stop methods without reconnecting on diagnostic state updates. |
| `src/hooks/useGnssStatus.ts:32` | Include stable start/stop methods; typed-GNSS/diagnostic provenance logic unchanged. |
| `src/hooks/useHighLevelStatus.ts:18` | Include stable start/stop methods; telemetry updates do not reconnect. |
| `src/hooks/useRobotDescription.ts:80` | Include stable start/stop methods; duplicate XML detection remains instance scoped. |
| `src/hooks/useSettings.ts:228` | Stable settings namespace plus fresh error Effect Event; no reload triggered by translations/notification identity. |
| `src/hooks/useSettingsManager.ts:399` | Stable settings namespace plus error Effect Event; preserves dirty draft and one-time initialization. |
| `src/hooks/useSettingsManager.ts:708` | Remove unused hardware.backend dependency; used hardware.parameterRoutes remains reactive. |
| `src/hooks/useSettingsManager.ts:846` | Hoist the static hidden-key Set out of the hook; no unstable per-render Set dependency. |
| `src/hooks/useSettingsSchema.ts:81` | Stable settings namespace plus fresh error Effect Event; preserves parallel schema/value reads and loading ownership. |
| `src/hooks/useWeather.ts:32` | Include singleton API; preserve last-good-value fallback and polling interval. |
| `src/pages/LogsPage.tsx:142` | Initial listing remains mount-only via an Effect Event; reusable callback still refreshes after explicit container actions. Functional selection update avoids stale captured container ID. |
| `src/pages/LogsPage.tsx:150` | Depend on stable start/stop methods and container ID; switching containers cleans the prior stream once. |
| `src/pages/MapPage.tsx:604` | Memoize feature builder against offsets/datum and include it; retain edit-mode guard and live robot features. |
| `src/pages/MapPage.tsx:622` | Memoize label builder against language/t; label effect reacts to that builder without looping on each render. |
| `src/pages/MapPage.tsx:774` | Include i18n.language used by area formatting; no network or command effect. |
| `src/pages/OnboardingPage.tsx:1088` | GPS entry snapshot is a non-reactive Effect Event invoked only on step entry; edits cannot replace the restart comparison baseline. |
| `src/pages/ParametersPage.tsx:205` | Include singleton API in fetch callback; no new parameter writes. |
| `src/pages/ParametersPage.tsx:222` | Include singleton API in commit callback; safety confirmation and coercion unchanged. |
| `src/pages/StatisticsPage.tsx:104` | Include singleton API in fetch callback; preserve concurrent reads and polling. |
| `src/pages/StatisticsPage.tsx:114` | Include singleton API in clear callback; operation still runs only on explicit confirmation. |
| `src/pages/map/hooks/useMapStreams.ts:341` | Include the ref object owning the Mapbox instance; reading its current value remains inside the effect. |
| `src/pages/map/hooks/useMapStreams.ts:427` | Depend on state name, stable joy/recording methods and setters; retain the exact 1200 ms exit debounce. |
| `src/pages/map/hooks/useMapStreams.ts:453` | Cleanup depends on stable methods/cancellers, not telemetry-bearing wrapper objects. View effect now declares stable resources and primitive datum values; removed its prior suppression. |

`useApi` already returns a module singleton. Including its reference (or its
stable namespace) does not introduce a poll/refetch transition in production.
`useWS`, however, returned new methods and an object every render. Its teardown
and start are now memoized; its return object is memoized against actual methods.
Consumers use the exact stable start/stop methods when only stream ownership
matters, avoiding dependence on telemetry objects that change on every sample.

`useEffectEvent` is used for non-reactive observations: fresh error text at
completion, a draft snapshot on GPS-step entry, and the initial container-list
invocation. It does not hide a reactive subscription input. API/form resource
identities, current step, selected container and datum values still control their
proper effects. Notifications/language changes must not overwrite an unsaved
configuration draft or recreate a flashing form.

## Phase 2: all 18 ref findings

| Initial sites | Cause and correction |
| --- | --- |
| `DrawControl.tsx:37,43,45,47,49,51,53` | Edit mode and six callbacks were written into refs during render. A layout effect now publishes committed values before useControl registers its passive listeners. Registration and removal still use the same stored function identities; control creation is unchanged. |
| `useWS.ts:46,48,50` | Three callback refs were render-written. Layout-effect updates keep the callback current without exposing an abandoned render or restarting the stream. |
| `OnboardingPage.tsx:1165,1167,1209` | Calling an adapter factory with the ref-bearing GPS transition handler during render triggered the compiler's ref-argument diagnostic. Construct/invoke the existing adapter inside the actual Next callback. GPS snapshot reads and commands remain event-only. |
| `useLatestThrottle.ts:62,64,69,70,85` | Render-written callback/budget refs plus a render-read throttle ref. A lazy per-mount resource owns the same throttle, with committed callback/budget updates in a layout effect. No timer starts until a push; interval-transition cancellation and trailing latest-value delivery remain unchanged. |

DrawControl additionally owns and cancels its deferred selection-blocking timers
on teardown, so they cannot change the mode of an already removed control.
Existing feature-set/geometry synchronization and its 300 ms timer remain intact.
BT/diagnostics cleanup now nulls cancelled throttle handles, allowing StrictMode
replay to accept and schedule the next sample.

## Phase 3: compiler/state investigation

| Initial finding | Decision |
| --- | --- |
| `BatterySection.tsx:40` memoization | Position helper is local to the memo, with complete range/geometry dependencies; threshold and current-voltage calculations unchanged. |
| `useCalibrationStatus.ts:55` memoization | Include the stable API dependency, matching compiler-inferred resource ownership. |
| `useSectionFocus.ts:9` immutability | DOM scroll mutation is imperative external synchronization, not React data mutation. Use the DOM scrollTo API with explicit instant behavior; only vertical scroll resets, then focus retains preventScroll. |
| `useIsMobile.ts:13` state | UseSyncExternalStore observes the actual media query, with exact listener cleanup and no mount-time effect-state round trip. Browser viewport transitions remain external inputs. |
| `useIOSInstallPrompt.ts:28` state | Detect initial eligibility lazily; dismissal remains event-driven and stored for remounts. This removes the initial false-to-true paint for eligible iOS Safari. |
| Remaining original state/purity findings | Preserve the existing synchronization and clock semantics; rationale below. |

## Unresolved findings and rationale

All eleven are still reported by ESLint, with no suppression:

| Final site | Why retained; separate-review requirement |
| --- | --- |
| `components/YearOfLawn.tsx:63` purity | Date.now defines the civil-day anchor when the existing session/language/time-zone memo recalculates. Freezing it at mount or introducing a ticker changes midnight/day-window behavior. Review an explicit calendar-clock contract with timezone/DST tests. |
| `hooks/useValueSince.ts:14` purity | Observed value transitions currently establish their timestamp during render using a ref. This also masks underlying render-ref debt in the same function. Moving that transition to effects/state changes elapsed-time baselines and transition visibility. Review observation-clock ownership rather than substituting a timer. |
| `hooks/useCalibrationStatus.ts:70` state | Refresh normally updates state after an awaited request. The compiler conservatively follows its catch/update path through the API call. Retain immediate first polling request and existing error behavior; no artificial timeout added. |
| `hooks/useFleet.ts:125` state | Refresh awaits concurrent HTTP reads; similar conservative async/catch diagnostic. Preserve immediate first poll and alive guard. A refresh-ownership review should also distinguish StrictMode generations of outstanding requests. |
| `hooks/useNotificationCenter.tsx:65` state | Host notice refresh reconciles optimistic local read/dismiss state with backend state. Replacing this with direct derived notices can immediately undo acknowledgment/dismissal before the next refresh. Review a versioned optimistic overlay before changing it. |
| `pages/map/hooks/useManualMode.ts:38` state | Immediate telemetry entry latch and 1200 ms sustained-exit debounce protect manual UI/joystick continuity. Do not delay entry or replace the latch with the latest telemetry frame. Requires a separately reviewed state-machine change. |
| `pages/map/hooks/useMapBearing.ts:31` state | External persisted config synchronizes the local bearing draft while pending writes flush on leave. Review precedence between external updates and pending local edits before changing it. |
| `pages/map/hooks/useMapOffset.ts:25` state | Config synchronization affects map/robot coordinate presentation and pending debounce writes. Review draft/config transitions and both map instances together. |
| `pages/OnboardingPage.tsx:1076` state, newly exposed | Existing saved-values synchronization hydrates the wizard draft. Removing it could drop loaded config; making it derived could overwrite edits. Requires explicit saved/draft ownership. |
| `pages/map/hooks/useMapStreams.ts:193` state, newly exposed | Existing image arrival clears the overlapping point-cloud visualization. Preserve raster/point-cloud handover until the overlay-state contract is reviewed. |
| `pages/map/hooks/useMapStreams.ts:387` state, newly exposed | Existing edit-mode transition stops streams and clears path/plan/lidar snapshots, preventing stale overlays and lost user edits. Review the complete map editing/resource transition together before moving those updates. |

## Files modified

All task changes are in `gui/web`; the full inventory follows:

- `docs/async-promise-safety.md`
- `docs/react-hooks-lifecycle-safety.md`
- `src/components/DrawControl.test.tsx`
- `src/components/DrawControl.tsx`
- `src/components/FlashBoardComponent.test.tsx`
- `src/components/FlashBoardComponent.tsx`
- `src/components/RobotComponentEditor.tsx`
- `src/components/settings/BatterySection.tsx`
- `src/components/settings/DriveMotorSection.tsx`
- `src/components/settings/NtripSection.tsx`
- `src/hooks/useBTLog.ts`
- `src/hooks/useBrowserLifecycle.test.tsx`
- `src/hooks/useCalibrationStatus.ts`
- `src/hooks/useConfig.test.tsx`
- `src/hooks/useConfig.tsx`
- `src/hooks/useDiagnostics.ts`
- `src/hooks/useDiagnosticsSnapshot.ts`
- `src/hooks/useEnv.tsx`
- `src/hooks/useFirmwareDebugLogs.ts`
- `src/hooks/useFusionGraphDiagnostics.ts`
- `src/hooks/useGnssStatus.ts`
- `src/hooks/useHighLevelStatus.ts`
- `src/hooks/useIOSInstallPrompt.ts`
- `src/hooks/useIsMobile.ts`
- `src/hooks/useRobotDescription.ts`
- `src/hooks/useSectionFocus.ts`
- `src/hooks/useSettings.ts`
- `src/hooks/useSettingsManager.ts`
- `src/hooks/useSettingsSchema.ts`
- `src/hooks/useWS.test.ts`
- `src/hooks/useWS.ts`
- `src/hooks/useWeather.ts`
- `src/pages/LogsPage.test.tsx`
- `src/pages/LogsPage.tsx`
- `src/pages/MapPage.tsx`
- `src/pages/OnboardingPage.tsx`
- `src/pages/ParametersPage.tsx`
- `src/pages/StatisticsPage.tsx`
- `src/pages/map/hooks/useLatestThrottle.test.ts`
- `src/pages/map/hooks/useLatestThrottle.ts`
- `src/pages/map/hooks/useMapEditing.test.ts`
- `src/pages/map/hooks/useMapEditing.ts`
- `src/pages/map/hooks/useMapStreams.test.tsx`
- `src/pages/map/hooks/useMapStreams.ts`

## Regression tests and validation

Thirteen new tests across the new DrawControl, browser lifecycle, configuration
load and map-stream suites, plus existing useWS/throttle suites, verify:

- StrictMode mount/cleanup/replay and real unmount/remount, with one effective
  Mapbox listener, exact on/off function identity, and no deferred mode change
  after teardown.
- Stable stream/throttle identity and the latest committed callback after rerender.
- No map reconnection on settings-object churn, correct edit/view transitions,
  and unchanged 1200 ms manual-exit debounce without repeated joy starts.
- Pending trailing-frame cancellation on budget transition and unmount.
- Configuration literal-array rerenders and language changes cause no refetch
  or write; a pending failure reports in the newly committed language.
- Media-query listener cleanup, iOS prompt dismissal across remounts, and
  section scrolling/focus only on actual context transitions.

Existing flashing and logs fixtures now use stable API/stream objects matching
the real application. The map-edit fixture includes the real hook's language
field now read by the memo dependency. Assertions were retained; production
contracts were not weakened to accommodate incomplete mocks.

Final validation as the normal project user:

- `yarn lint`: PASS: 391 warnings, zero errors; no floating/misused promises.
- `yarn build`: PASS: TypeScript plus Vite production output. Vite's existing advisory large-chunk warning remains.
- `yarn test`: PASS: 1,152 Vitest tests across 150 files, plus the separate Node appearance-registry suite.
- `git diff --check`: PASS.

Focused checks passed 35 tests across six callback/config/browser/flashing
suites, five browser/map dependency-transition tests, and 18 existing log/map-edit
tests. Temporary JSON/logs remain under `/tmp/gui-hooks-*`, outside version control.

## Release browser checks and limits

Before release, exercise the real Mapbox/WebGL renderer in desktop/mobile and
StrictMode: create/delete/combine/split, vertex dragging through telemetry frames,
read-only selection blocking, map remount, edit/view transitions and datum reload.
Confirm selection is preserved for geometry-only updates and no listener duplication
or stale timer appears after route navigation.

Against a mocked backend, also check dirty settings through language changes,
GPS-step edits and back/forward entry, one expected GPS restart after confirmed
changes, firmware manual target overrides and cancellation, log-container switches,
and manual joystick entry/short telemetry blips/sustained exit. No rendering or
telemetry-only rerender should send motor/mowing commands or configuration writes.

These are software/mocked validations; no robot or field acceptance is claimed.
No actuation payloads, command sequences, safety guards, firmware authority,
parameter coercion, dirty-key writes or AbortController behavior were changed.
The intentional visual changes are current-language error/label text, removal of
the eligible iOS prompt's first-paint flicker, and responsive state directly based
on its media query. Real WebGL/browser checks remain recommended; GPU behavior
is not established by the mocked listener tests.

No commit, push, merge, deployment or pull request was created.
