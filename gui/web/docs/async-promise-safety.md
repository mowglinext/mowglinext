# GUI asynchronous promise safety cleanup

## Baseline and result

Date: 2026-10-14. Branch: `fix/gui-warnings-errors`. Base commit:
`d125e9248874e5120555e0afe754c37a6c764d44`. Findings were measured from the
actual working checkout before edits, using `yarn lint --format json`.
All task changes are within `gui/web`. The pre-existing modified
`ros2/src/external/mowglimavros` checkout was left untouched.

| Findings | Initial | Final |
| --- | ---: | ---: |
| ESLint errors | 0 | 0 |
| All ESLint warnings | 534 | 453 |
| `@typescript-eslint/no-floating-promises` | 33 | 0 |
| `@typescript-eslint/no-misused-promises` | 48 | 0 |
| Other warnings | 453 | 453 |

No lint settings, warning ceiling, dependency versions, generated source,
backend, submodule contents or CI configuration were changed.

## Causes and correction patterns

The complete 81-site audit is listed below. Line numbers identify the initial
checkout; wrappers and imports move the final lines.

The 48 misused promises consisted of 35 synchronous UI callbacks (buttons,
switches, language changes and navigation), six controlled modal OK callbacks,
five promise-aware Popconfirm callbacks, and two DOM file-input listeners.

- `utils/async.ts`: `asyncEvent` invokes its callback synchronously, with the
  same arguments, and observes its result without serializing concurrent calls.
  Both synchronous throws and asynchronous rejections go to an explicit error
  reporter. Failures in reporting itself are logged. It returns `void` for
  synchronous event APIs; it must not wrap APIs that consume a returned promise.
- `hooks/useAsyncHandler.ts`: supplies the event adapter with the existing AntD
  application notification API and translated `asyncButton.errorOccurred`
  title. Existing handler-specific inline errors and toasts still run inside
  their original handlers. They resolve normally after handling the error, so
  the adapter does not duplicate their notifications. Unhandled failures receive
  a toast using `errorMessage(unknown)` plus a console diagnostic.
- Controlled `<Modal>` callbacks retain their own open/loading state and error
  handling. They use the synchronous adapter. Imperative `modal.confirm` promise
  callbacks retain their original return values and cancellation behavior.
- `components/AsyncPopconfirm.tsx` is a typed alias of AntD's original component,
  without casts or a runtime wrapper. AntD's declaration says `onConfirm` returns
  `void`, but its ActionButton actually awaits a returned promise to show loading
  and close after settlement. The alias describes this supported contract.
  Callers retain their existing rejection notifications and returned promises.
- File listeners use `asyncEvent` with their existing supplied notification
  instance and specific restore/import error titles. A rejected `file.text()`
  during backup restore now reports failure before any editor mutation. Temporary
  inputs are removed after change or cancellation, and listeners are one-shot.

The 33 floating promises consisted of eight effect/debounce IIFEs, ten initial
or conditional background fetches, four navigation calls, two NTRIP provider
loads, three schedule updates/loads, one parameter write, one mobile mowing
menu action, one i18n initialization and three test setup/timer calls.

- `observeAsync` observes detached results, including navigation's
  `void | Promise<void>` result. Existing catches continue to update error state
  or show notifications. Its console fallback records unexpected errors escaping
  those handlers, including notification failures. Poll callbacks in the touched
  hooks now use the same observer; intervals and concurrency are unchanged.
- Mobile mowing uses the existing `safeCall` error notification around the
  existing `finally` loading reset. A rejected start is no longer unhandled.
- Weather, statistics and onboarding checks retain their previous fallback
  behavior but now log failures instead of silently ignoring them. A failed
  applied drive-report fetch uses the existing report-failure toast, gated by
  the effect's original cancellation flag, rather than announcing applied values.
- Corridor validation catches only identifiable inline field errors. Unexpected
  validation or save-callback failures reach the notification adapter; invalid
  widths still prevent saving and retain their inline feedback.
- Test setup awaits language initialization. Timer test callbacks use block
  bodies so they do not return Vitest's timer result to React `act`; synchronous
  timer advancement and assertions keep their original timing.

No detached operation was silenced using a new `void`, empty catch, rule
suppression, retry, or success notification on a rejected request.

## Full initial audit

| File (under gui/web) | Floating | Misused | Initial lines |
| --- | ---: | ---: | --- |
| `src/components/AppShell.tsx` | 4 | 2 | 93, 101, 106, 194, 200, 222 |
| `src/components/LanguageSwitcher.tsx` | 0 | 1 | 35 |
| `src/components/MowerStatus.tsx` | 0 | 1 | 143 |
| `src/components/RobotComponentEditor.tsx` | 0 | 1 | 951 |
| `src/components/onboarding/ReadinessStep.tsx` | 1 | 3 | 104, 158, 162, 255 |
| `src/components/settings/DockCalibrationCard.tsx` | 0 | 1 | 54 |
| `src/components/settings/DriveMotorSection.tsx` | 1 | 3 | 177, 443, 581, 701 |
| `src/components/settings/FirmwareParamsCard.tsx` | 0 | 2 | 389, 402 |
| `src/components/settings/GnssReceiverActionsCard.tsx` | 0 | 4 | 276, 285, 301, 310 |
| `src/components/settings/NtripSection.tsx` | 2 | 2 | 104, 123, 163, 187 |
| `src/components/settings/PositioningSection.tsx` | 0 | 1 | 139 |
| `src/hooks/useCalibrationStatus.ts` | 1 | 0 | 69 |
| `src/hooks/useConfig.tsx` | 1 | 0 | 29 |
| `src/hooks/useDiagnosticsSnapshot.ts` | 1 | 0 | 69 |
| `src/hooks/useDriveTuning.ts` | 1 | 0 | 157 |
| `src/hooks/useEnv.tsx` | 1 | 0 | 12 |
| `src/hooks/useFirmwareDebugLogs.ts` | 1 | 0 | 122 |
| `src/hooks/useRosbag.ts` | 2 | 0 | 68, 70 |
| `src/hooks/useSettings.ts` | 1 | 0 | 188 |
| `src/hooks/useSettingsSchema.ts` | 1 | 0 | 57 |
| `src/hooks/useWeather.ts` | 1 | 0 | 31 |
| `src/i18n/index.ts` | 1 | 0 | 17 |
| `src/pages/FleetPage.tsx` | 0 | 9 | 129, 168, 174, 178, 208, 250, 258, 265, 278 |
| `src/pages/LogsPage.tsx` | 1 | 0 | 141 |
| `src/pages/MowgliNextPage.tsx` | 0 | 4 | 410, 429, 742, 804 |
| `src/pages/ParametersPage.tsx` | 2 | 0 | 85, 206 |
| `src/pages/SchedulePage.tsx` | 3 | 4 | 82, 136, 238, 272, 296, 417, 520 |
| `src/pages/SettingsPage.tsx` | 0 | 1 | 572 |
| `src/pages/StatisticsPage.tsx` | 1 | 1 | 113, 368 |
| `src/pages/map/components/EditLidarCorridorModal.tsx` | 0 | 1 | 42 |
| `src/pages/map/components/ImportOpenMowerModal.tsx` | 0 | 5 | 113, 145, 146, 157, 158 |
| `src/pages/map/components/MapToolbarMobile.tsx` | 1 | 0 | 283 |
| `src/pages/map/hooks/useManualMode.test.ts` | 2 | 0 | 74, 77 |
| `src/pages/map/hooks/useMapFiles.ts` | 0 | 2 | 289, 409 |
| `src/pages/map/hooks/useMapOffset.ts` | 2 | 0 | 31, 45 |
| `src/test/setup.ts` | 1 | 0 | 8 |

Every source file in that table was modified. Supporting files added:
`src/utils/async.ts`, `src/hooks/useAsyncHandler.ts`,
`src/components/AsyncPopconfirm.tsx`, `src/utils/async.test.ts`,
`src/components/AsyncPopconfirm.test.tsx`, and this report.
Existing test files additionally modified: `src/components/MowerStatus.test.tsx`,
`src/components/settings/DriveMotorSection.test.tsx`,
`src/pages/map/components/MapToolbarMobile.test.tsx`, and
`src/pages/map/hooks/useMapFiles.test.ts`.

## Regression tests

Eleven new tests cover:

- Immediate callback invocation, arguments and concurrent completion order.
- Rejection reporting exactly once, synchronous failures, already-reported
  failures, and an error reporter that itself throws.
- Popconfirm staying open and loading while its action remains pending.
- Rejected rearming and area-start requests producing failure notifications;
  area-start loading resets after failure.
- A failed applied calibration-report read producing no local value acceptance
  or applied-success notification.
- Backup-read rejection producing no editor mutation or success notification,
  plus temporary-input removal on selection and cancellation.

Existing manual-mode latch timing, command sequencing, corridor bounds,
schedule conflicts, firmware-reset freshness/confirmation and map-save identity
coverage were also exercised.

## Validation

Final commands were run as the normal project user from `gui/web`:

- `yarn lint`: PASS; 453 warnings, zero errors; both targeted rules zero.
- `yarn build`: PASS; TypeScript and Vite production bundle.
- `yarn test`: PASS; final counts recorded below.
- `git diff --check`: PASS.

Targeted batches passed: 18 drive-motor/firmware tests, 26 focused
callback/rejection/confirmation tests, and 22 report-fetch/file/confirmation
tests. Full-suite validation also covers the changed schedule, corridor and
manual-mode tests.

The production build retains Vite's advisory warning about chunks over 500 kB;
no bundle-splitting or warning-threshold changes were made.

Full-suite result: 146 Vitest files, 1,139 tests passed; the separate Node appearance-registry test also passed.
Temporary lint JSON and validation logs are under `/tmp/gui-*`, outside version
control. The build's normal dist output is ignored.

## Remaining debt, behavior and review requirements

The 453 remaining warnings are unchanged by rule: 145 explicit-any, 94 unsafe
assignments, 80 unsafe member accesses, 40 unsafe arguments, 11 unsafe returns,
six unsafe calls, 35 hook dependency warnings, 18 hook ref warnings, eight
hook effect-state warnings, two purity, two preserve-memoization, one
immutability, and 11 Fast Refresh warnings. They are typing/hook/component-boundary
debt outside this task. No asynchronous warning was deferred for architecture.

Command payloads, order, guards, confirmation choices, loading ownership,
settings dirty-key writes, manual-control limits, stream subscriptions, existing
AbortController handling and effect dependencies are preserved. This is software
validation against tests/mocks; no robot or field behavior was measured.

Review the changed notification behavior (previously unhandled failures now
produce toasts; background failures now produce diagnostics). Before release,
a browser smoke check against a mocked backend should verify rejected rearm,
start/pause/home and configuration/calibration requests, confirmation cancel
without a request, firmware-reset safeguards, loading recovery, and mobile menu
behavior. These checks can be performed without actuation. Physical acceptance
is outside this frontend cleanup and is not claimed by these results.

No commit, push, merge, deployment or pull request was created.
