# Ant Design Modal lifecycle — GUI Batch 13B

Baseline: `fix/gui-warnings-errors`, HEAD
`d53dd581dd5273ffebe85059bccac9525071ee6a`, Ant Design **5.29.3**.
The initial worktree contained only independent MAVROS submodule changes.
No backend, ROS2, firmware, generated files or submodule files are changed.
Nothing is staged, committed or pushed.

## Inventory and migration

The frontend source contains exactly three uses of `destroyOnClose`:

| Component | Modal | Change |
| --- | --- | --- |
| `pages/map/components/NewAreaModal.tsx` | New workarea/navigation/obstacle | `destroyOnHidden` |
| `pages/map/components/EditAreaModal.tsx` | Edit workarea/navigation/obstacle | `destroyOnHidden` |
| `components/RobotComponentEditor.tsx` | IMU yaw calibration | `destroyOnHidden` |

Production changes consist solely of these three property replacements. Existing
`destroyOnHidden` uses in SchedulePage, MeasurementEditor and FirmwareUpdateCard
are unchanged. No `destroyOnClose` remains in production frontend source. A
TypeScript JSX inventory also finds no Modal `visible`, `bodyStyle` or `maskStyle`
properties (the other deprecated aliases checked by installed Ant Design 5.29.3).
Nine deprecated Input/Input.Password addons remain outside this batch.

The [public Modal API](https://ant.design/components/modal/) documents
`destroyOnHidden` as available since 5.25.0. Installed 5.29.3 declares it in
`antd/es/modal/interface.d.ts`; `Modal.js` passes
`destroyOnHidden ?? destroyOnClose` to the same rc-dialog destruction property.
Thus the migration keeps the existing close-animation/unmount mechanism. No
internal API is imported and no dependency is upgraded.

## Ownership and behavior

Area modal values belong to the parent. Form.Items contain controlled fields;
there is no new Form reset or preserve option. Save, Cancel and the NewArea name
input's Enter handler are unchanged. Edit callbacks retain area identity, index,
original type/order and the obstacle shrink choice. These components do not
calculate polygon geometry or persist anything when unmounted.

The IMU hook remains mounted in RobotComponentEditor; only dialog children are
destroyed. Its existing close handler refuses closure while running, the close
button is hidden and mask closure disabled. Start disappears during a pending
request. Results remain available for explicit Retry, Discard or Apply. Retry
clears the result without starting another request; Discard closes without
applying; Apply forwards the hook's existing rounded radians to the form.
No angle conversion, backend interface, timeout, dock calibration behavior or
robot command changes. The tests verify UI request counts, not a new concurrency
lock or cancellation guarantee in the hook's public functions.

## Regression coverage

Existing area tests are extended with Save → close → reopen → Cancel → close →
reopen for all three types, controlled names/order, shrink choice, Enter and
callback counts. They assert the actual modal body is removed after closure and
that removal does not trigger Save, Cancel or change callbacks.

`RobotComponentEditor.calibrationLifecycle.test.tsx` exercises the real IMU hook
and real Ant Design Modal with mocked fetch, API, telemetry and graphics:

- Cancel/reopen does not request calibration or apply settings.
- Pending calibration hides Start/close, ignores Escape and mask clicks, and
  issues one request. A completed result remains until Discard or Apply.
- Discard/reopen does not apply or restart; a failed result offers no Apply.
- Retry alone makes no request; explicit Start retries, and only explicit Apply
  changes `imu_yaw`, retaining the existing four-decimal radian rounding.
- HTTP rejection is reported; Cancel/reopen neither retries nor applies.

Console warn/error spies call through to the original functions and assert that
`[antd: Modal]` warnings mentioning `destroyOnClose` are absent. No global warning
suppression is added.

jsdom does not finish CSS animations. Area lifecycle tests use the public
ConfigProvider `motion: false` option only in tests. IMU tests keep motion enabled,
wait for the closing animation class and dispatch its completion event before
checking destruction. No arbitrary sleep or production animation change is used.
These are DOM/software regressions, not browser focus-trap/animation timing or
physical calibration evidence. No real robot request is sent.

## Validation

Focused final run: **57 tests passed across four files**, using
`vitest run` with the two area suites, the existing RobotComponentEditor suite
and the new calibration lifecycle suite (`--pool=threads --maxWorkers=2`).
Full suite (`yarn test --pool=threads --maxWorkers=2`): **1,887 Vitest tests
passed in 181 files**, in 205.80 seconds, plus the separate appearance registry
test. Baseline: 1,877 tests; ten new lifecycle regressions. No failed or skipped
Vitest tests were reported. The final full log also contains no
`Modal.destroyOnClose` deprecation warning.

The consolidated validation uses Vitest's thread pool. Earlier fork-pool
launches were interrupted after reporting no completion/progress; their cause
was not isolated, and they are not counted as passing runs. No repository test
configuration or timeouts were changed.

ESLint JSON inventories remain **189 warnings → 189 warnings**, **zero errors**.
All modified/new source and test files have zero ESLint warnings or errors.

| Rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 78 | 78 |
| `@typescript-eslint/no-unsafe-assignment` | 35 | 35 |
| `@typescript-eslint/no-unsafe-member-access` | 29 | 29 |
| `@typescript-eslint/no-unsafe-argument` | 19 | 19 |
| `react-refresh/only-export-components` | 11 | 11 |
| `react-hooks/set-state-in-effect` | 8 | 8 |
| `@typescript-eslint/no-unsafe-call` | 4 | 4 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 |
| `react-hooks/purity` | 2 | 2 |
| **Total** | **189** | **189** |

TypeScript (`yarn tsc --noEmit`) and frontend build
(`yarn build --outDir /tmp/gui-batch13b-dist`) pass. The existing large-chunk build
warning remains, as do jsdom's existing getComputedStyle pseudo-element notices
in the full test log. The final source inventory finds zero deprecated Modal aliases;
`git diff --check` passes.

The two preexisting Playwright failures documented in the
[Batch 13 report](gui-batch13-antd-inputnumber-migration.md) (MAVROS backend-label
assertion and absent LiDAR drag target) are not retested or claimed fixed here.
No new browser/e2e acceptance result is claimed. Backend/Go tests are not run
because no backend code changes. The IMU fetch boundary retains its existing
unchecked response contract; improving it is outside this property-only batch.
