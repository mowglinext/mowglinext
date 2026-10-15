# GUI stabilization audit and Lot 21 — Power-menu Playwright regressions

## Baseline and evidence

Branch `fix/gui-warnings-errors`, HEAD `3f94bc6b` (Lot 20 committed before any
edit). Index empty; the only prior worktree change was MAVROS. No source,
backend, ROS, submodule, dependency or workflow modification in this lot.
Baseline: 105 ESLint warnings, zero errors, 2,073 Vitest tests.

The referenced CI runs were inspected with read-only GitHub Actions metadata
and failed-step logs:

- [38699903510](https://github.com/mowglinext/mowglinext/actions/runs/38699903510),
  commit `16add50b`: Go passed; the GUI job failed at the browser step. The power
  response-loss assertion failed on both attempts; battery Advanced was flaky
  (passed on retry). Final browser summary: 1 failed, 1 flaky, 7 passed.
- [38701375570](https://github.com/mowglinext/mowglinext/actions/runs/38701375570),
  commit `2dae8ee1`: Go passed; browser summary 1 failed, 8 passed. The remaining
  failure is the same response-loss assertion.

Initial local reproduction on HEAD: the unchanged CI browser group had 1 failed,
8 passed. A separate eight-run repetition of battery Advanced produced **3 failed,
5 passed**. Error contexts and traces show actual top-right Ant Design notices
containing `Failed to load settings / Invalid settings payload` intercepting
pointer events. This is reproduced evidence, not an inferred timing fix.

## Contracts and fixes

### 1. A lost reboot response is not an explicit server rejection

Lot 17 (`16add50b`) introduced the deliberate distinction in PowerMenu:
HTTP rejection reports failure and stops reconnect observation; lost transport
or acknowledgement reports unknown outcome and preserves reconnect observation.
The old E2E still aborted the request while expecting `Restart failed` and a
closed overlay. Its expectation became stale at that commit. Production already
matches the intended contract; it is not changed to satisfy the old test.

The replacement browser cases distinguish:

- HTTP 500 with a textual backend refusal: error notification includes that text,
  the reconnect overlay closes, no reconnect GET is scheduled and exactly one
  confirmed reboot POST occurs.
- Connection reset: unknown-outcome warning, no failure or success notification,
  reconnect overlay retained and two 15-second availability observations, with
  exactly one confirmed reboot POST. Availability is not reboot identity.

A request observer also verifies that these paths do not introduce another power,
container, mower service or YAML-persistence command. Read-only DB config POSTs
are deliberately separate. Existing confirmation, Cancel, desktop/mobile,
shutdown and GNSS layout tests are retained. Browser clock control exercises the
existing intervals; no new sleeps, timeout increase, retry or forced click.

### 2. A nominal mock must not accidentally describe a settings failure

`DEFAULT_REST['/api/settings']` returned `{}`. The loader requires a `settings`
envelope before decoding that source; its error notification was therefore
expected for this fixture and could cover the battery dropdown. The fixture
predates the stricter loader: history points to `a553fd7a` for the mock default
and `60d7df0c` (Lot 10) for strict payload rejection. These establish the contract
mismatch's introduction, not the first ever occurrence of a hover failure.

The neutral fixture now explicitly returns `{settings: {}}`. YAML remains its
independent `{}` document. No component z-index, notification placement,
notification visibility, production validation or save behavior changes. Battery
Advanced must open through normal hover and confirmation, with only the intended
shutdown POST. A new deliberately malformed `/settings: {}` scenario still
requires a visible error and no dangerous/persistence write; notifications and
console output are not globally hidden.

This fixes the demonstrated fixture-induced CI obstruction. It does **not**
claim that real notifications can never cover the header. Actual load errors
remain visible and may require dismissal; changing notification ergonomics is
not silently included in this test-contract correction.

The valid empty envelope models a nominal client response. It does not repair
the real Go empty-envelope omission discussed below: that is a separate active
contract issue, preserved by the negative browser regression.

## Separate anomaly register — no fixes mixed into Lot 21

| Item | Evidence / status | Separate next verification |
| --- | --- | --- |
| 3. Invalid settings payload | Confirmed producer/consumer edge: GetSettingsResponse.Settings has omitempty, and GetSettings returns an empty map when the shell file is absent/empty. Go can serialize `{}`; useSettings requires the envelope. This reproduces one cause, not necessarily every robot loading report. | Capture the failing deployment's exact HTTP status/body and source availability. Add Go HTTP and frontend regressions for absent/empty shell, valid YAML fallback, malformed nonempty data and actual read failures. Choose the explicit empty-source contract without accepting arbitrary malformed data. |
| 4. GNSS fix display disagreement | Reported on robot, not localized to a producer here. Active path: decoded GNSS → useGnssStatus/gnssTelemetry → gpsStatus → individual consumers. Some old shared E2E scenarios still use numeric GNSS constants from an earlier enum contract (`rtk_mode:2` is FLOAT now, and `fix_type:4` is DGPS). That fixture observation is not evidence that a live producer sends wrong values. | Record raw typed status and diagnostic fallback together, exact enum definitions, capability/value masks, fix_valid, producer/publication identity and the affected view. Replay before changing classification or precedence. Preserve explicit negative authority and unsupported values. |
| 5. Global critical diagnostic | Health hero in DiagnosticsPage is driven by emergency, stopped containers, absent valid GNSS, CPU heat and battery; diagnostic alerts themselves feed its warning branch. Missing horizontal accuracy with a valid fix is already a warning, not by itself critical. Do not attribute the verdict to the unplugged LiDAR without tracing the actual inputs. | Use mocked enabled/disabled/unsupported/unavailable sensor cases and capture the exact verdict contributors on the reported deployment. Distinguish unknown availability from failed required hardware. No lowering thresholds or safety states to clear a badge. |
| 6. Map replaced / dock not written | Confirmed source-level partial backend operation, already documented and mocked in Lot 16. openmower_import applies replacement before dock; later dock rejection can yield HTTP 500 after map mutation. | Separate provider-fake regression with map A/dock A → map B, forcing dock failure after successful replacement. Require transactionality or an explicit partial-success contract. No frontend rollback claim. |
| 7. GeoJSON inner rings lost on Save | Confirmed save-path risk: local import preserves polygon rings, but useMapFiles.handleSaveMap converts coordinates[0] for areas/obstacles. See the existing Lot 16 limitations. | Offline round-trip test with a polygon hole through import → Save → reload. Decide how holes map to backend obstacle/area models, or explicitly prevent an unsupported destructive save. Do not silently flatten holes. |

References for follow-up: `hooks/useSettings.ts`, `pkg/api/types.go`,
`pkg/api/settings.go`; `utils/gpsStatus.ts`, `hooks/useGnssStatus.ts`,
`utils/gnssTelemetry.ts`; `pages/DiagnosticsPage.tsx`;
[Lot 16 report](gui-lot16-map-import-types.md). None of these production files are
changed. No physical GNSS accuracy, sensor presence or successful robot action is
claimed. A future capture must identify the deployed commit, firmware, receiver
revision and submodule baseline; the present source audit cannot establish them.

Additional console finding during the broader smoke run: React reported duplicate
`Status unknown` keys in some diagnostic scenarios. The root is not localized;
recorded separately rather than changing grouping or severity in this lot.

## Validation

All REST and WebSocket robot traffic is mocked. Browser runs use a local Vite
server, Chromium, two workers unless noted, no test retries. The initial sandbox
could not reach GitHub/loopback; read-only CI inspection and local browser runs
used the permitted execution mode. No real host/container/robot command ran.

- CI browser group after correction: **11 passed** (system-power and blackbox),
  22.3 seconds. Two additional browser cases versus the old nine-test group.
- Ten repetitions of the four critical regressions: **40/40 passed**, 48.5 s.
- Broader shared-fixture impact check: **79 passed, 1 failed** across 80 cases
  (72 page/scenario cases and eight text/numeric addon cases). The failure is
  the unmodified standalone numeric modal fixture at 390px: measured input width
  25.94px versus expected >30px. That case does not call installMockBackend or
  load the application, so neither modified code path participates. No threshold
  is loosened. The pages matrix and text addon/save cases passed.
- Independent numeric-modal characterization: **4/4 failed**, unchanged fixture
  and test. These attempts measured the fixed group at about 249–257px versus
  its requested 280px. Trace snapshots still include ant-zoom-appear-active;
  measurement during entrance animation is therefore a demonstrated harness
  issue. This test originated in `d53dd581` (Lot 13). Settled fluid-field width
  still needs a separate check before concluding whether any actual layout
  change is required. Do not lower dimensions or call it a new Lot 21 regression;
  synchronize on the modal lifecycle in a separate focused follow-up.
- Full `yarn test --pool=threads --maxWorkers=2`: **2,073 passed**, 188 files,
  plus the appearance registry test; 239.00 seconds.
- `yarn tsc --noEmit`: passed (production tsconfig includes src, not E2E files;
  Playwright executes/transforms the browser tests separately).
- `yarn build --outDir /tmp/gui-lot21-dist`: passed.
- `yarn lint` and global JSON inventory: **105 → 105 warnings**, zero errors;
  src is unchanged. Existing Ant Design v5/React 19 compatibility, duplicate-key,
  jsdom pseudo-element and build chunk notices are not suppressed or claimed fixed.
- `git diff --check` passed, including the new report; index remains empty.

The extra numeric fixture failure is not relabelled as one of the two historical
Playwright failures (MAVROS label and LiDAR drag target) from earlier batch reports.
Those two cases were not rerun and are not claimed resolved. No hosted GitHub CI
rerun was launched: new results are local and require review before commit/push.

## Review scope

Only `tests/e2e/system-power.spec.ts`, `tests/e2e/mock/mockBackend.ts` and this
report change. No production rendering or operational logic changes. Existing
ignored screenshots remain runner output; logs/traces/review diff are in `/tmp`.
No staging, commit, push, reset, stash, merge or rebase. MAVROS remains untouched.
