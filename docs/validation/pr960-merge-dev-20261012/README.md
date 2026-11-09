# PR960 — functional dev/MAVROS merge in an isolated worktree

Owner MowgliNext; worktree `/ros2_ws/src/mowglinext-pr960`, local branch
`pr960-merge-dev`. Source parent `dad71aa1ddb46ed6a875cc5854d17c337a699c43`
(`feat/mavros-refresh`); merged target snapshot
`be927ebbc8d1e18ff02e5283880a0d7959f2cc91` (`origin/dev`). The pre-existing
`git merge origin/dev` was resumed, without reset, blanket ours/theirs selection,
new branch or push. This prepares the local merge commit, not GitHub's PR merge.

## Conflict resolutions

- `useSettingsManager.ts`: retain runtime-confirmed parameter routes from MAVROS,
  including the existing live `/params` payload. Retain dev's display-only dock,
  shell-style and shell-colour exclusions from ROS restart. Geometry changes and
  pending/unavailable routes still require restart. Derive the live-change toast
  from confirmed dirty routes, and keep the display-only toast. Save remains a
  dirty-key patch; dev's legacy-caster materialization/reset logic is preserved.
- EN/FR: merge JSON objects against their common ancestor recursively. No scalar
  translation required choosing between divergent edits. Keep the FCU feature
  builder, six-group parameter assistant, hardware backend and wheel telemetry
  namespaces together with dev's measurement editor and mower-visual namespaces.
  No duplicate JSON keys; both locale trees have4624 key paths and match exactly.
  Only two old generic-artwork toolbar keys disappear, as intentionally removed
  by dev; no code consumers remain after its layered-artwork replacement.
- Automatically merged shared files reviewed: package scripts, HardwareSection,
  SettingsPage and the ROS template. Shared SettingsPage/HardwareSection retain
  the MAVROS switcher/backend-specific field wiring and dev's appearance/geometry
  editors. Review also covers RobotComponentEditor, robot-description parsing,
  model/appearance metadata, search bindings, schema/type assets and launch/URDF.
  Schema/model parity is exercised by the Go suite; URDF/launch by Python tests.

No extra odometry, FusionGraph, backend, firmware or deployed settings correction.
The calibrator files are byte-identical to source HEAD. The template/URDF changes
are the already-existing dev changes, not adjustments to physical robot settings.
The three gitlinks remain MVR811eda29, Universal GNSS383caba3, opennavd6e41a29.
Submodules are intentionally uninitialized in this worktree; their indexed commits
are verified without touching the main worktree's populated submodules.

## Regression and software validation

New `useSettingsManager.merge.test.tsx` runs the actual hook with disconnected
API/notification/backend mocks: eight cases exercise display-only saves for
MAVROS/native/OpenMower, available native/MAVROS routes, pending/unavailable
restart fallback, mixed geometry/live saves and dirty-only payloads. No test
contacts a robot or changes a loaded coefficient.

- Locked offline Yarn installation: PASS, Node22/Yarn1.22, normal project user.
- TypeScript: PASS.
- Focused hook/locale/backend suites:3 files,22 cases PASS (1.19s final run).
- Initial full GUI run:138 files,1099 cases PASS (118.36s), plus10 appearance
  registry node:test cases PASS. Uses `CI=true yarn test --maxWorkers=2`, the
  existing CI worker limit and unchanged20000ms timeout.
- ESLint initially hit904 warnings (zero errors) and exceeded the existing900
  ceiling. Three came from untyped request mocks in the added test. Type those
  mocks and explicitly mark the hook's intentionally launched load promise with
  `void`; behavior and error handling are unchanged. Final lint PASS:900 warnings,
  zero errors, ceiling unchanged. No assertions disabled or timing limits raised.
- `yarn build --outDir /tmp/pr960-web-build`: PASS; prebuild appearance registry
  and TypeScript run normally. Existing large-chunk advisory remains.
- Entire Go API suite:398 top-level tests +122 subtests PASS (2.313s). One existing
  `TestReceiverSnapshotLive` skipped because this is offline, not a live receiver
  acceptance run. No failures. Settings/parity/native/MAVROS/calibration contracts
  are included; no Go/backend code is changed by conflict resolution.
- Python tools+bringup URDF suite:121 cases PASS (2.37s):111 tools and10 URDF cases.
- Both worktree/index `git diff --check` PASS; no unmerged entries or source
  conflict markers. JSON syntax, uniqueness and exact FR/EN key parity PASS.

Final full GUI rerun after the type-only/void cleanup: **138 files,1099 cases
PASS**, zero failed/skipped cases (119.82s), plus10 appearance registry cases
PASS. Both complete runs passed with no timeout or deadline change.
Local results are not GitHub Actions or physical acceptance results.
Playwright/browser visual tests and the full ROS workspace build were not run;
new dev visual behavior has unit/build/parity coverage here, not browser acceptance.
Temporary logs/build/dependencies stay outside version control; logs are
`/tmp/pr960-{install,tsc,focused,vitest,vitest-final,lint,build,python}.log` and
`/tmp/pr960-go-api.json`. No main-worktree files are written or cleaned. No robot,
actuator, firmware, parameter, deployment or production service is accessed.

## Handoff

Local merge commit parents must be the exact source/dev snapshots above. Verify
its file set, preserved gitlinks and clean worktree, then review before any push.
This task does not push to `dev`, `feat/mavros-refresh` or `pr960-merge-dev`.
The original main worktree remains on `feat/mavros-refresh` atdad71aa1 with its
existing local changes. Future publication must be explicitly selected/reviewed;
GitHub CI still has to validate the published merge SHA.
