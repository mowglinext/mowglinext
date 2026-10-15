# Settings loading envelope — GUI Lot 22

## Baseline and scope

Branch: `fix/gui-warnings-errors`, HEAD `36a4516d`.
Lots 21 (`5ddf1d1f`) and 21 bis (`36a4516d`) were committed before this work.
The index was empty and no GUI changes were pending. The independent local
MAVROS modification was left untouched. Initial ESLint: **105 warnings, zero
errors**; frontend test baseline: **2,073 tests**.

This is a read-contract fix. The only production change removes `omitempty`
from `GetSettingsResponse.Settings` in `gui/pkg/api/types.go`.
Both successful branches of `GetSettings` already initialize non-nil maps;
no handler rewrite or frontend relaxation is necessary.

## Reproduction and canonical JSON

`GET /api/settings` reads the legacy shell configuration. A configured path
whose file does not exist is intentionally accepted as an empty configuration.
An empty file or comments-only file also produces an empty final map.

Previously, `json:"settings,omitempty"` removed the empty map entirely:

```json
{}
```

`useSettings` requires an own JSON envelope containing `settings`; it correctly
reported `Invalid settings payload` for that response. Existing Go tests decoded
the body into `GetSettingsResponse`, where an absent map and an empty map both
passed `assert.Empty`, hiding the wire-contract defect.

Before changing production code, `TestGetSettings_EmptyHTTPEnvelope` was added
and run. All three subtests failed on the actual HTTP body: expected
`{"settings":{}}`, received `{}`, with HTTP 200. After removing `omitempty`,
the same tests pass. The canonical empty success is now:

```json
{"settings":{}}
```

Nonempty responses keep their existing shape and values. Unknown shell variables
remain under `custom_environment`; they are not discarded to manufacture an
empty result. The comments-only fixture exercises parsing with no final values.

## Preserved contracts and regressions

Nine new HTTP Go cases exercise:

- Missing, empty and comments-only shell files: HTTP 200 with the explicit empty
  object, verified with `assert.JSONEq` on the response body.
- Missing DB path key, an actual read failure (a directory used as the file),
  and invalid shell syntax: HTTP 500 with a nonempty string `error` and no
  success envelope. Permission-dependent fixtures are avoided.
- Valid shell configuration under each `HARDWARE_BACKEND` process selection
  (`mowgli`, `mavros`, `openmower`): numeric zero, boolean false, integer, empty
  string and custom environment values retain their JSON types and values.
  These tests use a synthetic schema with the existing extraction/coercion
  functions; they do not claim hardware execution or backend-specific routing.

Thirteen new frontend cases reuse the existing `useConfig.test.tsx` harness
(which also tests `useSettings`) and `settingsContract.test.ts`:

- Valid empty/nonempty envelopes are accepted without notifications.
- Missing `settings`, null, arrays, scalar containers and invalid nested values
  remain rejected. Independent valid YAML data is retained.
- API error bodies, unsuccessful HTTP responses and rejected requests produce
  exactly one load error per failed source, without a success notification.
- Rerendering neither repeats the notification nor refetches the sources.
- Loading calls only the shell/YAML read methods and the DB key-read method;
  the DB setter is never called. A DB key read is legitimately a POST to
  `/config/keys/get`, not a configuration write.
- Decoder output preserves zero, false, empty strings, null and nested values.

Existing tests continue to verify YAML precedence, DB authority, delayed DB
reads, legacy YAML endpoint 404/405 compatibility and save confirmation. No
frontend production code, Settings manager, persistence route, dirty-state
handling, restart command or calibration behavior is changed. The nominal
Playwright mock already returns the correct envelope and was not modified.

## Validation

- Red reproduction: `go test ./pkg/api -run '^TestGetSettings_EmptyHTTPEnvelope$'
  -count=1` failed in all three expected cases before the fix.
- Focused Go: `go test ./pkg/api -run '^TestGetSettings_' -count=1` passed.
- Full Go: `go test ./...` passed. An initial sandboxed run failed on existing
  tests requiring local TCP/Unix sockets; the complete suite passed when rerun
  with those local restrictions lifted. No production service was contacted.
- Focused frontend: **47 tests passed in two files**.
- Full frontend: `yarn test --pool=threads --maxWorkers=2` passed: **2,086 tests
  in 188 files**, plus the appearance registry test (228.59 seconds for Vitest).
  This is 13 additional tests over the 2,073 baseline.
- TypeScript: `yarn tsc --noEmit` passed.
- Build: `yarn build --outDir /tmp/gui-lot22-dist` passed.
- `yarn lint` passed. Full JSON inventories before/after confirm **105 → 105
  warnings, zero errors**. Both modified frontend test files have zero warnings.
- Existing Chromium checks: **10 passed**: eight Settings route scenarios,
  the nominal battery Advanced menu (explicitly checking no invalid-settings
  notification), and the malformed-settings scenario (error remains visible,
  no power command). All REST/WebSocket traffic is mocked.
- `gofmt -l` returned no files; `git diff --check` and the new report's whitespace
  check passed. Final index check is empty.

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

## Limits and review files

The browser tests validate the existing mocked envelope; Go HTTP tests establish
the actual server serialization, and hook tests establish consumer acceptance.
They are not an end-to-end test against a running Go server or the robot.
No robot command, live configuration write or hardware check was performed.
Deployment verification on the robot remains unperformed; these results do not
prove that every possible settings-load error has this cause.

Existing jsdom pseudo-element style notices, the Ant Design v5/React 19
compatibility notice in the browser and the large-chunk build warning remain.
No new application anomaly was found. GNSS, global diagnostics, map/dock partial
failure, GeoJSON rings, datum and remaining ESLint debt remain outside this fix.

Modified files:

- `gui/pkg/api/types.go`
- `gui/pkg/api/settings_test.go`
- `gui/web/src/hooks/useConfig.test.tsx`
- `gui/web/src/utils/settingsContract.test.ts`
- `gui/web/docs/gui-lot22-settings-payload-contract.md`

Review diff: `/tmp/gui-lot22-final.diff` (includes this report, excludes MAVROS).
No staging, commit or push; the index remains empty.
