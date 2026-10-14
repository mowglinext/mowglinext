# Configuration persistence integrity

## Baseline and scope

2026-10-14, branch `fix/gui-warnings-errors`, baseline commit
`a92fffd4338ebfffc28de79fecbb1c2083810546`. This follows the frontend
[data-boundary investigation](typescript-data-boundaries.md). Changes are limited
to the Go configuration write route, related tests and this report. The existing
modified `ros2/src/external/mowglimavros` checkout was left untouched.

## Root causes and correction

`ConfigSetKeysRoute` previously validated and wrote each entry in the same Go map
loop. An invalid value could therefore be discovered after other entries had
already been written. Failed `db.Set` calls were ignored, and the route returned
HTTP 200 with the requested values even when storage rejected them.

The route now accepts exactly one non-null JSON object, validates all values as
strings before any write, and then writes keys in lexicographic order. Empty
strings and empty objects remain valid. No coercion, retries or rollback are
introduced. The first failed `Set` stops processing and returns HTTP 500 using
the existing `ErrorResponse` envelope. The message identifies the quoted key and
warns that earlier keys may already have been stored. It does not include
submitted/stored values or the raw storage error, which could contain secrets.

| Condition | Previous behavior | Corrected behavior |
| --- | --- | --- |
| All writes succeed | 200, original string map | Same response shape; every requested `Set` must return nil |
| Empty string | Accepted | Accepted, including orderly storage reopen |
| Empty object | 200, no writes | Unchanged |
| Mixed string/non-string values | 400, possibly after partial writes | 400 JSON error, zero writes |
| Malformed JSON | Bind error handling | 400 JSON error, zero writes, no raw decoder details |
| Null/non-object or additional JSON/trailing garbage | Not fully constrained by the old binding path | 400 JSON error, zero writes |
| Storage write fails | Ignored; eventually 200 | 500 JSON error; no success echo, no later writes |

Sorting removes map-order ambiguity for write attempts and failure tests. It is
not a transaction. Normal successful requests still perform one `Set` per key
and return the existing map. `/config/keys/get`, `/config/envs`, generated Swagger
and API clients are unchanged.

## Storage guarantees and partial completion

`IDBProvider` exposes individual `Set`, `Get`, `Delete` and `KeysWithSuffix`
operations. There is no batch transaction, rollback, snapshot or durability
acknowledgment contract. `DBProvider.Set` delegates to Bitcask `Put`.

The pinned Bitcask v1.0.2 defaults to `Sync=false`; `NewDBProvider` does not enable
per-write sync. HTTP 200 therefore means all `Set` calls returned nil. It does
not establish multi-key atomicity or survival of a power loss. Tests involving
an orderly close/reopen do not establish crash durability.

If a later write fails, earlier successful writes remain. Later keys are not
attempted. A failed low-level operation can itself have uncertain side effects;
the mock's unchanged failed key is not a universal storage guarantee. Concurrent
requests can interleave, and a lost HTTP response can leave the caller uncertain
about completion. An automatic rollback could overwrite concurrent changes or
fail itself, so none is attempted.

Client reconciliation should keep the dirty draft and report failure, then
reload the requested effective values and compare them with the draft before a
reviewed retry of remaining differences. Do not automatically resubmit a whole
stale form. Current reads cannot reliably prove reconciliation after a storage
error, as described below. Reliable verified reconciliation needs a separately
reviewed provider/API contract, rather than an extra blind readback or retry.

The existing `useConfig` rejects non-2xx writes; inspected settings, map-offset
and map-bearing callers already handle rejection and own their failure
notifications. The settings caller does not issue its success notification after
rejection. No client adjustment was necessary, and frontend code is unchanged.

## Read compatibility and unresolved classification

Bitcask itself returns `ErrKeyNotFound` for an absent entry. `DBProvider.Get`
handles it by trying a nonempty environment value, then a nonempty application
default. If neither exists, it constructs a new missing-key error without
wrapping the sentinel. The ordinary mock also returns an unclassified missing
error. Other storage errors are returned, but `IDBProvider` offers no public
classification distinguishing them from legitimate absence across providers.

A stored empty value currently returns an empty result with nil error, even
when an environment value/default exists. It must not be reinterpreted as a
missing value. `GetWithEnvFallback` has its own empty-value semantics and is
unchanged.

`ConfigGetKeysRoute` therefore retains its legacy behavior: successful reads
replace placeholders; any `Get` error preserves the incoming placeholder and
the route returns 200. This includes an operational read failure, which remains
an explicit unresolved integrity limitation. No error-message matching or
selective guess at error categories was added. This task does not claim that
read failures now propagate correctly.

A follow-up can introduce an additive typed lookup capability with explicit
found/missing/error outcomes and source information for stored, environment and
default values, while preserving existing `Get` callers and fallback precedence.
A versioned or optional API can then distinguish operational failure from
legitimate missing keys without changing legacy placeholder clients. Every
production and mock provider must implement the same distinction. Separately,
genuine batch acknowledgments need a defined concurrency/transaction and
durability contract; per-key acknowledgments alone would not make writes atomic.

## Regression coverage

Thirty new table-driven/regression cases use an injectable failure provider and
real temporary Bitcask stores:

- Write route: 20 cases covering single/multiple keys, empty strings/objects,
  whitespace, malformed/non-object/multiple-document payloads, every non-string
  JSON value category, first-write failure and failure after one successful write.
  Assertions cover zero writes on validation failure, deterministic attempts,
  stored state, exact successful response maps, status, the string error envelope,
  failed-key identification, absence of values/raw errors, and no retries or
  rollback.
- Read route: five cases covering stored values, stored empty values, absent keys
  with empty/nonempty placeholders and an injected operational error. The last
  case explicitly captures the unresolved legacy behavior rather than asserting
  a fabricated missing/error distinction.
- Provider: four fallback cases covering environment, defaults, stored values
  and stored empty strings. Stored cases close/reopen the real temporary store.
  One additional test establishes that the underlying missing sentinel is lost
  at the public provider boundary.

Tests do not rely on Go map iteration or access robot hardware.

## Modified files and validation

- `gui/pkg/api/config.go`: full write validation and truthful failure response.
- `gui/pkg/api/config_test.go`: injectable failures and write/read regressions.
- `gui/pkg/providers/db_config_contract_test.go`: actual storage/fallback contract tests.
- `gui/web/docs/config-persistence-integrity.md`: this report.

Normal-user validation:

| Check | Result |
| --- | --- |
| `go test ./pkg/api ./pkg/providers` from `gui` | PASS |
| `go test ./...` from `gui` | PASS |
| `go vet ./...` from `gui` | PASS |
| Focused provider contract tests after adding orderly reopen assertions | PASS |
| `yarn lint` from `gui/web` | PASS: 365 warnings, zero errors, unchanged |
| `yarn test` from `gui/web` | PASS: 1,210 Vitest tests across 155 files; separate appearance-registry suite passes |
| `gofmt` check on changed Go files | PASS |
| `git diff --check` | PASS |

The initial sandbox run could not bind localhost for existing fleet tests, and
the compiler cache was read-only. Required Go checks were rerun successfully as
the normal project user with those sandbox restrictions lifted. Temporary logs
are under `/tmp/config-integrity-*`; no evidence archive is versioned. Frontend
build was not rerun because no frontend source or generated contract changed.

## Compatibility and release checks

Native Mowgli, MAVROS and OpenMower share this GUI string-store API. No ROS YAML,
backend routing, GNSS, firmware, motor, navigation or control-command contract
was changed. Read fallback and legitimate empty-string behavior remain intact.
Valid successful write bodies and response shapes remain compatible. Legacy
clients that depended on a false 200 after a failed write now receive the existing
failure envelope, intentionally.

Before release, use an isolated development server with injected storage failures
to check dirty-form retention, save-error notifications, absence of success
notifications, and explicit reload/reconciliation after a partial write. Verify
normal map-offset/bearing saves and empty-string settings too. Do not simulate
disk failures on an operating robot. No robot/field acceptance or power-loss
durability is claimed. All changes are uncommitted for review; no push, merge or
pull request was made.
