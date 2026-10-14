# GUI Batch 12 — advanced settings integrity

Starting HEAD: `66da130a75b64fed9162ad4099316c1d4b5e3dcf`, on
`fix/gui-warnings-errors`. Batch 11 is committed at that HEAD. Initial worktree
changes consisted only of the existing local MAVROS submodule modifications,
which are outside this batch and remain untouched.

## Original risks and inspected contracts

The path is installed YAML → `GetSettingsYAML` in `gui/pkg/api/settings.go` →
`useSettingsManager` → SettingsPage → AdvancedSection → manager draft → general
Save (`POST /settings/yaml`) → YAML. `SettingsValues` is generated and deliberately
has an `unknown` index signature; it was read, not edited.

The Go reader exposes a flat parameter map across ROS YAML nodes. Each value can
be a string, boolean, JSON number, null, array or nested object; it does not turn
nested parameter values into editable string fields. The flattening contract
assumes parameter names are unique across nodes. The writer merges only supplied
keys into the installed YAML, deletes explicit null keys, preserves unrelated
content, prunes schema defaults and scrubs retired parameters. Numeric YAML type
hints come from schema/template/on-disk types (`settings_yaml_types.go`). Its
numeric typing deliberately excludes strings containing digits.

Previously the advanced table called `String(value)` for every value and inferred
booleans/numbers on each keystroke. Editing a string `"42"` or `"true"` could change
its type. Arrays and objects lost their structure in ordinary input rendering.
Partial numeric text could become a string, an empty string could be confused
with numeric parsing, and infinity/overflow could become a nonfinite number.
Ant Design renderer records and callbacks propagated unchecked `any` values.
Add normalized names to lowercase/underscores and could overwrite existing keys
or introduce fields deliberately owned/hidden by other sections.

## Scalar editing and runtime typing

`AdvancedSection` now has a precise row (`name`, `key`, `value: unknown`) and typed
scalar/delete callbacks. Runtime shape checks select the editor:

- Existing strings remain strings, including `"42"`, `"true"` and empty strings.
- Existing booleans use a boolean switch; false is never deletion.
- Existing finite numbers use a local text draft. Typing `-`, `.`, `1e` or `1.`
  does not change manager state. Enter or blur commits a complete finite decimal
  number. Invalid drafts are reported and reset to the current numeric value,
  without passing a string, null or nonfinite number to the manager.
- Arrays and plain JSON objects receive a read-only JSON preview, capped at 1,000
  characters plus an ellipsis. No ordinary input can coerce or replace them.
  Unsupported/non-JSON/nonfinite values are explicitly read-only and unavailable
  for scalar editing. React escapes the preview text. Explicit deletion still
  applies to a whole structured parameter if the operator confirms it.

A small dedicated `advancedSettings.ts` utility owns decimal parsing, exact new
key syntax and bounded structured presentation, with independent tests. Its
structured guard reuses the existing settings JSON decoder; there is no new
validation framework or arbitrary JSON editing mode.

Decimal parsing accepts zero, negative zero, signs, decimals and finite exponent
notation. NaN, infinities, overflowing exponents, hex/binary/octal syntax, numeric
prefixes with suffixes, empty input and incomplete numeric syntax are rejected.
New keys have an explicit String/Number/Boolean selector. The default is String,
so numeric-looking text is never inferred silently. No UI edit or Add operation
calls the backend; only the existing general Settings Save does so.

## Key ownership and deletion

SettingsPage passes the manager's narrowly scoped `validateAdvancedKey` callback.
The manager shares its protected set between row visibility and Add validation:
all `SECTION_DEFINITIONS` keys, across all backends, plus `HIDDEN_FROM_ADVANCED`.
The hidden set now also includes the remaining names in the actual Go producer's
`retiredParamKeys` list (including retired localizer fields and
`ticks_per_revolution`). A manager regression imports that Go source as test-only
raw text and asserts every retired key remains protected. This adds no backend
runtime dependency and does not duplicate the protected list in the component.

New names must be exact flat ASCII identifiers: a letter or underscore followed
by letters, digits or underscores. Case is preserved; no lowercase, trim or
underscore substitution is performed. Empty names, whitespace, punctuation,
paths/dotted names and JS prototype-reserved names are rejected. The existing
own-key check rejects duplicates, including keys marked null for pending deletion.
Legitimate unknown keys remain available; existing names are never renamed.
Existing advanced parameters with other naming conventions can still be displayed
and edited according to their value shape, but cannot be newly introduced through
Add. This is UI ownership protection, not a new authorization layer for direct
HTTP clients.

Delete remains `onChange(key, null)`: only that key is marked, its row disappears,
and no write occurs until Save. Revert restores its original value. Successful
Save sends the null deletion and prunes the key from the local/saved baseline.
Empty strings, zero, false and invalid numeric drafts cannot trigger deletion.
The Batch 7B partial-persistence behavior, Batch 10 sensor-null handling and dock
capture/save guards, dirty-only YAML payloads and restart rules are unchanged.
No persistence implementation was rewritten.

## ESLint comparison

Verified inventory baseline: **200 warnings, zero errors**. Final inventory:
**189 warnings, zero errors**. All 11 AdvancedSection warnings are removed;
other files and rules retain their counts. No suppressions, generated edits or
unsafe assertions were added.

| Rule | Before | After | Removed |
| --- | ---: | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 85 | 78 | 7 |
| `@typescript-eslint/no-unsafe-member-access` | 31 | 29 | 2 |
| `@typescript-eslint/no-unsafe-argument` | 21 | 19 | 2 |
| `@typescript-eslint/no-unsafe-assignment` | 35 | 35 | 0 |
| `@typescript-eslint/no-unsafe-call` | 4 | 4 | 0 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 | 0 |
| `react-refresh/only-export-components` | 11 | 11 | 0 |
| `react-hooks/set-state-in-effect` | 8 | 8 | 0 |
| `react-hooks/purity` | 2 | 2 | 0 |
| **Total** | **200** | **189** | **11** |

| Changed/new maintained file | Before | After |
| --- | ---: | ---: |
| `components/settings/AdvancedSection.tsx` | 11 | 0 |
| `components/settings/advancedSettings.ts` | 0 | 0 |
| `components/settings/advancedSettings.test.ts` | 0 | 0 |
| `components/settings/AdvancedSection.test.tsx` | 0 | 0 |
| `hooks/useSettingsManager.ts` | 0 | 0 |
| `hooks/useSettingsManager.merge.test.tsx` | 0 | 0 |
| `pages/SettingsPage.tsx` | 0 | 0 |
| `i18n/locales/en.json` and `fr.json` | 0 | 0 |

These files and this report are the complete scope. No Go, ROS2, firmware,
MAVROS, generated TypeScript or API definitions were changed. Existing numeric
YAML hints, backend parameter routes, GNSS/receiver/motor commands, DB versus
YAML ownership, dock yaw and blade/emergency behavior remain untouched.

## Regression coverage and remaining limits

Component tests run the actual settings manager with mocked APIs, asserting form
values, dirty counts, exact Save payloads and lack of premature writes. They cover
new explicit scalar types, literal strings, zero/false/empty strings, intermediate
and invalid numeric drafts, structured previews and preservation through unrelated
Save, null deletion with Save/Revert, duplicate/owned/retired/invalid keys and name
normalization collisions. Manager tests additionally cover pending-null duplicates,
inactive backend section ownership and parity with all producer-retired keys.

Structured editing remains read-only except explicit whole-key deletion. There
is no new-key type migration for existing scalars and no support for newly adding
arrays, objects or null values. Invalid or uncommitted numeric text is not part
of the manager draft; it cannot enter a general Save payload. Key validation does
not promise that an arbitrary unknown parameter has a ROS consumer. The backend
continues to require unique names across YAML nodes and to scrub retired keys and
prune defaults on Save.

JSON numbers do not distinguish YAML int versus float and use JavaScript number
precision. The frontend does not infer an integer declaration from a value that
happens to be integral, add parameter-specific ranges, or claim to preserve numeric
lexemes/comments. Backend schema/template/on-disk hint behavior is unchanged;
unknown numeric declarations still need a trustworthy producer contract. Negative
zero is accepted numerically; JSON serialization does not preserve its sign.
No hardware action or robot motion was exercised.

## Initial full validation (before numerical review follow-up)

- Focused new regressions and existing manager integrations: **98 tests passed**
  across three files (66 new tests and 32 existing manager tests).
- `yarn test`: **1,839 Vitest tests passed**, 177 files; separate appearance
  registry test passed. Baseline was 1,773 tests. No existing test was removed.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-batch12-dist`: passed; existing large-chunk notice
  remains. No lint/build configuration or timeout was changed.
- Full ESLint JSON comparison: **200 → 189 warnings**, **0 → 0 errors**.
  All changed source and test files report zero warnings.
- `git diff --check`: passed. Final status/stat inspected; no files staged and
  starting HEAD unchanged. Batch 12 remains uncommitted for review.
- No backend code changed, so Go tests were not rerun for this frontend batch.
  Temporary inventories/logs/build outputs are under `/tmp/gui-batch12-*`.


## Final targeted numerical review follow-up

The decimal parser now rejects a mathematically nonzero mantissa whose numeric
conversion becomes zero, including negative underflow (`1e-400`, `-1e-400`).
Actual zero mantissas, including signed zero and zero with very large exponents,
remain accepted. Representable nonzero subnormal values remain valid.

For mathematically integral decimal input outside the JavaScript safe-integer
range, the parser computes the exact decimal integer with BigInt and compares it
to the converted Number's exact integer value. Inaccurate conversions such as
`9007199254740993`, its negative/decimal/exponent spellings, and
`1000000000000000100` are rejected. Exactly representable larger integers such as
`9007199254740992`, `1e20` and `1000000000000000128` remain valid. This does not
attempt to eliminate ordinary binary rounding of fractional decimals or change
any YAML key, deletion, scalar-type or manager contract.

NumberEditor also records its last logical numeric commit synchronously, so
Enter followed by blur cannot call onChange twice before the parent echoes the
new numeric value. The regression intentionally delays that echo and asserts
exactly one callback and no premature backend write.

Only `advancedSettings.ts`, `advancedSettings.test.ts`, `AdvancedSection.tsx`,
`AdvancedSection.test.tsx` and this report receive complementary changes.
Validation after the correction: **89 targeted tests pass** across the helper
and component suites; `yarn tsc --noEmit` passes; full ESLint remains **189
warnings, zero errors**; `git diff --check` passes. The previous full-suite/build
results above belong to the initial lot; these were not rerun for this targeted
follow-up. No Go or manager code changed.

After validation, only these five complementary files are added to the existing
index, as requested. Previously indexed files are preserved. MAVROS remains
unstaged and untouched. No commit or push is performed.
