# GUI Lot 19 — Settings callback types

## Preflight and contracts

Branch `fix/gui-warnings-errors`, HEAD `2dae8ee1` (Lot 18 committed).
Index empty; the sole preexisting worktree change was the independent MAVROS
submodule modification. ESLint baseline confirmed: **123 warnings, zero errors**.
Vitest baseline: **2,044 tests**.

Inspected:

- `types/settings.generated.ts`: sparse optional settings with declared scalar
  types and an unknown index signature for undeclared keys. No generated edit.
- `utils/settingsContract.ts`: SettingsValue describes recursive JSON, including
  arrays and records. That broader document type does not describe the scalar
  controls in these nine sections.
- `hooks/useSettingsManager.ts`: handleChange accepts unknown to support other
  editors and changes local state only; resetToDefault also changes local state
  only. Revert restores the saved baseline. Dirty detection and Save submit only
  changed keys. Null values retain the existing YAML override-deletion meaning;
  the saved baseline and local pruning, partial failures and concurrent-edit
  handling are unchanged.
- `pages/SettingsPage.tsx` and all nine sections: handleChange is supplied as the
  callback, with separate reset/default callbacks where supported. Other callers
  remain compatible under the full TypeScript check.
- Public Ant Design InputNumber/Switch/Input/Password/Select props and existing
  NumberWithAddon/TextWithAddon wrappers: number controls emit number or null,
  switches boolean, text/password controls strings. The three Mowing selects
  use numeric options and values. NTRIP provider/station callbacks emit strings
  and numeric ports; optional provider defaults are checked before emission.

## Exact changes

Only nine callback prop signatures and three erased JSX type arguments change.
Keys remain strings; there is no new key routing or value conversion.

| Section | Callback value type | Warnings before | After |
| --- | --- | ---: | ---: |
| BatterySection.tsx | number \| null | 1 | 0 |
| DockingSection.tsx | number \| boolean \| null | 1 | 0 |
| LedsSection.tsx | string \| number \| boolean \| null | 1 | 0 |
| MowingSection.tsx | number \| boolean \| null | 1 | 0 |
| MqttSection.tsx | string \| number \| boolean \| null | 1 | 0 |
| NavigationSection.tsx | number \| null | 1 | 0 |
| NtripSection.tsx | string \| number \| boolean \| null | 1 | 0 |
| RainSection.tsx | number \| null | 1 | 0 |
| SafetySection.tsx | string \| number \| boolean \| null | 1 | 0 |

SafetySection remains read-only and never invokes its compatibility callback;
its scalar Settings prop is retained without exposing any safety control.
Rain modes are numeric, not text. Mowing's three Select components now specify
`Select<number>` so Ant Design does not propagate its default generic any into
the narrowed callback. All option values, handlers and selection rules remain
identical.

No inspected control emits undefined. Optional initial values and display
fallbacks do not imply that a change event emits undefined. Null remains part
of every numeric-edit callback, while zero, false and empty text are preserved.
Existing explicit conversions also remain: for example, manual mow-angle
clearing already uses `v ?? 0`; this lot does not change that behavior or add
an implicit conversion for other numeric fields.

Exact per-section unions avoid allowing JSON arrays/objects or undefined that
these controls do not emit. No new shared abstraction or export from the private
SettingsFieldCard scalar type is needed. No runtime assertions, unknown
substitution, suppression, or lint configuration change is introduced.

## Validation

- Focused Vitest: **110 passed**, ten files: MowingSection, LedsSection,
  SafetySection, NtripSection.addons, SettingsFieldCard, NumberWithAddon,
  TextWithAddon, useSettingsManager.merge, settingsValues and settings-types.
  Existing tests cover numeric edits/clearing, zero/false/empty values, boolean
  changes, password visibility, controlled inputs, reset affordances, read-only
  safety, local edits, dirty-only/partial persistence and concurrent edits.
- No new regression test is needed for this signature-only change. The existing
  manager implementation was inspected for Revert/default behavior; no new
  dedicated UI claim is made for the five sections without their own test file.
- TypeScript transpileModule comparison against HEAD produced **identical
  JavaScript for all nine sections** (same React JSX/ES2020/ES module options).
  This separately confirms that the edits introduce no executable change to
  event handling, defaults, local edits, Save/Revert or HTTP requests.
- `yarn tsc --noEmit`: passed, including callers of the narrowed props.
- `yarn build --outDir /tmp/gui-lot19-dist`: passed.
- Full `yarn test --pool=threads --maxWorkers=2`: **2,044 passed**, 187 files,
  plus the appearance registry test; 208.40 seconds. Test count unchanged.
- Global ESLint JSON: **123 → 114 warnings**, zero errors. All nine target files
  have zero warnings. Only no-explicit-any decreases (51 → 42); other rules
  are unchanged.
- `yarn lint`: passed, 114 warnings and zero errors.
- `git diff --check`: passed; index remains empty.

| ESLint rule | Before | After |
| --- | ---: | ---: |
| @typescript-eslint/no-explicit-any | 51 | 42 |
| @typescript-eslint/no-unsafe-assignment | 24 | 24 |
| @typescript-eslint/no-unsafe-member-access | 13 | 13 |
| @typescript-eslint/no-unsafe-argument | 8 | 8 |
| @typescript-eslint/no-unsafe-call | 3 | 3 |
| @typescript-eslint/no-unsafe-return | 3 | 3 |
| react-refresh/only-export-components | 11 | 11 |
| react-hooks/set-state-in-effect | 8 | 8 |
| react-hooks/purity | 2 | 2 |
| **Total** | **123** | **114** |

## Limits and files

This is compile-time callback typing, not a new runtime settings decoder or a
key-to-value schema. The manager intentionally retains its broader contract for
other editors. Existing validation/persistence boundaries and their limitations
are unchanged. No API request is introduced, and no real YAML or robot setting
is written by validation. Password tests use fictional credentials.

PositioningSection, GNSS logic, GPS/fix classification, diagnostic severity and
robot settings-loading issues are deferred as requested. Existing typing debt
in other components and test helpers remains outside the selected signatures.
Build large-chunk and existing jsdom pseudo-element notices are not ESLint
failures. Temporary validation artifacts remain in `/tmp`.

Exact modified files (all under gui/web):

- src/components/settings/BatterySection.tsx
- src/components/settings/DockingSection.tsx
- src/components/settings/LedsSection.tsx
- src/components/settings/MowingSection.tsx
- src/components/settings/MqttSection.tsx
- src/components/settings/NavigationSection.tsx
- src/components/settings/NtripSection.tsx
- src/components/settings/RainSection.tsx
- src/components/settings/SafetySection.tsx
- docs/gui-lot19-settings-callback-types.md

No tests, manager, SettingsPage, generated source, backend, ROS, dependencies,
CSS or MAVROS files are changed. Index remains empty; no commit or push.
