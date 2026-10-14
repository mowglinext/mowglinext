# GUI Batch 13 — Ant Design InputNumber migration

Branch: `fix/gui-warnings-errors`. Starting HEAD:
`f67fd88fe5e2b9b87bc6036c95492149972f302a` (Batch 12 committed).
The initial worktree contained only independent MAVROS submodule changes.
Ant Design remains **5.29.3**; no dependency or lockfile was changed.

## Inventory and scope

TypeScript JSX syntax was inspected rather than counting metadata/test strings.
There were **104 InputNumber controls across 23 maintained source files**:
100 addonAfter occurrences and four addonBefore occurrences. All 104 are migrated.
After migration, **zero InputNumber addonBefore/addonAfter JSX props remain**.

| Source under `gui/web/src` | Before | Remaining |
| --- | ---: | ---: |
| `components/RobotComponentEditor.tsx` | 6 | 0 |
| `components/robot/MeasurementEditor.tsx` | 1 | 0 |
| `components/settings/BatterySection.tsx` | 9 | 0 |
| `components/settings/DockingSection.tsx` | 6 | 0 |
| `components/settings/DriveMotorSection.tsx` | 10 | 0 |
| `components/settings/HardwareSection.tsx` | 18 | 0 |
| `components/settings/IrriSenseRuleCard.tsx` | 3 | 0 |
| `components/settings/LedsSection.tsx` | 8 | 0 |
| `components/settings/LocalizationSection.tsx` | 3 | 0 |
| `components/settings/MowingSection.tsx` | 9 | 0 |
| `components/settings/NavigationSection.tsx` | 6 | 0 |
| `components/settings/NtripSection.tsx` | 2 | 0 |
| `components/settings/ObstaclesSection.tsx` | 6 | 0 |
| `components/settings/PositioningSection.tsx` | 2 | 0 |
| `components/settings/RainSection.tsx` | 2 | 0 |
| `components/settings/SensorsSection.tsx` | 2 | 0 |
| `components/settings/SettingsFieldCard.tsx` | 1 | 0 |
| `components/settings/UniversalGnssAdvancedSettings.tsx` | 1 | 0 |
| `pages/FleetPage.tsx` | 2 | 0 |
| `pages/OnboardingPage.tsx` | 3 | 0 |
| `pages/ParametersPage.tsx` | 1 | 0 |
| `pages/map/components/EditLidarCorridorModal.tsx` | 1 | 0 |
| `pages/map/components/ImportOpenMowerModal.tsx` | 2 | 0 |

`gnssConfig.ts` retains an `addonAfter` metadata field; its value is now passed as
`after` to the new component, not as a deprecated prop to InputNumber. The helper's
Omit type and tests also mention the deprecated property names without using them.

## Composition and compatibility

`components/inputs/NumberWithAddon.tsx` renders the public `Space.Compact` and
`Space.Addon` composition with the original InputNumber. This follows the official
[InputNumber migration guidance](https://ant.design/components/input-number/)
and uses APIs confirmed in the installed 5.29.3 declarations/implementation.
An initial internal Compact context import was removed after representative
sizing tests exposed a module-entry mismatch. No internal Ant Design runtime
imports remain in the helper.

The helper is a single React 19 form control, forwarding controlled props, id,
ARIA/data attributes, keyboard/focus/change handlers and the numeric input ref.
Units are visible siblings, never replacements for numeric values. Optional empty
units return the plain InputNumber. Explicit size reaches both the compact group
and the input; Form/ConfigProvider size context remains available. The only
migrated input already nested in a Compact group is IMU yaw, which explicitly
specifies small size. No private context inference is needed for these callers.
Future controls nested in an explicitly sized Compact group should also specify
their desired input size rather than depending on an undocumented internal context.

The previous group width is assigned to Compact; the numeric member uses flexible
remaining width and minWidth 0, with its unit kept readable. Fixed widths (160 or
280 px) and 100% widths retain their intent. The group is capped to its container
on small screens. React ref forwarding targets InputNumber, not the layout div.
Form.Item can still inject value/onChange/id and associate labels with the input.

A syntax-level comparison against HEAD verified all **104 attribute lists and
expressions**, including value, onChange, onBlur, onPressEnter, min/max/step/
precision, disabled/status/size, refs, identifiers and ARIA, were unchanged except
for the component tag and addon prop names. Only unused InputNumber imports were
removed; imports for remaining plain inputs were retained. TypeScript validates
all source references. No calculations, defaults, routing, confirmations,
validation code or producer/backend logic was changed.

## Sensitive paths and warning checks

The live ParametersPage retains its coercion, null rejection, profile
acknowledgement and safety confirmation. Its migrated numeric control must open
the confirmation before any POST to `/params`; the regression then confirms the
modal and observes the original write path. Its text Input remains out of scope.

ImportOpenMowerModal's existing Enter and blur handlers remain intact. Each
explicit event produces one preview reprojection; the wrapper adds no event
listeners or extra calls, and neither event applies the map. Enter followed by a
separate blur after the preview completes remains two preview requests, as before.
This batch does not deduplicate those business-level events or introduce retries.

Warning tests spy on console.error/console.warn while preserving their original
implementations. Browser tests collect console messages without suppressing them.
Covered unit, Form.Item, modal, settings, live tuning and sensor scenarios have
**no InputNumber addon deprecation warnings**. Existing Modal deprecations and
text-input addon warnings are separate and have not been hidden.

## Input deprecations left separate

Nine actual text-input addon usages remain unchanged:

| Source/line | Component | Deprecated prop |
| --- | --- | --- |
| `components/settings/NtripSection.tsx:179` | `Input` | `addonBefore` |
| `components/settings/NtripSection.tsx:185` | `Input` | `addonBefore` |
| `components/settings/NtripSection.tsx:187` | `Input.Password` | `addonBefore` |
| `components/settings/NtripSection.tsx:197` | `Input` | `addonBefore` |
| `components/settings/NtripSection.tsx:230` | `Input` | `addonBefore` |
| `components/settings/NtripSection.tsx:232` | `Input` | `addonBefore` |
| `components/settings/NtripSection.tsx:233` | `Input` | `addonBefore` |
| `components/settings/NtripSection.tsx:234` | `Input.Password` | `addonBefore` |
| `pages/ParametersPage.tsx:141` | `Input` | `addonAfter` |

No Input bordered props or Input.Group JSX usages were found in `src`. These nine
text usages need their own migration/review; their warnings are not ESLint counts.

## ESLint comparison

Full inventories remain **189 warnings, zero errors** before and after:

| Rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 78 | 78 |
| `@typescript-eslint/no-unsafe-argument` | 19 | 19 |
| `@typescript-eslint/no-unsafe-assignment` | 35 | 35 |
| `@typescript-eslint/no-unsafe-call` | 4 | 4 |
| `@typescript-eslint/no-unsafe-member-access` | 29 | 29 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 |
| `react-hooks/purity` | 2 | 2 |
| `react-hooks/set-state-in-effect` | 8 | 8 |
| `react-refresh/only-export-components` | 11 | 11 |
| **Total** | **189** | **189** |

Every migrated production file retains its prior warning count. New helper and
new/modified source tests add zero warnings. No lint suppression was added.

## Validation evidence and limits

Representative checks were completed before global validation. The shared helper
and SettingsFieldCard tests cover units, zero/precision, bounds, disabled/status,
ref focus, Enter/blur/change, explicit/provider sizing, no-unit fallback, width
props, Form.Item label/value injection and numeric validation. A valid form submits
2.5; an invalid negative value displays a validation error and cannot submit.
Targeted regressions also cover import previews and live parameter confirmations.
The expanded focused run passes **95 tests across 10 files**. The subsequently
strengthened Form validation check passes all **11 helper tests**.

Chromium tests pass **4 cases**: real sensor fields at 390/1440 px and an isolated
Form.Item modal fixture at 390/1440 px. They check actual bounding boxes, visible
unit alignment, fixed 280 px and fluid widths, long addon text, edits and form
submission, and collect InputNumber warning messages. The modal fixture lives in
`tests/e2e/fixtures` and does not enter the production app. REST/WebSocket calls in
application scenarios are completely mocked; no robot or real backend is used.

Additional existing browser scenarios yielded one pass and two failures:
`hardware-backend-settings` MAVROS label assertion and `sensor-editor-regressions`
missing LiDAR drag target. The same two failures and one pass were reproduced
against an unchanged HEAD copy under `/tmp`, so those scenarios do not validate
this migration and remain a pre-existing browser-suite limitation. Their fixtures
were not rewritten to make them pass. The new focused scenarios establish the
specific unit/layout behavior without claiming the entire browser suite is green.

All 104 source sites are migrated, but browser screenshots of every field and
unit/backend combination were not exhaustively acquired. No physical behavior or
hardware accuracy claim follows from display composition tests. Temporary logs,
inventories, comparison source copy and browser artifacts remain under `/tmp`.
Backend, ROS2, firmware and MAVROS files are untouched; nothing is staged, committed
or pushed by this batch.

## Final validation

- Focused components: **95 passing tests** across 10 files; additional focused
  form-validation rerun: **11 passing helper tests**.
- Full `yarn test`: **1,877 passing Vitest tests**, 180 files, plus the separate
  appearance registry test. Baseline: 1,863 tests; 14 regressions added.
- New Chromium regression suite: **4 passing tests**, desktop and mobile.
  The additional legacy browser-suite failures are recorded above with baseline
  reproduction; they are not reported as passes.
- `yarn tsc --noEmit` and `yarn build --outDir /tmp/gui-batch13-dist`: passed.
- Full ESLint JSON inventories: **189 → 189 warnings**, **0 → 0 errors**;
  no file-level warning counts changed. Package/lint/build settings are unchanged.
- Final JSX inventory: **0 InputNumber deprecated addon usages**, **9 untouched
  Input/Input.Password usages**. Numeric metadata/property-name test strings
  are excluded from the count of actual JSX usages.
- Syntax audit: all 104 migrated controls preserve their original attribute
  lists/expressions apart from component/addon names. Compilation verifies that
  remaining InputNumber imports and ref usages are valid.
- `git diff --check`: passed. Status/stat and empty staged diff checked;
  HEAD/branch remain unchanged and MAVROS remains outside the index.
- No backend code changed; Go tests were not needed for this presentation batch.
