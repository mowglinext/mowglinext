# Ant Design text input addons — GUI Batch 13C

## Preflight and scope

Branch: `fix/gui-warnings-errors`. Baseline HEAD:
`2dbf09925fd2206f50a83b5acfd409dd835b8f77` (Batch 13B committed).
The index was empty; the only existing worktree change was the independent
MAVROS submodule modification. Ant Design is **5.29.3**, unchanged.
Baseline: **1,887 Vitest tests**, **189 ESLint warnings**, **zero errors**.

The source inventory confirmed the nine expected deprecated JSX properties:

| Location | Control | Before | After |
| --- | --- | ---: | ---: |
| NtripSection manual host | Input `addonBefore` | 1 | 0 |
| NtripSection manual user | Input `addonBefore` | 1 | 0 |
| NtripSection manual password | Input.Password `addonBefore` | 1 | 0 |
| NtripSection RTK2go email | Input `addonBefore` | 1 | 0 |
| NtripSection raw host/mountpoint/user | Input `addonBefore` | 3 | 0 |
| NtripSection raw password | Input.Password `addonBefore` | 1 | 0 |
| ParametersPage text parameter | Input `addonAfter` | 1 | 0 |
| **Total** | | **9** | **0** |

There were no remaining deprecated InputNumber addons or Modal destroyOnClose
properties at preflight. The final TypeScript JSX inventory finds none on Input,
Input.Password, InputNumber or Modal. The `addonAfter` metadata in gnssConfig,
Omit type names and warning assertions are not deprecated component usages and
remain unchanged.

## Implementation and ownership

`TextWithAddon` composes the public `Space.Compact` and `Space.Addon` APIs with the
original Input or Input.Password. The [public Space documentation](https://ant.design/components/space/)
identifies Addon as available since 5.29.0; installed 5.29.3 exports it. No internal
Ant Design import, dependency upgrade, CSS stylesheet or lint suppression is added.

A small discriminated prop union selects password mode and derives control props
from the public components. Values, handlers, IDs, ARIA, refs, disabled/status,
size, placeholder and password visibility options reach the original control.
An absent addon renders the original control directly. Layout styles (240/220px
width, fluid width, email maxWidth/margin) apply to the whole compact group; the
input flexes with minWidth zero, and addons retain their intrinsic width. The
existing wrapping Space, mobile tuner width and password visibility component
remain in place. There is no new validation, value conversion, state or saving
logic in the helper.

NtripSection changes only its eight render sites and imports. Provider selection,
credential defaults, sourcetable loading, RTCM/GNSS behavior, onChange keys and the
Settings manager save flow are unchanged. ParametersPage changes only its text
render site/import; its ordinary search Input is retained. Tier acknowledgment,
danger confirmation, Enter/blur/change handling, coercion and `/params` requests
are unchanged. No backend, ROS2, firmware, Universal GNSS, generated, dependency
or MAVROS file is modified. No staging, commit or push is performed.

## Regression coverage

Focused Vitest: **13 tests pass across three files** (12 new tests and the existing
numeric tuner regression):

- TextWithAddon: controlled empty/edited values, addons before/after, ID/ARIA,
  placeholder, status/size, imperative ref/focus, change/Enter/blur, Password
  visibility and focus, disabled text/password, inherited size, no-addon cases,
  complete group layout styles, Form.Item injection, required validation and
  exactly one valid submit with fictional credentials.
- NtripSection: provider and raw fields remain synchronized on mobile/desktop,
  both password controls retain masking, visibility toggles do not emit edits,
  the RTK2go email field retains its callback, and disabling NTRIP hides fields.
  Editing does not add a sourcetable request; the initial existing read is allowed.
- ParametersPage: the text branch retains exact input strings, Enter/blur require
  confirmation, Cancel restores the prior value without POST, explicit approval
  produces one POST, and a later blur does not repeat it. Expert inputs remain
  disabled until acknowledgment; opening, searching and acknowledgment do not
  write parameters. String-valued fixtures deliberately exercise the text branch;
  they do not claim numeric ROS parameters accept strings on a real backend.

Console warn/error spies call through and check for Input addon deprecations;
no global console masking is introduced. Tests use only fictional credentials,
no production credential logging is added, and no robot action is executed.

Targeted Chromium tests use 390px and 1440px viewports. They check fixed 240px,
fluid and max-380px groups, the full RTK2go email label, addon alignment and
viewport bounds. Password visibility retains focus, disabled inputs remain
unavailable, Form.Item submits, and actual NTRIP provider/raw controls stay
synchronized. The real Settings Save path emits one intercepted YAML POST with
the edited fictional password, and no YAML write occurs before Save. All robot
API/WebSocket traffic is mocked. The fixture reports only a saved flag, never
credentials. Browser console deprecations are observed without suppressing logs.

## Validation

- Full `yarn test --pool=threads --maxWorkers=2`: **1,899 Vitest tests passed in
  183 files**, 209.69 seconds, plus the appearance registry test.
- Targeted Chromium: **4 tests passed**, 7.3 seconds, using
  `E2E_PORT=5185 yarn playwright test tests/e2e/text-input-addons.spec.ts
  --workers=1 --reporter=line --output=/tmp/gui-batch13c-playwright-final`.
  No Input addon deprecation was observed.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-batch13c-dist`: passed.
- ESLint JSON before/after: **189 → 189 warnings**, **zero errors**. No new warning
  in helpers/tests; preexisting NtripSection/ParametersPage typing debt is unchanged.
- `yarn lint` (including unused-directive checks): passed, 189 warnings/zero errors.
- `git diff --check`: passed; index remains empty.
- Final deprecated JSX inventory: **zero** Input/Input.Password addons,
  **zero** InputNumber addons and **zero** Modal destroyOnClose.

| ESLint rule | Before | After |
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

## Harness findings and limits

A first browser measurement caught the modal's opening zoom transform instead of
its final dimensions. The fixture test now waits for the animation class to leave
before measuring, without sleeps or changing production animation. The Save
button's accessible name includes its icon; the locator matches its visible Save
text and change count. These were harness fixes, not changes to application logic.
The sandbox blocks loopback (EPERM), so the targeted browser run uses an authorized
local Vite server outside that restriction. Logs, screenshots and build outputs
remain in `/tmp`, outside version control.

The two preexisting Playwright failures in the
[Batch 13 report](gui-batch13-antd-inputnumber-migration.md) (MAVROS label assertion
and missing LiDAR drag target) are outside these targeted runs and are not claimed
fixed. Acceptance here covers Chromium at the two stated widths, not every browser
or locale. Existing large-chunk build warnings and jsdom pseudo-element
getComputedStyle notices remain. The browser Save scenarios also emit the Ant
Design v5/React 19 compatibility notice; the pinned versions are unchanged and
resolving their broader integration is outside this addon migration. This notice
is not suppressed and did not prevent the tested save flow. Runtime Ant Design
addon warnings are distinct
from ESLint typing warnings: eliminating the former does not reduce the latter.

Modified files: NtripSection.tsx, ParametersPage.tsx and its existing addons test;
new TextWithAddon.tsx and its test, NtripSection.addons.test.tsx, the text-addons
browser fixture HTML/TSX, text-input-addons.spec.ts, and this report.
