# Hardware backend identity and parameter routing integrity — Batch 7B

Software review baseline: `fix/gui-warnings-errors`, HEAD
`f56774ea4e2bb5d799efa7a0a0f6bcd3bdeae0b6`, plus the uncommitted changes
listed below. This report establishes GUI/API behavior, not physical hardware
identity or runtime-image compatibility.

## Findings and authority

Previously, both frontend and Go normalization promoted unsupported selections to
Mowgli. The frontend also supplied native live routes while loading or after a
metadata failure. Live-key classification was separate from value conversion:
`Number(null)` and `Number("")` became zero, and an invalid value could be counted
as applied. A successful YAML write was reported before the ROS response, without
reliable confirmation. Independently, YAML could be saved before a sidecar/env
write failed, causing the browser to incorrectly keep saved changes dirty.

Installer authority is preserved. `install/compose/docker-compose.gui.yml`
mounts the installer `docker/` directory at `/runtime_config` and sets
`MOWER_RUNTIME_ENV_FILE=/runtime_config/.env`. `gui/pkg/providers/db.go` resolves
`system.mower.runtimeEnvFile` using a nonempty stored DB value first, then its
process environment fallback, then `/runtime_config/.env`. The selected file's
`HARDWARE_BACKEND` takes precedence over process `HARDWARE_BACKEND`. If the file
has no such key, process selection applies. An absent/empty selection retains
the installer legacy Mowgli default; an explicit unsupported value is unresolved.
File read/parse errors and unavailable DB paths propagate instead of becoming
Mowgli. The existing missing-file loader contract still allows process/legacy
selection; this is not proof of installed or running hardware.

`GET /settings/hardware-backend` returns 503 with `state: unsupported` or
`state: unavailable` when resolution fails. Successful identity describes the
configured backend. The browser parses an unknown response and distinguishes
loading, confirmed, absent legacy endpoint (404/405), unavailable API/network,
malformed response and unsupported selection. All unconfirmed states have no
live routes and authorize no model presets. Legacy installations retain YAML
editing, generic ROS operations and independent external savers where their
existing APIs support them.

## Route contract and types

Backend-wide `runtime_routing` is compiled routing capability, not observed node
presence. The current Go producer returns `available`. Each route independently
has `available`, `pending_image` or `restart_required`; the parser preserves all
three and rejects unknown states. Missing routing metadata means unavailable
live routing even when identity can be displayed.

| Backend / canonical YAML key | ROS target | Type / application |
| --- | --- | --- |
| Mowgli `ticks_per_meter` | `hardware_bridge.ticks_per_meter` | double / live |
| Mowgli `wheel_pid_kp`, `wheel_pid_ki`, `wheel_pid_kd`, `wheel_pid_integral_limit`, `wheel_pid_pwm_per_mps` | `hardware_bridge.<same key>` | double / live |
| MAVROS `ticks_per_meter` | `mavros/esc_wheel_odometry.ticks_per_meter` | double / live |
| MAVROS `wheel_track` | `mavros/esc_wheel_odometry.track_width_m` | double / live |
| MAVROS `wheel_pid_pwm_per_mps` | `hardware_bridge.manual_control_linear_scale` | double / live |
| MAVROS `mowing_enabled` | `hardware_bridge.mowing_enabled` | boolean / restart required |
| OpenMower | none | no fabricated live routes |

MAVROS targets were checked against their existing bridge and wheel-odometry
consumers. No route, hardware implementation or firmware parameter was renamed.
Generated backend defaults contain OpenMower overrides; native/MAVROS empty
override objects are legitimate. OpenMower presets require the supplied positive
`ticks_per_meter` override. Malformed numeric geometry overrides reject metadata.

One validated plan supplies both outgoing parameters and the live-key set used
for restart classification. It requires confirmed identity, backend-wide
availability, an available canonical route and a valid value. Double values may
be finite numbers or fully parsed decimal strings; null, empty, undefined,
booleans, partial numeric strings, NaN and infinity cannot become double writes.
Zero is preserved. Boolean validation accepts actual booleans, including false;
no current boolean hardware route is available live. Strings and arrays remain
legitimate types for unrelated generic ROS parameter operations.

Go resolves installer identity per `/params` request, rejects unresolved hardware
namespaces, cross-backend known hardware targets, STM32 PID routes on nonnative
backends and restart-only routes. Finite double validation and returned-value
confirmation protect authorized hardware writes. Other ROS node namespaces retain
the owning node/provider's validation. Protected hardware namespaces use the
explicit direct-operation exceptions described below; there is no unrestricted
bridge fallback. PID middleware requires Mowgli; shared
feed-forward/rollback middleware admits only the two implemented backends,
Mowgli and MAVROS. Backend-aware YAML defaults/pruning rejects unresolved
identity instead of pruning against native defaults.

## Persistence outcomes and limits

YAML remains canonical. Dirty snapshots are marked saved after confirmed YAML
persistence, and edits made during that request remain dirty. Nonfinite numeric
settings are rejected before JSON can silently encode them as null. Runtime
application has its own confirmation and notification. A rejected/missing/wrong
ROS echo leaves YAML saved and sets restart required. Deferred routing messages
distinguish unconfirmed identity, unavailable routing, pending image and restart
required. External savers remain independent.

Post-YAML sidecar/env errors retain HTTP 500 and add `persisted: true` to their
error body. Both resolved error responses and generated-client rejections are
handled without interpreting error text as proof of persistence. The GUI marks
YAML saved, warns, requires restart and skips subsequent runtime application.
Partial-save consumers receive failure for incomplete synchronization while
retaining the persisted keys. Failures before the YAML write keep edits dirty.
Older servers without that explicit marker cannot reliably communicate partial
persistence; the browser conservatively treats their error as unconfirmed save.

There is no transaction across YAML, sidecars, env and ROS. A transport failure
can still hide a completed write. Metadata is a request snapshot; no runtime
epoch, physical backend attestation, automatic freshness guarantee or atomic
installer-change lock is introduced. Server validation rechecks configuration
at request entry. Node availability/image support ultimately depends on the
provider response. A restart indicator is advice, not confirmation of a restart
or physical application.

## Changed files

- API: `gui/pkg/api/{settings_backend.go,settings.go,params.go,drive_tuning.go}`.
- API regression coverage: `gui/pkg/api/{hardware_backend_integrity_test.go,settings_backend_test.go,settings_test.go,params_test.go}`.
- Frontend authority/planning: `gui/web/src/{constants/hardwareBackends.ts,hooks/useHardwareBackend.ts,hooks/useSettingsManager.ts}`.
- UI: `gui/web/src/components/settings/{HardwareBackendCard.tsx,HardwareSection.tsx}` and `gui/web/src/pages/{SettingsPage.tsx,OnboardingPage.tsx}`.
- Frontend regressions: `gui/web/src/constants/hardwareBackends.test.ts`, `gui/web/src/hooks/{useHardwareBackend.test.tsx,useSettingsManager.merge.test.tsx}`, `gui/web/src/components/settings/HardwareSection.backend.test.tsx`, and metadata fixtures in `gui/web/src/pages/MowgliNextPage.firmware.test.tsx`.
- Locales: `gui/web/src/i18n/locales/{en.json,fr.json}`; this report.

Existing local MAVROS submodule changes were preserved. Generated definitions,
production ROS, firmware and hardware routing implementations were not edited.

## Validation

All commands ran as the normal project user. Temporary logs/build output are in
`/tmp/gui-7b`, outside version control.

- Focused authority/planning/settings tests: 89 passed; preset interaction tests: 2 passed. Earlier focused checks also covered locale parity and existing firmware-page metadata fixtures.
- Initial full `yarn test`: appearance-registry test passed; Vitest **1,534 tests across 165 files passed**. Review-follow-up validation is recorded below.
- Full GUI Go `GOCACHE=/tmp/gui-7b/go-cache go test ./...`: passed, including API, providers and Foxglove packages and a real persisted-YAML/failed-sidecar regression. Local test servers required loopback socket access.
- `yarn lint`: **356 warnings before, 356 after; zero errors**. No lint suppression was added.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-7b/dist`: passed; existing large-chunk advisory remains.
- `gofmt -l` on all touched Go files: empty output.
- `git diff --check`: passed.

No commit, push, merge or hardware operation was performed.

## Review follow-up: partial notifications and namespace authorization

A partial sidecar/env failure now emits only the partial-persistence warning,
after its response arrives. It returns before normal success, routing advice or
live application. Saved snapshots, restart state and concurrent edits are
unchanged. Deferred routing emits a warning instead of a simultaneous normal
success. Fully synchronized saves report YAML persistence separately from live
confirmation or runtime rejection. Tests cover resolved and rejected partial
responses, notification ordering and concurrent edits during partial persistence.

The direct `/params` fallback previously allowed unknown bridge parameters on
recognized backends. Classification now protects node basenames
`hardware_bridge` and `esc_wheel_odometry` even under additional ROS namespaces.
Unknown targets, startup-only parameters and cross-backend targets are rejected
before any parameter in the request batch is dispatched. Canonical routes retain
their existing availability/type checks.

Explicit compatibility exceptions preserve actual dynamic consumers:

| Configured backend | Additional direct targets | Value type |
| --- | --- | --- |
| Mowgli | `hardware_bridge.min_linear_vel`, `hardware_bridge.yaw_kp`, `hardware_bridge.yaw_ki`, `hardware_bridge.yaw_trim_limit_mps` | finite double |
| MAVROS | `hardware_bridge.manual_control_yaw_scale` | finite double |
| MAVROS | `hardware_bridge.right_esc_slot`, `hardware_bridge.left_esc_slot`, `hardware_bridge.blade_esc_slot` | integer / int64 |
| All three recognized backends | `hardware_bridge.use_sim_time` | boolean |
| MAVROS | `mavros/esc_wheel_odometry.use_sim_time` | boolean |

The native bridge's on-set callback implements the listed native additions.
MAVROS validation and post-set callbacks implement the listed MAVROS additions;
consumer-side range and overlapping-role checks remain authoritative. These
exceptions are direct API capabilities, not extra canonical YAML routes or
metadata-derived GUI writes. `use_sim_time` is the standard rclcpp node clock
parameter; its explicit exception does not permit arbitrary bridge settings.
Unresolved identity authorizes no writes in these protected namespaces,
including these exceptions. Unrelated generic ROS namespaces remain available
for all identities, including unresolved ones, with strings, arrays and booleans
preserved.

The OpenMower bridge snapshots hardware parameters at startup and has no dynamic
hardware parameter callback. It therefore receives no added live hardware
exception; existing YAML/restart behavior remains its configuration path.
Direct attempts to set startup-only bridge settings now fail explicitly instead
of potentially echoing a changed ROS parameter while the node keeps its cached
value. New bridge capabilities or renamed/namespaced deployments require an
explicit compatibility rule and test; unknown parameters no longer gain implicit
authority. No ROS consumer, firmware, hardware safety behavior or submodule was
modified by this follow-up.

Follow-up validation: focused settings-manager tests **29 passed**; focused Go
`TestBackendIntegrity|TestSetParams` passed. Full `yarn test` passed the appearance
registry and **1,535 Vitest tests across 165 files**. Full GUI `go test ./...`,
ESLint (**356 warnings before and after, zero errors**), TypeScript, frontend
build, Go formatting and `git diff --check` passed. The build retains the existing
large-chunk advisory. Follow-up edits are limited to `params.go`,
`hardware_backend_integrity_test.go`, `useSettingsManager.ts`,
`useSettingsManager.merge.test.tsx` and this report. All earlier Batch 7B changes
and existing local MAVROS modifications were retained; nothing was committed
or pushed.
