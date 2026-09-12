# gui/ — working notes for Claude

The operator web stack: a Go/gin HTTP+WebSocket server on `:4006` (`pkg/`) that fronts a React 19 SPA (`web/`), reaches ROS2 **only** through `foxglove_bridge` (`ws://localhost:8765`) plus the teleop relay (`ws://localhost:8766`), and owns the settings backend for the sparse installed `mowgli_robot.yaml`, the bitcask key-value DB, the scheduler, Docker-driven tools and firmware flashing.
It must NOT own robot behaviour: no autonomy, no localizer, no TF, no *default* values of its own — defaults live in the ROS2 template (root CLAUDE.md Invariant 15) and are mirrored into `asserts/mower_config.schema.json`; ROS-side logic belongs in `ros2/`, never re-implemented here.

## Read next

| File | Read it when… |
|------|----------------|
| [`../docs/claude/codemaps/gui_backend.md`](../docs/claude/codemaps/gui_backend.md) | Anything under `gui/pkg/` — routes, foxglove/CDR, settings, providers, tests |
| [`../docs/claude/codemaps/gui_frontend.md`](../docs/claude/codemaps/gui_frontend.md) | Anything under `gui/web/` — pages, hooks, map editor, settings sections, i18n |
| [`../docs/claude/ros-interfaces.md`](../docs/claude/ros-interfaces.md) | You need the real topic/service/QoS behind a GUI topic key or `call/:command` |
| [`../docs/claude/parameters.md`](../docs/claude/parameters.md) | Adding/editing a settings field — which yaml key, its template default, whether a launch file injects it (`INERT` keys do nothing) |
| [`../docs/claude/high-level-api.md`](../docs/claude/high-level-api.md) | Touching `HighLevelControl`/`HighLevelStatus` numbers, area recording, manual mowing, scheduler + IrriSense gate |
| [`../docs/claude/testing-ci.md`](../docs/claude/testing-ci.md) | Before pushing — which of these suites CI actually gates (Go and Playwright: none) |
| [`../docs/claude/commands.md`](../docs/claude/commands.md) | After changing a `.msg`/`.srv` — the full three-consumer codegen workflow |
| [`../docs/claude/contributing.md`](../docs/claude/contributing.md) | Commit/branch/PR conventions and per-language style |
| [`../docs/claude/codemaps/ci_repo_tooling.md`](../docs/claude/codemaps/ci_repo_tooling.md) | Editing `gui-ci.yml` / `gui-docker.yml` / `msg-codegen-drift.yml` |
| [`../docs/claude/doc-index.md`](../docs/claude/doc-index.md) | You don't know which doc covers a topic |
| [`web/tests/e2e/README.md`](web/tests/e2e/README.md) | Writing or debugging Playwright specs / adding a mock robot-state scenario |
| [`README.md`](README.md) | Deploy/podman notes only — its MQTT + env sections are OpenMower-era and stale |
| [`../wiki/GUI.md`](../wiki/GUI.md) | You need the operator-facing description of a screen |
| [`../wiki/Configuration.md`](../wiki/Configuration.md) | Operator-facing meaning of a settings field |
| [`../docs/UNIVERSAL_GNSS_SIDECAR_MIGRATION.md`](../docs/UNIVERSAL_GNSS_SIDECAR_MIGRATION.md) | Touching `GNSS_*` env derivation or the `mowgli-gps` sidecar routes |

## Build · test · run

```bash
# Go backend (gui/)
make deps                       # web: yarn; go mod download
make run-backend                # CGO_ENABLED=0 go run main.go   (serves :4006)
make build                      # docker build -t mowglinext .
go test ./...                   # 36 test files, ~30 s, no ROS/Docker needed — gated by gui-ci.yml go-tests
gofmt -l . && go vet ./...      # run gofmt locally; go test includes standard vet checks

# React frontend (gui/web/)
yarn install --frozen-lockfile
yarn dev                                              # vite :5173, /api proxied
MOWGLI_API_TARGET=http://<robot-ip>:4006 yarn dev     # against a live robot (see gotchas)
make run-gui                    # = yarn dev --host, from gui/
npx tsc --noEmit && yarn lint && yarn test            # exactly what gui-ci.yml gates
npx vitest run src/utils/gpsStatus.test.ts            # one unit file
yarn test:e2e                   # playwright, fully mocked; yarn test:e2e:ui for UI mode
npx playwright test -g "emergency-latched"

# Codegen (from gui/)
LC_ALL=C ./generate_go_msgs.sh && LC_ALL=C ./generate_ts_types.sh
cd web && yarn generate:api     # src/api/Api.ts from ../docs/swagger.json
```

## Conventions

- **Go:** stock `gofmt`; declare routes in a `*Routes(...)` fn registered from `pkg/api/api.go`, and annotate them with swaggo `// @Router` comments.
- **TypeScript:** eslint flat config (`web/eslint.config.js`) — the hard gate is **0 errors**; `--max-warnings 900` is a debt ratchet (lowering it is the point; raising needs a written reason). There is no prettier config in-repo despite `contributing.md`.
- **Never hand-edit:** `pkg/msgs/**/*_generated.go`, `web/src/types/ros.generated.ts` (`web/src/types/ros.ts` is the 1-line re-export every consumer imports), `web/src/api/Api.ts`, `gui/docs/{docs.go,swagger.json,swagger.yaml}`, `asserts/board.h` (the `.template` is the source). Re-run the generator instead.
- **`.msg`/`.srv` change** → both `generate_*.sh` here **and** `firmware/scripts/sync_ros_lib.py`; commit all three or `msg-codegen-drift.yml` fails. Use `LC_ALL=C` on macOS or sort order fabricates ~20 lines of phantom drift.
- **Swagger:** nothing in the repo runs `swag` — regenerate `gui/docs/` by hand after route changes, then `yarn generate:api` so `Api.ts` matches. Hand-written `guiApi.request({path})` calls (`/params`, `/settings/yaml/defaults`, `/tools/*`, `/irrisense/*`) bypass the generated client entirely.
- **i18n:** every string goes into `web/src/i18n/locales/en.json` **and** `fr.json` in lockstep — `locales.test.ts` asserts exact key parity in both directions.
- **Defaults:** a new `mowgli_robot.yaml` template default must be mirrored into `asserts/mower_config.schema.json` (or allowlisted in `pkg/api/schema_template_parity_test.go`), else `TestSchemaDefaultsMatchTemplate` fails and the Settings "at default" dot lies.

## Component-specific gotchas

- `getSchema` opens `asserts/mower_config.schema.json` **relative to the process CWD** (`pkg/api/settings.go`) — run the binary from `gui/` or every settings route 500s. Tests call `chdirToGuiRoot`.
- A yaml key with **no schema default is never pruned** once written (`sparsifyFlat`, `pkg/api/settings.go`) — that is why `retiredParamKeys` exists. `use_scan_matching` / `use_loop_closure` / `use_magnetometer` are written straight through and persist verbatim.
- `writePreservingPerms` keeps the yaml's uid/gid/mode so the ROS-side line-splice writers (root Invariant 6: dock pose, calibration, drive-tuning rollback) can still write it. Do not replace it with a plain `os.WriteFile`.
- A new browser topic needs **three** edits: `topicMap` (`pkg/providers/ros.go`), `topicSubscribeInterval` (`pkg/api/mowglinext.go`), and a `useTopic` wrapper. `dockingSensor` and MQTT's `mowingPath` are listed downstream but missing from `topicMap` — they silently never deliver.
- `useTopic`'s first argument is the backend **topic key** (`fusionRaw`, `mowProgress`), not a ROS topic name; an invented key yields no error and no data (`web/src/hooks/useTopic.ts`).
- `multiplexedSocket.ts` is a module singleton with ref-counted subscriptions — a `useEffect` that subscribes without returning the unsubscribe leaks a server-side ROS subscription for the tab's life.
- Two different wire formats: `/mowglinext/multiplex` frames are **msgpack binary**, `/mowglinext/subscribe/:topic` frames are **base64 text**. `/mowglinext/publish/:topic` ignores its path param and always publishes `/cmd_vel_teleop`.
- WebSocket `CheckOrigin` demands `Origin` host == `Host` exactly and there is **no auth layer anywhere** — dev against a robot must go through the vite proxy (`MOWGLI_API_TARGET`, `web/vite.config.ts`); `VITE_API_HOST` alone breaks every stream, and a Host-rewriting reverse proxy kills all WS routes.
- `foxglove.CallService` fails with "service not advertised" until the bridge has sent `advertiseServices`, and channel/service tables are wiped on reconnect (`pkg/foxglove/client.go`) — `initMapPolling` deliberately sleeps 5 s before its first `pollMap`.
- Service responses are inconsistent by design: some return `{message}`, others `OkResponse` (`pkg/api/mowglinext.go`); `useMowerAction` depends on that shape — don't "normalise" one side only.
- Settings Save sends **only dirty keys** (`useSettingsManager.persistSettings`) so concurrent writers (dock calibration, "set docking point") are not clobbered; `AdvancedSection` deletes are `null` values, pruned from local state after save.
- `SafetySection` claims `lift_recovery_mode` / `lift_blade_resume_delay_sec` in `SECTION_DEFINITIONS` but deliberately does not render them — the claim is what keeps them out of the free-form `AdvancedSection`.
- `useMapStreams` depends on `settings["datum_lat"]/["datum_lon"]`, **not** the whole `settings` object; widening that dependency re-creates the re-subscribe storm that made every map stream look stale.
- The theme is hard-locked to dark (`web/src/theme/ThemeContext.tsx`); `toggleMode` is an intentional no-op and the light tokens are unused. Display mode (Visual/Balanced/Efficient) changes render cadence only — never subscriptions or robot-side rates.
- `metersPerDegreeLat` (`pkg/api/openmower_import.go`) must equal `METERS_PER_DEG` (`web/src/utils/map.tsx`); the mow-progress rasterisers on both dashboard and map follow root Invariant 14 — a mis-sized raster shows as a 90°-rotated overlay.
- Container names are hardcoded: `mowgli-gps` (`pkg/api/gnss.go`), `mowgli-ros2` (`pkg/api/rosbag.go`, `pkg/api/drive_tuning.go`); renaming them in `install/compose/*.yml` silently breaks those tools.
- `main.go` panics if the `system.homekit.enabled` / `system.mqtt.enabled` DB keys are unreadable, and `NewDBProvider` panics on a non-recoverable bitcask error.
- Playwright specs live under `web/tests/e2e/` and are explicitly excluded by `vitest.config.ts` — never co-locate a `.spec.ts` under `web/src/`. Add a robot state to `tests/e2e/mock/scenarios.ts` and every page spec picks it up.
- `gui/openmower-gui` is a tracked 40 MB build artifact — do not rely on it and do not re-commit it.

## Safety

This component can move the robot and rewrite firmware — root CLAUDE.md **§ Safety** applies here in full, so never add a GUI-side "override" or bypass. Treat these paths as safety-critical in review: STM32 flashing (`pkg/providers/firmware.go` — sha256 + openocd `verify` + post-flash protocol check; only the expert path may set `DisableEmergency`), `POST /system/{reboot,shutdown}` (`nsenter -t 1 … systemctl` on the **host**), teleop publishing to `/cmd_vel_teleop`, `mow_enabled` / `emergency` / `high_level_control` service calls, drive PID/FF tuning that persists into `mowgli_robot.yaml`, and GNSS `apply` / `factory-reset-apply` (destructive to the receiver, gated on explicit confirm flags). There is no authentication on `:4006` — assume any reachable client can invoke all of the above.
