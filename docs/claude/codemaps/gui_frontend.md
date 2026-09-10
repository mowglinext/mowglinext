# Codemap: gui_frontend

> The React 19 + TypeScript single-page app served by the Go backend on port 4006 (`gui/web/`). It owns every operator-facing screen — dashboard, map editor, schedule, statistics, settings, live ROS parameters, diagnostics, logs, onboarding — plus the shared hook layer that turns the backend's ONE multiplexed WebSocket into typed React state. It talks only to the Go backend (`/api/...`); it never speaks rosbridge/foxglove directly.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `gui/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Add a route / page | `gui/web/src/main.tsx` L15–24 (`React.lazy` imports) + L26–75 (`createHashRouter`); then `components/AppShell.tsx` `NAV` L46–56 and `PAGE_META` L60–70 |
| Subscribe to a new ROS topic | `hooks/useTopic.ts` (`useTopic<T>(topicKey, initial, {throttleMs, withTimestamp, select})`) — copy a one-liner like `hooks/useGPS.ts`. The topic KEY is a backend alias, not a ROS topic name (see Runtime surface) |
| Debug "no live data / stale" | `hooks/multiplexedSocket.ts` (single `ws://<host>/api/mowglinext/multiplex`, msgpack binary frames, ref-counted subscribe/unsubscribe, exponential reconnect ≤30 s); `useMultiplexStatus()` in `hooks/useWS.ts` L20–24 |
| Call a ROS service from the UI | `components/MowerActions.tsx` `useMowerAction()` L22–33 → `guiApi.mowglinext.callCreate(command, args)`; command names listed under Runtime surface |
| Settings: add a field to an existing section | `hooks/useSettingsManager.ts` `SECTION_DEFINITIONS` L48 (add the yaml key to that section's `keys`), then the section component in `components/settings/`, then `i18n/locales/{en,fr}.json` |
| Settings: the "overridden dot" / reset-to-default | `hooks/useSettingsManager.ts` `hasDefault` L328 / `isDefault` L333 / `isOverridden` L339 / `resetToDefault` L348 (defaults come from `GET /api/settings/yaml/defaults`, L288–299); UI in `components/settings/SettingFieldLabel.tsx`; equality in `utils/settingsValues.ts` `valuesMatch` |
| Settings: what happens on Save | `hooks/useSettingsManager.ts` `persistSettings` L420 — sends ONLY dirty keys, prunes nulls, then optional GPS-container restart, dock-pose push (`mapDockingCreate`), and live `hardware_bridge.*` param push via `POST /api/params` |
| Add a settings section | `SettingsSection` union L21–38 + `SECTION_DEFINITIONS` L48 in `hooks/useSettingsManager.ts`; render arm in `pages/SettingsPage.tsx` L121–241; nav entry via `components/settings/SettingsNav.tsx` |
| Map editor: draw/split/undo behaviour | `pages/map/hooks/useMapEditing.ts` (pure helpers `segSegIntersect` L28, `getCutterBetween` L45, `inside` L70, `sortFeatures` L148); undo/redo `useMapEditHistory.ts`; custom Mapbox-draw modes `src/modes/SplitLineMode.ts`, `src/modes/DirectSelectWithBoxMode.tsx` |
| Map: which streams start/stop and when | `pages/map/hooks/useMapStreams.ts` L384–415 (edit-mode teardown), L417–450 (joy + recording on `RECORDING`/`MANUAL_MOWING`, 1200 ms teardown debounce), L462–479 (start once datum known) |
| Map: save / load / OpenMower import | `pages/map/hooks/useMapFiles.ts` — `putMowglinext` L168, `mapDockingCreate` L181, `POST /api/import/openmower` L335/L379/L427, datum write `POST /api/settings/yaml` L441 |
| Manual mowing / joystick | `pages/map/hooks/useManualMode.ts` (caps `MAX_LINEAR_MPS = 0.25`, `MAX_ANGULAR_RAD_S = 0.6`, `JOY_SEND_INTERVAL_MS = 100`, `MANUAL_EXIT_DEBOUNCE_MS = 1200`) + `pages/map/components/JoystickOverlay.tsx` |
| Fusion-graph diagnostics panel | `pages/DiagnosticsPage.tsx` `sectionFusionGraph` L906–1021 (keys parsed at L867–902), service buttons `callFusionService` L832; feed `hooks/useFusionGraphDiagnostics.ts` |
| Add a diagnostics panel | `pages/DiagnosticsPage.tsx` — write a `sectionX` const (list at L455–1823), then register it in the mobile `Collapse` L1872–1924 AND the desktop `tabItems` L1933+ |
| GPS/RTK status derivation | `utils/gpsStatus.ts` (`deriveGpsStatus`, `gnssRtkModeLabel`, `deriveGnssStatusFromDiagnostics`, …) — consumed by `hooks/useGnssStatus.ts`, which falls back to `/diagnostics` when no typed `gnssStatus` sample has arrived |
| Display modes (Visual / Balanced / Efficient) | `theme/ThemeContext.tsx` L5–8, `readDisplayMode` L14 (localStorage `mowgli.display-mode`, default `balanced`), consumed by `components/settings/DisplayModeSection.tsx` and the render budgets in `pages/map/hooks/mapRenderBudget.ts` |
| Add/translate a UI string | `src/i18n/locales/en.json` **and** `fr.json` in lockstep (97 top-level namespaces, 3233 lines each); loader `src/i18n/index.ts`; parity guard `src/i18n/locales.test.ts` |
| Regenerate ROS message types | `cd gui && ./generate_ts_types.sh` → writes `gui/web/src/types/ros.generated.ts` (`src/types/ros.ts` is a one-line re-export). CI gate: `.github/workflows/msg-codegen-drift.yml` |
| Onboarding wizard step order / readiness gate | `components/onboarding/steps.ts` (`STEP_*` indices, `STEP_COUNT = 9`), `components/onboarding/readinessChecks.ts` `computeReadinessChecks` L269 |
| Reproduce a robot state without hardware | `tests/e2e/mock/scenarios.ts` `SCENARIOS` L45+ (8 permutations) + `tests/e2e/mock/mockBackend.ts` (`page.route` REST, `page.routeWebSocket` msgpack multiplex) |
| Where a container gets restarted from the UI | `utils/containers.ts` (`restartRos2`, `restartGui`, `restartGps`, `restartMowgliStack`, `GPS_RESTART_KEYS` L102) + `hooks/useContainerRestart.ts` (probe socket waits for the first live frame) |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **entry / build config** | | |
| `gui/web/index.html` | 50 | Dark shell (`#02110D`), PWA manifest, Mapbox CSS, font links (Satoshi, Instrument Serif, Space Grotesk, JetBrains Mono) |
| `gui/web/src/main.tsx` | 148 | Hash router (10 routes, all `React.lazy`), AntD dark `ConfigProvider`, `ThemeProvider`/`NotificationCenterProvider`/`TimeFormatProvider`/`MotionConfig` |
| `gui/web/package.json` | 77 | scripts `dev/build/lint/test/test:e2e/generate:api`; React 19, antd 5, mapbox-gl 3, msgpackr, i18next, framer-motion, vite 8, vitest 4 |
| `gui/web/vite.config.ts` | 35 | `/api` proxy → `MOWGLI_API_TARGET` (default `http://localhost:4006`), rewrites `Origin`/`Host` so the backend's WS origin check passes |
| `gui/web/vitest.config.ts` | 29 | jsdom, globals, `src/test/setup.ts`, `testTimeout: 20000`, excludes `tests/e2e/**` |
| `gui/web/playwright.config.ts` | 35 | testDir `tests/e2e`, chromium only, 1440×900, auto-starts `vite --port 5173` |
| `gui/web/eslint.config.js` | 178 | Flat config; "debt ratchet" block L63+ demotes rules to warn (hard gate = 0 errors) |
| `gui/web/tsconfig.json`, `tsconfig.node.json`, `.env.example`, `.npmrc`, `public/` | 28/12/1/3/+svg | TS project refs; `VITE_MAPBOX_TOKEN` sample; yarn registry; PWA `manifest.json` (9) + favicon/logo SVGs |
| **`src/api/`, `src/types/`** | | |
| `src/api/Api.ts` | 921 | GENERATED swagger client (`yarn generate:api` from `gui/docs/swagger.json`); namespaces `mowglinext`, `settings`, `config`, `containers`, `schedules`, `setup`, `system` |
| `src/types/ros.generated.ts` | 487 | GENERATED ROS message types (snake_case) from `ros2/src/mowgli_interfaces/msg` |
| `src/types/ros.ts` | 1 | `export * from "./ros.generated.ts"` — the import path every consumer uses |
| `src/types/map.ts` | 440 | Map feature class hierarchy (`MowingAreaFeature`, `NavigationFeature`, `ObstacleFeature`, `DockFeatureBase`, `PathFeature`, `RobotPartFeature`, `DynObstacleFeature`) + `serializeFeatures`/`featuresFromJSON` (prototype-preserving round-trip) |
| `src/types/irrisense.ts`, `src/types/mapbox-gl-draw.d.ts`, `src/global.d.ts`, `src/vite-env.d.ts` | 76/72/12/1 | IrriSense DTOs; ambient typings |
| **`src/pages/` — one per route** | | |
| `pages/MowgliNextPage.tsx` | 711 | `/mowglinext` dashboard: state-adaptive hero, battery ring, live mini-map, progress ribbon, weather chip, soil banner |
| `pages/MapPage.tsx` | 1227 | `/map` Mapbox GL editor + live overlay; `MAPBOX_TOKEN` L46; `editMap` toggle L87 drives stream teardown |
| `pages/DiagnosticsPage.tsx` | 1979 | `/diagnostics` — 10 panels (system, localization, fusion_graph, heading sources, BT+coverage, cross-checks, calibration, sensors, rosbag, raw `/diagnostics`) rendered as `Collapse` on mobile / tabs on desktop |
| `pages/SettingsPage.tsx` | 427 | `/settings` shell: nav + search + one Save/Revert bar; section switch L121–241 |
| `pages/SchedulePage.tsx` | 497 | `/schedule` weekly grid, CRUD on `/api/schedules`, subscribes `map` for zone names, IrriSense chip |
| `pages/StatisticsPage.tsx` | 395 | `/statistics` — `/api/diagnostics/sessions{,/stats}`, `YearOfLawn` heatmap, per-zone bars |
| `pages/ParametersPage.tsx` | 323 | `/parameters` LIVE ROS param editor (`GET`/`POST /api/params`); tier filter + danger-confirm (`DANGER_RE` L36) |
| `pages/LogsPage.tsx` | 427 | `/logs` container picker + live tail (`/api/containers/{id}/logs`), level filter, `MAX_LINES = 5000` |
| `pages/OnboardingPage.tsx` | 1260 | `/onboarding` 9-step wizard (model → firmware → NTRIP → GPS → datum → sensors → calibration) |
| `pages/MapStyle.tsx`, `pages/logBatcher.ts` | 345/48 | Mapbox layer/style definitions; log-line batcher (100 ms coalescing) |
| **`src/pages/map/`** | | |
| `map/hooks/useMapStreams.ts` | 516 | Owns the 9 map streams + mow-progress raster + LiDAR/pose render throttles |
| `map/hooks/useMapEditing.ts` | 1154 | Draw/split/merge/shape-stamp editing state machine + pure geometry helpers |
| `map/hooks/useMapFiles.ts` | 576 | Save/load map, set dock pose, OpenMower import (preview + apply) |
| `map/hooks/useMapEditHistory.ts` / `useMapOffset.ts` / `useMapBearing.ts` / `useManualMode.ts` / `useLatestThrottle.ts` / `mapRenderBudget.ts` / `useResetMowingProgress.tsx` | 118/57/71/127/86/15/53 | Undo stack (10 entries); GUI-config map offset (`gui.map.offset.x/y`); bearing (`gui.map.display.bearing`); teleop latch; trailing-edge throttle; per-display-mode pose/lidar intervals; "reset mowing progress" confirm |
| `map/components/` (11 files) | 52–442 | `MapToolbar`, `MapToolbarMobile`, `MapEditorToolbar`, `AreasListPanel`, `TrackedObstaclesPanel`, `NewAreaModal`, `EditAreaModal`, `ImportOpenMowerModal`, `MapOffsetPanel`, `ShapePickerDropdown`, `JoystickOverlay` |
| `map/utils/emojiToPolygon.ts`, `map/utils/types.ts` | 135/26 | Emoji → polygon ring (marching squares + RDP); `MowingAreaEdit`, `AreaListItem` |
| **`src/hooks/` — transport** | | |
| `hooks/multiplexedSocket.ts` | 273 | THE shared WebSocket: JSON `{op,topic}` up, msgpack `{topic,data}` down; ref-counted per topic, reconnect backoff to 30 s, rate-limited decode warnings |
| `hooks/useTopic.ts` | 93 | Typed subscribe + throttle/select/timestamp wrapper over the multiplexer |
| `hooks/useWS.ts` | 153 | Legacy `start(uri)/stop()` API; subscribe URIs go through the multiplexer, `/publish/...` keeps a dedicated base64 socket. Also `useMultiplexStatus()` |
| `hooks/useApi.ts`, `utils/apiHost.ts` | 9/30 | One shared `Api` instance (`baseUrl = "/api"`); `httpBase()` / `wsBase()` |
| **`src/hooks/` — topic hooks** (thin `useTopic` wrappers unless noted) | | |
| `useGPS` `usePose` `useStatus` `usePower` `useImu` `useWheelTicks` `useEmergency` `useDockingSensor` | 4 each | topic keys `gps`, `pose`, `status`, `power`, `imu`, `ticks`, `emergency`, `dockingSensor` |
| `useMowingMap.ts` / `useMowProgress.ts` | 10/12 | `map`; `mowProgress` (client throttle 1000 ms on top of the backend's 500 ms cap) |
| `useFusionOdom.ts` / `useIcpOdom.ts` / `useWheelOdom.ts` | 31/11/27 | `fusionRaw` (throttle 200 ms), `icpOdom`, `wheelOdom` |
| `useCogHeading.ts` / `useMagYaw.ts` | 15/14 | `cogHeading`, `magYaw` — both `withTimestamp` for staleness display |
| `useCoverageResumeAvailable.ts` | 12 | `coverageResumeAvailable` (`std_msgs/Bool` → `select` unwraps `.data`) |
| `useHighLevelStatus.ts` / `useDiagnostics.ts` / `useFusionGraphDiagnostics.ts` / `useGnssStatus.ts` / `useBTLog.ts` / `useDockCalibration.ts` / `useRobotDescription.ts` | 19/73/67/42/66/95/189 | `useWS`-based (not `useTopic`): `highLevelStatus`, `diagnostics` (accumulates by name, `DIAGNOSTIC_STALE_MS = 30_000`), `fusionDiag`, `gnssStatus` (+ `/diagnostics` fallback), `btLog`, `dockCalibrationStatus`, `robotDescription` (URDF → robot silhouette geometry) |
| `useWheelRpm.ts` / `useFirmwareStatus.ts` / `useValueSince.ts` | 91/25/17 | Tick differentiation (`DEFAULT_WHEEL_RADIUS_M = 0.04475`, rear-axle only); `/status` firmware handshake projection; "value unchanged since" clock |
| **`src/hooks/` — REST / state hooks** | | |
| `useSettingsManager.ts` | 732 | Settings page engine: 17 sections, dirty tracking, defaults, external savers, save orchestration |
| `useSettings.ts` / `useSettingsSchema.ts` / `useConfig.tsx` / `useEnv.tsx` | 256/155/48/27 | Shell+DB settings merge (`SettingsDesc` catalog L47); JSON schema for onboarding forms; GUI key-value DB; container env |
| `useDiagnosticsSnapshot.ts` / `useCalibrationStatus.ts` / `useImuYawCalibration.ts` / `useDriveTuning.ts` / `useRosbag.ts` / `useGnssRuntimeConfig.ts` / `useSoilStatus.ts` / `useWeather.ts` / `useFirmwareDebugLogs.ts` / `useContainerRestart.ts` | 75/75/182/220/131/77/52/37/130/107 | `/diagnostics/snapshot`; `/calibration/status`; `/api/calibration/imu-yaw`; `/tools/drive/*`; `/tools/rosbag/*`; `/settings/gnss/runtime-config`; `/irrisense/status`; `/weather`; container log tail; restart + readiness probe |
| `useNotificationCenter.tsx` / `useTimeFormat.tsx` / `useIsMobile.ts` / `useIOSInstallPrompt.ts` | 143/100/17/38 | App-wide notification buffer + `useAutoNotifications`; local/UTC timestamp preference; breakpoint; iOS A2HS banner |
| **`src/components/`** | | |
| `AppShell.tsx` | 553 | Layout, side rail / bottom nav, page meta, onboarding redirect (`GET /api/settings/status` L101) |
| `MowerActions.tsx` / `MowerStatus.tsx` / `LiveStatusStrip.tsx` / `NotificationBell.tsx` / `LanguageSwitcher.tsx` / `IOSInstallBanner.tsx` | 254/326/55/219/63/75 | Command buttons + `useMowerAction`; status pill (`/api/system/info`); shell strip; notifications drawer; language picker; PWA banner |
| `BTStateGraph.tsx` / `RobotAnatomy.tsx` / `TelemetryStat.tsx` | 319/239/81 | Diagnostics visualisations |
| `RobotComponentEditor.tsx` / `FlashBoardComponent.tsx` | 1123/649 | 3D-ish sensor placement editor (Sensors section + onboarding); STM32 flashing UI |
| `DrawControl.tsx` / `YearOfLawn.tsx` / `AsyncButton.tsx` / `AsyncDropDownButton.tsx` / `Spinner.tsx` / `StyledTerminal.tsx` / `utils.tsx` | 167/250/33/47/17/22/67 | mapbox-gl-draw React wrapper; statistics heatmap; async-aware buttons; misc |
| `components/dashboard/` (7 files) | 5–391 | Barrel `index.ts` re-exports `DashCard`, `ActionButton`, `Bar`, `Icons`, and `MOWER_STATES`/`fmt`/`KEYFRAMES_CSS`/`FONT`/`DISPLAY_FONT`/`MONO_FONT` from `constants.ts`; plus `SoilWetBanner` |
| `components/gnss/` (5 files) | 9–421 | `GnssLiveDiagnosticsCard` (Diagnostics), `GnssLiveStatusSummaryCard` (onboarding), `GnssDiagnosticBarRow`, `gnssFormatting`, `gnssPresentation` |
| `components/onboarding/` (3 files) | 14–306 | `steps.ts` (`STEP_*`), `readinessChecks.ts` (pure gate logic), `ReadinessStep.tsx` |
| `components/schedule/IrriSenseStatusChip.tsx` | 64 | Soil-gate chip on the Schedule page |
| **`src/components/settings/`** | | |
| `SettingsNav.tsx` / `SettingsPreview.tsx` / `SettingFieldLabel.tsx` | 117/265/71 | Section nav; live SVG preview of the edited chassis/tool/battery values; overridden-dot + reset button |
| Section components: `HardwareSection` `DriveMotorSection` `NtripSection` `PositioningSection` `SensorsSection` `LocalizationSection` `MowingSection` `DockingSection` `BatterySection` `SafetySection` `ObstaclesSection` `NavigationSection` `RainSection` `LedsSection` `IrriSenseSection` `AdvancedSection` `DisplayModeSection` `LogTimeZoneSection` | 64–964 | One per `SettingsSection` id, except `appearance`, which renders `DisplayModeSection` + `LogTimeZoneSection` (18 components for 17 ids); `DriveMotorSection` (964) also hosts PID/FF tuning runs |
| `gnssConfig.ts` / `ntripProviders.ts` / `GnssReceiverActionsCard.tsx` / `GnssSerialDeviceConfigField.tsx` / `UniversalGnssAdvancedSettings.tsx` / `GnssSignalProfileHelp.tsx` / `NtripStationMap.tsx` | 365/86/501/201/197/42/138 | Receiver families/profiles, NTRIP provider presets + station map, receiver actions `POST /settings/gnss/{plan,apply,factory-reset-apply,restart}` (`GnssReceiverActionsCard.tsx` L162–180) |
| `DockCalibrationCard.tsx` / `IrriSenseConnectionCard.tsx` / `IrriSenseRuleCard.tsx` / `IrriSenseStatusLine.tsx` | 102/129/75/59 | One-click dock calibration; IrriSense token/garden/rule UI |
| `paramCatalog.ts` | 142 | Curated tier (`basic`/`middle`/`expert`) + group + unit for ~45 ROS params; unknown params default to expert/"Other" |
| **`src/theme/`, `src/i18n/`, `src/concept/`, `src/modes/`, `src/constants/`** | | |
| `theme/ThemeContext.tsx` / `theme/colors.ts` | 84/248 | Dark-locked provider (`toggleMode` is a no-op) + display mode; palette and `cssVars()` |
| `i18n/index.ts` + `locales/en.json` + `locales/fr.json` | 46/3233/3233 | i18next; detector order localStorage→navigator, key `mowglinext.lang`; fallback chain `["fr","en"]` |
| `concept/` (21 files, ~3.7k lines) | | DEV-ONLY design playground mounted at `/concept` only when `import.meta.env.DEV` (`main.tsx` L29–32). `concept/motion.ts` + `tokens.css`/`concept.css` ARE used by production pages; `concept/components/*` are reused by `MowgliNextPage` |
| `modes/SplitLineMode.ts` / `modes/DirectSelectWithBoxMode.tsx` | 41/375 | Custom mapbox-gl-draw modes (2-click split; box-select direct-select) |
| `constants/mowerModels.ts` | 115 | `MOWER_MODELS` presets used by onboarding + Hardware section |
| **`src/utils/`** | | |
| `gpsStatus.ts` | 438 | Fix/RTK derivation + diagnostics projection (largest util; heavily tested) |
| `map.tsx` / `mowProgress.ts` / `quaternion.ts` | 162/63/22 | `transpose`/`itranspose` (map-frame m ↔ lon/lat about the datum), robot silhouette, occupancy-grid rasteriser, yaw/roll/pitch |
| `containers.ts` / `settingsValues.ts` / `settingsSave.ts` / `datumGps.ts` | 118/45/81/37 | Container actions + `GPS_RESTART_KEYS`; `parseBoolish`/`valuesMatch`; partial YAML save; `set_datum` helper |
| `logTime.ts` / `battery.ts` / `diagnosticsAlerts.ts` / `nav2Recovery.ts` / `telemetryFormat.ts` / `driveTuningI18n.ts` / `stringifyValue.ts` | 336/51/47/39/19/211/29 | Log timestamp parse/format (+ zone mode); battery %/level; alert grouping; Nav2 recovery detection from BT log; tiny-value clamp; drive-tuning message i18n; safe stringify |
| **`src/test/`, `tests/e2e/`** | | |
| `src/test/setup.ts` / `src/test/mocks.tsx` / `src/test/eslint-config.test.ts` | 23/293/179 | Pins i18n to `en`, mocks `matchMedia`; shared `HighLevelStatus`/`GnssStatus` fixtures; asserts the flat eslint config still resolves and its real rules are still enabled |
| `tests/e2e/mock/mockBackend.ts` / `mock/scenarios.ts` | 83/135 | REST + msgpack-WS interception; 8 robot-state scenarios |
| `tests/e2e/*.spec.ts` (5 files) | 55–167 | See Build, test, run |
| co-located `*.test.ts[x]` under `src/` (45) + `pages/logBatcher.bench.ts` + `tests/e2e/README.md` | 24–304 / 22 / 43 | Vitest units, enumerated under Build, test, run; the e2e README documents the mocked harness |

## Runtime surface

### Routes (hash router, `main.tsx`)
`/mowglinext` · `/map` · `/schedule` · `/diagnostics` · `/statistics` · `/settings` · `/parameters` · `/logs` · `/onboarding` — all lazy, all children of `AppShell`. `/concept` exists only in dev builds (`main.tsx` L29–32). `/` redirects to `/mowglinext` (`AppShell.tsx` L88–93).

### Multiplex topic keys → hook → page
Wire: client sends `{"op":"subscribe"|"unsubscribe","topic":"<key>"}` as JSON text on `ws://<host>/api/mowglinext/multiplex`; server replies with msgpack binary `{topic, data}`. Keys are backend aliases, NOT ROS topic names.

| Topic key | Hook | Consumed by |
|-----------|------|-------------|
| `highLevelStatus` | `useHighLevelStatus` | AppShell, dashboard, MapPage (via `useMapStreams`), MowerActions, Diagnostics |
| `status`, `power`, `emergency` | `useStatus`, `usePower`, `useEmergency` | AppShell, dashboard, Diagnostics |
| `gps`, `pose` | `useGPS`, `usePose` | Diagnostics (`useGPS`), `RobotComponentEditor` (`usePose`); MapPage subscribes `pose` inline in `useMapStreams` |
| `gnssStatus` | `useGnssStatus` | dashboard, Diagnostics, Onboarding |
| `map`, `mowProgress`, `path`, `plan`, `lidar`, `obstacles`, `recordingTrajectory` | `useMowingMap`, `useMowProgress`, rest inline in `useMapStreams` | MapPage (all seven), SchedulePage (`map`, inline `useWS`), dashboard + `ReadinessStep` (`useMowingMap`), dashboard mini-map (`useMowProgress`) |
| `fusionRaw`, `wheelOdom` | `useFusionOdom`, `useWheelOdom` | dashboard mini-map pose; Diagnostics. `useIcpOdom` (`icpOdom`) exists but currently has NO consumer |
| `imu`, `cogHeading`, `magYaw`, `ticks`, `dockingSensor` | `useImu`, `useCogHeading`, `useMagYaw`, `useWheelTicks`, `useDockingSensor` | Diagnostics heading-sources + sensors panels; `useDockingSensor` only in `DriveMotorSection` |
| `diagnostics`, `fusionDiag`, `btLog` | `useDiagnostics`, `useFusionGraphDiagnostics`, `useBTLog` | Diagnostics (raw array, fusion_graph card, BT state graph) |
| `dockCalibrationStatus`, `robotDescription` | `useDockCalibration`, `useRobotDescription` | DockCalibrationCard; map robot silhouette |
| `coverageResumeAvailable` | `useCoverageResumeAvailable` | MowerActions ("Start" vs "Start fresh") |

Non-multiplexed sockets: `/api/mowglinext/publish/joy` (teleop, dedicated bidirectional socket, `useMapStreams` L424/L432) and `/api/containers/{id}/logs` (LogsPage L151, `useFirmwareDebugLogs`).

### ROS services called through `POST /api/mowglinext/call/{command}`
`high_level_control` (`{Command: 1 start | 2 home | 3 record | 4 next-zone | 8 stop}`, `MowerActions.tsx` L53–163), `emergency` (`{Emergency: 0|1}`), `mow_enabled` (`{mow_enabled, mow_direction}`), `coverage_clear_resume` (`MowerActions.tsx` L49, `useResetMowingProgress.tsx` L27), `set_datum` (`utils/datumGps.ts` L25, `PositioningSection.tsx` L72, `OnboardingPage.tsx` L598), `promote_obstacle` (`TrackedObstaclesPanel.tsx` L68), `fusion_graph_save` / `fusion_graph_clear` (`DiagnosticsPage.tsx` L835).

### REST endpoints the frontend uses (all under `/api`)
`settings/status` · `settings` · `settings/yaml` (GET/POST) · `settings/yaml/defaults` · `settings/schema` · `settings/gnss/{runtime-config,plan,apply,factory-reset-apply,restart}` · `params` (GET/POST live ROS params) · `config/keys/{get,set}` · `config/envs` · `containers` (list) · `containers/{id}/{start|stop|restart}` · `diagnostics/snapshot` · `diagnostics/sessions` · `diagnostics/sessions/stats` · `calibration/status` · `calibration/imu-yaw` · `calibration/dock/start` · `calibration/magnetometer` · `schedules` (CRUD) · `weather` · `irrisense/{status,settings,gardens}` · `tools/rosbag/{start,stop,status}` · `tools/drive/{pid-tuning/start,ff-calibration/start,tuning/status,tuning/rollback,tuning/report/latest}` · `import/openmower` · `mowglinext` (PUT map) · `mowglinext/map/docking` · `ntrip/sourcetable` · `diagnostics/firmware_debug` · `setup/flashBoard` (SSE) · `system/{info,reboot,shutdown}`.

### Browser-local state (never a robot setting)
`localStorage`: `mowgli.display-mode` (`theme/ThemeContext.tsx` L8), `mowgli.log-timezone` (`utils/logTime.ts` L28), `mowglinext.lang` (`i18n/index.ts` L41). `sessionStorage`: `pwa-install-dismissed` (`hooks/useIOSInstallPrompt.ts` L19). GUI key-value DB (server-side, via `useConfig`): `gui.map.offset.x/y`, `gui.map.display.bearing`.

### Env vars read at build/dev time
`VITE_MAPBOX_TOKEN` (`pages/MapPage.tsx` L46, falls back to a hardcoded public token), `VITE_API_HOST` (`utils/apiHost.ts`), `MOWGLI_API_TARGET` (vite proxy target, node-side).

## Build, test, run

```bash
cd gui/web
yarn install --frozen-lockfile
yarn dev                          # vite on :5173, /api proxied to MOWGLI_API_TARGET
MOWGLI_API_TARGET=http://10.69.4.198:4006 yarn dev   # against a live robot
yarn build                        # tsc && vite build
npx tsc --noEmit                  # typecheck only (what CI runs)
yarn lint                         # eslint, --max-warnings ceiling in package.json
yarn test                         # vitest run (unit)
npx vitest run src/utils/gpsStatus.test.ts           # one file
yarn test:e2e                     # playwright; auto-starts vite :5173, fully mocked
npx playwright test -g "emergency-latched"
cd gui && ./generate_ts_types.sh  # regenerate src/types/ros.generated.ts
cd gui/web && yarn generate:api   # regenerate src/api/Api.ts from ../docs/swagger.json
```

**Unit tests (vitest, 45 test files + 1 bench).** Pure logic: `utils/gpsStatus.test.ts` (fix/RTK derivation + diagnostics fallback), `utils/logTime.test.ts`, `utils/map.test.ts`, `utils/settingsValues.test.ts`, `utils/nav2Recovery.test.ts`, `utils/diagnosticsAlerts.test.ts`, `utils/telemetryFormat.test.ts`, `types/map.test.ts` (feature class hierarchy: constructors, accessors, id conventions — it does NOT cover the serialize round-trip), `components/settings/{gnssConfig,ntripProviders,paramCatalog}.test.ts`, `components/onboarding/readinessChecks.test.ts`, `pages/logBatcher.test.ts` (+ `logBatcher.bench.ts`). Hooks: `pages/map/hooks/{useManualMode,useMapEditHistory,useMapOffset,useLatestThrottle,mapRenderBudget,useResetMowingProgress}.test.ts`. Components: `BTStateGraph`, `MowerStatus`, `SoilWetBanner`, `IrriSenseStatusChip`, `GnssLiveDiagnosticsCard`, `GnssLiveStatusSummaryCard`, `LogsPage`, `ThemeContext`, and the settings set (`DisplayModeSection`, `DockCalibrationCard`, `GnssReceiverActionsCard`, `GnssSerialDeviceConfigField`, `IrriSenseSection`, `LedsSection`, `LogTimeZoneSection`, `MowingSection`, `SafetySection`, `UniversalGnssAdvancedSettings`), plus `map/components/{AreasListPanel,EditAreaModal,JoystickOverlay,MapToolbar,MapToolbarMobile,NewAreaModal}`. Meta: `i18n/locales.test.ts` (en/fr key parity, no empty strings), `src/test/eslint-config.test.ts`.

**E2E (playwright, fully mocked — no backend, no robot).** `tests/e2e/pages.spec.ts` (8 pages × 8 scenarios: shell mounts, title renders, ZERO uncaught page errors, screenshot to `tests/e2e/.artifacts/`), `log-stream.spec.ts` (high-rate log retention + docker-stamped lines), `map-console.spec.ts` (map layers mount with no source/fragment errors), `reset-mowing-progress.spec.ts`, `visual-effects.spec.ts` (Visual/Balanced/Efficient compositing + reduced-motion).

**CI.** `.github/workflows/gui-ci.yml` — single job `unit-tests` on `ubuntu-24.04`, node 22, paths filter `gui/web/**`: `npx tsc --noEmit` → `yarn lint` → `yarn test`. **Playwright e2e is NOT run in CI.** `.github/workflows/msg-codegen-drift.yml` job `codegen-drift` re-runs `gui/generate_ts_types.sh` and fails on a diff in `gui/web/src/types/ros.generated.ts`.

## Change coupling — "if you change X, also update Y"

- **`.msg` in `ros2/src/mowgli_interfaces/msg`** → `cd gui && ./generate_ts_types.sh` (and `./generate_go_msgs.sh`) → commit `src/types/ros.generated.ts`, else `msg-codegen-drift.yml` fails. Regenerate with `LC_ALL=C` on macOS or the sort order fakes ~20 lines of drift.
- **New backend REST route or handler signature** → `gui/docs/swagger.json` → `yarn generate:api` → `src/api/Api.ts`. Hand-written `guiApi.request({path: …})` calls (`/params`, `/settings/yaml/defaults`, `/tools/*`, `/irrisense/*`) bypass the generated client and must be updated by hand.
- **New multiplex topic** → backend `MultiplexRoute`/topic alias table in `gui/pkg/api/mowglinext.go` → a `useTopic` wrapper here → add a payload to `tests/e2e/mock/scenarios.ts` so e2e still renders it.
- **New `mowgli_robot.yaml` key** → `gui/asserts/mower_config.schema.json` (its default IS the GUI's notion of "default" for the overridden dot) → `SECTION_DEFINITIONS` in `hooks/useSettingsManager.ts` → the section component → `i18n/locales/{en,fr}.json` → optionally `components/settings/paramCatalog.ts` if it is also a live ROS param.
- **Any new UI string** → BOTH locale files, or `i18n/locales.test.ts` fails (it asserts exact key parity in both directions).
- **New page route** → `main.tsx` router + `AppShell` `NAV`/`PAGE_META` + a `pageMeta.*` i18n namespace + the `PAGES` list in `tests/e2e/pages.spec.ts` L14–23.
- **New settings section id** → the `SettingsSection` union, `SECTION_DEFINITIONS`, the `switch` in `SettingsPage.tsx`, and a `settingsSections.<id>.{label,description}` pair in both locales.
- **Feature class added in `types/map.ts`** → `serializeFeature`/`featureFromJSON` (L320/L354) must handle it, or undo/redo silently drops it (history stores serialized snapshots precisely because `structuredClone` strips prototypes).
- **ROS param renamed** → `components/settings/paramCatalog.ts` key (keyed on the SHORT name after the last `.`/`/`) and, if it is injected at launch, the injection line in `ros2/src/mowgli_bringup/launch/*.launch.py`.

## Pitfalls

- `src/types/ros.ts` is a ONE-LINE re-export; the generated file is `src/types/ros.generated.ts`. Editing either by hand is reverted by the codegen drift gate.
- `useTopic`'s first argument is the backend TOPIC KEY (`fusionRaw`, `mowProgress`, `dockingSensor`), not a ROS topic name. Inventing a key silently yields a subscription the backend never answers — no error, just no data.
- The multiplexer is a module singleton (`getMultiplexedSocket()`); it closes the socket when the LAST listener unsubscribes and reconnects on the next subscribe. A component that subscribes in a `useEffect` without returning the unsubscribe leaks a server-side ROS subscription for the tab's lifetime.
- `useMapStreams` deliberately depends on `settings["datum_lon"]`/`["datum_lat"]` and NOT on the whole `settings` object (L462–479) — reverting that re-creates the re-subscribe storm that made every map stream look stale.
- Teleop teardown is debounced by 1200 ms in BOTH `useMapStreams` L438–445 and `useManualMode.ts`; a single stray non-`MANUAL_MOWING` frame must not kill the joystick mid-drive.
- `theme/ThemeContext.tsx` is hard-locked to dark; `toggleMode` is an intentional no-op and the light tokens in `colors.ts` are unused. Do not wire a light-mode switch expecting it to work.
- Display mode is presentation-only: `mapRenderBudget.ts` changes render cadence, never subscriptions or robot-side rates.
- Settings Save sends ONLY dirty keys (`persistSettings` L459–470) so concurrent writers (dock calibration service, map "set docking point") are not clobbered; sending the full local state would.
- `AdvancedSection` deletes are expressed as `null` values; `persistSettings` prunes them from local state after save (L462–470) or the deleted rows reappear.
- `SafetySection` CLAIMS `lift_recovery_mode` / `lift_blade_resume_delay_sec` in `SECTION_DEFINITIONS` but deliberately does not render them — the claim is what keeps them out of `AdvancedSection`'s free-form editor. Every `led_*` key is claimed by the `leds` section for the same exclusion reason, but `LedsSection` DOES render them.
- The Fusion Graph diagnostics card is rendered UNCONDITIONALLY (`DiagnosticsPage.tsx` L1884, L1933+). There is no `use_fusion_graph` key anywhere in the GUI — `fusion_graph` is the sole localizer (CLAUDE.md Invariant 1).
- The dashboard mini-map (`MowgliNextPage`) and MapPage (`useMapStreams`) both rasterize the same `OccupancyGrid` through `utils/mowProgress.ts`; the grid↔OccupancyGrid axis convention is CLAUDE.md Invariant 14 — mis-sized rasters show as a 90°-rotated overlay.
- `MAPBOX_TOKEN` falls back to a hardcoded public token (`MapPage.tsx` L46); set `VITE_MAPBOX_TOKEN` rather than editing the literal.
- Dev against a robot must go through the vite proxy (`MOWGLI_API_TARGET`): the Go backend rejects WebSocket upgrades whose `Origin` host differs from `Host`, so `VITE_API_HOST` alone breaks every stream (see `utils/apiHost.ts` header comment).
- `yarn lint` blocks on ERRORS only; the `--max-warnings` ceiling in `package.json` is a ratchet with headroom (see the "debt ratchet" block, `eslint.config.js` L63+). Raising it needs a written reason.
- Playwright specs live under `tests/e2e/` and import `@playwright/test`; `vitest.config.ts` excludes them explicitly — do not co-locate a `.spec.ts` under `src/`.

## Generated & vendored — do not hand-edit

- `src/types/ros.generated.ts` — produced by `gui/generate_ts_types.sh` from `ros2/src/mowgli_interfaces/msg`; gated by `msg-codegen-drift.yml`.
- `src/api/Api.ts` — produced by `swagger-typescript-api` (`yarn generate:api`) from `gui/docs/swagger.json`; carries `/* eslint-disable */` + `@ts-nocheck`.
- `yarn.lock`, `node_modules/`, `dist/`, `tests/e2e/.artifacts/` — lockfile is edited only via yarn; the rest are build/test output.
- `src/concept/` is a dev-only design playground (route mounted only under `import.meta.env.DEV`), but `concept/motion.ts`, `concept/tokens.css`, `concept/concept.css` and several `concept/components/*` ARE imported by production pages — it is not dead code.
