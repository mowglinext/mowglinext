# Codemap: mowgli_leds

> Optional WS2812 ("NeoPixel") status ring driven from the SBC over SPI MOSI (`/dev/spidev*`). One
> plain node (`led_ring_node`) subscribes to three status topics, renders a pure pixel pattern
> (emergency / charging / stale / low-battery / mowing / recording / manual / idle), encodes it as
> 3-SPI-bits-per-WS2812-bit at 2.4 MHz and writes the frame. It publishes nothing, commands nothing,
> and is NOT in the blade-safety path (firmware keeps its own status LED). Off by default
> (`led_enabled: false`); **not hardware-verified** as of the README.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / rename / re-default a `led_*` parameter | `ros2/src/mowgli_leds/src/led_ring_node.cpp:65-77` (declare + clamps), then the 6 mirrors in *Change coupling* |
| Change a colour or animation | `ros2/src/mowgli_leds/include/mowgli_leds/led_pattern.hpp` — `colors::` (l.135-144), `RenderFrame()` (l.313-394); mirror swatches in `gui/web/src/components/settings/LedsSection.tsx:24-34` |
| Change mode priority / what beats what | `led_pattern.hpp` `SelectMode()` (l.241-275); pinned by `test/test_led_pattern.cpp` `LedPatternMode.*` (l.58-143) |
| Add a new `HIGH_LEVEL_STATE_*` | `led_pattern.hpp` `HighLevelState` enum (l.80-87) + `led_ring_node.cpp` `static_assert`s (l.22-36) + `ToHighLevelState()` (l.42-57) + `SelectMode` switch (l.262-274) |
| Change staleness / source precedence (BT vs Power vs GNSS) | `led_ring_node.cpp` `collectInputs()` (l.191-218) |
| Change device open / retry / warn-once policy | `led_ring_node.cpp` `ensureDevice()` (l.220-260), `writeFrame()` (l.262-301) |
| Change SPI mode, bits-per-word, ioctls | `ros2/src/mowgli_leds/src/spi_device.cpp` `Open()` (l.50-105) |
| Change bit timing / SPI clock / symbol table | `ros2/src/mowgli_leds/include/mowgli_leds/ws2812_encoder.hpp` `kSpiClockHz` (l.74), `ExpandByte()` (l.101-114); header comment l.16-36 explains why 3 bits @ 2.4 MHz |
| Change the reset (latch) gap | `ws2812_encoder.hpp` `kResetLowBytes` (l.92); pinned by `test_ws2812_encoder.cpp:109` |
| Change brightness scaling / GRB order | `ws2812_encoder.hpp` `ScaleChannel()` (l.119), `Encode()` (l.134-155) |
| Progress-arc end rules (≥1 lit, ≥1 dark) | `led_pattern.hpp` `FilledCount()` (l.214-238) |
| Launch gating / param passthrough | `ros2/src/mowgli_bringup/launch/full_system.launch.py:100` (early read), `:147-151` (`led_enabled` arg), `:668-706` (`led_ring_node` Node #11) |
| Template defaults | `ros2/src/mowgli_bringup/config/mowgli_robot.yaml:706-739` |
| GUI settings section (fields, legend, prerequisites) | `gui/web/src/components/settings/LedsSection.tsx`; registered in `gui/web/src/hooks/useSettingsManager.ts:221-233`; mounted in `gui/web/src/pages/SettingsPage.tsx:223-232` |
| GUI JSON schema (`led_settings` group) | `gui/asserts/mower_config.schema.json:713-806` |
| GUI strings (legend, tooltips, section label) | `gui/web/src/i18n/locales/en.json` (`settingsLeds` block l.3024-3075, `settingsSections.leds` l.3138), `fr.json` (l.3134) |
| What "RTK fixed" means for the ring | `ros2/src/mowgli_interfaces/include/mowgli_interfaces/gnss_status_utils.hpp:148` `BehaviorTreeRtkFixed()` (shared with BT + hardware bridge) |
| Hardware bring-up (pin 19, overlay, 3.3 V level caveat, permissions) | `ros2/src/mowgli_leds/README.md` §1-2 |
| Shutdown blanking | `led_ring_node.cpp` `blank()` (l.303-313), called from the destructor (l.180-183) |
| Add a pattern test | `test/test_led_pattern.cpp` helpers `MakeCfg()` / `MakeMowingInputs()` / `CountLit()` (l.20-50) |
| Add an encoder test | `test/test_ws2812_encoder.cpp` helper `BitAt()` (l.20-25) |
| Docker image / rosdep layer | `ros2/Dockerfile:317-318` (package.xml + CMakeLists COPY) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_leds/`** | | |
| `ros2/src/mowgli_leds/package.xml` | 32 | ament_cmake pkg; deps `rclcpp`, `mowgli_interfaces`; test deps lint + gtest |
| `ros2/src/mowgli_leds/CMakeLists.txt` | 122 | Static lib `mowgli_leds_core` (node + spidev), exe `led_ring_node`, two header-only gtests (no rclcpp link) |
| `ros2/src/mowgli_leds/README.md` | 318 | Operator + developer doc: wiring, U-Boot overlay, params, display semantics, encoding, failure behaviour |
| `ros2/src/mowgli_leds/include/mowgli_leds/led_pattern.hpp` | 396 | **Pure**: `LedInputs`/`LedPatternCfg` → `SelectMode()` → `RenderFrame()`; palette; `Breathe`/`BlinkOn`/`RotationIndex`/`FilledCount` |
| `ros2/src/mowgli_leds/include/mowgli_leds/ws2812_encoder.hpp` | 159 | **Pure**: `Rgb` struct, `ws2812::Encode()` (GRB, 3 SPI bits/bit, 90-byte reset gap), `kSpiClockHz` |
| `ros2/src/mowgli_leds/include/mowgli_leds/spi_device.hpp` | 79 | `SpiDevice` RAII wrapper API, `SpiResult` (never throws, never logs) |
| `ros2/src/mowgli_leds/include/mowgli_leds/led_ring_node.hpp` | 107 | `LedRingNode` class: params, latest-status cache, output state, subs, timer |
| `ros2/src/mowgli_leds/src/led_ring_node.cpp` | 320 | Node impl: param declare/clamp, subs, `collectInputs`, `ensureDevice`, `writeFrame`, `blank`, `static_assert`s |
| `ros2/src/mowgli_leds/src/spi_device.cpp` | 135 | Only hardware-touching code: `open`/`ioctl`(mode 0, 8 bpw, MSB, speed)/`write`; `#ifdef __linux__` |
| `ros2/src/mowgli_leds/src/led_ring_main.cpp` | 18 | `main()`: `rclcpp::spin(LedRingNode)` |
| `ros2/src/mowgli_leds/test/test_led_pattern.cpp` | 446 | 32 gtests: mode priority, arc arithmetic, animation helpers, rendered frames per mode |
| `ros2/src/mowgli_leds/test/test_ws2812_encoder.cpp` | 181 | 13 gtests: symbol expansion, MSB-first, GRB order, frame size, reset gap, clock, brightness |
| **Integration points outside the package** | | |
| `ros2/src/mowgli_bringup/launch/full_system.launch.py` | 742 | Launches `led_ring_node` (l.668-706) under `IfCondition(led_enabled)`; passes every `led_*` from merged config |
| `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` | 739 | Template defaults for all 12 `led_*` keys (l.721-737) — CLAUDE.md Invariant 15 |
| `ros2/src/mowgli_bringup/package.xml` | 70 | `<exec_depend>mowgli_leds</exec_depend>` (l.55-56) |
| `ros2/src/mowgli_interfaces/include/mowgli_interfaces/gnss_status_utils.hpp` | 153 | `BehaviorTreeRtkFixed()` (l.148) used by the ring's `/gps/status` callback |
| `ros2/Dockerfile` | 559 | COPY of `mowgli_leds/package.xml` + `CMakeLists.txt` into the rosdep layer (l.317-318) |
| `gui/asserts/mower_config.schema.json` | 838 | `led_settings` group (l.713-806): defaults MUST equal the template |
| `gui/web/src/components/settings/LedsSection.tsx` | 312 | *Settings → Status LEDs* section: enable switch, fields, prerequisite alerts, pattern legend (`LED_MODES` swatches) |
| `gui/web/src/components/settings/LedsSection.test.tsx` | 194 | 14 vitest cases: off-by-default, ON/OFF writes, overlay + 3.3 V hints, field set, legend, reset |
| `gui/web/src/hooks/useSettingsManager.ts` | 732 | `"leds"` section id + the 12 `led_*` keys claimed (l.221-233) so they never fall into AdvancedSection |
| `gui/web/src/pages/SettingsPage.tsx` | 427 | Renders `<LedsSection>` for the `leds` case (l.223-232) |
| `gui/pkg/api/settings_leds_test.go` | 139 | Go tests: `led_enabled` default false + sparse-prune coupling (#508); schema↔template parity for all `led_*`; SPI clock ↔ encoder |
| `gui/pkg/api/schema_template_parity_test.go` | 149 | `TestSchemaDefaultsMatchTemplate` — whole-schema parity, also covers `led_*` |

## Runtime surface

### Nodes
| Node | Executable | Package | Launched by | Kind |
|------|-----------|---------|-------------|------|
| `led_ring_node` | `led_ring_node` (`lib/mowgli_leds/led_ring_node`) | `mowgli_leds` | `full_system.launch.py` Node #11 (l.668), `condition=IfCondition(LaunchConfiguration("led_enabled"))`, `output="screen"` | plain `rclcpp::Node` (not lifecycle); one `create_wall_timer` at `1000/led_refresh_hz` ms (l.163-169) |

Constructor **early-returns** with no subscriptions, no timer, no device when `led_enabled=false` or `led_count=0` (`led_ring_node.cpp:113-123`). The launch arg default is read from the merged config (`_rp.get("led_enabled", False)`, l.100); CLI/compose `led_enabled:=` overrides.

### Topics (all subscriptions; the node publishes nothing)
| Topic | Type | Dir | QoS | Other end |
|-------|------|-----|-----|-----------|
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` | sub (l.128-135) | `rclcpp::QoS(10)` | `behavior_tree_node` `~/high_level_status` (`mowgli_behavior/src/status_nodes.cpp:57-59`, depth 10; re-published at 1 Hz by `behavior_tree_node.cpp:696`). Fields used: `state`, `coverage_percent`, `battery_percent`, `is_charging`, `emergency` |
| `/gps/status` | `mowgli_interfaces/msg/GnssStatus` | sub (l.137-147) | `rclcpp::QoS(10)` | `sensors/gps/universal_gnss_topic_bridge.py:167` (`output_status_topic`), reliable/volatile depth 10. Reduced to one bool via `gnss_status_utils::BehaviorTreeRtkFixed()` |
| `/hardware_bridge/power` | `mowgli_interfaces/msg/Power` | sub (l.152-159) | `rclcpp::QoS(10)` | `hardware_bridge_node` `~/power` (`hardware_bridge_node.cpp:707`, QoS(10)), remapped in `mowgli.launch.py:263`. Only `charger_enabled` (= firmware `STATUS_BIT_CHARGING`, `hardware_bridge_node.cpp:1399`) is used, as the charging fallback when the BT is silent |

Freshness: every source is stale after `led_status_timeout_s` (`collectInputs()` l.197, 208, 216). Precedence: fresh `HighLevelStatus` → else fresh `Power` gives `is_charging` only (battery stays invalid) → `rtk_fixed` needs fresh GNSS.

### Services & actions
None.

### Parameters
All 12 declared in the constructor (`led_ring_node.cpp:65-77`), read **once at startup** (no parameter callback). Defaults live in the template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (l.721-737) and are mirrored in `gui/asserts/mower_config.schema.json` (l.717-806); `full_system.launch.py:679-704` forwards them with matching fallbacks.

| Param | Default | Node clamp (file:line) | Effect |
|-------|---------|------------------------|--------|
| `led_enabled` | `false` | — (l.65; gate l.113) | Launch gate AND in-node stand-down. MUST stay `false` in template + schema (#508 prune bug; `settings_leds_test.go:54`) |
| `led_count` | `16` | `[0, 512]` (`kMaxLedCount` l.40, clamp l.79-87) | Ring length; placeholder default |
| `led_spi_device` | `/dev/spidev4.1` | — (l.66) | spidev path; numbering is kernel-assigned |
| `led_brightness` | `0.6` | `[0, 1]` (l.89) | Linear per-channel scale in `Encode()` |
| `led_spi_speed_hz` | `2400000` | `≥ 1` + WARN if ≠ `ws2812::kSpiClockHz` (l.90-102) | Symbol table is derived from exactly 2.4 MHz |
| `led_refresh_hz` | `20.0` | `[1, 60]` (l.103) | Render tick |
| `led_status_timeout_s` | `5.0` | `≥ 0.5` (l.104) | Staleness for all three inputs |
| `led_keepalive_s` | `2.0` | `≥ 0.2` (l.105) | Re-send unchanged frame |
| `led_device_retry_s` | `30.0` | `≥ 1.0` (l.106) | Silent re-open interval |
| `led_low_battery_percent` | `20.0` | `[0, 100]` (l.109) | Blinking-red threshold (`LedPatternCfg`) |
| `led_charge_full_percent` | `99.0` | `[0, 100]` (l.110) | Steady-green threshold |
| `led_idle_scale` | `0.10` | `[0, 1]` (l.111) | Idle ring dim factor |

`use_sim_time` is also passed by launch (l.676) but has no effect: animations use `std::chrono::steady_clock` (`monotonicSeconds()` l.185-189) and the tick is a wall timer.

### TF frames
None (no TF publish or lookup).

### Display modes (priority order, `SelectMode()` l.241-275 → `RenderFrame()` l.313-394)
| # | `LedMode` | Condition | Frame |
|---|-----------|-----------|-------|
| 1 | `kEmergency` | `status_fresh && emergency` | solid `kRed` |
| 2 | `kCharging` | `is_charging` (from BT or Power) | green arc breathing 3 s; comet if `!battery_valid`; solid green at `charge_full_percent` |
| 3 | `kStale` | `!status_fresh` | `kAmber` comet, 1.5 s/rev |
| 4 | `kLowBattery` | `battery_valid && battery_percent < low_battery_percent` | whole ring `kRed` blink 1 Hz |
| 5 | `kMowing` / `kMowingDegraded` | `kAutonomous` + `rtk_fixed` / not | green arc + white head / amber arc + 2 Hz blinking head |
| 6 | `kRecording` | `kRecording` | `kCyan` comet 2 s/rev |
| 7 | `kManual` | `kManualMowing` | `kPurple` breathing 2 s |
| 8 | `kIdle` | `kIdle` / `kNull` / default | `Dim(kWhite, idle_scale)` |

Wire format: `EncodedSize(n) = 9·n + 90` bytes (`ws2812_encoder.hpp:95-98`); 16 LEDs = 234 bytes. Frames are written only on change or keepalive (`writeFrame()` l.271-276).

## Build, test, run
```bash
# Package only (from ros2/; scripts/build.sh honours PACKAGES)
cd ros2 && make build-pkg PKG=mowgli_leds
# or, inside a sourced workspace
colcon build --packages-select mowgli_leds
colcon test  --packages-select mowgli_leds && colcon test-result --verbose
# run by hand (stands down unless led_enabled is true)
ros2 run mowgli_leds led_ring_node --ros-args -p led_enabled:=true -p led_count:=24
```
Lint: `ament_lint_auto` with copyright/cpplint/uncrustify suppressed (`CMakeLists.txt:87-90`); formatting is clang-format via `ros2/.clang-format`.

| Test | Registered in | Pins |
|------|---------------|------|
| `ros2/src/mowgli_leds/test/test_led_pattern.cpp` (32 cases) | `CMakeLists.txt:103-108` `ament_add_gtest(test_led_pattern)` | Mode priority (emergency > charging > stale > low-battery > activity; stale emergency ignored), `FilledCount` end rules, `Breathe`/`BlinkOn`/`RotationIndex` bounds + negative time, exact rendered frames per mode, zero-length ring |
| `ros2/src/mowgli_leds/test/test_ws2812_encoder.cpp` (13 cases) | `CMakeLists.txt:96-101` `ament_add_gtest(test_ws2812_encoder)` | `0→0b100`, `1→0b110`, MSB-first, every symbol starts high/ends low, GRB order, 9 bytes/pixel + reset gap, gap ≥ 280 µs of zeros, 3 bits @ 2.4 MHz = 1.25 µs, brightness linear/clamped/NaN-safe, empty buffer still emits the gap |
| `gui/pkg/api/settings_leds_test.go` | `cd gui && go test ./pkg/api/...` | `led_enabled` schema default `false` + `sparsifyFlat` round-trip; every `led_*` schema default == template value and no extra `led_*` in schema; `3 / led_spi_speed_hz == 1.25 µs` |
| `gui/web/src/components/settings/LedsSection.test.tsx` (14 cases) | `cd gui/web && yarn test` | Absent key renders OFF, controls hidden until enabled, ON writes `true` / OFF writes `false`, overlay + `ls /dev/spidev*` hint, 3.3 V warning, hardware + appearance field sets, count/device edits, legend lists every mode, two reds differ by motion, reset-to-default, no-defaults render |
| `ros2/src/mowgli_behavior/test/test_gnss_status_authority.cpp` | mowgli_behavior gtest | `BehaviorTreeRtkFixed()` semantics the ring inherits |

CI: `.github/workflows/ros2-ci.yml` job `build-and-test` ("Build & Test (ROS2 kilted)") builds the whole workspace and runs `colcon test` (l.336-350) on changes under `ros2/**` (path filter l.75-77); job `config-drift` (l.96-114) runs `ros2/scripts/check_config_drift.py` against the template. `.github/workflows/gui-ci.yml` runs `yarn test` (l.75-76) for the vitest file. No workflow runs `go test` for `settings_leds_test.go` (grep of `.github/workflows/` finds none) — run it locally.

## Change coupling — "if you change X, also update Y"
- **New / renamed `led_*` param** → (1) `led_ring_node.cpp:65-77` declare; (2) template `mowgli_robot.yaml:721-737`; (3) `full_system.launch.py:679-704` passthrough; (4) `mower_config.schema.json` `led_settings` (l.713-806); (5) `useSettingsManager.ts:229-232` key list (else it leaks into AdvancedSection); (6) `LedsSection.tsx` field + `en.json`/`fr.json` strings; (7) `settings_leds_test.go:96-101` `expected` list — `TestLedSchemaDefaultsMatchTemplate` asserts `len(defaults) == len(expected)` (l.114), so an unlisted key fails the Go test.
- **Change a default** → template AND schema together; `TestLedSchemaDefaultsMatchTemplate` + `TestSchemaDefaultsMatchTemplate` (`schema_template_parity_test.go:94`) fail on divergence. The launch `.get(..., fallback)` values (l.682-704) and the C++ `declare_parameter` defaults are dead once the template has the key, but keep them equal to avoid confusion.
- **`led_enabled` default** must stay `false` in BOTH template (`mowgli_robot.yaml:721`) and schema (`schema.json:720`): `sparsifyFlat` (`gui/pkg/api/settings.go:388`) prunes values equal to the schema default, so a `true` default makes the ON toggle inert (#508). Pinned by `settings_leds_test.go:54-75` and `LedsSection.tsx:48`.
- **Palette / mode change** in `led_pattern.hpp` `colors::` (l.135-144) or `LedMode` (l.89-100) → `LedsSection.tsx` `LED_MODES` swatches (l.24-34), `en.json`/`fr.json` `mode*` + `mode*Description` strings, `LedsSection.test.tsx` legend tests (l.152-173), README §4 table.
- **`HighLevelStatus.msg` `HIGH_LEVEL_STATE_*`** (`ros2/src/mowgli_interfaces/msg/HighLevelStatus.msg`) → `HighLevelState` enum (`led_pattern.hpp:80-87`) + `static_assert`s (`led_ring_node.cpp:22-36`, build breaks on renumber) + `ToHighLevelState()` (l.42-57) + `SelectMode` switch (l.262-274).
- **SPI clock** `ws2812::kSpiClockHz` (`ws2812_encoder.hpp:74`) ↔ template/schema `led_spi_speed_hz` ↔ `TestLedSpiClockMatchesTheEncoderAssumption` (`settings_leds_test.go:126`) ↔ `test_ws2812_encoder.cpp:125`. Changing the symbol scheme (`kSpiBitsPerLedBit`, `ExpandByte`) changes all of them.
- **RTK-fixed definition** lives in `gnss_status_utils.hpp:148` and is shared with `behavior_tree_node.cpp:453` and `hardware_bridge_node.cpp:764` — do not re-derive it in the ring.
- **Topic renames**: `~/power` remap in `ros2/src/mowgli_bringup/launch/mowgli.launch.py:263`; `~/high_level_status` in `mowgli_behavior/src/status_nodes.cpp:59`; `/gps/status` param `output_status_topic` in `sensors/gps/universal_gnss_topic_bridge.py:167` — the ring hardcodes the absolute names (`led_ring_node.cpp:128,138,152`).
- **New source file / dependency** → `CMakeLists.txt` (`mowgli_leds_core` l.32-35; keep `led_pattern.hpp`/`ws2812_encoder.hpp` header-only so the gtests link without rclcpp, l.93-108) + `package.xml`; a new dependency also needs `ros2/Dockerfile:317-318` (rosdep layer is keyed on package.xml).
- **Sparse installed config**: never add `led_*` lines to `install/config/mowgli/mowgli_robot.yaml` at template value — `check_config_drift.py` (CI `config-drift`) rejects padded defaults (Invariant 15).

## Pitfalls
- `ros2 run mowgli_leds led_ring_node` with no args does nothing: the constructor returns before creating subscriptions/timer when `led_enabled=false` or `led_count=0` (`led_ring_node.cpp:113-123`). Pass `-p led_enabled:=true`.
- The `LedInputs::battery_valid` comment (`led_pattern.hpp:112-114`) says the percent can come from `Power`; the node never does that — the Power fallback sets only `is_charging` (`led_ring_node.cpp:208-214`) and charging-with-unknown-level renders a comet (`led_pattern.hpp:329-337`). Do not "fix" by deriving a percent from `v_battery`: `batteryPercentFromVoltage` (`mowgli_behavior/src/battery_filter.cpp:74`) is the single SoC source.
- `emergency`/`state` are honoured only while the status is fresh (`SelectMode` l.245); `is_charging` outranks stale (l.249-256). A dead BT on a charging dock reads "charging", not "stale".
- `Rgb` is RGB; the GRB swap happens exactly once in `Encode()` (`ws2812_encoder.hpp:141-145`). Do not pre-swap in the pattern.
- `kResetLowBytes = 90` = 300 µs (`ws2812_encoder.hpp:85-92`); WS2812B-V5 merges frames below 280 µs. Don't trim to the datasheet's 50 µs.
- `led_spi_speed_hz ≠ 2400000` only WARNs (`led_ring_node.cpp:91-102`) and then produces wrong colours; it is not rejected.
- One WARN per outage: `warned_device_` (`led_ring_node.cpp:248-259`, `281-288`); a write failure closes the fd and re-arms the retry (l.292-294). Absence of log lines does not mean the device is open — check the single "unavailable" line at startup.
- On device (re)open `have_last_pixels_ = false` (l.239) forces a full write; otherwise frames are skipped unless changed or keepalive is due (l.271-276). A ring that lost 5 V mid-run recovers only at the next keepalive.
- `spi_device.cpp` is `#ifdef __linux__` (l.11-16, 100-104, 130-132): on macOS it builds but `Open()` returns "spidev is only available on Linux".
- `Breathe()` spells out `kTwoPi` (`led_pattern.hpp:175-179`) because `CMAKE_CXX_EXTENSIONS OFF` (`CMakeLists.txt:14`) hides `M_PI` — don't reintroduce `M_PI`.
- Container access: compose service `mowgli` is `privileged: true` with `/dev:/dev` (`install/compose/docker-compose.base.yml:39,46`), so `/dev/spidev*` appears once the U-Boot overlay is enabled (README §2); no `devices:` entry is needed, but the overlay step is manual and not done by the installer (no `spidev`/`led` mention in `install/`).
- `use_sim_time` is passed but irrelevant (steady clock + wall timer, l.165, 185-189). Tests render exact phases by setting `LedInputs::now_s`.
- Hardware unverified: README §1-2 (3.3 V data vs `VIH = 0.7·VDD`) — wrong colours with correct software are a level-shifting problem first.
- Safety: this package is deliberately outside the blade path (CLAUDE.md "Safety — READ FIRST"); it must never grow a publisher or service that commands the robot.

## Generated & vendored — do not hand-edit
- Nothing generated inside `ros2/src/mowgli_leds/`; message headers `mowgli_interfaces/msg/{high_level_status,gnss_status,power}.hpp` are rosidl-generated from `ros2/src/mowgli_interfaces/msg/*.msg` at build time.
