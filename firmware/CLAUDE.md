# Firmware (STM32) — working notes for Claude

The live target is `stm32/ros_usbnode/` (PlatformIO + stm32cube HAL): it owns the per-wheel velocity PI **and** the gyro yaw-rate loop (root CLAUDE.md preamble, "Option C"), the `ANTIDIG_*` cutout, blade/emergency/charger authority, the IMU + panel drivers, and the COBS+CRC-16 USB-CDC wire to the host. Secondary trees: `stm32/custom_panel_fw/` (STM32F0 panel), `stm32/test_code/` (bring-up), `stm32/{mainboard,panel}_firmware/` (stock backup/restore).
It must NOT own: any ROS topic/TF/pose (`ros2/src/mowgli_hardware`'s `hardware_bridge_node` is the only node on this wire), the wheel-**slip** dig detector (root Invariant 16 — this tree's `ANTIDIG_*` covers *blocked* wheels only), the source of truth for tuning (the host owns it; the board only keeps a flash copy of the last committed set — `fw_params.c`), or the flashing UX (`gui/pkg/providers/firmware.go`).

## Read next

| File | Read it when… |
|------|----------------|
| [`../docs/claude/codemaps/firmware.md`](../docs/claude/codemaps/firmware.md) | **always, before editing anything here** — file map, wire-packet table, main-loop order, change-coupling, pitfalls |
| [`../docs/claude/codemaps/mowgli_hardware.md`](../docs/claude/codemaps/mowgli_hardware.md) | the host end of the wire: `hardware_bridge_node`, `ll_datatypes.hpp`, handshake, dig detector |
| [`../docs/claude/ros-interfaces.md`](../docs/claude/ros-interfaces.md) | which ROS topic/service a packet fans out to (§ *Firmware ↔ ROS bridging*) |
| [`../docs/claude/parameters.md`](../docs/claude/parameters.md) | the runtime values the bridge pushes down (`ticks_per_meter`, `wheel_pid_*`, `yaw_*`; `wheel_track` must equal `board.h` `WHEEL_BASE`) |
| [`../docs/claude/testing-ci.md`](../docs/claude/testing-ci.md) | which CI gate a change trips + the pre-push checklist (items 2, 3, 11) |
| [`../docs/claude/codemaps/ci_repo_tooling.md`](../docs/claude/codemaps/ci_repo_tooling.md) | internals of `firmware-ci.yml`, `protocol-version-drift.yml`, `msg-codegen-drift.yml` |
| [`../docs/claude/codemaps/gui_backend.md`](../docs/claude/codemaps/gui_backend.md) · [`gui_frontend.md`](../docs/claude/codemaps/gui_frontend.md) | the flash path — prebuilt manifest, `board.h.template` rendering, `FlashBoardComponent.tsx` defaults |
| [`../docs/claude/commands.md`](../docs/claude/commands.md) | the repo-wide codegen workflow after a `.msg`/`.srv` change |
| [`../docs/claude/doc-index.md`](../docs/claude/doc-index.md) | before trusting any README here — it flags which are current, stale or vendored |
| [`../docs/claude/contributing.md`](../docs/claude/contributing.md) | commit type prefixes, branch/PR workflow |
| [`../wiki/Firmware.md`](../wiki/Firmware.md) | the per-wheel velocity-PI rationale (§ *Drive Motor Control* is live; the rosserial→COBS migration framing is historical) |
| [`../wiki/Architecture.md`](../wiki/Architecture.md) | where the board sits in the whole stack |
| [`README.md`](README.md) | tree layout + supported boards (partly stale — the codemap wins) |
| [`stm32/ros_usbnode/README.md`](stm32/ros_usbnode/README.md) | ST-Link wiring, the J18 serial-debug tap, the `usbreset` trick (its ROS-noetic/rosserial half is dead) |
| [`stm32/mainboard_firmware/README.md`](stm32/mainboard_firmware/README.md) · [`stm32/panel_firmware/README.md`](stm32/panel_firmware/README.md) | **before a first flash** — dump the stock image (mainboard README carries the known-good SHA256 table; the panel one is procedure only) |
| [`../ros2/src/mowgli_hardware/firmware/README.md`](../ros2/src/mowgli_hardware/firmware/README.md) | the COBS framing narrative only — the code beside it is a stale v3 copy |

## Build · test · run

```bash
cd firmware/stm32/ros_usbnode
pio run                                      # default_envs = Yardforce500 (STM32F103VC)
pio run -e Yardforce500B                     # STM32F401VC
pio run -e Yardforce500 -t upload            # ST-Link v2 (upload_protocol = stlink)
pio run -e Yardforce500_STLINK_V3 -t upload  # OpenOCD custom upload_command (also …500B_STLINK_V3)
pio run -t swo_viewer                        # SWO/ITM trace target (add_swo_viewer.py)
pio device monitor -b 115200 -p /dev/ttyAMA0 # UART debug text (ORIG variant only)
```

```bash
# Guards — pure Python, run from the repo root, no toolchain needed
python3 firmware/scripts/board_defaults_parity.py           # firmware-ci `defaults-parity` (takes no flags)
python3 firmware/scripts/protocol_version_guard.py --check  # protocol-version-drift
python3 firmware/scripts/sync_ros_lib.py --check            # msg-codegen-drift
python3 firmware/scripts/protocol_version_guard.py          # refresh the baseline AFTER a version bump

# Release packaging (what CI does on a v*.*.* tag)
python3 firmware/scripts/package_release.py --build-root firmware/stm32/ros_usbnode \
  --tag vX.Y.Z --repo mowglinext/mowglinext --out-dir dist
```

Firmware tests are thin: `pio test -e native` runs `test/test_cmd_vel_safety.cpp` (firmware-ci `test` job), and the Python harnesses in `firmware/scripts/` compile production C against HAL shims — `test_fw_params.py` (runtime parameters + flash log), `test_onboard_i2c_recovery.py`, `test_soft_i2c_recovery.py`, `test_blade_reverse.py` (firmware-ci `defaults-parity` job). The ROS2 suite compiles the pure firmware headers directly: `test_protocol.cpp` (host mirror), `test_fw_param_log.cpp`, `test_fw_param_catalog.cpp`, plus the heartbeat/emergency/blade policies (`cd ros2 && make test`). Add the three guards above and `gui/pkg/providers/firmware_test.go`. The PI / yaw / anti-dig loops are validated by a manual supervised field procedure only (`stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp:765-773`).

## Conventions

- **No formatter for this tree.** The repo's only `.clang-format` is `ros2/.clang-format` and `make format` covers `ros2/src` only — match the surrounding file's style by hand.
- **`stm32/ros_usbnode/include/mowgli_protocol.h` is the single wire source of truth.** Every `pkt_*_t` is packed and pinned by `_Static_assert`. Changing one means: bump `MOWGLI_PROTOCOL_VERSION` (currently `7u`) → hand-mirror the struct **and** `kMowgliProtocolVersion` into `ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp` → update `test_protocol.cpp` → `protocol_version_guard.py` (no `--check`) to refresh `firmware/scripts/protocol_baseline.json`. A mismatch makes every existing board report "incompatible, reflash".
- **Safety defaults are single-sourced in `stm32/ros_usbnode/include/board_defaults.h`.** `board.h` and `board.h.template` may only `#include` it — re-`#define`ing a managed macro fails `board_defaults_parity.py`.
- **`board.h` has a Go-template twin**, `board.h.template`, rendered by the GUI. A new compile-time knob needs `{{.Field}}` there **plus** `types.FirmwareConfig` (`gui/pkg/types/firmware.go`) plus `FlashBoardComponent.tsx`.
- **Do not hand-edit:** `stm32/ros_usbnode/src/ros/ros_lib/**` (vendored rosserial; the `mower_msgs/` subset is regenerated by `firmware/scripts/sync_ros_lib.py`), `stm32/ros_usbnode/CDC/**` (patched ST middleware), `include/pid.hpp` (vendored PX4, BSD-3), `src/proxy_inc/**`, `i2c_lis3dh.*`, `firmware/scripts/protocol_baseline.json`.
- `MOWGLI_FW_VERSION_{MAJOR,MINOR,PATCH}` is injected at build time by `git_build_id.py`; the identical encoding is duplicated in `package_release.py` — keep the two in lockstep or `postFlashProtocolCheck` disagrees with the manifest.

## Component-specific gotchas

- `on_*()` packet handlers run in **USB-RX interrupt context**. No `debug_printf` on success paths — it starved the main loop and tripped the WWDG (`cpp_main.cpp:492-495`); take `__disable_irq()` snapshots the way `motors_handler()` does.
- `MOWGLI_COMMS_MAX_HANDLERS` is 16 and `init_ROS()` registers 9. A registration past the cap is **silently dropped** (only a `debug_printf`) and surfaces as "Firmware incompatible / vunknown" forever — history in `include/mowgli_comms.h:73-83`.
- **Runtime parameters (protocol v7) are persisted in flash.** The bridge sends every parameter with `SET_PARAM` (0x58) in its reconnect burst, then `PARAM_COMMIT` (0x5A) + `GET_PARAM(ALL)` (0x59); the firmware reports `PARAM_VALUE` (0x13) / `PARAM_STORE_STATUS` (0x14). `fw_params_init()` (`main.c`, BEFORE `WATCHDOG_vInit()`) loads the newest valid record from the append-only log (`include/fw_param_log.h`) in the reserved region (`src/fw_param_store.c`: F103 last 4 pages at `0x0803E000`, F401 sector 5 at `0x08020000`; `board_upload.maximum_size` in `platformio.ini` keeps the image out of it), and `init_ROS()` applies it — so the board runs the stored set before the host connects. Erasing blocks far past the ~40 ms WWDG window, so it happens **only at boot** (log full or foreign data); a runtime commit programs one word per `spinOnce()` and skips an unchanged set. The gyro bias is volatile (never stored). `board.h` values are only the default when nothing is stored. Never "fix" a tuning bug by editing `board.h`.
- **Envelope, not tighten-only.** Each parameter has a stable wire id and an absolute `[min, max]` in `include/fw_param_catalog.h` (ids key the flash records — never renumber or reuse one). A value may be stricter or looser than the compiled default but never leaves the envelope (charge 25.2–29.4 V / 0.1–1.2 A, `max_mps` 0.1–0.6, trips ≥10 ms and ≤5000/3000/1000/250 ms, play-clear 500–10000 ms, tilt threshold 44–64); `charger.c`, `emergency.c` and `drivemotor.c` re-clamp to the same envelope. `I_DONT_NEED_MY_FINGERS` stays compile-time only — no packet can disable a sensor. The host mirror is `FirmwareParamId` (`ll_datatypes.hpp`) + `firmware_params.hpp`, pinned by `test_fw_param_catalog.cpp`; the `kMin/MaxRuntime*` ROS ranges in `hardware_bridge_node.cpp` mirror the drive/yaw envelopes — change both.
- `on_set_param()` (USB IRQ) only stores the coerced value and marks its group dirty; `spinOnce()` → `apply_param_groups()` applies it from the main loop. The tilt threshold goes through `I2C_Onboard_SetInclinationThreshold()`, which rewrites INT1_THS from the `SENSOR_POLL` state so the register audit never sees a mismatch (a mismatch latches a sensor fault).
- `main_eOpenmowerStatus` boots to `OPENMOWER_STATUS_IDLE`: until the host sends an `HL_STATE ≠ IDLE`, `cmd_vel` is dropped and the blade target is forced 0 (gates `cpp_main.cpp:408` / `:660`, state set in `on_hl_state()` `:595`, re-asserted `:835-845`). `HL_MODE_*` is **triplicated** — `mowgli_protocol.h:418-422` ↔ `HighLevelStatus.msg` ↔ `hardware_bridge_node.cpp:75-79`.
- **Header comments lie about cadence.** `mowgli_protocol.h:302` says "~25 Hz" status (actual 4 Hz), `:123`/`:391` heartbeat "~250/500 ms" (actual timeout 2000 ms), `mowgli_comms.h:30` "pkt_imu_t = 40 bytes" (41). Trust `cpp_main.cpp:64-69` and the `_Static_assert`s.
- Firmware `HEARTBEAT_TIMEOUT_MS` is 2000 vs the bridge's 4 Hz `heartbeat_rate` (`install/config/mowgli/hardware_bridge.yaml`) — lower that rate and the board e-stops itself.
- `chargecontrol_is_charging` carries the `CHARGER_STATE_e` value (1 connected / 2 CC / 3 CV), not a bool; `STATUS_BIT_CHARGING` is set for any non-zero. `batt_percentage` is hard-coded 0 (`cpp_main.cpp:1318`) although `charger.c` maintains an RTC-persisted SOC.
- `ros2/src/mowgli_hardware/firmware/` is a **stale v3 copy** of these C files (its `MAX_HANDLERS` is still 8) and is built by nothing. The real host mirror is `ll_datatypes.hpp`.
- Generated `src/ros/ros_lib/mower_msgs/*.h` are **excluded from the build** (`platformio.ini` `build_src_filter`) and no source includes them — the wire is `pkt_*_t` only. `sync_ros_lib.py` keeps that dead output in sync purely to satisfy the CI gate.
- `patch_usb.py` runs pre-build and copies `CDC/` **over your PlatformIO package cache** (`~/.platformio/packages/framework-stm32cubef1`, F1 only) — a build mutates the host toolchain.
- `DEBUG_TYPE_UART` is a `#error` on the 500B (`main.c:1277`) — SWO only there. On ORIG the debug UART is `MASTER_USART` (UART4 on J18), DMA, best-effort drop-if-busy.
- USB telemetry is gated by `CDC_ShouldSendTelemetry()` / `CDC_IsComportOpen()` (`src/usbd_cdc_if.c:933-939`): with no host holding the port open, TX stops on purpose. Do not "fix" silence with an unconditional `CDC_Transmit`.
- Encoder glitch rejection scales with the **runtime** `ticks_per_meter` (`drivemotor_max_ticks_per_frame()` = `MAX_MPS × tpm × 0.02 × 3`), so a wrong `ticks_per_meter` from `mowgli_robot.yaml` also changes what counts as a glitch (`src/drivemotor.c:172`, applied `:544`).
- `platformio.ini` has **no `LUV1000RI` env** even though the GUI board list maps to one; the comment block at the end of that file says the MCU/pinout and blade-UART wiring are unknown. Do not add a guessed env — a wrong pinout bricks the board.
- `stm32/ros_usbnode/.gitignore` lists `platformio.ini` although it is tracked, and `Yardforce500_REMOTE_UPLOAD` names an untracked `raspi_remote_upload.py` (also gitignored) plus a hardcoded `custom_mowgli_host` IP. Neither is a bug to "clean up" blindly.

## Safety

This tree **is** the safety authority described in the root CLAUDE.md § *Safety — READ FIRST*: blade enable, the emergency latch, the charge envelope and the `ANTIDIG_*` cutout all decide here, and ROS2 only requests. Consequences specific to this directory:

- `Emergency_SetState()` asserts/releases only — the old "disable checking" opcodes were deliberately removed (`src/emergency.c:103-110`); do not reintroduce them, and do not add a wire packet that can disable a sensor.
- `I_DONT_NEED_MY_FINGERS` (`include/board.h`, template `{{.DisableEmergency}}`) compiles `EmergencyController()` out entirely. It is a bench-only switch; `board_defaults.h` never sets it.
- Weakening `ANTIDIG_*` to compensate for the host dig detector is explicitly banned by the root "What NOT to Do" — the two cover different failures.
- **Remove the blades** before any bench work: the custom firmware has no tilt sensing on the bench harness, and a flash reboots the board with motors powered.

## Sensor recovery ownership

Onboard LIS3DH I2C1 transactions and recovery belong to I2C_Onboard_Service in the main loop. Interrupt callbacks use health/tilt snapshots only. Failed/stale readings inhibit motion and detected faults stay latched after recovery until explicit release. See [I2C-RECOVERY.md](stm32/ros_usbnode/I2C-RECOVERY.md); test both recovery paths with firmware/scripts/test_soft_i2c_recovery.py and test_onboard_i2c_recovery.py. The external IMU uses a separate software-I2C bus.
