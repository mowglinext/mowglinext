# Codemap: firmware

> STM32 firmware for the Mowgli mainboard. The live target is `firmware/stm32/ros_usbnode/` (PlatformIO, stm32cube HAL): it owns the wheel-velocity PI loops, the gyro yaw-rate loop, the always-on `ANTIDIG_*` cutout, blade/charger/emergency authority, the IMU/panel drivers, and the COBS+CRC-16 USB-CDC link to `hardware_bridge_node`. `mowgli_protocol.h` is the single wire-format source of truth (mirrored by hand into `ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp`). Secondary trees: `custom_panel_fw/` (STM32F0 panel replacement), `test_code/` (UART-proxy bring-up), `mainboard_firmware/` + `panel_firmware/` (stock backup/restore via OpenOCD).
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `firmware/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Add / change a wire packet | `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` (`PKT_ID_*`, `pkt_*_t`, `_Static_assert` sizes) → bump `MOWGLI_PROTOCOL_VERSION` (:59) → mirror in `ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp` (`kMowgliProtocolVersion` :51) → `python3 firmware/scripts/protocol_version_guard.py` to refresh `firmware/scripts/protocol_baseline.json` |
| Handle a new Host→Firmware packet | `firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp` `init_ROS()` (:1435, `mowgli_comms_register_handler` ×10) + a `static void on_*()` handler; check `MOWGLI_COMMS_MAX_HANDLERS` (`include/mowgli_comms.h:83`, 16) |
| Change telemetry cadence | `cpp_main.cpp:64-69` (`IMU_NBT_TIME_MS` 10, `MOTORS_NBT_TIME_MS` 20, `STATUS_NBT_TIME_MS` 250, `PANEL_NBT_TIME_MS` 100, `LED_NBT_TIME_MS` 1000, `BLADE_NBT_TIME_MS` 250); `broadcast_handler()` :1255 |
| Per-wheel velocity PI (gains, feedforward, anti-windup) | `cpp_main.cpp` `motors_handler()` :813 (`#if USE_WHEEL_PI` block :986-1101); defaults `WHEEL_PI_KP_PWM_PER_MPS` 30 / `WHEEL_PI_KI_PWM_PER_MPS_S` 5000 / `WHEEL_PI_INT_MAX_PWM` 100 (:111-115); PID core `include/pid.hpp` |
| Gyro yaw-rate loop (Option C) | `cpp_main.cpp:180-249` (`YAW_PI_KP_DEFAULT` 0.30, `YAW_PI_KI_DEFAULT` 0.40, `YAW_TRIM_LIMIT_MPS_DEFAULT` 0.15, `YAW_GYRO_LP_ALPHA` 0.30, `YAW_TRIM_SLEW_MPS_PER_CYCLE` 0.03, turn-exit/low-speed constants) and the loop body :858-965; runtime retune via `on_set_yaw_pid()` :498 |
| Firmware anti-dig (CLAUDE.md Invariant 16 backstop) | `cpp_main.cpp:129-164` (`ANTIDIG_WINDOW_MS` 1500, `ANTIDIG_MIN_TARGET_MPS` 0.02, `ANTIDIG_MIN_ABS_PWM` 60, `ANTIDIG_PROGRESS_FRACTION` 0.30), `antidig_step()` :774, applied :1083-1094; manual field-test procedure :765-773 |
| Heartbeat / cmd_vel watchdogs | `cpp_main.cpp` `HEARTBEAT_TIMEOUT_MS` 2000 (:273) + latch logic :1114-1122; `on_heartbeat()` :356 (`heartbeat_only_latch` auto-clear); cmd_vel 200 ms hard-stop + 25 s blade-off :847-856 |
| IDLE gate (no motion / no blade while docked) | `cpp_main.cpp` `on_hl_state()` :595 (`HL_MODE_*` → `main_eOpenmowerStatus`), re-asserted in `motors_handler()` :835-845, `on_cmd_blade()` :649 (gate :660), `on_cmd_vel()` :399 (gate :408) |
| Emergency sensors, trip timeouts, play-button clear | `firmware/stm32/ros_usbnode/src/emergency.c` `EmergencyController()` :187, `emergency_set_timeouts()` :73 (clamps :58-71), `Emergency_SetState()` :112; defaults in `include/board_defaults.h:88-102` |
| Charge envelope / CC-CV state machine / SOC | `firmware/stm32/ros_usbnode/src/charger.c` `ChargeController()` :211, `charger_set_charge_limits()` :77 (lower-only clamps :64-75); `MAX_CHARGE_VOLTAGE` 29.4 / `MAX_CHARGE_CURRENT` 1.2 in `include/board_defaults.h:44-49`; ADC + RTC-backup persistence `src/adc.c:244-248` |
| Blade motor | `firmware/stm32/ros_usbnode/src/blademotor.c` (`BLADEMOTOR_Set()` :246, `BLADEMOTOR_App()` :215, `BLADEMOTOR_ReceiveIT()` :268; exported `BLADEMOTOR_bActivated/u16RPM/u16Power/u32Error`); only caller `motors_handler()` :1124 |
| Drive-motor UART (PAC5210), encoder sign / reset filter, runtime caps | `firmware/stm32/ros_usbnode/src/drivemotor.c` (`DRIVEMOTOR_App_10ms()` :328, `DRIVEMOTOR_App_Rx()` :558 → `wheelTicks_handler`, `DRIVEMOTOR_UpdateWheel()`/`resolve_direction()` :490-556, `DRIVEMOTOR_SetSpeedSigned()` :648, `g_ticks_per_meter`/`g_max_mps` :137-141, clamps :143-170) |
| Odometry packet contents | `cpp_main.cpp` `wheelTicks_handler()` :1195 (signed cumulative ticks + firmware-side mm/s) |
| IMU driver selection / add an IMU | `firmware/stm32/ros_usbnode/src/imu/imu.c` `IMU_Init()` :137 (probe order LSM6 → WT901 → MPU6050 → ICM45686; mag LIS3MDL :188); driver files `src/imu/*.c`, `include/imu/*.h`; `DISABLE_<IMU>` macros skip a probe |
| Onboard LIS3DH tilt interrupt threshold | `firmware/stm32/ros_usbnode/src/i2c.c:237` (`lis3dh_int1_gen_threshold_set(..., IMU_ONBOARD_INCLINATION_THRESHOLD)`); clamp 0x2C..0x40 → 0x38 in `include/board_defaults.h:115-121` |
| Panel LEDs / buttons | `firmware/stm32/ros_usbnode/include/panel.h` (`PANEL_LED_*` per `PANEL_TYPE`, `PANEL_BUTTON_DEF_*`), `src/panel.c` (`PANEL_Set_LED()` :222, `PANEL_Tick()` :251); UI events in `cpp_main.cpp` `panel_handler()` :1131 |
| Board variant / pinout / compile-time defaults | `firmware/stm32/ros_usbnode/include/board.h` (`MAX_MPS`, `PWM_PER_MPS`, `TICKS_PER_M`, `WHEEL_BASE`, `PANEL_TYPE`, `DEBUG_TYPE`, USART instances); GUI-rendered twin `include/board.h.template` (`{{.Field}}` placeholders); shared safety defaults `include/board_defaults.h` |
| Watchdog reset diagnosis (WWDG breadcrumb) | `firmware/stm32/ros_usbnode/src/main.c` `WATCHDOG_vInit()` :1322, `HAL_WWDG_EarlyWakeupCallback()` :1388, `WATCHDOG_LoadBootBreadcrumb()` :433, stage names :303; `WATCHDOG_STAGE_*` ids in `mowgli_protocol.h:226-272` |
| USB CDC TX back-pressure / "host not reading" | `firmware/stm32/ros_usbnode/src/usbd_cdc_if.c` `CDC_ShouldSendTelemetry()` :939, `CDC_IsComportOpen()` :933, `CDC_TXQueue_GetWriteAvailable()` :630, `CDC_TX_BUSY_TIMEOUT_MS`/`CDC_TELEMETRY_PROBE_MS` :78-79; gating helpers `cpp_main.cpp:327-350` |
| USB VID/PID/product string | `firmware/stm32/ros_usbnode/src/usbd_desc.c:65-69` (VID 1155 = 0x0483, PID 22336 = 0x5740, "Mowgli") |
| Reported firmware version (semver in `pkt_config_rsp_t`) | `firmware/stm32/ros_usbnode/git_build_id.py` (PlatformIO pre hook; `major = 1 | 0x80 if dirty`, `minor.patch` = git commit count) — same encoding duplicated in `firmware/scripts/package_release.py:38-77` |
| Prebuilt-binary release / manifest | `firmware/scripts/package_release.py` (`PERMUTATIONS` :48) + `.github/workflows/firmware-ci.yml` `release` job; consumer `gui/pkg/providers/firmware.go` `flashPrebuilt` (:258) via `gui/pkg/providers/firmware_manifest.go:21` `DefaultFirmwareManifestURL` |
| Safety-default drift guard | `firmware/scripts/board_defaults_parity.py` (`MANAGED_MACROS` :40) — CI job `defaults-parity` in `firmware-ci.yml` |
| Stock firmware backup / restore | `firmware/stm32/mainboard_firmware/README.md` (SHA256 table :113-124), `backup_firmware.sh`, `restore_firmware.sh`; panel: `firmware/stm32/panel_firmware/` |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **`firmware/` root + scripts** | | |
| `firmware/README.md` | 53 | Area overview (partly stale — see structured findings) |
| `firmware/.gitignore` | 3 | ignores `sync_msg.sh`, `.idea/` |
| `firmware/scripts/protocol_version_guard.py` | 138 | Fingerprints `PKT_ID_*` + `pkt_*_t` in `mowgli_protocol.h`; `--check` fails on un-versioned wire drift or firmware/host version mismatch |
| `firmware/scripts/protocol_baseline.json` | 4 | `{version: 6, hash}` baseline for the guard above |
| `firmware/scripts/board_defaults_parity.py` | 108 | Asserts `board.h`/`board.h.template` consume `board_defaults.h` and never re-`#define` the 10 managed safety macros |
| `firmware/scripts/package_release.py` | 182 | Stamps `.pio/build/<env>/firmware.{bin,elf}` → `mowgli-fw_<board>_<panel>_p<proto>_v<ver>_<sha>.bin` + `manifest.json` |
| `firmware/scripts/sync_ros_lib.py` | 395 | Generates rosserial-style `ros_lib/mower_msgs/*.h` from `ros2/src/mowgli_interfaces/msg/*.msg` (output is NOT compiled — see Pitfalls) |
| **`firmware/stm32/ros_usbnode/` — build & tooling** | | |
| `firmware/stm32/ros_usbnode/platformio.ini` | 81 | Envs `Yardforce500` (genericSTM32F103VC), `Yardforce500_STLINK_V3`, `Yardforce500B` (custom `boards/genericSTM32F401VC.json`), `Yardforce500B_STLINK_V3`, `Yardforce500_REMOTE_UPLOAD`; `build_src_filter` excludes `proxy_inc/**` and `ros/ros_lib/**`; pre-scripts below |
| `firmware/stm32/ros_usbnode/boards/genericSTM32F401VC.json` | 46 | PlatformIO board manifest for the 500B (Cortex-M4; declares `f_cpu` 84 MHz — `SystemClock_Config()` actually runs it at 72 MHz) |
| `firmware/stm32/ros_usbnode/git_build_id.py` | 56 | pre: injects `MOWGLI_FW_VERSION_{MAJOR,MINOR,PATCH}` `-D` flags from git |
| `firmware/stm32/ros_usbnode/patch_usb.py` | 21 | pre: copies `CDC/` over `~/.platformio/packages/framework-stm32cubef1/.../Class/CDC/` (mutates the host toolchain) |
| `firmware/stm32/ros_usbnode/add_swo_viewer.py` | 61 | pre: registers custom target `swo_viewer` (OpenOCD ITM trace + `swo_parser.py`) |
| `firmware/stm32/ros_usbnode/swo_parser.py` | 197 | ITM/SWO stream decoder used by the `swo_viewer` target |
| `firmware/stm32/ros_usbnode/cppcheck-suppressions.txt` | 18 | cppcheck suppressions — referenced by nothing; the only cppcheck CI job (`ros2-ci.yml`) scans `ros2/src/` only |
| `firmware/stm32/ros_usbnode/remote_upload/yardforce500.cfg`, `prog.cfg` | 6, 1 | OpenOCD ST-Link v2 program scripts for `/tmp/firmware.bin` (paired with gitignored `raspi_remote_upload.py`) |
| `firmware/stm32/ros_usbnode/README.md` | 218 | Legacy rosserial/ROS-noetic how-to (stale — see findings) |
| `firmware/stm32/ros_usbnode/.gitignore` | 13 | ignores `.pio`, `raspi_remote_upload.py`, `*.bin`… and (already-tracked) `platformio.ini` |
| **`ros_usbnode/include/` — protocol & control** | | |
| `include/mowgli_protocol.h` | 780 | `MOWGLI_PROTOCOL_VERSION 6u`; `PKT_ID_*`; `STATUS_BIT_*`, `EMERGENCY_BIT_*`, `RESET_CAUSE_*`, `WATCHDOG_STAGE_*`, `HL_MODE_*`, `CONFIG_FLAG_FIRMWARE_DEBUG`; packed `pkt_*_t` + `_Static_assert` layout pins |
| `include/mowgli_comms.h` | 243 | RX frame assembly / CRC / dispatch API; `MOWGLI_COMMS_RX_BUF_SIZE` 512, `MOWGLI_COMMS_MAX_HANDLERS` 16; extern `usb_cdc_transmit()` |
| `include/cobs.h`, `include/crc16.h` | 80, 41 | COBS encode/decode; CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) |
| `include/pid.hpp` | 140 | Vendored PX4 PID (header-only, BSD-3): `setGains/setIntegralLimit/setOutputLimit/update(feedback, dt, update_integral)/setIntegral` |
| `include/board.h` | 306 | Committed board config compiled by CI; includes `board_defaults.h` (:112) |
| `include/board.h.template` | 348 | Go-template twin rendered by the GUI (`providers.FirmwareConfig`); `{{.BoardType}}`, `{{.PanelType}}`, `{{.MaxMps}}`, `{{.TickPerM}}`, `{{.WheelBase}}`, `{{.DisableEmergency}}`, `{{.PerimeterWire}}`, charge/e-stop fields, `{{.ImuOnboardInclinationThreshold}}`, `{{.ExternalImuAcceleration}}`, `{{.ExternalImuAngular}}` |
| `include/board_defaults.h` | 123 | `#ifndef`-guarded single source of the 10 safety defaults + fixed battery constants (`LOW_BAT_THRESHOLD` 25.2, `LOW_CRI_THRESHOLD` 23.5, `MIN_DOCKED_VOLTAGE` 20.0…) |
| `include/main.h` | 152 | `openmower_status_e` (`OPENMOWER_STATUS_MOWING/DOCKING/UNDOCKING/IDLE/RECORD`), `DB_TRACE`, globals (`g_boot_reset_cause_code`, `g_firmware_debug_enabled`, `main_eOpenmowerStatus`), `WATCHDOG_SetMainLoopStage()` |
| `include/drivemotor.h`, `blademotor.h`, `charger.h`, `emergency.h`, `panel.h`, `adc.h` | 92, 64, 67, 31, 178, 85 | Driver APIs (see "Where to look"); `battery.h`/`src/battery.c` (55/44) are empty stubs |
| `include/imu/imu.h` + `imu/{lsm6,wt901,mpu6050,icm45686,lis3mdl}.h` | 63 + 98/56/37/201/32 | IMU abstraction (`IMU_TryRead{Accelerometer,Gyro,Mag}`, `RAD_PER_DEG`, `MS2_PER_G`, `T_PER_GAUSS`) and per-chip drivers |
| `include/i2c.h`, `soft_i2c.h`, `perimeter.h`, `ultrasonic_sensor.h` | 14, 49, 89, 21 | Hard I2C (onboard LIS3DH), bit-banged I2C (J18 external IMU), optional perimeter (`OPTION_PERIMETER`) and USS (`OPTION_ULTRASONIC`) |
| `include/stm32f_board_hal.h`, `stm32f1xx_it.h`, `stm32f4xx_it.h` | 22, 69, 80 | Variant HAL include switch + IRQ prototypes |
| `include/usb_device.h`, `usbd_cdc_if.h`, `usbd_conf.h`, `usbd_desc.h` | 101, 225, 174, 143 | USB device stack glue headers |
| **`ros_usbnode/src/` — runtime** | | |
| `src/main.c` | 1447 | Boot (reset-cause decode + LED blink code), init order (:461-586), main loop (:588-679), IWDG+WWDG (:1322-1392), debug UART/SWO `debug_printf`, `StatusLEDUpdate()`, `chirp()` |
| `src/ros/ros_custom/cpp_main.cpp` | 1489 | COBS handlers, motors/yaw/anti-dig control, panel events, odometry + IMU + status + blade broadcasts, `init_ROS()` |
| `src/ros/ros_custom/cpp_main.h` | 46 | C-linkage API called from `main.c` (`init_ROS`, `motors_handler`, `broadcast_handler`, `wheelTicks_handler`, `CDC_DataReceivedHandler`, `usb_cdc_transmit`) |
| `src/ros/ros_custom/nbt.{cpp,h}` | 28, 31 | Non-blocking timer (`NBT_init`, `NBT_handler`) |
| `src/ros/ros_custom/ringbuffer.{cpp,h}` | 313, 75 | Legacy rosserial ring buffer — compiled, no callers |
| `src/mowgli_comms.c` | 324 | Frame scan (0x00 delimiters), size guard (`MAX_RAW_PKT_SIZE` 64), CRC verify, handler table, `mowgli_comms_send*()` |
| `src/cobs.c`, `src/crc16.c` | 119, 43 | Implementations |
| `src/drivemotor.c` | 743 | PAC5210 UART protocol (38/12/20-byte frames), encoder fold + direction filter, runtime `g_ticks_per_meter`/`g_max_mps` |
| `src/blademotor.c` | 290 | Blade ESC UART protocol (22/7-byte frames), status decode |
| `src/charger.c` | 325 | TIM1 charge PWM (0..1350), `CHARGER_STATE_*` machine, current-offset + Ah persistence in RTC backup regs |
| `src/adc.c` | 435 | TIM2-paced ADC of `battery_voltage`, `charge_voltage`, `current`, `blade_temperature`, `chargerInputVoltage`; RTC init |
| `src/emergency.c` | 381 | Sensor debounce/trip timers, latch bits, play-button clear, 5 s buzzer |
| `src/panel.c` | 415 | Panel UART (init/LED/request frames), button decode, `HAL_UARTEx_RxEventCallback` |
| `src/imu/imu.c`, `imu/imu_mag_trans.c`, `imu/{lsm6,wt901,mpu6050,icm45686,lis3mdl}.c` | 195, 66, 149/247/89/162/56 | IMU probe/dispatch, hard/soft-iron mag transform, chip drivers |
| `src/i2c.c`, `src/soft_i2c.c` | 267, 718 | Onboard LIS3DH (tilt INT) via HAL I2C; bit-banged I2C master |
| `src/perimeter.c`, `src/ultrasonic_sensor.c` | 370, 95 | Optional sensors (off in committed `board.h`) |
| `src/usbd_cdc_if.c` | 1078 | CDC class glue: RX/TX queues, `CDC_Transmit`, busy-stuck recovery, host-open detection, USB reset/suspend notifications, diagnostic counters |
| `src/usb_device.c`, `src/usbd_desc.c` | 119, 392 | USB device init and descriptors |
| `src/stm32f_it.c`, `src/startup_stm32f.S`, `src/usbd_conf.c` | 5, 5, 5 | Variant shims that `#include` `src/proxy_inc/stm32f{1,4}/{stm32fXxx_it.c, startup_stm32fXxx.S, usbd_conf.c}` (405/0/669 and 385/431/676 lines) |
| `src/ros/generate_ros_lib.sh`, `src/ros/update_ros_lib.sh` | 8, 184 | Legacy rosserial `ros_lib` regeneration (scp from a ROS1 host / docker `open_mower_ros` image) |
| `test/README`, `lib/README`, `include/README` | 11, 46, 39 | PlatformIO placeholders — no firmware unit tests exist |
| **Secondary trees** | | |
| `firmware/stm32/custom_panel_fw/YF500C_panel/` | src 274+81+145+247 | STM32F0/GD32F303 replacement panel firmware (`platformio.ini` env `GD32F303_STM32F030`, board `disco_f051r8`); `flash_firmware.sh`, `trace.sh`, `configure-trace.cfg`, `gd32f303.cfg`, `yardforce500.cfg`, `firmware_bin.py` (post: bin) |
| `firmware/stm32/test_code/` | main.c 926 (+ USB glue, vendored `lis3dh_reg.c` 2740) | Pre-ROS "UART proxy" bring-up firmware; envs `genericSTM32F103C8` (`-DBOARD_BLUEPILL`) and `Yardforce 500 (STM32F103 VCT6)` (`-DBOARD_YARDFORCE500`) |
| `firmware/stm32/mainboard_firmware/` | README 124 + 3 scripts | OpenOCD dump/restore of stock mainboard flash (`0x08000000`, 256 KB) + known-good SHA256 table |
| `firmware/stm32/panel_firmware/` | README 53 + 4 scripts | OpenOCD dump/restore of stock panel flash (64 KB) + RAM dump |

## Runtime surface

### Wire link (USB CDC, COBS-framed, CRC-16/CCITT-FALSE little-endian, `[0x00][COBS][0x00]`)

| ID | Name / struct | Dir | Size | Producer / handler | Cadence & notes |
|----|---------------|-----|------|--------------------|-----------------|
| 0x01 | `PKT_ID_STATUS` / `pkt_status_t` | F→H | 38 | `broadcast_handler()` | 4 Hz (`STATUS_NBT_TIME_MS` 250); `status_bitmask` (`STATUS_BIT_INITIALIZED\|RASPI_POWER\|CHARGING\|RAIN\|UI_AVAIL`), `emergency_bitmask` (`LATCH\|STOP\|LIFT`), `v_charge`, `v_system`, `charging_current`; `batt_percentage` always 0; `uss_ranges_m[5]` zero unless `OPTION_ULTRASONIC` |
| 0x06 | `PKT_ID_RESET_CAUSE` / `pkt_reset_cause_t` | F→H | 5 | `broadcast_handler()` | sent immediately before every STATUS; `reset_cause` + WWDG breadcrumb `last_stage_before_reset` |
| 0x02 | `PKT_ID_IMU` / `pkt_imu_t` | F→H | 41 | `broadcast_handler()` | 100 Hz (`IMU_NBT_TIME_MS` 10); accel/gyro/mag in m/s², rad/s, µT; skipped entirely if any `IMU_TryRead*` fails |
| 0x04 | `PKT_ID_ODOMETRY` / `pkt_odometry_t` | F→H | 17 | `wheelTicks_handler()` ← `DRIVEMOTOR_App_Rx()` | per motor-controller frame (~20 ms); signed cumulative ticks + firmware-computed mm/s |
| 0x05 | `PKT_ID_BLADE_STATUS` / `pkt_blade_status_t` | F→H | 16 | `broadcast_handler()` | 4 Hz (`BLADE_NBT_TIME_MS`), only once a heartbeat has been seen |
| 0x03 | `PKT_ID_UI_EVENT` / `pkt_ui_event_t` | F→H | 5 | `panel_handler()` | `button_id` 1=S1 2=S2 3=LOCK 4=START 5=HOME; `press_duration` always 0 |
| 0x12 | `PKT_ID_CONFIG_RSP` / `pkt_config_rsp_t` | F→H | 8 | `on_config_req()` | `protocol_version`, `active_flags`, `fw_version_{major,minor,patch}` |
| 0x11 | `PKT_ID_CONFIG_REQ` / `pkt_config_req_t` | H→F | 4 | `on_config_req()` | bridge sends on every (re)connect; `flags & CONFIG_FLAG_FIRMWARE_DEBUG` → `g_firmware_debug_enabled` (enables fine-grained watchdog stages) |
| 0x42 | `PKT_ID_HEARTBEAT` / `pkt_heartbeat_t` | H→F | 5 | `on_heartbeat()` | bridge `heartbeat_rate` 4 Hz; absent > `HEARTBEAT_TIMEOUT_MS` 2000 → `Emergency_SetState(1)`; `emergency_requested` / `emergency_release_requested` (release refused while any physical sensor asserted) |
| 0x43 | `PKT_ID_HL_STATE` / `pkt_hl_state_t` | H→F | 5 | `on_hl_state()` | `HL_MODE_NULL/IDLE/AUTONOMOUS/RECORDING/MANUAL_MOWING` (0-4, must equal `HighLevelStatus.msg` `HIGH_LEVEL_STATE_*`); `gps_quality` < 90 → LOCK LED off |
| 0x50 | `PKT_ID_CMD_VEL` / `pkt_cmd_vel_t` | H→F | 11 | `on_cmd_vel()` | IK `v ± wz·wheel_base/2`, clamped ±`DRIVEMOTOR_GetMaxMps()`; ignored in IDLE; stale > 200 ms → hard stop |
| 0x51 | `PKT_ID_CMD_BLADE` / `pkt_cmd_blade_t` | H→F | 5 | `on_cmd_blade()` | fire-and-forget; target forced 0 in IDLE and on emergency |
| 0x52 | `PKT_ID_REBOOT` / `pkt_reboot_t` | H→F | 4 | `on_reboot()` | `magic == PKT_REBOOT_MAGIC` (0xB0) → `NVIC_SystemReset()` on next `chatter_handler` tick |
| 0x54 | `PKT_ID_SET_DRIVE_PID` / `pkt_set_drive_pid_t` | H→F | 27 | `on_set_drive_pid()` | `ticks_per_meter` 50..5000, `kp` 0..200, `ki` 0..20000, `kd` 0..500, `integral_limit` 0..255, `pwm_per_mps` 50..600; non-finite → whole packet rejected |
| 0x55 | `PKT_ID_SET_YAW_PID` / `pkt_set_yaw_pid_t` | H→F | 21 | `on_set_yaw_pid()` | `yaw_kp` 0..5, `yaw_ki` 0..20, `trim_limit_mps` 0..max_mps, `enabled`, `gyro_sign` ±1, `gyro_bias_radps` ±0.5 (v6 field) |
| 0x56 | `PKT_ID_SET_KINEMATICS` / `pkt_set_kinematics_t` | H→F | 11 | `on_set_kinematics()` | `max_mps` clamped to (0.1, compiled `MAX_MPS`] (lower-only), `wheel_base` 0.15..0.60 |
| 0x57 | `PKT_ID_SET_SAFETY_LIMITS` / `pkt_set_safety_limits_t` | H→F | 21 | `on_set_safety_limits()` | charge V/I lower-only; 4 trip timeouts shorten-only (floor 10 ms); `play_clear_ms` lengthen-only (cap 10 s) |

No config persistence on the board: the bridge re-sends 0x54/0x55/0x56/0x57 after every connect/handshake (`hardware_bridge_node.cpp:923-931`); compile-time values are the power-on fallback.

### Main loop (`src/main.c:588-679`, cooperative, no RTOS)
`chatter_handler` (1 s LED + reboot) → `motors_handler` (20 ms: snapshot → hard-stop decision → yaw loop → wheel PIs → anti-dig → `DRIVEMOTOR_SetSpeedSigned` → heartbeat check → `BLADEMOTOR_Set`) → `panel_handler` (100 ms) → `spinOnce` (no-op) → `broadcast_handler` (one packet group per pass) → `DRIVEMOTOR_App_Rx` → [`Perimeter_vApp`] → every 10 ms `ADC_input` + `ChargeController` → 1 s `StatusLEDUpdate` → 10 ms `WATCHDOG_Refresh` → 20 ms `DRIVEMOTOR_App_10ms` → 100 ms `BLADEMOTOR_App` → 200 ms buzzer → 10 ms `EmergencyController` (compiled out by `I_DONT_NEED_MY_FINGERS`).
USB RX runs in interrupt context: `CDC_DataReceivedHandler` → `mowgli_comms_process_rx` → `on_*` handlers (hence the `__disable_irq()` snapshots in `motors_handler` and the handlers).

### Watchdogs & diagnostics
IWDG (prescaler 256, reload 0xFFF) + WWDG (prescaler 8, counter 0x7F ≈ 40 ms, EWI on). `HAL_WWDG_EarlyWakeupCallback` writes `g_main_loop_stage` to RTC backup `RTC_BKP_DR5`; next boot reports it in `pkt_reset_cause_t` when `reset_cause == RESET_CAUSE_WWDG`. Boot LED blinks the reset-cause code 1-7 (`BOOT_BlinkResetCause`). Debug text: `DEBUG_TYPE_UART` (ORIG: `MASTER_USART` = UART4 on J18, since `board.h:127` defines `MASTER_J18`; 115200, DMA, best-effort drop-if-busy) or `DEBUG_TYPE_SWO` (500B). RTC backup DR1-4 hold Ah accumulator + charge-current offset.

### Peripherals per variant (`include/board.h`)
| | `Yardforce500` (`BOARD_YARDFORCE500_VARIANT_ORIG`) | `Yardforce500B` (`BOARD_YARDFORCE500_VARIANT_B`) |
|--|--|--|
| MCU / clock | STM32F103VC, HSE×9 = 72 MHz | STM32F401VC, HSE 8 MHz, PLLM 4 / PLLN 144 / PLLP 4 → 72 MHz |
| Drive / blade / panel UART | USART2 / USART3 / USART1 | (USART2) / USART6 / USART1 |
| `PWM_PER_MPS` / `TICKS_PER_M` / `MAX_MPS` / `WHEEL_BASE` | 337.0 / 339.0 / 0.5 / 0.325 | 275.0 / 277.0 / 0.5 / 0.325 |
| `PANEL_TYPE` | `PANEL_TYPE_YARDFORCE_500_CLASSIC` | `PANEL_TYPE_YARDFORCE_500B_CLASSIC` |
| Debug | UART (`BOARD_HAS_MASTER_USART` 1) | SWO |
`BOARD_LUV1000RI` exists in `board.h`/template (`PANEL_TYPE_YARDFORCE_LUV1000RI`, `OPTION_ULTRASONIC` 1) but has no PlatformIO env (`platformio.ini:72-82`). Committed `board.h` sets `EXTERNAL_IMU_ACCELERATION`/`EXTERNAL_IMU_ANGULAR` 1, `OPTION_ULTRASONIC` 0, `OPTION_BUMPER` 0, no `OPTION_PERIMETER`.

### Host-side consumers
- `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp`: handshake (`send_config_request()` :2558, `fw_compatible_ = (fw_protocol_version_ == kMowgliProtocolVersion)` :2466 — mismatch blocks mowing), `send_drive_pid/yaw_pid/kinematics/safety_limits()` :2214-2326, `heartbeat_rate` param (4 Hz).
- `gui/pkg/providers/firmware.go`: `FirmwareSourcePrebuilt` (default) → manifest → sha256 → flash → `postFlashProtocolCheck`; source path renders `board.h.template` (`buildBoardHeader`) then `platformio run -e {Yardforce500|Yardforce500B|LUV1000RI} -t upload`. Form defaults in `gui/web/src/components/FlashBoardComponent.tsx:487-596`.
- `install/lib/checks.sh` `check_firmware()` :275 reads `/hardware_bridge/status` for `mower_status`.

## Build, test, run

```bash
cd firmware/stm32/ros_usbnode
pio run                       # default_envs = Yardforce500
pio run -e Yardforce500B
pio run -e Yardforce500 -t upload            # ST-Link (upload_protocol = stlink)
pio run -e Yardforce500_STLINK_V3 -t upload  # OpenOCD custom upload_command
pio run -t swo_viewer                        # SWO trace (add_swo_viewer.py)
pio device monitor -b 115200 -p /dev/ttyAMA0 # UART debug (ORIG variant)

# Guards (pure Python, run from repo root)
python3 firmware/scripts/protocol_version_guard.py --check
python3 firmware/scripts/board_defaults_parity.py
python3 firmware/scripts/sync_ros_lib.py --check
python3 firmware/scripts/protocol_version_guard.py        # refresh baseline AFTER a version bump

# Release packaging (what CI does on a v*.*.* tag)
python3 firmware/scripts/package_release.py --build-root firmware/stm32/ros_usbnode --tag vX.Y.Z --repo mowglinext/mowglinext --out-dir dist
```

Tests — there are **no firmware-side unit tests** (`test/README` is the PlatformIO placeholder). What pins firmware behaviour:
- `ros2/src/mowgli_hardware/test/test_protocol.cpp` — `ProtocolSizes.*` (every `Ll*` struct size), `ProtocolIds.PacketIdValues`, `OdometryPacket.FieldOffsetsAreCorrect`/`SignedDirectionEncoding`/`NegativeTicksRoundtrip`, `SetDrivePidPacket.FieldOffsetsAreCorrect`/`PreservesFractionalTicksPerMeter`, `ProtocolRoundtrip.*`, `StatusBitmask.BitPositions`, `EmergencyBitmask.BitPositions` (host mirror only — run via `cd ros2 && make test`).
- `firmware/scripts/protocol_version_guard.py --check` — wire fingerprint vs `protocol_baseline.json` + firmware/host version lockstep.
- `firmware/scripts/board_defaults_parity.py` — single-source safety defaults.
- `gui/pkg/providers/firmware_test.go` — `TestFlashFirmwareRouting`, `TestBuildBoard` (template rendering / env routing).
- Anti-dig, yaw loop, PI loop: manual supervised field procedure only (`cpp_main.cpp:765-773`).

CI: `.github/workflows/firmware-ci.yml` (jobs `defaults-parity`, `build` matrix `Yardforce500`/`Yardforce500B`, `release` on `v*.*.*` tags → attaches `dist/*.bin`, `dist/*.elf`, `dist/manifest.json`), `.github/workflows/protocol-version-drift.yml` (`protocol_version_guard.py --check`), `.github/workflows/msg-codegen-drift.yml` (`sync_ros_lib.py --check`). Only `firmware-ci.yml` triggers on `firmware/**`; the two drift guards trigger on their own explicit file lists (`mowgli_protocol.h` + `ll_datatypes.hpp` + guard + baseline; `mowgli_interfaces/**` + the GUI codegen paths + `sync_ros_lib.py` + `ros_lib/mower_msgs/**`). All push on `feat/**`, `fix/**`… branches.

## Change coupling — "if you change X, also update Y"

- `mowgli_protocol.h` `pkt_*_t` / `PKT_ID_*` → bump `MOWGLI_PROTOCOL_VERSION` AND `ll_datatypes.hpp` `kMowgliProtocolVersion` + the matching `Ll*` struct/`static_assert`s → refresh `protocol_baseline.json` (`protocol_version_guard.py`, refuses without a bump) → update `test_protocol.cpp` sizes/offsets → old boards become "incompatible, reflash" in the bridge/GUI.
- New Host→Firmware packet → `mowgli_comms_register_handler()` in `init_ROS()`; keep count ≤ `MOWGLI_COMMS_MAX_HANDLERS` (registrations past the cap are silently dropped — only a `debug_printf`).
- `HL_MODE_*` (`mowgli_protocol.h:418-422`) ↔ `HighLevelStatus.msg` `HIGH_LEVEL_STATE_*` ↔ `hardware_bridge_node.cpp:75-79` local `HL_MODE_*` constants.
- `board_defaults.h` value → `gui/web/src/components/FlashBoardComponent.tsx` `default={}` for the same field (`board_defaults_parity.py` checks structure only; the GUI defaults are hand-synced) → robot-verify.
- New compile-time knob in `board.h` → add the `{{.Field}}` to `board.h.template` + `providers.FirmwareConfig` (Go) + `FlashBoardComponent.tsx`; keep `board.h` and template `#include "board_defaults.h"`.
- `git_build_id.py` version encoding ↔ `package_release.py` `fw_version()` (hand-kept lockstep; `postFlashProtocolCheck` compares manifest vs `pkt_config_rsp_t`).
- New PlatformIO env → `firmware-ci.yml` build matrix + `release` job + `package_release.py` `PERMUTATIONS` + `firmware.go` env switch (`LUV1000RI` is already referenced by the GUI but has no env).
- `WATCHDOG_STAGE_*` / `RESET_CAUSE_*` → `main.c` `WATCHDOG_StageName()` / `WATCHDOG_StageAlwaysOn()` and `ll_datatypes.hpp` constexprs.
- `TICKS_PER_M` / `MAX_MPS` / `WHEEL_BASE` compile-time → only the fallback; the live values come from `mowgli_robot.yaml` via the bridge (`ticks_per_meter`, kinematics packets). `MAX_MPS` is also the hard ceiling the wire cannot raise.
- Firmware `heartbeat` timeout (2000 ms) vs bridge `heartbeat_rate` (`install/config/mowgli/hardware_bridge.yaml:5`, 4 Hz) — lower the rate and the board e-stops.
- `CDC/` patched middleware → `patch_usb.py` copies it into `~/.platformio/packages/framework-stm32cubef1` on every build (F1 only).

## Pitfalls

- **Generated `ros_lib/mower_msgs/*.h` are not compiled.** `platformio.ini:16-17` excludes `ros/ros_lib/**` and no source includes them; the wire is `pkt_*_t` only. `sync_ros_lib.py` + `msg-codegen-drift.yml` keep dead output in sync.
- `ros2/src/mowgli_hardware/firmware/` is a **stale v3 copy** (`mowgli_protocol.h:60` there says `3u`, `mowgli_comms.h:90` there still has `MAX_HANDLERS 8`); it is not built by `mowgli_hardware/CMakeLists.txt`. The real host mirror is `ll_datatypes.hpp`.
- `MOWGLI_COMMS_MAX_HANDLERS` too small = silent "Firmware incompatible / vunknown" forever (history in `mowgli_comms.h:73-80`); `init_ROS()` registers 10.
- Header comments disagree with code: `mowgli_protocol.h:302` "~25 Hz" status (actual 4 Hz), `:123`/`:391` heartbeat "~250/500 ms" (actual timeout 2000 ms), `mowgli_comms.h:30` "pkt_imu_t = 40 bytes" (41). Trust `cpp_main.cpp` + the `_Static_assert`s.
- `on_*` handlers run in USB-RX interrupt context — no `debug_printf` on success paths (`on_set_drive_pid` comment :492-495: it starved the loop and tripped WWDG).
- `main_eOpenmowerStatus` starts `OPENMOWER_STATUS_IDLE`; until the host sends `HL_STATE` ≠ IDLE, `cmd_vel` is dropped and blade stays off (CLAUDE.md Safety section).
- `Emergency_SetState()` only asserts/releases; the old "disable checking" opcodes were removed (`emergency.c:103-110`) — do not reintroduce.
- `I_DONT_NEED_MY_FINGERS` (`board.h:96-105`, template `{{.DisableEmergency}}`) compiles out `EmergencyController()`; `board_defaults.h` never sets it.
- `DEBUG_TYPE_UART` on the 500B is a `#error` (`main.c:1277`); SWO only.
- `proxy_inc/stm32f1/startup_stm32f1xx.S` is an **empty** file — the F1 startup comes from the framework; the F4 shim carries its own (431 lines).
- `patch_usb.py` mutates the user's PlatformIO package cache; CI caches `~/.platformio` keyed on `platformio.ini`.
- `.gitignore` lists `platformio.ini` though it is tracked (`Yardforce500_REMOTE_UPLOAD` env expects an untracked `raspi_remote_upload.py` and hardcodes host `10.146.111.222`).
- `batt_percentage` is hard-coded 0 (`cpp_main.cpp:1318`); SOC exists in `charger.c` (`SOC`, RTC-persisted Ah) but is not sent.
- `chargecontrol_is_charging` carries the `CHARGER_STATE_e` value (1 connected, 2 CC, 3 CV), not a bool; `STATUS_BIT_CHARGING` is set for any non-zero value.
- Encoder tick anomalies (reversal coast, controller counter reset) are handled in `DRIVEMOTOR_UpdateWheel`/`resolve_direction`; glitches above `drivemotor_max_ticks_per_frame()` (`MAX_MPS × tpm × 0.02 × 3`) are dropped, so a wrong runtime `ticks_per_meter` also changes what counts as a glitch.
- Anti-dig here (`ANTIDIG_*`) covers **blocked** wheels only; slipping wheels are the host `dig_detector` (CLAUDE.md Invariant 16 — do not weaken either).
- `firmware/README.md`, `ros_usbnode/README.md`, `ros2/README.md` § Hardware Protocol, `wiki/Firmware.md`, `wiki/Architecture.md` packet tables are out of date (details in the structured findings) — use `mowgli_protocol.h`.

## Generated & vendored — do not hand-edit

- `firmware/stm32/ros_usbnode/src/ros/ros_lib/**` (564 files) — vendored rosserial headers; `mower_msgs/` subset regenerated by `firmware/scripts/sync_ros_lib.py`; excluded from the build.
- `firmware/stm32/ros_usbnode/CDC/**` — patched ST `STM32_USB_Device_Library` CDC class, copied over the framework by `patch_usb.py`.
- `firmware/stm32/ros_usbnode/include/i2c_lis3dh.h` + `src/i2c_lis3dh.c` (1050 + 2742) and `firmware/stm32/test_code/{include/lis3dh_reg.h,src/lis3dh_reg.c}` — ST LIS3DH register driver.
- `firmware/stm32/ros_usbnode/include/pid.hpp` — vendored PX4 PID (BSD-3; local `pid_constrain`, header-only) — edit only with upstream in mind.
- `firmware/stm32/ros_usbnode/src/proxy_inc/**` — CubeMX-style startup/IT/`usbd_conf` per MCU; `firmware/stm32/custom_panel_fw/YF500C_panel/{src/system_stm32f0xx.c,include/stm32f0xx_hal_conf.h}` — CubeMX output.
- `firmware/scripts/protocol_baseline.json` — written by `protocol_version_guard.py`; `MOWGLI_FW_VERSION_*` — injected by `git_build_id.py` at build time.
