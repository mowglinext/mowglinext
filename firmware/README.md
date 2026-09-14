# Firmware

STM32 firmware for Mowgli robot mower — motor control, IMU, blade safety, and the USB-CDC link to ROS2.

Forked from [cloudn1ne/Mowgli](https://github.com/cloudn1ne/Mowgli).

## Safety

> **Remove the razor blades before any bench work.**

This firmware is the sole blade-safety authority: `EmergencyController()`
(`stm32/ros_usbnode/src/emergency.c`) polls both stop buttons, both wheel-lift sensors and the
tilt / low-Z accelerometer inputs and latches the emergency itself — ROS2 only ever requests.
Two caveats make a bench harness dangerous anyway: those GPIOs must be verified wired and not
floating per chassis (see the warning in `include/board.h`), and the bench-only
`I_DONT_NEED_MY_FINGERS` switch compiles the whole controller out.

## Structure

```
scripts/                # Repo-root Python guards + release packaging (no toolchain needed)
stm32/
├── ros_usbnode/        # Main firmware: ROS2 bridge over USB-CDC, COBS + CRC-16
├── custom_panel_fw/    # Replacement panel controller (GD32F303/STM32F0)
├── test_code/          # Hardware bring-up / UART-proxy firmware
├── mainboard_firmware/ # Stock firmware backup/restore (OpenOCD)
└── panel_firmware/     # Stock panel firmware backup/restore (OpenOCD)
```

## Main Firmware (ros_usbnode)

The active firmware in `stm32/ros_usbnode/` builds for the YardForce 500 Classic (STM32F103VCT6)
and the 500B (STM32F401VC), and provides:

- Drive-motor control — **both** control loops close here: a per-wheel velocity PI and a
  gyro-based differential yaw-rate loop (`src/ros/ros_custom/cpp_main.cpp`). ROS2 sends `cmd_vel`
  straight through with no host-side shaping.
- Blade control plus the always-on `ANTIDIG_*` cutout (the blocked-wheel backstop)
- IMU reading (accelerometer, gyroscope, magnetometer)
- Battery voltage, the CC/CV charge envelope and charging state
- Rain sensor, stop buttons, wheel-lift and tilt emergency sensors
- USB-CDC link to ROS2's `hardware_bridge_node` — COBS-framed and CRC-16 checked
  (`include/mowgli_protocol.h`, protocol version 6)

### Building

Requires [PlatformIO](https://platformio.org/). The board variant is selected by the PlatformIO
*env*, not by editing `board.h` — the env's `build_flags` set `BOARD_YARDFORCE500_VARIANT_ORIG`
or `BOARD_YARDFORCE500_VARIANT_B`.

```bash
cd stm32/ros_usbnode
pio run                   # default_envs = Yardforce500 (STM32F103VC)
pio run -e Yardforce500B  # STM32F401VC
```

Safety defaults (charge envelope, e-stop timeouts, tilt threshold) are single-sourced in
`include/board_defaults.h`; `board.h` and its GUI-rendered twin `board.h.template` may only
`#include` it — re-`#define`ing one of those macros fails the `defaults-parity` CI job.

### Flashing

**Back up your stock firmware first** — see [`stm32/mainboard_firmware/`](stm32/mainboard_firmware)
and [`stm32/panel_firmware/`](stm32/panel_firmware).

The GUI setup page is the recommended path. Its default ("prebuilt") downloads the binary matching
your board from the latest GitHub release manifest, verifies its sha256, flashes it and then
re-reads the protocol version from the board — no toolchain, no compile. The "custom" option is the
expert path: it clones a branch, renders `board.h.template` and runs `platformio run -t upload`.

With an ST-Link directly:

```bash
pio run -e Yardforce500 -t upload            # ST-Link v2
pio run -e Yardforce500_STLINK_V3 -t upload  # OpenOCD (also Yardforce500B_STLINK_V3)
```

### Guards & codegen

Pure Python, run from the repo root — no toolchain needed:

```bash
python3 firmware/scripts/board_defaults_parity.py           # CI: firmware-ci / defaults-parity
python3 firmware/scripts/protocol_version_guard.py --check  # CI: protocol-version-drift
python3 firmware/scripts/sync_ros_lib.py --check            # CI: msg-codegen-drift
```

`include/mowgli_protocol.h` is the single source of truth for the wire format. Changing a `pkt_*_t`
means bumping `MOWGLI_PROTOCOL_VERSION`, hand-mirroring the struct into
`ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp`, and re-running
`protocol_version_guard.py` (without `--check`) to refresh `scripts/protocol_baseline.json` —
otherwise every already-flashed board reports "incompatible, reflash".

`sync_ros_lib.py` regenerates the rosserial-style `ros_lib/mower_msgs/*.h` from
`mowgli_interfaces/*.msg`. That output is **excluded from the build** (`platformio.ini`
`build_src_filter`) and included by no source file — it exists only to satisfy the CI drift gate;
the live wire is `pkt_*_t` only.

There are no firmware-side unit tests. Behaviour is pinned by the host mirror
`ros2/src/mowgli_hardware/test/test_protocol.cpp`, the three guards above, and
`gui/pkg/providers/firmware_test.go`.

## Supported Hardware

- YardForce Classic 500 — env `Yardforce500` (STM32F103VCT6), the default
- YardForce Classic 500B — env `Yardforce500B` (STM32F401VC)

`BOARD_LUV1000RI` exists in `board.h` / `board.h.template` and the GUI offers the board, but
`platformio.ini` has **no `LUV1000RI` env** — the target MCU/clock and the LUV blade-motor UART
wiring are not in the repo. Do not add a guessed env; a wrong pinout can brick the board.

## Working on this tree

[`CLAUDE.md`](CLAUDE.md) beside this file is the entry point for code work here (what this tree owns,
conventions, per-component gotchas). The detailed reference is
[`../docs/claude/codemaps/firmware.md`](../docs/claude/codemaps/firmware.md) — file map, wire-packet
table, main-loop order — alongside
[`../docs/claude/ros-interfaces.md`](../docs/claude/ros-interfaces.md),
[`../docs/claude/parameters.md`](../docs/claude/parameters.md) and
[`../docs/claude/testing-ci.md`](../docs/claude/testing-ci.md).
