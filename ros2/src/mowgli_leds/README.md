# mowgli_leds — WS2812 status ring

An addressable WS2812/WS2812B ("NeoPixel") ring driven from the Orange Pi 5B,
showing robot state at a glance: emergency, charging, mowing progress, RTK
health, recording, idle.

> **NOT hardware-verified.** At the time of writing the SPI device does not
> exist on the robot yet (`ls /dev/spidev*` returns nothing) because the
> device-tree overlay has not been enabled. Everything below the SPI write is
> unit-tested; the wire timing, colours and physical ring have **never been
> observed working**. Treat the first bring-up as a bring-up.

- **Read-only.** Subscribes to status topics, writes to `/dev/spidev`. It
  commands nothing, publishes nothing, and shares no state with the motion,
  dig-detection, coverage or localization stacks.
- **Not a safety device.** The STM32 firmware is the sole blade-safety
  authority and keeps its own status LED. Nothing here is in that path.
- **Off by default** (`led_enabled: false`). An existing robot is unaffected
  until the operator opts in.

---

## 1. Wiring

| | |
|---|---|
| Board | Orange Pi 5B (RK3588S), Ubuntu 24.04, kernel 6.1.0-1025-rockchip |
| Ring DATA | 40-pin header **pin 19** = `GPIO1_C1` |
| Function used | **`SPI4_MOSI_M0`** |
| Ring VCC | 5 V |
| Ring GND | any header ground — must be common with the Pi's ground |

Pin 19 is muxed three ways on this board: `I2C3_SCL_M0`, `UART3_TX_M0` and
`SPI4_MOSI_M0`. We drive it as **SPI MOSI, not UART**, on the same physical
wire with no rewiring:

- A UART TX line **idles HIGH**. WS2812 needs the line to idle **LOW** — a high
  idle is decoded as a stream of garbage bits and the inter-frame reset gap
  never happens. The UART route therefore needs an external inverter.
- SPI MOSI in mode 0 **idles LOW** and matches the protocol natively.

### The 3.3 V data caveat — read this first if colours are wrong

The ring is powered at **5 V** but the Pi's data line is **3.3 V**. WS2812B
specifies `VIH = 0.7 x VDD` = **3.5 V**, so 3.3 V is *marginally below spec*.
It very often works anyway (especially on the first LED of a short run, at
room temperature), and it very often does not.

**If the software is correct and the colours are still wrong — wrong hue,
flicker, only the first LED lights, colours "walk" down the ring — suspect
this before you suspect the code.** It is a hardware level problem and is not
fixable in software. The usual remedies are a 74AHCT125 / 74HCT245 level
shifter on the data line, or dropping the ring's supply to ~4.3 V with a
series diode so `0.7 x VDD` falls below 3.3 V.

Also standard WS2812 practice, unrelated to the above: a 300-500 ohm series
resistor in the data line at the ring end, and a 100-1000 uF capacitor across
the ring's 5 V and GND.

---

## 2. Enable the SPI4-M0 overlay (operator step, required)

`/dev/spidev*` does not exist until the device-tree overlay is enabled. **This
board boots through U-Boot + extlinux.** The `dtoverlay=` lines in
`/boot/config.txt` are Raspberry-Pi syntax, are **never read** on this board,
and must not be used.

The overlay ships with the kernel package:

```bash
ls /usr/lib/firmware/$(uname -r)/device-tree/rockchip/overlay/ | grep spi4
# rk3588-spi4-m0-cs1-spidev.dtbo    <-- the one we want
```

Enable it:

```bash
sudo cp /etc/default/u-boot /etc/default/u-boot.bak

# Uncomment/set these two lines in /etc/default/u-boot.
# If U_BOOT_FDT_OVERLAYS already has entries, APPEND to it (space-separated)
# rather than replacing them.
sudo sed -i \
  -e 's|^#\?U_BOOT_FDT_OVERLAYS=.*|U_BOOT_FDT_OVERLAYS="rockchip/overlay/rk3588-spi4-m0-cs1-spidev.dtbo"|' \
  -e 's|^#\?U_BOOT_FDT_OVERLAYS_DIR=.*|U_BOOT_FDT_OVERLAYS_DIR="/usr/lib/firmware/'"$(uname -r)"'/device-tree/"|' \
  /etc/default/u-boot

grep U_BOOT_FDT /etc/default/u-boot     # confirm before regenerating

sudo u-boot-update                       # regenerates /boot/extlinux/extlinux.conf
grep -i fdtoverlays /boot/extlinux/extlinux.conf   # the overlay must appear here

sudo reboot
```

After the reboot:

```bash
ls -l /dev/spidev*        # expect e.g. /dev/spidev4.1
```

The exact `spidevB.C` numbering is assigned by the kernel and is **not**
guaranteed to be `4.1` — take whatever `ls` reports and put it in
`led_spi_device`. If nothing appears, `dmesg | grep -i spi` and
`grep -r spi /proc/device-tree/ 2>/dev/null | head` are the next stops; do not
change the ROS config until the device node exists.

### Permissions

The node opens the device as whatever user the ROS container runs as. Either
add that user to the `spi`/`dialout` group that owns the node, or bind-mount
the device into the container (`--device=/dev/spidev4.1`). A permissions
failure is handled exactly like a missing device: one WARN, then stand down.

---

## 3. Enable the node

**From the GUI (preferred):** *Settings → Status LEDs*. Flip **Enable Status
Ring**, set **LED count** to what your ring actually has, and correct the **SPI
device** if `ls /dev/spidev*` reported something other than the default. Save,
then restart the ROS2 stack. The section also carries the prerequisites above
and a legend of every ring pattern.

Every default lives in the **in-package template**
`ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (CLAUDE.md Invariant 15) and
is mirrored in `gui/asserts/mower_config.schema.json`, so each field gets the
GUI's "overridden" dot and its reset-to-default button, and only genuine
overrides are written to the installed sparse config.

`led_enabled` defaults to **false** in BOTH the template and the schema, and
must stay that way. The settings backend prunes any saved value equal to its
schema default; with a default of `true`, "switch the ring ON" would write the
default value, be pruned as redundant, never reach the installed file, and the
toggle would be permanently inert. That is the `lidar_enabled` bug fixed in
#508, and `gui/pkg/api/settings_leds_test.go` pins it here.

**By hand**, if you prefer, set only what differs from the default in
`/ros2_ws/config/mowgli_robot.yaml`:

```yaml
mowgli:
  ros__parameters:
    led_enabled: true
    led_count: 24              # whatever your ring actually has
    # led_spi_device: "/dev/spidev4.0"   # only if it is not the default
```

Either way, restart the stack afterwards. `full_system.launch.py` reads
`led_enabled` at launch and only starts `led_ring_node` when it is true (same
pattern as `lidar_enabled`); a CLI/compose `led_enabled:=false` still overrides
it.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `led_enabled` | `false` | Master switch. Gates the launch **and** re-checked inside the node, so `ros2 run` also stands down. |
| `led_count` | `16` | LEDs on the ring. **Set this** — 16 is a placeholder, not a measurement. |
| `led_spi_device` | `/dev/spidev4.1` | Confirm with `ls /dev/spidev*`. |
| `led_brightness` | `0.6` | Linear 0..1 on every channel. Full-white on 16 LEDs at 1.0 is ~1 A; 0.6 keeps it under ~0.6 A and is still daylight-readable. |
| `led_refresh_hz` | `20.0` | Render tick, clamped to [1, 60]. |
| `led_keepalive_s` | `2.0` | Re-send an unchanged frame at least this often. |
| `led_status_timeout_s` | `5.0` | No `HighLevelStatus` for this long selects the STALE pattern. |
| `led_device_retry_s` | `30.0` | Re-open a missing/failed device this often, silently. |
| `led_low_battery_percent` | `20.0` | Below this, and not charging, the ring blinks red. |
| `led_charge_full_percent` | `99.0` | At or above this a charging ring goes steady. |
| `led_idle_scale` | `0.10` | Idle-ring brightness, relative to `led_brightness`. |
| `led_spi_speed_hz` | `2400000` | Do **not** retune without re-reading `ws2812_encoder.hpp` — the symbol table is derived from this exact clock. |

---

## 4. Display semantics

Priority order, first match wins — an alarm always beats an activity:

| # | State | Pattern |
|---|---|---|
| 1 | **Emergency** | Whole ring **solid red**, static. The only solid red and the only static full ring. |
| 2 | **Charging** | **Green arc** proportional to battery, breathing over ~3 s. Goes **steady full green** at `led_charge_full_percent`. If the level is unknown (behavior tree down, charge state from the hardware bridge alone) a **green comet** instead of an arc — it will not invent a level. |
| 3 | **Stale** | **Amber comet** on a dark ring, ~1.5 s per revolution. "Powered and running, but the behavior tree is not talking to me." |
| 4 | **Low battery** | Whole ring **blinking red** at 1 Hz. Same hue as emergency because both mean "attend to me"; **motion** is the discriminator (blinking vs solid). |
| 5 | **Mowing** | **Green arc** proportional to `coverage_percent`, with a **white head pixel** at the tip so the boundary is crisp at distance. Steady. |
| 5b | **Mowing, RTK not fixed** | Same arc in **amber**, head pixel **blinking at 2 Hz**. Cutting accuracy is degraded without RTK-Fixed. Both hue *and* motion change, so it survives bright sun where hue alone is unreliable. |
| 6 | **Recording** | **Cyan comet**, ~2 s per revolution. |
| 7 | **Manual mowing** | Whole ring **breathing purple**, ~2 s. |
| 8 | **Idle** | Whole ring **dim white**, static. |

Design rules behind those choices, for anyone changing them:

- This is an outdoor machine read from several metres in daylight, so every
  mode is distinguishable by **colour plus motion**, never by counting pixels
  or by a single indicator LED. Blue is deliberately unused as a state colour
  (weakest of the saturated colours outdoors).
- The arc never lies at its ends: any non-zero progress lights **at least one**
  pixel, and anything short of 100 % leaves **at least one** pixel dark, so a
  full ring means done and nothing else.
- Emergency and `state` are only honoured while the status is **fresh**. A
  latched flag from a dead behavior tree must not hold the ring red forever.
- Charging outranks Stale on purpose: the charge bit comes from the hardware
  bridge's `Power` message, which is alive even when the behavior tree is not,
  and "robot is on the dock charging" is the most common at-a-glance question.

### Inputs

| Topic | Type | Used for |
|---|---|---|
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/HighLevelStatus` | `state`, `coverage_percent`, `battery_percent`, `is_charging`, `emergency` |
| `/gps/status` | `mowgli_interfaces/GnssStatus` | RTK-Fixed, via the shared `gnss_status_utils::BehaviorTreeRtkFixed` helper so the ring can never disagree with the BT or the hardware bridge |
| `/hardware_bridge/power` | `mowgli_interfaces/Power` | `charger_enabled` only — the charging fallback when the behavior tree is silent |

All three at `rclcpp::QoS(10)` reliable, matching their publishers.

---

## 5. Encoding: 3 SPI bits per WS2812 bit at 2.4 MHz

One SPI bit at 2.4 MHz is 416.67 ns, so three of them are **1.25 us — exactly
the WS2812B bit period**:

| Symbol | SPI bits | High | Low | Datasheet (+/-150 ns) |
|---|---|---|---|---|
| `0` | `100` | 417 ns | 833 ns | T0H 400, T0L 850 |
| `1` | `110` | 833 ns | 417 ns | T1H 800, T1L 450 |

All four figures land within ~35 ns of nominal — dead centre, leaving the full
+/-150 ns of margin for SPI clock error.

**Why not 4 bits at 3.2 MHz** (the other common scheme): the natural symbols
`1000`/`1100` put T1H at 625 ns, *below* the 650 ns lower bound; you must use
`1110` instead, which pushes T1L to 312 ns near *its* 300 ns lower bound. It
also costs 12 bytes per pixel instead of 9 — 33 % more SPI traffic for worse
timing margin.

The 3-bit scheme is byte-aligned by construction: 8 WS2812 bits expand to 24
SPI bits = exactly 3 bytes, so a colour byte never straddles an output byte
boundary. A pixel is 9 bytes; a 16-LED frame is 144 bytes plus the reset gap.

**Reset gap:** 90 trailing zero bytes = **300 us**. The original WS2812B
datasheet asks for RES > 50 us, but the widely-shipped **WS2812B-V5** revision
needs > 280 us and merges frames below that. 90 bytes covers both and is free
at this data rate.

**Colour order is GRB, not RGB.** The swap happens once, in `Encode()`, where
`Ws2812Encoder.EncodesPixelsInGrbWireOrderNotRgb` pins it.

---

## 6. Cost

Rendering is 16 structs of 3 bytes per tick. A frame is only **written** when
it differs from the last one written, plus a keepalive every
`led_keepalive_s`:

- Static modes (idle, emergency, charge-complete): ~0.5 writes/s of 234 bytes.
- Animated modes: 20 writes/s of 234 bytes = ~4.7 kB/s, ~0.8 ms of SPI
  transfer time each.

No busy loops, no polling of the device, one wall timer.

---

## 7. Failure behaviour

Missing `/dev/spidev`, a permissions error, or a device that disappears mid-run
all behave identically:

1. One `WARN` naming the syscall and errno. **Once per outage**, not per tick.
2. Stand down. The node keeps running, keeps subscribing, renders nothing.
3. Retry the open every `led_device_retry_s` (30 s), **silently**.
4. On recovery, one `INFO` and a forced full frame (the strip's contents are
   unknown after an outage).

A write error also closes the descriptor so step 3 re-opens it. The node never
crashes, never spins, and never spams. On shutdown it writes an all-off frame
so the ring does not stay lit after the stack stops.

---

## 8. Layout and tests

| File | Role |
|---|---|
| `include/mowgli_leds/led_pattern.hpp` | **Pure**: status -> pixel buffer. No ROS, no hardware, no clock. |
| `include/mowgli_leds/ws2812_encoder.hpp` | **Pure**: pixel buffer -> SPI byte stream. |
| `include/mowgli_leds/spi_device.hpp`, `src/spi_device.cpp` | The only hardware-touching code, and the only untested part. |
| `include/mowgli_leds/led_ring_node.hpp`, `src/led_ring_node.cpp` | ROS glue: subscriptions, staleness, change detection, retry policy. |
| `test/test_led_pattern.cpp` | 32 tests: mode priority, arc arithmetic, animation helpers, rendered frames. |
| `test/test_ws2812_encoder.cpp` | 13 tests: symbol expansion, GRB order, frame size, reset gap, brightness. |

Same shape as `mowgli_hardware/dig_detector.hpp` and
`mowgli_nav2_plugins/ftc_start_index.hpp`: all the logic lives in a header that
can be tested without a robot. Keep display logic out of `spi_device.cpp`.

A file-level map of this package — including every mirror a change has to touch
(launch, template, schema, GUI section, i18n) — is
[`docs/claude/codemaps/mowgli_leds.md`](../../../docs/claude/codemaps/mowgli_leds.md).

```bash
colcon build --packages-select mowgli_leds
colcon test  --packages-select mowgli_leds && colcon test-result --verbose
```

The pattern header mirrors `HighLevelStatus`'s `HIGH_LEVEL_STATE_*` constants
so it can stay ROS-free; `led_ring_node.cpp` `static_assert`s the two against
each other, so a renumbering on the message side breaks the **build**, not the
ring.

### GUI side

| File | Role |
|---|---|
| `gui/asserts/mower_config.schema.json` | `led_settings` group. Every default here MUST equal the template's. |
| `gui/web/src/components/settings/LedsSection.tsx` | The *Status LEDs* settings section, including the pattern legend. Its swatches mirror this package's palette — keep them in sync. |
| `gui/pkg/api/settings_leds_test.go` | Pins `led_enabled`'s default to `false` and exercises the sparse-prune coupling, so the ON toggle can never become inert (bug #508). Also pins the SPI clock to the value this encoder is derived from. |
| `gui/web/src/components/settings/LedsSection.test.tsx` | 14 tests: off-by-default, the ON/OFF writes, the prerequisite hints, the field set, the legend, and reset-to-default. |

```bash
cd gui && go test ./pkg/api/...
cd gui/web && yarn test
```
