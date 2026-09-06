# Blade direction commands and reversal guard

This change carries an explicit host direction request through the existing blade
driver. It does not alternate direction automatically. Current MowgliNext ROS2
`SetMowerEnabled` always sends direction zero; the existing MowerControl service
and USB blade packet can carry a different direction without a protocol change.

## Attribution and controller evidence

Adapted from Jeremy Salwen's original Mowgli firmware fix:
https://github.com/jeremysalwen/Mowgli/commit/dd6c01b64ac92e3c5f5edbea1112903c5ee82d35

Latch the direction in `BLADEMOTOR_Set` and use it in `blademotor_prepareMsg`,
which formerly overwrote the direction before every UART transmission. Generate
the additive checksum using the existing `crcCalc` function:

| Requested state | UART frame |
| --- | --- |
| Off | `55 AA 03 20 80 00 A2` |
| On, direction 0 | `55 AA 03 20 80 80 22` |
| On, direction nonzero | `55 AA 03 20 80 C0 62` |

The old commented reverse checksum `E2` was wrong. Jeremy's bench-test commit
reports that a 500B accepted `C0/62`, reached about 3300 RPM and reported no
protocol errors. It explicitly leaves physical rotation direction unverified:
https://github.com/jeremysalwen/Mowgli/commit/d78e2cc34cc1b03b03ada8bb7c1594c4bd0c7039

That bench auto-start/drive-reset code is not included here.

## Firmware guard

- A change from the last transmitted running direction first sends OFF.
- Wait at least 1000 ms after UART accepts that OFF transmission, and require
  fresh, checksum-valid responses showing inactive, zero RPM and no error over
  at least 300 ms. Feedback older than 300 ms cannot qualify.
- Startup/cached zero RPM, a single old zero, malformed replies, active/nonzero
  replies and feedback gaps cannot release reversal. RX tracks interruptions
  even if a later good reply replaces them before the foreground checks.
- If communication never confirms stopping, continue sending OFF indefinitely.
  There is no timeout that forces the opposite running direction.
- An OFF request cancels pending reversal. An opposite-direction re-enable must
  establish a new stop interval. Repeated ON requests do not reset the interval.
- Changing the requested direction back during a pending reversal still waits
  for the stopped confirmation; it then uses the latest requested direction.
- Do not mutate the UART DMA request buffer from the setter or while TX is busy.
  A failed/busy OFF transmission does not start the guard's dwell timer.
- Existing emergency/heartbeat/idle blade gating and error stop remain in place.
  Normal direction-zero starts are unchanged. This change contains no charging,
  battery-chemistry, ADC, temperature, wheel-control or protocol changes.

The first requested reverse start also requires the stop confirmation. These
guards reduce reliance on assumptions about spin-down; they cannot prove that
the controller's feedback or direction opcode matches physical shaft motion.

## Software validation

`python firmware/scripts/test_blade_reverse.py` (MSVC: add `--cc cl`) compiles
and runs the actual setter, frame builder, application loop and RX callback with
UART/clock stubs. Only hardware initialization is removed. The checksum function
is extracted from production `main.c`. The tests compile with the production
`board.h` and `board_defaults.h` for both supported board selections, including
guard boundaries, invalid/stale feedback, overwritten bad
replies, cancellation, TX failure/busy, IRQ-mask preservation and tick wrap.

| Build | MCU | Blade UART | RX / TX DMA | Feedback length |
| --- | --- | --- | --- | --- |
| `Yardforce500` | STM32F103VC | USART3, PB10/PB11 | DMA1 channels 3 / 2 | 16 bytes |
| `Yardforce500B` | STM32F401VC | USART6, PC6/PC7 | DMA2 streams 1 / 6, channel 5 | 16 bytes |

The UART initialization and board mappings are unchanged. Both use the same
command builder and RX callback, reached through `HAL_UART_RxCpltCallback`.
The 14-byte response definition belongs to the unsupported LUV board, not 500B.
Build both standard environments with `pio run -e Yardforce500 -e Yardforce500B`.

## Firmware version

`git_build_id.py` generates a new firmware version from the new commit's count;
there is no manually maintained firmware patch number to bump. The build job
uses full Git history so its version matches local builds and release packaging.
Protocol version remains 6: the existing direction byte and packet layout are
unchanged. The eventual upstream merge gets its own build identity, so match
artifacts by commit as well as the displayed version.

## HARDWARE_REQUIRED before claiming reverse support

On the exact mower/controller being evaluated, in a supervised blades-removed
test, separately request each direction with a stop between starts. Confirm the
shaft/disc actually rotates in opposite directions; unsigned RPM or an accepted
UART response is insufficient evidence. Check that missing feedback leaves it
off and that the normal emergency/stop path cancels pending reversal. No deployed
mower has been tested or flashed as part of this build-only change.

The separate [ROS2/GUI PR #558](https://github.com/mowglinext/mowglinext/pull/558)
adds opt-in, random direction selection per mowing session. This firmware change
also supports existing explicit direction requests without that PR. Do not port
the upstream bench build's automatic blade-start override into normal operation.
