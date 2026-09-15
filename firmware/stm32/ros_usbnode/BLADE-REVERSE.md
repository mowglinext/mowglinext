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
- After five seconds pending, increment the existing blade error counter and
  print `Blade reversal waiting: OFF retained (...)` at most every five seconds.
  This is a diagnostic, never permission to start. OFF/error cancellation stops
  reporting; repeated ON does not reset the reporting interval.
- An OFF request cancels pending reversal. An opposite-direction re-enable must
  establish a new stop interval. Repeated ON requests do not reset the interval.
- Changing the requested direction back during a pending reversal still waits
  for the stopped confirmation; it then uses the latest requested direction.
- Do not mutate the UART DMA request buffer from the setter or while TX is busy.
  A failed/busy OFF transmission does not start the guard's dwell timer.
  Receive DMA re-arming and ESC error handling still run while TX is busy.
- Existing emergency/heartbeat/idle blade gating and error stop remain in place.
  Normal direction-zero starts are unchanged. This change contains no charging,
  battery-chemistry, ADC, temperature, wheel-control or protocol changes.

The first requested reverse start also requires the stop confirmation.
**Merge blocker:** it is not yet established that ESC RPM continues to measure
rotor motion after OFF. If the ESC immediately reports inactive/zero while the
rotor coasts, the guard can release after only 1000 ms. That delay is not a
validated mechanical stopping time. Successful direction tests do not answer
this question; do not claim the guard is physically validated until the
independent coast-down measurement below is complete on each controller type.

## Software validation

`python firmware/scripts/test_blade_reverse.py` (MSVC: add `--cc cl`) compiles
and runs the actual setter, frame builder, application loop and RX callback with
UART/clock stubs. Only hardware initialization is removed. The checksum function
is extracted from production `main.c`. The tests compile with the production
`board.h` and `board_defaults.h` for both supported board selections, including
guard boundaries, invalid/stale feedback, overwritten bad
replies, cancellation, TX failure/busy, RX re-arm/error handling while TX is busy,
throttled pending diagnostics, IRQ-mask preservation and tick wrap. The bench
configuration is also tested for no auto-start, no reversal despite qualifying
feedback, and timestamped completed-sample tracing.

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
there is no manually maintained firmware patch number to bump. Local builds need
full Git history for that count. The existing shallow CI build checkout is left
unchanged by this PR; release packaging already fetches full history.
Protocol version remains 6: the existing direction byte and packet layout are
unchanged. The eventual upstream merge gets its own build identity, so match
artifacts by commit as well as the displayed version.

## HARDWARE_REQUIRED before claiming reverse support

On the exact mower/controller being evaluated, in a supervised blades-removed
test, separately request each direction with a stop between starts. Confirm the
shaft/disc actually rotates in opposite directions; unsigned RPM or an accepted
UART response is insufficient evidence. Check that missing feedback leaves it
off and that the normal emergency/stop path cancels pending reversal. Earlier
operator-reported opposite-direction operation is recorded with its exact 500
and combined 500B baselines in [PR #559](https://github.com/mowglinext/mowglinext/pull/559).
It does not validate coast-down feedback, and it does not extend to this revision.

## Coast-down validation image and procedure

`HARDWARE_PENDING` on the standard 500 bench unit: software instrumentation and
procedure are supplied; the physical run and controller identity are outstanding.
The equivalent 500B coast-down evidence remains `HARDWARE_REQUIRED`; the UART
bench environment below intentionally supports the original 500 only.

Build `pio run -e Yardforce500_COASTDOWN_VALIDATION`. This uses the standard
non-LFP Yardforce500 configuration plus `BLADEMOTOR_COASTDOWN_VALIDATION=1`.
Forward and OFF still require the normal host/firmware gates. It never auto-starts,
never releases a pending reversal, and adds no safety-input override. It is a
bench artifact, not a release firmware. Record its ELF/bin SHA256, source SHA,
build environment, displayed version, host image digests, submodule gitlinks,
mower/ESC board and controller revisions before running. The version alone does
not distinguish this environment from a standard image built at the same commit.

1. With power isolated, remove the cutting blades, secure the mower and exposed
   rotating assembly, restore the intended STOP/lift/panel wiring, and arrange
   physical emergency-stop access. Do not reuse the earlier panel jumpers.
   Keep people clear throughout; do not touch the rotor to judge stopping.
2. Capture the standard 500 UART debug output at 115200 baud using the existing
   [serial-debug connection](README.md#serial-debugging). Use an independent
   optical tachometer with a synchronized time record to observe rotor motion.
   Verify a continuous `blade coast` trace before commanding the motor.
3. Through the normal supervised host controls, run forward to steady speed,
   then explicitly request OFF. Do not request reverse. Capture from before OFF
   until at least five seconds after the independently observed full stop.
   Repeat several times on this same recorded baseline.
4. Compare `tx=00` / `tx_t` (latest accepted UART command transition, **not** ESC
   acknowledgment) with `rx_t`, `seq`, `valid`, `active`, `rpm` and `err` from
   completed ESC replies. `t` is the foreground log time. Invalid samples are
   marked `valid=0`; their other fields are cached and must be ignored. Repeated
   foreground reads do not create samples. Sequence gaps expose overwritten
   samples or best-effort debug loss; a gap around OFF/zero makes the run
   inconclusive and requires another capture, not interpolation.
5. **Pass for the feedback assumption:** valid RPM follows independently observed
   motion during coast-down and does not report a qualifying zero interval while
   rotation continues. The activated bit may describe enable state; assess it
   separately rather than assuming it measures motion. **Fail:** zero/inactive
   reports persist while the tachometer still shows rotation. Missing/ambiguous
   evidence is inconclusive. Retain the raw trace and synchronized tachometer
   record for review before allowing the standard reversing firmware.

If feedback fails this check, leave approval blocked. A replacement stopping
criterion or a measured worst-case coast time plus justified margin is needed.
A blades-removed measurement cannot establish worst-case stopping time with
cutting blades attached; do not choose a longer production timeout from this
unloaded run. No stopping delay has been guessed or increased in this revision.

The full OFF/emergency/missing-feedback guard checks remain required after this
measurement; the trace alone does not certify all blade safety behavior.

The separate [ROS2/GUI PR #558](https://github.com/mowglinext/mowglinext/pull/558)
adds opt-in, random direction selection per mowing session. This firmware change
also supports existing explicit direction requests without that PR. Do not port
the upstream bench build's automatic blade-start override into normal operation.
