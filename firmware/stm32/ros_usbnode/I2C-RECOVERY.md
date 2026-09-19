# External IMU software-I2C recovery

Applies to `src/soft_i2c.c` on the supported 500 and 500B targets. This brings
the existing LFP external-IMU recovery into the upstream firmware.

## Failure and recovery

On .118, the MPU6050 stopped producing samples during runtime. The driver
callbacks remained installed, MCU tick advanced, and PB3/SCL was high while
PB4/SDA remained low despite both outputs being released. The previous driver
abandoned every read on a busy bus and never attempted to clear it.

The IMU register read, multi-read, and register write entry points now release
the wires and check idle before START. A busy bus triggers at most nine SCL
pulses with SDA released, then STOP once SDA releases. SCL must actually rise;
each wait has a fixed 16-poll limit using the existing bus timing primitive.
If either wire stays low, the transaction fails with both outputs released.
Recovery attempts are limited to once per second with wrap-safe tick arithmetic;
a naturally freed bus is immediately usable even during that cooldown.

The original transaction proceeds only after successful bus clear and must
still receive its normal ACKs. Failed reads remain invalid; there is no stale
sample publication or fabricated IMU value. Normal IMU and yaw polling share
this main-loop driver, so no concurrent debugger or interrupt recovery is used.
No changes to motion control, charging logic, protocol, or emergency handling.

This deliberately does not call the blocking 20-second `IMU_Init()` during
runtime, reset sensor configuration, or re-probe a missing sensor. It recovers
an interrupted transaction on an already configured sensor; a sensor that has
lost configuration or is electrically stuck may still need a power cycle.
RAM counters `sw_i2c_recovery_diag` record attempts/successes/failures, but are
not published over the wire. Successful startup after flashing alone does not
prove that an in-service stuck-bus event has been recovered.

Reference: NXP UM10204 section 3.1.16, Bus clear:
https://www.nxp.com/docs/en/user-guide/UM10204.pdf

## Validation and flashing

Run `python firmware/scripts/test_soft_i2c_recovery.py` with a native C compiler
(`--cc cl` inside a Visual Studio developer shell on Windows). It executes the
production driver against GPIO faults, including release on each clock 1–9,
permanent SDA/SCL faults, stretching, SCL failure midway and during STOP,
cooldown/tick wrap, invalid reads, and ACK/NACK handling after recovery.

Build the upstream `Yardforce500` and `Yardforce500B` targets. Existing LFP
observations belong to their recorded custom builds, not these new binaries.
Do not deliberately inject an electrical bus fault into a running mower.
Unit fault injection establishes logic, not physical timing or field reliability.

## Onboard LIS3DH recovery (I2C1, PB6/PB7)

`src/i2c.c` now owns the onboard sensor in the foreground. It replaces the
old unchecked HAL results and the I2C reads made by emergency-release USB
callbacks. Callbacks use a validity/age-qualified snapshot instead; only a
successful INT1_SRC read publishes a new snapshot. Read failures invalidate it.
A 100 ms maximum snapshot age detects stalled polling; it does not turn a
cached value into a new observation.

Startup and recovery verify WHO_AM_I, write/read back the safety configuration,
and require a successful interrupt-source read before becoming healthy. The
configuration retains 100 Hz XYZ, +/-2g, HR/BDU, temperature ADC, configured
tilt threshold, duration 1, active-high pulsed Z-low INT1, and bypass FIFO.
Configuration/identity audits interleave with polling roughly once per second
and detect a sensor reset even if it keeps returning a zero interrupt source.

When the bus is stuck, a main-loop state machine disables/resets I2C1, releases
PB6/PB7 as open drain, clocks up to nine pulses and emits STOP. Each edge waits
at least 1 ms without a busy wait. SCL must actually rise. It then restores
hardware-I2C pin ownership and verifies sensor configuration again. Failed
attempts leave both pins released and wait 1 second before retrying. No MCU
reset, trace output, GPIO push-pull high, or fault-bypass command is required.

Each service call performs at most one HAL transaction (requested timeout 2 ms),
one GPIO edge, or peripheral initialization. Busy/held lines are checked before
HAL to avoid its normal 25 ms BUSY wait. This is not a hard 2 ms wall-time
claim: the vendor HAL has its own BUSY handling, interrupt latency adds time,
and actual watchdog/control timing requires measurements on the target.

An unavailable sensor immediately inhibits motion through Emergency_State.
A detected I/O/configuration/freshness fault additionally latches a distinct
internal emergency bit (0x40), logs a foreground warning and increments RAM
counters. It cannot be auto-cleared as a pure heartbeat-loss latch. Recovery
never clears it; an explicit release is accepted only after fresh healthy,
untriggered sensor status. The physical play-button path uses the same release
gate. Wire protocol remains v6; the existing emergency indication is used.

`onboard_i2c_diag` retains fault/attempt/recovery/sample counts in RAM. These
are diagnostic counters, not USB fields. Successful recovery means verified
communication/configuration, not independent proof that a sensor measures
physical tilt correctly.

### Validation and remaining hardware work

Run `python3 firmware/scripts/test_onboard_i2c_recovery.py` with GCC/G++.
It compiles production I2C code for both board selections, the production
emergency latch functions, and the real heartbeat callback. It covers release
on pulses 1-9, permanently held lines, retry limits/wrap, BUSY/NACK/timeouts,
wrong identity, corrupted writes, sensor resets, stale snapshots, IRQ exclusion,
and fault retention across communication recovery and explicit release.
Both recovery harnesses are included in Firmware CI.

HARDWARE_PENDING for the new onboard runtime state machine. Software fault tests
and stock firmware builds pass; the manually executed clock/STOP recovery on
.118 (LFP diagnostics 0df13383, 1.11.13) validates only that earlier procedure,
not this automatic firmware implementation. External-IMU LFP evidence likewise
does not validate a newly built upstream image.

Record the candidate commit, environment, ELF/binary SHA256, board identity and
sensor revision. With blades removed, wheels raised, clear rotor and accessible
cutoff, flash the matching board/profile and keep it IDLE. Observe startup and
normal-use recovery through USB plus diagnostic counters: sensor status must
become valid only after verification, IMU/blade telemetry must continue, no
watchdog reset may occur, and wheels/blade must remain inhibited while unhealthy.
After an observed recoverable bus fault, require an increased recovery count,
fresh samples and retained emergency until explicit release. A permanent fault
must remain inhibited. Do not short live bus pins to manufacture a fault.
Healthy startup alone cannot complete the recovery acceptance test.