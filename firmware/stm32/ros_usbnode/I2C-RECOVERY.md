# External IMU software-I2C recovery

Maintenance patch for both LFP branches and the charging diagnostic build.
Applies to `src/soft_i2c.c`, independent of interrupt/DMA ADC acquisition.
Retain it when rebasing the LFP work onto upstream.

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

Build `Yardforce500B_LFP` on both branches; .118 uses the non-DMA
`Yardforce500B_LFP_DIAG` build. Retain the LFP charging, temperature GPIO and
blade reversal patches. Use `remote_upload/yardforce500b_no_trace.cfg` with
`itm ports off` before OpenOCD init, verify the flash, then disconnect SWD.
Keep the recurring charging recorder paused: repeated debugger attachment is
a possible trigger under investigation, not an established cause.

Verify finite `/imu/data` samples and idle/charging hardware status through ROS
after flashing and again after a settling period. Do not deliberately inject
an electrical bus fault into a running mower. Unit fault injection establishes
the recovery logic; further normal-use observation establishes field reliability.
