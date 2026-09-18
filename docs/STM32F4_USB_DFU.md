# STM32F401 USB DFU updates

> **WIP / EXPERIMENTAL — NOT HARDWARE VALIDATED**
>
> This path must remain draft-only until the complete HIL procedure below has
> passed on the intended STM32F401 mower hardware. Software tests do not prove
> motor-controller receipt, blade/drive inhibit levels during reset, USB clock
> behaviour, or recovery from real power interruption.

## Scope

The initial USB update capability is deliberately narrow:

- supported application target: `BOARD_YARDFORCE500B` / PlatformIO
  `Yardforce500B`, whose board manifest selects STM32F401VCT6;
- transport: the STM32F401xC factory system-memory USB DFU bootloader;
- artifacts: blessed prebuilt release binaries whose manifest metadata,
  board/panel identity, size, vector table, protocol/version and SHA-256 all
  validate before DFU entry;
- STM32F103 boards remain ST-Link-only;
- ST-Link remains available for explicit advanced flashing and final recovery.

Compatible mower models use this capability through their MCU/board selection.
There is no product-name special case for Biltema RM1000.

Existing F401 installations need one final ST-Link flash of firmware that
advertises and implements USB-DFU entry. The GUI must present that as a clear
migration requirement rather than waiting for a capability that old firmware
cannot report.

## Primary-source hardware facts

The repository board file selects `stm32f401vct6`, `STM32F401xC`, an 8 MHz HSE,
and application USB on PA11/PA12. The application clock configuration derives
72 MHz CPU and 48 MHz USB clocks from that HSE; the generic board manifest's
84 MHz field is not the clock actually configured by `SystemClock_Config()`.

ST documents the STM32F401xB/xC system-memory loader in
[AN2606, section 35, tables 77–78 and figure 45](https://www.st.com/resource/en/application_note/an2606-stm32microcontroller-system-memory-boot-mode-stmicroelectronics.pdf):

- system memory starts at `0x1FFF0000`;
- USB OTG FS DFU uses PA11 (DM) and PA12 (DP), with no external pull-up;
- the ROM loader measures an integer-MHz HSE in the supported 4–26 MHz range;
- the documented ROM communication set also includes USART1 and USART2.

The last point matters on this board: USART1 is the panel bus and USART2 is the
drive-controller bus. The application therefore establishes explicit drive and
blade hardware inhibits before handing control to ROM. Repository code alone
does not prove the external pull states during the reset interval; that is a
mandatory oscilloscope/logic-level HIL check.

[RM0368](https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
documents that a system reset is required to clear the active window watchdog.
[AN3156](https://www.st.com/resource/en/application_note/an3156-usb-dfu-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)
documents ST's DfuSe address, erase, download, upload/readback and leave
commands. The ROM bootloader authenticates no application image; artifact trust
is therefore entirely a host responsibility before erase.

## Architecture

The normal flow is:

1. The user explicitly selects **Update via USB**.
2. The host validates the complete prebuilt artifact and acquires exclusive,
   idempotent update ownership.
3. Fresh firmware, high-level and blade telemetry must prove compatible,
   USB-DFU-capable, IDLE, no active motion/blade command and a stopped blade.
   A failed attempt is terminal; becoming idle later does not start a deferred
   flash.
4. The bridge enables firmware-update maintenance. During maintenance it can
   send only safe zero-motion, blade-off, IDLE, non-releasing heartbeat,
   handshake and ENTER_DFU packets. PID/config bursts, motion, blade enable and
   emergency release are inhibited at the final serial boundary.
5. The bridge sends the explicit `ENTER_DFU` packet. The firmware accepts it
   only while its own high-level state is IDLE.
6. Firmware locks out later motion/blade commands, clears targets/controller
   state, repeatedly commands wheel zero and blade off, and gives the UART motor
   controllers a bounded transmission window.
7. Firmware writes a magic-plus-inverse one-shot token to linker-reserved
   `NOLOAD` SRAM and performs `NVIC_SystemReset()`.
8. The F401 reset handler consumes and clears the token before `.data`, `.bss`,
   clocks, peripherals, USB or watchdogs initialize. Only an intentional
   software reset with a valid token may assert hardware inhibits and jump to
   the validated ROM vector at `0x1FFF0000`. A corrupt/stale token continues a
   normal application boot and cannot create a reset loop.
9. The host accepts only a new STM32 ROM DFU device (`0483:df11`) at the same
   physical USB port as the original `/dev/mowgli` CDC device. Missing,
   unrelated, pre-existing or ambiguous DFU devices fail closed.
10. `dfu-util` writes only at `0x08000000`, performs readback verification, and
    leaves DFU. Tool exit alone is never success.
11. Success requires the Mowgli CDC device to return at the same physical port
    and publish a fresh handshake generation whose protocol, firmware version
    and USB-DFU capability match the manifest.

The reset-marker approach is intentional. Jumping directly from the running
application would need to prove cleanup of USB, DMA, SysTick, timers, UARTs,
pending interrupts, PLL state, IWDG and WWDG. A software reset first removes
that large and failure-prone cleanup surface.

## Normal and recovery policy

Normal USB update requires fresh, consistent stopped telemetry. If the mower is
moving or mowing, the request is rejected and the user must stop it and press
Update again. There is no delayed automatic start.

USB Recovery is an advanced path for the specific case where the application
still communicates and can accept ENTER_DFU, but blade telemetry is stale or
known false (for example, a physically stopped blade reported as approximately
3500 RPM). It requires an explicit confirmation that the mower is physically
stationary and the blade is stopped. Recovery may override only the identified
blade-telemetry failure. It does not override active/unknown high-level state,
non-zero or unknown motion, missing capability, incompatible firmware, artifact
failure, ambiguous USB identity, or the firmware make-safe sequence.

If application USB does not enumerate or the firmware cannot receive
ENTER_DFU, ST-Link is the final recovery path.

## Failure and retry rules

- Validation failure occurs before DFU entry and therefore before erase.
- Only one firmware operation owns the board at a time. An idempotency key can
  reconnect to the same operation but cannot start another transfer.
- Cancellation is safe only before destructive DFU work begins. Once erase or
  write may have started, interruption is reported as potentially requiring
  ST-Link; it is never described as a clean rollback.
- This initial implementation performs no automatic retry. A recoverable
  pre-entry failure returns control to the user for a deliberate new attempt;
  identity, validation, verification and runtime-handshake failures never
  retry automatically.
- DFU write success followed by failed readback, leave, CDC return or runtime
  handshake is not update success.
- Browser disconnect does not block progress logging and cannot duplicate the
  operation. Reopening the UI reads the existing operation state.

## HIL baseline and prerequisites

Record all of the following with every run:

- repository commit and exact firmware artifact SHA-256;
- host architecture/image digest and `dfu-util --version`;
- mainboard marking and STM32 package marking;
- panel/mower variant and USB topology;
- ST-Link firmware/version, attached as observer/recovery;
- blade physically removed and wheels raised or mechanically constrained;
- scope/logic-analyser channels and probe reference.

The remaining state is **HARDWARE_PENDING** only after the exact branch builds,
all deterministic tests pass, the Draft PR exists, and the tested artifact is
attached to that baseline. Until then it is **HARDWARE_REQUIRED**.

## HIL procedure

1. Back up the complete original flash with ST-Link and record its SHA-256.
2. With the blade removed and traction mechanically unable to move the mower,
   install the DFU-capable application once through ST-Link.
3. Verify normal `/dev/mowgli` CDC, matching protocol/version/capability, and
   ordinary zero-motion operation.
4. Scope PE14 (blade nRESET), PE15 (drive OE), drive/blade UART traffic, PA11,
   PA12 and reset. Trigger one USB update while idle. Pass only if the final
   zero/off UART frames precede reset and PE14/PE15 remain at their documented
   inhibited levels through reset and the full ROM dwell.
5. Confirm exactly one `0483:df11` device appears at the original physical USB
   port. Record enumeration time and DfuSe alternate-setting output.
6. Update to a distinct, known version. Read back application flash and compare
   it byte-for-byte with the selected artifact. Confirm CDC returns and the new
   handshake generation/version/protocol/capability matches.
7. Repeat at least 25 successful upgrade/downgrade cycles between compatible
   test builds and retain timing/error logs.
8. Attempt a wrong-board, wrong-panel, bad-protocol, malformed-manifest, empty,
   truncated, oversized, invalid-vector and bad-SHA artifact. Confirm no DFU
   transition and no flash erase.
9. Request update while driving, mowing and with a blade command active.
   Confirm each attempt terminates rejected, commands stop normally, and no
   flash starts later when the mower becomes idle.
10. Inject stale/false blade telemetry. Confirm normal mode rejects it. With the
    mower physically verified stopped, exercise the explicit USB Recovery
    confirmation and re-run the scoped make-safe checks.
11. Disconnect USB before DFU, during enumeration, during erase/write, before
    readback, during leave and before CDC return. Kill `dfu-util` at the same
   boundaries. Confirm bounded failure, no success report, no automatic retry,
   and correct ST-Link guidance.
12. Delay or suppress DFU enumeration; attach an unrelated STM32 DFU device;
    attach two DFU devices. Confirm physical-port/uniqueness checks fail closed.
13. Return the application with wrong version/protocol, corrupt handshake,
    stale cached status and no telemetry. Confirm none can reach success.
14. Intentionally leave application flash incomplete, then recover through the
    documented ST-Link path and byte-verify the restored image.
15. Test controlled power interruption only with a current-limited bench supply
    and an independently supervised procedure. Do not automate mower power or
    blade/motion hardware merely to satisfy this test plan.

### Repeatable cycle runner

The repository includes a deliberately narrow bench helper. A read-only status
lookup is always available:

```bash
python3 tools/firmware/usb_dfu_hil.py \
  --log hil-evidence.jsonl status usb-dfu-1
```

Starting update cycles requires the exact acknowledgement printed by `--help`.
Use it only after removing the blade, mechanically constraining the wheels and
attaching ST-Link for recovery:

```bash
python3 tools/firmware/usb_dfu_hil.py \
  --base-url http://mowgli.local:8080 \
  --log hil-evidence.jsonl run --cycles 25 \
  --i-confirm-bench-safe blade-removed-wheels-constrained-stlink-ready
```

The runner uses one idempotency key per cycle, polls only with GET after the
single start POST, stops at the first non-success terminal state, and records
every observed state transition in append-only JSONL. It does not automate
power interruption, USB unplugging, ST-Link recovery, motion or blade control.

## Identifying DFU and troubleshooting

Before triggering an update, record the physical USB parent of `/dev/mowgli`:

```bash
udevadm info --query=path --name=/dev/mowgli
```

During ROM DFU, the expected ST device is `0483:df11`. Inspect it without
writing flash:

```bash
lsusb -d 0483:df11
dfu-util -d 0483:df11 -l
```

- **USB option disabled on F401:** the live firmware did not advertise the
  capability. Install this DFU-capable firmware once with ST-Link.
- **Readiness rejected:** stop the active mower/blade command and retry as a
  new explicit operation. USB Recovery is only for demonstrably false/stale
  blade telemetry.
- **DFU not found or ambiguous:** do not guess a device. Check the original
  physical port, cabling and `lsusb`; use ST-Link if application USB cannot
  receive the entry request.
- **Write/readback/return/handshake failure:** the GUI must not report success.
  Preserve the log and recover/verify with ST-Link.

## Pass/fail boundary

Passing software tests proves state-machine, policy, protocol, artifact and
failure-handling behaviour under deterministic mocks. Passing HIL additionally
proves only the exact board/artifact/host baseline recorded above. Neither is a
general production-safety claim. Do not remove the Draft/WIP label, mark the PR
ready, or claim Biltema/F401 hardware validation until the physical evidence is
attached and reviewed.
