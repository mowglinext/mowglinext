# Howto

Flashing, wiring and serial-debugging howto for the `ros_usbnode` firmware.
For the file map, wire-packet table and pitfalls see
[`docs/claude/codemaps/firmware.md`](../../../docs/claude/codemaps/firmware.md);
[`firmware/CLAUDE.md`](../../CLAUDE.md) indexes the rest of the reference material
(`docs/claude/ros-interfaces.md`, `parameters.md`, `testing-ci.md`, `doc-index.md`).

## Make sure you have a backup of your stock firmware

[Check here how to do that ...](../mainboard_firmware), only then continue.

## Platform.IO - compile and upload

- Start VS Code and open `firmware/stm32/ros_usbnode` in platform.io (or run `pio` from that directory).
- Make sure you Build and Upload the env that matches your mainboard: `Yardforce500` (STM32F103VC, the `default_envs`) or `Yardforce500B` (STM32F401VC). The `Yardforce500_STLINK_V3` / `Yardforce500B_STLINK_V3` variants upload through OpenOCD instead of PlatformIO's built-in ST-Link protocol. From a shell that is e.g. `pio run -e Yardforce500 -t upload`.
- The "Yardforce 500 (STM32F103 VCT6)" and generic `genericSTM32F103C8` (bluepill) envs belong to the old bring-up firmware in [`../test_code`](../test_code), not to this project.
- Having your ST-Link hooked up to the J9 connector on the mainboard (the 4-pin GND / SWCL / SWDA / 3V3 header) the firmware should now be flashed
- The LED (D3) near the STM32 cpu should flash and you should hear a "double" chirp on bootup

## Hardware

- REMOVE THE BLADES !!! (you have been warned)
- Either use a A<->A USB cable or solder up an USB A cable + connector to J14 pin on the mainboard. You need to connect your YF500 mainboard with your Raspi via USB.
- The panel  (J6) must be plugged in as the switches are hardwired to the motor controllers, if they not shorted you will not be able to start the blade motor.
- Brigde all 4 switch cables coming off the panel board (JP4 connector) by shorting the outer pins, emulating a "ok" condition.

## Talking to the board from ROS2

There is no rosserial and no ROS1 master here. The board speaks a COBS-framed,
CRC-16/CCITT-FALSE binary protocol over its USB CDC link - `include/mowgli_protocol.h`
is the single source of truth for that wire format (pinned by `MOWGLI_PROTOCOL_VERSION`)
and the only host peer is `hardware_bridge_node`
([`ros2/src/mowgli_hardware`](../../../ros2/src/mowgli_hardware)), which fans the packets
out onto the ROS2 topics and services. The packet table lives in
[`docs/claude/codemaps/firmware.md`](../../../docs/claude/codemaps/firmware.md), the
resulting topics in [`docs/claude/ros-interfaces.md`](../../../docs/claude/ros-interfaces.md).

- Plug your USB cable into the Raspi on one end and the GForce mainboard on the other, and turn on the bot.
 (note the Raspi will power up the mainbord, but you need the battery on to do drive it actually)

- Open a terminal to your Raspi and run:

```
lsusb
```
  You should see a serial port discovered that looks simliar to:
 
```
  Bus 001 Device 038: ID 0483:5740 STMicroelectronics Virtual COM Port
```
  Figure out which tty device this is with something like:
  
```
dmesg | grep -2 STM |tail -5
[10950.671675] usb 1-1.3: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[10950.671687] usb 1-1.3: Product: Mowgli
[10950.671698] usb 1-1.3: Manufacturer: STMicroelectronics
[10950.671708] usb 1-1.3: SerialNumber: 5CF8673F3430
[10950.676549] cdc_acm 1-1.3:1.0: ttyACM0: USB ACM device
```
  In this example it is /dev/ttyACM0. The installer's udev rules (`install/lib/udev.sh`)
  also create a stable `/dev/mowgli` symlink for that board - that is what the
  `serial_port` parameter of `hardware_bridge` points at by default.

  To see if the stack can talk to the bot, echo one of the topics the bridge feeds
  from the status packet:

```
ros2 topic echo /hardware_bridge/power
```
  You should see a new `v_battery` / `v_charge` reading appear 4 times a second
  (the firmware's status broadcast runs at 250 ms).

## Note

The raspi USB stack is not happy when you flash a new firmware version onto the
mainboard and the CDC stack gets reinitialized. `hardware_bridge_node` handles that
itself: a multi-second gap in the incoming stream on a nominally open port makes it
close and reopen the device (`serial_rx_timeout_s`), so a flash or a board reboot
self-heals. If you are talking to the port with your own tool instead, unplug and
replug the USB cable to settle things.

## Drive the bot

Manual driving goes through twist_mux on `/cmd_vel_teleop` (priority 20, above
navigation and docking), which the GUI's manual-control page and
`cmd_vel_ws_relay.py` publish to; the merged output reaches the board as a
`PKT_ID_CMD_VEL` packet.

Two firmware gates to know about before wondering why nothing moves:

- The board boots into `OPENMOWER_STATUS_IDLE` and simply drops `cmd_vel` (and forces
  the blade target to 0) until the host sends a high-level state other than IDLE.
- A `cmd_vel` older than 200 ms is a hard stop, so the source has to keep publishing.

The twist is not forwarded raw: the board closes BOTH the per-wheel velocity PI loop
(`USE_WHEEL_PI`) and the gyro-based yaw-rate loop on it, and the anti-dig cutout
(`ANTIDIG_*`) can zero a wheel that is spinning without making progress -
all in `src/ros/ros_custom/cpp_main.cpp` `motors_handler()`. The ROS2 side does no
host-side shaping. Gains are pushed down at every reconnect and never persisted on
the board, so do not "fix" a tuning problem by editing `board.h`.

## Enable UART5 on raspi for serial debugging

add this to /boot/firmware/usercfg.txt

```
dtoverlay=uart5
```

then reboot.

## Serial Debugging

<a id="serial_debug">

Note this only applies to the Yardforce 500 ORIG (STM32F103VC) build: `board.h` sets
`DEBUG_TYPE DEBUG_TYPE_UART` there, and the debug port is UART4 on J18 pins 7/8.
The 500B (STM32F401VC) build has no master USART and traces over SWO instead -
selecting `DEBUG_TYPE_UART` on it is a compile-time `#error`. Use `pio run -t swo_viewer`
for that board.

Wire your serial adapter (or ESP32, or Raspi) to the serial port on the GForce board.
   
I used the J18 (Red connector on the mainboard) because the connector from J5 (Signal will fit) and i dont need the signal sense board anymore.   
As the pins are unfortunatly in the wrong place on the original J5 connector i used a sharp pick tool to relocate the pins.
   
When you then flash the ros_usbnode firmware or reboot the board you should see output simliar to
   
```
 * Master USART (debug) initialized
 * LED initialized
 * Charging ADC initialized
 * Timer3 (Beeper) initialized
 * 24V switched on
 * RAIN Sensor enabled
 * HALL Sensor enabled
 * Hard I2C initialized
 * Accelerometer (onboard/tilt safety) initialized
 * Soft I2C (J18) initialized
 * Testing supported IMUs:
 * Panel initialized
 * Emergency sensors initialized
 * Timer1 (Charge PWM) initialized
 * USB CDC initialized
 * Drive Motors USART initialized
 * NBT Main timers initialized
 * ROS serial node initialized

 >>> entering main loop ...
```

An ASCII "Mowgli" banner and a decoded reset cause are printed just above that list.
After the banner the board is quiet unless something happens - the periodic
charge/battery voltage chatter of the old firmware is gone; battery and charge
voltage now travel to the host in the status packet instead.
   
   
