# STM32 wire protocol

The STM32 and ROS 2 bridge exchange packed little-endian packets framed with
COBS and protected by CRC-16. A zero byte delimits each encoded frame; the CRC
covers the packet payload before its CRC field.

The maintained firmware implementation and packet definitions are in
[`firmware/stm32/ros_usbnode`](../../../../firmware/stm32/ros_usbnode/).
The host implementation and its packet definitions are in
[`mowgli_hardware/include/mowgli_hardware`](../include/mowgli_hardware/).
The two protocol version constants must remain in sync; see the
[protocol version guard](../../../../firmware/scripts/protocol_version_guard.py).

The original reference implementation in this directory was derived from
OpenMower STM32 firmware and was distributed under GPL-3.0. It is available in
the repository history for porting reference; use the maintained sources above
for current packet layouts and behavior.
