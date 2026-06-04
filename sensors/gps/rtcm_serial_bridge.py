#!/usr/bin/env python3
"""Forward NTRIP RTCM3 corrections to a serial GPS receiver.

The ntrip_client_node publishes parsed RTCM3 frames on /ntrip_client/rtcm
(rtcm_msgs/Message). The UBX path (serial_ublox_driver.py) consumes that
topic directly because it owns the serial port. The NMEA path uses
nmea_navsat_driver, which is read-only — it never writes RTCM back into
the receiver. Without this bridge an LC29H or similar NMEA-capable RTK
receiver gets a fix but stays at autonomous accuracy, even with NTRIP
configured.

Linux allows multiple processes to hold the same /dev/ttyXXX open. The
NMEA driver opens it for read with its own baud/termios settings; we
open it write-only with no termios changes so the kernel keeps the
driver's baud rate. Writes from this single writer don't race the
driver's reads.

Subscribes:
  /ntrip_client/rtcm      rtcm_msgs/Message     RTCM3 corrections from NTRIP

Parameters:
  device          (string)  default '/dev/gps'   serial device to write to

Publishes nothing — this is a one-way bridge. The gps_health_aggregator
counts the same /ntrip_client/rtcm topic for diagnostics.
"""
from __future__ import annotations

import os
import select
import time

import rclpy
from rclpy.node import Node

from rtcm_msgs.msg import Message as RtcmMessage


class RtcmSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("rtcm_serial_bridge")
        self.declare_parameter("device", "/dev/gps")
        self.device_path: str = self.get_parameter("device").value

        self.fd: int | None = None
        self._open_device()

        self.bytes_written = 0
        self.msgs_written = 0
        self.last_log_t = time.time()

        self.create_subscription(
            RtcmMessage, "/ntrip_client/rtcm", self._on_rtcm, 50
        )
        self.get_logger().info(
            f"rtcm_serial_bridge: forwarding /ntrip_client/rtcm → {self.device_path}"
        )

    def _open_device(self) -> None:
        # Open write-only, non-blocking, no controlling-terminal claim, no
        # termios reset — the NMEA driver already configured baud/parity.
        for attempt in range(20):
            try:
                self.fd = os.open(
                    self.device_path,
                    os.O_WRONLY | os.O_NONBLOCK | os.O_NOCTTY,
                )
                self.get_logger().info(f"opened {self.device_path} for RTCM write")
                return
            except OSError as e:
                self.get_logger().warn(
                    f"open {self.device_path} attempt {attempt + 1}: {e}"
                )
                time.sleep(1.0)
        raise RuntimeError(f"could not open {self.device_path} for write")

    def _try_reopen(self) -> bool:
        """Single non-blocking open attempt for the hot path. Never sleeps,
        never raises — returns True on success so _on_rtcm can proceed, False
        to drop the frame and retry on the next one."""
        try:
            self.fd = os.open(
                self.device_path, os.O_WRONLY | os.O_NONBLOCK | os.O_NOCTTY
            )
            self.get_logger().info(f"reopened {self.device_path} for RTCM write")
            return True
        except OSError:
            return False

    def _on_rtcm(self, msg: RtcmMessage) -> None:
        if self.fd is None:
            # Single non-blocking reopen attempt (NOT the 20×1s blocking
            # _open_device — that would stall the executor for up to 20 s
            # inside this subscription callback and raise on failure, killing
            # the node). If it fails we just drop this frame and retry next.
            if not self._try_reopen():
                return
        data = bytes(msg.message)
        try:
            # Write the WHOLE frame: os.write may do a partial write on a
            # non-blocking fd, and a truncated RTCM3 frame corrupts the
            # correction stream (the receiver discards it). Loop over the
            # remainder, waiting briefly for writability; drop the frame only
            # if it can't be flushed within a bounded budget.
            view = memoryview(data)
            sent = 0
            deadline = time.time() + 0.2  # 200 ms budget per frame
            while sent < len(data):
                try:
                    sent += os.write(self.fd, view[sent:])
                except BlockingIOError:
                    if time.time() >= deadline:
                        self.get_logger().warn(
                            f"serial write stalled — dropping RTCM frame "
                            f"({sent}/{len(data)} bytes flushed)"
                        )
                        break
                    select.select([], [self.fd], [], 0.02)
            self.bytes_written += sent
            self.msgs_written += 1
        except OSError as e:
            self.get_logger().error(f"serial write: {e} — will reopen on next frame")
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
            return

        now = time.time()
        if now - self.last_log_t > 5.0:
            self.get_logger().info(
                f"forwarded msgs={self.msgs_written} bytes={self.bytes_written}"
            )
            self.last_log_t = now

    def destroy_node(self) -> bool:
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RtcmSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
