#!/usr/bin/env python3
"""Bridge Universal GNSS ROS topics onto the public Mowgli GNSS contract."""

from __future__ import annotations

import rclpy
from mowgli_interfaces.msg import GnssStatus as PublicGnssStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rtcm_msgs.msg import Message as PublicRtcmMessage
from universal_gnss_ros2.msg import GnssStatus as UniversalGnssStatus
from universal_gnss_ros2.msg import RtcmFrame


UNIVERSAL_TO_PUBLIC_FIX_TYPE = {
    UniversalGnssStatus.FIX_TYPE_UNKNOWN: PublicGnssStatus.FIX_TYPE_NO_FIX,
    UniversalGnssStatus.FIX_TYPE_NO_FIX: PublicGnssStatus.FIX_TYPE_NO_FIX,
    UniversalGnssStatus.FIX_TYPE_FIX: PublicGnssStatus.FIX_TYPE_GPS_FIX,
    UniversalGnssStatus.FIX_TYPE_RTK_FLOAT: PublicGnssStatus.FIX_TYPE_RTK_FLOAT,
    UniversalGnssStatus.FIX_TYPE_RTK_FIXED: PublicGnssStatus.FIX_TYPE_RTK_FIXED,
    UniversalGnssStatus.FIX_TYPE_DEAD_RECKONING: PublicGnssStatus.FIX_TYPE_DEAD_RECKONING,
}

UNIVERSAL_TO_PUBLIC_RTK_MODE = {
    UniversalGnssStatus.RTK_MODE_UNKNOWN: PublicGnssStatus.RTK_MODE_UNKNOWN,
    UniversalGnssStatus.RTK_MODE_NONE: PublicGnssStatus.RTK_MODE_NONE,
    UniversalGnssStatus.RTK_MODE_FLOAT: PublicGnssStatus.RTK_MODE_FLOAT,
    UniversalGnssStatus.RTK_MODE_FIXED: PublicGnssStatus.RTK_MODE_FIXED,
}

UNIVERSAL_TO_PUBLIC_CAPABILITY = {
    UniversalGnssStatus.CAP_RTK_MODE: PublicGnssStatus.CAP_RTK_MODE,
    UniversalGnssStatus.CAP_HORIZONTAL_ACCURACY: PublicGnssStatus.CAP_HORIZONTAL_ACCURACY,
    UniversalGnssStatus.CAP_VERTICAL_ACCURACY: PublicGnssStatus.CAP_VERTICAL_ACCURACY,
    UniversalGnssStatus.CAP_HDOP: PublicGnssStatus.CAP_HDOP,
    UniversalGnssStatus.CAP_VDOP: PublicGnssStatus.CAP_VDOP,
    UniversalGnssStatus.CAP_SATELLITES_USED: PublicGnssStatus.CAP_SATELLITES_USED,
    UniversalGnssStatus.CAP_SATELLITES_VISIBLE: PublicGnssStatus.CAP_SATELLITES_VISIBLE,
    UniversalGnssStatus.CAP_SATELLITES_TRACKED: PublicGnssStatus.CAP_SATELLITES_TRACKED,
    UniversalGnssStatus.CAP_MEAN_CN0: PublicGnssStatus.CAP_MEAN_CN0,
    UniversalGnssStatus.CAP_MAX_CN0: PublicGnssStatus.CAP_MAX_CN0,
    UniversalGnssStatus.CAP_CORRECTION_AGE: PublicGnssStatus.CAP_CORRECTION_AGE,
    UniversalGnssStatus.CAP_HEADING: PublicGnssStatus.CAP_HEADING,
    UniversalGnssStatus.CAP_DUAL_ANTENNA_HEADING: PublicGnssStatus.CAP_DUAL_ANTENNA_STATUS,
    UniversalGnssStatus.CAP_INTERFERENCE_STATE: PublicGnssStatus.CAP_INTERFERENCE_STATUS,
    UniversalGnssStatus.CAP_JAMMING_STATE: PublicGnssStatus.CAP_JAMMING_STATUS,
}

FIX_TYPE_QUALITY = {
    PublicGnssStatus.FIX_TYPE_NO_FIX: 0.0,
    PublicGnssStatus.FIX_TYPE_GPS_FIX: 25.0,
    PublicGnssStatus.FIX_TYPE_RTK_FLOAT: 50.0,
    PublicGnssStatus.FIX_TYPE_RTK_FIXED: 100.0,
    PublicGnssStatus.FIX_TYPE_DEAD_RECKONING: 10.0,
}


def _normalize_receiver_vendor(receiver_family: str) -> str:
    family = receiver_family.strip().lower()
    if family == "ublox":
        return "u-blox"
    if family == "unicore":
        return "Unicore"
    if family == "nmea":
        return "NMEA"
    return ""


def _map_capability_flags(flags: int) -> int:
    mapped = 0
    for source_flag, target_flag in UNIVERSAL_TO_PUBLIC_CAPABILITY.items():
        if flags & source_flag:
            mapped |= target_flag
    return mapped


class UniversalGnssTopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("universal_gnss_topic_bridge")

        self.declare_parameter("backend", "universal")
        self.declare_parameter("receiver_family", "auto")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("input_status_topic", "/universal_gnss/status")
        self.declare_parameter("output_status_topic", "/gps/status")
        self.declare_parameter("input_rtcm_topic", "/universal_gnss/rtcm")
        self.declare_parameter("output_rtcm_topic", "/rtcm")

        self._backend = str(self.get_parameter("backend").value)
        self._receiver_family = str(self.get_parameter("receiver_family").value)
        self._receiver_vendor = _normalize_receiver_vendor(self._receiver_family)
        self._frame_id = str(self.get_parameter("frame_id").value)

        input_status_topic = str(self.get_parameter("input_status_topic").value)
        output_status_topic = str(self.get_parameter("output_status_topic").value)
        input_rtcm_topic = str(self.get_parameter("input_rtcm_topic").value)
        output_rtcm_topic = str(self.get_parameter("output_rtcm_topic").value)

        reliable_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        rtcm_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._status_pub = self.create_publisher(
            PublicGnssStatus,
            output_status_topic,
            reliable_qos,
        )
        self._rtcm_pub = self.create_publisher(
            PublicRtcmMessage,
            output_rtcm_topic,
            rtcm_qos,
        )

        self.create_subscription(
            UniversalGnssStatus,
            input_status_topic,
            self._on_status,
            reliable_qos,
        )
        self.create_subscription(
            RtcmFrame,
            input_rtcm_topic,
            self._on_rtcm,
            rtcm_qos,
        )

        self.get_logger().info(
            "Bridging Universal GNSS topics: "
            f"{input_status_topic} -> {output_status_topic}, "
            f"{input_rtcm_topic} -> {output_rtcm_topic}"
        )

    def _on_status(self, msg: UniversalGnssStatus) -> None:
        public_msg = PublicGnssStatus()
        public_msg.header.stamp = msg.stamp
        public_msg.header.frame_id = self._frame_id
        public_msg.backend = self._backend
        public_msg.receiver_vendor = self._receiver_vendor

        fix_type = UNIVERSAL_TO_PUBLIC_FIX_TYPE.get(
            msg.fix_type,
            PublicGnssStatus.FIX_TYPE_NO_FIX,
        )
        public_msg.fix_type = fix_type
        public_msg.fix_valid = msg.fix_valid
        public_msg.dead_reckoning = fix_type == PublicGnssStatus.FIX_TYPE_DEAD_RECKONING
        public_msg.rtk_mode = UNIVERSAL_TO_PUBLIC_RTK_MODE.get(
            msg.rtk_mode,
            PublicGnssStatus.RTK_MODE_UNKNOWN,
        )
        public_msg.quality_percent = FIX_TYPE_QUALITY.get(fix_type, 0.0)
        public_msg.capability_flags = _map_capability_flags(msg.capability_flags)
        public_msg.value_flags = _map_capability_flags(msg.value_flags)

        public_msg.hdop = msg.hdop
        public_msg.vdop = msg.vdop
        public_msg.horizontal_accuracy_m = msg.horizontal_accuracy_m
        public_msg.vertical_accuracy_m = msg.vertical_accuracy_m
        public_msg.heading_deg = msg.heading_deg
        public_msg.satellites_used = msg.satellites_used
        public_msg.satellites_visible = msg.satellites_visible
        public_msg.satellites_tracked = msg.satellites_tracked
        public_msg.correction_age_s = msg.correction_age_s
        public_msg.mean_cn0_db_hz = msg.mean_cn0_db_hz
        public_msg.max_cn0_db_hz = msg.max_cn0_db_hz
        public_msg.dual_antenna_heading = msg.dual_antenna_heading
        public_msg.interference_detected = msg.interference_detected
        public_msg.jamming_detected = msg.jamming_detected

        self._status_pub.publish(public_msg)

    def _on_rtcm(self, msg: RtcmFrame) -> None:
        public_msg = PublicRtcmMessage()
        public_msg.message = list(msg.data)
        self._rtcm_pub.publish(public_msg)


def main() -> None:
    rclpy.init()
    node = UniversalGnssTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
