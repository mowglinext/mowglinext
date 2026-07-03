#!/usr/bin/env python3
"""Bridge Universal GNSS ROS topics onto the public Mowgli GNSS contract."""

from __future__ import annotations

from typing import Any

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from mowgli_interfaces.msg import GnssStatus as PublicGnssStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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

UNIVERSAL_TO_PUBLIC_BASELINE_STATUS = {
    UniversalGnssStatus.BASELINE_STATUS_UNKNOWN: PublicGnssStatus.BASELINE_STATUS_UNKNOWN,
    UniversalGnssStatus.BASELINE_STATUS_COMPUTED: PublicGnssStatus.BASELINE_STATUS_COMPUTED,
    UniversalGnssStatus.BASELINE_STATUS_NOT_SOLVED: PublicGnssStatus.BASELINE_STATUS_NOT_SOLVED,
    UniversalGnssStatus.BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS:
        PublicGnssStatus.BASELINE_STATUS_INSUFFICIENT_OBSERVATIONS,
    UniversalGnssStatus.BASELINE_STATUS_NO_CONVERGENCE:
        PublicGnssStatus.BASELINE_STATUS_NO_CONVERGENCE,
    UniversalGnssStatus.BASELINE_STATUS_OUT_OF_TOLERANCE:
        PublicGnssStatus.BASELINE_STATUS_OUT_OF_TOLERANCE,
    UniversalGnssStatus.BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED:
        PublicGnssStatus.BASELINE_STATUS_COVARIANCE_TRACE_EXCEEDED,
    UniversalGnssStatus.BASELINE_STATUS_NOT_CONFIGURED:
        PublicGnssStatus.BASELINE_STATUS_NOT_CONFIGURED,
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
    UniversalGnssStatus.CAP_HEADING_ACCURACY: PublicGnssStatus.CAP_HEADING_ACCURACY,
    UniversalGnssStatus.CAP_DIFFERENTIAL_CORRECTIONS: PublicGnssStatus.CAP_DIFFERENTIAL_CORRECTIONS,
    UniversalGnssStatus.CAP_CORRECTIONS_ACTIVE: PublicGnssStatus.CAP_CORRECTIONS_ACTIVE,
    UniversalGnssStatus.CAP_DUAL_ANTENNA_HEADING: PublicGnssStatus.CAP_DUAL_ANTENNA_STATUS,
    UniversalGnssStatus.CAP_INTERFERENCE_STATE: PublicGnssStatus.CAP_INTERFERENCE_STATUS,
    UniversalGnssStatus.CAP_JAMMING_STATE: PublicGnssStatus.CAP_JAMMING_STATUS,
    UniversalGnssStatus.CAP_DUAL_ANTENNA_BASELINE: PublicGnssStatus.CAP_DUAL_ANTENNA_BASELINE,
    UniversalGnssStatus.CAP_BASELINE_AZIMUTH: PublicGnssStatus.CAP_BASELINE_AZIMUTH,
    UniversalGnssStatus.CAP_BASELINE_PITCH: PublicGnssStatus.CAP_BASELINE_PITCH,
    UniversalGnssStatus.CAP_BASELINE_LENGTH: PublicGnssStatus.CAP_BASELINE_LENGTH,
    UniversalGnssStatus.CAP_BASELINE_SOLUTION_STATUS: PublicGnssStatus.CAP_BASELINE_SOLUTION_STATUS,
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


def _parse_diagnostic_bool(value: str | None) -> bool | None:
    if value is None:
        return None
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def _parse_diagnostic_uint(value: str | None) -> int | None:
    if value is None or value.strip() == "":
        return None
    try:
        parsed = int(value, 10)
    except ValueError:
        return None
    return parsed if parsed >= 0 else None


def _parse_diagnostic_float(value: str | None) -> float | None:
    if value is None or value.strip() == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _diagnostic_value_map(status: Any) -> dict[str, str]:
    values: dict[str, str] = {}
    for item in getattr(status, "values", []):
        key = getattr(item, "key", "").strip()
        if not key:
            continue
        values[key] = getattr(item, "value", "").strip()
    return values


def _correction_stream_status_from_message(message: str) -> int:
    normalized = message.strip().lower()
    if "write error" in normalized or normalized.endswith("error"):
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_ERROR
    if "unavailable" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_UNAVAILABLE
    if "waiting" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_WAITING
    if "active" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_ACTIVE
    if "idle" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_IDLE
    return PublicGnssStatus.CORRECTION_STREAM_STATUS_UNKNOWN


class UniversalGnssTopicBridge(Node):
    def __init__(self) -> None:
        super().__init__("universal_gnss_topic_bridge")

        self.declare_parameter("backend", "universal")
        self.declare_parameter("receiver_family", "auto")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("input_status_topic", "/_gps_internal/universal/status")
        self.declare_parameter("output_status_topic", "/gps/status")
        self.declare_parameter("input_diagnostics_topic", "/diagnostics")
        self.declare_parameter("input_rtcm_topic", "/_gps_internal/universal/rtcm")
        self.declare_parameter("output_rtcm_topic", "/rtcm")

        self._backend = str(self.get_parameter("backend").value)
        self._receiver_family = str(self.get_parameter("receiver_family").value)
        self._receiver_vendor = _normalize_receiver_vendor(self._receiver_family)
        self._frame_id = str(self.get_parameter("frame_id").value)

        input_status_topic = str(self.get_parameter("input_status_topic").value)
        output_status_topic = str(self.get_parameter("output_status_topic").value)
        input_diagnostics_topic = str(self.get_parameter("input_diagnostics_topic").value)
        input_rtcm_topic = str(self.get_parameter("input_rtcm_topic").value)
        output_rtcm_topic = str(self.get_parameter("output_rtcm_topic").value)

        self._diagnostic_entries: dict[str, tuple[str, dict[str, str]]] = {}

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
            DiagnosticArray,
            input_diagnostics_topic,
            self._on_diagnostics,
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
            f"{input_diagnostics_topic} -> {output_status_topic} correction_stream/msm_summary, "
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
        public_msg.heading_accuracy_deg = msg.heading_accuracy_deg
        public_msg.differential_corrections = msg.differential_corrections
        public_msg.corrections_active = msg.corrections_active
        public_msg.satellites_used = msg.satellites_used
        public_msg.satellites_visible = msg.satellites_visible
        public_msg.satellites_tracked = msg.satellites_tracked
        public_msg.correction_age_s = msg.correction_age_s
        public_msg.mean_cn0_db_hz = msg.mean_cn0_db_hz
        public_msg.max_cn0_db_hz = msg.max_cn0_db_hz
        public_msg.dual_antenna_heading = msg.dual_antenna_heading
        public_msg.dual_antenna_baseline = msg.dual_antenna_baseline
        public_msg.interference_detected = msg.interference_detected
        public_msg.jamming_detected = msg.jamming_detected
        public_msg.baseline_azimuth_deg = msg.baseline_azimuth_deg
        public_msg.baseline_pitch_deg = msg.baseline_pitch_deg
        public_msg.baseline_length_m = msg.baseline_length_m
        public_msg.baseline_solution_status = UNIVERSAL_TO_PUBLIC_BASELINE_STATUS.get(
            msg.baseline_solution_status,
            PublicGnssStatus.BASELINE_STATUS_UNKNOWN,
        )

        self._apply_diagnostic_projection(public_msg)

        self._status_pub.publish(public_msg)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            self._diagnostic_entries[status.name] = (
                status.message.strip(),
                _diagnostic_value_map(status),
            )

    def _pick_diagnostic_entry(
        self,
        *names: str,
    ) -> tuple[str, dict[str, str]] | None:
        for name in names:
            entry = self._diagnostic_entries.get(name)
            if entry is not None:
                return entry
        return None

    def _apply_diagnostic_projection(self, public_msg: PublicGnssStatus) -> None:
        correction_stream = self._derive_correction_stream()
        if correction_stream is not None:
            public_msg.capability_flags |= PublicGnssStatus.CAP_CORRECTION_STREAM
            public_msg.correction_stream_status = correction_stream["correction_stream_status"]
            if correction_stream["has_value"]:
                public_msg.value_flags |= PublicGnssStatus.CAP_CORRECTION_STREAM

        msm_summary = self._derive_msm_summary()
        if msm_summary is not None:
            public_msg.capability_flags |= PublicGnssStatus.CAP_MSM_SUMMARY
            public_msg.msm_summary_seen = msm_summary["seen"]
            public_msg.msm_summary_decoded = msm_summary["decoded"]
            public_msg.msm_summary_valid = msm_summary["valid"]
            public_msg.msm_summary_message_type = msm_summary["message_type"]
            public_msg.msm_summary_station_id = msm_summary["station_id"]
            public_msg.msm_summary_constellations_seen = msm_summary["constellations_seen"]
            public_msg.msm_summary_satellite_count = msm_summary["satellite_count"]
            public_msg.msm_summary_signal_count = msm_summary["signal_count"]
            public_msg.msm_summary_cell_count = msm_summary["cell_count"]
            public_msg.msm_summary_age_s = msm_summary["age_s"]
            if msm_summary["has_value"]:
                public_msg.value_flags |= PublicGnssStatus.CAP_MSM_SUMMARY

    def _derive_correction_stream(self) -> dict[str, Any] | None:
        entry = self._pick_diagnostic_entry(
            "universal_gnss_ntrip/rtcm_forwarding",
            "universal_gnss/rtcm_forwarding",
        )
        if entry is None:
            return None

        correction_stream_status = _correction_stream_status_from_message(entry[0])
        return {
            "correction_stream_status": correction_stream_status,
            "has_value": correction_stream_status != PublicGnssStatus.CORRECTION_STREAM_STATUS_UNKNOWN,
        }

    def _derive_msm_summary(self) -> dict[str, Any] | None:
        entry = self._pick_diagnostic_entry(
            "universal_gnss_ntrip/rtcm_semantic/msm_summary",
            "universal_gnss/rtcm_semantic/msm_summary",
        )
        if entry is None:
            return None

        values = entry[1]
        seen = _parse_diagnostic_bool(values.get("seen"))
        decoded = _parse_diagnostic_bool(values.get("decoded"))
        valid = _parse_diagnostic_bool(values.get("valid"))
        message_type = _parse_diagnostic_uint(values.get("message_type"))
        station_id = _parse_diagnostic_uint(values.get("station_id"))
        satellite_count = _parse_diagnostic_uint(values.get("satellite_count"))
        signal_count = _parse_diagnostic_uint(values.get("signal_count"))
        cell_count = _parse_diagnostic_uint(values.get("cell_count"))
        age_s = _parse_diagnostic_float(values.get("age_s"))
        constellations_seen = values.get("constellations_seen", "")

        has_value = any((
            seen is not None,
            decoded is not None,
            valid is not None,
            message_type is not None,
            station_id is not None,
            bool(constellations_seen),
            satellite_count is not None,
            signal_count is not None,
            cell_count is not None,
            age_s is not None,
        ))

        return {
            "seen": bool(seen),
            "decoded": bool(decoded),
            "valid": bool(valid),
            "message_type": 0 if message_type is None else message_type,
            "station_id": 0 if station_id is None else station_id,
            "constellations_seen": constellations_seen,
            "satellite_count": 0 if satellite_count is None else satellite_count,
            "signal_count": 0 if signal_count is None else signal_count,
            "cell_count": 0 if cell_count is None else cell_count,
            "age_s": 0.0 if age_s is None else age_s,
            "has_value": has_value,
        }

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
