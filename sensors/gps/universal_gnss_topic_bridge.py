#!/usr/bin/env python3
"""Bridge Universal GNSS ROS topics onto the public Mowgli GNSS contract."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
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
    if "write error" in normalized or "write errors" in normalized or normalized.endswith("error"):
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_ERROR
    if "unavailable" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_UNAVAILABLE
    if "waiting" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_WAITING
    if "active" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_ACTIVE
    if "stale" in normalized or "idle" in normalized:
        return PublicGnssStatus.CORRECTION_STREAM_STATUS_IDLE
    return PublicGnssStatus.CORRECTION_STREAM_STATUS_UNKNOWN


@dataclass
class _CorrectionSnapshot:
    valid: bool
    source: str
    entries: dict[str, tuple[str, dict[str, str]]]
    received_at: float


class CorrectionDiagnosticTracker:
    """Bounded, source-owned Universal GNSS correction projection.

    Arrays that contain an owner prefix replace that owner's complete snapshot,
    while unrelated arrays are OMIT. Monotonic receipt time owns liveness; the
    ROS diagnostic stamp is ordering evidence only.
    """

    _NTRIP_PREFIX = "universal_gnss_ntrip/"
    _RECEIVER_PREFIX = "universal_gnss/"
    _SEMANTIC_FRESHNESS_S = 5.0

    def __init__(self, timeout_s: float) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("correction diagnostic timeout must be finite and positive")
        self._timeout_s = timeout_s
        self._ntrip: _CorrectionSnapshot | None = None
        self._receiver: _CorrectionSnapshot | None = None
        self._ntrip_stamp_watermark_ns: int | None = None
        self._receiver_stamp_watermark_ns: int | None = None

    @staticmethod
    def _stamp_ns(msg: DiagnosticArray) -> int | None:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        sec = int(getattr(stamp, "sec", 0))
        nanosec = int(getattr(stamp, "nanosec", 0))
        return None if sec == 0 and nanosec == 0 else sec * 1_000_000_000 + nanosec

    def _replace_owner(
        self,
        msg: DiagnosticArray,
        prefix: str,
        received_at: float,
        snapshot_attr: str,
        watermark_attr: str,
    ) -> None:
        represented = False
        source_ids: set[str] = set()
        entries: dict[str, tuple[str, dict[str, str]]] = {}
        for status in msg.status:
            if not status.name.startswith(prefix):
                continue
            represented = True
            hardware_id = status.hardware_id.strip()
            if hardware_id:
                source_ids.add(hardware_id)
            entries[status.name] = (status.message.strip(), _diagnostic_value_map(status))

        if not represented:
            return

        stamp_ns = self._stamp_ns(msg)
        watermark = getattr(self, watermark_attr)
        if stamp_ns is not None and watermark is not None and stamp_ns < watermark:
            return
        if stamp_ns is not None and (watermark is None or stamp_ns > watermark):
            setattr(self, watermark_attr, stamp_ns)

        valid = len(source_ids) == 1
        setattr(
            self,
            snapshot_attr,
            _CorrectionSnapshot(
                valid=valid,
                source=next(iter(source_ids)) if valid else "",
                entries=entries if valid else {},
                received_at=received_at,
            ),
        )

    def update(self, msg: DiagnosticArray, received_at: float | None = None) -> None:
        now = time.monotonic() if received_at is None else received_at
        self._replace_owner(
            msg,
            self._NTRIP_PREFIX,
            now,
            "_ntrip",
            "_ntrip_stamp_watermark_ns",
        )
        self._replace_owner(
            msg,
            self._RECEIVER_PREFIX,
            now,
            "_receiver",
            "_receiver_stamp_watermark_ns",
        )

    def _current(self, snapshot: _CorrectionSnapshot | None, now: float) -> _CorrectionSnapshot | None:
        if snapshot is None or not snapshot.valid or now < snapshot.received_at:
            return None
        return snapshot if now - snapshot.received_at < self._timeout_s else None

    @staticmethod
    def _transport_status(snapshot: _CorrectionSnapshot) -> int:
        names = snapshot.entries
        if "universal_gnss_ntrip/ntrip_failed" in names:
            return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_FAILED
        if "universal_gnss_ntrip/ntrip_reconnecting" in names:
            return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_RECONNECTING
        if "universal_gnss_ntrip/ntrip_streaming" in names:
            return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_STREAMING
        if "universal_gnss_ntrip/ntrip_connected" in names:
            return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_CONNECTED
        if "universal_gnss_ntrip/ntrip_connecting" in names:
            return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_CONNECTING
        if "universal_gnss_ntrip/ntrip_disconnected" in names:
            return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_DISCONNECTED
        return PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_UNKNOWN

    @staticmethod
    def _observation(entry: tuple[str, dict[str, str]] | None) -> dict[str, Any]:
        if entry is None:
            return {"present": False, "has_value": False}
        values = entry[1]
        parsed = {
            "present": True,
            "seen": _parse_diagnostic_bool(values.get("seen")),
            "decoded": _parse_diagnostic_bool(values.get("decoded")),
            "valid": _parse_diagnostic_bool(values.get("valid")),
            "message_type": _parse_diagnostic_uint(values.get("message_type")),
            "station_id": _parse_diagnostic_uint(values.get("station_id")),
            "satellite_count": _parse_diagnostic_uint(values.get("satellite_count")),
            "signal_count": _parse_diagnostic_uint(values.get("signal_count")),
            "cell_count": _parse_diagnostic_uint(values.get("cell_count")),
            "decode_failure_count": _parse_diagnostic_uint(values.get("decode_failure_count")),
            "malformed_count": _parse_diagnostic_uint(values.get("malformed_count")),
            "age_s": _parse_diagnostic_float(values.get("age_s")),
            "constellations_seen": values.get("constellations_seen", ""),
        }
        parsed["has_value"] = any(
            value is not None and value != ""
            for key, value in parsed.items()
            if key not in ("present", "has_value")
        )
        return parsed

    @staticmethod
    def _flow_status(snapshot: _CorrectionSnapshot, transport: int) -> int:
        if transport != PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_STREAMING:
            return PublicGnssStatus.CORRECTION_FLOW_STATUS_IDLE
        summary = snapshot.entries.get("universal_gnss_ntrip/summary")
        values = {} if summary is None else summary[1]
        if _parse_diagnostic_bool(values.get("parser_healthy")) is False:
            return PublicGnssStatus.CORRECTION_FLOW_STATUS_INVALID
        if (
            _parse_diagnostic_bool(values.get("stale_data")) is True
            or "universal_gnss_ntrip/rtcm.stream_stale" in snapshot.entries
        ):
            return PublicGnssStatus.CORRECTION_FLOW_STATUS_STALE
        if "universal_gnss_ntrip/correction_flowing" in snapshot.entries:
            return PublicGnssStatus.CORRECTION_FLOW_STATUS_ACTIVE
        return PublicGnssStatus.CORRECTION_FLOW_STATUS_WAITING

    @classmethod
    def _semantic_status(cls, snapshot: _CorrectionSnapshot, transport: int) -> int:
        if transport != PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_STREAMING:
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_UNAVAILABLE
        summary = snapshot.entries.get("universal_gnss_ntrip/summary")
        if summary is None:
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_WAITING
        values = summary[1]
        parser_healthy = _parse_diagnostic_bool(values.get("parser_healthy"))
        if parser_healthy is False:
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_INVALID
        if _parse_diagnostic_bool(values.get("stale_data")) is True:
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_STALE

        correction_available = _parse_diagnostic_bool(values.get("correction_available"))
        base = cls._observation(
            snapshot.entries.get("universal_gnss_ntrip/rtcm_semantic/base_station_arp")
        )
        msm = cls._observation(
            snapshot.entries.get("universal_gnss_ntrip/rtcm_semantic/msm_summary")
        )
        base_usable = (
            base["present"]
            and base.get("seen") is True
            and base.get("decoded") is True
            and base.get("valid") is True
        )
        age_s = msm.get("age_s")
        msm_fresh = age_s is not None and 0.0 <= age_s <= cls._SEMANTIC_FRESHNESS_S
        msm_usable = (
            msm["present"]
            and msm.get("seen") is True
            and msm.get("decoded") is True
            and msm.get("valid") is True
            and (msm.get("cell_count") or 0) > 0
            and msm_fresh
            and (msm.get("malformed_count") or 0) == 0
        )
        station_matches = (
            (base.get("station_id") or 0) > 0
            and base.get("station_id") == msm.get("station_id")
        )
        if correction_available is True and parser_healthy is True and base_usable and msm_usable and station_matches:
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_HEALTHY
        if (
            (msm["present"] and msm.get("seen") is True and (
                msm.get("decoded") is not True
                or msm.get("valid") is not True
                or (msm.get("cell_count") or 0) == 0
                or (msm.get("malformed_count") or 0) > 0
            ))
            or (base["present"] and base.get("seen") is True and (
                base.get("decoded") is not True or base.get("valid") is not True
            ))
            or (base_usable and msm_usable and not station_matches)
            or correction_available is True
        ):
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_INVALID
        if msm["present"] and msm.get("seen") is True and not msm_fresh:
            return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_STALE
        return PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_WAITING

    @classmethod
    def _apply_msm(
        cls,
        snapshot: _CorrectionSnapshot,
        name: str,
        public_msg: PublicGnssStatus,
    ) -> None:
        msm = cls._observation(snapshot.entries.get(name))
        if not msm["present"]:
            return
        public_msg.msm_summary_seen = bool(msm.get("seen"))
        public_msg.msm_summary_decoded = bool(msm.get("decoded"))
        public_msg.msm_summary_valid = bool(msm.get("valid"))
        public_msg.msm_summary_message_type = msm.get("message_type") or 0
        public_msg.msm_summary_station_id = msm.get("station_id") or 0
        public_msg.msm_summary_constellations_seen = msm.get("constellations_seen") or ""
        public_msg.msm_summary_satellite_count = msm.get("satellite_count") or 0
        public_msg.msm_summary_signal_count = msm.get("signal_count") or 0
        public_msg.msm_summary_cell_count = msm.get("cell_count") or 0
        public_msg.msm_summary_age_s = msm.get("age_s") or 0.0
        public_msg.msm_summary_source = snapshot.source
        if msm["has_value"]:
            public_msg.value_flags |= PublicGnssStatus.CAP_MSM_SUMMARY

    def apply(self, public_msg: PublicGnssStatus, now: float | None = None) -> None:
        current_time = time.monotonic() if now is None else now
        correction_capabilities = (
            PublicGnssStatus.CAP_CORRECTION_STREAM
            | PublicGnssStatus.CAP_MSM_SUMMARY
            | PublicGnssStatus.CAP_CORRECTION_TRANSPORT
            | PublicGnssStatus.CAP_CORRECTION_FLOW
            | PublicGnssStatus.CAP_CORRECTION_SEMANTIC
        )
        public_msg.capability_flags |= correction_capabilities
        public_msg.value_flags &= ~correction_capabilities
        public_msg.correction_stream_status = PublicGnssStatus.CORRECTION_STREAM_STATUS_UNKNOWN
        public_msg.correction_transport_status = PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_UNKNOWN
        public_msg.correction_response_accepted = False
        public_msg.correction_flow_status = PublicGnssStatus.CORRECTION_FLOW_STATUS_UNKNOWN
        public_msg.correction_semantic_status = PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_UNKNOWN
        public_msg.correction_source = ""
        public_msg.correction_forwarding_source = ""
        public_msg.msm_summary_source = ""

        ntrip = self._current(self._ntrip, current_time)
        receiver = self._current(self._receiver, current_time)
        forwarding_owner = receiver
        forwarding = None if receiver is None else receiver.entries.get("universal_gnss/rtcm_forwarding")
        if receiver is None:
            forwarding_owner = ntrip
            forwarding = None if ntrip is None else ntrip.entries.get(
                "universal_gnss_ntrip/rtcm_forwarding"
            )
        if forwarding is not None and forwarding_owner is not None:
            public_msg.correction_stream_status = _correction_stream_status_from_message(forwarding[0])
            public_msg.correction_forwarding_source = forwarding_owner.source
            if public_msg.correction_stream_status != PublicGnssStatus.CORRECTION_STREAM_STATUS_UNKNOWN:
                public_msg.value_flags |= PublicGnssStatus.CAP_CORRECTION_STREAM

        if ntrip is not None:
            transport = self._transport_status(ntrip)
            public_msg.correction_transport_status = transport
            public_msg.correction_response_accepted = (
                transport == PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_STREAMING
            )
            public_msg.correction_flow_status = self._flow_status(ntrip, transport)
            public_msg.correction_semantic_status = self._semantic_status(ntrip, transport)
            public_msg.correction_source = ntrip.source
            if transport != PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_UNKNOWN:
                public_msg.value_flags |= PublicGnssStatus.CAP_CORRECTION_TRANSPORT
            public_msg.value_flags |= PublicGnssStatus.CAP_CORRECTION_FLOW
            public_msg.value_flags |= PublicGnssStatus.CAP_CORRECTION_SEMANTIC

        if (
            ntrip is not None
            and public_msg.correction_transport_status
            == PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_STREAMING
        ):
            self._apply_msm(
                ntrip,
                "universal_gnss_ntrip/rtcm_semantic/msm_summary",
                public_msg,
            )
        elif receiver is not None:
            self._apply_msm(
                receiver,
                "universal_gnss/rtcm_semantic/msm_summary",
                public_msg,
            )


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
        self.declare_parameter("correction_diagnostic_timeout_s", 2.0)

        self._backend = str(self.get_parameter("backend").value)
        self._receiver_family = str(self.get_parameter("receiver_family").value)
        self._receiver_vendor = _normalize_receiver_vendor(self._receiver_family)
        self._frame_id = str(self.get_parameter("frame_id").value)

        input_status_topic = str(self.get_parameter("input_status_topic").value)
        output_status_topic = str(self.get_parameter("output_status_topic").value)
        input_diagnostics_topic = str(self.get_parameter("input_diagnostics_topic").value)
        input_rtcm_topic = str(self.get_parameter("input_rtcm_topic").value)
        output_rtcm_topic = str(self.get_parameter("output_rtcm_topic").value)

        self._correction_diagnostics = CorrectionDiagnosticTracker(
            float(self.get_parameter("correction_diagnostic_timeout_s").value)
        )

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
        # Receiver observation identity, copied verbatim. It advances only for a
        # genuinely accepted position observation and stays put for the
        # timer-driven cached republication, which is what lets consumers tell a
        # frozen receiver from a live one. Mirrors the C++ bridge exactly; see
        # mowgli_interfaces/gnss_observation_freshness.hpp.
        public_msg.position_observation_sequence = msg.position_observation_sequence
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
        self._correction_diagnostics.update(msg)

    def _apply_diagnostic_projection(self, public_msg: PublicGnssStatus) -> None:
        self._correction_diagnostics.apply(public_msg)

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
