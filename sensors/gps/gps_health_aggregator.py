#!/usr/bin/env python3
# GPS health aggregator — subscribes to ublox_dgnss UBX topics and to the
# NTRIP RTCM stream, publishes a 1 Hz diagnostic_msgs/DiagnosticArray on
# the ROS2-standard /diagnostics topic so the GUI's Diagnostics page
# (which already aggregates by-name from /diagnostics) renders these
# entries alongside everything else.
#
# Three diagnostic groups:
#   * GPS Fix           : carr_soln (none/float/fixed), gps_fix_ok,
#                         diff_corr, ttff_s, sigma_xy_mm, hdop/vdop,
#                         horizontal/vertical accuracy
#   * GPS Satellites    : visible / used counts, mean CN0 of used sats,
#                         CN0 ≥ 40 dB-Hz count (RTK-Fixed signal floor)
#   * NTRIP/RTCM        : msgs/s, % used, mean age of last correction,
#                         the set of RTCM types observed in the last
#                         window (1004/1077/1087/1097/…)
#
# Levels follow standard convention: OK = green, WARN = "still float /
# weak signal", ERROR = no fix, no RTCM, or CRC-failing stream.
#
# NAV-PVT fallback: the Fix and Satellites diagnostics prefer UBX-NAV-STATUS
# and UBX-NAV-SAT (richest: ttff, per-satellite CN0, constellation split).
# But some genuine ZED-F9P units silently never emit those two messages even
# when CFG-MSGOUT-*-USB is set and the VALSET is ACK'd, while NAV-PVT streams
# fine. NAV-PVT carries fixType, carrSoln, gnssFixOk, diffSoln, numSv and
# h/v accuracy + pDOP — enough to keep both diagnostics alive (minus the
# per-satellite CN0 detail). So when STATUS/SAT are stale we fall back to
# NAV-PVT instead of erroring.

from __future__ import annotations

import collections
import math
from typing import Deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rtcm_msgs.msg import Message as RtcmMessage

try:
    from ublox_ubx_msgs.msg import (
        UBXNavStatus, UBXNavSat, UBXRxmRTCM, UBXNavCov, UBXNavDOP, UBXNavPVT,
    )
    _HAVE_UBX = True
except ImportError:
    _HAVE_UBX = False

# Map RTCM "carrSoln" to a label.
_CARR_SOLN_LABELS = {0: "none", 1: "float", 2: "fixed"}

# UBX-NAV-STATUS gpsFix.fix_type → label (subset we care about).
_FIX_TYPE_LABELS = {
    0: "no-fix",
    1: "dead-reckoning",
    2: "2D",
    3: "3D",
    4: "3D+DR",
    5: "time-only",
}

# Number of RTCM samples to keep for rate / type statistics.
_RTCM_WINDOW_S = 5.0


class GpsHealthAggregator(Node):
    def __init__(self) -> None:
        super().__init__("gps_health_aggregator")

        self.declare_parameter("protocol", "UBX")
        self._protocol: str = str(self.get_parameter("protocol").value).upper()
        self._ubx_enabled = self._protocol == "UBX" and _HAVE_UBX

        self._last_status: UBXNavStatus | None = None
        self._last_status_t: float = 0.0
        self._last_sat: UBXNavSat | None = None
        self._last_sat_t: float = 0.0
        self._last_cov: UBXNavCov | None = None
        self._last_dop: UBXNavDOP | None = None
        # NAV-PVT is the fallback source for fix + satellite-count when a
        # receiver doesn't emit NAV-STATUS / NAV-SAT (see module header).
        self._last_pvt: UBXNavPVT | None = None
        self._last_pvt_t: float = 0.0
        # Each entry: (recv_time, msg_type, msg_used, crc_failed). msg_type and
        # msg_used are -1 in NMEA mode where the receiver does not echo RTCM
        # ingestion telemetry — we only know what we forwarded into it.
        self._rtcm: Deque[tuple[float, int, int, bool]] = collections.deque(
            maxlen=512
        )
        self._rtcm_bytes: int = 0

        # ublox_dgnss publishes UBX topics with default QoS (reliable, depth
        # 10). Match it on our side.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self._ubx_enabled:
            self.create_subscription(UBXNavStatus, "/ubx_nav_status", self._on_status, qos)
            self.create_subscription(UBXNavSat, "/ubx_nav_sat", self._on_sat, qos)
            self.create_subscription(UBXNavCov, "/ubx_nav_cov", self._on_cov, qos)
            self.create_subscription(UBXNavDOP, "/ubx_nav_dop", self._on_dop, qos)
            self.create_subscription(UBXNavPVT, "/ubx_nav_pvt", self._on_pvt, qos)
            self.create_subscription(UBXRxmRTCM, "/ubx_rxm_rtcm", self._on_rtcm_ubx, qos)
        else:
            # NMEA mode (or UBX msgs unavailable): only the NTRIP/RTCM
            # diagnostic is meaningful, sourced from the raw NTRIP topic.
            self.create_subscription(RtcmMessage, "/ntrip_client/rtcm", self._on_rtcm_nmea, 50)

        self._pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            f"gps_health_aggregator running (protocol={self._protocol}, "
            f"ubx_enabled={self._ubx_enabled})"
        )

    # ── Subscribers ──────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_status(self, msg: UBXNavStatus) -> None:
        self._last_status = msg
        self._last_status_t = self._now()

    def _on_sat(self, msg: UBXNavSat) -> None:
        self._last_sat = msg
        self._last_sat_t = self._now()

    def _on_cov(self, msg: UBXNavCov) -> None:
        self._last_cov = msg

    def _on_dop(self, msg: UBXNavDOP) -> None:
        self._last_dop = msg

    def _on_pvt(self, msg: UBXNavPVT) -> None:
        self._last_pvt = msg
        self._last_pvt_t = self._now()

    def _on_rtcm_ubx(self, msg: UBXRxmRTCM) -> None:
        # UBX path: the receiver tells us which RTCM type it ingested and
        # whether the CRC validated. msg_used == 2 means the frame was
        # accepted and applied.
        self._rtcm.append(
            (self._now(), int(msg.msg_type), int(msg.msg_used), bool(msg.crc_failed))
        )

    def _on_rtcm_nmea(self, msg: RtcmMessage) -> None:
        # NMEA path: ntrip_client publishes the raw RTCM3 frame. We only
        # know it was forwarded — the receiver does not report ingestion.
        # msg.message is the raw RTCM3 byte stream; msg type is the 12-bit
        # field starting at bit 24 of the frame (preamble 0xD3 + 6 reserved
        # bits + 10 length bits + 12 type bits).
        self._rtcm_bytes += len(msg.message)
        msg_type = -1
        if len(msg.message) >= 5:
            b3, b4 = msg.message[3], msg.message[4]
            msg_type = ((b3 << 4) | (b4 >> 4)) & 0x0FFF
        self._rtcm.append((self._now(), msg_type, -1, False))

    # ── Publisher ────────────────────────────────────────────────────
    def _publish(self) -> None:
        now = self._now()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        if self._ubx_enabled:
            arr.status.append(self._fix_status(now))
            arr.status.append(self._sat_status(now))
        arr.status.append(self._rtcm_status(now))
        self._pub.publish(arr)

    # ── Diagnostic builders ──────────────────────────────────────────
    def _fix_status(self, now: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = "GPS: fix"
        s.hardware_id = "ublox_f9p"

        status_fresh = (
            self._last_status is not None and now - self._last_status_t <= 5.0
        )
        pvt_fresh = self._last_pvt is not None and now - self._last_pvt_t <= 5.0

        # Prefer NAV-STATUS (it additionally carries ttff); fall back to
        # NAV-PVT, which has the same fix / carrier-solution fields.
        if status_fresh:
            st = self._last_status
            source = "nav-status"
            carr = int(st.carr_soln.status)
            fix_type = int(st.gps_fix.fix_type)
            gps_fix_ok = bool(st.gps_fix_ok)
            diff = bool(st.diff_corr)
            ttff_s = st.ttff / 1000.0
        elif pvt_fresh:
            pvt = self._last_pvt
            source = "nav-pvt"
            carr = int(pvt.carr_soln.status)
            fix_type = int(pvt.gps_fix.fix_type)
            gps_fix_ok = bool(pvt.gnss_fix_ok)
            diff = bool(pvt.diff_soln)
            ttff_s = math.nan
        else:
            s.level = DiagnosticStatus.ERROR
            s.message = "no UBX-NAV-STATUS or UBX-NAV-PVT in 5 s — driver dead?"
            return s

        carr_label = _CARR_SOLN_LABELS.get(carr, f"unknown({carr})")
        fix_label = _FIX_TYPE_LABELS.get(fix_type, str(fix_type))

        # Accuracy: prefer the full covariance (NAV-COV); else use NAV-PVT's
        # scalar h/v accuracy estimates (sigma_xy is unavailable from PVT).
        sigma_xy_mm = -1.0
        horizontal_accuracy_m = math.nan
        vertical_accuracy_m = math.nan
        if self._last_cov is not None:
            cxx = float(self._last_cov.pos_cov_nn)
            cyy = float(self._last_cov.pos_cov_ee)
            czz = float(self._last_cov.pos_cov_dd)
            sigma_xy_mm = math.sqrt(max(0.0, cxx + cyy)) * 1000.0
            horizontal_accuracy_m = math.sqrt(max(0.0, (cxx + cyy) / 2.0))
            vertical_accuracy_m = math.sqrt(max(0.0, czz))
        elif pvt_fresh:
            horizontal_accuracy_m = self._last_pvt.h_acc / 1000.0
            vertical_accuracy_m = self._last_pvt.v_acc / 1000.0

        # DOP: prefer NAV-DOP (separate h/v); else NAV-PVT's single pDOP.
        hdop = math.nan
        vdop = math.nan
        if self._last_dop is not None:
            hdop = float(self._last_dop.h_dop) * 0.01
            vdop = float(self._last_dop.v_dop) * 0.01
        pdop = float(self._last_pvt.p_dop) * 0.01 if pvt_fresh else math.nan

        s.values = [
            KeyValue(key="source", value=source),
            KeyValue(key="carr_soln", value=carr_label),
            KeyValue(key="fix_type", value=fix_label),
            KeyValue(key="gps_fix_ok", value=str(gps_fix_ok)),
            KeyValue(key="diff_corr", value=str(diff)),
            KeyValue(key="ttff_s", value=f"{ttff_s:.1f}" if math.isfinite(ttff_s) else "n/a"),
            KeyValue(key="sigma_xy_mm", value=f"{sigma_xy_mm:.1f}" if sigma_xy_mm >= 0 else "n/a"),
            KeyValue(key="hdop", value=f"{hdop:.2f}" if math.isfinite(hdop) else "n/a"),
            KeyValue(key="vdop", value=f"{vdop:.2f}" if math.isfinite(vdop) else "n/a"),
            KeyValue(key="pdop", value=f"{pdop:.2f}" if math.isfinite(pdop) else "n/a"),
            KeyValue(
                key="horizontal_accuracy_m",
                value=f"{horizontal_accuracy_m:.3f}" if math.isfinite(horizontal_accuracy_m) else "n/a",
            ),
            KeyValue(
                key="vertical_accuracy_m",
                value=f"{vertical_accuracy_m:.3f}" if math.isfinite(vertical_accuracy_m) else "n/a",
            ),
        ]

        if not gps_fix_ok:
            s.level = DiagnosticStatus.ERROR
            s.message = f"no fix ({fix_label})"
        elif carr == 2:
            s.level = DiagnosticStatus.OK
            s.message = "RTK Fixed"
        elif carr == 1:
            s.level = DiagnosticStatus.WARN
            s.message = "RTK Float — converging, not yet validated"
        else:
            s.level = DiagnosticStatus.WARN
            s.message = f"{fix_label} fix, no RTK"
        return s

    def _sat_status(self, now: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = "GPS: satellites"
        s.hardware_id = "ublox_f9p"

        if self._last_sat is None or now - self._last_sat_t > 5.0:
            # No per-satellite NAV-SAT — fall back to NAV-PVT's used count.
            return self._sat_status_from_pvt(now, s)

        sv_info = self._last_sat.sv_info
        visible = len(sv_info)
        used_cnos = [int(x.cno) for x in sv_info if x.flags.sv_used]
        used_n = len(used_cnos)
        mean_cno = (sum(used_cnos) / used_n) if used_n else 0.0
        max_cno = max(used_cnos) if used_cnos else 0
        cno_ge_40 = sum(1 for c in used_cnos if c >= 40)

        # Per-constellation breakdown is useful for diagnosing why a
        # caster's GPS+GLO+GAL+BDS stream isn't all being tracked.
        per_const: dict[int, int] = collections.defaultdict(int)
        for sv in sv_info:
            if sv.flags.sv_used:
                per_const[int(sv.gnss_id)] += 1
        const_names = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "BDS", 5: "QZSS", 6: "GLO"}
        const_str = ", ".join(
            f"{const_names.get(k, str(k))}={v}" for k, v in sorted(per_const.items())
        )

        s.values = [
            KeyValue(key="source", value="nav-sat"),
            KeyValue(key="visible", value=str(visible)),
            KeyValue(key="used", value=str(used_n)),
            KeyValue(key="mean_cno_db_hz", value=f"{mean_cno:.1f}"),
            KeyValue(key="max_cno_db_hz", value=f"{max_cno:.1f}"),
            KeyValue(key="cno_ge_40_count", value=str(cno_ge_40)),
            KeyValue(key="constellations_used", value=const_str),
        ]

        # Thresholds picked for a YardForce-class robot in a residential
        # garden: ≥6 used + mean CN0 ≥ 40 + 4+ strong sats is a sane
        # "RTK-Fixed-capable" envelope. WARN when corrections will
        # struggle to validate ambiguities.
        if used_n < 4:
            s.level = DiagnosticStatus.ERROR
            s.message = f"only {used_n} sats used"
        elif used_n < 6 or mean_cno < 35.0 or cno_ge_40 < 3:
            s.level = DiagnosticStatus.WARN
            s.message = (
                f"{used_n} sats used, mean CN0 {mean_cno:.0f} dB-Hz "
                f"({cno_ge_40} ≥40) — weak for RTK-Fixed"
            )
        else:
            s.level = DiagnosticStatus.OK
            s.message = (
                f"{used_n} sats used, mean CN0 {mean_cno:.0f} dB-Hz, "
                f"{cno_ge_40} ≥40"
            )
        return s

    def _sat_status_from_pvt(self, now: float, s: DiagnosticStatus) -> DiagnosticStatus:
        """Fallback when NAV-SAT isn't emitted: NAV-PVT gives only the count
        of satellites used in the solution — no per-satellite CN0 / sky view."""
        if self._last_pvt is None or now - self._last_pvt_t > 5.0:
            s.level = DiagnosticStatus.ERROR
            s.message = "no UBX-NAV-SAT or UBX-NAV-PVT in 5 s"
            return s

        used_n = int(self._last_pvt.num_sv)
        s.values = [
            KeyValue(key="source", value="nav-pvt"),
            KeyValue(key="used", value=str(used_n)),
            KeyValue(key="note", value="CN0 / sky-view unavailable (NAV-SAT not emitted)"),
        ]

        # Same used-count thresholds as the NAV-SAT path; CN0-based WARNs
        # can't be evaluated without per-satellite data.
        if used_n < 4:
            s.level = DiagnosticStatus.ERROR
            s.message = f"only {used_n} sats used"
        elif used_n < 6:
            s.level = DiagnosticStatus.WARN
            s.message = f"{used_n} sats used — marginal"
        else:
            s.level = DiagnosticStatus.OK
            s.message = f"{used_n} sats used"
        return s

    def _rtcm_status(self, now: float) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = "GPS: NTRIP/RTCM"
        s.hardware_id = "ntrip_client"

        # Drop entries older than the window.
        while self._rtcm and now - self._rtcm[0][0] > _RTCM_WINDOW_S:
            self._rtcm.popleft()

        if not self._rtcm:
            s.level = DiagnosticStatus.ERROR
            s.message = "no RTCM in last 5 s"
            s.values = [
                KeyValue(key="msgs_per_sec", value="0.0"),
                KeyValue(key="age_of_last_corr_s", value="∞"),
            ]
            return s

        n = len(self._rtcm)
        rate = n / _RTCM_WINDOW_S
        types: collections.Counter[int] = collections.Counter(
            t for _, t, _, _ in self._rtcm if t >= 0
        )
        types_str = ", ".join(f"{t}({c})" for t, c in sorted(types.items()))
        last_age = now - self._rtcm[-1][0]

        s.values = [
            KeyValue(key="msgs_per_sec", value=f"{rate:.1f}"),
            KeyValue(key="age_of_last_corr_s", value=f"{last_age:.1f}"),
            KeyValue(key="types_seen", value=types_str),
        ]

        if self._ubx_enabled:
            used_count = sum(1 for _, _, used, _ in self._rtcm if used == 2)
            crc_fail = sum(1 for _, _, _, crc in self._rtcm if crc)
            used_pct = 100.0 * used_count / n if n else 0.0
            s.values.extend([
                KeyValue(key="msgs_used_pct", value=f"{used_pct:.0f}"),
                KeyValue(key="crc_failed_count", value=str(crc_fail)),
            ])

            if last_age > 5.0:
                s.level = DiagnosticStatus.ERROR
                s.message = f"corrections stalled ({last_age:.1f} s old)"
            elif crc_fail >= max(2, n // 4):
                s.level = DiagnosticStatus.WARN
                s.message = f"{crc_fail} CRC failures in {n} msgs"
            elif used_pct < 50.0:
                s.level = DiagnosticStatus.WARN
                s.message = f"only {used_pct:.0f}% of corrections accepted"
            else:
                s.level = DiagnosticStatus.OK
                s.message = (
                    f"{rate:.1f} msg/s, {used_pct:.0f}% used, "
                    f"last {last_age:.1f} s ago"
                )
        else:
            # NMEA mode: we only know what we forwarded into the receiver.
            # The bytes counter and message rate confirm the bridge is
            # alive; downstream correction quality is observable through
            # NavSatStatus (FIX/SBAS/GBAS) on /gps/fix.
            s.values.append(KeyValue(key="bytes_total", value=str(self._rtcm_bytes)))
            if last_age > 5.0:
                s.level = DiagnosticStatus.ERROR
                s.message = f"corrections stalled ({last_age:.1f} s old)"
            else:
                s.level = DiagnosticStatus.OK
                s.message = (
                    f"{rate:.1f} msg/s forwarded to receiver, "
                    f"last {last_age:.1f} s ago"
                )
        return s


def main() -> None:
    rclpy.init()
    node = GpsHealthAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
