"""UM960 GNSS/RTK receiver driver (ROS 2).

Serial reader + NMEA / Unicore ASCII / Unicore binary parsers + ROS publishers.
Kept dependency-free on purpose: the serial port is opened with ``termios`` so no
``pyserial`` install is needed on the target image.
"""

import re
import struct
import time
from typing import Callable, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------------------
# Fix-quality tables
# --------------------------------------------------------------------------------------

# NMEA GGA quality indicator -> (sensor_msgs/NavSatStatus constant name, human label).
GGA_QUALITY = {
    0: ("STATUS_NO_FIX", "INVALID"),
    1: ("STATUS_FIX", "GPS"),
    2: ("STATUS_DGPS_FIX", "DGPS"),
    4: ("STATUS_FIX", "RTK_FIXED"),
    5: ("STATUS_FIX", "RTK_FLOAT"),
    6: ("STATUS_FIX", "DR"),
}

# NMEA GSA fix type (field 2) -> human label, per NMEA 0183 table:
# 0 = invalid, 1 = GPS fix, 2 = differential (DGPS) fix, 3 = PPM/waas fix.
GSA_FIX_TYPE = {0: "NO_FIX", 1: "GPS_FIX", 2: "DGPS_FIX", 3: "PPM_FIX"}

# GSA mode indicator in the older numeric encoding: 2 == differentially corrected.
GSA_MODE_DIFFERENTIAL = 2

# Unicore position types (BESTNAV posType), table 0-4 of the UM960 manual, matching
# the vendor's own enumeration in 03_ros_interfaces/mower_gps_msgs/UnicoreNav.msg.
UNICORE_POS_TYPE = {
    0: "NONE",
    1: "FIXEDPOS",
    2: "FIXEDHEIGHT",
    3: "DOPPLER_VELOCITY",
    4: "SINGLE",
    5: "PSRDIFF",
    6: "SBAS",
    7: "L1_FLOAT",
    8: "IONOFREE_FLOAT",
    9: "NARROW_FLOAT",
    10: "L1_INT",
    11: "WIDE_INT",
    12: "NARROW_INT",
    13: "INS",
    14: "INS_PSRSP",
    15: "INS_PSRDIFF",
    16: "INS_RTKFLOAT",
    17: "INS_RTKFIXED",
    18: "PPP_CONVERGING",
    19: "PPP",
}

# Unicore velocity types, table 0-4.
UNICORE_VEL_TYPE = {
    0: "INVALID",
    1: "CALCULATED",
    2: "CALCULATED_OPT",
    3: "DOPPLER_VELOCITY",
    4: "DOPPLER_VELOCITY_OPT",
}

# Unicore solution status, table 0-5.
UNICORE_SOL_STATUS = {
    0: "SOL_COMPUTED",
    1: "INSUFFICIENT_OBS",
    2: "NO_CONVERGENCE",
    3: "COV_TRACE",
}

# The only status for which a position may be published.
SOL_COMPUTED = 0

# Unicore GPHPR/GNHPR quality flag (same table as GGA quality): 0 invalid, 1 single point,
# 2 pseudorange differential, 4 RTK fixed (integer ambiguities fixed on the antenna
# baseline), 5 RTK float, 6 dead reckoning, 7 manual input, 8 simulation.
HPR_STATUS_FIXED = 4
_HPR_UTC = re.compile(r"^\d{6}(\.\d*)?$")

# Unicore binary message ids we care about.
UNICORE_MSG_VERSION = 0x01
UNICORE_MSG_BESTNAV = 0x0A


def _to_float(value: str) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value: str) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None


def nmea_checksum_ok(sentence: str) -> bool:
    """Validate the XOR checksum of a full NMEA/Unicore ASCII sentence."""
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, _, given = sentence[1:].partition("*")
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    try:
        return checksum == int(given[:2], 16)
    except ValueError:
        return False


def dm_to_degrees(value: str, hemisphere: str) -> Optional[float]:
    """Convert an NMEA ``ddmm.mmmm`` / ``dddmm.mmmm`` field to signed decimal degrees."""
    if not value:
        return None
    raw = _to_float(value)
    if raw is None:
        return None
    degrees = int(raw / 100.0)
    minutes = raw - degrees * 100.0
    if minutes >= 60.0:
        return None
    decimal = degrees + minutes / 60.0
    hemisphere = hemisphere.strip().upper()
    if hemisphere in ("S", "W"):
        decimal = -decimal
    elif hemisphere not in ("N", "E", ""):
        return None
    return decimal


def _is_sane_position(lat: float, lon: float) -> bool:
    """Reject coordinates that cannot be on Earth."""
    return -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0


# User-equivalent range error [m] per NMEA GGA quality, used when the receiver
# does not stream its own per-axis sigmas (BESTNAVA). 1.2 m is the usual
# multi-GNSS single-point figure; DGPS ~0.6; RTK float ~0.25; RTK fixed ~0.02.
# Without this an RTK-fixed solution was reported at 0.45 m and could never
# pass the 4 cm dock-set gate or get a sensible EKF weight.
UERE_BY_QUALITY = {0: 1.2, 1: 1.2, 2: 0.6, 4: 0.02, 5: 0.25, 6: 1.2}


def hdop_to_covariance(hdop: float, sats: int, quality: int = 1) -> List[float]:
    """Rough ENU covariance for NavSatFix from HDOP, satellite count and GGA quality.

    Refined once the Unicore binary/ASCII sigma fields are available (those are
    used in preference).
    """
    if hdop <= 0.0:
        return []
    uere = UERE_BY_QUALITY.get(int(quality), 1.2)
    sigma_h = hdop * uere
    if sats >= 6:
        sigma_h *= 0.9
    sigma_v = sigma_h * 1.5
    return [sigma_h ** 2, 0.0, 0.0, 0.0, sigma_h ** 2, 0.0, 0.0, 0.0, sigma_v ** 2]


def sigma_to_covariance(lat_sigma: float, lon_sigma: float, alt_sigma: float) -> List[float]:
    """Convert receiver-reported per-axis standard deviations to a covariance matrix.

    Only the upper triangle of the ENU covariance is populated, which is what
    NavSatFix consumers (robot_localization) expect.
    """
    cov = [0.0] * 9
    if lat_sigma > 0.0:
        cov[0] = lat_sigma ** 2
    if lon_sigma > 0.0:
        cov[4] = lon_sigma ** 2
    if alt_sigma > 0.0:
        cov[8] = alt_sigma ** 2
    return cov


class NmeaParser:
    """Incremental NMEA 0183 / Unicore ASCII sentence parser."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        # Monotonic clock used to age-stamp samples that must not outlive their sentence
        # (the GPHPR heading); injectable for tests.
        self.clock = clock
        self.reset()

    def reset(self) -> None:
        self.fix: Dict[str, object] = {
            "lat": None,
            "lon": None,
            "altitude": None,
            "quality": 0,
            "num_sats": 0,
            "hdop": 0.0,
            "vdop": 0.0,
            "pdop": 0.0,
            "fix_type": 0,
            "gsa_differential": False,
            "fix_age": None,
            "station_id": "",
            "satellites_in_view": 0,
            "speed_mps": None,
            "course_deg": None,
            "gps_date": None,
            "gps_time": None,
            "gga_time": None,
            "gga_sentence": "",
            "hdop_from_gst": None,
            "position_type": None,
            "solution_status": None,
            "velocity_type": None,
            "lat_sigma": None,
            "lon_sigma": None,
            "alt_sigma": None,
            "num_sats_used": 0,
            "horizontal_speed": None,
            "vertical_speed": None,
            "track_ground": None,
            "version": None,
            "reference_station_status": None,
            "gps_hardware_status": None,
        }

    # -- helpers ---------------------------------------------------------------------
    @staticmethod
    def _fields(body: str) -> List[str]:
        return body.split(',')

    def _apply_quality(self, quality: int) -> str:
        """Map a quality indicator to a NavSatStatus name; returns the label too."""
        name, label = GGA_QUALITY.get(quality, ("STATUS_FIX", "FIX"))
        self.fix["quality"] = quality
        self.fix["quality_label"] = label
        self.fix["status"] = name
        return label

    # -- individual sentences --------------------------------------------------------
    def parse_gga(self, fields: List[str]) -> None:
        if len(fields) < 9:
            return
        lat = dm_to_degrees(fields[2], fields[3] if len(fields) > 3 else "")
        lon = dm_to_degrees(fields[4], fields[5] if len(fields) > 5 else "")
        quality = _to_int(fields[6]) if len(fields) > 6 else None
        num_sats = _to_int(fields[7]) if len(fields) > 7 else None
        hdop = _to_float(fields[8]) if len(fields) > 8 else None
        altitude = _to_float(fields[9]) if len(fields) > 9 else None
        if altitude is not None and (len(fields) <= 10 or fields[10].upper() != "M"):
            altitude = None
        fix_age = _to_float(fields[13]) if len(fields) > 13 else None
        station_id = fields[14] if len(fields) > 14 else ""
        time_field = fields[1] if len(fields) > 1 else ""

        usable = (
            (quality or 0) != 0
            and lat is not None
            and lon is not None
            and _is_sane_position(lat, lon)
        )
        if usable:
            self.fix["lat"] = lat
            self.fix["lon"] = lon
            self.fix["gga_time"] = time_field or None
            self.fix["gga_sentence"] = "GGA"
        else:
            # Quality 0 or impossible coordinates: drop the old position so consumers
            # stop seeing a stale fix instead of a repeating dead one.
            self.fix["lat"] = None
            self.fix["lon"] = None
        if altitude is not None:
            self.fix["altitude"] = altitude
        if quality is not None:
            self._apply_quality(quality)
        if num_sats:
            self.fix["num_sats"] = num_sats
        if hdop:
            self.fix["hdop"] = hdop
        if fix_age is not None:
            self.fix["fix_age"] = fix_age
        if station_id:
            self.fix["station_id"] = station_id

    def parse_rmc(self, fields: List[str]) -> None:
        if len(fields) < 10:
            return
        status = fields[2].strip().upper()
        speed_knots = _to_float(fields[7]) if len(fields) > 7 else None
        course = _to_float(fields[8]) if len(fields) > 8 else None
        lat = dm_to_degrees(fields[3], fields[4] if len(fields) > 4 else "")
        lon = dm_to_degrees(fields[5], fields[6] if len(fields) > 6 else "")
        if status == "A" and lat is not None and lon is not None:
            # Only fill in the position if nothing better (GGA / BESTNAV) has it yet,
            # and only if the coordinates are actually possible.
            if self.fix["lat"] is None and _is_sane_position(lat, lon):
                self.fix["lat"] = lat
                self.fix["lon"] = lon
            self.fix["gga_time"] = self.fix["gga_time"] or fields[1]
            self.fix["gga_sentence"] = "RMC"
        elif status == "V":
            # RMC explicitly marks the data invalid.
            self.fix["lat"] = None
            self.fix["lon"] = None
        self.fix["gps_time"] = fields[1] or None
        self.fix["gps_date"] = fields[9] if len(fields) > 9 else None
        if speed_knots is not None:
            self.fix["speed_mps"] = speed_knots * 0.514444
        if course is not None:
            self.fix["course_deg"] = course

    def parse_vtg(self, fields: List[str]) -> None:
        # $GNVTG,course_true,T,course_mag,M,speed_knots,N,speed_kmh,K,mode*cs
        if len(fields) < 8:
            return
        course = _to_float(fields[1])
        speed_kmh = _to_float(fields[7])
        if course is not None:
            self.fix["course_deg"] = course
        if speed_kmh is not None:
            self.fix["speed_mps"] = speed_kmh / 3.6

    def parse_gsa(self, fields: List[str]) -> None:
        # $--GSA,<mode>,<fix type>,<sv 1..12>,<pdop>,<hdop>,<vdop>,<system id>
        if len(fields) < 3:
            return
        mode = fields[1].strip().upper()
        fix_type = _to_int(fields[2])
        if fix_type is not None:
            self.fix["fix_type"] = fix_type
        self.fix["gsa_mode"] = mode
        # NMEA 2.3+ uses A (autonomous 3D) / M (manual); older receivers use the
        # numeric 1/2 (2D/3D) encoding where bit 1 means differentially corrected.
        self.fix["gsa_differential"] = mode == "D" or _to_int(mode) == GSA_MODE_DIFFERENTIAL
        sv_fields = fields[3:15]
        pdop = _to_float(fields[15]) if len(fields) > 15 else None
        hdop = _to_float(fields[16]) if len(fields) > 16 else None
        vdop = _to_float(fields[17]) if len(fields) > 17 else None
        if pdop:
            self.fix["pdop"] = pdop
        if hdop:
            self.fix["hdop"] = hdop
        if vdop:
            self.fix["vdop"] = vdop
        used = [f for f in sv_fields if f.strip()]
        self.fix["num_sats_used"] = len(used)
        if used:
            self.fix["num_sats"] = max(self.fix["num_sats"] or 0, len(used))

    def parse_gsv(self, fields: List[str]) -> None:
        # $GPGSV,total_msgs,msg_num,num_sats,[prn,elev,azim,snr]*4
        if len(fields) >= 4:
            in_view = _to_int(fields[3])
            if in_view is not None:
                self.fix["satellites_in_view"] = in_view
            self.fix["snr_max"] = max(
                [int(f) for f in fields[7::4] if f.strip().isdigit()] or [0]
            )

    def parse_gst(self, fields: List[str]) -> None:
        if len(fields) < 7:
            return
        hdop = _to_float(fields[5]) if len(fields) > 5 else None
        if hdop:
            self.fix["hdop_from_gst"] = hdop

    def parse_gphpr(self, fields: List[str]) -> None:
        """Unicore heading/pitch/roll log.

        Unicore layout: ``$GNHPR,<utc hhmmss.ss>,<heading>,<pitch>,<roll>,<QF>,<sats>,<age>,
        <station>``; QF uses the GGA quality table (4 = baseline RTK fixed). The older
        4-field form ``$GPHPR,<heading>,<status>,<pitch>,<roll>`` is still accepted. The
        sample is age-stamped (``heading_mono``) so the node can drop it when no new HPR
        arrives; a status-0 (invalid) sentence clears the heading instead of storing it.
        """
        if len(fields) < 2:
            return
        if len(fields) >= 6 or _HPR_UTC.match(fields[1].strip()):
            heading = _to_float(fields[2]) if len(fields) > 2 else None
            status = _to_int(fields[5]) if len(fields) > 5 else None
        else:
            heading = _to_float(fields[1])
            status = _to_int(fields[2]) if len(fields) > 2 else None
        self.fix["heading_status"] = status
        if heading is None or status == 0:
            self.fix.pop("heading_deg", None)
            self.fix.pop("heading_mono", None)
            return
        self.fix["heading_deg"] = heading
        self.fix["heading_mono"] = self.clock()

    def expire_heading(self, max_age_s: float) -> None:
        """Forget a GPHPR heading older than ``max_age_s`` (the receiver stopped sending)."""
        stamp = self.fix.get("heading_mono")
        if stamp is not None and self.clock() - float(stamp) > max_age_s:
            self.fix.pop("heading_deg", None)
            self.fix.pop("heading_mono", None)

    def heading_is_fixed(self, max_age_s: float) -> bool:
        """True when the latest GPHPR is fresh and reports a fixed (integer) RTK baseline."""
        stamp = self.fix.get("heading_mono")
        return (
            self.fix.get("heading_deg") is not None
            and stamp is not None
            and self.clock() - float(stamp) <= max_age_s
            and self.fix.get("heading_status") == HPR_STATUS_FIXED
        )

    # -- Unicore ASCII logs ----------------------------------------------------------
    def parse_bestnav(self, fields: List[str]) -> None:
        """Parse Unicore ASCII BESTNAV/BESTNAVA (the vendor's primary fusion log).

        Field order per the UM960 protocol manual, 0-indexed from the message id
        (already stripped by feed()):
        0 time, 1 week, 2 ms, 3 solStatus, 4 posType, 5 lat, 6 lon, 7 hgt,
        8 latStd, 9 lonStd, 10 hgtStd, 11 diffAge, 12 solAge, 13 #SV, 14 #solnSV,
        15-17 reserved, 18 undulation, 19 datumId, 20 extSolStatus,
        21 galileoBds3SigMask, 22 gpsGlonassBds2SigMask, 23 velStatus, 24 velType,
        25 age, 26 horSpeed, 27 latency, 28 vertSpeed, 29 trackGround,
        30 verSpdStd, 31 horSpdStd.
        """
        def get(index: int) -> Optional[str]:
            return fields[index] if index < len(fields) else None

        def num(index: int) -> Optional[float]:
            return _to_float(get(index))

        def integer(index: int) -> Optional[int]:
            return _to_int(get(index))

        sol_status = integer(3)
        pos_type = integer(4)
        lat = num(5)
        lon = num(6)
        computed = sol_status is None or sol_status == SOL_COMPUTED
        if computed and lat is not None and lon is not None and _is_sane_position(lat, lon):
            self.fix["lat"] = lat
            self.fix["lon"] = lon
            self.fix["gga_sentence"] = "BESTNAV"
            self.fix["gga_time"] = get(0)
        elif not computed:
            # The receiver says the solution is not valid: drop the old position.
            self.fix["lat"] = None
            self.fix["lon"] = None
        altitude = num(7)
        if altitude is not None:
            self.fix["altitude"] = altitude
        self.fix["lat_sigma"] = num(8)
        self.fix["lon_sigma"] = num(9)
        self.fix["alt_sigma"] = num(10)
        self.fix["fix_age"] = num(11)
        num_sats = integer(13)
        if num_sats:
            self.fix["num_sats"] = num_sats
        self.fix["num_sats_used"] = integer(14) or 0
        if pos_type is not None:
            self.fix["position_type"] = UNICORE_POS_TYPE.get(pos_type, "UNKNOWN(%d)" % pos_type)
        if sol_status is not None:
            self.fix["solution_status"] = UNICORE_SOL_STATUS.get(
                sol_status, "UNKNOWN(%d)" % sol_status
            )
        vel_type = integer(24)
        if vel_type is not None:
            self.fix["velocity_type"] = UNICORE_VEL_TYPE.get(vel_type, "UNKNOWN(%d)" % vel_type)
        speed = num(26)
        if speed is not None:
            self.fix["horizontal_speed"] = speed
        vertical = num(28)
        if vertical is not None:
            self.fix["vertical_speed"] = vertical
        track = num(29)
        if track is not None:
            self.fix["track_ground"] = track

    def parse_gnver(self, fields: List[str]) -> None:
        """$GNVER,<receiver>,<hw>,<fw>,... - strip the trailing *cs from each field."""
        self.fix["version"] = ",".join(f.split("*")[0] for f in fields if f) or None

    def parse_gnsta(self, fields: List[str]) -> None:
        """$GNSTA,<mode>,<ref_station_status>,... e.g. FINE / FAIL."""
        for field in fields:
            token = field.split("*")[0].strip().upper()
            if token in ("FINE", "FAIL"):
                self.fix["reference_station_status"] = token
                return

    def parse_gnref(self, fields: List[str]) -> None:
        """$GNREF,<stn_id>,<stn_name>,<lat>,<lon>,... reference station description."""
        station = fields[1] if len(fields) > 1 else None
        if station:
            self.fix["station_id"] = station

    def parse_ghardm(self, fields: List[str]) -> None:
        """GHARDM - hardware status log (antenna, self-test, supply voltage)."""
        self.fix["gps_hardware_status"] = ",".join(f.split("*")[0] for f in fields if f) or None

    # -- dispatch --------------------------------------------------------------------
    def feed(self, sentence: str) -> Optional[str]:
        """Feed one ASCII sentence. Returns the message id if it was recognised."""
        sentence = sentence.strip()
        if not sentence.startswith("$"):
            return None
        if not nmea_checksum_ok(sentence):
            return None
        body = sentence[1:].partition("*")[0]
        fields = self._fields(body)
        talker = fields[0]
        kind = talker[2:] if len(talker) > 2 else ""

        if kind == "GGA":
            self.parse_gga(fields)
        elif kind == "RMC":
            self.parse_rmc(fields)
        elif kind == "VTG":
            self.parse_vtg(fields)
        elif kind == "GSA":
            self.parse_gsa(fields)
        elif kind == "GSV":
            self.parse_gsv(fields)
        elif kind == "GST":
            self.parse_gst(fields)
        elif kind == "HPR":
            self.parse_gphpr(fields)
        # Unicore ASCII logs use the whole message id as the "talker" field, so
        # they are dispatched on fields[0] rather than on the NMEA suffix.
        elif talker in ("BESTNAV", "BESTNAVA"):
            self.parse_bestnav(fields[1:])
        elif talker == "GNVER":
            self.parse_gnver(fields[1:])
        elif talker == "GNSTA":
            self.parse_gnsta(fields[1:])
        elif talker == "GNREF":
            self.parse_gnref(fields[1:])
        elif talker in ("GHARDM", "HWSTATUSA", "HWSTATUS"):
            self.parse_ghardm(fields[1:])
        else:
            return None
        return talker

    @property
    def has_fix(self) -> bool:
        return self.fix["lat"] is not None and self.fix["lon"] is not None


def crc16_xmodem(payload: bytes) -> int:
    """CRC-16/XMODEM over the binary message id + length + payload."""
    crc = 0
    for byte in payload:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class UnicoreBinaryParser:
    """Parser for the Unicore binary synch framing.

    Wire format: ``AA 44 B5 <id> <len LSB> <len MSB> <payload> <crc16 LSB> <crc16 MSB>``
    where the CRC-16/XMODEM covers the message id, the length field and the payload.

    Only BESTNAV (0x0A) is fully decoded; other message ids are counted so the
    operator can see what the receiver is actually streaming.
    """

    SYNC = b"\xaa\x44\xb5"
    HEADER_SIZE = 6  # 3 sync bytes + message id + 2-byte length
    # The last BESTNAV field decoded here (horizontal speed sigma) ends at H+120.
    # Lengths of 120 and 124 are both seen in the field, so accept anything that
    # covers the decoded offsets and ignore the rest.
    BESTNAV_PAYLOAD_LENGTH = 120

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.message_counts: Dict[int, int] = {}
        self.bestnav: Optional[Dict[str, object]] = None

    def reset(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes) -> List[Dict[str, object]]:
        """Append bytes to the internal buffer and decode every complete message."""
        self.buffer.extend(data)
        return self.consume_stream(self.buffer)

    def consume_stream(self, stream: bytearray) -> List[Dict[str, object]]:
        """Decode complete frames from the front of ``stream``, in arrival order.

        Consumed bytes are removed in place. Bytes that are not part of a complete
        frame are left behind (only a leading run of garbage is dropped), so a caller
        can share one buffer between the binary parser and an ASCII line splitter
        and still preserve the order in which things arrived on the wire.
        """
        results: List[Dict[str, object]] = []
        while True:
            start = stream.find(self.SYNC)
            if start < 0:
                # Leave the buffer alone: it may be pure ASCII that the caller is
                # splitting itself, or the first bytes of a triplet split across two
                # reads. Bounding the buffer is the caller's job.
                return results
            if start > 0:
                del stream[:start]
            if len(stream) < self.HEADER_SIZE:
                return results
            length = stream[4] | (stream[5] << 8)
            end = self.HEADER_SIZE + length + 2  # + CRC16
            if len(stream) < end:
                return results
            frame = self._pop_frame(stream, length, end)
            decoded = self._register(frame)
            if decoded is not None:
                results.append(decoded)

    def _pop_frame(self, stream: bytearray, length: int, end: int) -> Tuple[int, bytes]:
        """Validate and remove one frame at the front of ``stream``."""
        message_id = stream[3]
        # The CRC covers the message id, the length field and the payload.
        body = bytes(stream[3 : 6 + length])
        crc_received = stream[6 + length] | (stream[7 + length] << 8)
        del stream[:end]
        # Note: a CR LF may follow a binary frame on some Unicore configurations.
        # It is deliberately left in the buffer, because with NMEA logs interleaved
        # on the same port that CR LF could equally be the start of the next ASCII
        # sentence. A stray empty line is harmless; swallowing a sentence is not.
        if crc16_xmodem(body) != crc_received:
            # Drop the frame but keep going: the length field is still trustworthy
            # enough to resynchronise on the next triplet.
            return message_id, b""
        # body still carries the 3-byte id+length prefix; the payload starts after it.
        return message_id, body[3:]

    def _register(self, frame: Tuple[int, bytes]) -> Optional[Dict[str, object]]:
        message_id, payload = frame
        self.message_counts[message_id] = self.message_counts.get(message_id, 0) + 1
        if not payload:
            return None
        if message_id == UNICORE_MSG_BESTNAV and len(payload) >= self.BESTNAV_PAYLOAD_LENGTH:
            decoded = self._decode_bestnav(payload)
            if decoded is not None:
                self.bestnav = decoded
                return decoded
        elif message_id == UNICORE_MSG_VERSION and len(payload) >= 28:
            return {"kind": "version", "version": self._decode_version(payload)}
        return None

    def _decode_version(self, payload: bytes) -> str:
        """ASCII version log (0x01) - fields are printable characters."""
        text = payload.decode("ascii", errors="replace").replace("\x00", "").strip()
        return ",".join(part for part in text.split(",") if part)

    def _decode_bestnav(self, payload: bytes) -> Optional[Dict[str, object]]:
        """Decode a binary BESTNAV payload. Offsets are from the UM960 manual."""
        try:
            sol_status, pos_type = struct.unpack_from("<II", payload, 0)
            lat = struct.unpack_from("<q", payload, 8)[0] * 1e-8
            lon = struct.unpack_from("<q", payload, 16)[0] * 1e-8
            height = struct.unpack_from("<i", payload, 24)[0] * 1e-4
            lat_std, lon_std, height_std = struct.unpack_from("<fff", payload, 28)
            station_id = payload[40:44].split(b"\x00")[0].decode("ascii", errors="replace")
            diff_age, sol_age = struct.unpack_from("<ff", payload, 44)
            num_svs = payload[52]
            num_svs_used = payload[53]
            undulation = struct.unpack_from("<f", payload, 56)[0]
            datum_id = struct.unpack_from("<I", payload, 60)[0]
            vel_status, vel_type = struct.unpack_from("<II", payload, 68)
            age = struct.unpack_from("<f", payload, 76)[0]
            latency = struct.unpack_from("<f", payload, 80)[0]
            hor_speed = struct.unpack_from("<d", payload, 88)[0]
            track_ground = struct.unpack_from("<d", payload, 96)[0]
            vert_speed = struct.unpack_from("<d", payload, 104)[0]
            ver_speed_std, hor_speed_std = struct.unpack_from("<ff", payload, 112)
        except struct.error:
            return None
        if not _is_sane_position(lat, lon):
            return None
        return {
            "kind": "bestnav",
            "latitude": lat,
            "longitude": lon,
            "height": height,
            "lat_sigma": lat_std,
            "lon_sigma": lon_std,
            "height_sigma": height_std,
            "station_id": station_id,
            "diff_age": diff_age,
            "solution_age": sol_age,
            "undulation": undulation,
            "datum_id": datum_id,
            "num_sats": num_svs,
            "num_sats_used": num_svs_used,
            "solution_status": SOL_COMPUTED if sol_status is None else sol_status,
            "position_type": pos_type,
            "velocity_status": vel_status,
            "velocity_type": vel_type,
            "age": age,
            "latency": latency,
            "horizontal_speed": hor_speed,
            "vertical_speed": vert_speed,
            "track_ground": track_ground,
            "vertical_speed_sigma": ver_speed_std,
            "horizontal_speed_sigma": hor_speed_std,
        }


class Um960StreamSplitter:
    """Splits a mixed Unicore byte stream into ASCII lines and binary messages.

    The vendor configuration streams NMEA/Unicore ASCII logs and binary BESTNAV
    frames on the same port, so the two framings interleave on the wire. Both
    parsers share one buffer here and are advanced in arrival order, which keeps
    the freshest solution winning when they disagree. Partial data is retained
    across ``feed`` calls, so chunk boundaries do not matter.
    """

    MAX_BUFFER = 8192

    def __init__(self, binary: Optional[UnicoreBinaryParser] = None) -> None:
        self.binary = binary or UnicoreBinaryParser()
        self.buffer = bytearray()

    def reset(self) -> None:
        self.buffer.clear()
        self.binary.reset()

    def feed(self, data: bytes) -> List[Tuple[str, object]]:
        """Return ``[("line", str) | ("frame", dict), ...]`` in arrival order."""
        self.buffer.extend(data)
        events: List[Tuple[str, object]] = []
        while True:
            sync_pos = self.buffer.find(self.binary.SYNC)
            newline_pos = self.buffer.find(b"\n")
            if sync_pos < 0 and newline_pos < 0:
                break
            if sync_pos >= 0 and (newline_pos < 0 or sync_pos < newline_pos):
                # A binary frame comes next; consume_stream() stops as soon as it
                # needs more bytes, leaving the remainder in the buffer.
                before = len(self.buffer)
                frames = self.binary.consume_stream(self.buffer)
                if not frames and len(self.buffer) == before:
                    break  # frame header seen but the rest has not arrived yet
                for frame in frames:
                    events.append(("frame", frame))
                continue
            raw = bytes(self.buffer[:newline_pos])
            del self.buffer[: newline_pos + 1]
            line = raw.decode("ascii", errors="replace").strip().lstrip("\x00")
            if line:
                events.append(("line", line))
        # Guard against an unbounded buffer if the receiver emits neither framing.
        if len(self.buffer) > self.MAX_BUFFER:
            del self.buffer[: -len(self.binary.SYNC)]
        return events


def bestnav_position_type_name(pos_type: int) -> str:
    return UNICORE_POS_TYPE.get(pos_type, "UNKNOWN(%d)" % pos_type)


def bestnav_solution_status_name(sol_status: int) -> str:
    return UNICORE_SOL_STATUS.get(sol_status, "UNKNOWN(%d)" % sol_status)


def bestnav_velocity_type_name(vel_type: int) -> str:
    return UNICORE_VEL_TYPE.get(vel_type, "UNKNOWN(%d)" % vel_type)


__all__ = [
    "NmeaParser",
    "UnicoreBinaryParser",
    "Um960StreamSplitter",
    "crc16_xmodem",
    "dm_to_degrees",
    "hdop_to_covariance",
    "sigma_to_covariance",
    "nmea_checksum_ok",
    "bestnav_position_type_name",
    "bestnav_solution_status_name",
    "bestnav_velocity_type_name",
    "GGA_QUALITY",
    "GSA_FIX_TYPE",
]
