"""End-to-end tests: push a synthetic UM960 stream through the serial port and
check what the node publishes.

These exercise the whole path - termios open, reader thread, mixed ASCII/binary
stream splitter, parsers and message construction - against a pty pair, so no
hardware is needed. They run with real ``rclpy`` message types on a ROS 2 Jazzy
machine and with the shim from ``conftest.py`` elsewhere.
"""

import os
import pty
import struct
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from sensor_msgs.msg import NavSatFix, NavSatStatus  # noqa: E402
from std_msgs.msg import Bool, Float32, String  # noqa: E402

from um960_gps_driver.parsers import SOL_COMPUTED, crc16_xmodem  # noqa: E402
from um960_gps_driver.um960_node import Um960Node  # noqa: E402

BESTNAV_ID = 0x0A
PUBLISH_TIMEOUT_S = 3.0


def _cs(body: str) -> bytes:
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return ("$%s*%02X\r\n" % (body, checksum)).encode()


def _bestnav_frame(
    pos_type=12,
    lat=48.1173,
    lon=11.5166666,
    speed=1.234,
    track=87.6,
    solution_status=SOL_COMPUTED,
):
    """A 124-byte Unicore binary BESTNAV payload inside its synch framing."""
    payload = bytearray(124)
    struct.pack_into("<II", payload, 0, solution_status, pos_type)
    struct.pack_into("<q", payload, 8, int(lat * 1e8))
    struct.pack_into("<q", payload, 16, int(lon * 1e8))
    struct.pack_into("<i", payload, 24, int(545.4 * 1e4))
    struct.pack_into("<fff", payload, 28, 0.005, 0.007, 0.012)
    payload[40:44] = b"BAS1"
    struct.pack_into("<ff", payload, 44, 0.5, 0.1)
    payload[52] = 21
    payload[53] = 18
    struct.pack_into("<II", payload, 68, 0, 3)  # velStatus, velType = doppler
    struct.pack_into("<d", payload, 88, speed)
    struct.pack_into("<d", payload, 96, track)
    struct.pack_into("<d", payload, 104, -0.015)
    body = bytes([BESTNAV_ID]) + struct.pack("<H", len(payload)) + bytes(payload)
    return b"\xaa\x44\xb5" + body + struct.pack("<H", crc16_xmodem(body))


GGA_RTK = _cs("GNGGA,101530.00,4807.038,N,01131.000,E,4,21,0.6,545.4,M,46.9,M,0.5,0451")
GGA_FLOAT = _cs("GNGGA,101530.00,4807.038,N,01131.000,E,5,18,0.8,545.4,M,46.9,M,1.5,0451")
GGA_NO_FIX = _cs("GNGGA,101530.00,,,,,0,00,99.9,,,,,,")
GGA_INSANE = _cs("GNGGA,101530.00,9130.000,N,18131.000,E,4,21,0.6,545.4,M,46.9,M,,")
GGA_BAD_CS = b"$GNGGA,101530.00,4807.038,N,01131.000,E,4,21,0.6,545.4,M,46.9,M,,*00\r\n"
RMC = _cs("GNRMC,101530.00,A,4807.038,N,01131.000,E,2.4,87.6,051026,,,A")
VTG = _cs("GNVTG,87.6,T,90.0,M,2.4,N,4.4,K,A")
GSA = _cs("GNGSA,A,3,01,03,05,11,17,,,,,,,,2.10,1.05,2.33")
GSV = _cs("GPGSV,3,1,21,01,40,083,44,03,17,313,40")
GNSTA = _cs("GNSTA,3,FINE")
GNVER = _cs("GNVER,UM960,HN1.10,VER1.20")


class _Recorder:
    """Wraps a publisher so tests can inspect what was published.

    An optional publisher (``/nmea`` is only created when enabled) is tolerated so
    tests can assert on an empty topic without special-casing.
    """

    def __init__(self, publisher=None):
        self._publisher = publisher
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)
        if self._publisher is not None:
            self._publisher.publish(msg)


class Harness:
    """A pty pair wired to a live Um960Node, with every topic recorded."""

    TOPICS = ("/fix", "/vel", "/heading", "/heading_valid", "/fix_status", "/nmea")

    def __init__(self, publish_nmea=False, device=None, extra_params=None):
        self.master, slave = pty.openpty()
        self.device = device or os.ttyname(slave)
        os.close(slave)
        params = {
            "port": self.device,
            "baud": 115200,
            "publish_nmea": publish_nmea,
            "publish_rate_hz": 50.0,
            "fix_timeout_s": 0.5,
            # Never pick up a real /userdata/mower/ntrip.yaml from the test host.
            "ntrip_config_file": "",
            "ntrip_settings_file": "",
        }
        params.update(extra_params or {})
        self.node = Um960Node(
            parameter_overrides=[
                rclpy.parameter.Parameter(name, value=value) for name, value in params.items()
            ]
        )
        # The reader thread opens the port asynchronously; nothing written to the
        # pty before then would be missed. A deliberately missing device is the
        # one case where that never happens.
        if device is None:
            self._await(lambda: self.node._connected, 3.0)
        self._absent = {}
        for topic in self.TOPICS:
            attribute = topic[1:] + "_pub"
            publisher = getattr(self.node, attribute)
            if publisher is not None:
                setattr(self.node, attribute, _Recorder(publisher))
            else:
                # /nmea is only created when publish_nmea is true; keep a recorder
                # that records nothing so tests can assert the topic stayed silent.
                self._absent[topic] = _Recorder()

    def recorder(self, topic):
        return self._absent.get(topic) or getattr(self.node, topic[1:] + "_pub")

    def send(self, data: bytes, timeout: float = 3.0):
        """Write to the receiver end of the pty and wait for the reader thread.

        Waiting on the node's byte counter instead of a fixed sleep keeps the tests
        fast and free of timing flakes.
        """
        before = self.node._bytes_read
        os.write(self.master, data)
        self._await(lambda: self.node._bytes_read >= before + len(data), timeout)

    @staticmethod
    def _await(predicate, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        raise AssertionError("timed out waiting for the reader thread")

    def send_slowly(self, data: bytes, per_byte_delay: float = 0.002):
        """Write one byte at a time to force awkward read boundaries."""
        before = self.node._bytes_read
        for byte in data:
            os.write(self.master, bytes([byte]))
            time.sleep(per_byte_delay)
        self._await(lambda: self.node._bytes_read >= before + len(data))

    def tick(self):
        """Run one publish cycle (the real node does this from a ROS timer)."""
        self.node._last_publish = 0.0  # bypass the rate limiter
        self.node._publish_tick()

    def close(self):
        self.node.destroy_node()
        if self.master is not None:
            os.close(self.master)
            self.master = None


@pytest.fixture
def harness():
    h = Harness()
    yield h
    h.close()


@pytest.fixture
def nmea_harness():
    h = Harness(publish_nmea=True)
    yield h
    h.close()


def test_gga_produces_a_fix(harness):
    harness.send(GGA_RTK)
    harness.tick()
    fix = harness.recorder("/fix").msgs[-1]
    assert abs(fix.latitude - 48.1173) < 1e-4
    assert abs(fix.longitude - 11.5166666) < 1e-5
    assert abs(fix.altitude - 545.4) < 1e-6
    assert fix.status.status == NavSatStatus.STATUS_GBAS_FIX  # GGA quality 4 (RTK fixed)
    assert fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert fix.position_covariance[0] > 0.0
    assert fix.header.frame_id == "gps"
    assert "quality=RTK_FIXED" in harness.recorder("/fix_status").msgs[-1].data


def test_binary_bestnav_wins_and_reports_the_solution_type(harness):
    harness.send(GGA_FLOAT + _bestnav_frame(pos_type=12, lat=48.2, lon=11.6))
    harness.tick()
    assert abs(harness.recorder("/fix").msgs[-1].latitude - 48.2) < 1e-7
    status = harness.recorder("/fix_status").msgs[-1].data
    assert "solution=NARROW_INT" in status
    assert "sol_status=SOL_COMPUTED" in status
    assert "station=BAS1" in status


def test_ascii_and_binary_interleave_on_one_port(harness):
    harness.send(RMC + _bestnav_frame() + VTG + GGA_RTK + GSA + GSV)
    harness.tick()
    fix = harness.recorder("/fix").msgs[-1]
    assert abs(fix.latitude - 48.1173) < 1e-4
    assert harness.recorder("/heading").msgs[-1].data == pytest.approx(87.6, abs=1e-2)
    assert "sats=" in harness.recorder("/fix_status").msgs[-1].data


def test_stream_survives_arbitrary_read_boundaries(harness):
    payload = bytearray(124)
    struct.pack_into("<II", payload, 0, 0, 12)
    struct.pack_into("<q", payload, 8, int(48.0 * 1e8))
    struct.pack_into("<q", payload, 16, int(11.0 * 1e8))
    body = bytes([BESTNAV_ID]) + struct.pack("<H", len(payload)) + bytes(payload)
    frame = b"\xaa\x44\xb5" + body + struct.pack("<H", crc16_xmodem(body))
    harness.send_slowly(RMC + frame + VTG)
    harness.tick()
    fix = harness.recorder("/fix").msgs[-1]
    assert abs(fix.latitude - 48.0) < 1e-6
    assert abs(fix.longitude - 11.0) < 1e-6


def test_velocity_and_heading_from_bestnav(harness):
    harness.send(_bestnav_frame(speed=1.234, track=87.6))
    harness.tick()
    assert harness.recorder("/vel").msgs[-1].twist.linear.x == pytest.approx(1.234, abs=1e-6)
    assert harness.recorder("/heading").msgs[-1].data == pytest.approx(87.6, abs=1e-6)


def test_velocity_sign_follows_the_track_angle(harness):
    harness.send(_bestnav_frame(speed=1.0, track=270.0))
    harness.tick()
    assert harness.recorder("/vel").msgs[-1].twist.linear.x == pytest.approx(-1.0, abs=1e-6)


def test_heading_is_normalised_to_zero_360(harness):
    harness.send(_bestnav_frame(track=-30.0))
    harness.tick()
    heading = harness.recorder("/heading").msgs[-1].data
    assert 0.0 <= heading < 360.0
    assert heading == pytest.approx(330.0, abs=1e-3)


HPR_FIXED = _cs("GNHPR,071208.00,275.34,1.20,-0.80,4,20,0.5,0000")
HPR_SINGLE = _cs("GNHPR,071208.00,275.34,1.20,-0.80,1,20,0.5,0000")


def test_gphpr_heading_wins_but_is_not_valid_by_default(harness):
    # GPHPR arrives after the BESTNAV track angle and takes precedence on /heading, but the
    # UM960 is single-antenna: /heading_valid stays false unless publish_rtk_heading is set.
    assert harness.node.publish_rtk_heading is False
    harness.send(_bestnav_frame(track=87.6) + HPR_FIXED)
    harness.tick()
    assert harness.recorder("/heading").msgs[-1].data == pytest.approx(275.34, abs=1e-3)
    assert harness.recorder("/heading_valid").msgs[-1].data is False


@pytest.fixture
def rtk_harness():
    h = Harness(extra_params={"publish_rtk_heading": True})
    yield h
    h.close()


def test_heading_valid_needs_a_fixed_baseline_status(rtk_harness):
    rtk_harness.send(_bestnav_frame(track=87.6) + HPR_SINGLE)
    rtk_harness.tick()
    assert rtk_harness.recorder("/heading_valid").msgs[-1].data is False
    rtk_harness.send(_bestnav_frame(track=87.6) + HPR_FIXED)
    rtk_harness.tick()
    assert rtk_harness.recorder("/heading_valid").msgs[-1].data is True


def test_heading_valid_status_zero_falls_back_to_track(rtk_harness):
    rtk_harness.send(_bestnav_frame(track=87.6) + HPR_FIXED
                     + _cs("GNHPR,071209.00,,,,0,0,,"))
    rtk_harness.tick()
    assert rtk_harness.recorder("/heading_valid").msgs[-1].data is False
    assert rtk_harness.recorder("/heading").msgs[-1].data == pytest.approx(87.6, abs=1e-3)


def test_one_gphpr_does_not_stay_valid_forever(rtk_harness):
    rtk_harness.send(_bestnav_frame(track=87.6) + HPR_FIXED)
    rtk_harness.tick()
    assert rtk_harness.recorder("/heading_valid").msgs[-1].data is True
    # no further GPHPR: after heading_timeout_s the sample is dropped, not re-published
    rtk_harness.node.nmea.fix["heading_mono"] -= 1.5
    rtk_harness.send(_bestnav_frame(track=87.6))
    rtk_harness.tick()
    assert rtk_harness.recorder("/heading_valid").msgs[-1].data is False
    assert rtk_harness.recorder("/heading").msgs[-1].data == pytest.approx(87.6, abs=1e-3)
    assert "heading_deg" not in rtk_harness.node.nmea.fix


def test_track_fallback_heading_is_not_a_dual_antenna_heading(harness):
    harness.send(_bestnav_frame(track=87.6))
    harness.tick()
    assert harness.recorder("/heading").msgs[-1].data == pytest.approx(87.6, abs=1e-6)
    assert harness.recorder("/heading_valid").msgs[-1].data is False


def test_velocity_falls_back_to_nmea(harness):
    harness.send(RMC + GGA_RTK)
    harness.tick()
    assert harness.recorder("/vel").msgs[-1].twist.linear.x == pytest.approx(
        2.4 * 0.514444, abs=1e-4
    )


def test_no_fix_publishes_status_but_not_a_fix(harness):
    harness.send(GGA_NO_FIX)
    harness.tick()
    assert not harness.recorder("/fix").msgs
    assert "quality=INVALID" in harness.recorder("/fix_status").msgs[-1].data


def test_insane_coordinates_are_not_published(harness):
    harness.send(GGA_INSANE)
    harness.tick()
    assert not harness.recorder("/fix").msgs


def test_invalid_checksum_is_ignored(harness):
    harness.send(GGA_BAD_CS)
    harness.tick()
    assert not harness.recorder("/fix").msgs


def test_unicore_proprietary_logs_reach_the_status(harness):
    harness.send(GNSTA + GNVER + GGA_RTK)
    harness.tick()
    assert "ref=FINE" in harness.recorder("/fix_status").msgs[-1].data


def test_stale_fix_stops_the_fix_but_keeps_the_status(harness):
    harness.send(GGA_RTK)
    harness.tick()
    assert harness.recorder("/fix").msgs
    harness.recorder("/fix").msgs.clear()
    harness.recorder("/fix_status").msgs.clear()
    time.sleep(0.8)  # longer than fix_timeout_s, with no new sentences
    harness.tick()
    assert not harness.recorder("/fix").msgs
    assert harness.recorder("/fix_status").msgs[-1].data.startswith("STALE ")


def test_reader_thread_survives_the_device_disappearing(harness):
    harness.send(GGA_RTK)
    harness.tick()
    assert harness.recorder("/fix").msgs
    os.close(harness.master)
    harness.master = None
    # The reader thread must notice the vanished device and keep running rather
    # than take the node down with it.
    time.sleep(0.5)
    assert harness.node._reader_thread.is_alive()
    harness.recorder("/fix").msgs.clear()
    harness.tick()  # the node is still spinning and still reporting status
    assert harness.recorder("/fix_status").msgs


def test_missing_device_is_reported_not_fatal():
    h = Harness(device="/dev/definitely_missing_um960")
    try:
        time.sleep(0.5)
        assert h.node._connected is False
        assert h.node._reader_thread.is_alive()
        h.tick()
        assert "um960=disconnected" in h.recorder("/fix_status").msgs[-1].data
        assert not h.recorder("/fix").msgs
    finally:
        h.close()


def test_nmea_mirror_is_off_by_default(harness):
    harness.send(GGA_RTK)
    harness.tick()
    assert not harness.recorder("/nmea").msgs


def test_nmea_mirror_when_enabled(nmea_harness):
    nmea_harness.send(GGA_RTK)
    nmea_harness.tick()
    mirrored = nmea_harness.recorder("/nmea").msgs
    assert mirrored
    assert mirrored[-1].data.startswith("$GNGGA")


def test_published_message_types(harness):
    harness.send(_bestnav_frame())
    harness.tick()
    assert isinstance(harness.recorder("/fix").msgs[-1], NavSatFix)
    assert isinstance(harness.recorder("/heading").msgs[-1], Float32)
    assert isinstance(harness.recorder("/heading_valid").msgs[-1], Bool)
    assert isinstance(harness.recorder("/fix_status").msgs[-1], String)
