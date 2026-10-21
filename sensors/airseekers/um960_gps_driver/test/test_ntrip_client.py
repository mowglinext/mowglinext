"""NTRIP client tests against an in-process fake caster (threading + socketserver).

No network beyond 127.0.0.1, no ROS.
"""

import base64
import socket
import threading
import time

import pytest

from um960_gps_driver import lora
from um960_gps_driver.fake_caster import FakeCaster, sample_frame
from um960_gps_driver.ntrip_client import (
    STATE_AUTH_FAILED,
    STATE_BAD_MOUNTPOINT,
    STATE_OFF,
    STATE_STREAMING,
    ChunkedDecoder,
    NtripClient,
    NtripConfig,
    Rtcm3FrameCounter,
    build_request,
    make_gga,
    nmea_wrap,
    parse_response_head,
)
from um960_gps_driver.parsers import NmeaParser, nmea_checksum_ok

GGA = nmea_wrap("GNGGA,101530.00,4807.038,N,01131.000,E,4,21,0.6,545.4,M,46.9,M,0.5,0451")


def _wait(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    raise AssertionError("condition not met within %.1fs" % timeout)


class Sink:
    def __init__(self):
        self.data = bytearray()
        self.lock = threading.Lock()

    def __call__(self, chunk):
        with self.lock:
            self.data.extend(chunk)


def _client(caster, sink, version=2, user="user", password="pass", mount="TEST",
            gga=lambda: GGA, **kwargs):
    cfg = NtripConfig(caster.host, caster.port, mount, user, password, version=version,
                      reconnect_min_s=0.05, reconnect_max_s=0.2, **kwargs)
    return NtripClient(cfg, sink, gga)


# -- pure helpers --------------------------------------------------------------------
def test_v2_request_has_version_host_and_basic_auth():
    req = build_request("caster.example", 2101, "MOUNT", "u", "p:w", 2, GGA).decode()
    assert req.startswith("GET /MOUNT HTTP/1.1\r\n")
    assert "Ntrip-Version: Ntrip/2.0\r\n" in req
    assert "Host: caster.example:2101\r\n" in req
    assert "Authorization: Basic %s\r\n" % base64.b64encode(b"u:p:w").decode() in req
    assert "Ntrip-GGA: $GNGGA" in req
    assert req.endswith("\r\n\r\n")


def test_v1_request_is_plain_http10_without_ntrip_version():
    req = build_request("h", 2101, "/M", "", "", 1).decode()
    assert req.startswith("GET /M HTTP/1.0\r\n")
    assert "Ntrip-Version" not in req and "Authorization" not in req


def test_parse_response_head_variants():
    assert parse_response_head(b"ICY 200 OK")[1] == 200
    status, code, headers = parse_response_head(
        b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\nContent-Type: gnss/data")
    assert code == 200 and headers["transfer-encoding"] == "chunked"
    assert parse_response_head(b"HTTP/1.1 401 Unauthorized")[1] == 401


def test_chunked_decoder_handles_arbitrary_boundaries():
    payload = bytes(range(256)) * 3
    encoded = b"".join(b"%X\r\n" % len(payload[i:i + 100]) + payload[i:i + 100] + b"\r\n"
                       for i in range(0, len(payload), 100)) + b"0\r\n\r\n"
    decoder = ChunkedDecoder()
    out = b"".join(decoder.feed(encoded[i:i + 7]) for i in range(0, len(encoded), 7))
    assert out == payload and decoder.finished


def test_rtcm3_counter_validates_crc_and_resyncs():
    counter = Rtcm3FrameCounter()
    good = sample_frame(1077, 3)
    bad = bytearray(sample_frame(1005, 1))
    bad[-1] ^= 0xFF
    stream = b"\x00\xd3junk" + bytes(bad) + good + good
    total = sum(counter.feed(stream[i:i + 5]) for i in range(0, len(stream), 5))
    assert total == 2 and counter.frames == 2
    assert counter.crc_errors >= 1
    assert counter.message_types == {1077: 2}


def test_make_gga_round_trips_through_the_nmea_parser():
    sentence = make_gga(-33.8688, 151.2093, 58.0, utc=0.0)
    assert sentence.endswith("\r\n") and nmea_checksum_ok(sentence.strip())
    parser = NmeaParser()
    parser.feed(sentence)
    assert parser.fix["lat"] == pytest.approx(-33.8688, abs=1e-6)
    assert parser.fix["lon"] == pytest.approx(151.2093, abs=1e-6)


# -- against the fake caster ---------------------------------------------------------
@pytest.mark.parametrize("mode,version", [("v2", 2), ("icy", 1), ("v2_chunked", 2)])
def test_handshake_variants_stream_valid_rtcm(mode, version):
    sink = Sink()
    with FakeCaster(mode=mode, frame_period_s=0.01) as caster:
        client = _client(caster, sink, version=version)
        client.start()
        try:
            _wait(lambda: client.stats()["frames"] >= 5)
            stats = client.stats()
            assert stats["state"] == STATE_STREAMING
            assert stats["crc_errors"] == 0, "chunk framing leaked into the RTCM stream"
            assert set(stats["message_types"]) <= {1005, 1077}
            with sink.lock:
                assert bytes(sink.data).startswith(sample_frame(1005, 0))
            assert stats["bytes_written"] == stats["bytes_rx"] > 0
        finally:
            client.stop()
        head = caster.requests[0]
        assert ("Ntrip-Version: Ntrip/2.0" in head) == (version == 2)


def test_auth_failure_is_reported_and_nothing_is_written():
    sink = Sink()
    with FakeCaster() as caster:
        client = _client(caster, sink, password="wrong")
        client.start()
        try:
            _wait(lambda: client.stats()["state"] == STATE_AUTH_FAILED)
            assert "credentials" in client.stats()["last_error"]
            assert not sink.data
        finally:
            client.stop()
        assert client.stats()["state"] == STATE_OFF


def test_unknown_mountpoint_reports_sourcetable():
    with FakeCaster() as caster:
        client = _client(caster, Sink(), mount="NOPE")
        client.start()
        try:
            _wait(lambda: client.stats()["state"] == STATE_BAD_MOUNTPOINT)
        finally:
            client.stop()


def test_reconnects_after_the_caster_drops_the_stream():
    sink = Sink()
    with FakeCaster(close_after=3, frame_period_s=0.01) as caster:
        client = _client(caster, sink)
        client.start()
        try:
            _wait(lambda: caster.connections >= 3)
            _wait(lambda: client.stats()["connects"] >= 3)
            assert client.stats()["frames"] >= 6
        finally:
            client.stop()


def test_reconnects_once_the_caster_comes_up():
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    sink = Sink()
    cfg = NtripConfig("127.0.0.1", port, "TEST", "user", "pass",
                      reconnect_min_s=0.05, reconnect_max_s=0.1, connect_timeout_s=0.5)
    client = NtripClient(cfg, sink, lambda: GGA)
    client.start()
    try:
        _wait(lambda: client.stats()["attempts"] >= 2)
        assert client.stats()["state"] != STATE_STREAMING
        with FakeCaster(port=port, frame_period_s=0.01):
            _wait(lambda: client.stats()["frames"] >= 1)
    finally:
        client.stop()


def test_gga_upload_cadence():
    with FakeCaster(frame_period_s=0.02) as caster:
        client = _client(caster, Sink(), gga_interval_s=0.25)
        client.start()
        try:
            _wait(lambda: len([u for u in caster.uploads if u.startswith("$GNGGA")]) >= 5,
                  timeout=4.0)
        finally:
            client.stop()
        uploads = [t for u, t in zip(caster.uploads, caster.upload_times)
                   if u.startswith("$GNGGA")]
        # The first one goes in the NTRIP 2 request header, the next straight after
        # the handshake, then one per interval.
        assert caster.uploads[0] == GGA
        gaps = [b - a for a, b in zip(uploads[2:], uploads[3:])]
        assert all(0.15 < gap < 0.6 for gap in gaps), gaps
        assert client.stats()["gga_sent"] >= 4


def test_no_gga_without_a_position_or_when_disabled():
    with FakeCaster(frame_period_s=0.02) as caster:
        a = _client(caster, Sink(), gga=lambda: None, gga_interval_s=0.05)
        b = _client(caster, Sink(), gga_interval_s=0.0, version=1)
        a.start()
        b.start()
        try:
            _wait(lambda: a.stats()["frames"] >= 3 and b.stats()["frames"] >= 3)
        finally:
            a.stop()
            b.stop()
        assert caster.uploads == []
        assert a.stats()["gga_sent"] == 0 and b.stats()["gga_sent"] == 0


def test_stats_rate_and_age_and_write_errors_do_not_kill_the_client():
    calls = []

    def failing_writer(data):
        calls.append(len(data))
        raise OSError("port closed")

    with FakeCaster(frame_period_s=0.01) as caster:
        client = _client(caster, failing_writer)
        client.start()
        try:
            _wait(lambda: len(calls) >= 5)
            stats = client.stats()
            assert stats["state"] == STATE_STREAMING
            assert stats["write_errors"] >= 5 and stats["bytes_written"] == 0
            assert stats["rate_bps"] > 0
            assert stats["age_s"] is not None and stats["age_s"] < 1.0
        finally:
            client.stop()
        assert not client.running


# -- vendor LoRa pairing commands ------------------------------------------------------
def test_vendor_command_literals_match_the_xor_checksum():
    for literal in list(lora.GLIST_BY_AREA.values()) + [
            lora.QUERY_PAIRING, lora.NRTK_ON, lora.NRTK_OFF, "$GNVER,*64", "$GHARD,F*32"]:
        assert nmea_wrap(literal) == literal


def test_lora_pairing_commands():
    assert lora.gncon_command(0x1234, 75) == nmea_wrap("GNCON,1234,4B")
    ok, cmds, _ = lora.pairing_commands("AB12030000", 1, 10)
    assert ok and cmds == ["$GLIST,2*5B", nmea_wrap("GNCON,0001,0A")]
    ok, cmds, _ = lora.pairing_commands("3001034903XYZ", 1, 10)   # forced region 01
    assert ok and cmds[0] == "$GLIST,1*58"
    ok, cmds, note = lora.pairing_commands("x", 1, 10)             # bad SN: pair anyway
    assert ok and len(cmds) == 1 and "sn error" in note
    assert lora.pairing_commands("", 1, 10, area="F")[1][0] == "$GLIST,F*2F"
    assert lora.pairing_commands("", 1, 10, area="ZZ")[0] is False
    assert lora.pairing_commands("", 1, 0)[0] is False
    assert lora.pairing_commands("", 1, 76)[0] is False
