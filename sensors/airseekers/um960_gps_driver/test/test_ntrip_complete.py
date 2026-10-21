"""NTRIP completeness: auth, GGA upload, reconnect/backoff stats, bad RTCM dropped,
sourcetable, the owner settings file, /ntrip/status and rtcm_source=auto hand-over.

Client tests need no ROS; the node tests use the pty harness of test_um960_node.py.
"""

import json
import os
import stat
import time

import pytest

from um960_gps_driver import lora
from um960_gps_driver.fake_caster import FakeCaster, sample_frame
from um960_gps_driver.ntrip_client import (
    STATE_AUTH_FAILED,
    STATE_STREAMING,
    NtripClient,
    NtripConfig,
    NtripError,
    Rtcm3FrameCounter,
    fetch_sourcetable,
    nmea_wrap,
    parse_sourcetable,
)
from um960_gps_driver.ntrip_settings import load_settings, save_settings

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
        self.writes = 0

    def __call__(self, chunk):
        self.data.extend(chunk)
        self.writes += 1


def _client(caster, sink, password="pass", **kwargs):
    cfg = NtripConfig(caster.host, caster.port, caster.mountpoint, "user", password,
                      reconnect_min_s=0.05, reconnect_max_s=0.2, **kwargs)
    return NtripClient(cfg, sink, lambda: GGA)


# -- client --------------------------------------------------------------------------
def test_basic_auth_rejected_then_accepted():
    with FakeCaster(frame_period_s=0.01) as caster:
        sink = Sink()
        bad = _client(caster, sink, password="wrong")
        bad.start()
        try:
            _wait(lambda: bad.state == STATE_AUTH_FAILED)
            assert sink.data == b""
            assert "wrong" not in bad.config.describe()
        finally:
            bad.stop()
        good = _client(caster, sink)
        good.start()
        try:
            _wait(lambda: good.state == STATE_STREAMING and good.stats()["frames"] > 2)
            assert good.stats()["connected"] is True
            assert "Authorization: Basic dXNlcjpwYXNz" in caster.requests[-1]
        finally:
            good.stop()


def test_gga_in_request_header_and_every_interval():
    with FakeCaster(frame_period_s=0.02) as caster:
        client = _client(caster, Sink(), gga_interval_s=0.2)
        client.start()
        try:
            _wait(lambda: len(caster.uploads) >= 4)
        finally:
            client.stop()
        assert "Ntrip-GGA: %s" % GGA in caster.requests[0]
        assert "Ntrip-Version: Ntrip/2.0" in caster.requests[0]
        assert all(u == GGA for u in caster.uploads)
        gaps = [b - a for a, b in zip(caster.upload_times[1:], caster.upload_times[2:])]
        assert gaps and all(0.1 < g < 0.6 for g in gaps)


def test_reconnects_with_backoff_and_counts_retries():
    with FakeCaster(frame_period_s=0.01, close_after=3) as caster:
        client = _client(caster, Sink())
        client.start()
        try:
            _wait(lambda: client.connects >= 3)
            stats = client.stats()
            assert stats["attempts"] >= 3 and "retries" in stats and "backoff_s" in stats
        finally:
            client.stop()
    # Caster gone: retries pile up, backoff is capped at reconnect_max_s.
    client = NtripClient(NtripConfig("127.0.0.1", 1, "X", reconnect_min_s=0.02,
                                     reconnect_max_s=0.1, connect_timeout_s=0.2), Sink())
    client.start()
    try:
        _wait(lambda: client.consecutive_failures >= 4)
        assert client.stats()["backoff_s"] <= 0.1
        assert client.stats()["connected"] is False
    finally:
        client.stop()


def test_bad_rtcm_is_dropped_before_the_receiver():
    with FakeCaster(frame_period_s=0.005, corrupt_every=3) as caster:
        sink = Sink()
        client = _client(caster, sink)
        client.start()
        try:
            _wait(lambda: caster.corrupt_sent >= 5 and client.stats()["frames_written"] >= 10)
        finally:
            client.stop()
    stats = client.stats()
    assert stats["crc_errors"] >= 1 and stats["dropped_bytes"] >= 4
    # Everything written re-parses into valid frames with nothing left over.
    check = Rtcm3FrameCounter()
    frames = check.extract(bytes(sink.data))
    assert len(frames) == stats["frames_written"]
    assert check.crc_errors == 0 and check.dropped_bytes == 0
    assert b"junk" not in sink.data


def test_unvalidated_mode_forwards_raw_bytes():
    with FakeCaster(frame_period_s=0.005, corrupt_every=2) as caster:
        sink = Sink()
        client = _client(caster, sink, validate_rtcm=False)
        client.start()
        try:
            _wait(lambda: caster.corrupt_sent >= 2 and b"junk" in sink.data)
        finally:
            client.stop()


def test_frame_split_across_reads_is_reassembled():
    frame = sample_frame(1077, 3, size=200)
    framer = Rtcm3FrameCounter()
    assert framer.extract(b"\x00\x01" + frame[:50]) == []
    assert framer.extract(frame[50:]) == [frame]
    assert framer.dropped_bytes == 2


SOURCETABLE = (
    "STR;MOUNT1;Berlin;RTCM 3.2;1005(10),1077(1);2;GPS+GLO;NET;DEU;52.52;13.40;1;0;"
    "sNTRIP;none;B;N;9600;misc\r\n"
    "STR;MOUNT2;Paris;RTCM 3.3;1006(10);2;GPS;NET;FRA;48.85;2.35;0;0;gen;none;N;N;0;\r\n"
    "CAS;caster.example;2101;Example;Op;0;DEU;52;13;;0;\r\n"
    "NET;NET;Operator;B;N;;;;\r\n"
    "garbage line\r\n"
    "ENDSOURCETABLE\r\n"
    "STR;AFTER;;;;;;;;0;0;0;0;;;;;0;\r\n"
)


def test_parse_sourcetable():
    table = parse_sourcetable(SOURCETABLE)
    assert [s["mountpoint"] for s in table["streams"]] == ["MOUNT1", "MOUNT2"]
    m1 = table["streams"][0]
    assert m1["lat"] == pytest.approx(52.52) and m1["lon"] == pytest.approx(13.40)
    assert m1["nmea"] is True and table["streams"][1]["nmea"] is False
    assert m1["bitrate"] == 9600 and m1["country"] == "DEU" and m1["format"] == "RTCM 3.2"
    assert table["casters"][0]["host"] == "caster.example"
    assert table["networks"][0]["identifier"] == "NET"


@pytest.mark.parametrize("version", [1, 2])
def test_fetch_sourcetable_from_caster(version):
    with FakeCaster(sourcetable=SOURCETABLE) as caster:
        table = fetch_sourcetable(caster.host, caster.port, version=version, timeout_s=2.0)
        assert [s["mountpoint"] for s in table["streams"]] == ["MOUNT1", "MOUNT2"]
        assert caster.requests[-1].startswith("GET / HTTP/1.")


def test_fetch_sourcetable_unreachable():
    with pytest.raises((OSError, NtripError)):
        fetch_sourcetable("127.0.0.1", 1, timeout_s=0.5)


# -- settings file -------------------------------------------------------------------
def test_settings_roundtrip_is_private(tmp_path):
    path = str(tmp_path / "sub" / "ntrip.yaml")
    assert load_settings(path) is None
    save_settings(path, {"enabled": True, "host": "h.example", "port": 2102,
                         "mountpoint": "/MP", "user": "me@x", "password": 'p"w:#d'})
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600
    loaded = load_settings(path)
    assert loaded == {"enabled": True, "host": "h.example", "port": 2102,
                      "mountpoint": "MP", "user": "me@x", "password": 'p"w:#d'}


def test_settings_tolerates_garbage(tmp_path):
    path = tmp_path / "ntrip.yaml"
    path.write_text("- not\n- a mapping\n")
    assert load_settings(str(path)) is None
    path.write_text("enabled: yes\nhost: h\n")
    assert load_settings(str(path))["enabled"] is True
    assert load_settings(str(path))["port"] == 2101


# -- node ------------------------------------------------------------------------------
@pytest.fixture
def harness_mod():
    pytest.importorskip("rclpy")
    import test_corrections
    import test_um960_node
    return test_um960_node, test_corrections


def _write_settings(path, caster, enabled=True, password=None):
    save_settings(str(path), {"enabled": enabled, "host": caster.host, "port": caster.port,
                              "mountpoint": caster.mountpoint, "user": caster.user,
                              "password": caster.password if password is None else password})


def test_auto_hands_over_from_lora_to_ntrip_and_back(harness_mod, tmp_path):
    tum, tc = harness_mod
    caster = FakeCaster(frame_period_s=0.02, password="s3cr3t-Pw").start()
    path = tmp_path / "ntrip.yaml"
    _write_settings(path, caster)
    h = tum.Harness(extra_params={"ntrip_settings_file": str(path),
                                  "forward_rtcm_without_board_ack": True,
                                  "ntrip_fallback_grace_s": 0.5,
                                  "ntrip_keepalive_s": 0.5,
                                  "ntrip_no_data_timeout_s": 1.0,
                                  "ntrip_gga_interval_s": 0.2})
    try:
        assert h.node.rtcm_source == "auto" and h.node.ntrip_enabled
        assert h.node.ntrip_config_source == str(path)
        reader = tc.PtyReader(h.master)
        _wait(lambda: h.node.correction_source == "ntrip")
        _wait(lambda: reader.poll().startswith(b"RTKRESET\r\n" + lora.NRTK_ON.encode()))
        _wait(lambda: sample_frame(1005, 4) in reader.poll())
        status = h.node.ntrip_status()
        assert status["active_source"] == "ntrip" and status["connected"]
        assert status["rate_bps"] > 0 and status["rtcm_age_s"] is not None
        text = json.dumps(status)
        assert caster.password not in text and "password_set" in text
        h.node.publish_ntrip_status()
        caster.stop()
        _wait(lambda: h.node.correction_source == "lora", timeout=5.0)
        # Keep-alive stops (the board falls back to LoRa by itself); no OFF command.
        reader.poll(0.2)
        before = len(reader.data)
        assert lora.NRTK_ON.encode() not in reader.poll(1.5)[before:]
        assert lora.NRTK_OFF.encode() not in reader.data
        assert h.node.ntrip_status()["switches"] == 2
    finally:
        h.close()
        try:
            caster.stop()
        except Exception:  # noqa: BLE001
            pass


def test_auto_disabled_stays_on_lora_and_writes_nothing(harness_mod, tmp_path):
    tum, tc = harness_mod
    with FakeCaster(frame_period_s=0.02) as caster:
        path = tmp_path / "ntrip.yaml"
        _write_settings(path, caster, enabled=False)
        h = tum.Harness(extra_params={"ntrip_settings_file": str(path)})
        try:
            assert h.node.correction_source == "lora" and h.node.ntrip is None
            assert tc.PtyReader(h.master).poll(0.3) == b""
            assert caster.connections == 0
            assert h.node.ntrip_status()["state"] == "off"
        finally:
            h.close()


def test_settings_file_edit_is_picked_up(harness_mod, tmp_path):
    tum, tc = harness_mod
    with FakeCaster(frame_period_s=0.02) as caster:
        path = tmp_path / "ntrip.yaml"
        _write_settings(path, caster, enabled=False)
        h = tum.Harness(extra_params={"ntrip_settings_file": str(path),
                                      "forward_rtcm_without_board_ack": True})
        try:
            assert h.node.ntrip is None
            time.sleep(0.05)
            _write_settings(path, caster, enabled=True)
            os.utime(path, (time.time() + 5, time.time() + 5))
            h.node.publish_ntrip_status()  # the 1 Hz timer checks the mtime
            _wait(lambda: h.node.ntrip is not None)
            _wait(lambda: h.node.correction_source == "ntrip")
        finally:
            h.close()


def test_rtcm_source_lora_ignores_enabled_ntrip(harness_mod, tmp_path):
    tum, tc = harness_mod
    with FakeCaster(frame_period_s=0.02) as caster:
        path = tmp_path / "ntrip.yaml"
        _write_settings(path, caster)
        h = tum.Harness(extra_params={"ntrip_settings_file": str(path), "rtcm_source": "lora"})
        try:
            assert h.node.correction_source == "lora" and h.node.ntrip is None
            assert tc.PtyReader(h.master).poll(0.3) == b""
        finally:
            h.close()


def test_live_param_change_persists_settings(harness_mod, tmp_path):
    try:
        from rclpy.parameter import Parameter
    except ImportError:
        pytest.skip("needs ROS 2 rclpy parameter callbacks")

    tum, tc = harness_mod
    with FakeCaster(frame_period_s=0.02) as caster:
        path = tmp_path / "ros2" / "ntrip.yaml"
        h = tum.Harness(extra_params={"ntrip_settings_file": str(path),
                                      "forward_rtcm_without_board_ack": True})
        try:
            assert not path.exists() and h.node.correction_source == "lora"
            # What the GUI NTRIP card sends (param_bindings.go AirseekersTron).
            results = h.node.set_parameters([
                Parameter("ntrip_host", value=caster.host),
                Parameter("ntrip_port", value=caster.port),
                Parameter("ntrip_mountpoint", value=caster.mountpoint),
                Parameter("ntrip_user", value=caster.user),
                Parameter("ntrip_password", value=caster.password),
                Parameter("correction_source", value="ntrip"),
            ])
            assert all(r.successful for r in results)
            _wait(lambda: path.exists() and h.node.correction_source == "ntrip")
            saved = load_settings(str(path))
            assert saved["enabled"] is True and saved["host"] == caster.host
            assert saved["password"] == caster.password
            assert stat.S_IMODE(os.stat(path).st_mode) == 0o600
            reader = tc.PtyReader(h.master)
            _wait(lambda: sample_frame(1005, 2) in reader.poll())
            # Switching off persists enabled: false.
            h.node.set_parameters([Parameter("correction_source", value="lora")])
            _wait(lambda: load_settings(str(path))["enabled"] is False)
            _wait(lambda: h.node.correction_source == "lora" and h.node.ntrip is None)
        finally:
            h.close()
