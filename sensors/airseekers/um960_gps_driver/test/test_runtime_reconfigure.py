"""Runtime parameter changes switch the correction source (GUI settings binding).

Uses the pty harness of test_um960_node.py and the fake caster: the node starts
in one mode, a ``set_parameters`` call (what ``ros2 param set`` and the foxglove
bridge do) switches it, and the bytes the node writes to the receiver port show
the switch took effect. Needs real rclpy (parameter callbacks).
"""

import time

import pytest

rclpy = pytest.importorskip("rclpy")
# The conftest.py shim exposes an attribute called `parameter` that is not a module, so
# `hasattr(rclpy, "parameter")` is true even without ROS and the import below then fails
# at *collection* time instead of skipping. Importorskip the submodule: ModuleNotFoundError
# is an ImportError, so a plain-Python run skips cleanly and a ROS 2 run proceeds.
pytest.importorskip("rclpy.parameter", reason="needs ROS 2 rclpy parameter callbacks")

from rclpy.parameter import Parameter  # noqa: E402

from test_corrections import PtyReader, _corrections, _ntrip_params, _wait  # noqa: E402
from test_um960_node import Harness  # noqa: E402

from um960_gps_driver.fake_caster import FakeCaster, sample_frame  # noqa: E402


@pytest.fixture
def caster():
    c = FakeCaster(frame_period_s=0.02).start()
    yield c
    c.stop()


def _set(h, **params):
    results = h.node.set_parameters([Parameter(k, value=v) for k, v in params.items()])
    return [r.successful for r in results]


def _reconfigured(h, count):
    _wait(lambda: h.node.corrections_reconfigured >= count)


def test_lora_to_ntrip_at_runtime(caster):
    h = Harness(extra_params={"forward_rtcm_without_board_ack": True})
    try:
        reader = PtyReader(h.master)
        assert h.node.correction_source == "lora" and h.node.ntrip is None
        assert reader.poll(0.2) == b"", "lora mode never writes"

        # Caster fields and the source in one request, as the GUI binding sends them.
        params = _ntrip_params(caster)
        params.pop("ntrip_version")
        params.pop("ntrip_gga_interval_s")
        assert all(_set(h, **params))
        _reconfigured(h, 1)
        assert h.node.correction_source == "ntrip" and h.node.ntrip is not None
        # Handshake for the already-open port, then RTCM from the new caster.
        _wait(lambda: reader.poll().startswith(b"RTKRESET\r\n"))
        _wait(lambda: sample_frame(1005, 0) in reader.poll())
        diag = _corrections(h)
        assert diag["source"] == "ntrip" and diag["host"] == caster.host
    finally:
        h.close()


def test_ntrip_to_lora_stops_writing(caster):
    h = Harness(extra_params=_ntrip_params(caster, forward_rtcm_without_board_ack=True))
    try:
        reader = PtyReader(h.master)
        _wait(lambda: sample_frame(1005, 0) in reader.poll())
        old_client = h.node.ntrip

        assert all(_set(h, correction_source="lora"))
        _reconfigured(h, 1)
        assert h.node.correction_source == "lora" and h.node.ntrip is None
        assert not old_client.running
        # Drain what was in flight, then nothing more reaches the receiver.
        reader.poll(0.3)
        before = len(reader.data)
        time.sleep(0.5)
        assert len(reader.poll(0.1)) == before
        assert _corrections(h)["source"] == "lora"
    finally:
        h.close()


def test_caster_change_reconnects(caster):
    other = FakeCaster(frame_period_s=0.02, mountpoint="OTHER").start()
    h = Harness(extra_params=_ntrip_params(caster, forward_rtcm_without_board_ack=True))
    try:
        _wait(lambda: _corrections(h).get("frames", 0) > 0)
        assert all(_set(h, ntrip_port=other.port, ntrip_mountpoint="OTHER"))
        _reconfigured(h, 1)
        diag = _corrections(h)
        assert diag["port"] == other.port and diag["mountpoint"] == "OTHER"
        # A fresh client: its counters start over and climb from the new caster.
        _wait(lambda: _corrections(h).get("connects", 0) >= 1 and _corrections(h)["frames"] > 0)
    finally:
        h.close()
        other.stop()


def test_unknown_source_is_rejected():
    h = Harness()
    try:
        assert _set(h, correction_source="carrier-pigeon") == [False]
        assert h.node.get_parameter("correction_source").value == ""
        time.sleep(0.5)
        assert h.node.corrections_reconfigured == 0
        assert h.node.correction_source == "lora"
    finally:
        h.close()


def test_unrelated_parameter_does_not_reconfigure():
    h = Harness()
    try:
        assert _set(h, publish_rate_hz=20.0) == [True]
        time.sleep(0.5)
        assert h.node.corrections_reconfigured == 0
    finally:
        h.close()
