# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Contract-parity tests for the production Python fallback bridge."""

import os
from pathlib import Path
import sys
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mowgli_interfaces.msg import GnssStatus as PublicGnssStatus

_BRIDGE_FILENAME = 'universal_gnss_topic_bridge.py'


def _locate_reference_bridge() -> Path:
    """
    Find sensors/gps/universal_gnss_topic_bridge.py.

    In the repo it sits two levels above this file. Under colcon it does NOT:
    the workspace is assembled by copying ONLY the package
    (`cp -r sensors/gps/mowgli_gnss_bridge /ws/src/...`), which leaves the
    reference bridge outside the tree entirely. CMake therefore passes its
    configure-time path in MOWGLI_GNSS_PYTHON_BRIDGE_DIR, and we fall back to
    walking up for the benefit of anyone running pytest directly.
    """
    env_dir = os.environ.get('MOWGLI_GNSS_PYTHON_BRIDGE_DIR')
    candidates = []
    if env_dir:
        candidates.append(Path(env_dir))
    candidates.extend(Path(__file__).resolve().parents)
    for directory in candidates:
        if (directory / _BRIDGE_FILENAME).is_file():
            return directory
    raise RuntimeError(
        f'cannot find {_BRIDGE_FILENAME}; looked in '
        + ', '.join(str(c) for c in candidates)
        + '. When assembling a colcon workspace by copying the package, copy '
        'sensors/gps/universal_gnss_topic_bridge.py alongside it.'
    )


sys.path.insert(0, str(_locate_reference_bridge()))

from universal_gnss_topic_bridge import (  # noqa: E402
    CorrectionDiagnosticTracker,
    UniversalGnssStatus,
    UniversalGnssTopicBridge,
)


class _CapturePublisher:

    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _make_bridge() -> UniversalGnssTopicBridge:
    bridge = object.__new__(UniversalGnssTopicBridge)
    bridge._frame_id = 'gps_link'
    bridge._backend = 'universal'
    bridge._receiver_vendor = 'u-blox'
    bridge._correction_diagnostics = CorrectionDiagnosticTracker(2.0)
    bridge._status_pub = _CapturePublisher()
    return bridge


def _status(name: str, source: str, message: str = '', **values: str) -> DiagnosticStatus:
    entry = DiagnosticStatus()
    entry.name = name
    entry.hardware_id = source
    entry.message = message
    entry.values = [KeyValue(key=key, value=value) for key, value in values.items()]
    return entry


def _healthy_ntrip(source: str = 'caster:2101/MOUNT', stamp: int = 10) -> DiagnosticArray:
    diagnostics = DiagnosticArray()
    diagnostics.header.stamp.sec = stamp
    diagnostics.status = [
        _status(
            'universal_gnss_ntrip/summary',
            source,
            correction_available='true',
            transport_healthy='true',
            parser_healthy='true',
            stale_data='false',
        ),
        _status('universal_gnss_ntrip/ntrip_streaming', source),
        _status('universal_gnss_ntrip/correction_flowing', source),
        _status(
            'universal_gnss_ntrip/rtcm_forwarding',
            source,
            'RTCM forwarding active',
        ),
        _status(
            'universal_gnss_ntrip/rtcm_semantic/base_station_arp',
            source,
            seen='true',
            decoded='true',
            valid='true',
            message_type='1006',
            station_id='42',
            age_s='20.0',
        ),
        _status(
            'universal_gnss_ntrip/rtcm_semantic/msm_summary',
            source,
            seen='true',
            decoded='true',
            valid='true',
            message_type='1077',
            station_id='42',
            constellations_seen='GPS',
            satellite_count='12',
            signal_count='18',
            cell_count='24',
            malformed_count='0',
            age_s='0.2',
        ),
    ]
    return diagnostics


def _project(tracker: CorrectionDiagnosticTracker, now: float) -> PublicGnssStatus:
    status = PublicGnssStatus()
    tracker.apply(status, now)
    return status


class PythonBridgeContractTest(unittest.TestCase):

    def test_receipt_stamp_and_sequence_match_cpp_contract_vector(self) -> None:
        bridge = _make_bridge()
        incoming = UniversalGnssStatus()
        incoming.fix_type = UniversalGnssStatus.FIX_TYPE_RTK_FIXED
        incoming.fix_valid = True
        incoming.stamp.sec = 123
        incoming.stamp.nanosec = 456
        incoming.position_observation_sequence = 41

        bridge._on_status(incoming)

        public = bridge._status_pub.messages[-1]
        self.assertEqual(public.header.stamp.sec, 123)
        self.assertEqual(public.header.stamp.nanosec, 456)
        self.assertEqual(public.position_observation_sequence, 41)
        self.assertEqual(public.header.frame_id, 'gps_link')

    def test_cached_and_genuine_identical_observation_sequences_are_preserved(self) -> None:
        bridge = _make_bridge()
        incoming = UniversalGnssStatus()
        incoming.fix_type = UniversalGnssStatus.FIX_TYPE_RTK_FIXED
        incoming.fix_valid = True
        incoming.stamp.sec = 200
        incoming.stamp.nanosec = 300
        incoming.position_observation_sequence = 700

        bridge._on_status(incoming)
        bridge._on_status(incoming)
        incoming.position_observation_sequence = 701
        bridge._on_status(incoming)

        self.assertEqual(
            [msg.position_observation_sequence for msg in bridge._status_pub.messages],
            [700, 700, 701],
        )


class PythonCorrectionTrackerParityTest(unittest.TestCase):
    """Vectors mirror test_correction_diagnostic_tracker.cpp."""

    def test_healthy_dynamic_flow_without_1230(self) -> None:
        tracker = CorrectionDiagnosticTracker(2.0)
        tracker.update(_healthy_ntrip(), 100.0)
        status = _project(tracker, 100.0)

        self.assertEqual(
            status.correction_transport_status,
            PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_STREAMING,
        )
        self.assertTrue(status.correction_response_accepted)
        self.assertEqual(
            status.correction_flow_status,
            PublicGnssStatus.CORRECTION_FLOW_STATUS_ACTIVE,
        )
        self.assertEqual(
            status.correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_HEALTHY,
        )
        self.assertEqual(status.msm_summary_cell_count, 24)

    def test_transport_response_forwarding_and_invalid_semantics_are_distinct(self) -> None:
        connected = CorrectionDiagnosticTracker(2.0)
        array = DiagnosticArray()
        array.header.stamp.sec = 1
        array.status = [_status('universal_gnss_ntrip/ntrip_connected', 'caster:2101/MOUNT')]
        connected.update(array, 100.0)
        status = _project(connected, 100.0)
        self.assertEqual(
            status.correction_transport_status,
            PublicGnssStatus.CORRECTION_TRANSPORT_STATUS_CONNECTED,
        )
        self.assertFalse(status.correction_response_accepted)
        self.assertNotEqual(
            status.correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_HEALTHY,
        )

        accepted = CorrectionDiagnosticTracker(2.0)
        array.status = [
            _status('universal_gnss_ntrip/ntrip_streaming', 'caster:2101/MOUNT'),
            _status(
                'universal_gnss_ntrip/rtcm_forwarding',
                'caster:2101/MOUNT',
                'RTCM forwarding active',
            ),
        ]
        accepted.update(array, 100.0)
        status = _project(accepted, 100.0)
        self.assertTrue(status.correction_response_accepted)
        self.assertEqual(
            status.correction_stream_status,
            PublicGnssStatus.CORRECTION_STREAM_STATUS_ACTIVE,
        )
        self.assertEqual(
            status.correction_flow_status,
            PublicGnssStatus.CORRECTION_FLOW_STATUS_WAITING,
        )
        self.assertEqual(
            status.correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_WAITING,
        )

    def test_malformed_zero_cell_static_only_and_station_change_fail_closed(self) -> None:
        malformed_array = _healthy_ntrip()
        malformed_array.status[0].values = [
            KeyValue(key=item.key, value='false' if item.key == 'parser_healthy' else item.value)
            for item in malformed_array.status[0].values
        ]
        tracker = CorrectionDiagnosticTracker(2.0)
        tracker.update(malformed_array, 100.0)
        self.assertEqual(
            _project(tracker, 100.0).correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_INVALID,
        )

        zero_cell_array = _healthy_ntrip()
        zero_cell_array.status[-1].values = [
            KeyValue(key=item.key, value='0' if item.key == 'cell_count' else item.value)
            for item in zero_cell_array.status[-1].values
        ]
        tracker = CorrectionDiagnosticTracker(2.0)
        tracker.update(zero_cell_array, 100.0)
        self.assertEqual(
            _project(tracker, 100.0).correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_INVALID,
        )

        static_only = _healthy_ntrip()
        static_only.status = [
            entry
            for entry in static_only.status
            if entry.name != 'universal_gnss_ntrip/rtcm_semantic/msm_summary'
        ]
        for item in static_only.status[0].values:
            if item.key == 'correction_available':
                item.value = 'false'
        tracker = CorrectionDiagnosticTracker(2.0)
        tracker.update(static_only, 100.0)
        self.assertEqual(
            _project(tracker, 100.0).correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_WAITING,
        )

        station_change = _healthy_ntrip()
        for item in station_change.status[-1].values:
            if item.key == 'station_id':
                item.value = '43'
        tracker = CorrectionDiagnosticTracker(2.0)
        tracker.update(station_change, 100.0)
        self.assertEqual(
            _project(tracker, 100.0).correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_INVALID,
        )

    def test_expiry_disconnect_reconnect_source_change_and_old_set(self) -> None:
        tracker = CorrectionDiagnosticTracker(1.0)
        tracker.update(_healthy_ntrip('caster:2101/A', 10), 100.0)
        expired = _project(tracker, 101.0)
        self.assertEqual(expired.value_flags & PublicGnssStatus.CAP_CORRECTION_SEMANTIC, 0)

        receiver = DiagnosticArray()
        receiver.header.stamp.sec = 11
        receiver.status = [
            _status(
                'universal_gnss/rtcm_forwarding',
                'serial:/dev/ttyACM0',
                'RTCM forwarding active',
            ),
            _status(
                'universal_gnss/rtcm_semantic/msm_summary',
                'serial:/dev/ttyACM0',
                seen='true',
                decoded='true',
                valid='true',
                station_id='77',
                cell_count='9',
                age_s='0.1',
            ),
        ]
        tracker.update(receiver, 101.0)
        fallback = _project(tracker, 101.1)
        self.assertEqual(
            fallback.correction_stream_status,
            PublicGnssStatus.CORRECTION_STREAM_STATUS_ACTIVE,
        )
        self.assertEqual(fallback.msm_summary_station_id, 77)

        reconnecting = DiagnosticArray()
        reconnecting.header.stamp.sec = 12
        reconnecting.status = [
            _status('universal_gnss_ntrip/ntrip_reconnecting', 'caster:2101/B')
        ]
        tracker.update(reconnecting, 101.2)
        cleared = _project(tracker, 101.2)
        self.assertEqual(
            cleared.correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_UNAVAILABLE,
        )
        # Reconnecting NTRIP clears NTRIP-owned semantics, but must not shadow
        # the still-current receiver-owned MSM observation.
        self.assertEqual(cleared.msm_summary_source, 'serial:/dev/ttyACM0')

        tracker.update(_healthy_ntrip('caster:2101/A', 10), 101.3)
        still_clear = _project(tracker, 101.3)
        self.assertEqual(still_clear.correction_source, 'caster:2101/B')

        tracker.update(_healthy_ntrip('caster:2101/B', 13), 101.4)
        restored = _project(tracker, 101.4)
        self.assertEqual(
            restored.correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_HEALTHY,
        )

    def test_unrelated_array_is_omit_owner_snapshot_without_entries_is_clear(self) -> None:
        tracker = CorrectionDiagnosticTracker(2.0)
        tracker.update(_healthy_ntrip(), 100.0)
        receiver_clear = DiagnosticArray()
        receiver_clear.header.stamp.sec = 10
        receiver_clear.status = [
            _status('universal_gnss/summary', 'serial:/dev/ttyACM0')
        ]
        tracker.update(receiver_clear, 100.05)
        self.assertEqual(
            _project(tracker, 100.05).value_flags
            & PublicGnssStatus.CAP_CORRECTION_STREAM,
            0,
        )
        unrelated = DiagnosticArray()
        unrelated.header.stamp.sec = 11
        unrelated.status = [_status('other/component', 'other')]
        tracker.update(unrelated, 100.1)
        self.assertEqual(
            _project(tracker, 100.1).correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_HEALTHY,
        )

        clear = DiagnosticArray()
        clear.header.stamp.sec = 12
        clear.status = [_status('universal_gnss_ntrip/ntrip_streaming', 'caster:2101/MOUNT')]
        tracker.update(clear, 100.2)
        self.assertEqual(
            _project(tracker, 100.2).correction_semantic_status,
            PublicGnssStatus.CORRECTION_SEMANTIC_STATUS_WAITING,
        )
        self.assertEqual(
            _project(tracker, 100.2).value_flags & PublicGnssStatus.CAP_MSM_SUMMARY,
            0,
        )


if __name__ == '__main__':
    unittest.main()
