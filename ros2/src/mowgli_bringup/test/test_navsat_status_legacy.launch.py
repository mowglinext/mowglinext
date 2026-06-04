# Copyright 2026 Mowgli Project
#
# SPDX-License-Identifier: GPL-3.0

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from mowgli_interfaces.msg import GnssStatus
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _make_fix() -> NavSatFix:
    msg = NavSatFix()
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.header.frame_id = "gps_link"
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = 48.137154
    msg.longitude = 11.576124
    msg.altitude = 520.0
    msg.position_covariance = [
        0.04, 0.0, 0.0,
        0.0, 0.04, 0.0,
        0.0, 0.0, 0.09,
    ]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


@pytest.mark.launch_test
def generate_test_description():
    navsat_node = launch_ros.actions.Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose_legacy",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "datum_lat": 48.137154,
                "datum_lon": 11.576124,
                "publish_gnss_status": True,
                "gps_protocol": "UBX",
                "gnss_backend": "ublox",
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                navsat_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"navsat_node": navsat_node},
    )


class TestNavSatLegacyStatus(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_navsat_legacy_helper")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_sec: float, message: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            if predicate():
                return
        self.fail(message)

    def test_local_status_path_is_advertised(self) -> None:
        self._spin_until(
            lambda: self.node.count_publishers("/gps/status") == 1
            and self.node.count_subscribers("/diagnostics") == 1,
            timeout_sec=10.0,
            message="legacy GNSS status path was not fully advertised",
        )

    def test_fix_produces_legacy_gnss_status(self) -> None:
        received = []
        status_sub = self.node.create_subscription(
            GnssStatus,
            "/gps/status",
            lambda msg: received.append(msg),
            10,
        )
        fix_pub = self.node.create_publisher(NavSatFix, "/gps/fix", 10)
        diagnostics_pub = self.node.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.addCleanup(self.node.destroy_subscription, status_sub)
        self.addCleanup(self.node.destroy_publisher, fix_pub)
        self.addCleanup(self.node.destroy_publisher, diagnostics_pub)

        self._spin_until(
            lambda: self.node.count_subscribers("/gps/fix") == 1,
            timeout_sec=5.0,
            message="navsat node did not subscribe to /gps/fix",
        )

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp.sec = 123
        diagnostics.header.stamp.nanosec = 456
        diagnostics_pub.publish(diagnostics)
        fix_pub.publish(_make_fix())

        self._spin_until(
            lambda: len(received) > 0,
            timeout_sec=5.0,
            message="legacy /gps/status was not published after a NavSatFix",
        )

        msg = received[-1]
        self.assertEqual(msg.backend, "ublox")
        self.assertTrue(msg.fix_valid)
        self.assertEqual(msg.fix_type, GnssStatus.FIX_TYPE_GPS_FIX)


@launch_testing.post_shutdown_test()
class TestNavSatLegacyStatusShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, navsat_node) -> None:
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=navsat_node,
            allowable_exit_codes=[0],
        )
