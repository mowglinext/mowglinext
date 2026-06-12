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
from mowgli_interfaces.msg import AbsolutePose
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import NavSatFix, NavSatStatus


def _make_fix() -> NavSatFix:
    msg = NavSatFix()
    msg.header.stamp.sec = 222
    msg.header.stamp.nanosec = 333
    msg.header.frame_id = "gps_link"
    msg.status.status = NavSatStatus.STATUS_GBAS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = 48.137154
    msg.longitude = 11.576124
    msg.altitude = 520.0
    msg.position_covariance = [
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.04,
    ]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


@pytest.mark.launch_test
def generate_test_description():
    navsat_node = launch_ros.actions.Node(
        package="mowgli_localization",
        executable="navsat_to_absolute_pose_node",
        name="navsat_to_absolute_pose_universal",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "datum_lat": 48.137154,
                "datum_lon": 11.576124,
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


class TestNavSatUniversalStatus(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.node = rclpy.create_node("test_navsat_universal_helper")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.executor.remove_node(cls.node)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)

    def _spin_until(self, predicate, timeout_sec: float, message: str) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.1)
            if predicate():
                return
        self.fail(message)

    def test_pose_outputs_remain_without_local_gnss_status_path(self) -> None:
        self._spin_until(
            lambda: self.node.count_publishers("/gps/absolute_pose") == 1
            and self.node.count_publishers("/gps/pose_cov") == 1,
            timeout_sec=10.0,
            message="navsat node did not advertise its localization outputs",
        )
        self.assertEqual(self.node.count_publishers("/gps/status"), 0)
        self.assertEqual(self.node.count_subscribers("/diagnostics"), 0)

    def test_fix_still_updates_absolute_pose_without_local_status_publish(self) -> None:
        absolute_pose = []
        pose_sub = self.node.create_subscription(
            AbsolutePose,
            "/gps/absolute_pose",
            lambda msg: absolute_pose.append(msg),
            10,
        )
        fix_pub = self.node.create_publisher(NavSatFix, "/gps/fix", 10)
        self.addCleanup(self.node.destroy_subscription, pose_sub)
        self.addCleanup(self.node.destroy_publisher, fix_pub)

        self._spin_until(
            lambda: self.node.count_subscribers("/gps/fix") == 1,
            timeout_sec=5.0,
            message="navsat node did not subscribe to /gps/fix",
        )

        fix_pub.publish(_make_fix())

        self._spin_until(
            lambda: len(absolute_pose) > 0,
            timeout_sec=5.0,
            message="navsat node stopped publishing /gps/absolute_pose in universal mode",
        )
        self._spin_for(1.0)
        self.assertEqual(self.node.count_publishers("/gps/status"), 0)


@launch_testing.post_shutdown_test()
class TestNavSatUniversalStatusShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, navsat_node) -> None:
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=navsat_node,
            allowable_exit_codes=[0],
        )
