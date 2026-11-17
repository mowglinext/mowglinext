# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Independent ROS clocks and executor barriers expose GNSS delivery ordering."""

import time

from geometry_msgs.msg import PoseStamped
from mowgli_interfaces.msg import GnssStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from test_sim_gnss_delivery import load_relay


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class Transport:

    def __init__(self, latency):
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true',
                         '-p', f'delivery_latency_s:={latency}',
                         '-r', '/clock:=/producer_clock'])
        self.sensor = load_relay().SimNavSatRtkFix()
        self.consumer = rclpy.create_node(
            'gnss_delivery_consumer', use_global_arguments=False,
            cli_args=['--ros-args', '-r', '/clock:=/consumer_clock'],
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.driver = rclpy.create_node('gnss_delivery_driver', use_global_arguments=False)
        self.producer_executor = SingleThreadedExecutor()
        self.producer_executor.add_node(self.sensor)
        self.consumer_executor = SingleThreadedExecutor()
        self.consumer_executor.add_node(self.consumer)
        self.pclock = self.driver.create_publisher(Clock, '/producer_clock', 10)
        self.cclock = self.driver.create_publisher(Clock, '/consumer_clock', 10)
        self.truth = self.driver.create_publisher(PoseStamped, '/sim/ground_truth_pose', 10)
        self.raw = self.driver.create_publisher(NavSatFix, '/gps/fix_raw', 10)
        self.fixes, self.statuses, self.deliveries = [], [], []
        self.consumer.create_subscription(NavSatFix, '/gps/fix', self.fixes.append, 10)
        self.consumer.create_subscription(GnssStatus, '/gps/status', self.on_status, 10)
        self.until(lambda: self.pclock.get_subscription_count() == 1
                   and self.cclock.get_subscription_count() == 1
                   and self.truth.get_subscription_count() == 1
                   and self.raw.get_subscription_count() == 1
                   and self.sensor._status_pub.get_subscription_count() == 1
                   and self.sensor._pub.get_subscription_count() == 1,
                   [self.producer_executor, self.consumer_executor])

    def on_status(self, msg):
        now = self.consumer.get_clock().now().nanoseconds
        self.statuses.append(msg)
        self.deliveries.append((msg.position_observation_sequence, stamp_ns(msg.header), now))

    @staticmethod
    def until(predicate, executors):
        deadline = time.monotonic() + 3.0
        while not predicate() and time.monotonic() < deadline:
            for executor in executors:
                executor.spin_once(timeout_sec=0.005)
        assert predicate()

    def clock(self, producer, epoch):
        pub = self.pclock if producer else self.cclock
        node = self.sensor if producer else self.consumer
        executor = self.producer_executor if producer else self.consumer_executor
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(epoch, 1_000_000_000)
        pub.publish(msg)
        self.until(lambda: node.get_clock().now().nanoseconds == epoch, [executor])

    def acquire(self, epoch):
        truth = PoseStamped()
        truth.header.frame_id = 'map'
        truth.header.stamp.sec, truth.header.stamp.nanosec = divmod(epoch, 1_000_000_000)
        truth.pose.orientation.w = 1.0
        self.truth.publish(truth)
        self.until(lambda: self.sensor._last_truth_stamp_ns == epoch, [self.producer_executor])
        raw = NavSatFix()
        raw.header.frame_id = 'gps_link'
        raw.header.stamp = truth.header.stamp
        self.raw.publish(raw)
        self.until(lambda: self.sensor._last_fix_stamp_ns == epoch, [self.producer_executor])

    def consume(self, count):
        self.until(lambda: len(self.statuses) == count and len(self.fixes) == count,
                   [self.producer_executor, self.consumer_executor])

    def close(self):
        self.producer_executor.shutdown()
        self.consumer_executor.shutdown()
        self.sensor.destroy_node()
        self.consumer.destroy_node()
        self.driver.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('epoch', [68_240_000_000, 240_660_000_000])
@pytest.mark.parametrize('latency,consumer_offset', [(0.0, -20_000_000), (0.02, 0), (0.02, -1)])
def test_independent_clocks_preserve_source_epoch_at_delivery(epoch, latency, consumer_offset):
    fixture = Transport(latency)
    try:
        # Establish prior healthy Fixed authority, then recreate the captured
        # producer-ahead phase using genuinely independent ROS clock endpoints.
        previous = epoch - 200_000_000
        fixture.clock(False, previous)
        fixture.clock(True, previous)
        fixture.acquire(previous)
        fixture.clock(True, previous + 20_000_000)
        fixture.consume(1)
        assert fixture.statuses[0].rtk_mode == GnssStatus.RTK_MODE_FIXED
        assert fixture.deliveries[0] == (1, previous, previous)
        fixture.clock(False, epoch - 20_000_000)
        fixture.clock(True, epoch)
        fixture.acquire(epoch)
        if latency:
            assert len(fixture.statuses) == 1
            assert len(fixture.sensor._pending_deliveries) == 1
            # The timer, rather than a subsequent GPS callback, releases final
            # pending data. Clock callbacks are explicit barriers at each edge.
            fixture.clock(True, epoch + 19_999_999)
            assert len(fixture.sensor._pending_deliveries) == 1
            fixture.clock(False, epoch + consumer_offset)
            fixture.clock(True, epoch + 20_000_000)
        fixture.consume(2)
        assert fixture.deliveries[-1] == (2, epoch, epoch + consumer_offset)
        assert fixture.fixes[-1].header == fixture.statuses[-1].header
        assert fixture.statuses[-1].rtk_mode == GnssStatus.RTK_MODE_FIXED
        assert fixture.sensor._position_observation_sequence == 2
        assert not fixture.sensor._pending_deliveries
    finally:
        fixture.close()


def test_rewind_and_catch_up_clear_pending_without_recycling_sequence():
    fixture = Transport(0.02)
    try:
        epoch = 2_000_000_000
        fixture.clock(False, epoch)
        fixture.clock(True, epoch)
        fixture.acquire(epoch)
        assert len(fixture.sensor._pending_deliveries) == 1
        # No acquisition/timer callbacks between these two direct clock changes:
        # the rcl jump callback must invalidate the pending delivery immediately.
        from rclpy.time import Time
        fixture.sensor.get_clock().set_ros_time_override(Time(nanoseconds=1_000_000_000))
        fixture.sensor.get_clock().set_ros_time_override(Time(nanoseconds=epoch))
        assert not fixture.sensor._pending_deliveries
        assert fixture.sensor._ground_truth is None
        fixture.clock(True, epoch + 20_000_000)
        fixture.acquire(epoch + 20_000_000)
        fixture.clock(False, epoch + 20_000_000)
        fixture.clock(True, epoch + 40_000_000)
        fixture.consume(1)
        assert fixture.statuses[0].position_observation_sequence == 2
        fixture.clock(True, epoch + 1_000_000_000)
        assert not fixture.sensor._pending_deliveries
        assert len(fixture.statuses) == 1
    finally:
        fixture.close()
