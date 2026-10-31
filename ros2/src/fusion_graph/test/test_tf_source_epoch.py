# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise source and predicted TF through the real localizer and tf2 buffer."""

import os
import subprocess
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


@pytest.mark.parametrize('broadcast_hz,lead_s', [(20.0, 0.1), (0.0, 0.1), (20.0, 0.0)])
def test_source_and_forecast_tf(tmp_path, broadcast_hz, lead_s):
    rclpy.init()
    node = rclpy.create_node('tf_source_epoch_test')
    node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    buffer = Buffer(cache_time=Duration(seconds=20), node=node)
    knots = {}
    batches = []

    def on_tf(message):
        batch = []
        for transform in message.transforms:
            stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
            key = (transform.header.frame_id, transform.child_frame_id, stamp)
            batch.append(key)
            buffer.set_transform(transform, 'fusion_graph_test')
            if key[:2] == ('odom', 'base_footprint'):
                knots[stamp] = transform.transform.translation.x
        batches.append(batch)

    node.create_subscription(TFMessage, '/tf', on_tf, 100)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    wheel_pub = node.create_publisher(Odometry, '/wheel_odom', 50)
    imu_pub = node.create_publisher(
        Imu, '/imu/data', QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
    seed_pub = node.create_publisher(
        PoseWithCovarianceStamped, '/fusion_graph_node/set_pose',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log_path = tmp_path / 'fusion_graph.log'
    log = log_path.open('w')
    args = [
        os.environ['FUSION_GRAPH_TEST_NODE'], '--ros-args',
        '-p', 'use_sim_time:=true', '-p', 'primary_mode:=true',
        '-p', f'tf_publish_lead_s:={lead_s}', '-p', f'tf_broadcast_rate_hz:={broadcast_hz}',
        '-p', 'node_period_s:=0.04', '-p', 'autoload_graph:=false',
        '-p', 'auto_save_enabled:=false', '-p', 'use_lidar_map_anchor:=false',
        '-p', f'graph_save_prefix:={tmp_path / "graph"}',
    ]
    process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
    lead_ns = round(lead_s * 1e9)
    # Inline publication runs at the 40 ms timer cadence; advance the clock
    # once after each consumed sample so that timer publishes that sample.
    step_ns = 40_000_000 if broadcast_hz == 0 else 20_000_000
    base_ns = 100_000_000_000

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, end - time.monotonic())))

    def wait_for(predicate, description, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            spin_for(0.02)
        pytest.fail(f'Timed out waiting for {description}: {log_path.read_text()}')

    def publish_clock(stamp):
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(stamp, 1_000_000_000)
        clock_pub.publish(message)
        spin_for(0.06)

    def publish_motion(stamp, speed):
        odom = Odometry()
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        odom.twist.twist.linear.x = speed
        wheel_pub.publish(odom)
        # Node integrates the held wheel sample when the IMU callback runs.
        spin_for(0.06)
        imu = Imu()
        imu.header.stamp = odom.header.stamp
        imu.angular_velocity.z = 0.0
        imu_pub.publish(imu)

    def check_sample(stamp, expected_x, speed, anchored):
        if broadcast_hz == 0:
            spin_for(0.06)
            publish_clock(stamp + step_ns)
        forecast_stamp = stamp + lead_ns
        wait_for(lambda: forecast_stamp in knots, f'TF forecast at {forecast_stamp}')
        assert knots[forecast_stamp] == pytest.approx(expected_x + speed * lead_s, abs=1e-7)
        if not anchored:
            return
        source = buffer.lookup_transform(
            'odom', 'base_footprint', Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
        assert source.transform.translation.x == pytest.approx(expected_x, abs=1e-7)
        expected_stamps = {stamp, forecast_stamp}

        # Both TF legs, including the anchor, must be from one capture.
        def paired_batch():
            for batch in batches:
                odom = {v[2] for v in batch if v[:2] == ('odom', 'base_footprint')}
                anchor = {v[2] for v in batch if v[:2] == ('map', 'odom')}
                if odom == expected_stamps and anchor == expected_stamps:
                    assert len(batch) == len(set(batch))
                    return True
            return False

        wait_for(paired_batch, 'paired map/odom source and forecast TF')
        for point in expected_stamps:
            query = Time(nanoseconds=point, clock_type=ClockType.ROS_TIME)
            anchor = buffer.lookup_transform('map', 'odom', query)
            local = buffer.lookup_transform('odom', 'base_footprint', query)
            mapped = buffer.lookup_transform('map', 'base_footprint', query)
            # The explicit zero-yaw seed and zero gyro make the anchor's
            # orientation identity; compare the full TF composition.
            assert anchor.transform.rotation.z == pytest.approx(0.0, abs=1e-7)
            assert mapped.transform.translation.x == pytest.approx(
                anchor.transform.translation.x + local.transform.translation.x, abs=1e-7)
            assert mapped.transform.translation.y == pytest.approx(
                anchor.transform.translation.y + local.transform.translation.y, abs=1e-7)

    try:
        wait_for(lambda: wheel_pub.get_subscription_count() > 0 and
                 imu_pub.get_subscription_count() > 0 and seed_pub.get_subscription_count() > 0,
                 'localizer subscriptions', timeout=10.0)
        for tick in range(3):
            stamp = base_ns + tick * step_ns
            publish_clock(stamp)
            publish_motion(stamp, 0.0)
            if tick > 0:
                check_sample(stamp, 0.0, 0.0, False)
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = 'map'
        seed.header.stamp.sec, seed.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        seed.pose.pose.orientation.w = 1.0
        seed.pose.covariance[0] = seed.pose.covariance[7] = seed.pose.covariance[35] = 0.01
        seed_pub.publish(seed)
        for tick in range(3, 6):
            stamp = base_ns + tick * step_ns
            publish_clock(stamp)
            publish_motion(stamp, 0.0)
            check_sample(stamp, 0.0, 0.0, False)
        wait_for(lambda: 'bootstrap init from /set_pose' in log_path.read_text(), 'seed')
        source_x = 0.0
        for step in range(1, 21):
            stamp = base_ns + (5 + step) * step_ns
            speed = (step if step <= 10 else 20 - step) * 0.02
            source_x += speed * step_ns * 1e-9
            publish_clock(stamp)
            publish_motion(stamp, speed)
            check_sample(stamp, source_x, speed, True)
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
