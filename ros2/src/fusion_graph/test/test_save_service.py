# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the real node's save response while deterministic I/O is held."""

import os
from pathlib import Path
import subprocess
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger


def test_save_result_and_executor_progress(tmp_path):
    rclpy.init()
    client_node = rclpy.create_node('checkpoint_service_test')
    prefix = tmp_path / 'graph'
    control = tmp_path / 'io_held'
    environment = dict(os.environ)
    environment['LD_PRELOAD'] = os.environ['CHECKPOINT_TEST_DELAY']
    environment['CHECKPOINT_TEST_CONTROL'] = str(control)
    log = (tmp_path / 'node.log').open('w')
    process = subprocess.Popen([
        os.environ['FUSION_GRAPH_TEST_NODE'], '--ros-args',
        '-p', f'graph_save_prefix:={prefix}', '-p', 'autoload_graph:=false',
        '-p', 'auto_save_enabled:=false', '-p', 'use_lidar_map_anchor:=false',
        '-p', 'tf_broadcast_rate_hz:=0.0',
    ], env=environment, stdout=log, stderr=subprocess.STDOUT)
    poses = []
    subscription = client_node.create_subscription(
        Odometry, '/odometry/filtered_map',
        lambda message: poses.append(message.pose.pose.position.x), 10)
    service = client_node.create_client(Trigger, '/fusion_graph_node/save_graph')
    seed = client_node.create_publisher(
        PoseWithCovarianceStamped, '/fusion_graph_node/set_pose',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wheel = client_node.create_publisher(Odometry, '/wheel_odom', 50)
    imu = client_node.create_publisher(Imu, '/imu/data', 50)

    def pump_until(predicate, timeout=10.0, moving=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'node.log').read_text()
            if moving:
                message = Odometry()
                message.header.stamp = client_node.get_clock().now().to_msg()
                message.twist.twist.linear.x = 0.5
                wheel.publish(message)
                # Source-mode integration needs an IMU baseline and consumed
                # intervals; wheel callbacks alone do not advance its epoch.
                motion = Imu()
                motion.header.stamp = message.header.stamp
                imu.publish(motion)
            rclpy.spin_once(client_node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError((tmp_path / 'node.log').read_text())

    try:
        pump_until(service.service_is_ready)
        empty = service.call_async(Trigger.Request())
        pump_until(empty.done)
        assert not empty.result().success
        message = PoseWithCovarianceStamped()
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01
        seed.publish(message)
        pump_until(lambda: len(poses) >= 3, moving=True)

        saved = service.call_async(Trigger.Request())
        pump_until(control.exists, moving=True)
        assert not saved.done(), 'service acknowledged before I/O completed'
        start = poses[-1]
        # New localization outputs and position progress must continue while
        # the save worker is blocked in real file I/O outside the graph lock.
        pump_until(lambda: poses[-1] > start + 0.05, moving=True)
        busy = service.call_async(Trigger.Request())
        pump_until(busy.done, moving=True)
        assert not busy.result().success
        assert not saved.done()
        control.unlink()
        pump_until(saved.done, moving=True)
        assert saved.result().success
        assert Path(str(prefix) + '.checkpoint').exists()

        # A real publication failure after file sync must reach the response.
        Path(str(prefix) + '.checkpoint.previous').mkdir()
        failed = service.call_async(Trigger.Request())
        pump_until(control.exists, moving=True)
        control.unlink()
        pump_until(failed.done, moving=True)
        assert not failed.result().success
    finally:
        control.unlink(missing_ok=True)
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        log.close()
        client_node.destroy_subscription(subscription)
        client_node.destroy_node()
        rclpy.shutdown()
