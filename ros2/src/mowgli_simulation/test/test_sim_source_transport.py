# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""Exercise native observation delivery through a real ROS executor and clock."""

import importlib.util
import os
from pathlib import Path
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


def test_future_turn_and_final_zero_survive_real_clock_delivery_and_rewind():
    source = Path(os.environ.get(
        'SIM_IMU_SOURCE',
        Path(__file__).resolve().parents[1] / 'scripts' / 'sim_imu_noise.py',
    ))
    spec = importlib.util.spec_from_file_location('imu_transport_under_test', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true',
                     '-p', 'synthesize_from_applied_twist:=true'])
    sensor = module.SimImuNoise()
    probe = rclpy.create_node('source_transport_probe', use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(sensor)
    executor.add_node(probe)
    clock_pub = probe.create_publisher(Clock, '/clock', 10)
    applied_pub = probe.create_publisher(TwistStamped, '/sim/applied_twist', 10)
    outputs = []
    probe.create_subscription(Imu, '/imu/data', outputs.append, qos_profile_sensor_data)

    def until(predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert predicate()

    def pump():
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)

    def clock(stamp_ns):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
        clock_pub.publish(msg)
        until(lambda: sensor.get_clock().now().nanoseconds == stamp_ns)

    def applied(stamp_ns, wz):
        msg = TwistStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = 'base_footprint'
        msg.twist.angular.z = wz
        applied_pub.publish(msg)
        pump()

    def epochs():
        return [msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                for msg in outputs]

    try:
        until(lambda: clock_pub.get_subscription_count() > 0
              and sensor._pub.get_subscription_count() > 0)
        clock(2_000_000_000)
        applied(2_020_000_000, 0.6)
        applied(2_040_000_000, 0.0)
        assert outputs == []
        # No subsequent input packet: a ROS timer must drain the pending final zero.
        clock(2_060_000_000)
        until(lambda: len(outputs) == 2)
        assert epochs() == [2_020_000_000, 2_040_000_000]
        assert [msg.angular_velocity.z for msg in outputs] == [0.6, 0.0]
        applied(3_000_000_000, 9.0)
        clock(1_000_000_000)
        applied(1_020_000_000, -0.2)
        clock(1_040_000_000)
        until(lambda: len(outputs) == 3)
        assert epochs()[-1] == 1_020_000_000
        assert outputs[-1].angular_velocity.z == -0.2
        clock(3_020_000_000)
        pump()
        assert len(outputs) == 3  # pre-rewind pending observation was discarded
    finally:
        executor.shutdown()
        sensor.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
