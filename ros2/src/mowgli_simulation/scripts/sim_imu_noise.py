#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_imu_noise.py — SIMULATION ONLY.

Adds realistic noise + bias-random-walk to the perfect IMU stream that
the Webots InertialUnit emits, so fusion_graph's gyro between-factor and
the hardware_bridge IMU bias calibration are exercised under conditions
representative of the production MEMS IMU on the real robot.

Noise model
-----------
For each axis (3 gyro, 3 accel):
    measurement = ground_truth + bias + white_noise
    bias        = bias_prev + random_walk_step
    white_noise ~ N(0, white_std)
    random_walk ~ N(0, walk_std * sqrt(dt))

Defaults match a typical MPU-9250 / LIS6DSL-class MEMS:
    gyro:  white_std = 0.005 rad/s   walk_std = 0.0001 rad/s/sqrt(s)
                                      init bias N(0, 0.001 rad/s)
    accel: white_std = 0.05  m/s^2   walk_std = 0.001  m/s^2/sqrt(s)
                                      init bias N(0, 0.05 m/s^2)

These are tunable via parameters; set everything to 0 to disable noise
(useful for an A/B baseline).

Wiring
------
  Webots URDF (urdf_webots/mowgli_webots.urdf): InertialUnit -> /imu/data_sim
  this node:          /imu/data_sim -> /imu/data    (with noise + bias)

  With synthesize_from_applied_twist=true, each /sim/applied_twist sample
  instead supplies both the synthesized gyro value and its native step epoch.
  Raw IMU messages and independently delivered commands are not used in that mode.
  Up to ten source-ordered samples can wait for /clock; overflow drops the
  oldest, matching the transport's KEEP_LAST history depth.

  NOTE: the `input_topic` default below is the stale Gazebo-era
  '/imu/data_gz', which nothing publishes. sim_full_system.launch.py
  always overrides it with '/imu/data_sim'; running this node without
  that override subscribes to a dead topic.

Production code stays unchanged. This is sim-side plumbing.

Safety: read-only consumer of one sim observation topic, publishes a
single sensor topic. No drive commands, no TF, no safety topic.
"""

from __future__ import annotations

from bisect import bisect_left
from copy import deepcopy
import math
import random
from typing import Optional

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


_APPLIED_HISTORY_DEPTH = 10


def _stamp_to_float(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SimImuNoise(Node):

    def __init__(self) -> None:
        super().__init__('sim_imu_noise')

        self._input_topic = str(
            self.declare_parameter('input_topic', '/imu/data_gz').value
        )
        self._output_topic = str(
            self.declare_parameter('output_topic', '/imu/data').value
        )

        # Gyro noise parameters.
        self._gyro_white = float(
            self.declare_parameter('gyro_white_std', 0.005).value
        )
        self._gyro_walk = float(
            self.declare_parameter('gyro_bias_walk_std', 1.0e-4).value
        )
        gyro_init = float(
            self.declare_parameter('gyro_bias_init_std', 1.0e-3).value
        )

        # Accel noise parameters.
        self._accel_white = float(
            self.declare_parameter('accel_white_std', 0.05).value
        )
        self._accel_walk = float(
            self.declare_parameter('accel_bias_walk_std', 1.0e-3).value
        )
        accel_init = float(
            self.declare_parameter('accel_bias_init_std', 0.05).value
        )

        # Reproducibility.
        self._rng = random.Random(
            int(self.declare_parameter('noise_seed', 42).value)
        )

        # Bias state — drawn once at init then random-walked.
        self._gyro_bias = [self._rng.gauss(0.0, gyro_init) for _ in range(3)]
        self._accel_bias = [
            self._rng.gauss(0.0, accel_init) for _ in range(3)
        ]
        self._prev_t: Optional[float] = None

        # KinematicDrive reports the exact twist it integrated, with that
        # step's native epoch. A separately cached command and raw IMU header
        # do not identify the same observation, even when both streams are live.
        self._synth_from_applied = bool(
            self.declare_parameter('synthesize_from_applied_twist', False).value
        )
        self._applied_twist_topic = str(
            self.declare_parameter('applied_twist_topic', '/sim/applied_twist').value
        )
        self._last_applied_stamp_ns: Optional[int] = None
        self._last_applied_ros_now_ns: Optional[int] = None
        self._pending_applied: list[tuple[int, TwistStamped]] = []

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=_APPLIED_HISTORY_DEPTH,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(
            Imu, self._output_topic, sensor_qos
        )
        if self._synth_from_applied:
            self._sub = self.create_subscription(
                TwistStamped, self._applied_twist_topic, self._on_applied_twist, sensor_qos
            )
            # Native samples may precede delivery of the matching /clock tick.
            # Eligibility polling flushes pending samples without
            # changing its epoch or imposing a guessed future tolerance.
            self._eligibility_timer = self.create_timer(0.01, self._flush_pending_applied)
        else:
            self._sub = self.create_subscription(
                Imu, self._input_topic, self._on_imu, sensor_qos
            )

        self._published = 0
        self.create_timer(15.0, self._log_stats)

        self.get_logger().info(
            'sim_imu_noise ready: %s -> %s | '
            'gyro white=%.4f rad/s walk=%.5f init_bias=(%.4f, %.4f, %.4f) | '
            'accel white=%.3f m/s^2 walk=%.4f init_bias=(%.3f, %.3f, %.3f)'
            % (
                self._applied_twist_topic if self._synth_from_applied else self._input_topic,
                self._output_topic,
                self._gyro_white,
                self._gyro_walk,
                *self._gyro_bias,
                self._accel_white,
                self._accel_walk,
                *self._accel_bias,
            )
        )

    def _on_imu(self, msg: Imu) -> None:
        if self._synth_from_applied:
            return
        t = _stamp_to_float(msg.header.stamp)
        if self._prev_t is None:
            dt = 0.01  # first sample: assume ~100 Hz
        else:
            dt = max(0.0, t - self._prev_t)
        self._prev_t = t

        # Random-walk the biases. Step std is walk_std * sqrt(dt).
        if dt > 0.0:
            gyro_step = self._gyro_walk * math.sqrt(dt)
            accel_step = self._accel_walk * math.sqrt(dt)
            for i in range(3):
                self._gyro_bias[i] += self._rng.gauss(0.0, gyro_step)
                self._accel_bias[i] += self._rng.gauss(0.0, accel_step)

        gw = self._gyro_white
        gx = msg.angular_velocity.x + self._gyro_bias[0] + self._rng.gauss(0.0, gw)
        gy = msg.angular_velocity.y + self._gyro_bias[1] + self._rng.gauss(0.0, gw)
        gz = msg.angular_velocity.z + self._gyro_bias[2] + self._rng.gauss(0.0, gw)

        aw = self._accel_white
        ax = msg.linear_acceleration.x + self._accel_bias[0] + self._rng.gauss(0.0, aw)
        ay = msg.linear_acceleration.y + self._accel_bias[1] + self._rng.gauss(0.0, aw)
        az = msg.linear_acceleration.z + self._accel_bias[2] + self._rng.gauss(0.0, aw)

        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        self._publish_sample(out)

    def _check_applied_epoch(self) -> int:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_applied_ros_now_ns is not None and now_ns < self._last_applied_ros_now_ns:
            self._last_applied_stamp_ns = None
            self._pending_applied.clear()
            self._prev_t = None
        self._last_applied_ros_now_ns = now_ns
        return now_ns

    def _flush_pending_applied(self) -> None:
        self._publish_pending_applied(self._check_applied_epoch())

    def _publish_pending_applied(self, now_ns: int) -> None:
        while self._pending_applied and self._pending_applied[0][0] <= now_ns:
            stamp_ns, pending = self._pending_applied[0]
            if self._last_applied_stamp_ns is None or stamp_ns > self._last_applied_stamp_ns:
                self._publish_applied(pending, stamp_ns)
            self._pending_applied.pop(0)

    def _on_applied_twist(self, msg: TwistStamped) -> None:
        now_ns = self._check_applied_epoch()
        self._publish_pending_applied(now_ns)
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        values = (
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
            msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
        )
        if (stamp_ns <= 0 or stamp.nanosec >= 1_000_000_000
                or msg.header.frame_id != 'base_footprint'
                or not all(math.isfinite(value) for value in values)
                or (self._last_applied_stamp_ns is not None
                    and stamp_ns <= self._last_applied_stamp_ns)):
            return
        if stamp_ns > now_ns:
            # Match the DDS KEEP_LAST depth. Preserve distinct source samples
            # in epoch order, including a final applied zero. On overflow,
            # discard the oldest just as the bounded transport history does;
            # retain the newest observations without inventing timestamps.
            position = bisect_left([stamp for stamp, _ in self._pending_applied], stamp_ns)
            if (position == len(self._pending_applied)
                    or self._pending_applied[position][0] != stamp_ns):
                self._pending_applied.insert(position, (stamp_ns, deepcopy(msg)))
                if len(self._pending_applied) > _APPLIED_HISTORY_DEPTH:
                    self._pending_applied.pop(0)
            return
        self._publish_applied(msg, stamp_ns)

    def _publish_applied(self, msg: TwistStamped, stamp_ns: int) -> None:
        out = Imu()
        out.header.stamp = deepcopy(msg.header.stamp)
        out.header.frame_id = 'imu_link'
        out.orientation.w = 1.0
        out.orientation_covariance = [
            1e-6, 0.0, 0.0,
            0.0, 1e-6, 0.0,
            0.0, 0.0, 99.0,
        ]
        out.angular_velocity.z = msg.twist.angular.z
        out.linear_acceleration.z = 9.80665
        self._publish_sample(out)
        self._last_applied_stamp_ns = stamp_ns
        self._prev_t = stamp_ns * 1e-9

    def _publish_sample(self, out: Imu) -> None:
        # Inflate gyro covariance to reflect the added noise so
        # robot_localization weights it appropriately.
        var_g = self._gyro_white * self._gyro_white
        out.angular_velocity_covariance = [
            var_g, 0.0, 0.0,
            0.0, var_g, 0.0,
            0.0, 0.0, var_g,
        ]
        var_a = self._accel_white * self._accel_white
        out.linear_acceleration_covariance = [
            var_a, 0.0, 0.0,
            0.0, var_a, 0.0,
            0.0, 0.0, var_a,
        ]
        self._pub.publish(out)
        self._published += 1

    def _log_stats(self) -> None:
        self.get_logger().info(
            'sim_imu_noise stats: published=%d, '
            'gyro_bias=(%.4f, %.4f, %.4f) rad/s, '
            'accel_bias=(%.3f, %.3f, %.3f) m/s^2'
            % (
                self._published,
                *self._gyro_bias,
                *self._accel_bias,
            )
        )
        self._published = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimImuNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
