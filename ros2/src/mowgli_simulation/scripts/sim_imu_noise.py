#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
sim_imu_noise.py — SIMULATION ONLY.

Adds realistic noise + bias-random-walk to the perfect IMU stream that
Gazebo's gz-sim-imu-system emits, so the dual-EKF, slam_toolbox motion
prior, and slam_pose_anchor_node are exercised under conditions
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
  gazebo_bridge.yaml: ros_topic_name=/imu/data_gz   (was /imu/data)
  this node:          /imu/data_gz -> /imu/data     (with noise + bias)

Production code stays unchanged. This is sim-side plumbing.

Safety: read-only consumer of one Gazebo-bridged topic, publishes a
single sensor topic. No drive commands, no TF, no safety topic.
"""

from __future__ import annotations

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


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

        # Optional perfect-IMU mode (sim only). When enabled, the script
        # ignores the Webots-side gyro/accel readings entirely and emits
        # an IMU synthesized from the latest /cmd_vel: gyro_z = cmd.az,
        # gx/gy = 0, accel = (0, 0, g) in IMU frame, orientation = identity.
        # Webots' kinematic teleport leaks ODE physics noise (~0.03 rad/s
        # gyro_z drift) between teleport ticks; with EKF process noise on
        # yaw at 0.06, this bias accumulates into a 5°/s yaw drift on
        # /odometry/filtered_map even when the robot is stationary, which
        # makes obstacles drift on the map and FTC PRE_ROTATE diverge.
        # This mode bypasses that by using cmd_vel as the source of truth.
        self._synth_from_cmd = bool(
            self.declare_parameter('synthesize_from_cmd_vel', False).value
        )
        self._cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        )
        self._cmd_az = 0.0  # latest commanded yaw rate (rad/s)
        if self._synth_from_cmd:
            from geometry_msgs.msg import TwistStamped

            def _on_cmd(msg: TwistStamped) -> None:
                self._cmd_az = float(msg.twist.angular.z)

            self.create_subscription(TwistStamped, self._cmd_vel_topic, _on_cmd, 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(
            Imu, self._output_topic, sensor_qos
        )
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
                self._input_topic,
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

        if self._synth_from_cmd:
            # Perfect IMU from cmd_vel — see comment in __init__.
            gx = 0.0
            gy = 0.0
            gz = self._cmd_az
            ax = 0.0
            ay = 0.0
            az = 9.80665  # gravity in IMU body Z (robot is upright)
        else:
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
        if self._synth_from_cmd:
            # Identity orientation — robot is perfectly upright in sim.
            out.orientation.x = 0.0
            out.orientation.y = 0.0
            out.orientation.z = 0.0
            out.orientation.w = 1.0
            # Zero covariance on roll/pitch so EKF trusts them; loose
            # yaw cov per the existing /imu/data convention.
            out.orientation_covariance = [
                1e-6, 0.0, 0.0,
                0.0, 1e-6, 0.0,
                0.0, 0.0, 99.0,  # yaw — keep loose, EKF uses gyro_z
            ]
        else:
            out.orientation = msg.orientation
            out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz
        # Inflate gyro covariance to reflect the added noise so
        # robot_localization weights it appropriately.
        var_g = self._gyro_white * self._gyro_white
        out.angular_velocity_covariance = [
            var_g, 0.0, 0.0,
            0.0, var_g, 0.0,
            0.0, 0.0, var_g,
        ]
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
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
