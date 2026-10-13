# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
# Ported from the Airseekers ROS 2 port (mower_control/cmd_vel_slew.py DriveShaper).
"""Straight-line shaping of the commanded yaw rate (pure, no ROS).

The Tron's vendor MCU runs a plain speed loop per wheel and no yaw loop, so a
torque imbalance between the two drives makes it pull to one side when asked
to drive straight. These backend-only knobs (``airseekers_angular_trim_radps``,
``airseekers_angular_deadband_radps``, ``airseekers_heading_hold*``) compensate
on the host. Defaults are neutral: no behaviour change until tuned.

* trim: rad/s added to angular.z while |linear.x| > 0.02, signed with linear.x
  (the imbalance pulls the other way when reversing);
* deadband: |angular.z| below it is zeroed while driving (joystick drift);
* heading hold: when the angular command is ~0 while driving
  (|linear.x| > 0.05), lock the IMU yaw and steer back onto it,
  ``kp * yaw_error - kd * gyro_z`` clamped to ``max_radps``. Released on a
  turn command, below the speed, or when the IMU is stale.
"""

from __future__ import annotations

import math

TRIM_MIN_LINEAR = 0.02
HOLD_MIN_LINEAR = 0.05
HOLD_ZERO_ANGULAR = 1e-3


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class DriveShaper:
    def __init__(self, angular_trim_radps: float = 0.0, angular_deadband_radps: float = 0.0,
                 heading_hold: bool = False, heading_hold_kp: float = 1.0,
                 heading_hold_kd: float = 0.1, heading_hold_max_radps: float = 0.15) -> None:
        self.angular_trim_radps = float(angular_trim_radps)
        self.angular_deadband_radps = float(angular_deadband_radps)
        self.heading_hold = bool(heading_hold)
        self.heading_hold_kp = float(heading_hold_kp)
        self.heading_hold_kd = float(heading_hold_kd)
        self.heading_hold_max_radps = float(heading_hold_max_radps)
        self.yaw_ref = None

    @property
    def holding(self) -> bool:
        return self.yaw_ref is not None

    def reset(self) -> None:
        self.yaw_ref = None

    def shape(self, lin: float, ang: float, yaw=None, gyro_z: float = 0.0) -> float:
        """Shaped angular.z for target (lin, ang); ``yaw`` None = no fresh IMU."""
        driving = abs(lin) > TRIM_MIN_LINEAR
        deadband = max(0.0, self.angular_deadband_radps)
        straight = abs(ang) < (deadband if deadband > 0.0 else HOLD_ZERO_ANGULAR)
        if driving and deadband > 0.0 and abs(ang) < deadband:
            ang = 0.0
        if self.heading_hold and straight and abs(lin) > HOLD_MIN_LINEAR and yaw is not None:
            if self.yaw_ref is None:
                self.yaw_ref = yaw
            limit = abs(self.heading_hold_max_radps)
            corr = (self.heading_hold_kp * wrap_angle(self.yaw_ref - yaw)
                    - self.heading_hold_kd * gyro_z)
            ang = max(-limit, min(limit, corr))
        else:
            self.yaw_ref = None
        if driving and self.angular_trim_radps:
            ang += math.copysign(self.angular_trim_radps, lin)
        return ang
