# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0
# From the Airseekers ROS 2 port (mower_sim/kinematics.py), reduced to the
# drive model: no world, obstacles or dock geometry.
"""Diff-drive model with the Tron MCU's drive behaviour (no ROS).

* linear and angular are clamped SEPARATELY to +-0.3 m/s and +-0.3 rad/s
  (not curvature preserving);
* while TURNING, a wheel asked for less than ~0.06 m/s does not turn (the
  drive's speed floor); straight driving is not floored;
* the wheels lag the command (first-order, ``wheel_tau_s``).
The command is whatever setpoint the MCU holds (see airseekers_emulator.Mcu).
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def _clamp(v: float, lim: float) -> float:
    return max(-lim, min(lim, v))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class DriveParams:
    linear_max: float = 0.3
    angular_max: float = 0.3
    track_width: float = 0.48
    min_wheel_speed: float = 0.06
    floor_straight: bool = False
    wheel_tau_s: float = 0.15


def wheel_speeds(v: float, w: float, p: DriveParams):
    """(left, right) after the MCU clamp and speed floor."""
    v = _clamp(v, p.linear_max)
    w = _clamp(w, p.angular_max)
    half = p.track_width / 2.0
    left, right = v - w * half, v + w * half
    turning = abs(left - right) > 1e-6
    if p.min_wheel_speed > 0.0 and (turning or p.floor_straight):
        left = 0.0 if abs(left) < p.min_wheel_speed else left
        right = 0.0 if abs(right) < p.min_wheel_speed else right
    return left, right


class Plant:
    def __init__(self, params: DriveParams = None):
        self.p = params or DriveParams()
        self.x = self.y = self.yaw = 0.0
        self.left = self.right = 0.0
        self.v = self.w = 0.0

    def step(self, dt: float, v_cmd: float, w_cmd: float) -> None:
        if dt <= 0.0:
            return
        tl, tr = wheel_speeds(v_cmd, w_cmd, self.p)
        a = 1.0 if self.p.wheel_tau_s <= 0.0 else min(1.0, dt / self.p.wheel_tau_s)
        self.left += a * (tl - self.left)
        self.right += a * (tr - self.right)
        if abs(self.left) < 1e-4:
            self.left = 0.0
        if abs(self.right) < 1e-4:
            self.right = 0.0
        self.v = 0.5 * (self.left + self.right)
        self.w = (self.right - self.left) / self.p.track_width
        mid = self.yaw + 0.5 * self.w * dt
        self.x += self.v * dt * math.cos(mid)
        self.y += self.v * dt * math.sin(mid)
        self.yaw = wrap(self.yaw + self.w * dt)
