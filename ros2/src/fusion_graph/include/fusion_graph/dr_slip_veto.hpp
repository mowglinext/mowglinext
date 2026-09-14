// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Dead-reckoning slip veto, factored out of OnImu so it is unit-testable
// without ROS. See fusion_graph_node.hpp for the design rationale: when the
// wheel encoders claim a yaw rate the chassis gyro does not corroborate, the
// wheels are skating and their forward velocity is a fiction that must not
// accumulate into dr_x_/dr_y_.
//
// The subtlety this header exists to document: `wheel_wz` is not a continuous
// measurement. /wheel_odom derives it from a tick DIFFERENCE over one
// aggregation window (odometry_publisher.cpp),
//
//     wheel_wz = (d_right_m - d_left_m) / wheel_track / dt
//
// so its resolution is one encoder tick of left/right asymmetry:
//
//     1 LSB = (1 / ticks_per_meter) / wheel_track / dt
//           = (1 / 280.44) / 0.325 / 0.066 = 0.166 rad/s   (typical 66 ms window)
//           = (1 / 280.44) / 0.325 / 0.050 = 0.219 rad/s   (shortest 50 ms window)
//
// Any threshold at or below ~0.22 rad/s therefore cannot distinguish "one tick
// of encoder rounding" from "the wheels are skating". Issue #488: with the
// original 0.15 rad/s threshold the veto fired on 32-45 % of windows during a
// slow STRAIGHT reverse (gyro ~0, ±1 tick asymmetry is the norm at 3 ticks per
// wheel per window), zeroing that fraction of the translation. odom→base_footprint
// then under-reported travel by ~25-30 %, and Nav2's BackUp — which measures
// odom-frame displacement — drove 2.1 m for a 1.50 m command.
//
// The failure this veto was written for (2026-05-27, odom→base reaching 74 m on
// a 10 m² lawn) is a skating pivot, where wheel_wz is order 1-3 rad/s. A
// threshold above the 2-LSB floor keeps that caught with a wide margin.

#pragma once

#include <cmath>

namespace fusion_graph
{

// Shipped thresholds. Defined here rather than at the member declaration so
// the unit test can assert them against this robot's quantization floor
// without pulling in ROS/GTSAM (see test_dr_slip_veto.cpp).
//
// kDrSlipWheelMinDefaultRadPerS clears 2 LSB at the shortest /wheel_odom
// aggregation window on this drivetrain (2 x 0.219 = 0.439 rad/s); the skating
// pivot the veto exists for runs 1-3 rad/s. The gyro rate needs no such floor —
// it is a continuous 91 Hz measurement, not a tick difference.
inline constexpr double kDrSlipWheelMinDefaultRadPerS = 0.44;
inline constexpr double kDrSlipGyroMaxDefaultRadPerS = 0.15;

// Quantization floor of the wheel-derived yaw rate: the |wheel_wz| produced by
// a single encoder tick of left/right asymmetry over one aggregation window.
// Below this, |wheel_wz| carries no information about chassis rotation.
inline double WheelYawRateQuantumRadPerS(double ticks_per_meter,
                                         double wheel_track_m,
                                         double window_s)
{
  if (ticks_per_meter <= 0.0 || wheel_track_m <= 0.0 || window_s <= 0.0)
    return 0.0;
  return (1.0 / ticks_per_meter) / wheel_track_m / window_s;
}

// True when the wheels claim a yaw rate the gyro does not corroborate, i.e. the
// chassis is being skated rather than driven and wheel_vx is phantom.
//
// All three conditions must hold: the wheels and the gyro must disagree, the
// gyro must read near-still (a coordinated turn is never vetoed), and the
// wheels must claim a rate large enough to be real rather than quantization.
inline bool DrSlipVetoed(double wheel_wz,
                         double gyro_wz,
                         double wheel_min_rad_per_s,
                         double gyro_max_rad_per_s)
{
  return std::abs(wheel_wz - gyro_wz) > wheel_min_rad_per_s &&
         std::abs(gyro_wz) < gyro_max_rad_per_s && std::abs(wheel_wz) > wheel_min_rad_per_s;
}

}  // namespace fusion_graph
