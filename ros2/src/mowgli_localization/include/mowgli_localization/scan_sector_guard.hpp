// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// scan_sector_guard.hpp — pure helper deciding whether a LaserScan shows an
// obstacle inside a bearing sector in the direction of motion. Used by
// calibrate_imu_yaw_node to guard its open-loop drive profiles (forward/
// backward cruise, figure-8, dock reverse): those publish on
// /cmd_vel_teleop, which BYPASSES the collision_monitor (teleop twist_mux
// lane), so without this guard the calibration drives blind.
//
// Consumes /scan_collision (self-return + sector-limited dock blanked, NOT
// ground-filtered) — the same conservative stream the collision_monitor
// polls, so anything that would trip PolygonStop/Slow is visible here.
//
// Header-only and node-free — unit-tested in test_scan_sector_guard.cpp.

#ifndef MOWGLI_LOCALIZATION__SCAN_SECTOR_GUARD_HPP_
#define MOWGLI_LOCALIZATION__SCAN_SECTOR_GUARD_HPP_

#include <cmath>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

/// True when @p scan carries at least @p min_points finite returns closer
/// than @p range_m whose BASE-frame bearing lies within ±@p half_angle_rad
/// of @p motion_bearing_rad (0 = robot forward, π = reverse).
///
/// A beam at LIDAR index angle α points along base bearing
/// ψ = α + lidar_yaw (same convention as costmap_scan_filter's ground
/// projection and sector blank). min_points debounces single stray returns
/// — the same rationale as collision_monitor's PolygonStop min_points.
///
/// An empty scan is "not blocked": staleness must be handled by the CALLER
/// (compare the scan's arrival time; a guard that treats "no data" as
/// blocked would deadlock LiDAR-less robots, and one that treats stale data
/// as fresh would trust a scan from before the obstacle appeared).
inline bool sector_blocked(const sensor_msgs::msg::LaserScan& scan,
                           double motion_bearing_rad,
                           double lidar_yaw,
                           double half_angle_rad,
                           double range_m,
                           int min_points = 3)
{
  if (scan.ranges.empty() || range_m <= 0.0)
    return false;
  const double a0 = scan.angle_min;
  const double da = scan.angle_increment;
  int hits = 0;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i)
  {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r >= range_m)
      continue;
    if (r < scan.range_min)
      continue;  // sensor-floor noise, never a real return
    // Angular distance between the beam's base bearing and the motion
    // bearing, wrapped to [0, π].
    double diff = a0 + da * static_cast<double>(i) + lidar_yaw - motion_bearing_rad;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    if (std::fabs(diff) <= half_angle_rad)
    {
      if (++hits >= min_points)
        return true;
    }
  }
  return false;
}

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__SCAN_SECTOR_GUARD_HPP_
