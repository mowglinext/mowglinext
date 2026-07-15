// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// test_scan_sector_guard.cpp — unit tests for sector_blocked()
// (mowgli_localization/scan_sector_guard.hpp), the pure helper behind the
// calibration drive guard: does a LaserScan show >= min_points returns
// within range inside a bearing sector around the motion direction?

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_localization/scan_sector_guard.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using mowgli_localization::sector_blocked;

namespace
{

constexpr double kHalf30Deg = 30.0 * M_PI / 180.0;  // 60° total sector

/// Full-circle scan with `n` beams, all at `fill` metres.
sensor_msgs::msg::LaserScan make_scan_360(std::size_t n, float fill)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = static_cast<float>(-M_PI);
  s.angle_max = static_cast<float>(M_PI);
  s.angle_increment = static_cast<float>(2.0 * M_PI / static_cast<double>(n - 1));
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges.assign(n, fill);
  return s;
}

/// Set the beams whose LIDAR index angle is within ±width of `angle_rad`
/// to `range_m` (a synthetic obstacle subtending 2·width).
void put_obstacle(sensor_msgs::msg::LaserScan& s, double angle_rad, double width, float range_m)
{
  for (std::size_t i = 0; i < s.ranges.size(); ++i)
  {
    double a = s.angle_min + s.angle_increment * static_cast<double>(i);
    double diff = a - angle_rad;
    while (diff > M_PI)
      diff -= 2.0 * M_PI;
    while (diff < -M_PI)
      diff += 2.0 * M_PI;
    if (std::fabs(diff) <= width)
      s.ranges[i] = range_m;
  }
}

}  // namespace

TEST(ScanSectorGuard, ForwardObstacleBlocksForwardNotReverse)
{
  auto s = make_scan_360(360, 5.0f);
  put_obstacle(s, 0.0, 0.1, 0.30f);  // ~11 beams at 0.30 m dead ahead

  EXPECT_TRUE(sector_blocked(s, /*bearing*/ 0.0, /*yaw*/ 0.0, kHalf30Deg, 0.45));
  EXPECT_FALSE(sector_blocked(s, /*bearing*/ M_PI, /*yaw*/ 0.0, kHalf30Deg, 0.45))
      << "a forward obstacle must not block the REVERSE drive";
}

TEST(ScanSectorGuard, RearObstacleBlocksReverse)
{
  auto s = make_scan_360(360, 5.0f);
  put_obstacle(s, M_PI, 0.1, 0.30f);

  EXPECT_TRUE(sector_blocked(s, M_PI, 0.0, kHalf30Deg, 0.45));
  EXPECT_FALSE(sector_blocked(s, 0.0, 0.0, kHalf30Deg, 0.45));
}

TEST(ScanSectorGuard, MountYawRotatesBearings)
{
  // 180°-rotated mount (this robot): an obstacle at LIDAR index angle π is
  // physically in FRONT of the base. With lidar_yaw=π the forward guard
  // must trip and the reverse guard must not.
  auto s = make_scan_360(360, 5.0f);
  put_obstacle(s, M_PI, 0.1, 0.30f);

  EXPECT_TRUE(sector_blocked(s, 0.0, M_PI, kHalf30Deg, 0.45));
  EXPECT_FALSE(sector_blocked(s, M_PI, M_PI, kHalf30Deg, 0.45));
}

TEST(ScanSectorGuard, MinPointsDebouncesStrayReturn)
{
  auto s = make_scan_360(360, 5.0f);
  s.ranges[180] = 0.30f;  // single stray return dead ahead (index 180 ≈ α 0)

  EXPECT_FALSE(sector_blocked(s, 0.0, 0.0, kHalf30Deg, 0.45, /*min_points*/ 3))
      << "one stray return must not trip the guard";
  EXPECT_TRUE(sector_blocked(s, 0.0, 0.0, kHalf30Deg, 0.45, /*min_points*/ 1));
}

TEST(ScanSectorGuard, FarReturnsDoNotBlock)
{
  auto s = make_scan_360(360, 5.0f);
  put_obstacle(s, 0.0, 0.1, 1.50f);  // ahead but beyond the 0.45 m guard range
  EXPECT_FALSE(sector_blocked(s, 0.0, 0.0, kHalf30Deg, 0.45));
}

TEST(ScanSectorGuard, NonFiniteAndSubFloorReturnsIgnored)
{
  auto s = make_scan_360(360, std::numeric_limits<float>::infinity());
  EXPECT_FALSE(sector_blocked(s, 0.0, 0.0, kHalf30Deg, 0.45)) << "all-inf scan is clear";

  // Returns below range_min are sensor-floor noise, never a real object.
  auto s2 = make_scan_360(360, 5.0f);
  put_obstacle(s2, 0.0, 0.1, 0.01f);  // below range_min 0.05
  EXPECT_FALSE(sector_blocked(s2, 0.0, 0.0, kHalf30Deg, 0.45));
}

TEST(ScanSectorGuard, EmptyScanIsClear)
{
  sensor_msgs::msg::LaserScan s;  // no ranges — staleness handled by caller
  EXPECT_FALSE(sector_blocked(s, 0.0, 0.0, kHalf30Deg, 0.45));
}
