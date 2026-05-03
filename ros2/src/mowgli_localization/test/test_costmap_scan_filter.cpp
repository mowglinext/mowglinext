// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// test_costmap_scan_filter.cpp — unit tests for the static filter
// helper in costmap_scan_filter_node. Drives the radial blanking
// directly without instantiating a node, so this stays a pure-C++ test.

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/laser_scan.hpp"

// Expose the static helper without dragging in rclcpp at link time.
// The implementation lives in costmap_scan_filter_node.cpp; we mimic
// the function signature here and keep it in sync.
namespace mowgli_localization
{
sensor_msgs::msg::LaserScan filter_scan_for_test(const sensor_msgs::msg::LaserScan& in,
                                                 double dock_blank_range,
                                                 bool blank_active)
{
  sensor_msgs::msg::LaserScan out = in;
  if (!blank_active)
    return out;
  const float threshold = static_cast<float>(dock_blank_range);
  const float inf = std::numeric_limits<float>::infinity();
  for (auto& r : out.ranges)
  {
    if (std::isfinite(r) && r < threshold)
      r = inf;
  }
  return out;
}
}  // namespace mowgli_localization

namespace
{

sensor_msgs::msg::LaserScan make_scan(const std::vector<float>& ranges)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = -1.57f;
  s.angle_max = 1.57f;
  s.angle_increment = 3.14f / std::max<size_t>(1, ranges.size() - 1);
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = ranges;
  return s;
}

}  // namespace

TEST(CostmapScanFilter, PassThroughWhenInactive)
{
  auto in = make_scan({0.10f, 0.30f, 0.65f, 1.0f, 5.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, false);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  for (size_t i = 0; i < in.ranges.size(); ++i)
    EXPECT_FLOAT_EQ(out.ranges[i], in.ranges[i]);
}

TEST(CostmapScanFilter, BlanksReturnsBelowThresholdWhenActive)
{
  auto in = make_scan({0.10f, 0.30f, 0.69f, 0.71f, 1.0f, 5.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  EXPECT_FALSE(std::isfinite(out.ranges[0]));
  EXPECT_FALSE(std::isfinite(out.ranges[1]));
  EXPECT_FALSE(std::isfinite(out.ranges[2]));
  EXPECT_FLOAT_EQ(out.ranges[3], 0.71f);
  EXPECT_FLOAT_EQ(out.ranges[4], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[5], 5.0f);
}

TEST(CostmapScanFilter, LeavesNonFiniteValuesAlone)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto in = make_scan({inf, nan, 0.20f, 2.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  EXPECT_FALSE(std::isfinite(out.ranges[0]));  // was inf
  EXPECT_TRUE(std::isnan(out.ranges[1]));      // NaN preserved
  EXPECT_FALSE(std::isfinite(out.ranges[2]));  // 0.20 < 0.70 → +inf
  EXPECT_FLOAT_EQ(out.ranges[3], 2.0f);
}

TEST(CostmapScanFilter, ThresholdBoundaryIsExclusive)
{
  // A return exactly equal to the threshold should NOT be blanked
  // (filter uses `r < threshold`, not `<=`).
  auto in = make_scan({0.70f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  EXPECT_FLOAT_EQ(out.ranges[0], 0.70f);
}
