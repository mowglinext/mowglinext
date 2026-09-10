// Copyright (C) 2026 MowgliNext contributors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include "mowgli_map/map_server_node.hpp"
#include "mowgli_map/mow_progress.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_map::GetMowProgressInhibitReason;
using mowgli_map::MowProgressInhibitReason;

TEST(MowProgress, RequiresFreshVerifiedBladeActivity)
{
  EXPECT_EQ(GetMowProgressInhibitReason(true, true, true, 3000.0, 1000.0),
            MowProgressInhibitReason::kActive);
  EXPECT_EQ(GetMowProgressInhibitReason(true, true, false, 3000.0, 1000.0),
            MowProgressInhibitReason::kBladeInactive);
  EXPECT_EQ(GetMowProgressInhibitReason(true, true, true, 999.0, 1000.0),
            MowProgressInhibitReason::kRpmTooLow);
  EXPECT_EQ(GetMowProgressInhibitReason(true, false, true, 3000.0, 1000.0),
            MowProgressInhibitReason::kTelemetryMissingOrStale);
  EXPECT_EQ(GetMowProgressInhibitReason(false, true, true, 3000.0, 1000.0),
            MowProgressInhibitReason::kBladeNotRequested);
}

TEST(MowProgress, SweepsEveryGridCellAlongStraightMotion)
{
  EXPECT_EQ(mowgli_map::SweepStepCount(0.0, 0.05), 1U);
  EXPECT_EQ(mowgli_map::SweepStepCount(0.05, 0.05), 1U);
  EXPECT_EQ(mowgli_map::SweepStepCount(0.21, 0.05), 5U);
}

TEST(MowProgress, StampsTheCompleteStraightToolSweep)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.05);
  options.append_parameter_override("map_size_x", 4.0);
  options.append_parameter_override("map_size_y", 4.0);
  options.append_parameter_override("tool_width", 0.10);
  auto node = std::make_shared<mowgli_map::MapServerNode>(options);

  node->stamp_mow_progress_for_test(-1.0, 0.0);
  node->stamp_mow_progress_for_test(1.0, 0.0);

  EXPECT_FLOAT_EQ(node->mow_progress_value_for_test(-1.0, 0.0), 100.0F);
  EXPECT_FLOAT_EQ(node->mow_progress_value_for_test(0.0, 0.0), 100.0F);
  EXPECT_FLOAT_EQ(node->mow_progress_value_for_test(1.0, 0.0), 100.0F);

  node.reset();
  rclcpp::shutdown();
}

TEST(MowProgress, DoesNotSweepAcrossAProgressReset)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.append_parameter_override("resolution", 0.05);
  options.append_parameter_override("map_size_x", 4.0);
  options.append_parameter_override("map_size_y", 4.0);
  options.append_parameter_override("tool_width", 0.10);
  auto node = std::make_shared<mowgli_map::MapServerNode>(options);

  node->stamp_mow_progress_for_test(-1.0, 0.0);
  node->clear_map_layers();
  node->stamp_mow_progress_for_test(1.0, 0.0);

  EXPECT_FLOAT_EQ(node->mow_progress_value_for_test(0.0, 0.0), 0.0F);
  EXPECT_FLOAT_EQ(node->mow_progress_value_for_test(1.0, 0.0), 100.0F);

  node.reset();
  rclcpp::shutdown();
}

TEST(MowProgress, CachesChangedCoverageAndInvalidatesItOnReset)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.append_parameter_override("mow_progress_publish_period_s", 0.0);
  auto node = std::make_shared<mowgli_map::MapServerNode>(options);

  node->stamp_mow_progress_for_test(0.0, 0.0);
  node->publish_mow_progress_for_test();
  EXPECT_TRUE(node->mow_progress_cache_valid_for_test());

  // No coverage changed, but the timer may still republish the cached grid for
  // a reconnecting GUI. The cache remains valid and requires no rebuild.
  node->publish_mow_progress_for_test();
  EXPECT_TRUE(node->mow_progress_cache_valid_for_test());

  {
    std::lock_guard<std::mutex> lock(node->map_mutex());
    node->clear_map_layers();
  }
  EXPECT_FALSE(node->mow_progress_cache_valid_for_test());

  node.reset();
  rclcpp::shutdown();
}

}  // namespace
