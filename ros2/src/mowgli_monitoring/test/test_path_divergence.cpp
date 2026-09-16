// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the path-space divergence detector. ROS-free.
// The field traces quoted here are from the 2026-09-16 mows.

#include <cmath>
#include <cstdint>

#include "gtest/gtest.h"
#include "mowgli_interfaces/path_divergence.hpp"

namespace
{

using mowgli_interfaces::path_divergence::Config;
using mowgli_interfaces::path_divergence::Detector;

/// Feed `n` samples at 10 Hz, the controller_server rate.
void Feed(Detector& d,
          double start_t,
          int n,
          double error0,
          double d_error,
          std::uint32_t index0,
          std::uint32_t d_index)
{
  for (int i = 0; i < n; ++i)
  {
    d.Update(error0 + i * d_error, index0 + i * d_index, start_t + i * 0.1);
  }
}

TEST(PathDivergence, HealthyTrackingNeverTriggers)
{
  // Field: FTC holds +/-0.02 m with the index climbing steadily.
  Detector d;
  for (int i = 0; i < 200; ++i)
  {
    d.Update((i % 2 == 0) ? 0.018 : -0.021, static_cast<std::uint32_t>(i), i * 0.1);
  }
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, DeliberateObstacleSkirtNeverTriggers)
{
  // A skirt grows the error on purpose — up to max_lateral_deviation — but the
  // robot keeps advancing ALONG the path while offset from it.
  Detector d;
  Feed(d, 0.0, 100, 0.02, 0.006, 0, 2);  // error 0.02 -> 0.62 m, index +2/sample
  EXPECT_FALSE(d.Triggered()) << "growth alone must not fire: that is every avoidance episode";
}

TEST(PathDivergence, HeldByAnObstacleTriggers)
{
  // The collision signature: index frozen, error climbing.
  Detector d;
  Feed(d, 0.0, 60, 0.14, 0.01, 655, 0);  // 6 s, error 0.14 -> 0.73 m, index fixed
  EXPECT_TRUE(d.Triggered());
}

TEST(PathDivergence, StoppedRobotWithConstantErrorNeverTriggers)
{
  // An obstacle HOLD parks the robot: index frozen, but the error is constant.
  // Only growth counts, otherwise every wait-for-costmap would abort the strip.
  Detector d;
  Feed(d, 0.0, 100, 0.35, 0.0, 400, 0);
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, ErrorComingBackDownReAnchors)
{
  // Recovering towards the line must clear the episode, not bank the growth.
  Detector d;
  Feed(d, 0.0, 30, 0.10, 0.01, 500, 0);  // climbing, 3 s — window not closed
  Feed(d, 3.0, 30, 0.40, -0.01, 500, 0);  // coming back down
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, NeedsTheFullWindowBeforeReporting)
{
  Detector d;
  Feed(d, 0.0, 20, 0.10, 0.03, 700, 0);  // 2 s only, but 0.60 m of growth
  EXPECT_FALSE(d.Triggered()) << "a 2 s burst must not beat the window";
  Feed(d, 2.0, 30, 0.70, 0.001, 700, 0);  // carry on to close the 4 s window
  EXPECT_TRUE(d.Triggered());
}

TEST(PathDivergence, ABoundedReverseEscapeCannotCloseTheWindowAlone)
{
  // The escape is capped at 0.30 m at 0.15 m/s, so about 2 s of motion.
  Detector d;
  Feed(d, 0.0, 20, 0.20, 0.01, 640, 0);
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, NewGoalReAnchorsInsteadOfMixingEpisodes)
{
  Detector d;
  Feed(d, 0.0, 30, 0.10, 0.02, 655, 0);
  // New goal: index restarts near 0 and the clock keeps running.
  d.Update(0.05, 0, 3.1);
  Feed(d, 3.2, 20, 0.05, 0.0, 0, 1);
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, NonFiniteSamplesAreIgnored)
{
  Detector d;
  Feed(d, 0.0, 60, 0.14, 0.01, 655, 0);
  ASSERT_TRUE(d.Triggered());
  d.Reset();
  d.Update(std::nan(""), 655, 7.0);
  d.Update(0.5, 655, std::nan(""));
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, ResetClearsTheVerdict)
{
  Detector d;
  Feed(d, 0.0, 60, 0.14, 0.01, 655, 0);
  ASSERT_TRUE(d.Triggered());
  d.Reset();
  EXPECT_FALSE(d.Triggered());
}

TEST(PathDivergence, SlowIndexCreepWithinToleranceStillCounts)
{
  // A wheel-slipping robot can still nudge the index by a pose or two; that is
  // not progress, and max_index_advance covers it.
  Detector d;
  Config cfg;
  Detector tolerant(cfg);
  for (int i = 0; i < 60; ++i)
  {
    tolerant.Update(0.14 + i * 0.01, 655 + static_cast<std::uint32_t>(i / 40), i * 0.1);
  }
  EXPECT_TRUE(tolerant.Triggered());
}

}  // namespace
