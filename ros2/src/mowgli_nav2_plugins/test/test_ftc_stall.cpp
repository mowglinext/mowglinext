// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for FTC's anti-wheelspin stall decision. Pure logic, no ROS.

#include "mowgli_nav2_plugins/ftc_stall.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

TEST(StallDecision, DisabledWhenRatioIsZero)
{
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;
  cfg.stall_speed_ratio = 0.0;  // disabled

  // Even a total stall (measured=0 while commanding 0.5) must pass through
  // unmodified when the feature is off.
  const auto out = mnp::StallDecision(
      /*target_speed=*/0.5, /*cmd_speed=*/0.5, /*measured_fwd=*/0.0, /*dt=*/1.0, cfg, stall_time);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.5);
  EXPECT_FALSE(out.in_stall);
  EXPECT_DOUBLE_EQ(stall_time, 0.0);
}

TEST(StallDecision, GoodTractionNeverAccumulatesStallTime)
{
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;  // defaults: ratio 0.35, grace 0.6 s, crawl 0.08.

  // Measured forward speed tracks the commanded speed closely — no slip.
  const auto out = mnp::StallDecision(0.5, 0.5, 0.48, 0.1, cfg, stall_time);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.5);
  EXPECT_FALSE(out.in_stall);
  EXPECT_DOUBLE_EQ(stall_time, 0.0);
}

TEST(StallDecision, BriefStallUnderGraceDoesNotEaseSpeed)
{
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  // Wheels slipping (measured << commanded), but only 0.3 s so far — below
  // the 0.6 s grace period.
  const auto out = mnp::StallDecision(0.5, 0.5, 0.05, 0.3, cfg, stall_time);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.5);
  EXPECT_FALSE(out.in_stall);
  EXPECT_DOUBLE_EQ(stall_time, 0.3);
}

TEST(StallDecision, SustainedStallPastGraceEasesToCrawlSpeed)
{
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  // Two ticks accumulate 0.35 + 0.35 = 0.7 s > 0.6 s grace — the second
  // call must clamp target_speed down to stall_crawl_speed.
  mnp::StallDecision(0.5, 0.5, 0.05, 0.35, cfg, stall_time);
  EXPECT_DOUBLE_EQ(stall_time, 0.35);
  const auto out = mnp::StallDecision(0.5, 0.5, 0.05, 0.35, cfg, stall_time);
  EXPECT_DOUBLE_EQ(out.target_speed, cfg.stall_crawl_speed);
  EXPECT_TRUE(out.in_stall);
  EXPECT_NEAR(stall_time, 0.7, 1e-9);
}

TEST(StallDecision, TractionRecoveryResetsStallTimeImmediately)
{
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  // Build up past the grace period...
  mnp::StallDecision(0.5, 0.5, 0.05, 0.7, cfg, stall_time);
  ASSERT_GT(stall_time, cfg.stall_grace_s);

  // ...then traction returns for one tick: stall_time must reset to 0, not
  // decay gradually, so the crawl clamp lifts as soon as the wheels grip
  // again.
  const auto out = mnp::StallDecision(0.5, 0.5, 0.48, 0.1, cfg, stall_time);
  EXPECT_DOUBLE_EQ(stall_time, 0.0);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.5);
  EXPECT_FALSE(out.in_stall);
}

TEST(StallDecision, EasedSpeedNeverExceedsRequestedTarget)
{
  // If the path-derived target_speed is already below stall_crawl_speed
  // (e.g. speed_slow < crawl), easing must not RAISE it — min(), not
  // assign.
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;
  cfg.stall_crawl_speed = 0.08;

  mnp::StallDecision(0.05, 0.5, 0.05, 0.7, cfg, stall_time);
  const auto out = mnp::StallDecision(0.05, 0.5, 0.05, 0.1, cfg, stall_time);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.05);  // min(0.05, 0.08) == 0.05, not clamped up.
  EXPECT_TRUE(out.in_stall);  // still blocked — caller must still cap the output.
}

TEST(StallDecision, LowCommandedSpeedNeverCountsAsStall)
{
  // cmd_speed <= stall_crawl_speed: the robot is already crawling on
  // purpose (e.g. final approach), so a low measured speed is expected,
  // not a stall. Must never accumulate stall_time.
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  const auto out = mnp::StallDecision(0.08, 0.08, 0.0, 1.0, cfg, stall_time);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.08);
  EXPECT_FALSE(out.in_stall);
  EXPECT_DOUBLE_EQ(stall_time, 0.0);
}

TEST(StallDecision, ExactlyAtGraceBoundaryDoesNotEaseYet)
{
  // stall_time == stall_grace_s exactly must NOT ease (strict > only,
  // mirrors GpsJumpImplausible's strict boundary convention).
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  const auto out = mnp::StallDecision(0.5, 0.5, 0.05, cfg.stall_grace_s, cfg, stall_time);
  EXPECT_DOUBLE_EQ(stall_time, cfg.stall_grace_s);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.5);
  EXPECT_FALSE(out.in_stall);
}

// The in_stall flag is what the controller uses to (a) cap the commanded
// velocity at stall_crawl_speed instead of flooring it up to min_speed_mps,
// and (b) freeze the carrot so lon_error can't run away ahead of a blocked
// robot. These tests pin that flag for the scenarios that drove the
// hole-digging regression fix.

TEST(StallDecision, BlockedRobotFlagsStallSoOutputIsCappedNotFloored)
{
  // A robot pushing hard into an obstacle: commanding 0.30 m/s but the odom
  // reads ~0. Once past the grace period in_stall must be true so the caller
  // caps cmd_vel at the 0.08 crawl and BYPASSES the 0.15 min_speed_mps floor
  // (flooring here is what dug holes in soft turf).
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  const auto out = mnp::StallDecision(0.30, 0.30, 0.0, 0.7, cfg, stall_time);
  EXPECT_TRUE(out.in_stall);
  EXPECT_DOUBLE_EQ(out.target_speed, cfg.stall_crawl_speed);  // 0.08, below the 0.15 floor.
}

TEST(StallDecision, NormalDrivingDoesNotFlagStallSoFloorStillApplies)
{
  // Healthy traction: in_stall stays false so the caller keeps the normal
  // min_speed_mps floor behaviour for smooth driving.
  double stall_time = 0.0;
  mnp::FtcStallCfg cfg;

  for (int i = 0; i < 10; ++i)
  {
    const auto out = mnp::StallDecision(0.30, 0.30, 0.29, 0.1, cfg, stall_time);
    EXPECT_FALSE(out.in_stall);
    EXPECT_DOUBLE_EQ(out.target_speed, 0.30);
  }
  EXPECT_DOUBLE_EQ(stall_time, 0.0);
}
