// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the 180° yaw-flip recovery decision. Pure logic, no
// ROS/GTSAM — the actual ForceAnchor call and RTK-freshness gating stay in
// the node (fusion_graph_node_callbacks_b.cpp).

#include <cmath>

#include "fusion_graph/cog_flip_recovery.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
constexpr double kDeg = M_PI / 180.0;
}  // namespace

TEST(CogFlipRecoveryFeed, SmallDisagreementNeverAnchors)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;  // defaults: 150° threshold, 3 consecutive, 10 s interval.

  // Estimate and COG agree closely (10°) — nowhere near a flip.
  const auto r = fg::CogFlipRecoveryFeed(
      /*yaw=*/10.0 * kDeg, /*current_yaw=*/0.0, /*seconds_since_last_recovery=*/std::nullopt, cfg,
      count, prev_yaw);
  EXPECT_FALSE(r.should_anchor);
  EXPECT_EQ(count, 0);
  EXPECT_FALSE(prev_yaw.has_value());
}

TEST(CogFlipRecoveryFeed, SingleFlippedSampleBelowConsecutiveThreshold)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;

  // ~180° disagreement, but only one sample so far — must not anchor yet.
  const auto r = fg::CogFlipRecoveryFeed(
      M_PI, 0.0, std::nullopt, cfg, count, prev_yaw);
  EXPECT_FALSE(r.should_anchor);
  EXPECT_EQ(r.count, 1);
  EXPECT_EQ(count, 1);
}

TEST(CogFlipRecoveryFeed, ConsecutiveConsistentFlipsAnchorOnNthSample)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;  // flip_consecutive_n = 3.

  const double current_yaw = 0.0;
  const double flipped_yaw = M_PI;  // consistently reports the same flipped heading.

  auto r1 = fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  EXPECT_FALSE(r1.should_anchor);
  EXPECT_EQ(count, 1);

  auto r2 = fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  EXPECT_FALSE(r2.should_anchor);
  EXPECT_EQ(count, 2);

  auto r3 = fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  EXPECT_TRUE(r3.should_anchor);
  EXPECT_EQ(r3.count, 3);
  // Consumed — the streak resets so a repeat anchor needs a fresh run of N.
  EXPECT_EQ(count, 0);
}

TEST(CogFlipRecoveryFeed, InconsistentFlippedSamplesNeverReachThreshold)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;

  const double current_yaw = 0.0;
  // Each sample disagrees with the estimate by >150° (a suspected flip) —
  // 155° and -155° both qualify — but the two samples disagree with EACH
  // OTHER by 50° (wrapped), well outside the ~30° flip_consistency_rad, so
  // they never corroborate each other.
  fg::CogFlipRecoveryFeed(155.0 * kDeg, current_yaw, std::nullopt, cfg, count, prev_yaw);
  EXPECT_EQ(count, 1);

  const auto jitter =
      fg::CogFlipRecoveryFeed(-155.0 * kDeg, current_yaw, std::nullopt, cfg, count, prev_yaw);
  // Inconsistent with the previous flipped sample -> count restarts at 1,
  // not 2.
  EXPECT_EQ(jitter.count, 1);
  EXPECT_FALSE(jitter.should_anchor);
}

TEST(CogFlipRecoveryFeed, RateLimitBlocksAnchorEvenAtThreshold)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;  // flip_min_interval_s = 10.0 default.

  const double flipped_yaw = M_PI;
  const double current_yaw = 0.0;

  fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  // Third sample reaches consecutive_n=3, but only 2 s have elapsed since
  // the last recovery — well under the 10 s rate limit.
  const auto r = fg::CogFlipRecoveryFeed(
      flipped_yaw, current_yaw, /*seconds_since_last_recovery=*/2.0, cfg, count, prev_yaw);
  EXPECT_FALSE(r.should_anchor);
  // Count is NOT consumed when the rate limit blocks the anchor — the streak
  // should keep accumulating so the anchor fires as soon as the rate limit
  // clears, rather than restarting from zero.
  EXPECT_EQ(count, 3);
}

TEST(CogFlipRecoveryFeed, RateLimitClearsAfterInterval)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;

  const double flipped_yaw = M_PI;
  const double current_yaw = 0.0;

  fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, std::nullopt, cfg, count, prev_yaw);
  // 15 s since the last recovery — past the 10 s interval.
  const auto r = fg::CogFlipRecoveryFeed(flipped_yaw, current_yaw, /*seconds_since_last_recovery=*/15.0,
                                         cfg, count, prev_yaw);
  EXPECT_TRUE(r.should_anchor);
}

TEST(CogFlipRecoveryFeed, ExactlyAtThresholdDoesNotCountAsFlip)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;

  // err == flip_threshold_rad exactly must NOT trigger (strict > only,
  // mirrors GpsJumpImplausible's strict boundary convention).
  const auto r = fg::CogFlipRecoveryFeed(cfg.flip_threshold_rad, 0.0, std::nullopt, cfg, count,
                                         prev_yaw);
  EXPECT_FALSE(r.should_anchor);
  EXPECT_EQ(count, 0);
}

TEST(CogFlipRecoveryFeed, AngleWrapsCorrectlyNearPiBoundary)
{
  int count = 0;
  std::optional<double> prev_yaw;
  fg::CogFlipRecoveryCfg cfg;

  // current_yaw near +π, COG near -π: the raw difference is ~2π but the
  // wrapped disagreement is tiny (~0) — must NOT be treated as a flip.
  const auto r = fg::CogFlipRecoveryFeed(-179.0 * kDeg, 179.0 * kDeg, std::nullopt, cfg, count,
                                         prev_yaw);
  EXPECT_FALSE(r.should_anchor);
  EXPECT_EQ(count, 0);
  EXPECT_NEAR(r.err_rad, 2.0 * kDeg, 1e-9);
}
