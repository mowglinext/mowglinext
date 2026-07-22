// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for FTC's bounded reverse-escape decision. Pure logic, no ROS.
// SAFETY-CRITICAL: these pin the fail-safe ordering (rear-blocked and
// budget-spent both forbid reversing) and the hard distance cap.

#include "mowgli_nav2_plugins/ftc_reverse_escape.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

TEST(ReverseEscape, DefaultCfgIsOptInDisabled)
{
  // The struct default is OPT-IN: a fresh cfg must never reverse until the
  // operator explicitly enables the maneuver.
  mnp::ReverseEscapeCfg cfg;  // enabled defaults false
  EXPECT_FALSE(cfg.enabled);
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.0, true), mnp::ReverseEscapeAction::kNone);
}

TEST(ReverseEscape, DisabledReturnsNone)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.enabled = false;
  // Even with a clear rear and full budget, a disabled feature must never
  // reverse — the caller proceeds straight to wait/abort.
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.0, true), mnp::ReverseEscapeAction::kNone);
}

TEST(ReverseEscape, ClearRearWithBudgetReverses)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.enabled = true;  // opt in (default is false)
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.0, true), mnp::ReverseEscapeAction::kReverse);
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.15, true), mnp::ReverseEscapeAction::kReverse);
}

TEST(ReverseEscape, RearBlockedNeverReverses)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.enabled = true;
  // SAFETY: an obstacle behind must forbid the maneuver even with full budget.
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.0, false), mnp::ReverseEscapeAction::kExhausted);
}

TEST(ReverseEscape, BudgetSpentStopsReversing)
{
  mnp::ReverseEscapeCfg cfg;  // max 0.30
  cfg.enabled = true;
  // At/over the cap the maneuver ends even with a clear rear (strict >=).
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.30, true), mnp::ReverseEscapeAction::kExhausted);
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.45, true), mnp::ReverseEscapeAction::kExhausted);
}

TEST(ReverseEscape, JustUnderCapStillReverses)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.enabled = true;
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, 0.29, true), mnp::ReverseEscapeAction::kReverse);
}

TEST(ReverseEscape, AdvanceAccumulatesMagnitudeOfReverseSpeed)
{
  mnp::ReverseEscapeCfg cfg;
  // Reversing at -0.10 m/s for 0.1 s adds 0.01 m regardless of sign.
  const double d = mnp::ReverseEscapeAdvance(cfg, 0.0, -0.10, 0.1);
  EXPECT_NEAR(d, 0.01, 1e-12);
}

TEST(ReverseEscape, AdvanceHardCapsAtMaxDist)
{
  mnp::ReverseEscapeCfg cfg;  // max 0.30
  // A big step must be clamped so the total NEVER exceeds the budget.
  const double d = mnp::ReverseEscapeAdvance(cfg, 0.28, -0.10, 5.0);
  EXPECT_DOUBLE_EQ(d, cfg.max_dist_m);
}

TEST(ReverseEscape, AdvanceIgnoresNegativeDt)
{
  mnp::ReverseEscapeCfg cfg;
  // A pathological negative dt must not decrement the accumulator.
  const double d = mnp::ReverseEscapeAdvance(cfg, 0.10, -0.10, -1.0);
  EXPECT_DOUBLE_EQ(d, 0.10);
}

// Integration-style: reversing tick by tick reaches the cap, then the decision
// flips to kExhausted — the full escape lifecycle the controller drives.
TEST(ReverseEscape, ReversesUntilCapThenGivesUp)
{
  mnp::ReverseEscapeCfg cfg;  // max 0.30, speed 0.10
  cfg.enabled = true;
  double dist = 0.0;
  int reverse_ticks = 0;
  for (int i = 0; i < 100; ++i)
  {
    const auto action = mnp::ReverseEscapeDecide(cfg, dist, /*rear_clear=*/true);
    if (action != mnp::ReverseEscapeAction::kReverse)
    {
      break;
    }
    ++reverse_ticks;
    dist = mnp::ReverseEscapeAdvance(cfg, dist, -0.10, 0.1);  // 0.01 m/tick
  }
  EXPECT_DOUBLE_EQ(dist, cfg.max_dist_m);  // capped exactly
  EXPECT_EQ(mnp::ReverseEscapeDecide(cfg, dist, true), mnp::ReverseEscapeAction::kExhausted);
  EXPECT_EQ(reverse_ticks, 30);  // 0.30 m / 0.01 m per tick
}
