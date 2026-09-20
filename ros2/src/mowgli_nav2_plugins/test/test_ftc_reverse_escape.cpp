// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for FTC's bounded reverse-escape decision. Pure logic, no ROS.
// SAFETY-CRITICAL: these pin the fail-safe ordering (rear-blocked and
// budget-spent both forbid reversing) and the hard distance cap.

#include <limits>

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

// ── Leaving a reverse-escape, and earning a new one (field 2026-09-20) ───────

TEST(ReverseEscapeRelease, OneFollowableTickDoesNotCancelACommittedReverse)
{
  double followable = 0.0;
  EXPECT_FALSE(mnp::ReverseEscapeShouldRelease(/*can_continue=*/true, 0.1, 1.0, followable));
  EXPECT_DOUBLE_EQ(followable, 0.1);
}

TEST(ReverseEscapeRelease, ReleasesOnceTheProfileHeldForTheWholeHold)
{
  double followable = 0.0;
  int ticks = 0;
  while (!mnp::ReverseEscapeShouldRelease(true, 0.1, 1.0, followable))
  {
    ++ticks;
    ASSERT_LT(ticks, 100);
  }
  EXPECT_EQ(ticks, 9);  // released on the 10th consecutive followable tick
}

TEST(ReverseEscapeRelease, AnInfeasibleTickRestartsTheHold)
{
  double followable = 0.0;
  for (int i = 0; i < 9; ++i)
  {
    EXPECT_FALSE(mnp::ReverseEscapeShouldRelease(true, 0.1, 1.0, followable));
  }
  followable = 0.0;  // what reverseEscapeOrWait does on an infeasible tick
  EXPECT_FALSE(mnp::ReverseEscapeShouldRelease(true, 0.1, 1.0, followable));
}

TEST(ReverseEscapeRelease, ReleasesAtOnceWhenTheEscapeCannotContinue)
{
  // Budget spent or rear blocked, and the planner has a profile: going forward
  // on it is all that is left — holding a reverse that cannot move is a stall.
  double followable = 0.0;
  EXPECT_TRUE(mnp::ReverseEscapeShouldRelease(/*can_continue=*/false, 0.1, 1.0, followable));
}

TEST(ReverseEscapeRelease, NegativeDtNeverShortensTheHold)
{
  double followable = 0.5;
  EXPECT_FALSE(mnp::ReverseEscapeShouldRelease(true, -5.0, 1.0, followable));
  EXPECT_DOUBLE_EQ(followable, 0.5);
}

TEST(ReverseBudgetRefill, NeedsRealProgressPastTheEngagementPoint)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.max_dist_m = 0.30;
  EXPECT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, 0.0));  // same spot
  EXPECT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, 0.30));  // merely back where it started
  EXPECT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, 0.59));
  EXPECT_TRUE(mnp::ReverseBudgetEarnsRefill(cfg, 0.60));
}

TEST(ReverseBudgetRefill, NonFiniteProgressOrNoBudgetNeverRefills)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.max_dist_m = 0.30;
  EXPECT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, -1.0));
  cfg.max_dist_m = 0.0;
  EXPECT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, 10.0));
}

// The field failure as a whole: planner feasibility flickering every other tick
// at ONE path index. With the old policy (cancel + refill on any followable
// tick) the budget was never spent and the robot wagged for minutes; now the
// escape is carried through and, with no progress, the budget stays spent so
// the caller reaches wait → abort and hands over to the BT's detour.
TEST(ReverseEscapeRelease, FlickeringFeasibilitySpendsTheBudgetInsteadOfDithering)
{
  mnp::ReverseEscapeCfg cfg;
  cfg.enabled = true;
  cfg.max_dist_m = 0.30;
  cfg.speed_mps = 0.15;
  constexpr double kDt = 0.1;
  constexpr double kHold = 1.0;

  bool active = false;
  double dist = 0.0;
  double followable = 0.0;
  int engagements = 0;
  int direction_flips = 0;
  bool was_reversing = false;
  int exhausted_at_tick = -1;

  for (int tick = 0; tick < 1800 && exhausted_at_tick < 0; ++tick)  // 3 minutes
  {
    const bool feasible = (tick % 2) == 1;
    if (active)
    {
      dist = mnp::ReverseEscapeAdvance(cfg, dist, -cfg.speed_mps, kDt);
    }
    const bool can_reverse = mnp::ReverseEscapeDecide(cfg, dist, /*rear_clear=*/true) ==
                             mnp::ReverseEscapeAction::kReverse;
    if (!feasible)
    {
      followable = 0.0;
      if (can_reverse)
      {
        engagements += active ? 0 : 1;
        active = true;
      }
      else
      {
        active = false;
        exhausted_at_tick = tick;  // caller falls through to wait → abort
      }
    }
    else if (active && mnp::ReverseEscapeShouldRelease(can_reverse, kDt, kHold, followable))
    {
      active = false;
    }
    // No path progress at all → never refilled.
    ASSERT_FALSE(mnp::ReverseBudgetEarnsRefill(cfg, 0.0));
    direction_flips += (active != was_reversing) ? 1 : 0;
    was_reversing = active;
  }

  EXPECT_EQ(engagements, 1) << "field: 282 engagements in 3 minutes";
  EXPECT_LE(direction_flips, 2);
  EXPECT_DOUBLE_EQ(dist, cfg.max_dist_m);
  ASSERT_GE(exhausted_at_tick, 0) << "the bound must be reached so the BT detour gets its turn";
  EXPECT_LT(exhausted_at_tick, 60) << "within a few seconds, not minutes";
}
