// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for FTC's blade-load slowdown decision. Pure logic, no ROS.

#include <limits>

#include "mowgli_nav2_plugins/ftc_blade_load.hpp"
#include <gtest/gtest.h>

namespace mnp = mowgli_nav2_plugins;

namespace
{

mnp::FtcBladeLoadCfg EnabledCfg()
{
  mnp::FtcBladeLoadCfg cfg;
  cfg.enabled = true;
  cfg.rpm_full = 2500.0;
  cfg.rpm_min = 1800.0;
  cfg.min_speed_ratio = 0.4;
  cfg.telemetry_max_age_s = 1.0;
  return cfg;
}

constexpr double kTarget = 0.20;  // mowing_speed
constexpr double kFloor = 0.08;  // stall_crawl_speed
constexpr double kFresh = 0.1;  // telemetry age well inside the 1 s window

}  // namespace

// ── Gates: every "no evidence" path must fail OPEN (scale 1.0) ──────────────

TEST(BladeLoadDecision, DisabledPassesThroughEvenUnderHeavyLoad)
{
  auto cfg = EnabledCfg();
  cfg.enabled = false;
  const auto out = mnp::BladeLoadDecision(kTarget, true, 0.0, kFresh, kFloor, cfg);
  EXPECT_DOUBLE_EQ(out.target_speed, kTarget);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
  EXPECT_FALSE(out.is_limited);
}

TEST(BladeLoadDecision, InactiveBladeNeverSlowsDown)
{
  // Dry run (mowing_enabled=false) or blade not yet confirmed spinning: RPM is
  // 0 but that is NOT load, so the robot must run at full speed.
  const auto out = mnp::BladeLoadDecision(kTarget, false, 0.0, kFresh, kFloor, EnabledCfg());
  EXPECT_DOUBLE_EQ(out.target_speed, kTarget);
  EXPECT_FALSE(out.is_limited);
}

TEST(BladeLoadDecision, StaleTelemetryNeverSlowsDown)
{
  const auto cfg = EnabledCfg();
  // 1.5 s old > 1.0 s max age.
  const auto stale = mnp::BladeLoadDecision(kTarget, true, 0.0, 1.5, kFloor, cfg);
  EXPECT_DOUBLE_EQ(stale.target_speed, kTarget);
  EXPECT_FALSE(stale.is_limited);
  // Negative age = never received.
  const auto never = mnp::BladeLoadDecision(kTarget, true, 0.0, -1.0, kFloor, cfg);
  EXPECT_DOUBLE_EQ(never.target_speed, kTarget);
  EXPECT_FALSE(never.is_limited);
}

TEST(BladeLoadDecision, TelemetryExactlyAtMaxAgeStillCounts)
{
  const auto out = mnp::BladeLoadDecision(kTarget, true, 0.0, 1.0, kFloor, EnabledCfg());
  EXPECT_TRUE(out.is_limited);
}

TEST(BladeLoadDecision, DegenerateThresholdsNeverLimit)
{
  auto cfg = EnabledCfg();
  cfg.rpm_min = cfg.rpm_full;  // zero span
  EXPECT_FALSE(mnp::BladeLoadDecision(kTarget, true, 0.0, kFresh, kFloor, cfg).is_limited);
  cfg.rpm_min = cfg.rpm_full + 100.0;  // inverted
  EXPECT_FALSE(mnp::BladeLoadDecision(kTarget, true, 0.0, kFresh, kFloor, cfg).is_limited);
}

TEST(BladeLoadDecision, NonFiniteRpmNeverLimits)
{
  const auto cfg = EnabledCfg();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mnp::BladeLoadDecision(kTarget, true, nan, kFresh, kFloor, cfg).is_limited);
}

// ── The ramp ────────────────────────────────────────────────────────────────

TEST(BladeLoadScale, FullSpeedAtOrAboveRpmFull)
{
  const auto cfg = EnabledCfg();
  EXPECT_DOUBLE_EQ(mnp::BladeLoadScale(2500.0, cfg), 1.0);
  EXPECT_DOUBLE_EQ(mnp::BladeLoadScale(3200.0, cfg), 1.0);
}

TEST(BladeLoadScale, MinRatioAtOrBelowRpmMin)
{
  const auto cfg = EnabledCfg();
  EXPECT_DOUBLE_EQ(mnp::BladeLoadScale(1800.0, cfg), 0.4);
  EXPECT_DOUBLE_EQ(mnp::BladeLoadScale(500.0, cfg), 0.4);
}

TEST(BladeLoadScale, LinearBetweenThresholds)
{
  const auto cfg = EnabledCfg();
  // Midpoint of [1800, 2500] → midpoint of [0.4, 1.0].
  EXPECT_NEAR(mnp::BladeLoadScale(2150.0, cfg), 0.7, 1e-12);
  // Quarter of the way up the ramp.
  EXPECT_NEAR(mnp::BladeLoadScale(1975.0, cfg), 0.55, 1e-12);
}

TEST(BladeLoadScale, MinRatioIsClampedToUnitInterval)
{
  auto cfg = EnabledCfg();
  cfg.min_speed_ratio = 1.5;  // nonsense: would "slow" to 150 %
  EXPECT_DOUBLE_EQ(mnp::BladeLoadScale(0.0, cfg), 1.0);
  cfg.min_speed_ratio = -0.5;
  EXPECT_DOUBLE_EQ(mnp::BladeLoadScale(0.0, cfg), 0.0);
}

// ── Applying the scale ──────────────────────────────────────────────────────

TEST(BladeLoadDecision, HealthyRpmIsNotLimited)
{
  const auto out = mnp::BladeLoadDecision(kTarget, true, 2900.0, kFresh, kFloor, EnabledCfg());
  EXPECT_DOUBLE_EQ(out.target_speed, kTarget);
  EXPECT_DOUBLE_EQ(out.scale, 1.0);
  EXPECT_FALSE(out.is_limited);
}

TEST(BladeLoadDecision, SaggingRpmScalesTargetSpeed)
{
  // 2150 rpm → scale 0.7 → 0.20 × 0.7 = 0.14 m/s.
  const auto out = mnp::BladeLoadDecision(kTarget, true, 2150.0, kFresh, kFloor, EnabledCfg());
  EXPECT_NEAR(out.target_speed, 0.14, 1e-12);
  EXPECT_NEAR(out.scale, 0.7, 1e-12);
  EXPECT_TRUE(out.is_limited);
}

TEST(BladeLoadDecision, BoggedBladeDropsToMinRatio)
{
  const auto out = mnp::BladeLoadDecision(kTarget, true, 1500.0, kFresh, kFloor, EnabledCfg());
  EXPECT_NEAR(out.target_speed, 0.08, 1e-12);  // 0.20 × 0.4, exactly the floor here
  EXPECT_TRUE(out.is_limited);
}

TEST(BladeLoadDecision, SpeedFloorBoundsTheSlowdownFromBelow)
{
  // With a 0.12 m/s floor, min_ratio × target = 0.08 is raised to 0.12 so the
  // robot keeps creeping instead of sitting under the wheel deadband.
  const auto out = mnp::BladeLoadDecision(kTarget, true, 1500.0, kFresh, 0.12, EnabledCfg());
  EXPECT_DOUBLE_EQ(out.target_speed, 0.12);
  EXPECT_TRUE(out.is_limited);
}

TEST(BladeLoadDecision, FloorNeverRaisesSpeedAboveTheUnscaledTarget)
{
  // A turn target (0.10) below the floor (0.12): the slowdown must not
  // accelerate the robot through the bend.
  const auto out = mnp::BladeLoadDecision(0.10, true, 1500.0, kFresh, 0.12, EnabledCfg());
  EXPECT_DOUBLE_EQ(out.target_speed, 0.10);
  EXPECT_FALSE(out.is_limited);  // nothing was actually taken off
}

TEST(BladeLoadDecision, NegativeFloorIsTreatedAsZero)
{
  const auto out = mnp::BladeLoadDecision(kTarget, true, 1500.0, kFresh, -1.0, EnabledCfg());
  EXPECT_NEAR(out.target_speed, 0.08, 1e-12);
}

TEST(BladeLoadDecision, RecoveryIsImmediate)
{
  // No hysteresis/debounce state: the moment RPM is back, so is the speed.
  const auto cfg = EnabledCfg();
  EXPECT_TRUE(mnp::BladeLoadDecision(kTarget, true, 1600.0, kFresh, kFloor, cfg).is_limited);
  EXPECT_FALSE(mnp::BladeLoadDecision(kTarget, true, 2600.0, kFresh, kFloor, cfg).is_limited);
}
