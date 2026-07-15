// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the yaw-robustness gates (Level 1 COG discipline + Level 2
// LiDAR yaw yield). Pure logic, no ROS/GTSAM.

#include <cmath>

#include "fusion_graph/yaw_gates.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

// ── CogShouldApply ──────────────────────────────────────────────────────
TEST(CogShouldApply, UninitializedAlwaysAccepts)
{
  // Before init the seed needs the COG even at rest / no RTK.
  EXPECT_TRUE(fg::CogShouldApply(/*init=*/false, /*rtk=*/false, /*vx=*/0.0,
                                 /*require_rtk=*/true, /*min_speed=*/0.08));
}

TEST(CogShouldApply, GatedWhenRtkRequiredAndNotFresh)
{
  EXPECT_FALSE(fg::CogShouldApply(true, /*rtk_fresh=*/false, 0.20, true, 0.08));
}

TEST(CogShouldApply, GatedWhenSlow)
{
  EXPECT_FALSE(fg::CogShouldApply(true, true, /*vx=*/0.05, true, 0.08));
}

TEST(CogShouldApply, GatedInReverse)
{
  // Reverse (undock BackUp): COG = heading + 180°. wheel_vx < 0 < min_speed.
  EXPECT_FALSE(fg::CogShouldApply(true, true, /*vx=*/-0.16, true, 0.08));
}

TEST(CogShouldApply, AcceptsForwardRtkFresh)
{
  EXPECT_TRUE(fg::CogShouldApply(true, true, /*vx=*/0.18, true, 0.08));
}

TEST(CogShouldApply, RtkGateOffStillSpeedGates)
{
  // require_rtk=false: RTK freshness ignored, but the speed gate still applies.
  EXPECT_TRUE(fg::CogShouldApply(true, /*rtk_fresh=*/false, 0.18, /*require_rtk=*/false, 0.08));
  EXPECT_FALSE(fg::CogShouldApply(true, false, /*vx=*/0.02, false, 0.08));
}

// ── CogEffectiveSigma ───────────────────────────────────────────────────
TEST(CogEffectiveSigma, FloorsTightVariance)
{
  // variance (0.05 rad)^2 → σ 0.05, below floor 0.15 → returns floor.
  EXPECT_NEAR(fg::CogEffectiveSigma(0.05 * 0.05, 0.15), 0.15, 1e-9);
}

TEST(CogEffectiveSigma, KeepsLooseVariance)
{
  // variance (0.3 rad)^2 → σ 0.3, above floor 0.15 → returns 0.3.
  EXPECT_NEAR(fg::CogEffectiveSigma(0.3 * 0.3, 0.15), 0.3, 1e-9);
}

TEST(CogEffectiveSigma, HandlesInvalidVariance)
{
  // 0 / negative / NaN variance → default 0.05, then floored.
  EXPECT_NEAR(fg::CogEffectiveSigma(0.0, 0.15), 0.15, 1e-9);
  EXPECT_NEAR(fg::CogEffectiveSigma(-1.0, 0.15), 0.15, 1e-9);
  EXPECT_NEAR(fg::CogEffectiveSigma(std::nan(""), 0.02), 0.05, 1e-9);
}

// ── ScanYawSigma ────────────────────────────────────────────────────────
TEST(ScanYawSigma, FloorsTightScanYaw)
{
  EXPECT_NEAR(fg::ScanYawSigma(0.005, 0.30), 0.30, 1e-9);
}

TEST(ScanYawSigma, KeepsLooseScanYaw)
{
  EXPECT_NEAR(fg::ScanYawSigma(0.50, 0.30), 0.50, 1e-9);
}

TEST(ScanYawSigma, ZeroFloorIsPassthrough)
{
  EXPECT_NEAR(fg::ScanYawSigma(0.005, 0.0), 0.005, 1e-9);
}
