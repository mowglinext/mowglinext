// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the dock-prior vs RTK-Fixed GPS consistency check (issue
// #512). Pure logic, no ROS/GTSAM. The numbers come from the 2026-09-02
// field log: dock_pose (6.27, 2.77); receiver "Fixed 14-20 mm" while placing
// the docked robot at (7.88, 0.92) / (4.70, 1.05) / (4.44, 3.19) — 2.45 /
// 2.33 / 1.88 m off — for ~45 min after a power cycle.

#define _USE_MATH_DEFINES
#include <cmath>

#include "fusion_graph/dock_gps_consistency.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
constexpr double kDockX = 6.27;
constexpr double kDockY = 2.77;
constexpr double kMaxDisagreementM = 0.50;
constexpr double kMaxGpsSigmaM = 0.05;
constexpr double kFixedSigmaM = 0.017;  // "Fixed 14-20 mm"
}  // namespace

// ── DockGpsDisagreementM ────────────────────────────────────────────────
TEST(DockGpsDisagreementM, IsPlanarDistance)
{
  EXPECT_DOUBLE_EQ(fg::DockGpsDisagreementM(0.0, 0.0, 3.0, 4.0), 5.0);
  EXPECT_DOUBLE_EQ(fg::DockGpsDisagreementM(kDockX, kDockY, kDockX, kDockY), 0.0);
}

TEST(DockGpsDisagreementM, ReproducesFieldNumbers)
{
  EXPECT_NEAR(fg::DockGpsDisagreementM(kDockX, kDockY, 7.88, 0.92), 2.45, 0.01);
  EXPECT_NEAR(fg::DockGpsDisagreementM(kDockX, kDockY, 4.70, 1.05), 2.33, 0.01);
  EXPECT_NEAR(fg::DockGpsDisagreementM(kDockX, kDockY, 4.44, 3.19), 1.88, 0.01);
}

// ── DockAntennaMapXY ────────────────────────────────────────────────────
TEST(DockAntennaMapXY, ZeroLeverArmIsTheDock)
{
  const auto a = fg::DockAntennaMapXY(kDockX, kDockY, 1.23, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(a.x, kDockX);
  EXPECT_DOUBLE_EQ(a.y, kDockY);
}

TEST(DockAntennaMapXY, LeverArmRotatesWithDockYaw)
{
  // Antenna 0.30 m ahead of base_link; dock facing +Y (yaw 90°) puts it at
  // dock + (0, 0.30).
  const auto a = fg::DockAntennaMapXY(kDockX, kDockY, M_PI / 2.0, 0.30, 0.0);
  EXPECT_NEAR(a.x, kDockX, 1e-9);
  EXPECT_NEAR(a.y, kDockY + 0.30, 1e-9);
}

TEST(DockAntennaMapXY, LeverArmAloneMustNotReadAsDisagreement)
{
  // A GPS sample sitting exactly on the antenna's true spot disagrees by 0
  // once measured antenna-to-antenna — this is why the node compares against
  // the rotated antenna position, not base_link.
  const auto a = fg::DockAntennaMapXY(kDockX, kDockY, 0.7, 0.25, -0.05);
  EXPECT_NEAR(fg::DockGpsDisagreementM(a.x, a.y, a.x, a.y), 0.0, 1e-12);
}

// ── DockPriorShouldYield ────────────────────────────────────────────────
TEST(DockPriorShouldYield, NoFixKeepsThePrior)
{
  // The terrace case: no RTK-Fixed at all. Disagreement is irrelevant.
  EXPECT_FALSE(fg::DockPriorShouldYield(
      /*rtk_fixed_fresh=*/false,
      /*sigma=*/-1.0,
      /*disagreement=*/2.45,
      kMaxDisagreementM,
      kMaxGpsSigmaM));
}

TEST(DockPriorShouldYield, FloatKeepsThePrior)
{
  // Float: not Fixed (fresh=false) AND σ well above the trust cap. Either
  // alone must keep the prior.
  EXPECT_FALSE(fg::DockPriorShouldYield(false, 0.35, 2.45, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_FALSE(fg::DockPriorShouldYield(true, 0.35, 2.45, kMaxDisagreementM, kMaxGpsSigmaM));
}

TEST(DockPriorShouldYield, FixedWithSmallDisagreementKeepsThePrior)
{
  // The normal 5-30 cm ambiguity-set shift between sessions.
  EXPECT_FALSE(
      fg::DockPriorShouldYield(true, kFixedSigmaM, 0.15, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_FALSE(
      fg::DockPriorShouldYield(true, kFixedSigmaM, 0.30, kMaxDisagreementM, kMaxGpsSigmaM));
}

TEST(DockPriorShouldYield, ExactlyAtThresholdKeepsThePrior)
{
  // Strict > only, same convention as GpsJumpImplausible.
  EXPECT_FALSE(fg::DockPriorShouldYield(
      true, kFixedSigmaM, kMaxDisagreementM, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_TRUE(fg::DockPriorShouldYield(
      true, kFixedSigmaM, kMaxDisagreementM + 1e-6, kMaxDisagreementM, kMaxGpsSigmaM));
}

TEST(DockPriorShouldYield, FixedWithBigDisagreementYields)
{
  // The 2026-09-02 transient: Fixed at 14-20 mm, 1.88-2.45 m off.
  EXPECT_TRUE(fg::DockPriorShouldYield(true, 0.014, 2.45, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_TRUE(fg::DockPriorShouldYield(true, 0.020, 2.33, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_TRUE(fg::DockPriorShouldYield(true, 0.017, 1.88, kMaxDisagreementM, kMaxGpsSigmaM));
}

TEST(DockPriorShouldYield, UnknownOrZeroSigmaKeepsThePrior)
{
  EXPECT_FALSE(fg::DockPriorShouldYield(true, 0.0, 2.45, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_FALSE(fg::DockPriorShouldYield(true, -1.0, 2.45, kMaxDisagreementM, kMaxGpsSigmaM));
  EXPECT_FALSE(fg::DockPriorShouldYield(true, NAN, 2.45, kMaxDisagreementM, kMaxGpsSigmaM));
}

TEST(DockPriorShouldYield, DisabledThresholdsKeepThePrior)
{
  // Either threshold ≤ 0 disables the check entirely, even on the field case.
  EXPECT_FALSE(fg::DockPriorShouldYield(true, kFixedSigmaM, 2.45, 0.0, kMaxGpsSigmaM));
  EXPECT_FALSE(fg::DockPriorShouldYield(true, kFixedSigmaM, 2.45, -1.0, kMaxGpsSigmaM));
  EXPECT_FALSE(fg::DockPriorShouldYield(true, kFixedSigmaM, 2.45, kMaxDisagreementM, 0.0));
  EXPECT_FALSE(fg::DockPriorShouldYield(true, kFixedSigmaM, 2.45, kMaxDisagreementM, -0.05));
}
