// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// SPDX-License-Identifier: GPL-3.0

#include <cmath>

#include "mowgli_behavior/dock_alignment.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_behavior::ComputeDockContactDelta;
using mowgli_behavior::EvaluateDockYawDrift;
using mowgli_behavior::kDockStagingRunwayM;
using mowgli_behavior::kDockYawDeclaredSigmaRad;

constexpr double kDeg = M_PI / 180.0;

// The live 2026-08-24 dock (issue #486): mowgli_robot.yaml dock_pose_x/y/yaw.
constexpr double kDockX = 6.273296;
constexpr double kDockY = 2.773037;
constexpr double kDockYaw = -0.8517;  // -48.80 deg

// ── ComputeDockContactDelta ────────────────────────────────────────────────

TEST(DockContactDelta, reports_zero_offset_when_seated_exactly_on_the_dock_pose)
{
  // Arrange / Act
  const auto d = ComputeDockContactDelta(kDockX, kDockY, kDockX, kDockY, kDockYaw);

  // Assert
  EXPECT_NEAR(d.along_m, 0.0, 1e-9);
  EXPECT_NEAR(d.cross_m, 0.0, 1e-9);
  EXPECT_NEAR(d.range_m, 0.0, 1e-9);
}

TEST(DockContactDelta, stopping_short_of_the_cradle_reads_as_negative_along_track)
{
  // Arrange: 10 cm back along the dock axis, dead on the centreline.
  const double back = 0.10;
  const double x = kDockX - back * std::cos(kDockYaw);
  const double y = kDockY - back * std::sin(kDockYaw);

  // Act
  const auto d = ComputeDockContactDelta(x, y, kDockX, kDockY, kDockYaw);

  // Assert
  EXPECT_NEAR(d.along_m, -back, 1e-9);
  EXPECT_NEAR(d.cross_m, 0.0, 1e-9);
  EXPECT_NEAR(d.range_m, back, 1e-9);
}

TEST(DockContactDelta, seating_off_to_the_left_reads_as_positive_cross_track)
{
  // Arrange: 10 cm to the robot's left, at the correct depth. This is the
  // issue #446 signature — the range alone looks like a near-miss, and only
  // the cross-track term says the contacts were never lined up.
  const double left = 0.10;
  const double x = kDockX - left * std::sin(kDockYaw);
  const double y = kDockY + left * std::cos(kDockYaw);

  // Act
  const auto d = ComputeDockContactDelta(x, y, kDockX, kDockY, kDockYaw);

  // Assert
  EXPECT_NEAR(d.along_m, 0.0, 1e-9);
  EXPECT_NEAR(d.cross_m, left, 1e-9);
  EXPECT_NEAR(d.range_m, left, 1e-9);
}

TEST(DockContactDelta, separates_a_combined_offset_into_its_two_axes)
{
  // Arrange: 4 cm deep AND 9 cm right — the two failure modes superposed.
  const double along = 0.04;
  const double cross = -0.09;
  const double x = kDockX + along * std::cos(kDockYaw) - cross * std::sin(kDockYaw);
  const double y = kDockY + along * std::sin(kDockYaw) + cross * std::cos(kDockYaw);

  // Act
  const auto d = ComputeDockContactDelta(x, y, kDockX, kDockY, kDockYaw);

  // Assert
  EXPECT_NEAR(d.along_m, along, 1e-9);
  EXPECT_NEAR(d.cross_m, cross, 1e-9);
  EXPECT_NEAR(d.range_m, std::hypot(along, cross), 1e-9);
}

// ── EvaluateDockYawDrift ───────────────────────────────────────────────────

TEST(DockYawDrift, accepts_a_measurement_that_agrees_with_the_persisted_yaw)
{
  // Arrange: 0.5 deg apart, a tight fit.
  // Act
  const auto drift = EvaluateDockYawDrift(kDockYaw + 0.5 * kDeg, kDockYaw, 0.25 * kDeg);

  // Assert
  EXPECT_FALSE(drift.is_stale);
  EXPECT_NEAR(drift.delta_rad, 0.5 * kDeg, 1e-9);
}

TEST(DockYawDrift, flags_the_live_2026_08_24_drift_and_sizes_its_lateral_cost)
{
  // Arrange: the 09:30:11 undock line fit — yaw=-52.02 deg at sigma=0.25 deg —
  // against the persisted -48.80 deg.
  const double measured = -52.02 * kDeg;

  // Act
  const auto drift = EvaluateDockYawDrift(measured, kDockYaw, 0.25 * kDeg);

  // Assert: 3.2 deg of drift, which is ~8 cm of lateral miss over the 1.5 m
  // staging runway — the mechanism behind #486's intermittent mating.
  EXPECT_TRUE(drift.is_stale);
  EXPECT_NEAR(drift.delta_rad / kDeg, -3.22, 0.02);
  EXPECT_NEAR(drift.staging_lateral_m, 0.084, 0.002);
}

TEST(DockYawDrift, stays_quiet_when_a_noisy_fit_cannot_support_the_disagreement)
{
  // Arrange: the same 3.2 deg gap, but measured at sigma=2 deg. 3-sigma is
  // 6 deg, so this fit is not entitled to call the persisted value wrong.
  // Act
  const auto drift = EvaluateDockYawDrift(-52.02 * kDeg, kDockYaw, 2.0 * kDeg);

  // Assert
  EXPECT_FALSE(drift.is_stale);
  EXPECT_NEAR(drift.band_rad / kDeg, 6.0, 1e-6);
}

TEST(DockYawDrift, never_narrows_the_band_below_the_persisted_poses_declared_sigma)
{
  // Arrange: a very tight fit (sigma=0.05 deg) disagreeing by 1.5 deg. That is
  // 30 sigma on the fit, but still inside the ~2 deg the persisted pose
  // declares for itself, so it must not warn.
  // Act
  const auto drift = EvaluateDockYawDrift(kDockYaw + 1.5 * kDeg, kDockYaw, 0.05 * kDeg);

  // Assert
  EXPECT_FALSE(drift.is_stale);
  EXPECT_NEAR(drift.band_rad, kDockYawDeclaredSigmaRad, 1e-12);
}

TEST(DockYawDrift, wraps_a_disagreement_that_straddles_the_pi_boundary)
{
  // Arrange: 175 deg vs -175 deg is 10 deg apart, not 350.
  // Act
  const auto drift = EvaluateDockYawDrift(175.0 * kDeg, -175.0 * kDeg, 0.1 * kDeg);

  // Assert
  EXPECT_NEAR(drift.delta_rad / kDeg, -10.0, 1e-6);
  EXPECT_TRUE(drift.is_stale);
}

TEST(DockYawDrift, reports_lateral_cost_as_the_runway_projection_of_the_error)
{
  // Arrange: an exact 10 deg error.
  // Act
  const auto drift = EvaluateDockYawDrift(kDockYaw + 10.0 * kDeg, kDockYaw, 0.1 * kDeg);

  // Assert
  EXPECT_TRUE(drift.is_stale);
  EXPECT_NEAR(drift.staging_lateral_m, kDockStagingRunwayM * std::sin(10.0 * kDeg), 1e-9);
}

TEST(DockYawDrift, is_symmetric_in_the_sign_of_the_error)
{
  // Arrange / Act
  const auto pos = EvaluateDockYawDrift(kDockYaw + 4.0 * kDeg, kDockYaw, 0.2 * kDeg);
  const auto neg = EvaluateDockYawDrift(kDockYaw - 4.0 * kDeg, kDockYaw, 0.2 * kDeg);

  // Assert
  EXPECT_TRUE(pos.is_stale);
  EXPECT_TRUE(neg.is_stale);
  EXPECT_NEAR(pos.delta_rad, -neg.delta_rad, 1e-9);
  EXPECT_NEAR(pos.staging_lateral_m, neg.staging_lateral_m, 1e-9);
}

}  // namespace
