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
// Unit tests for the pure dock-pose helpers:
//   dock_set_gates.hpp       — which gates a SetDockingPoint request must pass
//   dock_antenna_capture.hpp — antenna -> base_footprint math, pending validity
//
// Regressions pinned: the dock calibration measures its yaw while REVERSING
// OFF the dock, so its MOTION writes must not demand is_charging (that made the
// calibration fail deterministically, 2026-09-17) — and that exemption must
// not leak to any request that captures a position NOW or takes one from the
// caller. And the lever arm is applied exactly once, in the right direction.

#include <cmath>

#include "mowgli_map/dock_antenna_capture.hpp"
#include "mowgli_map/dock_set_gates.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_map::ClassifyPendingAntenna;
using mowgli_map::DockBaseFromAntenna;
using mowgli_map::DockSetKind;
using mowgli_map::Enu;
using mowgli_map::kDockYawSourceMotion;
using mowgli_map::kDockYawSourcePreserve;
using mowgli_map::kDockYawSourceRequest;
using mowgli_map::PendingAntennaCapture;
using mowgli_map::PendingAntennaState;
using mowgli_map::ResolveDockSetGates;

// ── ResolveDockSetGates ─────────────────────────────────────────────────────

TEST(DockSetGates, PendingAntennaMotionWriteDoesNotRequireChargingButNeedsACapture)
{
  // Arrange / Act
  const auto gates = ResolveDockSetGates(/*use_gps_position=*/false,
                                         /*preserve_position=*/false,
                                         /*use_pending_antenna=*/true,
                                         kDockYawSourceMotion);

  // Assert
  EXPECT_EQ(gates.kind, DockSetKind::PENDING_ANTENNA_MOTION);
  EXPECT_FALSE(gates.require_charging);
  EXPECT_FALSE(gates.require_yaw_convergence);
  EXPECT_TRUE(gates.require_gps_accuracy);
  EXPECT_TRUE(gates.require_pending_antenna);
  EXPECT_FALSE(gates.require_existing_pose);  // a fresh install has none to preserve
}

TEST(DockSetGates, YawOnlyMotionWriteDoesNotRequireCharging)
{
  const auto gates =
      ResolveDockSetGates(false, /*preserve_position=*/true, false, kDockYawSourceMotion);

  EXPECT_EQ(gates.kind, DockSetKind::YAW_ONLY_MOTION);
  EXPECT_FALSE(gates.require_charging);
  EXPECT_FALSE(gates.require_yaw_convergence);
}

TEST(DockSetGates, YawOnlyMotionWriteKeepsRtkGateAndNeedsAStoredPosition)
{
  const auto gates = ResolveDockSetGates(false, true, false, kDockYawSourceMotion);

  EXPECT_TRUE(gates.require_gps_accuracy);
  EXPECT_TRUE(gates.require_existing_pose);
  EXPECT_FALSE(gates.require_pending_antenna);
}

TEST(DockSetGates, GpsPositionCaptureKeepsEveryGateForEveryYawSource)
{
  for (const uint8_t yaw_source :
       {kDockYawSourcePreserve, kDockYawSourceRequest, kDockYawSourceMotion})
  {
    const auto gates = ResolveDockSetGates(/*use_gps_position=*/true, false, false, yaw_source);

    EXPECT_EQ(gates.kind, DockSetKind::POSITION_CAPTURE) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_charging) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_gps_accuracy) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_yaw_convergence) << "yaw_source=" << int{yaw_source};
    EXPECT_FALSE(gates.require_existing_pose) << "yaw_source=" << int{yaw_source};
    EXPECT_FALSE(gates.require_pending_antenna) << "yaw_source=" << int{yaw_source};
  }
}

TEST(DockSetGates, ManualPositionSetKeepsEveryGateEvenWithMotionYaw)
{
  // The off-dock exemption is keyed on the explicit flags, NOT on "MOTION
  // with use_gps_position=false" — that combination takes a position from the
  // CALLER, which could have been measured anywhere.
  const auto gates = ResolveDockSetGates(false, false, false, kDockYawSourceMotion);

  EXPECT_EQ(gates.kind, DockSetKind::MANUAL);
  EXPECT_TRUE(gates.require_charging);
  EXPECT_TRUE(gates.require_gps_accuracy);
  EXPECT_TRUE(gates.require_yaw_convergence);
}

TEST(DockSetGates, PositionFlagsAreMutuallyExclusive)
{
  const bool combos[4][3] = {{true, true, false},
                             {true, false, true},
                             {false, true, true},
                             {true, true, true}};
  for (const auto& c : combos)
  {
    const auto gates = ResolveDockSetGates(c[0], c[1], c[2], kDockYawSourceMotion);

    EXPECT_EQ(gates.kind, DockSetKind::INVALID);
    ASSERT_NE(gates.invalid_reason, nullptr);
    EXPECT_TRUE(gates.require_charging);  // an INVALID result never relaxes a gate
  }
}

TEST(DockSetGates, OffDockKindsWithoutMotionYawAreRefused)
{
  for (const uint8_t yaw_source : {kDockYawSourcePreserve, kDockYawSourceRequest, uint8_t{3}})
  {
    const auto yaw_only = ResolveDockSetGates(false, true, false, yaw_source);
    const auto pending = ResolveDockSetGates(false, false, true, yaw_source);

    EXPECT_EQ(yaw_only.kind, DockSetKind::INVALID) << "yaw_source=" << int{yaw_source};
    EXPECT_EQ(pending.kind, DockSetKind::INVALID) << "yaw_source=" << int{yaw_source};
    EXPECT_NE(yaw_only.invalid_reason, nullptr);
    EXPECT_NE(pending.invalid_reason, nullptr);
    EXPECT_TRUE(yaw_only.require_charging);
    EXPECT_TRUE(pending.require_charging);
  }
}

// ── DockBaseFromAntenna ─────────────────────────────────────────────────────

TEST(DockAntennaCapture, BaseIsOneLeverArmBehindTheAntennaAlongTheHeading)
{
  // Antenna 0.30 m AHEAD of base_footprint, chassis facing +X (east).
  const Enu base = DockBaseFromAntenna(Enu{10.30, 5.0}, /*yaw=*/0.0, 0.30, 0.0);

  EXPECT_NEAR(base.east, 10.0, 1e-12);
  EXPECT_NEAR(base.north, 5.0, 1e-12);
}

TEST(DockAntennaCapture, LeverArmRotatesWithTheHeading)
{
  // Field geometry 2026-09-17: dock heading about -54 deg, gps_x 0.30 m.
  const double yaw = -54.0 * M_PI / 180.0;
  const Enu true_base{6.263, 2.811};
  const Enu antenna{true_base.east + 0.30 * std::cos(yaw), true_base.north + 0.30 * std::sin(yaw)};

  const Enu base = DockBaseFromAntenna(antenna, yaw, 0.30, 0.0);

  EXPECT_NEAR(base.east, true_base.east, 1e-9);
  EXPECT_NEAR(base.north, true_base.north, 1e-9);
}

TEST(DockAntennaCapture, LateralLeverArmIsAppliedToTheLeftOfTheHeading)
{
  // Facing north (+Y), antenna 0.10 m to the LEFT (+y body) = 0.10 m WEST.
  const Enu base = DockBaseFromAntenna(Enu{-0.10, 0.0}, M_PI / 2.0, 0.0, 0.10);

  EXPECT_NEAR(base.east, 0.0, 1e-12);
  EXPECT_NEAR(base.north, 0.0, 1e-12);
}

TEST(DockAntennaCapture, ApplyingTheLeverArmTwicePutsTheDockOneLeverArmShort)
{
  // Documents the failure signature, so it is recognisable in the field: a
  // capture fed an ALREADY-corrected position lands one lever arm BEHIND the
  // true base along the heading (the dock "0.3 m short of the contacts").
  const Enu true_base{0.0, 0.0};
  const Enu antenna{0.30, 0.0};
  const Enu once = DockBaseFromAntenna(antenna, 0.0, 0.30, 0.0);
  const Enu twice = DockBaseFromAntenna(once, 0.0, 0.30, 0.0);

  EXPECT_NEAR(once.east, true_base.east, 1e-12);
  EXPECT_NEAR(twice.east, -0.30, 1e-12);
}

// ── ClassifyPendingAntenna ──────────────────────────────────────────────────

TEST(DockAntennaCapture, NeverCapturedIsAbsent)
{
  EXPECT_EQ(ClassifyPendingAntenna(PendingAntennaCapture{}, 100.0, 300.0),
            PendingAntennaState::ABSENT);
}

TEST(DockAntennaCapture, FreshCaptureIsUsableUpToTheTtl)
{
  PendingAntennaCapture capture;
  capture.valid = true;
  capture.stamp_s = 1000.0;

  EXPECT_EQ(ClassifyPendingAntenna(capture, 1000.0, 300.0), PendingAntennaState::USABLE);
  EXPECT_EQ(ClassifyPendingAntenna(capture, 1300.0, 300.0), PendingAntennaState::USABLE);
}

TEST(DockAntennaCapture, CaptureOlderThanTheTtlIsExpired)
{
  PendingAntennaCapture capture;
  capture.valid = true;
  capture.stamp_s = 1000.0;

  EXPECT_EQ(ClassifyPendingAntenna(capture, 1300.1, 300.0), PendingAntennaState::EXPIRED);
}

TEST(DockAntennaCapture, CaptureStampedInTheFutureIsExpiredNotTrusted)
{
  // A clock jump (sim time reset) must not make a capture immortal.
  PendingAntennaCapture capture;
  capture.valid = true;
  capture.stamp_s = 2000.0;

  EXPECT_EQ(ClassifyPendingAntenna(capture, 1000.0, 300.0), PendingAntennaState::EXPIRED);
}

}  // namespace
