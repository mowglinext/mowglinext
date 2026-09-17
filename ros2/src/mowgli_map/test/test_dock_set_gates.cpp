// SPDX-License-Identifier: GPL-3.0
//
// Unit tests for ResolveDockSetGates — which gates a SetDockingPoint request
// must pass. Pure logic, no ROS. The regression these pin: the one-click dock
// calibration measures its yaw while REVERSING OFF the dock, so a yaw-only
// MOTION write must not demand is_charging (it made the calibration fail
// deterministically, 2026-09-17) — and that exemption must not leak to any
// request that captures or sets a POSITION.

#include "mowgli_map/dock_set_gates.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_map::DockSetKind;
using mowgli_map::kDockYawSourceMotion;
using mowgli_map::kDockYawSourcePreserve;
using mowgli_map::kDockYawSourceRequest;
using mowgli_map::ResolveDockSetGates;

TEST(DockSetGates, YawOnlyMotionWriteDoesNotRequireCharging)
{
  // Arrange / Act
  const auto gates = ResolveDockSetGates(/*use_gps_position=*/false,
                                         /*preserve_position=*/true,
                                         kDockYawSourceMotion);

  // Assert
  EXPECT_EQ(gates.kind, DockSetKind::YAW_ONLY_MOTION);
  EXPECT_FALSE(gates.require_charging);
  EXPECT_FALSE(gates.require_yaw_convergence);
}

TEST(DockSetGates, YawOnlyMotionWriteKeepsRtkGateAndNeedsAStoredPosition)
{
  const auto gates = ResolveDockSetGates(false, true, kDockYawSourceMotion);

  EXPECT_TRUE(gates.require_gps_accuracy);
  EXPECT_TRUE(gates.require_existing_pose);
}

TEST(DockSetGates, GpsPositionCaptureKeepsEveryGateForEveryYawSource)
{
  for (const uint8_t yaw_source :
       {kDockYawSourcePreserve, kDockYawSourceRequest, kDockYawSourceMotion})
  {
    const auto gates = ResolveDockSetGates(/*use_gps_position=*/true,
                                           /*preserve_position=*/false,
                                           yaw_source);

    EXPECT_EQ(gates.kind, DockSetKind::POSITION_CAPTURE) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_charging) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_gps_accuracy) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_yaw_convergence) << "yaw_source=" << int{yaw_source};
    EXPECT_FALSE(gates.require_existing_pose) << "yaw_source=" << int{yaw_source};
  }
}

TEST(DockSetGates, ManualPositionSetKeepsEveryGateEvenWithMotionYaw)
{
  // The off-dock exemption is keyed on preserve_position, NOT on "MOTION with
  // use_gps_position=false" — that combination still SETS a position.
  const auto gates = ResolveDockSetGates(false, false, kDockYawSourceMotion);

  EXPECT_EQ(gates.kind, DockSetKind::MANUAL);
  EXPECT_TRUE(gates.require_charging);
  EXPECT_TRUE(gates.require_gps_accuracy);
  EXPECT_TRUE(gates.require_yaw_convergence);
}

TEST(DockSetGates, PreservePositionTogetherWithGpsCaptureIsRefused)
{
  const auto gates = ResolveDockSetGates(true, true, kDockYawSourceMotion);

  EXPECT_EQ(gates.kind, DockSetKind::INVALID);
  ASSERT_NE(gates.invalid_reason, nullptr);
  EXPECT_TRUE(gates.require_charging);  // an INVALID result never relaxes a gate
}

TEST(DockSetGates, PreservePositionWithoutMotionYawIsRefused)
{
  for (const uint8_t yaw_source : {kDockYawSourcePreserve, kDockYawSourceRequest})
  {
    const auto gates = ResolveDockSetGates(false, true, yaw_source);

    EXPECT_EQ(gates.kind, DockSetKind::INVALID) << "yaw_source=" << int{yaw_source};
    EXPECT_NE(gates.invalid_reason, nullptr) << "yaw_source=" << int{yaw_source};
    EXPECT_TRUE(gates.require_charging) << "yaw_source=" << int{yaw_source};
  }
}

TEST(DockSetGates, PreservePositionWithUnknownYawSourceIsRefused)
{
  const auto gates = ResolveDockSetGates(false, true, /*yaw_source=*/3);

  EXPECT_EQ(gates.kind, DockSetKind::INVALID);
  EXPECT_NE(gates.invalid_reason, nullptr);
}

}  // namespace
