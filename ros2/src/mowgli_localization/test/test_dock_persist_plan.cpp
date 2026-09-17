// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for dock_persist_plan.hpp — the two set_docking_point requests of
// the one-click dock calibration. Regression pinned: the yaw step runs with
// the robot OFF the dock, so it must never be a GPS position capture
// (map_server rejects those unless is_charging — every run failed,
// field-diagnosed 2026-09-17).

#include <string>

#include "mowgli_localization/dock_persist_plan.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_localization::DockPositionFailedMessage;
using mowgli_localization::DockPositionStep;
using mowgli_localization::DockRedockFailedMessage;
using mowgli_localization::DockYawSavedNote;
using mowgli_localization::DockYawStep;
using mowgli_localization::kSetDockYawMotion;
using mowgli_localization::kSetDockYawPreserve;

TEST(DockPersistPlan, YawStepNeverCapturesAGpsPosition)
{
  // Arrange / Act
  const auto req = DockYawStep(-0.9346);

  // Assert
  EXPECT_FALSE(req.use_gps_position);
  EXPECT_TRUE(req.preserve_position);
}

TEST(DockPersistPlan, YawStepCarriesTheMotionDerivedYaw)
{
  const auto req = DockYawStep(-0.9346);

  EXPECT_EQ(req.yaw_source, kSetDockYawMotion);
  EXPECT_DOUBLE_EQ(req.yaw_rad, -0.9346);
}

TEST(DockPersistPlan, PositionStepCapturesGpsAndPreservesTheYawFromTheYawStep)
{
  const auto req = DockPositionStep();

  EXPECT_TRUE(req.use_gps_position);
  EXPECT_FALSE(req.preserve_position);
  EXPECT_EQ(req.yaw_source, kSetDockYawPreserve);
}

TEST(DockPersistPlan, RedockFailureSaysYawSavedPositionNotUpdatedRobotNotDocked)
{
  const std::string msg = DockRedockFailedMessage(-0.9346);  // -53.55 deg

  EXPECT_NE(msg.find("saved (-54°)"), std::string::npos) << msg;
  EXPECT_NE(msg.find("position NOT updated"), std::string::npos) << msg;
  EXPECT_NE(msg.find("NOT on the dock"), std::string::npos) << msg;
}

TEST(DockPersistPlan, PositionFailureRelaysMapServersReason)
{
  const std::string msg = DockPositionFailedMessage(0.5, "only 3 RTK-Fixed /gps/fix sample(s)");

  EXPECT_NE(msg.find("re-docked"), std::string::npos) << msg;
  EXPECT_NE(msg.find("NOT updated"), std::string::npos) << msg;
  EXPECT_NE(msg.find("only 3 RTK-Fixed"), std::string::npos) << msg;
}

TEST(DockPersistPlan, YawSavedNoteNamesTheSavedHeading)
{
  const std::string note = DockYawSavedNote(M_PI / 2.0);

  EXPECT_NE(note.find("90°"), std::string::npos) << note;
  EXPECT_NE(note.find("position NOT updated"), std::string::npos) << note;
}

}  // namespace
