// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for dock_persist_plan.hpp — the set_docking_point requests of the
// dock calibration and the verdict for each way it can end. Regressions
// pinned: the write happens with the robot OFF the dock, so it must never be a
// live GPS position capture (map_server rejects those unless is_charging —
// every run failed, 2026-09-17); and a confirmation re-dock that stops short
// must not turn a saved calibration into a "failure" (field test, same day).

#include <string>

#include "mowgli_localization/dock_persist_plan.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_localization::DecideDockVerdict;
using mowgli_localization::DockPoseStep;
using mowgli_localization::DockSaved;
using mowgli_localization::DockSavedNote;
using mowgli_localization::DockSavedPose;
using mowgli_localization::DockYawStep;
using mowgli_localization::kDockRetryNoChargeOnRedock;
using mowgli_localization::kDockRetryNone;
using mowgli_localization::kDockRetryPersistFailed;
using mowgli_localization::kSetDockYawMotion;

DockSavedPose saved_both()
{
  DockSavedPose pose;
  pose.saved = DockSaved::YAW_AND_POSITION;
  pose.x = 6.263;
  pose.y = 2.811;
  pose.yaw_rad = -0.9346;  // -53.55 deg
  return pose;
}

DockSavedPose saved_yaw_only()
{
  DockSavedPose pose;
  pose.saved = DockSaved::YAW_ONLY;
  pose.yaw_rad = -0.9346;
  pose.position_skip_reason = "only 3 RTK-Fixed /gps/fix sample(s)";
  return pose;
}

TEST(DockPersistPlan, PoseStepUsesThePendingAntennaNeverALiveGpsCapture)
{
  // Arrange / Act
  const auto req = DockPoseStep(-0.9346);

  // Assert
  EXPECT_TRUE(req.use_pending_antenna);
  EXPECT_FALSE(req.use_gps_position);
  EXPECT_FALSE(req.preserve_position);
  EXPECT_EQ(req.yaw_source, kSetDockYawMotion);
  EXPECT_DOUBLE_EQ(req.yaw_rad, -0.9346);
}

TEST(DockPersistPlan, YawStepFallbackPreservesThePositionAndNeverCapturesGps)
{
  const auto req = DockYawStep(-0.9346);

  EXPECT_TRUE(req.preserve_position);
  EXPECT_FALSE(req.use_gps_position);
  EXPECT_FALSE(req.use_pending_antenna);
  EXPECT_EQ(req.yaw_source, kSetDockYawMotion);
  EXPECT_DOUBLE_EQ(req.yaw_rad, -0.9346);
}

TEST(DockPersistPlan, BothSavedAndRedockedIsAPlainSuccess)
{
  const auto v = DecideDockVerdict(saved_both(), /*redock_verified=*/true);

  EXPECT_TRUE(v.success);
  EXPECT_EQ(v.retry_reason, kDockRetryNone);
  EXPECT_NE(v.message.find("yaw -54° and position (6.263, 2.811)"), std::string::npos) << v.message;
  EXPECT_NE(v.message.find("restart mowgli-ros2"), std::string::npos) << v.message;
}

TEST(DockPersistPlan, BothSavedButRedockShortIsASuccessThatSaysTheRobotIsNotDocked)
{
  const auto v = DecideDockVerdict(saved_both(), /*redock_verified=*/false);

  EXPECT_TRUE(v.success);
  EXPECT_EQ(v.retry_reason, kDockRetryNoChargeOnRedock);
  EXPECT_NE(v.message.find("NOT on the dock"), std::string::npos) << v.message;
  EXPECT_NE(v.message.find("not a calibration failure"), std::string::npos) << v.message;
  EXPECT_NE(v.message.find("restart mowgli-ros2"), std::string::npos) << v.message;
}

TEST(DockPersistPlan, YawOnlyIsAFailureThatListsTheYawAsSavedAndWhyThePositionWasNot)
{
  const auto v = DecideDockVerdict(saved_yaw_only(), /*redock_verified=*/true);

  EXPECT_FALSE(v.success);
  EXPECT_EQ(v.retry_reason, kDockRetryPersistFailed);
  EXPECT_NE(v.message.find("yaw -54° ONLY"), std::string::npos) << v.message;
  EXPECT_NE(v.message.find("position NOT updated"), std::string::npos) << v.message;
  EXPECT_NE(v.message.find("only 3 RTK-Fixed"), std::string::npos) << v.message;
  EXPECT_NE(v.message.find("is charging"), std::string::npos) << v.message;
}

TEST(DockPersistPlan, YawOnlyWithoutRedockSaysTheRobotIsNotDocked)
{
  const auto v = DecideDockVerdict(saved_yaw_only(), /*redock_verified=*/false);

  EXPECT_FALSE(v.success);
  EXPECT_EQ(v.retry_reason, kDockRetryNoChargeOnRedock);
  EXPECT_NE(v.message.find("NOT on the dock"), std::string::npos) << v.message;
}

TEST(DockPersistPlan, NothingSavedIsAPersistFailure)
{
  const auto v = DecideDockVerdict(DockSavedPose{}, true);

  EXPECT_FALSE(v.success);
  EXPECT_EQ(v.retry_reason, kDockRetryPersistFailed);
}

TEST(DockPersistPlan, AbortNoteStatesWhatIsAlreadySaved)
{
  const std::string note = DockSavedNote(saved_both());

  EXPECT_NE(note.find("Already saved: yaw -54° and position"), std::string::npos) << note;
  EXPECT_NE(note.find("NOT on the dock"), std::string::npos) << note;
}

}  // namespace
