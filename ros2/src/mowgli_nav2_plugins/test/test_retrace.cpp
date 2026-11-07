// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// retrace.hpp: the recovery that undoes the last piece the robot drove instead of reversing blindly
// along its heading (field 2026-10-26: Nav2's BackUp took the mower from 0.21 m to 0.57 m outside
// the boundary, a lethal violation).

#include <cmath>

#include "mowgli_nav2_plugins/retrace.hpp"
#include <gtest/gtest.h>

namespace
{
using mowgli_nav2_plugins::retrace::Breadcrumbs;
using mowgli_nav2_plugins::retrace::Limits;
using mowgli_nav2_plugins::retrace::Plan;
using mowgli_nav2_plugins::retrace::Pose2D;
using mowgli_nav2_plugins::retrace::Refusal;

// Drive a straight line from `from` to `to` (x only) at heading `yaw`, one breadcrumb per 2 cm.
void driveX(Breadcrumbs& crumbs, double from, double to, double yaw)
{
  const double step = from < to ? 0.02 : -0.02;
  for (double x = from; step > 0 ? x <= to + 1e-9 : x >= to - 1e-9; x += step)
    crumbs.add({x, 0.0, yaw});
}
}  // namespace

TEST(Retrace, UndoesTheLastPieceStraightBackAfterDrivingForward)
{
  Breadcrumbs crumbs;
  driveX(crumbs, 0.0, 1.0, 0.0);
  const Plan plan = crumbs.plan({1.0, 0.0, 0.0}, 0.30);
  ASSERT_EQ(plan.refusal, Refusal::kNone);
  EXPECT_NEAR(plan.target.x, 0.70, 1e-6);
  EXPECT_NEAR(plan.distance_m, 0.30, 1e-6);
  EXPECT_NEAR(plan.ux, -1.0, 1e-9) << "back along the line the robot came from";
  EXPECT_NEAR(plan.uy, 0.0, 1e-9);
}

TEST(Retrace, RefusesWhenTheLastPieceWasDrivenInReverse)
{
  Breadcrumbs crumbs;
  driveX(crumbs, 1.0, 0.7, 0.0);  // heading +x, driven backwards
  const Plan plan = crumbs.plan({0.7, 0.0, 0.0}, 0.30);
  EXPECT_EQ(plan.refusal, Refusal::kWrongDirection)
      << "undoing a reverse means driving forward into ground nobody is watching";
}

TEST(Retrace, RefusesAfterTheRobotHasTurnedInPlace)
{
  Breadcrumbs crumbs;
  driveX(crumbs, 0.0, 1.0, 0.0);
  // 90 degrees to the left: the way back is now to the robot's side, not behind it.
  const Plan plan = crumbs.plan({1.0, 0.0, M_PI / 2.0}, 0.30);
  EXPECT_EQ(plan.refusal, Refusal::kNotStraight);
}

TEST(Retrace, ToleratesASmallHeadingErrorButNotALarge)
{
  Breadcrumbs crumbs;
  driveX(crumbs, 0.0, 1.0, 0.0);
  EXPECT_EQ(crumbs.plan({1.0, 0.0, 10.0 * M_PI / 180.0}, 0.30).refusal, Refusal::kNone);
  EXPECT_EQ(crumbs.plan({1.0, 0.0, -10.0 * M_PI / 180.0}, 0.30).refusal, Refusal::kNone);
  EXPECT_EQ(crumbs.plan({1.0, 0.0, 30.0 * M_PI / 180.0}, 0.30).refusal, Refusal::kNotStraight);
}

TEST(Retrace, RefusesACurvedPiece)
{
  Breadcrumbs crumbs;
  // A circle of radius 0.25 m: the last 0.30 m turns the robot by 1.2 rad, so the way straight back
  // is 35 degrees off its rear axis.
  const double radius = 0.25;
  for (double angle = 0.0; angle <= 2.0 + 1e-9; angle += 0.1)
    crumbs.add({radius * std::sin(angle), radius * (1.0 - std::cos(angle)), angle});
  const double end = 2.0;
  const Pose2D current{radius * std::sin(end), radius * (1.0 - std::cos(end)), end};
  const Plan plan = crumbs.plan(current, 0.30);
  EXPECT_EQ(plan.refusal, Refusal::kNotStraight);
}

TEST(Retrace, RefusesWithoutEnoughHistory)
{
  Breadcrumbs empty;
  EXPECT_EQ(empty.plan({0.0, 0.0, 0.0}, 0.30).refusal, Refusal::kNoHistory);

  Breadcrumbs short_history;
  driveX(short_history, 0.0, 0.10, 0.0);
  EXPECT_EQ(short_history.plan({0.10, 0.0, 0.0}, 0.30).refusal, Refusal::kNoHistory)
      << "only 0.10 m was ever driven: it cannot be undone by 0.30 m";
}

TEST(Retrace, AfterASuccessfulRetraceTheUndoneGroundIsForgotten)
{
  Breadcrumbs crumbs;
  driveX(crumbs, 0.0, 1.0, 0.0);
  ASSERT_EQ(crumbs.plan({1.0, 0.0, 0.0}, 0.30).refusal, Refusal::kNone);
  crumbs.retreat(0.30);
  // The robot now stands at 0.70: the next retrace goes back from there, not over the old end.
  const Plan again = crumbs.plan({0.70, 0.0, 0.0}, 0.30);
  ASSERT_EQ(again.refusal, Refusal::kNone);
  EXPECT_NEAR(again.target.x, 0.40, 1e-6);
}

TEST(Retrace, KeepsOnlyTheConfiguredHistory)
{
  Limits limits;
  limits.history_m = 1.0;
  Breadcrumbs crumbs(limits);
  driveX(crumbs, 0.0, 3.0, 0.0);
  // 3 m were driven, 1 m is kept: a retrace of 0.30 m still works, one of 1.5 m cannot.
  EXPECT_EQ(crumbs.plan({3.0, 0.0, 0.0}, 0.30).refusal, Refusal::kNone);
  EXPECT_EQ(crumbs.plan({3.0, 0.0, 0.0}, 1.5).refusal, Refusal::kNoHistory);
}

TEST(Retrace, DropsBreadcrumbsCloserThanTheMinimumStep)
{
  Breadcrumbs crumbs;
  crumbs.add({0.0, 0.0, 0.0});
  crumbs.add({0.005, 0.0, 0.0});
  crumbs.add({0.010, 0.0, 0.0});
  EXPECT_EQ(crumbs.size(), 1U);
  crumbs.add({0.03, 0.0, 0.0});
  EXPECT_EQ(crumbs.size(), 2U);
}

TEST(Retrace, ProgressIsMeasuredAlongTheRetraceDirection)
{
  const Pose2D start{1.0, 0.0, 0.0};
  EXPECT_NEAR(mowgli_nav2_plugins::retrace::progressAlong(start, {0.8, 0.02, 0.0}, -1.0, 0.0),
              0.20,
              1e-9);
  // Sideways drift does not count as progress.
  EXPECT_NEAR(mowgli_nav2_plugins::retrace::progressAlong(start, {1.0, 0.3, 0.0}, -1.0, 0.0),
              0.0,
              1e-9);
}

TEST(Retrace, AJumpInThePoseIsNotDriving)
{
  Breadcrumbs crumbs;
  driveX(crumbs, 0.0, 1.0, 0.0);
  // The mower is carried 3 m away and set down: the old track says nothing about the ground
  // behind it now, so the history starts over and a reverse has nothing to undo.
  crumbs.add({4.0, 0.0, 0.0});
  EXPECT_EQ(crumbs.size(), 1U);
  EXPECT_EQ(crumbs.plan({4.0, 0.0, 0.0}, 0.30).refusal, Refusal::kNoHistory);
}
