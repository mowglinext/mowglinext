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
/**
 * @file test_start_occupied_retry.cpp
 * @brief Regression for issue #487 — one occupied start pose forfeited a whole
 *        field.
 *
 * 2026-08-24, attempt 1 of an area-0 mow cut ZERO grass. The robot undocked to
 * ~(4.47, 4.70), which sits inside the inflated keepout of a 0.25 m obstacle
 * circle, and SmacPlanner2D (no start tolerance) answered every one of 26 plan
 * calls with "Start occupied". FollowStrip treated each refused blade-off
 * transit as an ordinary skipped swath, ran out of sub-paths, and declared the
 * area unmowable. A second attempt 13 minutes later mowed the same area to
 * 100 %.
 *
 * Three things are pinned here:
 *   1. classifyTransitFailure() tells a START_OCCUPIED refusal (about the
 *      ROBOT'S pose) apart from every other transit failure (about the GOAL),
 *      from the nav2 error_code ALONE — the planner's error_msg is never
 *      parsed, so a nav2 upgrade that rewords the exception degrades to an
 *      honest UNKNOWN rather than silently misclassifying.
 *   2. IsCoverageStartBlocked consumes the flag exactly once, so the recovery
 *      branch cannot re-fire on an unrelated later failure.
 *   3. main_tree.xml still carries the recovery branch (clear the costmaps,
 *      wait, retry) and that its ONLY motion primitive is the bounded,
 *      direction-aware EscapeStartBlocked, ordered after the blade-off + stop
 *      and before the costmap clear. The escape itself is unit-tested
 *      separately and without ROS in test_start_blocked_escape.cpp.
 */

#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/transit_failure.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::classifyTransitFailure;
using mowgli_behavior::IsCoverageStartBlocked;
using mowgli_behavior::isStartPoseBlocked;
using mowgli_behavior::TransitFailure;
using mowgli_behavior::transitFailureName;

using ComputePath = nav2_msgs::action::ComputePathToPose::Result;
using Navigate = nav2_msgs::action::NavigateToPose::Result;

// ---------------------------------------------------------------------------
// 1. Classification — pure, no ROS spinning.
// ---------------------------------------------------------------------------

// The exact field signature: bt_navigator copies ComputePathToPose's
// START_OCCUPIED (205) into the NavigateToPose result because
// navigate_to_pose.xml wires error_code_id="{compute_path_error_code}" and
// "compute_path" is in bt_navigator's default error_code_name_prefixes.
TEST(TransitFailureClassification, StartOccupiedErrorCodeIsRecognised)
{
  const auto kind = classifyTransitFailure(ComputePath::START_OCCUPIED, "Start occupied");
  EXPECT_EQ(kind, TransitFailure::kStartOccupied);
  EXPECT_TRUE(isStartPoseBlocked(kind));
  EXPECT_STREQ(transitFailureName(kind), "START_OCCUPIED");
}

// Classification is ERROR-CODE-ONLY. A robot whose bt_navigator drops
// "compute_path" from error_code_name_prefixes (or a tree without
// error_code_id) reports UNKNOWN, and the planner's wording in error_msg must
// NOT be used to recover the verdict: matching text would break silently on a
// nav2 upgrade that rewords the exception. An honest kUnknown skips the swath
// exactly as the pre-#487 code did. This test is the guard against anyone
// re-adding a message fallback.
TEST(TransitFailureClassification, StartOccupiedMessageWithoutACodeIsUnknown)
{
  const auto kind = classifyTransitFailure(
      Navigate::UNKNOWN,
      "GridBasedplugin failed to plan from (4.47, 4.70) to (2.07, 9.54): \"Start occupied\"");
  EXPECT_EQ(kind, TransitFailure::kUnknown);
  EXPECT_FALSE(isStartPoseBlocked(kind));

  const auto no_code = classifyTransitFailure(ComputePath::NONE, "START OCCUPIED");
  EXPECT_EQ(no_code, TransitFailure::kUnknown);
  EXPECT_FALSE(isStartPoseBlocked(no_code));
}

// The whole point of the change: a failure about the GOAL must stay an ordinary
// skipped swath, because that swath really is unreachable this pass.
TEST(TransitFailureClassification, GoalSideFailuresAreNotStartBlocked)
{
  EXPECT_EQ(classifyTransitFailure(ComputePath::GOAL_OCCUPIED, "Goal occupied"),
            TransitFailure::kGoalOccupied);
  EXPECT_EQ(classifyTransitFailure(ComputePath::NO_VALID_PATH, "No valid path"),
            TransitFailure::kNoValidPath);
  EXPECT_EQ(classifyTransitFailure(ComputePath::TIMEOUT, "timed out"), TransitFailure::kTimeout);
  EXPECT_EQ(classifyTransitFailure(ComputePath::TF_ERROR, "tf"), TransitFailure::kTfError);
  EXPECT_EQ(classifyTransitFailure(Navigate::TIMEOUT, ""), TransitFailure::kTimeout);

  EXPECT_FALSE(isStartPoseBlocked(classifyTransitFailure(ComputePath::GOAL_OCCUPIED, "")));
  EXPECT_FALSE(isStartPoseBlocked(classifyTransitFailure(ComputePath::NO_VALID_PATH, "")));
  EXPECT_FALSE(isStartPoseBlocked(classifyTransitFailure(ComputePath::TIMEOUT, "")));
}

// No result at all (goal rejected, or bt_navigator died before answering) must
// never be guessed into the start-blocked bucket — that bucket suppresses the
// "area not mowable" verdict and must stay evidence-backed.
TEST(TransitFailureClassification, MissingResultIsUnknownNotStartBlocked)
{
  const auto kind = classifyTransitFailure(ComputePath::NONE, "");
  EXPECT_EQ(kind, TransitFailure::kUnknown);
  EXPECT_FALSE(isStartPoseBlocked(kind));
  EXPECT_STREQ(transitFailureName(kind), "UNKNOWN");
}

// An unrelated non-zero code is "other", not a silent start-block.
TEST(TransitFailureClassification, UnrelatedCodeIsOther)
{
  const auto kind = classifyTransitFailure(ComputePath::INVALID_PLANNER, "no such planner");
  EXPECT_EQ(kind, TransitFailure::kOther);
  EXPECT_FALSE(isStartPoseBlocked(kind));
}

// (ExplicitCodeWinsOverTheMessageFallback was dropped: with the message
// fallback gone there is no fallback for a code to win over, and the "message
// text is ignored" property is already pinned by
// StartOccupiedMessageWithoutACodeIsUnknown above.)

// ---------------------------------------------------------------------------
// 2. IsCoverageStartBlocked — consumed exactly once.
// ---------------------------------------------------------------------------

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

class StartBlockedConditionTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_start_occupied_retry");
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    factory.registerNodeType<IsCoverageStartBlocked>("IsCoverageStartBlocked");
  }

  BT::Tree makeTree()
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<IsCoverageStartBlocked/>"
        "</BehaviorTree></root>";
    return factory.createTreeFromText(xml, blackboard);
  }
};

TEST_F(StartBlockedConditionTest, FailsWhenNoPassWasStartBlocked)
{
  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(StartBlockedConditionTest, FiresOnceThenConsumesTheFlag)
{
  ctx->coverage_start_blocked = true;
  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->coverage_start_blocked) << "the flag must be consumed on read";
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
      << "a second tick must not re-run the recovery for the same blocked pass";
}

// The two consumers are independent: the tree's condition node must not eat the
// per-area retirement exemption that GetNextUnmowedArea reads.
TEST_F(StartBlockedConditionTest, DoesNotConsumeTheAreaRetirementExemption)
{
  ctx->coverage_start_blocked = true;
  ctx->start_blocked_area = 0u;
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(ctx->start_blocked_area.has_value());
  EXPECT_EQ(*ctx->start_blocked_area, 0u);
}

// SAFETY (issue #487 escape motion): this condition node is the SOLE writer of
// the escape's arming token. A pass that was not start-blocked must leave the
// escape disarmed, so the motion provably cannot fire on any other failure.
TEST_F(StartBlockedConditionTest, DoesNotArmTheEscapeWhenNoPassWasStartBlocked)
{
  ASSERT_FALSE(ctx->start_blocked_escape_armed);
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(ctx->start_blocked_escape_armed)
      << "the escape motion must stay disarmed unless a CONFIRMED start-blocked pass fired";
}

TEST_F(StartBlockedConditionTest, ArmsTheEscapeOnAConfirmedBlockedPass)
{
  ctx->coverage_start_blocked = true;
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->start_blocked_escape_armed);
  EXPECT_NE(ctx->start_blocked_escape_armed_time.time_since_epoch().count(), 0)
      << "the arming must be timestamped so a stale token can be refused";
}

// ---------------------------------------------------------------------------
// 3. main_tree.xml still carries the NON-MOTION recovery branch.
// ---------------------------------------------------------------------------

namespace
{

std::string ReadMainTree()
{
  std::ifstream f(MOWGLI_MAIN_TREE_PATH);
  EXPECT_TRUE(f.is_open()) << "Cannot open " << MOWGLI_MAIN_TREE_PATH;
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Text of the <Sequence name="StartPoseBlockedRetry"> ... </Sequence> block.
std::string ExtractStartBlockedBranch(const std::string& xml)
{
  const std::string open = "<Sequence name=\"StartPoseBlockedRetry\">";
  const auto begin = xml.find(open);
  if (begin == std::string::npos)
  {
    return {};
  }
  const auto end = xml.find("</Sequence>", begin);
  if (end == std::string::npos)
  {
    return {};
  }
  return xml.substr(begin, end - begin);
}

}  // namespace

TEST(StartBlockedTreeStructure, RecoveryBranchExistsAndRetriesInsteadOfGivingUp)
{
  const std::string branch = ExtractStartBlockedBranch(ReadMainTree());
  ASSERT_FALSE(branch.empty()) << "StartPoseBlockedRetry branch missing from main_tree.xml — a "
                                  "START_OCCUPIED pass would forfeit the whole area again (#487)";
  EXPECT_NE(branch.find("<IsCoverageStartBlocked/>"), std::string::npos)
      << "the branch must be gated on the consumed start-blocked signal";
  EXPECT_NE(branch.find("<ClearCostmap/>"), std::string::npos)
      << "the retry must clear the costmaps first (helps only for a TRANSIENT obstacle; a keepout "
         "is a static filter and survives the clear)";
  EXPECT_NE(branch.find("<AlwaysFailure/>"), std::string::npos)
      << "the branch must bubble up FAILURE so FollowStripRetry re-ticks FollowStrip";
}

// SAFETY: the recovery branch is allowed EXACTLY ONE motion primitive — the
// bounded, direction-aware EscapeStartBlocked (issue #487, option B, chosen by
// the maintainer after this branch originally shipped motionless). Nothing
// else. This test used to assert the branch commanded no motion at all; it was
// updated deliberately when the escape was added, and it still forbids every
// UNBOUNDED alternative that was considered and rejected:
//
//   * <BackUp> — fixed direction. Correct mid-mow, WRONG right after an undock,
//     which is the reported case: the robot reversed into the blocked cell, so
//     reversing again drives it deeper.
//   * <Spin> — sweeps the chassis through cells we know even less about while
//     standing on a lethal one.
//   * <NavigateToPose> — needs a plan, and "no plan exists from here" is the
//     entire failure being recovered from.
TEST(StartBlockedTreeStructure, RecoveryBranchMotionIsOnlyTheBoundedEscape)
{
  const std::string branch = ExtractStartBlockedBranch(ReadMainTree());
  ASSERT_FALSE(branch.empty());
  EXPECT_EQ(branch.find("<BackUp"), std::string::npos)
      << "a fixed-direction BackUp drives DEEPER after an undock (#487)";
  EXPECT_EQ(branch.find("<Spin"), std::string::npos)
      << "no spin escape — see the rejected options on #487";
  EXPECT_EQ(branch.find("<NavigateToPose"), std::string::npos)
      << "no drive-out escape: there is no plan from this pose, which is the bug";
  EXPECT_NE(branch.find("<EscapeStartBlocked/>"), std::string::npos)
      << "the bounded, direction-aware escape must be present (#487 option B)";
  EXPECT_NE(branch.find("<SetMowerEnabled enabled=\"false\"/>"), std::string::npos)
      << "the blade must be OFF while the robot sits blocked";
  EXPECT_NE(branch.find("<StopMoving/>"), std::string::npos);
}

// Ordering is a safety property, not cosmetics: the blade must be commanded off
// and the robot brought to a stop BEFORE any escape command is issued, and the
// costmaps must be cleared AFTER the robot has moved so the retry plans from
// the pose it actually ended up in.
TEST(StartBlockedTreeStructure, EscapeRunsAfterBladeOffAndStopAndBeforeTheClear)
{
  const std::string branch = ExtractStartBlockedBranch(ReadMainTree());
  ASSERT_FALSE(branch.empty());

  const auto blade_off = branch.find("<SetMowerEnabled enabled=\"false\"/>");
  const auto stop = branch.find("<StopMoving/>");
  const auto escape = branch.find("<EscapeStartBlocked/>");
  const auto clear = branch.find("<ClearCostmap/>");

  ASSERT_NE(blade_off, std::string::npos);
  ASSERT_NE(stop, std::string::npos);
  ASSERT_NE(escape, std::string::npos);
  ASSERT_NE(clear, std::string::npos);

  EXPECT_LT(blade_off, escape) << "the blade must be commanded off before the robot moves";
  EXPECT_LT(stop, escape) << "the robot must be stopped before the escape takes the wire";
  EXPECT_LT(escape, clear) << "clear the costmaps AFTER the escape, at the new pose";
}
