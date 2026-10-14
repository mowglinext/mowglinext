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
 * @file test_incomplete_resume.cpp
 * @brief Play resumes the area that was in progress when a session ended unfinished.
 *
 * Field 2026-10-09: a single-area mow was at 98 %, the coverage ended in
 * COVERAGE_FAILED_DOCKING, and EndSession wiped every area's progress; Play then started the
 * lawn over, area 1 included. EndSession(keep_incomplete) now keeps the progress when the session
 * did not finish, and a plain Start continues it (single-area target included) unless it is more
 * than a day old.
 */

#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::EndSession;
using mowgli_behavior::hasRecordedCoverageProgress;
using mowgli_behavior::incompleteResumeExpired;
using mowgli_behavior::keepsSingleAreaOnStart;

namespace
{

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

}  // namespace

// ---------------------------------------------------------------------------
// The pure decisions
// ---------------------------------------------------------------------------

TEST(IncompleteResumeHelpers, NothingRecordedMeansNothingToKeep)
{
  BTContext ctx;
  EXPECT_FALSE(hasRecordedCoverageProgress(ctx));
  (void)ctx.area_completed_swaths[1];  // an empty set is no progress
  EXPECT_FALSE(hasRecordedCoverageProgress(ctx));
}

TEST(IncompleteResumeHelpers, AnyKindOfProgressCounts)
{
  {
    BTContext ctx;
    ctx.area_completed_swaths[1] = {0, 1, 2};
    EXPECT_TRUE(hasRecordedCoverageProgress(ctx));
  }
  {
    BTContext ctx;
    ctx.area_resume_pose_index[1] = 12156;
    EXPECT_TRUE(hasRecordedCoverageProgress(ctx));
  }
  {
    BTContext ctx;
    ctx.completed_areas.insert(0);
    EXPECT_TRUE(hasRecordedCoverageProgress(ctx));
  }
}

TEST(IncompleteResumeHelpers, StartKeepsTheSingleAreaForAPauseOrAKeptIncompleteSession)
{
  // The operator's Pause (state IDLE) always kept it.
  EXPECT_TRUE(keepsSingleAreaOnStart("IDLE", false));
  // A session that ended unfinished with its progress kept keeps it too, whatever it published
  // last (a failed coverage dock and HOME both end in IDLE_DOCKED).
  EXPECT_TRUE(keepsSingleAreaOnStart("IDLE_DOCKED", true));
  // A plain Start from a finished or never-started session still means "the whole lawn".
  EXPECT_FALSE(keepsSingleAreaOnStart("IDLE_DOCKED", false));
  EXPECT_FALSE(keepsSingleAreaOnStart("CHARGING", false));
}

// The combined decision the START handler uses: a Pause, a manual Resume out of a charge hold and a
// kept unfinished session all continue the same run; everything else means "mow the lawn".
TEST(IncompleteResumeHelpers, TheStartDecisionCombinesPauseChargeHoldAndKeptProgress)
{
  using mowgli_behavior::startClearsSingleAreaMode;
  EXPECT_FALSE(startClearsSingleAreaMode("IDLE"));
  EXPECT_FALSE(startClearsSingleAreaMode("CHARGING"));
  EXPECT_FALSE(startClearsSingleAreaMode("IDLE_DOCKED", true));
  EXPECT_TRUE(startClearsSingleAreaMode("IDLE_DOCKED"));
  EXPECT_TRUE(startClearsSingleAreaMode("IDLE_DOCKED", false));
}

TEST(IncompleteResumeHelpers, KeptProgressExpiresAfterADay)
{
  const auto kept = std::chrono::steady_clock::now();
  EXPECT_FALSE(incompleteResumeExpired(kept, kept + std::chrono::hours(23)));
  EXPECT_FALSE(incompleteResumeExpired(kept, kept + mowgli_behavior::kIncompleteResumeMaxAge));
  EXPECT_TRUE(incompleteResumeExpired(kept,
                                      kept + mowgli_behavior::kIncompleteResumeMaxAge +
                                          std::chrono::minutes(1)));
}

// ---------------------------------------------------------------------------
// EndSession
// ---------------------------------------------------------------------------

class EndSessionKeepTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_incomplete_resume");
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    factory.registerNodeType<EndSession>("EndSession");
  }

  void TearDown() override
  {
    std::remove(resume_path.c_str());
  }

  /// A session as the field one was: a targeted run on area 1, three units done, the cursor at 98
  /// %.
  void unfinishedTargetedSession()
  {
    ctx->current_command = 1;
    ctx->single_area_target = 1;
    ctx->completed_areas.insert(0);
    ctx->area_completed_swaths[1] = {0, 1, 2};
    ctx->area_resume_pose_index[1] = 12156;
    ctx->area_plan_fingerprint[1] = 0xABCDEF;
    ctx->area_path_pose_count[1] = 12347;
    ctx->area_swath_count[1] = 4;
    ctx->attempted_areas.insert(1);
    ctx->incomplete_retired_areas.insert(1);
    ctx->area_attempt_count[1] = 5;
    ctx->area_last_coverage[1] = 3.0f;
  }

  BT::NodeStatus endSession(bool keep_incomplete)
  {
    const std::string xml = std::string(
                                "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">"
                                "<EndSession keep_incomplete=\"") +
                            (keep_incomplete ? "true" : "false") + "\"/></BehaviorTree></root>";
    auto tree = factory.createTreeFromText(xml, blackboard);
    return tree.tickOnce();
  }

  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::string resume_path = ::testing::TempDir() + "test_incomplete_resume_state.txt";
};

TEST_F(EndSessionKeepTest, AFinishedSessionStillWipesEverything)
{
  unfinishedTargetedSession();

  ASSERT_EQ(endSession(false), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(ctx->area_completed_swaths.empty());
  EXPECT_TRUE(ctx->area_resume_pose_index.empty());
  EXPECT_TRUE(ctx->completed_areas.empty());
  EXPECT_TRUE(ctx->area_plan_fingerprint.empty());
  EXPECT_FALSE(ctx->single_area_target.has_value());
  EXPECT_FALSE(ctx->resume_after_incomplete_end);
}

TEST_F(EndSessionKeepTest, AnUnfinishedSessionKeepsItsProgressAndTheTargetedArea)
{
  unfinishedTargetedSession();

  ASSERT_EQ(endSession(true), BT::NodeStatus::SUCCESS);

  // What describes the lawn stays, exactly as it was.
  EXPECT_EQ(ctx->area_completed_swaths.at(1), (std::set<std::size_t>{0, 1, 2}));
  EXPECT_EQ(ctx->area_resume_pose_index.at(1), 12156u);
  EXPECT_EQ(ctx->completed_areas, (std::set<uint32_t>{0}));
  EXPECT_EQ(ctx->area_plan_fingerprint.at(1), 0xABCDEFu)
      << "a changed fingerprint would discard the progress";
  EXPECT_EQ(ctx->area_path_pose_count.at(1), 12347u);
  EXPECT_EQ(ctx->area_swath_count.at(1), 4u);
  ASSERT_TRUE(ctx->single_area_target.has_value());
  EXPECT_EQ(*ctx->single_area_target, 1u);
  EXPECT_TRUE(ctx->resume_after_incomplete_end);
  EXPECT_NE(ctx->resume_after_incomplete_end_time, std::chrono::steady_clock::time_point{});

  // What belonged to the run that failed is reset, so the retired area is tried again.
  EXPECT_TRUE(ctx->attempted_areas.empty());
  EXPECT_TRUE(ctx->incomplete_retired_areas.empty());
  EXPECT_TRUE(ctx->area_attempt_count.empty());
  EXPECT_TRUE(ctx->area_last_coverage.empty());

  // A kept file must never carry a START: a restart before the next Play would mow on its own.
  EXPECT_EQ(ctx->current_command, 0u);
}

TEST_F(EndSessionKeepTest, TheKeptProgressIsWrittenToTheResumeFileWithoutAStartCommand)
{
  ctx->coverage_resume_path = resume_path;
  unfinishedTargetedSession();

  ASSERT_EQ(endSession(true), BT::NodeStatus::SUCCESS);

  std::ifstream in(resume_path);
  ASSERT_TRUE(in.good()) << "the kept progress was not written to " << resume_path;
  std::stringstream text;
  text << in.rdbuf();
  const std::string file = text.str();
  EXPECT_NE(file.find("current_command 0"), std::string::npos) << file;
  EXPECT_NE(file.find("single_area_target 1"), std::string::npos) << file;
  EXPECT_NE(file.find("area 1 "), std::string::npos) << file;
}

TEST_F(EndSessionKeepTest, WithNothingRecordedThereIsNothingToKeep)
{
  ctx->current_command = 1;
  ctx->single_area_target = 1;  // a targeted run that never mowed a swath

  ASSERT_EQ(endSession(true), BT::NodeStatus::SUCCESS);

  EXPECT_FALSE(ctx->resume_after_incomplete_end);
  EXPECT_FALSE(ctx->single_area_target.has_value());
}

TEST_F(EndSessionKeepTest, TheNextFinishedSessionClearsTheFlagAgain)
{
  unfinishedTargetedSession();
  ASSERT_EQ(endSession(true), BT::NodeStatus::SUCCESS);
  ASSERT_TRUE(ctx->resume_after_incomplete_end);

  ASSERT_EQ(endSession(false), BT::NodeStatus::SUCCESS);

  EXPECT_FALSE(ctx->resume_after_incomplete_end);
  EXPECT_TRUE(ctx->area_completed_swaths.empty());
}

// ---------------------------------------------------------------------------
// The tree
// ---------------------------------------------------------------------------

#ifdef MOWGLI_MAIN_TREE_PATH
TEST(IncompleteResumeTree, OnlyTheUnfinishedEndingsKeepTheProgress)
{
  std::ifstream in(MOWGLI_MAIN_TREE_PATH);
  ASSERT_TRUE(in.good());
  std::stringstream text;
  text << in.rdbuf();
  const std::string tree = text.str();

  // The iterator must not be built from a temporary regex (deleted overload).
  const std::regex keep_re(R"(<EndSession\s+keep_incomplete="true"\s*/>)");
  std::size_t keep = 0;
  for (std::sregex_iterator it(tree.begin(), tree.end(), keep_re), end; it != end; ++it)
  {
    ++keep;
  }
  // FailedCoverageDock and HomeSequence.
  EXPECT_EQ(keep, 2u);

  // The normal-completion ending must keep wiping: find CoverageCompleteDock's EndSession.
  const auto complete = tree.find("name=\"CoverageCompleteDock\"");
  ASSERT_NE(complete, std::string::npos);
  const auto end_session = tree.find("<EndSession", complete);
  ASSERT_NE(end_session, std::string::npos);
  EXPECT_EQ(tree.compare(end_session, std::string("<EndSession/>").size(), "<EndSession/>"), 0)
      << "a finished mow must still wipe its progress";
}
#endif
