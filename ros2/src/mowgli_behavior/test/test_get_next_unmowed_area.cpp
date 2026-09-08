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
 * @file test_get_next_unmowed_area.cpp
 * @brief Regression test: the mow-selection path SKIPS navigation-only areas.
 *
 * GetNextUnmowedArea iterates the map's areas via get_mowing_area. A
 * navigation-only zone (is_navigation_area=true) is a transit corridor, not a
 * mowing target — the blades must never run inside it. map_server still returns
 * these areas (success=true) because the obstacle tracker needs their geometry,
 * so the skip lives on the BT selection side. These tests stand up a REAL
 * in-process get_mowing_area service (no robot, no mocked interfaces) and tick
 * the StatefulActionNode to verify a nav-only area is never selected.
 *
 * It also covers TARGETED runs (~/start_in_area, "mow only this area"): the
 * single-area constraint is session state, so a run must not roll over into
 * another area when the BT re-enters MowingSequence after the requested area
 * completes (field regression, 2026-08-24).
 */

#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_behavior/coverage_orientation_service.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::clearSingleAreaMode;
using mowgli_behavior::EndSession;
using mowgli_behavior::GetNextUnmowedArea;
using GetMowingArea = mowgli_interfaces::srv::GetMowingArea;

// ---------------------------------------------------------------------------
// Global ROS2 init/shutdown
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

// ---------------------------------------------------------------------------
// One entry the fake map_server returns per index: name + nav-only flag.
// index past the last entry → success=false (matches real map_server).
// ---------------------------------------------------------------------------
struct AreaEntry
{
  std::string name;
  bool is_navigation_area;
};

class GetNextUnmowedAreaTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  rclcpp::Node::SharedPtr server_node;
  rclcpp::Service<GetMowingArea>::SharedPtr service;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::map<uint32_t, AreaEntry> areas;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_get_next_unmowed_area");
    ctx->helper_node = rclcpp::Node::make_shared("test_get_next_unmowed_area_helper");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);

    factory.registerNodeType<GetNextUnmowedArea>("GetNextUnmowedArea");
    factory.registerNodeType<EndSession>("EndSession");

    server_node = rclcpp::Node::make_shared("fake_map_server");
    service = server_node->create_service<GetMowingArea>(
        "/map_server_node/get_mowing_area",
        [this](const std::shared_ptr<GetMowingArea::Request> req,
               std::shared_ptr<GetMowingArea::Response> resp)
        {
          auto it = areas.find(req->index);
          if (it == areas.end())
          {
            resp->success = false;  // index past the last defined area
            return;
          }
          resp->area.name = it->second.name;
          resp->area.is_navigation_area = it->second.is_navigation_area;
          resp->success = true;
        });

    executor.add_node(ctx->helper_node);
    executor.add_node(server_node);
  }

  /// Wait for the helper-side client to discover the fake service.
  void waitForService()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      auto client =
          ctx->helper_node->create_client<GetMowingArea>("/map_server_node/get_mowing_area");
      if (client->service_is_ready())
      {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    FAIL() << "fake get_mowing_area service was never discovered";
  }

  /// Tick the node to completion, spinning the executor between ticks so the
  /// async service round-trips complete. Returns the terminal status.
  BT::NodeStatus tickToCompletion(BT::Tree& tree)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (std::chrono::steady_clock::now() < deadline)
    {
      status = tree.tickOnce();
      if (status != BT::NodeStatus::RUNNING)
      {
        break;
      }
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return status;
  }

  BT::Tree makeTree(uint32_t max_areas)
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<GetNextUnmowedArea max_areas=\"" +
        std::to_string(max_areas) +
        "\" area_index=\"{area_index}\"/>"
        "</BehaviorTree></root>";
    return factory.createTreeFromText(xml, blackboard);
  }

  /// A bare <EndSession/> tree — the real session-boundary node, so the
  /// "cleared at session end" assertion exercises production code rather than
  /// a hand-rolled reset.
  BT::Tree makeEndSessionTree()
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<EndSession/>"
        "</BehaviorTree></root>";
    return factory.createTreeFromText(xml, blackboard);
  }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// A navigation-only area at index 0 followed by a mowing area at index 1: the
// selection must SKIP index 0 and pick index 1.
TEST_F(GetNextUnmowedAreaTest, SkipsNavigationOnlyAreaAndSelectsMowingArea)
{
  areas[0] = {"front_path", /*is_navigation_area=*/true};
  areas[1] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);

  uint32_t selected = 99;
  ASSERT_TRUE(blackboard->get("area_index", selected));
  EXPECT_EQ(selected, 1u) << "must skip the nav-only area 0 and select mowing area 1";
  EXPECT_EQ(ctx->current_area, 1);
  // The skipped nav area is marked attempted so it is not re-evaluated.
  EXPECT_GT(ctx->attempted_areas.count(0u), 0u);
}

// The ONLY area is navigation-only: nothing is mowable → FAILURE, and the nav
// area is never selected.
TEST_F(GetNextUnmowedAreaTest, NavigationOnlyAreaIsNeverMowed)
{
  areas[0] = {"perimeter_corridor", /*is_navigation_area=*/true};
  waitForService();

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->current_area, -1) << "no area should have been selected for mowing";
  EXPECT_GT(ctx->attempted_areas.count(0u), 0u);
}

// Control: an ordinary mowing area at index 0 is selected as before (the skip
// must not regress normal selection).
TEST_F(GetNextUnmowedAreaTest, SelectsMowingAreaAtIndexZero)
{
  areas[0] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);

  uint32_t selected = 99;
  ASSERT_TRUE(blackboard->get("area_index", selected));
  EXPECT_EQ(selected, 0u);
  EXPECT_EQ(ctx->current_area, 0);
}

// ---------------------------------------------------------------------------
// Issue #487 — a START_OCCUPIED pass must not retire the area.
//
// FollowStrip sets ctx->start_blocked_area when a whole pass ended with every
// blade-off sub-path transit refused because the ROBOT'S OWN pose is a lethal
// or keepout cell and ZERO swaths were mowed. Such a pass never had a chance to
// make progress; charging it to the no-progress retirement counter is what
// forfeited a mowable field at 0 % coverage on 2026-08-24.
// ---------------------------------------------------------------------------

// Before the fix, kMaxAreaAttempts (5) consecutive zero-progress dispatches
// retired the area. With the exemption, five start-blocked dispatches all still
// select the area.
TEST_F(GetNextUnmowedAreaTest, StartBlockedPassesDoNotBurnTheNoProgressBudget)
{
  areas[0] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  for (uint32_t attempt = 0; attempt < BTContext::kMaxAreaAttempts; ++attempt)
  {
    // Simulate the previous FollowStrip pass ending start-pose-blocked.
    ctx->start_blocked_area = 0u;
    auto tree = makeTree(/*max_areas=*/5);
    EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS)
        << "dispatch " << attempt << " must still select the area";
    EXPECT_EQ(ctx->current_area, 0) << "dispatch " << attempt;
  }
  EXPECT_EQ(ctx->attempted_areas.count(0u), 0u)
      << "a run of START_OCCUPIED passes must not retire a mowable area (#487)";
}

// ...but the exemption is BOUNDED. A robot genuinely parked on a lethal cell
// forever must still give up and dock rather than loop.
TEST_F(GetNextUnmowedAreaTest, StartBlockedExemptionIsBoundedSoTheAreaStillRetires)
{
  areas[0] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  const uint32_t kMaxDispatches =
      BTContext::kMaxStartBlockedAttempts + BTContext::kMaxAreaAttempts + 2;
  bool retired = false;
  for (uint32_t attempt = 0; attempt < kMaxDispatches && !retired; ++attempt)
  {
    ctx->start_blocked_area = 0u;
    auto tree = makeTree(/*max_areas=*/5);
    tickToCompletion(tree);
    retired = ctx->attempted_areas.count(0u) > 0;
  }
  EXPECT_TRUE(retired) << "the start-blocked exemption must be bounded — an area the robot can "
                          "never plan from has to retire so the session can dock";
}

// The flag describes ONE finished pass and is consumed by the dispatch that
// reads it, so a single blocked pass buys exactly one exemption.
TEST_F(GetNextUnmowedAreaTest, StartBlockedFlagIsConsumedByOneDispatch)
{
  areas[0] = {"lawn", /*is_navigation_area=*/false};
  waitForService();

  ctx->start_blocked_area = 0u;
  auto tree = makeTree(/*max_areas=*/5);
  ASSERT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);

  EXPECT_FALSE(ctx->start_blocked_area.has_value());
  EXPECT_EQ(ctx->area_start_blocked_count[0u], 1u);
  EXPECT_EQ(ctx->area_attempt_count[0u], 0u)
      << "the exempted dispatch must not have advanced the no-progress counter";
}

// ---------------------------------------------------------------------------
// Targeted run (~/start_in_area): mow ONE area, then stop — no roll-over.
//
// Field log 2026-08-24: "StartInArea: received area=1" → "targeted run — mowing
// only area 1 (single-area mode)" → ~56 min of mowing → "area 0 selected" with
// NO targeted-run line. The BT re-enters MowingSequence when the targeted area
// completes, so GetNextUnmowedArea::onStart() runs again; the clip used to live
// only in members onStart() resets plus a one-shot optional consumed on the
// first entry, so the second entry iterated from area 0 and the robot mowed an
// area the operator never selected.
// ---------------------------------------------------------------------------

// The regression itself: the dispatch that follows the targeted area's
// completion must NOT select another area.
TEST_F(GetNextUnmowedAreaTest, TargetedRunDoesNotRollOverToTheNextArea)
{
  areas[0] = {"front_lawn", /*is_navigation_area=*/false};
  areas[1] = {"back_lawn", /*is_navigation_area=*/false};
  areas[2] = {"side_strip", /*is_navigation_area=*/false};
  waitForService();

  // Operator picks area 1 in the GUI (~/start_in_area).
  ctx->target_area_index = 1;
  {
    auto tree = makeTree(/*max_areas=*/5);
    ASSERT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(ctx->current_area, 1);
  }
  // The one-shot request is consumed, but the constraint is now session state.
  EXPECT_FALSE(ctx->target_area_index.has_value());
  ASSERT_TRUE(ctx->single_area_target.has_value());
  EXPECT_EQ(*ctx->single_area_target, 1u);

  // FollowStrip mows area 1 to completion, and the BT re-enters MowingSequence.
  ctx->completed_areas.insert(1u);
  {
    auto tree = makeTree(/*max_areas=*/5);
    EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::FAILURE)
        << "a targeted run must end after its area, not roll over to another";
  }
  EXPECT_EQ(ctx->current_area, 1) << "no other area may be selected for mowing";
  uint32_t selected = 99;
  ASSERT_TRUE(blackboard->get("area_index", selected));
  EXPECT_EQ(selected, 1u);
  // FAILURE + coverage_all_complete is the CLEAN exit: the tree routes it to
  // MOWING_COMPLETE + dock (CoverageCompleteDock), not COVERAGE_FAILED_DOCKING.
  EXPECT_TRUE(ctx->coverage_all_complete)
      << "a finished targeted run must dock via MOWING_COMPLETE, not report a coverage failure";
}

// An explicitly targeted area is re-mown even when it is already marked
// completed/attempted this session (the operator asked for it on purpose).
TEST_F(GetNextUnmowedAreaTest, TargetedRunReMowsAnAlreadyCompletedArea)
{
  areas[0] = {"front_lawn", /*is_navigation_area=*/false};
  areas[1] = {"back_lawn", /*is_navigation_area=*/false};
  waitForService();

  ctx->completed_areas.insert(1u);
  ctx->attempted_areas.insert(1u);

  ctx->target_area_index = 1;
  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS)
      << "an explicit re-mow request must clear the stale completed/attempted flags";
  EXPECT_EQ(ctx->current_area, 1);
}

// ...but that erase is tied to the ONE-SHOT request, not to the session flag:
// repeating it on every onStart() would wipe the completion the targeted area
// just earned and re-mow it forever.
TEST_F(GetNextUnmowedAreaTest, TargetedRunDoesNotReMowItsOwnCompletedArea)
{
  areas[0] = {"front_lawn", /*is_navigation_area=*/false};
  areas[1] = {"back_lawn", /*is_navigation_area=*/false};
  waitForService();

  ctx->target_area_index = 1;
  {
    auto tree = makeTree(/*max_areas=*/5);
    ASSERT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);
  }
  ctx->completed_areas.insert(1u);

  // Two further re-entries: both must end the run, never re-select area 1.
  for (int i = 0; i < 2; ++i)
  {
    auto tree = makeTree(/*max_areas=*/5);
    EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::FAILURE) << "re-entry " << i;
    EXPECT_GT(ctx->completed_areas.count(1u), 0u)
        << "the targeted area's completion must survive re-entry " << i;
  }
}

// A plain COMMAND_START after a targeted run iterates all areas again. The
// clear is production code (clearSingleAreaMode), called by the
// ~/high_level_control handler on COMMAND_START.
TEST_F(GetNextUnmowedAreaTest, PlainStartAfterATargetedRunIteratesAllAreas)
{
  areas[0] = {"front_lawn", /*is_navigation_area=*/false};
  areas[1] = {"back_lawn", /*is_navigation_area=*/false};
  waitForService();

  ctx->target_area_index = 1;
  {
    auto tree = makeTree(/*max_areas=*/5);
    ASSERT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);
    ASSERT_EQ(ctx->current_area, 1);
  }
  ctx->completed_areas.insert(1u);

  // Operator presses the ordinary "Start" button — same session (no EndSession,
  // as after a low-battery dock + resume).
  clearSingleAreaMode(*ctx);

  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->current_area, 0) << "a plain start must resume normal all-areas iteration";
}

TEST_F(GetNextUnmowedAreaTest, EndSessionStillClearsCommandWhenPhasePersistenceFails)
{
  const auto path = std::string(::testing::TempDir()) + "/end_session_phase_failure.txt";
  ctx->coverage_resume_path = path;
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->cross_hatch[0].begin(true);
  ctx->cross_hatch[0].used = true;
  ASSERT_TRUE(mowgli_behavior::saveCoverageResumeState(*ctx));
  ASSERT_TRUE(std::filesystem::create_directory(path + ".tmp"));

  factory.registerNodeType<mowgli_behavior::ClearCommand>("ClearCommand");
  auto end_tree = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"End\">"
      "<Sequence><EndSession/><ClearCommand/></Sequence>"
      "</BehaviorTree></root>",
      blackboard);
  EXPECT_EQ(end_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_TRUE(ctx->area_resume_pose_index.empty());
  EXPECT_TRUE(ctx->cross_hatch[0].next_perpendicular);
  EXPECT_TRUE(std::filesystem::remove(path + ".tmp"));

  BTContext restarted;
  restarted.coverage_resume_path = path;
  EXPECT_FALSE(mowgli_behavior::loadCoverageResumeState(restarted));
  EXPECT_EQ(restarted.current_command, 0);
  EXPECT_TRUE(restarted.area_resume_pose_index.empty());
  std::filesystem::remove(path);
}

// EndSession is the session boundary: the single-area clip dies there with the
// other per-session sets, so the next session starts unconstrained.
TEST_F(GetNextUnmowedAreaTest, EndSessionClearsSingleAreaMode)
{
  areas[0] = {"front_lawn", /*is_navigation_area=*/false};
  areas[1] = {"back_lawn", /*is_navigation_area=*/false};
  waitForService();

  ctx->target_area_index = 1;
  {
    auto tree = makeTree(/*max_areas=*/5);
    ASSERT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);
    ASSERT_TRUE(ctx->single_area_target.has_value());
  }

  auto end_tree = makeEndSessionTree();
  ASSERT_EQ(end_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->single_area_target.has_value());
  EXPECT_FALSE(ctx->target_area_index.has_value());

  // Next session: normal iteration from area 0.
  auto tree = makeTree(/*max_areas=*/5);
  EXPECT_EQ(tickToCompletion(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->current_area, 0);
}

TEST_F(GetNextUnmowedAreaTest, CrossHatchPhaseReachesPlannerAndEndSessionAdvancesIt)
{
  using Plan = mowgli_behavior::PlanCoverageArea::PlanCoverage;
  std::vector<Plan::Goal> goals;
  auto action = rclcpp_action::create_server<Plan>(
      server_node,
      "/plan_coverage",
      [&goals](const auto&, const std::shared_ptr<const Plan::Goal> goal)
      {
        goals.push_back(*goal);
        return rclcpp_action::GoalResponse::REJECT;
      },
      [](const auto&)
      {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](const auto&) {});
  executor.add_node(ctx->node);
  factory.registerNodeType<mowgli_behavior::PlanCoverageArea>("PlanCoverageArea");
  areas[0] = {"lawn", false};
  waitForService();
  ctx->mow_cross_hatch = true;
  auto plan = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><PlanCoverageArea/>"
      "</BehaviorTree></root>",
      blackboard);
  for (double angle : {-1.0, 25.0})
  {
    blackboard->set("mow_angle_deg", angle);
    EXPECT_EQ(tickToCompletion(plan), BT::NodeStatus::FAILURE);  // fake server rejects
    ASSERT_FALSE(goals.empty());
    EXPECT_DOUBLE_EQ(goals.back().mow_angle_deg, angle);
    EXPECT_FALSE(goals.back().perpendicular);
    ctx->cross_hatch[0].used = true;  // simulate coverage having started
    auto end = makeEndSessionTree();
    EXPECT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);
    EXPECT_EQ(tickToCompletion(plan), BT::NodeStatus::FAILURE);
    EXPECT_TRUE(goals.back().perpendicular);
    EXPECT_EQ(tickToCompletion(plan), BT::NodeStatus::FAILURE);  // same-session replan
    EXPECT_TRUE(goals.back().perpendicular);
    ctx->cross_hatch[0].used = true;
    EXPECT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);
  }
}

TEST_F(GetNextUnmowedAreaTest, SelectedAreaOnlyAdvancesThatArea)
{
  ctx->mow_cross_hatch = true;
  ctx->single_area_target = 2u;
  ctx->cross_hatch[0].begin(true);  // planned but not mowed
  ctx->cross_hatch[2].begin(true);
  ctx->cross_hatch[2].used = true;
  auto end = makeEndSessionTree();
  ASSERT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->single_area_target.has_value());
  EXPECT_FALSE(ctx->cross_hatch[0].begin(true));
  EXPECT_TRUE(ctx->cross_hatch[2].begin(true));
  // A subsequent all-areas run advances each used area independently.
  ctx->cross_hatch[0].used = true;
  ctx->cross_hatch[2].used = true;
  ASSERT_EQ(end.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->cross_hatch[0].next());
  EXPECT_FALSE(ctx->cross_hatch[2].next());
}

TEST_F(GetNextUnmowedAreaTest, OrientationServiceEditsNextWithoutChangingActivePlan)
{
  using Service = mowgli_interfaces::srv::CoverageOrientation;
  ctx->coverage_resume_path = ::testing::TempDir() + "/cross_hatch_service.txt";
  ctx->mow_cross_hatch = true;
  ctx->node->declare_parameter<double>("mow_angle_deg", 25.0);
  ctx->cross_hatch[2].begin(true);
  ctx->cross_hatch[2].used = true;
  mowgli_behavior::CoverageOrientationService service(*ctx->node, ctx);
  executor.add_node(ctx->node);
  auto client =
      server_node->create_client<Service>("/test_get_next_unmowed_area/coverage_orientation");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));
  auto req = std::make_shared<Service::Request>();
  req->area_index = 2;
  auto call = [&]()
  {
    auto future = client->async_send_request(req);
    EXPECT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(5)),
              rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  };
  auto status = call();
  ASSERT_TRUE(status->success);
  EXPECT_TRUE(status->next_perpendicular);
  EXPECT_DOUBLE_EQ(status->base_angle_deg, 25.0);
  req->set_next = true;
  req->perpendicular = false;
  status = call();
  ASSERT_TRUE(status->success);
  EXPECT_TRUE(status->current_active);
  EXPECT_FALSE(status->current_perpendicular);
  EXPECT_FALSE(status->next_perpendicular);
  auto end = makeEndSessionTree();
  end.tickOnce();
  EXPECT_FALSE(ctx->cross_hatch[2].begin(true));
  // Persistence failure must not pretend the requested change was saved.
  ctx->coverage_resume_path.clear();
  req->perpendicular = true;
  EXPECT_FALSE(call()->success);
  EXPECT_FALSE(ctx->cross_hatch[2].next());
}
