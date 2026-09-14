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
 * @file test_manual_resume.cpp
 * @brief Operator-forced resume out of a mid-session charge hold.
 *
 * When BatteryGuard / CriticalBatteryDock park the robot on the charger, the
 * tree waits in RetryUntilSuccessful { IsChargingProgressing; Fallback {
 * IsBatteryAbove battery_full_pct; ... } } until the pack reaches 95 %. A
 * COMMAND_START during that hold used to be a no-op (current_command is
 * already 1). It now sets BTContext::manual_resume_requested, and the new
 * IsManualResumeRequested node inside both wait loops consumes it — honouring
 * it only at or above battery_manual_resume_pct.
 *
 * Covers: the node's consume / refuse / stale semantics, the handler's
 * charge-hold decision helper (isChargeHoldState — the handler lambda itself
 * is not unit-testable in isolation), the wait loop's exit order with the
 * real IsBatteryAbove, and structurally that BOTH loops in main_tree.xml carry
 * the node, poll at <= 5 s so a Play press is noticed promptly, and still
 * bound at 8 h.
 */

#include <chrono>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsBatteryAbove;
using mowgli_behavior::isChargeHoldState;
using mowgli_behavior::IsManualResumeRequested;

namespace
{

constexpr float kFullPct = 95.0f;
constexpr float kManualResumePct = 30.0f;
constexpr double kChargeHoldBoundSec = 8.0 * 3600.0;
constexpr double kMaxPollSec = 5.0;

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
// Fixture: a context + the real condition nodes.
// ---------------------------------------------------------------------------

class ManualResumeTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_manual_resume");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    blackboard->set("battery_full_pct", kFullPct);
    blackboard->set("battery_manual_resume_pct", kManualResumePct);

    factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
    factory.registerNodeType<IsManualResumeRequested>("IsManualResumeRequested");
  }

  /// Mirror of the handler's flagging: what ~/high_level_control does on a
  /// COMMAND_START while the published state is a charge hold.
  void requestManualResume(std::chrono::seconds age = std::chrono::seconds{0})
  {
    ctx->manual_resume_requested = true;
    ctx->manual_resume_requested_time = std::chrono::steady_clock::now() - age;
  }

  BT::Tree makeNodeOnlyTree()
  {
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <IsManualResumeRequested min_battery_pct="{battery_manual_resume_pct}"/>
        </BehaviorTree>
      </root>
    )";
    return factory.createTreeFromText(xml, blackboard);
  }

  /// The inner Fallback of both charge wait loops, with the timed wait
  /// collapsed to a bare AlwaysFailure (it only matters that it is LAST).
  BT::Tree makeWaitLoopExitTree()
  {
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <Fallback>
            <IsBatteryAbove threshold="{battery_full_pct}"/>
            <IsManualResumeRequested min_battery_pct="{battery_manual_resume_pct}"/>
            <AlwaysFailure/>
          </Fallback>
        </BehaviorTree>
      </root>
    )";
    return factory.createTreeFromText(xml, blackboard);
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Handler decision helper
// ---------------------------------------------------------------------------

TEST(ManualResumeHelperTest, ChargeHoldStatesAreExactlyTheTwoBatteryHolds)
{
  EXPECT_TRUE(isChargeHoldState("CHARGING"));
  EXPECT_TRUE(isChargeHoldState("CRITICAL_BATTERY_CHARGING"));

  // A START from these is a fresh session / a different branch, not a resume
  // request — the handler must leave the token alone.
  EXPECT_FALSE(isChargeHoldState("IDLE_DOCKED"));
  EXPECT_FALSE(isChargeHoldState("IDLE"));
  EXPECT_FALSE(isChargeHoldState("MOWING"));
  EXPECT_FALSE(isChargeHoldState("CHARGER_FAILED"));
  EXPECT_FALSE(isChargeHoldState("LOW_BATTERY_DOCKING"));
  EXPECT_FALSE(isChargeHoldState(""));
}

// ---------------------------------------------------------------------------
// IsManualResumeRequested
// ---------------------------------------------------------------------------

TEST_F(ManualResumeTest, NotRequestedIsFailureAndTouchesNothing)
{
  ctx->battery_percent = 80.0f;

  auto tree = makeNodeOnlyTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(ctx->manual_resume_requested);
}

TEST_F(ManualResumeTest, RequestedAboveFloorSucceedsAndConsumesTheToken)
{
  ctx->battery_percent = 50.0f;
  requestManualResume();

  auto tree = makeNodeOnlyTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  // One-shot: a second tick without a new request must not resume again.
  EXPECT_FALSE(ctx->manual_resume_requested);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ManualResumeTest, RequestedExactlyAtFloorIsHonoured)
{
  ctx->battery_percent = kManualResumePct;
  requestManualResume();

  auto tree = makeNodeOnlyTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ManualResumeTest, RequestedBelowFloorIsRefusedAndCleared)
{
  ctx->battery_percent = 20.0f;
  requestManualResume();

  auto tree = makeNodeOnlyTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  // Refused requests are dropped, not queued: the operator has to press Play
  // again once the pack is above the floor, and the WARN fires once.
  EXPECT_FALSE(ctx->manual_resume_requested);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ManualResumeTest, StaleRequestIsDroppedAndCleared)
{
  ctx->battery_percent = 80.0f;
  requestManualResume(
      std::chrono::seconds{static_cast<int>(BTContext::kManualResumeMaxAgeSec) + 1});

  auto tree = makeNodeOnlyTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(ctx->manual_resume_requested);
}

TEST_F(ManualResumeTest, FloorPortDefaultsWhenBlackboardKeyIsAbsent)
{
  // No {battery_manual_resume_pct} on this blackboard: the compiled default
  // (30 %) applies, matching the node's declare_parameter default.
  auto bare = BT::Blackboard::create();
  bare->set("context", ctx);
  ctx->battery_percent = 29.0f;
  requestManualResume();

  static const char* xml = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <IsManualResumeRequested/>
      </BehaviorTree>
    </root>
  )";
  auto tree = factory.createTreeFromText(xml, bare);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// ---------------------------------------------------------------------------
// Wait-loop exit order (real IsBatteryAbove + IsManualResumeRequested)
// ---------------------------------------------------------------------------

TEST_F(ManualResumeTest, WaitLoopStillExitsOnFullBatteryWithoutARequest)
{
  ctx->battery_percent = 96.0f;

  auto tree = makeWaitLoopExitTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(ManualResumeTest, WaitLoopKeepsChargingBelowFullWithoutARequest)
{
  ctx->battery_percent = 50.0f;

  auto tree = makeWaitLoopExitTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(ManualResumeTest, WaitLoopExitsEarlyOnManualResumeAboveFloor)
{
  ctx->battery_percent = 50.0f;
  requestManualResume();

  auto tree = makeWaitLoopExitTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->manual_resume_requested);
}

TEST_F(ManualResumeTest, WaitLoopKeepsChargingOnManualResumeBelowFloor)
{
  ctx->battery_percent = 15.0f;
  requestManualResume();

  auto tree = makeWaitLoopExitTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(ctx->manual_resume_requested);
}

TEST_F(ManualResumeTest, FullBatteryWinsWithoutConsumingARequest)
{
  // IsBatteryAbove sits FIRST in the Fallback, so a full pack exits the loop
  // before the token is looked at; the token then expires on its own.
  ctx->battery_percent = 96.0f;
  requestManualResume();

  auto tree = makeWaitLoopExitTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->manual_resume_requested);
}

// ---------------------------------------------------------------------------
// Structural: the real main_tree.xml wait loops
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

/// Text of a <Fallback name="..."> element, closed at its own indentation so
/// the nested Fallback does not end the span early (same approach as
/// test_guard_fallthrough.cpp's ExtractGuardBlock).
std::string ExtractNamedFallback(const std::string& xml, const std::string& name)
{
  std::istringstream in(xml);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);)
  {
    lines.push_back(line);
  }

  const std::string open = "<Fallback name=\"" + name + "\"";
  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    if (lines[i].find(open) == std::string::npos)
    {
      continue;
    }
    const std::size_t indent = lines[i].find_first_not_of(" \t");
    std::string block;
    for (std::size_t j = i; j < lines.size(); ++j)
    {
      block += lines[j] + "\n";
      const std::size_t j_indent = lines[j].find_first_not_of(" \t");
      if (j > i && j_indent == indent && lines[j].substr(j_indent) == "</Fallback>")
      {
        return block;
      }
    }
  }
  return {};
}

const char* const kChargeLoops[] = {"ChargeOrAbort", "CriticalChargeOrAbort"};

}  // namespace

TEST(ManualResumeTreeTest, BothChargeWaitLoopsCarryTheManualResumeExit)
{
  const std::string xml = ReadMainTree();
  for (const char* loop : kChargeLoops)
  {
    const std::string block = ExtractNamedFallback(xml, loop);
    ASSERT_FALSE(block.empty()) << loop << " not found in main_tree.xml.";

    const std::size_t above_at = block.find("<IsBatteryAbove threshold=\"{battery_full_pct}\"/>");
    const std::size_t manual_at =
        block.find("<IsManualResumeRequested min_battery_pct=\"{battery_manual_resume_pct}\"/>");
    const std::size_t wait_at = block.find("<WaitForDuration");

    ASSERT_NE(above_at, std::string::npos) << loop << ": IsBatteryAbove exit missing.";
    ASSERT_NE(manual_at, std::string::npos)
        << loop
        << ": IsManualResumeRequested exit missing — a Play press during this charge "
           "hold is a no-op again.";
    ASSERT_NE(wait_at, std::string::npos) << loop << ": timed wait missing.";

    // Exit order: full battery first (never consumes the token), then the
    // operator override, then the timed wait that fails the attempt.
    EXPECT_LT(above_at, manual_at) << loop << ": IsBatteryAbove must precede the manual exit.";
    EXPECT_LT(manual_at, wait_at) << loop << ": the manual exit must precede the timed wait.";
  }
}

TEST(ManualResumeTreeTest, BothChargeWaitLoopsPollFastEnoughAndStillBoundAtEightHours)
{
  const std::string xml = ReadMainTree();
  for (const char* loop : kChargeLoops)
  {
    const std::string block = ExtractNamedFallback(xml, loop);
    ASSERT_FALSE(block.empty()) << loop << " not found in main_tree.xml.";

    std::smatch attempts_m;
    std::smatch duration_m;
    ASSERT_TRUE(std::regex_search(block, attempts_m, std::regex(R"rx(num_attempts="([0-9]+)")rx")))
        << loop;
    ASSERT_TRUE(std::regex_search(block, duration_m, std::regex(R"rx(duration_sec="([0-9.]+)")rx")))
        << loop;

    const double poll_sec = std::stod(duration_m[1].str());
    const double total_sec = std::stod(attempts_m[1].str()) * poll_sec;

    // The poll period is the latency of a Play press: 30 s was the old value
    // and is what made the operator wait.
    EXPECT_LE(poll_sec, kMaxPollSec) << loop << ": wait loop polls every " << poll_sec
                                     << " s; a manual resume request is only noticed on a tick.";
    // The bound is a product of the two, so shortening the poll must have
    // raised num_attempts to keep the charge hold at 8 h.
    EXPECT_DOUBLE_EQ(total_sec, kChargeHoldBoundSec)
        << loop << ": charge hold no longer bounds at 8 h (got " << total_sec / 3600.0 << " h).";
  }
}

TEST(ManualResumeTreeTest, ManualChargeGuardIsNotAManualResumeLoop)
{
  // The third CHARGING loop (operator docked the mower by hand) exits when the
  // charger bit drops, not on this token — it has no undock step to resume
  // with. Pin that the node did not leak into it.
  const std::string block = ExtractNamedFallback(ReadMainTree(), "ManualChargeGuard");
  ASSERT_FALSE(block.empty());
  EXPECT_EQ(block.find("IsManualResumeRequested"), std::string::npos);
}
