// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0

// Execute RainGuard extracted from the production tree. External motion and
// waits and dock results are deterministic stand-ins; rain, charging, budget
// and session conditions/actions are production nodes. No wall-clock sleeps or
// source-string assertions are used to decide whether departure is allowed.

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include <gtest/gtest.h>
#include <tinyxml2.h>

namespace
{

class RosEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const ros_environment =
    ::testing::AddGlobalTestEnvironment(new RosEnvironment());

struct ExternalActions
{
  bool dock_succeeds{false};
  bool backup_succeeds{true};
  bool backup_running{false};
  int dock_calls{0};
  int backup_calls{0};
  int stop_calls{0};
  int coverage_calls{0};
  bool mower_enabled{true};
};

class DockStub : public BT::StatefulActionNode
{
public:
  DockStub(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return mowgli_behavior::DockRobot::providedPorts();
  }
  BT::NodeStatus onStart() override
  {
    auto ctx = config().blackboard->get<std::shared_ptr<mowgli_behavior::BTContext>>("context");
    auto actions = config().blackboard->get<std::shared_ptr<ExternalActions>>("actions");
    ++actions->dock_calls;
    // This is DockRobot's production onStart/onRunning outcome contract:
    // a new attempt invalidates a previous successful dock immediately.
    ctx->last_dock_succeeded = false;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    auto ctx = config().blackboard->get<std::shared_ptr<mowgli_behavior::BTContext>>("context");
    auto actions = config().blackboard->get<std::shared_ptr<ExternalActions>>("actions");
    ctx->last_dock_succeeded = actions->dock_succeeds;
    return actions->dock_succeeds ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override
  {
  }
};

class BackUpStub : public BT::StatefulActionNode
{
public:
  BackUpStub(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("backup_dist"), BT::InputPort<double>("backup_speed")};
  }
  BT::NodeStatus onStart() override
  {
    ++config().blackboard->get<std::shared_ptr<ExternalActions>>("actions")->backup_calls;
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    auto actions = config().blackboard->get<std::shared_ptr<ExternalActions>>("actions");
    if (actions->backup_running)
    {
      return BT::NodeStatus::RUNNING;
    }
    return actions->backup_succeeds ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  void onHalted() override
  {
  }
};

class WaitStub : public BT::StatefulActionNode
{
public:
  WaitStub(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration_sec")};
  }
  BT::NodeStatus onStart() override
  {
    double duration = 0.0;
    getInput("duration_sec", duration);
    // The one-second departure preparation needs no physical clock. Keep
    // the thirty-second rain polling wait RUNNING until the test releases it.
    return duration < 30.0 && !config().blackboard->get<bool>("hold_departure_wait")
               ? BT::NodeStatus::SUCCESS
               : BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    double duration = 0.0;
    getInput("duration_sec", duration);
    const char* release = duration < 30.0 ? "release_departure_wait" : "release_rain_wait";
    return config().blackboard->get<bool>(release) ? BT::NodeStatus::SUCCESS
                                                   : BT::NodeStatus::RUNNING;
  }
  void onHalted() override
  {
  }
};

const tinyxml2::XMLElement* FindNamed(const tinyxml2::XMLElement* element, const std::string& name)
{
  if (!element)
  {
    return nullptr;
  }
  if (const char* attribute = element->Attribute("name"); attribute && name == attribute)
  {
    return element;
  }
  for (auto child = element->FirstChildElement(); child; child = child->NextSiblingElement())
  {
    if (const auto* found = FindNamed(child, name))
    {
      return found;
    }
  }
  return nullptr;
}

class RainResumeTest : public ::testing::Test
{
protected:
  std::shared_ptr<mowgli_behavior::BTContext> ctx;
  std::shared_ptr<ExternalActions> actions;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  void SetUp() override
  {
    ctx = std::make_shared<mowgli_behavior::BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_rain_resume");
    ctx->current_command = 1;
    ctx->area_resume_pose_index[0] = 42;
    ctx->latest_status.rain_detected = true;
    actions = std::make_shared<ExternalActions>();
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    blackboard->set("actions", actions);
    blackboard->set("rain_mode", 2);
    blackboard->set("rain_debounce_sec", 0.0);
    blackboard->set("undock_distance", 1.0);
    blackboard->set("undock_speed", 0.15);
    blackboard->set("release_rain_wait", false);
    blackboard->set("hold_departure_wait", false);
    blackboard->set("release_departure_wait", false);

    factory.registerNodeType<mowgli_behavior::IsNewRain>("IsNewRain");
    factory.registerNodeType<mowgli_behavior::IsCommand>("IsCommand");
    factory.registerNodeType<mowgli_behavior::IsRainDetected>("IsRainDetected");
    factory.registerNodeType<mowgli_behavior::IsRainModeAtLeast>("IsRainModeAtLeast");
    factory.registerNodeType<mowgli_behavior::IsLastDockSucceeded>("IsLastDockSucceeded");
    factory.registerNodeType<mowgli_behavior::IsCharging>("IsCharging");
    factory.registerNodeType<mowgli_behavior::IsResumeUndockAllowed>("IsResumeUndockAllowed");
    factory.registerNodeType<mowgli_behavior::RecordResumeUndockFailure>(
        "RecordResumeUndockFailure");
    factory.registerNodeType<mowgli_behavior::ClearCommand>("ClearCommand");
    factory.registerNodeType<mowgli_behavior::MarkGuardHalt>("MarkGuardHalt");
    factory.registerNodeType<mowgli_behavior::PublishHighLevelStatus>("PublishHighLevelStatus");
    factory.registerNodeType<DockStub>("DockRobot");
    factory.registerNodeType<WaitStub>("WaitForDuration");
    factory.registerSimpleAction("SetMowerEnabled",
                                 [this](BT::TreeNode& node)
                                 {
                                   node.getInput("enabled", actions->mower_enabled);
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 {BT::InputPort<bool>("enabled")});
    factory.registerSimpleAction("StopMoving",
                                 [this](BT::TreeNode&)
                                 {
                                   ++actions->stop_calls;
                                   return BT::NodeStatus::SUCCESS;
                                 });
    factory.registerSimpleAction("ClearCostmap",
                                 [](BT::TreeNode&)
                                 {
                                   return BT::NodeStatus::SUCCESS;
                                 });
    factory.registerSimpleAction("CoverageMotion",
                                 [this](BT::TreeNode&)
                                 {
                                   ++actions->coverage_calls;
                                   return BT::NodeStatus::SUCCESS;
                                 });
    factory.registerNodeType<BackUpStub>("BackUp");
  }

  BT::Tree MakeTree(bool include_command_guard = false)
  {
    tinyxml2::XMLDocument source;
    if (source.LoadFile(MOWGLI_MAIN_TREE_PATH) != tinyxml2::XML_SUCCESS)
    {
      throw std::runtime_error("Could not load production main_tree.xml");
    }
    const auto* guard = FindNamed(source.RootElement(), "RainGuard");
    if (!guard)
    {
      throw std::runtime_error("Production tree has no RainGuard");
    }
    tinyxml2::XMLDocument extracted;
    auto* root = extracted.NewElement("root");
    root->SetAttribute("BTCPP_format", "4");
    extracted.InsertEndChild(root);
    auto* tree = extracted.NewElement("BehaviorTree");
    tree->SetAttribute("ID", "RainGuardTest");
    root->InsertEndChild(tree);
    if (include_command_guard)
    {
      // Preserve the real reactive command guard and its first child. The
      // downstream coverage/recovery actions become one observable marker:
      // a cleared START must stop traversal before any of those can execute.
      const auto* command_guard = FindNamed(source.RootElement(), "MowingCommandGuard");
      if (!command_guard || !command_guard->FirstChildElement())
      {
        throw std::runtime_error("Production tree has no mowing command guard");
      }
      auto* shell = extracted.NewElement(command_guard->Name());
      shell->SetAttribute("name", "MowingCommandGuard");
      shell->InsertEndChild(command_guard->FirstChildElement()->DeepClone(&extracted));
      shell->InsertEndChild(guard->DeepClone(&extracted));
      shell->InsertEndChild(extracted.NewElement("CoverageMotion"));
      tree->InsertEndChild(shell);
    }
    else
    {
      tree->InsertEndChild(guard->DeepClone(&extracted));
    }
    tinyxml2::XMLPrinter printer;
    extracted.Print(&printer);
    return factory.createTreeFromText(printer.CStr(), blackboard);
  }

  void EnterRainWait(BT::Tree& tree, bool expect_dock = true)
  {
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    if (expect_dock)
    {
      ASSERT_EQ(actions->dock_calls, 1);
      ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    }
    EXPECT_EQ(ctx->last_high_level_status.state_name, "RAIN_WAITING");
    EXPECT_EQ(actions->backup_calls, 0);
    EXPECT_FALSE(actions->mower_enabled);
    EXPECT_EQ(ctx->guard_halted_reason, "rain");
  }

  BT::NodeStatus ClearRain(BT::Tree& tree)
  {
    ctx->latest_status.rain_detected = false;
    blackboard->set("release_rain_wait", true);
    // Completing the polling wait produces its intentional FAILURE. The
    // RetryUntilSuccessful then observes the dry sample on the next tick.
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    for (int tick = 0; tick < 3 && status == BT::NodeStatus::RUNNING; ++tick)
    {
      status = tree.tickOnce();
    }
    return status;
  }
};

TEST_F(RainResumeTest, FailedDockDoesNotBackUpOnLawn)
{
  for (int mode : {2, 3})
  {
    SCOPED_TRACE(mode);
    blackboard->set("rain_mode", mode);
    actions->dock_calls = 0;
    actions->backup_calls = 0;
    ctx->latest_status.rain_detected = true;
    ctx->current_command = 1;
    blackboard->set("release_rain_wait", false);
    auto tree = MakeTree();
    EnterRainWait(tree);
    EXPECT_FALSE(ctx->last_dock_succeeded);
    EXPECT_EQ(ClearRain(tree), BT::NodeStatus::RUNNING);
    EXPECT_EQ(actions->backup_calls, 0);
    EXPECT_EQ(ctx->resume_undock_failures, 0);
    EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
    EXPECT_EQ(ctx->current_command, 0);
    EXPECT_EQ(ctx->last_high_level_status.state_name, "RAIN_DOCK_FAILED");
  }
}

TEST_F(RainResumeTest, FailedCurrentAttemptInvalidatesPreviousDockSuccess)
{
  ctx->last_dock_succeeded = true;
  auto tree = MakeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_FALSE(ctx->last_dock_succeeded);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::RUNNING);
  EXPECT_EQ(actions->backup_calls, 0);
}

TEST_F(RainResumeTest, PauseInPlaceSkipsDockAndDeparture)
{
  blackboard->set("rain_mode", 1);
  ctx->last_dock_succeeded = true;
  auto tree = MakeTree();
  EnterRainWait(tree, false);
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(actions->dock_calls, 0);
  EXPECT_EQ(actions->backup_calls, 0);
  EXPECT_EQ(ctx->current_command, 1);
  EXPECT_EQ(ctx->last_high_level_status.state_name, "MOWING");
}

TEST_F(RainResumeTest, SuccessfulDockStillChargingRunsDeparture)
{
  actions->dock_succeeds = true;
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(actions->backup_calls, 1);
  EXPECT_EQ(ctx->current_command, 1);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
  EXPECT_EQ(ctx->resume_undock_failures, 0);
}

TEST_F(RainResumeTest, OperatorDockAfterFailedActionRunsDeparture)
{
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->last_dock_succeeded);
  EXPECT_EQ(actions->backup_calls, 1);
}

TEST_F(RainResumeTest, SuccessfulDockThenOperatorRemovalSkipsDeparture)
{
  actions->dock_succeeds = true;
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = false;
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->last_dock_succeeded);
  EXPECT_EQ(actions->backup_calls, 0);
  EXPECT_EQ(ctx->current_command, 0);
}

TEST_F(RainResumeTest, FailedDockHoldsUntilCommandGuardStopsWithoutCoverageMotion)
{
  auto tree = MakeTree(true);
  EnterRainWait(tree);
  ctx->latest_status.rain_detected = false;
  blackboard->set("release_rain_wait", true);
  // First finish the poll, then enter the failed-dock hold. Do not tick the
  // command guard again before asserting that the hold prevented fallthrough.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(ctx->last_high_level_status.state_name, "RAIN_DOCK_FAILED");
  EXPECT_EQ(actions->backup_calls, 0);
  EXPECT_EQ(actions->coverage_calls, 0);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(actions->dock_calls, 1);
  EXPECT_EQ(actions->backup_calls, 0);
  EXPECT_EQ(actions->coverage_calls, 0);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
}

TEST_F(RainResumeTest, ChargerLossDuringDeparturePreparationDoesNotBackUp)
{
  actions->dock_succeeds = true;
  blackboard->set("hold_departure_wait", true);
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = true;
  ASSERT_EQ(ClearRain(tree), BT::NodeStatus::RUNNING);
  ASSERT_EQ(actions->backup_calls, 0);
  ctx->latest_power.charger_enabled = false;
  blackboard->set("release_departure_wait", true);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(actions->backup_calls, 0);
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(ctx->last_high_level_status.state_name, "RAIN_DOCK_FAILED");
  EXPECT_EQ(ctx->resume_undock_failures, 0);
}

TEST_F(RainResumeTest, ChargerArrivalDuringDeparturePreparationAllowsBackUp)
{
  blackboard->set("hold_departure_wait", true);
  auto tree = MakeTree();
  EnterRainWait(tree);
  ASSERT_EQ(ClearRain(tree), BT::NodeStatus::RUNNING);
  ASSERT_EQ(actions->backup_calls, 0);
  ctx->latest_power.charger_enabled = true;
  blackboard->set("release_departure_wait", true);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(actions->backup_calls, 1);
  EXPECT_EQ(ctx->current_command, 1);
}

TEST_F(RainResumeTest, DepartureMayReleaseContactAndFinish)
{
  actions->dock_succeeds = true;
  actions->backup_running = true;
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = true;
  ASSERT_EQ(ClearRain(tree), BT::NodeStatus::RUNNING);
  ASSERT_EQ(actions->backup_calls, 1);
  ctx->latest_power.charger_enabled = false;
  actions->backup_running = false;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(actions->backup_calls, 1);
  EXPECT_EQ(ctx->current_command, 1);
  EXPECT_EQ(ctx->last_high_level_status.state_name, "MOWING");
  EXPECT_EQ(ctx->resume_undock_failures, 0);
}

TEST_F(RainResumeTest, FailedDepartureConsumesBudgetAndClearsCommand)
{
  actions->dock_succeeds = true;
  actions->backup_succeeds = false;
  ctx->resume_undock_failures = 2;
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(actions->backup_calls, 1);
  EXPECT_EQ(ctx->resume_undock_failures, 3);
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
}

TEST_F(RainResumeTest, ExhaustedDepartureBudgetDoesNotBackUp)
{
  actions->dock_succeeds = true;
  ctx->resume_undock_failures = 3;
  auto tree = MakeTree();
  EnterRainWait(tree);
  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(ClearRain(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(actions->backup_calls, 0);
  EXPECT_EQ(ctx->resume_undock_failures, 3);
}

}  // namespace
