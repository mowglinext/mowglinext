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
 * @file test_battery_critical_resume.cpp
 * @brief Regression for "robot stuck after a critical-battery charge".
 *
 * The CriticalBatteryDock branch used to END the session unconditionally
 * (EndSession + ClearCommand) once it left the charge-hold, so after a
 * critical-battery event the robot docked, charged fully, and then sat idle
 * forever — current_command was cleared and the coverage resume cursor was
 * wiped. The fix makes recovery AUTO-CONTINUE: after charging to the resume
 * level it undocks and falls through WITHOUT EndSession/ClearCommand (so
 * MowingSequence resumes from the saved cursor), and it ONLY ends the session
 * on a dead charger (CriticalChargerFailed), aborting the branch with FAILURE
 * so the undock/resume tail is skipped.
 *
 * These tests exercise the exact control flow of the tail of CriticalBatteryDock
 * (from the CriticalChargeOrAbort Fallback onward) using the real EndSession,
 * ClearCommand, IsBatteryAbove, IsChargeCurrentBelow and IsResumeUndockAllowed
 * nodes, with stand-ins for IsChargingProgressing (controllable) and BackUp (a
 * marker that records whether the undock/resume tail ran). The IsBatteryLow
 * entry gate is unchanged by the fix and is omitted here (it cannot coexist
 * with the resume gate in a single tick — entry needs battery < 10 %, resume
 * needs battery >= 95 % AND a tapered charge current, issue #759's follow-up).
 */

#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include "mowgli_behavior/docking_nodes.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::ClearCommand;
using mowgli_behavior::EndSession;
using mowgli_behavior::IsBatteryAbove;
using mowgli_behavior::IsChargeCurrentBelow;
using mowgli_behavior::IsCommand;
using mowgli_behavior::IsCriticalChargeStopHeld;
using mowgli_behavior::IsCriticalDockFailureLatched;
using mowgli_behavior::IsLastDockSucceeded;
using mowgli_behavior::IsResumeUndockAllowed;
using mowgli_behavior::LatchCriticalDockFailure;

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

// BehaviorTree.CPP rejects RUNNING from a SyncActionNode. This stateful
// surrogate stays RUNNING until the ReactiveSequence halts it on STOP.
class WaitForCharge : public BT::StatefulActionNode
{
public:
  WaitForCharge(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
  }
};

// ---------------------------------------------------------------------------
// Fixture — mirrors the critical branch entry, post-dock charge hold, and
// stop-hold fallthrough. DockMarker/ChargingProgress/UndockMarker isolate the
// branch traversal while the command gate and charge-stop node are real.
//
// A returned SUCCESS means the recovery/undock tail ran; FAILURE means the
// charger-failed abort ran (undock tail skipped). ChargingProgress and
// UndockMarker are stand-ins we control / observe; every other node is real.
// ---------------------------------------------------------------------------

class CriticalBatteryResumeTest : public ::testing::Test
{
protected:
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;

  bool charging_ok = true;  // stand-in for IsChargingProgressing
  bool battery_critical = true;
  bool docking_ok = true;  // stand-in for DockRobot's terminal result
  int dock_count = 0;  // stand-in for DockRobot
  int undock_count = 0;  // stand-in for BackUp (undock/resume tail)
  int stop_hold_count = 0;

  void SetUp() override
  {
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_battery_critical_resume");

    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    // Resume level pulled by {battery_full_pct} — the same knob MowingSequence
    // uses; matches the mowgli_robot.yaml default.
    blackboard->set("battery_full_pct", 95.0f);
    // Tail-current gate (issue #759 follow-up) — matches battery_charge_
    // tail_current_a's mowgli_robot.yaml default (mirrors the firmware's own
    // CHARGE_END_LIMIT_CURRENT).
    blackboard->set("battery_charge_tail_current_a", 0.08f);

    factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
    factory.registerNodeType<IsChargeCurrentBelow>("IsChargeCurrentBelow");
    factory.registerNodeType<IsCommand>("IsCommand");
    factory.registerNodeType<IsCriticalChargeStopHeld>("IsCriticalChargeStopHeld");
    factory.registerNodeType<IsCriticalDockFailureLatched>("IsCriticalDockFailureLatched");
    factory.registerNodeType<IsLastDockSucceeded>("IsLastDockSucceeded");
    factory.registerNodeType<LatchCriticalDockFailure>("LatchCriticalDockFailure");
    factory.registerNodeType<IsResumeUndockAllowed>("IsResumeUndockAllowed");
    factory.registerNodeType<EndSession>("EndSession");
    factory.registerNodeType<ClearCommand>("ClearCommand");

    // Default to "charger active, current already tapered" so tests that only
    // care about battery_percent don't also have to think about the new gate.
    ctx->latest_power.charger_enabled = true;
    ctx->latest_power.charge_current = 0.0f;

    factory.registerSimpleCondition("ChargingProgress",
                                    [this](BT::TreeNode&)
                                    {
                                      return charging_ok ? BT::NodeStatus::SUCCESS
                                                         : BT::NodeStatus::FAILURE;
                                    });
    factory.registerSimpleCondition("BatteryLow",
                                    [this](BT::TreeNode&)
                                    {
                                      return battery_critical ? BT::NodeStatus::SUCCESS
                                                              : BT::NodeStatus::FAILURE;
                                    });
    factory.registerSimpleAction("UndockMarker",
                                 [this](BT::TreeNode&)
                                 {
                                   ++undock_count;
                                   return BT::NodeStatus::SUCCESS;
                                 });
    factory.registerSimpleAction("DockMarker",
                                 [this](BT::TreeNode&)
                                 {
                                   ++dock_count;
                                   ctx->last_dock_succeeded = docking_ok;
                                   return docking_ok ? BT::NodeStatus::SUCCESS
                                                     : BT::NodeStatus::FAILURE;
                                 });
    factory.registerSimpleAction("StopHoldMarker",
                                 [this](BT::TreeNode&)
                                 {
                                   if (ctx->current_command != 8)
                                   {
                                     return BT::NodeStatus::FAILURE;
                                   }
                                   ++stop_hold_count;
                                   return BT::NodeStatus::SUCCESS;
                                 });
    factory.registerNodeType<WaitForCharge>("WaitForCharge");
  }

  BT::Tree makeTree()
  {
    // The inner 5 s wait is a controllable RUNNING stand-in: recovery
    // short-circuits it, a dead charger fails before it, and STOP must halt it
    // immediately through the surrounding ReactiveSequence.
    static const char* xml = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <Fallback name="MainLogic">
          <Sequence name="CriticalBatteryDock">
            <Fallback>
              <IsCriticalDockFailureLatched/>
              <BatteryLow/>
            </Fallback>
            <Inverter><IsCriticalChargeStopHeld/></Inverter>
            <Fallback name="CriticalDockRetryGate">
              <Sequence>
                <Inverter><IsCriticalDockFailureLatched/></Inverter>
                <Fallback>
                  <DockMarker/>
                  <Sequence>
                    <LatchCriticalDockFailure/>
                    <AlwaysSuccess/>
                  </Sequence>
                </Fallback>
                <IfThenElse name="DockSucceededOrStayStopped">
                  <IsLastDockSucceeded/>
                  <Sequence>
                    <ReactiveSequence name="CriticalChargeHold">
                      <Inverter name="CriticalChargeHoldNotStopped">
                        <IsCriticalChargeStopHeld latch_current_stop="true"/>
                      </Inverter>
                      <Fallback name="CriticalChargeOrAbort">
                      <RetryUntilSuccessful num_attempts="960">
                        <Sequence>
                          <ChargingProgress/>
                          <Fallback>
                            <Sequence>
                              <IsBatteryAbove threshold="{battery_full_pct}"/>
                              <IsChargeCurrentBelow threshold="{battery_charge_tail_current_a}"/>
                            </Sequence>
                            <WaitForCharge/>
                          </Fallback>
                        </Sequence>
                      </RetryUntilSuccessful>
                      <Sequence name="CriticalChargerFailed">
                        <EndSession/>
                        <ClearCommand/>
                        <AlwaysFailure/>
                      </Sequence>
                      </Fallback>
                    </ReactiveSequence>
                    <IsResumeUndockAllowed max_attempts="3"/>
                    <UndockMarker/>
                  </Sequence>
                  <AlwaysSuccess/>
                </IfThenElse>
              </Sequence>
              <Sequence>
                <IsCriticalDockFailureLatched/>
                <AlwaysSuccess/>
              </Sequence>
            </Fallback>
          </Sequence>
          <StopHoldMarker/>
          </Fallback>
        </BehaviorTree>
      </root>
    )";
    return factory.createTreeFromText(xml, blackboard);
  }
};

// Recovery: charged past battery_full_pct AND tapered (SetUp's default
// charger_enabled=true/charge_current=0.0f) with a healthy charger MUST
// auto-continue — undock and fall through WITHOUT clearing the command or the
// resume cursor, so MowingSequence resumes from where it left off.
//
// A "full voltage, current not yet tapered" case is not modeled here. This
// fixture's inner Fallback ends in WaitForCharge, a controllable RUNNING
// stand-in for the real tree's 5 s wait (see makeTree()), so the gate would
// hold RUNNING rather than fail — but the scenario is already covered
// directly by test_manual_resume.cpp's
// WaitLoopKeepsChargingOnFullVoltageWithUntaperedCurrent (same Fallback,
// ticked in isolation).
TEST_F(CriticalBatteryResumeTest, RecoveryAutoContinuesWithoutEndingSession)
{
  ctx->current_command = 1;  // COMMAND_START in flight
  ctx->area_resume_pose_index[0] = 42;  // saved coverage cursor
  ctx->battery_percent = 100.0f;  // fully charged
  charging_ok = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  // The whole point of the fix: neither EndSession nor ClearCommand ran.
  EXPECT_EQ(ctx->current_command, 1);
  ASSERT_EQ(ctx->area_resume_pose_index.count(0), 1u);
  EXPECT_EQ(ctx->area_resume_pose_index[0], 42u);
  // The undock/resume tail actually executed.
  EXPECT_EQ(undock_count, 1);
}

// An operator cancel during the *completed* critical charge hold must leave
// the loop immediately. It is a normal STOP hold, not a dead-charger session
// end: preserve the cursor and never run the undock/resume tail.
TEST_F(CriticalBatteryResumeTest, StopDuringChargeHoldPreservesSessionAndSkipsUndock)
{
  ctx->current_command = 1;  // COMMAND_START; already waiting on the dock
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_percent = 50.0f;
  charging_ok = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ctx->current_command = 8;  // COMMAND_STOP arrives while WaitForCharge runs
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(ctx->current_command, 8);
  ASSERT_EQ(ctx->area_resume_pose_index.count(0), 1u);
  EXPECT_EQ(ctx->area_resume_pose_index[0], 42u);
  EXPECT_EQ(undock_count, 0);
}

// Once STOP has been observed after DockRobot completes, the critical branch
// must remain out on later root ticks. Otherwise its high priority re-enters
// IsBatteryLow and dispatches a second DockRobot goal before StopHoldSequence.
TEST_F(CriticalBatteryResumeTest, StopAfterCriticalDockDoesNotReissueDockGoal)
{
  ctx->current_command = 1;
  ctx->battery_percent = 5.0f;
  charging_ok = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dock_count, 1);

  ctx->current_command = 8;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->critical_charge_stop_latched);
  EXPECT_EQ(undock_count, 0);
  EXPECT_EQ(stop_hold_count, 1);

  // Simulate process restart after persisting the canceled session. The stop
  // command and latch must both survive so startup cannot reissue DockRobot.
  const auto path = ::testing::TempDir() + "/critical_charge_stop_restart.txt";
  ctx->coverage_resume_path = path;
  ASSERT_TRUE(mowgli_behavior::saveCoverageResumeState(*ctx));
  auto restarted = std::make_shared<BTContext>();
  restarted->coverage_resume_path = path;
  ASSERT_TRUE(mowgli_behavior::loadCoverageResumeState(*restarted));
  ctx = restarted;
  blackboard->set("context", ctx);

  // A fresh root traversal represents the next timer tick. The restored gate
  // must keep the tree in StopHold without issuing DockRobot again.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(stop_hold_count, 2);
  std::filesystem::remove(path);
}

TEST_F(CriticalBatteryResumeTest, StopBeforeCriticalDockDoesNotCancelSafetyDocking)
{
  ctx->current_command = 8;
  ctx->battery_percent = 5.0f;
  charging_ok = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dock_count, 1);
  EXPECT_TRUE(ctx->critical_charge_stop_latched);
  EXPECT_EQ(undock_count, 0);
  EXPECT_EQ(stop_hold_count, 1);
}

TEST_F(CriticalBatteryResumeTest, FailedCriticalDockDoesNotEnterChargeHoldOrLatchStop)
{
  ctx->current_command = 8;
  ctx->battery_percent = 5.0f;
  docking_ok = false;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dock_count, 1);
  EXPECT_FALSE(ctx->last_dock_succeeded);
  EXPECT_TRUE(ctx->critical_dock_failure_latched);
  EXPECT_FALSE(ctx->critical_charge_stop_latched);
  EXPECT_EQ(stop_hold_count, 0);

  // The failure remains stopped without issuing another goal at timer rate.
  battery_critical = false;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dock_count, 1);
  EXPECT_TRUE(ctx->critical_dock_failure_latched);

  // An explicit operator retry clears the failure latch; docking can proceed.
  ctx->critical_dock_failure_latched = false;
  ctx->current_command = 1;
  battery_critical = true;
  docking_ok = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dock_count, 2);
  EXPECT_FALSE(ctx->critical_charge_stop_latched);
}

TEST(CriticalBatteryResumeStructureTest, StopGatesArePostDockReactiveHoldsBeforeBothWaits)
{
  std::ifstream input(MOWGLI_MAIN_TREE_PATH);
  ASSERT_TRUE(input.good());
  std::ostringstream contents;
  contents << input.rdbuf();
  const std::string tree = contents.str();
  const auto critical = tree.find("<Sequence name=\"CriticalBatteryDock\">");
  const auto dock = tree.find("<DockRobot", critical);
  const auto reactive_hold = tree.find("<ReactiveSequence name=\"CriticalChargeHold\">", critical);
  const auto stop_gate = tree.find("CriticalChargeHoldNotStopped", critical);
  const auto wait = tree.find("<Fallback name=\"CriticalChargeOrAbort\">", critical);
  const auto undock = tree.find("<IsResumeUndockAllowed", critical);
  ASSERT_NE(critical, std::string::npos);
  ASSERT_NE(dock, std::string::npos);
  ASSERT_NE(reactive_hold, std::string::npos);
  ASSERT_NE(stop_gate, std::string::npos);
  ASSERT_NE(wait, std::string::npos);
  ASSERT_NE(undock, std::string::npos);
  EXPECT_LT(dock, stop_gate);
  EXPECT_LT(reactive_hold, stop_gate);
  EXPECT_LT(stop_gate, wait);
  EXPECT_LT(wait, undock);

  // #765 renamed the BatteryGuard handler BatteryDockAndResume ->
  // BatteryGuardHandler, matching RainGuardHandler / SensorFaultHandler.
  const auto low_dock = tree.find("<Sequence name=\"BatteryGuardHandler\">");
  const auto low_dock_action = tree.find("<DockRobot", low_dock);
  const auto low_reactive_hold = tree.find("<ReactiveSequence name=\"ChargeHold\">", low_dock);
  const auto low_stop_gate = tree.find("ChargeHoldNotStopped", low_dock);
  const auto low_wait = tree.find("<Fallback name=\"ChargeOrAbort\">", low_dock);
  const auto low_undock = tree.find("<IsResumeUndockAllowed", low_dock);
  ASSERT_NE(low_dock, std::string::npos);
  ASSERT_NE(low_dock_action, std::string::npos);
  ASSERT_NE(low_reactive_hold, std::string::npos);
  ASSERT_NE(low_stop_gate, std::string::npos);
  ASSERT_NE(low_wait, std::string::npos);
  ASSERT_NE(low_undock, std::string::npos);
  EXPECT_LT(low_dock_action, low_reactive_hold);
  EXPECT_LT(low_reactive_hold, low_stop_gate);
  EXPECT_LT(low_stop_gate, low_wait);
  EXPECT_LT(low_wait, low_undock);
}

// Dead charger: no charge progress MUST end the session (EndSession +
// ClearCommand) and abort the branch with FAILURE so the undock tail is
// SKIPPED — never resume mowing on a critical pack.
TEST_F(CriticalBatteryResumeTest, DeadChargerEndsSessionAndSkipsUndock)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_percent = 50.0f;  // charge stalled below resume level
  charging_ok = false;  // charger not progressing

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Session ended: command cleared and resume cursor wiped.
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_TRUE(ctx->area_resume_pose_index.empty());
  // The undock/resume tail must NOT have run.
  EXPECT_EQ(undock_count, 0);
}

// The resume-undock attempt cap still gates the auto-continue: once the session
// has exhausted its resume-undock budget, the tail fails (IsResumeUndockAllowed
// FAILURE) instead of undocking again — but the session/command are preserved
// (EndSession did not run on this healthy-charger path).
TEST_F(CriticalBatteryResumeTest, ResumeUndockCapBlocksUndockButKeepsSession)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_percent = 100.0f;
  ctx->resume_undock_failures = 3;  // budget exhausted (max_attempts=3)
  charging_ok = true;

  auto tree = makeTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Charger was healthy → EndSession never ran → command + cursor survive.
  EXPECT_EQ(ctx->current_command, 1);
  ASSERT_EQ(ctx->area_resume_pose_index.count(0), 1u);
  EXPECT_EQ(undock_count, 0);
}

// Tick the production XML, including its battery entry gates and reactive
// contact handling. Only external actions and elapsed waits are controlled.
class BatteryProbeAction : public BT::StatefulActionNode
{
public:
  BatteryProbeAction(const std::string& name,
                     const BT::NodeConfig& config,
                     std::function<BT::NodeStatus(bool)> step,
                     std::function<void()> halt)
      : BT::StatefulActionNode(name, config), step_(std::move(step)), halt_(std::move(halt))
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id"),
            BT::InputPort<std::string>("dock_type"),
            BT::InputPort<double>("backup_dist"),
            BT::InputPort<double>("backup_speed"),
            BT::InputPort<double>("duration_sec")};
  }
  BT::NodeStatus onStart() override
  {
    return step_(true);
  }
  BT::NodeStatus onRunning() override
  {
    return step_(false);
  }
  void onHalted() override
  {
    halt_();
  }

private:
  std::function<BT::NodeStatus(bool)> step_;
  std::function<void()> halt_;
};

class ProductionBatteryRecoveryTest : public CriticalBatteryResumeTest
{
protected:
  bool dock_pending = false;
  bool wait_complete = false;
  int dock_halts = 0;
  std::string last_state;

  void SetUp() override
  {
    CriticalBatteryResumeTest::SetUp();
    ctx->latest_power.charger_enabled = false;
    blackboard->set("battery_critical_pct", 10.0f);
    blackboard->set("battery_critical_voltage", 0.0f);
    blackboard->set("battery_low_pct", 20.0f);
    blackboard->set("battery_manual_resume_pct", 30.0f);
    blackboard->set("undock_distance", 1.0);
    blackboard->set("undock_speed", 0.15);
    factory.registerNodeType<mowgli_behavior::ConfirmChargingDock>("ConfirmChargingDock");
    factory.registerNodeType<mowgli_behavior::IsBatteryChargePending>("IsBatteryChargePending");
    factory.registerNodeType<mowgli_behavior::SetBatteryChargePending>("SetBatteryChargePending");
    factory.registerNodeType<mowgli_behavior::IsDocking>("IsDocking");
    factory.registerNodeType<mowgli_behavior::IsBatteryLow>("IsBatteryLow");
    factory.registerNodeType<mowgli_behavior::NeedsDocking>("NeedsDocking");
    factory.registerNodeType<mowgli_behavior::IsManualResumeRequested>("IsManualResumeRequested");
    factory.registerSimpleCondition("IsChargingProgressing",
                                    [this](BT::TreeNode&)
                                    {
                                      return charging_ok ? BT::NodeStatus::SUCCESS
                                                         : BT::NodeStatus::FAILURE;
                                    },
                                    {BT::InputPort<float>("full_pct")});
    factory.registerSimpleAction("PublishHighLevelStatus",
                                 [this](BT::TreeNode& node)
                                 {
                                   last_state = node.getInput<std::string>("state_name").value();
                                   return BT::NodeStatus::SUCCESS;
                                 },
                                 {BT::InputPort<int>("state"),
                                  BT::InputPort<std::string>("state_name")});
    for (const auto* name : {"SetMowerEnabled",
                             "StopMoving",
                             "SaveObstacles",
                             "ClearCostmap",
                             "MarkGuardHalt",
                             "RecordResumeUndockFailure"})
    {
      factory.registerSimpleAction(name,
                                   [](BT::TreeNode&)
                                   {
                                     return BT::NodeStatus::SUCCESS;
                                   },
                                   {BT::InputPort<bool>("enabled"),
                                    BT::InputPort<std::string>("reason")});
    }
    factory.registerBuilder<BatteryProbeAction>("DockRobot",
                                                [this](const std::string& name,
                                                       const BT::NodeConfig& config)
                                                {
                                                  return std::make_unique<BatteryProbeAction>(
                                                      name,
                                                      config,
                                                      [this](bool start)
                                                      {
                                                        if (start)
                                                          ++dock_count;
                                                        if (dock_pending)
                                                          return BT::NodeStatus::RUNNING;
                                                        ctx->last_dock_succeeded = docking_ok;
                                                        ctx->latest_power.charger_enabled =
                                                            docking_ok;
                                                        return docking_ok ? BT::NodeStatus::SUCCESS
                                                                          : BT::NodeStatus::FAILURE;
                                                      },
                                                      [this]()
                                                      {
                                                        ++dock_halts;
                                                      });
                                                });
    factory.registerBuilder<BatteryProbeAction>("BackUp",
                                                [this](const std::string& name,
                                                       const BT::NodeConfig& config)
                                                {
                                                  return std::make_unique<BatteryProbeAction>(
                                                      name,
                                                      config,
                                                      [this](bool)
                                                      {
                                                        ++undock_count;
                                                        ctx->latest_power.charger_enabled = false;
                                                        return BT::NodeStatus::SUCCESS;
                                                      },
                                                      []() {});
                                                });
    factory.registerBuilder<BatteryProbeAction>("WaitForDuration",
                                                [this](const std::string& name,
                                                       const BT::NodeConfig& config)
                                                {
                                                  return std::make_unique<BatteryProbeAction>(
                                                      name,
                                                      config,
                                                      [this](bool)
                                                      {
                                                        return wait_complete
                                                                   ? BT::NodeStatus::SUCCESS
                                                                   : BT::NodeStatus::RUNNING;
                                                      },
                                                      []() {});
                                                });
  }

  std::string productionBranchXml(const std::string& name, const std::string& tag)
  {
    std::ifstream input(MOWGLI_MAIN_TREE_PATH);
    std::ostringstream contents;
    contents << input.rdbuf();
    const auto xml = std::regex_replace(contents.str(), std::regex("<!--[\\s\\S]*?-->"), "");
    const auto begin = xml.find("<" + tag + " name=\"" + name + "\">");
    if (begin == std::string::npos)
      throw std::runtime_error("Missing production battery branch: " + name);
    const std::regex tags("</?" + tag + "(?:[ >])");
    const auto tail = xml.substr(begin);
    int depth = 0;
    for (auto it = std::sregex_iterator(tail.begin(), tail.end(), tags);
         it != std::sregex_iterator();
         ++it)
    {
      const auto pos = static_cast<std::size_t>(it->position());
      depth += tail[pos + 1] == '/' ? -1 : 1;
      if (depth == 0)
      {
        const auto end = tail.find('>', pos) + 1;
        return tail.substr(0, end);
      }
    }
    throw std::runtime_error("Unclosed production battery branch: " + name);
  }

  BT::Tree productionBranch(const std::string& name, const std::string& tag = "Sequence")
  {
    return factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" +
                                          productionBranchXml(name, tag) + "</BehaviorTree></root>",
                                      blackboard);
  }

  void restartFromSnapshot()
  {
    const auto path = ::testing::TempDir() + "/battery_recovery_restart.txt";
    ctx->coverage_resume_path = path;
    ASSERT_TRUE(mowgli_behavior::saveCoverageResumeState(*ctx));
    auto restarted = std::make_shared<BTContext>();
    restarted->node = ctx->node;
    restarted->coverage_resume_path = path;
    ASSERT_TRUE(mowgli_behavior::loadCoverageResumeState(*restarted));
    restarted->restored_mow_charge_check_pending =
        restarted->current_command == 1 && mowgli_behavior::hasCoverageResumeState(*restarted);
    ctx = restarted;
    blackboard->set("context", ctx);
    std::filesystem::remove(path);
  }
};

TEST_F(ProductionBatteryRecoveryTest, RestartDuringMowChargesThenResumesSameCursorOnce)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  restartFromSnapshot();
  ctx->battery_percent = 5.0f;
  auto tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(undock_count, 0);
  ctx->battery_percent = 100.0f;
  wait_complete = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(undock_count, 1);
  EXPECT_EQ(ctx->current_command, 1);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
  for (int i = 0; i < 20; ++i)
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(undock_count, 1);
}

TEST_F(ProductionBatteryRecoveryTest, IdleChargingRestartNeverNavigatesOrUndocks)
{
  ctx->current_command = 0;
  restartFromSnapshot();
  ctx->battery_percent = 5.0f;
  ctx->latest_power.charger_enabled = true;
  auto tree = productionBranch("CriticalBatteryDock");
  for (int i = 0; i < 20; ++i)
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(last_state, "CRITICAL_BATTERY_CHARGING");
  ctx->battery_percent = 100.0f;
  wait_complete = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(dock_count, 0);
  EXPECT_EQ(undock_count, 0);
}

TEST_F(ProductionBatteryRecoveryTest, RestoredFailureRecoversOnChargerAboveCriticalWithoutMotion)
{
  ctx->current_command = 0;
  ctx->critical_dock_failure_latched = true;
  ctx->area_resume_pose_index[0] = 42;
  restartFromSnapshot();
  ctx->battery_percent = 75.0f;
  ctx->latest_power.charger_enabled = true;
  auto tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_FALSE(ctx->critical_dock_failure_latched);
  EXPECT_TRUE(ctx->critical_dock_failure_persistence_requested);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
  EXPECT_EQ(dock_count, 0);
  EXPECT_EQ(undock_count, 0);
}

TEST_F(ProductionBatteryRecoveryTest, FailedCriticalDockHoldsAcrossTicksAndRestart)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_percent = 5.0f;
  docking_ok = false;
  auto tree = productionBranch("CriticalBatteryDock");
  for (int i = 0; i < 20; ++i)
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(undock_count, 0);
  EXPECT_EQ(last_state, "CRITICAL_BATTERY_NAV_FAILED");
  restartFromSnapshot();
  ctx->battery_percent = 75.0f;
  auto restarted_tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(restarted_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
}

TEST_F(ProductionBatteryRecoveryTest, CriticalContactHaltsDockBeforeActionResult)
{
  ctx->current_command = 1;
  ctx->battery_percent = 5.0f;
  dock_pending = true;
  auto tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(dock_halts, 1);
  EXPECT_EQ(last_state, "CRITICAL_BATTERY_CHARGING");
  EXPECT_EQ(undock_count, 0);
}

TEST_F(ProductionBatteryRecoveryTest, LowBatteryContactSkipsDockAndResumesActiveMow)
{
  ctx->current_command = 1;
  ctx->battery_percent = 15.0f;
  ctx->latest_power.charger_enabled = true;
  auto tree = productionBranch("BatteryGuard", "Fallback");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(last_state, "CHARGING");
  EXPECT_EQ(dock_count, 0);
  ctx->battery_percent = 100.0f;
  wait_complete = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(undock_count, 1);
}

TEST_F(ProductionBatteryRecoveryTest, RealDockActionAlreadyChargingSucceedsWithoutServer)
{
  ctx->latest_power.charger_enabled = true;
  factory.registerNodeType<mowgli_behavior::DockRobot>("RealDockRobot");
  auto tree = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><RealDockRobot/>"
      "</BehaviorTree></root>",
      blackboard);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(ctx->last_dock_succeeded);
  EXPECT_FALSE(ctx->docking_active);
}

TEST_F(ProductionBatteryRecoveryTest, ChargeHoldSurvivesGuardHaltAndSnapshotRestart)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  ctx->battery_charge_pending = true;
  ctx->battery_percent = 75.0f;
  ctx->latest_power.charger_enabled = true;
  auto tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  tree.haltTree();  // EmergencyGuard stops ticking MainLogic.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(undock_count, 0);
  tree.haltTree();
  restartFromSnapshot();
  ASSERT_TRUE(ctx->battery_charge_pending);
  ctx->battery_percent = 75.0f;
  ctx->latest_power.charger_enabled = true;
  auto restarted_tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(restarted_tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dock_count, 0);
  EXPECT_EQ(undock_count, 0);
  ctx->battery_percent = 100.0f;
  ctx->latest_power.charge_current = 1.2f;
  EXPECT_EQ(restarted_tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(undock_count, 0);  // Voltage alone cannot release the hold.
  ctx->latest_power.charge_current = 0.01f;
  wait_complete = true;
  EXPECT_EQ(restarted_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(ctx->battery_charge_pending);
  EXPECT_EQ(undock_count, 1);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
}

TEST_F(ProductionBatteryRecoveryTest, LegacyMowSnapshotOnChargerWaitsFullButOffDockContinues)
{
  ctx->current_command = 1;
  ctx->area_resume_pose_index[0] = 42;
  restartFromSnapshot();
  // A pre-fix snapshot has the same default false pending state.
  ASSERT_FALSE(ctx->battery_charge_pending);
  ctx->battery_percent = 75.0f;
  ctx->latest_power.charger_enabled = true;
  mowgli_behavior::observeRestoredMowPower(*ctx, true);
  EXPECT_TRUE(ctx->battery_charge_pending);
  auto tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(undock_count, 0);
  EXPECT_EQ(dock_count, 0);

  tree.haltTree();
  ctx->battery_charge_pending = false;
  restartFromSnapshot();
  ctx->battery_percent = 75.0f;
  ctx->latest_power.charger_enabled = false;
  mowgli_behavior::observeRestoredMowPower(*ctx, false);
  EXPECT_FALSE(ctx->battery_charge_pending);
  auto off_dock_tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(off_dock_tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->current_command, 1);
  EXPECT_EQ(ctx->area_resume_pose_index.at(0), 42u);
}

TEST_F(ProductionBatteryRecoveryTest, StopKeepsPendingChargeAndBlocksUndockAcrossRestart)
{
  ctx->current_command = 1;
  ctx->battery_charge_pending = true;
  ctx->battery_percent = 75.0f;
  ctx->latest_power.charger_enabled = true;
  auto tree = productionBranch("CriticalBatteryDock");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ctx->current_command = 8;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(ctx->critical_charge_stop_latched);
  EXPECT_TRUE(ctx->battery_charge_pending);
  restartFromSnapshot();
  ctx->battery_percent = 100.0f;
  ctx->latest_power.charger_enabled = true;
  wait_complete = true;
  auto restarted_tree = productionBranch("CriticalBatteryDock");
  for (int i = 0; i < 20; ++i)
    EXPECT_EQ(restarted_tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_EQ(dock_count, 0);
  EXPECT_EQ(undock_count, 0);
}

TEST_F(ProductionBatteryRecoveryTest, LowBatteryContactHaltsRunningDock)
{
  ctx->current_command = 1;
  ctx->battery_percent = 15.0f;
  dock_pending = true;
  auto tree = productionBranch("BatteryGuard", "Fallback");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_TRUE(ctx->battery_charge_pending);
  EXPECT_TRUE(ctx->critical_dock_failure_persistence_requested);
  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dock_count, 1);
  EXPECT_EQ(dock_halts, 1);
  EXPECT_TRUE(ctx->battery_charge_pending);
  EXPECT_EQ(last_state, "CHARGING");
  EXPECT_EQ(undock_count, 0);
}

TEST_F(ProductionBatteryRecoveryTest, FirstPowerWhileStartupRunsYieldsToChargeRecovery)
{
  ctx->current_command = 1;
  auto xml = productionBranchXml("MowingSequence", "ReactiveSequence");
  const auto action = xml.find("<Sequence name=\"MowingAction\">");
  ASSERT_NE(action, std::string::npos);
  // Keep the production mission command and charge gates, replacing external
  // startup/coverage work with a RUNNING action.
  xml = xml.substr(0, action) + "<WaitForCharge/></ReactiveSequence>";
  auto tree = factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\">" +
                                             xml + "</BehaviorTree></root>",
                                         blackboard);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ctx->restored_mow_charge_check_pending = true;
  mowgli_behavior::observeRestoredMowPower(*ctx, true);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(undock_count, 0);

  // A battery docking action already in flight retains ownership until contact
  // or its result, rather than being canceled by its own pending charge flag.
  ctx->docking_active = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
}
