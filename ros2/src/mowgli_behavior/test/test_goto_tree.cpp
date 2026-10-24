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
 * @file test_goto_tree.cpp
 * @brief COMMAND_GOTO (9): the ~/goto_pose decision and the GOTO branches of main_tree.xml.
 *
 * The tree tests run the REAL GotoSequence and the REAL GOTO exit of
 * CriticalBatteryDock, cut out of trees/main_tree.xml at test time, so they
 * cannot drift from the shipped tree. IsCommand, ClearCommand, NeedsDocking,
 * IsBatteryLow, IsLastDockSucceeded and SetStopReason are the real nodes; motion/blade/
 * status/docking nodes are recording stand-ins (scripted NavigateToPose and
 * DockRobot count starts and halts).
 *
 * What is pinned:
 *   - a GOTO request is accepted only from a settled, undocked rest state, and
 *     a refusal changes nothing; the target never outlives its GOTO;
 *   - one Nav2 attempt per request: success or failure both end the mission
 *     (current_command back to 0) and a failed goal is never re-sent;
 *   - low battery, or the critical percentage/voltage test, during the transit
 *     halts NavigateToPose, docks once, ends the mission, never resumes it and
 *     never loops on a failed dock; a failed dock has its own terminal status;
 *   - the blade is switched off before any motion, on every entry, and is
 *     never enabled on the GOTO path;
 *   - after a critical-battery charge a GOTO ends docked (no BackUp) and
 *     clears its command, while only mowing keeps the AUTO-CONTINUE tail;
 *   - no root guard exempts command 9, except the blade-off dock transit
 *     (IsDocking), exactly like mowing.
 */

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include "mowgli_behavior/fleet_yield_service.hpp"
#include "mowgli_behavior/goto_request.hpp"
#include "mowgli_behavior/status_nodes.hpp"
#include "mowgli_behavior/stop_reason.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::acceptGotoRequest;
using mowgli_behavior::BTContext;
using mowgli_behavior::ClearCommand;
using mowgli_behavior::dropStaleGotoTargetLocked;
using mowgli_behavior::FleetYieldState;
using mowgli_behavior::GotoBatteryLimits;
using mowgli_behavior::GotoRestObserver;
using mowgli_behavior::IsBatteryLow;
using mowgli_behavior::IsCommand;
using mowgli_behavior::IsDocking;
using mowgli_behavior::IsLastDockSucceeded;
using mowgli_behavior::IsResumeUndockAllowed;
using mowgli_behavior::kCommandGoto;
using mowgli_behavior::NeedsDocking;
using mowgli_behavior::recordStopReason;
using mowgli_behavior::SetStopReason;
namespace stop_reason = mowgli_behavior::stop_reason;

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

std::string readMainTree()
{
  std::ifstream in(MOWGLI_MAIN_TREE_PATH);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::string stripComments(const std::string& xml)
{
  return std::regex_replace(xml, std::regex("<!--[\\s\\S]*?-->"), "");
}

/// The element that starts with `opening` (e.g. `<Fallback name="X">`), up to its
/// matching close tag, counting nested elements of the same tag.
std::string extractElement(const std::string& xml, const std::string& tag, const std::string& name)
{
  const std::string opening = "<" + tag + " name=\"" + name + "\">";
  const auto start = xml.find(opening);
  if (start == std::string::npos)
  {
    return "";
  }
  const std::string open_prefix = "<" + tag;
  const std::string close_tag = "</" + tag + ">";
  int depth = 0;
  std::size_t pos = start;
  while (pos < xml.size())
  {
    const auto next_open = xml.find(open_prefix, pos);
    const auto next_close = xml.find(close_tag, pos);
    if (next_close == std::string::npos)
    {
      return "";
    }
    if (next_open != std::string::npos && next_open < next_close)
    {
      const char after = xml[next_open + open_prefix.size()];
      if (after == ' ' || after == '>' || after == '\n')
      {
        ++depth;
      }
      pos = next_open + open_prefix.size();
      continue;
    }
    --depth;
    pos = next_close + close_tag.size();
    if (depth == 0)
    {
      return xml.substr(start, pos - start);
    }
  }
  return "";
}

std::string wrapTree(const std::string& body)
{
  return "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">" + body + "</BehaviorTree></root>";
}

// ---------------------------------------------------------------------------
// Recording stand-ins
// ---------------------------------------------------------------------------

struct Recorder
{
  std::vector<std::string> statuses;
  std::vector<bool> blade_requests;
  /// Ordered "blade_off" / "blade_on" / "stop" / "nav_start" / "dock_start".
  std::vector<std::string> events;
  int nav_starts{0};
  int nav_halts{0};
  BT::NodeStatus nav_result{BT::NodeStatus::RUNNING};
  int dock_starts{0};
  int dock_halts{0};
  /// DockRobot stays RUNNING while this is RUNNING, then returns it.
  BT::NodeStatus dock_result{BT::NodeStatus::SUCCESS};
  int backups{0};

  bool published(const std::string& name) const
  {
    for (const auto& s : statuses)
    {
      if (s == name)
      {
        return true;
      }
    }
    return false;
  }
};

Recorder* g_rec = nullptr;

class StubNavigateToPose : public BT::StatefulActionNode
{
public:
  StubNavigateToPose(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal"), BT::InputPort<double>("timeout_sec")};
  }
  BT::NodeStatus onStart() override
  {
    ++g_rec->nav_starts;
    g_rec->events.push_back("nav_start");
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    return g_rec->nav_result;
  }
  void onHalted() override
  {
    ++g_rec->nav_halts;
  }
};

/// Mirrors the real DockRobot's contract on ctx->last_dock_succeeded (reset on
/// start, set only on success), which the tree reads via IsLastDockSucceeded.
class StubDockRobot : public BT::StatefulActionNode
{
public:
  StubDockRobot(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("dock_id"), BT::InputPort<std::string>("dock_type")};
  }
  BT::NodeStatus onStart() override
  {
    ++g_rec->dock_starts;
    g_rec->events.push_back("dock_start");
    context()->last_dock_succeeded = false;
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    if (g_rec->dock_result == BT::NodeStatus::SUCCESS)
    {
      context()->last_dock_succeeded = true;
    }
    return g_rec->dock_result;
  }
  void onHalted() override
  {
    ++g_rec->dock_halts;
  }

private:
  std::shared_ptr<BTContext> context()
  {
    return config().blackboard->get<std::shared_ptr<BTContext>>("context");
  }
};

class StubSetMowerEnabled : public BT::SyncActionNode
{
public:
  StubSetMowerEnabled(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("enabled")};
  }
  BT::NodeStatus tick() override
  {
    const bool enabled = getInput<bool>("enabled").value();
    g_rec->blade_requests.push_back(enabled);
    g_rec->events.push_back(enabled ? "blade_on" : "blade_off");
    return BT::NodeStatus::SUCCESS;
  }
};

class StubStopMoving : public BT::SyncActionNode
{
public:
  StubStopMoving(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override
  {
    g_rec->events.push_back("stop");
    return BT::NodeStatus::SUCCESS;
  }
};

class StubPublishHighLevelStatus : public BT::SyncActionNode
{
public:
  StubPublishHighLevelStatus(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<int>("state"), BT::InputPort<std::string>("state_name")};
  }
  BT::NodeStatus tick() override
  {
    g_rec->statuses.push_back(getInput<std::string>("state_name").value());
    return BT::NodeStatus::SUCCESS;
  }
};

class StubBackUp : public BT::SyncActionNode
{
public:
  StubBackUp(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("backup_dist"), BT::InputPort<double>("backup_speed")};
  }
  BT::NodeStatus tick() override
  {
    ++g_rec->backups;
    return BT::NodeStatus::SUCCESS;
  }
};

class StubSucceed : public BT::SyncActionNode
{
public:
  StubSucceed(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// ~/goto_pose decision
// ---------------------------------------------------------------------------

namespace
{

/// A robot parked off the dock in IDLE after a session that seeded its
/// heading, with a healthy battery: the one state a GOTO may start from.
void makeAvailable(BTContext& ctx)
{
  ctx.current_command = 0;
  ctx.last_high_level_status.state_name = "IDLE";
  ctx.latest_power.charger_enabled = false;
  ctx.undock_start_recorded = false;
  ctx.yaw_seeded_this_session = true;
  ctx.battery_percent = 80.0f;
  ctx.battery_voltage_filtered = 27.2f;
  ctx.critical_dock_failure_latched = false;
  ctx.critical_charge_stop_latched = false;
  ctx.battery_charge_pending = false;
  ctx.tree_at_rest = true;
  ctx.localization_degraded = false;
  ctx.last_power_time = std::chrono::steady_clock::now();
}

const GotoBatteryLimits kLimits{20.0f, 10.0f, 0.0f};

mowgli_behavior::GotoRequestDecision request(
    BTContext& ctx, double x, double y, double yaw, const GotoBatteryLimits& limits = kLimits)
{
  return acceptGotoRequest(
      ctx, x, y, yaw, limits, builtin_interfaces::msg::Time{}, std::chrono::steady_clock::now());
}

/// A refusal must leave the command and the (absent) target untouched.
void expectRefusedUnchanged(BTContext& ctx,
                            const std::string& reason,
                            const GotoBatteryLimits& limits = kLimits)
{
  const uint8_t before = ctx.current_command;
  const auto decision = request(ctx, 1.0, 2.0, 0.0, limits);
  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, reason);
  EXPECT_EQ(ctx.current_command, before);
  EXPECT_FALSE(ctx.goto_target_pose.has_value());
}

}  // namespace

TEST(GotoRequestTest, AcceptedRequestStoresMapFrameTargetAndRaisesGoto)
{
  BTContext ctx;
  makeAvailable(ctx);

  const auto decision = request(ctx, 3.5, -2.0, M_PI / 2.0);

  EXPECT_TRUE(decision.accepted);
  EXPECT_TRUE(decision.reason.empty());
  EXPECT_EQ(ctx.current_command, kCommandGoto);
  ASSERT_TRUE(ctx.goto_target_pose.has_value());
  EXPECT_EQ(ctx.goto_target_pose->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(ctx.goto_target_pose->pose.position.x, 3.5);
  EXPECT_DOUBLE_EQ(ctx.goto_target_pose->pose.position.y, -2.0);
  // yaw = pi/2 about z
  EXPECT_NEAR(ctx.goto_target_pose->pose.orientation.z, std::sin(M_PI / 4.0), 1e-9);
  EXPECT_NEAR(ctx.goto_target_pose->pose.orientation.w, std::cos(M_PI / 4.0), 1e-9);
}

// Like every accepted command except STOP, a GOTO starts something new: the
// reason the previous run ended is cleared. A refusal keeps it.
TEST(GotoRequestTest, AcceptedRequestClearsStopReasonRefusalKeepsIt)
{
  BTContext ctx;
  makeAvailable(ctx);
  recordStopReason(ctx, stop_reason::kDockNavFailed, "detail", true);
  ctx.current_command = 1;

  EXPECT_FALSE(request(ctx, 1.0, 1.0, 0.0).accepted);
  EXPECT_EQ(ctx.stop_reason, stop_reason::kDockNavFailed);

  ctx.current_command = 0;
  EXPECT_TRUE(request(ctx, 1.0, 1.0, 0.0).accepted);
  EXPECT_TRUE(ctx.stop_reason.empty());
  EXPECT_TRUE(ctx.stop_detail.empty());
}

// No hot pre-emption: a second request while a GOTO is active is refused and
// changes nothing, so the robot never silently keeps driving to the old target
// while the caller believes it is heading for the new one.
TEST(GotoRequestTest, SecondRequestWhileGotoActiveIsBusyAndChangesNothing)
{
  BTContext ctx;
  makeAvailable(ctx);
  ASSERT_TRUE(request(ctx, 1.0, 1.0, 0.0).accepted);

  const auto decision = request(ctx, 9.0, 9.0, 1.0);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "busy");
  EXPECT_EQ(ctx.current_command, kCommandGoto);
  EXPECT_DOUBLE_EQ(ctx.goto_target_pose->pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(ctx.goto_target_pose->pose.position.y, 1.0);
}

// After STOP (the documented way to change target) a new request is accepted
// once StopHoldSequence has published IDLE.
TEST(GotoRequestTest, RequestAfterStopIsAccepted)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.current_command = 8;  // COMMAND_STOP

  const auto decision = request(ctx, 2.0, 0.0, 0.0);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(ctx.current_command, kCommandGoto);
}

// A GOTO accepted while a fleet coordinator holds the robot (PAUSE turned
// mowing into STOP) fences that ownership, as every other operator command
// does: the coordinator's RESUME must not turn the GOTO back into mowing.
TEST(GotoRequestTest, AcceptedRequestRevokesFleetHold)
{
  BTContext ctx;
  ctx.current_command = 1;
  ctx.fleet_yield.observeInterlock(false);
  ctx.fleet_yield.observeStatus(2, "MOWING");
  const std::string epoch = ctx.fleet_yield.epoch();
  ASSERT_EQ(ctx.fleet_yield.apply(FleetYieldState::PAUSE, epoch, "req-1", ctx.current_command),
            FleetYieldState::HELD);
  ASSERT_EQ(ctx.current_command, 8);
  ctx.fleet_yield.observeStatus(1, "IDLE");
  makeAvailable(ctx);
  ctx.current_command = 8;

  ASSERT_TRUE(request(ctx, 2.0, 0.0, 0.0).accepted);

  EXPECT_FALSE(ctx.fleet_yield.held());
  uint8_t command = ctx.current_command;
  EXPECT_NE(ctx.fleet_yield.apply(FleetYieldState::RESUME, epoch, "req-1", command),
            FleetYieldState::RESUMED);
  EXPECT_EQ(command, kCommandGoto);
}

// A refused GOTO is not a command: it leaves the fleet hold in place.
TEST(GotoRequestTest, RefusedRequestKeepsFleetHold)
{
  BTContext ctx;
  ctx.current_command = 1;
  ctx.fleet_yield.observeInterlock(false);
  ctx.fleet_yield.observeStatus(2, "MOWING");
  ASSERT_EQ(ctx.fleet_yield.apply(
                FleetYieldState::PAUSE, ctx.fleet_yield.epoch(), "req-1", ctx.current_command),
            FleetYieldState::HELD);
  makeAvailable(ctx);
  ctx.current_command = 8;
  ctx.yaw_seeded_this_session = false;

  EXPECT_FALSE(request(ctx, 2.0, 0.0, 0.0).accepted);
  EXPECT_TRUE(ctx.fleet_yield.held());
}

// Chaining: a new GOTO is accepted right after the previous one ended, before
// IdleSequence has republished IDLE.
TEST(GotoRequestTest, RequestRightAfterGotoEndedIsAccepted)
{
  for (const char* state : {"GOTO_REACHED", "GOTO_FAILED"})
  {
    BTContext ctx;
    makeAvailable(ctx);
    ctx.last_high_level_status.state_name = state;
    EXPECT_TRUE(request(ctx, 2.0, 0.0, 0.0).accepted) << state;
  }
}

TEST(GotoRequestTest, NonFiniteCoordinatesAreRefused)
{
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  for (const auto& [x, y, yaw] : std::vector<std::tuple<double, double, double>>{
           {nan, 0.0, 0.0}, {0.0, inf, 0.0}, {0.0, 0.0, nan}, {-inf, 0.0, 0.0}})
  {
    BTContext ctx;
    makeAvailable(ctx);
    const auto decision = request(ctx, x, y, yaw);
    EXPECT_FALSE(decision.accepted);
    EXPECT_EQ(decision.reason, "invalid_pose");
    EXPECT_EQ(ctx.current_command, 0);
    EXPECT_FALSE(ctx.goto_target_pose.has_value());
  }
}

// No implicit pre-emption of another mission: mowing, HOME, recording
// (3/5/6) and manual mowing are left running.
TEST(GotoRequestTest, RefusedWhileAnotherMissionIsActive)
{
  for (uint8_t cmd : {1, 2, 3, 5, 6, 7})
  {
    BTContext ctx;
    makeAvailable(ctx);
    ctx.current_command = cmd;
    SCOPED_TRACE("command " + std::to_string(cmd));
    expectRefusedUnchanged(ctx, "busy");
  }
}

// current_command 0/8 is not proof of availability: the tree may still be in
// a critical-battery dock or charge hold (command 0), the end of a GOTO
// low-battery dock, or a root guard handler.
TEST(GotoRequestTest, RefusedWhileTreeIsNotSettledInIdle)
{
  for (const char* state : {"",
                            "CRITICAL_BATTERY_DOCKING",
                            "CRITICAL_BATTERY_CHARGING",
                            "CRITICAL_BATTERY_NAV_FAILED",
                            "GOTO_LOW_BATTERY_DOCKING",
                            "GOTO_LOW_BATTERY_DOCK_FAILED",
                            "GOTO_CRITICAL_BATTERY_DOCKED",
                            "IDLE_DOCKED",
                            "EMERGENCY",
                            "WAITING_FOR_RTK"})
  {
    BTContext ctx;
    makeAvailable(ctx);
    ctx.last_high_level_status.state_name = state;
    SCOPED_TRACE(std::string("state ") + state);
    expectRefusedUnchanged(ctx, "busy");
  }
}

// A published "IDLE" can outlive the rest branch: a root guard handler (sensor
// fault, emergency stop) halts MainLogic without publishing a new state. The
// tree's own state (tree_at_rest, from the last tick) decides.
TEST(GotoRequestTest, RefusedWhenTreeIsNotParkedInItsRestBranch)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.tree_at_rest = false;  // last published state still "IDLE"
  expectRefusedUnchanged(ctx, "busy");
}

TEST(GotoRequestTest, RefusedWhileLocalizationIsDegraded)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.localization_degraded = true;
  expectRefusedUnchanged(ctx, "localization_degraded");
}

// battery_percent defaults to 100 % and freezes when samples stop: without a
// valid, recent reading the battery checks below would pass on nothing.
TEST(GotoRequestTest, RefusedWithoutValidRecentBatteryData)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.battery_voltage_filtered = 0.0f;  // no valid sample yet
  expectRefusedUnchanged(ctx, "battery_unknown");

  makeAvailable(ctx);
  ctx.last_power_time = {};  // never received
  expectRefusedUnchanged(ctx, "battery_unknown");

  makeAvailable(ctx);
  ctx.last_power_time = std::chrono::steady_clock::now() - std::chrono::seconds(3);
  expectRefusedUnchanged(ctx, "battery_unknown");
}

// The CriticalBatteryDock latches are reported, and never cleared by a GOTO
// request (only an operator command on ~/high_level_control or ~/start_in_area
// clears them).
TEST(GotoRequestTest, CriticalLatchesRefuseAndAreKept)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.critical_dock_failure_latched = true;
  expectRefusedUnchanged(ctx, "critical_dock_failure");
  EXPECT_TRUE(ctx.critical_dock_failure_latched);

  makeAvailable(ctx);
  ctx.current_command = 8;
  ctx.critical_charge_stop_latched = true;
  expectRefusedUnchanged(ctx, "critical_charge_hold");
  EXPECT_TRUE(ctx.critical_charge_stop_latched);
}

// An unfinished battery-driven charge refuses a GOTO (fail-closed) and is kept:
// a GOTO request never releases it, only STOP / HOME (applyCommandToBatteryRecovery).
TEST(GotoRequestTest, PendingBatteryChargeRefusesAndIsKept)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.battery_charge_pending = true;
  expectRefusedUnchanged(ctx, "battery_charge_pending");
  EXPECT_TRUE(ctx.battery_charge_pending);
}

// GOTO has no undock sequence: refused on the charger and mid-undock.
TEST(GotoRequestTest, RefusedWhenDocked)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.latest_power.charger_enabled = true;
  expectRefusedUnchanged(ctx, "docked");

  makeAvailable(ctx);
  ctx.undock_start_recorded = true;
  expectRefusedUnchanged(ctx, "docked");
}

TEST(GotoRequestTest, RefusedWithoutHeadingSeed)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.yaw_seeded_this_session = false;
  expectRefusedUnchanged(ctx, "heading_not_calibrated");
}

TEST(GotoRequestTest, RefusedOnLowOrCriticalBattery)
{
  BTContext ctx;
  makeAvailable(ctx);
  ctx.battery_percent = 20.0f;  // == battery_low_pct: GotoLowBatteryGuard would trip
  expectRefusedUnchanged(ctx, "battery_low");

  // Percentage fine, filtered voltage below battery_critical_voltage.
  makeAvailable(ctx);
  ctx.battery_percent = 60.0f;
  ctx.battery_voltage_filtered = 25.0f;
  expectRefusedUnchanged(ctx, "battery_low", GotoBatteryLimits{20.0f, 10.0f, 25.5f});
}

TEST(GotoRequestTest, GotoTimeoutValidation)
{
  EXPECT_TRUE(mowgli_behavior::isValidGotoTimeout(0.0));  // no limit
  EXPECT_TRUE(mowgli_behavior::isValidGotoTimeout(300.0));
  EXPECT_FALSE(mowgli_behavior::isValidGotoTimeout(-1.0));
  EXPECT_FALSE(mowgli_behavior::isValidGotoTimeout(std::nan("")));
  EXPECT_FALSE(mowgli_behavior::isValidGotoTimeout(std::numeric_limits<double>::infinity()));
}

// The target lives exactly as long as COMMAND_GOTO: dropped by the per-tick
// cleanup once the command changes, kept while it is 9.
TEST(GotoRequestTest, TargetIsDroppedOnceGotoIsNoLongerActive)
{
  BTContext ctx;
  makeAvailable(ctx);
  ASSERT_TRUE(request(ctx, 1.0, 1.0, 0.0).accepted);
  dropStaleGotoTargetLocked(ctx);
  ASSERT_TRUE(ctx.goto_target_pose.has_value()) << "active GOTO lost its target";

  for (uint8_t cmd : {0, 2, 8})  // ClearCommand, HOME, STOP/maintenance
  {
    ctx.current_command = kCommandGoto;
    ctx.goto_target_pose.emplace();
    ctx.current_command = cmd;
    dropStaleGotoTargetLocked(ctx);
    EXPECT_FALSE(ctx.goto_target_pose.has_value()) << "command " << int(cmd);
  }
}

// A request accepted between the end of a GOTO and the next tick must keep its
// own target: the cleanup sees command 9 again and leaves it alone.
TEST(GotoRequestTest, NewTargetAcceptedBeforeCleanupIsKept)
{
  BTContext ctx;
  makeAvailable(ctx);
  ASSERT_TRUE(request(ctx, 1.0, 1.0, 0.0).accepted);
  ctx.current_command = 0;  // GOTO ended (ClearCommand)
  ctx.last_high_level_status.state_name = "GOTO_REACHED";
  ASSERT_TRUE(request(ctx, 5.0, -3.0, 0.0).accepted);

  dropStaleGotoTargetLocked(ctx);

  ASSERT_TRUE(ctx.goto_target_pose.has_value());
  EXPECT_DOUBLE_EQ(ctx.goto_target_pose->pose.position.x, 5.0);
  EXPECT_DOUBLE_EQ(ctx.goto_target_pose->pose.position.y, -3.0);
}

// ---------------------------------------------------------------------------
// tree_at_rest
// ---------------------------------------------------------------------------

namespace
{

/// Multi-tick stand-in for StopMoving / WaitForDuration: RUNNING for 3 ticks.
class StubThreeTicks : public BT::StatefulActionNode
{
public:
  StubThreeTicks(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration_sec"), BT::InputPort<std::string>("command")};
  }
  BT::NodeStatus onStart() override
  {
    ticks_ = 0;
    return BT::NodeStatus::RUNNING;
  }
  BT::NodeStatus onRunning() override
  {
    return ++ticks_ >= 3 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }
  void onHalted() override
  {
  }

private:
  int ticks_{0};
};

bool g_fault = false;

class StubFault : public BT::ConditionNode
{
public:
  StubFault(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config)
  {
  }
  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override
  {
    return g_fault ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace

// The REAL StopHoldSequence and IdleSequence under a root guard shaped like
// SensorSafetyGuard (blocks every command, publishes nothing): tree_at_rest,
// computed by GotoRestObserver as behavior_tree_node does, drops on the
// very tick the guard fires — while the last published state is still IDLE —
// and comes back once the guard clears.
TEST(GotoTreeAtRestTest, RootGuardWithoutStatusClearsRestOnItsFirstTick)
{
  Recorder rec;
  g_rec = &rec;
  g_fault = false;
  auto ctx = std::make_shared<BTContext>();
  ctx->node = rclcpp::Node::make_shared("test_goto_rest");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<StubFault>("StubFault");
  factory.registerNodeType<StubSetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StubPublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<StubThreeTicks>("StopMoving");
  factory.registerNodeType<StubThreeTicks>("WaitForDuration");
  factory.registerNodeType<StubThreeTicks>("SetNav2Lifecycle");

  const std::string xml = stripComments(readMainTree());
  const std::string stop_hold = extractElement(xml, "ReactiveSequence", "StopHoldSequence");
  const std::string idle = extractElement(xml, "Sequence", "IdleSequence");
  ASSERT_FALSE(stop_hold.empty());
  ASSERT_FALSE(idle.empty());
  auto tree = factory.createTreeFromText(
      wrapTree("<ReactiveSequence name=\"Root\">"
               "<ReactiveFallback name=\"FaultGuard\"><Inverter><StubFault/></Inverter>"
               "<Sequence><StopMoving/><AlwaysFailure/></Sequence></ReactiveFallback>"
               "<Fallback name=\"MainLogic\">" +
               stop_hold + idle + "</Fallback></ReactiveSequence>"),
      blackboard);
  GotoRestObserver observer(tree);
  ASSERT_EQ(observer.restBranchCount(), 2u);
  // This synthetic tree has no GotoSequence: reported, not silently ignored.
  EXPECT_EQ(observer.missingBranches(), std::vector<std::string>{"GotoSequence"});

  std::string last_state;
  auto tick = [&]()
  {
    observer.beforeTick();
    const auto status = tree.tickOnce();
    if (!rec.statuses.empty())
    {
      last_state = rec.statuses.back();
    }
    ctx->tree_at_rest = observer.afterTick(status);
  };

  for (uint8_t cmd : {0, 8})
  {
    SCOPED_TRACE("command " + std::to_string(cmd));
    ctx->current_command = cmd;
    g_fault = false;
    tree.haltTree();
    ctx->tree_at_rest = false;
    // Two full rest cycles: at rest on every tick, completion ticks included.
    tick();
    for (int i = 0; i < 16; ++i)
    {
      tick();
      EXPECT_TRUE(ctx->tree_at_rest) << "tick " << i;
    }
    ASSERT_EQ(last_state, "IDLE");

    g_fault = true;
    for (int i = 0; i < 6; ++i)
    {
      tick();
      EXPECT_FALSE(ctx->tree_at_rest) << "fault tick " << i;
    }
    EXPECT_EQ(last_state, "IDLE") << "the guard publishes nothing: IDLE is stale";

    g_fault = false;
    tick();
    EXPECT_TRUE(ctx->tree_at_rest);
  }
  g_rec = nullptr;
}

// Rest must come from WHICH branch completed, not from the last published
// state. Another MainLogic branch completing with SUCCESS while the last
// status still reads GOTO_REACHED is not rest; GotoSequence itself completing
// is (chaining), and so is the rest branch finishing its cycle.
TEST(GotoTreeAtRestTest, OnlyRestOrGotoCompletionCountsWhateverWasPublished)
{
  Recorder rec;
  g_rec = &rec;
  auto ctx = std::make_shared<BTContext>();
  ctx->node = rclcpp::Node::make_shared("test_goto_rest_branch");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<ClearCommand>("ClearCommand");
  factory.registerNodeType<StubSetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StubPublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<StubThreeTicks>("StopMoving");
  factory.registerNodeType<StubThreeTicks>("WaitForDuration");
  factory.registerNodeType<StubThreeTicks>("SetNav2Lifecycle");
  factory.registerNodeType<StubThreeTicks>("Work");

  const std::string xml = stripComments(readMainTree());
  const std::string idle = extractElement(xml, "Sequence", "IdleSequence");
  const std::string stop_hold = extractElement(xml, "ReactiveSequence", "StopHoldSequence");
  ASSERT_FALSE(idle.empty());
  ASSERT_FALSE(stop_hold.empty());
  // "OtherBranch" completes without publishing anything; the GotoSequence
  // stand-in completes like the real one (SUCCESS after ClearCommand).
  auto tree = factory.createTreeFromText(
      wrapTree("<ReactiveSequence name=\"Root\"><Fallback name=\"MainLogic\">"
               "<Sequence name=\"GotoSequence\"><IsCommand command=\"9\"/><Work/>"
               "<ClearCommand/></Sequence>"
               "<Sequence name=\"OtherBranch\"><IsCommand command=\"3\"/><Work/>"
               "<ClearCommand/></Sequence>" +
               stop_hold + idle + "</Fallback></ReactiveSequence>"),
      blackboard);
  GotoRestObserver observer(tree);
  EXPECT_TRUE(observer.missingBranches().empty()) << "all three branches are present";

  auto tickUntilCompleted = [&](bool& at_rest_on_completion)
  {
    for (int i = 0; i < 10; ++i)
    {
      observer.beforeTick();
      const auto status = tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS)
      {
        at_rest_on_completion = observer.afterTick(status);
        return true;
      }
      if (ctx->current_command != 0)
      {
        EXPECT_FALSE(observer.afterTick(status)) << "busy branch reported as rest";
      }
    }
    return false;
  };

  bool at_rest = true;
  rec.statuses.push_back("GOTO_REACHED");  // last published state: a GOTO just ended
  ctx->current_command = 3;
  ASSERT_TRUE(tickUntilCompleted(at_rest));
  EXPECT_FALSE(at_rest) << "another branch completing is not rest";
  EXPECT_EQ(rec.statuses.back(), "GOTO_REACHED") << "test precondition: nothing published";

  ctx->current_command = 9;
  ASSERT_TRUE(tickUntilCompleted(at_rest));
  EXPECT_TRUE(at_rest) << "GotoSequence completing counts (chaining)";

  ctx->current_command = 0;
  ASSERT_TRUE(tickUntilCompleted(at_rest));
  EXPECT_TRUE(at_rest) << "the rest branch finishing its cycle counts";
  g_rec = nullptr;
}

// ---------------------------------------------------------------------------
// Tree fixture
// ---------------------------------------------------------------------------

class GotoTreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    g_rec = &rec;
    ctx = std::make_shared<BTContext>();
    ctx->node = rclcpp::Node::make_shared("test_goto_tree");
    ctx->battery_percent = 80.0f;
    ctx->current_command = kCommandGoto;
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    blackboard->set("battery_low_pct", 20.0f);
    blackboard->set("battery_critical_pct", 10.0f);
    blackboard->set("battery_critical_voltage", 0.0f);
    blackboard->set("goto_timeout_sec", 300.0);
    blackboard->set("undock_distance", 1.0);
    blackboard->set("undock_speed", 0.15);

    factory.registerNodeType<IsCommand>("IsCommand");
    factory.registerNodeType<ClearCommand>("ClearCommand");
    factory.registerNodeType<NeedsDocking>("NeedsDocking");
    factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
    factory.registerNodeType<IsLastDockSucceeded>("IsLastDockSucceeded");
    factory.registerNodeType<SetStopReason>("SetStopReason");
    factory.registerNodeType<IsResumeUndockAllowed>("IsResumeUndockAllowed");
    factory.registerNodeType<StubNavigateToPose>("NavigateToPose");
    factory.registerNodeType<StubDockRobot>("DockRobot");
    factory.registerNodeType<StubSetMowerEnabled>("SetMowerEnabled");
    factory.registerNodeType<StubPublishHighLevelStatus>("PublishHighLevelStatus");
    factory.registerNodeType<StubBackUp>("BackUp");
    factory.registerNodeType<StubStopMoving>("StopMoving");
    factory.registerNodeType<StubSucceed>("ClearCostmap");

    tree_xml = stripComments(readMainTree());
  }

  void TearDown() override
  {
    tree.reset();
    g_rec = nullptr;
  }

  void loadSubtree(const std::string& tag, const std::string& name)
  {
    const std::string body = extractElement(tree_xml, tag, name);
    ASSERT_FALSE(body.empty()) << name << " not found in main_tree.xml";
    tree = std::make_unique<BT::Tree>(factory.createTreeFromText(wrapTree(body), blackboard));
  }

  bool bladeEverEnabled() const
  {
    for (bool b : rec.blade_requests)
    {
      if (b)
      {
        return true;
      }
    }
    return false;
  }

  Recorder rec;
  std::shared_ptr<BTContext> ctx;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::unique_ptr<BT::Tree> tree;
  std::string tree_xml;
};

TEST_F(GotoTreeTest, ReachedTargetEndsMissionWithGotoReached)
{
  loadSubtree("ReactiveSequence", "GotoSequence");

  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  rec.nav_result = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_TRUE(rec.published("GOTO_ACTIVE"));
  EXPECT_EQ(rec.statuses.back(), "GOTO_REACHED");
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(rec.nav_starts, 1);
  EXPECT_FALSE(bladeEverEnabled());
  EXPECT_FALSE(rec.blade_requests.empty()) << "blade must be explicitly disabled on entry";
}

// A goal Nav2 cannot reach must not be re-sent forever: the mission ends with
// GOTO_FAILED and the next ticks do not enter the branch again.
TEST_F(GotoTreeTest, FailedNavigationEndsMissionWithoutRetry)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  rec.nav_result = BT::NodeStatus::FAILURE;

  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);  // goal sent
  // The failure is absorbed into GOTO_FAILED: the branch ends cleanly.
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::SUCCESS);
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);  // IsCommand(9) now fails
  }

  EXPECT_TRUE(rec.published("GOTO_FAILED"));
  EXPECT_FALSE(rec.published("GOTO_REACHED"));
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(rec.nav_starts, 1);
}

// Low battery during the transit: the running NavigateToPose is halted (its
// onHalted cancels the Nav2 goal), the robot docks once and the mission ends —
// the GOTO is not resumed after charging.
TEST_F(GotoTreeTest, LowBatteryHaltsTransitDocksAndEndsMission)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(rec.nav_starts, 1);
  ASSERT_EQ(rec.nav_halts, 0);

  ctx->battery_percent = 15.0f;
  tree->tickOnce();
  for (int i = 0; i < 5; ++i)
  {
    tree->tickOnce();
  }

  EXPECT_EQ(rec.nav_halts, 1) << "NavigateToPose was not interrupted";
  EXPECT_EQ(rec.dock_starts, 1);
  EXPECT_TRUE(rec.published("GOTO_LOW_BATTERY_DOCKING"));
  EXPECT_EQ(rec.statuses.back(), "GOTO_LOW_BATTERY_DOCKED");
  EXPECT_FALSE(rec.published("GOTO_LOW_BATTERY_DOCK_FAILED"));
  EXPECT_FALSE(rec.published("GOTO_REACHED"));
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(rec.nav_starts, 1) << "the GOTO must not be resumed";
  EXPECT_FALSE(bladeEverEnabled());
}

// A failed dock is tried once and ends on its own terminal status, so the last
// GOTO result never reads like a successful return.
TEST_F(GotoTreeTest, LowBatteryDockFailureSignalsAndDoesNotLoop)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  rec.dock_result = BT::NodeStatus::FAILURE;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);

  ctx->battery_percent = 15.0f;
  for (int i = 0; i < 6; ++i)
  {
    tree->tickOnce();
  }

  EXPECT_EQ(rec.dock_starts, 1) << "docking must be tried once, not looped";
  EXPECT_EQ(rec.statuses.back(), "GOTO_LOW_BATTERY_DOCK_FAILED");
  EXPECT_EQ(ctx->stop_reason, stop_reason::kDockNavFailed);
  EXPECT_FALSE(rec.published("GOTO_LOW_BATTERY_DOCKED"));
  EXPECT_EQ(ctx->current_command, 0);
  EXPECT_EQ(rec.nav_starts, 1);
}

// The same critical test CriticalBatteryDock enters on, inside the GOTO: a
// filtered voltage below battery_critical_voltage trips the guard even while
// the percentage still reads above battery_low_pct.
TEST_F(GotoTreeTest, CriticalVoltageWithNormalPercentHaltsAndDocks)
{
  blackboard->set("battery_critical_voltage", 25.5f);
  ctx->battery_voltage_filtered = 27.0f;
  loadSubtree("ReactiveSequence", "GotoSequence");
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(rec.nav_starts, 1);

  ctx->battery_percent = 60.0f;
  ctx->battery_voltage_filtered = 25.0f;
  for (int i = 0; i < 4; ++i)
  {
    tree->tickOnce();
  }

  EXPECT_EQ(rec.nav_halts, 1) << "NavigateToPose was not interrupted";
  EXPECT_EQ(rec.dock_starts, 1);
  EXPECT_EQ(rec.statuses.back(), "GOTO_LOW_BATTERY_DOCKED");
  EXPECT_EQ(ctx->current_command, 0);
}

// The pack voltage recovers once the motors stop. That must not abort the
// return already under way, nor restart the GOTO: the guard's Fallback keeps
// its running child.
TEST_F(GotoTreeTest, BatteryRecoveryDuringDockingDoesNotAbortIt)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  rec.dock_result = BT::NodeStatus::RUNNING;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ctx->battery_percent = 15.0f;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(rec.dock_starts, 1);

  ctx->battery_percent = 35.0f;
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  }
  rec.dock_result = BT::NodeStatus::SUCCESS;
  tree->tickOnce();

  EXPECT_EQ(rec.dock_halts, 0);
  EXPECT_EQ(rec.dock_starts, 1);
  EXPECT_EQ(rec.nav_starts, 1) << "the GOTO must not restart";
  EXPECT_EQ(rec.statuses.back(), "GOTO_LOW_BATTERY_DOCKED");
  EXPECT_EQ(ctx->current_command, 0);
}

// An operator command during the low-battery dock (here STOP) halts the
// docking — the same precedence HOME's dock transit has — and leaves the new
// command for its own branch; the GOTO is not resumed.
TEST_F(GotoTreeTest, CommandChangeDuringDockingHaltsIt)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  rec.dock_result = BT::NodeStatus::RUNNING;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ctx->battery_percent = 15.0f;
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(rec.dock_starts, 1);

  ctx->current_command = 8;
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);

  EXPECT_EQ(rec.dock_halts, 1);
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_EQ(rec.nav_starts, 1);
  EXPECT_FALSE(rec.published("GOTO_LOW_BATTERY_DOCKED"));
  EXPECT_FALSE(rec.published("GOTO_LOW_BATTERY_DOCK_FAILED"));
}

// Blade off, then stop, BEFORE any motion — on the first entry and again when
// the branch is re-entered after a halt from above (a root guard pausing the
// GOTO and releasing it with command 9 still set).
TEST_F(GotoTreeTest, BladeIsSwitchedOffBeforeAnyMotionOnEveryEntry)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  tree->haltTree();
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(rec.nav_starts, 2);
  ctx->battery_percent = 15.0f;
  tree->tickOnce();
  ASSERT_EQ(rec.dock_starts, 1);

  bool blade_off = false;
  bool stopped = false;
  for (const auto& e : rec.events)
  {
    if (e == "blade_off")
    {
      blade_off = true;
      stopped = false;
    }
    else if (e == "stop")
    {
      stopped = blade_off;
    }
    else if (e == "nav_start" || e == "dock_start")
    {
      EXPECT_TRUE(blade_off && stopped) << e << " without blade-off + stop just before";
      blade_off = false;
      stopped = false;
    }
    EXPECT_NE(e, "blade_on");
  }
}

TEST_F(GotoTreeTest, LowBatteryAtStartDocksWithoutStartingNavigation)
{
  ctx->battery_percent = 10.0f;
  loadSubtree("ReactiveSequence", "GotoSequence");

  tree->tickOnce();

  EXPECT_EQ(rec.nav_starts, 0);
  EXPECT_EQ(rec.dock_starts, 1);
  EXPECT_EQ(ctx->current_command, 0);
}

// Another command (here STOP) during the transit halts the navigation and
// leaves the new command in place for its own branch.
TEST_F(GotoTreeTest, CommandChangeHaltsTransit)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);

  ctx->current_command = 8;
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);

  EXPECT_EQ(rec.nav_halts, 1);
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_FALSE(rec.published("GOTO_REACHED"));
  EXPECT_FALSE(rec.published("GOTO_FAILED"));
}

// Update maintenance raised mid-GOTO, through the same locked prelude as
// BehaviorTreeNode::tickTree(): refreshFleetYieldLocked forces STOP, then
// dropStaleGotoTargetLocked drops the target in the same tick. The transit is
// halted (the real node cancels its Nav2 goal on halt, see
// test_goto_navigate_to_pose) and nothing restarts on later ticks.
TEST_F(GotoTreeTest, MaintenanceMidGotoDropsTargetHaltsTransitAndNeverRestarts)
{
  loadSubtree("ReactiveSequence", "GotoSequence");
  ctx->goto_target_pose = geometry_msgs::msg::PoseStamped{};
  ASSERT_EQ(tree->tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(rec.nav_starts, 1);

  const auto flag = std::filesystem::temp_directory_path() / "test_goto_tree_maintenance";
  std::ofstream(flag).put('\n');
  ASSERT_EQ(setenv("MOWGLI_UPDATE_MAINTENANCE", flag.c_str(), 1), 0);
  auto prelude = [this]()
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    mowgli_behavior::refreshFleetYieldLocked(*ctx);
    dropStaleGotoTargetLocked(*ctx);
  };

  prelude();
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_FALSE(ctx->goto_target_pose.has_value());
  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(rec.nav_halts, 1);

  for (int i = 0; i < 5; ++i)
  {
    prelude();
    tree->tickOnce();
  }
  unsetenv("MOWGLI_UPDATE_MAINTENANCE");
  std::filesystem::remove(flag);

  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_FALSE(ctx->goto_target_pose.has_value());
  EXPECT_EQ(rec.nav_starts, 1) << "no new transit after maintenance";
  EXPECT_FALSE(rec.published("GOTO_REACHED"));
  EXPECT_FALSE(rec.published("GOTO_FAILED"));
}

// ---------------------------------------------------------------------------
// CriticalBatteryDock exit after the charge hold
// ---------------------------------------------------------------------------

TEST_F(GotoTreeTest, CriticalChargeEndsGotoDockedWithoutBackUp)
{
  loadSubtree("Fallback", "CriticalBatteryResumeOrGotoTerminate");
  ctx->current_command = kCommandGoto;

  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::SUCCESS);

  EXPECT_EQ(rec.backups, 0) << "a GOTO must stay docked after a critical charge";
  EXPECT_EQ(rec.statuses.back(), "GOTO_CRITICAL_BATTERY_DOCKED");
  EXPECT_FALSE(rec.published("MOWING"));
  EXPECT_EQ(ctx->current_command, 0);
}

// Regression guard: only an active mow (command 1) takes the AUTO-CONTINUE
// tail (undock with BackUp, publish MOWING, keep current_command).
TEST_F(GotoTreeTest, CriticalChargeKeepsAutoContinueForMowing)
{
  ctx->current_command = 1;
  ctx->resume_undock_failures = 0;
  loadSubtree("Fallback", "CriticalBatteryResumeOrGotoTerminate");

  EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(rec.backups, 1);
  ASSERT_EQ(rec.statuses.size(), 2u);
  EXPECT_EQ(rec.statuses[0], "RESUMING_UNDOCKING");
  EXPECT_EQ(rec.statuses[1], "MOWING");
  EXPECT_EQ(ctx->current_command, 1);
}

// Every other command leaves the robot docked with its command untouched.
TEST_F(GotoTreeTest, CriticalChargeStaysDockedForOtherCommands)
{
  for (uint8_t cmd : {0, 2, 7, 8})
  {
    rec = Recorder{};
    ctx->current_command = cmd;
    loadSubtree("Fallback", "CriticalBatteryResumeOrGotoTerminate");

    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE) << "command " << int(cmd);
    EXPECT_EQ(rec.backups, 0) << "command " << int(cmd);
    EXPECT_TRUE(rec.statuses.empty()) << "command " << int(cmd);
    EXPECT_EQ(ctx->current_command, cmd) << "command " << int(cmd);
  }
}

// ---------------------------------------------------------------------------
// Structure of the shipped tree
// ---------------------------------------------------------------------------

// A root guard may exempt GOTO only for the blade-off dock transit, exactly
// like mowing: command 9 appears in the root guards only as
// IsCommand 9 AND IsDocking, once in BoundaryGuard and once in
// LocalizationGuard. The GOTO transit itself stays fully protected.
TEST(GotoTreeStructureTest, RootGuardsExemptGotoOnlyDuringDockTransit)
{
  const std::string xml = stripComments(readMainTree());
  const auto main_logic = xml.find("<Fallback name=\"MainLogic\">");
  ASSERT_NE(main_logic, std::string::npos);
  std::string root_guards = xml.substr(0, main_logic);
  const std::regex dock_exempt(
      "<Sequence name=\"(Loc)?GotoDockTransitExempt\">\\s*<IsCommand command=\"9\"/>"
      "\\s*<IsDocking/>\\s*</Sequence>");
  for (const char* guard : {"BoundaryGuard", "LocalizationGuard"})
  {
    const std::string body = extractElement(root_guards, "ReactiveFallback", guard);
    ASSERT_FALSE(body.empty()) << guard << " not found";
    const auto count = std::distance(std::sregex_iterator(body.begin(), body.end(), dock_exempt),
                                     std::sregex_iterator());
    EXPECT_EQ(count, 1) << guard << " must hold exactly one GOTO dock-transit exemption";
  }
  root_guards = std::regex_replace(root_guards, dock_exempt, "");
  EXPECT_EQ(root_guards.find("command=\"9\""), std::string::npos)
      << "a root guard exempts COMMAND_GOTO outside the dock transit";
}

// The exemption only passes while DockRobot runs (IsDocking), never during the
// GOTO transit.
TEST_F(GotoTreeTest, GotoDockTransitExemptionNeedsDocking)
{
  factory.registerNodeType<IsDocking>("IsDocking");
  for (const char* name : {"GotoDockTransitExempt", "LocGotoDockTransitExempt"})
  {
    loadSubtree("Sequence", name);
    ctx->current_command = kCommandGoto;
    ctx->docking_active = false;
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE) << name << ": GOTO transit";
    ctx->docking_active = true;
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::SUCCESS) << name << ": dock transit";
    ctx->current_command = 0;
    EXPECT_EQ(tree->tickOnce(), BT::NodeStatus::FAILURE) << name << ": other command";
  }
}

TEST(GotoTreeStructureTest, GotoBranchNeverEnablesTheBlade)
{
  const std::string xml = stripComments(readMainTree());
  const std::string goto_seq = extractElement(xml, "ReactiveSequence", "GotoSequence");
  ASSERT_FALSE(goto_seq.empty());
  EXPECT_EQ(goto_seq.find("enabled=\"true\""), std::string::npos);
  EXPECT_NE(goto_seq.find("<SetMowerEnabled enabled=\"false\"/>"), std::string::npos);
}

// GotoRestObserver (and so ~/goto_pose) relies on these three branch names.
// Each must exist exactly once in the shipped tree, so a rename or a removal
// is caught here rather than in the field (where every GOTO would be refused).
TEST(GotoTreeStructureTest, BranchesGotoReliesOnExistOnce)
{
  const std::string xml = stripComments(readMainTree());
  for (const char* name : {"IdleSequence", "StopHoldSequence", "GotoSequence"})
  {
    const std::regex re(std::string("name=\"") + name + "\"");
    const auto count =
        std::distance(std::sregex_iterator(xml.begin(), xml.end(), re), std::sregex_iterator());
    EXPECT_EQ(count, 1) << name;
  }
}

// A goal bt_navigator keeps recovering must not hold the GOTO forever.
TEST(GotoTreeStructureTest, GotoNavigationIsTimeBounded)
{
  const std::string xml = stripComments(readMainTree());
  const std::string goto_seq = extractElement(xml, "ReactiveSequence", "GotoSequence");
  ASSERT_FALSE(goto_seq.empty());
  EXPECT_NE(goto_seq.find("<NavigateToPose timeout_sec=\"{goto_timeout_sec}\"/>"),
            std::string::npos);
}

// GOTO navigation must use bt_navigator's default tree (AuthorizedTransitPlanner),
// never the unrestricted BoundaryRecovery lane.
TEST(GotoTreeStructureTest, GotoBranchDoesNotTouchBoundaryRecovery)
{
  const std::string xml = stripComments(readMainTree());
  const std::string goto_seq = extractElement(xml, "ReactiveSequence", "GotoSequence");
  ASSERT_FALSE(goto_seq.empty());
  EXPECT_EQ(goto_seq.find("NavigateInsideBoundary"), std::string::npos);
  EXPECT_EQ(goto_seq.find("BoundaryRecovery"), std::string::npos);
}
