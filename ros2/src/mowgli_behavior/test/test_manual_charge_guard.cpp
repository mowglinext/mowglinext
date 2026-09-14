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
 * @file test_manual_charge_guard.cpp
 * @brief IsCharging's stable_for_sec debounce, and the ManualChargeGuard
 *        shape that consumes it.
 *
 * ManualChargeGuard stops the mow when the operator physically puts the mower
 * back on the dock mid-run. It is the first reader of IsCharging for which a
 * TRANSIENT charger bit is a state transition rather than a no-op: it turns the
 * blade off, stops the robot, publishes CHARGING, waits, then publishes MOWING.
 * The firmware bit is not clean enough for that — it stays high ~100 ms into a
 * BackUp undock (CLAUDE.md invariant 11) and can bounce when the mower brushes
 * the dock contacts on a swath that runs close to the station.
 *
 * So IsCharging gained an OPTIONAL stable_for_sec port. Two things have to hold
 * and both are pinned here:
 *
 *   (a) stable_for_sec defaults to 0.0 == the raw bit, byte-for-byte the old
 *       behaviour, because the other ten <IsCharging/> sites in main_tree.xml
 *       (LocalizationGuard's exemption, UndockOrSkip, BatteryGuard, ...) name
 *       no port and must not change at all;
 *
 *   (b) the guard wraps the debounced condition in an <Inverter>, which is the
 *       one place a mistake here would be subtle. "not (charging, settled)" has
 *       to pass mowing through BOTH when the robot is off the dock AND during
 *       the not-yet-settled window, and only engage the handler once the bit
 *       has held. Getting that backwards would either stop the mow on every
 *       flicker or never stop it at all.
 */

#include <chrono>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsCharging;

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

namespace
{

// The real guard's debounce. Used wherever the test asserts that a transient
// does NOT trip, which needs no waiting and so can use the production value.
constexpr double kGuardStableForSec = 3.0;

// A short window for the assertions that must actually WAIT the debounce out,
// so the suite stays fast. The elapsed-time arithmetic is identical.
constexpr double kShortStableForSec = 0.20;

void SleepMs(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

std::shared_ptr<BTContext> MakeContext(const std::string& node_name)
{
  auto ctx = std::make_shared<BTContext>();
  ctx->node = rclcpp::Node::make_shared(node_name);
  ctx->latest_power.charger_enabled = false;
  return ctx;
}

// ---------------------------------------------------------------------------
// A single bare IsCharging, so the port can be exercised in isolation.
// ---------------------------------------------------------------------------

std::string BareConditionXml(const std::string& attrs)
{
  return R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <IsCharging )" +
         attrs + R"(/>
      </BehaviorTree>
    </root>)";
}

BT::Tree MakeBareTree(BT::BehaviorTreeFactory& factory,
                      const std::shared_ptr<BTContext>& ctx,
                      const std::string& attrs)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);
  factory.registerNodeType<IsCharging>("IsCharging");
  return factory.createTreeFromText(BareConditionXml(attrs), blackboard);
}

// ---------------------------------------------------------------------------
// The ManualChargeGuard shape: the real StripGuards nesting with the handler
// body reduced to a probe. Mirrors main_tree.xml's
//   ReactiveSequence[ ..., Fallback[ Inverter[IsCharging ...], Sequence ], ... ]
//
// The probe sits under KeepRunningUntilFailure so the handler reports RUNNING,
// which is what the real handler does while its RetryUntilSuccessful wait loop
// is holding for the operator. That is load-bearing: a handler that returned
// SUCCESS would let the StripGuards ReactiveSequence advance to the coverage
// child on the same tick, and the "guard blocks coverage" assertions would
// pass or fail for the wrong reason.
// ---------------------------------------------------------------------------

struct Probe
{
  int handler_ticks{0};
  int coverage_ticks{0};
};

std::string GuardShapeXml(double stable_for_sec)
{
  std::ostringstream ss;
  ss << R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <ReactiveSequence name="StripGuards">
          <Fallback name="ManualChargeGuard">
            <Inverter><IsCharging stable_for_sec=")"
     << stable_for_sec << R"("/></Inverter>
            <Sequence name="ManualChargeHandler">
              <KeepRunningUntilFailure>
                <ManualChargeHandlerProbe/>
              </KeepRunningUntilFailure>
            </Sequence>
          </Fallback>
          <CoverageProbe/>
        </ReactiveSequence>
      </BehaviorTree>
    </root>)";
  return ss.str();
}

BT::Tree MakeGuardTree(BT::BehaviorTreeFactory& factory,
                       const std::shared_ptr<BTContext>& ctx,
                       Probe* probe,
                       double stable_for_sec)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);
  factory.registerNodeType<IsCharging>("IsCharging");
  factory.registerSimpleAction("ManualChargeHandlerProbe",
                               [probe](BT::TreeNode&)
                               {
                                 probe->handler_ticks++;
                                 return BT::NodeStatus::SUCCESS;
                               });
  factory.registerSimpleAction("CoverageProbe",
                               [probe](BT::TreeNode&)
                               {
                                 probe->coverage_ticks++;
                                 return BT::NodeStatus::SUCCESS;
                               });
  return factory.createTreeFromText(GuardShapeXml(stable_for_sec), blackboard);
}

}  // namespace

// ---------------------------------------------------------------------------
// (a) The default port must reproduce the raw bit exactly.
// ---------------------------------------------------------------------------

// The other ten <IsCharging/> nodes in main_tree.xml name no port. Each must keep
// tracking the charger bit tick-for-tick, with no window of any kind.
TEST(ManualChargeGuardTest, DefaultPortTracksTheRawChargerBit)
{
  auto ctx = MakeContext("test_ischarging_default");
  BT::BehaviorTreeFactory factory;
  auto tree = MakeBareTree(factory, ctx, "");

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
      << "The undebounced default introduced a window. Every existing "
         "<IsCharging/> call site would change behaviour.";

  ctx->latest_power.charger_enabled = false;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// An explicit 0.0 is the same thing as omitting the port.
TEST(ManualChargeGuardTest, ExplicitZeroIsTheRawBit)
{
  auto ctx = MakeContext("test_ischarging_zero");
  BT::BehaviorTreeFactory factory;
  auto tree = MakeBareTree(factory, ctx, R"(stable_for_sec="0.0")");

  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// ---------------------------------------------------------------------------
// (b) The debounce itself.
// ---------------------------------------------------------------------------

// The documented failure: the charger bit stays high ~100 ms into a BackUp
// undock. Against the production 3 s window that must never read SUCCESS.
TEST(ManualChargeGuardTest, ShortBlipDoesNotTripTheDebounce)
{
  auto ctx = MakeContext("test_ischarging_blip");
  BT::BehaviorTreeFactory factory;
  auto tree = MakeBareTree(factory, ctx, R"(stable_for_sec="3.0")");

  ctx->latest_power.charger_enabled = true;
  const auto started = std::chrono::steady_clock::now();
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
        << "A 100 ms charger blip tripped a " << kGuardStableForSec << " s debounce on tick " << i;
    SleepMs(20);
  }
  // Guard the guard: if this loop ever ran longer than the window the
  // assertion above would be vacuous.
  ASSERT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
            kGuardStableForSec)
      << "The blip loop outlasted the debounce window; the test proves nothing.";

  ctx->latest_power.charger_enabled = false;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

// Continuous charging past the window does trip it.
TEST(ManualChargeGuardTest, ContinuousChargingTripsOnceTheWindowElapses)
{
  auto ctx = MakeContext("test_ischarging_elapsed");
  BT::BehaviorTreeFactory factory;
  auto tree = MakeBareTree(factory, ctx, R"(stable_for_sec="0.20")");

  ctx->latest_power.charger_enabled = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE) << "Tripped on the very first charging tick.";

  SleepMs(static_cast<int>(kShortStableForSec * 1000) + 60);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
      << "Charging held continuously past the window and the debounce never released.";
}

// A single off sample restarts the window — an intermittent bit must not
// accumulate credit across separate contacts.
TEST(ManualChargeGuardTest, DropResetsTheWindow)
{
  auto ctx = MakeContext("test_ischarging_reset");
  BT::BehaviorTreeFactory factory;
  auto tree = MakeBareTree(factory, ctx, R"(stable_for_sec="0.20")");

  // Most of the way through the window...
  ctx->latest_power.charger_enabled = true;
  tree.tickOnce();
  SleepMs(150);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // ...then one off sample.
  ctx->latest_power.charger_enabled = false;
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  // Back on. The accumulated 150 ms must be gone, so a further 100 ms — which
  // would have completed the ORIGINAL window — must still read FAILURE.
  ctx->latest_power.charger_enabled = true;
  tree.tickOnce();
  SleepMs(100);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE)
      << "The window survived a non-charging sample, so a bouncing charger bit "
         "can accumulate a full window out of transients.";

  SleepMs(static_cast<int>(kShortStableForSec * 1000));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS) << "The window never restarted either.";
}

// ---------------------------------------------------------------------------
// (c) The inverted condition inside the real guard shape.
// ---------------------------------------------------------------------------

// Off the dock: the guard is a pass-through and coverage runs on every tick.
TEST(ManualChargeGuardTest, GuardPassesThroughInstantlyWhenNotCharging)
{
  auto ctx = MakeContext("test_guard_not_charging");
  Probe probe;
  BT::BehaviorTreeFactory factory;
  auto tree = MakeGuardTree(factory, ctx, &probe, kGuardStableForSec);

  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(probe.handler_ticks, 0) << "The handler engaged with the robot off the dock.";
  EXPECT_EQ(probe.coverage_ticks, 5) << "The guard blocked coverage while not charging.";
}

// The subtle one. Inverted, a NOT-YET-SETTLED charging bit must still read as
// "pass mowing through" — that is the whole point of the debounce. If the
// inversion were the other way round, this is where an undock blip or a
// contact bounce would stop the mow.
TEST(ManualChargeGuardTest, GuardKeepsMowingDuringTheUnsettledWindow)
{
  auto ctx = MakeContext("test_guard_unsettled");
  Probe probe;
  BT::BehaviorTreeFactory factory;
  auto tree = MakeGuardTree(factory, ctx, &probe, kGuardStableForSec);

  ctx->latest_power.charger_enabled = true;
  const auto started = std::chrono::steady_clock::now();
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    SleepMs(20);
  }
  ASSERT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count(),
            kGuardStableForSec);

  EXPECT_EQ(probe.handler_ticks, 0)
      << "A charger transient shorter than the debounce still engaged the handler — this is "
         "exactly the blade stop + CHARGING/MOWING round trip the debounce exists to prevent.";
  EXPECT_EQ(probe.coverage_ticks, 5) << "Coverage was interrupted during the unsettled window.";
}

// ...and once the bit HAS settled, the handler engages and coverage stops.
TEST(ManualChargeGuardTest, GuardEngagesOnceChargingHasSettled)
{
  auto ctx = MakeContext("test_guard_settled");
  Probe probe;
  BT::BehaviorTreeFactory factory;
  auto tree = MakeGuardTree(factory, ctx, &probe, kShortStableForSec);

  ctx->latest_power.charger_enabled = true;
  tree.tickOnce();
  ASSERT_EQ(probe.handler_ticks, 0);
  const int coverage_before = probe.coverage_ticks;

  SleepMs(static_cast<int>(kShortStableForSec * 1000) + 60);
  tree.tickOnce();

  EXPECT_EQ(probe.handler_ticks, 1)
      << "The mower has been sitting on the dock past the debounce and the guard never engaged.";
  EXPECT_EQ(probe.coverage_ticks, coverage_before)
      << "Coverage kept running while the guard's handler was active.";
}

// ---------------------------------------------------------------------------
// (d) Structural — the real main_tree.xml.
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

/// Text of the <Fallback name="ManualChargeGuard"> element, closed at its own
/// indentation so the nested Fallback does not end the span early. Same
/// approach as test_guard_fallthrough.cpp's ExtractGuardBlock.
std::string ExtractManualChargeGuard(const std::string& xml)
{
  std::istringstream in(xml);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);)
  {
    lines.push_back(line);
  }

  const std::regex open_re(R"rx(<Fallback name="ManualChargeGuard")rx");
  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    if (!std::regex_search(lines[i], open_re))
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

}  // namespace

// The guard's ENTRY condition must stay debounced. Dropping the port would
// silently restore "stop the mow on any charger flicker".
TEST(ManualChargeGuardTest, TreeEntryConditionIsDebounced)
{
  const std::string block = ExtractManualChargeGuard(ReadMainTree());
  ASSERT_FALSE(block.empty()) << "ManualChargeGuard not found in main_tree.xml.";

  std::smatch m;
  // Custom delimiter: the pattern contains )" which would close a plain R"( ).
  const std::regex entry_re(R"rx(<Inverter><IsCharging stable_for_sec="([0-9.]+)"/></Inverter>)rx");
  ASSERT_TRUE(std::regex_search(block, m, entry_re))
      << "ManualChargeGuard's entry condition is no longer a debounced IsCharging. Undebounced, "
         "the ~100 ms charger-high window during a BackUp undock (CLAUDE.md invariant 11) and any "
         "dock-contact bounce become a blade stop plus a CHARGING/MOWING round trip mid-mow.";
  EXPECT_GT(std::stod(m[1].str()), 0.0) << "stable_for_sec=0 is the raw bit — no debounce at all.";
}

// The EXIT condition must stay RAW. It is a separate node instance with its own
// window, so a debounced exit would read FAILURE on its first tick and release
// the guard immediately — the guard would never hold at all.
TEST(ManualChargeGuardTest, TreeExitConditionIsNotDebounced)
{
  const std::string block = ExtractManualChargeGuard(ReadMainTree());
  ASSERT_FALSE(block.empty());

  const std::string retry_marker = "<RetryUntilSuccessful";
  const std::size_t retry_at = block.find(retry_marker);
  ASSERT_NE(retry_at, std::string::npos);

  const std::string wait_loop = block.substr(retry_at);
  EXPECT_NE(wait_loop.find("<Inverter><IsCharging/></Inverter>"), std::string::npos)
      << "The wait loop's exit condition is no longer a raw <IsCharging/>. A debounced instance "
         "there starts its own window at zero and releases the guard on its first tick.";
}

// The 24 h fall-through is a deliberate design decision, so pin BOTH factors.
// Changing either silently changes how long a manually docked mower is held
// before its run is declared finished.
TEST(ManualChargeGuardTest, TreeWaitLoopStillBoundsAtTwentyFourHours)
{
  const std::string block = ExtractManualChargeGuard(ReadMainTree());
  ASSERT_FALSE(block.empty());

  std::smatch attempts_m;
  std::smatch duration_m;
  ASSERT_TRUE(std::regex_search(block, attempts_m, std::regex(R"rx(num_attempts="([0-9]+)")rx")));
  ASSERT_TRUE(std::regex_search(block, duration_m, std::regex(R"rx(duration_sec="([0-9.]+)")rx")));

  const double total_sec = std::stod(attempts_m[1].str()) * std::stod(duration_m[1].str());
  EXPECT_DOUBLE_EQ(total_sec, 24.0 * 3600.0)
      << "ManualChargeGuard's wait loop no longer bounds at 24 h (got " << total_sec / 3600.0
      << " h). On expiry the guard FAILS, StripGuards fails, and "
         "StripCoverageWithRecovery falls through to CoverageEnded — the run is treated as "
         "finished. If that duration is meant to change, update the XML comment that documents "
         "the expiry outcome along with it.";
}

// ...and the outcome must stay documented in the tree, since it is not obvious
// from the node names that expiry ends the mowing session.
TEST(ManualChargeGuardTest, TreeDocumentsTheExpiryOutcome)
{
  const std::string block = ExtractManualChargeGuard(ReadMainTree());
  ASSERT_FALSE(block.empty());

  EXPECT_NE(block.find("24 h"), std::string::npos)
      << "The wait loop's duration is not stated in the comment.";
  EXPECT_NE(block.find("CoverageEnded"), std::string::npos)
      << "The comment does not say where the tree goes when the wait loop expires.";
}
