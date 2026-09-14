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
 * @file test_guard_fallthrough.cpp
 * @brief Regression tests for the "blocking guard falls through into
 *        MainLogic" bug class.
 *
 * Field incident 2026-08-18 (issues #459, #445): fusion_graph's marginal
 * covariance spiked to sigma_xy = 0.83 m while the receiver held RTK-Fixed
 * at 100 %. LocalizationGuard correctly latched and published
 * WAITING_FOR_RTK — but its handler Sequence ended with
 * <WaitForDuration duration_sec="2.0"/>, which returns SUCCESS. That made
 * the ReactiveFallback succeed, so the Root ReactiveSequence advanced into
 * MainLogic -> MowingSequence for one tick per cycle, restarting the mowing
 * sequence from its FIRST node (PREFLIGHT_CHECK, UNDOCKING) every 2 s:
 *
 *   249.788  WAITING_FOR_RTK      <- handler starts its 2.0 s wait
 *   251.887  PREFLIGHT_CHECK      <- 249.788 + 2.0 s, handler returned SUCCESS
 *   251.889  UNDOCKING
 *   251.989  SetMowerEnabled=false / StopMoving   <- guard re-latches
 *
 * Because each restart was halted ~1 s later at the UndockOrSkip wait, the
 * tree could never reach the coverage branch while sigma flapped — a
 * livelock, and on recovery the run re-entered from the top instead of
 * resuming the strip.
 *
 * The contract every BLOCKING guard must honour: while the guard's fault
 * condition holds, the guard returns FAILURE so the Root ReactiveSequence
 * halts BEFORE MainLogic. EmergencyGuard, SensorSafetyGuard, RainGuard and
 * BatteryGuard already terminated their handlers with <AlwaysFailure/>;
 * LocalizationGuard and BoundaryGuard did not.
 *
 * Two layers of coverage here:
 *   (a) behavioural — tick the guard shape and assert MainLogic never runs
 *       while degraded, and DOES run once it clears;
 *   (b) structural — parse the real main_tree.xml and assert every blocking
 *       guard handler terminates with FAILURE, so a NEWLY ADDED guard cannot
 *       reintroduce the bug.
 */

#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::IsCharging;
using mowgli_behavior::IsCommand;
using mowgli_behavior::IsDigEscalated;
using mowgli_behavior::IsLocalizationDegraded;

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

// Counts how many times MainLogic was reached. Stands in for the whole
// MainLogic -> MowingSequence subtree, whose first observable act in the
// field log was publishing PREFLIGHT_CHECK.
struct MainLogicProbe
{
  int ticks{0};
};

/// Root ReactiveSequence [ LocalizationGuard, MainLogicProbe ], mirroring
/// main_tree.xml. @p terminator is spliced in as the last child of the
/// handler Sequence so the fixed and pre-fix shapes can be compared.
std::string BuildTreeXml(const std::string& terminator)
{
  return R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="Main">
        <ReactiveSequence name="Root">
          <ReactiveFallback name="LocalizationGuard">
            <IsCharging/>
            <IsCommand command="7"/>
            <Inverter><IsLocalizationDegraded/></Inverter>
            <Sequence name="LocalizationDegradedHandler">
              <!-- Stands in for blade-off + StopMoving + status publish +
                   WaitForDuration: a handler body that COMPLETES, i.e.
                   returns SUCCESS. That completion is what leaked. -->
              <AlwaysSuccess/>
              )" +
         terminator + R"(
            </Sequence>
          </ReactiveFallback>
          <MainLogicProbe/>
        </ReactiveSequence>
      </BehaviorTree>
    </root>)";
}

std::shared_ptr<BTContext> MakeContext(const std::string& node_name)
{
  auto ctx = std::make_shared<BTContext>();
  ctx->node = rclcpp::Node::make_shared(node_name);
  ctx->current_command = 1;  // COMMAND_START — mowing, not an exempt mode
  ctx->latest_power.charger_enabled = false;
  ctx->localization_degraded = true;
  return ctx;
}

/// Build the guard tree with @p terminator spliced into the handler.
/// The context reaches the condition nodes through the "context" blackboard
/// entry, matching behavior_tree_node.cpp and the other BT tests.
BT::Tree MakeTree(BT::BehaviorTreeFactory& factory,
                  const std::shared_ptr<BTContext>& ctx,
                  MainLogicProbe* probe,
                  const std::string& terminator)
{
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);

  factory.registerNodeType<IsCharging>("IsCharging");
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<IsLocalizationDegraded>("IsLocalizationDegraded");
  factory.registerSimpleAction("MainLogicProbe",
                               [probe](BT::TreeNode&)
                               {
                                 probe->ticks++;
                                 return BT::NodeStatus::SUCCESS;
                               });
  return factory.createTreeFromText(BuildTreeXml(terminator), blackboard);
}

// ---------------------------------------------------------------------------
// (a) Behavioural
// ---------------------------------------------------------------------------

// The bug, pinned: without the terminator the handler's SUCCESS propagates
// and MainLogic runs on every cycle despite localization being degraded.
TEST(GuardFallthroughTest, HandlerWithoutTerminatorLeaksIntoMainLogic)
{
  auto ctx = MakeContext("test_guard_leak");
  MainLogicProbe probe;
  BT::BehaviorTreeFactory factory;
  auto tree = MakeTree(factory, ctx, &probe, "");

  for (int i = 0; i < 5; ++i)
  {
    tree.tickOnce();
  }

  EXPECT_GT(probe.ticks, 0) << "Expected the pre-fix shape to leak into MainLogic — if this now "
                               "reports 0, BT.CPP semantics changed and the fix's rationale needs "
                               "re-deriving rather than the assertion being flipped.";
}

// The fix: while degraded, MainLogic is never reached.
TEST(GuardFallthroughTest, DegradedLocalizationNeverReachesMainLogic)
{
  auto ctx = MakeContext("test_guard_blocks");
  MainLogicProbe probe;
  BT::BehaviorTreeFactory factory;
  auto tree = MakeTree(factory, ctx, &probe, "<AlwaysFailure/>");

  for (int i = 0; i < 20; ++i)
  {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  }

  EXPECT_EQ(probe.ticks, 0) << "MainLogic ran while localization was degraded — MowingSequence "
                               "would restart from PREFLIGHT_CHECK/UNDOCKING (issues #459, #445).";
}

// ...and the guard must RELEASE promptly once the hysteresis clears, or the
// fix would trade a livelock for a permanent stall.
TEST(GuardFallthroughTest, MainLogicResumesOnTheTickAfterRecovery)
{
  auto ctx = MakeContext("test_guard_release");
  MainLogicProbe probe;
  BT::BehaviorTreeFactory factory;
  auto tree = MakeTree(factory, ctx, &probe, "<AlwaysFailure/>");

  tree.tickOnce();
  ASSERT_EQ(probe.ticks, 0);

  ctx->localization_degraded = false;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(probe.ticks, 1) << "Guard did not release within one tick of recovery.";
}

// The exemptions must still short-circuit ahead of the handler: manual
// mowing (command 7) is operator-supervised, and on the charger the pose is
// gauge-pinned. Neither may be blocked by a degraded-localization latch.
TEST(GuardFallthroughTest, ExemptModesStillReachMainLogicWhileDegraded)
{
  for (const auto& [command, charging] : std::vector<std::pair<int, bool>>{{7, false}, {1, true}})
  {
    auto ctx = MakeContext("test_guard_exempt_" + std::to_string(command) +
                           (charging ? "_chg" : "_nochg"));
    ctx->current_command = command;
    ctx->latest_power.charger_enabled = charging;
    ASSERT_TRUE(ctx->localization_degraded);

    MainLogicProbe probe;
    BT::BehaviorTreeFactory factory;
    auto tree = MakeTree(factory, ctx, &probe, "<AlwaysFailure/>");

    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS)
        << "command=" << command << " charging=" << charging;
    EXPECT_EQ(probe.ticks, 1) << "Exempt mode was blocked by LocalizationGuard: command=" << command
                              << " charging=" << charging;
  }
}

// ---------------------------------------------------------------------------
// (b) Structural — the real main_tree.xml
// ---------------------------------------------------------------------------

std::string ReadMainTree()
{
  std::ifstream f(MOWGLI_MAIN_TREE_PATH);
  EXPECT_TRUE(f.is_open()) << "Cannot open " << MOWGLI_MAIN_TREE_PATH;
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

/// Extract the text of the guard element named @p guard_name, matching the
/// close tag at the guard's own indentation so nested Fallbacks don't end
/// the span early.
std::string ExtractGuardBlock(const std::string& xml, const std::string& guard_name)
{
  std::istringstream in(xml);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(in, line))
  {
    lines.push_back(line);
  }

  const std::regex open_re("<(ReactiveFallback|Fallback) name=\"" + guard_name + "\"");
  for (std::size_t i = 0; i < lines.size(); ++i)
  {
    std::smatch m;
    if (!std::regex_search(lines[i], m, open_re))
    {
      continue;
    }
    const std::string close = "</" + m[1].str() + ">";
    const std::size_t indent = lines[i].find_first_not_of(" \t");
    std::string block;
    for (std::size_t j = i; j < lines.size(); ++j)
    {
      block += lines[j] + "\n";
      const std::size_t j_indent = lines[j].find_first_not_of(" \t");
      if (j > i && j_indent == indent && lines[j].substr(j_indent) == close)
      {
        return block;
      }
    }
  }
  return {};
}

// Every guard that must BLOCK MainLogic while its fault holds. Nav2ResumeGuard
// is deliberately always-SUCCESS (it is a pass-through that resumes the Nav2
// lifecycle) and RecordingCommandGuard is a pure condition, so neither is
// listed here.
TEST(GuardFallthroughTest, AllBlockingGuardsTerminateWithFailure)
{
  const std::string xml = ReadMainTree();
  ASSERT_FALSE(xml.empty());

  for (const std::string guard : {"EmergencyGuard",
                                  "SensorSafetyGuard",
                                  "BoundaryGuard",
                                  "LocalizationGuard",
                                  "DigObstructionGuard",
                                  "RainGuard",
                                  "BatteryGuard"})
  {
    const std::string block = ExtractGuardBlock(xml, guard);
    ASSERT_FALSE(block.empty()) << "Guard not found in main_tree.xml: " << guard;
    EXPECT_NE(block.find("<AlwaysFailure/>"), std::string::npos)
        << guard
        << " has no <AlwaysFailure/> terminator. A blocking guard whose "
           "handler can return SUCCESS lets the Root ReactiveSequence advance "
           "into MainLogic for one tick per cycle, restarting MowingSequence "
           "from PREFLIGHT_CHECK/UNDOCKING (issues #459, #445).";
  }
}

// The guards whose halt interrupts a mowing PASS (as opposed to ending the
// session — emergency, boundary, rain, battery, dig obstruction) must tick
// <MarkGuardHalt/> as the FIRST child of their handler Sequence, or every
// pause is charged to GetNextUnmowedArea's per-area no-progress budget and a
// flapping sensor fails the mow at 0 swaths (field 2026-09-07 / 2026-09-08:
// three IsScanStale halts in 25 s exhausted the five attempts). First, not
// merely before <AlwaysFailure/>: the ReactiveFallback halts the handler the
// moment the fault clears, so a marker behind StopMoving + WaitForDuration
// would miss every pause shorter than their combined duration.
TEST(GuardFallthroughTest, PausingGuardsMarkTheHaltFirst)
{
  const std::string xml = ReadMainTree();
  ASSERT_FALSE(xml.empty());

  for (const auto& [guard, reason] : std::vector<std::pair<std::string, std::string>>{
           {"SensorSafetyGuard", "scan_stale"}, {"LocalizationGuard", "localization_degraded"}})
  {
    const std::string block = ExtractGuardBlock(xml, guard);
    ASSERT_FALSE(block.empty()) << "Guard not found in main_tree.xml: " << guard;
    const std::size_t mark = block.find("<MarkGuardHalt reason=\"" + reason + "\"/>");
    ASSERT_NE(mark, std::string::npos)
        << guard << " does not tick <MarkGuardHalt reason=\"" << reason
        << "\"/> — its pauses would be charged to the no-progress budget and retire the area.";
    // Nothing that can return RUNNING or FAILURE may precede the marker: the
    // handler's other children (SetMowerEnabled, StopMoving, WaitForDuration,
    // PublishHighLevelStatus) must all come after it.
    const std::size_t handler = block.find("Handler\">");
    ASSERT_NE(handler, std::string::npos) << guard << ": no *Handler Sequence found";
    ASSERT_LT(handler, mark) << guard << ": <MarkGuardHalt/> is outside the handler Sequence";
    const std::string between = block.substr(handler, mark - handler);
    for (const std::string preceding :
         {"<SetMowerEnabled", "<StopMoving", "<WaitForDuration", "<PublishHighLevelStatus"})
    {
      EXPECT_EQ(between.find(preceding), std::string::npos)
          << guard << ": " << preceding << " runs before <MarkGuardHalt/> — a fault that clears "
          << "while it is RUNNING (or a service call that FAILS) skips the marker and the "
          << "interrupted pass is charged after all.";
    }
  }
}

// ---------------------------------------------------------------------------
// DigObstructionGuard (issue #500) — the newest blocking guard
// ---------------------------------------------------------------------------

// Tick contract of the condition node itself: it is a pure mirror of the
// latched /hardware_bridge/dig_escalated flag the bridge publishes.
TEST(DigObstructionGuardTest, ConditionMirrorsTheLatchedFlag)
{
  auto ctx = MakeContext("test_dig_escalated_condition");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("context", ctx);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<IsDigEscalated>("IsDigEscalated");
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><IsDigEscalated/></BehaviorTree>)"
      R"(</root>)",
      blackboard);

  ctx->dig_escalated = false;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);

  ctx->dig_escalated = true;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

// The guard must stop the MISSION without ever blocking the lanes the
// operator needs to recover a wedged robot — a guard that also blocked HOME
// or teleop would strand the robot exactly where it is already stuck.
TEST(DigObstructionGuardTest, ExemptsEveryOperatorRecoveryLane)
{
  const std::string xml = ReadMainTree();
  ASSERT_FALSE(xml.empty());

  const std::string block = ExtractGuardBlock(xml, "DigObstructionGuard");
  ASSERT_FALSE(block.empty()) << "DigObstructionGuard missing from main_tree.xml";

  EXPECT_NE(block.find("<IsDigEscalated/>"), std::string::npos)
      << "DigObstructionGuard no longer keys on IsDigEscalated.";

  // charging (latch already cleared), idle (no mission), HOME, and the
  // manual/recording modes.
  for (const std::string& exempt : {std::string("<IsCharging/>"),
                                    std::string("command=\"0\""),
                                    std::string("command=\"2\""),
                                    std::string("command=\"3\""),
                                    std::string("command=\"5\""),
                                    std::string("command=\"6\""),
                                    std::string("command=\"7\"")})
  {
    EXPECT_NE(block.find(exempt), std::string::npos)
        << "DigObstructionGuard stopped exempting " << exempt
        << " — the operator must always be able to recall or drive a wedged "
           "robot out (issue #500).";
  }
}

}  // namespace
