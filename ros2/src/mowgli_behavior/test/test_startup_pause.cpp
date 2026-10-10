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

#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/condition_nodes.hpp"
#include <gtest/gtest.h>
#include <tinyxml2.h>

namespace
{

struct Stage
{
  const char* tag;
  int occurrence;
  bool charging;
  bool nav2_ready;
  bool preflight_ready;
};

struct Observations
{
  std::shared_ptr<mowgli_behavior::BTContext> context;
  Stage stage;
  bool release = false;
  int held_starts = 0;
  int held_ticks = 0;
  int held_halts = 0;
  int stop_starts = 0;
  int home_starts = 0;
  /// Per leaf tag, and per "tag#occurrence" for leaves that appear twice.
  std::map<std::string, int> starts;
  /// "tag#occurrence" leaves that report FAILURE.
  std::set<std::string> failing;
};

// External services, clocks and motion are controlled here. The production
// control-node structure and command condition are executed without copying
// the startup sequence into the test.
class Probe : public BT::StatefulActionNode
{
public:
  Probe(const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<Observations> observations,
        std::string tag,
        int occurrence)
      : BT::StatefulActionNode(name, config),
        observations_(std::move(observations)),
        tag_(std::move(tag)),
        occurrence_(occurrence)
  {
  }

  BT::NodeStatus onStart() override
  {
    ++observations_->starts[tag_];
    ++observations_->starts[key()];
    if (held())
      ++observations_->held_starts;
    if (tag_ == "StopMoving" && observations_->context->current_command == 8)
      ++observations_->stop_starts;
    return step();
  }

  BT::NodeStatus onRunning() override
  {
    if (held())
      ++observations_->held_ticks;
    return step();
  }

  void onHalted() override
  {
    if (held())
      ++observations_->held_halts;
  }

private:
  std::string key() const
  {
    return tag_ + "#" + std::to_string(occurrence_);
  }

  bool held() const
  {
    return tag_ == observations_->stage.tag && occurrence_ == observations_->stage.occurrence;
  }

  BT::NodeStatus step()
  {
    using Status = BT::NodeStatus;
    if (held() && !observations_->release)
      return Status::RUNNING;
    if (tag_ == "StopMoving" && observations_->context->current_command == 8)
      return Status::RUNNING;
    if (observations_->failing.count(key()))
      return Status::FAILURE;
    // The undock bookkeeping the resume branch keys on, as the real nodes do.
    if (tag_ == "RecordUndockStart")
      observations_->context->undock_start_recorded = true;
    if (tag_ == "CalibrateHeadingFromUndock")
      observations_->context->undock_start_recorded = false;
    if (tag_ == "IsUndockInterrupted")
      return observations_->context->undock_start_recorded ? Status::SUCCESS : Status::FAILURE;
    if (tag_ == "IsCharging")
    {
      // The mowing contact debounce is irrelevant to startup and must not
      // engage a charge hold merely because this fixture started on the dock.
      if (config().input_ports.count("stable_for_sec"))
        return Status::FAILURE;
      return observations_->stage.charging ? Status::SUCCESS : Status::FAILURE;
    }
    if (tag_ == "Nav2Active")
      return observations_->stage.nav2_ready ? Status::SUCCESS : Status::FAILURE;
    if (tag_ == "PreFlightCheck")
      return observations_->stage.preflight_ready ? Status::SUCCESS : Status::FAILURE;
    if (tag_ == "ClearCommand")
      observations_->context->current_command = 0;
    if (tag_ == "EndSession")
      observations_->context->area_resume_pose_index.clear();
    if (tag_ == "GetNextUnmowedArea")
      return Status::RUNNING;
    static const std::set<std::string> inactive_guards = {"IsNewRain",
                                                          "IsBatteryLow",
                                                          "IsObstacleStuck",
                                                          "IsCoverageStartBlocked",
                                                          "WasRecentlyInCollisionStop",
                                                          "IsManualResumeRequested",
                                                          "IsRainModeAtLeast"};
    return inactive_guards.count(tag_) ? Status::FAILURE : Status::SUCCESS;
  }

  std::shared_ptr<Observations> observations_;
  std::string tag_;
  int occurrence_;
};

tinyxml2::XMLElement* namedElement(tinyxml2::XMLElement* element, const char* name)
{
  if (!element)
    return nullptr;
  const auto* current_name = element->Attribute("name");
  if (current_name && std::string(current_name) == name)
    return element;
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
    if (auto* found = namedElement(child, name))
      return found;
  return nullptr;
}

class StartupPauseTest : public ::testing::TestWithParam<Stage>
{
protected:
  std::shared_ptr<Observations> observations;
  BT::Blackboard::Ptr blackboard;
  BT::BehaviorTreeFactory factory;
  std::map<std::string, int> occurrences;

  void SetUp() override
  {
    observations = std::make_shared<Observations>();
    observations->stage = GetParam();
    observations->context = std::make_shared<mowgli_behavior::BTContext>();
    observations->context->current_command = 1;
    observations->context->area_resume_pose_index[0] = 42;
    blackboard = BT::Blackboard::create();
    blackboard->set("context", observations->context);
    factory.registerNodeType<mowgli_behavior::IsCommand>("IsCommand");
    factory.registerSimpleAction("HomeProbe",
                                 [this](BT::TreeNode&)
                                 {
                                   ++observations->home_starts;
                                   return BT::NodeStatus::SUCCESS;
                                 });
  }

  void registerProbes(tinyxml2::XMLElement* element)
  {
    if (!element->FirstChildElement())
    {
      const std::string tag = element->Name();
      if (tag == "IsCommand" || tag == "AlwaysSuccess" || tag == "AlwaysFailure")
        return;
      // Unique registration IDs give each leaf its static production-tree
      // occurrence, so two identical waits remain independently controllable.
      const int occurrence = ++occurrences[tag];
      const std::string id = tag + "Probe" + std::to_string(occurrence);
      BT::PortsList ports;
      for (auto* attribute = element->FirstAttribute(); attribute; attribute = attribute->Next())
        if (std::string(attribute->Name()) != "name")
          ports.emplace(BT::InputPort<std::string>(attribute->Name()));
      BT::TreeNodeManifest manifest{BT::NodeType::ACTION, id, ports, {}};
      factory.registerBuilder(manifest,
                              [state = observations, tag, occurrence](const std::string& name,
                                                                      const BT::NodeConfig& config)
                              {
                                return std::make_unique<Probe>(
                                    name, config, state, tag, occurrence);
                              });
      element->SetName(id.c_str());
      return;
    }
    for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement())
      registerProbes(child);
  }

  BT::Tree makeTree()
  {
    tinyxml2::XMLDocument production;
    if (production.LoadFile(MOWGLI_MAIN_TREE_PATH) != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("Cannot load production main_tree.xml");
    auto* mowing = namedElement(production.RootElement(), "MowingSequence");
    auto* stop = namedElement(production.RootElement(), "StopHoldSequence");
    if (!mowing || !stop)
      throw std::runtime_error("Production mowing/stop subtree is missing");
    tinyxml2::XMLDocument test;
    test.Parse(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><Fallback name=\"MainLogic\"/>"
        "</BehaviorTree></root>");
    auto* fallback = test.RootElement()->FirstChildElement()->FirstChildElement();
    fallback->InsertEndChild(mowing->DeepClone(&test));
    fallback->InsertEndChild(stop->DeepClone(&test));
    auto* home = test.NewElement("ReactiveSequence");
    auto* command = test.NewElement("IsCommand");
    command->SetAttribute("command", "2");
    home->InsertEndChild(command);
    home->InsertEndChild(test.NewElement("HomeProbe"));
    fallback->InsertEndChild(home);
    registerProbes(fallback->FirstChildElement());
    registerProbes(fallback->FirstChildElement()->NextSiblingElement());
    tinyxml2::XMLPrinter printer;
    test.Print(&printer);
    return factory.createTreeFromText(printer.CStr(), blackboard);
  }
};

TEST_P(StartupPauseTest, StopHaltsStartupAndNeverReachesItsFailureOrMotionTail)
{
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(observations->held_starts, 1);
  auto startup_starts = observations->starts;
  observations->context->current_command = 8;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observations->held_halts, 1);
  EXPECT_EQ(observations->held_ticks, 0);
  EXPECT_EQ(observations->stop_starts, 1);
  EXPECT_EQ(observations->context->current_command, 8);
  EXPECT_EQ(observations->starts["ClearCommand"], 0);
  EXPECT_EQ(observations->starts["EndSession"], 0);
  EXPECT_EQ(observations->context->area_resume_pose_index.at(0), 42u);
  observations->release = true;
  for (int tick = 0; tick < 3; ++tick)
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observations->held_ticks, 0);
  EXPECT_EQ(observations->held_starts, 1);
  EXPECT_EQ(observations->home_starts, 0);
  EXPECT_EQ(observations->starts["ClearCommand"], 0);
  for (const auto* tag : {"BackUp",
                          "SeedYawFromMotion",
                          "CalibrateHeadingFromUndock",
                          "RecordUndockStart",
                          "ClearCostmap",
                          "GetNextUnmowedArea"})
    EXPECT_EQ(observations->starts[tag], startup_starts[tag]) << tag;
}

TEST_P(StartupPauseTest, StartAfterPauseReentersPrerequisitesAndPreservesResume)
{
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(observations->held_starts, 1);
  observations->context->current_command = 8;
  tree.tickOnce();
  const int readiness_checks = observations->starts["Nav2Active"];
  observations->context->current_command = 1;
  // MainLogic's memory Fallback first deselects the running stop branch.
  // It re-enters mowing from the first child on the following root tick.
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observations->held_starts, 2);
  EXPECT_EQ(observations->starts["Nav2Active"], readiness_checks + 1);
  EXPECT_EQ(observations->context->current_command, 1);
  EXPECT_EQ(observations->context->area_resume_pose_index.at(0), 42u);
}

TEST_P(StartupPauseTest, HomeHaltsStartupWithoutClearingTheNewCommand)
{
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(observations->held_starts, 1);
  observations->context->current_command = 2;
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(observations->held_halts, 1);
  EXPECT_EQ(observations->home_starts, 1);
  EXPECT_EQ(observations->context->current_command, 2);
  EXPECT_EQ(observations->starts["ClearCommand"], 0);
  EXPECT_EQ(observations->starts["EndSession"], 0);
}

TEST_P(StartupPauseTest, ContinuedStartDoesNotReplaySuccessfulStartupSteps)
{
  auto tree = makeTree();
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(observations->held_starts, 1);
  const auto starts = observations->starts;
  // The charging bit drops during undock; reevaluating earlier startup
  // branches could otherwise replace the active reverse with a forward seed.
  observations->stage.charging = false;
  for (int tick = 0; tick < 3; ++tick)
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(observations->held_starts, 1);
  EXPECT_EQ(observations->held_halts, 0);
  EXPECT_EQ(observations->held_ticks, 3);
  EXPECT_EQ(observations->starts, starts);
}

// Pause during the undock reverse, after the robot has left the contacts, then
// Play: the robot still faces the dock, so the tree must finish the reverse —
// never run SeedYawFromMotion's 1 m forward drive back onto the dock.
class InterruptedUndockTest : public StartupPauseTest
{
protected:
  BT::Tree pauseMidUndockThenPlay()
  {
    auto tree = makeTree();
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(observations->held_starts, 1);
    EXPECT_TRUE(observations->context->undock_start_recorded);
    observations->context->current_command = 8;
    tree.tickOnce();
    EXPECT_EQ(observations->held_halts, 1);
    observations->stage.charging = false;
    observations->context->current_command = 1;
    // MainLogic first deselects the stop branch, then re-enters mowing.
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
    tree.tickOnce();
    return tree;
  }
};

TEST_P(InterruptedUndockTest, PlayFinishesTheReverseInsteadOfDrivingForward)
{
  auto tree = pauseMidUndockThenPlay();
  EXPECT_EQ(observations->starts["SeedYawFromMotion"], 0);
  EXPECT_EQ(observations->starts["BackUp#1"], 1) << "the dock-side reverse is not restarted";
  EXPECT_EQ(observations->starts["BackUp#2"], 1) << "ResumeInterruptedUndock's reverse";
  EXPECT_EQ(observations->starts["CalibrateHeadingFromUndock#2"], 1);
  EXPECT_FALSE(observations->context->undock_start_recorded);
  EXPECT_EQ(observations->starts["ClearCommand"], 0);
  EXPECT_EQ(observations->starts["GetNextUnmowedArea"], 1);
}

TEST_P(InterruptedUndockTest, NothingLeftToReverseStillCalibratesWithoutMoving)
{
  observations->failing.insert("RemainingUndockDistance#1");
  auto tree = pauseMidUndockThenPlay();
  EXPECT_EQ(observations->starts["SeedYawFromMotion"], 0);
  EXPECT_EQ(observations->starts["BackUp#2"], 0);
  EXPECT_EQ(observations->starts["CalibrateHeadingFromUndock#2"], 1);
  EXPECT_EQ(observations->starts["GetNextUnmowedArea"], 1);
}

TEST_P(InterruptedUndockTest, AFailedResumeEndsInUndockFailedNotTheForwardSeed)
{
  observations->failing.insert("BackUp#2");
  auto tree = pauseMidUndockThenPlay();
  EXPECT_EQ(observations->starts["BackUp#2"], 1);
  EXPECT_EQ(observations->starts["SeedYawFromMotion"], 0);
  EXPECT_EQ(observations->starts["GetNextUnmowedArea"], 0);
  EXPECT_GE(observations->starts["ClearCommand"], 1);
  EXPECT_EQ(observations->context->current_command, 0);
}

INSTANTIATE_TEST_SUITE_P(UndockReverse,
                         InterruptedUndockTest,
                         ::testing::Values(Stage{"BackUp", 1, true, true, true}));

INSTANTIATE_TEST_SUITE_P(ProductionStartupStages,
                         StartupPauseTest,
                         ::testing::Values(Stage{"WaitForDuration", 1, true, false, true},
                                           Stage{"WaitForDuration", 2, true, true, true},
                                           Stage{"WaitForDuration", 3, true, true, false},
                                           Stage{"BackUp", 1, true, true, true},
                                           Stage{"WaitForGpsFix", 1, true, true, true},
                                           Stage{"SeedYawFromMotion", 1, false, true, true},
                                           Stage{"ClearCostmap", 3, true, true, true},
                                           Stage{"WaitForDuration", 4, true, true, true}));

}  // namespace
