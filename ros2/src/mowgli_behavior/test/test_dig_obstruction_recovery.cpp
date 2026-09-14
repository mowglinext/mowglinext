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
 * @file test_dig_obstruction_recovery.cpp
 * @brief Structural regression test for the DIG_OBSTRUCTION exits.
 *
 * Field report 2026-09-14: a robot in DIG_OBSTRUCTION could not be recovered.
 * HOME never moved (the dock transit planned from inside the pending dig
 * keepouts stamped under the robot) and, once lifted clear, Play did nothing
 * (the escalation latch only cleared at the charger).
 *
 * The tree-side contract pinned here: the HOME branch must drop the dig
 * keepouts under the robot, gated on IsDigEscalated, BEFORE its DockRobot,
 * and the hold itself must keep publishing state=1 (the firmware hard stop
 * is what keeps a wedged robot from grinding on).
 */
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace
{
std::string readMainTree()
{
  std::ifstream file(MOWGLI_MAIN_TREE_PATH);
  EXPECT_TRUE(file.is_open()) << "cannot open " << MOWGLI_MAIN_TREE_PATH;
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

TEST(DigObstructionRecovery, HomeDropsTheDigKeepoutsUnderTheRobotBeforeDocking)
{
  const std::string tree = readMainTree();
  const auto home = tree.find("name=\"HomeSequence\"");
  ASSERT_NE(home, std::string::npos) << "HomeSequence not found";
  const auto dock = tree.find("<DockRobot", home);
  ASSERT_NE(dock, std::string::npos) << "HomeSequence has no DockRobot";
  const std::string between = tree.substr(home, dock - home);

  const auto gate = between.find("<IsDigEscalated/>");
  const auto discard = between.find("<DiscardNearbyDigKeepouts/>");
  EXPECT_NE(gate, std::string::npos)
      << "HomeSequence must gate the keepout discard on IsDigEscalated";
  EXPECT_NE(discard, std::string::npos)
      << "HomeSequence must call DiscardNearbyDigKeepouts before DockRobot";
  EXPECT_LT(gate, discard) << "the gate must precede the discard";
  // The pair must be wrapped so a non-escalated robot goes home unchanged.
  const auto force = between.rfind("<ForceSuccess>", gate);
  EXPECT_NE(force, std::string::npos) << "FreeDigKeepoutsIfEscalated must sit under ForceSuccess";
}

TEST(DigObstructionRecovery, HoldKeepsPublishingIdleSoTheFirmwareHardStops)
{
  const std::string tree = readMainTree();
  const std::regex hold_re(
      R"RE(<PublishHighLevelStatus\s+state="(\d+)"\s+state_name="DIG_OBSTRUCTION")RE");
  std::smatch match;
  ASSERT_TRUE(std::regex_search(tree, match, hold_re)) << "DIG_OBSTRUCTION publish not found";
  EXPECT_EQ(match[1].str(), "1")
      << "the hold relies on HL_MODE_IDLE's wheel+blade hard stop in firmware";
}
