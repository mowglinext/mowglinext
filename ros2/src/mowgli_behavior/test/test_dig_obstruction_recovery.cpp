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
 * Both causes are gone at the root: a dig no longer stamps a keepout at all
 * (map_server records an inert proposal), and the bridge clears the latch on
 * displacement. The tree-side contract pinned here: HOME bypasses the hold
 * and needs no keepout clean-up step, and the hold itself keeps publishing
 * state=1 (the firmware hard stop is what keeps a wedged robot from grinding
 * on).
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

// HOME out of a DIG_OBSTRUCTION hold needs no keepout clean-up any more: a dig
// is only a PROPOSAL in map_server and is never stamped into the keepout mask,
// so the dock transit can always plan from the robot's pose. The clean-up node
// and its map_server service were removed; nothing may call them again (an XML
// tag with no registered node makes the whole tree fail to load).
TEST(DigObstructionRecovery, HomeNeedsNoDigKeepoutCleanUpBeforeDocking)
{
  const std::string tree = readMainTree();
  EXPECT_EQ(tree.find("<DiscardNearbyDigKeepouts"), std::string::npos)
      << "DiscardNearbyDigKeepouts was removed together with the dig keepouts it dropped";

  const auto home = tree.find("name=\"HomeSequence\"");
  ASSERT_NE(home, std::string::npos) << "HomeSequence not found";
  const auto dock = tree.find("<DockRobot", home);
  ASSERT_NE(dock, std::string::npos) << "HomeSequence must still dock";
}

// HOME and the manual modes are how the operator recovers a held robot: the
// guard must let them through, or the hold has no exit but lifting the robot.
TEST(DigObstructionRecovery, GuardExemptsHomeSoTheOperatorCanRecallTheRobot)
{
  const std::string tree = readMainTree();
  const auto guard = tree.find("name=\"DigObstructionGuard\"");
  ASSERT_NE(guard, std::string::npos) << "DigObstructionGuard not found";
  const auto gate = tree.find("<IsDigEscalated/>", guard);
  ASSERT_NE(gate, std::string::npos) << "the guard must still read the escalation latch";
  const std::string exemptions = tree.substr(guard, gate - guard);
  EXPECT_NE(exemptions.find("<IsCommand command=\"2\"/>"), std::string::npos)
      << "COMMAND_HOME must bypass the DIG_OBSTRUCTION hold";
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
