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
//
// Structural pin for trees/navigate_to_pose_transit.xml (field 2026-09-12):
// coverage transits must select transit_goal_checker (final heading ignored)
// while the default navigate_to_pose.xml keeps stopped_goal_checker for
// opennav_docking's staging approach. The two files must otherwise match.
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace
{
std::string readFile(const char* path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}
std::string stripHeaderComment(std::string s)
{
  const auto end = s.find("-->");
  return end == std::string::npos ? s : s.substr(end + 3);
}
}  // namespace

TEST(TransitTree, SelectsTheTransitGoalChecker)
{
  const std::string transit = readFile(MOWGLI_TRANSIT_TREE_PATH);
  ASSERT_FALSE(transit.empty()) << MOWGLI_TRANSIT_TREE_PATH;
  EXPECT_NE(transit.find("default_goal_checker=\"transit_goal_checker\""), std::string::npos);
  EXPECT_EQ(transit.find("default_goal_checker=\"stopped_goal_checker\""), std::string::npos);
  EXPECT_NE(transit.find("goal_checker_id=\"{selected_goal_checker}\""), std::string::npos);
}

TEST(TransitTree, DefaultTreeStillUsesStoppedGoalCheckerForDocking)
{
  const std::string base = readFile(MOWGLI_NAV_TREE_PATH);
  ASSERT_FALSE(base.empty()) << MOWGLI_NAV_TREE_PATH;
  EXPECT_NE(base.find("default_goal_checker=\"stopped_goal_checker\""), std::string::npos);
}

TEST(TransitTree, OnlyTheGoalCheckerDiffers)
{
  std::string base = stripHeaderComment(readFile(MOWGLI_NAV_TREE_PATH));
  std::string transit = stripHeaderComment(readFile(MOWGLI_TRANSIT_TREE_PATH));
  base = std::regex_replace(base, std::regex("default_goal_checker=\"[a-z_]+\""), "GC");
  transit = std::regex_replace(transit, std::regex("default_goal_checker=\"[a-z_]+\""), "GC");
  EXPECT_EQ(base, transit) << "navigate_to_pose_transit.xml drifted from navigate_to_pose.xml";
}

// The recovery RoundRobin must try BackUp BEFORE the other actions. A transit
// fails when the robot's own footprint is over a lethal cell (RPP checks the
// current pose first and then refuses to move), and backing up is the only
// action that changes that; field 2026-09-18 the watchdog cancelled the transit
// before the stock order ever reached it.
TEST(TransitTree, RecoveryBacksUpBeforeAnythingElse)
{
  const std::string xml = stripHeaderComment(readFile(MOWGLI_TRANSIT_TREE_PATH));
  const auto round_robin = xml.find("RecoveryActions");
  ASSERT_NE(round_robin, std::string::npos) << "no recovery RoundRobin in the transit tree";

  const auto backup = xml.find("<BackUp", round_robin);
  const auto clearing = xml.find("ClearingActions", round_robin);
  const auto spin = xml.find("<Spin", round_robin);
  const auto wait = xml.find("<Wait", round_robin);
  ASSERT_NE(backup, std::string::npos);
  ASSERT_NE(clearing, std::string::npos);
  ASSERT_NE(spin, std::string::npos);
  ASSERT_NE(wait, std::string::npos);

  EXPECT_LT(backup, clearing) << "BackUp must come before clearing the costmaps";
  EXPECT_LT(backup, spin) << "BackUp must come before Spin";
  EXPECT_LT(backup, wait) << "BackUp must come before Wait";
}

// Both trees must validate the path as a POINT, the model SmacPlanner2D plans
// with. Nav2 Lyrical's ValidatePath defaults to the robot footprint, which made
// nearly every fresh transit plan near the boundary or a drawn obstacle
// "invalid" and replanned it every second (field 2026-09-22: 118 of 124).
TEST(TransitTree, ValidatesThePathAsAPointLikeSmac)
{
  for (const char* path : {MOWGLI_NAV_TREE_PATH, MOWGLI_TRANSIT_TREE_PATH})
  {
    const std::string xml = stripHeaderComment(readFile(path));
    std::smatch m;
    ASSERT_TRUE(std::regex_search(xml, m, std::regex("<ValidatePath[^>]*/>"))) << path;
    const std::string node = m.str();
    std::smatch fp;
    ASSERT_TRUE(std::regex_search(node, fp, std::regex("footprint=\"([^\"]+)\"")))
        << path << ": ValidatePath has no footprint — it falls back to the robot footprint";
    const std::string poly = fp[1].str();
    const std::regex number("-?[0-9]*\\.?[0-9]+");
    int count = 0;
    for (auto it = std::sregex_iterator(poly.begin(), poly.end(), number);
         it != std::sregex_iterator();
         ++it, ++count)
    {
      EXPECT_LE(std::abs(std::stod(it->str())), 0.05)
          << path << ": footprint " << poly << " is not a point";
    }
    EXPECT_GE(count, 6) << path << ": footprint " << poly << " is not a polygon";
    EXPECT_EQ(node.find("max_cost"), std::string::npos)
        << path << ": keep the default max_cost (lethal only), Jazzy's point check";
  }
}
