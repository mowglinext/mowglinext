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
