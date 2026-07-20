// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// test_robot_yaml_scalar.cpp
//
// Unit tests for the in-place dock-pose splice used to PERSIST the unified
// one-click dock calibration result. The calibration action derives x/y from
// averaged RTK-Fixed GPS and yaw from the COG-monitored reverse leg, then
// routes the write through map_server's on_set_docking_point (yaw_source=
// MOTION), which persists via robot_yaml_scalar::UpdateDockPose — the ONE
// canonical dock_pose writer (CLAUDE.md Invariant 6 collapse).
//
// mowgli_robot.yaml is a hand-maintained, heavily-commented SPARSE config
// (Invariant 15). The persist MUST:
//   * replace only the numeric values of dock_pose_x / _y / _yaw,
//   * preserve every comment and all surrounding structure verbatim,
//   * not touch a lookalike key (dock_pose_yaw_sigma_rad),
//   * write the file exactly once (atomic tmp+rename, no leftover .tmp,
//     no duplicated keys).

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "mowgli_interfaces/robot_yaml_scalar.hpp"
#include <gtest/gtest.h>

namespace
{
using mowgli_interfaces::robot_yaml_scalar::UpdateDockPose;

// A representative slice of the real sparse, commented config.
const char* kSampleYaml =
    "# Mowgli robot config (sparse install file over the template).\n"
    "mowgli_robot:\n"
    "  ros__parameters:\n"
    "    # Dock pose — single source of truth (Invariant 6).\n"
    "    dock_pose_x: 1.234567     # ENU east (m)\n"
    "    dock_pose_y: -7.654321    # ENU north (m)\n"
    "    dock_pose_yaw: 0.100000   # chassis heading (rad)\n"
    "    dock_pose_yaw_sigma_rad: 0.035  # lookalike key — must NOT be spliced\n"
    "    # Unrelated calibration output below.\n"
    "    ticks_per_meter: 305.0\n";

std::string read_all(const std::string& path)
{
  std::ifstream in(path);
  std::stringstream buf;
  buf << in.rdbuf();
  return buf.str();
}

std::string write_temp(const std::string& name, const std::string& content)
{
  const std::string path = (std::filesystem::path(::testing::TempDir()) / name).string();
  std::ofstream out(path, std::ios::trunc);
  out << content;
  out.close();
  return path;
}

// Count occurrences of a substring — guards against a duplicated key.
int count_occurrences(const std::string& hay, const std::string& needle)
{
  int n = 0;
  size_t pos = 0;
  while ((pos = hay.find(needle, pos)) != std::string::npos)
  {
    ++n;
    pos += needle.size();
  }
  return n;
}
}  // namespace

TEST(RobotYamlScalar, MotionPersistUpdatesAllThreeValues)
{
  // Arrange
  const std::string path = write_temp("motion_persist_values.yaml", kSampleYaml);

  // Act — a motion-derived dock pose (averaged GPS x/y + COG yaw).
  ASSERT_TRUE(UpdateDockPose(path, 12.5, 34.75, -1.5708));
  const std::string out = read_all(path);

  // Assert — the three values are the new ones at 6-decimal precision.
  EXPECT_NE(out.find("dock_pose_x: 12.500000"), std::string::npos) << out;
  EXPECT_NE(out.find("dock_pose_y: 34.750000"), std::string::npos) << out;
  EXPECT_NE(out.find("dock_pose_yaw: -1.570800"), std::string::npos) << out;
  // Old values are gone.
  EXPECT_EQ(out.find("1.234567"), std::string::npos) << out;
  EXPECT_EQ(out.find("-7.654321"), std::string::npos) << out;
}

TEST(RobotYamlScalar, MotionPersistPreservesCommentsAndStructure)
{
  // Arrange
  const std::string path = write_temp("motion_persist_comments.yaml", kSampleYaml);

  // Act
  ASSERT_TRUE(UpdateDockPose(path, 12.5, 34.75, -1.5708));
  const std::string out = read_all(path);

  // Assert — every comment and unrelated line survives verbatim.
  EXPECT_NE(out.find("# Mowgli robot config (sparse install file over the template)."),
            std::string::npos);
  EXPECT_NE(out.find("# Dock pose — single source of truth (Invariant 6)."), std::string::npos);
  EXPECT_NE(out.find("# ENU east (m)"), std::string::npos);
  EXPECT_NE(out.find("# chassis heading (rad)"), std::string::npos);
  EXPECT_NE(out.find("# Unrelated calibration output below."), std::string::npos);
  EXPECT_NE(out.find("ticks_per_meter: 305.0"), std::string::npos);
  // Structure: same number of lines in and out.
  EXPECT_EQ(count_occurrences(out, "\n"), count_occurrences(kSampleYaml, "\n"));
}

TEST(RobotYamlScalar, MotionPersistDoesNotTouchLookalikeKey)
{
  // Arrange
  const std::string path = write_temp("motion_persist_lookalike.yaml", kSampleYaml);

  // Act
  ASSERT_TRUE(UpdateDockPose(path, 12.5, 34.75, -1.5708));
  const std::string out = read_all(path);

  // Assert — dock_pose_yaw_sigma_rad is anchored on its own indent and is a
  // different key; its value must be untouched.
  EXPECT_NE(out.find("dock_pose_yaw_sigma_rad: 0.035"), std::string::npos) << out;
}

TEST(RobotYamlScalar, MotionPersistWritesOnceNoLeftoverTmpNoDuplicateKeys)
{
  // Arrange
  const std::string path = write_temp("motion_persist_atomic.yaml", kSampleYaml);

  // Act
  ASSERT_TRUE(UpdateDockPose(path, 12.5, 34.75, -1.5708));
  const std::string out = read_all(path);

  // Assert — each key appears exactly once (no append-duplication), and the
  // atomic tmp file was renamed away (no .tmp residue).
  EXPECT_EQ(count_occurrences(out, "dock_pose_x:"), 1);
  EXPECT_EQ(count_occurrences(out, "dock_pose_y:"), 1);
  EXPECT_EQ(count_occurrences(out, "dock_pose_yaw:"), 1);
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));
}

TEST(RobotYamlScalar, ReturnsFalseWhenFileMissing)
{
  // A non-existent path must fail cleanly (caller keeps the in-memory pose).
  const std::string path =
      (std::filesystem::path(::testing::TempDir()) / "does_not_exist_dock.yaml").string();
  std::filesystem::remove(path);
  EXPECT_FALSE(UpdateDockPose(path, 1.0, 2.0, 3.0));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
