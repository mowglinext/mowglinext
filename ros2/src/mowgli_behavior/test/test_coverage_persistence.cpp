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

// Unit coverage for the disk-backed coverage resume state (issue #334): an
// interrupted mow must survive a full process/container restart, so the
// per-area resume cursor + completed-swath sets round-trip through a file and a
// changed map is detected as stale.

#include <cstdio>
#include <fstream>
#include <string>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;

namespace
{
std::string tempPath(const std::string& name)
{
  return std::string(::testing::TempDir()) + "/" + name;
}

// Seed a context with a representative mid-session resume state. BTContext holds
// a std::mutex (non-copyable/non-movable), so it is populated in place rather
// than returned by value.
void seedContext(BTContext& ctx, const std::string& path)
{
  ctx.coverage_resume_path = path;
  ctx.current_area = 2;
  ctx.area_path_pose_count[0] = 1000;
  ctx.area_path_pose_count[2] = 3500;
  ctx.area_plan_fingerprint[0] = 0x0123456789abcdefULL;
  ctx.area_plan_fingerprint[2] = 0xfedcba9876543210ULL;  // staleness key
  ctx.area_resume_pose_index[2] = 1234;  // area 2 interrupted mid-path
  ctx.area_completed_swaths[0] = {0};  // area 0 fully done
  ctx.area_completed_swaths[2] = {};  // present but empty
  ctx.completed_areas = {0};
}
}  // namespace

TEST(CoveragePersistence, RoundTripsAllResumeState)
{
  const std::string path = tempPath("coverage_resume_roundtrip.txt");
  std::remove(path.c_str());

  BTContext saved;
  seedContext(saved, path);
  ASSERT_TRUE(saveCoverageResumeState(saved));

  BTContext loaded;
  loaded.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));

  EXPECT_EQ(loaded.current_area, 2);
  EXPECT_EQ(loaded.area_path_pose_count[0], 1000u);
  EXPECT_EQ(loaded.area_path_pose_count[2], 3500u);
  // Plan-geometry fingerprints (the staleness key) round-trip exactly.
  EXPECT_EQ(loaded.area_plan_fingerprint[0], 0x0123456789abcdefULL);
  EXPECT_EQ(loaded.area_plan_fingerprint[2], 0xfedcba9876543210ULL);
  ASSERT_EQ(loaded.area_resume_pose_index.count(2), 1u);
  EXPECT_EQ(loaded.area_resume_pose_index[2], 1234u);
  // area 0 has no live cursor (fully mowed) → must NOT be resurrected.
  EXPECT_EQ(loaded.area_resume_pose_index.count(0), 0u);
  EXPECT_EQ(loaded.area_completed_swaths[0], (std::set<std::size_t>{0}));
  EXPECT_EQ(loaded.completed_areas, (std::set<uint32_t>{0}));

  std::remove(path.c_str());
}

TEST(CoveragePersistence, MissingFileLeavesContextUntouched)
{
  BTContext ctx;
  ctx.coverage_resume_path = tempPath("coverage_resume_does_not_exist.txt");
  std::remove(ctx.coverage_resume_path.c_str());

  EXPECT_FALSE(loadCoverageResumeState(ctx));
  EXPECT_EQ(ctx.current_area, -1);  // BTContext default, unchanged
  EXPECT_TRUE(ctx.area_resume_pose_index.empty());
}

TEST(CoveragePersistence, EmptyPathIsNoOp)
{
  BTContext ctx;  // coverage_resume_path defaults to ""
  EXPECT_FALSE(saveCoverageResumeState(ctx));
  EXPECT_FALSE(loadCoverageResumeState(ctx));
  EXPECT_TRUE(clearCoverageResumeState(ctx));  // empty path is a benign no-op
}

TEST(CoveragePersistence, CorruptOrUnknownHeaderIsIgnored)
{
  const std::string path = tempPath("coverage_resume_corrupt.txt");
  {
    std::ofstream f(path, std::ios::trunc);
    f << "garbage not our header\narea 0 100 5 completed 1 2\n";
  }
  BTContext ctx;
  ctx.coverage_resume_path = path;
  EXPECT_FALSE(loadCoverageResumeState(ctx));
  EXPECT_TRUE(ctx.area_path_pose_count.empty());  // nothing parsed
  std::remove(path.c_str());
}

TEST(CoveragePersistence, MalformedRowIsSkippedNotFatal)
{
  const std::string path = tempPath("coverage_resume_partial.txt");
  {
    std::ofstream f(path, std::ios::trunc);
    f << "mowgli_coverage_resume v2\n";
    f << "current_area 1\n";
    f << "area not_a_number garbage\n";  // malformed → skipped
    // v2 row: area idx pose_count fingerprint resume completed ...
    f << "area 1 2048 777 512 completed 0 1 2\n";  // valid → loaded
  }
  BTContext ctx;
  ctx.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(ctx));
  EXPECT_EQ(ctx.current_area, 1);
  ASSERT_EQ(ctx.area_path_pose_count.count(1), 1u);
  EXPECT_EQ(ctx.area_path_pose_count[1], 2048u);
  EXPECT_EQ(ctx.area_plan_fingerprint[1], 777u);
  EXPECT_EQ(ctx.area_resume_pose_index[1], 512u);
  EXPECT_EQ(ctx.area_completed_swaths[1], (std::set<std::size_t>{0, 1, 2}));
  std::remove(path.c_str());
}

TEST(CoveragePersistence, ClearRemovesFile)
{
  const std::string path = tempPath("coverage_resume_clear.txt");
  BTContext ctx;
  seedContext(ctx, path);
  ASSERT_TRUE(saveCoverageResumeState(ctx));
  ASSERT_TRUE(std::ifstream(path).good());

  EXPECT_TRUE(clearCoverageResumeState(ctx));
  EXPECT_FALSE(std::ifstream(path).good());  // gone

  // Loading after a clear starts fresh.
  BTContext reloaded;
  reloaded.coverage_resume_path = path;
  EXPECT_FALSE(loadCoverageResumeState(reloaded));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
