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
#include <filesystem>
#include <fstream>
#include <string>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_persistence.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;

TEST(CoveragePersistence, NeverEnabledHasNoCompletedPhaseHistory)
{
  BTContext ctx;
  ctx.coverage_resume_path = ::testing::TempDir() + "/cross_hatch_disabled.txt";
  EXPECT_FALSE(mowgli_behavior::beginCoverageOrientation(ctx, 2));
  mowgli_behavior::markCoverageStarted(ctx, 2);
  EXPECT_TRUE(ctx.cross_hatch.empty());
  ASSERT_TRUE(saveCoverageResumeState(ctx));
  BTContext loaded;
  loaded.coverage_resume_path = ctx.coverage_resume_path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));
  loaded.mow_cross_hatch = true;  // changing config cannot rotate this resumed run
  EXPECT_FALSE(mowgli_behavior::beginCoverageOrientation(loaded, 2));
  EXPECT_TRUE(loaded.cross_hatch.empty());
  ASSERT_TRUE(clearCoverageResumeState(loaded));  // progress reset retains latch
  BTContext reset;
  reset.coverage_resume_path = ctx.coverage_resume_path;
  ASSERT_TRUE(loadCoverageResumeState(reset));
  EXPECT_EQ(reset.base_orientation_areas.count(2), 1u);
  reset.base_orientation_areas.clear();  // EndSession clears the temporary latch
  ASSERT_TRUE(clearCoverageResumeState(reset));
  EXPECT_FALSE(std::filesystem::exists(ctx.coverage_resume_path));
  EXPECT_FALSE(std::filesystem::exists(ctx.coverage_resume_path + ".tmp"));
}

TEST(CoveragePersistence, DisablingDuringResumeRetainsPerpendicularAndHistory)
{
  BTContext ctx;
  ctx.coverage_resume_path = ::testing::TempDir() + "/cross_hatch_toggle.txt";
  ctx.mow_cross_hatch = true;
  ctx.cross_hatch[2].next_perpendicular = true;
  ASSERT_TRUE(mowgli_behavior::beginCoverageOrientation(ctx, 2));
  BTContext loaded;
  loaded.coverage_resume_path = ctx.coverage_resume_path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));
  EXPECT_FALSE(loaded.mow_cross_hatch);
  EXPECT_TRUE(mowgli_behavior::beginCoverageOrientation(loaded, 2));
  mowgli_behavior::markCoverageStarted(loaded, 2);
  mowgli_behavior::markCoverageStarted(loaded, 2);
  loaded.cross_hatch[2].finish();
  loaded.cross_hatch[2].finish();
  EXPECT_FALSE(loaded.cross_hatch[2].next());
  ASSERT_TRUE(clearCoverageResumeState(loaded));
  EXPECT_TRUE(std::filesystem::exists(ctx.coverage_resume_path));
  std::remove(ctx.coverage_resume_path.c_str());
}

TEST(CoveragePersistence, FailedPlanningCountsSessionsAndSurvivesRestart)
{
  BTContext ctx;
  ctx.coverage_resume_path = ::testing::TempDir() + "/cross_hatch_failures.txt";
  auto& state = ctx.cross_hatch[2];
  state.next_perpendicular = true;
  for (unsigned count = 1; count <= 3; ++count)
  {
    ASSERT_TRUE(state.begin(true));
    state.planning_failed = true;
    state.begin(true);  // retry is not a new failed session
    state.finish();
    state.finish();
    EXPECT_EQ(state.failed_sessions, count);
    EXPECT_TRUE(state.next());
  }
  ASSERT_TRUE(saveCoverageResumeState(ctx));
  BTContext loaded;
  loaded.coverage_resume_path = ctx.coverage_resume_path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));
  ASSERT_EQ(loaded.cross_hatch.at(2).failed_sessions, 3u);
  loaded.cross_hatch[2].begin(true);
  mowgli_behavior::markCoverageStarted(loaded, 2);
  EXPECT_EQ(loaded.cross_hatch[2].failed_sessions, 0u);
  loaded.cross_hatch[2].finish();
  EXPECT_FALSE(loaded.cross_hatch[2].next());
  state.next_override = false;
  state.finish();
  EXPECT_EQ(state.failed_sessions, 0u);
  EXPECT_FALSE(state.next());
  std::remove(ctx.coverage_resume_path.c_str());
}

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

TEST(CoveragePersistence, FailedPhaseWriteCannotRestoreAnEndedSession)
{
  for (bool enabled : {false, true})
  {
    const auto path = tempPath(enabled ? "clear_phase_enabled.txt" : "clear_phase_disabled.txt");
    BTContext ctx;
    seedContext(ctx, path);
    ctx.current_command = 1;  // COMMAND_START with resumable progress
    ctx.single_area_target = 2;
    ctx.cross_hatch[2].begin(enabled);  // also populated when cross-hatch is disabled
    ctx.cross_hatch[2].used = true;
    ASSERT_TRUE(saveCoverageResumeState(ctx));
    ctx.cross_hatch[2].finish();

    // Force the phase-only replacement to fail to open. The old snapshot is
    // still readable/removable; clearing safety state must not depend on a write.
    const auto tmp = path + ".tmp";
    ASSERT_TRUE(std::filesystem::create_directory(tmp));
    EXPECT_FALSE(clearCoverageResumeState(ctx));  // report the lost metadata
    EXPECT_TRUE(std::filesystem::remove(tmp));

    BTContext restarted;
    restarted.coverage_resume_path = path;
    EXPECT_FALSE(loadCoverageResumeState(restarted));
    EXPECT_EQ(restarted.current_command, 0);
    EXPECT_TRUE(restarted.area_resume_pose_index.empty());
    EXPECT_TRUE(restarted.completed_areas.empty());
    EXPECT_FALSE(restarted.single_area_target.has_value());
    const bool auto_continue =
        restarted.current_command == 1 &&
        (!restarted.area_resume_pose_index.empty() || !restarted.completed_areas.empty());
    EXPECT_FALSE(auto_continue);
    std::remove(path.c_str());
  }
}

TEST(CoveragePersistence, ClearReportsRemovalFailure)
{
  // A non-empty directory cannot be removed as a resume file. Unlike ENOENT,
  // this must be reported as a failure, even without cross-hatch metadata.
  const auto path = tempPath("clear_resume_remove_failure");
  ASSERT_TRUE(std::filesystem::create_directory(path));
  const auto child = path + "/keep";
  {
    std::ofstream file(child);
  }
  BTContext ctx;
  ctx.coverage_resume_path = path;
  EXPECT_FALSE(clearCoverageResumeState(ctx));
  EXPECT_TRUE(std::filesystem::remove(child));
  EXPECT_TRUE(std::filesystem::remove(path));
  EXPECT_TRUE(clearCoverageResumeState(ctx));  // absent is already clear
}

TEST(CoveragePersistence, RoundTripsCurrentCommand)
{
  // The active high-level command must survive a restart so the node can
  // auto-re-enter MowingSequence instead of coming up IDLE (issue: mowing
  // restarts from scratch after a container restart).
  const std::string path = tempPath("coverage_resume_command.txt");
  std::remove(path.c_str());

  BTContext saved;
  seedContext(saved, path);
  saved.current_command = 1;  // COMMAND_START — a mow was running
  ASSERT_TRUE(saveCoverageResumeState(saved));

  BTContext loaded;
  loaded.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));
  EXPECT_EQ(loaded.current_command, 1);
  // The resume cursor still round-trips alongside the command.
  EXPECT_EQ(loaded.area_resume_pose_index[2], 1234u);

  std::remove(path.c_str());
}

TEST(CoveragePersistence, AbsentCurrentCommandDefaultsToIdle)
{
  // Older files (and terminal/empty snapshots) predate the current_command line;
  // an absent tag must leave current_command at its default 0 (IDLE) so a fresh
  // install never starts moving on boot without a real command on disk.
  const std::string path = tempPath("coverage_resume_no_command.txt");
  {
    std::ofstream f(path, std::ios::trunc);
    f << "mowgli_coverage_resume v2\n";
    f << "current_area 0\n";
    f << "area 0 1000 5 512 completed 0 1\n";  // resumable, but no command line
  }
  BTContext ctx;
  ctx.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(ctx));
  EXPECT_EQ(ctx.current_command, 0);  // default preserved — start IDLE
  std::remove(path.c_str());
}

TEST(CoveragePersistence, RoundTripsSingleAreaTarget)
{
  // A targeted run (~/start_in_area) must stay targeted across a restart: the
  // restored current_command auto-re-enters MowingSequence, so without this the
  // run would silently widen into a mow-the-whole-lawn run once the requested
  // area finished.
  const std::string path = tempPath("coverage_resume_single_area.txt");
  std::remove(path.c_str());

  BTContext saved;
  seedContext(saved, path);
  saved.current_command = 1;
  saved.single_area_target = 3u;
  ASSERT_TRUE(saveCoverageResumeState(saved));

  BTContext loaded;
  loaded.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));
  ASSERT_TRUE(loaded.single_area_target.has_value());
  EXPECT_EQ(*loaded.single_area_target, 3u);

  std::remove(path.c_str());
}

TEST(CoveragePersistence, AbsentSingleAreaTargetMeansAllAreas)
{
  // Absent in every non-targeted run (and in files written before targeted runs
  // became session state) — that must read back as "no clip", not as area 0.
  const std::string path = tempPath("coverage_resume_no_single_area.txt");
  std::remove(path.c_str());

  BTContext saved;
  seedContext(saved, path);
  saved.current_command = 1;
  ASSERT_TRUE(saveCoverageResumeState(saved));

  BTContext loaded;
  loaded.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(loaded));
  EXPECT_FALSE(loaded.single_area_target.has_value());

  std::remove(path.c_str());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(CoveragePersistence, CrossHatchSurvivesRestartAndAdvancesOnlyAtSessionEnd)
{
  const auto path = tempPath("cross_hatch_resume.txt");
  std::remove(path.c_str());
  BTContext ctx;
  ctx.coverage_resume_path = path;
  EXPECT_FALSE(ctx.cross_hatch[0].begin(true));
  ctx.cross_hatch[0].used = true;
  ctx.current_command = 1;
  ctx.area_resume_pose_index[0] = 42;
  ASSERT_TRUE(saveCoverageResumeState(ctx));

  BTContext restored;
  restored.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(restored));
  EXPECT_FALSE(restored.cross_hatch[0].begin(true));
  EXPECT_EQ(restored.area_resume_pose_index[0], 42u);
  // A clear-progress request preserves the current phase without a cursor.
  ASSERT_TRUE(clearCoverageResumeState(restored));
  BTContext cleared;
  cleared.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(cleared));
  EXPECT_EQ(cleared.current_command, 0u);
  EXPECT_TRUE(cleared.area_resume_pose_index.empty());
  EXPECT_FALSE(cleared.cross_hatch[0].begin(true));

  restored.cross_hatch[0].finish();
  ASSERT_TRUE(clearCoverageResumeState(restored));
  BTContext next;
  next.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(next));
  EXPECT_EQ(next.current_command, 0u);  // phase history must never auto-start
  EXPECT_TRUE(next.area_resume_pose_index.empty());
  EXPECT_TRUE(next.cross_hatch[0].begin(true));
  EXPECT_TRUE(
      next.cross_hatch[0].begin(false));  // configuration change cannot rotate a live session
  next.cross_hatch[0].used = true;
  next.cross_hatch[0].finish();
  ASSERT_TRUE(clearCoverageResumeState(next));
  BTContext third;
  third.coverage_resume_path = path;
  loadCoverageResumeState(third);
  EXPECT_FALSE(third.cross_hatch[0].begin(true));
  std::remove(path.c_str());
}

TEST(CoveragePersistence, CrossHatchDoesNotAdvanceForUnusedOrDisabledSessions)
{
  mowgli_behavior::CrossHatch state;
  EXPECT_FALSE(state.begin(true));
  state.finish();  // planning failed; no coverage started
  EXPECT_FALSE(state.begin(true));
  state.used = true;
  state.finish();
  state.finish();  // repeated EndSession is harmless
  EXPECT_FALSE(state.begin(false));
  state.used = true;
  state.finish();
  EXPECT_TRUE(state.begin(true));  // disabled run did not consume the next phase
}

TEST(CoveragePersistence, MalformedCrossHatchStateIsIgnored)
{
  const auto path = tempPath("cross_hatch_corrupt.txt");
  {
    std::ofstream out(path);
    out << "mowgli_coverage_resume v2\ncross_hatch_area 0 1 5 1 1 -1\n";
  }
  BTContext ctx;
  ctx.coverage_resume_path = path;
  ASSERT_TRUE(loadCoverageResumeState(ctx));
  EXPECT_FALSE(ctx.cross_hatch[0].begin(true));
  std::remove(path.c_str());
}

TEST(CoveragePersistence, AreasAndNextOverridesRoundTripIndependently)
{
  BTContext ctx;
  ctx.coverage_resume_path = tempPath("cross_hatch_areas.txt");
  ctx.cross_hatch[0].begin(true);
  ctx.cross_hatch[0].used = true;
  ctx.cross_hatch[2].next_override = true;
  ASSERT_TRUE(saveCoverageResumeState(ctx));
  BTContext restored;
  restored.coverage_resume_path = ctx.coverage_resume_path;
  ASSERT_TRUE(loadCoverageResumeState(restored));
  EXPECT_FALSE(restored.cross_hatch[0].begin(true));
  EXPECT_TRUE(restored.cross_hatch[0].next());
  EXPECT_TRUE(restored.cross_hatch[2].begin(true));
  EXPECT_FALSE(restored.cross_hatch[1].begin(true));
  // An override during an active run affects NEXT, not CURRENT, and survives end.
  restored.cross_hatch[0].next_override = false;
  EXPECT_FALSE(restored.cross_hatch[0].begin(true));
  restored.cross_hatch[0].finish();
  EXPECT_FALSE(restored.cross_hatch[0].begin(true));
  std::remove(ctx.coverage_resume_path.c_str());
}
