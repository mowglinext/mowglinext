// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager integration checks use real kernel file-size failures and real
// persisted archives. No hardware or physical power-loss behavior is exercised.

#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "fusion_graph/checkpoint_file.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>
#include <gtsam/base/serialization.h>
#include <gtsam/inference/Symbol.h>
#include <sys/resource.h>

namespace fg = fusion_graph;
namespace fs = std::filesystem;

namespace
{
class PersistenceFaultTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto pattern = (fs::temp_directory_path() / "fg_persistence_fault_XXXXXX").string();
    char* created = ::mkdtemp(pattern.data());
    ASSERT_NE(created, nullptr);
    directory_ = created;
    prefix_ = directory_ + "/graph";
  }

  void TearDown() override
  {
    if (!directory_.empty())
    {
      std::error_code ec;
      fs::remove_all(directory_, ec);
    }
  }

  void WriteLegacy(const gtsam::Pose2& pose, uint64_t next_index = 1)
  {
    gtsam::Values values;
    values.insert(gtsam::Symbol('x', 0), pose);
    std::ofstream graph(prefix_ + ".graph");
    graph << gtsam::serializeXML(values);
    std::ofstream metadata(prefix_ + ".meta");
    // Datum fields were optional in the original format.
    metadata << "next_index=" << next_index << "\nlast_node_time_s=0.000000\n";
  }

  std::string ReadBytes(const std::string& path)
  {
    std::ifstream stream(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(stream)), {});
  }

  void ExpectPose(const gtsam::Pose2& expected)
  {
    fg::GraphManager loaded(fg::GraphParams{});
    ASSERT_TRUE(loaded.Load(prefix_));
    ASSERT_TRUE(loaded.IsInitialized());
    const auto latest = loaded.LatestSnapshot();
    ASSERT_TRUE(latest);
    EXPECT_NEAR(latest->pose.x(), expected.x(), 1e-9);
    EXPECT_NEAR(latest->pose.y(), expected.y(), 1e-9);
    EXPECT_NEAR(latest->pose.theta(), expected.theta(), 1e-9);
    // Reloaded localization must remain usable when a new node is created.
    loaded.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
    EXPECT_TRUE(loaded.Tick(0.2).has_value());
  }

  bool SaveWithFileSizeLimit(fg::GraphManager& manager)
  {
    struct rlimit original
    {
    };
    if (::getrlimit(RLIMIT_FSIZE, &original) != 0)
    {
      ADD_FAILURE() << "getrlimit(RLIMIT_FSIZE) failed";
      return true;
    }
    struct sigaction ignore
    {
    };
    struct sigaction old_action
    {
    };
    ignore.sa_handler = SIG_IGN;
    ::sigemptyset(&ignore.sa_mask);
    if (::sigaction(SIGXFSZ, &ignore, &old_action) != 0)
    {
      ADD_FAILURE() << "sigaction(SIGXFSZ) failed";
      return true;
    }
    auto limited = original;
    limited.rlim_cur = 1;
    if (::setrlimit(RLIMIT_FSIZE, &limited) != 0)
    {
      ::sigaction(SIGXFSZ, &old_action, nullptr);
      ADD_FAILURE() << "setrlimit(RLIMIT_FSIZE) failed";
      return true;
    }
    bool saved = false;
    try
    {
      saved = manager.Save(prefix_);
    }
    catch (...)
    {
      ::setrlimit(RLIMIT_FSIZE, &original);
      ::sigaction(SIGXFSZ, &old_action, nullptr);
      throw;
    }
    // Restore before gtest reports results, including when output is a file.
    EXPECT_EQ(::setrlimit(RLIMIT_FSIZE, &original), 0);
    EXPECT_EQ(::sigaction(SIGXFSZ, &old_action, nullptr), 0);
    return saved;
  }

  std::string directory_;
  std::string prefix_;
};
}  // namespace

TEST_F(PersistenceFaultTest, KernelWriteLimitNeverReportsSuccessOrReplacesPrior)
{
  const gtsam::Pose2 old_pose(1.0, 2.0, 0.5);
  fg::GraphManager manager(fg::GraphParams{});
  manager.Initialize(old_pose, 0.0);
  ASSERT_TRUE(manager.Save(prefix_));
  const auto original = ReadBytes(prefix_ + ".checkpoint");
  ASSERT_FALSE(original.empty());
  manager.Reset();
  manager.Initialize(gtsam::Pose2(10.0, 20.0, 1.0), 0.0);

  EXPECT_FALSE(SaveWithFileSizeLimit(manager));
  EXPECT_EQ(ReadBytes(prefix_ + ".checkpoint"), original);
  ExpectPose(old_pose);
}

TEST_F(PersistenceFaultTest, FailedFirstMigrationPreservesLoadableLegacyPair)
{
  const gtsam::Pose2 legacy_pose(1.0, 2.0, 0.5);
  WriteLegacy(legacy_pose);
  const auto graph = ReadBytes(prefix_ + ".graph");
  const auto metadata = ReadBytes(prefix_ + ".meta");
  fg::GraphManager manager(fg::GraphParams{});
  manager.Initialize(gtsam::Pose2(10.0, 20.0, 1.0), 0.0);

  EXPECT_FALSE(SaveWithFileSizeLimit(manager));
  EXPECT_FALSE(fs::exists(prefix_ + ".checkpoint"));
  EXPECT_EQ(ReadBytes(prefix_ + ".graph"), graph);
  EXPECT_EQ(ReadBytes(prefix_ + ".meta"), metadata);
  ExpectPose(legacy_pose);
}

TEST_F(PersistenceFaultTest, LegacyWithoutDatumRoundTripsIntoCombinedFormat)
{
  const gtsam::Pose2 legacy_pose(1.0, 2.0, 0.5);
  WriteLegacy(legacy_pose);
  fg::GraphManager migrated(fg::GraphParams{});
  ASSERT_TRUE(migrated.Load(prefix_));
  ASSERT_TRUE(migrated.Save(prefix_));
  ASSERT_TRUE(fs::exists(prefix_ + ".checkpoint"));
  fs::remove(prefix_ + ".graph");
  fs::remove(prefix_ + ".meta");
  ExpectPose(legacy_pose);
}

TEST_F(PersistenceFaultTest, DamagedModernCheckpointCannotResurrectStaleLegacy)
{
  WriteLegacy(gtsam::Pose2(99.0, 99.0, 0.0));
  {
    std::ofstream damaged(prefix_ + ".checkpoint");
    damaged << "interrupted or corrupted checkpoint";
  }
  fg::GraphManager loaded(fg::GraphParams{});
  EXPECT_FALSE(loaded.Load(prefix_));
  EXPECT_FALSE(loaded.IsInitialized());
  EXPECT_FALSE(loaded.Tick(1.0).has_value());
}

TEST_F(PersistenceFaultTest, TemporaryFilesDoNotHideLegacyCheckpoint)
{
  const gtsam::Pose2 legacy_pose(1.0, 2.0, 0.5);
  WriteLegacy(legacy_pose);
  {
    std::ofstream temporary(prefix_ + ".checkpoint.tmp.abandoned");
    temporary << "incomplete replacement";
  }
  ExpectPose(legacy_pose);
}

TEST_F(PersistenceFaultTest, CorruptPrimaryFallsBackToCompletePreviousGeneration)
{
  const gtsam::Pose2 old_pose(1.0, 2.0, 0.5);
  fg::GraphManager manager(fg::GraphParams{});
  manager.Initialize(old_pose, 0.0);
  ASSERT_TRUE(manager.Save(prefix_));
  manager.Reset();
  manager.Initialize(gtsam::Pose2(10.0, 20.0, 1.0), 0.0);
  ASSERT_TRUE(manager.Save(prefix_));
  {
    std::ofstream damaged(prefix_ + ".checkpoint");
    damaged << "corrupt primary";
  }
  ExpectPose(old_pose);
}

TEST_F(PersistenceFaultTest, FailedReplacementCannotDestroyRecoveredPrevious)
{
  const gtsam::Pose2 old_pose(1.0, 2.0, 0.5);
  fg::GraphManager manager(fg::GraphParams{});
  manager.Initialize(old_pose, 0.0);
  ASSERT_TRUE(manager.Save(prefix_));
  manager.Reset();
  manager.Initialize(gtsam::Pose2(10.0, 20.0, 1.0), 0.0);
  ASSERT_TRUE(manager.Save(prefix_));
  const auto recovery = ReadBytes(prefix_ + ".checkpoint.previous");
  ASSERT_FALSE(recovery.empty());
  // A directory cannot be atomically replaced by the checkpoint file. The
  // failure occurs at publication, after the replacement bytes were flushed.
  ASSERT_TRUE(fs::remove(prefix_ + ".checkpoint"));
  ASSERT_TRUE(fs::create_directory(prefix_ + ".checkpoint"));

  EXPECT_FALSE(manager.Save(prefix_));
  EXPECT_EQ(ReadBytes(prefix_ + ".checkpoint.previous"), recovery);
  ExpectPose(old_pose);
}

TEST_F(PersistenceFaultTest, LegacyMetadataReferencingAnAbsentLatestPoseIsRejected)
{
  WriteLegacy(gtsam::Pose2(1.0, 2.0, 0.5), 2);
  fg::GraphManager loaded(fg::GraphParams{});
  EXPECT_FALSE(loaded.Load(prefix_));
  EXPECT_FALSE(loaded.IsInitialized());
  EXPECT_FALSE(loaded.Tick(1.0).has_value());
}

TEST_F(PersistenceFaultTest, SemanticallyInvalidPrimaryNeverReplacesRecoveredPrevious)
{
  const gtsam::Pose2 old_pose(1.0, 2.0, 0.5);
  const gtsam::Pose2 next_pose(10.0, 20.0, 1.0);
  for (const bool invalid_xml : {true, false})
  {
    SCOPED_TRACE(invalid_xml ? "invalid graph XML" : "inconsistent next_index");
    fg::GraphManager manager(fg::GraphParams{});
    manager.Initialize(old_pose, 0.0);
    ASSERT_TRUE(manager.Save(prefix_));
    fg::checkpoint::Payload invalid;
    ASSERT_TRUE(fg::checkpoint::Read(prefix_ + ".checkpoint", invalid));
    if (invalid_xml)
      invalid.graph = "not a serialized GTSAM graph";
    else
      invalid.metadata = "next_index=2\nlast_node_time_s=0.000000\n";
    // Publish a complete, checksum-valid file that GraphManager cannot use.
    ASSERT_TRUE(fg::checkpoint::Save(prefix_, invalid));
    const auto recovery = ReadBytes(prefix_ + ".checkpoint.previous");
    ASSERT_FALSE(recovery.empty());
    fg::GraphManager recovered(fg::GraphParams{});
    ASSERT_TRUE(recovered.Load(prefix_));
    const auto latest = recovered.LatestSnapshot();
    ASSERT_TRUE(latest);
    EXPECT_NEAR(latest->pose.x(), old_pose.x(), 1e-9);
    recovered.Reset();
    recovered.Initialize(next_pose, 0.0);
    ASSERT_TRUE(recovered.Save(prefix_));
    // A failed replacement can only preserve this baseline if preparation did
    // not rotate the unreadable primary over the recovered generation.
    EXPECT_EQ(ReadBytes(prefix_ + ".checkpoint.previous"), recovery);
    ExpectPose(next_pose);
  }
}
