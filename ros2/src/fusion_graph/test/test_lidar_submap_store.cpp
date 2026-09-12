// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "fusion_graph/lidar_map_file.hpp"
#include "fusion_graph/lidar_submap_store.hpp"
#include <gtest/gtest.h>
#include <unistd.h>

namespace fusion_graph
{
namespace
{
class SubmapStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char path[] = "/tmp/mowgli-submaps-XXXXXX";
    const auto directory = mkdtemp(path);
    ASSERT_NE(directory, nullptr);
    directory_ = directory;
    prefix_ = directory_ + "/map";
    params_.resolution_m = 0.1;
  }
  void TearDown() override
  {
    std::filesystem::remove_all(directory_);
  }
  bool Ready(LidarSubmapStore& store, double x = 0, double y = 0)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (store.UpdateWindow(x, y))
        return true;
      if (!store.last_error().empty())
        return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }
  std::string directory_, prefix_;
  LidarOccupancyMapperParams params_;
};

TEST_F(SubmapStoreTest, NegativeCoordinatesUseFloorAndBoundedWindow)
{
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store, -0.1, -10.1));
  auto grid = store.mapper()->Export();
  EXPECT_DOUBLE_EQ(grid.origin_x, -30);
  EXPECT_DOUBLE_EQ(grid.origin_y, -40);
  EXPECT_EQ(grid.width, 500u);
  EXPECT_EQ(grid.height, 500u);
  EXPECT_EQ(store.window_cells(), 500u);
  EXPECT_TRUE(store.TakeWindowChanged());
  EXPECT_FALSE(store.TakeWindowChanged());
  ASSERT_TRUE(Ready(store, 12345, -4567));
  EXPECT_EQ(store.mapper()->size(), 500u);
}

TEST_F(SubmapStoreTest, PreservesExactEvidenceAcrossMoveAndRestart)
{
  double expected;
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store));
    store.mapper()->Insert(0, 0, 0, {{3.05, 0.05}});
    store.mapper()->Insert(0, 0, 0, {{3.05, 0.05}});
    expected = store.mapper()->LogOddsAt(3.05, 0.05);
    ASSERT_GT(expected, 0.85);
    ASSERT_TRUE(Ready(store, 100, 100));
    EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(3.05, 0.05), 0);
    ASSERT_TRUE(Ready(store));
    EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(3.05, 0.05), expected);
    store.RequestSave();
    ASSERT_TRUE(Ready(store));
  }
  LidarSubmapStore restored(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(restored));
  EXPECT_DOUBLE_EQ(restored.mapper()->LogOddsAt(3.05, 0.05), expected);
}

TEST_F(SubmapStoreTest, BorderEvidenceSurvivesOverlappingShift)
{
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store));
  auto* map = store.mapper();
  ASSERT_TRUE(map->SetEvidence(199, 200, {0.45, 1}));
  ASSERT_TRUE(map->SetEvidence(200, 200, {-1.2345, 1}));
  ASSERT_TRUE(Ready(store, 11.1, 0));
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(-0.05, 0.05), 0.45);
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(0.05, 0.05), -1.2345);
  ASSERT_TRUE(Ready(store, -10, 0));
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(-0.05, 0.05), 0.45);
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(0.05, 0.05), -1.2345);
  // Unclassified but touched evidence (0.45) must not disappear via trinary export.
  auto evidence = store.mapper()->EvidenceAt(299, 200);
  EXPECT_EQ(evidence.touched, 1);
}

TEST_F(SubmapStoreTest, RejectsDifferentDatumEvenInEmptyDistantWindow)
{
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store));
    store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
    store.RequestSave();
    ASSERT_TRUE(Ready(store));
  }
  LidarSubmapStore other(params_, prefix_, 49, 2);
  EXPECT_FALSE(Ready(other, 1000, 1000));
  EXPECT_NE(other.last_error().find("datum"), std::string::npos);
  EXPECT_EQ(other.mapper(), nullptr);
}

TEST_F(SubmapStoreTest, ClearDuringSaveCannotResurrectTiles)
{
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store));
    store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
    store.RequestSave();
    store.Clear();
    EXPECT_EQ(store.mapper(), nullptr);
    ASSERT_TRUE(Ready(store));
    EXPECT_EQ(store.mapper()->Export().occupied, 0u);
  }
  LidarSubmapStore restored(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(restored));
  EXPECT_EQ(restored.mapper()->Export().occupied, 0u);
}

TEST_F(SubmapStoreTest, ClearDuringSaveThenDestructStillDeletes)
{
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store));
    store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
    store.RequestSave();
    store.Clear();
  }
  EXPECT_FALSE(std::filesystem::exists(prefix_ + ".lidartiles"));
}

TEST_F(SubmapStoreTest, CorruptedTileIsRejectedBeforeWindowBecomesVisible)
{
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store));
    store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
    store.RequestSave();
    ASSERT_TRUE(Ready(store));
  }
  std::ofstream corrupt(prefix_ + ".lidartiles/0_0.tile", std::ios::binary | std::ios::trunc);
  corrupt << "broken";
  corrupt.close();
  LidarSubmapStore restored(params_, prefix_, 48, 2);
  EXPECT_FALSE(Ready(restored));
  EXPECT_NE(restored.last_error().find("length"), std::string::npos);
  EXPECT_EQ(restored.mapper(), nullptr);
}

TEST_F(SubmapStoreTest, RejectsNonFiniteEvidenceInOtherwiseValidTile)
{
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store));
    store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
    store.RequestSave();
    ASSERT_TRUE(Ready(store));
  }
  std::fstream corrupt(prefix_ + ".lidartiles/0_0.tile",
                       std::ios::binary | std::ios::in | std::ios::out);
  corrupt.seekp(52);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  corrupt.write(reinterpret_cast<const char*>(&nan), sizeof(nan));
  corrupt.close();
  LidarSubmapStore restored(params_, prefix_, 48, 2);
  EXPECT_FALSE(Ready(restored));
  EXPECT_NE(restored.last_error().find("evidence"), std::string::npos);
}

TEST_F(SubmapStoreTest, LegacyImportPersistsCellsOutsideActiveWindow)
{
  ExportedOccupancyGrid grid;
  grid.origin_x = -50;
  grid.origin_y = -1;
  grid.resolution_m = 0.1;
  grid.width = 1100;
  grid.height = 10;
  grid.data.assign(grid.width * grid.height, -1);
  grid.data[0] = 100;
  grid.data[1099] = 0;
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store));
  ASSERT_TRUE(store.ImportLegacy(grid));
  ASSERT_TRUE(Ready(store));
  ASSERT_TRUE(Ready(store, -50, 0));
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(-49.95, -0.95), params_.occupied_threshold);
  ASSERT_TRUE(Ready(store, 59, 0));
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(59.95, -0.95), params_.free_threshold);
  auto invalid = grid;
  invalid.resolution_m = 0.2;
  EXPECT_FALSE(store.ImportLegacy(invalid));
  invalid = grid;
  invalid.origin_x += 0.05;
  EXPECT_FALSE(store.ImportLegacy(invalid));
}

TEST_F(SubmapStoreTest, HysteresisPreventsRepeatedBoundaryReloads)
{
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store));
  ASSERT_TRUE(store.TakeWindowChanged());
  for (const auto x : {9.9, 10.1, 9.8, 10.5, 10.99})
  {
    EXPECT_TRUE(store.UpdateWindow(x, 0));
    EXPECT_FALSE(store.busy());
    EXPECT_FALSE(store.TakeWindowChanged());
  }
  ASSERT_TRUE(Ready(store, 11.1, 0));
  EXPECT_TRUE(store.TakeWindowChanged());
  for (const auto x : {10.9, 10.1, 9.8, 9.1})
  {
    EXPECT_TRUE(store.UpdateWindow(x, 0));
    EXPECT_FALSE(store.TakeWindowChanged());
  }
  ASSERT_TRUE(Ready(store, 8.9, 0));
  EXPECT_TRUE(store.TakeWindowChanged());
  ASSERT_TRUE(Ready(store, -1.1, 0));
  EXPECT_TRUE(store.TakeWindowChanged());
  for (const auto x : {-0.9, 0.1, -0.1, 0.9})
  {
    EXPECT_TRUE(store.UpdateWindow(x, 0));
    EXPECT_FALSE(store.TakeWindowChanged());
  }
}

TEST_F(SubmapStoreTest, FailedMoveRetainsOriginalEvidenceAndBacksOff)
{
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store));
  store.TakeWindowChanged();
  store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
  const auto evidence = store.mapper()->LogOddsAt(1.05, 0.05);
  std::filesystem::create_directories(prefix_ + ".lidartiles");
  std::ofstream corrupt(prefix_ + ".lidartiles/10_10.tile");
  corrupt << "bad";
  corrupt.close();
  // Create a valid manifest by saving current evidence before the corrupt load.
  // Missing manifest refuses preexisting tiles, so remove the fixture temporarily.
  std::filesystem::rename(prefix_ + ".lidartiles/10_10.tile", prefix_ + ".lidartiles/10_10.bad");
  store.RequestSave();
  ASSERT_TRUE(Ready(store));
  std::filesystem::rename(prefix_ + ".lidartiles/10_10.bad", prefix_ + ".lidartiles/10_10.tile");
  EXPECT_FALSE(Ready(store, 100, 100));
  ASSERT_NE(store.mapper(), nullptr);
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(1.05, 0.05), evidence);
  EXPECT_FALSE(store.TakeWindowChanged());
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_FALSE(store.UpdateWindow(100, 100));
    EXPECT_FALSE(store.busy());
  }
  EXPECT_TRUE(store.UpdateWindow(0, 0));
  EXPECT_NE(store.last_error().find("length"), std::string::npos);
}

TEST_F(SubmapStoreTest, FailedSaveRetainsEvidenceAndBacksOff)
{
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store));
  store.mapper()->Insert(0, 0, 0, {{1.05, 0.05}});
  const auto evidence = store.mapper()->LogOddsAt(1.05, 0.05);
  std::filesystem::create_directories(prefix_ + ".lidartiles/0_0.tile.tmp");
  store.RequestSave();
  // Ready can return true with the old in-memory window after save fails.
  ASSERT_TRUE(Ready(store));
  ASSERT_NE(store.mapper(), nullptr);
  EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(1.05, 0.05), evidence);
  EXPECT_FALSE(store.last_error().empty());
  store.RequestSave();
  EXPECT_FALSE(store.busy());
}

TEST_F(SubmapStoreTest, SaveDoesNotSignalWindowChange)
{
  LidarSubmapStore store(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(store));
  ASSERT_TRUE(store.TakeWindowChanged());
  store.RequestSave();
  EXPECT_TRUE(store.busy());
  EXPECT_EQ(store.mapper(), nullptr);
  ASSERT_TRUE(Ready(store));
  EXPECT_FALSE(store.TakeWindowChanged());
}

TEST_F(SubmapStoreTest, MigratesLegacyFileAsynchronouslyAndOnlyOnce)
{
  LidarMapFile legacy;
  legacy.datum_lat = 48;
  legacy.datum_lon = 2;
  legacy.grid.origin_x = -50;
  legacy.grid.origin_y = 0;
  legacy.grid.resolution_m = 0.1;
  legacy.grid.width = 2;
  legacy.grid.height = 1;
  legacy.grid.data = {100, 0};
  ASSERT_TRUE(WriteLidarMapFile(prefix_ + ".lidarmap", legacy));
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    ASSERT_TRUE(Ready(store, -50, 0));
    EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(-49.95, 0.05), params_.occupied_threshold);
    EXPECT_DOUBLE_EQ(store.mapper()->LogOddsAt(-49.85, 0.05), params_.free_threshold);
    EXPECT_FALSE(std::filesystem::exists(prefix_ + ".lidarmap"));
    EXPECT_TRUE(std::filesystem::exists(prefix_ + ".lidarmap.migrated"));
    store.Clear();
    ASSERT_TRUE(Ready(store, -50, 0));
  }
  LidarSubmapStore restored(params_, prefix_, 48, 2);
  ASSERT_TRUE(Ready(restored, -50, 0));
  EXPECT_EQ(restored.mapper()->Export().occupied, 0u);
}

TEST_F(SubmapStoreTest, RejectsLegacyWrongDatumAndMaliciousDimensions)
{
  LidarMapFile legacy;
  legacy.datum_lat = 49;
  legacy.datum_lon = 2;
  legacy.grid.resolution_m = 0.1;
  legacy.grid.width = legacy.grid.height = 1;
  legacy.grid.data = {100};
  ASSERT_TRUE(WriteLidarMapFile(prefix_ + ".lidarmap", legacy));
  {
    LidarSubmapStore store(params_, prefix_, 48, 2);
    EXPECT_FALSE(Ready(store));
    EXPECT_NE(store.last_error().find("datum"), std::string::npos);
  }
  std::fstream file(prefix_ + ".lidarmap", std::ios::binary | std::ios::in | std::ios::out);
  file.seekp(48);
  const uint64_t huge = 100000;
  file.write(reinterpret_cast<const char*>(&huge), sizeof(huge));
  file.close();
  LidarSubmapStore store(params_, prefix_, 49, 2);
  EXPECT_FALSE(Ready(store));
  EXPECT_EQ(store.mapper(), nullptr);
}

TEST_F(SubmapStoreTest, RejectsInvalidLegacyProbabilitiesAndOverflowingTileIndices)
{
  params_.resolution_m = 1e-9;
  LidarSubmapStore store(params_, prefix_, 48, 2, 1e-7);
  ExportedOccupancyGrid grid;
  grid.resolution_m = params_.resolution_m;
  grid.width = grid.height = 1;
  grid.data = {101};
  EXPECT_FALSE(store.ImportLegacy(grid));
  grid.data = {-2};
  EXPECT_FALSE(store.ImportLegacy(grid));
  grid.data = {100};
  grid.resolution_m = -params_.resolution_m;
  EXPECT_FALSE(store.ImportLegacy(grid));
  grid.resolution_m = params_.resolution_m;
  grid.origin_x = 10000;
  EXPECT_FALSE(store.ImportLegacy(grid));
  grid.origin_x = 0;
  grid.origin_y = -10000;
  EXPECT_FALSE(store.ImportLegacy(grid));
  EXPECT_FALSE(store.busy());
}

TEST_F(SubmapStoreTest, RejectsInvalidEvidenceParameters)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  for (const auto value : {0.0, -0.1, nan, infinity})
  {
    auto params = params_;
    params.resolution_m = value;
    EXPECT_THROW(LidarSubmapStore(params, prefix_, 48, 2), std::invalid_argument);
  }
  for (const auto value : {0.0, -1.0, nan, infinity, params_.log_odds_max + 1})
  {
    auto params = params_;
    params.log_odds_hit = value;
    EXPECT_THROW(LidarSubmapStore(params, prefix_, 48, 2), std::invalid_argument);
    params = params_;
    params.occupied_threshold = value;
    EXPECT_THROW(LidarSubmapStore(params, prefix_, 48, 2), std::invalid_argument);
  }
  for (const auto value : {0.0, 1.0, nan, -infinity, params_.log_odds_min - 1})
  {
    auto params = params_;
    params.log_odds_miss = value;
    EXPECT_THROW(LidarSubmapStore(params, prefix_, 48, 2), std::invalid_argument);
    params = params_;
    params.free_threshold = value;
    EXPECT_THROW(LidarSubmapStore(params, prefix_, 48, 2), std::invalid_argument);
  }
}

TEST_F(SubmapStoreTest, RejectsUnboundedOrMisalignedGeometry)
{
  EXPECT_THROW(LidarSubmapStore(params_, prefix_, 48, 2, 10.05), std::invalid_argument);
  EXPECT_THROW(LidarSubmapStore(params_, prefix_, 48, 2, 100, 8), std::invalid_argument);
  EXPECT_THROW(LidarSubmapStore(params_, prefix_, 100, 2), std::invalid_argument);
}
}  // namespace
}  // namespace fusion_graph
