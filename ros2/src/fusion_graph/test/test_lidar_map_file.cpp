// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#include <cstdio>
#include <string>

#include "fusion_graph/lidar_map_file.hpp"
#include <gtest/gtest.h>

using namespace fusion_graph;

namespace
{
std::string TmpPath(const char* name)
{
  return std::string(::testing::TempDir()) + "/" + name;
}
}  // namespace

TEST(LidarMapFile, RoundTripsAMapThroughDiskAndImport)
{
  LidarOccupancyMapperParams p;
  p.resolution_m = 0.10;
  p.half_extent_m = 5.0;
  LidarOccupancyMapper a(p);
  std::vector<std::pair<double, double>> wall;
  for (double y = -1.0; y <= 1.0; y += 0.05)
    wall.emplace_back(2.0, y);
  for (int i = 0; i < 4; ++i)
    a.Insert(0.0, 0.0, 0.0, wall);
  LidarMapFile m;
  m.datum_lat = 48.879649550;
  m.datum_lon = 2.172814460;
  m.grid = a.Export();
  const auto path = TmpPath("fg_lidar_map_test.lidarmap");
  ASSERT_TRUE(WriteLidarMapFile(path, m));
  const auto r = ReadLidarMapFile(path);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(r->grid.data, m.grid.data);
  EXPECT_EQ(r->grid.occupied, m.grid.occupied);
  EXPECT_EQ(r->grid.free, m.grid.free);
  EXPECT_NEAR(r->datum_lat, m.datum_lat, 1e-12);
  EXPECT_TRUE(LidarMapDatumMatches(*r, 48.879649550, 2.172814460));
  EXPECT_FALSE(LidarMapDatumMatches(*r, 48.88, 2.17));
  LidarOccupancyMapper b(p);
  b.ImportCells(r->grid.resolution_m,
                r->grid.origin_x,
                r->grid.origin_y,
                static_cast<int>(r->grid.width),
                static_cast<int>(r->grid.height),
                r->grid.data);
  EXPECT_EQ(b.Export().occupied, m.grid.occupied);
  std::remove(path.c_str());
}

TEST(LidarMapFile, RefusesGarbage)
{
  const auto path = TmpPath("fg_lidar_map_garbage.lidarmap");
  {
    std::ofstream os(path, std::ios::binary);
    os << "not a map";
  }
  EXPECT_FALSE(ReadLidarMapFile(path).has_value());
  EXPECT_FALSE(ReadLidarMapFile(TmpPath("does_not_exist.lidarmap")).has_value());
  std::remove(path.c_str());
}
