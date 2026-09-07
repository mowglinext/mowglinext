// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
//
// On-disk form of the LiDAR anchor map: the exported occupancy grid (0 free,
// 100 occupied, -1 unknown) with its geometry and the datum it was built in.
// Loaded through LidarOccupancyMapper::ImportCells, i.e. at the occupied /
// free thresholds — known, not saturated — so live scans can still overturn
// what changed since (a hedge trimmed, a chair moved). A map built under a
// different datum is refused: its cells would land in the wrong garden.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include "fusion_graph/lidar_occupancy_mapper.hpp"

namespace fusion_graph
{

struct LidarMapFile
{
  double datum_lat = 0.0;
  double datum_lon = 0.0;
  ExportedOccupancyGrid grid;
};

inline constexpr char kLidarMapMagic[8] = {'F', 'G', 'L', 'M', 'A', 'P', '0', '1'};

inline bool WriteLidarMapFile(const std::string& path, const LidarMapFile& m)
{
  std::ofstream os(path, std::ios::binary | std::ios::trunc);
  if (!os)
    return false;
  const uint64_t w = m.grid.width, h = m.grid.height;
  if (m.grid.data.size() != static_cast<std::size_t>(w * h))
    return false;
  os.write(kLidarMapMagic, sizeof(kLidarMapMagic));
  os.write(reinterpret_cast<const char*>(&m.datum_lat), sizeof(double));
  os.write(reinterpret_cast<const char*>(&m.datum_lon), sizeof(double));
  os.write(reinterpret_cast<const char*>(&m.grid.resolution_m), sizeof(double));
  os.write(reinterpret_cast<const char*>(&m.grid.origin_x), sizeof(double));
  os.write(reinterpret_cast<const char*>(&m.grid.origin_y), sizeof(double));
  os.write(reinterpret_cast<const char*>(&w), sizeof(uint64_t));
  os.write(reinterpret_cast<const char*>(&h), sizeof(uint64_t));
  os.write(reinterpret_cast<const char*>(m.grid.data.data()),
           static_cast<std::streamsize>(m.grid.data.size()));
  return static_cast<bool>(os);
}

inline std::optional<LidarMapFile> ReadLidarMapFile(const std::string& path)
{
  std::ifstream is(path, std::ios::binary);
  if (!is)
    return std::nullopt;
  char magic[8];
  is.read(magic, sizeof(magic));
  if (!is || std::memcmp(magic, kLidarMapMagic, sizeof(magic)) != 0)
    return std::nullopt;
  LidarMapFile m;
  uint64_t w = 0, h = 0;
  is.read(reinterpret_cast<char*>(&m.datum_lat), sizeof(double));
  is.read(reinterpret_cast<char*>(&m.datum_lon), sizeof(double));
  is.read(reinterpret_cast<char*>(&m.grid.resolution_m), sizeof(double));
  is.read(reinterpret_cast<char*>(&m.grid.origin_x), sizeof(double));
  is.read(reinterpret_cast<char*>(&m.grid.origin_y), sizeof(double));
  is.read(reinterpret_cast<char*>(&w), sizeof(uint64_t));
  is.read(reinterpret_cast<char*>(&h), sizeof(uint64_t));
  if (!is || w == 0 || h == 0 || w > 100000 || h > 100000 || !(m.grid.resolution_m > 0.0))
    return std::nullopt;
  m.grid.width = static_cast<std::size_t>(w);
  m.grid.height = static_cast<std::size_t>(h);
  m.grid.data.resize(m.grid.width * m.grid.height);
  is.read(reinterpret_cast<char*>(m.grid.data.data()),
          static_cast<std::streamsize>(m.grid.data.size()));
  if (!is)
    return std::nullopt;
  for (const int8_t v : m.grid.data)
  {
    if (v >= 50)
      ++m.grid.occupied;
    else if (v >= 0)
      ++m.grid.free;
  }
  return m;
}

// Same garden? The datum is the map frame's origin (CLAUDE.md Invariant 4);
// 1e-6 deg ≈ 0.1 m, far below any real datum change.
inline bool LidarMapDatumMatches(const LidarMapFile& m,
                                 double lat,
                                 double lon,
                                 double tol_deg = 1e-6)
{
  return std::fabs(m.datum_lat - lat) <= tol_deg && std::fabs(m.datum_lon - lon) <= tol_deg;
}

}  // namespace fusion_graph
