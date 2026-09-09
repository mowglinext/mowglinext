// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "fusion_graph/lidar_occupancy_mapper.hpp"

namespace fusion_graph
{
// Executor-owned interface. File I/O runs on one background task; call UpdateWindow
// regularly to collect completion. No mapper access is granted during a task, so
// a saved snapshot cannot overwrite newer evidence. RAM is bounded by two windows.
class LidarSubmapStore
{
public:
  LidarSubmapStore(const LidarOccupancyMapperParams& params,
                   std::string path_prefix,
                   double datum_lat,
                   double datum_lon,
                   double tile_size_m = 10.0,
                   int radius_tiles = 2);
  ~LidarSubmapStore();
  LidarSubmapStore(const LidarSubmapStore&) = delete;
  LidarSubmapStore& operator=(const LidarSubmapStore&) = delete;

  bool UpdateWindow(double x, double y);
  LidarOccupancyMapper* mapper();
  bool busy() const;
  bool TakeWindowChanged();
  void RequestSave();
  void Clear();
  // Caller validates the legacy file's datum. Imports all source tiles, including
  // those outside the active window, without allocating a global dense map.
  bool ImportLegacy(const ExportedOccupancyGrid& grid);
  const std::string& last_error() const
  {
    return last_error_;
  }
  std::size_t window_cells() const
  {
    return window_cells_;
  }

private:
  struct Result
  {
    std::unique_ptr<LidarOccupancyMapper> map;
    int x = 0;
    int y = 0;
    bool changed = false;
    bool clearing = false;
    std::string error;
  };
  bool ValidLegacy(const ExportedOccupancyGrid& grid) const;
  void ImportGrid(const ExportedOccupancyGrid& grid) const;
  void MigrateLegacy() const;
  void ValidateMetadata(bool create) const;
  void Poll();
  void StartClear();
  std::unique_ptr<LidarOccupancyMapper> LoadWindow(int x, int y) const;
  void SaveWindow(const LidarOccupancyMapper& map, int x, int y) const;
  void WriteTile(const LidarOccupancyMapper& map,
                 int tx,
                 int ty,
                 std::size_t offset_x,
                 std::size_t offset_y) const;
  void ReadTile(
      LidarOccupancyMapper& map, int tx, int ty, std::size_t offset_x, std::size_t offset_y) const;
  std::string TilePath(int x, int y) const;

  LidarOccupancyMapperParams params_;
  std::string directory_;
  std::string legacy_path_;
  double datum_lat_;
  double datum_lon_;
  double tile_size_m_;
  int radius_;
  std::size_t tile_cells_;
  std::size_t window_cells_;
  std::unique_ptr<LidarOccupancyMapper> mapper_;
  std::future<Result> pending_;
  int center_x_ = 0;
  int center_y_ = 0;
  bool changed_ = false;
  bool clear_requested_ = false;
  std::string last_error_;
  std::chrono::steady_clock::time_point retry_after_{};
};
}  // namespace fusion_graph
