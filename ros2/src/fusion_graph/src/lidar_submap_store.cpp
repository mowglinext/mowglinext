// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "fusion_graph/lidar_submap_store.hpp"

#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

#include "fusion_graph/lidar_map_file.hpp"

namespace fusion_graph
{
namespace
{
constexpr std::array<char, 8> kMagic = {'M', 'W', 'L', 'T', 'I', 'L', 'E', '1'};
// Explicit fields (no struct padding). This format uses native little-endian
// IEEE754, shared by supported ARM64 and x86_64 deployments.
constexpr std::uintmax_t kHeaderBytes = 8 + 4 + 4 + 4 + 8 * 4;
template <typename T>
void Write(std::ostream& out, const T& value)
{
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
}
template <typename T>
T Read(std::istream& in)
{
  T value{};
  in.read(reinterpret_cast<char*>(&value), sizeof(value));
  if (!in)
    throw std::runtime_error("truncated LiDAR tile");
  return value;
}
bool Close(double a, double b, double tolerance = 1e-9)
{
  return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= tolerance;
}
}  // namespace

LidarSubmapStore::LidarSubmapStore(const LidarOccupancyMapperParams& params,
                                   std::string prefix,
                                   double lat,
                                   double lon,
                                   double tile_size,
                                   int radius)
    : params_(params),
      directory_(prefix + ".lidartiles"),
      legacy_path_(prefix + ".lidarmap"),
      datum_lat_(lat),
      datum_lon_(lon),
      tile_size_m_(tile_size),
      radius_(radius)
{
  if (directory_ == ".lidartiles")
    throw std::invalid_argument("empty LiDAR storage prefix");
  const double cells = tile_size / params.resolution_m;
  if (!std::isfinite(params.resolution_m) || params.resolution_m <= 0 || !std::isfinite(cells) ||
      cells < 1 || cells > 2048 || std::abs(cells - std::round(cells)) > 1e-6 || radius < 1 ||
      radius > 8 || !std::isfinite(tile_size) || tile_size <= 0 || !std::isfinite(lat) ||
      !std::isfinite(lon) || std::abs(lat) > 90 || std::abs(lon) > 180 ||
      !std::isfinite(params.log_odds_min) || !std::isfinite(params.log_odds_max) ||
      params.log_odds_min >= 0 || params.log_odds_max <= 0 || !std::isfinite(params.log_odds_hit) ||
      params.log_odds_hit <= 0 || params.log_odds_hit > params.log_odds_max ||
      !std::isfinite(params.log_odds_miss) || params.log_odds_miss >= 0 ||
      params.log_odds_miss < params.log_odds_min || !std::isfinite(params.occupied_threshold) ||
      params.occupied_threshold <= 0 || params.occupied_threshold > params.log_odds_max ||
      !std::isfinite(params.free_threshold) || params.free_threshold >= 0 ||
      params.free_threshold < params.log_odds_min || !std::isfinite(params.max_range_m) ||
      params.max_range_m <= 0)
    throw std::invalid_argument("invalid LiDAR submap geometry or datum");
  tile_cells_ = static_cast<std::size_t>(std::llround(cells));
  window_cells_ = tile_cells_ * static_cast<std::size_t>(2 * radius + 1);
  if (window_cells_ > 2048 || !std::isfinite(window_cells_ * params_.resolution_m))
    throw std::invalid_argument("LiDAR submap window exceeds RAM limit");
  params_.half_extent_m = window_cells_ * params_.resolution_m / 2.0;
}
LidarSubmapStore::~LidarSubmapStore()
{
  // Joining is required because the background task uses immutable store geometry.
  if (pending_.valid())
    pending_.wait();
  if (clear_requested_)
  {
    std::error_code error;
    std::filesystem::remove_all(directory_, error);
    std::filesystem::remove(legacy_path_, error);
  }
}
LidarOccupancyMapper* LidarSubmapStore::mapper()
{
  return busy() ? nullptr : mapper_.get();
}
bool LidarSubmapStore::busy() const
{
  return pending_.valid() || clear_requested_;
}
bool LidarSubmapStore::TakeWindowChanged()
{
  return std::exchange(changed_, false);
}
void LidarSubmapStore::Poll()
{
  if (pending_.valid() && pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    auto result = pending_.get();
    last_error_ = result.error;
    if (!result.error.empty())
      retry_after_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    if (result.clearing && !result.error.empty())
      clear_requested_ = true;
    if (!clear_requested_ && result.map)
    {
      mapper_ = std::move(result.map);
      center_x_ = result.x;
      center_y_ = result.y;
      changed_ = changed_ || result.changed;
    }
  }
  if (clear_requested_ && !pending_.valid() && std::chrono::steady_clock::now() >= retry_after_)
    StartClear();
}
bool LidarSubmapStore::UpdateWindow(double x, double y)
{
  Poll();
  if (busy())
    return false;
  const double tx = std::floor(x / tile_size_m_), ty = std::floor(y / tile_size_m_);
  constexpr double limit = std::numeric_limits<int>::max() / 2;
  if (!std::isfinite(tx) || !std::isfinite(ty) || std::abs(tx) > limit || std::abs(ty) > limit)
  {
    last_error_ = "non-finite or out-of-range LiDAR window position";
    return false;
  }
  const int cx = static_cast<int>(tx), cy = static_cast<int>(ty);
  // Retain the current window across small boundary crossings. The margin is
  // bounded to 10% of a tile (at most 1 m), leaving >=19 m around the robot
  // with default geometry, comfortably beyond the default 12 m scan range.
  const double hysteresis = std::min(tile_size_m_ * 0.1, 1.0);
  if (mapper_ && x >= center_x_ * tile_size_m_ - hysteresis &&
      x < (center_x_ + 1) * tile_size_m_ + hysteresis &&
      y >= center_y_ * tile_size_m_ - hysteresis && y < (center_y_ + 1) * tile_size_m_ + hysteresis)
    return true;
  if (std::chrono::steady_clock::now() < retry_after_)
    return false;
  auto old = std::move(mapper_);
  const int ox = center_x_, oy = center_y_;
  pending_ = std::async(std::launch::async,
                        [this, old = std::move(old), ox, oy, cx, cy]() mutable
                        {
                          Result result;
                          try
                          {
                            if (old)
                              SaveWindow(*old, ox, oy);
                            MigrateLegacy();
                            result.map = LoadWindow(cx, cy);
                            result.x = cx;
                            result.y = cy;
                            result.changed = true;
                          }
                          catch (const std::exception& error)
                          {
                            result.map = std::move(old);
                            result.x = ox;
                            result.y = oy;
                            result.error = error.what();
                          }
                          return result;
                        });
  return false;
}
void LidarSubmapStore::RequestSave()
{
  Poll();
  if (busy() || !mapper_ || std::chrono::steady_clock::now() < retry_after_)
    return;
  auto map = std::move(mapper_);
  const int x = center_x_, y = center_y_;
  pending_ = std::async(std::launch::async,
                        [this, map = std::move(map), x, y]() mutable
                        {
                          Result result;
                          try
                          {
                            SaveWindow(*map, x, y);
                          }
                          catch (const std::exception& error)
                          {
                            result.error = error.what();
                          }
                          result.map = std::move(map);
                          result.x = x;
                          result.y = y;
                          return result;
                        });
}
void LidarSubmapStore::Clear()
{
  mapper_.reset();
  changed_ = true;
  clear_requested_ = true;
  retry_after_ = {};
  Poll();
}
void LidarSubmapStore::StartClear()
{
  clear_requested_ = false;
  pending_ = std::async(std::launch::async,
                        [this]
                        {
                          Result result;
                          result.clearing = true;
                          try
                          {
                            std::filesystem::remove_all(directory_);
                            std::filesystem::remove(legacy_path_);
                          }
                          catch (const std::exception& error)
                          {
                            result.error = error.what();
                          }
                          return result;
                        });
}
std::string LidarSubmapStore::TilePath(int x, int y) const
{
  return directory_ + "/" + std::to_string(x) + "_" + std::to_string(y) + ".tile";
}
void LidarSubmapStore::ValidateMetadata(bool create) const
{
  const auto path = directory_ + "/manifest";
  if (std::filesystem::exists(path))
  {
    if (std::filesystem::file_size(path) != 8 + 4 + 8 * 4)
      throw std::runtime_error("invalid LiDAR submap manifest length");
    std::ifstream in(path, std::ios::binary);
    std::array<char, 8> magic{};
    in.read(magic.data(), magic.size());
    const auto cells = Read<std::uint32_t>(in);
    const double resolution = Read<double>(in), size = Read<double>(in);
    const double lat = Read<double>(in), lon = Read<double>(in);
    if (magic != kMagic || cells != tile_cells_ || !Close(resolution, params_.resolution_m) ||
        !Close(size, tile_size_m_) || !Close(lat, datum_lat_) || !Close(lon, datum_lon_))
      throw std::runtime_error("LiDAR submap geometry or datum mismatch");
    return;
  }
  // A missing manifest in a nonempty tile directory is not a fresh map.
  if (std::filesystem::exists(directory_))
    for (const auto& entry : std::filesystem::directory_iterator(directory_))
      if (entry.path().extension() == ".tile")
        throw std::runtime_error("LiDAR submap manifest missing");
  if (!create)
    return;
  std::filesystem::create_directories(directory_);
  std::ofstream out(path + ".tmp", std::ios::binary | std::ios::trunc);
  out.write(kMagic.data(), kMagic.size());
  Write(out, static_cast<std::uint32_t>(tile_cells_));
  Write(out, params_.resolution_m);
  Write(out, tile_size_m_);
  Write(out, datum_lat_);
  Write(out, datum_lon_);
  out.flush();
  if (!out)
    throw std::runtime_error("cannot write LiDAR submap manifest");
  out.close();
  if (!out)
    throw std::runtime_error("cannot close LiDAR submap manifest");
  std::filesystem::rename(path + ".tmp", path);
}
void LidarSubmapStore::WriteTile(
    const LidarOccupancyMapper& map, int tx, int ty, std::size_t ox, std::size_t oy) const
{
  bool touched = false;
  for (std::size_t y = 0; y < tile_cells_ && !touched; ++y)
    for (std::size_t x = 0; x < tile_cells_; ++x)
      if (map.EvidenceAt(ox + x, oy + y).touched)
      {
        touched = true;
        break;
      }
  if (!touched)
    return;  // Unknown tiles require no disk or directory entry.
  ValidateMetadata(true);
  const auto path = TilePath(tx, ty), temporary = path + ".tmp";
  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out)
    throw std::runtime_error("cannot create LiDAR tile");
  out.write(kMagic.data(), kMagic.size());
  Write(out, static_cast<std::uint32_t>(tile_cells_));
  Write(out, static_cast<std::int32_t>(tx));
  Write(out, static_cast<std::int32_t>(ty));
  Write(out, params_.resolution_m);
  Write(out, tile_size_m_);
  Write(out, datum_lat_);
  Write(out, datum_lon_);
  for (std::size_t y = 0; y < tile_cells_; ++y)
    for (std::size_t x = 0; x < tile_cells_; ++x)
    {
      const auto evidence = map.EvidenceAt(ox + x, oy + y);
      Write(out, evidence.log_odds);
      Write(out, evidence.touched);
    }
  out.flush();
  if (!out)
    throw std::runtime_error("cannot write LiDAR tile");
  out.close();
  if (!out)
    throw std::runtime_error("cannot close LiDAR tile");
  std::filesystem::rename(temporary, path);
}
void LidarSubmapStore::ReadTile(
    LidarOccupancyMapper& map, int tx, int ty, std::size_t ox, std::size_t oy) const
{
  const auto path = TilePath(tx, ty);
  if (!std::filesystem::exists(path))
    return;
  if (std::filesystem::file_size(path) != kHeaderBytes + tile_cells_ * tile_cells_ * 9)
    throw std::runtime_error("invalid LiDAR tile length");
  std::ifstream in(path, std::ios::binary);
  std::array<char, 8> magic{};
  in.read(magic.data(), magic.size());
  const auto cells = Read<std::uint32_t>(in);
  const auto file_x = Read<std::int32_t>(in), file_y = Read<std::int32_t>(in);
  const double resolution = Read<double>(in), size = Read<double>(in);
  const double lat = Read<double>(in), lon = Read<double>(in);
  if (magic != kMagic || cells != tile_cells_ || file_x != tx || file_y != ty ||
      !Close(resolution, params_.resolution_m) || !Close(size, tile_size_m_) ||
      !Close(lat, datum_lat_) || !Close(lon, datum_lon_))
    throw std::runtime_error("LiDAR tile geometry or datum mismatch");
  for (std::size_t y = 0; y < tile_cells_; ++y)
    for (std::size_t x = 0; x < tile_cells_; ++x)
    {
      const auto odds = Read<double>(in);
      const auto touched = Read<std::uint8_t>(in);
      if (!map.SetEvidence(ox + x, oy + y, {odds, touched}))
        throw std::runtime_error("invalid LiDAR tile evidence");
    }
}
void LidarSubmapStore::SaveWindow(const LidarOccupancyMapper& map, int cx, int cy) const
{
  for (int dy = -radius_; dy <= radius_; ++dy)
    for (int dx = -radius_; dx <= radius_; ++dx)
      WriteTile(map, cx + dx, cy + dy, (dx + radius_) * tile_cells_, (dy + radius_) * tile_cells_);
}
std::unique_ptr<LidarOccupancyMapper> LidarSubmapStore::LoadWindow(int cx, int cy) const
{
  ValidateMetadata(false);
  auto map = std::make_unique<LidarOccupancyMapper>(params_,
                                                    (cx - radius_) * tile_size_m_,
                                                    (cy - radius_) * tile_size_m_);
  for (int dy = -radius_; dy <= radius_; ++dy)
    for (int dx = -radius_; dx <= radius_; ++dx)
      ReadTile(*map, cx + dx, cy + dy, (dx + radius_) * tile_cells_, (dy + radius_) * tile_cells_);
  return map;
}
void LidarSubmapStore::MigrateLegacy() const
{
  if (!std::filesystem::exists(legacy_path_))
    return;
  // Bound allocation using the actual on-disk length before reading dimensions.
  const auto bytes = std::filesystem::file_size(legacy_path_);
  if (bytes < 64 || bytes > 64 + 4096 * 4096)
    throw std::runtime_error("invalid legacy LiDAR map length");
  std::ifstream in(legacy_path_, std::ios::binary);
  std::array<char, 8> magic{};
  in.read(magic.data(), magic.size());
  LidarMapFile legacy;
  legacy.datum_lat = Read<double>(in);
  legacy.datum_lon = Read<double>(in);
  legacy.grid.resolution_m = Read<double>(in);
  legacy.grid.origin_x = Read<double>(in);
  legacy.grid.origin_y = Read<double>(in);
  const auto width = Read<uint64_t>(in), height = Read<uint64_t>(in);
  if (!std::equal(magic.begin(), magic.end(), kLidarMapMagic) || width == 0 || height == 0 ||
      width > 4096 || height > 4096 || bytes != 64 + width * height ||
      !Close(legacy.datum_lat, datum_lat_) || !Close(legacy.datum_lon, datum_lon_))
    throw std::runtime_error("invalid legacy LiDAR map geometry or datum");
  legacy.grid.width = width;
  legacy.grid.height = height;
  legacy.grid.data.resize(width * height);
  in.read(reinterpret_cast<char*>(legacy.grid.data.data()), legacy.grid.data.size());
  if (!in || !ValidLegacy(legacy.grid))
    throw std::runtime_error("invalid legacy LiDAR map cells");
  ValidateMetadata(true);
  ImportGrid(legacy.grid);
  // Rename only after every tile is saved. A crash halfway through is resumable:
  // on restart ImportGrid keeps existing evidence and fills missing cells.
  std::filesystem::rename(legacy_path_, legacy_path_ + ".migrated");
}
bool LidarSubmapStore::ValidLegacy(const ExportedOccupancyGrid& grid) const
{
  if (!std::isfinite(grid.resolution_m) || grid.resolution_m <= 0 ||
      std::abs(grid.resolution_m / params_.resolution_m - 1.0) > 1e-6 ||
      !Close(grid.resolution_m, params_.resolution_m, 1e-6) || !std::isfinite(grid.origin_x) ||
      !std::isfinite(grid.origin_y) || grid.width == 0 || grid.height == 0 || grid.width > 4096 ||
      grid.height > 4096 || grid.data.size() != grid.width * grid.height ||
      std::abs(grid.origin_x) > 1e7 || std::abs(grid.origin_y) > 1e7)
    return false;
  // Check every tile bound before any floating-point-to-integer conversion.
  constexpr double limit = std::numeric_limits<int>::max() / 2;
  for (const double index :
       {grid.origin_x / tile_size_m_,
        grid.origin_y / tile_size_m_,
        (grid.origin_x + (grid.width - 0.5) * grid.resolution_m) / tile_size_m_,
        (grid.origin_y + (grid.height - 0.5) * grid.resolution_m) / tile_size_m_})
    if (!std::isfinite(index) || std::abs(std::floor(index)) > limit)
      return false;
  if (std::any_of(grid.data.begin(),
                  grid.data.end(),
                  [](int8_t value)
                  {
                    return value < -1 || value > 100;
                  }))
    return false;
  // No silent half-cell shifts when converting an external raster to tile axes.
  const auto aligned = [this](double v)
  {
    return std::abs(v / params_.resolution_m - std::round(v / params_.resolution_m)) < 1e-6;
  };
  if (!aligned(grid.origin_x) || !aligned(grid.origin_y))
    return false;
  return true;
}
void LidarSubmapStore::ImportGrid(const ExportedOccupancyGrid& grid) const
{
  const int min_x = static_cast<int>(std::floor(grid.origin_x / tile_size_m_));
  const int min_y = static_cast<int>(std::floor(grid.origin_y / tile_size_m_));
  const int max_x = static_cast<int>(
      std::floor((grid.origin_x + (grid.width - 0.5) * grid.resolution_m) / tile_size_m_));
  const int max_y = static_cast<int>(
      std::floor((grid.origin_y + (grid.height - 0.5) * grid.resolution_m) / tile_size_m_));
  auto tile_params = params_;
  tile_params.half_extent_m = tile_size_m_ / 2;
  for (int y = min_y; y <= max_y; ++y)
    for (int x = min_x; x <= max_x; ++x)
    {
      LidarOccupancyMapper tile(tile_params, x * tile_size_m_, y * tile_size_m_);
      ReadTile(tile, x, y, 0, 0);
      // Existing evidence has priority over the lossy trinary legacy file.
      const auto start_x = static_cast<long long>(
          std::llround((x * tile_size_m_ - grid.origin_x) / grid.resolution_m));
      const auto start_y = static_cast<long long>(
          std::llround((y * tile_size_m_ - grid.origin_y) / grid.resolution_m));
      for (std::size_t row = 0; row < tile_cells_; ++row)
        for (std::size_t col = 0; col < tile_cells_; ++col)
        {
          const auto gx = start_x + static_cast<long long>(col),
                     gy = start_y + static_cast<long long>(row);
          if (gx < 0 || gy < 0 || gx >= static_cast<long long>(grid.width) ||
              gy >= static_cast<long long>(grid.height) || tile.EvidenceAt(col, row).touched)
            continue;
          const auto value = grid.data[gy * grid.width + gx];
          if (value >= 0)
            tile.SetEvidence(
                col, row, {value >= 50 ? params_.occupied_threshold : params_.free_threshold, 1});
        }
      WriteTile(tile, x, y, 0, 0);
    }
}
bool LidarSubmapStore::ImportLegacy(const ExportedOccupancyGrid& grid)
{
  Poll();
  if (busy() || std::chrono::steady_clock::now() < retry_after_ || !ValidLegacy(grid))
    return false;
  auto old = std::move(mapper_);
  const int cx = center_x_, cy = center_y_;
  pending_ = std::async(std::launch::async,
                        [this, grid, old = std::move(old), cx, cy]() mutable
                        {
                          Result result;
                          try
                          {
                            ValidateMetadata(true);
                            if (old)
                              SaveWindow(*old, cx, cy);
                            ImportGrid(grid);
                            result.map = LoadWindow(cx, cy);
                            result.changed = true;
                          }
                          catch (const std::exception& error)
                          {
                            result.map = std::move(old);
                            result.error = error.what();
                          }
                          result.x = cx;
                          result.y = cy;
                          return result;
                        });
  return true;
}
}  // namespace fusion_graph
