// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
/// @file scan_ground_filter.hpp
/// @brief Pure LiDAR ground filter of costmap_scan_filter_node (shared with its tests).
///
/// A 2-D LiDAR beam whose endpoint projects below `min_obstacle_z_m` is a
/// candidate ground return, and long contiguous runs of them are stripped from
/// /scan_costmap. The projection needs to know how the scan plane is tilted
/// relative to the ground ahead.
///
/// Gravity alone (the IMU "up" vector, ~50 ms low-pass) answers that only on
/// level ground. On a uniform slope the terrain is parallel to the wheels, so
/// beams stay level relative to it, yet gravity says every downhill beam
/// descends: with lidar_height 0.20 m and min_obstacle_z 0.15 m, an 8° slope
/// classified every downhill return beyond 0.36 m as ground, and a tree trunk
/// 0.7 m behind the robot (~14 beams, more than min_ground_run) vanished from
/// the costmap (field 2026-11-09; collision_monitor's stream is not
/// ground-filtered and still saw it).
///
/// The terrain reference is the same up vector averaged over
/// `terrain_tilt_time_constant_s`, i.e. the slope the robot is standing on. A
/// return is stripped only when BOTH references put it below the floor: a bump
/// or a fresh pitch (fast) still strips the ground it causes; a sustained slope
/// no longer strips what stays level with the terrain. Deliberate trade-off:
/// after a long dwell on a ramp facing flat ground, those ground hits come back
/// as obstacles — a phantom obstacle is safer than a missed tree.
#ifndef MOWGLI_LOCALIZATION__SCAN_GROUND_FILTER_HPP_
#define MOWGLI_LOCALIZATION__SCAN_GROUND_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

/// Unit vector: the gravity-reaction ("up") direction in the IMU/base frame.
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{1.0};
};

struct GroundFilterConfig
{
  bool enabled{false};
  double min_obstacle_z_m{0.08};
  double max_obstacle_z_m{1.5};
  double lidar_height_m{0.30};
  /// Yaw of the LIDAR frame relative to base_link/IMU (rad): a beam at LIDAR
  /// index angle α points along base bearing α + lidar_mount_yaw.
  double lidar_mount_yaw{0.0};
  /// SAFETY: minimum run of consecutive ground-classified beams before any is
  /// stripped (ground forms long arcs; an obstacle usually subtends few beams).
  /// 0 disables the guard (per-beam stripping).
  int min_ground_run{8};
};

/// Return height above the reference plane of the beam at @p psi (base bearing).
inline double ground_filter_return_z(const Vec3& up, double lidar_height_m, double psi, double r)
{
  return lidar_height_m + r * (up.x * std::cos(psi) + up.y * std::sin(psi));
}

/// Apply the ground filter to @p io in place (returns pushed to +inf).
///
/// `up` is gravity; std::nullopt (no fresh IMU) or a disabled config is a
/// no-op — never strip obstacles when the attitude is unknown. `terrain_up`,
/// when present, is the up direction relative to the terrain the robot stands
/// on (TerrainTiltTracker::relative): a return is ground only if it is below
/// the floor under BOTH references. Overhead stripping (above max_obstacle_z)
/// stays gravity-only, per beam.
inline void apply_ground_filter(sensor_msgs::msg::LaserScan& io,
                                const GroundFilterConfig& cfg,
                                const std::optional<Vec3>& up,
                                const std::optional<Vec3>& terrain_up = std::nullopt)
{
  if (!cfg.enabled || !up.has_value())
    return;
  const double min_z = cfg.min_obstacle_z_m;
  const double max_z = cfg.max_obstacle_z_m;
  const float inf = std::numeric_limits<float>::infinity();
  const std::size_t n = io.ranges.size();

  // Pass 1: 0 = keep, 1 = ground (below the floor), 2 = overhead.
  std::vector<uint8_t> klass(n, 0);
  for (std::size_t i = 0; i < n; ++i)
  {
    const float r = io.ranges[i];
    if (!std::isfinite(r))
      continue;
    const double psi =
        io.angle_min + io.angle_increment * static_cast<double>(i) + cfg.lidar_mount_yaw;
    const double gravity_z = ground_filter_return_z(*up, cfg.lidar_height_m, psi, r);
    if (gravity_z > max_z)
    {
      klass[i] = 2;
      continue;
    }
    if (gravity_z >= min_z)
      continue;
    if (terrain_up.has_value() &&
        ground_filter_return_z(*terrain_up, cfg.lidar_height_m, psi, r) >= min_z)
      continue;  // level with the terrain it stands on: not ground
    klass[i] = 1;
  }

  // Pass 2: strip overhead per beam; strip ground only in runs >= min_ground_run.
  const int min_run = std::max(1, cfg.min_ground_run);
  for (std::size_t i = 0; i < n; ++i)
  {
    if (klass[i] == 2)
    {
      io.ranges[i] = inf;
      continue;
    }
    if (klass[i] != 1)
      continue;
    std::size_t j = i;
    while (j < n && klass[j] == 1)
      ++j;
    if (static_cast<int>(j - i) >= min_run)
    {
      for (std::size_t k = i; k < j; ++k)
        io.ranges[k] = inf;
    }
    i = j - 1;
  }
}

/// Slow average of the up vector (the terrain's tilt) and the current up
/// vector expressed relative to it. A time constant <= 0 disables the terrain
/// reference: relative() then returns std::nullopt (gravity-only filter).
class TerrainTiltTracker
{
public:
  explicit TerrainTiltTracker(double time_constant_s = 0.0) : time_constant_s_(time_constant_s)
  {
  }

  void update(const Vec3& up, double t_s)
  {
    if (!(time_constant_s_ > 0.0) || !std::isfinite(t_s) || !finite(up))
      return;
    if (!have_ || !(t_s > last_t_) || t_s - last_t_ > kMaxGapS)
    {
      // First sample, a clock step, or a gap (stale IMU): start over from it.
      slow_ = normalized(up);
      have_ = true;
      last_t_ = t_s;
      return;
    }
    const double alpha = 1.0 - std::exp(-(t_s - last_t_) / time_constant_s_);
    slow_ = normalized(Vec3{slow_.x + alpha * (up.x - slow_.x),
                            slow_.y + alpha * (up.y - slow_.y),
                            slow_.z + alpha * (up.z - slow_.z)});
    last_t_ = t_s;
  }

  /// @p up rotated by the rotation that takes the terrain up to +Z.
  std::optional<Vec3> relative(const Vec3& up) const
  {
    if (!(time_constant_s_ > 0.0) || !have_ || !finite(up))
      return std::nullopt;
    // Rodrigues: axis k = slow × ẑ, sin θ = |k|, cos θ = slow·ẑ.
    const Vec3 s = slow_;
    const double kx = s.y;
    const double ky = -s.x;
    const double sin_t = std::hypot(kx, ky);
    const double cos_t = s.z;
    if (sin_t < 1e-12)
      return normalized(up);
    const double ux = kx / sin_t;
    const double uy = ky / sin_t;  // unit axis (ux, uy, 0)
    const double dot = ux * up.x + uy * up.y;
    const Vec3 cross{uy * up.z, -ux * up.z, ux * up.y - uy * up.x};
    return normalized(Vec3{up.x * cos_t + cross.x * sin_t + ux * dot * (1.0 - cos_t),
                           up.y * cos_t + cross.y * sin_t + uy * dot * (1.0 - cos_t),
                           up.z * cos_t + cross.z * sin_t});
  }

  std::optional<Vec3> terrain() const
  {
    return have_ ? std::optional<Vec3>(slow_) : std::nullopt;
  }

private:
  static constexpr double kMaxGapS = 1.0;

  static bool finite(const Vec3& v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  static Vec3 normalized(const Vec3& v)
  {
    const double m = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return m > 1e-12 ? Vec3{v.x / m, v.y / m, v.z / m} : Vec3{};
  }

  double time_constant_s_;
  bool have_{false};
  double last_t_{0.0};
  Vec3 slow_{};
};

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__SCAN_GROUND_FILTER_HPP_
