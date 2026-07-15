// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// scan_filters.hpp — pure LaserScan filter helpers shared by
// costmap_scan_filter_node and its unit tests (test_costmap_scan_filter).
// Header-only, no rclcpp: the tests used to carry hand-mirrored copies of
// these functions "kept in sync" with the node — this header is the
// refactor that note asked for once a third filter pass appeared (the
// sector-limited dock blank).
//
// Filters:
//   1. filter_scan — chassis self-return blank (radial, always-on) +
//      dock blank (radial range, but SECTOR-LIMITED in bearing while
//      charging / post-undock). The dock/canopy sits FORWARD of a
//      nose-in-docked robot, so only the forward sector needs blanking;
//      keeping the REAR beams live is what lets the undock BackUp's
//      collision check (behavior_server sim-ahead on local_costmap) and
//      the calibration drive's rear guard see a real obstacle behind the
//      robot during the blank window. dock_blank_sector_rad = 2π restores
//      the legacy full-circle blank (per-site escape hatch).
//   2. apply_ground_filter — gravity-aware, slope-tolerant ground strip
//      (see costmap_scan_filter_node.cpp header comment for the physics).

#ifndef MOWGLI_LOCALIZATION__SCAN_FILTERS_HPP_
#define MOWGLI_LOCALIZATION__SCAN_FILTERS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

/// Wrap an angle to (-π, π].
inline double wrap_to_pi(double a)
{
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a <= -M_PI)
    a += 2.0 * M_PI;
  return a;
}

/// Three-component vector — the gravity-aligned "up" direction expressed
/// in the IMU/base_link frame.
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{1.0};
};

/// Ground-filter parameters bundled so tests can call the pure filter
/// without a node.
struct GroundFilterConfig
{
  bool enabled{false};
  double min_obstacle_z_m{0.08};
  double max_obstacle_z_m{1.5};
  double lidar_height_m{0.22};
  /// Yaw of the LIDAR frame relative to base_link/IMU (rad). A beam at
  /// LIDAR index angle α points along base bearing α + lidar_mount_yaw.
  double lidar_mount_yaw{0.0};
  /// SAFETY: minimum run of consecutive ground-classified beams before any of
  /// them is stripped as ground. A 2-D LiDAR can't tell a real vertical
  /// obstacle from sloped ground at the same bearing/range — on a downslope a
  /// leg/trunk/child projects BELOW min_obstacle_z and would be discarded. But
  /// ground returns form LONG contiguous angular arcs while an obstacle
  /// subtends only a few beams, so we only strip a "ground" return when it is
  /// part of a run >= this length. A short ground-classified cluster is kept
  /// (treated as a possible obstacle) — the planner/CostCritic then avoids it.
  /// 0 disables the guard (legacy per-beam stripping).
  int min_ground_run{8};
};

/// Chassis + (sector-limited) dock blank applied to a copy of @p in.
///
///   - Chassis blank: any finite return < chassis_blank_range → +inf,
///     at every bearing (self-returns come from all around the housing).
///   - Dock blank (only when @p dock_active): finite return <
///     dock_blank_range AND base bearing ψ = angle(i) + lidar_mount_yaw
///     within ±dock_blank_sector_rad/2 of 0 (robot forward) → +inf.
///     A sector of 2π (360°) blanks every bearing — exact legacy
///     behaviour.
///
/// `dock_active` is the cached output of the charging/post-undock state
/// machine — passed in so tests can drive it without a clock.
inline sensor_msgs::msg::LaserScan filter_scan(const sensor_msgs::msg::LaserScan& in,
                                               double chassis_blank_range,
                                               double dock_blank_range,
                                               bool dock_active,
                                               double dock_blank_sector_rad,
                                               double lidar_mount_yaw)
{
  sensor_msgs::msg::LaserScan out = in;
  const bool chassis_on = chassis_blank_range > 0.0;
  const bool dock_on = dock_active && dock_blank_range > 0.0;
  if (!chassis_on && !dock_on)
    return out;
  const float chassis_thr = static_cast<float>(chassis_blank_range);
  const float dock_thr = static_cast<float>(dock_blank_range);
  const double half_sector = std::max(0.0, dock_blank_sector_rad) * 0.5;
  const float inf = std::numeric_limits<float>::infinity();
  const double a0 = out.angle_min;
  const double da = out.angle_increment;
  for (std::size_t i = 0; i < out.ranges.size(); ++i)
  {
    float& r = out.ranges[i];
    if (!std::isfinite(r))
      continue;
    if (chassis_on && r < chassis_thr)
    {
      r = inf;
      continue;
    }
    if (dock_on && r < dock_thr)
    {
      const double psi = wrap_to_pi(a0 + da * static_cast<double>(i) + lidar_mount_yaw);
      if (std::fabs(psi) <= half_sector)
        r = inf;
    }
  }
  return out;
}

/// Apply the gravity-aware ground filter to @p io in place. For each
/// beam at LIDAR index angle α, rotate into the base/IMU frame by the
/// LIDAR mount yaw (ψ = α + lidar_mount_yaw) before projecting onto the
/// IMU's measured "up" unit vector:
///
///     ψ        = α + cfg.lidar_mount_yaw
///     z_dir    = up_in_imu.x · cos ψ + up_in_imu.y · sin ψ
///     return_Z = lidar_height + range · z_dir
///
/// where up_in_imu = accel / |accel| (the gravity reaction direction).
/// Returns whose Z is outside [min_obstacle_z_m, max_obstacle_z_m] get
/// pushed to +inf so obstacle_layer ignores them.
///
/// `up_in_imu` is std::nullopt when no fresh sample exists (or the
/// filter is disabled). In that case the function is a no-op — better
/// to publish phantom obstacles than to silently strip real ones.
inline void apply_ground_filter(sensor_msgs::msg::LaserScan& io,
                                const GroundFilterConfig& cfg,
                                const std::optional<Vec3>& up_in_imu)
{
  if (!cfg.enabled || !up_in_imu.has_value())
    return;
  const Vec3& u = *up_in_imu;
  const float min_z = static_cast<float>(cfg.min_obstacle_z_m);
  const float max_z = static_cast<float>(cfg.max_obstacle_z_m);
  const float inf = std::numeric_limits<float>::infinity();
  const double a0 = io.angle_min;
  const double da = io.angle_increment;
  const std::size_t n = io.ranges.size();

  // Pass 1: classify each finite beam. 0 = keep, 1 = ground (projects below
  // min_z), 2 = overhead (above max_z). The overhead strip is per-beam (a
  // canopy/overhang return is genuinely above the robot and safe to drop); the
  // GROUND strip is gated below by a run-length test so a real obstacle that a
  // downslope mis-projects below min_z is not silently discarded.
  std::vector<uint8_t> klass(n, 0);
  for (std::size_t i = 0; i < n; ++i)
  {
    const float r = io.ranges[i];
    if (!std::isfinite(r))
      continue;
    const double psi = a0 + da * static_cast<double>(i) + cfg.lidar_mount_yaw;
    const double z_dir = u.x * std::cos(psi) + u.y * std::sin(psi);
    const float return_z = static_cast<float>(cfg.lidar_height_m + r * z_dir);
    if (return_z > max_z)
      klass[i] = 2;
    else if (return_z < min_z)
      klass[i] = 1;
  }

  // Pass 2: strip overhead returns outright; strip ground returns only where
  // they form a contiguous run >= min_ground_run (a long sweep of ground),
  // leaving short ground-classified clusters as possible obstacles.
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
    // Extent of the contiguous ground run containing i.
    std::size_t j = i;
    while (j < n && klass[j] == 1)
      ++j;
    const std::size_t run_len = j - i;
    if (static_cast<int>(run_len) >= min_run)
    {
      for (std::size_t k = i; k < j; ++k)
        io.ranges[k] = inf;
    }
    // else: keep the short cluster (possible obstacle the slope mis-projected).
    i = j - 1;  // skip past the run we just handled
  }
}

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__SCAN_FILTERS_HPP_
