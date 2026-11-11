// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
/**
 * @file dock_approach_refinement.hpp
 * @brief Final-approach refinement for gps_dock_detection: a heading taken from
 *        the GPS TRACK, and a target that stops moving near the cradle.
 *
 * Why. The detection is the dock contact pose expressed in odom. Its HEADING is
 * `dock_pose_yaw − Δ`, Δ being the map→odom yaw. Taken from the map→odom TF, Δ
 * carries the fused heading error of the moment, and during the approach that
 * heading is gyro-only (fusion_graph drops COG then, gate_cog_during_docking),
 * right after the pivot at the staging pose. An error ε rotates the line the
 * graceful controller converges to: the robot reaches the right point with its
 * heading ε off, and the front of the chassis (≈0.5 m ahead of base_footprint,
 * the rear axle) lands 0.5·ε to one side. ε changes from run to run.
 *
 * 1. TRACK FIT. During one approach, RTK-Fixed antenna positions (map) are
 *    paired with the same point in odom at the GPS stamp. A 2-D rigid fit of the
 *    pairs measures Δ directly from where the robot actually went. It needs no
 *    fused heading, and it improves as the robot advances (longer baseline). It
 *    is blended in between `min_extent_m` and `full_extent_m` of travel and
 *    rejected when it disagrees with the TF by more than `max_disagreement_rad`
 *    or fits badly (`max_rms_m`).
 *
 * 2. FREEZE. The graceful law's curvature grows as 1/r, so a target that moves
 *    by a few cm in the last tens of cm turns into several degrees of heading
 *    at contact. Inside `freeze_distance_m` the detection is held; odom dead
 *    reckoning over that distance is good to millimetres.
 *
 * Everything here is pure and clock-agnostic so it unit-tests without ROS.
 */
#ifndef MOWGLI_LOCALIZATION__DOCK_APPROACH_REFINEMENT_HPP_
#define MOWGLI_LOCALIZATION__DOCK_APPROACH_REFINEMENT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

#include "mowgli_localization/delta_gate.hpp"  // wrap_to_pi

namespace mowgli_localization
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

inline Vec2 rotate(const Vec2& v, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * v.x - s * v.y, s * v.x + c * v.y};
}

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// The docking TARGET: the calibrated dock pose pushed `overshoot_m` further
/// along its own heading (`dock_approach_overshoot`, mowgli_robot.yaml).
///
/// SimpleChargingDock stops on `isDocked()` as soon as base_footprint is within
/// `docking_threshold` (0.05 m) of the target, then waits for charge without
/// moving. Aimed at the dock pose itself, the robot therefore stops short of the
/// contacts. The legacy graph-TF path already shifted `home_dock.pose` by the
/// overshoot (navigation.launch.py); this detection path did not, so on
/// 2026-11-11 four "contact" stops landed 0.09–0.15 m short of the dock pose and
/// never charged (issues #515, #661).
inline PlanarPose approach_target(const PlanarPose& dock_map, double overshoot_m)
{
  return {dock_map.x + overshoot_m * std::cos(dock_map.yaw),
          dock_map.y + overshoot_m * std::sin(dock_map.yaw),
          dock_map.yaw};
}

/// Dock contact pose in odom, anchored at one point seen in both frames.
///
/// `anchor_odom` and `anchor_map` are the SAME physical point (the GPS antenna)
/// in odom and in map; `delta` is the map→odom yaw (p_map = R(delta)·p_odom + t).
/// Anchoring on the antenna rather than on the lever-arm-corrected base keeps
/// the fused heading out of the position: the base position published by
/// navsat_to_absolute_pose was corrected with the fused yaw.
inline PlanarPose dock_in_odom(const Vec2& anchor_odom,
                               const Vec2& anchor_map,
                               const PlanarPose& dock_map,
                               double delta)
{
  const Vec2 d_map{dock_map.x - anchor_map.x, dock_map.y - anchor_map.y};
  const Vec2 d_odom = rotate(d_map, -delta);
  return {anchor_odom.x + d_odom.x, anchor_odom.y + d_odom.y, wrap_to_pi(dock_map.yaw - delta)};
}

struct TrackFitConfig
{
  /// Travel (m, max distance from the first pair) before the fit gets any weight.
  double min_extent_m{0.30};
  /// Travel (m) at which the fit fully replaces the TF Δ.
  double full_extent_m{0.60};
  /// A fit further than this from the TF Δ is rejected as bad data.
  double max_disagreement_rad{10.0 * M_PI / 180.0};
  /// RMS residual (m) above which the fit is rejected.
  double max_rms_m{0.04};
  /// Oldest pairs are dropped beyond this count.
  std::size_t max_pairs{300};
};

struct TrackFitResult
{
  bool valid{false};
  double delta{0.0};
  double extent_m{0.0};
  double rms_m{0.0};
  std::size_t pairs{0};
};

/// Least-squares 2-D rotation between odom and map point sets (Procrustes).
class TrackRotationFit
{
public:
  TrackRotationFit() = default;
  explicit TrackRotationFit(const TrackFitConfig& cfg) : cfg_(cfg)
  {
  }

  void reset()
  {
    pairs_.clear();
  }

  void add(const Vec2& odom, const Vec2& map)
  {
    pairs_.push_back({odom, map});
    while (pairs_.size() > cfg_.max_pairs)
    {
      pairs_.pop_front();
    }
  }

  std::size_t size() const
  {
    return pairs_.size();
  }

  /// Fit p_map = R(delta)·p_odom + t. `valid` only says the geometry allows a
  /// fit (≥ 3 pairs and a non-zero extent); blend_track_delta() applies the
  /// acceptance gates.
  TrackFitResult fit() const
  {
    TrackFitResult out;
    out.pairs = pairs_.size();
    if (pairs_.size() < 3)
    {
      return out;
    }
    Vec2 mo;
    Vec2 mm;
    const Vec2 first = pairs_.front().odom;
    for (const auto& p : pairs_)
    {
      mo.x += p.odom.x;
      mo.y += p.odom.y;
      mm.x += p.map.x;
      mm.y += p.map.y;
      out.extent_m = std::max(out.extent_m, std::hypot(p.odom.x - first.x, p.odom.y - first.y));
    }
    const double n = static_cast<double>(pairs_.size());
    mo = {mo.x / n, mo.y / n};
    mm = {mm.x / n, mm.y / n};

    double sum_cross = 0.0;
    double sum_dot = 0.0;
    for (const auto& p : pairs_)
    {
      const Vec2 a{p.odom.x - mo.x, p.odom.y - mo.y};
      const Vec2 b{p.map.x - mm.x, p.map.y - mm.y};
      sum_cross += a.x * b.y - a.y * b.x;
      sum_dot += a.x * b.x + a.y * b.y;
    }
    if (out.extent_m <= 0.0 || (sum_cross == 0.0 && sum_dot == 0.0))
    {
      return out;
    }
    out.delta = std::atan2(sum_cross, sum_dot);

    double sq = 0.0;
    for (const auto& p : pairs_)
    {
      const Vec2 a = rotate({p.odom.x - mo.x, p.odom.y - mo.y}, out.delta);
      const double ex = (p.map.x - mm.x) - a.x;
      const double ey = (p.map.y - mm.y) - a.y;
      sq += ex * ex + ey * ey;
    }
    out.rms_m = std::sqrt(sq / n);
    out.valid = true;
    return out;
  }

  const TrackFitConfig& config() const
  {
    return cfg_;
  }

private:
  struct Pair
  {
    Vec2 odom;
    Vec2 map;
  };
  TrackFitConfig cfg_{};
  std::deque<Pair> pairs_;
};

enum class TrackBlendState
{
  TF_ONLY,  ///< Not enough travel yet, or no fit.
  BLENDING,  ///< Between min and full extent.
  TRACK,  ///< The fit replaces the TF Δ.
  REJECTED  ///< Fit too far from the TF or too noisy: TF Δ kept.
};

struct TrackBlend
{
  double delta{0.0};
  double weight{0.0};
  TrackBlendState state{TrackBlendState::TF_ONLY};
};

/// Blend the fitted Δ into the TF Δ by travelled extent, with the safety gates.
inline TrackBlend blend_track_delta(double tf_delta,
                                    const TrackFitResult& fit,
                                    const TrackFitConfig& cfg)
{
  TrackBlend out{wrap_to_pi(tf_delta), 0.0, TrackBlendState::TF_ONLY};
  if (!fit.valid || fit.extent_m < cfg.min_extent_m)
  {
    return out;
  }
  const double diff = wrap_to_pi(fit.delta - tf_delta);
  if (std::abs(diff) > cfg.max_disagreement_rad || fit.rms_m > cfg.max_rms_m)
  {
    out.state = TrackBlendState::REJECTED;
    return out;
  }
  const double span = cfg.full_extent_m - cfg.min_extent_m;
  const double w =
      span <= 0.0 ? 1.0 : std::clamp((fit.extent_m - cfg.min_extent_m) / span, 0.0, 1.0);
  out.weight = w;
  out.delta = wrap_to_pi(tf_delta + w * diff);
  out.state = w >= 1.0 ? TrackBlendState::TRACK : TrackBlendState::BLENDING;
  return out;
}

/// "Final approach in progress" from the docking controller's own output.
///
/// opennav_docking publishes /cmd_vel_docking only while it controls the robot.
/// The approach is FORWARD (dock_direction forward); the retry's return to the
/// staging pose and the dock-calibration reverse leg drive backwards and do
/// not count, so each forward approach is its own episode.
class ApproachEpisode
{
public:
  explicit ApproachEpisode(double timeout_s = 1.0, double min_forward_mps = 0.01)
      : timeout_s_(timeout_s), min_forward_mps_(min_forward_mps)
  {
  }

  void on_command(double vx, double t_s)
  {
    if (vx > min_forward_mps_)
    {
      last_forward_s_ = t_s;
      have_forward_ = true;
    }
  }

  bool active(double t_s) const
  {
    return have_forward_ && (t_s - last_forward_s_) <= timeout_s_;
  }

private:
  double timeout_s_;
  double min_forward_mps_;
  double last_forward_s_{0.0};
  bool have_forward_{false};
};

/// True once the robot is inside the freeze radius of the current target.
/// `freeze_distance_m <= 0` disables freezing.
inline bool should_freeze(const Vec2& robot_odom,
                          const PlanarPose& dock_odom,
                          double freeze_distance_m)
{
  if (freeze_distance_m <= 0.0)
  {
    return false;
  }
  return std::hypot(dock_odom.x - robot_odom.x, dock_odom.y - robot_odom.y) < freeze_distance_m;
}

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__DOCK_APPROACH_REFINEMENT_HPP_
