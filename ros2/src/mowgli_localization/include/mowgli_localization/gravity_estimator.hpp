// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
/// @file gravity_estimator.hpp
/// @brief Conservative gravity-direction gate with stale-latch recovery.
//
// The scan ground filter needs a trustworthy gravity direction, but must not
// mistake a bump or body acceleration for gravity. Samples near the active
// gravity-magnitude baseline are accepted immediately. A plausible, but
// out-of-band, acceleration vector is held as a candidate; only five seconds
// of continuous, mutually-consistent full vectors can replace a stale baseline.
// This avoids a permanent rejected-sample latch while preserving pass-through
// behaviour until the new baseline is proven.

#ifndef MOWGLI_LOCALIZATION__GRAVITY_ESTIMATOR_HPP_
#define MOWGLI_LOCALIZATION__GRAVITY_ESTIMATOR_HPP_

#include <cmath>
#include <cstddef>
#include <limits>

namespace mowgli_localization
{

constexpr double kStandardGravityMs2 = 9.80665;
constexpr double kDefaultMaxReseedTiltRad = 0.52359877559829887308;

struct GravityVector
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GravityEstimatorConfig
{
  /// Allowed deviation from the active gravity baseline for ordinary samples.
  double accel_g_tolerance_ms2{3.0};
  /// Plausibility envelope applies even before the first accepted sample.
  double min_plausible_magnitude_ms2{0.5 * kStandardGravityMs2};
  double max_plausible_magnitude_ms2{1.5 * kStandardGravityMs2};
  /// Maximum full-vector distance within a stable rejected candidate run.
  double candidate_consistency_ms2{0.5};
  /// Maximum time between samples in a continuous candidate run.
  double candidate_max_gap_s{0.5};
  /// Stable candidate duration needed to replace a stale baseline.
  double reseed_after_s{5.0};
  /// Largest tilt allowed when adopting a replacement gravity direction.
  double max_reseed_tilt_rad{kDefaultMaxReseedTiltRad};
  /// Direction low-pass weight for ordinary accepted samples.
  double direction_alpha{0.2};
};

enum class GravityEstimatorAction
{
  SEEDED,
  ACCEPTED,
  REJECTED,
  RESEEDED,
  INVALID,
};

class GravityEstimator
{
public:
  GravityEstimator() = default;
  explicit GravityEstimator(const GravityEstimatorConfig& cfg) : cfg_(cfg)
  {
  }

  GravityEstimatorAction update(const GravityVector& acceleration, double t_s)
  {
    const double mag = magnitude(acceleration);
    if (!std::isfinite(t_s) || !finite(acceleration) || !std::isfinite(mag) || mag < 1e-3 ||
        mag < cfg_.min_plausible_magnitude_ms2 || mag > cfg_.max_plausible_magnitude_ms2)
    {
      clear_candidate();
      return GravityEstimatorAction::INVALID;
    }

    const GravityVector direction = normalized(acceleration, mag);
    if (std::abs(mag - baseline_magnitude_ms2_) <= cfg_.accel_g_tolerance_ms2)
    {
      clear_candidate();
      if (!have_direction_)
      {
        direction_ = direction;
        have_direction_ = true;
        return GravityEstimatorAction::SEEDED;
      }
      direction_ = normalized(add(scale(direction, cfg_.direction_alpha),
                                  scale(direction_, 1.0 - cfg_.direction_alpha)));
      return GravityEstimatorAction::ACCEPTED;
    }

    // Out of the normal band but physically plausible. It cannot refresh
    // freshness until a consistent, stationary-looking run proves a new bias.
    const double candidate_gap_s = t_s - candidate_last_s_;
    if (have_candidate_ && candidate_gap_s >= 0.0 && candidate_gap_s <= cfg_.candidate_max_gap_s &&
        distance(acceleration, candidate_anchor_) <= cfg_.candidate_consistency_ms2)
    {
      candidate_last_s_ = t_s;
      ++candidate_count_;
      candidate_mean_ = add(candidate_mean_,
                            scale(subtract(acceleration, candidate_mean_),
                                  1.0 / static_cast<double>(candidate_count_)));
      if (candidate_window_complete(t_s))
      {
        if (!candidate_tilt_is_safe())
        {
          // An off-axis frozen bus must not turn recovery into obstacle removal.
          // Require a new, full candidate window before trying again.
          clear_candidate();
          return GravityEstimatorAction::REJECTED;
        }
        direction_ = normalized(candidate_mean_);
        previous_baseline_magnitude_ms2_ = baseline_magnitude_ms2_;
        baseline_magnitude_ms2_ = magnitude(candidate_mean_);
        have_direction_ = true;
        clear_candidate();
        return GravityEstimatorAction::RESEEDED;
      }
      return GravityEstimatorAction::REJECTED;
    }

    candidate_anchor_ = acceleration;
    candidate_mean_ = acceleration;
    candidate_count_ = 1;
    candidate_start_s_ = t_s;
    candidate_last_s_ = t_s;
    have_candidate_ = true;
    return GravityEstimatorAction::REJECTED;
  }

  bool has_direction() const
  {
    return have_direction_;
  }
  GravityVector direction() const
  {
    return direction_;
  }
  bool has_candidate() const
  {
    return have_candidate_;
  }
  double baseline_magnitude_ms2() const
  {
    return baseline_magnitude_ms2_;
  }
  double previous_baseline_magnitude_ms2() const
  {
    return previous_baseline_magnitude_ms2_;
  }

private:
  static bool finite(const GravityVector& v)
  {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  }
  static double magnitude(const GravityVector& v)
  {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  }
  static GravityVector normalized(const GravityVector& v, double mag)
  {
    return GravityVector{v.x / mag, v.y / mag, v.z / mag};
  }
  static GravityVector normalized(const GravityVector& v)
  {
    return normalized(v, magnitude(v));
  }
  static GravityVector add(const GravityVector& a, const GravityVector& b)
  {
    return GravityVector{a.x + b.x, a.y + b.y, a.z + b.z};
  }
  static GravityVector subtract(const GravityVector& a, const GravityVector& b)
  {
    return GravityVector{a.x - b.x, a.y - b.y, a.z - b.z};
  }
  static GravityVector scale(const GravityVector& v, double factor)
  {
    return GravityVector{factor * v.x, factor * v.y, factor * v.z};
  }
  static double distance(const GravityVector& a, const GravityVector& b)
  {
    return magnitude(subtract(a, b));
  }
  bool reseed_config_is_valid() const
  {
    // A valid window must need more than one bounded interval. This prevents a
    // permissive direct-use configuration from treating two isolated samples
    // several seconds apart as a continuous five-second observation.
    return std::isfinite(cfg_.candidate_max_gap_s) && cfg_.candidate_max_gap_s > 0.0 &&
           std::isfinite(cfg_.reseed_after_s) && cfg_.reseed_after_s > 0.0 &&
           cfg_.candidate_max_gap_s < cfg_.reseed_after_s &&
           std::isfinite(cfg_.max_reseed_tilt_rad) && cfg_.max_reseed_tilt_rad >= 0.0 &&
           cfg_.max_reseed_tilt_rad <= kHalfPi;
  }
  std::size_t minimum_reseed_sample_count() const
  {
    if (!reseed_config_is_valid())
    {
      return std::numeric_limits<std::size_t>::max();
    }
    const double intervals = std::ceil(cfg_.reseed_after_s / cfg_.candidate_max_gap_s);
    if (!std::isfinite(intervals) ||
        intervals >= static_cast<double>(std::numeric_limits<std::size_t>::max() - 1U))
    {
      return std::numeric_limits<std::size_t>::max();
    }
    return static_cast<std::size_t>(intervals) + 1U;
  }
  bool candidate_window_complete(double t_s) const
  {
    return reseed_config_is_valid() && t_s >= candidate_start_s_ &&
           t_s - candidate_start_s_ >= cfg_.reseed_after_s &&
           candidate_count_ >= minimum_reseed_sample_count();
  }
  bool candidate_tilt_is_safe() const
  {
    const double candidate_magnitude = magnitude(candidate_mean_);
    if (!std::isfinite(candidate_magnitude) || candidate_magnitude < 1e-3)
    {
      return false;
    }
    const double horizontal_magnitude = std::hypot(candidate_mean_.x, candidate_mean_.y);
    const double sin_tilt = horizontal_magnitude / candidate_magnitude;
    return std::isfinite(sin_tilt) && sin_tilt <= std::sin(cfg_.max_reseed_tilt_rad);
  }
  void clear_candidate()
  {
    have_candidate_ = false;
    candidate_anchor_ = GravityVector{};
    candidate_mean_ = GravityVector{};
    candidate_count_ = 0;
    candidate_start_s_ = 0.0;
    candidate_last_s_ = 0.0;
  }

  GravityEstimatorConfig cfg_{};
  static constexpr double kHalfPi = 1.57079632679489661923;
  bool have_direction_{false};
  GravityVector direction_{0.0, 0.0, 1.0};
  double baseline_magnitude_ms2_{kStandardGravityMs2};
  double previous_baseline_magnitude_ms2_{kStandardGravityMs2};

  bool have_candidate_{false};
  GravityVector candidate_anchor_{};
  GravityVector candidate_mean_{};
  std::size_t candidate_count_{0};
  double candidate_start_s_{0.0};
  double candidate_last_s_{0.0};
};

}  // namespace mowgli_localization

#endif  // MOWGLI_LOCALIZATION__GRAVITY_ESTIMATOR_HPP_
