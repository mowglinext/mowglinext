// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
//
// Self-calibrated trust for the LiDAR anchor.
//
// A particle filter's own covariance is structurally over-confident — a tight
// cloud in the wrong place reports centimetres — so the anchor's XY factor
// carries a σ floor. The right floor is the anchor's REAL error, and the only
// reference for that is RTK-Fixed through shadow mode: under Fixed every
// shadow estimate yields |estimate − fused pose|. This keeps a rolling window
// of those and serves a quantile of it as the effective floor once enough
// samples exist. Field 2026-09-08: p50 0.07–0.09 m, p90 0.15 m, while the
// fixed floor was 0.05 m — the anchor out-weighed RTK-Float fixes by 3× and
// imposed its own ~15 cm bias on short Float episodes.
#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace fusion_graph
{

class LidarAnchorShadowStats
{
public:
  explicit LidarAnchorShadowStats(std::size_t window = 300, std::size_t min_samples = 50)
      : window_(std::max<std::size_t>(1, window)),
        min_samples_(std::max<std::size_t>(1, min_samples))
  {
    buf_.reserve(window_);
  }

  void Clear()
  {
    buf_.clear();
    next_ = 0;
    total_ = 0;
  }

  void Push(double err_m)
  {
    if (!(err_m >= 0.0))
      return;  // NaN / negative: not evidence
    if (buf_.size() < window_)
      buf_.push_back(err_m);
    else
    {
      buf_[next_] = err_m;
    }
    next_ = (next_ + 1) % window_;
    ++total_;
  }

  std::size_t size() const
  {
    return buf_.size();
  }
  std::size_t total() const
  {
    return total_;
  }
  bool ready() const
  {
    return buf_.size() >= min_samples_;
  }

  // Quantile q in [0,1] of the window (nearest-rank); NaN-free, 0 when empty.
  double Quantile(double q) const
  {
    if (buf_.empty())
      return 0.0;
    std::vector<double> v(buf_);
    std::sort(v.begin(), v.end());
    const double qc = std::clamp(q, 0.0, 1.0);
    const std::size_t idx = static_cast<std::size_t>(qc * static_cast<double>(v.size() - 1) + 0.5);
    return v[std::min(idx, v.size() - 1)];
  }

  // Effective σ floor: the window quantile once ready, clamped to
  // [floor_min, floor_max]; floor_min before that.
  double EffectiveFloor(double q, double floor_min, double floor_max) const
  {
    const double hi = std::max(floor_min, floor_max);
    if (!ready())
      return floor_min;
    return std::clamp(Quantile(q), floor_min, hi);
  }

private:
  std::size_t window_;
  std::size_t min_samples_;
  std::vector<double> buf_;
  std::size_t next_ = 0;
  std::size_t total_ = 0;
};

}  // namespace fusion_graph
