// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Path-space divergence detector: the robot is leaving its line while it has
// stopped making progress ALONG it.
//
// This is a second, independent family of evidence for the same failure the dig
// detector (CLAUDE.md Invariant 16) exists to catch — the robot pinned against
// something while the wheels keep turning. It is NOT faster: on the 2026-09-16
// tree collision the dig detector fired at t+5 s, while the path index only
// froze at t+7 s, so this check would have been LATER. Its value is that it has
// none of the dig detector's stand-down conditions. That detector steps aside
// above `dig_max_pos_sigma` (RTK Float), above `dig_max_yaw_rate` (turning), and
// whenever the fused pose, raw RTK, gyro or cmd_vel goes stale — every one of
// those is a window where a robot can grind against an obstacle unobserved.
// This one reads only what the controller server already measures about the
// path, so no GNSS, no encoders, no gyro, and no freshness gate applies.
//
// The discriminator is the CONJUNCTION, and both halves are load-bearing:
//
//   * lateral error GROWING — a deliberate obstacle skirt also grows the error
//     (FTC deviates laterally on purpose, up to max_lateral_deviation), so
//     growth alone would fire on every avoidance episode. Field values: 0.05 to
//     0.20 m of deliberate deviation with the footprint model, up to 0.70 m
//     before it.
//   * path index NOT advancing — a skirt keeps advancing along the path while
//     offset from it; a robot held by an obstacle does not. Field values: FTC
//     tracks to +/-0.02 m with the index climbing steadily, and the collision
//     showed the index frozen at 655/697 while the error ran to 0.71 m.
//
// A stopped robot is deliberately NOT a trigger: during an obstacle hold FTC
// commands zero velocity, so the index is frozen but the error is CONSTANT.
// Only growth counts.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mowgli_interfaces::path_divergence
{

/// Tuning, all field-derived. Defaults are deliberately conservative: a false
/// trigger costs one aborted sub-path, which the behavior tree already recovers
/// from, but it also stops a healthy mow.
struct Config
{
  /// Seconds of continuous divergence before reporting. Long enough that a
  /// bounded reverse-escape (<= 0.30 m at 0.15 m/s, so ~2 s) cannot complete a
  /// window on its own.
  double window_s{4.0};
  /// Metres the |lateral error| must GROW across the window.
  double min_growth_m{0.20};
  /// Path poses the index may advance across the window and still count as
  /// "not progressing". 2 poses is 0.10 m of an F2C route (0.05 m sampling).
  std::uint32_t max_index_advance{2};
  /// Samples required before a verdict, so a burst of two messages cannot arm
  /// it. At the 10 Hz controller rate a 4 s window carries ~40.
  std::size_t min_samples{8};
};

/// Rolling verdict over the samples fed to it. Not thread-safe; the caller
/// holds its own lock, as the FollowPath feedback callback already does.
class Detector
{
public:
  explicit Detector(const Config& config = {}) : config_(config)
  {
  }

  void Reset()
  {
    armed_ = false;
    samples_ = 0;
    anchor_abs_error_m_ = 0.0;
    anchor_index_ = 0;
    anchor_time_s_ = 0.0;
    last_time_s_ = 0.0;
    triggered_ = false;
  }

  /// Feed one controller-server tracking sample.
  ///
  /// `time_s` must be monotonic; a backwards step (a new goal, a clock jump)
  /// re-anchors rather than producing a verdict from mixed episodes.
  void Update(const double lateral_error_m, const std::uint32_t path_index, const double time_s)
  {
    if (!std::isfinite(lateral_error_m) || !std::isfinite(time_s))
    {
      return;
    }
    const double abs_error = std::fabs(lateral_error_m);

    if (!armed_ || time_s < last_time_s_ || path_index + config_.max_index_advance < anchor_index_)
    {
      Anchor(abs_error, path_index, time_s);
      return;
    }
    last_time_s_ = time_s;
    ++samples_;

    // Progress along the path clears the episode: whatever the lateral error is
    // doing, the robot is still advancing, which is what a skirt looks like.
    if (path_index > anchor_index_ + config_.max_index_advance)
    {
      Anchor(abs_error, path_index, time_s);
      return;
    }

    // The error must not come back down: re-anchor on any improvement so the
    // window always measures a MONOTONIC departure from the line.
    if (abs_error < anchor_abs_error_m_)
    {
      Anchor(abs_error, path_index, time_s);
      return;
    }

    if (time_s - anchor_time_s_ < config_.window_s || samples_ < config_.min_samples)
    {
      return;
    }
    triggered_ = (abs_error - anchor_abs_error_m_) >= config_.min_growth_m;
  }

  /// True once the window closed on a growing error with a frozen index. Stays
  /// true until Reset(), so the caller acts once and decides what happens next.
  bool Triggered() const
  {
    return triggered_;
  }

  /// Metres of growth accumulated since the anchor — for the log line that
  /// explains the stop.
  double GrowthM(const double current_abs_error_m) const
  {
    return current_abs_error_m - anchor_abs_error_m_;
  }

  double WindowSeconds(const double now_s) const
  {
    return armed_ ? now_s - anchor_time_s_ : 0.0;
  }

private:
  void Anchor(const double abs_error, const std::uint32_t path_index, const double time_s)
  {
    armed_ = true;
    samples_ = 1;
    anchor_abs_error_m_ = abs_error;
    anchor_index_ = path_index;
    anchor_time_s_ = time_s;
    last_time_s_ = time_s;
    triggered_ = false;
  }

  Config config_;
  bool armed_{false};
  std::size_t samples_{0};
  double anchor_abs_error_m_{0.0};
  std::uint32_t anchor_index_{0};
  double anchor_time_s_{0.0};
  double last_time_s_{0.0};
  bool triggered_{false};
};

}  // namespace mowgli_interfaces::path_divergence
