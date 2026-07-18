// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure 180° yaw-flip recovery decision, factored out of OnCogHeading so it is
// unit-testable without ROS/GTSAM (see fusion_graph_node_callbacks_b.cpp).
// COG yaw is the physical travel direction (wheels + GPS displacement, only
// emitted on a solid straight baseline), so a sustained ~180° disagreement
// with the fused estimate means the estimate is flipped — and the non-robust
// COG unary factor can fail to pull it back across the half-turn. After N
// consecutive, mutually-consistent flipped samples, the caller should snap
// the yaw onto the COG (keep the estimated xy). The actual GTSAM
// ForceAnchor call, dead-reckoning reset, and RTK-freshness gating stay in
// the node — this function only owns the counting/consistency/rate-limit
// decision.

#pragma once

#include <cmath>
#include <optional>

namespace fusion_graph
{

struct CogFlipRecoveryCfg
{
  double flip_threshold_rad = 2.618;  // ~150° disagreement to suspect a flip
  double flip_consistency_rad = 0.52;  // ~30° — consecutive COGs must agree within this
  int flip_consecutive_n = 3;  // consistent flipped samples required before anchoring
  double flip_min_interval_s = 10.0;  // rate limit between recoveries
};

struct CogFlipRecoveryResult
{
  bool should_anchor = false;
  int count = 0;
  double err_rad = 0.0;  // |angle-wrapped disagreement| this sample — for logging
};

// Feed one COG sample. `count` and `prev_yaw` are caller-owned debounce state
// (mutated in place, like AnchorSlewStep's px/py/pyaw) — reset to 0/nullopt
// whenever the sample is not a suspected flip or the flipped COGs disagree
// with each other. `seconds_since_last_recovery` is nullopt if no recovery
// has ever fired (the rate limit always passes then); the caller computes it
// from its own clock so this function stays clock-agnostic.
inline CogFlipRecoveryResult CogFlipRecoveryFeed(double yaw,
                                                 double current_yaw,
                                                 std::optional<double> seconds_since_last_recovery,
                                                 const CogFlipRecoveryCfg& cfg,
                                                 int& count,
                                                 std::optional<double>& prev_yaw)
{
  const double d = yaw - current_yaw;
  const double err = std::fabs(std::atan2(std::sin(d), std::cos(d)));
  if (err <= cfg.flip_threshold_rad)
  {
    count = 0;
    prev_yaw.reset();
    return CogFlipRecoveryResult{false, count, err};
  }

  // Consecutive flipped COGs must agree WITH EACH OTHER, else the COG is
  // jittering and snapping to it would amplify rather than fix.
  bool consistent = true;
  if (prev_yaw)
  {
    const double dd = yaw - *prev_yaw;
    consistent = std::fabs(std::atan2(std::sin(dd), std::cos(dd))) < cfg.flip_consistency_rad;
  }
  count = consistent ? (count + 1) : 1;
  prev_yaw = yaw;

  const bool rate_ok = !seconds_since_last_recovery.has_value() ||
                       *seconds_since_last_recovery > cfg.flip_min_interval_s;

  CogFlipRecoveryResult result;
  result.count = count;
  result.err_rad = err;
  if (count >= cfg.flip_consecutive_n && rate_ok)
  {
    result.should_anchor = true;
    count = 0;  // consumed — next sample starts a fresh streak.
  }
  return result;
}

}  // namespace fusion_graph
