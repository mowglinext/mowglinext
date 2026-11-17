// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure RTK wrong-fix motion-consistency gate, factored out of OnGnss so it is
// unit-testable without ROS/GTSAM (see fusion_graph_node_callbacks_a.cpp).
// F9P can re-solve the carrier-phase ambiguity on a different integer set
// after a brief signal drop (vegetation, multipath spike) and the new
// solution jumps by a few cm while still reporting a trustworthy status +
// sub-cm covariance. The gate rejects a GPS step that is larger than the
// chassis could plausibly have travelled since the last fix (wheel arc +
// lever-arm sweep during rotation) plus a fixed slack budget.
//
// CRITICAL DESIGN NOTE — bounded vs. runaway: the two accumulators
// (wheel_dist_m, abs_dtheta_rad) MUST be reset after every fix, whether it is
// ACCEPTED or REJECTED. This is what keeps the gate's budget bounded to "at
// most one inter-fix interval of travel". The reverted GnssMobileGate
// (bundled into PR #307) instead reset its equivalent accumulator only on
// ACCEPT: once it started rejecting, the accumulator never cleared, motion
// kept piling up against a stale reference, "expected motion" ran away
// (observed 20.9 m for <1 m of real travel), and every subsequent fix
// rejected forever (GPS locked out, cov_xx ballooned to ~2.5 m σ). See
// CLAUDE.md "What NOT to Do" for the incident writeup.

#pragma once

#include <algorithm>
#include <cmath>

namespace fusion_graph
{

// True if a GPS step of `jump_m` cannot be explained by chassis motion since
// the last fix (wheel-arc distance `wheel_dist_m` plus lever-arm sweep
// `lever_arm_radius_m * abs_dtheta_rad` from in-place rotation) plus the fixed
// slack budget `max_jump_m` — i.e. the fix should be dropped as a likely
// wrong-fix rather than fused.
inline bool GpsJumpImplausible(double jump_m,
                               double max_jump_m,
                               double lever_arm_radius_m,
                               double abs_dtheta_rad,
                               double wheel_dist_m)
{
  const double expected_pivot_jump_m = lever_arm_radius_m * abs_dtheta_rad;
  const double jump_budget_m = max_jump_m + expected_pivot_jump_m + wheel_dist_m;
  return jump_m > jump_budget_m;
}

// Slack for GpsJumpImplausible: the fixed budget, or k times the combined
// reported σ of the two fixes being compared, whichever is larger. Under
// RTK-Fixed (σ ≈ 1 cm → 3·√2·0.01 ≈ 0.04 m) the fixed budget wins and nothing
// changes; under RTK-Float (σ 5-30 cm) the epoch-to-epoch jitter of a parked
// robot would otherwise exceed a fixed 8 cm and reject almost every sample.
// A non-finite or non-positive σ contributes nothing.
inline double WrongFixSlackM(double max_jump_m,
                             double sigma_k,
                             double sigma_prev_m,
                             double sigma_now_m)
{
  const auto valid = [](double s)
  {
    return std::isfinite(s) && s > 0.0 ? s : 0.0;
  };
  const double k = std::isfinite(sigma_k) && sigma_k > 0.0 ? sigma_k : 0.0;
  const double combined = std::hypot(valid(sigma_prev_m), valid(sigma_now_m));
  return std::max(max_jump_m, k * combined);
}

// Unconditional post-fix reset of the bounded motion accumulators. Call this
// after EVERY GPS fix regardless of GpsJumpImplausible's verdict — accepted
// or rejected — so the next fix's budget only ever reflects travel since
// *this* fix, never an unbounded "since last accepted fix" total. Skipping
// this call on the reject path is exactly the GnssMobileGate regression
// described above.
inline void ResetRtkWrongFixAccumulators(double& wheel_dist_m, double& abs_dtheta_rad)
{
  wheel_dist_m = 0.0;
  abs_dtheta_rad = 0.0;
}

}  // namespace fusion_graph
