// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure bounded-reverse-escape decision, factored out of FTCController so it is
// unit-testable without ROS (see ftc_controller.cpp, test_ftc_reverse_escape.cpp).
//
// SAFETY-CRITICAL. This governs a robot with spinning blades briefly driving
// BACKWARDS to escape a spot where BOTH sides of an obstacle are blocked (or
// the lateral deviation needed to skirt it exceeds the cap). It exists ONLY as
// an escape sub-state; normal path following stays forward_only. The maneuver is
// a PURE STRAIGHT reverse (no rotation — a turning reverse re-creates the ±180°
// pirouette deadlock that forward_only was introduced to avoid), bounded HARD to
// max_dist_m, and gated on the rear footprint being clear of lethal cells. Any
// ambiguity fails safe (no reverse → the caller holds/aborts as before).
//
// The controller owns the accumulator state (dist_done) and the costmap-based
// rear-clear check; this header is only the branch-free arithmetic + policy so
// the budget cap and the fail-safe ordering can be pinned by unit tests.

#pragma once

#include <algorithm>
#include <cmath>

namespace mowgli_nav2_plugins
{

struct ReverseEscapeCfg
{
  // OPT-IN (default false): the maneuver drives the bladed robot BACKWARDS and
  // is field-validated per-robot. The controller always sets this explicitly
  // from the ROS param, so this default governs only unconfigured callers/tests.
  bool enabled = false;
  double max_dist_m = 0.30;  // hard cap on total reversed distance
  double speed_mps = 0.10;  // straight reverse speed (must clear firmware deadband)
};

enum class ReverseEscapeAction
{
  kNone,  ///< Feature disabled — caller proceeds straight to wait/abort.
  kReverse,  ///< Command a bounded straight reverse this tick.
  kExhausted,  ///< Budget spent or rear blocked — caller falls through to wait/abort.
};

/// Pure decision for the WEDGED case (deviation search found no skirt this
/// tick). `dist_done` is the reversed distance accumulated so far (m, >= 0);
/// `rear_clear` is whether the footprint at the backed-up pose is free of lethal
/// cells. The rear-clear gate is checked BEFORE committing to a reverse so we
/// never back into an obstacle — false forbids the maneuver (fail safe). The
/// budget cap is strict: once dist_done reaches max_dist_m we stop reversing.
inline ReverseEscapeAction ReverseEscapeDecide(const ReverseEscapeCfg& cfg,
                                               double dist_done,
                                               bool rear_clear)
{
  if (!cfg.enabled)
  {
    return ReverseEscapeAction::kNone;
  }
  if (dist_done >= cfg.max_dist_m)
  {
    return ReverseEscapeAction::kExhausted;  // budget spent
  }
  if (!rear_clear)
  {
    return ReverseEscapeAction::kExhausted;  // SAFETY: obstacle behind
  }
  return ReverseEscapeAction::kReverse;
}

/// Advance the reversed-distance accumulator by the odom-measured motion this
/// tick, hard-capped at max_dist_m so the total reverse NEVER exceeds the
/// budget. Returns the new dist_done. `measured_fwd_speed` is the odom forward
/// speed (negative while reversing — magnitude is what matters).
inline double ReverseEscapeAdvance(const ReverseEscapeCfg& cfg,
                                   double dist_done,
                                   double measured_fwd_speed,
                                   double dt)
{
  const double step = std::abs(measured_fwd_speed) * std::max(0.0, dt);
  return std::min(cfg.max_dist_m, dist_done + step);
}

}  // namespace mowgli_nav2_plugins
