// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure loop-closure rate/travel gate + GPS σ floor, factored out of the
// per-node loop-closure search in fusion_graph_node_timer.cpp so it is
// unit-testable without ROS/GTSAM (same shape as rtk_wrongfix_gate.hpp and
// dr_slip_veto.hpp).
//
// Why (issue #513): the loop-closure search runs every node (node_period_s =
// 0.04 s → 25 Hz) with up to lc_max_candidates = 3 ICP matches, so it can add
// up to 75 factors/s. It is skipped while an RTK-Fixed sample is fresh
// (lc_skip_when_rtk_fixed), so it runs exactly when GPS is Float or absent —
// and there the RTK-yield σ inflation never applies. Measured live
// (2026-09-02, mow #1): 13.7 accepted loop closures per second while mowing
// under Float (5194 over 1900 samples at 5 Hz; 3816 in the 286 s MOWING
// phase = 13.3 /s; 0 under Fixed), each a 5 cm between-factor to an adjacent
// swath on featureless grass (ICP converges to *something* with 10–20 cm
// RMSE almost every time). Each one out-votes the decimetre-σ Float GPS
// factor by an order of magnitude → lateral snaps of one swath spacing,
// ~13× per second.
//
// Two independent bounds:
//   1. LoopClosureRateAllows — at most one accept per lc_min_interval_s AND
//      per lc_min_travel_m of wheel travel (the caller also breaks after the
//      first accept in a node, so the 3-candidates-per-node multiplier is
//      gone too).
//   2. LoopClosureSigmaFloor — an LC can never be tighter than the most
//      recent accepted GNSS σ (× lc_gps_sigma_ratio). Under Float (σ≈0.3 m)
//      LC becomes advisory; under a true no-fix window the last σ is the last
//      GPS we had and the graph still gets its global-consistency constraint.
//
// DESIGN NOTE — reset polarity, and why it is the OPPOSITE of
// rtk_wrongfix_gate.hpp: the two accumulators (travel_since_accept_m,
// time_since_accept_s) are reset on ACCEPT ONLY. A candidate rejected by
// this gate must NOT reset them — if it did, at 25 Hz the accumulators would
// be zeroed every node and the gate would never open again (a permanent
// "no loop closures" lockout). CLAUDE.md's GnssMobileGate incident is the
// mirror-image bug: there the accumulator was an unbounded "expected motion
// since last ACCEPTED fix" that fed the *rejection budget*, so reset-on-
// accept-only made every rejection grow the budget and lock GPS out forever.
// Here the accumulators feed the *acceptance* condition ("enough has
// happened since the last one"), so reset-on-accept-only is exactly what
// keeps the gate live: each rejected node moves the accumulators toward
// opening, never away from it. Both are still bounded per interval — they
// reset to zero the moment one LC is accepted, so the worst case is one
// accepted LC per (lc_min_interval_s, lc_min_travel_m), never a runaway.

#pragma once

#include <algorithm>
#include <cmath>

namespace fusion_graph
{

// Bounded "since last ACCEPTED loop closure" accumulators. Reset on accept
// only — see the design note above for why a gate rejection must leave them
// untouched.
struct LoopClosureGateState
{
  double travel_since_accept_m = 0.0;
  double time_since_accept_s = 0.0;
};

// True when a loop closure may be attempted: BOTH the wheel travel since the
// last accepted LC has reached `min_travel_m` AND the elapsed time has
// reached `min_interval_s`. A non-positive threshold disables that half
// (its condition is always satisfied). Comparisons are >= so a threshold of
// exactly the accumulated value opens the gate.
inline bool LoopClosureRateAllows(double travel_since_accept_m,
                                  double time_since_accept_s,
                                  double min_travel_m,
                                  double min_interval_s)
{
  const bool travel_ok = min_travel_m <= 0.0 || travel_since_accept_m >= min_travel_m;
  const bool interval_ok = min_interval_s <= 0.0 || time_since_accept_s >= min_interval_s;
  return travel_ok && interval_ok;
}

// Floor the loop-closure translational σ to `ratio * last_gps_sigma` so an LC
// can never be tighter than the most recent accepted GNSS fix. Returns
// `lc_sigma` unchanged when the floor is disabled (`ratio <= 0`) or when no
// usable GPS σ has ever been seen (`last_gps_sigma` non-finite or <= 0 — the
// node latches -1 for "none").
inline double LoopClosureSigmaFloor(double lc_sigma, double last_gps_sigma, double ratio)
{
  if (ratio <= 0.0 || !std::isfinite(last_gps_sigma) || last_gps_sigma <= 0.0)
    return lc_sigma;
  return std::max(lc_sigma, ratio * last_gps_sigma);
}

}  // namespace fusion_graph
