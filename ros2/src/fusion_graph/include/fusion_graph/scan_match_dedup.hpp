// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Gate that stops OnTimer re-running the scan-between ICP on input it has
// already matched. Pure — no ROS / GTSAM — so it is unit-testable, same shape
// as slip_window.hpp / loop_closure_gate.hpp.
//
// OnTimer runs at node_period_s (0.04 s → 25 Hz) and, every tick, copied
// latest_scan_ and matched it against prev_node_scan_. The LiDAR publishes at
// ~10 Hz. Measured on the docked robot 2026-09-06 over 10 s: 89 scans
// received, 225 scan matches run — ~2.5 ICPs per scan, ~60 % of them on a
// (scan, prev-node scan) pair that had not changed since the previous tick.
// fusion_graph's main thread sat at 14 % of a core while stationary.
//
// The two inputs of that ICP are identified by counters: the scan sequence
// (bumped in OnScan) and the prev-node generation (bumped when Tick stores a
// new prev_node_scan_). If neither moved since the last match, the ICP would
// realign the same two point clouds — the same physical measurement.
//
// Conservative on purpose: a SUCCESSFUL match on a pair is not repeated; a
// FAILED one may be retried on the next tick. The init guess drifts with
// odometry between ticks, and a match that fell to a guard rail on one tick
// could clear it on the next, so retrying preserves that chance while
// removing only the redundant successful recomputation.

#pragma once

#include <cstdint>

namespace fusion_graph
{

class ScanMatchDedupGate
{
public:
  // True when the ICP should run for this (scan, prev-node) pair.
  bool ShouldMatch(uint64_t scan_seq, uint64_t prev_node_gen) const
  {
    if (!have_last_)
      return true;
    if (scan_seq != last_scan_seq_ || prev_node_gen != last_prev_node_gen_)
      return true;
    return !last_ok_;
  }

  // Record the outcome of a match that did run on this pair.
  void RecordOutcome(uint64_t scan_seq, uint64_t prev_node_gen, bool ok)
  {
    have_last_ = true;
    last_scan_seq_ = scan_seq;
    last_prev_node_gen_ = prev_node_gen;
    last_ok_ = ok;
  }

private:
  bool have_last_ = false;
  uint64_t last_scan_seq_ = 0;
  uint64_t last_prev_node_gen_ = 0;
  bool last_ok_ = false;
};

}  // namespace fusion_graph
