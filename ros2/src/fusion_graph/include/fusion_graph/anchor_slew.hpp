// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure map→odom anchor slew-rate limiter — no ROS / gtsam dependency so it is
// trivially unit-testable in isolation (test_map_odom_slew.cpp). The graph
// recomputes the raw map→odom anchor only when a new node lands, which steps
// discontinuously on every correction (GPS innovation, loop closure,
// scan-match, or the whole refinement snapped in at the end of a stationary
// window). Feeding that step straight into map→base = anchor ⊙ odom→base makes
// Nav2's controller track a teleporting pose → left/right weave + in-place
// hunting. AnchorSlewStep eases a published anchor toward the raw target at a
// bounded velocity so corrections enter continuously (REP-105), while snapping
// on relocalization-scale jumps so a genuine re-anchor is never lagged.

#pragma once

#include <algorithm>
#include <cmath>

namespace fusion_graph
{

struct AnchorSlewCfg
{
  bool enabled = true;
  double max_lin_mps = 0.10;  // ≤ injected map-frame velocity from corrections
  double max_ang_radps = 0.20;  // ≤ injected map-frame yaw rate
  double snap_dist_m = 0.50;  // above → relocalization, apply immediately
  double snap_yaw_rad = 0.35;  // above → relocalization, apply immediately
};

// Advance a published anchor (px,py,pyaw + pub_valid, all mutated in place) one
// step toward a raw target (tx,ty,tyaw, gated by anchor_valid) over dt seconds.
// Writes the anchor to broadcast into (ox,oy,oyaw).
inline void AnchorSlewStep(bool& pub_valid,
                           double& px,
                           double& py,
                           double& pyaw,
                           bool anchor_valid,
                           double tx,
                           double ty,
                           double tyaw,
                           double dt,
                           const AnchorSlewCfg& cfg,
                           double& ox,
                           double& oy,
                           double& oyaw)
{
  if (!cfg.enabled)
  {
    ox = tx;
    oy = ty;
    oyaw = tyaw;
    return;  // A/B: reproduce pre-slew step behaviour exactly.
  }
  if (!anchor_valid)
  {
    // Re-anchor / clear / re-init in flight — resnap to the next target.
    pub_valid = false;
    ox = tx;
    oy = ty;
    oyaw = tyaw;
    return;
  }
  if (!pub_valid)
  {
    px = tx;  // first fix: snap to the graph.
    py = ty;
    pyaw = tyaw;
    pub_valid = true;
    ox = px;
    oy = py;
    oyaw = pyaw;
    return;
  }

  const double ex = tx - px;
  const double ey = ty - py;
  const double derr = std::hypot(ex, ey);
  const double dyaw = std::atan2(std::sin(tyaw - pyaw), std::cos(tyaw - pyaw));

  // Relocalization-scale jump (re-seed, first fix after a long Float, big loop
  // closure) → apply immediately rather than crawl for seconds.
  if (derr > cfg.snap_dist_m || std::fabs(dyaw) > cfg.snap_yaw_rad)
  {
    px = tx;
    py = ty;
    pyaw = tyaw;
    ox = px;
    oy = py;
    oyaw = pyaw;
    return;
  }

  const double lin_step = cfg.max_lin_mps * std::max(dt, 0.0);
  const double ang_step = cfg.max_ang_radps * std::max(dt, 0.0);
  if (derr <= lin_step || derr < 1e-9)
  {
    px = tx;
    py = ty;
  }
  else
  {
    px += ex / derr * lin_step;
    py += ey / derr * lin_step;
  }
  pyaw += std::max(-ang_step, std::min(dyaw, ang_step));
  ox = px;
  oy = py;
  oyaw = pyaw;
}

}  // namespace fusion_graph
