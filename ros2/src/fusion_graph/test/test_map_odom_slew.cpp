// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the map→odom anchor slew-rate limiter (anchor_slew.hpp).
// Pure logic, no ROS / gtsam plumbing.

#include <cmath>

#include "fusion_graph/anchor_slew.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
fg::AnchorSlewCfg DefaultCfg()
{
  // Mirrors the fusion_graph.yaml defaults.
  return fg::AnchorSlewCfg{true, 0.10, 0.20, 0.50, 0.35};
}
}  // namespace

// enabled=false must be a pass-through: the raw target is emitted verbatim so
// the node reproduces the pre-slew step behaviour exactly (A/B toggle).
TEST(AnchorSlew, DisabledPassesTargetThrough)
{
  // Arrange
  fg::AnchorSlewCfg cfg = DefaultCfg();
  cfg.enabled = false;
  bool pub_valid = true;
  double px = 0.0, py = 0.0, pyaw = 0.0, ox, oy, oyaw;

  // Act
  fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, 5.0, 5.0, 1.0, 0.05, cfg, ox, oy, oyaw);

  // Assert
  EXPECT_DOUBLE_EQ(ox, 5.0);
  EXPECT_DOUBLE_EQ(oy, 5.0);
  EXPECT_DOUBLE_EQ(oyaw, 1.0);
}

// The very first valid target snaps (no crawl from an arbitrary origin).
TEST(AnchorSlew, FirstValidSnaps)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = false;
  double px = 0.0, py = 0.0, pyaw = 0.0, ox, oy, oyaw;

  fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, 2.0, -3.0, 0.4, 0.05, cfg, ox, oy, oyaw);

  EXPECT_TRUE(pub_valid);
  EXPECT_DOUBLE_EQ(ox, 2.0);
  EXPECT_DOUBLE_EQ(oy, -3.0);
  EXPECT_DOUBLE_EQ(oyaw, 0.4);
}

// An invalid anchor (re-anchor / clear in flight) resets pub_valid so the next
// valid target snaps rather than crawling from a stale published pose.
TEST(AnchorSlew, InvalidResetsPubValid)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = true;
  double px = 1.0, py = 1.0, pyaw = 0.0, ox, oy, oyaw;

  fg::AnchorSlewStep(pub_valid, px, py, pyaw, false, 9.0, 9.0, 0.0, 0.05, cfg, ox, oy, oyaw);

  EXPECT_FALSE(pub_valid);
}

// A small per-node correction is rate-limited: one 0.05 s step of a 0.10 m/s
// limiter moves 5 mm toward the target, not the whole 0.10 m step.
TEST(AnchorSlew, SmallStepIsRateLimited)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = true;
  double px = 0.0, py = 0.0, pyaw = 0.0, ox, oy, oyaw;

  fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, 0.10, 0.0, 0.0, 0.05, cfg, ox, oy, oyaw);

  EXPECT_NEAR(ox, 0.005, 1e-9);  // 0.10 m/s * 0.05 s
  EXPECT_NEAR(oy, 0.0, 1e-9);
}

// Iterating the limiter must converge exactly onto the target and stop there.
TEST(AnchorSlew, ConvergesAndBoundedVelocity)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = true;
  double px = 0.0, py = 0.0, pyaw = 0.0, ox, oy, oyaw;
  const double tx = 0.08, ty = 0.06;  // 0.10 m away, below snap_dist
  const double dt = 0.05;
  double prev_x = px, prev_y = py;

  for (int i = 0; i < 200; ++i)
  {
    fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, tx, ty, 0.0, dt, cfg, ox, oy, oyaw);
    // Per-step travel never exceeds the rate limit (+ epsilon).
    const double step = std::hypot(ox - prev_x, oy - prev_y);
    EXPECT_LE(step, cfg.max_lin_mps * dt + 1e-9);
    prev_x = ox;
    prev_y = oy;
  }

  EXPECT_NEAR(ox, tx, 1e-9);
  EXPECT_NEAR(oy, ty, 1e-9);
}

// A relocalization-scale jump (> snap_dist_m) is applied immediately.
TEST(AnchorSlew, LargeXyJumpSnaps)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = true;
  double px = 0.0, py = 0.0, pyaw = 0.0, ox, oy, oyaw;

  fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, 1.0, 0.0, 0.0, 0.05, cfg, ox, oy, oyaw);

  EXPECT_DOUBLE_EQ(ox, 1.0);  // 1.0 m > 0.50 snap_dist → snap
}

// Yaw is rate-limited independently and takes the shortest angular path.
TEST(AnchorSlew, YawRateLimitedShortestPath)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = true;
  // pub yaw just below +pi, target just above -pi: shortest path is +Δ, small.
  double px = 0.0, py = 0.0, pyaw = 3.10, ox, oy, oyaw;
  const double target_yaw = -3.10;  // ~0.083 rad away the short way

  fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, 0.0, 0.0, target_yaw, 0.05, cfg, ox, oy, oyaw);

  // Moves at most max_ang*dt = 0.01 rad, in the +wrap direction (past +pi).
  const double delta = std::atan2(std::sin(oyaw - 3.10), std::cos(oyaw - 3.10));
  EXPECT_GT(delta, 0.0);
  EXPECT_LE(std::fabs(delta), 0.20 * 0.05 + 1e-9);
}

// A large yaw jump (> snap_yaw_rad) snaps immediately.
TEST(AnchorSlew, LargeYawJumpSnaps)
{
  fg::AnchorSlewCfg cfg = DefaultCfg();
  bool pub_valid = true;
  double px = 0.0, py = 0.0, pyaw = 0.0, ox, oy, oyaw;

  fg::AnchorSlewStep(pub_valid, px, py, pyaw, true, 0.0, 0.0, 0.6, 0.05, cfg, ox, oy, oyaw);

  EXPECT_DOUBLE_EQ(oyaw, 0.6);  // 0.6 rad > 0.35 snap_yaw → snap
}
