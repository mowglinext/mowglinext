// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
//
// A dead-reckoning pose that never jumps.
//
// fusion_graph re-bases its odom frame every odom_rebase_dist_m (dr_x_/dr_y_
// snap back to the origin, map→odom absorbs the difference) and resets it on
// set_pose, clear_graph and the dock re-anchor. The fused pose stays
// continuous; the raw dead reckoning does not. The particle filter's motion
// model and the dead-reckoning plausibility witness both need CONTINUOUS
// odometry: fed the raw value, Beluga applied the 6 m re-base as a delta to
// every particle (teleport, field 2026-09-07 at t+105 s and t+166 s) and the
// witness moved with it, so the wrong estimate looked plausible.
//
// This integrates the raw pose's deltas and drops any single step no robot
// can make in one tick. Pure, tested.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include <sophus/se2.hpp>

namespace fusion_graph
{

class ContinuousOdom
{
public:
  explicit ContinuousOdom(double max_step_m = 0.5, double max_step_rad = 1.0)
      : max_step_m_(max_step_m), max_step_rad_(max_step_rad)
  {
  }

  // Feed the raw dead-reckoning pose; returns the continuous pose.
  const Sophus::SE2d& Advance(const Sophus::SE2d& raw)
  {
    if (prev_raw_)
    {
      const Sophus::SE2d delta = prev_raw_->inverse() * raw;
      const bool jump =
          delta.translation().norm() > max_step_m_ || std::fabs(delta.so2().log()) > max_step_rad_;
      if (jump)
        ++rebases_;  // a re-base or reset, not motion: keep the pose, drop the step
      else
        pose_ = pose_ * delta;
    }
    prev_raw_ = raw;
    return pose_;
  }

  void RebaseRaw(const Sophus::SE2d& raw)
  {
    prev_raw_ = raw;
    ++rebases_;
  }

  const Sophus::SE2d& pose() const
  {
    return pose_;
  }
  uint64_t rebases() const
  {
    return rebases_;
  }

private:
  double max_step_m_;
  double max_step_rad_;
  std::optional<Sophus::SE2d> prev_raw_;
  Sophus::SE2d pose_;
  uint64_t rebases_ = 0;
};

}  // namespace fusion_graph
