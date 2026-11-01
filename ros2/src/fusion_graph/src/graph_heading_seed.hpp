// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace fusion_graph
{
struct HeadingSeedState
{
  gtsam::NonlinearFactor::shared_ptr prior;
  // Non-null for a raw observation seed: its separate antenna likelihood
  // remains intact when only the explicitly invalid heading is retired.
  gtsam::NonlinearFactor::shared_ptr source_position;
  uint64_t source_gnss_id = 0;
  uint64_t source_heading_id = 0;
  gtsam::Pose2 pose;
  double sigma_xy = 0.0;
  uint64_t epoch = 0;
  int64_t source_ns = 0;
  bool source_bound = false;
};
}  // namespace fusion_graph
