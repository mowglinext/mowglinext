// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstdint>
#include <optional>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace fusion_graph
{
struct HeadingSeedState
{
  enum class HeadingProvenance
  {
    PosePrior,
    RawSourceObservation,
    RetimedPrior
  };
  gtsam::NonlinearFactor::shared_ptr prior;
  // Separate original antenna likelihood for both observation bootstraps.
  gtsam::NonlinearFactor::shared_ptr source_position;
  HeadingProvenance heading_provenance = HeadingProvenance::PosePrior;
  uint64_t source_gnss_id = 0;
  std::optional<uint64_t> source_heading_id;
  gtsam::Pose2 pose;
  double sigma_xy = 0.0;
  uint64_t epoch = 0;
  int64_t source_ns = 0;
  bool source_bound = false;
};
}  // namespace fusion_graph
