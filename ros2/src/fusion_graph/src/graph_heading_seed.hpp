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
    RetimedPrior,
    CoupledSourceObservations
  };
  // Birth likelihood identity. A coupled family's heading may later be
  // rekeyed; explicit recovery rejects that provenance before using this.
  gtsam::NonlinearFactor::shared_ptr prior;
  // Original antenna likelihood retained independently of heading provenance.
  gtsam::NonlinearFactor::shared_ptr source_position;
  HeadingProvenance heading_provenance = HeadingProvenance::PosePrior;
  uint64_t source_gnss_id = 0;
  std::optional<uint64_t> source_heading_id;
  std::optional<int64_t> source_heading_ns;
  gtsam::Pose2 pose;
  double sigma_xy = 0.0;
  uint64_t epoch = 0;
  int64_t source_ns = 0;
  bool source_bound = false;
};
}  // namespace fusion_graph
