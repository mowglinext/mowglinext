// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "fusion_graph/conditional_process_bridge.hpp"

namespace fusion_graph
{

// Original producer identity/payload/support, followed by its consumed duration in
// this window. Repeated uses of one ID share one rate error; durations sum before
// squaring. Producer support is provenance, not an inferred consumption interval.
struct SharedSourceGyroUse
{
  MotionGyroSource source;
  double duration_s = 0.0;
};

struct SharedSourceGyroWindow
{
  double raw_integral_rad = 0.0;
  double duration_s = 0.0;
  double sigma_floor_rad = 1e-4;
  std::vector<SharedSourceGyroUse> sources;
};

struct SharedSourceProcessWindow
{
  // The caller owns selection and supplies an immutable trajectory snapshot.
  // endpoint_covariance is the selected wheel/process diffusion D, not C.
  // Gyro-on trajectories must rotate over the whole consumed window with the
  // right-end bias convention. Their per-ID angular sensitivities must agree
  // with gyro-use metadata; contradictory endpoint/query recipes reject.
  FrozenProcessEdge edge;
  std::optional<SharedSourceGyroWindow> gyro;
};

class UnsupportedSharedSourceConditional : public UnsupportedRelativeConditional
{
public:
  using UnsupportedRelativeConditional::UnsupportedRelativeConditional;
};

// A declared first-order, pinned shared-source model in local relative-error
// coordinates. It is not the old ConditionalProcessBridge kernel, nor a physical
// raw-sensor calibration or globally normalized SE(2) density. With preintegration,
// one correlated gyro factor replaces the independent scalar gyro factors.
// Wheel, bias-prior/RW and absolute-observation factors remain caller-owned.
class SharedSourceProcessFamily
{
public:
  struct Acquisition
  {
    int64_t source_ns;
    gtsam::Key key;
  };
  struct Insertion
  {
    gtsam::Key key;
    std::optional<gtsam::Pose2> initial_value;
    gtsam::NonlinearFactor::shared_ptr factor;
  };

  // One or two contiguous windows, all with gyro metadata or all without it.
  // The no-gyro case has no bias keys or rate variances, and reduces to the
  // per-window Brownian bridge. Neither case makes policy/clock decisions.
  explicit SharedSourceProcessFamily(std::vector<SharedSourceProcessWindow> windows);
  ~SharedSourceProcessFamily();
  SharedSourceProcessFamily(const SharedSourceProcessFamily& other);
  SharedSourceProcessFamily& operator=(const SharedSourceProcessFamily& other);
  SharedSourceProcessFamily(SharedSourceProcessFamily&&) noexcept;
  SharedSourceProcessFamily& operator=(SharedSourceProcessFamily&&) noexcept;

  gtsam::NonlinearFactor::shared_ptr endpoint_gyro_factor() const;
  Insertion Insert(int64_t source_ns,
                   gtsam::Key new_key,
                   const gtsam::Values* current_values = nullptr);
  gtsam::Pose2 ConditionalMean(int64_t source_ns, const gtsam::Values& current_values) const;

  const std::vector<SharedSourceProcessWindow>& windows() const;
  std::vector<Acquisition> acquisitions() const;
  uint64_t source_epoch() const;
  int64_t start_ns() const;
  int64_t end_ns() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace fusion_graph
