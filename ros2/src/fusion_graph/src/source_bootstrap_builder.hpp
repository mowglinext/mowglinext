// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "fusion_graph/shared_source_process_family.hpp"
#include "graph_motion_state.hpp"
#include "process_window_policy.hpp"
#include <gtsam/nonlinear/ISAM2.h>

namespace fusion_graph
{
struct SourceBootstrapBuildRequest
{
  GraphParams params;
  uint64_t epoch = 0;
  int64_t source_s_ns = 0;
  int64_t frontier_ns = 0;
  bool promote = false;
  int64_t schedule_ns = 0;
  int64_t initial_schedule_ns = 0;
  // Original consumed records, with original producer IDs, support and bias
  // references. Window clipping changes consumed support only.
  std::vector<ConsumedMotionStep> original_steps;
  GraphMotionState::Observation position;
  GraphMotionState::Observation heading;
  std::vector<GraphMotionState::Observation> additional_observations;
  ProcessWindowPolicyState public_policy_baseline;
  // Required exactly when frontier > S. Includes the caller's actual schedule
  // times and raw source-S accumulator, independent of source-span duration.
  std::optional<ProcessWindowPolicyInput> suffix_policy_input;
  std::optional<gtsam::Key> prefix_left_key;
  uint64_t next_acquisition = 0;
  gtsam::NonlinearFactor::shared_ptr preserved_primary_gnss;
  // Existing absolute/legacy factors are retained by pointer, once each.
  gtsam::NonlinearFactorGraph extra_factors;
};

struct SourceBootstrapRawBinding
{
  bool yaw;
  uint64_t id;
  int64_t source_ns;
  gtsam::Key key;
  gtsam::NonlinearFactor::shared_ptr factor;
};

struct SourceBootstrapPrefixPolicy
{
  // Private source-span recipe: last schedule = 0, current schedule = S-T.
  ProcessWindowPolicyInput input;
  ProcessWindowPolicyResult selection;
};

struct SourceBootstrapCandidate
{
  std::unique_ptr<gtsam::ISAM2> isam;
  gtsam::Values values;
  std::shared_ptr<SharedSourceProcessFamily> family;
  gtsam::Pose2 source_s_pose;
  gtsam::Matrix3 source_s_covariance;
  gtsam::Pose2 frontier_pose;
  gtsam::Matrix3 frontier_covariance;
  gtsam::Key frontier_key = 0;
  std::optional<gtsam::Key> prefix_key;
  uint64_t next_acquisition = 0;
  std::vector<SourceBootstrapRawBinding> raw_bindings;
  // Every nuisance pose, including private window endpoints, belongs to the
  // first public process edge when the caller publishes promotion.
  std::map<gtsam::Key, int64_t> auxiliary_poses;
  std::optional<ProcessWindowPolicyResult> suffix_policy;
  std::optional<SourceBootstrapPrefixPolicy> prefix_policy;
};

// Builds and optimizes a complete isolated cross-time raw-observation family.
// Invalid support, identities, policy recipes or solver results throw; no live
// graph, admission ID, counter, queue, or policy state is changed.
SourceBootstrapCandidate BuildSourceBootstrapCandidate(const SourceBootstrapBuildRequest& request);
}  // namespace fusion_graph
