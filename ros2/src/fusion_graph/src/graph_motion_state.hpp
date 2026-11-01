// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <deque>
#include <set>

#include "fusion_graph/conditional_process_bridge.hpp"

namespace fusion_graph
{
struct SourceBootstrapState;

struct GraphMotionState
{
  std::shared_ptr<SourceBootstrapState> bootstrap;
  uint64_t epoch = 0;
  bool baseline_set = false;
  int64_t baseline_ns = 0;
  int64_t consumed_ns = 0;
  int64_t committed_ns = 0;
  int64_t schedule_ns = 0;
  int64_t active_schedule_ns = 0;
  std::optional<int64_t> active_end;
  uint64_t revision = 0;
  uint64_t gnss_factors = 0;
  uint64_t yaw_factors = 0;
  uint64_t next_acquisition = 0;
  std::optional<MotionSourceId> last_committed_gyro;
  std::set<MotionSourceId> pending_gyro_ids;
  std::deque<ConsumedMotionStep> pending_steps;
  MotionSourceHistory validation{100000};
  std::shared_ptr<MotionSourceHistory> active_history;
  struct Edge
  {
    struct ConditionalInsertion
    {
      gtsam::Key key;
      int64_t source_ns;
      gtsam::NonlinearFactor::shared_ptr factor;
    };
    uint64_t owner;
    std::shared_ptr<ConditionalProcessBridge> bridge;
    std::map<gtsam::Key, int64_t> acquisitions;
    // Actual insertion order and immutable factor ownership, not timestamp order.
    std::vector<ConditionalInsertion> conditionals;
  };
  std::deque<Edge> edges;
  bool relative_conditionals = false;
  struct Observation
  {
    gtsam::Vector2 xy;
    double sigma;
    bool robust;
    int64_t source_ns;
    uint64_t id;
    std::optional<double> yaw;
  };
  std::deque<Observation> observations;
  // IDs are retained only over the supported source-time window.
  std::map<uint64_t, int64_t> observation_ids;
  std::map<uint64_t, int64_t> yaw_observation_ids;
};
}  // namespace fusion_graph
