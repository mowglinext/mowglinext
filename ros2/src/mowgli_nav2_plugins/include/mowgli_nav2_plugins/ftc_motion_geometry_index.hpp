// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include <memory>
#include <vector>

#include "mowgli_interfaces/motion_geometry_types.hpp"
namespace mowgli_interfaces::motion
{
struct Snapshot;
}

namespace mowgli_nav2_plugins
{
// Immutable original-edge index. Only candidate selection is accelerated;
// existing boundary/parity/crossing arithmetic remains the authority.
class MotionRegionIndex
{
  struct Data;

public:
  // A bounded per-decision cache. Copies of its shared handle serialize access;
  // prepared geometry itself stays immutable and has no cross-decision state.
  class Query
  {
  public:
    ~Query();
    bool authorized(mowgli_interfaces::motion::Point point);
    bool segmentAuthorized(mowgli_interfaces::motion::Point from,
                           mowgli_interfaces::motion::Point to);

  private:
    friend class MotionRegionIndex;
    explicit Query(std::shared_ptr<const Data> data);
    struct State;
    std::unique_ptr<State> state_;
  };
  std::shared_ptr<Query> newQuery() const;
  explicit MotionRegionIndex(const std::vector<mowgli_interfaces::motion::Ring>& rings);
  explicit MotionRegionIndex(std::shared_ptr<const mowgli_interfaces::motion::Snapshot> snapshot);
  bool matches(const mowgli_interfaces::motion::Snapshot* snapshot) const;
  bool authorized(mowgli_interfaces::motion::Point point) const;
  bool segmentAuthorized(mowgli_interfaces::motion::Point from,
                         mowgli_interfaces::motion::Point to) const;

private:
  std::shared_ptr<const Data> data_;
};
}  // namespace mowgli_nav2_plugins
