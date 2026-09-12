// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <deque>
#include <optional>

#include <gtsam/geometry/Pose2.h>
namespace fusion_graph
{
// Bounded odometry history in a continuous frame. Never extrapolate a scan.
class LidarScanHistory
{
public:
  struct Sample
  {
    double stamp;
    gtsam::Pose2 odom;
  };
  struct Node
  {
    double stamp;
    uint64_t index;
    gtsam::Pose2 odom;
  };
  struct Match
  {
    uint64_t index;
    gtsam::Pose2 offset;
    gtsam::Pose2 odom;
  };
  void Clear()
  {
    samples_.clear();
    nodes_.clear();
  }
  bool Push(double stamp, const gtsam::Pose2& odom)
  {
    const bool rewind = !samples_.empty() && stamp < samples_.back().stamp;
    if (rewind)
      Clear();
    if (samples_.empty() || stamp > samples_.back().stamp)
      samples_.push_back({stamp, odom});
    while (samples_.size() > 2 && samples_[1].stamp < stamp - 5.0)
      samples_.pop_front();
    while (!nodes_.empty() && nodes_.front().stamp < stamp - 5.0)
      nodes_.pop_front();
    return rewind;
  }
  void AddNode(double stamp, uint64_t index, const gtsam::Pose2& odom)
  {
    if (!nodes_.empty() && index <= nodes_.back().index)
      nodes_.clear();
    if (!nodes_.empty() && stamp == nodes_.back().stamp)
      nodes_.back() = {stamp, index, odom};
    else
      nodes_.push_back({stamp, index, odom});
  }
  std::optional<Match> At(double stamp, double now, double max_age) const
  {
    if (!std::isfinite(stamp) || stamp <= 0.0 || stamp > now || now - stamp > max_age ||
        samples_.empty() || stamp < samples_.front().stamp || stamp > samples_.back().stamp)
      return std::nullopt;
    auto odom = samples_.front().odom;
    for (std::size_t i = 1; i < samples_.size(); ++i)
    {
      const auto& a = samples_[i - 1];
      const auto& b = samples_[i];
      if (stamp <= b.stamp)
      {
        if (b.stamp - a.stamp > 0.25)
          return std::nullopt;
        const double u = (stamp - a.stamp) / (b.stamp - a.stamp);
        odom =
            a.odom.compose(gtsam::Pose2::Expmap(u * gtsam::Pose2::Logmap(a.odom.between(b.odom))));
        break;
      }
    }
    for (auto it = nodes_.rbegin(); it != nodes_.rend(); ++it)
      if (it->stamp <= stamp)
        return Match{it->index, it->odom.between(odom), odom};
    return std::nullopt;
  }

private:
  std::deque<Sample> samples_;
  std::deque<Node> nodes_;
};
}  // namespace fusion_graph
