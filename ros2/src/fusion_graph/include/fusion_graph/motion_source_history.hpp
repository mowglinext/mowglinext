// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <tuple>

#include <gtsam/geometry/Pose2.h>

namespace fusion_graph
{

enum class MotionSourceKind
{
  Wheel,
  Gyro
};

// Sequence identifies an accepted source payload in the caller's epoch. A header stamp alone
// does not establish physical acquisition identity, calibration, or independence of source errors.
struct MotionSourceId
{
  MotionSourceKind kind = MotionSourceKind::Wheel;
  uint64_t sequence = 0;
  int64_t observation_stamp_ns = 0;

  bool operator==(const MotionSourceId&) const = default;
  bool operator<(const MotionSourceId& other) const
  {
    return std::tie(kind, sequence, observation_stamp_ns) <
           std::tie(other.kind, other.sequence, other.observation_stamp_ns);
  }
};

// Caller-provided support provenance; a cached wheel observation need not cover the interval in
// which it was consumed. No freshness or physical acquisition claim is inferred from this support.
struct MotionSourceSupport
{
  int64_t start_ns = 0;
  int64_t end_ns = 0;
  bool operator==(const MotionSourceSupport&) const = default;
};

struct MotionWheelSource
{
  MotionSourceId id;
  MotionSourceSupport support;
  // Immutable raw cached body rates, before the consumed step's deterministic application gate.
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;  // Raw held wheel yaw rate for compatibility gate replay at bootstrap.
  bool operator==(const MotionWheelSource&) const = default;
};

struct MotionGyroSource
{
  MotionSourceId id{MotionSourceKind::Gyro, 0, 0};
  MotionSourceSupport support;
  double raw_z = 0.0;
  bool operator==(const MotionGyroSource&) const = default;
};

struct ConsumedMotionStep
{
  uint64_t epoch = 0;
  int64_t start_ns = 0;
  int64_t end_ns = 0;
  MotionWheelSource wheel;
  MotionGyroSource gyro;
  double bias_linearization = 0.0;
  // Transforms raw-rate errors only; disabling translation does not assert zero slip/fault noise.
  bool wheel_translation_enabled = true;
  // Deterministic stationary mean policy; endpoint stationary/fault noise is modeled separately.
  bool gyro_rotation_enabled = true;
  bool operator==(const ConsumedMotionStep&) const = default;
};

enum class MotionHistoryStatus
{
  Ok,
  Duplicate,
  WrongEpoch,
  InvalidInput,
  SourceConflict,
  RetiredSource,
  Gap,
  Overlap,
  CapacityExceeded,
  Uncovered
};

struct MotionTrajectory
{
  MotionHistoryStatus status = MotionHistoryStatus::Uncovered;
  uint64_t epoch = 0;
  gtsam::Pose2 mean;
  // Right-tangent derivatives of mean wrt ADDITIVE source rate errors (m/s, m/s, rad/s).
  // Each unique source appears once, including when shared across multiple steps or queries.
  std::map<MotionSourceId, gtsam::Matrix32> wheel_jacobians;
  std::map<MotionSourceId, gtsam::Vector3> gyro_jacobians;
  // One common additive bias perturbation about the recorded per-step bias linearizations.
  gtsam::Vector3 bias_jacobian = gtsam::Vector3::Zero();
};

// Bounded ledger of intervals ACTUALLY consumed, not interpolation between optimized graph poses.
// The consumed-hold model is a constant body twist (wheel_gain*vx, wheel_gain*vy,
// gyro_gain*(raw_z-bias)), integrated per step with Pose2 Expmap. This differs from the node's
// yaw-first Euler model: production must use consistent integration before these trajectories can
// claim to replay its actual DR.
// Producer support is retained separately from consumed support.
// The caller serializes access and supplies an epoch reset and its oldest-needed source time.
// A rejected gap leaves the accepted endpoint intact; continuation after a missing interval needs
// a caller-controlled Reset with a new integration generation. That is not a new physical
// acquisition epoch and is never inferred from a guessed timeout.
// Newly introduced source sequences must increase per kind. Still-retained identities can be
// reused; the latest source per kind stays available even after pruning its last interval. Other
// pruned identities cannot be reintroduced. This bounds identity storage without allowing
// a changed replay to replace the first accepted payload. Reset clears both ledger and identities.
class MotionSourceHistory
{
public:
  explicit MotionSourceHistory(size_t capacity) : capacity_(capacity)
  {
  }

  void Reset(uint64_t epoch)
  {
    epoch_ = epoch;
    steps_.clear();
    wheels_.clear();
    gyros_.clear();
    last_end_.reset();
    latest_wheel_.reset();
    latest_gyro_.reset();
  }

  MotionHistoryStatus Append(const ConsumedMotionStep& step)
  {
    if (step.epoch != epoch_)
      return MotionHistoryStatus::WrongEpoch;
    if (step.end_ns <= step.start_ns || step.wheel.id.kind != MotionSourceKind::Wheel ||
        step.gyro.id.kind != MotionSourceKind::Gyro ||
        step.wheel.support.end_ns < step.wheel.support.start_ns ||
        step.gyro.support.end_ns < step.gyro.support.start_ns || !std::isfinite(step.wheel.vx) ||
        !std::isfinite(step.wheel.vy) || !std::isfinite(step.wheel.wz) ||
        !std::isfinite(step.gyro.raw_z) || !std::isfinite(step.bias_linearization) ||
        !std::isfinite(step.gyro.raw_z - step.bias_linearization))
      return MotionHistoryStatus::InvalidInput;

    const auto wheel = wheels_.find(step.wheel.id);
    const auto gyro = gyros_.find(step.gyro.id);
    const bool latest_wheel = latest_wheel_ && latest_wheel_->id == step.wheel.id;
    const bool latest_gyro = latest_gyro_ && latest_gyro_->id == step.gyro.id;
    if ((latest_wheel && *latest_wheel_ != step.wheel) ||
        (latest_gyro && *latest_gyro_ != step.gyro) ||
        (wheel != wheels_.end() && wheel->second.source != step.wheel) ||
        (gyro != gyros_.end() && gyro->second.source != step.gyro))
      return MotionHistoryStatus::SourceConflict;
    const auto old = std::lower_bound(steps_.begin(),
                                      steps_.end(),
                                      step.start_ns,
                                      [](const auto& entry, int64_t start)
                                      {
                                        return entry.start_ns < start;
                                      });
    if (old != steps_.end() && *old == step)
      return MotionHistoryStatus::Duplicate;
    const bool wheel_known = latest_wheel || wheel != wheels_.end();
    const bool gyro_known = latest_gyro || gyro != gyros_.end();
    if ((!wheel_known && latest_wheel_ && step.wheel.id.sequence <= latest_wheel_->id.sequence) ||
        (!gyro_known && latest_gyro_ && step.gyro.id.sequence <= latest_gyro_->id.sequence))
      return MotionHistoryStatus::RetiredSource;
    if (last_end_ && step.start_ns != *last_end_)
      return step.start_ns > *last_end_ ? MotionHistoryStatus::Gap : MotionHistoryStatus::Overlap;
    if (steps_.size() >= capacity_)
      return MotionHistoryStatus::CapacityExceeded;

    steps_.push_back(step);
    ++wheels_.try_emplace(step.wheel.id, WheelEntry{step.wheel, 0}).first->second.references;
    ++gyros_.try_emplace(step.gyro.id, GyroEntry{step.gyro, 0}).first->second.references;
    last_end_ = step.end_ns;
    if (!wheel_known)
      latest_wheel_ = step.wheel;
    if (!gyro_known)
      latest_gyro_ = step.gyro;
    return MotionHistoryStatus::Ok;
  }

  MotionHistoryStatus PruneBefore(uint64_t epoch, int64_t oldest_needed_ns)
  {
    if (epoch != epoch_)
      return MotionHistoryStatus::WrongEpoch;
    while (!steps_.empty() && steps_.front().end_ns <= oldest_needed_ns)
    {
      const auto wheel = wheels_.find(steps_.front().wheel.id);
      const auto gyro = gyros_.find(steps_.front().gyro.id);
      if (--wheel->second.references == 0)
        wheels_.erase(wheel);
      if (--gyro->second.references == 0)
        gyros_.erase(gyro);
      steps_.pop_front();
    }
    return MotionHistoryStatus::Ok;
  }

  size_t size() const
  {
    return steps_.size();
  }

  MotionTrajectory Query(uint64_t epoch, int64_t start_ns, int64_t end_ns) const
  {
    MotionTrajectory result;
    result.epoch = epoch;
    if (epoch != epoch_)
    {
      result.status = MotionHistoryStatus::WrongEpoch;
      return result;
    }
    if (end_ns < start_ns)
    {
      result.status = MotionHistoryStatus::InvalidInput;
      return result;
    }
    if (steps_.empty() || start_ns < steps_.front().start_ns || end_ns > steps_.back().end_ns)
      return result;

    // Only visit overlapping steps. Deque iterators are random access, so both searches are
    // logarithmic even when a delayed query selects a tiny interval in a long retained history.
    const auto first = std::lower_bound(steps_.begin(),
                                        steps_.end(),
                                        start_ns,
                                        [](const auto& entry, int64_t start)
                                        {
                                          return entry.end_ns <= start;
                                        });
    auto stop = std::lower_bound(first,
                                 steps_.end(),
                                 end_ns,
                                 [](const auto& entry, int64_t end)
                                 {
                                   return entry.start_ns < end;
                                 });
    // Right perturbations of each increment are transported by Ad(suffix^-1). Processing from
    // the end visits each step once; accumulated source Jacobians never need retransformation.
    // result.mean holds the already-composed suffix, then becomes the full constant-twist mean.
    while (stop != first)
    {
      const auto& step = *--stop;
      const int64_t begin = std::max(start_ns, step.start_ns);
      const int64_t end = std::min(end_ns, step.end_ns);
      if (end <= begin)
        continue;
      // Subtract after widening: valid signed-nanosecond endpoints may straddle zero.
      const double dt = static_cast<double>(static_cast<long double>(end) - begin) * 1e-9;
      const double wheel_gain = step.wheel_translation_enabled ? 1.0 : 0.0;
      const double gyro_gain = step.gyro_rotation_enabled ? 1.0 : 0.0;
      const gtsam::Vector3 xi(dt * wheel_gain * step.wheel.vx,
                              dt * wheel_gain * step.wheel.vy,
                              dt * gyro_gain * (step.gyro.raw_z - step.bias_linearization));
      if (!xi.allFinite())
      {
        result = MotionTrajectory{};
        result.epoch = epoch;
        result.status = MotionHistoryStatus::InvalidInput;
        return result;
      }
      gtsam::Matrix3 exp_jacobian;
      const auto delta = gtsam::Pose2::Expmap(xi, exp_jacobian);
      const gtsam::Matrix3 rate_jacobian = result.mean.inverse().AdjointMap() * exp_jacobian * dt;
      result.mean = delta.compose(result.mean);
      auto [wheel, inserted_wheel] =
          result.wheel_jacobians.try_emplace(step.wheel.id, gtsam::Matrix32::Zero());
      wheel->second += wheel_gain * rate_jacobian.leftCols<2>();
      auto [gyro, inserted_gyro] =
          result.gyro_jacobians.try_emplace(step.gyro.id, gtsam::Vector3::Zero());
      gyro->second += gyro_gain * rate_jacobian.col(2);
      result.bias_jacobian -= gyro_gain * rate_jacobian.col(2);
    }
    result.status = MotionHistoryStatus::Ok;
    return result;
  }

private:
  struct WheelEntry
  {
    MotionWheelSource source;
    size_t references;
  };
  struct GyroEntry
  {
    MotionGyroSource source;
    size_t references;
  };
  const size_t capacity_;
  uint64_t epoch_ = 0;
  std::deque<ConsumedMotionStep> steps_;
  std::map<MotionSourceId, WheelEntry> wheels_;
  std::map<MotionSourceId, GyroEntry> gyros_;
  std::optional<int64_t> last_end_;
  std::optional<MotionWheelSource> latest_wheel_;
  std::optional<MotionGyroSource> latest_gyro_;
};

}  // namespace fusion_graph
