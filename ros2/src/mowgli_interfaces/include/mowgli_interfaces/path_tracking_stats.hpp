// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// Shared reduction of Nav2's per-tick path-tracking error into one summary.
//
// ROS 2 Lyrical's controller_server measures the tracking error itself, for
// WHICHEVER controller is loaded (FTC on the coverage lane, RotationShim+RPP on
// transit), and publishes it on `<controller_server>/tracking_feedback` as well
// as inside the FollowPath action feedback. That is the first objective measure
// of mowing quality this robot has had: a swath tiled at `tool_width -
// swath_overlap` leaves an uncut band as soon as the lateral error exceeds the
// 0.02 m overlap, and until now nothing on the robot could say whether that was
// happening.
//
// Two consumers reduce that stream and neither wants the samples themselves:
// diagnostics_node folds it into a 1 Hz DiagnosticStatus, and the behavior
// tree's FollowStrip logs one line per driven sub-path. Hence this header
// rather than two accumulators that would drift apart.
//
// Sign convention is Nav2's and is preserved: a POSITIVE position error means
// the robot is to the LEFT of the path, a positive heading error means the robot
// heading is rotated to the RIGHT of the path direction. Only the absolute value
// is aggregated — a left/right bias averages itself away, which is precisely the
// error an operator would not want hidden — but the last signed sample is kept
// so a caller can still report which side the robot sits on.

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace mowgli_interfaces::path_tracking
{

/// A path index that drops by more than this many poses marks a NEW FollowPath
/// goal rather than a re-plan inside the current one. The controller server
/// restarts its closest-segment search at index 0 for every goal, while within
/// one goal the FeasiblePathHandler only ever walks that index FORWARD; the
/// slack absorbs the one-or-two-pose retreat a mid-goal path update can cause.
inline constexpr std::uint32_t kGoalRestartIndexDrop = 5;

/// One tracking sample, already extracted from nav2_msgs/TrackingFeedback so
/// that this header stays free of ROS types (and therefore unit-testable
/// without a node).
struct Sample
{
  /// Signed lateral error, metres. Positive → robot is left of the path.
  double position_error_m{0.0};
  /// Signed heading error, radians. Positive → heading rotated right of path.
  double heading_error_rad{0.0};
  /// Index of the closest path segment, as reported by the controller server.
  std::uint32_t path_index{0};
};

/// Rolling error summary scoped to ONE FollowPath goal.
///
/// Not thread-safe by itself: callers that feed it from a subscription callback
/// and read it from a timer must hold their own lock, exactly as they already do
/// for the rest of their state snapshot.
class Summary
{
public:
  /// Drop every accumulated sample and start a new episode.
  void Reset()
  {
    count_ = 0;
    sum_abs_position_m_ = 0.0;
    sum_squared_position_m2_ = 0.0;
    max_abs_position_m_ = 0.0;
    max_abs_heading_rad_ = 0.0;
    last_position_error_m_ = 0.0;
    last_path_index_ = 0;
    has_index_ = false;
  }

  /// Accumulate one sample, starting a new episode first when the path index
  /// says the controller server moved on to another goal.
  ///
  /// Non-finite samples are DROPPED rather than propagated: a single NaN would
  /// poison the mean and the max for the rest of the goal, and the resulting
  /// "tracking error: nan" tells an operator strictly less than the previous
  /// sample did.
  void Add(const Sample& sample)
  {
    if (!std::isfinite(sample.position_error_m) || !std::isfinite(sample.heading_error_rad))
    {
      return;
    }

    if (IsNewGoal(sample.path_index))
    {
      Reset();
    }

    const double abs_position = std::fabs(sample.position_error_m);
    const double abs_heading = std::fabs(sample.heading_error_rad);

    ++count_;
    sum_abs_position_m_ += abs_position;
    sum_squared_position_m2_ += abs_position * abs_position;
    if (abs_position > max_abs_position_m_)
    {
      max_abs_position_m_ = abs_position;
    }
    if (abs_heading > max_abs_heading_rad_)
    {
      max_abs_heading_rad_ = abs_heading;
    }
    last_position_error_m_ = sample.position_error_m;
    last_path_index_ = sample.path_index;
    has_index_ = true;
  }

  bool HasSamples() const
  {
    return count_ > 0;
  }

  std::size_t Count() const
  {
    return count_;
  }

  double MaxAbsPositionErrorM() const
  {
    return max_abs_position_m_;
  }

  double MeanAbsPositionErrorM() const
  {
    return count_ > 0 ? sum_abs_position_m_ / static_cast<double>(count_) : 0.0;
  }

  /// Root-mean-square lateral error. Reported alongside the mean because a mower
  /// that tracks perfectly then swings wide once has a good mean and a bad RMS,
  /// and it is the excursion that leaves the uncut band.
  double RmsPositionErrorM() const
  {
    return count_ > 0 ? std::sqrt(sum_squared_position_m2_ / static_cast<double>(count_)) : 0.0;
  }

  double MaxAbsHeadingErrorRad() const
  {
    return max_abs_heading_rad_;
  }

  /// Last SIGNED lateral error — keeps the left/right information the absolute
  /// aggregates deliberately discard.
  double LastPositionErrorM() const
  {
    return last_position_error_m_;
  }

  std::uint32_t LastPathIndex() const
  {
    return last_path_index_;
  }

private:
  bool IsNewGoal(const std::uint32_t path_index) const
  {
    if (!has_index_)
    {
      return false;
    }
    return path_index + kGoalRestartIndexDrop < last_path_index_;
  }

  std::size_t count_{0};
  double sum_abs_position_m_{0.0};
  double sum_squared_position_m2_{0.0};
  double max_abs_position_m_{0.0};
  double max_abs_heading_rad_{0.0};
  double last_position_error_m_{0.0};
  std::uint32_t last_path_index_{0};
  bool has_index_{false};
};

}  // namespace mowgli_interfaces::path_tracking
