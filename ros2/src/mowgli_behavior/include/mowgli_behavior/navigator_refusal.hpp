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
#pragma once

#include <chrono>
#include <optional>

namespace mowgli_behavior
{

/// Bounded, spaced re-sends of a navigation goal that the navigator REFUSED.
///
/// bt_navigator refuses a goal with no result and no error code while it is busy with another
/// navigation, and the refusal costs only milliseconds. Field 2026-10-09 (issue #943): a goal
/// nobody owned kept it busy for seven minutes, and GetNextUnmowedArea spent its whole
/// five-dispatch budget on instant refusals in about one second, gave the area up at 98 % and
/// left the mower idle on the lawn. The refusing node cancels the stale goal
/// (cancelAllGoalsQuietly); this policy gives the navigator time to wind it down before the
/// next attempt instead of failing the attempt on the spot.
///
/// Pure (time is passed in) so the policy is unit-tested without ROS.
class NavigatorRefusalRetry
{
public:
  using Clock = std::chrono::steady_clock;

  /// Time for the navigator to halt a cancelled goal (its BT, a running recovery).
  static constexpr double kRetryDelaySec = 2.0;
  /// Re-sends per dispatch: about ten seconds before the refusal counts as a real failure.
  static constexpr int kMaxRetries = 5;

  /// A refusal was received at `now`. Returns true when a re-send is scheduled, false when the
  /// budget is spent (the caller then fails or skips exactly as before).
  bool onRefused(Clock::time_point now)
  {
    if (retries_ >= kMaxRetries)
    {
      retry_at_.reset();
      return false;
    }
    ++retries_;
    retry_at_ = now + std::chrono::duration_cast<Clock::duration>(
                          std::chrono::duration<double>(kRetryDelaySec));
    return true;
  }

  /// A re-send is scheduled and not yet made.
  bool pending() const
  {
    return retry_at_.has_value();
  }

  /// The scheduled re-send may be made now.
  bool due(Clock::time_point now) const
  {
    return retry_at_.has_value() && now >= *retry_at_;
  }

  /// The scheduled re-send is being made.
  void consume()
  {
    retry_at_.reset();
  }

  /// Re-sends made so far for the current dispatch.
  int retries() const
  {
    return retries_;
  }

  /// A new dispatch (another unit, another start, or a goal the navigator accepted).
  void reset()
  {
    retries_ = 0;
    retry_at_.reset();
  }

private:
  int retries_{0};
  std::optional<Clock::time_point> retry_at_;
};

}  // namespace mowgli_behavior
