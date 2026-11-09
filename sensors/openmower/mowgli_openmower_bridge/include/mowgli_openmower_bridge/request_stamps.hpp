// SPDX-License-Identifier: GPL-3.0
/**
 * @file request_stamps.hpp
 * @brief When a request/response controller actually sampled its values.
 *
 * The xESC mini (VESC protocol) answers COMM_GET_VALUES: the values it sends
 * were measured when the REQUEST arrived, but the bridge only sees the reply
 * at its next poll. Stamping a reply with the poll time is therefore one
 * interval late, and after a stall it pairs a whole stall's worth of ticks
 * with one normal interval — a 4x phantom speed in the simulation. Replies
 * come back in order, so each one takes the time of the oldest request still
 * outstanding.
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <deque>

namespace mowgli_openmower_bridge
{

class RequestStampQueue
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  /// Outstanding requests kept; older ones were lost on the wire.
  static constexpr std::size_t kCapacity = 8u;

  void OnRequest(TimePoint sent)
  {
    pending_.push_back(sent);
    while (pending_.size() > kCapacity)
    {
      pending_.pop_front();
    }
  }

  /// Sample time of the reply read at @p now (now when none is outstanding).
  [[nodiscard]] TimePoint OnResponse(TimePoint now)
  {
    if (pending_.empty())
    {
      return now;
    }
    const TimePoint sent = pending_.front();
    pending_.pop_front();
    return sent;
  }

  void Clear()
  {
    pending_.clear();
  }

  [[nodiscard]] std::size_t outstanding() const noexcept
  {
    return pending_.size();
  }

private:
  std::deque<TimePoint> pending_;
};

}  // namespace mowgli_openmower_bridge
