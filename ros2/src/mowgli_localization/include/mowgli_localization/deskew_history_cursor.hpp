// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <cstddef>
#include <deque>
#include <limits>

namespace mowgli_localization
{

/// A scan-local interpolation cursor over an immutable sensor history.
/// Increasing ray times advance through the history once. A backwards
/// query restarts the original first-match search (e.g. negative time_increment).
/// The cursor must not outlive a scan or any mutation/trim of its history.
/// Samples need t_s and a double value member supplied by the caller.
/// No sorted-history assumption is added: advancing past b.t_s < a previous
/// query is also safe for an unordered history when queries are nondecreasing.
template <typename Sample>
class DeskewHistoryCursor
{
public:
  DeskewHistoryCursor(const std::deque<Sample>& history, double Sample::*value)
      : history_(history), value_(value)
  {
  }

  bool Interpolate(double t_s, double& out)
  {
    // !(>=) also resets after/for a NaN query, matching the scalar search.
    if (!(t_s >= previous_query_))
    {
      next_ = 1;
    }
    previous_query_ = t_s;
    if (history_.empty())
    {
      out = 0.0;
      return false;
    }
    if (t_s <= history_.front().t_s)
    {
      out = history_.front().*value_;
      return false;
    }
    if (t_s >= history_.back().t_s)
    {
      out = history_.back().*value_;
      return false;
    }
    for (std::size_t i = next_; i < history_.size(); ++i)
    {
      const auto& b = history_[i];
      if (b.t_s >= t_s)
      {
        next_ = i;
        const auto& a = history_[i - 1];
        const double dt = b.t_s - a.t_s;
        if (dt <= 0.0)
        {
          out = b.*value_;
          return true;
        }
        const double f = (t_s - a.t_s) / dt;
        out = a.*value_ + f * (b.*value_ - a.*value_);
        return true;
      }
    }
    out = history_.back().*value_;
    return false;
  }

private:
  const std::deque<Sample>& history_;
  double Sample::*value_;
  std::size_t next_{1};
  double previous_query_{-std::numeric_limits<double>::infinity()};
};

}  // namespace mowgli_localization
