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
// Start-up hold for a (re)started behaviour tree. The node holds IDLE until all
// cleanup steps succeed. A deadline reports a delay, and pending steps keep
// running. Groups order upstream action cancellation before child cleanup.

#ifndef MOWGLI_BEHAVIOR__BOOT_HOLD_HPP_
#define MOWGLI_BEHAVIOR__BOOT_HOLD_HPP_

#include <chrono>
#include <filesystem>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace mowgli_behavior
{

/// A launch-owned marker distinguishes a respawn from the first BT in a fresh stack.
/// The first process claims the marker before it can dispatch any action. Errors
/// require cleanup too, because they leave the previous process state unknown.
inline bool boot_cleanup_required(const std::filesystem::path& marker, std::error_code& error)
{
  return !marker.empty() && !std::filesystem::create_directory(marker, error);
}

class BootHold
{
public:
  using Clock = std::chrono::steady_clock;
  /// Returns true once the step has been carried out; false asks to be retried.
  using Step = std::function<bool()>;

  BootHold(Clock::time_point start, Clock::duration warning_after)
      : deadline_(start + warning_after)
  {
  }

  /// Adds a step to the current group.
  void add_step(std::string name, Step step)
  {
    steps_.push_back({std::move(name), std::move(step), false, group_});
  }

  /// Steps added after this call only run once every step added before it has succeeded.
  void next_group()
  {
    ++group_;
  }

  /// Runs pending steps in order. Only completed cleanup releases the hold.
  bool poll(Clock::time_point now)
  {
    if (done_)
    {
      return true;
    }
    bool earlier_done = true;  // every step of the groups before the current one succeeded
    bool group_done = true;
    int group = 0;
    for (auto& s : steps_)
    {
      if (s.group != group)
      {
        earlier_done = earlier_done && group_done;
        group = s.group;
        group_done = true;
      }
      if (!s.done && earlier_done && s.run())
      {
        s.done = true;
      }
      group_done = group_done && s.done;
    }
    if (earlier_done && group_done)
    {
      done_ = true;
    }
    else if (now >= deadline_)
    {
      timed_out_ = true;
    }
    return done_;
  }

  bool done() const
  {
    return done_;
  }
  bool timed_out() const
  {
    return timed_out_;
  }

  /// Names of steps that still need to complete.
  std::vector<std::string> pending() const
  {
    std::vector<std::string> out;
    for (const auto& s : steps_)
    {
      if (!s.done)
      {
        out.push_back(s.name);
      }
    }
    return out;
  }

private:
  struct Entry
  {
    std::string name;
    Step run;
    bool done;
    int group;
  };
  std::vector<Entry> steps_;
  int group_{0};
  Clock::time_point deadline_;
  bool done_{false};
  bool timed_out_{false};
};

}  // namespace mowgli_behavior

#endif  // MOWGLI_BEHAVIOR__BOOT_HOLD_HPP_
