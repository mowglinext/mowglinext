// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Start-up hold for a (re)started behaviour tree. Pure logic, no ROS, so it is unit-testable
// standalone (see test_boot_hold.cpp).
//
// A restarted behavior_tree_node does not know what its dead predecessor left running: a Nav2 goal
// it sent keeps driving (the action server owns it, not the client), and the blade stays on until
// the firmware's own blade timeout. Neither the bridge nor the tree sees that the instance changed,
// so the new tree has to clean up before it ticks: blade OFF and cancel every motion goal, then
// start from a clean slate. The hold runs each step until it has taken effect (a service or action
// server that is not up yet is retried) and gives up after `timeout`, so a cold boot (nothing
// running yet, nothing to clean up) starts at most `timeout` late.
//
// Steps are grouped: a step only runs once every step of the earlier groups has succeeded. That
// orders "cancel the parent goal, wait for its answer" before "sweep the controller goals it
// would otherwise keep spawning".

#ifndef MOWGLI_BEHAVIOR__BOOT_HOLD_HPP_
#define MOWGLI_BEHAVIOR__BOOT_HOLD_HPP_

#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace mowgli_behavior
{

class BootHold
{
public:
  using Clock = std::chrono::steady_clock;
  /// Returns true once the step has been carried out; false asks to be retried.
  using Step = std::function<bool()>;

  /// `timeout <= 0` disables the hold: it is done from the start.
  BootHold(Clock::time_point start, Clock::duration timeout)
      : deadline_(start + timeout), disabled_(timeout <= Clock::duration::zero())
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

  /// Runs every step that has not succeeded yet, in order. Returns true when the hold is over:
  /// every step has succeeded, or the deadline has passed (steps still pending are then given up).
  bool poll(Clock::time_point now)
  {
    if (done_)
    {
      return true;
    }
    if (disabled_)
    {
      done_ = true;
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
      done_ = true;
      timed_out_ = true;
    }
    return done_;
  }

  bool done() const
  {
    return done_ || disabled_;
  }
  bool timed_out() const
  {
    return timed_out_;
  }

  /// Names of the steps that never succeeded (for the log line when the hold gives up).
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
  bool disabled_{false};
  bool done_{false};
  bool timed_out_{false};
};

}  // namespace mowgli_behavior

#endif  // MOWGLI_BEHAVIOR__BOOT_HOLD_HPP_
