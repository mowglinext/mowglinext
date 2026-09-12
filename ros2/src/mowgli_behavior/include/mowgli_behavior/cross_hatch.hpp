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

#include <optional>

namespace mowgli_behavior
{

// Session orientation is persisted with the coverage cursor. Replanning,
// pauses and reboots must not rotate a partially mowed plan underneath it.
struct CrossHatch
{
  bool next_perpendicular{false};
  std::optional<bool> session_perpendicular;
  bool alternate_session{false};
  bool used{false};
  std::optional<bool> next_override;
  unsigned failed_sessions{0};
  bool planning_failed{false};

  bool next() const
  {
    if (next_override)
      return *next_override;
    return session_perpendicular && alternate_session && used ? !*session_perpendicular
                                                              : next_perpendicular;
  }

  bool begin(bool enabled)
  {
    if (!session_perpendicular)
    {
      if (enabled && next_override)
      {
        next_perpendicular = *next_override;
        next_override.reset();
      }
      session_perpendicular = enabled && next_perpendicular;
      alternate_session = enabled;
      used = false;
      planning_failed = false;
    }
    return *session_perpendicular;
  }

  void finish()
  {
    if (used || next_override)
      failed_sessions = 0;
    else if (alternate_session && planning_failed && failed_sessions < 1000)
      ++failed_sessions;
    // A failed plan, manual-only run or repeated EndSession does not consume
    // an orientation. A coverage run that started and was then abandoned does.
    next_perpendicular = next();
    next_override.reset();
    session_perpendicular.reset();
    alternate_session = false;
    used = false;
    planning_failed = false;
  }
};

}  // namespace mowgli_behavior
