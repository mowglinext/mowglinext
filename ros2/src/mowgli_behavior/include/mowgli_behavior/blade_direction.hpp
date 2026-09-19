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

#include <cstdint>
#include <optional>
#include <random>

namespace mowgli_behavior
{

/// One direction per BT mowing session, shared by coverage and manual mowing.
/// Selection describes a REQUEST, not measured rotation. Firmware must enforce
/// a stopped reversal: a ROS2 restart can select a different direction.
class BladeDirection
{
public:
  BladeDirection() : random_(std::random_device{}())
  {
  }
  explicit BladeDirection(uint32_t seed) : random_(seed)
  {
  }

  uint8_t forCommand(bool enabled, bool auto_reverse)
  {
    // OFF must neither select nor change direction. Repeated ONs and temporary
    // OFFs (transits, guards, recharge) retain the session's first selection.
    if (enabled && !direction_)
    {
      direction_ = auto_reverse
                       ? static_cast<uint8_t>(std::uniform_int_distribution<int>(0, 1)(random_))
                       : 0u;
    }
    return direction_.value_or(0u);
  }

  struct Command
  {
    uint8_t enabled;
    uint8_t direction;
  };

  Command forMowerCommand(bool enabled, bool auto_reverse)
  {
    // Record even an OFF whose hardware service is temporarily unavailable.
    requested_enabled_ = enabled;
    const bool effective = enabled && !operator_inhibit_;
    return {static_cast<uint8_t>(effective), forCommand(effective, auto_reverse)};
  }

  Command forOperatorCommand(bool enabled, uint8_t direction)
  {
    // An explicit menu choice wins over the random selection until EndSession.
    // It cannot override a tree OFF (idle, transit, docking or safety guard).
    // Choosing Forward/Reverse while idle deliberately preselects the next
    // enable direction; it never starts an idle blade.
    operator_inhibit_ = !enabled;
    if (enabled)
      direction_ = direction;
    return {static_cast<uint8_t>(requested_enabled_ && !operator_inhibit_),
            direction_.value_or(0u)};
  }

  // An explicit mowing start renews tree permission, without rerolling the
  // session direction. Automatic continuations must never clear operator OFF.
  void clearOperatorInhibit()
  {
    operator_inhibit_ = false;
  }

  void endSession()
  {
    direction_.reset();
    operator_inhibit_ = false;
    requested_enabled_ = false;
  }

private:
  std::mt19937 random_;
  std::optional<uint8_t> direction_;
  bool requested_enabled_{false};
  bool operator_inhibit_{false};
};

}  // namespace mowgli_behavior
