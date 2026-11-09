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
#include <iomanip>
#include <optional>
#include <random>
#include <sstream>
#include <string>

namespace mowgli_behavior
{
// Bounded, process-local ownership: one active tuple and one completed resume.
// All calls are serialized by the owner's callback group and context_mutex.
class FleetYieldState
{
public:
  enum Operation : uint8_t
  {
    QUERY = 0,
    PAUSE = 1,
    RESUME = 2,
    CANCEL = 3
  };
  enum Result : uint8_t
  {
    AVAILABLE = 0,
    HELD = 1,
    RESUMED = 2,
    ALREADY_RESUMED = 3,
    CANCELLED = 4,
    NOT_READY = 5,
    INELIGIBLE = 6,
    BUSY = 7,
    REVOKED = 8,
    INVALID = 9
  };

  static bool autonomous(uint8_t state, const std::string& name)
  {
    // Deliberately conservative: docking, undocking, calibration, recovery,
    // charge/rain/localization holds and session completion are not fleet-owned.
    return state == 2 && (name == "MOWING" || name == "PLANNING" || name == "TRANSIT");
  }

  const std::string& epoch() const
  {
    return epoch_;
  }
  bool held() const
  {
    return owner_.has_value();
  }
  bool completed(const std::string& epoch, const std::string& id) const
  {
    return completed_ && completed_->matches(epoch, id);
  }

  void revoke()
  {
    owner_.reset();
    completed_.reset();
    ready_ = false;
    epoch_ = freshEpoch();
  }

  void observeInterlock(bool blocked)
  {
    // Fence both entry and exit, including QUERYs made during a safety hold.
    // A sustained maintenance gate must not rotate the fence every tick.
    if (blocked != blocked_)
    {
      blocked_ = blocked;
      revoke();
    }
  }

  // Some root guards stop before publishing a new status. Do not restore
  // eligibility from cached MOWING: only a subsequent tree publication can
  // demonstrate that the guard released. Repeated guard ticks are idempotent.
  void safetyHold()
  {
    if (owner_ || completed_ || status_eligible_)
      revoke();
    status_eligible_ = false;
  }

  void observeStatus(uint8_t state, const std::string& name)
  {
    const bool eligible = autonomous(state, name);
    if (owner_)
    {
      if (state == 1 && name == "IDLE")
      {
        ready_ = true;
        status_eligible_ = false;
        return;
      }
      // PublishHighLevelStatus can debounce IDLE's numeric value for two
      // ticks. It is not StopHold proof until the actual value is 1.
      if (!ready_ && (eligible || (state == 2 && name == "IDLE")))
        return;
      revoke();
    }
    if (eligible != status_eligible_)
    {
      if (!eligible)
        revoke();
      else
        epoch_ = freshEpoch();  // Fence QUERYs from the preceding unsafe state.
      status_eligible_ = eligible;
    }
  }

  Result apply(uint8_t operation, const std::string& epoch, const std::string& id, uint8_t& command)
  {
    // A completed replay is an acknowledgement only, even after mowing has
    // resumed. Never rerun START side effects or evaluate motion readiness.
    if (operation == RESUME && completed(epoch, id))
      return ALREADY_RESUMED;
    if (operation > CANCEL)
      return INVALID;
    if (operation == QUERY)
      return AVAILABLE;
    if (epoch.empty() || epoch.size() > 128 || id.empty() || id.size() > 128)
      return INVALID;
    if (blocked_)
      return INELIGIBLE;
    if (epoch != epoch_)
      return REVOKED;
    if (owner_ && !owner_->matches(epoch, id))
      return BUSY;
    if (operation == CANCEL)
    {
      // Also fence an unused epoch: CANCEL may overtake a delayed PAUSE.
      revoke();
      return CANCELLED;
    }
    if (operation == PAUSE)
    {
      if (owner_)
      {
        if (command == 8)
          return HELD;
        revoke();
        return REVOKED;
      }
      if (command != 1 || !status_eligible_)
        return INELIGIBLE;
      owner_ = Identity{epoch, id};
      ready_ = false;
      command = 8;
      return HELD;
    }
    if (!owner_)
      return REVOKED;
    if (command != 8)
    {
      revoke();
      return REVOKED;
    }
    if (!ready_)
      return NOT_READY;
    command = 1;  // Resume the same run; no ordinary START transformations.
    completed_ = owner_;
    owner_.reset();
    ready_ = false;
    epoch_ = freshEpoch();
    return RESUMED;
  }

private:
  struct Identity
  {
    std::string epoch, id;
    bool matches(const std::string& other_epoch, const std::string& other_id) const
    {
      return epoch == other_epoch && id == other_id;
    }
  };
  static std::string freshEpoch()
  {
    // Linux random_device uses the OS entropy source. Random 128-bit fences
    // are opaque, boot-unique and never loaded from coverage persistence.
    std::random_device entropy;
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (int i = 0; i < 4; ++i)
      out << std::setw(8) << static_cast<uint32_t>(entropy());
    return out.str();
  }
  std::string epoch_{freshEpoch()};
  std::optional<Identity> owner_, completed_;
  bool ready_{false};
  bool blocked_{false};
  bool status_eligible_{false};
};
}  // namespace mowgli_behavior
