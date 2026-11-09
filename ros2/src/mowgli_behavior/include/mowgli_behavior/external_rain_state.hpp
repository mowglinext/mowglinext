// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

/// What the mower currently makes of the external rain verdict on /external_rain, for
/// feedback to an operator (GUI, MQTT `<prefix>/external_rain_state`, Home Assistant).
/// Read-only: it never changes what the rain guard does.
struct ExternalRainStatus
{
  /// "disabled" (use_external_rain off), "no_data" (nothing received yet), "dry", "wet"
  /// (a fresh wet verdict, counted) or "expired" (a wet verdict older than the max age,
  /// no longer counted).
  std::string state;
  bool enabled{true};
  bool received{false};
  /// The last verdict received, whatever its age.
  bool wet{false};
  /// A fresh wet verdict that the rain guard counts right now.
  bool active{false};
  /// The mower's own rain sensor, shown beside it so the two sources can be told apart.
  bool onboard_wet{false};
  /// Seconds since the last verdict; < 0 when none was received.
  std::int64_t age_s{-1};
  /// Seconds until a wet verdict expires; < 0 when not applicable (dry, no expiry).
  std::int64_t expires_in_s{-1};
  /// Configured expiry in seconds; 0 = never expires.
  std::int64_t max_age_s{0};
};

/// Caller holds ctx.context_mutex.
inline ExternalRainStatus ComputeExternalRainStatus(const BTContext& ctx,
                                                    std::chrono::steady_clock::time_point now)
{
  ExternalRainStatus s;
  s.enabled = ctx.use_external_rain;
  s.received = ctx.external_rain_stamp.time_since_epoch().count() != 0;
  s.wet = s.received && ctx.external_rain;
  s.onboard_wet = ctx.latest_status.rain_detected;
  s.max_age_s = static_cast<std::int64_t>(ctx.external_rain_max_age_sec);
  if (s.received)
  {
    s.age_s = static_cast<std::int64_t>(
        std::chrono::duration<double>(now - ctx.external_rain_stamp).count());
  }
  s.active = ctx.ExternalRainActive(now);
  if (s.active && ctx.external_rain_max_age_sec > 0.0)
  {
    const double left = ctx.external_rain_max_age_sec -
                        std::chrono::duration<double>(now - ctx.external_rain_stamp).count();
    s.expires_in_s = left > 0.0 ? static_cast<std::int64_t>(left) : 0;
  }

  if (!s.enabled)
  {
    s.state = "disabled";
  }
  else if (!s.received)
  {
    s.state = "no_data";
  }
  else if (!s.wet)
  {
    s.state = "dry";
  }
  else
  {
    s.state = s.active ? "wet" : "expired";
  }
  return s;
}

/// The JSON that goes out on /behavior_tree_node/external_rain_state and, retained, on
/// MQTT `<prefix>/external_rain_state`. `age_s`, `expires_in_s` are null when not applicable.
inline std::string ExternalRainStatusJson(const ExternalRainStatus& s)
{
  const auto num = [](std::int64_t v) -> std::string
  {
    return v < 0 ? "null" : std::to_string(v);
  };
  const auto flag = [](bool v) -> const char*
  {
    return v ? "true" : "false";
  };
  return std::string("{\"state\":\"") + s.state + "\",\"enabled\":" + flag(s.enabled) +
         ",\"received\":" + flag(s.received) + ",\"wet\":" + flag(s.wet) +
         ",\"active\":" + flag(s.active) + ",\"onboard_wet\":" + flag(s.onboard_wet) +
         ",\"age_s\":" + num(s.age_s) + ",\"expires_in_s\":" + num(s.expires_in_s) +
         ",\"max_age_s\":" + std::to_string(s.max_age_s) + "}";
}

/// Only these change the operator-visible picture; the ages are refreshed on a slow heartbeat
/// instead of on every tick.
inline bool SameExternalRainPicture(const ExternalRainStatus& a, const ExternalRainStatus& b)
{
  return a.state == b.state && a.onboard_wet == b.onboard_wet && a.enabled == b.enabled;
}

}  // namespace mowgli_behavior
