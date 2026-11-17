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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file mqtt_connection.hpp
 * @brief Broker-facing identity and reconnect pacing of mqtt_bridge_node.
 *
 *  - mqtt_client_id(): a stable client ID per robot and per client role. A broker
 *    holds one session per client ID and drops the older one when a second
 *    client connects with the same ID, so two robots sharing an ID evict each
 *    other on every reconnect. The GUI schedule client derives its
 *    ID with the same rule (gui/pkg/api/schedule_mqtt.go mqttClientID).
 *  - ReconnectBackoff: when the bridge may try to connect again.
 *
 * Times are plain seconds on a monotonic clock so gtest drives them without a node.
 */

#ifndef MOWGLI_MONITORING__MQTT_CONNECTION_HPP_
#define MOWGLI_MONITORING__MQTT_CONNECTION_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

namespace mowgli_monitoring
{

/// FNV-1a, 32 bits, over the bytes of `text`.
inline std::uint32_t fnv1a32(const std::string& text)
{
  std::uint32_t hash = 2166136261u;
  for (const unsigned char c : text)
  {
    hash ^= c;
    hash *= 16777619u;
  }
  return hash;
}

/// "mowglinext_<prefix>_<hash>_<role>": the topic prefix with every byte other
/// than [A-Za-z0-9_-] replaced by '_' and cut to 32 bytes, then the FNV-1a hash
/// of the unmodified prefix as 8 lowercase hex digits, so prefixes that differ
/// only in replaced or cut bytes ("a/b" and "a_b") still get different IDs.
/// Robots on one broker already need distinct topic prefixes; the role keeps the
/// bridge and the GUI schedule client of one robot apart.
inline std::string mqtt_client_id(const std::string& topic_prefix, const std::string& role)
{
  constexpr std::size_t kMaxPrefixBytes = 32;
  std::string id{"mowglinext_"};
  for (std::size_t i = 0; i < topic_prefix.size() && i < kMaxPrefixBytes; ++i)
  {
    // ASCII only, independent of the locale, as in the GUI's mqttClientID.
    const char c = topic_prefix[i];
    const bool kept = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                      c == '_' || c == '-';
    id += kept ? c : '_';
  }
  char hex[9];
  std::snprintf(hex, sizeof(hex), "%08x", static_cast<unsigned>(fnv1a32(topic_prefix)));
  return id + "_" + hex + "_" + role;
}

/// Paces connection attempts while the bridge is disconnected. The first attempt
/// is immediate; each further one waits twice as long as the previous wait, from
/// kInitialDelayS up to kMaxDelayS. The wait drops back to kInitialDelayS only
/// once a connection has held for kStableAfterS, so a session the broker keeps
/// closing (a client ID still in use elsewhere, a refused login) is retried at
/// most every kMaxDelayS instead of once per timer tick.
class ReconnectBackoff
{
public:
  static constexpr double kInitialDelayS = 1.0;
  static constexpr double kMaxDelayS = 30.0;
  static constexpr double kStableAfterS = 30.0;

  /// While disconnected: true when an attempt is due now, which is then counted.
  bool attempt_due(double now_s)
  {
    connected_since_s_.reset();
    if (next_attempt_s_ && now_s < *next_attempt_s_)
    {
      return false;
    }
    next_attempt_s_ = now_s + delay_s_;
    delay_s_ = std::min(delay_s_ * 2.0, kMaxDelayS);
    return true;
  }

  /// While connected.
  void connected(double now_s)
  {
    if (!connected_since_s_)
    {
      connected_since_s_ = now_s;
    }
    else if (now_s - *connected_since_s_ >= kStableAfterS)
    {
      delay_s_ = kInitialDelayS;
    }
  }

  /// Seconds until the next attempt is due (0 when one is due now).
  double next_attempt_in_s(double now_s) const
  {
    return next_attempt_s_ ? std::max(0.0, *next_attempt_s_ - now_s) : 0.0;
  }

  /// The wait that will follow the next attempt.
  double delay_s() const
  {
    return delay_s_;
  }

private:
  double delay_s_{kInitialDelayS};
  std::optional<double> next_attempt_s_;
  std::optional<double> connected_since_s_;
};

}  // namespace mowgli_monitoring

#endif  // MOWGLI_MONITORING__MQTT_CONNECTION_HPP_
