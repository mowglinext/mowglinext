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
 * @file mqtt_publish_policy.hpp
 * @brief Pure "when to publish" policies for mqtt_bridge_node's outbound topics.
 *
 * Measured on a Home Assistant broker (2026-10-25, robot idle on the dock, 60 s):
 * the bridge forwarded every /diagnostics array (958 msg/min, 1.17 MB/min), every
 * emergency message (237/min, never changing) and every high_level_status message
 * (155/min, identical while idle), and published position/gps/pose once a second
 * even when the robot did not move — so Home Assistant's recorder wrote ~1 row/s
 * for the device_tracker alone. These classes decide what is worth sending:
 *
 *  - ChangeOrHeartbeatGate: state topics — publish when the payload changed,
 *    otherwise only a periodic heartbeat.
 *  - MotionDeadbandGate: position topics — publish when the robot moved or
 *    turned enough, otherwise only a periodic heartbeat.
 *  - DiagnosticsAggregator: merges the /diagnostics arrays of every publisher
 *    into one latest-status-per-name view and reports level changes.
 *
 * Times are plain seconds (any monotonic-enough epoch, the node passes its ROS
 * clock) so gtest drives them without a clock or a node.
 */

#ifndef MOWGLI_MONITORING__MQTT_PUBLISH_POLICY_HPP_
#define MOWGLI_MONITORING__MQTT_PUBLISH_POLICY_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace mowgli_monitoring
{

/// Mean Earth radius used for the local equirectangular GPS distance.
constexpr double kEarthRadiusM = 6371000.0;

/**
 * @brief Distance in metres between two WGS84 fixes, equirectangular approximation.
 *
 * Exact to well under a millimetre over the few metres a deadband compares, and
 * the same approximation the rest of the stack uses for its local map frame.
 */
inline double gps_distance_m(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg)
{
  constexpr double kDegToRad = M_PI / 180.0;
  const double mean_lat = 0.5 * (lat1_deg + lat2_deg) * kDegToRad;
  const double dx = (lon2_deg - lon1_deg) * kDegToRad * std::cos(mean_lat) * kEarthRadiusM;
  const double dy = (lat2_deg - lat1_deg) * kDegToRad * kEarthRadiusM;
  return std::hypot(dx, dy);
}

/// Smallest absolute difference between two angles (radians), in [0, pi].
inline double angle_difference_rad(double a, double b)
{
  return std::abs(std::remainder(a - b, 2.0 * M_PI));
}

/**
 * @brief Publish a state payload when it changed, otherwise every `heartbeat_s`.
 *
 * The caller marks a payload published only when the client accepted it, so a
 * publish that failed (broker down) is retried on the next message instead of
 * being treated as already delivered.
 */
class ChangeOrHeartbeatGate
{
public:
  explicit ChangeOrHeartbeatGate(double heartbeat_s = 10.0) : heartbeat_s_(heartbeat_s)
  {
  }

  bool should_publish(const std::string& payload, double now_s) const
  {
    if (!last_payload_.has_value() || *last_payload_ != payload)
    {
      return true;
    }
    return now_s - last_publish_s_ >= heartbeat_s_;
  }

  void mark_published(const std::string& payload, double now_s)
  {
    last_payload_ = payload;
    last_publish_s_ = now_s;
  }

private:
  double heartbeat_s_;
  std::optional<std::string> last_payload_;
  double last_publish_s_{0.0};
};

/**
 * @brief Publish a position when it moved or turned enough, otherwise every `heartbeat_s`.
 *
 * Coordinates are map/odom metres (Frame::kPlanarMetres) or WGS84 degrees
 * (Frame::kWgs84Degrees: `a` = latitude, `b` = longitude), compared with
 * gps_distance_m(). A sample without a heading (GPS) never triggers on turning.
 */
class MotionDeadbandGate
{
public:
  enum class Frame
  {
    kPlanarMetres,
    kWgs84Degrees
  };

  struct Sample
  {
    double a{0.0};
    double b{0.0};
    std::optional<double> yaw_rad{};
  };

  MotionDeadbandGate(Frame frame,
                     double deadband_m,
                     double heading_deadband_rad,
                     double heartbeat_s)
      : frame_(frame),
        deadband_m_(deadband_m),
        heading_deadband_rad_(heading_deadband_rad),
        heartbeat_s_(heartbeat_s)
  {
  }

  double distance_m(const Sample& from, const Sample& to) const
  {
    if (frame_ == Frame::kWgs84Degrees)
    {
      return gps_distance_m(from.a, from.b, to.a, to.b);
    }
    return std::hypot(to.a - from.a, to.b - from.b);
  }

  bool should_publish(const Sample& sample, double now_s) const
  {
    if (!last_.has_value())
    {
      return true;
    }
    if (now_s - last_publish_s_ >= heartbeat_s_)
    {
      return true;
    }
    if (distance_m(*last_, sample) > deadband_m_)
    {
      return true;
    }
    return sample.yaw_rad.has_value() && last_->yaw_rad.has_value() &&
           angle_difference_rad(*sample.yaw_rad, *last_->yaw_rad) > heading_deadband_rad_;
  }

  void mark_published(const Sample& sample, double now_s)
  {
    last_ = sample;
    last_publish_s_ = now_s;
  }

private:
  Frame frame_;
  double deadband_m_;
  double heading_deadband_rad_;
  double heartbeat_s_;
  std::optional<Sample> last_;
  double last_publish_s_{0.0};
};

/**
 * @brief Latest DiagnosticStatus per `name`, merged across every /diagnostics publisher.
 *
 * /diagnostics has several publishers (diagnostics_node's 1 Hz array plus the
 * arrays of other nodes, ~16 Hz in total); forwarding each array made the MQTT
 * topic flicker between unrelated subsets. The merged view always holds every
 * known status. It is bounded: an entry not refreshed for `stale_after_s` is
 * dropped (its publisher died or renamed it), and past `max_entries` the entry
 * seen longest ago is evicted for a new one.
 */
class DiagnosticsAggregator
{
public:
  DiagnosticsAggregator(std::size_t max_entries, double stale_after_s)
      : max_entries_(max_entries), stale_after_s_(stale_after_s)
  {
  }

  /**
   * @brief Fold one array into the view.
   * @return true when the set of names or any entry's level changed — the event
   *         worth publishing early. Message text alone does not count: several
   *         statuses embed live values (rates, ages) in it.
   */
  bool merge(const diagnostic_msgs::msg::DiagnosticArray& msg, double now_s)
  {
    bool changed = false;
    for (const auto& status : msg.status)
    {
      auto it = entries_.find(status.name);
      if (it == entries_.end())
      {
        evict_oldest_if_full();
        entries_.emplace(status.name, Entry{status.level, status.message, now_s});
        changed = true;
        continue;
      }
      if (it->second.level != status.level)
      {
        changed = true;
      }
      it->second = Entry{status.level, status.message, now_s};
    }
    return changed;
  }

  /// Drop entries not refreshed for stale_after_s. @return true when any was dropped.
  bool drop_stale(double now_s)
  {
    bool dropped = false;
    for (auto it = entries_.begin(); it != entries_.end();)
    {
      if (now_s - it->second.last_seen_s > stale_after_s_)
      {
        it = entries_.erase(it);
        dropped = true;
      }
      else
      {
        ++it;
      }
    }
    return dropped;
  }

  /// Merged view as one array, sorted by name so the payload is stable.
  diagnostic_msgs::msg::DiagnosticArray snapshot() const
  {
    diagnostic_msgs::msg::DiagnosticArray out;
    out.status.reserve(entries_.size());
    for (const auto& [name, entry] : entries_)
    {
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = name;
      status.level = entry.level;
      status.message = entry.message;
      out.status.push_back(std::move(status));
    }
    return out;
  }

  std::size_t size() const
  {
    return entries_.size();
  }

  bool empty() const
  {
    return entries_.empty();
  }

private:
  struct Entry
  {
    uint8_t level{0};
    std::string message{};
    double last_seen_s{0.0};
  };

  void evict_oldest_if_full()
  {
    if (entries_.size() < max_entries_ || entries_.empty())
    {
      return;
    }
    auto oldest = entries_.begin();
    for (auto it = entries_.begin(); it != entries_.end(); ++it)
    {
      if (it->second.last_seen_s < oldest->second.last_seen_s)
      {
        oldest = it;
      }
    }
    entries_.erase(oldest);
  }

  std::size_t max_entries_;
  double stale_after_s_;
  std::map<std::string, Entry> entries_;
};

}  // namespace mowgli_monitoring

#endif  // MOWGLI_MONITORING__MQTT_PUBLISH_POLICY_HPP_
