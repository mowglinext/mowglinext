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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <utility>

#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/gnss_fusion_update.hpp"
#include "mowgli_localization/navsat_status_association.hpp"

namespace mowgli_behavior
{
// Keeps the original raw-topic rendezvous separate from the later graph report.
// All clocks are supplied by the caller: ROS for acquisition age, steady for
// delivery pairing. Neither a report nor a repeated status refreshes a fix.
class PrecisionCoverageSupport
{
public:
  using Fix = sensor_msgs::msg::NavSatFix;
  using Status = mowgli_interfaces::msg::GnssStatus;
  using Report = mowgli_interfaces::msg::GnssFusionUpdate;
  /// Newest map->base_footprint transform may be at most this old for the pose
  /// support to count as current.
  static constexpr int64_t kMaxTfLagNs = 500000000LL;
  /// Whether the localizer's TF is current at @p now_ns, given the stamp of the
  /// NEWEST map->base_footprint transform (future-dated stamps are current).
  /// Never ask for the transform AT now: the newest broadcast is always a few
  /// ms to ~0.1 s in the past, so that lookup failed on most BT ticks; each
  /// failure moved loss_stamp_ past every collected fix, and the gate flapped
  /// about once a second with RTK-Fixed and the robot still (field
  /// 2026-11-09: blade ON/OFF every ~0.5 s for 35 s, coverage never sent).
  static bool TfCurrent(int64_t latest_tf_ns, int64_t now_ns)
  {
    return latest_tf_ns > 0 && now_ns - latest_tf_ns <= kMaxTfLagNs;
  }
  static int64_t Stamp(const builtin_interfaces::msg::Time& stamp)
  {
    return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
  }
  static bool SameFix(const Fix& a, const Fix& b)
  {
    const auto eq = [](double x, double y)
    {
      return x == y || (std::isnan(x) && std::isnan(y));
    };
    if (a.header != b.header || a.status != b.status || !eq(a.latitude, b.latitude) ||
        !eq(a.longitude, b.longitude) || !eq(a.altitude, b.altitude) ||
        a.position_covariance_type != b.position_covariance_type)
      return false;
    for (size_t i = 0; i < a.position_covariance.size(); ++i)
      if (!eq(a.position_covariance[i], b.position_covariance[i]))
        return false;
    return true;
  }
  void AddFix(const Fix& fix, int64_t now, int64_t steady)
  {
    std::lock_guard<std::mutex> guard(mu_);
    if (!Clock(now, steady))
      return;
    // Retain first raw delivery, never a report/duplicate receipt. The exact
    // raw pair still has to pass the existing association before use below.
    first_fix_delivery_.try_emplace(Stamp(fix.header.stamp), steady);
    while (first_fix_delivery_.size() > 64)
      first_fix_delivery_.erase(first_fix_delivery_.begin());
    Collect(association_.AddFix(fix, Stamp(fix.header.stamp), now, steady));
  }
  void AddStatus(const Status& status, int64_t now, int64_t steady)
  {
    std::lock_guard<std::mutex> guard(mu_);
    if (!Clock(now, steady))
      return;
    const auto stamp = Stamp(status.header.stamp);
    if (stamp <= 0 || stamp > now || now - stamp > kMaxAge)
    {
      loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
      raw_fixed_ = false;
      fixed_since_ = 0;
      was_ready_ = false;
      return;
    }
    if (stamp >= last_status_stamp_)
    {
      const auto source = std::make_pair(status.source_id, status.source_incarnation);
      if (source_seen_ &&
          (status.source_id != source_id_ || status.source_incarnation != source_incarnation_))
      {
        if (seen_sources_.count(source) || seen_sources_.size() >= 32)
          return;
        loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
        association_.Reset();
        first_fix_delivery_.clear();
        completed_.clear();
        reports_.clear();
        matched_.clear();
        raw_fixed_ = false;
        was_ready_ = false;
        fixed_since_ = 0;
      }
      source_seen_ = true;
      seen_sources_.insert(source);
      source_id_ = status.source_id;
      source_incarnation_ = status.source_incarnation;
      if (stamp > last_status_stamp_)
        last_status_steady_ = steady;
      const bool fixed = mowgli_interfaces::gnss_status_utils::BehaviorTreeRtkFixed(status);
      if (!fixed)
      {
        raw_fixed_ = false;
        fixed_since_ = 0;
        was_ready_ = false;
        loss_stamp_ = std::max(loss_stamp_, stamp);
      }
      else if (!raw_fixed_)
      {
        raw_fixed_ = true;
        fixed_since_ = now;
      }
      last_status_stamp_ = stamp;
    }
    Collect(association_.AddStatus(status, stamp, now, steady));
  }
  void AddReport(const Report& report, int64_t now, int64_t steady)
  {
    std::lock_guard<std::mutex> guard(mu_);
    if (!Clock(now, steady) || report.producer_incarnation.empty())
      return;
    if (producer_ != report.producer_incarnation)
    {
      if (seen_producers_.count(report.producer_incarnation) || seen_producers_.size() >= 32)
        return;
      if (!producer_.empty())
        loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
      seen_producers_.insert(report.producer_incarnation);
      producer_ = report.producer_incarnation;
      was_ready_ = false;
      epoch_ = report.graph_epoch;
      reports_.clear();
      matched_.clear();
    }
    else if (report.graph_epoch < epoch_)
      return;
    else if (report.graph_epoch > epoch_)
    {
      epoch_ = report.graph_epoch;
      if (was_ready_)
        loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
      was_ready_ = false;
      reports_.clear();
      matched_.clear();
    }
    if (!report.pose_published || !report.has_applied_fix)
    {
      if (was_ready_)
        loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
      was_ready_ = false;
      reports_.clear();
      matched_.clear();
      return;
    }
    if (!report.applied_revision || report.applied_revision > report.estimate_revision)
      return;
    if (report.applied_fix.status.status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX)
    {
      loss_stamp_ = std::max(loss_stamp_, Stamp(report.applied_fix.header.stamp));
      raw_fixed_ = false;
      fixed_since_ = 0;
      was_ready_ = false;
      reports_.clear();
      matched_.clear();
      return;
    }
    reports_.push_back(report);
    while (reports_.size() > 32)
      reports_.pop_front();
  }
  bool Ready(int64_t now, int64_t steady, const std::function<bool(int64_t)>& tf_supported)
  {
    std::lock_guard<std::mutex> guard(mu_);
    if (!Clock(now, steady))
      return false;
    Collect(association_.Poll(now, steady));
    if (!raw_fixed_ || !fixed_since_ || now - fixed_since_ < 2000000000LL ||
        last_status_stamp_ <= 0 || now < last_status_stamp_ || now - last_status_stamp_ > kMaxAge ||
        !last_status_steady_ || steady < last_status_steady_ ||
        steady - last_status_steady_ > kMaxAge)
    {
      if (was_ready_)
        loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
      was_ready_ = false;
      return false;
    }
    for (auto it = reports_.begin(); it != reports_.end();)
    {
      bool matched = false;
      for (const auto& pair : completed_)
        if (SameFix(it->applied_fix, pair.fix) &&
            pair.fix.status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX &&
            mowgli_interfaces::gnss_status_utils::BehaviorTreeRtkFixed(pair.status))
        {
          const auto delivery = first_fix_delivery_.find(Stamp(pair.fix.header.stamp));
          if (delivery != first_fix_delivery_.end())
          {
            matched_.push_back({*it, delivery->second});
            matched = true;
            break;
          }
        }
      if (matched)
        it = reports_.erase(it);
      else
        ++it;
    }
    while (matched_.size() > 32)
      matched_.pop_front();
    for (const auto& evidence : matched_)
    {
      const auto& report = evidence.report;
      const int64_t fix_time = Stamp(report.applied_fix.header.stamp);
      const int64_t support = Stamp(report.tf_support_stamp);
      if (fix_time > loss_stamp_ && fix_time > 0 && now >= fix_time && now - fix_time <= kMaxAge &&
          evidence.first_delivery > 0 && steady >= evidence.first_delivery &&
          steady - evidence.first_delivery <= kMaxAge && support > 0 && now >= support &&
          tf_supported(support))
      {
        was_ready_ = true;
        return true;
      }
    }
    if (was_ready_)
      loss_stamp_ = std::max(loss_stamp_, last_status_stamp_);
    was_ready_ = false;
    return false;
  }

private:
  bool Clock(int64_t now, int64_t steady)
  {
    if ((last_ros_ && now < last_ros_) || (last_steady_ && steady < last_steady_))
    {
      association_.Reset();
      first_fix_delivery_.clear();
      completed_.clear();
      reports_.clear();
      matched_.clear();
      raw_fixed_ = false;
      was_ready_ = false;
      fixed_since_ = 0;
      last_status_stamp_ = 0;
      last_status_steady_ = 0;
      source_seen_ = false;
      source_id_.clear();
      source_incarnation_.clear();
      seen_sources_.clear();
      loss_stamp_ = now;
      last_ros_ = now;
      last_steady_ = steady;
      return false;
    }
    last_ros_ = now;
    last_steady_ = steady;
    return true;
  }
  void Collect(const mowgli_localization::AssociationBatch& batch)
  {
    using Event = mowgli_localization::AssociationEvent;
    if (batch.event == Event::kSourceRestart || batch.event == Event::kReceiptRewind ||
        batch.event == Event::kClockDiscontinuity)
    {
      loss_stamp_ = std::max(loss_stamp_, batch.event_receipt_time_ns);
      first_fix_delivery_.clear();
      completed_.clear();
      reports_.clear();
      matched_.clear();
      was_ready_ = false;
      raw_fixed_ = false;
      fixed_since_ = 0;
    }
    for (const auto& pair : batch.ready)
      completed_.push_back(pair);
    while (completed_.size() > 32)
      completed_.pop_front();
  }
  static constexpr int64_t kMaxAge = 5000000000LL;
  std::mutex mu_;
  mowgli_localization::NavSatStatusAssociation association_{32, 50000000LL, 2000000000LL};
  std::deque<mowgli_localization::AssociatedNavSatObservation> completed_;
  std::deque<Report> reports_;
  struct MatchedSupport
  {
    Report report;
    int64_t first_delivery;
  };
  std::deque<MatchedSupport> matched_;
  std::map<int64_t, int64_t> first_fix_delivery_;
  std::set<std::string> seen_producers_;
  std::string producer_;
  std::string source_id_, source_incarnation_;
  std::set<std::pair<std::string, std::string>> seen_sources_;
  bool source_seen_ = false;
  uint64_t epoch_ = 0;
  int64_t last_ros_ = 0, last_steady_ = 0, last_status_stamp_ = 0, last_status_steady_ = 0,
          fixed_since_ = 0, loss_stamp_ = 0;
  bool raw_fixed_ = false;
  bool was_ready_ = false;
};
}  // namespace mowgli_behavior
