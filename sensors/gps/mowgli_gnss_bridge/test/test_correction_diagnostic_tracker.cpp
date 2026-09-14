// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// Unit tests for the correction-diagnostics projection. These pin the three
// facts the projection deliberately keeps apart: transport (the caster answered),
// flow (integrity-valid RTCM frames are arriving) and semantic health (a valid
// base plus usable dynamic MSM corrections for THIS station).

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <initializer_list>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "gtest/gtest.h"
#include "mowgli_gnss_bridge/correction_diagnostic_tracker.hpp"

using mowgli_gnss_bridge::CorrectionDiagnosticTracker;
using PublicGnssStatus = mowgli_interfaces::msg::GnssStatus;

namespace
{

using Array = diagnostic_msgs::msg::DiagnosticArray;
using Status = diagnostic_msgs::msg::DiagnosticStatus;
using Values = std::initializer_list<std::pair<const char *, const char *>>;

Status makeStatus(
  const char * name, const char * source, const char * message = "", Values values = {})
{
  Status status;
  status.name = name;
  status.hardware_id = source;
  status.message = message;
  for (const auto & pair : values) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = pair.first;
    item.value = pair.second;
    status.values.push_back(item);
  }
  return status;
}

Array healthyNtrip(const char * source = "caster:2101/MOUNT", std::int32_t stamp = 10)
{
  Array diagnostics;
  diagnostics.header.stamp.sec = stamp;
  diagnostics.status = {
    makeStatus(
      "universal_gnss_ntrip/summary", source, "healthy",
      {{"correction_available", "true"},
        {"transport_healthy", "true"},
        {"parser_healthy", "true"},
        {"stale_data", "false"}}),
    makeStatus("universal_gnss_ntrip/ntrip_streaming", source),
    makeStatus("universal_gnss_ntrip/correction_flowing", source),
    makeStatus("universal_gnss_ntrip/rtcm_forwarding", source, "RTCM forwarding active"),
    makeStatus(
      "universal_gnss_ntrip/rtcm_semantic/base_station_arp", source, "valid",
      {{"seen", "true"},
        {"decoded", "true"},
        {"valid", "true"},
        {"message_type", "1006"},
        {"station_id", "42"},
        {"age_s", "20.0"}}),
    makeStatus(
      "universal_gnss_ntrip/rtcm_semantic/msm_summary", source, "valid",
      {{"seen", "true"},
        {"decoded", "true"},
        {"valid", "true"},
        {"message_type", "1077"},
        {"station_id", "42"},
        {"constellations_seen", "GPS"},
        {"satellite_count", "12"},
        {"signal_count", "18"},
        {"cell_count", "24"},
        {"malformed_count", "0"},
        {"age_s", "0.2"}}),
  };
  return diagnostics;
}

Array receiverSnapshot(const char * message, std::int32_t stamp = 20)
{
  constexpr char kSource[] = "serial:/dev/ttyACM0";
  Array diagnostics;
  diagnostics.header.stamp.sec = stamp;
  diagnostics.status = {
    makeStatus("universal_gnss/rtcm_forwarding", kSource, message),
    makeStatus(
      "universal_gnss/rtcm_semantic/msm_summary", kSource, "valid",
      {{"seen", "true"},
        {"decoded", "true"},
        {"valid", "true"},
        {"message_type", "1087"},
        {"station_id", "77"},
        {"cell_count", "9"},
        {"age_s", "0.1"}}),
  };
  return diagnostics;
}

PublicGnssStatus project(
  const CorrectionDiagnosticTracker & tracker, CorrectionDiagnosticTracker::TimePoint now)
{
  PublicGnssStatus status;
  tracker.apply(status, now);
  return status;
}

}  // namespace

TEST(CorrectionDiagnosticTrackerTest, HealthyDynamicFlowDoesNotRequireGlonass1230)
{
  CorrectionDiagnosticTracker tracker(std::chrono::seconds(2));
  const auto now = CorrectionDiagnosticTracker::TimePoint{} + std::chrono::seconds(100);
  tracker.update(healthyNtrip(), now);

  const auto status = project(tracker, now);
  EXPECT_EQ(
    status.correction_transport_status, PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_STREAMING);
  EXPECT_TRUE(status.correction_response_accepted);
  EXPECT_EQ(status.correction_flow_status, PublicGnssStatus::CORRECTION_FLOW_STATUS_ACTIVE);
  EXPECT_EQ(status.correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_HEALTHY);
  EXPECT_EQ(status.correction_stream_status, PublicGnssStatus::CORRECTION_STREAM_STATUS_ACTIVE);
  EXPECT_TRUE(status.msm_summary_valid);
  EXPECT_EQ(status.msm_summary_cell_count, 24U);
  EXPECT_EQ(status.correction_source, "caster:2101/MOUNT");
}

TEST(CorrectionDiagnosticTrackerTest, ConnectionResponseAndForwardingDoNotImplySemanticHealth)
{
  const auto now = CorrectionDiagnosticTracker::TimePoint{} + std::chrono::seconds(100);

  CorrectionDiagnosticTracker connected(std::chrono::seconds(2));
  Array connected_array;
  connected_array.header.stamp.sec = 1;
  connected_array.status = {
    makeStatus("universal_gnss_ntrip/ntrip_connected", "caster:2101/MOUNT"),
  };
  connected.update(connected_array, now);
  auto status = project(connected, now);
  EXPECT_EQ(
    status.correction_transport_status, PublicGnssStatus::CORRECTION_TRANSPORT_STATUS_CONNECTED);
  EXPECT_FALSE(status.correction_response_accepted);
  EXPECT_NE(status.correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_HEALTHY);

  CorrectionDiagnosticTracker accepted(std::chrono::seconds(2));
  Array accepted_array;
  accepted_array.header.stamp.sec = 1;
  accepted_array.status = {
    makeStatus("universal_gnss_ntrip/ntrip_streaming", "caster:2101/MOUNT"),
    makeStatus(
      "universal_gnss_ntrip/rtcm_forwarding", "caster:2101/MOUNT", "RTCM forwarding active"),
  };
  accepted.update(accepted_array, now);
  status = project(accepted, now);
  EXPECT_TRUE(status.correction_response_accepted);
  EXPECT_EQ(status.correction_stream_status, PublicGnssStatus::CORRECTION_STREAM_STATUS_ACTIVE);
  EXPECT_EQ(status.correction_flow_status, PublicGnssStatus::CORRECTION_FLOW_STATUS_WAITING);
  EXPECT_EQ(status.correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_WAITING);
}

TEST(CorrectionDiagnosticTrackerTest, MalformedZeroCellAndStaticOnlyCorrectionsFailClosed)
{
  const auto now = CorrectionDiagnosticTracker::TimePoint{} + std::chrono::seconds(100);

  auto malformed_array = healthyNtrip();
  for (auto & entry : malformed_array.status) {
    if (entry.name == "universal_gnss_ntrip/summary") {
      for (auto & value : entry.values) {
        if (value.key == "parser_healthy") {
          value.value = "false";
        }
      }
    }
  }
  CorrectionDiagnosticTracker malformed(std::chrono::seconds(2));
  malformed.update(malformed_array, now);
  EXPECT_EQ(
    project(malformed, now).correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_INVALID);

  auto zero_cell_array = healthyNtrip();
  for (auto & entry : zero_cell_array.status) {
    if (entry.name == "universal_gnss_ntrip/rtcm_semantic/msm_summary") {
      for (auto & value : entry.values) {
        if (value.key == "cell_count") {
          value.value = "0";
        }
      }
    }
  }
  CorrectionDiagnosticTracker zero_cell(std::chrono::seconds(2));
  zero_cell.update(zero_cell_array, now);
  EXPECT_EQ(
    project(zero_cell, now).correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_INVALID);

  auto static_only_array = healthyNtrip();
  static_only_array.status.erase(
    std::remove_if(
      static_only_array.status.begin(), static_only_array.status.end(),
      [](const Status & entry) {
        return entry.name == "universal_gnss_ntrip/rtcm_semantic/msm_summary";
      }),
    static_only_array.status.end());
  for (auto & entry : static_only_array.status) {
    if (entry.name == "universal_gnss_ntrip/summary") {
      for (auto & value : entry.values) {
        if (value.key == "correction_available") {
          value.value = "false";
        }
      }
    }
  }
  CorrectionDiagnosticTracker static_only(std::chrono::seconds(2));
  static_only.update(static_only_array, now);
  EXPECT_EQ(
    project(static_only, now).correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_WAITING);
}

TEST(CorrectionDiagnosticTrackerTest, MonotonicExpiryRemovesAuthorityAndReceiverNoLongerShadowed)
{
  CorrectionDiagnosticTracker tracker(std::chrono::seconds(1));
  const auto start = CorrectionDiagnosticTracker::TimePoint{} + std::chrono::seconds(100);
  tracker.update(healthyNtrip("caster:2101/OLD", 100), start);

  // A paused ROS stamp cannot extend the steady-clock lifetime.
  auto expired = project(tracker, start + std::chrono::seconds(1));
  EXPECT_EQ(expired.value_flags & PublicGnssStatus::CAP_CORRECTION_STREAM, 0U);
  EXPECT_EQ(expired.value_flags & PublicGnssStatus::CAP_CORRECTION_SEMANTIC, 0U);

  // A live receiver snapshot becomes authoritative for receiver forwarding and
  // MSM display; expired NTRIP does not continue to shadow it.
  tracker.update(receiverSnapshot("RTCM forwarding active", 101), start + std::chrono::seconds(1));
  const auto fallback = project(tracker, start + std::chrono::milliseconds(1100));
  EXPECT_EQ(fallback.correction_stream_status, PublicGnssStatus::CORRECTION_STREAM_STATUS_ACTIVE);
  EXPECT_EQ(fallback.correction_forwarding_source, "serial:/dev/ttyACM0");
  EXPECT_EQ(fallback.msm_summary_station_id, 77U);
  EXPECT_EQ(fallback.msm_summary_source, "serial:/dev/ttyACM0");
  EXPECT_EQ(fallback.value_flags & PublicGnssStatus::CAP_CORRECTION_SEMANTIC, 0U);
}

TEST(CorrectionDiagnosticTrackerTest, DisconnectReconnectSourceAndStationChangesDoNotInherit)
{
  CorrectionDiagnosticTracker tracker(std::chrono::seconds(2));
  const auto now = CorrectionDiagnosticTracker::TimePoint{} + std::chrono::seconds(100);
  tracker.update(healthyNtrip("caster:2101/A", 10), now);

  Array disconnected;
  disconnected.header.stamp.sec = 11;
  disconnected.status = {
    makeStatus("universal_gnss_ntrip/ntrip_disconnected", "caster:2101/A"),
  };
  tracker.update(disconnected, now + std::chrono::milliseconds(100));
  auto status = project(tracker, now + std::chrono::milliseconds(100));
  EXPECT_EQ(
    status.correction_semantic_status, PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_UNAVAILABLE);
  EXPECT_EQ(status.value_flags & PublicGnssStatus::CAP_MSM_SUMMARY, 0U);

  Array reconnected;
  reconnected.header.stamp.sec = 12;
  reconnected.status = {
    makeStatus("universal_gnss_ntrip/ntrip_streaming", "caster:2101/A"),
  };
  tracker.update(reconnected, now + std::chrono::milliseconds(200));
  status = project(tracker, now + std::chrono::milliseconds(200));
  EXPECT_EQ(status.correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_WAITING);
  EXPECT_EQ(status.value_flags & PublicGnssStatus::CAP_MSM_SUMMARY, 0U);

  tracker.update(healthyNtrip("caster:2101/B", 13), now + std::chrono::milliseconds(300));
  status = project(tracker, now + std::chrono::milliseconds(300));
  EXPECT_EQ(status.correction_source, "caster:2101/B");

  auto station_change = healthyNtrip("caster:2101/B", 14);
  for (auto & entry : station_change.status) {
    if (entry.name == "universal_gnss_ntrip/rtcm_semantic/msm_summary") {
      for (auto & value : entry.values) {
        if (value.key == "station_id") {
          value.value = "43";
        }
      }
    }
  }
  tracker.update(station_change, now + std::chrono::milliseconds(400));
  EXPECT_EQ(
    project(tracker, now + std::chrono::milliseconds(400)).correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_INVALID);
}

TEST(CorrectionDiagnosticTrackerTest, OmitClearOlderSetAndLaterCurrentSetRemainDistinct)
{
  CorrectionDiagnosticTracker tracker(std::chrono::seconds(2));
  const auto now = CorrectionDiagnosticTracker::TimePoint{} + std::chrono::seconds(100);
  tracker.update(healthyNtrip("caster:2101/A", 20), now);

  Array receiver_clear;
  receiver_clear.header.stamp.sec = 20;
  receiver_clear.status = {
    makeStatus("universal_gnss/summary", "serial:/dev/ttyACM0"),
  };
  tracker.update(receiver_clear, now + std::chrono::milliseconds(50));
  EXPECT_EQ(
    project(tracker, now + std::chrono::milliseconds(50)).value_flags &
    PublicGnssStatus::CAP_CORRECTION_STREAM,
    0U);

  Array unrelated;
  unrelated.header.stamp.sec = 21;
  unrelated.status = {makeStatus("other/component", "other")};
  tracker.update(unrelated, now + std::chrono::milliseconds(100));
  EXPECT_EQ(
    project(tracker, now + std::chrono::milliseconds(100)).correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_HEALTHY);

  Array clear;
  clear.header.stamp.sec = 22;
  clear.status = {makeStatus("universal_gnss_ntrip/ntrip_reconnecting", "caster:2101/B")};
  tracker.update(clear, now + std::chrono::milliseconds(200));
  EXPECT_EQ(
    project(tracker, now + std::chrono::milliseconds(200)).correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_UNAVAILABLE);

  // Older source SET after CLEAR is rejected even though received later.
  tracker.update(healthyNtrip("caster:2101/A", 20), now + std::chrono::milliseconds(300));
  auto status = project(tracker, now + std::chrono::milliseconds(300));
  EXPECT_EQ(status.correction_source, "caster:2101/B");
  EXPECT_NE(status.correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_HEALTHY);

  // A later ordered SET restores availability normally.
  tracker.update(healthyNtrip("caster:2101/B", 23), now + std::chrono::milliseconds(400));
  status = project(tracker, now + std::chrono::milliseconds(400));
  EXPECT_EQ(status.correction_semantic_status,
    PublicGnssStatus::CORRECTION_SEMANTIC_STATUS_HEALTHY);
}
