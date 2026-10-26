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
 * @file test_mqtt_publish_policy.cpp
 * @brief ROS-free tests of mqtt_publish_policy.hpp: send-on-change + heartbeat,
 *        position deadband + heartbeat, and the merged diagnostics view.
 */

#include <cmath>
#include <initializer_list>
#include <string>
#include <tuple>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "mowgli_monitoring/mqtt_publish_policy.hpp"
#include <gtest/gtest.h>

using mowgli_monitoring::ChangeOrHeartbeatGate;
using mowgli_monitoring::DiagnosticsAggregator;
using mowgli_monitoring::MotionDeadbandGate;

namespace
{

constexpr double kFiveDegrees = 5.0 * M_PI / 180.0;

diagnostic_msgs::msg::DiagnosticArray make_array(
    std::initializer_list<std::tuple<std::string, uint8_t, std::string>> entries)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  for (const auto& [name, level, message] : entries)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = name;
    status.level = level;
    status.message = message;
    array.status.push_back(status);
  }
  return array;
}

}  // namespace

// ===========================================================================
// ChangeOrHeartbeatGate
// ===========================================================================

TEST(ChangeOrHeartbeatGate, FirstPayloadIsAlwaysPublished)
{
  const ChangeOrHeartbeatGate gate(10.0);
  EXPECT_TRUE(gate.should_publish("{}", 0.0));
}

TEST(ChangeOrHeartbeatGate, UnchangedPayloadWaitsForTheHeartbeat)
{
  ChangeOrHeartbeatGate gate(10.0);
  gate.mark_published("same", 100.0);

  EXPECT_FALSE(gate.should_publish("same", 100.25));
  EXPECT_FALSE(gate.should_publish("same", 109.9));
  EXPECT_TRUE(gate.should_publish("same", 110.0));
}

TEST(ChangeOrHeartbeatGate, ChangedPayloadIsPublishedImmediately)
{
  ChangeOrHeartbeatGate gate(10.0);
  gate.mark_published(R"({"active_emergency":false})", 100.0);

  // 1 ms later: a change is never held back by the heartbeat period.
  EXPECT_TRUE(gate.should_publish(R"({"active_emergency":true})", 100.001));
}

TEST(ChangeOrHeartbeatGate, UnacceptedPublishIsRetriedByTheNextMessage)
{
  ChangeOrHeartbeatGate gate(10.0);
  gate.mark_published("old", 100.0);
  // "new" was offered but the client refused it (broker down): not marked.
  ASSERT_TRUE(gate.should_publish("new", 100.5));
  EXPECT_TRUE(gate.should_publish("new", 100.75));
}

// ===========================================================================
// MotionDeadbandGate
// ===========================================================================

TEST(MotionDeadbandGate, PlanarMovementBeyondTheDeadbandPublishes)
{
  MotionDeadbandGate gate(MotionDeadbandGate::Frame::kPlanarMetres, 0.2, kFiveDegrees, 30.0);
  ASSERT_TRUE(gate.should_publish({1.0, 2.0, 0.0}, 0.0));
  gate.mark_published({1.0, 2.0, 0.0}, 0.0);

  EXPECT_FALSE(gate.should_publish({1.1, 2.1, 0.0}, 1.0));  // 0.14 m
  EXPECT_TRUE(gate.should_publish({1.0, 2.25, 0.0}, 1.0));  // 0.25 m
}

TEST(MotionDeadbandGate, HeadingChangeBeyondFiveDegreesPublishesInPlace)
{
  MotionDeadbandGate gate(MotionDeadbandGate::Frame::kPlanarMetres, 0.2, kFiveDegrees, 30.0);
  gate.mark_published({0.0, 0.0, 0.0}, 0.0);

  EXPECT_FALSE(gate.should_publish({0.0, 0.0, 4.0 * M_PI / 180.0}, 1.0));
  EXPECT_TRUE(gate.should_publish({0.0, 0.0, 6.0 * M_PI / 180.0}, 1.0));
}

TEST(MotionDeadbandGate, HeadingWrapsAroundPi)
{
  MotionDeadbandGate gate(MotionDeadbandGate::Frame::kPlanarMetres, 0.2, kFiveDegrees, 30.0);
  gate.mark_published({0.0, 0.0, M_PI - 0.01}, 0.0);
  // -pi + 0.01 is 0.02 rad (~1.1 deg) away, not ~2 pi.
  EXPECT_FALSE(gate.should_publish({0.0, 0.0, -M_PI + 0.01}, 1.0));
}

TEST(MotionDeadbandGate, StationaryRobotPublishesOnlyTheHeartbeat)
{
  MotionDeadbandGate gate(MotionDeadbandGate::Frame::kPlanarMetres, 0.2, kFiveDegrees, 30.0);
  gate.mark_published({5.0, 5.0, 1.0}, 100.0);

  EXPECT_FALSE(gate.should_publish({5.01, 4.99, 1.01}, 129.9));
  EXPECT_TRUE(gate.should_publish({5.01, 4.99, 1.01}, 130.0));
}

TEST(MotionDeadbandGate, GpsDistanceUsesLatLonMetres)
{
  MotionDeadbandGate gate(MotionDeadbandGate::Frame::kWgs84Degrees, 0.2, kFiveDegrees, 30.0);
  const double lat = 48.8566;
  const double lon = 2.3522;
  gate.mark_published({lat, lon, std::nullopt}, 0.0);

  // 1e-6 deg of latitude ≈ 0.11 m: receiver jitter on the dock.
  EXPECT_FALSE(gate.should_publish({lat + 1e-6, lon - 1e-6, std::nullopt}, 1.0));
  // 3e-6 deg of latitude ≈ 0.33 m.
  EXPECT_TRUE(gate.should_publish({lat + 3e-6, lon, std::nullopt}, 1.0));
  // Longitude shrinks with cos(latitude): 2.5e-6 deg at 48.86° ≈ 0.18 m.
  EXPECT_FALSE(gate.should_publish({lat, lon + 2.5e-6, std::nullopt}, 1.0));
}

TEST(GpsDistance, MatchesTheEquirectangularApproximation)
{
  EXPECT_NEAR(mowgli_monitoring::gps_distance_m(0.0, 0.0, 1e-5, 0.0), 1.112, 0.001);
  EXPECT_NEAR(mowgli_monitoring::gps_distance_m(60.0, 10.0, 60.0, 10.00002), 1.112, 0.001);
  EXPECT_DOUBLE_EQ(mowgli_monitoring::gps_distance_m(48.0, 2.0, 48.0, 2.0), 0.0);
}

// ===========================================================================
// DiagnosticsAggregator
// ===========================================================================

TEST(DiagnosticsAggregator, MergesArraysFromSeveralPublishersByName)
{
  DiagnosticsAggregator aggregator(128, 60.0);
  aggregator.merge(make_array({{"GPS", 0, "RTK fixed"}, {"IMU", 0, "OK"}}), 0.0);
  aggregator.merge(make_array({{"controller_server: Nav2 Health", 0, "OK"}}), 0.1);

  const auto merged = aggregator.snapshot();
  ASSERT_EQ(merged.status.size(), 3U);
  // Sorted by name, so the payload does not shuffle between publishes.
  EXPECT_EQ(merged.status[0].name, "GPS");
  EXPECT_EQ(merged.status[1].name, "IMU");
  EXPECT_EQ(merged.status[2].name, "controller_server: Nav2 Health");
}

TEST(DiagnosticsAggregator, ReportsNewNamesAndLevelChangesButNotMessageOnlyChanges)
{
  DiagnosticsAggregator aggregator(128, 60.0);
  EXPECT_TRUE(aggregator.merge(make_array({{"GPS", 0, "RTK fixed, 5.0 Hz"}}), 0.0));
  EXPECT_FALSE(aggregator.merge(make_array({{"GPS", 0, "RTK fixed, 4.9 Hz"}}), 0.1));
  EXPECT_TRUE(aggregator.merge(make_array({{"GPS", 1, "RTK float"}}), 0.2));

  const auto merged = aggregator.snapshot();
  ASSERT_EQ(merged.status.size(), 1U);
  EXPECT_EQ(merged.status[0].level, 1);
  EXPECT_EQ(merged.status[0].message, "RTK float");
}

TEST(DiagnosticsAggregator, DropsEntriesWhosePublisherWentQuiet)
{
  DiagnosticsAggregator aggregator(128, 60.0);
  aggregator.merge(make_array({{"stale", 2, "dead node"}}), 0.0);
  aggregator.merge(make_array({{"live", 0, "OK"}}), 50.0);

  EXPECT_FALSE(aggregator.drop_stale(60.0));
  EXPECT_TRUE(aggregator.drop_stale(60.5));
  const auto merged = aggregator.snapshot();
  ASSERT_EQ(merged.status.size(), 1U);
  EXPECT_EQ(merged.status[0].name, "live");
}

TEST(DiagnosticsAggregator, IsBoundedByEvictingTheLeastRecentlySeenEntry)
{
  DiagnosticsAggregator aggregator(2, 60.0);
  aggregator.merge(make_array({{"a", 0, ""}}), 0.0);
  aggregator.merge(make_array({{"b", 0, ""}}), 1.0);
  aggregator.merge(make_array({{"a", 0, ""}}), 2.0);  // refresh a: b is now the oldest
  aggregator.merge(make_array({{"c", 0, ""}}), 3.0);

  const auto merged = aggregator.snapshot();
  ASSERT_EQ(aggregator.size(), 2U);
  EXPECT_EQ(merged.status[0].name, "a");
  EXPECT_EQ(merged.status[1].name, "c");
}
