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
 * @file test_mqtt_connection.cpp
 * @brief ROS-free tests of mqtt_connection.hpp: the derived client ID and the
 *        reconnect backoff.
 */

#include <string>
#include <vector>

#include "mowgli_monitoring/mqtt_connection.hpp"
#include <gtest/gtest.h>

using mowgli_monitoring::mqtt_client_id;
using mowgli_monitoring::ReconnectBackoff;

// ===========================================================================
// mqtt_client_id — the same expected IDs are checked by the GUI's
// TestMqttClientIDMatchesTheBridgeRule (gui/pkg/api/schedule_mqtt_test.go).
// ===========================================================================

TEST(MqttClientId, DerivesAStableIdFromThePrefixAndRole)
{
  EXPECT_EQ(mqtt_client_id("mowgli", "bridge"), "mowglinext_mowgli_059db5cc_bridge");
  EXPECT_EQ(mqtt_client_id("mowgli", "gui"), "mowglinext_mowgli_059db5cc_gui");
  EXPECT_EQ(mqtt_client_id("mowgli/mowglinext", "bridge"),
            "mowglinext_mowgli_mowglinext_9726e199_bridge");
  EXPECT_EQ(mqtt_client_id("mowgli/mowglimavros", "bridge"),
            "mowglinext_mowgli_mowglimavros_860e58cc_bridge");
}

TEST(MqttClientId, ReplacedBytesDoNotMakeTwoPrefixesCollide)
{
  EXPECT_EQ(mqtt_client_id("a/b", "bridge"), "mowglinext_a_b_3a8e75c1_bridge");
  EXPECT_EQ(mqtt_client_id("a_b", "bridge"), "mowglinext_a_b_1ba46871_bridge");
  EXPECT_EQ(mqtt_client_id("garden/front mower", "bridge"),
            "mowglinext_garden_front_mower_5cefaa8a_bridge");
  // Every byte of a multi-byte UTF-8 character is replaced.
  EXPECT_EQ(mqtt_client_id("tondeuse-\xc3\xa9t\xc3\xa9", "bridge"),
            "mowglinext_tondeuse-__t___d2eb9165_bridge");
}

TEST(MqttClientId, CutsALongPrefixButKeepsItsWholeHash)
{
  const std::string long_prefix(40, 'x');
  EXPECT_EQ(mqtt_client_id(long_prefix, "bridge"),
            "mowglinext_" + std::string(32, 'x') + "_84ae4e65_bridge");
  EXPECT_NE(mqtt_client_id(long_prefix, "bridge"), mqtt_client_id(std::string(41, 'x'), "bridge"));
}

TEST(MqttClientId, EmptyPrefixStillGetsAnId)
{
  EXPECT_EQ(mqtt_client_id("", "bridge"), "mowglinext__811c9dc5_bridge");
}

// ===========================================================================
// ReconnectBackoff
// ===========================================================================

namespace
{

// Ticks every kStepS (exact in binary, so tick times are exact) from `from_s` to
// `until_s` while disconnected; returns the ticks at which an attempt was due.
constexpr double kStepS = 0.25;

std::vector<double> attempts(ReconnectBackoff& backoff, double from_s, double until_s)
{
  std::vector<double> times;
  for (int i = 0; from_s + i * kStepS <= until_s; ++i)
  {
    const double t = from_s + i * kStepS;
    if (backoff.attempt_due(t))
    {
      times.push_back(t);
    }
  }
  return times;
}

void stay_connected(ReconnectBackoff& backoff, double from_s, double until_s)
{
  for (int i = 0; from_s + i * kStepS <= until_s; ++i)
  {
    backoff.connected(from_s + i * kStepS);
  }
}

}  // namespace

TEST(ReconnectBackoff, DoublesTheWaitUpToTheMaximum)
{
  ReconnectBackoff backoff;
  ASSERT_TRUE(backoff.attempt_due(0.0));
  EXPECT_DOUBLE_EQ(backoff.next_attempt_in_s(0.0), 1.0);
  EXPECT_EQ(attempts(backoff, 0.25, 100.0),
            (std::vector<double>{1.0, 3.0, 7.0, 15.0, 31.0, 61.0, 91.0}));
  EXPECT_DOUBLE_EQ(backoff.delay_s(), ReconnectBackoff::kMaxDelayS);
}

TEST(ReconnectBackoff, AShortLivedSessionKeepsTheLongerWait)
{
  // A session evicted after a few seconds (client ID in use elsewhere) must not
  // reset the pacing, or two clients sharing an ID evict each other every second.
  ReconnectBackoff backoff;
  // Attempts at 0, 1, 3, 7 and 15 s: the one at 15 s set the next at 31 s.
  attempts(backoff, 0.0, 20.0);
  stay_connected(backoff, 20.0, 25.0);
  EXPECT_DOUBLE_EQ(backoff.delay_s(), ReconnectBackoff::kMaxDelayS);
  EXPECT_EQ(attempts(backoff, 25.0, 70.0), (std::vector<double>{31.0, 61.0}));
}

TEST(ReconnectBackoff, AStableSessionRestoresAQuickReconnect)
{
  ReconnectBackoff backoff;
  attempts(backoff, 0.0, 20.0);
  stay_connected(backoff, 20.0, 20.0 + ReconnectBackoff::kStableAfterS);
  EXPECT_DOUBLE_EQ(backoff.delay_s(), ReconnectBackoff::kInitialDelayS);
  // Lost after a long session: the first attempt is immediate, then 1 s, then 2 s.
  EXPECT_EQ(attempts(backoff, 60.0, 64.0), (std::vector<double>{60.0, 61.0, 63.0}));
}
