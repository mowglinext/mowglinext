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
 * @file test_mqtt_bridge.cpp
 * @brief Unit tests for MqttBridgeNode's JSON serialisers and command parsing.
 *
 * All `serialise_*` functions and `parse_command_payload`/`json_escape` are
 * pure static methods (mqtt_bridge_node.hpp's "exposed for testing" section),
 * so they're tested directly with no ROS2 middleware, broker, or node
 * construction required — same isolation strategy as test_diagnostics.cpp's
 * classify_*() tests.
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/map_area.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_monitoring/mqtt_bridge_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <gtest/gtest.h>

using mowgli_monitoring::IMqttClient;
using mowgli_monitoring::MqttBridgeNode;

namespace
{

class RecordingMqttClient : public IMqttClient
{
public:
  struct Publication
  {
    std::string topic;
    std::string payload;
    bool retained;
  };

  bool connect() noexcept override
  {
    connected = true;
    return true;
  }
  void disconnect() noexcept override
  {
    connected = false;
  }
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retained) noexcept override
  {
    publications.push_back({topic, payload, retained});
    return true;
  }
  bool subscribe(const std::string& topic, MessageCallback callback) noexcept override
  {
    subscriptions[topic] = std::move(callback);
    return true;
  }
  void spin_once() noexcept override
  {
  }
  bool is_connected() const noexcept override
  {
    return connected;
  }

  bool connected{true};
  std::vector<Publication> publications;
  std::unordered_map<std::string, MessageCallback> subscriptions;
};

class HomeAssistantDiscoveryNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

// ===========================================================================
// MQTT callback metadata
// ===========================================================================

TEST(MqttMessageCallback, DeliversRetainedMetadata)
{
  bool received_retained = false;
  IMqttClient::MessageCallback callback =
      [&received_retained](const std::string&, const std::string&, bool retained)
  {
    received_retained = retained;
  };

  callback("mowgli/command", "1", true);
  EXPECT_TRUE(received_retained);
}

TEST(MqttControlMessage, RejectsRetainedDelivery)
{
  EXPECT_FALSE(MqttBridgeNode::is_fresh_control_message(true));
  EXPECT_TRUE(MqttBridgeNode::is_fresh_control_message(false));
}

// ===========================================================================
// json_escape
// ===========================================================================

TEST(JsonEscape, PassesThroughPlainText)
{
  EXPECT_EQ(MqttBridgeNode::json_escape("hello world"), "hello world");
  EXPECT_EQ(MqttBridgeNode::json_escape(""), "");
}

TEST(JsonEscape, EscapesQuotesAndBackslashes)
{
  EXPECT_EQ(MqttBridgeNode::json_escape(R"(say "hi")"), R"(say \"hi\")");
  EXPECT_EQ(MqttBridgeNode::json_escape(R"(a\b)"), R"(a\\b)");
}

TEST(JsonEscape, EscapesControlCharacters)
{
  EXPECT_EQ(MqttBridgeNode::json_escape("a\nb\rc\td"), "a\\nb\\rc\\td");
}

// ===========================================================================
// Home Assistant MQTT device discovery
// ===========================================================================

TEST(HomeAssistantDiscovery, DerivesBrokerSafeStableDeviceTopic)
{
  EXPECT_EQ(MqttBridgeNode::home_assistant_device_id("garden/front mower"),
            "mowglinext_garden_front_mower");
  EXPECT_EQ(MqttBridgeNode::home_assistant_discovery_topic("garden/front mower"),
            "homeassistant/device/mowglinext_garden_front_mower/config");
  EXPECT_EQ(MqttBridgeNode::home_assistant_device_id(""), "mowglinext_mowgli");
}

TEST(HomeAssistantDiscovery, PublishesOneDeviceWithControlsAndTelemetry)
{
  const std::string json = MqttBridgeNode::serialise_home_assistant_discovery("garden");

  EXPECT_NE(json.find(R"("identifiers":["mowglinext_garden"])"), std::string::npos);
  EXPECT_NE(json.find(R"("origin":{"name":"MowgliNext MQTT bridge")"), std::string::npos);
  EXPECT_NE(json.find(R"("availability_topic":"garden/available")"), std::string::npos);
  EXPECT_NE(json.find(R"("platform":"lawn_mower")"), std::string::npos);
  EXPECT_NE(json.find(R"("activity_state_topic":"garden/high_level_status")"), std::string::npos);
  EXPECT_NE(json.find(R"("start_mowing_command_template":"1")"), std::string::npos);
  EXPECT_NE(json.find(R"("pause_command_template":"8")"), std::string::npos);
  EXPECT_NE(json.find(R"("dock_command_template":"2")"), std::string::npos);
  EXPECT_NE(json.find(R"("platform":"device_tracker")"), std::string::npos);
  EXPECT_NE(json.find(R"("json_attributes_topic":"garden/gps")"), std::string::npos);
  EXPECT_NE(json.find(R"("unique_id":"mowglinext_garden_battery")"), std::string::npos);
  EXPECT_NE(json.find("value_json.state_name"), std::string::npos);
}

TEST(HomeAssistantDiscovery, EmptyPrefixFallsBackToMowgliDataTopics)
{
  const std::string json = MqttBridgeNode::serialise_home_assistant_discovery("");
  EXPECT_NE(json.find(R"("state_topic":"mowgli/power")"), std::string::npos);
  EXPECT_NE(json.find(R"("availability_topic":"mowgli/available")"), std::string::npos);
}

TEST(HomeAssistantDiscovery, AddsExplicitMowButtonForEachCurrentArea)
{
  const std::vector<MqttBridgeNode::AreaSummary> areas{
      {2, "Back \"Garden\""},
      {7, "Side lawn"},
  };
  const std::string json = MqttBridgeNode::serialise_home_assistant_discovery("garden", areas);

  EXPECT_NE(json.find(R"("platform":"button","name":"Mow Back \"Garden\"")"), std::string::npos);
  EXPECT_NE(json.find(R"("command_topic":"garden/start_area","payload_press":"2")"),
            std::string::npos);
  EXPECT_NE(json.find(R"("platform":"button","name":"Mow Side lawn")"), std::string::npos);
  EXPECT_NE(json.find(R"("payload_press":"7")"), std::string::npos);
  EXPECT_NE(json.find("mow_area_2_mowglinext_Back__Garden_"), std::string::npos);
}

TEST_F(HomeAssistantDiscoveryNodeTest, RepublishesDiscoveryWhenHomeAssistantComesOnline)
{
  auto client = std::make_unique<RecordingMqttClient>();
  RecordingMqttClient* recording = client.get();
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("mqtt_topic_prefix", "back_garden"),
      rclcpp::Parameter("home_assistant_discovery_enabled", true),
      rclcpp::Parameter("publish_rate", 0.1),
  });
  auto node = std::make_shared<MqttBridgeNode>(std::move(client), options);

  ASSERT_EQ(recording->subscriptions.count("homeassistant/status"), 1U);
  recording->subscriptions.at("homeassistant/status")("homeassistant/status", "online", false);
  EXPECT_TRUE(recording->publications.empty());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  // MQTT servicing is independent of a deliberately slow telemetry rate.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  executor.spin_some();
  executor.remove_node(node);

  ASSERT_EQ(recording->publications.size(), 1U);
  EXPECT_EQ(recording->publications[0].topic, "homeassistant/device/mowglinext_back_garden/config");
  EXPECT_TRUE(recording->publications[0].retained);
  EXPECT_NE(recording->publications[0].payload.find(R"("platform":"lawn_mower")"),
            std::string::npos);
}

// ===========================================================================
// serialise_status
// ===========================================================================

TEST(SerialiseStatus, ProducesExpectedJson)
{
  mowgli_interfaces::msg::Status msg{};
  msg.mower_status = 1;
  msg.raspberry_pi_power = true;
  msg.is_charging = false;
  msg.esc_power = true;
  msg.rain_detected = false;
  msg.sound_module_available = true;
  msg.sound_module_busy = false;
  msg.ui_board_available = true;
  msg.mow_enabled = false;
  msg.mower_esc_status = 2;
  msg.mower_esc_temperature = 45.5f;
  msg.mower_esc_current = 1.25f;
  msg.mower_motor_temperature = 60.75f;
  msg.mower_motor_rpm = 3000.5f;

  const std::string json = MqttBridgeNode::serialise_status(msg);

  EXPECT_EQ(json,
            "{\"mower_status\":1,\"raspberry_pi_power\":true,\"is_charging\":false,"
            "\"esc_power\":true,\"rain_detected\":false,\"sound_module_available\":true,"
            "\"sound_module_busy\":false,\"ui_board_available\":true,\"mow_enabled\":false,"
            "\"mower_esc_status\":2,\"mower_esc_temperature\":45.50,"
            "\"mower_esc_current\":1.250,\"mower_motor_temperature\":60.75,"
            "\"mower_motor_rpm\":3000.5}");
}

// ===========================================================================
// serialise_power
// ===========================================================================

TEST(SerialisePower, ProducesExpectedJsonAndDerivesBatteryPercent)
{
  mowgli_interfaces::msg::Power msg{};
  msg.v_charge = 16.5f;
  msg.v_battery = 14.4f;  // midpoint of the 12.0-16.8V 4S LiPo range -> 50.0%
  msg.charge_current = 0.75f;
  msg.charger_enabled = true;
  msg.charger_status = "bulk";

  const std::string json = MqttBridgeNode::serialise_power(msg);

  EXPECT_EQ(json,
            "{\"v_charge\":16.500,\"v_battery\":14.400,\"charge_current\":0.750,"
            "\"charger_enabled\":true,\"charger_status\":\"bulk\",\"battery_pct\":50.0}");
}

TEST(SerialisePower, ClampsBatteryPercentToZeroAndHundred)
{
  mowgli_interfaces::msg::Power below{};
  below.v_battery = 5.0f;  // below kVEmpty (12.0)
  EXPECT_NE(MqttBridgeNode::serialise_power(below).find("\"battery_pct\":0.0"), std::string::npos);

  mowgli_interfaces::msg::Power above{};
  above.v_battery = 20.0f;  // above kVFull (16.8)
  EXPECT_NE(MqttBridgeNode::serialise_power(above).find("\"battery_pct\":100.0"),
            std::string::npos);
}

TEST(SerialisePower, EscapesChargerStatusString)
{
  mowgli_interfaces::msg::Power msg{};
  msg.charger_status = "fault: \"overcurrent\"";
  const std::string json = MqttBridgeNode::serialise_power(msg);
  EXPECT_NE(json.find(R"("charger_status":"fault: \"overcurrent\"")"), std::string::npos);
}

// ===========================================================================
// serialise_emergency
// ===========================================================================

TEST(SerialiseEmergency, ProducesExpectedJson)
{
  mowgli_interfaces::msg::Emergency msg{};
  msg.active_emergency = true;
  msg.latched_emergency = true;
  msg.reason = "lift detected";

  EXPECT_EQ(MqttBridgeNode::serialise_emergency(msg),
            "{\"active_emergency\":true,\"latched_emergency\":true,\"reason\":\"lift detected\"}");
}

// ===========================================================================
// serialise_position
// ===========================================================================

TEST(SerialisePosition, ExtractsXyAndYawFromOdometry)
{
  nav_msgs::msg::Odometry msg{};
  msg.pose.pose.position.x = 1.2345;
  msg.pose.pose.position.y = -6.789;
  msg.pose.pose.orientation.z = 0.0;
  msg.pose.pose.orientation.w = 1.0;  // identity quaternion -> theta = 0

  EXPECT_EQ(MqttBridgeNode::serialise_position(msg),
            "{\"x\":1.2345,\"y\":-6.7890,\"theta\":0.0000}");
}

// ===========================================================================
// serialise_diagnostics
// ===========================================================================

TEST(SerialiseDiagnostics, EmptyArrayIsEmptyJsonArray)
{
  diagnostic_msgs::msg::DiagnosticArray msg{};
  EXPECT_EQ(MqttBridgeNode::serialise_diagnostics(msg), "[]");
}

TEST(SerialiseDiagnostics, MultipleEntriesAndEscaping)
{
  diagnostic_msgs::msg::DiagnosticArray msg{};
  diagnostic_msgs::msg::DiagnosticStatus a;
  a.name = "GPS";
  a.level = 0;
  a.message = "OK";
  diagnostic_msgs::msg::DiagnosticStatus b;
  b.name = "LiDAR";
  b.level = 2;
  b.message = "No \"scan\" received";
  msg.status = {a, b};

  EXPECT_EQ(MqttBridgeNode::serialise_diagnostics(msg),
            "[{\"name\":\"GPS\",\"level\":0,\"message\":\"OK\"},"
            "{\"name\":\"LiDAR\",\"level\":2,\"message\":\"No \\\"scan\\\" received\"}]");
}

// ===========================================================================
// serialise_high_level_status
// ===========================================================================

TEST(SerialiseHighLevelStatus, ProducesExpectedJson)
{
  mowgli_interfaces::msg::HighLevelStatus msg{};
  msg.state = mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
  msg.state_name = "AUTONOMOUS";
  msg.sub_state_name = "MOWING";
  msg.current_area = 2;
  msg.current_path = -1;  // -1 = no active sub-path (e.g. mid blade-off transit)
  msg.current_path_index = 0;
  msg.total_swaths = 40;
  msg.completed_swaths = 12;
  msg.skipped_swaths = 1;
  msg.coverage_percent = 42.5f;
  // gps_quality_percent is misnamed at the source: behavior_tree_node.cpp's
  // context_->gps_quality is a 0.0-1.0 fraction, assigned straight into this
  // field with no *100 (status_snapshot.cpp). serialise_high_level_status()
  // scales it up here so the wire field genuinely means "percent" — see the
  // comment at its definition.
  msg.gps_quality_percent = 0.99f;
  msg.battery_percent = 73.5f;
  msg.is_charging = false;
  msg.emergency = false;

  const std::string json = MqttBridgeNode::serialise_high_level_status(msg);

  EXPECT_EQ(json,
            "{\"state\":2,\"state_name\":\"AUTONOMOUS\",\"sub_state_name\":\"MOWING\","
            "\"current_area\":2,\"current_path\":-1,\"current_path_index\":0,"
            "\"total_swaths\":40,\"completed_swaths\":12,\"skipped_swaths\":1,"
            "\"coverage_percent\":42.5,\"gps_quality_percent\":99.0,"
            "\"battery_percent\":73.5,\"is_charging\":false,\"emergency\":false}");
}

TEST(SerialiseHighLevelStatus, ScalesGpsQualityFractionToPercentAndClamps)
{
  mowgli_interfaces::msg::HighLevelStatus full{};
  full.gps_quality_percent = 1.0f;  // "fully good" fix, per the 0.0-1.0 source convention
  EXPECT_NE(MqttBridgeNode::serialise_high_level_status(full).find("\"gps_quality_percent\":100.0"),
            std::string::npos);

  mowgli_interfaces::msg::HighLevelStatus none{};
  none.gps_quality_percent = 0.0f;
  EXPECT_NE(MqttBridgeNode::serialise_high_level_status(none).find("\"gps_quality_percent\":0.0"),
            std::string::npos);

  // Defensive: the source field is documented 0.0-1.0 and should never
  // exceed it, but a caller passing an already-scaled 0-100 value by
  // mistake must not silently produce a nonsensical >100% reading.
  mowgli_interfaces::msg::HighLevelStatus over{};
  over.gps_quality_percent = 50.0f;
  EXPECT_NE(MqttBridgeNode::serialise_high_level_status(over).find("\"gps_quality_percent\":100.0"),
            std::string::npos);
}

TEST(SerialiseHighLevelStatus, EscapesSubStateName)
{
  mowgli_interfaces::msg::HighLevelStatus msg{};
  msg.sub_state_name = "DIG_OBSTRUCTION";
  const std::string json = MqttBridgeNode::serialise_high_level_status(msg);
  EXPECT_NE(json.find("\"sub_state_name\":\"DIG_OBSTRUCTION\""), std::string::npos);
}

// ===========================================================================
// serialise_gps
// ===========================================================================

TEST(SerialiseGps, ProducesExpectedJson)
{
  sensor_msgs::msg::NavSatFix msg{};
  msg.latitude = 52.12345678;
  msg.longitude = -6.98765432;
  msg.altitude = 12.345;
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  EXPECT_EQ(MqttBridgeNode::serialise_gps(msg),
            "{\"latitude\":52.12345678,\"longitude\":-6.98765432,\"altitude\":12.345,"
            "\"status\":0,\"service\":1}");
}

TEST(SerialiseGps, NoFixStatusIsNegative)
{
  sensor_msgs::msg::NavSatFix msg{};
  msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  EXPECT_NE(MqttBridgeNode::serialise_gps(msg).find("\"status\":-1"), std::string::npos);
}

// ===========================================================================
// serialise_rtk_status
// ===========================================================================

TEST(SerialiseRtkStatus, RtkFixedProducesExpectedJson)
{
  mowgli_interfaces::msg::GnssStatus msg{};
  msg.fix_type = mowgli_interfaces::msg::GnssStatus::FIX_TYPE_RTK_FIXED;
  msg.rtk_mode = mowgli_interfaces::msg::GnssStatus::RTK_MODE_FIXED;
  msg.fix_valid = true;

  EXPECT_EQ(MqttBridgeNode::serialise_rtk_status(msg),
            "{\"fix_type\":3,\"fix_type_name\":\"RTK_FIXED\",\"rtk_mode\":3,"
            "\"rtk_mode_name\":\"FIXED\",\"fix_valid\":true,\"quality_percent\":100}");
}

TEST(SerialiseRtkStatus, RtkFloatIsSeventyPercentQuality)
{
  // Matches gnss_status_utils::NormalizedQuality's RTK_FLOAT branch (0.7)
  // when no horizontal_accuracy_m capability is present — same value
  // test_gnss_status_authority.cpp pins for HardwareQualityPercent.
  mowgli_interfaces::msg::GnssStatus msg{};
  msg.fix_type = mowgli_interfaces::msg::GnssStatus::FIX_TYPE_RTK_FLOAT;
  msg.rtk_mode = mowgli_interfaces::msg::GnssStatus::RTK_MODE_FLOAT;
  msg.fix_valid = true;

  const std::string json = MqttBridgeNode::serialise_rtk_status(msg);
  EXPECT_NE(json.find("\"fix_type_name\":\"RTK_FLOAT\""), std::string::npos);
  EXPECT_NE(json.find("\"rtk_mode_name\":\"FLOAT\""), std::string::npos);
  EXPECT_NE(json.find("\"quality_percent\":70"), std::string::npos);
}

TEST(SerialiseRtkStatus, InvalidFixIsZeroQualityRegardlessOfFixType)
{
  // fix_valid=false must win over a stale/leftover fix_type — matches
  // gnss_status_utils::IsRtkFixed/IsRtkFloat/NormalizedQuality, which all
  // check fix_valid first.
  mowgli_interfaces::msg::GnssStatus msg{};
  msg.fix_type = mowgli_interfaces::msg::GnssStatus::FIX_TYPE_RTK_FIXED;
  msg.rtk_mode = mowgli_interfaces::msg::GnssStatus::RTK_MODE_FIXED;
  msg.fix_valid = false;

  const std::string json = MqttBridgeNode::serialise_rtk_status(msg);
  EXPECT_NE(json.find("\"fix_valid\":false"), std::string::npos);
  EXPECT_NE(json.find("\"quality_percent\":0"), std::string::npos);
}

TEST(SerialiseRtkStatus, UnknownEnumValuesFallBackToUnknownName)
{
  mowgli_interfaces::msg::GnssStatus msg{};
  msg.fix_type = 255;
  msg.rtk_mode = 255;

  const std::string json = MqttBridgeNode::serialise_rtk_status(msg);
  EXPECT_NE(json.find("\"fix_type_name\":\"UNKNOWN\""), std::string::npos);
  EXPECT_NE(json.find("\"rtk_mode_name\":\"UNKNOWN\""), std::string::npos);
}

// ===========================================================================
// serialise_areas
// ===========================================================================

TEST(SerialiseAreas, EmptyListIsEmptyJsonArray)
{
  EXPECT_EQ(MqttBridgeNode::serialise_areas({}), "[]");
}

TEST(SerialiseAreas, ProducesExpectedJsonWithRawIndices)
{
  // "index" is deliberately the raw map_server_node index, not a compacted
  // 0..N-1 position — a navigation-only area between two mowing areas would
  // leave a gap here, matching what GetMowingArea/StartInArea expect.
  std::vector<MqttBridgeNode::AreaSummary> areas{
      {0, "Front Lawn"},
      {2, "Back Garden"},
  };

  EXPECT_EQ(MqttBridgeNode::serialise_areas(areas),
            "[{\"index\":0,\"name\":\"Front Lawn\"},{\"index\":2,\"name\":\"Back Garden\"}]");
}

TEST(SerialiseAreas, EscapesAreaName)
{
  std::vector<MqttBridgeNode::AreaSummary> areas{{1, R"(Side "yard")"}};
  EXPECT_EQ(MqttBridgeNode::serialise_areas(areas), R"([{"index":1,"name":"Side \"yard\""}])");
}

// ===========================================================================
// serialise_area_boundaries
// ===========================================================================

TEST(SerialiseAreaBoundaries, EmptyListProducesEmptyAreasArray)
{
  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> areas{};

  EXPECT_EQ(MqttBridgeNode::serialise_area_boundaries(areas, 52.0, 4.5),
            "{\"datum_lat\":52.00000000,\"datum_lon\":4.50000000,\"areas\":[]}");
}

TEST(SerialiseAreaBoundaries, SingleAreaWithBoundaryAndNoObstacles)
{
  mowgli_interfaces::msg::MapArea area{};
  area.name = "Front Lawn";
  geometry_msgs::msg::Point32 p0;
  p0.x = 1.0f;
  p0.y = 2.0f;
  geometry_msgs::msg::Point32 p1;
  p1.x = 3.5f;
  p1.y = -4.25f;
  area.area.points = {p0, p1};

  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> areas{{0, area}};

  EXPECT_EQ(MqttBridgeNode::serialise_area_boundaries(areas, 0.0, 0.0),
            "{\"datum_lat\":0.00000000,\"datum_lon\":0.00000000,\"areas\":["
            "{\"index\":0,\"name\":\"Front Lawn\",\"boundary\":[[1.000,2.000],[3.500,-4.250]],"
            "\"obstacles\":[]}]}");
}

TEST(SerialiseAreaBoundaries, IncludesObstaclePolygons)
{
  mowgli_interfaces::msg::MapArea area{};
  area.name = "Back Lawn";
  geometry_msgs::msg::Point32 boundary_pt;
  boundary_pt.x = 10.0f;
  boundary_pt.y = 10.0f;
  area.area.points = {boundary_pt};

  geometry_msgs::msg::Point32 obstacle_pt0;
  obstacle_pt0.x = 1.0f;
  obstacle_pt0.y = 1.0f;
  geometry_msgs::msg::Point32 obstacle_pt1;
  obstacle_pt1.x = 2.0f;
  obstacle_pt1.y = 1.0f;
  geometry_msgs::msg::Polygon obstacle;
  obstacle.points = {obstacle_pt0, obstacle_pt1};
  area.obstacles = {obstacle};

  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> areas{{2, area}};

  const std::string json = MqttBridgeNode::serialise_area_boundaries(areas, 0.0, 0.0);
  EXPECT_NE(json.find("\"index\":2"), std::string::npos);
  EXPECT_NE(json.find("\"obstacles\":[[[1.000,1.000],[2.000,1.000]]]"), std::string::npos);
}

TEST(SerialiseAreaBoundaries, EscapesAreaName)
{
  mowgli_interfaces::msg::MapArea area{};
  area.name = "Back \"yard\"";

  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> areas{{0, area}};

  const std::string json = MqttBridgeNode::serialise_area_boundaries(areas, 0.0, 0.0);
  EXPECT_NE(json.find("\"name\":\"Back \\\"yard\\\"\""), std::string::npos);
}

TEST(SerialiseAreaBoundaries, PreservesNonContiguousIndicesAndMultipleAreas)
{
  // Indices are exactly whatever the caller polled — this serialiser does
  // not renumber, matching <prefix>/areas' own contract that indices are
  // not assumed stable/contiguous (mowglinext#637).
  mowgli_interfaces::msg::MapArea area0{};
  area0.name = "A";
  mowgli_interfaces::msg::MapArea area5{};
  area5.name = "B";

  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> areas{{0, area0},
                                                                                {5, area5}};

  const std::string json = MqttBridgeNode::serialise_area_boundaries(areas, 0.0, 0.0);
  EXPECT_NE(json.find("\"index\":0,\"name\":\"A\""), std::string::npos);
  EXPECT_NE(json.find("\"index\":5,\"name\":\"B\""), std::string::npos);
}

// ===========================================================================
// parse_command_payload
// ===========================================================================

TEST(ParseCommandPayload, AcceptsBoundaryValues)
{
  uint8_t out = 0;
  EXPECT_TRUE(MqttBridgeNode::parse_command_payload("0", out));
  EXPECT_EQ(out, 0);
  EXPECT_TRUE(MqttBridgeNode::parse_command_payload("255", out));
  EXPECT_EQ(out, 255);
  EXPECT_TRUE(MqttBridgeNode::parse_command_payload("1", out));
  EXPECT_EQ(out, 1);
  EXPECT_TRUE(MqttBridgeNode::parse_command_payload("254", out));
  EXPECT_EQ(out, 254);
}

TEST(ParseCommandPayload, RejectsOutOfRangeAndNonNumeric)
{
  uint8_t out = 0;
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("256", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("-1", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("abc", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("", out));
}

TEST(ParseCommandPayload, RejectsWhitespaceSignsAndTrailingCharacters)
{
  uint8_t out = 42;
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("1abc", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("1 ", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload(" 1", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("+1", out));
  EXPECT_FALSE(MqttBridgeNode::parse_command_payload("1\n", out));
  EXPECT_EQ(out, 42);
}

// ===========================================================================
// is_high_level_status_stale (mowglinext#644)
// ===========================================================================

TEST(IsHighLevelStatusStale, FalseBeforeAnyMessageEverReceived)
{
  // Never received one yet == still starting up (behavior_tree_node may not
  // be up), not evidence of a stuck subscription — regardless of how much
  // time has passed.
  const rclcpp::Time epoch{0, 0, RCL_ROS_TIME};
  const rclcpp::Time far_future = epoch + rclcpp::Duration::from_seconds(3600.0);
  EXPECT_FALSE(MqttBridgeNode::is_high_level_status_stale(
      /*received_before=*/false, far_future, epoch, /*threshold_s=*/5.0));
}

TEST(IsHighLevelStatusStale, FalseWithinThreshold)
{
  const rclcpp::Time last_received{10, 0, RCL_ROS_TIME};
  const rclcpp::Time now = last_received + rclcpp::Duration::from_seconds(2.0);
  EXPECT_FALSE(MqttBridgeNode::is_high_level_status_stale(
      /*received_before=*/true, now, last_received, /*threshold_s=*/5.0));
}

TEST(IsHighLevelStatusStale, FalseExactlyAtThreshold)
{
  // Strict '>' — exactly at the threshold is not yet stale, avoiding a
  // resubscribe right on the boundary of a perfectly-timed heartbeat.
  const rclcpp::Time last_received{10, 0, RCL_ROS_TIME};
  const rclcpp::Time now = last_received + rclcpp::Duration::from_seconds(5.0);
  EXPECT_FALSE(MqttBridgeNode::is_high_level_status_stale(
      /*received_before=*/true, now, last_received, /*threshold_s=*/5.0));
}

TEST(IsHighLevelStatusStale, TrueBeyondThreshold)
{
  const rclcpp::Time last_received{10, 0, RCL_ROS_TIME};
  const rclcpp::Time now = last_received + rclcpp::Duration::from_seconds(5.001);
  EXPECT_TRUE(MqttBridgeNode::is_high_level_status_stale(
      /*received_before=*/true, now, last_received, /*threshold_s=*/5.0));
}

// ===========================================================================
// is_publish_due (rate limiter for status/power/rtk_status/position/gps)
// ===========================================================================

TEST(IsPublishDue, NeverPublishedBeforeIsDueImmediately)
{
  const rclcpp::Time never_published{0, 0, RCL_ROS_TIME};
  const rclcpp::Time now{1000, 0, RCL_ROS_TIME};
  EXPECT_TRUE(MqttBridgeNode::is_publish_due(now, never_published, /*min_interval_s=*/1.0));
}

TEST(IsPublishDue, NotDueWithinInterval)
{
  const rclcpp::Time last{10, 0, RCL_ROS_TIME};
  const rclcpp::Time now = last + rclcpp::Duration::from_seconds(0.4);
  EXPECT_FALSE(MqttBridgeNode::is_publish_due(now, last, /*min_interval_s=*/1.0));
}

TEST(IsPublishDue, DueExactlyAtInterval)
{
  // '>=' — the boundary tick publishes, so a 1 Hz limiter driven by a 1 Hz
  // timer does not skip every other tick to timer jitter.
  const rclcpp::Time last{10, 0, RCL_ROS_TIME};
  const rclcpp::Time now = last + rclcpp::Duration::from_seconds(1.0);
  EXPECT_TRUE(MqttBridgeNode::is_publish_due(now, last, /*min_interval_s=*/1.0));
}

TEST(IsPublishDue, DueBeyondInterval)
{
  const rclcpp::Time last{10, 0, RCL_ROS_TIME};
  const rclcpp::Time now = last + rclcpp::Duration::from_seconds(5.0);
  EXPECT_TRUE(MqttBridgeNode::is_publish_due(now, last, /*min_interval_s=*/1.0));
}

// ===========================================================================
// <prefix>/pose (fused map-frame pose) and the dock in <prefix>/area_boundary
// ===========================================================================

namespace
{
nav_msgs::msg::Odometry pose_with_yaw(double x, double y, double yaw)
{
  nav_msgs::msg::Odometry msg{};
  msg.pose.pose.position.x = x;
  msg.pose.pose.position.y = y;
  msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
  return msg;
}
}  // namespace

TEST(SerialisePose, ExtractsXyAndYaw)
{
  EXPECT_EQ(MqttBridgeNode::serialise_pose(pose_with_yaw(1.25, -6.5, M_PI / 2.0)),
            "{\"x\":1.250,\"y\":-6.500,\"yaw\":1.5708}");
}

TEST(SerialisePose, IdentityOrientationIsYawZero)
{
  nav_msgs::msg::Odometry msg{};
  msg.pose.pose.orientation.w = 1.0;
  EXPECT_EQ(MqttBridgeNode::serialise_pose(msg), "{\"x\":0.000,\"y\":0.000,\"yaw\":0.0000}");
}

TEST(SerialisePose, YawIsWrappedIntoMinusPiToPi)
{
  // 270 degrees is the same heading as -90 degrees.
  EXPECT_EQ(MqttBridgeNode::serialise_pose(pose_with_yaw(0.0, 0.0, 3.0 * M_PI / 2.0)),
            "{\"x\":0.000,\"y\":0.000,\"yaw\":-1.5708}");
}

TEST(MakeDockPose, DefaultZeroMeansNoDockCalibrated)
{
  EXPECT_FALSE(MqttBridgeNode::make_dock_pose(0.0, 0.0, 0.0).has_value());
}

TEST(MakeDockPose, AnyNonZeroComponentIsARealDock)
{
  // The datum can sit on the dock (x = y = 0); a calibrated yaw is never exactly 0.
  const auto dock = MqttBridgeNode::make_dock_pose(0.0, 0.0, 1.5);
  ASSERT_TRUE(dock.has_value());
  EXPECT_DOUBLE_EQ(dock->yaw, 1.5);
  EXPECT_TRUE(MqttBridgeNode::make_dock_pose(3.0, -2.0, 0.0).has_value());
}

TEST(MakeDockPose, NonFiniteValuesAreRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(MqttBridgeNode::make_dock_pose(nan, 1.0, 1.0).has_value());
  EXPECT_FALSE(MqttBridgeNode::make_dock_pose(1.0, inf, 1.0).has_value());
  EXPECT_FALSE(MqttBridgeNode::make_dock_pose(1.0, 1.0, nan).has_value());
}

TEST(SerialiseAreaBoundaries, DockIsAppendedWhenSet)
{
  mowgli_interfaces::msg::MapArea area{};
  area.name = "Front Lawn";
  geometry_msgs::msg::Point32 p0;
  p0.x = 1.0f;
  p0.y = 2.0f;
  area.area.points = {p0};
  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> areas{{0, area}};

  const auto json = MqttBridgeNode::serialise_area_boundaries(
      areas, 52.0, 4.0, MqttBridgeNode::DockPose{1.25, -3.5, 1.5708});
  EXPECT_EQ(json,
            "{\"datum_lat\":52.00000000,\"datum_lon\":4.00000000,\"areas\":["
            "{\"index\":0,\"name\":\"Front Lawn\",\"boundary\":[[1.000,2.000]],"
            "\"obstacles\":[]}],"
            "\"dock\":{\"x\":1.250,\"y\":-3.500,\"yaw\":1.5708}}");
}

TEST(SerialiseAreaBoundaries, NoDockKeyWhenUnset)
{
  const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>> none{};
  EXPECT_EQ(MqttBridgeNode::serialise_area_boundaries(none, 0.0, 0.0, std::nullopt),
            "{\"datum_lat\":0.00000000,\"datum_lon\":0.00000000,\"areas\":[]}");
}
