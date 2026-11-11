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
 * @file test_mqtt_goto.cpp
 * @brief MQTT GOTO: the pure helpers of mqtt_goto.hpp, then mqtt_bridge_node
 *        driven through a recording MQTT client against a fake ~/goto server.
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mowgli_interfaces/action/goto.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/wgs84_projection.hpp"
#include "mowgli_monitoring/mqtt_bridge_node.hpp"
#include "mowgli_monitoring/mqtt_goto.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <gtest/gtest.h>

namespace mg = mowgli_monitoring::mqtt_goto;
using Goto = mowgli_interfaces::action::Goto;
using namespace std::chrono_literals;

namespace
{

int64_t nowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string gotoPayload(const std::string& id, int64_t issued_ms, double lat, double lon)
{
  char buf[256];
  std::snprintf(buf,
                sizeof(buf),
                "{\"request_id\":\"%s\",\"issued_at\":\"%s\",\"lat\":%.8f,\"lon\":%.8f,"
                "\"yaw\":1.0}",
                id.c_str(),
                mg::format_iso_utc_ms(issued_ms).c_str(),
                lat,
                lon);
  return buf;
}

std::string tempPath(const std::string& name)
{
  return (std::filesystem::temp_directory_path() / ("mqtt_goto_" + name)).string();
}

}  // namespace

// ---------------------------------------------------------------------------
// Pure helpers
// ---------------------------------------------------------------------------

TEST(MqttGotoJson, ParsesAFlatObject)
{
  const auto object =
      mg::parse_flat_json_object(R"( {"a": "x\"y", "b": -1.5e2, "c": true, "d": null} )");
  ASSERT_TRUE(object);
  EXPECT_EQ(std::get<std::string>(object->at("a")), "x\"y");
  EXPECT_DOUBLE_EQ(std::get<double>(object->at("b")), -150.0);
  EXPECT_TRUE(std::get<bool>(object->at("c")));
  EXPECT_TRUE(std::holds_alternative<std::monostate>(object->at("d")));
}

TEST(MqttGotoJson, RejectsNestingDuplicatesAndTrailingText)
{
  for (const char* text : {R"({"a": {"b": 1}})",
                           R"({"a": [1]})",
                           R"({"a": 1, "a": 2})",
                           R"({"a": 1} x)",
                           R"({"a": 1,})",
                           R"({"a": nan})",
                           "not json",
                           ""})
  {
    EXPECT_FALSE(mg::parse_flat_json_object(text)) << text;
  }
}

TEST(MqttGotoJson, RejectsNumbersAndWhitespaceJsonForbids)
{
  for (const char* text : {R"({"a": 01})",
                           R"({"a": .5})",
                           R"({"a": 5.})",
                           R"({"a": -.5})",
                           R"({"a": 1e})",
                           R"({"a": +1})",
                           "{\v\"a\": 1}"})
  {
    EXPECT_FALSE(mg::parse_flat_json_object(text)) << text;
  }
  EXPECT_TRUE(mg::parse_flat_json_object(R"({"a": -0.5e-3, "b": 0, "c": 10})"));
}

TEST(MqttGotoTime, IsoUtcRoundTripAndGrammar)
{
  const auto ms = mg::parse_iso_utc_ms("2026-11-05T17:40:12.345Z");
  ASSERT_TRUE(ms);
  EXPECT_EQ(mg::format_iso_utc_ms(*ms), "2026-11-05T17:40:12.345Z");
  EXPECT_EQ(*mg::parse_iso_utc_ms("2026-11-05T17:40:12Z") % 1000, 0);
  for (const char* text : {"2026-11-05T17:40:12",
                           "2026-11-05T17:40:12+01:00",
                           "2026-11-05 17:40:12Z",
                           "2026-13-05T17:40:12Z",
                           "2026-11-05T17:40:12.Z",
                           "+026-11-05T17:40:12Z",
                           "2026- 1-05T17:40:12Z",
                           "2026-11-05T-1:40:12Z",
                           "2026-11-05T17:-1:12Z"})
  {
    EXPECT_FALSE(mg::parse_iso_utc_ms(text)) << text;
  }
}

TEST(MqttGotoRequest, ParsesAValidRequest)
{
  const auto parsed = mg::parse_goto_request(gotoPayload("abc-123", 1000, 52.5, -6.25));
  ASSERT_TRUE(parsed.request);
  EXPECT_EQ(parsed.request->request_id, "abc-123");
  EXPECT_EQ(parsed.request->issued_at_ms, 1000);
  EXPECT_DOUBLE_EQ(parsed.request->lat, 52.5);
  EXPECT_DOUBLE_EQ(parsed.request->yaw, 1.0);
}

TEST(MqttGotoRequest, RefusesMissingOrInvalidFieldsButKeepsTheId)
{
  auto parsed = mg::parse_goto_request(R"({"request_id":"r1","issued_at":"2026-11-05T17:40:12Z",)"
                                       R"("lat":52.5,"lon":-6.25})");  // no yaw
  EXPECT_FALSE(parsed.request);
  EXPECT_EQ(parsed.request_id, "r1");
  EXPECT_EQ(parsed.error, "invalid_payload");

  parsed = mg::parse_goto_request(gotoPayload("r2", 1000, 95.0, -6.25));  // lat out of range
  EXPECT_FALSE(parsed.request);
  EXPECT_EQ(parsed.request_id, "r2");

  parsed = mg::parse_goto_request(gotoPayload("bad id!", 1000, 52.5, -6.25));
  EXPECT_FALSE(parsed.request);
  EXPECT_TRUE(parsed.request_id.empty());
}

TEST(MqttGotoChecks, FreshnessWindow)
{
  EXPECT_EQ(mg::check_freshness(10000, 10000, 30.0, 5.0), "");
  EXPECT_EQ(mg::check_freshness(10000 - 30000, 10000, 30.0, 5.0), "");
  EXPECT_EQ(mg::check_freshness(10000 - 30001, 10000, 30.0, 5.0), "stale");
  EXPECT_EQ(mg::check_freshness(10000 + 5000, 10000, 30.0, 5.0), "");
  EXPECT_EQ(mg::check_freshness(10000 + 5001, 10000, 30.0, 5.0), "future");
}

TEST(MqttGotoChecks, ClockMustBeSynchronizedAndPrecise)
{
  EXPECT_EQ(mg::check_clock({true, 0.1}, 0.5), "");
  EXPECT_EQ(mg::check_clock({true, 0.6}, 0.5), "clock_unsynced");
  EXPECT_EQ(mg::check_clock({false, 0.0}, 0.5), "clock_unsynced");
}

TEST(MqttGotoStore, SavesAndLoadsAtomically)
{
  const std::string path = tempPath("store_roundtrip");
  std::filesystem::remove(path);
  mg::RequestStore store;
  ASSERT_TRUE(store.load(path)) << "a missing file is an empty store";
  EXPECT_EQ(store.size(), 0U);
  store.put("a", {"SUCCEEDED", 100, 3, "", false, R"({"request_id":"a"})"});
  store.put("b", {"ACTIVE", 200, 2, std::string(32, 'a'), true, R"({"request_id":"b"})"});
  ASSERT_TRUE(store.save(path));
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));

  mg::RequestStore loaded;
  ASSERT_TRUE(loaded.load(path));
  ASSERT_EQ(loaded.size(), 2U);
  EXPECT_EQ(loaded.find("a")->phase, "SUCCEEDED");
  EXPECT_EQ(loaded.find("a")->seq, 3U);
  EXPECT_EQ(loaded.find("b")->last_event, R"({"request_id":"b"})");
  EXPECT_EQ(loaded.find("b")->goal_id, std::string(32, 'a'));
  EXPECT_TRUE(loaded.find("a")->goal_id.empty());
  EXPECT_TRUE(loaded.find("b")->cancel_pending);
  EXPECT_EQ(loaded.cancel_pending_ids(), std::vector<std::string>{"b"});
  EXPECT_EQ(loaded.active_ids(), std::vector<std::string>{"b"});
  std::filesystem::remove(path);
}

TEST(MqttGotoStore, MalformedFileIsRefused)
{
  const std::string path = tempPath("store_malformed");
  {
    std::ofstream out(path);
    out << "v1\tid\tACTIVE\tnot-a-number\t1\t-\t0\t{}\n";
  }
  mg::RequestStore store;
  EXPECT_FALSE(store.load(path));
  EXPECT_EQ(store.size(), 0U);
  std::filesystem::remove(path);
}

TEST(MqttGotoStore, PruneDropsOldTerminalEntriesOnly)
{
  mg::RequestStore store;
  store.put("old", {"SUCCEEDED", 0, 1, "", false, "{}"});
  store.put("old_active", {"ACTIVE", 0, 1, "", false, "{}"});
  store.put("new", {"CANCELED", 9000, 1, "", false, "{}"});
  store.prune(10000, 5000, 10, 0);
  EXPECT_FALSE(store.find("old"));
  EXPECT_TRUE(store.find("old_active"));
  EXPECT_TRUE(store.find("new"));

  for (int i = 0; i < 5; ++i)
  {
    store.put("t" + std::to_string(i), {"REFUSED", 9500 + i, 1, "", false, "{}"});
  }
  store.prune(10000, 5000, 3, 0);
  EXPECT_EQ(store.size(), 3U);
  EXPECT_TRUE(store.find("old_active")) << "a non-terminal entry is never dropped";
  EXPECT_TRUE(store.find("t4"));
}

// A request that could still be re-sent as fresh is never forgotten, whatever
// the ttl or the entry limit: otherwise it could run a second time.
TEST(MqttGotoStore, PruneKeepsRequestsThatCouldStillBeFresh)
{
  mg::RequestStore store;
  for (int i = 0; i < 5; ++i)
  {
    store.put("t" + std::to_string(i), {"REFUSED", 9000 + i, 1, "", false, "{}"});
  }
  store.prune(10000, 100, 2, 35000);
  EXPECT_EQ(store.size(), 5U);
}

TEST(MqttGotoEvent, FinalOnlyInTerminalPhases)
{
  mg::GotoEvent event;
  event.request_id = "r1";
  event.seq = 3;
  event.phase = "ACTIVE";
  event.current = mg::WgsPose{52.5, -6.25, 0.5, true, 0.3};
  event.distance_remaining_m = 4.25;
  event.final_pose = mg::WgsPose{52.5, -6.25, 0.5, true, 0.0};
  event.stamp_ms = 1000;
  std::string json = mg::serialise_goto_event(event);
  EXPECT_NE(json.find("\"final\":null"), std::string::npos) << json;
  EXPECT_NE(json.find("\"distance_remaining_m\":4.25"), std::string::npos) << json;
  ASSERT_TRUE(mg::parse_flat_json_object(json) == std::nullopt) << "nested by design";

  event.phase = "SUCCEEDED";
  event.settled = true;
  json = mg::serialise_goto_event(event);
  EXPECT_NE(json.find("\"settled\":true}"), std::string::npos) << json;
  EXPECT_EQ(json.find("\"final\":null"), std::string::npos) << json;
}

// ---------------------------------------------------------------------------
// mqtt_bridge_node against a fake ~/goto server
// ---------------------------------------------------------------------------

namespace
{

class RecordingMqttClient : public mowgli_monitoring::IMqttClient
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
    return true;
  }
  void disconnect() noexcept override
  {
  }
  bool publish(const std::string& topic, const std::string& payload, bool retain) noexcept override
  {
    publications.push_back({topic, payload, retain});
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
    return true;
  }
  std::vector<Publication> publications;
  std::unordered_map<std::string, MessageCallback> subscriptions;
};

constexpr double kDatumLat = 52.0;
constexpr double kDatumLon = -7.0;

class MqttGotoNodeTest : public ::testing::Test
{
protected:
  using GoalHandle = rclcpp_action::ServerGoalHandle<Goto>;

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

  void SetUp() override
  {
    store_path = tempPath(::testing::UnitTest::GetInstance()->current_test_info()->name());
    std::filesystem::remove(store_path);
    server_node = std::make_shared<rclcpp::Node>("behavior_tree_node");
    server = rclcpp_action::create_server<Goto>(
        server_node,
        "~/goto",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Goto::Goal> goal)
        {
          goals.push_back(*goal);
          return reject_goals ? rclcpp_action::GoalResponse::REJECT
                              : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<GoalHandle> handle)
        {
          ++cancels;
          canceled_ids.push_back(handle->get_goal_id());
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<GoalHandle> handle)
        {
          handles.push_back(handle);
        });
    hlc = server_node->create_service<mowgli_interfaces::srv::HighLevelControl>(
        "~/high_level_control",
        [this](const mowgli_interfaces::srv::HighLevelControl::Request::SharedPtr req,
               mowgli_interfaces::srv::HighLevelControl::Response::SharedPtr resp)
        {
          commands.push_back(req->command);
          resp->success = true;
        });
    odom_pub = server_node->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered_map", 5);
    executor.add_node(server_node);
  }

  void TearDown() override
  {
    if (node)
    {
      executor.remove_node(node);
    }
    executor.remove_node(server_node);
    std::filesystem::remove(store_path);
  }

  void startBridge(double datum_lat = kDatumLat,
                   double datum_lon = kDatumLon,
                   double cancel_window_s = 30.0)
  {
    auto client = std::make_unique<RecordingMqttClient>();
    recording = client.get();
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"mqtt_topic_prefix", "mower"},
                                 {"datum_lat", datum_lat},
                                 {"datum_lon", datum_lon},
                                 {"goto_require_clock_sync", false},
                                 {"goto_store_path", store_path},
                                 {"goto_cancel_window_s", cancel_window_s},
                                 {"publish_rate", 50.0}});
    node = std::make_shared<mowgli_monitoring::MqttBridgeNode>(std::move(client), options);
    executor.add_node(node);
    spinFor(200ms);
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end)
    {
      executor.spin_some(10ms);
    }
  }

  void deliver(const std::string& suffix, const std::string& payload, bool retained = false)
  {
    recording->subscriptions.at("mower/" + suffix)("mower/" + suffix, payload, retained);
    spinFor(300ms);
  }

  std::vector<std::string> events() const
  {
    std::vector<std::string> out;
    for (const auto& p : recording->publications)
    {
      if (p.topic == "mower/goto/event")
      {
        out.push_back(p.payload);
      }
    }
    return out;
  }

  std::vector<std::string> published(const std::string& suffix) const
  {
    std::vector<std::string> out;
    for (const auto& p : recording->publications)
    {
      if (p.topic == "mower/" + suffix)
      {
        out.push_back(p.payload);
      }
    }
    return out;
  }

  static std::string hex(const rclcpp_action::GoalUUID& uuid)
  {
    std::string out;
    char byte[3];
    for (const uint8_t b : uuid)
    {
      std::snprintf(byte, sizeof(byte), "%02x", static_cast<unsigned>(b));
      out += byte;
    }
    return out;
  }

  void finish(std::size_t index, uint16_t code)
  {
    auto result = std::make_shared<Goto::Result>();
    result->error_code = code;
    if (code == Goto::Result::NONE)
    {
      handles.at(index)->succeed(result);
    }
    else
    {
      handles.at(index)->abort(result);
    }
    spinFor(300ms);
  }

  static bool has(const std::string& json, const std::string& fragment)
  {
    return json.find(fragment) != std::string::npos;
  }

  std::string store_path;
  rclcpp::Node::SharedPtr server_node;
  rclcpp_action::Server<Goto>::SharedPtr server;
  std::vector<Goto::Goal> goals;
  std::vector<std::shared_ptr<GoalHandle>> handles;
  int cancels{0};
  std::vector<rclcpp_action::GoalUUID> canceled_ids;
  bool reject_goals{false};
  rclcpp::Service<mowgli_interfaces::srv::HighLevelControl>::SharedPtr hlc;
  std::vector<uint8_t> commands;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  std::shared_ptr<mowgli_monitoring::MqttBridgeNode> node;
  RecordingMqttClient* recording{nullptr};
  rclcpp::executors::SingleThreadedExecutor executor;
};

}  // namespace

TEST_F(MqttGotoNodeTest, ValidRequestBecomesAMapFrameGoalAndSucceeds)
{
  startBridge();
  const double lat = kDatumLat + 0.0001;
  const double lon = kDatumLon + 0.0002;
  deliver("goto", gotoPayload("m1", nowMs(), lat, lon));

  ASSERT_EQ(goals.size(), 1U);
  double x = 0.0;
  double y = 0.0;
  mowgli_interfaces::wgs84::ToEnu(lat, lon, kDatumLat, kDatumLon, x, y);
  EXPECT_NEAR(goals[0].x, x, 1e-6);
  EXPECT_NEAR(goals[0].y, y, 1e-6);
  EXPECT_DOUBLE_EQ(goals[0].yaw, 1.0);
  ASSERT_GE(events().size(), 2U);
  EXPECT_TRUE(has(events()[0], "\"phase\":\"SUBMITTED\"")) << events()[0];
  EXPECT_TRUE(has(events()[1], "\"phase\":\"ACCEPTED\"")) << events()[1];
  EXPECT_TRUE(has(events()[1], "\"durable\":true"));
  EXPECT_TRUE(recording->publications.back().topic == "mower/goto/state" &&
              recording->publications.back().retained);

  auto result = std::make_shared<Goto::Result>();
  result->error_code = Goto::Result::NONE;
  result->final_pose.header.frame_id = "map";
  result->final_pose.pose.position.x = x;
  result->final_pose.pose.position.y = y;
  result->final_pose.pose.orientation.w = 1.0;
  result->final_pose_valid = true;
  result->settled = true;
  ASSERT_EQ(handles.size(), 1U);
  handles[0]->succeed(result);
  spinFor(300ms);

  const std::string last = events().back();
  EXPECT_TRUE(has(last, "\"phase\":\"SUCCEEDED\"")) << last;
  EXPECT_TRUE(has(last, "\"error_code\":\"NONE\"")) << last;
  EXPECT_TRUE(has(last, "\"settled\":true}")) << last;
}

TEST_F(MqttGotoNodeTest, DuplicateRequestIsAnsweredNotResentEvenAfterRestart)
{
  startBridge();
  const std::string payload = gotoPayload("dup", nowMs(), kDatumLat, kDatumLon + 0.0001);
  deliver("goto", payload);
  ASSERT_EQ(goals.size(), 1U);
  auto result = std::make_shared<Goto::Result>();
  result->error_code = Goto::Result::FAILED;
  handles[0]->abort(result);
  spinFor(300ms);

  deliver("goto", payload);
  EXPECT_EQ(goals.size(), 1U) << "a duplicate must not start a second mission";
  EXPECT_TRUE(has(events().back(), "\"error_code\":\"FAILED\""));

  // A new bridge reads the same store: still a duplicate.
  executor.remove_node(node);
  node.reset();
  startBridge();
  deliver("goto", payload);
  EXPECT_EQ(goals.size(), 1U);
  ASSERT_FALSE(events().empty());
  EXPECT_TRUE(has(events().back(), "\"request_id\":\"dup\""));
}

TEST_F(MqttGotoNodeTest, RetainedRequestIsIgnored)
{
  startBridge();
  deliver("goto", gotoPayload("ret", nowMs(), kDatumLat, kDatumLon), /*retained=*/true);
  EXPECT_TRUE(goals.empty());
  EXPECT_TRUE(events().empty());
}

TEST_F(MqttGotoNodeTest, RefusalsSendNoGoal)
{
  startBridge();
  deliver("goto", gotoPayload("stale", nowMs() - 60000, kDatumLat, kDatumLon));
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"stale\""));
  deliver("goto", gotoPayload("future", nowMs() + 60000, kDatumLat, kDatumLon));
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"future\""));
  deliver("goto", gotoPayload("far", nowMs(), kDatumLat + 0.1, kDatumLon));
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"out_of_range\""));
  deliver("goto", R"({"request_id":"bad","lat":52.0})");
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"invalid_payload\""));
  EXPECT_TRUE(has(events().back(), "\"phase\":\"REFUSED\""));
  EXPECT_TRUE(goals.empty());
}

TEST_F(MqttGotoNodeTest, UnsetDatumIsRefused)
{
  startBridge(0.0, 0.0);
  deliver("goto", gotoPayload("nodatum", nowMs(), kDatumLat, kDatumLon));
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"datum_unset\""));
  EXPECT_TRUE(goals.empty());
}

TEST_F(MqttGotoNodeTest, SecondRequestWhileOneIsActiveIsBusy)
{
  startBridge();
  deliver("goto", gotoPayload("first", nowMs(), kDatumLat, kDatumLon + 0.0001));
  deliver("goto", gotoPayload("second", nowMs(), kDatumLat, kDatumLon + 0.0002));
  EXPECT_EQ(goals.size(), 1U);
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"busy\""));
}

TEST_F(MqttGotoNodeTest, CancelStopsTheGoalWithoutAnyFreshnessCheck)
{
  startBridge();
  deliver("goto", gotoPayload("c1", nowMs(), kDatumLat, kDatumLon + 0.0001));
  ASSERT_EQ(handles.size(), 1U);
  deliver("goto/cancel", R"({"request_id":"c1","issued_at":"2000-01-01T00:00:00Z"})");
  EXPECT_EQ(cancels, 1);
  ASSERT_FALSE(published("goto/cancel/result").empty());
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"canceling\""));
  auto result = std::make_shared<Goto::Result>();
  result->error_code = Goto::Result::CANCELED;
  handles[0]->canceled(result);
  spinFor(300ms);
  EXPECT_TRUE(has(events().back(), "\"phase\":\"CANCELED\""));
}

// A cancel only ever acts on its own request: a stale, unknown or invalid one
// must never stop another mission (stopping everything is <prefix>/command 8).
TEST_F(MqttGotoNodeTest, CancelOnlyActsOnItsOwnRequest)
{
  startBridge();
  deliver("goto", gotoPayload("a", nowMs(), kDatumLat, kDatumLon + 0.0001));
  ASSERT_EQ(handles.size(), 1U);
  deliver("goto/cancel", R"({"request_id":"unknown"})");
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"not_found\""));
  deliver("goto/cancel", "garbage");
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"invalid_payload\""));
  deliver("goto/cancel", R"({"request_id":"x"})", /*retained=*/true);
  EXPECT_EQ(cancels, 0);

  finish(0, Goto::Result::FAILED);
  deliver("goto", gotoPayload("b", nowMs(), kDatumLat, kDatumLon + 0.0002));
  ASSERT_EQ(handles.size(), 2U);
  deliver("goto/cancel", R"({"request_id":"a"})");  // the earlier, finished mission
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"already_ended\""));
  EXPECT_EQ(cancels, 0) << "mission b must not be touched";
}

// After a restart, only the goal the previous bridge owned is canceled, by its
// ID: a GOTO another ROS client started keeps running.
TEST_F(MqttGotoNodeTest, RestartCancelsOnlyItsOwnGoal)
{
  auto previous = rclcpp_action::create_client<Goto>(server_node, "behavior_tree_node/goto");
  auto other = rclcpp_action::create_client<Goto>(server_node, "behavior_tree_node/goto");
  ASSERT_TRUE(previous->wait_for_action_server(5s));
  previous->async_send_goal(Goto::Goal{});
  spinFor(300ms);
  other->async_send_goal(Goto::Goal{});
  spinFor(300ms);
  ASSERT_EQ(handles.size(), 2U);
  const auto owned = handles[0]->get_goal_id();
  {
    std::ofstream out(store_path);
    out << "v1\twas-active\tACTIVE\t" << nowMs() << "\t3\t" << hex(owned) << "\t0\t{}\n";
  }

  startBridge();
  spinFor(500ms);
  ASSERT_EQ(canceled_ids.size(), 1U);
  EXPECT_EQ(canceled_ids[0], owned) << "only the bridge's own goal is canceled";
  ASSERT_FALSE(published("goto/cancel/result").empty());
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"canceling\""))
      << "canceling is not stopped yet";
  auto result = std::make_shared<Goto::Result>();
  result->error_code = Goto::Result::CANCELED;
  handles[0]->canceled(result);
  spinFor(500ms);
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"stopped\""))
      << published("goto/cancel/result").back();
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"status\":\"CANCELED\""));
  mg::RequestStore after;
  ASSERT_TRUE(after.load(store_path));
  EXPECT_FALSE(after.find("was-active")->cancel_pending);
  mg::RequestStore store;
  ASSERT_TRUE(store.load(store_path));
  ASSERT_TRUE(store.find("was-active"));
  EXPECT_EQ(store.find("was-active")->phase, "ABORTED");
  EXPECT_TRUE(has(store.find("was-active")->last_event, "bridge_restarted"));
  ASSERT_FALSE(events().empty()) << "the end of the request is published";
  EXPECT_TRUE(has(events().back(), "\"seq\":4")) << events().back();
  EXPECT_TRUE(has(events().back(), "\"error_code\":\"LOST\""));
}

TEST_F(MqttGotoNodeTest, UnreadableStoreRefusesEveryRequest)
{
  {
    std::ofstream out(store_path);
    out << "garbage line\n";
  }
  startBridge();
  deliver("goto", gotoPayload("s1", nowMs(), kDatumLat, kDatumLon));
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"state_unavailable\""));
  EXPECT_TRUE(goals.empty());
}

TEST_F(MqttGotoNodeTest, RejectedGoalEndsRefused)
{
  reject_goals = true;
  startBridge();
  deliver("goto", gotoPayload("rej", nowMs(), kDatumLat, kDatumLon + 0.0001));
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"goal_rejected\"")) << events().back();
  // Not busy afterwards.
  reject_goals = false;
  deliver("goto", gotoPayload("next", nowMs(), kDatumLat, kDatumLon + 0.0001));
  EXPECT_EQ(goals.size(), 2U);
}

TEST_F(MqttGotoNodeTest, CancelBeforeTheGoalResponseIsApplied)
{
  startBridge();
  auto& on_goto = recording->subscriptions.at("mower/goto");
  auto& on_cancel = recording->subscriptions.at("mower/goto/cancel");
  on_goto("mower/goto", gotoPayload("early", nowMs(), kDatumLat, kDatumLon + 0.0001), false);
  on_cancel("mower/goto/cancel", R"({"request_id":"early"})", false);
  spinFor(500ms);
  ASSERT_EQ(goals.size(), 1U);
  EXPECT_GE(cancels, 1) << "the cancel must reach the goal once it is known";
}

TEST_F(MqttGotoNodeTest, UnsavableRequestIsRefusedBeforeAnyGoal)
{
  startBridge();
  std::filesystem::remove(store_path);
  std::filesystem::create_directory(store_path);  // the rename over it now fails
  deliver("goto", gotoPayload("nosave", nowMs(), kDatumLat, kDatumLon + 0.0001));
  EXPECT_TRUE(goals.empty());
  ASSERT_EQ(events().size(), 1U) << "no SUBMITTED for a request that was not saved";
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"state_unavailable\"")) << events().back();
  // The same request_id again is answered with that refusal.
  deliver("goto", gotoPayload("nosave", nowMs(), kDatumLat, kDatumLon + 0.0001));
  EXPECT_TRUE(goals.empty());
  ASSERT_EQ(events().size(), 2U);
  EXPECT_TRUE(has(events().back(), "\"phase\":\"REFUSED\"")) << events().back();
  std::filesystem::remove_all(store_path);
}

TEST_F(MqttGotoNodeTest, AnUnsavedEndIsStillTheAnswerToADuplicate)
{
  startBridge();
  deliver("goto", gotoPayload("late", nowMs(), kDatumLat, kDatumLon + 0.0001));
  ASSERT_EQ(handles.size(), 1U);
  std::filesystem::remove(store_path);
  std::filesystem::create_directory(store_path);  // later saves fail
  finish(0, Goto::Result::NONE);
  EXPECT_TRUE(has(events().back(), "\"phase\":\"SUCCEEDED\"")) << events().back();
  EXPECT_TRUE(has(events().back(), "\"durable\":false")) << events().back();
  deliver("goto", gotoPayload("late", nowMs(), kDatumLat, kDatumLon + 0.0001));
  EXPECT_EQ(handles.size(), 1U);
  EXPECT_TRUE(has(events().back(), "\"phase\":\"SUCCEEDED\"")) << events().back();
  std::filesystem::remove_all(store_path);
}

TEST_F(MqttGotoNodeTest, LostActionServerEndsTheRequest)
{
  startBridge();
  deliver("goto", gotoPayload("lost", nowMs(), kDatumLat, kDatumLon + 0.0001));
  ASSERT_EQ(goals.size(), 1U);
  handles.clear();
  server.reset();
  spinFor(1500ms);
  // Either the server's own abort arrived (no_result) or the bridge noticed it
  // was gone (action_lost): both end the request LOST.
  EXPECT_TRUE(has(events().back(), "\"error_code\":\"LOST\"")) << events().back();
  EXPECT_TRUE(has(events().back(), "\"phase\":\"ABORTED\""));
}

// The goal ID is chosen and saved before the goal leaves: a bridge that dies
// after sending the goal but before its response still has the ID, and the
// next run cancels that goal — and only that one.
TEST_F(MqttGotoNodeTest, CrashBeforeTheGoalResponseStillCancelsThatGoal)
{
  startBridge();
  recording->subscriptions.at("mower/goto")(
      "mower/goto", gotoPayload("crash", nowMs(), kDatumLat, kDatumLon + 0.0001), false);
  // The bridge stops being serviced right after sending the goal, so it never
  // processes the response, then dies.
  executor.remove_node(node);
  spinFor(300ms);
  node.reset();
  ASSERT_EQ(handles.size(), 1U) << "the server accepted and runs the goal";
  mg::RequestStore store;
  ASSERT_TRUE(store.load(store_path));
  ASSERT_TRUE(store.find("crash"));
  EXPECT_EQ(store.find("crash")->phase, "SUBMITTED");
  EXPECT_EQ(store.find("crash")->goal_id, hex(handles[0]->get_goal_id()));

  auto other = rclcpp_action::create_client<Goto>(server_node, "behavior_tree_node/goto");
  other->async_send_goal(Goto::Goal{});
  spinFor(300ms);
  ASSERT_EQ(handles.size(), 2U);

  startBridge();
  spinFor(500ms);
  ASSERT_EQ(canceled_ids.size(), 1U);
  EXPECT_EQ(canceled_ids[0], handles[0]->get_goal_id());
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"bridge_restarted\"")) << events().back();
}

// A goal the server never shows during the whole watch window ends the pending
// cancel as not_seen — not before, since an "unknown" answer proves nothing.
TEST_F(MqttGotoNodeTest, PendingCancelOfAnUnseenGoalEndsNotSeenAfterTheWindow)
{
  {
    std::ofstream out(store_path);
    out << "v1\tgone\tACTIVE\t" << nowMs() << "\t2\t" << std::string(32, '7') << "\t0\t{}\n";
  }
  startBridge(kDatumLat, kDatumLon, /*cancel_window_s=*/1.5);
  spinFor(500ms);
  EXPECT_TRUE(published("goto/cancel/result").empty()) << "no verdict while the window is open";
  spinFor(1500ms);
  ASSERT_FALSE(published("goto/cancel/result").empty());
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"not_seen\""));
  mg::RequestStore after;
  ASSERT_TRUE(after.load(store_path));
  EXPECT_FALSE(after.find("gone")->cancel_pending);
}

// The race the window exists for: the cancel reaches the server before the
// goal request does (the server answers "unknown"), then the goal is
// accepted. The bridge sees it on the action status and cancels it.
TEST_F(MqttGotoNodeTest, CancelArrivingBeforeTheGoalStillStopsIt)
{
  const auto late_id = mg::new_goal_id();
  {
    std::ofstream out(store_path);
    out << "v1\tlate\tSUBMITTED\t" << nowMs() << "\t1\t" << mg::goal_id_to_hex(late_id)
        << "\t0\t{}\n";
  }
  startBridge(kDatumLat, kDatumLon, /*cancel_window_s=*/10.0);
  spinFor(500ms);
  EXPECT_EQ(cancels, 0) << "the server did not know the goal yet";

  auto late =
      mowgli_monitoring::PresetIdGotoClient::create(*server_node, "behavior_tree_node/goto");
  ASSERT_TRUE(late->wait_for_action_server(5s));
  late->set_next_goal_id(late_id);
  late->async_send_goal(Goto::Goal{});
  spinFor(2500ms);
  ASSERT_EQ(handles.size(), 1U);
  ASSERT_EQ(canceled_ids.size(), 1U) << "the goal that arrived late is canceled";
  EXPECT_EQ(canceled_ids[0], late_id);
  auto result = std::make_shared<Goto::Result>();
  result->error_code = Goto::Result::CANCELED;
  handles[0]->canceled(result);
  spinFor(500ms);
  EXPECT_TRUE(has(published("goto/cancel/result").back(), "\"result\":\"stopped\""))
      << published("goto/cancel/result").back();
}

// PresetIdGotoClient relies on rclcpp_action drawing the goal ID from the
// virtual generate_goal_id(): the server must see exactly the chosen ID.
TEST_F(MqttGotoNodeTest, PresetIdClientSendsTheChosenId)
{
  auto client =
      mowgli_monitoring::PresetIdGotoClient::create(*server_node, "behavior_tree_node/goto");
  ASSERT_TRUE(client->wait_for_action_server(5s));
  const auto chosen = mg::new_goal_id();
  client->set_next_goal_id(chosen);
  client->async_send_goal(Goto::Goal{});
  client->async_send_goal(Goto::Goal{});  // the next one draws a fresh ID again
  spinFor(500ms);
  ASSERT_EQ(handles.size(), 2U);
  EXPECT_EQ(handles[0]->get_goal_id(), chosen);
  EXPECT_NE(handles[1]->get_goal_id(), chosen);
}

// A terminal result saved but never published (crash in between) is recovered
// by sending the same request again: answered, never run again.
TEST_F(MqttGotoNodeTest, SavedResultIsRecoveredBySendingTheRequestAgain)
{
  const std::string payload = gotoPayload("done", nowMs(), kDatumLat, kDatumLon + 0.0001);
  {
    std::ofstream out(store_path);
    out << "v1\tdone\tSUCCEEDED\t" << nowMs() << "\t4\t" << std::string(32, 'b')
        << "\t0\t{\"request_id\":\"done\",\"phase\":\"SUCCEEDED\"}\n";
  }
  startBridge();
  deliver("goto", payload);
  EXPECT_TRUE(goals.empty());
  ASSERT_FALSE(events().empty());
  EXPECT_TRUE(has(events().back(), "\"phase\":\"SUCCEEDED\""));
}

// A result the store cannot save is still reported, marked not durable, and
// no further mission is accepted until the store is writable again.
TEST_F(MqttGotoNodeTest, UnsavableResultIsReportedNotDurable)
{
  startBridge();
  deliver("goto", gotoPayload("r1", nowMs(), kDatumLat, kDatumLon + 0.0001));
  ASSERT_EQ(handles.size(), 1U);
  std::filesystem::remove(store_path);
  std::filesystem::create_directory(store_path);
  finish(0, Goto::Result::NONE);
  EXPECT_TRUE(has(events().back(), "\"phase\":\"SUCCEEDED\"")) << events().back();
  EXPECT_TRUE(has(events().back(), "\"durable\":false"));
  deliver("goto", gotoPayload("r2", nowMs(), kDatumLat, kDatumLon + 0.0001));
  EXPECT_EQ(goals.size(), 1U);
  EXPECT_TRUE(has(events().back(), "\"error_msg\":\"state_unavailable\""));
  std::filesystem::remove_all(store_path);
}

// <prefix>/command 8 (STOP) is never held back by the GOTO machinery: not by an
// unreadable store, not by a missing action server.
TEST_F(MqttGotoNodeTest, StopCommandPassesWhateverTheGotoState)
{
  {
    std::ofstream out(store_path);
    out << "garbage line\n";
  }
  server.reset();
  startBridge();
  deliver("command", "8");
  ASSERT_FALSE(commands.empty());
  EXPECT_EQ(commands.back(), 8);
}

// pose_wgs84 judges validity when it publishes, from the pose's own age.
TEST_F(MqttGotoNodeTest, StalePoseIsPublishedInvalid)
{
  startBridge();
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.orientation.w = 1.0;
  odom.header.stamp = rclcpp::Time(server_node->now().nanoseconds() - 10'000'000'000LL);
  odom_pub->publish(odom);
  spinFor(300ms);
  ASSERT_FALSE(published("pose_wgs84").empty());
  EXPECT_TRUE(has(published("pose_wgs84").back(), "\"valid\":false"));

  odom.header.stamp = server_node->now();
  odom.pose.pose.position.x = 5.0;  // past the deadband
  odom_pub->publish(odom);
  spinFor(300ms);
  EXPECT_TRUE(has(published("pose_wgs84").back(), "\"valid\":true"))
      << published("pose_wgs84").back();
}

TEST(MqttGotoPoseWgs84, ConvertsTheFusedPoseWithTheDatum)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp.sec = 1762364412;
  odom.pose.pose.position.x = 10.0;
  odom.pose.pose.position.y = 20.0;
  odom.pose.pose.orientation.w = 1.0;
  const std::string json = mowgli_monitoring::MqttBridgeNode::serialise_pose_wgs84(
      odom, kDatumLat, kDatumLon, 0.25, true);
  double lat = 0.0;
  double lon = 0.0;
  mowgli_interfaces::wgs84::FromEnu(10.0, 20.0, kDatumLat, kDatumLon, lat, lon);
  char expected[64];
  std::snprintf(expected, sizeof(expected), "\"lat\":%.8f,\"lon\":%.8f", lat, lon);
  EXPECT_NE(json.find(expected), std::string::npos) << json;
  EXPECT_NE(json.find("\"valid\":true"), std::string::npos);
  EXPECT_NE(json.find("\"datum\":{\"lat\":52.00000000,\"lon\":-7.00000000}"), std::string::npos);
}
