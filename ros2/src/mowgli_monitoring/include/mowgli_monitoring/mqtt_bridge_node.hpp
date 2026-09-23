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
 * @file mqtt_bridge_node.hpp
 * @brief MqttBridgeNode: bridges key ROS2 topics to MQTT for external
 *        monitoring (mobile app, Home Assistant, etc.).
 *
 * Architecture
 * ------------
 * The node owns an `IMqttClient` interface.  At startup it tries to construct
 * a `MosquittoMqttClient` (requires libmosquitto at link time).  If the build
 * was performed without mosquitto the `StubMqttClient` is used instead, which
 * simply logs every publish/subscribe call at DEBUG level.
 *
 * ROS2 → MQTT
 *   /hardware_bridge/status              → <prefix>/status             (JSON)
 *   /hardware_bridge/power               → <prefix>/power              (JSON)
 *   /hardware_bridge/emergency           → <prefix>/emergency          (JSON)
 *   /wheel_odom                          → <prefix>/position   (JSON: x, y, theta, odom frame) —
 * rate-limited /diagnostics                         → <prefix>/diagnostics        (JSON summary)
 *   /behavior_tree_node/high_level_status → <prefix>/high_level_status (JSON) — retained;
 *                                           subscription watchdog below (mowglinext#644)
 *   /gps/fix                             → <prefix>/gps        (JSON: lat/lon/alt) — rate-limited
 *   /odometry/filtered_map               → <prefix>/pose       (JSON: x, y, yaw, MAP frame) —
 *                                           the fused localizer pose, rate-limited
 *   /gps/status                          → <prefix>/rtk_status (JSON) — retained; the SAME
 *                                           mowgli_interfaces/msg/GnssStatus + gnss_status_utils
 *                                           helpers the LED ring and behavior tree use, so this
 *                                           can never disagree with what the robot's own ring or
 *                                           GUI "GPS %" badge shows.
 *   /map_server_node/get_mowing_area     → <prefix>/area_boundary (JSON) — retained; polled every
 *                                           ~10s (piggybacks on the existing on_timer() tick, no
 *                                           separate timer), republished only when it actually
 *                                           changed. Datum + map-frame (metres, X=east/Y=north)
 *                                           polygon geometry for every mowing area, so an external
 *                                           consumer can render a boundary/obstacle overview
 *                                           without needing the GUI's own map stack. Independent
 *                                           of <prefix>/areas' own index/name polling (mowglinext
 *                                           PR #638) — the two poll the same service separately,
 *                                           on their own timers/clients; consolidating them into
 *                                           one poll loop is a natural follow-up, not done here.
 *   (connection state)                   → <prefix>/available  ("online"/"offline", retained, LWT)
 *   (periodic poll, ~10s)                → <prefix>/areas      (JSON array of {index,name}) —
 *                                           retained; walks map_server_node's GetMowingArea
 *                                           index-by-index (same pattern the GUI backend's
 *                                           pollMap() uses). Navigation-only areas are excluded —
 *                                           "index" is the raw map_server_node area index, the
 *                                           SAME index space <prefix>/start_area and
 *                                           GetMowingArea/StartInArea use.
 *
 *                                           INTERIM CONTRACT, index-based on purpose: areas have
 *                                           no stable id today (mowglinext#637 tracks adding one).
 *                                           "index" is purely positional and the GUI's own
 *                                           edit/delete flow rebuilds the whole area list on any
 *                                           single-area change, which can reassign every index —
 *                                           so a client MUST re-fetch <prefix>/areas and re-resolve
 *                                           by name rather than caching an index across a session.
 *                                           Once #637 lands, <prefix>/areas is expected to gain a
 *                                           stable "id" field and <prefix>/start_area an id-based
 *                                           counterpart — this index-only shape is a stepping
 *                                           stone, not the final contract.
 *
 * MQTT → ROS2
 *   <prefix>/command    → /behavior_tree_node/high_level_control service call
 *                          (fresh, non-retained payload: ASCII decimal uint8, e.g. "1" — not a
 *                          raw byte)
 *   <prefix>/start_area → /behavior_tree_node/start_in_area service call — start mowing the given
 *                          area now, ahead of the normal area-iteration order (fresh, non-retained
 *                          payload: ASCII decimal uint8 area index, same index space as
 *                          <prefix>/areas above)
 *
 * See docs/MQTT_CONTROL.md for the full JSON schema of every topic above.
 *
 * <prefix>/high_level_status subscription watchdog (mowglinext#644)
 * -------------------------------------------------------------------------
 * Observed on a real deployment: this node's subscription to
 * /behavior_tree_node/high_level_status can go stale for extended periods
 * (30+ minutes seen in the field) — this node stays connected to the broker
 * ("online"/LWT) and alive the whole time, and behavior_tree_node's own
 * publish is fresh (confirmed via a brand-new `ros2 topic echo` subscriber
 * getting live data at the same moment) — yet <prefix>/high_level_status
 * keeps republishing old data. A full host reboot always clears it. A
 * 2026-09-21 capture of ROS and the broker side by side found a different
 * cause for at least that occurrence: this node's own outgoing MQTT queue.
 * The network loop was driven only from the publish_rate timer, so
 * <prefix>/high_level_status left at ~0.57 msg/s against ~1 msg/s produced
 * and lagged more and more (9+ min after 16 min); net_timer_ now drives it at
 * 20 Hz. A stuck DDS reader is still possible, so the watchdog below stays
 * (see is_high_level_status_stale()).
 * Since behavior_tree_node republishes this topic unconditionally at least
 * once a second regardless of state, on_timer() recreates JUST this one
 * subscription (create_high_level_status_subscription()) whenever more than
 * kHighLevelStatusStaleAfterS passes with nothing received — a local,
 * in-process recovery that needs no restart of this node, the ROS2 stack,
 * or the host, and is safe even mid-mow (this package has no motion/blade
 * authority, root CLAUDE.md Safety).
 *
 * Parameters
 * ----------
 * mqtt_host          string  "localhost"  — overridden from mowgli_robot.yaml (Invariant 15 /
 * mqtt_port          int     1883            GUI Settings → MQTT) by full_system.launch.py;
 * mqtt_username      string  ""              these package-share defaults only apply when the
 * mqtt_password      string  ""              node is run standalone (e.g. in tests).
 * mqtt_client_id     string  "mowgli_ros2"
 * mqtt_topic_prefix  string  "mowgli"
 * home_assistant_discovery_enabled bool false — publish
 * retained Home Assistant device discovery
 * publish_rate       double  1.0   Hz — max rate of
 * position/gps/status/power/rtk_status use_ssl            bool    false datum_lat          double
 * 0.0   — injected from mowgli_robot.yaml by full_system.launch.py, datum_lon          double  0.0
 * same as map_server_node/navsat_to_absolute_pose_node; used only to label <prefix>/area_boundary's
 * map-frame geometry with the WGS84 origin it's relative to.
 */

#ifndef MOWGLI_MONITORING__MQTT_BRIDGE_NODE_HPP_
#define MOWGLI_MONITORING__MQTT_BRIDGE_NODE_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "mowgli_interfaces/gnss_status_utils.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/map_area.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/srv/start_in_area.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_monitoring
{

// ---------------------------------------------------------------------------
// IMqttClient — pure interface
// ---------------------------------------------------------------------------

/**
 * @brief Minimal MQTT client interface.
 *
 * Implementations provide publish/subscribe/connect/disconnect operations.
 * All methods are noexcept to keep the bridge node simple; implementations
 * must handle failures internally (e.g. log and return false).
 */
class IMqttClient
{
public:
  /// `retained` is true when the broker delivered its stored last value rather
  /// than a newly published message. Consumers of control topics must treat
  /// that as stale intent.
  using MessageCallback =
      std::function<void(const std::string& topic, const std::string& payload, bool retained)>;

  virtual ~IMqttClient() = default;

  /**
   * @brief Connect to the broker.
   * @return true on success.
   */
  virtual bool connect() noexcept = 0;

  /**
   * @brief Disconnect from the broker gracefully.
   */
  virtual void disconnect() noexcept = 0;

  /**
   * @brief Publish a message.
   * @param topic   Full MQTT topic string.
   * @param payload UTF-8 payload (typically JSON).
   * @param retain  Whether the broker should retain the last message.
   * @return true if the message was accepted for delivery.
   */
  virtual bool publish(const std::string& topic,
                       const std::string& payload,
                       bool retain = false) noexcept = 0;

  /**
   * @brief Subscribe to a topic pattern.
   * @param topic    MQTT topic filter (may include wildcards + and #).
   * @param callback Invoked on each received message, including whether the
   *        broker marked that delivery as retained.
   * @return true if the subscription was accepted.
   */
  virtual bool subscribe(const std::string& topic, MessageCallback callback) noexcept = 0;

  /**
   * @brief Drive the client network loop (call regularly).
   *
   * Implementations that maintain an internal event loop (e.g. libmosquitto
   * in synchronous mode) should call their loop function here.
   */
  virtual void spin_once() noexcept = 0;

  /// @return true when currently connected to the broker.
  virtual bool is_connected() const noexcept = 0;
};

// ---------------------------------------------------------------------------
// StubMqttClient — no-op / logging implementation
// ---------------------------------------------------------------------------

/**
 * @brief Stub MQTT client that logs all operations instead of performing them.
 *
 * Used when libmosquitto is not available at build time, or in unit tests.
 */
class StubMqttClient : public IMqttClient
{
public:
  explicit StubMqttClient(rclcpp::Logger logger);

  bool connect() noexcept override;
  void disconnect() noexcept override;
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retain = false) noexcept override;
  bool subscribe(const std::string& topic, MessageCallback callback) noexcept override;
  void spin_once() noexcept override;
  bool is_connected() const noexcept override;

private:
  rclcpp::Logger logger_;
  bool connected_{false};
};

// ---------------------------------------------------------------------------
// MosquittoMqttClient — libmosquitto implementation
// ---------------------------------------------------------------------------

#ifdef MOWGLI_HAS_MOSQUITTO

/**
 * @brief libmosquitto-backed MQTT client.
 *
 * Only compiled when the build system detects libmosquitto
 * (MOWGLI_HAS_MOSQUITTO is set by CMakeLists.txt via find_library).
 */
class MosquittoMqttClient : public IMqttClient
{
public:
  struct Config
  {
    std::string host{"localhost"};
    int port{1883};
    std::string username{};
    std::string password{};
    std::string client_id{"mowgli_ros2"};
    bool use_ssl{false};
    /// Full topic (e.g. "mowgli/available") for the LWT + explicit online/
    /// offline publishes. Empty disables the availability feature entirely.
    std::string availability_topic{};
  };

  explicit MosquittoMqttClient(Config config, rclcpp::Logger logger);
  ~MosquittoMqttClient() override;

  bool connect() noexcept override;
  void disconnect() noexcept override;
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retain = false) noexcept override;
  bool subscribe(const std::string& topic, MessageCallback callback) noexcept override;
  void spin_once() noexcept override;
  bool is_connected() const noexcept override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif  // MOWGLI_HAS_MOSQUITTO

// ---------------------------------------------------------------------------
// MqttBridgeNode
// ---------------------------------------------------------------------------

/**
 * @brief Bridges selected ROS2 topics to/from an MQTT broker.
 *
 * The node accepts an externally created IMqttClient for testability.
 * When constructed without one the factory function `make_default_client()`
 * selects MosquittoMqttClient or StubMqttClient based on build configuration.
 */
class MqttBridgeNode : public rclcpp::Node
{
public:
  /**
   * @brief Primary constructor — creates the MQTT client internally.
   */
  explicit MqttBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Constructor for tests — accepts an externally provided client.
   */
  MqttBridgeNode(std::unique_ptr<IMqttClient> client,
                 const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // Default destructor is sufficient: destroying mqtt_client_ (a
  // MosquittoMqttClient) runs its own destructor, which calls disconnect()
  // — that already publishes the retained "offline" availability payload
  // before actually dropping the connection (see mqtt_bridge_node.cpp).
  ~MqttBridgeNode() override = default;

  // Exposed for testing.
  const std::string& topic_prefix() const
  {
    return topic_prefix_;
  }

  // ---- JSON serialisers ------------------------------------------------
  // Public (pure, static) so gtest can exercise the exact JSON shape without
  // friend-class machinery — same "exposed for testing" convention as
  // DiagnosticsNode's check_*() methods (diagnostics_node.hpp).

  static std::string serialise_status(const mowgli_interfaces::msg::Status& msg);
  static std::string serialise_power(const mowgli_interfaces::msg::Power& msg);
  static std::string serialise_emergency(const mowgli_interfaces::msg::Emergency& msg);
  static std::string serialise_position(const nav_msgs::msg::Odometry& msg);
  /// Map-frame pose {x, y, yaw} from the fused localizer (/odometry/filtered_map).
  static std::string serialise_pose(const nav_msgs::msg::Odometry& msg);
  static std::string serialise_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& msg);
  static std::string serialise_high_level_status(
      const mowgli_interfaces::msg::HighLevelStatus& msg);
  static std::string serialise_gps(const sensor_msgs::msg::NavSatFix& msg);
  static std::string serialise_rtk_status(const mowgli_interfaces::msg::GnssStatus& msg);

  /// Home Assistant MQTT device-discovery helpers. The device id is derived
  /// from the topic prefix, so separate mowers on one broker must use separate
  /// mqtt_topic_prefix values.
  static std::string home_assistant_device_id(const std::string& topic_prefix);
  static std::string home_assistant_discovery_topic(const std::string& topic_prefix);
  static std::string serialise_home_assistant_discovery(const std::string& topic_prefix);

  /**
   * @brief One mowing area's MQTT-relevant summary.
   *
   * `index` is the raw map_server_node area index — the same positional
   * index space GetMowingArea/StartInArea already use, NOT a stable id
   * (see the file-level doc comment above and mowglinext#637). Navigation-
   * only areas are never represented here — see serialise_areas().
   */
  struct AreaSummary
  {
    uint32_t index{0};
    std::string name{};
  };

  static std::string serialise_home_assistant_discovery(const std::string& topic_prefix,
                                                        const std::vector<AreaSummary>& areas);
  static std::string serialise_areas(const std::vector<AreaSummary>& areas);

  /// Charging dock pose in the map frame (metres, radians), as configured in mowgli_robot.yaml.
  struct DockPose
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  /**
   * @brief The dock pose to publish, or nullopt when there is none to show.
   *
   * mowgli_robot.yaml's dock_pose_x/y/yaw default to 0/0/0 on a robot whose dock has
   * not been calibrated; a real calibrated yaw is never exactly 0.0, so all three
   * being zero (or any being non-finite) means "not set" and nothing is published.
   */
  static std::optional<DockPose> make_dock_pose(double x, double y, double yaw);

  /**
   * @brief Build the <prefix>/area_boundary payload from a polled area list.
   * @param areas (index, MapArea) pairs, in whatever order they were polled —
   *        NOT necessarily sorted or contiguous (an area can be deleted,
   *        leaving gaps; see docs/MQTT_CONTROL.md's index-staleness caveat).
   *        Navigation-only areas (MapArea::is_navigation_area) must already
   *        be filtered out by the caller, matching <prefix>/areas' own
   *        exclusion (mowglinext PR #638).
   * @param datum_lat / datum_lon WGS84 origin the polygon points (map-frame
   *        metres, X=east/Y=north) are relative to.
   * @param dock Charging dock pose in the same frame; adds a "dock" object when set.
   */
  static std::string serialise_area_boundaries(
      const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>& areas,
      double datum_lat,
      double datum_lon,
      const std::optional<DockPose>& dock = std::nullopt);

  /// Escape a raw string so it is safe inside a JSON string literal.
  static std::string json_escape(const std::string& raw);

  /**
   * @brief Parse an MQTT command payload into a HighLevelControl command code.
   *
   * Expected payload: a single ASCII decimal integer, e.g. "1" for
   * COMMAND_START — NOT a raw byte. The entire payload must be digits in
   * [0, 255]; leading/trailing whitespace, signs, and trailing characters
   * are deliberately rejected.
   * @return true and sets out_command on a valid uint8 payload; false
   *         (out_command left unchanged) otherwise.
   */
  static bool parse_command_payload(const std::string& payload, uint8_t& out_command);

  /// Retained control messages are broker state, not fresh operator intent.
  /// This restriction applies only to inbound control topics; retained
  /// publishing for status and telemetry remains valid.
  static bool is_fresh_control_message(bool retained)
  {
    return !retained;
  }

  /**
   * @brief True if the <prefix>/high_level_status subscription looks stuck
   *        and should be recreated (mowglinext#644).
   *
   * behavior_tree_node republishes this topic unconditionally at least once
   * a second — its own heartbeat timer, see behavior_tree_node.cpp's
   * setupHighLevelStatusRepublish() — so once at least one message has ever
   * arrived, `threshold_s` of subsequent silence is strong evidence that
   * THIS subscription has gone stale (observed in the field: this node
   * stays connected/"online" and alive throughout, while a brand-new
   * subscriber to the same ROS topic gets live data immediately), not that
   * the publisher stopped.
   * @param received_before Has any high_level_status message EVER arrived
   *        on this subscription? False before the first one is normal
   *        startup (behavior_tree_node may not be up yet) and must never be
   *        treated as staleness.
   */
  static bool is_high_level_status_stale(bool received_before,
                                         const rclcpp::Time& now,
                                         const rclcpp::Time& last_received,
                                         double threshold_s);

  /**
   * @brief True if a rate-limited topic may publish its pending message now.
   * @param last_publish Time of the topic's previous publish; the epoch (never
   *        published) is always due for any sane interval.
   */
  static bool is_publish_due(const rclcpp::Time& now,
                             const rclcpp::Time& last_publish,
                             double min_interval_s);

private:
  // ---- Initialisation -------------------------------------------------------

  void declare_parameters();
  void create_mqtt_client();
  void create_subscriptions();
  /// (Re)create just the <prefix>/high_level_status subscription — used at
  /// startup (create_subscriptions()) and by the staleness watchdog in
  /// on_timer() to recover without restarting this node or the stack.
  void create_high_level_status_subscription();
  void create_service_client();
  void create_timer();

  // ---- Area boundary polling (piggybacks on on_timer(), ~every 10s) --------

  /// Kick off a fresh index-0..N poll chain, if one isn't already running.
  void maybe_poll_area_boundaries();
  /// Request GetMowingArea for `index`, then chain to `index + 1` on success.
  void poll_area_boundary_step(
      uint32_t index,
      std::shared_ptr<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>>
          accumulated);
  /// Serialise + publish (retained) `accumulated`, but only if it differs
  /// from the last payload actually sent — <prefix>/area_boundary is meant
  /// to be a quiet, retained topic, not a ~10s heartbeat.
  void finish_area_boundary_poll(
      std::shared_ptr<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>>
          accumulated);

  // ---- ROS2 subscription callbacks -----------------------------------------

  void on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
  void on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg);
  void on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_diagnostics(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);
  void on_high_level_status(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg);
  void on_gps_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void on_gnss_status(mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg);
  void on_pose(nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // ---- MQTT command callback ------------------------------------------------

  void on_mqtt_command(const std::string& topic, const std::string& payload, bool retained);
  void on_mqtt_start_area(const std::string& topic, const std::string& payload, bool retained);
  void on_home_assistant_status(const std::string& topic,
                                const std::string& payload,
                                bool retained);
  bool publish_home_assistant_discovery();

  // ---- Area list: periodic poll of GetMowingArea + publish ------------------

  void poll_areas();
  void poll_areas_step(uint32_t index, std::shared_ptr<std::vector<AreaSummary>> collected);
  void publish_areas_if_changed(const std::vector<AreaSummary>& areas);

  // ---- Timers: rate-limited publishes (on_timer) + network loop (net_timer_) -----

  void on_timer();

  // ---- Helpers --------------------------------------------------------------

  /// Construct the full MQTT topic: "<prefix>/<suffix>".
  std::string full_topic(const std::string& suffix) const;

  // ---- MQTT client ----------------------------------------------------------

  std::unique_ptr<IMqttClient> mqtt_client_;

  // ---- ROS2 interfaces ------------------------------------------------------

  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr sub_power_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr sub_emergency_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr sub_high_level_status_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
  rclcpp::Subscription<mowgli_interfaces::msg::GnssStatus>::SharedPtr sub_gnss_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose_;

  rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedPtr srv_high_level_;
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr srv_get_area_;
  rclcpp::Client<mowgli_interfaces::srv::StartInArea>::SharedPtr srv_start_area_;
  // Separate client (same service as srv_get_area_ above) so the
  // <prefix>/area_boundary poll chain (below) and the <prefix>/areas poll
  // chain (PR #638) never share in-flight request/response state.
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr srv_get_mowing_area_;

  rclcpp::TimerBase::SharedPtr timer_;
  // Drives IMqttClient::spin_once() on its own fast cadence, independent of publish_rate_.
  rclcpp::TimerBase::SharedPtr net_timer_;
  static constexpr int kNetworkLoopPeriodMs = 50;

  // ---- Parameters -----------------------------------------------------------

  std::string mqtt_host_{"localhost"};
  int mqtt_port_{1883};
  std::string mqtt_username_{};
  std::string mqtt_password_{};
  std::string mqtt_client_id_{"mowgli_ros2"};
  std::string topic_prefix_{"mowgli"};
  double publish_rate_{1.0};
  bool use_ssl_{false};
  bool home_assistant_discovery_enabled_{false};
  double datum_lat_{0.0};
  double datum_lon_{0.0};
  double dock_pose_x_{0.0};
  double dock_pose_y_{0.0};
  double dock_pose_yaw_{0.0};

  // Tracks MQTT connection edges so discovery is refreshed after reconnect.
  bool mqtt_was_connected_{false};
  // Set by MQTT callbacks and consumed by on_timer() after spin_once() has
  // fully returned. This avoids publishing from within the MQTT receive path.
  bool home_assistant_discovery_publish_pending_{false};

  // ---- Rate-limiting state --------------------------------------------------

  std::optional<nav_msgs::msg::Odometry> pending_odom_{};
  rclcpp::Time last_odom_publish_{0, 0, RCL_ROS_TIME};
  std::optional<sensor_msgs::msg::NavSatFix> pending_gps_{};
  rclcpp::Time last_gps_publish_{0, 0, RCL_ROS_TIME};
  std::optional<mowgli_interfaces::msg::Status> pending_status_{};
  rclcpp::Time last_status_publish_{0, 0, RCL_ROS_TIME};
  std::optional<mowgli_interfaces::msg::Power> pending_power_{};
  rclcpp::Time last_power_publish_{0, 0, RCL_ROS_TIME};
  std::optional<mowgli_interfaces::msg::GnssStatus> pending_gnss_status_{};
  rclcpp::Time last_gnss_status_publish_{0, 0, RCL_ROS_TIME};
  std::optional<nav_msgs::msg::Odometry> pending_pose_{};
  rclcpp::Time last_pose_publish_{0, 0, RCL_ROS_TIME};

  // ---- High-level-status subscription watchdog state -------------------------

  static constexpr double kHighLevelStatusStaleAfterS = 5.0;

  bool received_high_level_status_{false};
  rclcpp::Time last_high_level_status_received_{0, 0, RCL_ROS_TIME};

  // ---- Area-list poll state --------------------------------------------------
  // Areas change rarely (only on an explicit add/edit/record) and there is no
  // ROS notification for "the area list changed", so <prefix>/areas is a slow
  // periodic poll rather than driven off a subscription like every other
  // topic here. kAreasPollIntervalS is independent of publish_rate_ on
  // purpose — walking GetMowingArea index-by-index is much heavier than the
  // single-message publishes the rate limiter above governs.
  static constexpr double kAreasPollIntervalS = 10.0;
  // Matches the GUI backend's own cap in pollMap() (gui/pkg/providers/ros.go).
  static constexpr uint32_t kMaxAreasPoll = 100;

  rclcpp::Time last_areas_poll_{0, 0, RCL_ROS_TIME};
  bool areas_poll_in_flight_{false};
  std::string last_areas_json_{};
  std::vector<AreaSummary> last_areas_{};

  // ---- Area boundary polling state -------------------------------------------

  static constexpr double kAreaPollIntervalS = 10.0;
  static constexpr uint32_t kMaxAreaPollCount = 100;  // matches <prefix>/areas' own cap (PR #638)

  rclcpp::Time last_area_poll_{0, 0, RCL_ROS_TIME};
  bool area_poll_in_progress_{false};
  std::string last_area_boundary_json_{};
};

}  // namespace mowgli_monitoring

#endif  // MOWGLI_MONITORING__MQTT_BRIDGE_NODE_HPP_
