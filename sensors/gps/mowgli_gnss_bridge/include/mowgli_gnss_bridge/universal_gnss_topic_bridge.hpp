// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// C++ port of sensors/gps/universal_gnss_topic_bridge.py.
// Bridges the internal universal_gnss_ros2 topics onto the public Mowgli GNSS
// contract. Behaviour-exact with the Python node it replaces.

#ifndef MOWGLI_GNSS_BRIDGE__UNIVERSAL_GNSS_TOPIC_BRIDGE_HPP_
#define MOWGLI_GNSS_BRIDGE__UNIVERSAL_GNSS_TOPIC_BRIDGE_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "mowgli_interfaces/msg/gnss_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "universal_gnss_ros2/msg/gnss_status.hpp"
#include "universal_gnss_ros2/msg/rtcm_frame.hpp"

namespace mowgli_gnss_bridge
{

using PublicGnssStatus = mowgli_interfaces::msg::GnssStatus;
using UniversalGnssStatus = universal_gnss_ros2::msg::GnssStatus;
using RtcmFrame = universal_gnss_ros2::msg::RtcmFrame;
using PublicRtcmMessage = rtcm_msgs::msg::Message;

/// Node that republishes Universal GNSS status/RTCM onto the public contract.
///
/// Behaviour mirrors the reference Python implementation exactly: same topics,
/// same QoS (reliable, depth 10 for status/diagnostics, depth 50 for RTCM),
/// same parameter names/defaults, and the same field / capability-flag /
/// diagnostics projection logic.
class UniversalGnssTopicBridge : public rclcpp::Node
{
public:
  explicit UniversalGnssTopicBridge(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /// Cached (message, key/value) pair for one DiagnosticStatus entry.
  struct DiagnosticEntry
  {
    std::string message;
    std::unordered_map<std::string, std::string> values;
  };

  void onStatus(const UniversalGnssStatus & msg);
  void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray & msg);
  void onRtcm(const RtcmFrame & msg);

  void applyDiagnosticProjection(PublicGnssStatus & public_msg) const;

  const DiagnosticEntry * pickDiagnosticEntry(
    std::initializer_list<const char *> names) const;

  std::string backend_;
  std::string receiver_vendor_;
  std::string frame_id_;

  // status.name -> latest cached diagnostic entry.
  std::map<std::string, DiagnosticEntry> diagnostic_entries_;

  rclcpp::Publisher<PublicGnssStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<PublicRtcmMessage>::SharedPtr rtcm_pub_;

  rclcpp::Subscription<UniversalGnssStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::Subscription<RtcmFrame>::SharedPtr rtcm_sub_;
};

}  // namespace mowgli_gnss_bridge

#endif  // MOWGLI_GNSS_BRIDGE__UNIVERSAL_GNSS_TOPIC_BRIDGE_HPP_
