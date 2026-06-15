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

#include "mowgli_behavior/calibration_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"

namespace mowgli_behavior
{

namespace
{

// -------------------------------------------------------------------------
// Total-least-squares line fit through a chronological sequence of
// (x, y) GPS samples. Returns (yaw, sigma_yaw): yaw is the chronological
// motion direction (NOT yet flipped to robot heading), sigma_yaw is the
// 1σ angular uncertainty derived from perpendicular residuals.
// -------------------------------------------------------------------------
std::pair<double, double> fit_motion_yaw(const std::vector<std::pair<double, double>>& s)
{
  const size_t n = s.size();
  if (n < 2u) {
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
  }

  double xsum = 0.0;
  double ysum = 0.0;
  for (const auto& p : s) { xsum += p.first; ysum += p.second; }
  const double xbar = xsum / static_cast<double>(n);
  const double ybar = ysum / static_cast<double>(n);

  double Sxx = 0.0;
  double Syy = 0.0;
  double Sxy = 0.0;
  for (const auto& p : s) {
    const double dx = p.first - xbar;
    const double dy = p.second - ybar;
    Sxx += dx * dx;
    Syy += dy * dy;
    Sxy += dx * dy;
  }

  // Principal axis of the centred 2×2 covariance: yaw = ½·atan2(2·Sxy, Sxx−Syy).
  // This gives the line direction up to a ±π ambiguity.
  double yaw = 0.5 * std::atan2(2.0 * Sxy, Sxx - Syy);

  // Resolve sign by chronological order: motion is from samples.front to
  // samples.back, dot with the current yaw vector must be positive.
  const double dx_chron = s.back().first  - s.front().first;
  const double dy_chron = s.back().second - s.front().second;
  if (dx_chron * std::cos(yaw) + dy_chron * std::sin(yaw) < 0.0) {
    yaw += M_PI;
  }
  while (yaw >  M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;

  // Perpendicular residuals → σ_yaw ≈ rms_perp / baseline.
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  double sum_perp2 = 0.0;
  for (const auto& p : s) {
    const double dx = p.first - xbar;
    const double dy = p.second - ybar;
    const double perp = -dx * sy + dy * cy;
    sum_perp2 += perp * perp;
  }
  const double rms_perp = std::sqrt(sum_perp2 / static_cast<double>(n));
  const double baseline = std::hypot(dx_chron, dy_chron);
  const double sigma_yaw = (baseline > 0.01) ? (rms_perp / baseline) : 0.1;
  return {yaw, sigma_yaw};
}

// -------------------------------------------------------------------------
// Angular EMA: blend two yaws by working in the (cos, sin) plane to avoid
// the ±π wrap discontinuity. weight applies to `measured`.
// -------------------------------------------------------------------------
double ema_yaw(double current, double measured, double weight)
{
  const double w = std::max(0.0, std::min(1.0, weight));
  const double cx = (1.0 - w) * std::cos(current) + w * std::cos(measured);
  const double cy = (1.0 - w) * std::sin(current) + w * std::sin(measured);
  return std::atan2(cy, cx);
}

// -------------------------------------------------------------------------
// In-place YAML splice for a single scalar key. Mirrors the helper in
// mowgli_map::area_manager.cpp — keeps comments + structure intact (yaml-cpp
// would strip them).
// -------------------------------------------------------------------------
constexpr const char* kRuntimeRobotYaml = "/ros2_ws/config/mowgli_robot.yaml";

bool splice_yaml_scalar(std::string& content, const std::string& key, const std::string& new_value)
{
  size_t scan = 0;
  while (scan < content.size()) {
    const size_t line_start = scan;
    size_t cursor = line_start;
    while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t')) {
      ++cursor;
    }
    if (cursor > line_start && cursor + key.size() < content.size() &&
        content.compare(cursor, key.size(), key) == 0 && content[cursor + key.size()] == ':')
    {
      cursor += key.size() + 1;
      while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t')) {
        ++cursor;
      }
      const size_t val_start = cursor;
      while (cursor < content.size()) {
        const char c = content[cursor];
        const bool is_num =
            (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (!is_num) break;
        ++cursor;
      }
      if (cursor > val_start) {
        content.replace(val_start, cursor - val_start, new_value);
        return true;
      }
    }
    const size_t nl = content.find('\n', line_start);
    if (nl == std::string::npos) break;
    scan = nl + 1;
  }
  return false;
}

// Read the current `dock_pose_yaw` from the runtime YAML so we can EMA-blend
// against it. Returns nullopt if the key isn't found or the file can't be
// opened — caller falls back to using the measured value directly.
std::optional<double> read_dock_pose_yaw_from_yaml()
{
  std::ifstream in(kRuntimeRobotYaml);
  if (!in.good()) {
    return std::nullopt;
  }
  std::string line;
  while (std::getline(in, line)) {
    size_t cursor = 0;
    while (cursor < line.size() && (line[cursor] == ' ' || line[cursor] == '\t')) ++cursor;
    static const std::string kKey = "dock_pose_yaw:";
    if (cursor + kKey.size() <= line.size() && line.compare(cursor, kKey.size(), kKey) == 0) {
      cursor += kKey.size();
      while (cursor < line.size() && (line[cursor] == ' ' || line[cursor] == '\t')) ++cursor;
      const size_t val_start = cursor;
      while (cursor < line.size()) {
        const char c = line[cursor];
        const bool is_num =
            (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (!is_num) break;
        ++cursor;
      }
      if (cursor > val_start) {
        try {
          return std::stod(line.substr(val_start, cursor - val_start));
        } catch (...) {
          return std::nullopt;
        }
      }
    }
  }
  return std::nullopt;
}

// Atomic-rename writeback of a single yaml scalar. Mirrors the safety
// pattern of mowgli_map's update_dock_pose_in_robot_yaml.
bool persist_dock_pose_yaw(double yaw_rad)
{
  std::ifstream in(kRuntimeRobotYaml);
  if (!in.good()) return false;
  std::stringstream buf;
  buf << in.rdbuf();
  std::string content = buf.str();
  in.close();

  std::ostringstream s;
  s << std::fixed << std::setprecision(6) << yaw_rad;
  if (!splice_yaml_scalar(content, "dock_pose_yaw", s.str())) {
    return false;
  }

  const std::string tmp_path = std::string(kRuntimeRobotYaml) + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.good()) return false;
    out << content;
    if (!out.good()) return false;
  }
  return std::rename(tmp_path.c_str(), kRuntimeRobotYaml) == 0;
}

// Fill the covariance block for a yaw-plus-xy seed: tight trust on the
// states we want to set, effectively infinite variance on the states we
// want the filter to keep from its prior.
void set_seed_covariance(geometry_msgs::msg::PoseWithCovarianceStamped& msg, double yaw_var)
{
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = 0.01;  // x
  msg.pose.covariance[7] = 0.01;  // y
  msg.pose.covariance[14] = 1e6;  // z (keep prior)
  msg.pose.covariance[21] = 1e6;  // roll (keep prior)
  msg.pose.covariance[28] = 1e6;  // pitch (keep prior)
  msg.pose.covariance[35] = yaw_var;
}

}  // namespace

// ---------------------------------------------------------------------------
// RecordUndockStart
// ---------------------------------------------------------------------------

BT::NodeStatus RecordUndockStart::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  ctx->undock_start_x = ctx->gps_x;
  ctx->undock_start_y = ctx->gps_y;
  ctx->undock_start_recorded = true;
  // Clear the per-undock GPS buffer and seed it with the current sample
  // so CalibrateHeadingFromUndock can line-fit the whole BackUp
  // trajectory. The GPS subscriber will append further samples (with
  // 5 cm dedup) while undock_start_recorded stays true.
  ctx->undock_gps_samples.clear();
  ctx->undock_gps_samples.emplace_back(ctx->gps_x, ctx->gps_y);
  RCLCPP_INFO(ctx->node->get_logger(),
              "RecordUndockStart: pos=(%.3f, %.3f)",
              ctx->undock_start_x,
              ctx->undock_start_y);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock
// ---------------------------------------------------------------------------

BT::NodeStatus CalibrateHeadingFromUndock::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!ctx->undock_start_recorded)
  {
    // RecordUndockStart never fired (e.g., fresh retry after a failed
    // undock where the node was halted). Report SUCCESS rather than
    // FAILURE so the rest of the sequence runs; the dock_yaw injection
    // (hardware_bridge → ekf_map via dock_yaw_to_set_pose) is still
    // active and will have seeded the filter from the charging state.
    RCLCPP_WARN(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: no undock_start recorded, "
                "skipping (relying on dock_yaw seed).");
    return BT::NodeStatus::SUCCESS;
  }

  // Minimum GPS displacement to refine yaw. At RTK-Fixed σ≈7 mm,
  // σ_yaw = atan2(2σ_pos, displacement). 0.20 m → σ ≈ 4° — far tighter
  // than the dock_yaw seed σ=10° floor, so even a partial undock with
  // wheel slip on the ramp still gives us a useful yaw refinement based
  // on the robot's actual motion rather than yesterday's calibration.
  double min_displacement = 0.20;
  getInput("min_displacement_m", min_displacement);

  const double dx = ctx->gps_x - ctx->undock_start_x;
  const double dy = ctx->gps_y - ctx->undock_start_y;
  const double dist = std::hypot(dx, dy);

  if (dist < min_displacement)
  {
    const bool still_on_dock = ctx->latest_power.charger_enabled;
    ctx->undock_start_recorded = false;
    if (still_on_dock)
    {
      // BackUp reported complete but GPS barely moved AND the dock still
      // charges — robot is genuinely stuck on the dock latch. Fail so
      // UndockSequence fails and the outer ReactiveSequence retries.
      RCLCPP_WARN(ctx->node->get_logger(),
                  "CalibrateHeadingFromUndock: displacement %.3fm below min %.3fm "
                  "AND is_charging=true — robot stuck on dock, retrying undock.",
                  dist,
                  min_displacement);
      return BT::NodeStatus::FAILURE;
    }
    // Partial undock: GPS moved less than expected (wheel slip on the dock
    // ramp is common) but charging has dropped, so the robot IS off the
    // dock. The displacement is too short to refine yaw reliably — trust
    // the dock_yaw injected by dock_yaw_to_set_pose while still on the
    // dock and continue with the rest of the session.
    RCLCPP_INFO(ctx->node->get_logger(),
                "CalibrateHeadingFromUndock: partial undock (%.3fm < %.3fm) but "
                "charging dropped — keeping dock_yaw seed, skipping refinement.",
                dist,
                min_displacement);
    ctx->yaw_seeded_this_session = true;
    return BT::NodeStatus::SUCCESS;
  }

  // Derive the chassis yaw from the BackUp trajectory. Two strategies:
  //   * line_fit (preferred): total-least-squares through every GPS
  //     sample buffered during the BackUp. σ_yaw shrinks roughly as
  //     1/√n and uses the FULL baseline, so a 1.5 m undock at RTK-Fixed
  //     with ~30 samples reaches σ ≈ 0.05–0.1°, well under the 0.23°
  //     endpoint-only ceiling. Requires ≥ 4 samples spanning ≥ 0.5 m.
  //   * endpoint (fallback): atan2(-dy, -dx) on start vs. current
  //     position. σ ≈ atan(2·σ_GPS / dist).
  // Whichever produces the result, the value is the chassis heading at
  // the END of the BackUp — by construction the chassis didn't rotate
  // during the straight reverse so it equals the dock_pose_yaw, which
  // is what we want to persist.
  const auto& samples = ctx->undock_gps_samples;
  const bool have_line_fit_samples = samples.size() >= 4u &&
      std::hypot(samples.back().first  - samples.front().first,
                 samples.back().second - samples.front().second) >= 0.5;
  double yaw = 0.0;
  double sigma_yaw = 0.0;
  const char* method = "endpoint";
  if (have_line_fit_samples) {
    const auto [motion_yaw, motion_sigma] = fit_motion_yaw(samples);
    // Robot heading points opposite the motion (BackUp reverses).
    yaw = motion_yaw + M_PI;
    while (yaw >  M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    sigma_yaw = std::max(motion_sigma, 0.001);  // floor at ~0.06°
    method = "line_fit";
  } else {
    // Endpoint fallback. Motion vector (dx, dy) points OPPOSITE the
    // robot's heading so heading = atan2(-dy, -dx).
    yaw = std::atan2(-dy, -dx);
    sigma_yaw = std::atan2(2.0 * 0.007, std::max(dist, 0.05));
  }
  const double yaw_var = std::max(sigma_yaw * sigma_yaw, 5e-4);  // floor σ≈1.3°

  if (!set_pose_pub_)
  {
    // TRANSIENT_LOCAL matches fusion_graph_node's set_pose subscriber.
    // The calibration seed is a one-shot, fire-and-forget message:
    // VOLATILE on either side races subscription discovery (~tens of ms
    // after node up) and silently drops the seed, leaving fusion_graph
    // anchored at the persisted dock pose while the robot drives away.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    set_pose_pub_ = ctx->node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fusion_graph_node/set_pose", qos);
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(seed, yaw_var);
  set_pose_pub_->publish(seed);

  // EMA-persist into mowgli_robot.yaml so the next session's
  // SeedFromDockPose starts from a refined dock_pose_yaw. Weight 0.3
  // on the new measurement converges in ~5-10 sessions but stays
  // robust to a single bad reading (RTK glitch, sloppy partial undock).
  // Only writes back when the line-fit branch fired AND σ is tight
  // enough to be a real refinement — sloppier endpoint measurements
  // stay as the live /set_pose seed but don't pollute the persisted
  // value with a noisy update.
  constexpr double kEmaWeight = 0.30;
  constexpr double kMaxSigmaForPersist = 0.02;   // ~1.15°
  bool persisted = false;
  if (have_line_fit_samples && sigma_yaw <= kMaxSigmaForPersist) {
    const auto current_yaml = read_dock_pose_yaw_from_yaml();
    const double blended = current_yaml.has_value()
                              ? ema_yaw(current_yaml.value(), yaw, kEmaWeight)
                              : yaw;
    persisted = persist_dock_pose_yaw(blended);
    if (!persisted) {
      RCLCPP_WARN_THROTTLE(
          ctx->node->get_logger(),
          *ctx->node->get_clock(),
          5000,
          "CalibrateHeadingFromUndock: could not persist dock_pose_yaw to %s — "
          "file missing or not writable. /set_pose seed still applied.",
          kRuntimeRobotYaml);
    }
  }

  ctx->undock_start_recorded = false;
  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "CalibrateHeadingFromUndock: dist=%.3fm yaw=%.2f° σ=%.2f° "
              "method=%s n=%zu pos=(%.3f, %.3f) %s",
              dist,
              yaw * 180.0 / M_PI,
              sigma_yaw * 180.0 / M_PI,
              method,
              samples.size(),
              ctx->gps_x,
              ctx->gps_y,
              persisted ? "[yaml updated]" : "[set_pose only]");
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// SeedYawFromMotion
// ---------------------------------------------------------------------------

void SeedYawFromMotion::publish_forward(const rclcpp::Node::SharedPtr& node, double speed)
{
  geometry_msgs::msg::TwistStamped cmd{};
  cmd.header.stamp = node->now();
  cmd.header.frame_id = "base_footprint";
  cmd.twist.linear.x = speed;
  cmd_pub_->publish(cmd);
}

void SeedYawFromMotion::publish_zero(const rclcpp::Node::SharedPtr& node)
{
  publish_forward(node, 0.0);
}

BT::NodeStatus SeedYawFromMotion::onStart()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->yaw_seeded_this_session)
  {
    // Already seeded this session — don't re-drive on ReactiveSequence
    // halt/resume cycles.
    RCLCPP_DEBUG(ctx->node->get_logger(),
                 "SeedYawFromMotion: yaw already seeded this session, "
                 "skipping forward drive.");
    return BT::NodeStatus::SUCCESS;
  }

  if (!cmd_pub_)
  {
    cmd_pub_ = ctx->node->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_teleop", 10);
  }
  if (!set_pose_pub_)
  {
    // TRANSIENT_LOCAL matches fusion_graph_node's set_pose subscriber.
    // The calibration seed is a one-shot, fire-and-forget message:
    // VOLATILE on either side races subscription discovery (~tens of ms
    // after node up) and silently drops the seed, leaving fusion_graph
    // anchored at the persisted dock pose while the robot drives away.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    set_pose_pub_ = ctx->node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/fusion_graph_node/set_pose", qos);
  }

  distance_m_ = 1.0;
  speed_ms_ = 0.2;
  timeout_sec_ = 30.0;
  min_displacement_m_ = 0.5;
  getInput("distance_m", distance_m_);
  getInput("speed_ms", speed_ms_);
  getInput("timeout_sec", timeout_sec_);
  getInput("min_displacement_m", min_displacement_m_);

  if (!ctx->gps_is_fixed)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: no RTK fix at start (fix_type=%u), "
                "aborting seed.",
                ctx->gps_fix_type);
    return BT::NodeStatus::FAILURE;
  }

  x0_ = ctx->gps_x;
  y0_ = ctx->gps_y;
  start_time_ = ctx->node->now();

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: start pos=(%.3f, %.3f), drive %.2fm fwd at %.2fm/s",
              x0_,
              y0_,
              distance_m_,
              speed_ms_);
  publish_forward(ctx->node, speed_ms_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SeedYawFromMotion::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (ctx->latest_emergency.active_emergency || ctx->latest_emergency.latched_emergency)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: emergency during seed drive, aborting.");
    return BT::NodeStatus::FAILURE;
  }

  const double elapsed = (ctx->node->now() - start_time_).seconds();
  if (elapsed > timeout_sec_)
  {
    publish_zero(ctx->node);
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: timeout after %.1fs, aborting.",
                elapsed);
    return BT::NodeStatus::FAILURE;
  }

  const double dx = ctx->gps_x - x0_;
  const double dy = ctx->gps_y - y0_;
  const double dist = std::hypot(dx, dy);

  if (dist < distance_m_)
  {
    publish_forward(ctx->node, speed_ms_);
    return BT::NodeStatus::RUNNING;
  }

  publish_zero(ctx->node);

  if (dist < min_displacement_m_)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SeedYawFromMotion: displacement %.3fm below min %.3fm, "
                "seed invalid.",
                dist,
                min_displacement_m_);
    return BT::NodeStatus::FAILURE;
  }

  const double yaw = std::atan2(dy, dx);

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::PoseWithCovarianceStamped seed{};
  seed.header.stamp = ctx->node->now();
  seed.header.frame_id = "map";
  seed.pose.pose.position.x = ctx->gps_x;
  seed.pose.pose.position.y = ctx->gps_y;
  seed.pose.pose.orientation.x = q.x();
  seed.pose.pose.orientation.y = q.y();
  seed.pose.pose.orientation.z = q.z();
  seed.pose.pose.orientation.w = q.w();
  set_seed_covariance(seed, 5e-3);  // ~4° σ
  set_pose_pub_->publish(seed);

  ctx->yaw_seeded_this_session = true;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SeedYawFromMotion: dist=%.3fm yaw_seed=%.1f° pos=(%.3f, %.3f) — "
              "set_pose published.",
              dist,
              yaw * 180.0 / M_PI,
              ctx->gps_x,
              ctx->gps_y);
  return BT::NodeStatus::SUCCESS;
}

void SeedYawFromMotion::onHalted()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  if (cmd_pub_)
  {
    publish_zero(ctx->node);
  }
}

}  // namespace mowgli_behavior
