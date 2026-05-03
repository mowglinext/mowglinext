// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// calibrate_imu_yaw_node.cpp
//
// C++ port of scripts/calibrate_imu_yaw_node.py — autonomous IMU-yaw,
// pitch/roll, dock-yaw and magnetometer calibration. The service
// /calibrate_imu_yaw_node/calibrate drives the robot through a profile
// (forward/backward + figure-8) and computes the calibration outputs.
//
// Constants, motion profile, sample filters, formulas, and the YAML
// output layout match the Python implementation.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/calibrate_imu_yaw.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <yaml-cpp/yaml.h>

namespace mowgli_localization
{

namespace
{
// HighLevelStatus state codes.
constexpr int HL_STATE_NULL = 0;
constexpr int HL_STATE_IDLE = 1;
constexpr int HL_STATE_AUTONOMOUS = 2;
constexpr int HL_STATE_RECORDING = 3;

// HighLevelControl command codes.
constexpr int HL_CMD_RECORD_AREA = 3;
constexpr int HL_CMD_RECORD_CANCEL = 6;

double stamp_to_float(const builtin_interfaces::msg::Time& s)
{
  return static_cast<double>(s.sec) + static_cast<double>(s.nanosec) * 1e-9;
}

double monotonic()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void sleep_for(double sec)
{
  std::this_thread::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sec)));
}

// Splice a new numeric value into a "<indent><key>:<spaces><number><rest>"
// line, anchored on the indent so a key whose name happens to contain
// ours (e.g. dock_pose_x_offset) is not matched.
void splice_yaml_scalar(std::string& content, const std::string& key, const std::string& new_value)
{
  size_t scan = 0;
  while (scan < content.size())
  {
    const size_t line_start = scan;
    size_t cursor = line_start;
    while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
      ++cursor;
    const size_t indent_end = cursor;
    if (indent_end > line_start && cursor + key.size() < content.size() &&
        content.compare(cursor, key.size(), key) == 0 && content[cursor + key.size()] == ':')
    {
      cursor += key.size() + 1;
      while (cursor < content.size() && (content[cursor] == ' ' || content[cursor] == '\t'))
        ++cursor;
      const size_t val_start = cursor;
      while (cursor < content.size())
      {
        const char c = content[cursor];
        const bool is_num =
            (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
        if (!is_num)
          break;
        ++cursor;
      }
      if (cursor > val_start)
      {
        content.replace(val_start, cursor - val_start, new_value);
        return;
      }
    }
    const size_t nl = content.find('\n', line_start);
    if (nl == std::string::npos)
      break;
    scan = nl + 1;
  }
}

// Update dock_pose_x/y/yaw values in mowgli_robot.yaml in place via
// per-line substring splicing — preserves comments and surrounding
// structure (yaml-cpp would round-trip and strip them). Atomic via
// tmp+rename. Returns false if the file can't be read or written.
bool update_dock_pose_in_robot_yaml(const std::string& path, double x, double y, double yaw_rad)
{
  std::ifstream in(path);
  if (!in.good())
    return false;
  std::stringstream buf;
  buf << in.rdbuf();
  std::string content = buf.str();
  in.close();

  auto fmt = [](double v)
  {
    std::ostringstream s;
    s << std::fixed << std::setprecision(6) << v;
    return s.str();
  };
  splice_yaml_scalar(content, "dock_pose_x", fmt(x));
  splice_yaml_scalar(content, "dock_pose_y", fmt(y));
  splice_yaml_scalar(content, "dock_pose_yaw", fmt(yaw_rad));

  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream out(tmp_path, std::ios::trunc);
    if (!out.good())
      return false;
    out << content;
    if (!out.good())
      return false;
  }
  std::error_code ec;
  std::filesystem::rename(tmp_path, path, ec);
  return !ec;
}

std::string utc_iso8601_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t tt = std::chrono::system_clock::to_time_t(now);
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() %
      1000000;
  std::tm tm_utc{};
  gmtime_r(&tt, &tm_utc);
  char buf[64];
  std::snprintf(buf,
                sizeof(buf),
                "%04d-%02d-%02dT%02d:%02d:%02d.%06ld+00:00",
                tm_utc.tm_year + 1900,
                tm_utc.tm_mon + 1,
                tm_utc.tm_mday,
                tm_utc.tm_hour,
                tm_utc.tm_min,
                tm_utc.tm_sec,
                static_cast<long>(us));
  return std::string(buf);
}
}  // namespace

class CalibrateImuYawNode : public rclcpp::Node
{
public:
  // --- Sample filter thresholds ---
  static constexpr double WZ_STRAIGHT_THRESHOLD = 0.05;
  static constexpr int MIN_SAMPLES = 50;
  static constexpr double STATIONARY_VX_THRESHOLD = 0.01;
  static constexpr double STATIONARY_WZ_THRESHOLD = 0.02;
  static constexpr int MIN_STATIONARY_SAMPLES = 150;

  // --- Motion profile ---
  static constexpr double CRUISE_SPEED = 0.5;
  static constexpr double RAMP_SEC = 0.5;
  static constexpr double CRUISE_SEC = 1.0;
  static constexpr double PAUSE_SEC = 1.0;
  static constexpr double CMD_RATE_HZ = 20.0;
  static constexpr double SETTLE_SEC = 1.0;
  static constexpr double BASELINE_SEC = 1.5;
  static constexpr double ACCEL_BODY_THRESHOLD = 0.3;
  static constexpr int N_CYCLES = 3;

  // --- Dock yaw ---
  static constexpr double DOCK_UNDOCK_SPEED = 0.15;
  static constexpr double DOCK_UNDOCK_DISTANCE_M = 2.0;
  static constexpr double DOCK_UNDOCK_TIMEOUT_SEC = 25.0;
  static constexpr double DOCK_UNDOCK_MIN_DISPLACEMENT = 0.8;
  // Runtime mowgli_robot.yaml — bind-mounted, persists across redeploys.
  // Calibration writes the dock pose back here so the same file the launch
  // system reads at startup also carries the latest measured values.
  static constexpr const char* MOWGLI_ROBOT_YAML_PATH = "/ros2_ws/config/mowgli_robot.yaml";

  // --- Mag calibration ---
  static constexpr double MAG_FIG8_LINEAR_M_S = 0.20;
  static constexpr double MAG_FIG8_RADIUS_M = 0.60;
  static constexpr double MAG_FIG8_LOOPS_PER_SIDE = 1.5;
  static constexpr double MAG_FIG8_PAUSE_SEC = 1.5;
  static constexpr int MAG_MIN_SAMPLES = 150;
  static constexpr const char* MAG_CALIBRATION_PATH = "/ros2_ws/maps/mag_calibration.yaml";

  CalibrateImuYawNode() : Node("calibrate_imu_yaw_node")
  {
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    imu_qos_ = rclcpp::QoS(rclcpp::KeepLast(50));
    imu_qos_.best_effort();
    imu_qos_.durability_volatile();

    odom_qos_ = rclcpp::QoS(rclcpp::KeepLast(50));
    odom_qos_.reliable();
    odom_qos_.durability_volatile();

    rclcpp::QoS state_qos(rclcpp::KeepLast(10));
    state_qos.reliable();
    state_qos.durability_volatile();

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;

    status_sub_ = create_subscription<mowgli_interfaces::msg::Status>(
        "/hardware_bridge/status",
        state_qos,
        [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
        {
          is_charging_ = msg->is_charging;
        },
        sub_opts);
    emergency_sub_ = create_subscription<mowgli_interfaces::msg::Emergency>(
        "/hardware_bridge/emergency",
        state_qos,
        [this](mowgli_interfaces::msg::Emergency::ConstSharedPtr msg)
        {
          emergency_active_ = msg->active_emergency || msg->latched_emergency;
        },
        sub_opts);
    bt_status_sub_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        state_qos,
        [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
        {
          bt_state_ = static_cast<int>(msg->state);
        },
        sub_opts);

    hlc_client_ = create_client<mowgli_interfaces::srv::HighLevelControl>(
        "/behavior_tree_node/high_level_control", rclcpp::ServicesQoS(), cb_group_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_teleop", state_qos);

    srv_ = create_service<mowgli_interfaces::srv::CalibrateImuYaw>(
        "~/calibrate",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Request> req,
               std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Response> resp)
        {
          calibrate_cb(req, resp);
        },
        rclcpp::ServicesQoS(),
        cb_group_);

    RCLCPP_INFO(get_logger(),
                "IMU yaw calibration node ready. Ensure robot is undocked with "
                "~1 m of clear space in front and behind, then call "
                "~/calibrate.");
  }

private:
  // ── Subscription callbacks ──────────────────────────────────────────
  void imu_cb(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!collecting_)
      return;
    const double t = stamp_to_float(msg->header.stamp);
    std::lock_guard<std::mutex> lk(lock_);
    imu_samples_.emplace_back(t,
                              msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
  }

  void odom_cb(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    if (!collecting_)
      return;
    const double t = stamp_to_float(msg->header.stamp);
    std::lock_guard<std::mutex> lk(lock_);
    odom_samples_.emplace_back(t, msg->twist.twist.linear.x, msg->twist.twist.angular.z);
  }

  void mag_cb(sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
  {
    if (!collecting_mag_)
      return;
    std::lock_guard<std::mutex> lk(lock_);
    mag_samples_.emplace_back(msg->magnetic_field.x * 1e6,
                              msg->magnetic_field.y * 1e6,
                              msg->magnetic_field.z * 1e6);
  }

  void gps_cb(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
  {
    latest_gps_x_ = msg->pose.pose.position.x;
    latest_gps_y_ = msg->pose.pose.position.y;
    gps_position_accuracy_ = msg->position_accuracy;
    gps_rtk_fixed_ = (msg->flags & mowgli_interfaces::msg::AbsolutePose::FLAG_GPS_RTK_FIXED) != 0;
    gps_have_ = true;
  }

  void activate_sensor_subs()
  {
    if (imu_sub_)
      return;
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        imu_qos_,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
        {
          imu_cb(msg);
        },
        sub_opts);
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
        "/imu/mag_raw",
        imu_qos_,
        [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
        {
          mag_cb(msg);
        },
        sub_opts);
    gps_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
        "/gps/absolute_pose",
        imu_qos_,
        [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
        {
          gps_cb(msg);
        },
        sub_opts);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom",
        odom_qos_,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          odom_cb(msg);
        },
        sub_opts);
  }

  void deactivate_sensor_subs()
  {
    imu_sub_.reset();
    mag_sub_.reset();
    gps_sub_.reset();
    odom_sub_.reset();
  }

  // ── Drive primitives ────────────────────────────────────────────────
  void publish_vx(double vx)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.header.frame_id = "base_footprint";
    m.twist.linear.x = vx;
    cmd_pub_->publish(m);
  }

  void publish_arc(double vx, double wz)
  {
    geometry_msgs::msg::TwistStamped m;
    m.header.stamp = now();
    m.header.frame_id = "base_footprint";
    m.twist.linear.x = vx;
    m.twist.angular.z = wz;
    cmd_pub_->publish(m);
  }

  void drive_profile(double signed_cruise_speed)
  {
    const double period = 1.0 / CMD_RATE_HZ;
    int n = std::max(1, static_cast<int>(RAMP_SEC * CMD_RATE_HZ));
    for (int i = 0; i < n; ++i)
    {
      const double v = signed_cruise_speed * (i + 1) / n;
      publish_vx(v);
      sleep_for(period);
    }
    n = std::max(1, static_cast<int>(CRUISE_SEC * CMD_RATE_HZ));
    for (int i = 0; i < n; ++i)
    {
      publish_vx(signed_cruise_speed);
      sleep_for(period);
    }
    n = std::max(1, static_cast<int>(RAMP_SEC * CMD_RATE_HZ));
    for (int i = 0; i < n; ++i)
    {
      const double v = signed_cruise_speed * (n - i - 1) / n;
      publish_vx(v);
      sleep_for(period);
    }
    publish_vx(0.0);
  }

  void pause(double seconds)
  {
    const double period = 1.0 / CMD_RATE_HZ;
    int n = std::max(1, static_cast<int>(seconds * CMD_RATE_HZ));
    for (int i = 0; i < n; ++i)
    {
      publish_vx(0.0);
      sleep_for(period);
    }
  }

  void figure_eight_profile(double linear_m_s,
                            double radius_m,
                            double loops_per_side,
                            double pause_sec)
  {
    if (radius_m <= 1e-3 || linear_m_s <= 1e-3)
      return;
    const double wz_mag = linear_m_s / radius_m;
    const double period = 1.0 / CMD_RATE_HZ;
    const double seconds_per_side = (2.0 * M_PI * loops_per_side) / wz_mag;
    const int steps_per_side = std::max(1, static_cast<int>(seconds_per_side * CMD_RATE_HZ));

    for (double sign : {+1.0, -1.0})
    {
      for (int i = 0; i < steps_per_side; ++i)
      {
        if (emergency_active_)
        {
          publish_arc(0.0, 0.0);
          return;
        }
        publish_arc(linear_m_s, sign * wz_mag);
        sleep_for(period);
      }
      const int pause_steps = std::max(1, static_cast<int>(pause_sec * CMD_RATE_HZ));
      for (int i = 0; i < pause_steps; ++i)
      {
        if (emergency_active_)
        {
          publish_arc(0.0, 0.0);
          return;
        }
        publish_arc(0.0, 0.0);
        sleep_for(period);
      }
    }
    publish_arc(0.0, 0.0);
  }

  bool wait_for_rtk_fixed(double timeout_sec = 10.0)
  {
    const double deadline = monotonic() + timeout_sec;
    while (monotonic() < deadline)
    {
      if (gps_rtk_fixed_ && gps_position_accuracy_ < 0.05)
        return true;
      publish_vx(0.0);
      sleep_for(1.0 / CMD_RATE_HZ);
    }
    return false;
  }

  // ── Dock yaw drive ──────────────────────────────────────────────────
  struct DockYawResult
  {
    double dock_pose_x;
    double dock_pose_y;
    double dock_pose_yaw_rad;
    double dock_pose_yaw_deg;
    double undock_displacement_m;
    double yaw_sigma_rad;
    double yaw_sigma_deg;
    double speed_ms;
  };

  std::optional<DockYawResult> run_dock_yaw_drive()
  {
    RCLCPP_INFO(get_logger(), "Dock yaw calibration: waiting for RTK-Fixed (≤ 10 s)…");
    if (!wait_for_rtk_fixed(10.0))
    {
      RCLCPP_ERROR(get_logger(),
                   "No RTK-Fixed within 10 s (rtk_fixed=%d, pos_acc=%.3f m). "
                   "Cannot compute dock yaw — aborting.",
                   gps_rtk_fixed_ ? 1 : 0,
                   gps_position_accuracy_.load());
      return std::nullopt;
    }

    if (!gps_have_)
    {
      RCLCPP_ERROR(get_logger(),
                   "/gps/absolute_pose never arrived despite RTK Fixed — "
                   "aborting.");
      return std::nullopt;
    }
    const double x0 = latest_gps_x_;
    const double y0 = latest_gps_y_;

    RCLCPP_INFO(get_logger(),
                "RTK-Fixed acquired. Reversing at %+.2f m/s target %.1f m "
                "(timeout %.0f s) — start pos=(%+.3f, %+.3f).",
                DOCK_UNDOCK_SPEED,
                DOCK_UNDOCK_DISTANCE_M,
                DOCK_UNDOCK_TIMEOUT_SEC,
                x0,
                y0);

    const double period = 1.0 / CMD_RATE_HZ;
    const double t_deadline = monotonic() + DOCK_UNDOCK_TIMEOUT_SEC;
    double displacement = 0.0;
    while (monotonic() < t_deadline)
    {
      if (emergency_active_)
      {
        publish_vx(0.0);
        RCLCPP_ERROR(get_logger(), "Emergency during dock undock — aborting.");
        return std::nullopt;
      }
      publish_vx(-DOCK_UNDOCK_SPEED);
      if (gps_have_)
      {
        const double dx = latest_gps_x_ - x0;
        const double dy = latest_gps_y_ - y0;
        displacement = std::hypot(dx, dy);
        if (displacement >= DOCK_UNDOCK_DISTANCE_M)
          break;
      }
      sleep_for(period);
    }

    for (int i = 0; i < static_cast<int>(1.0 * CMD_RATE_HZ); ++i)
    {
      publish_vx(0.0);
      sleep_for(period);
    }

    if (!gps_have_)
    {
      RCLCPP_ERROR(get_logger(), "Lost GPS during dock undock — aborting.");
      return std::nullopt;
    }
    const double x1 = latest_gps_x_;
    const double y1 = latest_gps_y_;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    displacement = std::hypot(dx, dy);

    if (displacement < DOCK_UNDOCK_MIN_DISPLACEMENT)
    {
      RCLCPP_ERROR(get_logger(),
                   "Only %.3f m of GPS displacement after reverse "
                   "(need ≥ %.1f m). Wheels probably slipped on the dock "
                   "ramp. Aborting.",
                   displacement,
                   DOCK_UNDOCK_MIN_DISPLACEMENT);
      return std::nullopt;
    }

    const double dock_yaw = std::atan2(-dy, -dx);
    const double sigma_pos = std::max(static_cast<double>(gps_position_accuracy_.load()), 0.003);
    const double sigma_yaw_rad = std::atan2(2.0 * sigma_pos, displacement);

    DockYawResult result;
    result.dock_pose_x = x0;
    result.dock_pose_y = y0;
    result.dock_pose_yaw_rad = dock_yaw;
    result.dock_pose_yaw_deg = dock_yaw * 180.0 / M_PI;
    result.undock_displacement_m = displacement;
    result.yaw_sigma_rad = sigma_yaw_rad;
    result.yaw_sigma_deg = sigma_yaw_rad * 180.0 / M_PI;
    result.speed_ms = DOCK_UNDOCK_SPEED;

    if (!update_dock_pose_in_robot_yaml(MOWGLI_ROBOT_YAML_PATH,
                                        result.dock_pose_x,
                                        result.dock_pose_y,
                                        result.dock_pose_yaw_rad))
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to persist dock pose to %s — file missing or "
                   "not writable.",
                   MOWGLI_ROBOT_YAML_PATH);
      return std::nullopt;
    }

    RCLCPP_INFO(get_logger(),
                "Dock yaw calibration: start=(%+.3f, %+.3f) end=(%+.3f, "
                "%+.3f) displacement=%.3f m dock_yaw=%+.2f° (σ=%.2f°). "
                "Saved to %s.",
                x0,
                y0,
                x1,
                y1,
                displacement,
                result.dock_pose_yaw_deg,
                result.yaw_sigma_deg,
                MOWGLI_ROBOT_YAML_PATH);
    return result;
  }

  // ── Mag fit ─────────────────────────────────────────────────────────
  struct MagFit
  {
    double offset_x_uT;
    double offset_y_uT;
    double offset_z_uT;
    double scale_x;
    double scale_y;
    double scale_z;
    int sample_count;
    double magnitude_mean_uT;
    double magnitude_std_uT;
    double raw_min[3];
    double raw_max[3];
  };

  static MagFit fit_mag_min_max(const std::vector<std::tuple<double, double, double>>& samples)
  {
    MagFit f{};
    double mn[3] = {std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity(),
                    std::numeric_limits<double>::infinity()};
    double mx[3] = {-std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity(),
                    -std::numeric_limits<double>::infinity()};
    for (const auto& s : samples)
    {
      const double v[3] = {std::get<0>(s), std::get<1>(s), std::get<2>(s)};
      for (int i = 0; i < 3; ++i)
      {
        if (v[i] < mn[i])
          mn[i] = v[i];
        if (v[i] > mx[i])
          mx[i] = v[i];
      }
    }
    double offset[3], range[3];
    for (int i = 0; i < 3; ++i)
    {
      offset[i] = (mx[i] + mn[i]) / 2.0;
      range[i] = mx[i] - mn[i];
    }
    const double mean_range = (range[0] + range[1] + range[2]) / 3.0;
    double scale[3];
    for (int i = 0; i < 3; ++i)
      scale[i] = (range[i] > 1e-6) ? (mean_range / range[i]) : 1.0;

    double mag_sum = 0.0;
    std::vector<double> mags;
    mags.reserve(samples.size());
    for (const auto& s : samples)
    {
      const double cx = (std::get<0>(s) - offset[0]) * scale[0];
      const double cy = (std::get<1>(s) - offset[1]) * scale[1];
      const double cz = (std::get<2>(s) - offset[2]) * scale[2];
      const double m = std::sqrt(cx * cx + cy * cy + cz * cz);
      mags.push_back(m);
      mag_sum += m;
    }
    const double mag_mean = mags.empty() ? 0.0 : mag_sum / static_cast<double>(mags.size());
    double var = 0.0;
    for (double m : mags)
      var += (m - mag_mean) * (m - mag_mean);
    if (!mags.empty())
      var /= static_cast<double>(mags.size());

    f.offset_x_uT = offset[0];
    f.offset_y_uT = offset[1];
    f.offset_z_uT = offset[2];
    f.scale_x = scale[0];
    f.scale_y = scale[1];
    f.scale_z = scale[2];
    f.sample_count = static_cast<int>(samples.size());
    f.magnitude_mean_uT = mag_mean;
    f.magnitude_std_uT = std::sqrt(var);
    for (int i = 0; i < 3; ++i)
    {
      f.raw_min[i] = mn[i];
      f.raw_max[i] = mx[i];
    }
    return f;
  }

  void fit_and_save_mag(const std::vector<std::tuple<double, double, double>>& samples)
  {
    if (static_cast<int>(samples.size()) < MAG_MIN_SAMPLES)
    {
      RCLCPP_WARN(get_logger(),
                  "Too few mag samples for calibration (%zu < %d). Verify "
                  "/imu/mag_raw is publishing.",
                  samples.size(),
                  MAG_MIN_SAMPLES);
      return;
    }
    const MagFit cal = fit_mag_min_max(samples);
    try
    {
      namespace fs = std::filesystem;
      fs::path target(MAG_CALIBRATION_PATH);
      if (target.has_parent_path())
      {
        std::error_code ec;
        fs::create_directories(target.parent_path(), ec);
      }
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "mag_calibration" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "offset_x_uT" << YAML::Value << cal.offset_x_uT;
      out << YAML::Key << "offset_y_uT" << YAML::Value << cal.offset_y_uT;
      out << YAML::Key << "offset_z_uT" << YAML::Value << cal.offset_z_uT;
      out << YAML::Key << "scale_x" << YAML::Value << cal.scale_x;
      out << YAML::Key << "scale_y" << YAML::Value << cal.scale_y;
      out << YAML::Key << "scale_z" << YAML::Value << cal.scale_z;
      out << YAML::Key << "sample_count" << YAML::Value << cal.sample_count;
      out << YAML::Key << "magnitude_mean_uT" << YAML::Value << cal.magnitude_mean_uT;
      out << YAML::Key << "magnitude_std_uT" << YAML::Value << cal.magnitude_std_uT;
      out << YAML::Key << "raw_min_uT" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << cal.raw_min[0] << cal.raw_min[1] << cal.raw_min[2] << YAML::EndSeq;
      out << YAML::Key << "raw_max_uT" << YAML::Value << YAML::Flow << YAML::BeginSeq
          << cal.raw_max[0] << cal.raw_max[1] << cal.raw_max[2] << YAML::EndSeq;
      out << YAML::Key << "calibrated_at" << YAML::Value << utc_iso8601_now();
      out << YAML::Key << "source" << YAML::Value << "figure_eight_drive";
      out << YAML::Key << "fig8_radius_m" << YAML::Value << MAG_FIG8_RADIUS_M;
      out << YAML::Key << "fig8_linear_m_s" << YAML::Value << MAG_FIG8_LINEAR_M_S;
      out << YAML::Key << "fig8_loops_per_side" << YAML::Value << MAG_FIG8_LOOPS_PER_SIDE;
      out << YAML::EndMap;
      out << YAML::EndMap;
      std::ofstream fh(MAG_CALIBRATION_PATH);
      if (!fh)
        throw std::runtime_error("cannot open mag_calibration.yaml");
      fh << out.c_str();
    }
    catch (const std::exception& exc)
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to write mag calibration to %s: %s",
                   MAG_CALIBRATION_PATH,
                   exc.what());
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Mag calibration: samples=%d  offset=(%+7.2f, %+7.2f, %+7.2f) µT  "
                "scale=(%.3f, %.3f, %.3f)  |B|=%.2f ± %.2f µT  (Earth range ≈ 25–"
                "65 µT)  saved to %s",
                cal.sample_count,
                cal.offset_x_uT,
                cal.offset_y_uT,
                cal.offset_z_uT,
                cal.scale_x,
                cal.scale_y,
                cal.scale_z,
                cal.magnitude_mean_uT,
                cal.magnitude_std_uT,
                MAG_CALIBRATION_PATH);
  }

  // ── BT-state helpers ────────────────────────────────────────────────
  bool call_hlc(int command, const char* label)
  {
    if (!hlc_client_->wait_for_service(std::chrono::seconds(3)))
    {
      RCLCPP_WARN(get_logger(), "HighLevelControl service not available — cannot %s.", label);
      return false;
    }
    auto req = std::make_shared<mowgli_interfaces::srv::HighLevelControl::Request>();
    req->command = static_cast<uint8_t>(command);
    auto future = hlc_client_->async_send_request(req);
    const double deadline = monotonic() + 5.0;
    while (rclcpp::ok() && monotonic() < deadline)
    {
      if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready)
        break;
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      RCLCPP_WARN(get_logger(), "HighLevelControl %s timed out.", label);
      return false;
    }
    auto resp = future.get();
    if (!resp || !resp->success)
    {
      RCLCPP_WARN(get_logger(), "HighLevelControl %s rejected by BT.", label);
      return false;
    }
    return true;
  }

  bool wait_for_bt_state(int target, double timeout_sec)
  {
    const double deadline = monotonic() + timeout_sec;
    while (monotonic() < deadline)
    {
      if (bt_state_ == target)
        return true;
      sleep_for(0.1);
    }
    return bt_state_ == target;
  }

  // ── Service handler ─────────────────────────────────────────────────
  void calibrate_cb(const std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Request> request,
                    std::shared_ptr<mowgli_interfaces::srv::CalibrateImuYaw::Response> response)
  {
    const bool mag_only = request->mag_only;
    const bool do_dock_yaw_calibration = is_charging_ && !mag_only;
    if (emergency_active_)
    {
      response->success = false;
      response->message =
          "Refusing to calibrate while an emergency is active/latched. "
          "Clear the emergency first.";
      return;
    }
    if (bt_state_ == HL_STATE_AUTONOMOUS)
    {
      response->success = false;
      response->message =
          "Refusing to calibrate while BT is AUTONOMOUS (mowing in "
          "progress). Stop mowing first (HOME command).";
      return;
    }

    bool need_exit_recording = false;
    if (bt_state_ != HL_STATE_RECORDING)
    {
      RCLCPP_INFO(get_logger(), "Entering RECORDING mode for calibration drive.");
      if (!call_hlc(HL_CMD_RECORD_AREA, "enter recording"))
      {
        response->success = false;
        response->message =
            "Could not enter RECORDING mode via BT. Check that the "
            "behavior_tree_node is alive.";
        return;
      }
      // 15 s budget: when the BT is starting from IDLE_DOCKED the outer
      // Fallback may be partway through a 0.5 s WaitForDuration in
      // IdleSequence, then has to evaluate every branch (Emergency,
      // BoundaryRecovery, CriticalBattery, PreFlightCheck) before
      // reaching RecordingSequence. 5 s was tight enough to fail
      // routinely once the fallback chain grew. Anything larger than
      // observed worst-case (~7 s) buys us margin without making real
      // BT failures hang the operator.
      if (!wait_for_bt_state(HL_STATE_RECORDING, 15.0))
      {
        call_hlc(HL_CMD_RECORD_CANCEL, "cancel after failed entry");
        response->success = false;
        char buf[160];
        std::snprintf(buf,
                      sizeof(buf),
                      "BT did not transition to RECORDING within 15s "
                      "(stuck at state=%d).",
                      bt_state_.load());
        response->message = buf;
        return;
      }
      need_exit_recording = true;
    }

    activate_sensor_subs();

    {
      std::lock_guard<std::mutex> lk(lock_);
      imu_samples_.clear();
      odom_samples_.clear();
      collecting_ = true;
    }

    // Dock-yaw is a "bonus" pre-phase: when the robot starts on the dock
    // we use the 2 m reverse to also derive dock_pose_yaw from the GPS
    // track. If that fails (no RTK fix, insufficient displacement, YAML
    // write error) we MUST still run the IMU calibration drives — the
    // robot has driven off the dock at this point and is in position
    // for the forward/back motion profile. Aborting here was the bug:
    // operators saw the robot back off the dock then silently return
    // to IDLE without ever calibrating the IMU.
    std::optional<DockYawResult> dock_yaw_result;
    std::string dock_yaw_warning;
    if (do_dock_yaw_calibration)
    {
      dock_yaw_result = run_dock_yaw_drive();
      if (!dock_yaw_result)
      {
        dock_yaw_warning =
            "Dock yaw pre-phase failed (no RTK Fixed, insufficient GPS "
            "displacement, or yaml write error) — continuing with IMU "
            "calibration drives anyway.";
        RCLCPP_WARN(get_logger(), "%s", dock_yaw_warning.c_str());
      }
    }

    int baseline_start_count = 0;
    if (!mag_only)
    {
      RCLCPP_INFO(get_logger(), "Capturing %.1fs stationary baseline before drive.", BASELINE_SEC);
      pause(BASELINE_SEC);

      {
        std::lock_guard<std::mutex> lk(lock_);
        baseline_start_count = static_cast<int>(imu_samples_.size());
      }

      const double total_motion_sec = 2.0 * (RAMP_SEC + CRUISE_SEC + RAMP_SEC) + PAUSE_SEC;
      RCLCPP_INFO(get_logger(),
                  "Autonomous calibration drive starting — forward %.2f m/s "
                  "then back, ~%.0fs total.",
                  CRUISE_SPEED,
                  total_motion_sec);
    }

    bool drive_ok = true;
    try
    {
      if (!mag_only)
      {
        for (int cycle = 0; cycle < N_CYCLES; ++cycle)
        {
          RCLCPP_INFO(get_logger(), "Drive cycle %d/%d", cycle + 1, N_CYCLES);
          drive_profile(+CRUISE_SPEED);
          pause(PAUSE_SEC);
          drive_profile(-CRUISE_SPEED);
          pause(SETTLE_SEC);
        }
      }
    }
    catch (const std::exception& exc)
    {
      drive_ok = false;
      for (int i = 0; i < 5; ++i)
      {
        publish_vx(0.0);
        sleep_for(0.05);
      }
      {
        std::lock_guard<std::mutex> lk(lock_);
        collecting_ = false;
      }
      if (need_exit_recording)
        call_hlc(HL_CMD_RECORD_CANCEL, "cancel after drive error");
      response->success = false;
      char buf[200];
      std::snprintf(buf, sizeof(buf), "Drive profile errored: %s", exc.what());
      response->message = buf;
      deactivate_sensor_subs();
      return;
    }
    publish_vx(0.0);
    (void)drive_ok;

    // Mag phase. Opt-in only: triggered exclusively by mag_only=true. The
    // previous "auto-run if /imu/mag_raw has any publisher" gate fired the
    // 30s figure-8 on every IMU yaw calibration on real hardware (the bridge
    // always publishes mag_raw), which surprised operators who only wanted
    // the dock-yaw + IMU pitch/roll drives. The GUI calls calibrate twice
    // when mag is wanted: once with mag_only=false for the IMU pass, once
    // with mag_only=true for the figure-8.
    const std::size_t mag_publisher_count = count_publishers("/imu/mag_raw");
    const bool do_mag_calibration = mag_only;
    if (mag_only && mag_publisher_count == 0)
    {
      RCLCPP_WARN(get_logger(),
                  "mag_only requested but no /imu/mag_raw publisher detected "
                  "— running the figure-8 anyway, but the fit will fail with "
                  "'too few samples'.");
    }

    if (do_mag_calibration)
    {
      const double wz_mag = MAG_FIG8_LINEAR_M_S / MAG_FIG8_RADIUS_M;
      const double total_sec =
          2.0 * (2.0 * M_PI * MAG_FIG8_LOOPS_PER_SIDE) / wz_mag + 2.0 * MAG_FIG8_PAUSE_SEC;
      RCLCPP_INFO(get_logger(),
                  "Magnetometer calibration: figure-8 — v=%.2f m/s, R=%.2f "
                  "m (wz=±%.2f rad/s), %.1f loops per side, ~%.0fs total. "
                  "Need ~1.5 m clear in front and behind the robot.",
                  MAG_FIG8_LINEAR_M_S,
                  MAG_FIG8_RADIUS_M,
                  wz_mag,
                  MAG_FIG8_LOOPS_PER_SIDE,
                  total_sec);
      {
        std::lock_guard<std::mutex> lk(lock_);
        mag_samples_.clear();
        collecting_mag_ = true;
      }
      try
      {
        figure_eight_profile(MAG_FIG8_LINEAR_M_S,
                             MAG_FIG8_RADIUS_M,
                             MAG_FIG8_LOOPS_PER_SIDE,
                             MAG_FIG8_PAUSE_SEC);
      }
      catch (const std::exception& exc)
      {
        for (int i = 0; i < 5; ++i)
        {
          publish_arc(0.0, 0.0);
          sleep_for(0.05);
        }
        RCLCPP_ERROR(get_logger(), "Mag figure-8 phase errored: %s", exc.what());
      }
      {
        std::lock_guard<std::mutex> lk(lock_);
        collecting_mag_ = false;
      }
      publish_arc(0.0, 0.0);
    }

    if (need_exit_recording)
    {
      call_hlc(HL_CMD_RECORD_CANCEL, "cancel recording");
      wait_for_bt_state(HL_STATE_IDLE, 3.0);
    }

    std::vector<std::tuple<double, double, double, double>> imu_snap;
    std::vector<std::tuple<double, double, double>> odom_snap;
    std::vector<std::tuple<double, double, double>> mag_snap;
    {
      std::lock_guard<std::mutex> lk(lock_);
      collecting_ = false;
      imu_snap = imu_samples_;
      odom_snap = odom_samples_;
      mag_snap = mag_samples_;
    }

    if (do_mag_calibration)
      fit_and_save_mag(mag_snap);

    RCLCPP_INFO(get_logger(),
                "Drive complete — %zu IMU / %zu odom samples collected.",
                imu_snap.size(),
                odom_snap.size());

    if (mag_only)
    {
      response->success = true;
      char buf[160];
      std::snprintf(buf,
                    sizeof(buf),
                    "mag_only calibration: %zu samples collected during figure-8. "
                    "IMU yaw / pitch / roll left untouched.",
                    mag_snap.size());
      response->message = buf;
    }
    else
    {
      ComputeResult result = compute_imu_yaw(imu_snap, odom_snap, baseline_start_count);
      response->success = result.success;
      response->message = result.message;
      response->imu_yaw_rad = result.imu_yaw_rad;
      response->imu_yaw_deg = result.imu_yaw_deg;
      response->samples_used = result.samples_used;
      response->std_dev_deg = result.std_dev_deg;
      response->imu_pitch_rad = result.imu_pitch_rad;
      response->imu_pitch_deg = result.imu_pitch_deg;
      response->imu_roll_rad = result.imu_roll_rad;
      response->imu_roll_deg = result.imu_roll_deg;
      response->stationary_samples_used = result.stationary_samples_used;
      response->gravity_mag_mps2 = result.gravity_mag_mps2;
    }

    if (dock_yaw_result)
    {
      response->dock_valid = true;
      response->dock_pose_x = dock_yaw_result->dock_pose_x;
      response->dock_pose_y = dock_yaw_result->dock_pose_y;
      response->dock_pose_yaw_rad = dock_yaw_result->dock_pose_yaw_rad;
      response->dock_pose_yaw_deg = dock_yaw_result->dock_pose_yaw_deg;
      response->dock_yaw_sigma_deg = dock_yaw_result->yaw_sigma_deg;
      response->dock_undock_displacement_m = dock_yaw_result->undock_displacement_m;
    }
    else
    {
      response->dock_valid = false;
      if (!dock_yaw_warning.empty() && !response->message.empty())
      {
        response->message += " | " + dock_yaw_warning;
      }
      else if (!dock_yaw_warning.empty())
      {
        response->message = dock_yaw_warning;
      }
    }

    if (response->success)
    {
      RCLCPP_INFO(get_logger(),
                  "imu_yaw = %+.4f rad (%+.2f°) from %d samples, stddev "
                  "%.2f°",
                  response->imu_yaw_rad,
                  response->imu_yaw_deg,
                  response->samples_used,
                  response->std_dev_deg);
      if (response->stationary_samples_used >= MIN_STATIONARY_SAMPLES)
      {
        RCLCPP_INFO(get_logger(),
                    "imu_pitch = %+.4f rad (%+.2f°), imu_roll = %+.4f rad (%+.2f°) "
                    "from %d stationary samples (|g|=%.3f m/s²). Promote to "
                    "mowgli_robot.yaml if |pitch| or |roll| > 1°.",
                    response->imu_pitch_rad,
                    response->imu_pitch_deg,
                    response->imu_roll_rad,
                    response->imu_roll_deg,
                    response->stationary_samples_used,
                    response->gravity_mag_mps2);
      }
      else
      {
        RCLCPP_WARN(get_logger(),
                    "Pitch/roll not computed: only %d stationary IMU samples "
                    "(need ≥ %d).",
                    response->stationary_samples_used,
                    MIN_STATIONARY_SAMPLES);
      }
      if (response->dock_valid)
      {
        RCLCPP_INFO(get_logger(),
                    "dock_pose = (%+.3f, %+.3f) yaw=%+.2f° (σ=%.2f°, undock "
                    "displacement=%.3f m). Promote to mowgli_robot.yaml → "
                    "dock_pose_yaw = %.4f.",
                    response->dock_pose_x,
                    response->dock_pose_y,
                    response->dock_pose_yaw_deg,
                    response->dock_yaw_sigma_deg,
                    response->dock_undock_displacement_m,
                    response->dock_pose_yaw_rad);
      }
    }
    else
    {
      // cppcheck-suppress invalidLifetime
      // (false positive: response->message is std::string, c_str() points
      //  into the string's owned storage — not into the earlier local buf)
      RCLCPP_WARN(get_logger(), "Calibration failed: %s", response->message.c_str());
    }

    deactivate_sensor_subs();
  }

  // ── Numerical core ──────────────────────────────────────────────────
  struct ComputeResult
  {
    bool success{false};
    std::string message;
    double imu_yaw_rad{0.0};
    double imu_yaw_deg{0.0};
    int samples_used{0};
    double std_dev_deg{0.0};
    double imu_pitch_rad{0.0};
    double imu_pitch_deg{0.0};
    double imu_roll_rad{0.0};
    double imu_roll_deg{0.0};
    int stationary_samples_used{0};
    double gravity_mag_mps2{0.0};
  };

  ComputeResult compute_imu_yaw(std::vector<std::tuple<double, double, double, double>> imu_samples,
                                std::vector<std::tuple<double, double, double>> odom_samples,
                                int baseline_count)
  {
    ComputeResult empty;

    std::vector<std::tuple<double, double, double, double>> baseline_samples;
    double baseline_ax = 0.0, baseline_ay = 0.0;
    if (baseline_count > 0 && static_cast<int>(imu_samples.size()) > baseline_count)
    {
      baseline_samples.assign(imu_samples.begin(), imu_samples.begin() + baseline_count);
      double sum_ax = 0.0, sum_ay = 0.0;
      for (const auto& s : baseline_samples)
      {
        sum_ax += std::get<1>(s);
        sum_ay += std::get<2>(s);
      }
      const double n = static_cast<double>(baseline_count);
      baseline_ax = sum_ax / n;
      baseline_ay = sum_ay / n;
      imu_samples.erase(imu_samples.begin(), imu_samples.begin() + baseline_count);
    }
    else if (baseline_count > 0)
    {
      baseline_samples.assign(imu_samples.begin(), imu_samples.end());
    }

    if (odom_samples.size() < 3)
    {
      char buf[160];
      std::snprintf(buf,
                    sizeof(buf),
                    "Not enough wheel_odom samples (%zu). Is /wheel_odom "
                    "being published?",
                    odom_samples.size());
      empty.message = buf;
      return empty;
    }
    if (static_cast<int>(imu_samples.size()) < MIN_SAMPLES)
    {
      char buf[160];
      std::snprintf(buf,
                    sizeof(buf),
                    "Not enough /imu/data samples (%zu). Is the IMU running?",
                    imu_samples.size());
      empty.message = buf;
      return empty;
    }

    std::sort(odom_samples.begin(),
              odom_samples.end(),
              [](const auto& a, const auto& b)
              {
                return std::get<0>(a) < std::get<0>(b);
              });
    std::sort(imu_samples.begin(),
              imu_samples.end(),
              [](const auto& a, const auto& b)
              {
                return std::get<0>(a) < std::get<0>(b);
              });

    const std::size_t no = odom_samples.size();
    std::vector<double> odom_t(no), odom_vx(no), odom_wz(no);
    for (std::size_t i = 0; i < no; ++i)
    {
      odom_t[i] = std::get<0>(odom_samples[i]);
      odom_vx[i] = std::get<1>(odom_samples[i]);
      odom_wz[i] = std::get<2>(odom_samples[i]);
    }

    if (no < 3)
    {
      empty.message = "Not enough odom samples for central difference.";
      return empty;
    }
    std::vector<double> a_body(no, 0.0);
    for (std::size_t i = 1; i + 1 < no; ++i)
    {
      const double dt = std::max(odom_t[i + 1] - odom_t[i - 1], 1e-6);
      a_body[i] = (odom_vx[i + 1] - odom_vx[i - 1]) / dt;
    }
    a_body[0] = a_body[1];
    a_body[no - 1] = a_body[no - 2];

    // Per-imu interpolation.
    const std::size_t ni = imu_samples.size();
    std::vector<double> wz_interp(ni), a_interp(ni), vx_interp(ni);
    for (std::size_t i = 0; i < ni; ++i)
    {
      const double t = std::get<0>(imu_samples[i]);
      auto it = std::lower_bound(odom_t.begin(), odom_t.end(), t);
      std::size_t idx = static_cast<std::size_t>(it - odom_t.begin());
      if (idx < 1)
        idx = 1;
      if (idx > no - 1)
        idx = no - 1;
      const std::size_t left = idx - 1;
      const std::size_t right = idx;
      const double dt = std::max(odom_t[right] - odom_t[left], 1e-9);
      double frac = (t - odom_t[left]) / dt;
      if (frac < 0.0)
        frac = 0.0;
      if (frac > 1.0)
        frac = 1.0;
      wz_interp[i] = odom_wz[left] + (odom_wz[right] - odom_wz[left]) * frac;
      a_interp[i] = a_body[left] + (a_body[right] - a_body[left]) * frac;
      vx_interp[i] = odom_vx[left] + (odom_vx[right] - odom_vx[left]) * frac;
    }

    std::vector<std::size_t> mask_idx;
    mask_idx.reserve(ni);
    for (std::size_t i = 0; i < ni; ++i)
    {
      if (std::abs(wz_interp[i]) < WZ_STRAIGHT_THRESHOLD &&
          std::abs(a_interp[i]) > ACCEL_BODY_THRESHOLD)
      {
        mask_idx.push_back(i);
      }
    }
    const int valid_count = static_cast<int>(mask_idx.size());
    if (valid_count < MIN_SAMPLES)
    {
      empty.samples_used = valid_count;
      char buf[200];
      std::snprintf(buf,
                    sizeof(buf),
                    "Only %d valid samples (need ≥ %d). Is the robot stuck, "
                    "colliding, or on uneven ground?",
                    valid_count,
                    MIN_SAMPLES);
      empty.message = buf;
      return empty;
    }

    double sum_ax = 0.0, sum_ay = 0.0, sum_abs = 0.0;
    std::vector<double> per_sample_yaw;
    per_sample_yaw.reserve(mask_idx.size());
    for (std::size_t k : mask_idx)
    {
      const double ax = std::get<1>(imu_samples[k]) - baseline_ax;
      const double ay = std::get<2>(imu_samples[k]) - baseline_ay;
      const double sgn = (a_interp[k] > 0.0) - (a_interp[k] < 0.0);
      const double ax_s = ax * sgn;
      const double ay_s = ay * sgn;
      sum_ax += ax_s;
      sum_ay += ay_s;
      sum_abs += std::hypot(ax_s, ay_s);
      per_sample_yaw.push_back(std::atan2(-ay_s, ax_s));
    }
    const double imu_yaw_rad = std::atan2(-sum_ay, sum_ax);
    double r_bar = (sum_abs > 1e-9) ? std::hypot(sum_ax, sum_ay) / sum_abs : 0.0;
    if (r_bar < 1e-9)
      r_bar = 1e-9;
    if (r_bar > 1.0)
      r_bar = 1.0;
    const double std_rad = std::sqrt(-2.0 * std::log(r_bar));

    double per_cos = 0.0, per_sin = 0.0;
    for (double y : per_sample_yaw)
    {
      per_cos += std::cos(y);
      per_sin += std::sin(y);
    }
    per_cos /= static_cast<double>(per_sample_yaw.size());
    per_sin /= static_cast<double>(per_sample_yaw.size());
    const double per_sample_r = std::hypot(per_cos, per_sin);

    // Pitch/roll from stationary samples.
    std::vector<double> all_ax, all_ay, all_az;
    for (const auto& s : baseline_samples)
    {
      all_ax.push_back(std::get<1>(s));
      all_ay.push_back(std::get<2>(s));
      all_az.push_back(std::get<3>(s));
    }
    for (std::size_t i = 0; i < ni; ++i)
    {
      if (std::abs(vx_interp[i]) < STATIONARY_VX_THRESHOLD &&
          std::abs(wz_interp[i]) < STATIONARY_WZ_THRESHOLD)
      {
        all_ax.push_back(std::get<1>(imu_samples[i]));
        all_ay.push_back(std::get<2>(imu_samples[i]));
        all_az.push_back(std::get<3>(imu_samples[i]));
      }
    }
    const int stationary_count = static_cast<int>(all_az.size());
    double pitch_rad = 0.0, roll_rad = 0.0, gravity_mag = 0.0;
    if (stationary_count >= MIN_STATIONARY_SAMPLES)
    {
      const double n = static_cast<double>(stationary_count);
      double sx = 0.0, sy = 0.0, sz = 0.0;
      for (int i = 0; i < stationary_count; ++i)
      {
        sx += all_ax[i];
        sy += all_ay[i];
        sz += all_az[i];
      }
      const double mx = sx / n, my = sy / n, mz = sz / n;
      pitch_rad = std::atan2(-mx, mz);
      roll_rad = std::atan2(my, mz);
      gravity_mag = std::sqrt(mx * mx + my * my + mz * mz);
    }

    char msg[400];
    int written =
        std::snprintf(msg,
                      sizeof(msg),
                      "imu_yaw=%+.2f° from %d samples (vector R=%.2f, per-sample R=%.2f).",
                      imu_yaw_rad * 180.0 / M_PI,
                      valid_count,
                      r_bar,
                      per_sample_r);
    if (stationary_count >= MIN_STATIONARY_SAMPLES && written > 0 &&
        written < static_cast<int>(sizeof(msg)))
    {
      std::snprintf(msg + written,
                    sizeof(msg) - written,
                    " pitch=%+.2f° roll=%+.2f° from %d stationary samples "
                    "(|g|=%.3f m/s²).",
                    pitch_rad * 180.0 / M_PI,
                    roll_rad * 180.0 / M_PI,
                    stationary_count,
                    gravity_mag);
    }

    ComputeResult out;
    out.success = true;
    out.message = msg;
    out.imu_yaw_rad = imu_yaw_rad;
    out.imu_yaw_deg = imu_yaw_rad * 180.0 / M_PI;
    out.samples_used = valid_count;
    out.std_dev_deg = std_rad * 180.0 / M_PI;
    out.imu_pitch_rad = pitch_rad;
    out.imu_pitch_deg = pitch_rad * 180.0 / M_PI;
    out.imu_roll_rad = roll_rad;
    out.imu_roll_deg = roll_rad * 180.0 / M_PI;
    out.stationary_samples_used = stationary_count;
    out.gravity_mag_mps2 = gravity_mag;
    return out;
  }

  // ── State ───────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::QoS imu_qos_{rclcpp::KeepLast(50)};
  rclcpp::QoS odom_qos_{rclcpp::KeepLast(50)};

  std::mutex lock_;
  std::atomic<bool> collecting_{false};
  std::atomic<bool> collecting_mag_{false};
  std::vector<std::tuple<double, double, double, double>> imu_samples_;
  std::vector<std::tuple<double, double, double>> odom_samples_;
  std::vector<std::tuple<double, double, double>> mag_samples_;

  std::atomic<bool> gps_have_{false};
  std::atomic<bool> gps_rtk_fixed_{false};
  std::atomic<float> gps_position_accuracy_{99.0f};
  std::atomic<double> latest_gps_x_{0.0}, latest_gps_y_{0.0};

  std::atomic<bool> is_charging_{false};
  std::atomic<bool> emergency_active_{false};
  std::atomic<int> bt_state_{HL_STATE_NULL};

  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr emergency_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::HighLevelStatus>::SharedPtr bt_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedPtr hlc_client_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Service<mowgli_interfaces::srv::CalibrateImuYaw>::SharedPtr srv_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mowgli_localization::CalibrateImuYawNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
