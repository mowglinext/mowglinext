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
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_interfaces/action/calibrate_dock.hpp"
#include "mowgli_interfaces/motion_yaw_fit.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_interfaces/msg/dock_calibration_status.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/high_level_status.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_interfaces/srv/calibrate_imu_yaw.hpp"
#include "mowgli_interfaces/srv/high_level_control.hpp"
#include "mowgli_interfaces/srv/set_docking_point.hpp"
#include "mowgli_localization/dock_cog_gate.hpp"
#include "mowgli_localization/dock_persist_plan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"
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
constexpr int HL_CMD_HOME = 2;
constexpr int HL_CMD_RECORD_AREA = 3;
constexpr int HL_CMD_RECORD_CANCEL = 6;
constexpr int HL_CMD_STOP = 8;

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

// This node does NOT write dock_pose_x/y/yaw itself any more: every dock-pose
// write goes through map_server's ~/set_docking_point (Invariant 6 — ONE
// writer), for both the one-click calibration and the legacy ~/calibrate dock
// pre-phase. The pre-phase used to splice mowgli_robot.yaml directly with a
// position read from /gps/absolute_pose, a source that is pinned to the stored
// dock yaw while charging.

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
  // Default speed/distance are conservative fallbacks; the actual values are
  // read from the `undock_speed` / `undock_distance` ROS parameters so the
  // calibration drive matches what the operator configured for normal undocks.
  static constexpr double DOCK_UNDOCK_SPEED_DEFAULT = 0.15;
  static constexpr double DOCK_UNDOCK_DISTANCE_DEFAULT_M = 2.0;
  static constexpr double DOCK_UNDOCK_TIMEOUT_SEC = 25.0;
  // Re-dock leg: the return drive is DELEGATED to the production docking
  // pipeline (HL_CMD_HOME → Nav2 staging → SimpleChargingDock with charger
  // detection and its own retries). Three custom forward controllers were
  // field-tested 2026-08-01 (blind dead-reckoning, point-bearing P, Stanley
  // on the fused yaw) and each found a new failure mode, while the docking
  // server re-docked reliably all day — so the calibration only SUPERVISES:
  // an angle guard on the raw RTK track course aborts an askew final
  // approach, then a steered backoff re-lines the robot for a fresh attempt.
  static constexpr int REDOCK_MAX_ATTEMPTS = 3;
  static constexpr double REDOCK_NAV_TIMEOUT_SEC = 180.0;  // per HOME attempt (nav + dock)
  // Raw-track course estimate: bearing over the last ≥ this much travel. No
  // fused-yaw dependence — a startup-transient fusion bias steered run 5
  // consistently off target while keeping its own angle guard blind.
  static constexpr double REDOCK_COURSE_BASELINE_M = 0.20;
  static constexpr int REDOCK_APPROACH_TICKS_MIN = 6;  // guard arms only while closing in
  // Dock-protect angle guard: arriving askew must not shove the dock
  // sideways — abort the attempt (STOP → steered backoff → fresh HOME)
  // instead of shoving in at an angle. Not applied inside the cutoff radius,
  // where the wheel guides funnel the chassis and the heading swings by
  // design.
  static constexpr double REDOCK_ANGLE_GUARD_DIST_M = 0.6;
  static constexpr double REDOCK_ANGLE_GUARD_RAD = 0.20;  // ~11.5°
  static constexpr double REDOCK_STEER_CUTOFF_M = 0.35;
  // Steered backoff between attempts: reverse ~2 m while converging on the
  // ideal line (Stanley terms on the raw-track course; the cross-track term
  // flips sign in reverse — the TAIL must move toward the line).
  static constexpr double REDOCK_BACKOFF_M = 2.0;
  static constexpr double REDOCK_BACKOFF_TIMEOUT_SEC = 30.0;
  static constexpr double REDOCK_HDG_KP = 1.0;  // rad/s per rad of line-heading error
  static constexpr double REDOCK_XT_KP = 1.0;  // weight of the cross-track term
  static constexpr double REDOCK_XT_SHARPNESS = 4.0;  // atan(k·e): ~0.38 rad at e = 10 cm
  static constexpr double REDOCK_WZ_MAX = 0.10;  // rad/s clamp on the steer output
  // Give the firmware's charge-detection debounce a chance after an attempt.
  static constexpr double REDOCK_CHARGE_SETTLE_SEC = 8.0;
  // Persist retries: the write is RTK-accuracy gated, and a momentary
  // Fixed→Float flicker right after the reverse leg must not lose the run.
  static constexpr int PERSIST_MAX_ATTEMPTS = 8;
  static constexpr double PERSIST_RETRY_DELAY_SEC = 5.0;
  // On-dock antenna capture: map_server wants N RTK-Fixed samples taken WHILE
  // CHARGING inside its averaging window, and under the dock the receiver
  // alternates Fixed/Float. The robot is stationary on the dock, so waiting is
  // free — but bounded: after this the run continues yaw-only and says so.
  static constexpr double ANTENNA_CAPTURE_TIMEOUT_SEC = 90.0;
  static constexpr double ANTENNA_CAPTURE_RETRY_SEC = 2.0;

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

    // All calibration motion goes on /cmd_vel_docking (twist_mux priority 15)
    // so collision_monitor stays in the loop — NOT /cmd_vel_teleop (priority
    // 20), which bypasses it. Blade is never commanded here regardless.
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_docking", state_qos);

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

    dock_undock_distance_ =
        declare_parameter<double>("undock_distance", DOCK_UNDOCK_DISTANCE_DEFAULT_M);
    dock_undock_speed_ = declare_parameter<double>("undock_speed", DOCK_UNDOCK_SPEED_DEFAULT);

    // ── One-click dock calibration (CalibrateDock action) parameters ──
    // Defaults single-sourced from the mowgli_bringup template (Invariant 15);
    // the constants here are only the fallback when a param is absent.
    dc_reverse_distance_m_ = declare_parameter<double>("dock_calib_reverse_distance_m", 2.0);
    dc_reverse_speed_ms_ = declare_parameter<double>("dock_calib_reverse_speed_ms", 0.15);
    dc_redock_overshoot_m_ = declare_parameter<double>("dock_calib_redock_overshoot_m", 0.30);
    dc_rtk_wait_timeout_s_ = declare_parameter<double>("dock_calib_rtk_wait_timeout_s", 10.0);
    dc_redock_charge_timeout_s_ =
        declare_parameter<double>("dock_calib_redock_charge_timeout_s", 30.0);
    dc_cog_min_samples_ = declare_parameter<int>("dock_calib_cog_min_samples", 8);
    // Per-sample COG yaw scatter is dominated by the publisher's 0.10 m
    // baseline: even under RTK-Fixed (σ_pos 2-3 cm) individual samples spread
    // ~10-19° in theory and 10-27° std as field-measured across straight 2 m
    // reverses, so a few-degree std gate can NEVER pass and even ~17° trips
    // on ordinary condition-to-condition variation. This gate only screens
    // gross incoherence (RTK-Float wander); heading QUALITY is enforced by
    // the bearing-match gate below, which checks the circular MEAN against
    // the GPS displacement bearing.
    dc_cog_std_max_rad_ = declare_parameter<double>("dock_calib_cog_std_max_rad", 0.70);
    // Sanity cross-check only (the persisted yaw IS the reversed RTK travel
    // bearing, see dock_cog_gate.hpp): with ~16 COG samples of 10-19°
    // per-sample scatter the mean carries a 3-4° standard error, so a
    // few-degree match gate trips on pure noise. 0.35 rad (~20°) still
    // catches a dead/stale COG feed or an RTK-Float excursion.
    dc_cog_bearing_match_max_rad_ =
        declare_parameter<double>("dock_calib_cog_bearing_match_max_rad", 0.35);
    dc_min_baseline_disp_m_ =
        declare_parameter<double>("dock_calib_min_baseline_displacement_m", 0.5);

    // COG body-heading feed (already lever-arm-corrected + reverse-aware —
    // Resolution A: consume as-is, NO +pi). Sampled only while the reverse
    // leg is active (collecting_cog_).
    cog_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/cog_heading",
        imu_qos_,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
        {
          cog_cb(std::move(msg));
        },
        sub_opts);

    // The ONE canonical dock_pose writer (Invariant 6): persist via map_server.
    set_dock_client_ =
        create_client<mowgli_interfaces::srv::SetDockingPoint>("/map_server_node/set_docking_point",
                                                               rclcpp::ServicesQoS(),
                                                               cb_group_);

    // On-dock RAW antenna capture (held in map_server memory until the motion
    // yaw exists). map_server owns the measurement AND its gates.
    capture_antenna_client_ =
        create_client<std_srvs::srv::Trigger>("/map_server_node/capture_dock_antenna",
                                              rclcpp::ServicesQoS(),
                                              cb_group_);

    dock_action_ = rclcpp_action::create_server<CalibrateDock>(
        this,
        "~/calibrate_dock",
        [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const CalibrateDock::Goal> goal)
        {
          return dock_handle_goal(uuid, goal);
        },
        [this](const std::shared_ptr<DockGoalHandle> gh)
        {
          return dock_handle_cancel(gh);
        },
        [this](const std::shared_ptr<DockGoalHandle> gh)
        {
          dock_handle_accepted(gh);
        },
        rcl_action_server_get_default_options(),
        cb_group_);

    // Foxglove-friendly façade (the GUI transport has no ROS-action support):
    // a non-blocking start service + a live status topic mirroring the action.
    status_pub_ =
        create_publisher<mowgli_interfaces::msg::DockCalibrationStatus>("~/dock_calibration/status",
                                                                        state_qos);
    dock_start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/dock_calibration/start",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
        {
          dock_start_cb(req, res);
        },
        rclcpp::ServicesQoS(),
        cb_group_);

    RCLCPP_INFO(get_logger(),
                "Dock calibration node ready. Dock undock: %.2f m @ %.2f m/s. "
                "One-click: ~/calibrate_dock (action) or ~/dock_calibration/start "
                "(service, GUI) → ~/dock_calibration/status. Legacy IMU/mag: "
                "~/calibrate (CalibrateImuYaw service).",
                dock_undock_distance_,
                dock_undock_speed_);
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
    // x0/y0 (from /gps/absolute_pose) only anchor the reverse leg's
    // displacement + line fit. They are NOT the dock position: that topic is
    // lever-arm-corrected with the fused yaw, which on the dock is gauge-pinned
    // to the STORED dock_pose_yaw (and silently degrades to the raw antenna if
    // TF is late) — a circular, unflagged source. The position comes from
    // map_server's raw-antenna capture, taken here while still on the dock.
    const double x0 = latest_gps_x_;
    const double y0 = latest_gps_y_;
    const auto should_abort = [this]()
    {
      return emergency_active_.load();
    };
    std::string capture_err;
    const bool have_capture = capture_dock_antenna_with_wait(
        should_abort,
        [this](const std::string& why)
        {
          RCLCPP_INFO_THROTTLE(get_logger(),
                               *get_clock(),
                               10000,
                               "Dock yaw calibration: waiting for the on-dock antenna capture: %s",
                               why.c_str());
        },
        capture_err);
    if (!have_capture)
    {
      RCLCPP_WARN(get_logger(),
                  "Dock yaw calibration: no on-dock antenna capture (%s) — the dock POSITION "
                  "will not be updated (yaw only).",
                  capture_err.c_str());
    }

    // Minimum displacement threshold: 75 % of the configured target so a
    // short undock_distance (e.g. 0.8 m) does not make the check trivially
    // impossible due to GPS noise near the dock. Capped at 0.8 m.
    const double min_displacement = std::min(0.8, dock_undock_distance_ * 0.75);

    RCLCPP_INFO(get_logger(),
                "RTK-Fixed acquired. Reversing at %+.2f m/s target %.1f m "
                "(min %.2f m, timeout %.0f s) — start pos=(%+.3f, %+.3f).",
                dock_undock_speed_,
                dock_undock_distance_,
                min_displacement,
                DOCK_UNDOCK_TIMEOUT_SEC,
                x0,
                y0);

    // Buffer GPS samples across the whole reverse (task #47) so the heading
    // can be fit through the entire trajectory instead of just its two
    // endpoints — see the line-fit block after the reverse below. Drop
    // samples that haven't moved at least 5 cm since the last kept one so a
    // near-stationary tail doesn't bloat the buffer with noise (mirrors
    // CalibrateHeadingFromUndock's buffering in behavior_tree_node.cpp).
    std::vector<std::pair<double, double>> samples;
    samples.emplace_back(x0, y0);

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
      publish_vx(-dock_undock_speed_);
      if (gps_have_)
      {
        const double dx = latest_gps_x_ - x0;
        const double dy = latest_gps_y_ - y0;
        displacement = std::hypot(dx, dy);
        const double sdx = latest_gps_x_ - samples.back().first;
        const double sdy = latest_gps_y_ - samples.back().second;
        if ((sdx * sdx + sdy * sdy) >= (0.05 * 0.05))
        {
          samples.emplace_back(latest_gps_x_, latest_gps_y_);
        }
        if (displacement >= dock_undock_distance_)
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

    if (displacement < min_displacement)
    {
      RCLCPP_ERROR(get_logger(),
                   "Only %.3f m of GPS displacement after reverse "
                   "(need ≥ %.2f m). Wheels probably slipped on the dock "
                   "ramp. Aborting.",
                   displacement,
                   min_displacement);
      return std::nullopt;
    }

    // Task #47: prefer a total-least-squares line fit through the WHOLE
    // buffered reverse trajectory over the endpoint-only atan2(-dy,-dx)
    // below. An endpoint-only bearing assumes the reverse was perfectly
    // straight; any curvature (e.g. yaw-loop wiggle before #37/#39) biases
    // that single atan2 by a few degrees with no way to detect it, and
    // fusion_graph's own dock unary factor (σ floor 0.035 rad, ~2°) then
    // holds the whole session to that bias — enough to miss the ~0.4 m
    // docking corridor over a ~1.5 m approach. The line fit is robust to
    // curvature and gives a tighter σ_yaw from the SAME baseline (σ shrinks
    // ~1/√n instead of being limited to the two endpoints). Same gate as
    // CalibrateHeadingFromUndock (calibration_nodes.cpp): >=4 samples
    // spanning >=0.5 m; below that, fall back to the endpoint bearing (the
    // sample buffer's own span may be short even if the tracked
    // start/end displacement above cleared min_displacement, e.g. sparse
    // GPS arrivals near dock_undock_speed_'s low end).
    const bool have_line_fit_samples =
        samples.size() >= 4u && std::hypot(samples.back().first - samples.front().first,
                                           samples.back().second - samples.front().second) >= 0.5;
    double dock_yaw;
    double sigma_yaw_rad;
    const char* yaw_method = "endpoint";
    if (have_line_fit_samples)
    {
      const auto [motion_yaw, motion_sigma] =
          mowgli_interfaces::motion_yaw_fit::FitMotionYaw(samples);
      // Robot heading points opposite the motion (the maneuver reverses).
      dock_yaw = motion_yaw + M_PI;
      while (dock_yaw > M_PI)
        dock_yaw -= 2.0 * M_PI;
      while (dock_yaw < -M_PI)
        dock_yaw += 2.0 * M_PI;
      sigma_yaw_rad = std::max(motion_sigma, 0.001);  // floor at ~0.06°
      yaw_method = "line_fit";
    }
    else
    {
      dock_yaw = std::atan2(-dy, -dx);
      const double sigma_pos = std::max(static_cast<double>(gps_position_accuracy_.load()), 0.003);
      sigma_yaw_rad = std::atan2(2.0 * sigma_pos, displacement);
    }

    // Persist through map_server — the ONE dock-pose writer (Invariant 6). This
    // node used to splice mowgli_robot.yaml itself with x0/y0; see the note at
    // x0 for why that position source had to go.
    std::string persist_err;
    const DockSavedPose saved = persist_dock_pose(
        dock_yaw,
        have_capture,
        capture_err,
        should_abort,
        [this](const std::string& what)
        {
          RCLCPP_INFO(get_logger(), "Dock yaw calibration: %s", what.c_str());
        },
        persist_err);
    if (saved.saved == DockSaved::NOTHING)
    {
      RCLCPP_ERROR(get_logger(),
                   "Dock yaw calibration: dock pose NOT saved: %s",
                   persist_err.c_str());
      return std::nullopt;
    }

    DockYawResult result;
    result.dock_pose_x = saved.x;
    result.dock_pose_y = saved.y;
    result.dock_pose_yaw_rad = dock_yaw;
    result.dock_pose_yaw_deg = dock_yaw * 180.0 / M_PI;
    result.undock_displacement_m = displacement;
    result.yaw_sigma_rad = sigma_yaw_rad;
    result.yaw_sigma_deg = sigma_yaw_rad * 180.0 / M_PI;
    result.speed_ms = dock_undock_speed_;

    RCLCPP_INFO(get_logger(),
                "Dock yaw calibration: reverse start=(%+.3f, %+.3f) end=(%+.3f, %+.3f) "
                "displacement=%.3f m method=%s (n=%zu) dock_yaw=%+.2f° (σ=%.2f°). Saved via "
                "map_server: %s%s%s.%s",
                x0,
                y0,
                x1,
                y1,
                displacement,
                yaw_method,
                samples.size(),
                result.dock_pose_yaw_deg,
                result.yaw_sigma_deg,
                DescribeDockSaved(saved).c_str(),
                saved.saved == DockSaved::YAW_ONLY ? " — position NOT updated: " : "",
                saved.saved == DockSaved::YAW_ONLY ? saved.position_skip_reason.c_str() : "",
                DockRestartNote());
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

  // ═══ One-click dock calibration (CalibrateDock action, Resolution A) ═══
  // The dock yaw is circular_mean(/imu/cog_heading over the reverse leg) with
  // NO +pi (the topic is already the reverse-aware BODY heading). The sequence
  // reverses on /cmd_vel_docking, gates COG coherence, persists via
  // map_server's set_docking_point (yaw_source=MOTION) — the single canonical
  // writer — then attempts a supervised re-dock forward as a CONFIRMATION
  // pass (see the persist block's comment for why the write can no longer
  // wait on that pass: docking_server only reads dock_pose at container
  // startup, so a same-session re-dock always steers on the OLD heading).
  // Blade is never commanded; every drive loop zero-vels on emergency/cancel.
  using CalibrateDock = mowgli_interfaces::action::CalibrateDock;
  using DockGoalHandle = rclcpp_action::ServerGoalHandle<CalibrateDock>;
  using DockStatus = mowgli_interfaces::msg::DockCalibrationStatus;

  // Result of the IMU-yaw numerical solve. Declared here (before DockOutcome,
  // which embeds a ComputeResult) so the type is visible at that point.
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

  // Outcome of one calibration run, returned by run_dock_calibration_core so
  // the action wrapper can build a CalibrateDock::Result and the service façade
  // can ignore it (it reports via the status topic instead).
  struct DockOutcome
  {
    bool success{false};
    bool canceled{false};
    uint8_t retry_reason{0};
    std::string message;
    bool gate_valid{false};
    DockCogGateResult gate{};
    bool imu_valid{false};
    ComputeResult imu{};
    DockSavedPose saved{};
  };

  void cog_cb(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    if (!collecting_cog_.load())
      return;
    const auto& q = msg->orientation;
    const double yaw =
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    std::lock_guard<std::mutex> lk(cog_lock_);
    cog_samples_.push_back(yaw);
  }

  // Publish the foxglove-friendly status topic (the GUI has no action support).
  void publish_status(uint8_t phase,
                      float progress,
                      float displacement,
                      bool running,
                      bool success,
                      uint8_t retry_reason,
                      const std::string& message)
  {
    DockStatus s;
    s.phase = phase;
    s.progress = progress;
    s.displacement_m = displacement;
    s.charging = is_charging_.load();
    s.running = running;
    {
      std::lock_guard<std::mutex> lk(cog_lock_);
      s.cog_std_deg = static_cast<float>(circular_std(cog_samples_) * 180.0 / M_PI);
    }
    s.success = success;
    s.retry_reason = retry_reason;
    s.message = message;
    status_pub_->publish(s);
  }

  // ── Action wrapper: run the core on a thread, map DockOutcome → Result ──
  rclcpp_action::GoalResponse dock_handle_goal(const rclcpp_action::GoalUUID&,
                                               std::shared_ptr<const CalibrateDock::Goal>)
  {
    if (dock_action_busy_.exchange(true))
    {
      RCLCPP_WARN(get_logger(), "CalibrateDock: already running — rejecting new goal.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse dock_handle_cancel(const std::shared_ptr<DockGoalHandle>)
  {
    RCLCPP_WARN(get_logger(), "CalibrateDock: cancel requested — will stop and hold.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void dock_handle_accepted(const std::shared_ptr<DockGoalHandle> gh)
  {
    std::thread(
        [this, gh]()
        {
          const auto goal = gh->get_goal();
          if (goal->include_mag)
          {
            // Mag calibration needs an in-place figure-8 rotation, which would
            // scramble the straight-line re-dock geometry — so it is NOT folded
            // into the dock sequence. No-op it here with a clear pointer to the
            // legacy path (~/calibrate with mag_only=true).
            RCLCPP_WARN(get_logger(),
                        "CalibrateDock: include_mag ignored — mag calibration needs rotation "
                        "that would break the straight re-dock. Use ~/calibrate (mag_only=true).");
          }
          const DockOutcome out = run_dock_calibration_core(goal->include_imu_yaw,
                                                            [gh]()
                                                            {
                                                              return gh->is_canceling();
                                                            });
          auto res = std::make_shared<CalibrateDock::Result>();
          res->success = out.success;
          res->retry_reason = out.retry_reason;
          res->message = out.message;
          if (out.gate_valid)
          {
            res->dock_pose_yaw_rad = out.gate.dock_yaw_rad;
            res->dock_pose_yaw_deg = out.gate.dock_yaw_rad * 180.0 / M_PI;
            res->cog_std_deg = out.gate.cog_std_rad * 180.0 / M_PI;
            res->reverse_displacement_m = out.gate.displacement_m;
            // What map_server STORED — never this node's own GPS estimate.
            res->dock_pose_x = out.saved.x;
            res->dock_pose_y = out.saved.y;
          }
          if (out.imu_valid)
          {
            res->imu_yaw_valid = true;
            res->imu_yaw_rad = out.imu.imu_yaw_rad;
            res->imu_yaw_deg = out.imu.imu_yaw_deg;
            res->imu_pitch_rad = out.imu.imu_pitch_rad;
            res->imu_roll_rad = out.imu.imu_roll_rad;
          }
          if (out.canceled)
            gh->canceled(res);
          else if (out.success)
            gh->succeed(res);
          else
            gh->abort(res);
        })
        .detach();
  }

  // ── Service façade: NON-BLOCKING start for the GUI (foxglove transport). ──
  // Returns immediately (accepted / rejected); the run proceeds on a thread and
  // reports live phase/progress + the terminal outcome on ~/dock_calibration/
  // status. include_imu_yaw is off here (the GUI screen calibrates the dock; the
  // imu-yaw fold is an action/CLI opt-in whose result the status topic omits).
  void dock_start_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (emergency_active_)
    {
      res->success = false;
      res->message = "Emergency active/latched — clear it, then retry.";
      return;
    }
    if (bt_state_ == HL_STATE_AUTONOMOUS)
    {
      res->success = false;
      res->message = "Robot is mowing (AUTONOMOUS). Send HOME first, then retry.";
      return;
    }
    if (!is_charging_)
    {
      res->success = false;
      res->message = "Robot is not on the dock (not charging). Dock it, then retry.";
      return;
    }
    if (dock_action_busy_.exchange(true))
    {
      res->success = false;
      res->message = "A dock calibration is already running.";
      return;
    }
    std::thread(
        [this]()
        {
          run_dock_calibration_core(/*include_imu_yaw=*/false,
                                    []()
                                    {
                                      return false;
                                    });
        })
        .detach();
    res->success = true;
    res->message = "Dock calibration started — watch ~/dock_calibration/status.";
  }

  // One set_docking_point call. `plan` is one of the requests of
  // dock_persist_plan.hpp. On failure `err` carries map_server's own rejection
  // text; on success `warn` is non-empty when map_server applied the pose but
  // could not write the yaml file, and `stored` is the pose map_server holds.
  bool persist_dock_via_map_server(const DockPersistRequest& plan,
                                   std::string& err,
                                   std::string& warn,
                                   geometry_msgs::msg::Pose& stored)
  {
    using SetDockReq = mowgli_interfaces::srv::SetDockingPoint::Request;
    static_assert(kSetDockYawPreserve == SetDockReq::PRESERVE);
    static_assert(kSetDockYawMotion == SetDockReq::MOTION);
    static_assert(kDockRetryNone == CalibrateDock::Result::RETRY_NONE);
    static_assert(kDockRetryNoChargeOnRedock == CalibrateDock::Result::RETRY_NO_CHARGE_ON_REDOCK);
    static_assert(kDockRetryPersistFailed == CalibrateDock::Result::RETRY_PERSIST_FAILED);
    warn.clear();
    if (!set_dock_client_->wait_for_service(std::chrono::seconds(3)))
    {
      err = "map_server set_docking_point service unavailable";
      return false;
    }
    auto req = std::make_shared<SetDockReq>();
    req->use_gps_position = plan.use_gps_position;
    req->preserve_position = plan.preserve_position;
    req->use_pending_antenna = plan.use_pending_antenna;
    req->yaw_source = plan.yaw_source;
    req->yaw_rad = plan.yaw_rad;  // motion-derived chassis heading
    auto fut = set_dock_client_->async_send_request(req);
    const double deadline = monotonic() + 8.0;
    while (rclcpp::ok() && monotonic() < deadline)
    {
      if (fut.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready)
        break;
    }
    if (fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      err = "set_docking_point timed out after 8 s (map_server busy or hung)";
      return false;
    }
    auto resp = fut.get();
    if (!resp)
    {
      err = "set_docking_point returned no response";
      return false;
    }
    if (!resp->success)
    {
      err = "set_docking_point rejected: " +
            (resp->message.empty() ? std::string("(no reason given)") : resp->message);
      return false;
    }
    warn = resp->message;
    stored = resp->stored_pose;
    return true;
  }

  // One ~/capture_dock_antenna call (no retry). False + `err` on rejection.
  bool call_capture_dock_antenna(std::string& err)
  {
    if (!capture_antenna_client_->wait_for_service(std::chrono::seconds(3)))
    {
      err = "map_server capture_dock_antenna service unavailable";
      return false;
    }
    auto fut = capture_antenna_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    const double deadline = monotonic() + 8.0;
    while (rclcpp::ok() && monotonic() < deadline)
    {
      if (fut.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready)
        break;
    }
    if (fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      err = "capture_dock_antenna timed out after 8 s (map_server busy or hung)";
      return false;
    }
    auto resp = fut.get();
    if (!resp || !resp->success)
    {
      err = (resp && !resp->message.empty()) ? resp->message : "capture_dock_antenna failed";
      return false;
    }
    RCLCPP_INFO(get_logger(), "Dock calibration: %s", resp->message.c_str());
    return true;
  }

  // Capture the RAW antenna position while the robot is still seated on the
  // dock, charging and RTK-Fixed — the ONE moment of the routine where all
  // three hold (after the reverse there is no dock; after the re-dock there is
  // often no Fixed, and the re-dock itself depends on the very position this
  // fixes). Never derived from the fused pose or /gps/absolute_pose: on the
  // dock those are pinned to the STORED dock pose. Waits, bounded, for
  // map_server's sample window to fill. False + `err` = continue yaw-only.
  bool capture_dock_antenna_with_wait(const std::function<bool()>& should_abort,
                                      const std::function<void(const std::string&)>& on_wait,
                                      std::string& err)
  {
    const double deadline = monotonic() + ANTENNA_CAPTURE_TIMEOUT_SEC;
    while (rclcpp::ok())
    {
      if (should_abort())
      {
        err = "aborted while capturing the dock antenna position";
        return false;
      }
      if (!is_charging_)
      {
        err = "charging was lost before the dock antenna position could be captured";
        return false;
      }
      if (call_capture_dock_antenna(err))
        return true;
      if (monotonic() >= deadline)
      {
        err = "no on-dock antenna capture within " +
              std::to_string(static_cast<int>(ANTENNA_CAPTURE_TIMEOUT_SEC)) + " s — last: " + err;
        return false;
      }
      on_wait(err);
      const double t_retry = monotonic() + ANTENNA_CAPTURE_RETRY_SEC;
      while (rclcpp::ok() && monotonic() < t_retry && !should_abort())
        sleep_for(1.0 / CMD_RATE_HZ);
    }
    err = "shutting down";
    return false;
  }

  // One persistence request with retries. True once map_server accepted AND
  // wrote it; on failure `perr` holds the LAST rejection reason.
  bool persist_with_retries(const DockPersistRequest& plan,
                            const std::string& what,
                            const std::function<bool()>& should_abort,
                            const std::function<void(const std::string&)>& on_retry,
                            std::string& perr,
                            geometry_msgs::msg::Pose& stored)
  {
    for (int attempt = 0; attempt < PERSIST_MAX_ATTEMPTS && rclcpp::ok(); ++attempt)
    {
      if (attempt > 0)
      {
        on_retry(what + " (retrying: " + perr + ")");
        const double t_retry = monotonic() + PERSIST_RETRY_DELAY_SEC;
        while (rclcpp::ok() && monotonic() < t_retry && !should_abort())
          sleep_for(1.0 / CMD_RATE_HZ);
      }
      if (should_abort())
      {
        perr = "aborted while saving (cancel or emergency)";
        return false;
      }
      std::string warn;
      if (persist_dock_via_map_server(plan, perr, warn, stored))
      {
        if (!warn.empty())
        {
          // Applied in map_server's memory but NOT written to the yaml file:
          // for a calibration that is a failure — it would be gone at the
          // next restart, which is exactly when it is first used.
          perr = warn;
          return false;
        }
        return true;
      }
      RCLCPP_WARN(get_logger(),
                  "Dock calibration: %s attempt %d/%d failed: %s",
                  what.c_str(),
                  attempt + 1,
                  PERSIST_MAX_ATTEMPTS,
                  perr.c_str());
    }
    return false;
  }

  // Persist the dock pose through the ONE canonical writer (map_server), robot
  // OFF the dock, motion yaw in hand. Normal: yaw + the pending on-dock antenna
  // in one write. Fallback (no capture, or map_server refused it): yaw only,
  // stored position kept. `err` is set only when NOTHING was saved.
  DockSavedPose persist_dock_pose(double dock_yaw_rad,
                                  bool have_capture,
                                  const std::string& capture_err,
                                  const std::function<bool()>& should_abort,
                                  const std::function<void(const std::string&)>& on_status,
                                  std::string& err)
  {
    DockSavedPose saved;
    saved.yaw_rad = dock_yaw_rad;
    geometry_msgs::msg::Pose stored;
    std::string perr;
    if (have_capture)
    {
      on_status("saving dock yaw + position");
      if (persist_with_retries(DockPoseStep(dock_yaw_rad),
                               "saving dock yaw + position",
                               should_abort,
                               on_status,
                               perr,
                               stored))
      {
        saved.saved = DockSaved::YAW_AND_POSITION;
        saved.x = stored.position.x;
        saved.y = stored.position.y;
        return saved;
      }
      saved.position_skip_reason = perr;
      if (should_abort())
      {
        err = perr;
        return saved;
      }
    }
    else
    {
      saved.position_skip_reason = capture_err;
    }
    on_status("saving dock yaw only");
    if (persist_with_retries(DockYawStep(dock_yaw_rad),
                             "saving dock yaw only",
                             should_abort,
                             on_status,
                             perr,
                             stored))
    {
      saved.saved = DockSaved::YAW_ONLY;
      saved.x = stored.position.x;
      saved.y = stored.position.y;
      return saved;
    }
    err = perr + " (position: " + saved.position_skip_reason + ")";
    return saved;
  }

  static uint8_t cog_reason_to_retry(DockCogReason r)
  {
    switch (r)
    {
      case DockCogReason::INSUFFICIENT_DISPLACEMENT:
        return CalibrateDock::Result::RETRY_INSUFFICIENT_DISPLACEMENT;
      case DockCogReason::INSUFFICIENT_SAMPLES:
      case DockCogReason::HIGH_STD:
      case DockCogReason::BEARING_MISMATCH:
      default:
        return CalibrateDock::Result::RETRY_COG_INCOHERENT;
    }
  }

  // Core sequence shared by the action wrapper and the service façade. Publishes
  // DockCalibrationStatus at every phase and returns the outcome. is_canceled()
  // lets the action honour cancel requests (the service passes a constant false).
  DockOutcome run_dock_calibration_core(bool include_imu_yaw,
                                        const std::function<bool()>& is_canceled)
  {
    bool need_exit_recording = false;
    DockSavedPose saved_pose;  // what reached mowgli_robot.yaml so far

    // Terminal-state + cleanup helper. Runs on EVERY exit path: stops the robot,
    // exits RECORDING, deactivates subs, publishes the terminal status, clears
    // the busy flag, LOGS the outcome and returns it. The dock pose is persisted
    // separately (yaw after the reverse leg, position after the verified
    // re-dock) — this never writes it.
    auto finish = [&](bool success,
                      uint8_t reason,
                      const std::string& msg,
                      bool canceled,
                      const DockCogGateResult* gate,
                      const ComputeResult* imu) -> DockOutcome
    {
      publish_vx(0.0);
      collecting_cog_ = false;
      {
        std::lock_guard<std::mutex> lk(lock_);
        collecting_ = false;
      }
      if (need_exit_recording)
      {
        call_hlc(HL_CMD_RECORD_CANCEL, "exit recording");
        wait_for_bt_state(HL_STATE_IDLE, 3.0);
      }
      deactivate_sensor_subs();
      DockOutcome out;
      out.success = success;
      out.canceled = canceled;
      out.retry_reason = reason;
      out.message = msg;
      out.saved = saved_pose;
      if (gate)
      {
        out.gate_valid = true;
        out.gate = *gate;
      }
      if (imu && imu->success)
      {
        out.imu_valid = true;
        out.imu = *imu;
      }
      publish_status(DockStatus::PHASE_DONE,
                     success ? 1.0f : 0.0f,
                     0.0f,
                     /*running=*/false,
                     success,
                     reason,
                     msg);
      // EVERY terminal outcome goes to the ROS log too. The status topic is
      // VOLATILE: a failure used to leave no trace at all in `docker logs`,
      // which is what hid a deterministic persist rejection for two days.
      if (success && reason == CalibrateDock::Result::RETRY_NONE)
      {
        RCLCPP_INFO(get_logger(), "Dock calibration SUCCEEDED: %s", msg.c_str());
      }
      else if (success)
      {
        // Saved, but the operator has something to do (robot not on the dock).
        RCLCPP_WARN(get_logger(),
                    "Dock calibration SUCCEEDED WITH A WARNING (reason=%u): %s",
                    static_cast<unsigned>(reason),
                    msg.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(),
                    "Dock calibration %s (retry_reason=%u): %s",
                    canceled ? "CANCELED" : "FAILED",
                    static_cast<unsigned>(reason),
                    msg.c_str());
      }
      dock_action_busy_ = false;
      return out;
    };

    // ── Guards ──
    if (emergency_active_)
    {
      return finish(false,
                    CalibrateDock::Result::RETRY_EMERGENCY,
                    "Emergency active/latched — clear it, then retry.",
                    false,
                    nullptr,
                    nullptr);
    }
    if (bt_state_ == HL_STATE_AUTONOMOUS)
    {
      return finish(false,
                    CalibrateDock::Result::RETRY_WRONG_STATE,
                    "Robot is mowing (AUTONOMOUS). Send HOME first, then retry.",
                    false,
                    nullptr,
                    nullptr);
    }
    if (!is_charging_)
    {
      return finish(false,
                    CalibrateDock::Result::RETRY_WRONG_STATE,
                    "Robot is not on the dock (not charging). Dock it, then retry.",
                    false,
                    nullptr,
                    nullptr);
    }

    // ── (1) Wait for RTK-Fixed ──
    // Subscribe BEFORE the wait: gps_rtk_fixed_ is only fed by the
    // /gps/absolute_pose callback that activate_sensor_subs() creates (and
    // finish() tears down after every run) — without a live subscription the
    // wait polls a flag nothing updates and always times out.
    activate_sensor_subs();
    gps_rtk_fixed_ = false;  // require a fresh fix, not a stale one from a prior run
    publish_status(
        DockStatus::PHASE_WAIT_RTK, 0.05f, 0.0f, true, false, 0, "waiting for RTK-Fixed");
    if (!wait_for_rtk_fixed(dc_rtk_wait_timeout_s_))
    {
      return finish(false,
                    CalibrateDock::Result::RETRY_NO_RTK,
                    "No RTK-Fixed within timeout — wait for a fix, then retry.",
                    false,
                    nullptr,
                    nullptr);
    }

    // ── (2) Capture the dock ANTENNA position, NOW: seated, charging, RTK-Fixed.
    // See capture_dock_antenna_with_wait(). A failed capture does not abort
    // the run — the yaw is still worth measuring — it downgrades it to a
    // yaw-only save, reported as such.
    const auto should_abort = [&]()
    {
      return is_canceled() || emergency_active_.load();
    };
    std::string capture_err;
    publish_status(DockStatus::PHASE_CAPTURE_POSITION,
                   0.08f,
                   0.0f,
                   true,
                   false,
                   0,
                   "capturing the dock position (on the dock, RTK-Fixed)");
    const bool have_capture = capture_dock_antenna_with_wait(
        should_abort,
        [&](const std::string& why)
        {
          publish_status(DockStatus::PHASE_CAPTURE_POSITION,
                         0.08f,
                         0.0f,
                         true,
                         false,
                         0,
                         "waiting for the dock position capture: " + why);
        },
        capture_err);
    if (should_abort())
    {
      return finish(false,
                    emergency_active_ ? CalibrateDock::Result::RETRY_EMERGENCY
                                      : CalibrateDock::Result::RETRY_WRONG_STATE,
                    "Aborted on the dock before moving; nothing was changed.",
                    is_canceled(),
                    nullptr,
                    nullptr);
    }
    if (!have_capture)
    {
      RCLCPP_WARN(get_logger(),
                  "Dock calibration: no on-dock antenna capture (%s) — continuing, the dock "
                  "POSITION will not be updated (yaw only).",
                  capture_err.c_str());
    }

    // Enter RECORDING so the BT stands down (BoundaryGuard etc. exempt).
    if (bt_state_ != HL_STATE_RECORDING)
    {
      if (!call_hlc(HL_CMD_RECORD_AREA, "enter recording") ||
          !wait_for_bt_state(HL_STATE_RECORDING, 15.0))
      {
        return finish(false,
                      CalibrateDock::Result::RETRY_WRONG_STATE,
                      "Could not enter RECORDING via BT.",
                      false,
                      nullptr,
                      nullptr);
      }
      need_exit_recording = true;
    }

    // ── (3) Straight reverse, collecting COG (+ IMU/odom accel if folding) ──
    const double x0 = latest_gps_x_.load();
    const double y0 = latest_gps_y_.load();
    {
      std::lock_guard<std::mutex> lk(cog_lock_);
      cog_samples_.clear();
    }
    if (include_imu_yaw)
    {
      std::lock_guard<std::mutex> lk(lock_);
      imu_samples_.clear();
      odom_samples_.clear();
      collecting_ = true;
    }
    collecting_cog_ = true;

    const double period = 1.0 / CMD_RATE_HZ;
    double disp = 0.0;
    {
      const double t_deadline = monotonic() + DOCK_UNDOCK_TIMEOUT_SEC;
      while (rclcpp::ok())
      {
        if (is_canceled())
        {
          return finish(false,
                        CalibrateDock::Result::RETRY_WRONG_STATE,
                        "Canceled during reverse.",
                        true,
                        nullptr,
                        nullptr);
        }
        if (emergency_active_)
        {
          return finish(false,
                        CalibrateDock::Result::RETRY_EMERGENCY,
                        "Emergency during reverse.",
                        false,
                        nullptr,
                        nullptr);
        }
        publish_vx(-dc_reverse_speed_ms_);
        sleep_for(period);
        disp = std::hypot(latest_gps_x_.load() - x0, latest_gps_y_.load() - y0);
        publish_status(DockStatus::PHASE_REVERSING,
                       0.10f + 0.30f * static_cast<float>(
                                           std::min(1.0,
                                                    disp / std::max(dc_reverse_distance_m_, 1e-3))),
                       static_cast<float>(disp),
                       true,
                       false,
                       0,
                       "reversing (blade off)");
        if (disp >= dc_reverse_distance_m_ || monotonic() > t_deadline)
          break;
      }
    }
    publish_vx(0.0);
    collecting_cog_ = false;
    {
      std::lock_guard<std::mutex> lk(lock_);
      collecting_ = false;  // stop accel collection at the end of the reverse leg
    }
    const double x1 = latest_gps_x_.load();
    const double y1 = latest_gps_y_.load();

    // ── (7) COG-coherence gate + dock yaw (Resolution A: circular_mean, NO +pi) ──
    publish_status(DockStatus::PHASE_CHECK_COG,
                   0.45f,
                   static_cast<float>(disp),
                   true,
                   false,
                   0,
                   "checking COG coherence");
    std::vector<double> cog_snap;
    {
      std::lock_guard<std::mutex> lk(cog_lock_);
      cog_snap = cog_samples_;
    }
    const DockCogGateResult gate =
        evaluate_dock_cog_gate(cog_snap,
                               x1 - x0,
                               y1 - y0,
                               static_cast<std::size_t>(std::max(0, dc_cog_min_samples_)),
                               dc_cog_std_max_rad_,
                               dc_cog_bearing_match_max_rad_,
                               dc_min_baseline_disp_m_);
    if (!gate.coherent)
    {
      return finish(false,
                    cog_reason_to_retry(gate.reason),
                    "COG incoherent (RTK not truly fixed / GPS noisy) — retry.",
                    false,
                    &gate,
                    nullptr);
    }

    // Optional IMU-yaw fold from the same straight leg. Pitch/roll need a
    // stationary baseline the dock reverse does not provide, so compute_imu_yaw
    // may report them as skipped (imu_yaw itself comes from the accel segments).
    ComputeResult imu_result;
    bool have_imu = false;
    if (include_imu_yaw)
    {
      std::vector<std::tuple<double, double, double, double>> imu_snap;
      std::vector<std::tuple<double, double, double>> odom_snap;
      {
        std::lock_guard<std::mutex> lk(lock_);
        imu_snap = imu_samples_;
        odom_snap = odom_samples_;
      }
      imu_result = compute_imu_yaw(imu_snap, odom_snap, 0);
      have_imu = imu_result.success;
    }

    // ── Persist — ONE write, right now, with the robot OFF the dock: the motion
    //    yaw + the antenna captured on the dock in step (2). map_server (the ONE
    //    canonical writer) lever-arm-corrects that antenna with THIS yaw and
    //    stores x, y and yaw together. No capture → yaw only, position kept.
    //    See dock_persist_plan.hpp for why it is neither after the re-dock
    //    (94f01b38: docking_server only reads dock_pose at container STARTUP, so
    //    a same-session re-dock steers on the OLD pose and vouches for nothing —
    //    the COG-coherence gate above is what validates the yaw) nor split
    //    around it (circular: a wrong stored position makes the re-dock stop
    //    short, so the position capture that would fix it is never reached).
    publish_status(DockStatus::PHASE_PERSIST, 0.55f, 0.0f, true, false, 0, "saving dock pose");
    {
      std::string perr;
      saved_pose = persist_dock_pose(
          gate.dock_yaw_rad,
          have_capture,
          capture_err,
          should_abort,
          [&](const std::string& what)
          {
            publish_status(DockStatus::PHASE_PERSIST, 0.55f, 0.0f, true, false, 0, what);
          },
          perr);
      if (saved_pose.saved == DockSaved::NOTHING)
      {
        return finish(false,
                      CalibrateDock::Result::RETRY_PERSIST_FAILED,
                      "Dock pose NOT saved: " + perr +
                          " Nothing was changed; the robot is off the dock — send HOME.",
                      is_canceled(),
                      &gate,
                      have_imu ? &imu_result : nullptr);
      }
      RCLCPP_INFO(get_logger(),
                  "Dock calibration: saved %s.",
                  DescribeDockSaved(saved_pose).c_str());
    }
    // From here on every abort must say what is ALREADY on disk.
    const std::string yaw_saved_note = DockSavedNote(saved_pose);

    // ── (4) Re-dock via the production docking pipeline, supervised — this
    //    is a pure CONFIRMATION pass: everything is already saved, nothing is
    //    measured after it. It steers on the OLD dock pose (docking_server
    //    loaded it at startup), so with a wrong old position it may stop short
    //    of the contacts — reported, never a calibration failure. ──
    publish_status(DockStatus::PHASE_REDOCKING, 0.60f, 0.0f, true, false, 0, "re-docking");
    {
      // Line geometry shared by the guard and the steered backoff.
      const double ux = std::cos(gate.dock_yaw_rad);
      const double uy = std::sin(gate.dock_yaw_rad);

      // HOME needs the BT idle (the reverse leg ran under RECORDING).
      if (need_exit_recording)
      {
        call_hlc(HL_CMD_RECORD_CANCEL, "exit recording before HOME");
        wait_for_bt_state(HL_STATE_IDLE, 5.0);
        need_exit_recording = false;
      }

      for (int attempt = 0; attempt < REDOCK_MAX_ATTEMPTS && rclcpp::ok(); ++attempt)
      {
        if (!call_hlc(HL_CMD_HOME, "dock via Nav2"))
        {
          return finish(false,
                        CalibrateDock::Result::RETRY_WRONG_STATE,
                        "Could not start the Nav2 docking (HOME rejected)." + yaw_saved_note,
                        false,
                        &gate,
                        nullptr);
        }

        // Supervise the approach: success = charging; abort = askew near the
        // dock (angle guard on the RAW RTK track course — no fused-yaw
        // dependence); failure = BT back to IDLE without charge, or timeout.
        const double deadline = monotonic() + REDOCK_NAV_TIMEOUT_SEC;
        double seg_x = latest_gps_x_.load();
        double seg_y = latest_gps_y_.load();
        double course = std::numeric_limits<double>::quiet_NaN();
        double prev_to_dock = std::hypot(x0 - seg_x, y0 - seg_y);
        int approaching_ticks = 0;
        bool saw_autonomous = false;
        while (rclcpp::ok() && monotonic() < deadline)
        {
          if (is_canceled())
          {
            call_hlc(HL_CMD_STOP, "stop HOME on cancel");
            return finish(false,
                          CalibrateDock::Result::RETRY_WRONG_STATE,
                          "Canceled during re-dock." + yaw_saved_note,
                          true,
                          &gate,
                          nullptr);
          }
          if (emergency_active_)
          {
            return finish(false,
                          CalibrateDock::Result::RETRY_EMERGENCY,
                          "Emergency during re-dock." + yaw_saved_note,
                          false,
                          &gate,
                          nullptr);
          }
          if (is_charging_)
            break;

          const double cx = latest_gps_x_.load();
          const double cy = latest_gps_y_.load();
          if (std::hypot(cx - seg_x, cy - seg_y) >= REDOCK_COURSE_BASELINE_M)
          {
            course = std::atan2(cy - seg_y, cx - seg_x);
            seg_x = cx;
            seg_y = cy;
          }
          const double to_dock = std::hypot(x0 - cx, y0 - cy);
          approaching_ticks = (to_dock < prev_to_dock - 1e-4) ? approaching_ticks + 1 : 0;
          prev_to_dock = to_dock;
          if (bt_state_ == HL_STATE_AUTONOMOUS)
            saw_autonomous = true;

          // Angle guard: only while genuinely closing in on the dock (the
          // staging maneuver may pass nearby on an arbitrary heading).
          if (to_dock < REDOCK_ANGLE_GUARD_DIST_M && to_dock > REDOCK_STEER_CUTOFF_M &&
              approaching_ticks >= REDOCK_APPROACH_TICKS_MIN && std::isfinite(course) &&
              std::abs(wrap_angle(gate.dock_yaw_rad - course)) > REDOCK_ANGLE_GUARD_RAD)
          {
            RCLCPP_WARN(get_logger(),
                        "Re-dock: track course %.1f° off the dock line at %.2f m — "
                        "stopping for a steered backoff instead of shoving in askew.",
                        std::abs(wrap_angle(gate.dock_yaw_rad - course)) * 180.0 / M_PI,
                        to_dock);
            call_hlc(HL_CMD_STOP, "abort askew approach");
            wait_for_bt_state(HL_STATE_IDLE, 5.0);
            break;
          }

          // BT finished (back to IDLE) without a charge signal → this attempt
          // failed on the docking server's side.
          if (saw_autonomous && bt_state_ == HL_STATE_IDLE)
            break;

          publish_status(DockStatus::PHASE_REDOCKING,
                         0.60f + 0.25f * static_cast<float>(std::max(
                                             0.0, 1.0 - to_dock / DOCK_UNDOCK_DISTANCE_DEFAULT_M)),
                         static_cast<float>(to_dock),
                         true,
                         false,
                         0,
                         "re-docking (Nav2)");
          sleep_for(period);
        }
        if (monotonic() >= deadline)
          call_hlc(HL_CMD_STOP, "stop HOME on timeout");

        // Give the firmware's charge-detection debounce (~2-3 s ADC) a chance
        // before deciding this attempt failed.
        {
          const double t_settle = monotonic() + REDOCK_CHARGE_SETTLE_SEC;
          while (rclcpp::ok() && !is_charging_ && monotonic() < t_settle)
            sleep_for(period);
        }
        if (is_charging_ || attempt + 1 >= REDOCK_MAX_ATTEMPTS)
          break;

        // ── Steered backoff: reverse ~2 m converging on the ideal line so
        //    the next HOME starts from a clean, lined-up spot. Runs under
        //    RECORDING so the BT stands down while we publish cmd_vel. ──
        publish_status(DockStatus::PHASE_REDOCKING,
                       0.60f,
                       0.0f,
                       true,
                       false,
                       0,
                       "re-docking (steered backoff)");
        if (!call_hlc(HL_CMD_RECORD_AREA, "enter recording for backoff") ||
            !wait_for_bt_state(HL_STATE_RECORDING, 15.0))
        {
          return finish(false,
                        CalibrateDock::Result::RETRY_WRONG_STATE,
                        "Could not enter RECORDING for the backoff." + yaw_saved_note,
                        false,
                        &gate,
                        nullptr);
        }
        const double bx = latest_gps_x_.load();
        const double by = latest_gps_y_.load();
        double bseg_x = bx;
        double bseg_y = by;
        double bcourse = std::numeric_limits<double>::quiet_NaN();
        const double back_deadline = monotonic() + REDOCK_BACKOFF_TIMEOUT_SEC;
        while (rclcpp::ok() && !emergency_active_ && !is_canceled() && monotonic() < back_deadline)
        {
          const double cx = latest_gps_x_.load();
          const double cy = latest_gps_y_.load();
          if (std::hypot(cx - bx, cy - by) >= REDOCK_BACKOFF_M)
            break;
          if (std::hypot(cx - bseg_x, cy - bseg_y) >= REDOCK_COURSE_BASELINE_M)
          {
            bcourse = std::atan2(cy - bseg_y, cx - bseg_x);
            bseg_x = cx;
            bseg_y = cy;
          }
          double wz = 0.0;
          if (std::isfinite(bcourse))
          {
            // Reversing: the track course points backwards, so the chassis
            // heading is course + pi. The cross-track term flips sign vs the
            // forward case — the TAIL must move toward the line.
            const double heading = wrap_angle(bcourse + M_PI);
            const double hdg_err = wrap_angle(gate.dock_yaw_rad - heading);
            const double e_xt = ux * (cy - y0) - uy * (cx - x0);
            const double xt_term = +std::atan(REDOCK_XT_SHARPNESS * e_xt);
            wz =
                std::max(-REDOCK_WZ_MAX,
                         std::min(REDOCK_WZ_MAX, REDOCK_HDG_KP * hdg_err + REDOCK_XT_KP * xt_term));
          }
          publish_arc(-dc_reverse_speed_ms_, wz);
          sleep_for(period);
        }
        publish_vx(0.0);
        call_hlc(HL_CMD_RECORD_CANCEL, "exit recording after backoff");
        wait_for_bt_state(HL_STATE_IDLE, 5.0);
      }
    }
    publish_vx(0.0);

    // ── (5) Verify charging (give the charger handshake a few seconds) ──
    publish_status(
        DockStatus::PHASE_VERIFY_CHARGE, 0.85f, 0.0f, true, false, 0, "verifying charge");
    {
      const double t_verify = monotonic() + REDOCK_CHARGE_SETTLE_SEC;
      while (rclcpp::ok() && !is_charging_ && monotonic() < t_verify)
        sleep_for(period);
    }
    const DockVerdict verdict = DecideDockVerdict(saved_pose, is_charging_.load());
    return finish(verdict.success,
                  verdict.retry_reason,
                  verdict.message,
                  false,
                  &gate,
                  have_imu ? &imu_result : nullptr);
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
  double dock_undock_distance_{DOCK_UNDOCK_DISTANCE_DEFAULT_M};
  double dock_undock_speed_{DOCK_UNDOCK_SPEED_DEFAULT};

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

  // ── One-click dock calibration (CalibrateDock action) params + state ──
  double dc_reverse_distance_m_{2.0};
  double dc_reverse_speed_ms_{0.15};
  double dc_redock_overshoot_m_{0.30};
  double dc_rtk_wait_timeout_s_{10.0};
  double dc_redock_charge_timeout_s_{30.0};
  int dc_cog_min_samples_{8};
  double dc_cog_std_max_rad_{0.0524};
  double dc_cog_bearing_match_max_rad_{0.1047};
  double dc_min_baseline_disp_m_{0.5};

  std::mutex cog_lock_;
  std::vector<double> cog_samples_;
  std::atomic<bool> collecting_cog_{false};
  std::atomic<bool> dock_action_busy_{false};

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr cog_sub_;
  rclcpp::Client<mowgli_interfaces::srv::SetDockingPoint>::SharedPtr set_dock_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_antenna_client_;
  rclcpp_action::Server<mowgli_interfaces::action::CalibrateDock>::SharedPtr dock_action_;
  rclcpp::Publisher<mowgli_interfaces::msg::DockCalibrationStatus>::SharedPtr status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dock_start_srv_;
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
