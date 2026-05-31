// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// costmap_scan_filter_node.cpp
//
// Conditional LiDAR preprocessor for the local_costmap and global_costmap
// obstacle layers. Two filters chained on /scan → /scan_costmap:
//
//   1. Dock-blank (stateful)
//      Returns within `dock_blank_range` get +inf'd while the robot is
//      charging or for `post_undock_blank_sec` after charging drops, so
//      the dock body doesn't show up as a LETHAL obstacle to the BackUp
//      recovery's collision checker (which reads local_costmap/costmap_raw).
//      Outside that window, near returns pass through and the existing
//      collision_monitor (which polls /scan unfiltered) handles real-time
//      contact safety.
//
//   2. Ground filter (gravity-aware, slope-tolerant)
//      robot_localization runs in two_d_mode (forces pitch=roll=0 in TF),
//      so on a sloped garden the LiDAR scan plane is physically tilted
//      but TF reports it as horizontal. laser_geometry::projectLaser then
//      projects every return at LIDAR_Z and the obstacle_layer's
//      min_obstacle_height filter does nothing — real ground returns at
//      1–2 m show up as walls and the planner refuses to drive.
//
//      Fix: project each beam using the gravity vector measured directly
//      by /imu/data linear_acceleration (which carries actual robot
//      pitch/roll). The IMU orientation field on this stack is currently
//      hardcoded identity by hardware_bridge — accel is the only signal
//      that knows we're tilted. The gravity ("up") vector lives in the
//      IMU/base_link frame, but a beam's index angle α is in the LIDAR
//      frame, which on this robot is yaw-mounted ~π (180°-rotated) from
//      base_link (mowgli_robot.yaml lidar_yaw). So α must be rotated into
//      the base/IMU frame before projecting onto gravity, via the
//      lidar_mount_yaw param (= lidar_yaw − imu_yaw): a beam at LIDAR
//      angle α points along base bearing ψ = α + lidar_mount_yaw. Skipping
//      this rotation flips the front/back sign on a pitched robot — the
//      forward ground returns (LIDAR α≈π on this mount) get a POSITIVE
//      z_dir and survive as phantom obstacles, while the empty sky-side
//      beams get "filtered". On flat ground (ux,uy≈0) the error is
//      invisible, which is why it only bites on a sloped garden.
//        ψ        = α + lidar_mount_yaw
//        z_dir    = (ax·cos ψ + ay·sin ψ) / |accel|
//        return_Z = lidar_height + range · z_dir
//      where (ax, ay, az) is the latest IMU linear_acceleration. A 10°
//      nose-down pitch gives ax ≈ −1.7 m/s² → forward beam z_dir ≈ −0.17
//      → return at 2 m projects to Z ≈ 0.22 − 0.35 = −0.13 m, below the
//      0.08 m floor → filtered. Real obstacles whose top sits above the
//      0.08 m floor keep returns in-band.
//
//      Outlier guard: if |accel| differs from g by more than
//      accel_g_tolerance_ms2 (default 3.0 m/s²), the sample is treated
//      as motion-dominated and the previous low-pass-filtered estimate
//      is used. The robot mows at ~0.2 m/s with low body acceleration,
//      so this is rarely needed but protects against bumps.
//
//      Falls back to pass-through if no IMU sample within `imu_max_age_s`
//      so we never silently strip obstacles when localization is sick.
//
// collision_monitor still subscribes to /scan unfiltered.

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "mowgli_interfaces/msg/status.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

class CostmapScanFilterNode : public rclcpp::Node
{
public:
  CostmapScanFilterNode() : Node("costmap_scan_filter")
  {
    dock_blank_range_ = declare_parameter<double>("dock_blank_range", 0.70);
    // Always-on radial blank used to suppress the robot's own chassis from
    // the LiDAR scan. Real hardware: LiDAR mount usually clears the
    // chassis, so 0 is fine. Sim/edge cases: any return inside this
    // range gets +inf'd before downstream consumers (collision_monitor,
    // costmap, etc.) see it.
    chassis_blank_range_ = declare_parameter<double>("chassis_blank_range", 0.0);
    post_undock_blank_sec_ = declare_parameter<double>("post_undock_blank_sec", 5.0);
    enable_ground_filter_ = declare_parameter<bool>("enable_ground_filter", true);
    min_obstacle_z_m_ = declare_parameter<double>("min_obstacle_z_m", 0.08);
    max_obstacle_z_m_ = declare_parameter<double>("max_obstacle_z_m", 1.5);
    lidar_height_m_ = declare_parameter<double>("lidar_height_m", 0.22);
    // Yaw of the LIDAR frame relative to the IMU/base_link frame
    // (= lidar_yaw − imu_yaw from mowgli_robot.yaml). Needed to rotate a
    // beam's index angle into the gravity frame before the ground
    // projection. Default 0 keeps the old (flat-mount) behaviour; the
    // launch passes the real ~π value for the 180°-rotated mount.
    lidar_mount_yaw_ = declare_parameter<double>("lidar_mount_yaw", 0.0);
    imu_max_age_s_ = declare_parameter<double>("imu_max_age_s", 0.5);
    accel_g_tolerance_ms2_ = declare_parameter<double>("accel_g_tolerance_ms2", 3.0);
    const std::string input_topic = declare_parameter<std::string>("input_topic", "/scan");
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/scan_costmap");
    const std::string status_topic =
        declare_parameter<std::string>("status_topic", "/hardware_bridge/status");
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");

    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
    qos_reliable.reliable();
    qos_reliable.durability_volatile();

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos_sensor);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic,
        qos_sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });

    sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
        status_topic,
        qos_reliable,
        [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg) { on_status(*msg); });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        qos_sensor,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });

    RCLCPP_INFO(get_logger(),
                "costmap_scan_filter started — %s -> %s, chassis_blank_range=%.2f m, "
                "dock_blank_range=%.2f m, post_undock_blank_sec=%.1f s, "
                "ground_filter=%s [Z range %.2f..%.2f m, lidar_height=%.2f m, "
                "lidar_mount_yaw=%.3f rad, imu_max_age=%.2f s, "
                "accel_g_tol=±%.2f m/s², source %s].",
                input_topic.c_str(),
                output_topic.c_str(),
                chassis_blank_range_,
                dock_blank_range_,
                post_undock_blank_sec_,
                enable_ground_filter_ ? "on" : "off",
                min_obstacle_z_m_,
                max_obstacle_z_m_,
                lidar_height_m_,
                lidar_mount_yaw_,
                imu_max_age_s_,
                accel_g_tolerance_ms2_,
                imu_topic.c_str());
  }

  static constexpr double kGravityMs2 = 9.80665;

  // --- Pure logic exposed for unit tests ---------------------------------

  /// Three-component vector — used for the gravity-aligned "up" direction
  /// expressed in the IMU/base_link frame.
  struct Vec3
  {
    double x{0.0};
    double y{0.0};
    double z{1.0};
  };

  /// Ground-filter parameters bundled together so the test can call the
  /// pure filter without a node.
  struct GroundFilterConfig
  {
    bool enabled{false};
    double min_obstacle_z_m{0.08};
    double max_obstacle_z_m{1.5};
    double lidar_height_m{0.22};
    /// Yaw of the LIDAR frame relative to base_link/IMU (rad). A beam at
    /// LIDAR index angle α points along base bearing α + lidar_mount_yaw.
    double lidar_mount_yaw{0.0};
  };

  /// Apply the radial blank to a copy of @p in. Returns the result.
  /// `blank_active` is the cached output of `is_blank_active()` — passed
  /// in so the test can drive the state machine without a clock.
  static sensor_msgs::msg::LaserScan filter_scan(const sensor_msgs::msg::LaserScan& in,
                                                 double dock_blank_range,
                                                 bool blank_active)
  {
    sensor_msgs::msg::LaserScan out = in;
    if (!blank_active)
      return out;
    const float threshold = static_cast<float>(dock_blank_range);
    const float inf = std::numeric_limits<float>::infinity();
    for (auto& r : out.ranges)
    {
      if (std::isfinite(r) && r < threshold)
        r = inf;
    }
    return out;
  }

  /// Apply the gravity-aware ground filter to @p io in place. For each
  /// beam at LIDAR index angle α, rotate into the base/IMU frame by the
  /// LIDAR mount yaw (ψ = α + lidar_mount_yaw) before projecting onto the
  /// IMU's measured "up" unit vector:
  ///
  ///     ψ        = α + cfg.lidar_mount_yaw
  ///     z_dir    = up_in_imu.x · cos ψ + up_in_imu.y · sin ψ
  ///     return_Z = lidar_height + range · z_dir
  ///
  /// where up_in_imu = accel / |accel| (the gravity reaction direction).
  /// Returns whose Z is outside [min_obstacle_z_m, max_obstacle_z_m] get
  /// pushed to +inf so obstacle_layer ignores them.
  ///
  /// `up_in_imu` is std::nullopt when no fresh sample exists (or the
  /// filter is disabled). In that case the function is a no-op — better
  /// to publish phantom obstacles than to silently strip real ones.
  static void apply_ground_filter(sensor_msgs::msg::LaserScan& io,
                                  const GroundFilterConfig& cfg,
                                  const std::optional<Vec3>& up_in_imu)
  {
    if (!cfg.enabled || !up_in_imu.has_value())
      return;
    const Vec3& u = *up_in_imu;
    const float min_z = static_cast<float>(cfg.min_obstacle_z_m);
    const float max_z = static_cast<float>(cfg.max_obstacle_z_m);
    const float inf = std::numeric_limits<float>::infinity();
    const double a0 = io.angle_min;
    const double da = io.angle_increment;
    for (size_t i = 0; i < io.ranges.size(); ++i)
    {
      float& r = io.ranges[i];
      if (!std::isfinite(r))
        continue;
      const double psi = a0 + da * static_cast<double>(i) + cfg.lidar_mount_yaw;
      const double z_dir = u.x * std::cos(psi) + u.y * std::sin(psi);
      const float return_z = static_cast<float>(cfg.lidar_height_m + r * z_dir);
      if (return_z < min_z || return_z > max_z)
        r = inf;
    }
  }

private:
  void on_status(const mowgli_interfaces::msg::Status& msg)
  {
    const bool is_charging = msg.is_charging;
    if (last_is_charging_known_ && last_is_charging_ && !is_charging)
    {
      // Falling edge — start the post-undock grace window.
      charging_dropped_at_ = now();
      RCLCPP_INFO(get_logger(),
                  "charging dropped — extending dock-blank for %.1f s",
                  post_undock_blank_sec_);
    }
    last_is_charging_ = is_charging;
    last_is_charging_known_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu& msg)
  {
    // Use linear_acceleration as the gravity vector. The IMU on this
    // stack publishes orientation as identity (hardware_bridge does no
    // attitude estimation), so accel is the only signal that knows the
    // robot's tilt. Treat as gravity reaction (points UP in IMU frame
    // when at rest).
    const double ax = msg.linear_acceleration.x;
    const double ay = msg.linear_acceleration.y;
    const double az = msg.linear_acceleration.z;
    const double mag = std::sqrt(ax * ax + ay * ay + az * az);
    if (mag < 1e-3)
      return;  // bogus sample, ignore

    // Outlier guard: at mowing speeds (≤0.3 m/s) the body acceleration
    // is small, so |accel| stays close to g. A bump or step on a curb
    // can push it well above. Treat samples outside ±accel_g_tolerance
    // as motion-dominated and keep the previous low-pass estimate so we
    // don't briefly trust a tilted-by-impulse reading.
    const double dev = std::fabs(mag - kGravityMs2);
    if (last_up_in_imu_.has_value() && dev > accel_g_tolerance_ms2_)
    {
      // Reject — keep the latched estimate alive (don't refresh stamp,
      // so a sustained outlier window will eventually expire via
      // imu_max_age_s_ and the filter will fall back to pass-through).
      return;
    }

    Vec3 up{ax / mag, ay / mag, az / mag};
    if (!last_up_in_imu_.has_value())
    {
      last_up_in_imu_ = up;
    }
    else
    {
      // 1st-order low-pass (α = 0.2): smooths instantaneous accel jitter
      // without lagging real tilt changes (mowing speed → tilt changes
      // over many scan periods anyway).
      const double a = 0.2;
      Vec3 prev = *last_up_in_imu_;
      Vec3 mixed{a * up.x + (1.0 - a) * prev.x,
                 a * up.y + (1.0 - a) * prev.y,
                 a * up.z + (1.0 - a) * prev.z};
      const double mmag = std::sqrt(mixed.x * mixed.x + mixed.y * mixed.y + mixed.z * mixed.z);
      if (mmag > 1e-6)
      {
        mixed.x /= mmag;
        mixed.y /= mmag;
        mixed.z /= mmag;
        last_up_in_imu_ = mixed;
      }
    }
    last_imu_stamp_ = now();
  }

  void on_scan(const sensor_msgs::msg::LaserScan& msg)
  {
    // Two-stage radial blank: chassis_blank_range_ is always on, then
    // dock_blank_range_ kicks in only while charging / immediately
    // post-undock. The `effective` blank range is the larger of the two
    // currently-active values, so a single pass through filter_scan
    // suffices.
    const bool dock_active = is_blank_active();
    const double effective_blank = std::max(
        chassis_blank_range_, dock_active ? dock_blank_range_ : 0.0);
    sensor_msgs::msg::LaserScan out = filter_scan(msg, effective_blank, effective_blank > 0.0);

    // Ground filter — only when we have a fresh IMU sample. Stale IMU →
    // pass-through so we never silently strip obstacles when localization
    // is sick (better to have phantom ground returns than to blind the
    // costmap entirely).
    std::optional<Vec3> up;
    if (enable_ground_filter_ && last_imu_stamp_.nanoseconds() != 0)
    {
      const double age = (now() - last_imu_stamp_).seconds();
      if (age >= 0.0 && age < imu_max_age_s_)
        up = last_up_in_imu_;
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "ground filter idle: last IMU sample %.2fs old (>%.2fs)",
                             age,
                             imu_max_age_s_);
      }
    }
    GroundFilterConfig cfg{enable_ground_filter_,
                           min_obstacle_z_m_,
                           max_obstacle_z_m_,
                           lidar_height_m_,
                           lidar_mount_yaw_};
    apply_ground_filter(out, cfg, up);

    pub_scan_->publish(out);
  }

  bool is_blank_active() const
  {
    if (!last_is_charging_known_)
    {
      // Be conservative until we've heard from hardware_bridge: keep the
      // dock blank in case we're booting docked.
      return true;
    }
    if (last_is_charging_)
      return true;
    if (charging_dropped_at_.nanoseconds() == 0)
      return false;
    const double since_drop = (now() - charging_dropped_at_).seconds();
    return since_drop >= 0.0 && since_drop < post_undock_blank_sec_;
  }

  // --- Subscriptions / publishers ---------------------------------------

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;

  // --- Parameters --------------------------------------------------------

  double dock_blank_range_{0.70};
  double chassis_blank_range_{0.0};
  double post_undock_blank_sec_{5.0};
  bool enable_ground_filter_{true};
  double min_obstacle_z_m_{0.08};
  double max_obstacle_z_m_{1.5};
  double lidar_height_m_{0.22};
  double lidar_mount_yaw_{0.0};
  double imu_max_age_s_{0.5};
  double accel_g_tolerance_ms2_{3.0};

  // --- Charging-state machine -------------------------------------------

  bool last_is_charging_{false};
  bool last_is_charging_known_{false};
  rclcpp::Time charging_dropped_at_{0, 0, RCL_ROS_TIME};

  // --- IMU latch (for ground filter) ------------------------------------

  /// Low-pass-filtered "up" direction in IMU frame, derived from the
  /// gravity component of linear_acceleration. Empty until first sample.
  std::optional<Vec3> last_up_in_imu_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CostmapScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
