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
//      Outlier guard: if |accel| differs from the active gravity baseline
//      by more than accel_g_tolerance_ms2 (default 3.0 m/s²), the sample
//      is treated as motion-dominated and the previous low-pass-filtered
//      estimate is used. A plausible out-of-band vector can replace a stale
//      baseline only after remaining mutually consistent for five seconds;
//      this recovers from a stable sensor offset without blessing a transient.
//      Samples outside 0.5g..1.5g never qualify for recovery.
//
//      Falls back to pass-through if no IMU sample within `imu_max_age_s`
//      so we never silently strip obstacles when localization is sick.
//
// collision_monitor subscribes to /scan_collision — the self-return-blanked
// stream WITHOUT the ground filter (so a slope-stripped obstacle still trips the
// near-field hard stop), NOT raw /scan and NOT the ground-filtered /scan_costmap.
//
//   3. LiDAR-ignore corridor (operator opt-in, map-anchored)
//      An operator-drawn line (mowgli_map's LidarIgnoreCorridorEntry,
//      streamed on /mowgli/lidar_ignore_corridors) marks a stretch — e.g. a
//      hedge the recorded boundary intentionally runs along — where LiDAR
//      returns within width_m/2 of the line should stop being treated as an
//      obstacle. Unlike the two filters above, this one is applied to BOTH
//      /scan_costmap AND /scan_collision: a deliberate, explicit operator
//      choice (confirmed 2026-09-25) that a drawn corridor can make
//      collision_monitor's near-field hard stop blind too, not just
//      FTC/Nav2 planning. The corridor's placement and width_m become the
//      only thing standing between the robot and whatever is physically
//      there inside it — map_server_node clamps width_m to a sane bound,
//      but accuracy is otherwise entirely on the operator's drawn line.
//      Requires the robot's CURRENT pose in map frame
//      (/odometry/filtered_map, the same fused pose mowgli_hardware's dig
//      detector trusts) to project each beam; a stale pose falls back to
//      pass-through, same rule as the ground filter's stale-IMU case — never
//      silently suppress LiDAR near a corridor while the robot doesn't know
//      where it actually is.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mowgli_interfaces/msg/lidar_ignore_corridor_array.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "mowgli_localization/gravity_estimator.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
    lidar_height_m_ = declare_parameter<double>("lidar_height_m", 0.30);
    // Yaw of the LIDAR frame relative to the IMU/base_link frame
    // (= lidar_yaw − imu_yaw from mowgli_robot.yaml). Needed to rotate a
    // beam's index angle into the gravity frame before the ground
    // projection. Default 0 keeps the old (flat-mount) behaviour; the
    // launch passes the real ~π value for the 180°-rotated mount.
    lidar_mount_yaw_ = declare_parameter<double>("lidar_mount_yaw", 0.0);
    min_ground_run_ = declare_parameter<int>("min_ground_run", 8);
    imu_max_age_s_ = declare_parameter<double>("imu_max_age_s", 0.5);
    accel_g_tolerance_ms2_ = declare_parameter<double>("accel_g_tolerance_ms2", 3.0);
    // LiDAR-ignore corridor filter geometry — see the header comment's item 3.
    // x/y are the SAME lidar_x/lidar_y the URDF uses (mowgli_robot.yaml); the
    // ground filter above never needed the position, only lidar_mount_yaw.
    lidar_x_m_ = declare_parameter<double>("lidar_x_m", 0.0);
    lidar_y_m_ = declare_parameter<double>("lidar_y_m", 0.0);
    corridor_pose_max_age_s_ = declare_parameter<double>("corridor_pose_max_age_s", 1.0);
    GravityEstimatorConfig gravity_estimator_config;
    gravity_estimator_config.accel_g_tolerance_ms2 = accel_g_tolerance_ms2_;
    gravity_estimator_config.candidate_max_gap_s = imu_max_age_s_;
    gravity_estimator_ = GravityEstimator(gravity_estimator_config);
    const std::string input_topic = declare_parameter<std::string>("input_topic", "/scan");
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/scan_costmap");
    // Self-return-blanked but NOT ground-filtered stream for collision_monitor's
    // near-field hard stop (empty disables the second publisher).
    const std::string collision_output_topic =
        declare_parameter<std::string>("collision_output_topic", "/scan_collision");
    const std::string status_topic =
        declare_parameter<std::string>("status_topic", "/hardware_bridge/status");
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "/imu/data");
    const std::string corridors_topic =
        declare_parameter<std::string>("lidar_ignore_corridors_topic",
                                       "/mowgli/lidar_ignore_corridors");
    const std::string odom_topic =
        declare_parameter<std::string>("odometry_topic", "/odometry/filtered_map");

    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
    qos_reliable.reliable();
    qos_reliable.durability_volatile();
    // Always-latest, transient_local — a late-starting subscriber (this node
    // restarting) gets the current corridor list without a service round-trip,
    // same shape as map_server_node's /keepout_mask.
    rclcpp::QoS qos_corridors(rclcpp::KeepLast(1));
    qos_corridors.transient_local();

    pub_scan_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos_sensor);
    if (!collision_output_topic.empty())
    {
      pub_collision_scan_ =
          create_publisher<sensor_msgs::msg::LaserScan>(collision_output_topic, qos_sensor);
    }

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic,
        qos_sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
        {
          on_scan(*msg);
        });

    sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
        status_topic,
        qos_reliable,
        [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
        {
          on_status(*msg);
        });

    sub_imu_ =
        create_subscription<sensor_msgs::msg::Imu>(imu_topic,
                                                   qos_sensor,
                                                   [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
                                                   {
                                                     on_imu(*msg);
                                                   });

    sub_corridors_ = create_subscription<mowgli_interfaces::msg::LidarIgnoreCorridorArray>(
        corridors_topic,
        qos_corridors,
        [this](mowgli_interfaces::msg::LidarIgnoreCorridorArray::ConstSharedPtr msg)
        {
          on_corridors(*msg);
        });

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic,
        qos_sensor,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          on_odom(*msg);
        });

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
    double lidar_height_m{0.30};
    /// Yaw of the LIDAR frame relative to base_link/IMU (rad). A beam at
    /// LIDAR index angle α points along base bearing α + lidar_mount_yaw.
    double lidar_mount_yaw{0.0};
    /// SAFETY: minimum run of consecutive ground-classified beams before any of
    /// them is stripped as ground. A 2-D LiDAR can't tell a real vertical
    /// obstacle from sloped ground at the same bearing/range — on a downslope a
    /// leg/trunk/child projects BELOW min_obstacle_z and would be discarded. But
    /// ground returns form LONG contiguous angular arcs while an obstacle
    /// subtends only a few beams, so we only strip a "ground" return when it is
    /// part of a run >= this length. A short ground-classified cluster is kept
    /// (treated as a possible obstacle) — the planner/CostCritic then avoids it.
    /// 0 disables the guard (legacy per-beam stripping).
    int min_ground_run{8};
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
    const size_t n = io.ranges.size();

    // Pass 1: classify each finite beam. 0 = keep, 1 = ground (projects below
    // min_z), 2 = overhead (above max_z). The overhead strip is per-beam (a
    // canopy/overhang return is genuinely above the robot and safe to drop); the
    // GROUND strip is gated below by a run-length test so a real obstacle that a
    // downslope mis-projects below min_z is not silently discarded.
    std::vector<uint8_t> klass(n, 0);
    for (size_t i = 0; i < n; ++i)
    {
      const float r = io.ranges[i];
      if (!std::isfinite(r))
        continue;
      const double psi = a0 + da * static_cast<double>(i) + cfg.lidar_mount_yaw;
      const double z_dir = u.x * std::cos(psi) + u.y * std::sin(psi);
      const float return_z = static_cast<float>(cfg.lidar_height_m + r * z_dir);
      if (return_z > max_z)
        klass[i] = 2;
      else if (return_z < min_z)
        klass[i] = 1;
    }

    // Pass 2: strip overhead returns outright; strip ground returns only where
    // they form a contiguous run >= min_ground_run (a long sweep of ground),
    // leaving short ground-classified clusters as possible obstacles.
    const int min_run = std::max(1, cfg.min_ground_run);
    for (size_t i = 0; i < n; ++i)
    {
      if (klass[i] == 2)
      {
        io.ranges[i] = inf;
        continue;
      }
      if (klass[i] != 1)
        continue;
      // Extent of the contiguous ground run containing i.
      size_t j = i;
      while (j < n && klass[j] == 1)
        ++j;
      const size_t run_len = j - i;
      if (static_cast<int>(run_len) >= min_run)
      {
        for (size_t k = i; k < j; ++k)
          io.ranges[k] = inf;
      }
      // else: keep the short cluster (possible obstacle the slope mis-projected).
      i = j - 1;  // skip past the run we just handled
    }
  }

  // --- LiDAR-ignore corridor filter (pure) --------------------------------

  /// A point in the MAP frame.
  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  /// The robot's pose in the MAP frame (fused, from /odometry/filtered_map).
  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  /// One LidarIgnoreCorridor, reduced to what the filter needs: a polyline
  /// (>= 2 points) and the perpendicular half-width to suppress within.
  struct Corridor
  {
    std::vector<Point2D> polyline;
    double half_width_m{0.0};
  };

  /// LIDAR mount geometry relative to base_link — same lidar_x/lidar_y/
  /// lidar_mount_yaw the ground filter above uses, but here the POSITION
  /// matters too (the ground filter is angle-only).
  struct LidarExtrinsics
  {
    double x_m{0.0};
    double y_m{0.0};
    double mount_yaw{0.0};
  };

  /// Perpendicular distance from (px, py) to the segment [a, b].
  static double distance_point_to_segment(double px, double py, const Point2D& a, const Point2D& b)
  {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-12)
      return std::hypot(px - a.x, py - a.y);
    double t = ((px - a.x) * dx + (py - a.y) * dy) / len2;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
  }

  /// True if (px, py) is within `corridor`'s half_width_m of ANY of its
  /// segments. A corridor with fewer than 2 points is degenerate and never
  /// matches (map_server_node already rejects one on add, but a filter
  /// consuming a stale/malformed message must not misbehave on it either).
  static bool point_within_corridor(double px, double py, const Corridor& corridor)
  {
    if (corridor.polyline.size() < 2)
      return false;
    for (std::size_t i = 0; i + 1 < corridor.polyline.size(); ++i)
    {
      if (distance_point_to_segment(px, py, corridor.polyline[i], corridor.polyline[i + 1]) <=
          corridor.half_width_m)
        return true;
    }
    return false;
  }

  /// Apply the corridor-ignore filter to @p io in place: for each finite
  /// beam, project its map-frame endpoint (LIDAR mount extrinsics + the
  /// robot's current pose) and push the range to +inf if that point falls
  /// within any corridor. SAFETY: called on BOTH the /scan_costmap and
  /// /scan_collision paths (see the file header comment, item 3) — this is
  /// the one filter in this node not restricted to the costmap-only path,
  /// by explicit operator choice.
  ///
  /// `robot_pose_map` is std::nullopt when no fresh pose exists (stale or
  /// never received) — the function is then a no-op, same rule as
  /// apply_ground_filter's stale-IMU case: better to keep seeing a corridor's
  /// hedge than to silently blind the robot near one while it doesn't
  /// actually know where it is.
  /// Returns how many beams were suppressed (0 when it was a no-op).
  static std::size_t apply_corridor_ignore_filter(sensor_msgs::msg::LaserScan& io,
                                                  const std::vector<Corridor>& corridors,
                                                  const std::optional<Pose2D>& robot_pose_map,
                                                  const LidarExtrinsics& extrinsics)
  {
    if (corridors.empty() || !robot_pose_map.has_value())
      return 0;
    std::size_t suppressed = 0;
    const Pose2D& pose = *robot_pose_map;
    const float inf = std::numeric_limits<float>::infinity();
    const double a0 = io.angle_min;
    const double da = io.angle_increment;
    const size_t n = io.ranges.size();
    const double cy = std::cos(pose.yaw);
    const double sy = std::sin(pose.yaw);
    // LIDAR mount position in the map frame: rotate the base-frame offset by
    // the robot's yaw, then translate by the robot's map-frame position.
    const double lidar_map_x = pose.x + extrinsics.x_m * cy - extrinsics.y_m * sy;
    const double lidar_map_y = pose.y + extrinsics.x_m * sy + extrinsics.y_m * cy;

    for (size_t i = 0; i < n; ++i)
    {
      float& r = io.ranges[i];
      if (!std::isfinite(r))
        continue;
      // Beam angle: LIDAR index angle -> base frame (mount yaw) -> map frame
      // (robot yaw) — same rotation chain the ground filter uses for psi,
      // extended with the robot's own yaw since this needs a MAP-frame point,
      // not just a base-frame direction.
      const double alpha = a0 + da * static_cast<double>(i);
      const double psi = alpha + extrinsics.mount_yaw + pose.yaw;
      const double px = lidar_map_x + static_cast<double>(r) * std::cos(psi);
      const double py = lidar_map_y + static_cast<double>(r) * std::sin(psi);

      for (const auto& corridor : corridors)
      {
        if (point_within_corridor(px, py, corridor))
        {
          r = inf;
          ++suppressed;
          break;
        }
      }
    }
    return suppressed;
  }

  /// Smallest distance from (px, py) to any corridor polyline; +inf if none.
  /// Diagnostics only (tells whether the robot is anywhere near a drawn line).
  static double distance_to_nearest_corridor(double px,
                                             double py,
                                             const std::vector<Corridor>& corridors)
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& corridor : corridors)
    {
      for (std::size_t i = 0; i + 1 < corridor.polyline.size(); ++i)
        best = std::min(
            best,
            distance_point_to_segment(px, py, corridor.polyline[i], corridor.polyline[i + 1]));
    }
    return best;
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
    const rclcpp::Time sample_time = now();
    const GravityEstimatorAction action =
        gravity_estimator_.update(GravityVector{msg.linear_acceleration.x,
                                                msg.linear_acceleration.y,
                                                msg.linear_acceleration.z},
                                  sample_time.seconds());
    if (action == GravityEstimatorAction::INVALID || action == GravityEstimatorAction::REJECTED)
    {
      // Deliberately do not refresh last_imu_stamp_: the ground filter must
      // become pass-through while a transient or a new baseline is unproven.
      return;
    }

    const GravityVector up = gravity_estimator_.direction();
    last_up_in_imu_ = Vec3{up.x, up.y, up.z};
    last_imu_stamp_ = sample_time;
    if (action == GravityEstimatorAction::RESEEDED)
    {
      const double old_baseline = gravity_estimator_.previous_baseline_magnitude_ms2();
      const double new_baseline = gravity_estimator_.baseline_magnitude_ms2();
      const double absolute_change = std::abs(new_baseline - old_baseline);
      const double percentage_change = 100.0 * absolute_change / old_baseline;
      RCLCPP_WARN(get_logger(),
                  "ground filter IMU baseline re-seeded %.3f -> %.3f m/s² "
                  "(absolute change %.3f m/s², %.1f%%); IMU scale or calibration may be incorrect",
                  old_baseline,
                  new_baseline,
                  absolute_change,
                  percentage_change);
    }
  }

  void on_corridors(const mowgli_interfaces::msg::LidarIgnoreCorridorArray& msg)
  {
    std::vector<Corridor> corridors;
    corridors.reserve(msg.corridors.size());
    for (const auto& c : msg.corridors)
    {
      Corridor corridor;
      corridor.half_width_m = 0.5 * c.width_m;
      corridor.polyline.reserve(c.polyline.points.size());
      for (const auto& pt : c.polyline.points)
      {
        corridor.polyline.push_back(Point2D{static_cast<double>(pt.x), static_cast<double>(pt.y)});
      }
      corridors.push_back(std::move(corridor));
    }
    RCLCPP_INFO(get_logger(), "LiDAR-ignore corridors updated: %zu", corridors.size());
    last_corridors_ = std::move(corridors);
  }

  void on_odom(const nav_msgs::msg::Odometry& msg)
  {
    Pose2D pose;
    pose.x = msg.pose.pose.position.x;
    pose.y = msg.pose.pose.position.y;
    const auto& q = msg.pose.pose.orientation;
    pose.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    last_robot_pose_map_ = pose;
    last_pose_stamp_ = now();
  }

  void on_scan(const sensor_msgs::msg::LaserScan& msg)
  {
    // Two-stage radial blank: chassis_blank_range_ is always on, then
    // dock_blank_range_ kicks in only while charging / immediately
    // post-undock. The `effective` blank range is the larger of the two
    // currently-active values, so a single pass through filter_scan
    // suffices.
    const bool dock_active = is_blank_active();
    const double effective_blank =
        std::max(chassis_blank_range_, dock_active ? dock_blank_range_ : 0.0);
    sensor_msgs::msg::LaserScan out = filter_scan(msg, effective_blank, effective_blank > 0.0);

    // SAFETY (operator opt-in, unlike every other filter in this function):
    // applied BEFORE the collision-scan publish below, so a drawn corridor
    // suppresses LiDAR on BOTH outputs — see the file header comment, item 3,
    // and apply_corridor_ignore_filter's own doc comment for the full
    // rationale and the stale-pose fallback.
    std::optional<Pose2D> pose_for_corridor_filter;
    if (!last_corridors_.empty() && last_pose_stamp_.nanoseconds() != 0)
    {
      const double pose_age = (now() - last_pose_stamp_).seconds();
      if (pose_age >= 0.0 && pose_age < corridor_pose_max_age_s_)
        pose_for_corridor_filter = last_robot_pose_map_;
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "corridor filter idle: last fused pose %.2fs old (>%.2fs)",
                             pose_age,
                             corridor_pose_max_age_s_);
      }
    }
    const std::size_t corridor_suppressed =
        apply_corridor_ignore_filter(out,
                                     last_corridors_,
                                     pose_for_corridor_filter,
                                     LidarExtrinsics{lidar_x_m_, lidar_y_m_, lidar_mount_yaw_});
    if (pose_for_corridor_filter.has_value())
    {
      // Throttled diagnostics: is the robot near a drawn line, and did the
      // filter actually drop any beams? (field 2026-09-25: no way to tell.)
      RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "corridor filter: robot (%.2f, %.2f) is %.2f m from the nearest line, %zu beam(s) "
          "suppressed this scan",
          pose_for_corridor_filter->x,
          pose_for_corridor_filter->y,
          distance_to_nearest_corridor(pose_for_corridor_filter->x,
                                       pose_for_corridor_filter->y,
                                       last_corridors_),
          corridor_suppressed);
    }

    // SAFETY: collision_monitor gets the scan with chassis/dock self-returns
    // blanked but WITHOUT the gravity ground filter applied. The ground filter
    // can mis-classify a real vertical obstacle as ground on a slope and strip
    // it; that false negative is acceptable for the costmap obstacle_layer
    // (MPPI's CostCritic plans around what it sees) but MUST NOT defeat the
    // near-field hard stop. Publishing the un-ground-filtered stream here keeps
    // the PolygonStop/PolygonSlow zones reacting to every near return while
    // still never seeing the robot's own chassis or the dock — EXCEPT inside an
    // operator-drawn LiDAR-ignore corridor, applied just above.
    if (pub_collision_scan_)
      pub_collision_scan_->publish(out);

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
                           lidar_mount_yaw_,
                           min_ground_run_};
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
  rclcpp::Subscription<mowgli_interfaces::msg::LidarIgnoreCorridorArray>::SharedPtr sub_corridors_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_collision_scan_;

  // --- Parameters --------------------------------------------------------

  double dock_blank_range_{0.70};
  double chassis_blank_range_{0.0};
  double post_undock_blank_sec_{5.0};
  bool enable_ground_filter_{true};
  double min_obstacle_z_m_{0.08};
  double max_obstacle_z_m_{1.5};
  double lidar_height_m_{0.30};
  double lidar_mount_yaw_{0.0};
  int min_ground_run_{8};
  double imu_max_age_s_{0.5};
  double accel_g_tolerance_ms2_{3.0};
  GravityEstimator gravity_estimator_{};
  double lidar_x_m_{0.0};
  double lidar_y_m_{0.0};
  double corridor_pose_max_age_s_{1.0};

  // --- Charging-state machine -------------------------------------------

  bool last_is_charging_{false};
  bool last_is_charging_known_{false};
  rclcpp::Time charging_dropped_at_{0, 0, RCL_ROS_TIME};

  // --- IMU latch (for ground filter) ------------------------------------

  /// Low-pass-filtered "up" direction in IMU frame, derived from the
  /// gravity component of linear_acceleration. Empty until first sample.
  std::optional<Vec3> last_up_in_imu_;
  rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

  // --- Corridor filter state ----------------------------------------------

  /// Latest corridor list from /mowgli/lidar_ignore_corridors. Empty until
  /// the first message (transient_local, so that arrives promptly) or if the
  /// operator has drawn none — the filter is then a cheap no-op.
  std::vector<Corridor> last_corridors_;
  /// Latest fused pose from /odometry/filtered_map, freshness-gated by
  /// corridor_pose_max_age_s_ at use (on_scan), same pattern as
  /// last_up_in_imu_/last_imu_stamp_ above.
  Pose2D last_robot_pose_map_{};
  rclcpp::Time last_pose_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CostmapScanFilterNode>());
  rclcpp::shutdown();
  return 0;
}
