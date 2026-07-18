// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// mag_yaw_publisher_node.cpp
//
// C++ port of scripts/mag_yaw_publisher.py — reads /imu/mag_raw + /imu/data,
// applies the hard-iron / soft-iron calibration, tilt-compensates with
// roll/pitch derived from accel, and publishes an absolute yaw on
// /imu/mag_yaw (sensor_msgs/Imu, BEST_EFFORT).
//
// Behaviour matches the Python implementation 1:1.

#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <yaml-cpp/yaml.h>

namespace mowgli_localization
{

struct MagCalibration
{
  double offset_x_uT;
  double offset_y_uT;
  double offset_z_uT;
  double scale_x;
  double scale_y;
  double scale_z;
  double magnitude_mean_uT;
};

class MagYawPublisherNode : public rclcpp::Node
{
public:
  MagYawPublisherNode() : Node("mag_yaw_publisher")
  {
    cal_path_ =
        declare_parameter<std::string>("calibration_path", "/ros2_ws/maps/mag_calibration.yaml");
    const double declination_deg = declare_parameter<double>("declination_deg", 1.5);
    declination_rad_ = declination_deg * M_PI / 180.0;
    min_horizontal_uT_ = declare_parameter<double>("min_horizontal_uT", 5.0);
    yaw_var_ = declare_parameter<double>("yaw_variance", 2.7e-3);

    qos_sensor_ = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_sensor_.best_effort();
    qos_sensor_.durability_volatile();

    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/mag_yaw", qos_sensor_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cal_ = load_calibration(cal_path_);

    stats_timer_ = create_wall_timer(std::chrono::seconds(30),
                                     [this]()
                                     {
                                       log_stats();
                                     });
    reload_timer_ = create_wall_timer(std::chrono::seconds(30),
                                      [this]()
                                      {
                                        reload_cal_if_needed();
                                      });

    if (!cal_)
    {
      RCLCPP_WARN(get_logger(),
                  "No mag calibration found at %s — mag_yaw_publisher idle, "
                  "polling every 30 s for the calibration file.",
                  cal_path_.c_str());
    }
    else
    {
      activate_subs();
      RCLCPP_INFO(get_logger(),
                  "Mag calibration loaded: offset=(%+.2f, %+.2f, %+.2f) µT, "
                  "scale=(%.3f, %.3f, %.3f), |B|cal=%.2f µT",
                  cal_->offset_x_uT,
                  cal_->offset_y_uT,
                  cal_->offset_z_uT,
                  cal_->scale_x,
                  cal_->scale_y,
                  cal_->scale_z,
                  cal_->magnitude_mean_uT);
    }
  }

private:
  void activate_subs()
  {
    if (imu_sub_)
      return;
    imu_sub_ =
        create_subscription<sensor_msgs::msg::Imu>("/imu/data",
                                                   qos_sensor_,
                                                   [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
                                                   {
                                                     on_imu(msg);
                                                   });
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
        "/imu/mag_raw",
        qos_sensor_,
        [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
        {
          on_mag(*msg);
        });
  }

  void reload_cal_if_needed()
  {
    if (cal_)
      return;
    auto cal = load_calibration(cal_path_);
    if (!cal)
      return;
    cal_ = cal;
    activate_subs();
    RCLCPP_INFO(get_logger(), "Mag calibration appeared — subscriptions activated.");
  }

  std::optional<MagCalibration> load_calibration(const std::string& path)
  {
    namespace fs = std::filesystem;
    if (!fs::exists(path))
    {
      return std::nullopt;
    }
    try
    {
      YAML::Node root = YAML::LoadFile(path);
      if (!root["mag_calibration"])
      {
        RCLCPP_ERROR(get_logger(), "%s: missing 'mag_calibration' key", path.c_str());
        return std::nullopt;
      }
      const auto cal = root["mag_calibration"];
      MagCalibration out{};
      out.offset_x_uT = cal["offset_x_uT"].as<double>();
      out.offset_y_uT = cal["offset_y_uT"].as<double>();
      out.offset_z_uT = cal["offset_z_uT"].as<double>();
      out.scale_x = cal["scale_x"].as<double>(1.0);
      out.scale_y = cal["scale_y"].as<double>(1.0);
      out.scale_z = cal["scale_z"].as<double>(1.0);
      out.magnitude_mean_uT = cal["magnitude_mean_uT"].as<double>(0.0);
      return out;
    }
    catch (const std::exception& exc)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load %s: %s", path.c_str(), exc.what());
      return std::nullopt;
    }
  }

  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    latest_imu_ = msg;
  }

  void on_mag(const sensor_msgs::msg::MagneticField& msg)
  {
    if (!cal_)
    {
      ++rejected_no_cal_;
      return;
    }
    if (!latest_imu_)
    {
      ++rejected_no_imu_;
      return;
    }

    // 1) Calibration in µT.
    const double bx_uT = msg.magnetic_field.x * 1e6;
    const double by_uT = msg.magnetic_field.y * 1e6;
    const double bz_uT = msg.magnetic_field.z * 1e6;

    const double bx = (bx_uT - cal_->offset_x_uT) * cal_->scale_x;
    const double by = (by_uT - cal_->offset_y_uT) * cal_->scale_y;
    const double bz = (bz_uT - cal_->offset_z_uT) * cal_->scale_z;

    const double ax_imu = latest_imu_->linear_acceleration.x;
    const double ay_imu = latest_imu_->linear_acceleration.y;
    const double az_imu = latest_imu_->linear_acceleration.z;
    if (std::abs(az_imu) < 1e-6 && std::abs(ax_imu) < 1e-6 && std::abs(ay_imu) < 1e-6)
    {
      ++rejected_no_imu_;
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform("base_footprint",
                                       "imu_link",
                                       tf2::TimePointZero,
                                       tf2::durationFromSec(0.2));
    }
    catch (const std::exception&)
    {
      ++rejected_no_tf_;
      return;
    }

    double bx_b, by_b, bz_b, ax_b, ay_b, az_b;
    rotate_by_quat(bx, by, bz, tf.transform.rotation, bx_b, by_b, bz_b);
    rotate_by_quat(ax_imu, ay_imu, az_imu, tf.transform.rotation, ax_b, ay_b, az_b);

    const double roll = std::atan2(ay_b, az_b);
    const double pitch = std::atan2(-ax_b, std::sqrt(ay_b * ay_b + az_b * az_b));

    const double cr = std::cos(roll), sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double bx_h = bx_b * cp + by_b * sr * sp + bz_b * cr * sp;
    const double by_h = by_b * cr - bz_b * sr;

    const double horiz = std::sqrt(bx_h * bx_h + by_h * by_h);
    if (horiz < min_horizontal_uT_)
    {
      ++rejected_noise_;
      return;
    }

    const double yaw_mag = std::atan2(bx_h, by_h);
    const double yaw_true = yaw_mag + declination_rad_;

    publish_imu(msg.header.stamp, yaw_true);
    ++published_;
  }

  static void rotate_by_quat(double vx,
                             double vy,
                             double vz,
                             const geometry_msgs::msg::Quaternion& q,
                             double& xp,
                             double& yp,
                             double& zp)
  {
    const double qw = q.w, qx = q.x, qy = q.y, qz = q.z;
    const double tx = 2.0 * (qy * vz - qz * vy);
    const double ty = 2.0 * (qz * vx - qx * vz);
    const double tz = 2.0 * (qx * vy - qy * vx);
    xp = vx + qw * tx + (qy * tz - qz * ty);
    yp = vy + qw * ty + (qz * tx - qx * tz);
    zp = vz + qw * tz + (qx * ty - qy * tx);
  }

  void publish_imu(const builtin_interfaces::msg::Time& stamp, double yaw)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_footprint";
    imu.orientation.w = std::cos(yaw / 2.0);
    imu.orientation.x = 0.0;
    imu.orientation.y = 0.0;
    imu.orientation.z = std::sin(yaw / 2.0);
    for (auto& v : imu.orientation_covariance)
      v = 0.0;
    imu.orientation_covariance[8] = yaw_var_;
    for (auto& v : imu.angular_velocity_covariance)
      v = 0.0;
    imu.angular_velocity_covariance[0] = -1.0;
    for (auto& v : imu.linear_acceleration_covariance)
      v = 0.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    pub_->publish(imu);
  }

  void log_stats()
  {
    RCLCPP_INFO(get_logger(),
                "mag_yaw_publisher stats: published=%d, rejected no_cal=%d "
                "no_imu=%d no_tf=%d noise=%d",
                published_,
                rejected_no_cal_,
                rejected_no_imu_,
                rejected_no_tf_,
                rejected_noise_);
    published_ = 0;
    rejected_no_cal_ = 0;
    rejected_no_imu_ = 0;
    rejected_no_tf_ = 0;
    rejected_noise_ = 0;
  }

  std::string cal_path_;
  double declination_rad_{};
  double min_horizontal_uT_{};
  double yaw_var_{};
  std::optional<MagCalibration> cal_;

  rclcpp::QoS qos_sensor_{rclcpp::KeepLast(10)};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_, reload_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  sensor_msgs::msg::Imu::ConstSharedPtr latest_imu_;

  int published_{0};
  int rejected_noise_{0}, rejected_no_cal_{0}, rejected_no_imu_{0}, rejected_no_tf_{0};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::MagYawPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
