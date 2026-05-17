// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// scan_deskew_node — undo the rotation smear of a sequential 360° LiDAR
// scan acquired while the robot is rotating.
//
// A rotating LiDAR samples its rays sequentially (one ray every
// `time_increment` seconds). At 10 Hz with 720 rays, the rays spread
// over ~100 ms. While the robot rotates at, say, 0.8 rad/s, the lidar
// orientation drifts ~4.6° during the scan, but every ray is published
// with the SAME header.stamp. Downstream consumers transform the entire
// scan with a single TF lookup, smearing the obstacles by up to ~5° in
// the map frame.
//
// We compensate by rotating each ray's angle in the lidar frame by
// `-ω · dt_i`, where `dt_i = i*time_increment - reference_dt` is the
// ray's time offset relative to a chosen reference (the END of the
// scan here — matches the published header.stamp). The rays are
// re-binned into the original LaserScan grid; each output bin keeps
// the nearest range when multiple input rays remap to it.
//
// **Per-ray ω from an IMU history buffer.** Older versions used the
// single most recent gyro_z sample for every ray of every scan. That
// over-corrected during transitions (e.g. when cmd_vel went 0.8 → 0
// and the robot decelerated 0.6 → 0 rad/s within a scan window):
// `latest_omega_z_` was still 0.8 rad/s but the scan's average ω was
// much lower, so the deskew rotated all rays by ~5° too much, making
// the scan appear to "follow then snap back" in the costmap. The
// operator observed this as phantom rotations during pivots.
//
// Fix: keep a sliding-window buffer of (stamp, ω_z) IMU samples and
// linearly interpolate to the actual time of each ray. Buffer depth
// covers ~150 ms — one scan period plus margin for IMU latency.
//
// Linear motion compensation is currently skipped — at typical
// mowing speeds (≤ 0.3 m/s × ≤ 0.1 s scan) the resulting < 30 mm
// displacement is negligible compared to the rotational smear, and
// adding it requires a per-ray TF lookup or a full pose buffer.
//
// Output topic: `/scan_deskewed` (same structure as input). Configure
// downstream nodes (costmap_scan_filter, collision_monitor sources)
// to read this instead of /scan.

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mowgli_localization
{

class ScanDeskewNode : public rclcpp::Node
{
public:
  ScanDeskewNode() : Node("scan_deskew")
  {
    const std::string input_topic =
        declare_parameter<std::string>("input_topic", "/scan");
    const std::string output_topic =
        declare_parameter<std::string>("output_topic", "/scan_deskewed");
    const std::string imu_topic =
        declare_parameter<std::string>("imu_topic", "/imu/data");

    // Reference time for the deskewed output. "end" keeps the same
    // header.stamp as the input (= timestamp of the last ray); "start"
    // shifts the stamp back by the scan duration so consumers that
    // assume header.stamp = first-ray-time interpret the output
    // correctly. ROS convention is "end", which is what Webots uses.
    reference_ = declare_parameter<std::string>("reference", "end");

    // Maximum age of the cached IMU sample. If the IMU went stale,
    // fall back to passthrough rather than apply a wildly wrong ω.
    imu_max_age_s_ = declare_parameter<double>("imu_max_age_s", 0.5);

    // IMU buffer depth in seconds. Should cover one scan window plus
    // some margin (LiDAR end-stamp is the last ray; the first ray is
    // scan_duration before that, plus any IMU-to-scan publish lag).
    imu_buffer_horizon_s_ =
        declare_parameter<double>("imu_buffer_horizon_s", 0.5);

    rclcpp::QoS qos_sensor = rclcpp::SensorDataQoS();
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic, qos_sensor);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic,
        qos_sensor,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { on_scan(*msg); });

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic,
        qos_sensor,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(*msg); });

    pub_count_ = 0;
    skipped_count_ = 0;
    interp_misses_ = 0;
    create_wall_timer(std::chrono::seconds(15),
                      [this]() {
                        RCLCPP_INFO(get_logger(),
                                    "scan_deskew stats: published=%zu, "
                                    "passthrough(stale-IMU)=%zu, "
                                    "interp_misses=%zu, last_ω_z=%.3f rad/s, "
                                    "buf_size=%zu",
                                    pub_count_, skipped_count_, interp_misses_,
                                    latest_omega_z_, imu_buffer_.size());
                      });

    RCLCPP_INFO(get_logger(),
                "scan_deskew started — %s -> %s (reference=%s, imu=%s, "
                "max_age=%.2fs, buffer_horizon=%.2fs)",
                input_topic.c_str(),
                output_topic.c_str(),
                reference_.c_str(),
                imu_topic.c_str(),
                imu_max_age_s_,
                imu_buffer_horizon_s_);
  }

private:
  struct ImuSample
  {
    double t_s;     // ROS seconds
    double omega_z; // rad/s
  };

  void on_imu(const sensor_msgs::msg::Imu& msg)
  {
    const rclcpp::Time stamp(msg.header.stamp);
    const double t = stamp.seconds();
    if (t <= 0.0)
    {
      // Bad stamp — skip rather than corrupt the buffer order.
      return;
    }
    const double wz = msg.angular_velocity.z;
    imu_buffer_.push_back({t, wz});
    latest_omega_z_ = wz;
    latest_imu_t_ = stamp;
    have_imu_ = true;

    // Trim entries older than imu_buffer_horizon_s_ before the newest.
    while (!imu_buffer_.empty() &&
           (t - imu_buffer_.front().t_s) > imu_buffer_horizon_s_)
    {
      imu_buffer_.pop_front();
    }
  }

  // Linear interpolation of ω_z at query time `t_s`. Returns the
  // interpolated value via `omega_out` and `true` on success. If `t_s`
  // falls outside the buffer (e.g. before the oldest sample or after
  // the newest), the caller decides what to do; `omega_out` is then
  // set to the nearest endpoint value.
  bool interp_omega(double t_s, double& omega_out) const
  {
    if (imu_buffer_.empty())
    {
      omega_out = 0.0;
      return false;
    }
    if (t_s <= imu_buffer_.front().t_s)
    {
      // Query predates buffer — use oldest sample as best guess.
      omega_out = imu_buffer_.front().omega_z;
      return false;
    }
    if (t_s >= imu_buffer_.back().t_s)
    {
      // Query past newest — use newest sample. The caller can decide
      // to gate on the gap if it matters.
      omega_out = imu_buffer_.back().omega_z;
      return false;
    }
    // Buffer is small (≤ ~50 samples for 0.5 s @ 100 Hz IMU) → linear
    // scan is cheaper than binary search and avoids the iterator
    // arithmetic that std::lower_bound on a deque incurs.
    for (size_t i = 1; i < imu_buffer_.size(); ++i)
    {
      const auto& b = imu_buffer_[i];
      if (b.t_s >= t_s)
      {
        const auto& a = imu_buffer_[i - 1];
        const double dt = b.t_s - a.t_s;
        if (dt <= 0.0)
        {
          omega_out = b.omega_z;
          return true;
        }
        const double f = (t_s - a.t_s) / dt;
        omega_out = a.omega_z + f * (b.omega_z - a.omega_z);
        return true;
      }
    }
    omega_out = imu_buffer_.back().omega_z;
    return false;
  }

  void on_scan(const sensor_msgs::msg::LaserScan& in)
  {
    sensor_msgs::msg::LaserScan out = in;

    // If we have no fresh IMU, pass through unchanged. Better to emit a
    // smeared scan than a wildly mis-corrected one.
    if (!have_imu_)
    {
      pub_->publish(out);
      skipped_count_++;
      return;
    }
    const double imu_age = (now() - latest_imu_t_).seconds();
    if (imu_age < 0.0 || imu_age > imu_max_age_s_)
    {
      pub_->publish(out);
      skipped_count_++;
      return;
    }

    const size_t n = in.ranges.size();
    if (n < 2 || in.angle_increment == 0.0f || in.time_increment == 0.0f)
    {
      pub_->publish(out);
      return;
    }

    // Reset all output ranges to +inf; we'll fill in valid samples.
    const float inf_f = std::numeric_limits<float>::infinity();
    std::fill(out.ranges.begin(), out.ranges.end(), inf_f);
    if (!out.intensities.empty() && out.intensities.size() == out.ranges.size())
    {
      std::fill(out.intensities.begin(), out.intensities.end(), 0.0f);
    }

    // Scan reference time. The published header.stamp is the timestamp
    // of the reference ray (last for "end", first for "start"). Build
    // an absolute time per ray for the IMU buffer query.
    const rclcpp::Time scan_stamp(in.header.stamp);
    const double scan_stamp_s = scan_stamp.seconds();
    const double ref_offset_s =
        (reference_ == "start") ? 0.0 : static_cast<double>(in.time_increment) * (n - 1);

    // The angle_increment can be negative (CW scan); handle either way.
    const float ang_min = in.angle_min;
    const float ang_inc = in.angle_increment;
    const float inv_ang_inc = 1.0f / ang_inc;

    for (size_t i = 0; i < n; ++i)
    {
      const float r = in.ranges[i];
      if (!std::isfinite(r) || r < in.range_min || r > in.range_max)
      {
        continue;
      }

      // dt is the ray's time offset relative to the reference. For
      // "end", dt is negative for early rays (they were sampled
      // before the stamp). We want ω at the ray's time, not at the
      // reference, so the absolute ray time is:
      //     t_ray = scan_stamp_s + dt
      const double dt = static_cast<double>(in.time_increment) * i - ref_offset_s;
      const double t_ray = scan_stamp_s + dt;

      double omega_at_ray = 0.0;
      const bool ok = interp_omega(t_ray, omega_at_ray);
      if (!ok)
      {
        // t_ray fell outside the IMU buffer — interp_omega already
        // populated omega_at_ray with the nearest endpoint, but
        // bump the counter so the rate is visible in stats.
        interp_misses_++;
      }

      // Geometric: at ray-time the lidar had orientation θ_i, at scan-
      // stamp time it has θ_ref = θ_i + ∫ω dt over (t_ray, t_ref).
      // The exact integral would be ∫ω(t)dt; for short dt (≤ 100 ms)
      // and smoothly-varying ω, the trapezoidal approximation
      // 0.5 · (ω_at_ray + ω_at_ref) · dt is more accurate than
      // either endpoint alone — but the cost is one extra interp.
      // We use ω_at_ray · dt which is a first-order approximation
      // and matches the previous algebra; the buffer-driven ω is the
      // dominant accuracy gain. Switch to trapezoidal in a follow-up
      // if residual smear during fast ω transients is still visible.
      const double alpha = static_cast<double>(ang_min) +
                           static_cast<double>(ang_inc) * static_cast<double>(i);
      // dt is t_ray - t_ref (negative for "end" mode early rays). The
      // ray's endpoint angle expressed in the scan-stamp lidar frame
      // is shifted by -(θ_ref - θ_i) = -ω·(t_ref - t_ray) = +ω·dt.
      const double alpha_corr = alpha + omega_at_ray * dt;

      // Re-bin into the output grid (nearest bin).
      const int bin = static_cast<int>(
          std::lround((alpha_corr - static_cast<double>(ang_min)) *
                      static_cast<double>(inv_ang_inc)));
      if (bin < 0 || bin >= static_cast<int>(n))
      {
        continue;
      }

      // Multi-write resolution: keep the nearest range (most pessimistic
      // for collision_monitor — better safe than sorry).
      if (r < out.ranges[bin])
      {
        out.ranges[bin] = r;
        if (!in.intensities.empty() && i < in.intensities.size() &&
            !out.intensities.empty() && bin < static_cast<int>(out.intensities.size()))
        {
          out.intensities[bin] = in.intensities[i];
        }
      }
    }

    pub_->publish(out);
    pub_count_++;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  // IMU history. Deque so the trim path is O(1) on the oldest end.
  // Bounded by imu_buffer_horizon_s_ — at 100 Hz IMU × 0.5 s = 50
  // samples max, fits comfortably in CPU cache.
  std::deque<ImuSample> imu_buffer_;

  double latest_omega_z_{0.0};
  rclcpp::Time latest_imu_t_{0, 0, RCL_ROS_TIME};
  bool have_imu_{false};
  std::string reference_{"end"};
  double imu_max_age_s_{0.5};
  double imu_buffer_horizon_s_{0.5};
  size_t pub_count_{0};
  size_t skipped_count_{0};
  size_t interp_misses_{0};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::ScanDeskewNode>());
  rclcpp::shutdown();
  return 0;
}
