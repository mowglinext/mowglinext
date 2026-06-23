// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// cog_to_imu_node.cpp
//
// C++ port of scripts/cog_to_imu.py — derives a GPS course-over-ground
// heading from successive RTK-Fixed positions and publishes it as a
// sensor_msgs/Imu absolute-yaw observation on /imu/cog_heading. Also
// runs an online magnetometer calibration least-squares fit using
// COG yaw as ground-truth.
//
// Topics, parameters, formulas and gating thresholds match the Python
// implementation 1:1.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "mowgli_localization/cog_yaw_math.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include <yaml-cpp/yaml.h>

namespace mowgli_localization
{

namespace
{
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kMetersPerDeg = 111319.49079327357;

rclcpp::QoS sensor_qos()
{
  // Match Python QoSProfile(BEST_EFFORT, VOLATILE, KEEP_LAST, depth=10).
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

std::string utc_iso8601_now()
{
  // Format identical enough to Python datetime.isoformat() with tz info
  // (used as a free-form string in the YAML — consumers don't parse it).
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

class CogToImuNode : public rclcpp::Node
{
public:
  CogToImuNode() : Node("cog_to_imu")
  {
    // ── Sample-pair gating thresholds ────────────────────────────────
    min_abs_wheel_ = declare_parameter<double>("min_abs_wheel_ms", 0.05);
    // Upper bound on |ω| where the constant-rate approximation we use to
    // unbias the COG (mid-baseline drift + lever-arm tangential velocity)
    // still holds. Above this we hard-reject because the integrated
    // antenna path deviates non-linearly from R(ψ)·[v, ω·r] and our
    // variance model under-states the residual error. Below this, COG
    // *is* published during turns — the lever-arm + drift biases are
    // subtracted analytically and σ_yaw is inflated by their respective
    // ω-noise sensitivities, so a tight headland turn naturally yields
    // a high-σ measurement that ekf_map / fusion_graph down-weight.
    // 0.50 rad/s ≈ 29°/s lets gentle corrections through and rejects
    // full PRE_ROTATE pivots (typ. 0.6 rad/s).
    min_omega_for_anchor_ = declare_parameter<double>("min_omega_for_anchor_rps", 0.50);
    // Lever-arm-sweep dominance ratio (see cog_yaw_math.hpp). Reject a COG
    // sample when |omega|*lever_radius > ratio*|vx| — i.e. the antenna's
    // rotational sweep outpaces real translation, so atan2(dy,dx) is the
    // sweep tangent, not the heading. Catches the slow dock-alignment pivots
    // (0.1-0.3 rad/s) that slip under min_omega_for_anchor_rps. 1.0 = reject
    // once antenna rotational speed exceeds chassis forward speed.
    cog_sweep_dominance_ratio_ =
        declare_parameter<double>("cog_sweep_dominance_ratio", 1.0);
    // Max integrated |heading change| (rad) allowed over a COG baseline. A
    // straight segment is ~0; above this the displacement no longer tracks
    // the body axis (slow oscillation) and the COG is dropped. 0.20 ≈ 11°.
    cog_max_baseline_rotation_rad_ =
        declare_parameter<double>("cog_max_baseline_rotation_rad", 0.20);
    // Rotation gate for the stationary latched-yaw republish. Much lower
    // than min_omega_for_anchor_rps: this path only runs when the robot is
    // meant to be stationary, so any meaningful rotation (the dock's slow
    // 0.1-0.3 rad/s alignment pivots included) means the latched heading is
    // going stale and must not be republished. 0.05 rad/s ≈ 3°/s.
    latch_republish_max_omega_ =
        declare_parameter<double>("latch_republish_max_omega_rps", 0.05);
    // Cumulative-rotation staleness gate for the latch. The instantaneous
    // latch_republish_max_omega_ gate above only suppresses republish *while*
    // the robot is turning; once an in-place pivot ENDS and ω returns to ~0
    // the gate passes again and the pre-pivot heading gets republished — but
    // the latch only updates on forward translation, so after a 180° pivot it
    // is stale by π and corrupts the fused yaw (field 2026-06-20: manual
    // east→west flip teleported fusion yaw −6°→−131° while motionless, then
    // dragged position around the gps_x lever-arm circle). Track absolute
    // rotation accumulated since the latch was set and drop the latch once it
    // exceeds this; a real pivot trips it immediately. We do NOT reuse
    // abs_dtheta_since_anchor_ — that integrates |gyro_z| with no deadband and
    // resets every GPS fix, so over a multi-second stationary dwell its noise
    // floor would falsely invalidate a good latch. latch_rotation_deadband_rps
    // is the per-sample floor below which |gyro_z| is treated as noise and not
    // accumulated. 0.26 rad ≈ 15°.
    latch_max_rotation_rad_ = declare_parameter<double>("latch_max_rotation_rad", 0.26);
    latch_rotation_deadband_rps_ = declare_parameter<double>("latch_rotation_deadband_rps", 0.05);
    // Number of consecutive non-rotating GPS samples required before we
    // start accumulating a new baseline after a pivot ends. 2 samples
    // at 5 Hz GPS ≈ 400 ms of confirmed straight motion before COG
    // becomes a valid heading estimate again.
    rotation_quiet_min_samples_ = declare_parameter<int>("rotation_quiet_min_samples", 2);
    max_pos_accuracy_ = declare_parameter<double>("max_pos_accuracy_m", 0.05);
    min_dt_ = declare_parameter<double>("min_sample_dt_s", 0.05);
    max_dt_ = declare_parameter<double>("max_sample_dt_s", 0.50);
    max_yaw_var_ = declare_parameter<double>("max_yaw_variance", 3.0);
    min_yaw_var_ = declare_parameter<double>("min_yaw_variance", 7.6e-5);

    // Multi-sample baseline accumulator: don't publish a COG yaw until
    // the robot has travelled this distance since the anchor sample.
    // Per-pair F9P fixes at 8 Hz / 0.20 m/s yield ~25 mm displacement
    // and σ_yaw ≈ 18°, which floods fusion_graph with near-useless yaws
    // that the optimizer must integrate over many nodes before it
    // converges. Accumulating to e.g. 0.10 m gives σ_yaw ≈ 5° per
    // publish, so a single COG correction is enough to pull the graph.
    min_baseline_displacement_m_ =
        declare_parameter<double>("min_baseline_displacement_m", 0.10);

    // ── Stationary yaw latch ────────────────────────────────────────
    stationary_seed_rate_hz_ = declare_parameter<double>("stationary_seed_rate_hz", 2.0);
    stationary_drift_rate_ = declare_parameter<double>("stationary_yaw_drift_rate", 0.005);
    stationary_max_age_s_ = declare_parameter<double>("stationary_max_age_s", 600.0);

    // Datum for flat-earth ENU projection.
    datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
    datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
    datum_seeded_ = (datum_lat_ != 0.0 || datum_lon_ != 0.0);
    cos_datum_lat_ = std::cos(datum_lat_ * kDegToRad);

    // GPS antenna lever arm in base_link frame. Default matches the
    // YardForce 500 mount (gps_x=0.30 m in mowgli_robot.yaml) but is
    // expected to be overridden by the launch file from the same
    // source navsat_to_absolute_pose_node and fusion_graph read.
    // During a turn the antenna's velocity is v_body + ω × r_lever,
    // so atan2(dy, dx) ≠ ψ_body. We subtract atan2(ω·rx − ω·0, |v|)
    // (≈ ω·rx/|v| for small angles, exact for any angle via atan2)
    // from the raw COG to recover the body heading.
    lever_arm_x_ = declare_parameter<double>("lever_arm_x", 0.30);
    lever_arm_y_ = declare_parameter<double>("lever_arm_y", 0.0);
    // 1-σ noise on the ω used for bias correction. Covers both gyro
    // sensor noise (~0.02 rad/s) and the "constant ω across the
    // baseline" approximation we make by using the latest sample as a
    // proxy for the time-averaged rate. Feeds the σ_yaw inflation for
    // both the lever-arm and the mid-baseline drift corrections.
    omega_noise_rps_ = declare_parameter<double>("omega_noise_rps", 0.10);

    // ── Online mag calibration parameters ───────────────────────────
    enable_mag_cal_ = declare_parameter<bool>("enable_mag_cal", true);
    mag_cal_path_ = declare_parameter<std::string>("mag_calibration_path",
                                                   "/ros2_ws/maps/mag_calibration.yaml");
    mag_min_samples_ = declare_parameter<int>("mag_min_samples", 30);
    mag_max_samples_ = declare_parameter<int>("mag_max_samples", 600);
    mag_refit_period_sec_ = declare_parameter<double>("mag_refit_period_sec", 60.0);
    mag_refit_throttle_sec_ = declare_parameter<double>("mag_refit_throttle_sec", 300.0);

    auto qos = sensor_qos();
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps/fix",
        qos,
        [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
        {
          on_fix(*msg);
        });
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom",
        qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          on_wheel(*msg);
        });
    // Subscribe to /imu/data for the gyro_z rotation gate. At 100 Hz the
    // IMU sees the start of a pivot ~20 ms before /wheel_odom (50 Hz)
    // does, which is exactly the window in which a stale on_fix would
    // otherwise publish a wrong COG yaw.
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
        {
          gyro_z_ = msg->angular_velocity.z;
          // Integrate |gyro_z| since the anchor for the straight-baseline gate
          // in on_fix(). Reset happens on anchor (re)seed there.
          const double t = this->now().seconds();
          const double prev = last_imu_t_.exchange(t);
          const double dt = t - prev;
          if (prev > 0.0 && dt > 0.0 && dt < 0.5)
          {
            // CAS add (not a plain load+store) so a concurrent on_fix() reset
            // to 0 under a MultiThreadedExecutor can't be lost to a stale add.
            const double inc = std::abs(msg->angular_velocity.z) * dt;
            double cur = abs_dtheta_since_anchor_.load(std::memory_order_relaxed);
            while (!abs_dtheta_since_anchor_.compare_exchange_weak(cur, cur + inc))
            {
            }
            // Deadbanded rotation since the stationary latch was set. Unlike
            // abs_dtheta_since_anchor_ this ignores sub-deadband samples, so a
            // long stationary dwell doesn't accumulate gyro noise into a false
            // staleness trip. Drives the latch-invalidation in
            // republish_latched_when_stationary().
            const double linc =
                mowgli_localization::cog_latch_rotation_increment(msg->angular_velocity.z,
                                                                  dt,
                                                                  latch_rotation_deadband_rps_);
            if (linc > 0.0)
            {
              double lcur = abs_dtheta_since_latch_.load(std::memory_order_relaxed);
              while (!abs_dtheta_since_latch_.compare_exchange_weak(lcur, lcur + linc))
              {
              }
            }
          }
        });
    pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/cog_heading", qos);

    if (enable_mag_cal_)
    {
      mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
          "/imu/mag_raw",
          qos,
          [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
          {
            on_mag_raw(*msg);
          });
      mag_refit_timer_ = create_wall_timer(std::chrono::duration<double>(mag_refit_period_sec_),
                                           [this]()
                                           {
                                             maybe_refit_mag();
                                           });
    }

    stats_timer_ = create_wall_timer(std::chrono::seconds(30),
                                     [this]()
                                     {
                                       log_stats();
                                     });
    if (stationary_seed_rate_hz_ > 0.0)
    {
      stationary_timer_ =
          create_wall_timer(std::chrono::duration<double>(1.0 / stationary_seed_rate_hz_),
                            [this]()
                            {
                              republish_latched_when_stationary();
                            });
    }

    RCLCPP_INFO(get_logger(),
                "cog_to_imu started — publish /imu/cog_heading once the "
                "RTK-Fixed baseline accumulates %.3f m at |wheel_vx| > "
                "%.2f m/s. Turns up to |ω| = %.2f rad/s are accepted; "
                "the antenna lever-arm (r=[%+.2f, %+.2f] m) and the "
                "mid-baseline yaw drift are subtracted from the raw COG "
                "and σ_yaw is inflated by their ω-noise sensitivities "
                "(σ_ω=%.2f rad/s) so headland turns surface as high-σ "
                "measurements rather than biased ones.",
                min_baseline_displacement_m_,
                min_abs_wheel_,
                min_omega_for_anchor_,
                lever_arm_x_,
                lever_arm_y_,
                omega_noise_rps_);
  }

private:
  void on_wheel(const nav_msgs::msg::Odometry& msg)
  {
    wheel_vx_ = msg.twist.twist.linear.x;
    wheel_omega_ = msg.twist.twist.angular.z;
  }

  void on_mag_raw(const sensor_msgs::msg::MagneticField& msg)
  {
    latest_mag_ = std::array<double, 3>{static_cast<double>(msg.magnetic_field.x) * 1e6,
                                        static_cast<double>(msg.magnetic_field.y) * 1e6,
                                        static_cast<double>(msg.magnetic_field.z) * 1e6};
    latest_mag_valid_ = true;
  }

  void on_fix(const sensor_msgs::msg::NavSatFix& msg)
  {
    using sensor_msgs::msg::NavSatStatus;
    if (msg.status.status < NavSatStatus::STATUS_GBAS_FIX)
    {
      ++rejected_fix_;
      return;
    }

    const double var_lat = msg.position_covariance[0];
    const double var_lon = msg.position_covariance[4];
    double pos_acc;
    if (var_lat <= 0.0 || var_lon <= 0.0)
    {
      pos_acc = 10.0;
    }
    else
    {
      pos_acc = std::sqrt((var_lat + var_lon) * 0.5);
    }
    if (pos_acc > max_pos_accuracy_)
    {
      ++rejected_accuracy_;
      return;
    }

    if (!datum_seeded_)
    {
      datum_lat_ = msg.latitude;
      datum_lon_ = msg.longitude;
      cos_datum_lat_ = std::cos(datum_lat_ * kDegToRad);
      datum_seeded_ = true;
      RCLCPP_INFO(get_logger(),
                  "datum self-seeded from first RTK fix: lat=%.8f lon=%.8f",
                  datum_lat_,
                  datum_lon_);
    }

    const double x = (msg.longitude - datum_lon_) * cos_datum_lat_ * kMetersPerDeg;
    const double y = (msg.latitude - datum_lat_) * kMetersPerDeg;
    const double t = static_cast<double>(msg.header.stamp.sec) +
                     static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    pos_acc = std::max(pos_acc, 0.002);

    // Sample-to-sample dt sanity. A long gap means we lost lock or paused;
    // the anchor's age would inflate σ_pos and the wheel direction may
    // have flipped silently — drop the anchor and restart accumulation.
    if (have_last_sample_)
    {
      const double dt_sample = t - last_sample_t_;
      if (dt_sample > max_dt_)
      {
        have_anchor_ = false;
        have_last_sample_ = false;
      }
      else if (dt_sample < min_dt_)
      {
        // Duplicate / out-of-order; ignore but don't disturb anchor.
        return;
      }
    }
    last_sample_t_ = t;
    have_last_sample_ = true;

    // Rotation gate. During in-place pivots (PRE_ROTATE, headland turns)
    // the GPS antenna swings on a lever-arm arc around base_link. Each
    // arrival sees a 1–3 cm displacement in a direction perpendicular
    // to the current heading — atan2(dy, dx) of that displacement is
    // 90° off, and if the anchor was seeded just before the pivot the
    // accumulated baseline crosses min_baseline_displacement_m_ and a
    // wrong COG yaw gets published with a very tight (min_yaw_var_)
    // covariance, snapping the EKF map-frame yaw by tens of degrees.
    //
    // Watch BOTH /wheel_odom.twist.angular.z and /imu/data.gyro_z so
    // the rejection is robust to a brief lag in the wheel encoder
    // sample — the IMU updates at 100 Hz while wheel_odom is 50 Hz,
    // and at the transition translation→rotation the IMU sees the
    // pivot first. The latch also stays asserted for one extra cycle
    // (rotation_quiet_count_ debounce) so a single low gyro sample
    // doesn't unlatch us during the rotation.
    const bool rotating_wheels = std::abs(wheel_omega_) >= min_omega_for_anchor_;
    const bool rotating_imu = std::abs(gyro_z_) >= min_omega_for_anchor_;
    if (rotating_wheels || rotating_imu)
    {
      rotation_quiet_count_ = 0;
      ++rejected_rotating_;
      have_anchor_ = false;
      return;
    }
    // Require a few consecutive non-rotating samples before we trust
    // wheel_vx_ enough to seed a new anchor. Without this, the very
    // first GPS arrival after a pivot ends will already have a stale
    // antenna position from the rotation tail.
    if (rotation_quiet_count_ < rotation_quiet_min_samples_)
    {
      ++rotation_quiet_count_;
      ++rejected_rotating_;
      have_anchor_ = false;
      return;
    }

    // Stationary samples don't extend the baseline (GPS noise would just
    // pollute the integrated displacement). Drop the anchor so we don't
    // bake a stationary segment into the next yaw computation.
    if (std::abs(wheel_vx_) < min_abs_wheel_)
    {
      ++rejected_stationary_;
      have_anchor_ = false;
      return;
    }

    // Lever-arm-sweep dominance gate (see cog_yaw_math.hpp). The fixed
    // min_omega_for_anchor_ gate above only catches fast pivots (>=0.5
    // rad/s); slow dock-alignment rotations (0.1-0.3 rad/s) pass it but,
    // with a 0.3 m lever arm and near-zero real translation, still produce
    // a sweep-dominated GPS displacement whose COG heading is ~90° off the
    // body. Publishing it corrupts the fused yaw and drives the dock
    // approach the wrong way (field 2026-05-27). Use the larger of the
    // wheel/IMU rotation rate so a wheel-encoder lag at the rotation onset
    // doesn't let a sample through.
    const double rot_rate =
        std::max(std::abs(wheel_omega_.load()), std::abs(gyro_z_.load()));
    const double lever_radius = std::hypot(lever_arm_x_, lever_arm_y_);
    if (cog_sweep_dominates(rot_rate, lever_radius, wheel_vx_, cog_sweep_dominance_ratio_))
    {
      ++rejected_sweep_;
      have_anchor_ = false;
      return;
    }

    const int wheel_sign = (wheel_vx_ >= 0.0) ? 1 : -1;

    // Anchor seeding / direction-change reset.
    if (!have_anchor_ || (anchor_wheel_sign_ != 0 && anchor_wheel_sign_ != wheel_sign))
    {
      anchor_t_ = t;
      anchor_x_ = x;
      anchor_y_ = y;
      anchor_pa_ = pos_acc;
      anchor_wheel_sign_ = wheel_sign;
      have_anchor_ = true;
      abs_dtheta_since_anchor_ = 0.0;  // fresh straight baseline starts here
      return;
    }

    const double dx = x - anchor_x_;
    const double dy = y - anchor_y_;
    const double displacement = std::hypot(dx, dy);

    if (displacement < min_baseline_displacement_m_)
    {
      ++rejected_displacement_;
      return;
    }

    // Straight-baseline gate: if the heading swung more than
    // cog_max_baseline_rotation_rad_ over this baseline (a slow oscillation
    // that slipped under the instantaneous rotation/sweep gates), the GPS
    // displacement no longer points along the body axis — the COG heading
    // would be garbage. Drop the anchor and restart on a fresh straight run.
    if (abs_dtheta_since_anchor_.load() > cog_max_baseline_rotation_rad_)
    {
      ++rejected_baseline_rotation_;
      have_anchor_ = false;
      return;
    }

    if (wheel_sign > 0)
    {
      ++published_fwd_;
    }
    else
    {
      ++published_rev_;
    }

    // ── Unbias the COG against constant-rate turning ─────────────────
    // The antenna sits at r_lever in body frame, so its body-frame
    // velocity is (v_x - ω·r_y, ω·r_x) with SIGNED v_x. The GPS
    // displacement points along the antenna's world-frame velocity at
    // the baseline *midpoint* (ψ_anchor + ω·dt/2), not the current yaw,
    // so we subtract the lever-arm offset AND add the half-baseline drift
    // to recover the heading at the current sample.
    //
    // The lever-arm offset is direction-dependent: forward it is a small
    // first-quadrant angle, but in REVERSE the antenna body-x velocity is
    // negative so the offset lands near ±π — applying the forward-form
    // correction in reverse left the published COG yaw ~73° off the gyro
    // during the 2026-05-27 reverse+rotate teleop and fought the gyro
    // between-factor in fusion_graph. compute_cog_body_yaw() handles both
    // cases (signed v_x) — see cog_yaw_math.hpp. Forward path unchanged.
    //
    // Time-averaged ω is approximated by the latest gyro/wheel rate;
    // the resulting model error is folded into σ_yaw via omega_noise_rps_.
    const double omega_avg = 0.5 * (wheel_omega_.load() + gyro_z_.load());
    const double dt_baseline = std::max(t - anchor_t_, 1e-3);
    const double v_eff = std::max(std::abs(wheel_vx_.load()), min_abs_wheel_);
    const double vx_signed = wheel_vx_.load();

    const double yaw = compute_cog_body_yaw(
        dx, dy, wheel_sign, omega_avg, dt_baseline, vx_signed, lever_arm_x_, lever_arm_y_);

    // ── σ_yaw composition ────────────────────────────────────────────
    // 1. Positional noise → angular noise across the baseline (existing).
    // 2. Mid-baseline drift correction depends on ω; σ propagates as
    //      ∂(ω·dt/2)/∂ω = dt/2.
    // 3. Lever-arm correction:
    //      ∂/∂ω atan2(ω·rx, v - ω·ry)
    //        = (rx·(v - ω·ry) + ω·rx·ry) / ((v - ω·ry)² + (ω·rx)²)
    //        ≈ rx / v_eff for small ω
    //    Use the exact form so headland turns get a fair σ instead of
    //    one that linearises away the dominant term.
    const double sigma_pos = std::hypot(pos_acc, anchor_pa_);
    const double sigma_pos_yaw = std::atan2(2.0 * sigma_pos, std::max(displacement, 1e-3));
    const double sigma_drift = 0.5 * dt_baseline * omega_noise_rps_;
    // |∂lever/∂ω| is symmetric in v_x sign, so the same magnitude inflates
    // σ_yaw forward and reverse (uses |v_x| via v_eff).
    const double sigma_lever =
        compute_lever_sigma(omega_avg, v_eff, lever_arm_x_, lever_arm_y_, omega_noise_rps_);
    const double sigma_yaw_sq = sigma_pos_yaw * sigma_pos_yaw + sigma_drift * sigma_drift +
                                sigma_lever * sigma_lever;
    const double yaw_var = std::max(min_yaw_var_, std::min(sigma_yaw_sq, max_yaw_var_));

    // Advance the anchor to the current sample so the next baseline starts
    // fresh — keeps each published yaw independent and bounds the temporal
    // window over which wheel direction is assumed constant.
    anchor_t_ = t;
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_pa_ = pos_acc;
    anchor_wheel_sign_ = wheel_sign;
    abs_dtheta_since_anchor_ = 0.0;  // next baseline starts fresh

    // Stamp the heading with the GPS fix time, not wall-clock now(). The COG
    // yaw is derived from this fix's position; stamping it with arrival time
    // bakes in the GPS pipeline latency (~100-200 ms), which misassociates the
    // measurement with a later graph node during turns. msg.header.stamp is the
    // fix epoch.
    publish_imu(rclcpp::Time(msg.header.stamp), yaw, yaw_var);

    const double mono_now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    latched_yaw_ = LatchedYaw{mono_now, yaw, yaw_var};
    // Fresh latch — start counting rotation away from this heading from zero.
    abs_dtheta_since_latch_ = 0.0;

    // Online mag-cal sample collection — only fit against COG that
    // survives the inflated-σ test (positional + drift + lever).
    if (enable_mag_cal_ && latest_mag_valid_ &&
        std::sqrt(sigma_yaw_sq) < (15.0 * kDegToRad))
    {
      mag_samples_.emplace_back(latest_mag_[0], latest_mag_[1], latest_mag_[2], yaw);
      while (static_cast<int>(mag_samples_.size()) > mag_max_samples_)
      {
        mag_samples_.pop_front();
      }
    }
  }

  void republish_latched_when_stationary()
  {
    if (!latched_yaw_)
    {
      return;
    }
    if (std::abs(wheel_vx_) >= min_abs_wheel_)
    {
      return;
    }
    // Also skip the republish while the robot is *rotating in place*.
    // The latched yaw is the absolute heading from the last forward
    // motion segment — once the robot pivots, that anchor becomes
    // stale at ω rad/s. Republishing it as a tight-covariance EKF
    // measurement would pin the EKF's yaw against the gyro
    // integration that's correctly tracking the rotation. Watch BOTH
    // /wheel_odom and /imu — during tight FTC arcs the diff-drive
    // wheel_odom.angular.z momentarily reports near zero (one wheel
    // near-still) while the IMU correctly reports the body's rotation
    // rate. Without the IMU gate, the seed slips through, dumps a stale
    // measurement into the EKF with min_yaw_var_ covariance, and
    // snaps map→odom by tens of degrees — see the 153° jump at
    // session 2026-05-11-cog-gate-boundary-debounce, t≈78.28.
    //
    // 2026-05-27: this gate used min_omega_for_anchor_ (0.5 rad/s),
    // contradicting the "0.05 rad/s ≈ 3°/s" the comment always
    // claimed. The 10× too-high threshold let the dock graceful
    // controller's slow alignment pivots (0.1-0.3 rad/s) republish the
    // stale latched heading at 2 Hz, pinning the fused yaw against the
    // gyro and driving the dock approach the wrong way. This path is
    // the stationary-republish: ANY real rotation means the latch is
    // going stale, so gate at a low rate (latch_republish_max_omega).
    if (std::abs(wheel_omega_) >= latch_republish_max_omega_ ||
        std::abs(gyro_z_) >= latch_republish_max_omega_)
    {
      return;
    }
    // Cumulative-rotation staleness: the instantaneous gate above only catches
    // the moment of turning. If the robot has rotated past latch_max_rotation_rad_
    // since the latch was set (e.g. an in-place pivot that has since stopped),
    // the latched heading no longer describes where the robot points — drop it
    // rather than republish a stale, tight-covariance yaw into the graph. A new
    // latch is seeded by on_fix() once forward motion resumes.
    if (abs_dtheta_since_latch_.load() > latch_max_rotation_rad_)
    {
      latched_yaw_.reset();
      ++latch_invalidated_rotation_;
      return;
    }
    const double mono_now = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    const double age = std::max(0.0, mono_now - latched_yaw_->stamp);
    if (age > stationary_max_age_s_)
    {
      latched_yaw_.reset();
      return;
    }
    const double drift_var = (stationary_drift_rate_ * age) * (stationary_drift_rate_ * age);
    const double yaw_var =
        std::max(min_yaw_var_, std::min(latched_yaw_->base_var + drift_var, max_yaw_var_));
    publish_imu(now(), latched_yaw_->yaw, yaw_var);
    ++stationary_seeds_published_;
  }

  void publish_imu(const rclcpp::Time& stamp, double yaw, double yaw_var)
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
    imu.orientation_covariance[8] = yaw_var;
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
                "cog_to_imu stats: published fwd=%d rev=%d seed=%d, "
                "rejected fix=%d accuracy=%d stationary=%d rotating=%d sweep=%d "
                "displacement=%d baseline_rot=%lu latch_invalid_rot=%lu, "
                "mag_cal samples=%zu (writes=%d)",
                published_fwd_,
                published_rev_,
                stationary_seeds_published_,
                rejected_fix_,
                rejected_accuracy_,
                rejected_stationary_,
                rejected_rotating_,
                rejected_sweep_,
                rejected_displacement_,
                static_cast<unsigned long>(rejected_baseline_rotation_),
                static_cast<unsigned long>(latch_invalidated_rotation_),
                mag_samples_.size(),
                mag_fit_count_);
    published_fwd_ = 0;
    published_rev_ = 0;
    stationary_seeds_published_ = 0;
    rejected_fix_ = 0;
    rejected_accuracy_ = 0;
    rejected_stationary_ = 0;
    rejected_rotating_ = 0;
    rejected_sweep_ = 0;
    rejected_displacement_ = 0;
  }

  // ── Online magnetometer calibration ────────────────────────────────
  static bool headings_well_distributed(
      const std::deque<std::tuple<double, double, double, double>>& samples)
  {
    int bins[8] = {0};
    for (const auto& s : samples)
    {
      const double psi = std::get<3>(s);
      int idx = static_cast<int>(((psi + M_PI) / (2.0 * M_PI)) * 8.0);
      idx = ((idx % 8) + 8) % 8;
      bins[idx]++;
    }
    int populated = 0;
    for (int b : bins)
      if (b > 0)
        ++populated;
    return populated >= 3;
  }

  void maybe_refit_mag()
  {
    if (static_cast<int>(mag_samples_.size()) < mag_min_samples_)
    {
      return;
    }
    const double now_s = static_cast<double>(get_clock()->now().nanoseconds()) * 1e-9;
    if (mag_fit_count_ > 0 && (now_s - mag_last_write_t_) < mag_refit_throttle_sec_)
    {
      return;
    }
    if (!headings_well_distributed(mag_samples_))
    {
      RCLCPP_INFO(get_logger(),
                  "online mag fit: %zu samples but heading distribution too "
                  "narrow — waiting for more diverse motion.",
                  mag_samples_.size());
      return;
    }

    // Solve LS with normal equations: M^T M sol = M^T rhs.
    // Unknowns: [ox, oy, A, B] (4×4 symmetric).
    // Top equations:    bx = ox + A·sin ψ + B·cos ψ
    // Bottom equations: by = oy + A·cos ψ - B·sin ψ
    double mtm[4][4] = {{0}};
    double mtr[4] = {0};
    std::vector<double> bzs;
    bzs.reserve(mag_samples_.size());
    const std::size_t n = mag_samples_.size();

    for (const auto& s : mag_samples_)
    {
      const double bx = std::get<0>(s);
      const double by = std::get<1>(s);
      const double bz = std::get<2>(s);
      const double psi = std::get<3>(s);
      bzs.push_back(bz);
      const double sp = std::sin(psi);
      const double cp = std::cos(psi);
      // Top row: rt = [1, 0, sp, cp], target bx
      // Bot row: rb = [0, 1, cp, -sp], target by
      const double rt[4] = {1.0, 0.0, sp, cp};
      const double rb[4] = {0.0, 1.0, cp, -sp};
      for (int i = 0; i < 4; ++i)
      {
        mtr[i] += rt[i] * bx + rb[i] * by;
        for (int j = 0; j < 4; ++j)
        {
          mtm[i][j] += rt[i] * rt[j] + rb[i] * rb[j];
        }
      }
    }

    // Solve 4×4 via Gaussian elimination.
    double a[4][5];
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
        a[i][j] = mtm[i][j];
      a[i][4] = mtr[i];
    }
    for (int i = 0; i < 4; ++i)
    {
      // pivot
      int p = i;
      double pv = std::abs(a[p][i]);
      for (int k = i + 1; k < 4; ++k)
      {
        if (std::abs(a[k][i]) > pv)
        {
          pv = std::abs(a[k][i]);
          p = k;
        }
      }
      if (pv < 1e-12)
      {
        RCLCPP_WARN(get_logger(), "online mag fit: singular normal matrix, skipping.");
        return;
      }
      if (p != i)
      {
        for (int j = 0; j < 5; ++j)
          std::swap(a[i][j], a[p][j]);
      }
      const double inv = 1.0 / a[i][i];
      for (int j = i; j < 5; ++j)
        a[i][j] *= inv;
      for (int k = 0; k < 4; ++k)
      {
        if (k == i)
          continue;
        const double f = a[k][i];
        if (f == 0.0)
          continue;
        for (int j = i; j < 5; ++j)
          a[k][j] -= f * a[i][j];
      }
    }
    const double ox = a[0][4];
    const double oy = a[1][4];
    const double A_ = a[2][4];
    const double B_ = a[3][4];

    // Compute residual rms over both halves.
    double sse = 0.0;
    for (const auto& s : mag_samples_)
    {
      const double bx = std::get<0>(s);
      const double by = std::get<1>(s);
      const double psi = std::get<3>(s);
      const double sp = std::sin(psi);
      const double cp = std::cos(psi);
      const double pred_x = ox + A_ * sp + B_ * cp;
      const double pred_y = oy + A_ * cp - B_ * sp;
      sse += (bx - pred_x) * (bx - pred_x);
      sse += (by - pred_y) * (by - pred_y);
    }
    const double rms = std::sqrt(sse / static_cast<double>(2 * n));
    const double bh = std::sqrt(A_ * A_ + B_ * B_);

    if (!(bh >= 10.0 && bh <= 100.0))
    {
      RCLCPP_WARN(get_logger(),
                  "online mag fit rejected: |B_h|=%.1f µT out of range "
                  "(expected 25–60). N=%zu.",
                  bh,
                  n);
      return;
    }
    if (rms > 0.4 * bh)
    {
      RCLCPP_WARN(get_logger(),
                  "online mag fit rejected: rms=%.1f µT > 40%% of |B_h|=%.1f "
                  "µT. N=%zu.",
                  rms,
                  bh,
                  n);
      return;
    }

    // Median of bz.
    std::sort(bzs.begin(), bzs.end());
    const double oz = (bzs.size() % 2 == 1) ? bzs[bzs.size() / 2]
                                            : 0.5 * (bzs[bzs.size() / 2 - 1] + bzs[bzs.size() / 2]);

    try
    {
      write_mag_cal(ox, oy, oz, bh, rms, static_cast<int>(n));
    }
    catch (const std::exception& exc)
    {
      RCLCPP_ERROR(get_logger(), "online mag cal write failed: %s", exc.what());
      return;
    }
    mag_last_write_t_ = now_s;
    ++mag_fit_count_;
    RCLCPP_INFO(get_logger(),
                "online mag cal #%d: offset=(%+7.2f, %+7.2f, %+7.2f) µT  "
                "|B_h|=%.2f µT  rms=%.2f µT  N=%zu  → %s",
                mag_fit_count_,
                ox,
                oy,
                oz,
                bh,
                rms,
                n,
                mag_cal_path_.c_str());
  }

  void write_mag_cal(double ox, double oy, double oz, double bh, double rms, int n)
  {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "mag_calibration" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "offset_x_uT" << YAML::Value << ox;
    out << YAML::Key << "offset_y_uT" << YAML::Value << oy;
    out << YAML::Key << "offset_z_uT" << YAML::Value << oz;
    out << YAML::Key << "scale_x" << YAML::Value << 1.0;
    out << YAML::Key << "scale_y" << YAML::Value << 1.0;
    out << YAML::Key << "scale_z" << YAML::Value << 1.0;
    out << YAML::Key << "sample_count" << YAML::Value << n;
    out << YAML::Key << "magnitude_mean_uT" << YAML::Value << bh;
    out << YAML::Key << "magnitude_std_uT" << YAML::Value << rms;
    out << YAML::Key << "calibrated_at" << YAML::Value << utc_iso8601_now();
    out << YAML::Key << "source" << YAML::Value << "online_cog_fit";
    out << YAML::Key << "fit_count" << YAML::Value << (mag_fit_count_ + 1);
    out << YAML::EndMap;
    out << YAML::EndMap;

    namespace fs = std::filesystem;
    fs::path target(mag_cal_path_);
    if (target.has_parent_path())
    {
      std::error_code ec;
      fs::create_directories(target.parent_path(), ec);
    }
    const std::string tmp = mag_cal_path_ + ".tmp";
    {
      std::ofstream fh(tmp);
      if (!fh)
      {
        throw std::runtime_error("cannot open " + tmp + " for writing");
      }
      fh << out.c_str();
    }
    std::error_code ec;
    fs::rename(tmp, target, ec);
    if (ec)
    {
      throw std::runtime_error("rename failed: " + ec.message());
    }
  }

  // ── State ─────────────────────────────────────────────────────────
  double min_abs_wheel_{};
  double min_omega_for_anchor_{};
  double cog_sweep_dominance_ratio_{1.0};
  double latch_republish_max_omega_{0.05};
  double latch_max_rotation_rad_{0.26};
  double latch_rotation_deadband_rps_{0.05};
  double max_pos_accuracy_{};
  double min_dt_{}, max_dt_{};
  double max_yaw_var_{}, min_yaw_var_{};
  double stationary_seed_rate_hz_{};
  double stationary_drift_rate_{};
  double stationary_max_age_s_{};

  double datum_lat_{}, datum_lon_{};
  bool datum_seeded_{false};
  double cos_datum_lat_{1.0};

  double lever_arm_x_{0.30};
  double lever_arm_y_{0.0};
  double omega_noise_rps_{0.10};

  bool enable_mag_cal_{true};
  std::string mag_cal_path_;
  int mag_min_samples_{};
  int mag_max_samples_{};
  double mag_refit_period_sec_{};
  double mag_refit_throttle_sec_{};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_, stationary_timer_, mag_refit_timer_;

  // Baseline accumulator. The COG yaw is computed from the displacement
  // between an anchor sample and the current sample once the accumulated
  // displacement crosses min_baseline_displacement_m. The anchor advances
  // to the current sample after each successful publish; it is reset
  // whenever the wheel direction changes, the robot is stationary, or
  // the gap to the previous fix exceeds max_dt_.
  bool have_anchor_{false};
  double anchor_t_{}, anchor_x_{}, anchor_y_{}, anchor_pa_{};
  // Wheel sign at the anchor sample: +1 forward, -1 reverse, 0 unknown.
  // A change in sign during the baseline invalidates the anchor.
  int anchor_wheel_sign_{0};
  double last_sample_t_{};
  bool have_last_sample_{false};
  // wheel_vx_ / wheel_omega_ are mutated from the /wheel_odom callback
  // (50 Hz), gyro_z_ from the /imu/data callback (100 Hz). Both are
  // read by on_fix() (GPS callback, ~5 Hz) and by
  // republish_latched_when_stationary() (WallTimer, separate callback
  // group). The bringup uses MultiThreadedExecutor; without atomics
  // the read in on_fix could see a torn double from a partially
  // written update. std::atomic<double> with sequentially-consistent
  // ordering is a one-cycle penalty per read on x86 — negligible vs
  // GPS sample rate.
  std::atomic<double> wheel_vx_{0.0};
  std::atomic<double> wheel_omega_{0.0};
  std::atomic<double> gyro_z_{0.0};
  // Absolute heading change integrated since the current anchor (∫|gyro_z|dt,
  // accumulated in the /imu/data callback, reset on every anchor (re)seed).
  // The instantaneous min_omega_for_anchor_ / sweep gates only catch fast
  // pivots; a SLOW heading oscillation (e.g. an FTC limit-cycle on a tight
  // path segment) keeps each sample under threshold yet swings the heading
  // across the baseline, so the GPS displacement no longer points along the
  // body axis and the published COG yaw is garbage — which the non-robust COG
  // unary and the yaw-flip recovery then amplify into a 180° flip. Reject the
  // COG when this exceeds cog_max_baseline_rotation_rad_ (a straight baseline
  // is near zero). last_imu_t_ is the previous /imu/data stamp for dt.
  std::atomic<double> abs_dtheta_since_anchor_{0.0};
  // Deadbanded absolute rotation accumulated since the stationary latch was
  // set; drives latch invalidation in republish_latched_when_stationary().
  std::atomic<double> abs_dtheta_since_latch_{0.0};
  std::atomic<double> last_imu_t_{0.0};
  double cog_max_baseline_rotation_rad_{0.20};
  uint64_t rejected_baseline_rotation_{0};
  // Debounce counter for the post-rotation transition: how many
  // consecutive non-rotating samples we've seen since the last
  // detected pivot. Reset to 0 on every rotating sample; once it
  // reaches rotation_quiet_min_samples_, on_fix may seed an anchor
  // and accumulate baseline again.
  int rotation_quiet_count_{0};
  int rotation_quiet_min_samples_{2};
  double min_baseline_displacement_m_{};

  struct LatchedYaw
  {
    double stamp;
    double yaw;
    double base_var;
  };
  std::optional<LatchedYaw> latched_yaw_;

  std::array<double, 3> latest_mag_{};
  bool latest_mag_valid_{false};
  std::deque<std::tuple<double, double, double, double>> mag_samples_;
  double mag_last_write_t_{0.0};
  int mag_fit_count_{0};

  int published_fwd_{0}, published_rev_{0};
  int rejected_stationary_{0}, rejected_accuracy_{0}, rejected_fix_{0};
  int rejected_displacement_{0};
  int rejected_rotating_{0};
  int rejected_sweep_{0};
  int stationary_seeds_published_{0};
  uint64_t latch_invalidated_rotation_{0};
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::CogToImuNode>());
  rclcpp::shutdown();
  return 0;
}
