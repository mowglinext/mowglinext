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
 * @file gps_dock_detection_node.cpp
 * @brief Publish a GPS-derived dock "detection" so opennav_docking can
 *        approach the cradle off RTK-Fixed GPS instead of the corruptible
 *        map→odom factor-graph TF.
 *
 * ── Why this node exists ────────────────────────────────────────────────
 * On return-to-dock, fusion_graph (the GTSAM iSAM2 localizer that owns BOTH
 * map→odom AND odom→base_footprint) can come back CORRUPTED after reloading
 * its persisted graph (yaw flips, RTK overrides, a metre-scale jump). When
 * that happens the robot's map-frame TF pose is ~2 m off the true dock.
 *
 * opennav_docking (SimpleChargingDock + graceful controller, fixed_frame=odom,
 * base_frame=base_footprint) derives the robot↔dock geometry from TF. With
 * the default `use_external_detection_pose: false`, docking_server snapshots
 * map→odom EXACTLY ONCE at dockRobot() start (`transform(dock_pose, odom)`)
 * and then SimpleChargingDock::getRefinedPose() is a pure pass-through — the
 * dock target is frozen in odom at that one (possibly corrupt) snapshot. The
 * robot then spins in place / drives to the wrong spot and the action times
 * out. That single map→odom snapshot is the ONLY place graph corruption
 * enters the approach.
 *
 * Meanwhile /gps/absolute_pose is RTK-Fixed and cm-accurate at the cradle.
 *
 * ── What this node does ─────────────────────────────────────────────────
 * With `use_external_detection_pose: true`, getRefinedPose() instead reads
 * the latest message on the plugin's relative topic `detected_dock_pose`
 * (resolves to /detected_dock_pose — relative names resolve against the
 * docking_server NAMESPACE "/", not its node name), transforms it into the
 * fixed_frame (odom), filters it, and uses it as the live dock target. So if
 * we publish the TRUE dock location on that topic, the robot↔dock geometry
 * comes from US (GPS), not from the corrupt map→odom snapshot.
 *
 * Crucially we publish the detection ALREADY EXPRESSED IN `odom`: that way
 * getRefinedPose() sees detected.frame == pose.frame and SKIPS its internal
 * TF (it only transforms when frames differ). Publishing in `map` instead
 * would make opennav re-apply the corrupt map→odom and re-poison the target.
 *
 * The odom-frame dock pose needs the map→odom yaw offset Δ (= yaw_map→odom):
 *
 *   POSITION:
 *     p_dock_odom = p_robot_odom + R(−Δ)·(p_dock_map − p_robot_gps)
 *   The dock's displacement from the robot is a world ENU vector in MAP; odom is
 *     map rotated by Δ (odom is zeroed at the robot's start heading), so the
 *     displacement MUST be rotated by −Δ into odom before re-anchoring it at the
 *     robot's odom position. So position depends on Δ too — a wrong Δ flings the
 *     target to the wrong side, worse the farther from the dock (i.e. exactly
 *     during the approach). p_robot_gps is the RTK-Fixed, lever-arm-corrected
 *     base position; p_robot_odom is the continuous DR position (odom→base TF).
 *
 *   ORIENTATION:
 *     yaw_dock_odom = dock_pose_yaw − Δ
 *
 *   Δ SOURCE (delta_from_map_odom, default true): the live map→odom TF yaw — the
 *     EXACT offset, and reliable whenever the graph is healthy (gated on
 *     RTK-Fixed). The legacy alternative estimated Δ from /imu/cog_heading
 *     (course-over-ground minus odom yaw), but COG is MOTION direction, not
 *     heading: at the dock the robot turns/pivots/reverses, so COG ≠ heading and
 *     Δ is wrong — a bad seed then locks in (later correct samples look like
 *     >max_jump jumps and get rejected). 2026-06-26: COG held Δ=−106.5° while the
 *     true map→odom yaw was a stable +133°, flinging the dock ~2 m to the wrong
 *     side and out of bounds → 905. Whichever source feeds apply_delta_sample(),
 *     Δ is STABILISED: a jump larger than cog_yaw_max_jump_deg is rejected (for
 *     the TF source this guards a corrupt-reload yaw step; for COG, the
 *     reverse-flip), survivors EMA-smoothed, first sample seeds. Set
 *     delta_from_map_odom:=false to fall back to the COG estimator. The gate
 *     ALSO self-heals a STALE latch (issue #390): if it rejects a MUTUALLY
 *     consistent offset for reseed_after_s it re-seeds Δ to it, so a bad Δ
 *     slewed in during the docked mag/IMU recal cannot lock out recovery
 *     forever. See delta_gate.hpp.
 *
 * opennav re-transforms the odom-frame dock pose into base_footprint every
 * control loop via the (trusted, continuous) odom→base_footprint TF.
 *
 * ── RTK gate + Float fallback ───────────────────────────────────────────
 * Only RTK-Fixed /gps/absolute_pose samples drive a fresh detection. On
 * Float/no-fix we keep RE-PUBLISHING the LAST GOOD detection (it is anchored
 * in the continuous odom frame, so it stays geometrically valid as the robot
 * moves; opennav re-derives base-frame geometry from it each loop). We only
 * go silent if we have NEVER had a Fixed sample — then getRefinedPose() times
 * out (external_detection_timeout) and docking_server falls back to failing
 * the action, exactly as it would with no detector. The whole feature is
 * additionally behind the `use_gps_dock_detection` launch flag; when that is
 * off, `use_external_detection_pose` stays false and the legacy graph-TF
 * approach runs unchanged. This is the reversible escape hatch.
 *
 * ── Final-approach refinement (dock_approach_refinement.hpp) ────────────
 * While opennav_docking drives the final approach (forward commands on
 * /cmd_vel_docking), two things change:
 *   - TRACK FIT: Δ comes from a rigid fit between the antenna's RTK-Fixed map
 *     positions and the same point in odom, accumulated since the approach
 *     started, instead of the fused heading (gyro-only during the approach). It
 *     is blended in between track_fit_min_extent_m and track_fit_full_extent_m
 *     of travel and gated against the TF Δ.
 *   - FREEZE: inside freeze_distance_m of the dock the detection stops being
 *     re-anchored, so GPS noise cannot move the target where the 1/r control
 *     law is most sensitive to it.
 * Every sample is anchored on the GPS ANTENNA (lever arm from TF) rather than
 * on the lever-arm-corrected base, so the fused heading stays out of the
 * position as well, and the odom pose is read at the GPS stamp.
 *
 * fusion_graph keeps owning all TF — this node publishes NO TF and writes NO
 * graph state; it only publishes a PoseStamped on detected_dock_pose.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "mowgli_localization/delta_gate.hpp"
#include "mowgli_localization/dock_approach_refinement.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2/utils.hpp"  // tf2::getYaw
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // tf2::toMsg(Quaternion)
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace mowgli_localization
{

namespace
{
/// Convert an (x, y, yaw) pose to a planar tf2::Transform.
tf2::Transform make_planar_transform(double x, double y, double yaw)
{
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(x, y, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  t.setRotation(q);
  return t;
}
}  // namespace

class GpsDockDetectionNode : public rclcpp::Node
{
public:
  explicit GpsDockDetectionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("gps_dock_detection", options)
  {
    // Calibrated dock pose in the map frame (single source of truth =
    // mowgli_robot.yaml; flowed in by navigation.launch.py).
    dock_pose_x_ = declare_parameter<double>("dock_pose_x", 0.0);
    dock_pose_y_ = declare_parameter<double>("dock_pose_y", 0.0);
    dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

    // Frames. fixed_frame MUST match docking_server.fixed_frame (odom) so the
    // detection lands in the frame getRefinedPose() expects and skips its
    // internal TF. base_frame MUST match docking_server.base_frame.
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    // The map (global) frame whose offset from fixed_frame (odom) IS Δ.
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    // Δ (map→odom yaw) source. TRUE (default): read Δ directly from the live
    // map→odom TF — it is the EXACT offset and, when the graph is healthy, far
    // more reliable than COG. COG is course-OVER-GROUND (motion direction), NOT
    // robot heading: at the dock the robot turns/pivots/reverses, so COG ≠
    // heading and Δ=COG−odom_yaw is garbage; a bad seed then locks in (every
    // correct sample looks like a >max_jump "jump" and is rejected). The
    // 2026-06-26 divergence: COG held Δ=−106.5° while the true map→odom yaw was
    // a stable +133°, so the dock target was flung ~2 m to the wrong side and
    // out of bounds. FALSE restores the legacy COG estimator. Either way the
    // jump-rejection below still guards against a corrupt-reload Δ step.
    delta_from_map_odom_ = declare_parameter<bool>("delta_from_map_odom", true);

    // Republish cadence (Hz). opennav's external_detection_timeout default is
    // 1.0 s, so anything comfortably faster than 1 Hz keeps the detection
    // "fresh" between RTK-Fixed GPS samples (GPS is ~5-10 Hz).
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    // Require RTK-Fixed for a fresh detection. Set false to also accept Float
    // (NOT recommended here — Float is decimetre-level; the dock-localization
    // plan notes GPS may Float at some cradles, in which case the legacy
    // graph-TF path via use_gps_dock_detection:=false is the right choice).
    require_rtk_fixed_ = declare_parameter<bool>("require_rtk_fixed", true);

    // Δ stabilisation (applies to WHICHEVER source feeds apply_delta_sample()).
    // A genuine map→odom offset only drifts gradually, so: (a) reject any sample
    // that would move Δ by more than cog_yaw_max_jump_deg — for the TF source
    // this catches a corrupt-reload yaw step; for the COG source it catches the
    // reverse-flip/crawl glitch — keeping the last good Δ; (b) EMA-smooth the
    // surviving noise with cog_yaw_ema_alpha. The first sample seeds Δ directly.
    cog_yaw_max_jump_deg_ = declare_parameter<double>("cog_yaw_max_jump_deg", 25.0);
    cog_yaw_ema_alpha_ = declare_parameter<double>("cog_yaw_ema_alpha", 0.15);

    // Stale-latch recovery (issue #390). The jump gate above holds the last good
    // Δ across a transient, but must NOT latch a stale baseline forever: if the
    // docked IMU/mag recal slews Δ off truth, every correct sample afterwards
    // looks like a >max_jump jump and is rejected indefinitely. So if rejected
    // samples stay MUTUALLY consistent for reseed_after_s, the held baseline is
    // the outlier and Δ is re-seeded to the settled candidate. A flapping
    // transient never accumulates the window, so a glitch cannot re-latch. Set
    // <= 0 to disable (legacy hold-forever behaviour).
    reseed_after_s_ = declare_parameter<double>("reseed_after_s", 5.0);

    // Final-approach refinement (see dock_approach_refinement.hpp).
    track_fit_enabled_ = declare_parameter<bool>("track_fit_enabled", true);
    TrackFitConfig fit_cfg;
    fit_cfg.min_extent_m = declare_parameter<double>("track_fit_min_extent_m", 0.30);
    fit_cfg.full_extent_m = declare_parameter<double>("track_fit_full_extent_m", 0.60);
    fit_cfg.max_disagreement_rad =
        declare_parameter<double>("track_fit_max_disagreement_deg", 10.0) * M_PI / 180.0;
    fit_cfg.max_rms_m = declare_parameter<double>("track_fit_max_rms_m", 0.04);
    track_fit_ = TrackRotationFit(fit_cfg);
    freeze_distance_m_ = declare_parameter<double>("freeze_distance_m", 0.5);
    approach_episode_ =
        ApproachEpisode(declare_parameter<double>("approach_active_timeout_s", 1.0));
    gps_frame_ = declare_parameter<std::string>("gps_frame", "gps_link");

    gate_ = mowgli_localization::DeltaGate(mowgli_localization::DeltaGateConfig{
        cog_yaw_max_jump_deg_ * M_PI / 180.0, cog_yaw_ema_alpha_, reseed_after_s_});

    detection_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", rclcpp::QoS(1));

    abs_pose_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
        "/gps/absolute_pose",
        rclcpp::QoS(10),
        [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
        {
          on_absolute_pose(msg);
        });

    // The docking controller's output: forward commands mark the final approach.
    docking_cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_docking",
        rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
        {
          approach_episode_.on_command(msg->twist.linear.x, now().seconds());
        });

    // COG yaw (GPS course-over-ground, map frame, graph-INDEPENDENT) →
    // estimate the map→odom yaw offset Δ off the corruptible graph. SensorData
    // QoS to match cog_to_imu's publisher (best-effort).
    cog_sub_ =
        create_subscription<sensor_msgs::msg::Imu>("/imu/cog_heading",
                                                   rclcpp::SensorDataQoS(),
                                                   [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
                                                   {
                                                     on_cog_heading(msg);
                                                   });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               [this]()
                               {
                                 on_timer();
                               });

    RCLCPP_INFO(get_logger(),
                "gps_dock_detection started — dock_map=[%.3f, %.3f, %.3f] fixed=%s base=%s "
                "require_rtk_fixed=%s rate=%.1f Hz. Publishing detected_dock_pose "
                "(→ /detected_dock_pose).",
                dock_pose_x_,
                dock_pose_y_,
                dock_pose_yaw_,
                fixed_frame_.c_str(),
                base_frame_.c_str(),
                require_rtk_fixed_ ? "true" : "false",
                publish_rate_hz_);
  }

private:
  void on_absolute_pose(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
  {
    using AbsPose = mowgli_interfaces::msg::AbsolutePose;

    // Episode bookkeeping first, so an approach that ends under Float still
    // resets the fit and the freeze.
    const bool approach = update_approach_state();

    // RTK-Fixed gate. Float / DR / no-fix do NOT refresh the detection; we
    // keep republishing the last good one (see on_timer / class doc).
    const bool is_fixed = (msg->flags & AbsPose::FLAG_GPS_RTK_FIXED) != 0;
    if (require_rtk_fixed_ && !is_fixed)
    {
      return;
    }

    // Inside the freeze radius: hold the detection for the rest of the approach.
    if (approach && frozen_)
    {
      return;
    }

    // Robot's continuous-DR pose in odom (TF odom→base_footprint) at the GPS
    // stamp, so a moving robot is not paired with a newer odom pose. This is the
    // half of the graph that never jumps; map→odom does NOT enter here.
    tf2::Transform robot_odom;
    if (!lookup_robot_odom(robot_odom, rclcpp::Time(msg->header.stamp)))
    {
      return;  // odom→base_footprint not available yet; try next sample.
    }

    // Refresh Δ (map→odom yaw) from the live TF BEFORE using it below. This is
    // the EXACT offset both the position and orientation depend on; reading it
    // here (gated on RTK-Fixed via the early-return above) is far more reliable
    // than the COG estimator, which inverts on the slow/reverse dock approach.
    // apply_delta_sample() jump-rejects a corrupt-reload step and EMA-smooths.
    double tf_delta = gate_.value();
    bool have_tf_delta = false;
    if (delta_from_map_odom_)
    {
      double yaw_mo;
      if (lookup_map_odom_yaw(yaw_mo))
      {
        apply_delta_sample(yaw_mo, "map→odom TF");
        tf_delta = yaw_mo;
        have_tf_delta = true;
      }
    }

    // Anchor on the ANTENNA, seen in both frames. navsat_to_absolute_pose
    // subtracted R(fused map yaw)·lever from it; the fused map yaw is
    // odom yaw + map→odom yaw, so adding the same term back recovers the
    // measured antenna, free of the fused heading.
    const double robot_yaw_odom = tf2::getYaw(robot_odom.getRotation());
    const Vec2 lever = lever_arm();
    const Vec2 lever_odom = rotate(lever, robot_yaw_odom);
    const Vec2 lever_map =
        rotate(lever, robot_yaw_odom + (have_tf_delta ? tf_delta : gate_.value()));
    const Vec2 antenna_odom{robot_odom.getOrigin().x() + lever_odom.x,
                            robot_odom.getOrigin().y() + lever_odom.y};
    const Vec2 antenna_map{msg->pose.pose.position.x + lever_map.x,
                           msg->pose.pose.position.y + lever_map.y};

    // Δ = gate_.value() (stabilised TF Δ), replaced during the final approach by
    // the GPS track fit once the robot has travelled far enough.
    double delta = gate_.value();
    if (approach && track_fit_enabled_)
    {
      track_fit_.add(antenna_odom, antenna_map);
      last_fit_ = track_fit_.fit();
      last_blend_ = blend_track_delta(delta, last_fit_, track_fit_.config());
      delta = last_blend_.delta;
    }

    // POSITION + ORIENTATION: re-anchor the world antenna→dock displacement at
    // the antenna's odom position, rotated by −Δ (map→odom is NOT rotation-free:
    // odom is zeroed at the robot's start heading). The dock heading in odom is
    // dock_pose_yaw − Δ. Both use the same Δ so they stay consistent.
    const PlanarPose dock_odom = dock_in_odom(antenna_odom,
                                              antenna_map,
                                              {dock_pose_x_, dock_pose_y_, dock_pose_yaw_},
                                              delta);
    last_dock_odom_ = make_planar_transform(dock_odom.x, dock_odom.y, dock_odom.yaw);
    have_detection_ = true;

    const Vec2 robot_xy{robot_odom.getOrigin().x(), robot_odom.getOrigin().y()};
    if (approach && should_freeze(robot_xy, dock_odom, freeze_distance_m_))
    {
      frozen_ = true;
      RCLCPP_INFO(get_logger(),
                  "Dock approach: within %.2f m of the dock, holding the detection. %s",
                  freeze_distance_m_,
                  describe_fit().c_str());
    }
  }

  // Track start/end of the forward final approach. Returns whether it is active.
  bool update_approach_state()
  {
    const bool active = approach_episode_.active(now().seconds());
    if (active == approach_active_)
    {
      return active;
    }
    if (active)
    {
      RCLCPP_INFO(get_logger(), "Dock approach started: GPS track fit reset.");
    }
    else
    {
      RCLCPP_INFO(get_logger(),
                  "Dock approach ended (frozen=%s). %s",
                  frozen_ ? "yes" : "no",
                  describe_fit().c_str());
    }
    approach_active_ = active;
    track_fit_.reset();
    last_fit_ = TrackFitResult{};
    last_blend_ = TrackBlend{};
    frozen_ = false;
    return active;
  }

  std::string describe_fit() const
  {
    static const char* const kStates[] = {"tf_only", "blending", "track", "rejected"};
    const double deg = 180.0 / M_PI;
    char buf[256];
    std::snprintf(buf,
                  sizeof(buf),
                  "track fit: state=%s weight=%.2f pairs=%zu extent=%.2f m rms=%.3f m "
                  "fit=%.2f° tf=%.2f° (fit−tf=%.2f°)",
                  kStates[static_cast<int>(last_blend_.state)],
                  last_blend_.weight,
                  last_fit_.pairs,
                  last_fit_.extent_m,
                  last_fit_.rms_m,
                  last_fit_.delta * deg,
                  gate_.value() * deg,
                  wrap_to_pi(last_fit_.delta - gate_.value()) * deg);
    return std::string(buf);
  }

  // Antenna offset in base_frame (static URDF TF), resolved once. Zero until
  // available, which reproduces the pre-refinement base-anchored behaviour.
  Vec2 lever_arm()
  {
    if (!lever_known_)
    {
      try
      {
        const auto tf = tf_buffer_->lookupTransform(base_frame_, gps_frame_, tf2::TimePointZero);
        lever_ = {tf.transform.translation.x, tf.transform.translation.y};
        lever_known_ = true;
        RCLCPP_INFO(get_logger(),
                    "GPS lever arm from TF %s→%s: x=%.3f y=%.3f",
                    base_frame_.c_str(),
                    gps_frame_.c_str(),
                    lever_.x,
                    lever_.y);
      }
      catch (const tf2::TransformException&)
      {
        return {};
      }
    }
    return lever_;
  }

  // Track the map→odom yaw offset Δ from GPS course-over-ground, WITHOUT the
  // factor graph: Δ = yaw_cog_map − yaw_robot_odom, sampled at COG arrival.
  void on_cog_heading(sensor_msgs::msg::Imu::ConstSharedPtr msg)
  {
    // COG is the LEGACY Δ source. Disabled by default: at the dock the robot
    // turns/pivots/reverses, so course-over-ground ≠ heading and Δ is wrong
    // (the 2026-06-26 divergence). The map→odom TF in on_absolute_pose is the
    // default source. Kept behind delta_from_map_odom:=false for fallback.
    if (delta_from_map_odom_)
    {
      return;
    }
    tf2::Transform robot_odom;
    if (!lookup_robot_odom(robot_odom))
    {
      return;
    }
    tf2::Quaternion q(msg->orientation.x,
                      msg->orientation.y,
                      msg->orientation.z,
                      msg->orientation.w);
    const double yaw_cog_map = tf2::getYaw(q);
    const double yaw_robot_odom = tf2::getYaw(robot_odom.getRotation());
    apply_delta_sample(yaw_cog_map - yaw_robot_odom, "COG");
  }

  // Fold a Δ (map→odom yaw) measurement through the stabilisation gate (seed /
  // jump-reject+EMA / stale-latch re-seed) and log the outcome. All the math
  // lives in delta_gate.hpp so it unit-tests without ROS.
  void apply_delta_sample(double delta, const char* source)
  {
    const double deg = 180.0 / M_PI;
    switch (gate_.update(delta, now().seconds()))
    {
      case mowgli_localization::DeltaGateAction::SEEDED:
        RCLCPP_INFO(get_logger(),
                    "Acquired map→odom yaw offset Δ=%.1f° from %s.",
                    gate_.value() * deg,
                    source);
        break;
      case mowgli_localization::DeltaGateAction::RESEEDED:
        RCLCPP_WARN(get_logger(),
                    "Re-seeding stale map→odom Δ from %s: held baseline rejected a consistent "
                    "offset for ≥%.1f s — adopting Δ=%.1f° (stale-latch recovery, issue #390).",
                    source,
                    reseed_after_s_,
                    gate_.value() * deg);
        break;
      case mowgli_localization::DeltaGateAction::REJECTED:
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Rejecting Δ jump of %.0f° from %s (corrupt-reload / crawl noise); "
                             "holding Δ=%.1f°.",
                             gate_.last_jump_rad() * deg,
                             source,
                             gate_.value() * deg);
        break;
      case mowgli_localization::DeltaGateAction::ACCEPTED:
        break;
    }
  }

  // Δ = yaw of the live map→odom TF (the EXACT map→odom offset). Returns false
  // if the TF isn't available yet (fusion_graph not publishing).
  bool lookup_map_odom_yaw(double& yaw)
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(map_frame_, fixed_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
    tf2::Quaternion q(tf.transform.rotation.x,
                      tf.transform.rotation.y,
                      tf.transform.rotation.z,
                      tf.transform.rotation.w);
    yaw = tf2::getYaw(q);
    return true;
  }

  void on_timer()
  {
    if (!have_detection_)
    {
      // Never had a Fixed sample → publish nothing. getRefinedPose() then
      // times out and docking_server fails the action (legacy no-detector
      // behaviour). The use_gps_dock_detection launch flag is the operator's
      // escape hatch to the graph-TF path.
      return;
    }

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = now();  // fresh stamp every tick (timeout gate)
    out.header.frame_id = fixed_frame_;  // odom → getRefinedPose skips its TF
    out.pose.position.x = last_dock_odom_.getOrigin().x();
    out.pose.position.y = last_dock_odom_.getOrigin().y();
    out.pose.position.z = 0.0;
    out.pose.orientation = tf2::toMsg(last_dock_odom_.getRotation());
    detection_pub_->publish(out);
  }

  // odom→base_frame at `stamp`, falling back to the latest transform when the
  // stamp is outside the buffer (e.g. a GPS stamp just ahead of the last TF).
  bool lookup_robot_odom(tf2::Transform& out, const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, stamp);
    }
    catch (const tf2::TransformException&)
    {
      return lookup_robot_odom(out);
    }
    to_transform(tf, out);
    return true;
  }

  bool lookup_robot_odom(tf2::Transform& out)
  {
    geometry_msgs::msg::TransformStamped tf;
    try
    {
      tf = tf_buffer_->lookupTransform(fixed_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
    to_transform(tf, out);
    return true;
  }

  static void to_transform(const geometry_msgs::msg::TransformStamped& tf, tf2::Transform& out)
  {
    out.setOrigin(tf2::Vector3(tf.transform.translation.x,
                               tf.transform.translation.y,
                               tf.transform.translation.z));
    out.setRotation(tf2::Quaternion(tf.transform.rotation.x,
                                    tf.transform.rotation.y,
                                    tf.transform.rotation.z,
                                    tf.transform.rotation.w));
  }

  // Parameters
  double dock_pose_x_{0.0};
  double dock_pose_y_{0.0};
  double dock_pose_yaw_{0.0};
  std::string fixed_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  std::string map_frame_{"map"};
  double publish_rate_hz_{10.0};
  bool require_rtk_fixed_{true};
  bool delta_from_map_odom_{true};
  double cog_yaw_max_jump_deg_{25.0};
  double cog_yaw_ema_alpha_{0.15};
  double reseed_after_s_{5.0};
  bool track_fit_enabled_{true};
  double freeze_distance_m_{0.5};
  std::string gps_frame_{"gps_link"};

  // State
  tf2::Transform last_dock_odom_;
  bool have_detection_{false};
  // map→odom yaw offset Δ estimator. By default fed from the live map→odom TF
  // (delta_from_map_odom_); legacy COG estimator when false. Owns the seed /
  // jump-reject+EMA / stale-latch re-seed logic (delta_gate.hpp). 0 until seeded.
  mowgli_localization::DeltaGate gate_{};
  // Final approach (dock_approach_refinement.hpp).
  ApproachEpisode approach_episode_{};
  bool approach_active_{false};
  bool frozen_{false};
  TrackRotationFit track_fit_{};
  TrackFitResult last_fit_{};
  TrackBlend last_blend_{};
  bool lever_known_{false};
  Vec2 lever_{};

  // ROS
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr detection_pub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr abs_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr cog_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr docking_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace mowgli_localization

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::GpsDockDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
