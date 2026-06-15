// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// FusionGraphNode — SetupCommunications — publishers/subscriptions/services/timers. (The node
// implementation is split across several translation units to keep each file within the project's
// 600-line budget; all share fusion_graph_node.hpp + fusion_graph_node_util.hpp.)

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/fusion_graph_node_util.hpp"

namespace fusion_graph
{

void FusionGraphNode::SetupCommunications(double node_period_s)
{
  const double periodic_save_period_s = declare_parameter<double>("periodic_save_period_s", 300.0);

  // ── TF ────────────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Pubs/subs ─────────────────────────────────────────────────────
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered_map", 10);
  // /odometry/filtered + odom→base_footprint TF — replaces ekf_odom_node.
  // Continuous local-frame dead reckoning (wheel vx + gyro_z), never
  // sees GPS, never jumps. Nav2's odom_topic in nav2_params.yaml still
  // points here.
  pub_local_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

  // High-rate extrapolated pose (item #15). Off by default — set
  // fast_pose_publish_rate_hz > 0 in yaml to enable. 100 Hz is the
  // intended use; the publisher reuses the latest fusion pose and
  // projects yaw forward by latest IMU gyro_z. Marked as a
  // best-effort feed because consumers should fall back to the
  // canonical /odometry/filtered_map for control loops.
  fast_pose_publish_rate_hz_ = declare_parameter<double>("fast_pose_publish_rate_hz", 0.0);
  if (fast_pose_publish_rate_hz_ > 0.0)
  {
    pub_odom_fast_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered_map_fast",
                                                               rclcpp::SensorDataQoS());
    const auto period = std::chrono::duration<double>(1.0 / fast_pose_publish_rate_hz_);
    fast_pose_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                          [this]()
                          {
                            if (!pose_extrap_.HasFusionPose())
                              return;
                            const double now_s = this->now().seconds();
                            auto extrap = pose_extrap_.Extrapolate(now_s);
                            if (!extrap)
                              return;
                            nav_msgs::msg::Odometry msg;
                            msg.header.stamp = this->now();
                            msg.header.frame_id = map_frame_;
                            msg.child_frame_id = base_frame_;
                            msg.pose.pose.position.x = extrap->x();
                            msg.pose.pose.position.y = extrap->y();
                            msg.pose.pose.position.z = 0.0;
                            msg.pose.pose.orientation = QuatFromYaw(extrap->theta());
                            // Loose covariance — this is a display / latency-sensitive
                            // feed, not a primary measurement. Consumers that need
                            // tight σ should subscribe to /odometry/filtered_map.
                            for (int i = 0; i < 36; ++i)
                              msg.pose.covariance[i] = 0.0;
                            msg.pose.covariance[0] = 0.10 * 0.10;
                            msg.pose.covariance[7] = 0.10 * 0.10;
                            msg.pose.covariance[35] = 0.10 * 0.10;
                            pub_odom_fast_->publish(msg);
                          });
    RCLCPP_INFO(get_logger(),
                "fusion_graph: high-rate pose extrapolator enabled at %.1f Hz",
                fast_pose_publish_rate_hz_);
  }

  // /imu/fg_yaw — yaw-only sensor_msgs/Imu published BEST_EFFORT to
  // match cog_to_imu / mag_yaw_publisher conventions. Lets ekf_map_node
  // (when running as primary in observer mode) subscribe as a yaw
  // source, replacing the mag_yaw_publisher slot. fusion_graph yaw is
  // typically σ ≈ 0.5° vs ekf's σ ≈ 13° in stationary, so this
  // dramatically tightens the EKF map-frame yaw without changing the
  // primary localizer.
  pub_fg_yaw_ = create_publisher<sensor_msgs::msg::Imu>("/imu/fg_yaw", rclcpp::SensorDataQoS());
  pub_diag_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/fusion_graph/diagnostics", 10);
  pub_markers_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/fusion_graph/markers",
                                                             rclcpp::QoS(1).transient_local());

  auto sensor_qos = rclcpp::SensorDataQoS();

  sub_wheel_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel_odom", 50, std::bind(&FusionGraphNode::OnWheelOdom, this, std::placeholders::_1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos, std::bind(&FusionGraphNode::OnImu, this, std::placeholders::_1));

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix", sensor_qos, std::bind(&FusionGraphNode::OnGnss, this, std::placeholders::_1));

  // /imu/cog_heading and /imu/mag_yaw are published BEST_EFFORT by
  // cog_to_imu.py and mag_yaw_publisher.py — use SensorDataQoS or
  // the subscription is silently dropped at the QoS handshake.
  sub_cog_ = create_subscription<sensor_msgs::msg::Imu>("/imu/cog_heading",
                                                        sensor_qos,
                                                        std::bind(&FusionGraphNode::OnCogHeading,
                                                                  this,
                                                                  std::placeholders::_1));

  if (use_magnetometer_)
  {
    sub_mag_ = create_subscription<sensor_msgs::msg::Imu>("/imu/mag_yaw",
                                                          sensor_qos,
                                                          std::bind(&FusionGraphNode::OnMagYaw,
                                                                    this,
                                                                    std::placeholders::_1));
  }

  if (use_scan_matching_ || loop_closure_enabled_)
  {
    // Default to the deskewed scan so the matcher gets rotation-deskew (and,
    // once scan_deskew_node's linear comp is enabled, translation-deskew too).
    // scan_deskew_node passes through when its IMU is stale, so /scan_deskewed
    // is never worse than raw /scan. Override to "/scan" to bypass deskew.
    const std::string scan_topic = declare_parameter<std::string>("scan_topic", "/scan_deskewed");
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, sensor_qos, std::bind(&FusionGraphNode::OnScan, this, std::placeholders::_1));
  }

  // /hardware_bridge/status is always subscribed — OnHardwareStatus
  // serves two purposes that are independent of auto-save:
  //   1. Dock-arrival pose seed (rising edge of is_charging anchors
  //      the graph at the operator-calibrated dock_pose_*).
  //   2. Auto-checkpoint to disk (gated on auto_save_enabled_).
  sub_hw_status_ = create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      10,
      std::bind(&FusionGraphNode::OnHardwareStatus, this, std::placeholders::_1));

  // /cmd_vel_docking = the graceful dock controller's output; only published
  // during the final approach. Used as the "dock approach in progress" signal
  // for the COG/GPS stabilisation gates (DockingApproachActive()).
  sub_docking_cmd_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_docking",
      rclcpp::SensorDataQoS(),
      std::bind(&FusionGraphNode::OnDockingCmd, this, std::placeholders::_1));

  if (auto_save_enabled_)
  {
    sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        10,
        std::bind(&FusionGraphNode::OnHighLevelStatus, this, std::placeholders::_1));
    if (periodic_save_period_s > 0.0)
    {
      periodic_save_timer_ =
          create_wall_timer(std::chrono::duration<double>(periodic_save_period_s),
                            std::bind(&FusionGraphNode::OnPeriodicSaveTimer, this));
    }
  }

  // ── External set-pose channel ───────────────────────────────────
  // Equivalent to robot_localization's /<node>/set_pose: takes a
  // PoseWithCovarianceStamped, anchors the latest graph node at the
  // given pose with covariance-derived sigmas. Used by the BT
  // calibration nodes after a yaw-cal manoeuvre. The dock-arrival
  // seed (formerly dock_yaw_to_set_pose_node) now bypasses this
  // topic entirely — see SeedFromDockPose().
  //
  // QoS: TRANSIENT_LOCAL with depth-1, matching dock_yaw_to_set_pose's
  // publisher. The boot seed is a one-shot rising-edge event; with
  // VOLATILE either side, a subscriber that hasn't finished discovery
  // when the message is published silently loses it and the graph
  // never bootstraps (observed 2026-05-03 after a force-recreate). With
  // TL on both sides, a late-joining subscriber gets the last seed
  // pose latched on connect — node bootstraps without manual republish.
  rclcpp::QoS set_pose_qos(rclcpp::KeepLast(1));
  set_pose_qos.reliable();
  set_pose_qos.transient_local();
  sub_set_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/set_pose",
      set_pose_qos,
      std::bind(&FusionGraphNode::OnSetPose, this, std::placeholders::_1));

  // ── Save-graph service ──────────────────────────────────────────
  // Trigger from the GUI / a BT node when transitioning out of
  // RECORDING, or manually via:
  //   ros2 service call /fusion_graph_node/save_graph std_srvs/Trigger
  srv_save_ = create_service<std_srvs::srv::Trigger>(
      "~/save_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        const bool ok = graph_->Save(graph_save_prefix_);
        resp->success = ok;
        resp->message = ok ? "saved to " + graph_save_prefix_ + ".*" : "save failed";
        if (ok)
          RCLCPP_INFO(get_logger(), "fusion_graph: %s", resp->message.c_str());
        else
          RCLCPP_WARN(get_logger(), "fusion_graph: %s", resp->message.c_str());
      });

  // ── Clear-graph service ─────────────────────────────────────────
  // Wipes iSAM2 + accumulated factors + per-node scans + loop-closure
  // edges. The node stays alive; the next valid pose seed (GPS, set_pose
  // or scan-match relocalization) re-initializes the graph.
  // Trigger from the GUI when the operator wants to start a clean
  // session (e.g. after relocating to a new garden) without restarting
  // the whole stack:
  //   ros2 service call /fusion_graph_node/clear_graph std_srvs/Trigger
  srv_clear_ = create_service<std_srvs::srv::Trigger>(
      "~/clear_graph",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        graph_->Reset();
        // clear_graph is the explicit operator full-wipe — also drop the
        // RTK-anchored keyframe map (Reset() alone preserves it, since it is
        // the self-heal path). Re-arm capture spacing.
        graph_->ClearKeyframes();
        last_kf_capture_xy_.reset();
        // Drop the latched seed too, otherwise a stale GPS / yaw seed
        // from before the clear would re-initialize the graph at the
        // old position the operator was trying to escape.
        seed_xy_.reset();
        seed_yaw_.reset();
        seed_xy_rtk_fixed_ = false;
        // Re-zero the dead-reckoning frame. Without this the odom→base
        // transform keeps whatever offset it had accumulated (observed
        // 74 m), so map→odom = graph_pose ∘ dr⁻¹ still has the huge
        // lever arm that amplifies any graph-vs-DR yaw difference into
        // metres of map-pose jump — i.e. clearing the graph alone does
        // NOT stop the robot from "sliding". Resetting dr_* collapses
        // the lever arm to zero so map→odom tracks the fresh graph
        // directly. The odom→base TF jumps here, but clear_graph is an
        // explicit operator escape hatch (robot parked), so the local
        // costmap discontinuity is acceptable and self-heals on the
        // next costmap clear.
        {
          // tf_state_mu_: dr_* / anchor are read concurrently by
          // TfBroadcastLoop — reset them atomically so the loop never
          // sees zeroed dr_* paired with a stale (still-valid) anchor.
          std::lock_guard<std::mutex> lock(tf_state_mu_);
          dr_x_ = 0.0;
          dr_y_ = 0.0;
          dr_yaw_ = 0.0;
          t_map_odom_anchor_valid_ = false;
        }
        resp->success = true;
        resp->message = "graph cleared + odom re-based (waiting for re-initialization)";
        RCLCPP_WARN(get_logger(), "fusion_graph: %s", resp->message.c_str());
      });

  // ── Tick timer ────────────────────────────────────────────────────
  // Run at 1× node rate. Earlier 2× oversampling existed "to never
  // miss a node window" but doubled per-Tick CPU (ICP runs every
  // OnTimer call) for no functional gain — Tick() short-circuits when
  // dt < node_period_s, so a late wall_timer just creates the next
  // node a few ms late, with no graph-level effect.
  const double timer_period_s = node_period_s;
  tick_timer_ = create_wall_timer(std::chrono::duration<double>(timer_period_s),
                                  std::bind(&FusionGraphNode::OnTimer, this));

  // Maintenance timer at 30 s: prune old scans + check if iSAM2
  // needs to be rebased. PruneOldScans is cheap (just erasing old
  // entries under the lock) and stays inline. The rebase, however,
  // rebuilds the Bayes tree from ~50k PriorFactors — ~1 s of CPU
  // that used to block the executor and stall the map→odom TF
  // (observed 2026-05-14, caused DockRobot to abort with
  // `Transform data too old`). Dispatch it to a detached worker so
  // the callback returns immediately; GraphManager::RebaseISAM2
  // now does the heavy reconstruction outside the graph mutex and
  // only takes the lock briefly to replay pending factors + swap.
  maintenance_timer_ =
      create_wall_timer(std::chrono::seconds(30),
                        [this]()
                        {
                          if (!graph_->IsInitialized())
                            return;
                          graph_->PruneOldScans(scan_retention_nodes_);
                          const auto stats = graph_->Stats();
                          if (stats.total_nodes - last_rebase_index_ < isam2_rebase_every_nodes_)
                            return;
                          bool expected = false;
                          if (!rebase_in_flight_->compare_exchange_strong(expected, true))
                          {
                            // Previous async rebase still running — skip and try again
                            // at the next maintenance tick.
                            return;
                          }
                          // Commit the bookkeeping NOW so the next 30 s maintenance
                          // tick doesn't re-trigger while the worker is still building.
                          last_rebase_index_ = stats.total_nodes;
                          std::thread(
                              [graph = graph_,
                               logger = get_logger(),
                               total = stats.total_nodes,
                               flag = rebase_in_flight_]()
                              {
                                graph->RebaseISAM2();
                                RCLCPP_INFO(logger,
                                            "fusion_graph: iSAM2 rebased at node %lu",
                                            static_cast<unsigned long>(total));
                                flag->store(false);
                              })
                              .detach();
                        });

  // Diagnostics timer at 1 Hz — coarse, just for the session monitor.
  diag_timer_ =
      create_wall_timer(std::chrono::seconds(1),
                        [this]()
                        {
                          auto stats = graph_->Stats();
                          auto snap = graph_->LatestSnapshot();

                          diagnostic_msgs::msg::DiagnosticArray msg;
                          msg.header.stamp = this->now();
                          diagnostic_msgs::msg::DiagnosticStatus s;
                          s.name = "fusion_graph";
                          s.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                          s.message = graph_->IsInitialized() ? "running" : "waiting init";

                          auto add = [&s](const std::string& k, const std::string& v)
                          {
                            diagnostic_msgs::msg::KeyValue kv;
                            kv.key = k;
                            kv.value = v;
                            s.values.push_back(kv);
                          };
                          add("total_nodes", std::to_string(stats.total_nodes));
                          add("scans_attached", std::to_string(stats.scans_attached));
                          add("loop_closures", std::to_string(stats.loop_closures));
                          add("scans_received", std::to_string(scans_received_));
                          add("scan_matches_ok", std::to_string(scan_matches_ok_));
                          add("scan_matches_fail", std::to_string(scan_matches_fail_));
                          // RTK-anchored keyframe map (use_keyframe_map): map
                          // size + scan-to-keyframe absolute-match health.
                          add("keyframes_total", std::to_string(graph_->KeyframeCount()));
                          add("kf_matches_ok", std::to_string(kf_matches_ok_));
                          add("kf_matches_fail", std::to_string(kf_matches_fail_));
                          // Robustness-pass health counters. Each is a
                          // cumulative count since process start; the
                          // session monitor diffs consecutive samples
                          // to get a rate. A spike on any of these is
                          // worth surfacing — see PR notes.
                          add("gps_rejects_wrongfix", std::to_string(stats.gps_rejects_wrongfix));
                          add("icp_rejects_rmse", std::to_string(stats.icp_rejects_rmse));
                          add("icp_rejects_inliers", std::to_string(stats.icp_rejects_inliers));
                          add("icp_rejects_sanity", std::to_string(stats.icp_rejects_sanity));
                          add("icp_rejects_divergence",
                              std::to_string(stats.icp_rejects_divergence));
                          add("stationary_hand_push", std::to_string(stats.stationary_hand_push));
                          add("slip_veto", std::to_string(stats.slip_veto));
                          add("live_nodes", std::to_string(graph_->LiveNodeCount()));
                          // Gyro bias telemetry (item #3).
                          {
                            char buf[32];
                            std::snprintf(buf, sizeof(buf), "%.5f", stats.gyro_bias_z);
                            add("gyro_bias_z_rad_per_s", buf);
                            add("gyro_bias_updates", std::to_string(stats.gyro_bias_updates));
                          }
                          // Adaptive process-noise telemetry.
                          {
                            char buf[32];
                            std::snprintf(buf, sizeof(buf), "%.5f", stats.residual_ema_rad);
                            add("residual_ema_rad", buf);
                            std::snprintf(buf, sizeof(buf), "%.4f", stats.wheel_sigma_x_eff);
                            add("wheel_sigma_x_eff", buf);
                          }
                          if (snap)
                          {
                            char buf[64];
                            std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(0, 0));
                            add("cov_xx", buf);
                            std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(1, 1));
                            add("cov_yy", buf);
                            std::snprintf(buf, sizeof(buf), "%.4f", snap->covariance(2, 2));
                            add("cov_yawyaw", buf);
                          }
                          msg.status.push_back(s);
                          pub_diag_->publish(msg);

                          // ── Pose-graph viz ────────────────────────────────────────
                          // Emits a single MarkerArray with three markers, each owning
                          // its own id so subsequent publishes overwrite cleanly:
                          //   id=0  SPHERE_LIST  — every node's optimized xy
                          //   id=1  LINE_STRIP   — trajectory through nodes by index
                          //   id=2  LINE_LIST    — accepted loop-closure edges
                          // All in map_frame_; transient-local QoS so a Foxglove client
                          // joining mid-session sees the whole graph immediately.
                          const auto poses = graph_->GetAllPoses();
                          const auto loops = graph_->GetLoopClosureEdges();
                          const rclcpp::Time stamp = this->now();

                          visualization_msgs::msg::MarkerArray ma;

                          visualization_msgs::msg::Marker nodes;
                          nodes.header.stamp = stamp;
                          nodes.header.frame_id = map_frame_;
                          nodes.ns = "fusion_graph";
                          nodes.id = 0;
                          nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
                          nodes.action = visualization_msgs::msg::Marker::ADD;
                          nodes.scale.x = nodes.scale.y = nodes.scale.z = 0.10;
                          nodes.color.r = 0.1f;
                          nodes.color.g = 0.7f;
                          nodes.color.b = 1.0f;
                          nodes.color.a = 1.0f;
                          nodes.pose.orientation.w = 1.0;

                          visualization_msgs::msg::Marker traj;
                          traj.header = nodes.header;
                          traj.ns = "fusion_graph";
                          traj.id = 1;
                          traj.type = visualization_msgs::msg::Marker::LINE_STRIP;
                          traj.action = visualization_msgs::msg::Marker::ADD;
                          traj.scale.x = 0.03;
                          traj.color.r = 0.5f;
                          traj.color.g = 0.5f;
                          traj.color.b = 0.5f;
                          traj.color.a = 0.8f;
                          traj.pose.orientation.w = 1.0;

                          // Marker bandwidth control. With 4 k+ nodes, dumping every
                          // node to the SPHERE_LIST every second produces ~50 KB of
                          // payload per tick — Foxglove + DDS choke. Cap at the most
                          // recent `viz_max_nodes` (default 1500) and stride-decimate
                          // older history if the cap is exceeded. Trajectory line still
                          // includes the same set so the topology stays connected.
                          constexpr size_t kVizMaxNodes = 1500;
                          const size_t total = poses.size();
                          const size_t stride =
                              total > kVizMaxNodes ? std::max<size_t>(1, total / kVizMaxNodes) : 1;
                          size_t i = 0;
                          for (const auto& [idx, p] : poses)
                          {
                            if (i++ % stride != 0)
                              continue;
                            geometry_msgs::msg::Point pt;
                            pt.x = p.x();
                            pt.y = p.y();
                            pt.z = 0.0;
                            nodes.points.push_back(pt);
                            traj.points.push_back(pt);
                          }
                          ma.markers.push_back(nodes);
                          ma.markers.push_back(traj);

                          visualization_msgs::msg::Marker lc;
                          lc.header = nodes.header;
                          lc.ns = "fusion_graph";
                          lc.id = 2;
                          lc.type = visualization_msgs::msg::Marker::LINE_LIST;
                          lc.action = visualization_msgs::msg::Marker::ADD;
                          lc.scale.x = 0.04;
                          lc.color.r = 1.0f;
                          lc.color.g = 0.2f;
                          lc.color.b = 0.2f;
                          lc.color.a = 0.9f;
                          lc.pose.orientation.w = 1.0;
                          for (const auto& [a, b] : loops)
                          {
                            auto ia = poses.find(a);
                            auto ib = poses.find(b);
                            if (ia == poses.end() || ib == poses.end())
                              continue;
                            geometry_msgs::msg::Point pa, pb;
                            pa.x = ia->second.x();
                            pa.y = ia->second.y();
                            pb.x = ib->second.x();
                            pb.y = ib->second.y();
                            lc.points.push_back(pa);
                            lc.points.push_back(pb);
                          }
                          ma.markers.push_back(lc);

                          pub_markers_->publish(ma);
                        });

  // Decoupled TF broadcast (see header). Started last so every member
  // the loop reads is fully constructed. Observer mode never
  // broadcasts TF, so the thread only exists in primary mode.
  if (primary_mode_ && tf_broadcast_rate_hz_ > 0.0)
  {
    tf_thread_ = std::thread(
        [this]()
        {
          TfBroadcastLoop();
        });
    RCLCPP_INFO(get_logger(),
                "fusion_graph: dedicated TF broadcast thread at %.1f Hz "
                "(lead %.3f s)",
                tf_broadcast_rate_hz_,
                tf_publish_lead_s_);
  }

  RCLCPP_INFO(get_logger(),
              "fusion_graph_node up: datum=(%.6f, %.6f), node_period=%.3fs",
              datum_lat_,
              datum_lon_,
              node_period_s);
}

}  // namespace fusion_graph
