// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/fusion_graph_node.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fusion_graph
{

namespace
{

constexpr double kEarthRadius = 6378137.0;  // WGS84 equatorial, metres.

// Extract yaw from a geometry_msgs Quaternion.
double YawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double r, p, y;
  tf2::Matrix3x3(tq).getRPY(r, p, y);
  return y;
}

geometry_msgs::msg::Quaternion QuatFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x();
  m.y = q.y();
  m.z = q.z();
  m.w = q.w();
  return m;
}

}  // namespace

FusionGraphNode::FusionGraphNode(const rclcpp::NodeOptions& opts)
    : rclcpp::Node("fusion_graph_node", opts)
{
  // ── Parameters ────────────────────────────────────────────────────
  GraphParams gp;
  gp.node_period_s = declare_parameter<double>("node_period_s", 0.1);
  gp.wheel_sigma_x = declare_parameter<double>("wheel_sigma_x", 0.05);
  gp.wheel_sigma_y = declare_parameter<double>("wheel_sigma_y", 0.005);
  gp.wheel_sigma_theta = declare_parameter<double>("wheel_sigma_theta", 0.01);
  gp.gyro_sigma_theta = declare_parameter<double>("gyro_sigma_theta", 0.005);
  gp.gps_sigma_floor = declare_parameter<double>("gps_sigma_floor", 0.003);
  gp.prior_sigma_xy = declare_parameter<double>("prior_sigma_xy", 0.05);
  gp.prior_sigma_theta = declare_parameter<double>("prior_sigma_theta", 0.05);
  gp.lever_arm_x = declare_parameter<double>("lever_arm_x", 0.0);
  gp.lever_arm_y = declare_parameter<double>("lever_arm_y", 0.0);
  gp.cov_update_every_n = declare_parameter<int>("cov_update_every_n", 10);
  gp.isam2_relinearize_skip = declare_parameter<int>("isam2_relinearize_skip", 5);
  gp.stationary_motion_thresh_m = declare_parameter<double>("stationary_motion_thresh_m", 0.02);
  gp.stationary_motion_thresh_theta =
      declare_parameter<double>("stationary_motion_thresh_theta", 0.01);
  gp.stationary_node_period_s = declare_parameter<double>("stationary_node_period_s", 5.0);
  gp.stationary_thresh_xy_m =
      declare_parameter<double>("stationary_thresh_xy_m", 1.0e-3);
  gp.stationary_thresh_theta =
      declare_parameter<double>("stationary_thresh_theta", 2.0e-3);
  gp.stationary_sigma_theta =
      declare_parameter<double>("stationary_sigma_theta", 1.0e-3);

  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
  datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

  graph_ = std::make_unique<GraphManager>(gp);

  // ── Scan matching (optional) ─────────────────────────────────────
  use_scan_matching_ = declare_parameter<bool>("use_scan_matching", false);
  if (use_scan_matching_)
  {
    ScanMatcherParams sp;
    // 10 iters converges within 1 mm of the 15-iter solution on the
    // outdoor LiDAR shapes we see; the extra 5 iters were CPU sink.
    sp.max_iterations = declare_parameter<int>("icp_max_iter", 10);
    sp.max_correspondence_dist = declare_parameter<double>("icp_max_corresp_dist", 0.5);
    // 40 source points keeps ICP rmse within a few mm of the 60-pt
    // result while halving inner-loop NN cost. ARM hot-path saving.
    sp.source_subsample = static_cast<size_t>(declare_parameter<int>("icp_source_subsample", 40));
    sp.sigma_xy_base = declare_parameter<double>("icp_sigma_xy_base", 0.02);
    sp.sigma_theta_base = declare_parameter<double>("icp_sigma_theta_base", 0.005);
    scan_matcher_ = std::make_unique<ScanMatcher>(sp);
  }

  // ── Magnetometer (off by default) ───────────────────────────────
  // Motors near the chassis induce a heading-dependent bias on the
  // magnetometer that no static cal can remove (see CLAUDE.md
  // history). Default off so the graph never sees mag samples;
  // operators with a motor-isolated mag hardware setup can flip the
  // flag on at launch.
  use_magnetometer_ = declare_parameter<bool>("use_magnetometer", false);

  // Primary vs observer. Defaults to true for back-compat with the
  // standalone test harness; navigation.launch.py overrides to false
  // when no persisted graph exists yet (first session) so ekf_map
  // keeps driving Nav2 while fusion_graph builds the graph silently.
  primary_mode_ = declare_parameter<bool>("primary_mode", true);

  // ── Loop closure + persistence ───────────────────────────────────
  loop_closure_enabled_ = declare_parameter<bool>("use_loop_closure", false);
  lc_max_dist_m_ = declare_parameter<double>("lc_max_dist_m", 5.0);
  // Default 600s (10 min): stationary clutter at the dock or during
  // long IDLE windows produces O(N²) LC factors with the lower 30/120s
  // defaults. Real revisits across a mowing pattern are minutes apart,
  // so 600s is a comfortable floor. Override per-test if needed.
  lc_min_age_s_ = declare_parameter<double>("lc_min_age_s", 600.0);
  lc_max_candidates_ = static_cast<size_t>(declare_parameter<int>("lc_max_candidates", 3));
  lc_min_delta_m_ = declare_parameter<double>("lc_min_delta_m", 0.05);
  lc_min_delta_theta_ = declare_parameter<double>("lc_min_delta_theta", 0.05);
  // 0.10 m rmse rejected too aggressively in field tests: outdoor
  // LiDAR scans of the same place separated by minutes typically
  // see ~15-25 cm point-wise rmse from wind / shadow / dynamic
  // obstacles, even when the relative pose delta is sub-cm. The
  // BetweenFactor noise scales with rmse anyway (sigma_xy_base +
  // sigma_xy_scale * rmse), so a noisier match enters the graph
  // with proportionally lower weight rather than being dropped.
  lc_max_rmse_ = declare_parameter<double>("lc_max_rmse", 0.20);
  lc_sigma_xy_ = declare_parameter<double>("lc_sigma_xy", 0.05);
  lc_sigma_theta_ = declare_parameter<double>("lc_sigma_theta", 0.02);

  graph_save_prefix_ =
      declare_parameter<std::string>("graph_save_prefix", "/ros2_ws/maps/fusion_graph");

  scan_retention_nodes_ =
      static_cast<uint64_t>(declare_parameter<int>("scan_retention_nodes", 18000));
  isam2_rebase_every_nodes_ =
      static_cast<uint64_t>(declare_parameter<int>("isam2_rebase_every_nodes", 2000));
  const bool autoload = declare_parameter<bool>("autoload_graph", true);

  // Dock yaw read from mowgli_robot.yaml via the launch wrapper. Used
  // as a yaw seed at cold boot when GPS+COG aren't available yet but
  // a persisted graph exists — the robot is on the dock so this is
  // a tight prior.
  dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

  if (autoload)
  {
    if (graph_->Load(graph_save_prefix_))
    {
      autoload_succeeded_ = true;
      RCLCPP_INFO(get_logger(),
                  "fusion_graph: loaded persisted graph from '%s.*'",
                  graph_save_prefix_.c_str());
    }
  }

  // ── Auto-checkpoint configuration ───────────────────────────────
  // Persist the graph automatically on:
  //   - transition out of HIGH_LEVEL_STATE_RECORDING (the area
  //     polygon was just saved by the GUI; we want the matching pose
  //     graph + scans to land alongside it)
  //   - rising edge of is_charging (robot just docked; safe checkpoint
  //     before any potential power loss)
  //   - periodic timer during AUTONOMOUS state (default 5 min)
  // Set auto_save_enabled to false to keep checkpoints fully manual
  // via the ~/save_graph service.
  auto_save_enabled_ = declare_parameter<bool>("auto_save_enabled", true);
  const double periodic_save_period_s = declare_parameter<double>("periodic_save_period_s", 300.0);

  // ── TF ────────────────────────────────────────────────────────────
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Pubs/subs ─────────────────────────────────────────────────────
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered_map", 10);
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
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos, std::bind(&FusionGraphNode::OnScan, this, std::placeholders::_1));
  }

  if (auto_save_enabled_)
  {
    sub_hl_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
        "/behavior_tree_node/high_level_status",
        10,
        std::bind(&FusionGraphNode::OnHighLevelStatus, this, std::placeholders::_1));
    sub_hw_status_ = create_subscription<mowgli_interfaces::msg::Status>(
        "/hardware_bridge/status",
        10,
        std::bind(&FusionGraphNode::OnHardwareStatus, this, std::placeholders::_1));
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
  // given pose with covariance-derived sigmas. dock_yaw_to_set_pose
  // dual-publishes to /ekf_map_node/set_pose AND to this topic so
  // the dock-yaw seeding works regardless of which localizer is the
  // map-frame primary.
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
        // Drop the latched seed too, otherwise a stale GPS / yaw seed
        // from before the clear would re-initialize the graph at the
        // old position the operator was trying to escape.
        seed_xy_.reset();
        seed_yaw_.reset();
        seed_xy_rtk_fixed_ = false;
        resp->success = true;
        resp->message = "graph cleared (waiting for re-initialization)";
        RCLCPP_WARN(get_logger(), "fusion_graph: %s", resp->message.c_str());
      });

  // ── Tick timer ────────────────────────────────────────────────────
  // Run at 1× node rate. Earlier 2× oversampling existed "to never
  // miss a node window" but doubled per-Tick CPU (ICP runs every
  // OnTimer call) for no functional gain — Tick() short-circuits when
  // dt < node_period_s, so a late wall_timer just creates the next
  // node a few ms late, with no graph-level effect.
  const double timer_period_s = gp.node_period_s;
  tick_timer_ = create_wall_timer(std::chrono::duration<double>(timer_period_s),
                                  std::bind(&FusionGraphNode::OnTimer, this));

  // Maintenance timer at 30 s: prune old scans + check if iSAM2
  // needs to be rebased. Both operations briefly hold the graph
  // mutex; running off the main tick keeps Tick latency clean.
  maintenance_timer_ =
      create_wall_timer(std::chrono::seconds(30),
                        [this]()
                        {
                          if (!graph_->IsInitialized())
                            return;
                          graph_->PruneOldScans(scan_retention_nodes_);
                          const auto stats = graph_->Stats();
                          if (stats.total_nodes - last_rebase_index_ >= isam2_rebase_every_nodes_)
                          {
                            graph_->RebaseISAM2();
                            last_rebase_index_ = stats.total_nodes;
                            RCLCPP_INFO(get_logger(),
                                        "fusion_graph: iSAM2 rebased at node %lu",
                                        static_cast<unsigned long>(stats.total_nodes));
                          }
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

  RCLCPP_INFO(get_logger(),
              "fusion_graph_node up: datum=(%.6f, %.6f), node_period=%.3fs",
              datum_lat_,
              datum_lon_,
              gp.node_period_s);
}

// ── Callbacks ─────────────────────────────────────────────────────────

void FusionGraphNode::OnWheelOdom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (last_wheel_stamp_)
  {
    double dt = (stamp - *last_wheel_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0)
    {
      graph_->AddWheelTwist(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.angular.z,
                            dt);
    }
  }
  last_wheel_stamp_ = stamp;
}

void FusionGraphNode::OnImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (last_imu_stamp_)
  {
    double dt = (stamp - *last_imu_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0)
    {
      graph_->AddGyroDelta(msg->angular_velocity.z, dt);
    }
  }
  last_imu_stamp_ = stamp;
}

void FusionGraphNode::OnGnss(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    return;
  if (datum_lat_ == 0.0 && datum_lon_ == 0.0)
  {
    // Self-seed datum from first valid fix. Not ideal — operator should
    // set datum in mowgli_robot.yaml — but keeps the node from refusing
    // to start during sim/dev.
    datum_lat_ = msg->latitude;
    datum_lon_ = msg->longitude;
    datum_cos_lat_ = std::cos(datum_lat_ * M_PI / 180.0);
    RCLCPP_WARN(get_logger(),
                "fusion_graph: datum self-seeded from first fix "
                "(%.6f, %.6f) — set datum_lat/lon explicitly",
                datum_lat_,
                datum_lon_);
  }

  double mx, my;
  LatLonToMap(msg->latitude, msg->longitude, mx, my);

  // covariance[0] is variance of east; take sqrt for sigma. Use the
  // diagonal mean for a single sigma_xy (factor model is isotropic).
  const double var_x = msg->position_covariance[0];
  const double var_y = msg->position_covariance[4];
  double sigma = std::sqrt(0.5 * (var_x + var_y));
  if (!std::isfinite(sigma) || sigma <= 0.0)
    sigma = -1.0;  // floor

  // RTK-Fixed (GBAS_FIX = 2) is sub-cm and statistically Gaussian; any
  // lower fix quality (Float / single / DGPS) routinely carries
  // multi-decimetre multipath outliers that the reported covariance
  // doesn't capture. Robustify with Huber in that case so the optimizer
  // downweights aberrant samples instead of pulling the trajectory.
  const bool rtk_fixed = msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  const bool robust = !rtk_fixed;
  graph_->QueueGnss(mx, my, sigma, robust);
  seed_xy_ = gtsam::Vector2(mx, my);
  // Latch whether the most recent seed came from RTK-Fixed so the next
  // graph initialization can use a tight prior matching that quality.
  // Stale once seeded but TrySeedInitialPose only fires once per
  // (re)initialization, so the freshness window is the same as the
  // seed itself.
  seed_xy_rtk_fixed_ = rtk_fixed;

  TrySeedInitialPose();
}

void FusionGraphNode::OnCogHeading(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const double yaw = YawFromQuat(msg->orientation);
  // covariance[8] is yaw variance.
  double var = msg->orientation_covariance[8];
  if (!std::isfinite(var) || var <= 0.0)
    var = 0.05 * 0.05;
  const double sigma = std::sqrt(var);
  graph_->QueueYaw(yaw, sigma);
  seed_yaw_ = yaw;
  TrySeedInitialPose();
}

void FusionGraphNode::OnMagYaw(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const double yaw = YawFromQuat(msg->orientation);
  double var = msg->orientation_covariance[8];
  if (!std::isfinite(var) || var <= 0.0)
    var = 0.1 * 0.1;
  // Mag yaw carries heading-dependent calibration bias (~5-15° peaks)
  // even after tilt compensation. Always robustify so when COG is also
  // active the optimizer pulls toward COG and treats mag as a soft
  // anchor that prevents free drift, not as a precise observation.
  graph_->QueueYaw(yaw, std::sqrt(var), /*robust=*/true);
  if (!seed_yaw_)
    seed_yaw_ = yaw;
  TrySeedInitialPose();
}

void FusionGraphNode::OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  // Resolve scan_frame -> base_footprint at the scan timestamp; if TF
  // isn't ready yet, drop this scan rather than warp it with stale
  // extrinsics.
  geometry_msgs::msg::TransformStamped t_base_scan;
  try
  {
    t_base_scan = tf_buffer_->lookupTransform(base_frame_,
                                              msg->header.frame_id,
                                              msg->header.stamp,
                                              tf2::durationFromSec(0.05));
  }
  catch (const tf2::TransformException&)
  {
    return;
  }

  tf2::Transform T_base_scan;
  tf2::fromMsg(t_base_scan.transform, T_base_scan);

  std::vector<Eigen::Vector2d> pts;
  pts.reserve(msg->ranges.size());
  const double a0 = msg->angle_min;
  const double da = msg->angle_increment;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    const float r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
      continue;
    const double a = a0 + da * static_cast<double>(i);
    tf2::Vector3 p_scan(r * std::cos(a), r * std::sin(a), 0.0);
    tf2::Vector3 p_base = T_base_scan * p_scan;
    pts.emplace_back(p_base.x(), p_base.y());
  }

  std::lock_guard<std::mutex> lock(scan_mu_);
  latest_scan_ = std::move(pts);
  latest_scan_valid_ = !latest_scan_.empty();
  ++scans_received_;

  // Cold-boot relocalization: if we autoloaded a graph but never had
  // a fresh GPS+COG to validate the live pose, ICP-match this first
  // scan against scans of nodes near the dock and force-anchor the
  // last loaded node at the matched pose. This unsticks the case
  // "GPS dead since boot, robot was placed back on dock manually
  // between sessions".
  if (autoload_succeeded_ && !relocalize_done_ && scan_matcher_ && latest_scan_valid_ &&
      graph_->IsInitialized())
  {
    const auto candidates =
        graph_->FindNodesNearXY(0.0, 0.0, 5.0, 5);  // dock is map origin (datum)
    double best_rmse = std::numeric_limits<double>::infinity();
    gtsam::Pose2 best_pose;
    uint64_t best_idx = 0;
    for (uint64_t idx : candidates)
    {
      auto cand_scan = graph_->GetScan(idx);
      auto cand_pose = graph_->GetPose(idx);
      if (cand_scan.empty() || !cand_pose)
        continue;
      // Use the candidate's pose as init (we expect to be close).
      auto res = scan_matcher_->Match(cand_scan, latest_scan_, *cand_pose);
      if (res.ok && res.rmse < best_rmse)
      {
        best_rmse = res.rmse;
        best_pose = res.delta;
        best_idx = idx;
      }
    }
    if (std::isfinite(best_rmse) && best_rmse < 0.10)
    {
      // Anchor the latest loaded node at the matched pose so future
      // wheel/scan factors compose from a consistent reference.
      auto snap = graph_->LatestSnapshot();
      if (snap)
      {
        graph_->ForceAnchor(snap->node_index, best_pose, 0.05, 0.05);
        relocalize_done_ = true;
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: relocalized via scan match "
                    "node=%lu rmse=%.3f → (%.2f, %.2f, %.2f rad)",
                    static_cast<unsigned long>(best_idx),
                    best_rmse,
                    best_pose.x(),
                    best_pose.y(),
                    best_pose.theta());
      }
    }
  }
}

void FusionGraphNode::OnHighLevelStatus(mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
{
  // Rising-edge detection on RECORDING → other transition.
  if (last_hl_state_valid_)
  {
    constexpr uint8_t kRecording =
        mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_RECORDING;
    if (last_hl_state_ == kRecording && msg->state != kRecording &&
        graph_->IsInitialized())
    {
      const bool ok = graph_->Save(graph_save_prefix_);
      RCLCPP_INFO(get_logger(),
                  "fusion_graph: auto-save on RECORDING exit → %s",
                  ok ? "ok" : "failed");
    }
  }
  last_hl_state_ = msg->state;
  last_hl_state_valid_ = true;
}

void FusionGraphNode::OnSetPose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  // Extract Pose2 from the incoming PoseWithCovariance.
  const auto& q = msg->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const gtsam::Pose2 pose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);

  // Pull σ_xy and σ_yaw from the 6×6 covariance (positions [0]/[7],
  // yaw at [35]). Floor at sane minimums so a zero-cov caller doesn't
  // create a singular constraint.
  const auto& cov = msg->pose.covariance;
  const double sigma_xy = std::sqrt(std::max(cov[0], 1e-4));
  const double sigma_theta = std::sqrt(std::max(cov[35], 1e-4));

  // If we're not yet initialized (no graph loaded, no GPS+COG seed),
  // use the message as the bootstrap seed: build X_0 here.
  if (!graph_->IsInitialized())
  {
    graph_->Initialize(pose, this->now().seconds());
    RCLCPP_INFO(get_logger(),
                "fusion_graph: bootstrap init from /set_pose at "
                "(%.2f, %.2f, %.2f rad)",
                pose.x(),
                pose.y(),
                pose.theta());
    return;
  }

  // Otherwise, force-anchor the latest node at the provided pose.
  auto snap = graph_->LatestSnapshot();
  if (!snap)
    return;
  graph_->ForceAnchor(snap->node_index, pose, sigma_xy, sigma_theta);
  RCLCPP_INFO(get_logger(),
              "fusion_graph: re-anchored node %lu via /set_pose to "
              "(%.2f, %.2f, %.2f rad)",
              static_cast<unsigned long>(snap->node_index),
              pose.x(),
              pose.y(),
              pose.theta());
}

void FusionGraphNode::OnHardwareStatus(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  // Rising edge of is_charging = robot just docked.
  if (last_is_charging_valid_ && !last_is_charging_ && msg->is_charging &&
      graph_->IsInitialized())
  {
    const bool ok = graph_->Save(graph_save_prefix_);
    RCLCPP_INFO(get_logger(), "fusion_graph: auto-save on dock arrival → %s", ok ? "ok" : "failed");
  }
  last_is_charging_ = msg->is_charging;
  last_is_charging_valid_ = true;
}

void FusionGraphNode::OnPeriodicSaveTimer()
{
  // Only checkpoint while autonomously mowing — saving on the dock
  // is already handled by OnHardwareStatus, and saving while idle
  // wastes I/O on a graph that's not changing.
  constexpr uint8_t kAutonomous =
      mowgli_interfaces::msg::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
  if (!last_hl_state_valid_ || last_hl_state_ != kAutonomous)
    return;
  if (!graph_->IsInitialized())
    return;
  const bool ok = graph_->Save(graph_save_prefix_);
  RCLCPP_INFO(get_logger(), "fusion_graph: periodic auto-save → %s", ok ? "ok" : "failed");
}

void FusionGraphNode::OnTimer()
{
  const double now_s = this->now().seconds();

  // Run scan-matching against the previous-node scan and queue the
  // resulting between-factor before Tick — Tick consumes the queue
  // when it creates a new node.
  std::vector<Eigen::Vector2d> curr_scan;
  bool curr_valid = false;
  {
    std::lock_guard<std::mutex> lock(scan_mu_);
    if (latest_scan_valid_)
    {
      curr_scan = latest_scan_;
      curr_valid = true;
    }
  }

  if (use_scan_matching_ && scan_matcher_ && curr_valid && prev_node_scan_valid_)
  {
    auto res = scan_matcher_->Match(prev_node_scan_, curr_scan, gtsam::Pose2());
    if (res.ok)
    {
      graph_->QueueScanBetween(res.delta, res.sigma_xy, res.sigma_theta);
      ++scan_matches_ok_;
    }
    else
    {
      ++scan_matches_fail_;
    }
  }

  auto out = graph_->Tick(now_s);
  if (out)
  {
    // Attach the current scan to the new node (used for loop closures
    // + persistence). Use the still-valid current_scan we captured
    // above; reusing it as prev_node_scan is OK since std::move only
    // happens after this block.
    if (curr_valid)
    {
      graph_->AttachScan(out->node_index, curr_scan);
    }

    // Loop closure search — gated on loop_closure_enabled_ and on
    // having a scan matcher (we reuse it). Find candidates within
    // lc_max_dist_m_ that are at least lc_min_age_s_ old, run ICP for
    // each, accept those with rmse < lc_max_rmse_.
    if (loop_closure_enabled_ && scan_matcher_ && curr_valid)
    {
      auto candidates = graph_->FindLoopClosureCandidates(out->node_index,
                                                          lc_max_dist_m_,
                                                          lc_min_age_s_,
                                                          lc_max_candidates_);
      for (uint64_t cand_idx : candidates)
      {
        auto cand_scan = graph_->GetScan(cand_idx);
        auto cand_pose = graph_->GetPose(cand_idx);
        if (cand_scan.empty() || !cand_pose)
          continue;

        // Init guess: transform from cand to current in map frame,
        // i.e. cand.between(curr).
        const gtsam::Pose2 init = cand_pose->between(out->pose);
        auto res = scan_matcher_->Match(cand_scan, curr_scan, init);
        if (!res.ok || res.rmse > lc_max_rmse_)
          continue;

        // Skip near-identity LC factors: they pin the same pose to
        // itself and carry no constraint info, but each one costs an
        // iSAM2 update. Common at dock IDLE / before-undock revisits.
        const double dt2 = res.delta.x() * res.delta.x() + res.delta.y() * res.delta.y();
        if (dt2 < lc_min_delta_m_ * lc_min_delta_m_ &&
            std::abs(res.delta.theta()) < lc_min_delta_theta_)
        {
          continue;
        }

        graph_->AddLoopClosure(cand_idx, out->node_index, res.delta, lc_sigma_xy_, lc_sigma_theta_);
        RCLCPP_INFO(get_logger(),
                    "fusion_graph: loop closure %lu -> %lu accepted "
                    "(rmse=%.3f, dist=%.2f m)",
                    static_cast<unsigned long>(cand_idx),
                    static_cast<unsigned long>(out->node_index),
                    res.rmse,
                    std::hypot(out->pose.x() - cand_pose->x(), out->pose.y() - cand_pose->y()));
      }
    }

    if (curr_valid)
    {
      prev_node_scan_ = std::move(curr_scan);
      prev_node_scan_valid_ = true;
    }
  }

  // Always republish odom + TF from the latest snapshot, even when
  // Tick returned nullopt (stationary throttle, init still pending,
  // etc.). Nav2 / BT need a fresh map→odom TF at the OnTimer rate
  // (10 Hz) — gating publish on node creation drops the rate to 0.2 Hz
  // during stationary windows, which makes the planner think the
  // pose is stale and skips goals. Cheap: just composes latest_ with
  // odom→base_footprint and broadcasts.
  if (auto snap = graph_->LatestSnapshot())
  {
    PublishOutputs(*snap);
  }
}

// ── Helpers ───────────────────────────────────────────────────────────

void FusionGraphNode::LatLonToMap(double lat, double lon, double& x, double& y) const
{
  const double dlat = (lat - datum_lat_) * M_PI / 180.0;
  const double dlon = (lon - datum_lon_) * M_PI / 180.0;
  x = kEarthRadius * datum_cos_lat_ * dlon;  // east
  y = kEarthRadius * dlat;  // north
}

bool FusionGraphNode::TrySeedInitialPose()
{
  if (graph_->IsInitialized())
    return true;
  if (!seed_xy_ || !seed_yaw_)
    return false;
  // When the seed came from an RTK-Fixed fix, override the prior to
  // match the measurement quality. 5 mm is conservative w.r.t the
  // F9P's typical RTK-Fixed σ ~3 mm; tight enough that the wheel
  // between-factors can't pull the first few nodes off the GPS
  // anchor, but loose enough to absorb a few mm of antenna lever-arm
  // residual.
  std::optional<double> prior_override;
  if (seed_xy_rtk_fixed_)
    prior_override = 0.005;
  graph_->Initialize(gtsam::Pose2(seed_xy_->x(), seed_xy_->y(), *seed_yaw_),
                     this->now().seconds(),
                     prior_override);
  RCLCPP_INFO(get_logger(),
              "fusion_graph: initialized at (%.3f, %.3f, %.3f rad)%s",
              seed_xy_->x(),
              seed_xy_->y(),
              *seed_yaw_,
              seed_xy_rtk_fixed_ ? " [RTK-Fixed seed, σ=5mm prior]" : " [non-Fixed seed]");
  return true;
}

void FusionGraphNode::PublishOutputs(const TickOutput& out)
{
  // 1. nav_msgs/Odometry on /odometry/filtered_map.
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = out.pose.x();
  odom.pose.pose.position.y = out.pose.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = QuatFromYaw(out.pose.theta());

  // Pose covariance is 6x6 row-major: x, y, z, roll, pitch, yaw.
  for (auto& v : odom.pose.covariance)
    v = 0.0;
  odom.pose.covariance[0] = out.covariance(0, 0);
  odom.pose.covariance[1] = out.covariance(0, 1);
  odom.pose.covariance[5] = out.covariance(0, 2);
  odom.pose.covariance[6] = out.covariance(1, 0);
  odom.pose.covariance[7] = out.covariance(1, 1);
  odom.pose.covariance[11] = out.covariance(1, 2);
  odom.pose.covariance[30] = out.covariance(2, 0);
  odom.pose.covariance[31] = out.covariance(2, 1);
  odom.pose.covariance[35] = out.covariance(2, 2);
  // Z, roll, pitch — clamped, give them tiny variance so consumers
  // don't choke on zero.
  odom.pose.covariance[14] = 1e-9;
  odom.pose.covariance[21] = 1e-9;
  odom.pose.covariance[28] = 1e-9;
  pub_odom_->publish(odom);

  // 1b. /imu/fg_yaw — yaw-only sensor_msgs/Imu (cov_yaw, others 1e6
  //     to disable). Published in both primary and observer mode so
  //     ekf_map_node can fuse it as imu2 absolute yaw without
  //     creating a feedback loop (fusion_graph never reads
  //     /odometry/filtered_map).
  sensor_msgs::msg::Imu yaw_msg;
  yaw_msg.header.stamp = odom.header.stamp;
  yaw_msg.header.frame_id = base_frame_;
  yaw_msg.orientation = QuatFromYaw(out.pose.theta());
  // Roll/pitch covariances marked huge so robot_localization ignores
  // them; only orientation_covariance[8] (yaw variance) is real.
  for (auto& v : yaw_msg.orientation_covariance)
    v = 0.0;
  yaw_msg.orientation_covariance[0] = 1e6;
  yaw_msg.orientation_covariance[4] = 1e6;
  yaw_msg.orientation_covariance[8] = std::max(out.covariance(2, 2), 1e-6);
  // We don't fuse angular velocity / linear acceleration here; mark
  // them all as -1 covariance to tell consumers the field is invalid.
  yaw_msg.angular_velocity_covariance[0] = -1.0;
  yaw_msg.linear_acceleration_covariance[0] = -1.0;
  pub_fg_yaw_->publish(yaw_msg);

  // 2. TF map -> odom. Skipped in observer mode so the active
  //    map-frame primary (typically ekf_map_node) keeps single
  //    ownership of the map→odom transform. The /odometry/filtered_map
  //    publish above still happens — downstream consumers that
  //    explicitly route to /fusion_graph/odometry (via launch remap)
  //    can use it for diagnostics or A/B comparison.
  if (!primary_mode_)
    return;

  //    T_odom_base is the local EKF's odom->base_footprint TF.
  geometry_msgs::msg::TransformStamped t_odom_base;
  try
  {
    t_odom_base = tf_buffer_->lookupTransform(odom_frame_,
                                              base_frame_,
                                              tf2::TimePointZero,
                                              tf2::durationFromSec(0.05));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         2000,
                         "fusion_graph: TF %s -> %s not available: %s",
                         odom_frame_.c_str(),
                         base_frame_.c_str(),
                         ex.what());
    return;
  }

  tf2::Transform T_odom_base;
  tf2::fromMsg(t_odom_base.transform, T_odom_base);

  tf2::Transform T_map_base;
  T_map_base.setOrigin(tf2::Vector3(out.pose.x(), out.pose.y(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, out.pose.theta());
  T_map_base.setRotation(q);

  const tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

  geometry_msgs::msg::TransformStamped t_map_odom;
  t_map_odom.header.stamp = this->now();
  t_map_odom.header.frame_id = map_frame_;
  t_map_odom.child_frame_id = odom_frame_;
  t_map_odom.transform = tf2::toMsg(T_map_odom);
  tf_broadcaster_->sendTransform(t_map_odom);
}

}  // namespace fusion_graph

// ── Entry point ──────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fusion_graph::FusionGraphNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
