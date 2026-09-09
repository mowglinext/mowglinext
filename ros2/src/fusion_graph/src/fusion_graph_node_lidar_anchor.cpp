// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
//
// LiDAR map anchor: build a georeferenced occupancy grid under fresh RTK-Fixed,
// localise against it with a Beluga particle filter once Fixed goes stale, and
// feed the graph an XY-only prior — but ONLY when the estimate earns it (see
// lidar_anchor_validator.hpp for the 2026-09-06 failure that made this
// per-estimate validation load-bearing).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <execution>
#include <utility>
#include <vector>

#include "fusion_graph/fusion_graph_node.hpp"
#include "fusion_graph/lidar_covariance.hpp"
#include <Eigen/Eigenvalues>
#include <beluga/beluga.hpp>
#include <beluga_ros/amcl.hpp>
#include <beluga_ros/occupancy_grid.hpp>

namespace fusion_graph
{

namespace
{
double LargestSigma(const Eigen::Matrix2d& cov)
{
  Eigen::Matrix2d sym = 0.5 * (cov + cov.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(sym);
  if (es.info() != Eigen::Success)
    return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(std::max(0.0, es.eigenvalues().maxCoeff()));
}
}  // namespace

void FusionGraphNode::ResetLidarTiming()
{
  lidar_anchor_odom_.RebaseRaw(Sophus::SE2d(dr_yaw_, Eigen::Vector2d(dr_x_, dr_y_)));
  lidar_scan_history_.Clear();
  graph_->ClearLidarObservations();
  consumed_scan_stamp_s_ = 0.0;
  lidar_anchor_filter_.reset();
  lidar_compute_gate_.Reset();
  lidar_anchor_reference_valid_ = false;
  lidar_filter_map_dirty_ = true;
  lidar_anchor_shadow_stats_.Clear();
  lidar_anchor_floor_eff_m_ = lidar_anchor_sigma_floor_param_m_;
  lidar_anchor_lost_since_s_ = -1.0;
  lidar_anchor_undocked_s_ = this->now().seconds();
  lidar_map_last_rebuild_s_ = -1e9;
  if (lidar_anchor_gate_)
    lidar_anchor_gate_.emplace(true,
                               lidar_anchor_engage_age_s_,
                               lidar_map_insert_period_s_,
                               lidar_anchor_disengage_dwell_s_);
}

void FusionGraphNode::PollLidarSubmaps(double x, double y)
{
  const bool ready = lidar_submaps_->UpdateWindow(x, y);
  lidar_mapper_ = ready ? lidar_submaps_->mapper() : nullptr;
  if (lidar_map_import_pending_ && !lidar_submaps_->busy())
  {
    lidar_map_imported_ = lidar_submaps_->last_error().empty();
    lidar_map_import_pending_ = false;
  }
  if (lidar_mapper_ && lidar_submaps_->TakeWindowChanged())
  {
    graph_->ClearLidarObservations();
    lidar_anchor_filter_.reset();
    lidar_filter_map_dirty_ = true;
    RebuildLidarAnchorMap();
    lidar_map_last_rebuild_s_ = this->now().seconds();
    lidar_map_scans_at_rebuild_ = lidar_mapper_->inserted_scans();
  }
}

void FusionGraphNode::RebuildLidarAnchorMap(bool prepare_filter)
{
  if (!lidar_mapper_)
    return;
  if (!prepare_filter || !lidar_local_grid_)
  {
    const auto exported = lidar_mapper_->Export();
    lidar_map_occupied_cells_ = exported.occupied;
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    msg->header.frame_id = map_frame_;
    msg->header.stamp = this->now();
    msg->info.resolution = static_cast<float>(exported.resolution_m);
    msg->info.width = static_cast<uint32_t>(exported.width);
    msg->info.height = static_cast<uint32_t>(exported.height);
    msg->info.origin.position.x = exported.origin_x;
    msg->info.origin.position.y = exported.origin_y;
    msg->info.origin.orientation.w = 1.0;
    msg->data = exported.data;
    lidar_local_grid_ = msg;
    lidar_filter_map_dirty_ = true;
    if (lidar_map_pub_)
      lidar_map_pub_->publish(*msg);
    lidar_map_published_once_ = true;
  }
  if (!prepare_filter || lidar_map_occupied_cells_ == 0 ||
      (!lidar_filter_map_dirty_ && lidar_anchor_filter_))
    return;
  const auto start = std::chrono::steady_clock::now();
  const auto& msg = lidar_local_grid_;
  beluga_ros::OccupancyGrid grid(msg);
  if (!lidar_anchor_filter_)
  {
    beluga::DifferentialDriveModelParam motion;
    motion.rotation_noise_from_rotation = lidar_anchor_odom_alpha_rot_;
    motion.rotation_noise_from_translation = lidar_anchor_odom_alpha_rot_;
    motion.translation_noise_from_translation = lidar_anchor_odom_alpha_trans_;
    motion.translation_noise_from_rotation = lidar_anchor_odom_alpha_trans_;
    beluga::LikelihoodFieldModelParam sensor;
    sensor.max_obstacle_distance = 2.0;
    sensor.max_laser_distance = lidar_anchor_max_laser_distance_m_;
    sensor.z_hit = lidar_anchor_z_hit_;
    sensor.z_random = lidar_anchor_z_rand_;
    sensor.sigma_hit = lidar_anchor_sigma_hit_m_;
    beluga_ros::AmclParams amcl;
    amcl.update_min_d = lidar_anchor_update_min_d_;
    amcl.update_min_a = lidar_anchor_update_min_a_;
    amcl.alpha_slow = lidar_anchor_alpha_slow_;
    amcl.alpha_fast = lidar_anchor_alpha_fast_;
    amcl.selective_resampling = lidar_anchor_selective_resampling_;
    amcl.min_particles = static_cast<std::size_t>(std::max(1, lidar_anchor_min_particles_));
    amcl.max_particles = static_cast<std::size_t>(
        std::max(lidar_anchor_min_particles_, lidar_anchor_max_particles_));
    lidar_anchor_filter_ = std::make_unique<beluga_ros::Amcl>(
        grid,
        beluga::DifferentialDriveModel2d{motion},
        beluga::LikelihoodFieldModel<beluga_ros::OccupancyGrid>{sensor, grid},
        amcl,
        std::execution::seq);
    // Warm-up: beluga_ros::Amcl keeps a 2-pose rolling window of odometry
    // for its motion model and has no special case for the first update —
    // the "previous" pose is then the identity, so the first update applies
    // the WHOLE odom pose (metres, after minutes of driving) to every
    // particle instead of a delta. Replay 2026-09-07: a 3.2 m jump on the
    // first update after a 0.10 m seed, never after a re-seed. One discarded
    // update with the current odom pose and no measurement fills the window;
    // the seed that follows then sees a zero delta.
    lidar_anchor_filter_->update(lidar_scan_dr_, {});
  }
  else
  {
    lidar_anchor_filter_->update_map(grid);
  }
  lidar_filter_map_dirty_ = false;
  ++lidar_filter_map_builds_;
  lidar_filter_map_build_ms_ +=
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

// (Re)initialise the particle filter around `pose` and make it the reference
// for the dead-reckoning witness: from here on the DR-predicted map pose is
// pose ∘ dr_at_seed⁻¹ ∘ dr_now, and the drift budget grows with the path
// driven since this moment.
void FusionGraphNode::SeedLidarAnchorFilter(const Sophus::SE2d& pose,
                                            double sigma_x,
                                            double sigma_y)
{
  Sophus::Matrix3d cov = Sophus::Matrix3d::Zero();
  const double sx = std::max(lidar_anchor_seed_sigma_xy_m_, sigma_x);
  const double sy = std::max(lidar_anchor_seed_sigma_xy_m_, sigma_y);
  cov(0, 0) = sx * sx;
  cov(1, 1) = sy * sy;
  cov(2, 2) = lidar_anchor_seed_sigma_theta_rad_ * lidar_anchor_seed_sigma_theta_rad_;
  // Align Beluga's rolling odometry window before resetting the particles at
  // this acquisition. Otherwise the first update moves an already-current seed.
  lidar_anchor_filter_->force_update();
  lidar_anchor_filter_->update(lidar_scan_dr_, {});
  lidar_anchor_filter_->initialize(pose, cov);
  ResetLidarAnchorDeadReckoningReference(pose);
}

void FusionGraphNode::ResetLidarAnchorDeadReckoningReference(const Sophus::SE2d& pose)
{
  lidar_anchor_reference_valid_ = true;
  lidar_anchor_seed_pose_ = pose;
  lidar_anchor_seed_dr_ = lidar_scan_dr_;
  lidar_anchor_last_dr_ = lidar_anchor_seed_dr_;
  lidar_anchor_dr_path_m_ = 0.0;
  lidar_anchor_dr_ref_s_ = this->now().seconds();
}

void FusionGraphNode::PublishLidarAnchorCandidate(const Sophus::SE2d& pose,
                                                  const Eigen::Matrix2d& cov2,
                                                  LidarAnchorVerdict verdict,
                                                  bool applied)
{
  if (!lidar_anchor_candidate_pub_)
    return;
  geometry_msgs::msg::PoseWithCovarianceStamped m;
  m.header.frame_id = map_frame_;
  m.header.stamp =
      rclcpp::Time(static_cast<int64_t>(lidar_scan_stamp_s_ * 1e9), get_clock()->get_clock_type());
  m.pose.pose.position.x = pose.translation().x();
  m.pose.pose.position.y = pose.translation().y();
  const double th = pose.so2().log();
  m.pose.pose.orientation.z = std::sin(0.5 * th);
  m.pose.pose.orientation.w = std::cos(0.5 * th);
  m.pose.covariance[0] = cov2(0, 0);
  m.pose.covariance[1] = cov2(0, 1);
  m.pose.covariance[6] = cov2(1, 0);
  m.pose.covariance[7] = cov2(1, 1);
  // Out-of-band verdict for tooling: [14] = 1 when accepted for the queue
  // (expiry/cancellation can still drop it), [35] = verdict code (0 accepted, 1 score, 2 spread, 3
  // dead reckoning).
  m.pose.covariance[14] = applied ? 1.0 : 0.0;
  m.pose.covariance[35] = static_cast<double>(static_cast<int>(verdict));
  lidar_anchor_candidate_pub_->publish(m);
}

// Clear hides the live map immediately; the worker serializes disk deletion
// behind pending writes so an older save cannot resurrect the cleared tiles.
void FusionGraphNode::ClearLidarMap()
{
  if (!lidar_submaps_)
    return;
  graph_->ClearLidarObservations();
  lidar_submaps_->Clear();
  lidar_mapper_ = nullptr;
  lidar_anchor_filter_.reset();
  lidar_compute_gate_.Reset();
  lidar_anchor_reference_valid_ = false;
  lidar_map_imported_ = false;
  lidar_map_import_pending_ = false;
  lidar_anchor_shadow_stats_.Clear();
  lidar_anchor_floor_eff_m_ = lidar_anchor_sigma_floor_param_m_;
  lidar_map_occupied_cells_ = 0;
  lidar_map_scans_at_rebuild_ = 0;
  lidar_filter_map_dirty_ = true;
  auto empty = lidar_local_grid_ ? *lidar_local_grid_ : nav_msgs::msg::OccupancyGrid{};
  std::fill(empty.data.begin(), empty.data.end(), -1);
  lidar_local_grid_.reset();
  empty.header.frame_id = map_frame_;
  empty.header.stamp = this->now();
  empty.info.origin.orientation.w = 1.0;
  if (lidar_map_pub_)
    lidar_map_pub_->publish(empty);
}

void FusionGraphNode::OnLidarMapImport(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (lidar_map_imported_ || lidar_map_import_pending_ || !lidar_submaps_)
    return;
  if (!std::isfinite(msg->info.origin.orientation.x) ||
      !std::isfinite(msg->info.origin.orientation.y) ||
      !std::isfinite(msg->info.origin.orientation.z) ||
      !std::isfinite(msg->info.origin.orientation.w) || msg->header.frame_id != map_frame_ ||
      std::abs(msg->info.origin.orientation.x) > 1e-6 ||
      std::abs(msg->info.origin.orientation.y) > 1e-6 ||
      std::abs(msg->info.origin.orientation.z) > 1e-6 ||
      std::abs(std::abs(msg->info.origin.orientation.w) - 1.0) > 1e-6)
  {
    RCLCPP_WARN(get_logger(), "LiDAR map import rejected: incompatible frame or rotated origin");
    return;
  }
  ExportedOccupancyGrid imported;
  imported.resolution_m = msg->info.resolution;
  imported.origin_x = msg->info.origin.position.x;
  imported.origin_y = msg->info.origin.position.y;
  imported.width = msg->info.width;
  imported.height = msg->info.height;
  imported.data = msg->data;
  if (!lidar_submaps_->ImportLegacy(imported))
  {
    RCLCPP_WARN(get_logger(), "LiDAR map import rejected: invalid grid or tile I/O busy");
    return;
  }
  lidar_mapper_ = nullptr;
  lidar_compute_gate_.Reset();
  lidar_anchor_reference_valid_ = false;
  lidar_anchor_shadow_stats_.Clear();
  lidar_anchor_floor_eff_m_ = lidar_anchor_sigma_floor_param_m_;
  lidar_anchor_filter_.reset();
  graph_->ClearLidarObservations();
  if (lidar_anchor_gate_)
    lidar_anchor_gate_.emplace(true,
                               lidar_anchor_engage_age_s_,
                               lidar_map_insert_period_s_,
                               lidar_anchor_disengage_dwell_s_);
  lidar_map_import_pending_ = true;
  RCLCPP_INFO(get_logger(),
              "LiDAR map import dispatched (%u x %u)",
              msg->info.width,
              msg->info.height);
}

void FusionGraphNode::LidarMapAnchorStep(const std::vector<Eigen::Vector2d>& curr_scan,
                                         const LidarScanHistory::Match& scan_time)
{
  if (!lidar_mapper_ || !lidar_anchor_gate_)
    return;
  const double now_s = this->now().seconds();
  double rtk_age_s = 1.0e9;
  if (last_rtk_fixed_stamp_)
  {
    rtk_age_s = std::max(0.0, (this->now() - *last_rtk_fixed_stamp_).seconds());
  }
  const Sophus::SE2d dr_now = lidar_scan_dr_;
  const bool map_has_structure = lidar_map_occupied_cells_ > 0;
  const auto d = lidar_anchor_gate_->Step(rtk_age_s, map_has_structure, now_s);

  // Publish the grid once at startup, whatever the state: it is latched, and
  // the GUI map page draws the map instead of the raw scan points only once a
  // grid exists. Nothing is inserted while charging, so a node restarted on
  // the dock would otherwise never publish (field 2026-09-08).
  if (!lidar_map_published_once_)
  {
    lidar_map_published_once_ = true;
    RebuildLidarAnchorMap();
    lidar_map_last_rebuild_s_ = now_s;
    lidar_map_scans_at_rebuild_ = lidar_mapper_->inserted_scans();
  }

  auto snapshot = graph_->LatestSnapshot();
  const auto node_pose = graph_->GetPose(scan_time.index);
  if (!snapshot || !node_pose)
    return;
  const auto pose_at_scan = node_pose->compose(scan_time.offset);

  // On the charger the fused pose is gauge-pinned to the dock while the
  // BackUp undock actually moves the robot 1.5 m (field 2026-09-07, t+9..38 s:
  // scans inserted at a frozen pose, then a seed with the dock heading while
  // the robot had already turned — the cloud diverged to 5 m). No anchor
  // work while charging, nor for a dwell after it drops; the next seed then
  // starts from a live, re-anchored fused pose. An UNKNOWN charger state
  // (no hardware_bridge status yet) counts as docked: a node restarted on the
  // dock otherwise applied one factor in the seconds before the first status
  // (2026-09-08, factors=1 at 15 s after boot).
  const bool docked = !last_is_charging_valid_ || last_is_charging_;
  if (docked)
  {
    lidar_anchor_was_docked_ = true;
    lidar_compute_gate_.Reset();
    lidar_anchor_reference_valid_ = false;
    return;
  }
  if (lidar_anchor_was_docked_)
  {
    lidar_anchor_was_docked_ = false;
    lidar_anchor_undocked_s_ = now_s;
  }
  if ((now_s - lidar_anchor_undocked_s_) < lidar_anchor_undock_dwell_s_)
    return;

  // Evaluate against the previous map; only afterwards may this scan teach it.
  const auto evaluate = [&]()
  {
    const bool shadow = rtk_age_s <= lidar_anchor_engage_age_s_;
    const auto compute =
        lidar_compute_gate_.Step(now_s,
                                 rtk_age_s,
                                 map_has_structure,
                                 curr_scan.size() >=
                                     static_cast<std::size_t>(
                                         std::max(1, lidar_anchor_validator_.min_hit_count)),
                                 lidar_anchor_shadow_mode_,
                                 lidar_anchor_adaptive_floor_);
    if (lidar_anchor_reference_valid_)
    {
      lidar_anchor_dr_path_m_ +=
          (dr_now.translation() - lidar_anchor_last_dr_.translation()).norm();
      lidar_anchor_last_dr_ = dr_now;
    }
    if (!compute.run)
    {
      // Retain a trusted start for a future outage even while the PF is asleep.
      if (shadow)
        ResetLidarAnchorDeadReckoningReference(
            Sophus::SE2d(pose_at_scan.theta(), pose_at_scan.translation()));
      return;
    }
    const bool new_filter = !lidar_anchor_filter_;
    RebuildLidarAnchorMap(true);
    if (!lidar_anchor_filter_)
      return;

    const Sophus::SE2d fused(pose_at_scan.theta(), pose_at_scan.translation());
    // Snapshot covariance is in the node tangent frame: use its largest XY
    // sigma isotropically, never mistake local X/Y for the map axes.
    const double fused_sx = LargestSigma(snapshot->covariance.topLeftCorner<2, 2>());
    const double fused_sy = fused_sx;

    if (compute.reseed || new_filter || !lidar_anchor_reference_valid_)
    {
      if (!shadow && lidar_anchor_reference_valid_)
      {
        // A tile swap or compute pause must not erase the outage's DR budget.
        const auto reference_pose = lidar_anchor_seed_pose_;
        const auto reference_dr = lidar_anchor_seed_dr_;
        const double reference_time = lidar_anchor_dr_ref_s_;
        const double path = lidar_anchor_dr_path_m_;
        const auto predicted = reference_pose * reference_dr.inverse() * dr_now;
        const double sigma =
            std::max(fused_sx, DeadReckoningBudgetM(lidar_anchor_validator_, path));
        SeedLidarAnchorFilter(predicted, sigma, sigma);
        lidar_anchor_seed_pose_ = reference_pose;
        lidar_anchor_seed_dr_ = reference_dr;
        lidar_anchor_dr_ref_s_ = reference_time;
        lidar_anchor_dr_path_m_ = path;
      }
      else
        SeedLidarAnchorFilter(fused, fused_sx, fused_sy);
      ++lidar_anchor_seeds_;
      lidar_anchor_lost_since_s_ = -1.0;
    }
    else if (shadow && (now_s - lidar_anchor_dr_ref_s_) >= lidar_anchor_shadow_ref_period_s_)
    {
      // Under RTK the fused pose is a reference: refresh the dead-reckoning
      // reference from it so the plausibility budget stays tight. The filter
      // itself is left alone — that is the thing being measured.
      ResetLidarAnchorDeadReckoningReference(fused);
    }

    const Sophus::SE2d dr_pred = lidar_anchor_seed_pose_ * lidar_anchor_seed_dr_.inverse() * dr_now;

    std::vector<std::pair<double, double>> pts;
    const std::size_t stride = std::max<std::size_t>(
        1, curr_scan.size() / static_cast<std::size_t>(std::max(1, lidar_anchor_max_beams_)));
    pts.reserve(curr_scan.size() / stride + 1);
    for (std::size_t i = 0; i < curr_scan.size(); i += stride)
      pts.emplace_back(curr_scan[i].x(), curr_scan[i].y());

    const auto compute_start = std::chrono::steady_clock::now();
    const auto est = lidar_anchor_filter_->update(dr_now, std::move(pts));
    ++lidar_filter_calls_;
    lidar_filter_compute_ms_ +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compute_start)
            .count();
    if (!est)
    {
      ++lidar_anchor_skipped_;  // below update_min_d / update_min_a: nothing new to say
      return;
    }
    const auto& [pose, cov3] = *est;
    const Eigen::Matrix2d cov2 = cov3.topLeftCorner<2, 2>();

    // Score the full scan against the map before inserting this acquisition.
    // This is a support check, not statistically independent PF evidence.
    std::vector<std::pair<double, double>> full;
    full.reserve(curr_scan.size());
    for (const auto& p : curr_scan)
      full.emplace_back(p.x(), p.y());
    const auto score = lidar_mapper_->ScoreScan(pose.translation().x(),
                                                pose.translation().y(),
                                                pose.so2().log(),
                                                full);

    LidarAnchorCandidate cand;
    cand.x = pose.translation().x();
    cand.y = pose.translation().y();
    cand.sigma_m = LargestSigma(cov2);
    cand.hit_ratio = score.total > 0 ? static_cast<double>(score.hits) / score.total : 0.0;
    cand.hit_count = score.hits;
    cand.dr_x = dr_pred.translation().x();
    cand.dr_y = dr_pred.translation().y();
    cand.dist_since_seed_m = lidar_anchor_dr_path_m_;
    const LidarAnchorVerdict verdict = ValidateLidarAnchor(cand, lidar_anchor_validator_);

    lidar_anchor_last_hit_ratio_ = cand.hit_ratio;
    lidar_anchor_last_sigma_m_ = cand.sigma_m;
    lidar_anchor_last_verdict_ = verdict;
    switch (verdict)
    {
      case LidarAnchorVerdict::kRejectedScore:
        ++lidar_anchor_rej_score_;
        break;
      case LidarAnchorVerdict::kRejectedSpread:
        ++lidar_anchor_rej_spread_;
        break;
      case LidarAnchorVerdict::kRejectedDeadReckoning:
        ++lidar_anchor_rej_dr_;
        break;
      case LidarAnchorVerdict::kAccepted:
        break;
    }
    // Shadow under RTK-Fixed: the fused pose is a correlated reference; its
    // error calibrates the trust the anchor gets when it is applied for real.
    if (shadow && verdict == LidarAnchorVerdict::kAccepted)
    {
      lidar_anchor_shadow_stats_.Push(
          std::hypot(cand.x - fused.translation().x(), cand.y - fused.translation().y()));
    }
    lidar_anchor_floor_eff_m_ =
        lidar_anchor_adaptive_floor_
            ? lidar_anchor_shadow_stats_.EffectiveFloor(lidar_anchor_floor_quantile_,
                                                        lidar_anchor_sigma_floor_param_m_,
                                                        lidar_anchor_validator_.max_sigma_m)
            : lidar_anchor_sigma_floor_param_m_;
    const auto cov_applied = FloorLidarCovariance(cov2, lidar_anchor_floor_eff_m_);
    const bool apply =
        verdict == LidarAnchorVerdict::kAccepted && !shadow &&
        rtk_age_s >= std::max(lidar_anchor_apply_age_s_, lidar_anchor_engage_age_s_) &&
        cov_applied.has_value();
    PublishLidarAnchorCandidate(pose, cov2, verdict, apply);

    if (verdict != LidarAnchorVerdict::kAccepted)
    {
      // Lost. Dead reckoning is the better witness now; after a dwell, put the
      // cloud back where DR says the robot is and let it re-converge.
      if (lidar_anchor_lost_since_s_ < 0.0)
        lidar_anchor_lost_since_s_ = now_s;
      if ((now_s - lidar_anchor_lost_since_s_) >= lidar_anchor_reseed_after_s_)
      {
        const double budget =
            DeadReckoningBudgetM(lidar_anchor_validator_, lidar_anchor_dr_path_m_);
        // Keep the DR reference (and its grown budget); only the cloud moves.
        const Sophus::SE2d seed_pose = lidar_anchor_seed_pose_;
        const Sophus::SE2d seed_dr = lidar_anchor_seed_dr_;
        const double path = lidar_anchor_dr_path_m_;
        const double ref_s = lidar_anchor_dr_ref_s_;
        SeedLidarAnchorFilter(dr_pred, budget, budget);
        lidar_anchor_seed_pose_ = seed_pose;
        lidar_anchor_seed_dr_ = seed_dr;
        lidar_anchor_last_dr_ = dr_now;
        lidar_anchor_dr_path_m_ = path;
        lidar_anchor_dr_ref_s_ = ref_s;
        ++lidar_anchor_reseeds_;
        lidar_anchor_lost_since_s_ = -1.0;
      }
      return;
    }
    lidar_anchor_lost_since_s_ = -1.0;
    if (!apply)
      return;
    // Trust no better than the anchor's measured accuracy: inflate to the
    // effective floor (the graph consumer still applies the parameter floor).
    graph_->QueueLidarMapXy(gtsam::Vector2(cand.x, cand.y),
                            *cov_applied,
                            true,
                            scan_time.index,
                            scan_time.offset.translation(),
                            lidar_scan_stamp_s_ + lidar_scan_max_age_s_);
    ++lidar_anchor_updates_;
  };
  evaluate();
  if (d.insert_scan)
  {
    std::vector<std::pair<double, double>> pts;
    pts.reserve(curr_scan.size());
    for (const auto& p : curr_scan)
      pts.emplace_back(p.x(), p.y());
    lidar_mapper_->Insert(pose_at_scan.x(), pose_at_scan.y(), pose_at_scan.theta(), pts);
    const bool due = (now_s - lidar_map_last_rebuild_s_) >= lidar_map_rebuild_period_s_;
    if (due && lidar_mapper_->inserted_scans() > lidar_map_scans_at_rebuild_)
    {
      RebuildLidarAnchorMap();
      lidar_map_last_rebuild_s_ = now_s;
      lidar_map_scans_at_rebuild_ = lidar_mapper_->inserted_scans();
    }
  }
}

}  // namespace fusion_graph
