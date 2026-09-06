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
#include <execution>
#include <utility>
#include <vector>

#include "fusion_graph/fusion_graph_node.hpp"
#include <Eigen/Eigenvalues>
#include <beluga/beluga.hpp>

namespace fusion_graph
{

namespace
{
double MonotonicSeconds()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double LargestSigma(const Eigen::Matrix2d& cov)
{
  Eigen::Matrix2d sym = 0.5 * (cov + cov.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(sym);
  if (es.info() != Eigen::Success)
    return std::numeric_limits<double>::quiet_NaN();
  return std::sqrt(std::max(0.0, es.eigenvalues().maxCoeff()));
}
}  // namespace

void FusionGraphNode::RebuildLidarAnchorMap()
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
  if (lidar_map_pub_)
    lidar_map_pub_->publish(*msg);
  if (exported.occupied == 0)
    return;  // published for inspection, but nothing to localise against yet
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
    amcl.min_particles = static_cast<std::size_t>(std::max(1, lidar_anchor_min_particles_));
    amcl.max_particles = static_cast<std::size_t>(
        std::max(lidar_anchor_min_particles_, lidar_anchor_max_particles_));
    lidar_anchor_filter_ = std::make_unique<beluga_ros::Amcl>(
        grid,
        beluga::DifferentialDriveModel2d{motion},
        beluga::LikelihoodFieldModel<beluga_ros::OccupancyGrid>{sensor, grid},
        amcl,
        std::execution::seq);
  }
  else
  {
    lidar_anchor_filter_->update_map(grid);
  }
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
  lidar_anchor_filter_->initialize(pose, cov);
  ResetLidarAnchorDeadReckoningReference(pose);
}

void FusionGraphNode::ResetLidarAnchorDeadReckoningReference(const Sophus::SE2d& pose)
{
  lidar_anchor_seed_pose_ = pose;
  lidar_anchor_seed_dr_ = Sophus::SE2d(dr_yaw_, Eigen::Vector2d(dr_x_, dr_y_));
  lidar_anchor_last_dr_ = lidar_anchor_seed_dr_;
  lidar_anchor_dr_path_m_ = 0.0;
  lidar_anchor_dr_ref_s_ = MonotonicSeconds();
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
  m.header.stamp = this->now();
  m.pose.pose.position.x = pose.translation().x();
  m.pose.pose.position.y = pose.translation().y();
  const double th = pose.so2().log();
  m.pose.pose.orientation.z = std::sin(0.5 * th);
  m.pose.pose.orientation.w = std::cos(0.5 * th);
  m.pose.covariance[0] = cov2(0, 0);
  m.pose.covariance[1] = cov2(0, 1);
  m.pose.covariance[6] = cov2(1, 0);
  m.pose.covariance[7] = cov2(1, 1);
  // Out-of-band verdict for tooling: [14] = 1 when the estimate became a
  // factor, [35] = verdict code (0 accepted, 1 score, 2 spread, 3 dead reckoning).
  m.pose.covariance[14] = applied ? 1.0 : 0.0;
  m.pose.covariance[35] = static_cast<double>(static_cast<int>(verdict));
  lidar_anchor_candidate_pub_->publish(m);
}

void FusionGraphNode::LidarMapAnchorStep(const std::vector<Eigen::Vector2d>& curr_scan,
                                         bool curr_valid)
{
  if (!lidar_mapper_ || !lidar_anchor_gate_ || !curr_valid)
    return;
  const double now_s = MonotonicSeconds();
  double rtk_age_s = 1.0e9;
  if (last_rtk_fixed_stamp_)
  {
    rtk_age_s = std::max(0.0, (this->now() - *last_rtk_fixed_stamp_).seconds());
  }
  const bool map_has_structure = lidar_map_occupied_cells_ > 0;
  const auto d = lidar_anchor_gate_->Step(rtk_age_s, map_has_structure, now_s);

  auto snapshot = graph_->LatestSnapshot();
  if (!snapshot)
    return;

  if (d.insert_scan)
  {
    std::vector<std::pair<double, double>> pts;
    pts.reserve(curr_scan.size());
    for (const auto& p : curr_scan)
      pts.emplace_back(p.x(), p.y());
    lidar_mapper_->Insert(snapshot->pose.x(), snapshot->pose.y(), snapshot->pose.theta(), pts);
    const bool due = (now_s - lidar_map_last_rebuild_s_) >= lidar_map_rebuild_period_s_;
    if (due && lidar_mapper_->inserted_scans() > lidar_map_scans_at_rebuild_)
    {
      RebuildLidarAnchorMap();
      lidar_map_last_rebuild_s_ = now_s;
      lidar_map_scans_at_rebuild_ = lidar_mapper_->inserted_scans();
    }
  }

  // Shadow mode keeps the filter running under RTK too — every estimate is
  // scored, published and counted exactly as it would be when anchoring, but
  // never becomes a factor. It is how the anchor is measured against RTK in
  // the field before it is trusted.
  const bool shadow = lidar_anchor_shadow_mode_ && d.state == LidarAnchorState::kMapping;
  if (!(d.run_filter || shadow) || !lidar_anchor_filter_)
    return;

  const Sophus::SE2d fused(snapshot->pose.theta(),
                           Eigen::Vector2d(snapshot->pose.x(), snapshot->pose.y()));
  const double fused_sx = std::sqrt(std::max(snapshot->covariance(0, 0), 0.0));
  const double fused_sy = std::sqrt(std::max(snapshot->covariance(1, 1), 0.0));

  if (d.seed_filter || (shadow && !lidar_anchor_shadow_seeded_))
  {
    SeedLidarAnchorFilter(fused, fused_sx, fused_sy);
    ++lidar_anchor_seeds_;
    lidar_anchor_shadow_seeded_ = shadow;
    lidar_anchor_lost_since_s_ = -1.0;
  }
  else if (shadow && (now_s - lidar_anchor_dr_ref_s_) >= lidar_anchor_shadow_ref_period_s_)
  {
    // Under RTK the fused pose IS the truth: refresh the dead-reckoning
    // reference from it so the plausibility budget stays tight. The filter
    // itself is left alone — that is the thing being measured.
    ResetLidarAnchorDeadReckoningReference(fused);
  }

  const Sophus::SE2d dr_now(dr_yaw_, Eigen::Vector2d(dr_x_, dr_y_));
  lidar_anchor_dr_path_m_ += (dr_now.translation() - lidar_anchor_last_dr_.translation()).norm();
  lidar_anchor_last_dr_ = dr_now;
  const Sophus::SE2d dr_pred = lidar_anchor_seed_pose_ * lidar_anchor_seed_dr_.inverse() * dr_now;

  std::vector<std::pair<double, double>> pts;
  const std::size_t stride = std::max<std::size_t>(
      1, curr_scan.size() / static_cast<std::size_t>(std::max(1, lidar_anchor_max_beams_)));
  pts.reserve(curr_scan.size() / stride + 1);
  for (std::size_t i = 0; i < curr_scan.size(); i += stride)
    pts.emplace_back(curr_scan[i].x(), curr_scan[i].y());

  const auto est = lidar_anchor_filter_->update(dr_now, std::move(pts));
  if (!est)
  {
    ++lidar_anchor_skipped_;  // below update_min_d / update_min_a: nothing new to say
    return;
  }
  const auto& [pose, cov3] = *est;
  const Eigen::Matrix2d cov2 = cov3.topLeftCorner<2, 2>();

  // Score the FULL scan at the estimate, not the decimated beams the filter
  // weighed: this is an independent witness, so it gets all the evidence.
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
  const bool apply = verdict == LidarAnchorVerdict::kAccepted && !shadow;
  PublishLidarAnchorCandidate(pose, cov2, verdict, apply);

  if (verdict != LidarAnchorVerdict::kAccepted)
  {
    // Lost. Dead reckoning is the better witness now; after a dwell, put the
    // cloud back where DR says the robot is and let it re-converge.
    if (lidar_anchor_lost_since_s_ < 0.0)
      lidar_anchor_lost_since_s_ = now_s;
    if ((now_s - lidar_anchor_lost_since_s_) >= lidar_anchor_reseed_after_s_)
    {
      const double budget = DeadReckoningBudgetM(lidar_anchor_validator_, lidar_anchor_dr_path_m_);
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
  graph_->QueueLidarMapXy(gtsam::Vector2(cand.x, cand.y), cov2, /*robust=*/true);
  ++lidar_anchor_updates_;
}

}  // namespace fusion_graph
