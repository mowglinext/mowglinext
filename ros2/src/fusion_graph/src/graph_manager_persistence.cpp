// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — persistence: Save/Load/Reset. (The class
// implementation is split across several translation units to keep each file within the project's
// 600-line budget; all share graph_manager.hpp + the inline PoseKey().)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>

#include "fusion_graph/checkpoint_file.hpp"
#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <boost/serialization/export.hpp>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/serialization.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>
// cppcheck-suppress unknownMacro
BOOST_CLASS_EXPORT_GUID(gtsam::GenericValue<gtsam::Pose2>, "gtsam_GenericValue_Pose2")

namespace fusion_graph
{
namespace
{
bool DecodeSnapshot(const checkpoint::Payload& payload,
                    gtsam::Values& values,
                    uint64_t& next_index,
                    double& time,
                    double& latitude,
                    double& longitude)
{
  try
  {
    values.clear();
    gtsam::deserializeXML(payload.graph, values);
    next_index = 0;
    time = latitude = longitude = 0.0;
    std::istringstream metadata(payload.metadata);
    std::string line;
    const auto number = [](const std::string& val)
    {
      size_t consumed = 0;
      const double result = std::stod(val, &consumed);
      if (consumed != val.size() || !std::isfinite(result))
        throw std::invalid_argument("invalid checkpoint number");
      return result;
    };
    while (std::getline(metadata, line))
    {
      const auto eq = line.find('=');
      if (eq == std::string::npos)
        continue;
      const auto key = line.substr(0, eq);
      const auto val = line.substr(eq + 1);
      if (key == "next_index")
      {
        size_t consumed = 0;
        next_index = std::stoull(val, &consumed);
        if (val.empty() || val[0] == '-' || consumed != val.size())
          return false;
      }
      else if (key == "last_node_time_s")
        time = number(val);
      else if (key == "datum_lat")
        latitude = number(val);
      else if (key == "datum_lon")
        longitude = number(val);
    }
    if (next_index == 0 || !values.exists(PoseKey(next_index - 1)))
      return false;
    // Metadata must name the actual last pose, including legacy archives.
    // Otherwise the first live between-factor can reference a missing key.
    for (const auto& value : values)
    {
      const gtsam::Symbol sym(value.key);
      if (sym.chr() != 'x')
        continue;
      const auto pose = value.value.cast<gtsam::Pose2>();
      if (sym.index() >= next_index || !std::isfinite(pose.x()) || !std::isfinite(pose.y()) ||
          !std::isfinite(pose.theta()))
        return false;
    }
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}
}  // namespace

void GraphManager::Reset()
{
  std::lock_guard<std::mutex> lock(mu_);
  ResetLocked();
}

void GraphManager::ResetLocked()
{
  // Rebuild iSAM2 with the same parameters used in the constructor —
  // GTSAM has no public clear() API.
  gtsam::ISAM2Params p;
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
  isam_ = gtsam::ISAM2(p);

  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_ = gtsam::Values();
  current_estimate_ = gtsam::Values();
  estimate_dirty_ = true;

  next_index_ = 0;
  last_node_time_s_ = 0.0;
  node_time_index_.clear();
  initialized_ = false;

  accum_.Reset();
  // A new graph's absolute bias prior starts at zero. A reused instance must
  // not seed pose-only restored states from the previous graph/session bias.
  current_bias_estimate_ = 0.0;
  slip_window_.Clear();
  queue_ = UnaryQueue{};

  latest_.reset();
  ticks_since_cov_ = 0;
  // This reset is the live-graph self-heal path (IndeterminateSystem catch):
  // the graph re-seeds in the same datum frame. The LiDAR occupancy grid the
  // map anchor localises against lives in the node, not here, and survives.

  // Cancel any in-flight async rebase: phase 3 of RebaseISAM2 checks
  // this flag before swapping isam_, so clearing it here makes the
  // in-flight worker discard its freshly-built tree instead of
  // overwriting our just-cleared one.
  rebase_in_progress_ = false;
  rebase_pending_factors_.resize(0);
  rebase_pending_values_.clear();
}

bool GraphManager::Save(const std::string& prefix) const
try
{
  // Phase 1: snapshot under the lock. Holding the lock for the full
  // I/O duration (~500 ms on this robot's eMMC) is what stalled Nav2's
  // map→odom TF lookups during periodic 5-min auto-saves on
  // 2026-05-14 — controller_server hit `Transform data too old` and
  // aborted FollowStrip / DockRobot. Copy out everything Save needs
  // (gtsam::Values is value-copyable), then release the lock so Tick can keep running
  // while the bytes hit disk.
  gtsam::Values estimate_snapshot;
  uint64_t next_index_snapshot = 0;
  double last_node_time_s_snapshot = 0.0;
  {
    std::lock_guard<std::mutex> lock(mu_);
    // Refuse to persist an empty graph. An auto-save fired right after a
    // Reset() would otherwise overwrite the on-disk files with
    // next_index=0 / count=0; on next launch Load() restored that state,
    // marked initialized_, and the first Tick crashed with a Symbol-index
    // underflow. Keep whatever good state was on disk before the reset.
    if (!initialized_ || next_index_ == 0)
      return false;
    // Manual / on-checkpoint path. Always refreshes from iSAM2 — the
    // serialized state must reflect all factor updates since the last
    // RefreshEstimateLocked() call.
    RefreshEstimateLocked();
    // The restore contract is pose-only: transient bias states are
    // re-estimated against the new session's sensor calibration. Serializing
    // GenericValue<double> here was both unnecessary and unregistered, making
    // every checkpoint with preintegration enabled fail to save.
    for (const auto& value : current_estimate_)
    {
      if (gtsam::Symbol(value.key).chr() == 'x')
        estimate_snapshot.insert(value.key, value.value);
    }
    next_index_snapshot = next_index_;
    last_node_time_s_snapshot = last_node_time_s_;
  }

  // Phase 2: I/O without the lock.
  std::ostringstream meta_os;
  meta_os << "next_index=" << next_index_snapshot << "\n";
  // Datum (WGS84) tags the graph to its garden so a graph saved at one site
  // is rejected at another on Load. 9 decimals ≈ 0.1 mm at lat/lon scale.
  meta_os << "datum_lat=" << std::fixed << std::setprecision(9) << params_.datum_lat << "\n";
  meta_os << "datum_lon=" << std::fixed << std::setprecision(9) << params_.datum_lon << "\n";
  // Wall-clock seconds need ≥10 integer digits + a few fractional, so
  // default 6-digit iostream precision (1.7774e+09) silently corrupts
  // the timestamp. setprecision(15) is safe for double round-trip.
  meta_os << "last_node_time_s=" << std::setprecision(6) << last_node_time_s_snapshot << "\n";
  // If recovery selected .previous, never rotate an unreadable primary over
  // it. Framing/CRC alone cannot validate XML or metadata pose identities.
  checkpoint::Payload old;
  gtsam::Values old_values;
  uint64_t old_index = 0;
  double old_time = 0.0, old_latitude = 0.0, old_longitude = 0.0;
  const bool backup_current =
      checkpoint::Read(prefix + ".checkpoint", old) &&
      DecodeSnapshot(old, old_values, old_index, old_time, old_latitude, old_longitude);
  return checkpoint::Save(prefix,
                          {gtsam::serializeXML(estimate_snapshot), meta_os.str()},
                          backup_current);
}
catch (const std::exception& e)
{
  fprintf(stderr, "fusion_graph::Save: %s\n", e.what());
  return false;
}

bool GraphManager::Load(const std::string& prefix)
try
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_)
    return false;

  gtsam::Values loaded_values;
  uint64_t next_idx = 0;
  double last_t = 0.0;
  double loaded_datum_lat = 0.0;
  double loaded_datum_lon = 0.0;
  const std::string current = prefix + ".checkpoint";
  const std::string previous = current + ".previous";
  // An invalid modern file must never silently select a stale legacy pair.
  // symlink_status also treats a dangling symlink as a present, bad file.
  const auto present = [](const std::string& path)
  {
    return std::filesystem::symlink_status(path).type() != std::filesystem::file_type::not_found;
  };
  const bool modern = present(current) || present(previous);
  bool decoded = false;
  for (const auto& path :
       modern ? std::vector<std::string>{current, previous} : std::vector<std::string>{prefix})
  {
    try
    {
      checkpoint::Payload payload;
      if (modern)
      {
        if (!checkpoint::Read(path, payload))
          continue;
      }
      else
      {
        const auto read_legacy = [](const std::string& name, std::string& bytes)
        {
          std::ifstream in(name, std::ios::binary);
          if (!in)
            return false;
          bytes.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
          return !in.bad();
        };
        if (!read_legacy(path + ".graph", payload.graph) ||
            !read_legacy(path + ".meta", payload.metadata))
          continue;
      }
      if (!DecodeSnapshot(
              payload, loaded_values, next_idx, last_t, loaded_datum_lat, loaded_datum_lon))
        continue;
      decoded = true;
      break;
    }
    catch (const std::exception&)
    {
      // Try only the retained complete generation; no mixed sidecar fallback.
    }
  }
  if (!decoded)
    return false;

  // Cross-garden guard: if BOTH the configured and the persisted datum are set,
  // reject a graph whose datum differs — its node poses belong to a different
  // site and would inject wrong absolute factors. Skipped when the configured datum
  // is unset (0,0) so self-seeded bootstrap reloads.
  const bool have_cfg_datum =
      std::abs(params_.datum_lat) > 1.0e-9 || std::abs(params_.datum_lon) > 1.0e-9;
  const bool have_persisted_datum =
      std::abs(loaded_datum_lat) > 1.0e-9 || std::abs(loaded_datum_lon) > 1.0e-9;
  if (have_cfg_datum && have_persisted_datum &&
      (std::abs(loaded_datum_lat - params_.datum_lat) > 1.0e-6 ||
       std::abs(loaded_datum_lon - params_.datum_lon) > 1.0e-6))
  {
    fprintf(stderr,
            "fusion_graph::Load: datum mismatch (persisted map cfg=%.9f,%.9f vs "
            "loaded=%.9f,%.9f) — rejecting cross-garden map\n",
            params_.datum_lat,
            params_.datum_lon,
            loaded_datum_lat,
            loaded_datum_lon);
    return false;
  }

  // Refuse to restore a degenerate persisted state. With next_idx == 0
  // (or no values at all) marking initialized_ would let CreateNodeLocked
  // form PoseKey(next_idx - 1) and underflow into a 2^64-1 Symbol index
  // — GTSAM throws std::invalid_argument and the process aborts. Treat
  // this as "no graph on disk" so the node bootstraps from the next GPS
  // / set_pose seed.
  if (next_idx == 0 || loaded_values.empty())
    return false;

  // Re-seed iSAM2 with each loaded pose pinned by a tight prior; the
  // priors keep optimization stable as new wheel/GPS factors arrive.
  // The covariances allow new absolute observations to refine the loaded poses.
  //
  // Sliding-window cap on RESTORE. RebaseISAM2 caps the LIVE graph at
  // max_graph_nodes, but it only runs from the 30 s maintenance timer — Load()
  // used to restore the ENTIRE persisted graph, so a session with many restarts
  // (each reloading a graph that grew between saves) inflated the live graph to
  // tens of thousands of nodes (observed ~21k after ~8 restarts), blowing up the
  // first-Tick marginalCovariance() cost (O(N)) and pushing cov_xx toward ~1 m
  // until the maintenance rebase finally pruned it. Restore only the newest
  // max_graph_nodes poses so a reload lands in the same bounded state the live
  // graph holds. next_index_ is unchanged, so the first new node's
  // BetweenFactor(next_idx-1, next_idx) still references a restored node
  // (next_idx-1 >= cutoff for any max_graph_nodes >= 1).
  const uint64_t cutoff = (params_.max_graph_nodes > 0 && next_idx > params_.max_graph_nodes)
                              ? next_idx - params_.max_graph_nodes
                              : 0;
  auto pin_noise = MakeDiagonal({0.01, 0.01, 0.01});
  gtsam::NonlinearFactorGraph fg;
  gtsam::Values kept_values;
  for (const auto& key_value : loaded_values)
  {
    const gtsam::Symbol sym(key_value.key);
    // Only pose keys ('x') are restored. A graph saved with use_imu_preint=true
    // also serializes per-node gyro-bias variables (Symbol 'b', type double):
    // casting those to Pose2 throws at boot, and re-inserting them with no
    // factor referencing them leaves the update underconstrained. Bias is
    // cheap to re-estimate live, so drop them on load.
    if (sym.chr() != 'x')
      continue;
    if (cutoff > 0 && sym.index() < cutoff)
      continue;  // older than the window — drop (its info was already in priors)
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(key_value.key,
                                            key_value.value.cast<gtsam::Pose2>(),
                                            pin_noise));
    kept_values.insert(key_value.key, key_value.value);
  }
  auto restored = isam_;
  restored.update(fg, kept_values);
  isam_ = std::move(restored);
  estimate_dirty_ = true;

  next_index_ = next_idx;
  // Clamp loaded timestamp to "now" so a meta written with stale
  // precision (legacy iostream default, e.g. "1.7774e+09") doesn't
  // park last_node_time_s_ in the future, which would block Tick()
  // from creating any new node until wall-clock catches up.
  const double now_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  last_node_time_s_ = std::min(last_t, now_s);
  initialized_ = true;

  // Populate latest_ with the highest-index loaded pose so consumers
  // (LatestSnapshot, OnSetPose's ForceAnchor path, TF publisher) see
  // a valid snapshot the instant the load completes. Without this:
  //   - OnSetPose treats a freshly-loaded graph as "no latest" and
  //     silently drops the set_pose seed, leaving fusion_graph at the
  //     persisted last-session pose instead of the operator dock_pose.
  //   - PublishOutputs short-circuits until Tick() exits stationary
  //     throttle, leaving Nav2 without a map→odom TF for the whole
  //     warm-up window. ekf_odom_node used to mask that by publishing
  //     odom→base independently; with fusion_graph owning both TFs
  //     now, Nav2 hangs outright on the missing chain.
  // Try the real marginal covariance once; fall back to a loose
  // placeholder if isam_ throws (Bayes tree path may not include the
  // key yet for some edge cases). The first Tick() will overwrite
  // with a proper cov.
  if (next_index_ > 0 && HasPoseAt(next_index_ - 1))
  {
    TickOutput out;
    out.pose = PoseAt(next_index_ - 1);
    try
    {
      out.covariance = isam_.marginalCovariance(PoseKey(next_index_ - 1));
    }
    catch (const std::exception&)
    {
      out.covariance = Eigen::Matrix3d::Identity() * 0.01;
    }
    out.node_index = next_index_ - 1;
    out.timestamp = last_node_time_s_;
    latest_ = out;
    // Persisted files store the latest timestamp but not a complete per-node
    // timeline. Index only the pose whose epoch is known; subsequent live
    // ticks extend the history without guessing timestamps for older poses.
    node_time_index_.clear();
    node_time_index_.emplace_back(last_node_time_s_, out.node_index);
  }

  return true;
}
catch (const std::exception& e)
{
  fprintf(stderr, "fusion_graph::Load: %s\n", e.what());
  return false;
}

}  // namespace fusion_graph
