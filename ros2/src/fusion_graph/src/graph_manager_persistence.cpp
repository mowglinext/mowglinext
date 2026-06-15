// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — persistence: Save/Load/Reset + binary (de)serializers. (The class
// implementation is split across several translation units to keep each file within the project's
// 600-line budget; all share graph_manager.hpp + the inline PoseKey().)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>

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

void SerializeScansBinary(const std::map<uint64_t, std::vector<Eigen::Vector2d>>& scans,
                          std::ostream& os)
{
  uint64_t n = scans.size();
  os.write(reinterpret_cast<const char*>(&n), sizeof(n));
  for (const auto& [idx, pts] : scans)
  {
    os.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
    uint64_t m = pts.size();
    os.write(reinterpret_cast<const char*>(&m), sizeof(m));
    for (const auto& p : pts)
    {
      double xy[2] = {p.x(), p.y()};
      os.write(reinterpret_cast<const char*>(xy), sizeof(xy));
    }
  }
}

bool DeserializeScansBinary(std::istream& is,
                            std::map<uint64_t, std::vector<Eigen::Vector2d>>& scans)
{
  uint64_t n = 0;
  if (!is.read(reinterpret_cast<char*>(&n), sizeof(n)))
    return false;
  for (uint64_t i = 0; i < n; ++i)
  {
    uint64_t idx = 0, m = 0;
    if (!is.read(reinterpret_cast<char*>(&idx), sizeof(idx)))
      return false;
    if (!is.read(reinterpret_cast<char*>(&m), sizeof(m)))
      return false;
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(m);
    for (uint64_t j = 0; j < m; ++j)
    {
      double xy[2];
      if (!is.read(reinterpret_cast<char*>(xy), sizeof(xy)))
        return false;
      pts.emplace_back(xy[0], xy[1]);
    }
    scans.emplace(idx, std::move(pts));
  }
  return true;
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
  initialized_ = false;

  accum_.Reset();
  queue_ = UnaryQueue{};

  latest_.reset();
  loop_closures_added_ = 0;
  ticks_since_cov_ = 0;
  loop_closure_edges_.clear();
  scans_.clear();
  // keyframes_ is deliberately NOT cleared here. This reset is the live-graph
  // self-heal path (IndeterminateSystem catch) — the keyframe map is the
  // reset-exempt ABSOLUTE reference (datum-anchored, captured only under stable
  // RTK-Fixed) and stays valid after the live graph re-seeds in the same datum
  // frame. A user-triggered full wipe clears it via ClearKeyframes().

  // Cancel any in-flight async rebase: phase 3 of RebaseISAM2 checks
  // this flag before swapping isam_, so clearing it here makes the
  // in-flight worker discard its freshly-built tree instead of
  // overwriting our just-cleared one.
  rebase_in_progress_ = false;
  rebase_pending_factors_.resize(0);
  rebase_pending_values_.clear();
}

bool GraphManager::Save(const std::string& prefix) const
{
  // Phase 1: snapshot under the lock. Holding the lock for the full
  // I/O duration (~500 ms on this robot's eMMC) is what stalled Nav2's
  // map→odom TF lookups during periodic 5-min auto-saves on
  // 2026-05-14 — controller_server hit `Transform data too old` and
  // aborted FollowStrip / DockRobot. Copy out everything Save needs
  // (gtsam::Values is value-copyable; scans_ is write-once after the
  // entry is inserted, so the map copy is consistent without further
  // synchronization), then release the lock so Tick can keep running
  // while the bytes hit disk.
  gtsam::Values estimate_snapshot;
  std::map<uint64_t, std::vector<Eigen::Vector2d>> scans_snapshot;
  std::map<uint64_t, Keyframe> keyframes_snapshot;
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
    estimate_snapshot = current_estimate_;
    scans_snapshot = scans_;
    keyframes_snapshot = keyframes_;  // write-once entries → consistent copy
    next_index_snapshot = next_index_;
    last_node_time_s_snapshot = last_node_time_s_;
  }

  // Phase 2: I/O without the lock.
  try
  {
    std::ofstream graph_os(prefix + ".graph");
    if (!graph_os)
      return false;
    graph_os << gtsam::serializeXML(estimate_snapshot);
    graph_os.close();

    std::ofstream scans_os(prefix + ".scans", std::ios::binary);
    if (!scans_os)
      return false;
    SerializeScansBinary(scans_snapshot, scans_os);
    scans_os.close();

    // Keyframe map (4th file). Written even when empty (a valid header + 0
    // count) so the on-disk set is always complete.
    std::ofstream kf_os(prefix + ".keyframes", std::ios::binary);
    if (!kf_os)
      return false;
    SerializeKeyframesBinary(kf_os, keyframes_snapshot);
    kf_os.close();

    std::ofstream meta_os(prefix + ".meta");
    if (!meta_os)
      return false;
    meta_os << "next_index=" << next_index_snapshot << "\n";
    // Datum (WGS84) tags the map to its garden so a keyframe map is rejected
    // at a different site on Load. 9 decimals ≈ 0.1 mm at lat/lon scale.
    meta_os << "datum_lat=" << std::fixed << std::setprecision(9) << params_.datum_lat << "\n";
    meta_os << "datum_lon=" << params_.datum_lon << "\n";
    // Wall-clock seconds need ≥10 integer digits + a few fractional, so
    // default 6-digit iostream precision (1.7774e+09) silently corrupts
    // the timestamp. setprecision(15) is safe for double round-trip.
    meta_os << "last_node_time_s=" << std::setprecision(6) << last_node_time_s_snapshot << "\n";
    meta_os.close();
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "fusion_graph::Save: %s\n", e.what());
    return false;
  }
  return true;
}

bool GraphManager::Load(const std::string& prefix)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_)
    return false;

  gtsam::Values loaded_values;
  try
  {
    std::ifstream graph_is(prefix + ".graph");
    if (!graph_is)
      return false;
    std::stringstream buf;
    buf << graph_is.rdbuf();
    gtsam::deserializeXML(buf.str(), loaded_values);
  }
  catch (const std::exception&)
  {
    return false;
  }

  std::map<uint64_t, std::vector<Eigen::Vector2d>> loaded_scans;
  try
  {
    std::ifstream scans_is(prefix + ".scans", std::ios::binary);
    if (!scans_is)
      return false;
    if (!DeserializeScansBinary(scans_is, loaded_scans))
      return false;
  }
  catch (const std::exception&)
  {
    return false;
  }

  // Keyframe map (4th file). ABSENT is fine — pre-feature on-disk triples
  // ({.graph,.scans,.meta}) must still load. A present-but-corrupt/old-version
  // file degrades to an empty map rather than failing the whole restore.
  std::map<uint64_t, Keyframe> loaded_keyframes;
  try
  {
    std::ifstream kf_is(prefix + ".keyframes", std::ios::binary);
    if (kf_is && !DeserializeKeyframesBinary(kf_is, loaded_keyframes))
      loaded_keyframes.clear();
  }
  catch (const std::exception&)
  {
    loaded_keyframes.clear();
  }

  uint64_t next_idx = 0;
  double last_t = 0.0;
  double loaded_datum_lat = 0.0;
  double loaded_datum_lon = 0.0;
  try
  {
    std::ifstream meta_is(prefix + ".meta");
    if (!meta_is)
      return false;
    std::string line;
    while (std::getline(meta_is, line))
    {
      auto eq = line.find('=');
      if (eq == std::string::npos)
        continue;
      const std::string key = line.substr(0, eq);
      const std::string val = line.substr(eq + 1);
      if (key == "next_index")
        next_idx = std::stoull(val);
      else if (key == "last_node_time_s")
        last_t = std::stod(val);
      else if (key == "datum_lat")
        loaded_datum_lat = std::stod(val);
      else if (key == "datum_lon")
        loaded_datum_lon = std::stod(val);
    }
  }
  catch (const std::exception&)
  {
    return false;
  }

  // Cross-garden guard: if BOTH the configured and the persisted datum are set,
  // reject a map whose datum differs — its keyframe absolute poses (and graph)
  // belong to a different site and would inject wrong absolute factors. Skipped
  // when the configured datum is unset (0,0) so self-seeded bootstrap reloads.
  const bool have_cfg_datum =
      std::abs(params_.datum_lat) > 1.0e-9 || std::abs(params_.datum_lon) > 1.0e-9;
  const bool have_persisted_datum =
      std::abs(loaded_datum_lat) > 1.0e-9 || std::abs(loaded_datum_lon) > 1.0e-9;
  if (have_cfg_datum && have_persisted_datum &&
      (std::abs(loaded_datum_lat - params_.datum_lat) > 1.0e-6 ||
       std::abs(loaded_datum_lon - params_.datum_lon) > 1.0e-6))
  {
    fprintf(stderr,
            "fusion_graph::Load: datum mismatch (persisted map cfg=%.7f,%.7f vs "
            "loaded=%.7f,%.7f) — rejecting cross-garden map\n",
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
  // The covariances are looser than the live priors so loop-closures
  // can still re-balance the loaded portion if it was inconsistent.
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
    if (cutoff > 0 && gtsam::Symbol(key_value.key).index() < cutoff)
      continue;  // older than the window — drop (its info was already in priors)
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(key_value.key,
                                            key_value.value.cast<gtsam::Pose2>(),
                                            pin_noise));
    kept_values.insert(key_value.key, key_value.value);
  }
  isam_.update(fg, kept_values);
  estimate_dirty_ = true;

  scans_ = std::move(loaded_scans);
  // Drop scan blobs for nodes outside the restored window — their poses are no
  // longer in the graph so they can't be loop-closure candidates; keeping them
  // only wastes RAM.
  if (cutoff > 0)
  {
    for (auto it = scans_.begin(); it != scans_.end();)
    {
      if (it->first < cutoff)
        it = scans_.erase(it);
      else
        ++it;
    }
  }
  // Keyframes are the durable absolute map — restored WHOLE (no window cutoff),
  // independent of the pose window. Resume the id counter past the highest id.
  keyframes_ = std::move(loaded_keyframes);
  next_keyframe_id_ = keyframes_.empty() ? 0 : (keyframes_.rbegin()->first + 1);
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
  }

  return true;
}

}  // namespace fusion_graph
