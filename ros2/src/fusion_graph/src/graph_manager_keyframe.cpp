// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager — RTK-anchored keyframe map.
//
// A keyframe is a FROZEN {absolute map-frame base_footprint pose, body-frame
// scan} captured under stable RTK-Fixed. Keyframes are NOT iSAM2 variables and
// live in a std::map separate from scans_, so the precision-flooring paths
// (RebaseISAM2 / RigidTransformAll re-priors) and the sliding-window cutoff
// never touch them — their ~3 mm RTK precision survives by construction. They
// are the absolute reference the scan-to-keyframe factor matches against to
// hold <2 cm during multi-minute RTK-Float windows (when GPS itself is
// decimetre-biased and dead-reckoning would otherwise drift unbounded).
//
// This is a separate translation unit (methods of GraphManager defined here)
// to keep graph_manager.cpp from growing past the project's file-size budget
// and to isolate the keyframe feature.

#include <algorithm>
#include <cstdint>
#include <istream>
#include <ostream>

#include "fusion_graph/graph_manager.hpp"

namespace fusion_graph
{

std::optional<uint64_t> GraphManager::AddKeyframe(const gtsam::Pose2& abs_pose,
                                                  const std::vector<Eigen::Vector2d>& scan_body)
{
  std::lock_guard<std::mutex> lock(mu_);

  // Spatial decimation: reject a new keyframe that lands within
  // kf_spacing_m/2 of an existing one. Keeps the map a sparse ~grid so a
  // stationary RTK-Fixed dwell can't spam thousands of coincident keyframes
  // (the 2026-06-09 unbounded-growth lesson) and bounds FindKeyframesNearXY.
  const double min_d = params_.kf_spacing_m * 0.5;
  const double min_d2 = min_d * min_d;
  for (const auto& [id, kf] : keyframes_)
  {
    const double dx = kf.abs_pose.x() - abs_pose.x();
    const double dy = kf.abs_pose.y() - abs_pose.y();
    if (dx * dx + dy * dy < min_d2)
      return std::nullopt;
  }

  const uint64_t id = next_keyframe_id_++;
  keyframes_[id] = Keyframe{abs_pose, scan_body};

  // Hard cap: evict the oldest (lowest id) when over max_keyframes. With the
  // spacing decimation above a garden stays well under the cap, so eviction is
  // a backstop. Oldest-first is the simplest bounded policy; revisited regions
  // re-capture on the next pass.
  while (params_.max_keyframes > 0 && keyframes_.size() > params_.max_keyframes)
    keyframes_.erase(keyframes_.begin());

  return id;
}

std::optional<GraphManager::Keyframe> GraphManager::GetKeyframe(uint64_t kf_id) const
{
  std::lock_guard<std::mutex> lock(mu_);
  auto it = keyframes_.find(kf_id);
  if (it == keyframes_.end())
    return std::nullopt;
  return it->second;  // value copy — caller runs the matcher lock-free
}

std::vector<uint64_t> GraphManager::FindKeyframesNearXY(double x,
                                                        double y,
                                                        double max_dist_m,
                                                        size_t max_candidates) const
{
  std::lock_guard<std::mutex> lock(mu_);
  // Clone of FindNodesNearXY, but reads the FROZEN kf.abs_pose directly (no
  // Bayes-tree PoseAt) and applies NO window cutoff — keyframes are the durable
  // absolute map and remain queryable after their region left the pose window.
  std::vector<std::pair<double, uint64_t>> hits;
  hits.reserve(keyframes_.size());
  const double max_d2 = max_dist_m * max_dist_m;
  for (const auto& [id, kf] : keyframes_)
  {
    const double dx = kf.abs_pose.x() - x;
    const double dy = kf.abs_pose.y() - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 <= max_d2)
      hits.emplace_back(d2, id);
  }
  std::sort(hits.begin(), hits.end());
  std::vector<uint64_t> out;
  out.reserve(std::min(max_candidates, hits.size()));
  for (size_t i = 0; i < hits.size() && out.size() < max_candidates; ++i)
    out.push_back(hits[i].second);
  return out;
}

size_t GraphManager::KeyframeCount() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return keyframes_.size();
}

void GraphManager::ClearKeyframes()
{
  std::lock_guard<std::mutex> lock(mu_);
  keyframes_.clear();
  next_keyframe_id_ = 0;
}

// ── Persistence (binary, self-describing) ────────────────────────────
// Format: 'FGKF' magic (4 bytes) + uint32 version + uint64 count, then per
// keyframe: uint64 id, double x/y/theta (abs_pose), uint64 m, m*{double x,y}
// (scan_body). The magic+version header (unlike .scans) lets Load detect an
// absent or old-format file and start from an empty map rather than fail.
void GraphManager::SerializeKeyframesBinary(std::ostream& os,
                                            const std::map<uint64_t, Keyframe>& keyframes)
{
  const char magic[4] = {'F', 'G', 'K', 'F'};
  os.write(magic, 4);
  const uint32_t version = 1;
  os.write(reinterpret_cast<const char*>(&version), sizeof(version));
  uint64_t n = keyframes.size();
  os.write(reinterpret_cast<const char*>(&n), sizeof(n));
  for (const auto& [id, kf] : keyframes)
  {
    os.write(reinterpret_cast<const char*>(&id), sizeof(id));
    const double pose[3] = {kf.abs_pose.x(), kf.abs_pose.y(), kf.abs_pose.theta()};
    os.write(reinterpret_cast<const char*>(pose), sizeof(pose));
    uint64_t m = kf.scan_body.size();
    os.write(reinterpret_cast<const char*>(&m), sizeof(m));
    for (const auto& p : kf.scan_body)
    {
      const double xy[2] = {p.x(), p.y()};
      os.write(reinterpret_cast<const char*>(xy), sizeof(xy));
    }
  }
}

bool GraphManager::DeserializeKeyframesBinary(std::istream& is,
                                              std::map<uint64_t, Keyframe>& keyframes)
{
  char magic[4] = {0, 0, 0, 0};
  if (!is.read(magic, 4))
    return false;
  if (magic[0] != 'F' || magic[1] != 'G' || magic[2] != 'K' || magic[3] != 'F')
    return false;
  uint32_t version = 0;
  if (!is.read(reinterpret_cast<char*>(&version), sizeof(version)))
    return false;
  if (version != 1)
    return false;  // unknown version → caller treats as absent (empty map)
  uint64_t n = 0;
  if (!is.read(reinterpret_cast<char*>(&n), sizeof(n)))
    return false;
  for (uint64_t i = 0; i < n; ++i)
  {
    uint64_t id = 0, m = 0;
    double pose[3];
    if (!is.read(reinterpret_cast<char*>(&id), sizeof(id)))
      return false;
    if (!is.read(reinterpret_cast<char*>(pose), sizeof(pose)))
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
    keyframes.emplace(id, Keyframe{gtsam::Pose2(pose[0], pose[1], pose[2]), std::move(pts)});
  }
  return true;
}

}  // namespace fusion_graph
