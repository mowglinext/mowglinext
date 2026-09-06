// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
//
// Pure validation of a LiDAR-map anchor estimate before it becomes a factor.
//
// Why this exists (field 2026-09-06, GPS container stopped 2 min 40 s while
// the operator drove slowly with a U-turn): the particle filter lost the
// robot as it left the mapped area, yet every estimate still entered the
// graph as an XY prior at a 5 cm σ floor. Result: fused pose 10.0 m and ~88°
// off at RTK return, while wheels + gyro alone were within 0.39 m. Scan-vs-
// map consistency had fallen from 97 % to 13 % of beams on known walls the
// whole time — the filter's own covariance never said so. So trust is now
// earned per estimate, from three independent witnesses:
//
//   1. Scan consistency: the fraction (and count) of this scan's returns that
//      land on mapped occupied cells at the ESTIMATE. A lost filter, or a
//      robot outside the map, scores low — that is exactly the 13 % case.
//   2. Particle spread: a wide cloud is not an anchor.
//   3. Dead reckoning: wheels + gyro drift slowly (≈ 0.3 m + 2 % of distance
//      here). An estimate farther than that from the DR-predicted pose is
//      wrong, whatever the filter thinks — DR is the better witness at that
//      point, and the caller re-seeds the filter from it.
#pragma once

#include <algorithm>
#include <cmath>

namespace fusion_graph
{

struct LidarAnchorValidatorParams
{
  double min_hit_ratio = 0.5;  // fraction of beams on mapped occupied cells at the estimate
  int min_hit_count = 30;  // and at least this many, so a 5-beam scan can't pass
  double max_sigma_m = 0.5;  // particle spread cap (largest 1σ of the XY covariance)
  double dr_budget_m = 0.3;  // dead-reckoning drift budget: constant ...
  double dr_drift_frac = 0.02;  // ... plus this fraction of the distance driven since seed
};

struct LidarAnchorCandidate
{
  double x = 0.0;
  double y = 0.0;
  double sigma_m = 0.0;  // largest 1σ of the estimate's XY covariance
  double hit_ratio = 0.0;  // scan consistency at the estimate
  int hit_count = 0;
  double dr_x = 0.0;  // dead-reckoning prediction of the same pose
  double dr_y = 0.0;
  double dist_since_seed_m = 0.0;  // path driven since the filter was (re)seeded
};

enum class LidarAnchorVerdict
{
  kAccepted,
  kRejectedScore,  // scan does not fit the map at the estimate
  kRejectedSpread,  // particle cloud too wide
  kRejectedDeadReckoning,  // farther from DR than DR could have drifted
};

inline double DeadReckoningBudgetM(const LidarAnchorValidatorParams& p, double dist_since_seed_m)
{
  return p.dr_budget_m + p.dr_drift_frac * std::max(0.0, dist_since_seed_m);
}

inline LidarAnchorVerdict ValidateLidarAnchor(const LidarAnchorCandidate& c,
                                              const LidarAnchorValidatorParams& p)
{
  if (c.hit_ratio < p.min_hit_ratio || c.hit_count < p.min_hit_count)
    return LidarAnchorVerdict::kRejectedScore;
  if (!(c.sigma_m <= p.max_sigma_m))  // also rejects NaN
    return LidarAnchorVerdict::kRejectedSpread;
  const double d = std::hypot(c.x - c.dr_x, c.y - c.dr_y);
  if (!(d <= DeadReckoningBudgetM(p, c.dist_since_seed_m)))
    return LidarAnchorVerdict::kRejectedDeadReckoning;
  return LidarAnchorVerdict::kAccepted;
}

inline const char* ToString(LidarAnchorVerdict v)
{
  switch (v)
  {
    case LidarAnchorVerdict::kAccepted:
      return "accepted";
    case LidarAnchorVerdict::kRejectedScore:
      return "rejected_score";
    case LidarAnchorVerdict::kRejectedSpread:
      return "rejected_spread";
    case LidarAnchorVerdict::kRejectedDeadReckoning:
      return "rejected_dead_reckoning";
  }
  return "?";
}

}  // namespace fusion_graph
