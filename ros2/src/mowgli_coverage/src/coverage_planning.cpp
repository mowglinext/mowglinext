// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Simple boustrophedon coverage planner — see coverage_planning.hpp for the
// design rationale (headland rings + straight serpentine swaths, no turn
// planning; turns are the navigation stack's in-place pivots).

#include "mowgli_coverage/coverage_planning.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>

#include "mowgli_interfaces/coverage_geometry.hpp"

// GDAL/OGR (F2C's geometry backend) — bufferRingOutward grows drawn-obstacle
// rings with OGRPolygon::Buffer. Explicit include: the transitive path via
// fields2cover.h is an implementation detail of F2C.
#include "ogr_geometry.h"

namespace mowgli_coverage
{

namespace
{

constexpr double kDensifyStep = 0.10;  // m between ring polyline points

// Format a "dropped <what> <val><cmp><thresh>" diagnostics line without pulling
// in <sstream>/<iomanip>. Values are short distances/areas, so 4 decimals is
// plenty of resolution for a log line.
std::string fmtDrop(const char* what, double value, const char* cmp, double thresh)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), "dropped %s%.4f%s%.4f", what, value, cmp, thresh);
  return std::string(buf);
}

// AUTO swath-angle field-size gate. f2c::sg::BruteForce::generateBestSwaths
// sweeps 180 candidate angles (1° step) and regenerates the full swath set at
// each, so its cost scales ~linearly with field area and explodes on large
// fields — a 100×100 m field takes ~24 s, far past the action (15 s) and BT
// (12 s) planning timeouts, so the coverage action fails outright. Above this
// area we skip the exhaustive search and use the boundary's longest-edge
// orientation (the angle the search almost always converges to on a rectangular
// lawn anyway — swaths along the long side minimise turns), which is ~3 orders
// of magnitude cheaper and equally deterministic across re-plans.
//
// The threshold is deliberately conservative: the exhaustive search measures
// ~1.7 s at 400 m² on an x86 dev host, and the target ARM SBC is ~5× slower, so
// even a ~400 m² field is ~8 s of search there — already close to the 12 s BT
// timeout. Keeping the gate at 400 m² guarantees AUTO planning never times out
// on hardware; larger lawns simply use the (near-optimal) longest-edge angle.
constexpr double kAutoAngleMaxAreaM2 = 400.0;  // ~20 × 20 m

// AUTO swath-angle search resolution (radians). f2c::sg::BruteForce defaults to
// a 1° step, which — sweeping the FULL 2π — regenerates the entire swath set 360
// times per plan (the dominant coverage-planning cost, and single-threaded on
// the Pi4 / Cortex-A72 target). The swath-count objective (NSwath) is near-flat
// within a few degrees of the optimum, so a 5° step (72 candidates over 2π) cuts
// that cost ~5× while leaving the chosen angle — and thus the whole plan —
// essentially unchanged. It is a FIXED constant (not tuned per plan) so the
// argmin is deterministic across re-plans, which the resume cursor relies on.
constexpr double kAutoAngleStepRad = 5.0 * M_PI / 180.0;

// Largest far-side remainder strip (m) left by BruteForce's fixed lane stepping
// that is still accepted as is. Above it the lanes are re-spread evenly over the
// cell (generateEvenSwaths). ~ the default swath_overlap: a strip this thin is
// already inside the overlap between the blade and the neighbouring lane.
constexpr double kSwathRemainderTolM = 0.02;

// On-edge tolerance for allInside(). The outermost DRIVEN geometry can lie
// EXACTLY on the ring it is validated against: with the headland ring stage
// DISABLED (num_headland_passes < 0 → n_rings == 0, issue #429) the swath ENDS
// are GEOS intersection points sitting on safe_cells' exterior, which is also
// the connector clearance ring. pointInRing() is a strict-`>` ray-crossing
// test, so a pose exactly ON an edge is judged inside on one side of the
// polygon and outside on the opposite side — roughly half those ends would be
// rejected, and buildConnector's allInside() check would then split the
// sub-path at every U-turn (one blade-off Nav2 transit per swath).
//
// The fix belongs in the PREDICATE, not in the geometry. 1 mm is ~1e9× the
// double round-off at map-frame coordinate magnitudes, 1/80th of op_width/2 and
// 1/50th of the server's kBoundarySlackM (0.05 m) verify slack, so it cannot
// enable any turn arc and cannot move the chassis measurably. Mirrors the
// tolerant-containment pattern coverage_server.cpp already uses (!pointInRing
// && distanceToRing > slack).
//
// An earlier revision instead EXPANDED the zero-ring clearance ring outward by
// 0.03 m. Do not reintroduce that: 0.03 m is ~5× too small to fit any Dubins
// turn-around (buildConnector's production min radius is 0.20 m), so
// it bought no arc it was added for, yet it let a connector centerline ride
// 3 cm PAST the recorded line — invisible to the server's 0.05 m verify slack —
// which erodes the contract that no driven centerline crosses the recorded line
// (true at every chassis_safety_inset, including the 0.0 default where ring 0
// rides exactly ON it). Companion to
// kClearanceClampMarginM (see the second anonymous namespace below).
constexpr double kOnEdgeTolM = 0.001;

// Orientation (radians) of the longest edge of a cell's outer ring. A cheap,
// deterministic AUTO swath angle for large fields. Falls back to 0 (sweep along
// +x) for a degenerate ring.
double longestEdgeAngle(const f2c::types::Cell& cell)
{
  if (cell.size() == 0)
  {
    return 0.0;
  }
  const auto ring = cell.getGeometry(0);  // outer boundary
  const std::size_t n = ring.size();
  double best_len = -1.0;
  double best_angle = 0.0;
  for (std::size_t i = 0; i + 1 < n; ++i)
  {
    const auto a = ring.getGeometry(i);
    const auto b = ring.getGeometry(i + 1);
    const double dx = b.getX() - a.getX();
    const double dy = b.getY() - a.getY();
    const double len = std::hypot(dx, dy);
    if (len > best_len)
    {
      best_len = len;
      best_angle = std::atan2(dy, dx);
    }
  }
  // atan2 returns [-pi, pi]. Negative is the AUTO sentinel below, so an
  // unnormalised edge heading silently re-entered the exhaustive search.
  best_angle = std::fmod(best_angle, M_PI);
  return best_angle < 0.0 ? best_angle + M_PI : best_angle;
}

// Append the densified [a, b) segment to `out` (b exclusive: the next segment
// starts at b, so shared vertices are emitted once and segments chain).
void densifySegment(
    double ax, double ay, double bx, double by, std::vector<std::pair<double, double>>& out)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9)
  {
    return;
  }
  const int n = std::max(1, static_cast<int>(std::ceil(len / kDensifyStep)));
  for (int k = 0; k < n; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    out.emplace_back(ax + t * dx, ay + t * dy);
  }
}

// Like densifySegment but with a caller-supplied step (used by the continuous-
// path builder, which densifies at the controller step rather than the ring
// step). b exclusive.
void densifySegmentStep(double ax,
                        double ay,
                        double bx,
                        double by,
                        double step,
                        std::vector<std::pair<double, double>>& out)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9)
  {
    return;
  }
  const int n = std::max(1, static_cast<int>(std::ceil(len / std::max(1e-3, step))));
  for (int k = 0; k < n; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    out.emplace_back(ax + t * dx, ay + t * dy);
  }
}

// Convert one headland pass (an F2C MultiLineString of chained 2-point
// boundary segments — possibly several disjoint loops when the field has
// holes) into densified closed polylines, one per disjoint loop. Segments
// chain (segment c ends where c+1 begins); a new loop starts whenever the
// next segment does NOT continue from the previous one.
std::vector<std::vector<std::pair<double, double>>> ringPassToLoops(
    const f2c::types::MultiLineString& pass_lines)
{
  std::vector<std::vector<std::pair<double, double>>> loops;
  std::vector<std::pair<double, double>> current;
  double prev_end_x = 0.0, prev_end_y = 0.0;
  bool have_prev = false;

  auto close_current = [&]()
  {
    if (current.size() >= 3)
    {
      // Close the loop explicitly (the densifier emits segment ends
      // exclusively, so the final vertex == the loop start is missing).
      current.push_back(current.front());
      loops.push_back(std::move(current));
    }
    current.clear();
  };

  const std::size_t n_seg = pass_lines.size();
  for (std::size_t c = 0; c < n_seg; ++c)
  {
    const auto seg = pass_lines.getGeometry(c);
    if (seg.size() < 2)
    {
      continue;
    }
    const auto a = seg.getGeometry(0);
    const auto b = seg.getGeometry(seg.size() - 1);
    if (have_prev && std::hypot(a.getX() - prev_end_x, a.getY() - prev_end_y) > 1e-6)
    {
      close_current();  // disjoint loop boundary (field with holes)
    }
    // Densify every leg of the (usually 2-point) segment.
    for (std::size_t i = 0; i + 1 < seg.size(); ++i)
    {
      const auto p = seg.getGeometry(i);
      const auto q = seg.getGeometry(i + 1);
      densifySegment(p.getX(), p.getY(), q.getX(), q.getY(), current);
    }
    prev_end_x = b.getX();
    prev_end_y = b.getY();
    have_prev = true;
  }
  close_current();
  return loops;
}

// ── Forward-only Dubins connectors (for buildContinuousPath) ─────────────────
//
// A pose on the plane: position + heading.
struct Pose
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;  // rad
};

double wrapTwoPi(double a)
{
  while (a < 0.0)
  {
    a += 2.0 * M_PI;
  }
  while (a >= 2.0 * M_PI)
  {
    a -= 2.0 * M_PI;
  }
  return a;
}

// A forward Dubins word is three arcs/lines. We store, for the canonical
// (start at origin, heading +x, unit turn radius) frame, the three segment
// lengths t/p/q and the kind of each segment so we can sample.
enum class SegKind
{
  kLeft,
  kRight,
  kStraight
};

struct DubinsWord
{
  bool valid = false;
  double t = 0.0, p = 0.0, q = 0.0;  // segment lengths in the unit-radius frame
  SegKind s0 = SegKind::kStraight;
  SegKind s1 = SegKind::kStraight;
  SegKind s2 = SegKind::kStraight;
  double length() const
  {
    return t + p + q;
  }
};

// The six classic forward Dubins words, solved in the normalised frame where
// the start is at the origin heading +x, the goal is at distance d (already
// divided by the turn radius) and orientation (alpha→beta). Formulas from the
// standard Dubins set (e.g. Shkel & Lumelsky 2001 / OMPL DubinsStateSpace).
DubinsWord dubinsLSL(double d, double a, double b)
{
  DubinsWord w;
  w.s0 = SegKind::kLeft;
  w.s1 = SegKind::kStraight;
  w.s2 = SegKind::kLeft;
  const double tmp = d + std::sin(a) - std::sin(b);
  const double p2 = 2.0 + d * d - 2.0 * std::cos(a - b) + 2.0 * d * (std::sin(a) - std::sin(b));
  if (p2 < 0.0)
  {
    return w;
  }
  const double th = std::atan2(std::cos(b) - std::cos(a), tmp);
  w.t = wrapTwoPi(-a + th);
  w.p = std::sqrt(p2);
  w.q = wrapTwoPi(b - th);
  w.valid = true;
  return w;
}

DubinsWord dubinsRSR(double d, double a, double b)
{
  DubinsWord w;
  w.s0 = SegKind::kRight;
  w.s1 = SegKind::kStraight;
  w.s2 = SegKind::kRight;
  const double tmp = d - std::sin(a) + std::sin(b);
  const double p2 = 2.0 + d * d - 2.0 * std::cos(a - b) + 2.0 * d * (std::sin(b) - std::sin(a));
  if (p2 < 0.0)
  {
    return w;
  }
  const double th = std::atan2(std::cos(a) - std::cos(b), tmp);
  w.t = wrapTwoPi(a - th);
  w.p = std::sqrt(p2);
  w.q = wrapTwoPi(-b + th);
  w.valid = true;
  return w;
}

DubinsWord dubinsLSR(double d, double a, double b)
{
  DubinsWord w;
  w.s0 = SegKind::kLeft;
  w.s1 = SegKind::kStraight;
  w.s2 = SegKind::kRight;
  const double p2 = -2.0 + d * d + 2.0 * std::cos(a - b) + 2.0 * d * (std::sin(a) + std::sin(b));
  if (p2 < 0.0)
  {
    return w;
  }
  w.p = std::sqrt(p2);
  const double th =
      std::atan2(-std::cos(a) - std::cos(b), d + std::sin(a) + std::sin(b)) - std::atan2(-2.0, w.p);
  w.t = wrapTwoPi(-a + th);
  w.q = wrapTwoPi(-wrapTwoPi(b) + th);
  w.valid = true;
  return w;
}

DubinsWord dubinsRSL(double d, double a, double b)
{
  DubinsWord w;
  w.s0 = SegKind::kRight;
  w.s1 = SegKind::kStraight;
  w.s2 = SegKind::kLeft;
  const double p2 = -2.0 + d * d + 2.0 * std::cos(a - b) - 2.0 * d * (std::sin(a) + std::sin(b));
  if (p2 < 0.0)
  {
    return w;
  }
  w.p = std::sqrt(p2);
  const double th =
      std::atan2(std::cos(a) + std::cos(b), d - std::sin(a) - std::sin(b)) - std::atan2(2.0, w.p);
  w.t = wrapTwoPi(a - th);
  w.q = wrapTwoPi(wrapTwoPi(b) - th);
  w.valid = true;
  return w;
}

DubinsWord dubinsRLR(double d, double a, double b)
{
  DubinsWord w;
  w.s0 = SegKind::kRight;
  w.s1 = SegKind::kLeft;
  w.s2 = SegKind::kRight;
  const double tmp =
      (6.0 - d * d + 2.0 * std::cos(a - b) + 2.0 * d * (std::sin(a) - std::sin(b))) / 8.0;
  if (std::fabs(tmp) > 1.0)
  {
    return w;
  }
  w.p = wrapTwoPi(2.0 * M_PI - std::acos(tmp));
  w.t = wrapTwoPi(a - std::atan2(std::cos(a) - std::cos(b), d - std::sin(a) + std::sin(b)) +
                  w.p / 2.0);
  w.q = wrapTwoPi(a - b - w.t + w.p);
  w.valid = true;
  return w;
}

DubinsWord dubinsLRL(double d, double a, double b)
{
  DubinsWord w;
  w.s0 = SegKind::kLeft;
  w.s1 = SegKind::kRight;
  w.s2 = SegKind::kLeft;
  const double tmp =
      (6.0 - d * d + 2.0 * std::cos(a - b) + 2.0 * d * (std::sin(b) - std::sin(a))) / 8.0;
  if (std::fabs(tmp) > 1.0)
  {
    return w;
  }
  w.p = wrapTwoPi(2.0 * M_PI - std::acos(tmp));
  w.t = wrapTwoPi(-a + std::atan2(-std::cos(a) + std::cos(b), d + std::sin(a) - std::sin(b)) +
                  w.p / 2.0);
  w.q = wrapTwoPi(wrapTwoPi(b) - a + 2.0 * w.p - w.t);
  // q is derived to close the loop; recompute robustly.
  w.q = wrapTwoPi(b - a - w.t + w.p);
  w.valid = true;
  return w;
}

// Advance a unit-radius Dubins segment of the given kind by arc parameter `u`
// (radians for turns, distance for straight) from pose `p` (already in the
// unit-radius frame). Returns the new pose.
Pose dubinsStep(const Pose& p, SegKind kind, double u)
{
  Pose out = p;
  switch (kind)
  {
    case SegKind::kLeft:
      out.x = p.x + std::sin(p.theta + u) - std::sin(p.theta);
      out.y = p.y - std::cos(p.theta + u) + std::cos(p.theta);
      out.theta = p.theta + u;
      break;
    case SegKind::kRight:
      out.x = p.x - std::sin(p.theta - u) + std::sin(p.theta);
      out.y = p.y + std::cos(p.theta - u) - std::cos(p.theta);
      out.theta = p.theta - u;
      break;
    case SegKind::kStraight:
      out.x = p.x + u * std::cos(p.theta);
      out.y = p.y + u * std::sin(p.theta);
      break;
  }
  return out;
}

// Sample a forward Dubins path from `start` to `goal` (world frame) for a fixed
// `radius`, at arc step `step`. Returns the densified world-frame points
// (start inclusive, goal exclusive — the caller chains to goal). `out_len`
// receives the world-frame path length. If no word solves, returns empty and
// out_len = +inf.
std::vector<std::pair<double, double>> sampleDubins(const Pose& start,
                                                    const Pose& goal,
                                                    double radius,
                                                    double step,
                                                    double& out_len,
                                                    std::size_t word_index = 6)
{
  out_len = std::numeric_limits<double>::max();
  std::vector<std::pair<double, double>> pts;
  if (radius < 1e-6)
  {
    return pts;
  }
  // Normalise into the unit-radius, start-at-origin-heading-+x frame.
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double D = std::hypot(dx, dy);
  const double d = D / radius;
  const double th = std::atan2(dy, dx);
  const double a = wrapTwoPi(start.theta - th);
  const double b = wrapTwoPi(goal.theta - th);

  const DubinsWord words[6] = {dubinsLSL(d, a, b),
                               dubinsRSR(d, a, b),
                               dubinsLSR(d, a, b),
                               dubinsRSL(d, a, b),
                               dubinsRLR(d, a, b),
                               dubinsLRL(d, a, b)};
  const DubinsWord* best = nullptr;
  for (const auto& w : words)
  {
    if (word_index < 6 && &w != &words[word_index])
      continue;
    if (w.valid && (best == nullptr || w.length() < best->length()))
    {
      best = &w;
    }
  }
  if (best == nullptr)
  {
    return pts;
  }

  out_len = best->length() * radius;
  // Walk the three segments in the unit frame, then map back to world.
  const Pose origin{0.0, 0.0, a};  // start heading a in the rotated frame
  const SegKind kinds[3] = {best->s0, best->s1, best->s2};
  const double segs[3] = {best->t, best->p, best->q};
  const double ds = std::max(1e-3, step / radius);  // arc param step (unit frame)

  Pose cur = origin;
  const double cs = std::cos(th), sn = std::sin(th);
  auto emit = [&](const Pose& up)
  {
    // up is in unit-radius rotated frame: scale by radius, rotate by th, shift.
    const double wx = start.x + radius * (cs * up.x - sn * up.y);
    const double wy = start.y + radius * (sn * up.x + cs * up.y);
    pts.emplace_back(wx, wy);
  };
  emit(cur);
  for (int s = 0; s < 3; ++s)
  {
    double remaining = segs[s];
    while (remaining > 1e-9)
    {
      const double u = std::min(ds, remaining);
      cur = dubinsStep(cur, kinds[s], u);
      emit(cur);
      remaining -= u;
    }
  }
  // Drop the last sample (== goal) so the caller appends the next segment.
  if (!pts.empty())
  {
    pts.pop_back();
  }
  return pts;
}

// True iff every sampled point of `pts` is inside `boundary`. A point lying ON
// the boundary (within kOnEdgeTolM) counts as inside — pointInRing() alone
// resolves on-edge poses inconsistently, and the outermost driven geometry sits
// exactly on this ring when the headland stage is off (see kOnEdgeTolM).
bool allInside(const std::vector<std::pair<double, double>>& pts,
               const std::vector<std::pair<double, double>>& boundary)
{
  for (const auto& p : pts)
  {
    if (!pointInRing(p.first, p.second, boundary) &&
        distanceToRing(p.first, p.second, boundary) > kOnEdgeTolM)
    {
      return false;
    }
  }
  return true;
}

// True iff no sampled point of `pts` falls inside any hole ring (issue #333: a
// turn-around connector or corner fillet must not cut through an obstacle).
// Empty `holes` → trivially clear.
bool clearOfHoles(const std::vector<std::pair<double, double>>& pts,
                  const std::vector<std::vector<std::pair<double, double>>>& holes)
{
  for (const auto& hole : holes)
  {
    for (const auto& p : pts)
    {
      if (pointInRing(p.first, p.second, hole))
      {
        return false;
      }
    }
  }
  return true;
}

// Build a forward connector from `start` (oriented at the previous segment's
// exit heading) to `goal` (oriented at the next segment's entry heading) that
// stays inside `boundary`. Tries the nominal `turn_radius`, then shrinks toward
// `op_width/2`, picking the SHORTEST in-bounds Dubins path at each radius.
// Falls back to a straight connector (densified) if nothing fits — sets
// `used_fallback`. Returns points start-inclusive, goal-exclusive.
std::vector<std::pair<double, double>> buildConnector(
    const Pose& start,
    const Pose& goal,
    const std::vector<std::pair<double, double>>& boundary,
    const std::vector<std::vector<std::pair<double, double>>>& holes,
    double turn_radius,
    double min_radius,
    double step,
    bool& used_fallback)
{
  used_fallback = false;
  std::vector<std::pair<double, double>> best_pts;
  for (double r = turn_radius; r >= min_radius - 1e-9; r -= 0.02)
  {
    double len = 0.0;
    auto pts = sampleDubins(start, goal, r, step, len);
    if (!pts.empty() && allInside(pts, boundary) && clearOfHoles(pts, holes))
      return pts;
  }
  const double dx = goal.x - start.x, dy = goal.y - start.y;
  const double distance = std::hypot(dx, dy);
  // Preserve pivotable corners up to the existing 120-degree continuous-path
  // quality ceiling; only near-reversal joins need the longer curved option.
  constexpr double kMaxPivotTurnCos = -0.5;
  const bool reversing_join =
      dx * std::cos(start.theta) + dy * std::sin(start.theta) < kMaxPivotTurnCos * distance ||
      dx * std::cos(goal.theta) + dy * std::sin(goal.theta) < kMaxPivotTurnCos * distance;
  // Before accepting a near-reversal straight fallback, try the other Dubins words:
  // the unconstrained shortest word can leave the field while another fits.
  // This avoids replacing every compact pivot with a longer loop.
  for (double r = turn_radius; reversing_join && r >= min_radius - 1e-9; r -= 0.02)
  {
    double best_length = std::numeric_limits<double>::max();
    for (std::size_t word = 0; word < 6; ++word)
    {
      double len = 0.0;
      auto pts = sampleDubins(start, goal, r, step, len, word);
      if (len < best_length && !pts.empty() && allInside(pts, boundary) && clearOfHoles(pts, holes))
      {
        best_length = len;
        best_pts = std::move(pts);
      }
    }
    if (!best_pts.empty())
    {
      return best_pts;
    }
  }
  // Last resort: straight blind connector (may leave the boundary OR cross a
  // hole → a real, reportable gap the server surfaces).
  used_fallback = true;
  std::vector<std::pair<double, double>> straight;
  densifySegmentStep(start.x, start.y, goal.x, goal.y, step, straight);
  return straight;
}

namespace cg = mowgli_interfaces::coverage_geometry;

// Absolute heading difference, wrapped to [0, π].
double headingError(double a, double b)
{
  return std::abs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

// A straight fallback is only a continuous connector when its line is already
// tangent to both segments. Otherwise the polyline contains one or two
// zero-radius corners. FTC cannot track those while moving forward: the carrot
// crosses the corner, the heading error changes side, and the angular command
// alternates at its clamp. The 2026-09-09 field bag measured this exact pattern
// on the F2C swath ends (up to 29 sign flips in 3.3 s). Such a join is either
// kept as a PIVOT JOIN (planPivotJoin, the robot stops and rotates in place at
// each corner) or split. The 15° kink both accept while moving is the pivot
// corner contract's threshold (coverage_geometry.hpp), single-sourced there.
bool straightFallbackIsContinuous(const Pose& start, const Pose& goal)
{
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  if (std::hypot(dx, dy) < 1e-6)
  {
    return headingError(start.theta, goal.theta) <= cg::kPivotCornerMinTurnRad;
  }

  const double connector_heading = std::atan2(dy, dx);
  return headingError(connector_heading, start.theta) <= cg::kPivotCornerMinTurnRad &&
         headingError(goal.theta, connector_heading) <= cg::kPivotCornerMinTurnRad;
}

// Outcome of planPivotJoin: whether the join may stay in the sub-path, and at
// which of its two ends the robot has to pivot (a kink within
// kPivotCornerMinTurnRad is tracked while moving, like an aligned connector).
struct PivotJoin
{
  bool ok = false;
  bool corner_at_start = false;
  bool corner_at_goal = false;
};

// Decide whether a join that fits no arc and is not an aligned straight
// connector may stay inside the sub-path as a pivot join (see PivotJoinLimits
// for the rules and why). `straight` is buildConnector's straight fallback
// (start-inclusive, goal-exclusive). Pure: deterministic for a fixed plan, which
// the resume cursor relies on.
PivotJoin planPivotJoin(const Pose& start,
                        const Pose& goal,
                        const std::vector<std::pair<double, double>>& straight,
                        const std::vector<std::pair<double, double>>& boundary,
                        const std::vector<std::vector<std::pair<double, double>>>& holes,
                        const PivotJoinLimits& limits)
{
  PivotJoin join;
  if (limits.sweep_radius <= 0.0)
  {
    return join;  // disabled: split, the pre-pivot behaviour
  }
  const double dx = goal.x - start.x;
  const double dy = goal.y - start.y;
  const double length = std::hypot(dx, dy);
  // A turn-around between ADJACENT passes, never a relocation: the same bound
  // FollowStrip uses to decide a gap needs a blade-off transit.
  if (length > cg::kSegmentTransitGapM)
  {
    return join;
  }
  // The connector centreline obeys exactly the rules of every other blade-on
  // join: inside the connector boundary, clear of the margin-grown holes.
  if (!straight.empty() && (!allInside(straight, boundary) || !clearOfHoles(straight, holes)))
  {
    return join;
  }
  if (length < cg::kPivotCornerMaxStepM)
  {
    // Coincident ends: at most one pivot, straight from the exit heading to the
    // entry heading.
    join.corner_at_start = headingError(goal.theta, start.theta) > cg::kPivotCornerMinTurnRad;
  }
  else
  {
    const double heading = std::atan2(dy, dx);
    join.corner_at_start = headingError(heading, start.theta) > cg::kPivotCornerMinTurnRad;
    join.corner_at_goal = headingError(goal.theta, heading) > cg::kPivotCornerMinTurnRad;
  }
  // SAFETY: base_link is the REAR wheel axis, so a pivot sweeps the front of
  // the body round a disc of the chassis circumscribed radius. Only pivot where
  // that disc stays in the designed envelope (recorded line + soft band) and
  // off every drawn obstacle.
  if (join.corner_at_start && !pivotSweepFits(start.x, start.y, limits))
  {
    return join;
  }
  if (join.corner_at_goal && !pivotSweepFits(goal.x, goal.y, limits))
  {
    return join;
  }
  join.ok = true;
  return join;
}

// Append a pivot join to the sub-path being built. `path.back()` is the join's
// start (the previous segment's last pose). Emits, per the pivot corner
// contract: [start twin if it pivots], the straight connector, the goal, [goal
// twin if it pivots]. The next segment is then appended from its SECOND pose,
// its first being the goal already emitted here.
void appendPivotJoin(std::vector<std::pair<double, double>>& path,
                     const std::vector<std::pair<double, double>>& straight,
                     const Pose& goal,
                     const PivotJoin& join)
{
  if (join.corner_at_start)
  {
    const std::pair<double, double> corner = path.back();
    path.push_back(corner);  // exact twin: the outgoing-heading pose
  }
  if (straight.size() > 1)
  {
    path.insert(path.end(), straight.begin() + 1, straight.end());
  }
  const std::pair<double, double> goal_pt{goal.x, goal.y};
  if (std::hypot(goal_pt.first - path.back().first, goal_pt.second - path.back().second) >=
      cg::kPivotCornerMaxStepM)
  {
    path.push_back(goal_pt);
  }
  if (join.corner_at_goal)
  {
    const std::pair<double, double> corner = path.back();
    path.push_back(corner);
  }
}

// Enforce the pivot corner contract on one finished sub-path: a step shorter
// than kPivotCornerMaxStepM survives ONLY as a planner-made corner twin, and
// never more than two poses share a position. `is_twin[i]` marks pose i as the
// exact twin the join loop emitted (computed before clampInsideRing, which maps
// both twins identically but could also snap two distinct neighbours together).
// Every other near-coincident pose is dropped, so FTC can never mistake a
// numerical near-duplicate (Dubins segment seams, fillet exits, #388 clamps) for
// a pivot — whether pivot joins are enabled or not.
std::vector<std::pair<double, double>> enforcePivotCornerContract(
    const std::vector<std::pair<double, double>>& pts, const std::vector<bool>& is_twin)
{
  std::vector<std::pair<double, double>> out;
  out.reserve(pts.size());
  bool last_was_twin = false;
  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    if (!out.empty() && std::hypot(pts[i].first - out.back().first,
                                   pts[i].second - out.back().second) < cg::kPivotCornerMaxStepM)
    {
      if (is_twin[i] && !last_was_twin)
      {
        const std::pair<double, double> corner = out.back();
        out.push_back(corner);  // keep the corner, bit-exact
        last_was_twin = true;
      }
      continue;
    }
    out.push_back(pts[i]);
    last_was_twin = false;
  }
  // A corner is interior by construction: every segment has >= 2 poses (swaths
  // >= min_swath_length, closed rings), so something always follows a twin.
  // A twin left at either end could only come from a segment collapsed by the
  // near-duplicate pass above; with nothing to rotate towards (or from) it is
  // not a pivot — FTC would ignore it too (FindPivotCorners) — so drop it.
  while (out.size() >= 2 && std::hypot(out[out.size() - 1].first - out[out.size() - 2].first,
                                       out[out.size() - 1].second - out[out.size() - 2].second) <
                                cg::kPivotCornerMaxStepM)
  {
    out.pop_back();
  }
  while (out.size() >= 2 && std::hypot(out[1].first - out[0].first, out[1].second - out[0].second) <
                                cg::kPivotCornerMaxStepM)
  {
    out.erase(out.begin());
  }
  return out;
}

// Round any corner of `pts` whose turn angle exceeds `max_turn_rad` with a
// circular fillet tangent to both edges (cusp-free in/out), so the WHOLE path
// has no >90° turn. Such corners are intrinsic to the headland rings, which
// inherit acute vertices from the recorded boundary (e.g. a 91° polygon
// corner). The fillet curls toward the INSIDE of the turn (the mowed side); its
// radius is shrunk until the arc stays in-bounds and fits the adjacent edges.
// If a corner can't be rounded in-bounds with an arc of radius >= `min_radius`
// it is left as-is (a sharp corner the robot pivots through is better than a
// fillet too tight for FTC to track — that produces the very loop/hesitation
// we're avoiding; a reportable residual, rare). Operates on the densified
// polyline; arc sampled at `step`.
std::vector<std::pair<double, double>> roundSharpCorners(
    const std::vector<std::pair<double, double>>& pts,
    const std::vector<std::pair<double, double>>& boundary,
    const std::vector<std::vector<std::pair<double, double>>>& holes,
    double max_turn_rad,
    double fillet_r,
    double min_radius,
    double step)
{
  if (pts.size() < 3)
  {
    return pts;
  }
  std::vector<std::pair<double, double>> out;
  out.reserve(pts.size() + 64);
  out.push_back(pts.front());

  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    const auto& A = out.back();  // already-emitted previous point
    const auto& B = pts[i];  // corner vertex
    const auto& C = pts[i + 1];  // next point
    double inx = B.first - A.first, iny = B.second - A.second;
    double outx = C.first - B.first, outy = C.second - B.second;
    const double in_len = std::hypot(inx, iny), out_len = std::hypot(outx, outy);
    if (in_len < 1e-9 || out_len < 1e-9)
    {
      out.push_back(B);
      continue;
    }
    inx /= in_len;
    iny /= in_len;
    outx /= out_len;
    outy /= out_len;
    double c = inx * outx + iny * outy;
    c = std::max(-1.0, std::min(1.0, c));
    const double turn = std::acos(c);  // exterior turn angle [0, π]
    if (turn <= max_turn_rad)
    {
      out.push_back(B);
      continue;
    }

    // Fillet: trim back along each edge by `d`, then connect the two oriented
    // tangent points with a forward arc via the proven sampleDubins (tangent in
    // along the incoming dir, tangent out along the outgoing dir → cusp-free).
    // d = r / tan((π - turn)/2) is the circular-fillet tangent length. Shrink r
    // until the trims fit the edges AND the arc is in-bounds.
    const double in_dir = std::atan2(iny, inx);
    const double out_dir = std::atan2(outy, outx);
    bool done = false;
    for (double r = std::max(fillet_r, min_radius); r >= min_radius - 1e-9; r -= 0.01)
    {
      const double half = (M_PI - turn) / 2.0;
      const double d = r / std::tan(half);
      if (d > 0.49 * in_len || d > 0.49 * out_len)
      {
        continue;  // trim would overrun a neighbouring vertex
      }
      const Pose t0{B.first - inx * d, B.second - iny * d, in_dir};  // arc entry
      const Pose t1{B.first + outx * d, B.second + outy * d, out_dir};  // arc exit
      double arc_len = 0.0;
      auto arc = sampleDubins(t0, t1, r, step, arc_len);  // start-incl, end-excl
      if (arc.empty() || !allInside(arc, boundary) || !clearOfHoles(arc, holes))
      {
        continue;  // off-boundary or through a hole (#333) → try a smaller radius
      }
      // Sanity: a fillet should be short (a single tangent arc, not a loop).
      if (arc_len > 3.0 * r + 0.5)
      {
        continue;  // Dubins found a long way round — try a smaller radius
      }
      out.push_back({t0.x, t0.y});  // straight up to arc entry
      // arc[0] == t0 (already pushed) → skip it to avoid a duplicate.
      for (std::size_t k = 1; k < arc.size(); ++k)
      {
        out.push_back(arc[k]);
      }
      out.push_back({t1.x, t1.y});  // arc exit (Dubins end-exclusive)
      done = true;
      break;
    }
    if (!done)
    {
      out.push_back(B);  // couldn't round in-bounds — leave it (reportable)
    }
  }
  out.push_back(pts.back());
  return out;
}

// Collapse the collinear runs of a densified polyline back to its sparse
// vertices (endpoints always kept). On a headland ring — straight polygon edges
// densified at kDensifyStep — this recovers the original polygon, giving
// roundSharpCorners the TRUE edge lengths. Feeding it the densified polyline
// instead silently disabled every ring fillet: the trim budget (0.49 × in_len)
// with in_len = one 0.10 m densify step caps the fillet radius at ~0.03 m,
// below the min_turning_radius floor, so every corner was "left sharp".
std::vector<std::pair<double, double>> sparsifyCollinear(
    const std::vector<std::pair<double, double>>& pts, double angle_eps_rad)
{
  if (pts.size() < 3)
  {
    return pts;
  }
  std::vector<std::pair<double, double>> out;
  out.reserve(64);
  out.push_back(pts.front());
  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    const double ax = pts[i].first - out.back().first;
    const double ay = pts[i].second - out.back().second;
    const double bx = pts[i + 1].first - pts[i].first;
    const double by = pts[i + 1].second - pts[i].second;
    const double na = std::hypot(ax, ay), nb = std::hypot(bx, by);
    if (na < 1e-9 || nb < 1e-9)
    {
      continue;
    }
    double c = (ax * bx + ay * by) / (na * nb);
    c = std::max(-1.0, std::min(1.0, c));
    if (std::acos(c) > angle_eps_rad)
    {
      out.push_back(pts[i]);  // a real vertex — the heading changes here
    }
  }
  out.push_back(pts.back());
  return out;
}

// Densify the straight gaps of `pts` so consecutive points sit at most `step`
// apart (fillet arc samples are already dense and pass through unchanged).
std::vector<std::pair<double, double>> densifyPolyline(
    const std::vector<std::pair<double, double>>& pts, double step)
{
  std::vector<std::pair<double, double>> out;
  if (pts.empty())
  {
    return out;
  }
  out.reserve(pts.size() * 4);
  for (std::size_t i = 0; i + 1 < pts.size(); ++i)
  {
    densifySegmentStep(pts[i].first, pts[i].second, pts[i + 1].first, pts[i + 1].second, step, out);
  }
  out.push_back(pts.back());
  return out;
}

}  // namespace

// True iff every cell is a valid OGR/GEOS polygon (holes inside the shell, not
// overlapping each other). F2C's clip operations assume validity.
static bool cellsAreValid(const f2c::types::Cells& cells)
{
  for (std::size_t i = 0; i < cells.size(); ++i)
  {
    const auto cell = cells.getGeometry(i);
    if (cell.get() == nullptr || !cell.get()->IsValid())
    {
      return false;
    }
  }
  return true;
}

// Rebuild every cell as its shell MINUS each of its holes, one difference at a
// time: overlapping holes merge, a hole crossing the shell becomes a notch (or
// splits the cell). A plain zero-width buffer is NOT a safe repair here: on a
// polygon whose holes overlap it keeps the overlap as an island (inside two
// holes = outside neither), and swaths were planned inside the obstacles.
static f2c::types::Cells shellMinusHoles(const f2c::types::Cells& cells)
{
  f2c::types::Cells out;
  for (std::size_t i = 0; i < cells.size(); ++i)
  {
    const auto cell = cells.getGeometry(i);
    const OGRPolygon* poly = cell.get();
    if (poly == nullptr || poly->getExteriorRing() == nullptr)
    {
      continue;
    }
    OGRPolygon shell;
    shell.addRing(poly->getExteriorRing());
    std::unique_ptr<OGRGeometry> area(shell.Buffer(0.0));
    if (!area)
    {
      continue;
    }
    for (int r = 0; r < poly->getNumInteriorRings(); ++r)
    {
      OGRPolygon hole;
      hole.addRing(poly->getInteriorRing(r));
      std::unique_ptr<OGRGeometry> valid_hole(hole.Buffer(0.0));
      if (!valid_hole)
      {
        continue;
      }
      std::unique_ptr<OGRGeometry> rest(area->Difference(valid_hole.get()));
      if (rest)
      {
        area = std::move(rest);
      }
    }
    const auto type = wkbFlatten(area->getGeometryType());
    if (type == wkbPolygon)
    {
      out.addGeometry(f2c::types::Cell(area.get()));
    }
    else if (type == wkbMultiPolygon || type == wkbGeometryCollection)
    {
      for (const auto* part : *area->toGeometryCollection())
      {
        if (wkbFlatten(part->getGeometryType()) == wkbPolygon)
        {
          out.addGeometry(f2c::types::Cell(part));
        }
      }
    }
  }
  return out;
}

// Expand a cell's exterior ring outward by `margin`, preserving its holes.
// Used to place the outermost headland ring ON the recorded boundary (the
// perimeter the operator drove) instead of the op_width/2 inside it that
// generateHeadlandSwaths would otherwise produce. Holes (drawn obstacles) are
// carried over unchanged — the outer buffer does not move them.
static f2c::types::Cell expandCellOutward(const f2c::types::Cell& in, double margin)
{
  if (in.size() == 0)
  {
    return in;
  }
  f2c::types::Cell out(bufferRingOutward(in.getGeometry(0), margin));
  for (std::size_t r = 1; r < in.size(); ++r)  // ring 0 = exterior, 1.. = holes
  {
    out.addRing(in.getGeometry(r));
  }
  return out;
}

std::optional<double> longestValidSwathAngle(const f2c::types::Swaths& swaths)
{
  std::optional<double> angle;
  double longest = 1e-9;
  for (std::size_t i = 0; i < swaths.size(); ++i)
  {
    const auto line = swaths[i].getPath();
    if (line.size() < 2)
      continue;
    const auto a = line.getGeometry(0);
    const auto b = line.getGeometry(line.size() - 1);
    const double dx = b.getX() - a.getX(), dy = b.getY() - a.getY();
    const double length = std::hypot(dx, dy);
    if (std::isfinite(dx) && std::isfinite(dy) && std::isfinite(length) && length > longest)
    {
      longest = length;
      double heading = std::fmod(std::atan2(dy, dx), M_PI);
      angle = heading < 0.0 ? heading + M_PI : heading;
    }
  }
  return angle;
}

// Swaths at `angle` with the lane spacing spread EVENLY over the cell's extent
// across the swath direction.
//
// BruteForce places the first lane op_width/2 inside one bbox edge and then steps
// by a fixed op_width while the lane still FITS: it places floor(extent / op_width)
// lanes (a lane whose outer edge would pass the far side is not generated at all —
// measured on a field plan: extent/op = 10.97 gave 10 lanes, not 11). Whatever is
// left over, anywhere from 0 up to a whole op_width, piles up as ONE unplanned strip
// on the far side, which the robot then never mows. When that remainder is more than
// kSwathRemainderTolM, use n = floor(extent / op_width) + 1 lanes at extent / n
// spacing instead: both edges get a lane op_width/2 (or less) from the edge,
// neighbouring lanes overlap slightly more than planned, and no strip is left. The
// result is symmetric in the edge it starts from, so `angle` and `angle + π` give
// the same lanes. Deterministic for a fixed cell + angle.
static f2c::types::Swaths generateEvenSwaths(f2c::sg::BruteForce& bf,
                                             double angle,
                                             double op_width,
                                             const f2c::types::Cell& cell)
{
  const auto ring = cell.getGeometry(0);  // exterior
  double lo = std::numeric_limits<double>::infinity();
  double hi = -std::numeric_limits<double>::infinity();
  const double nx = -std::sin(angle), ny = std::cos(angle);
  for (std::size_t i = 0; i < ring.size(); ++i)
  {
    const auto p = ring.getGeometry(i);
    const double d = p.getX() * nx + p.getY() * ny;
    lo = std::min(lo, d);
    hi = std::max(hi, d);
  }
  const double extent = hi - lo;
  if (op_width > 1e-6 && std::isfinite(extent) && extent > op_width)
  {
    // Lanes F2C places: centres at 0.5·w, 1.5·w, … while the lane's outer edge still
    // fits inside the extent, i.e. floor(extent / w). The epsilon keeps an extent that
    // is a whole number of lanes (to float noise) from reading as one lane short.
    const double lanes = std::floor(extent / op_width + 1e-9);
    const double remainder = extent - lanes * op_width;  // uncovered far-side strip
    if (remainder > kSwathRemainderTolM)
    {
      const double n = lanes + 1.0;
      // The spacing is nudged 1e-6 under extent / n so that F2C's own floor(extent /
      // spacing) cannot round down to n - 1 lanes (extent / (extent / n) can come out as
      // n - epsilon); the lanes still span the full width to 1e-7 m.
      auto even = bf.generateSwaths(angle, extent / n * (1.0 - 1e-6), cell);
      if (even.size() > 0)
      {
        return even;
      }
    }
  }
  return bf.generateSwaths(angle, op_width, cell);
}

RingClosure pickRingClosure(const std::vector<std::pair<double, double>>& open_loop,
                            const std::optional<std::pair<double, double>>& hint,
                            double min_turn_radius)
{
  RingClosure out;
  const std::size_t n = open_loop.size();
  if (n == 0)
  {
    return out;
  }
  if (n == 1)
  {
    out.point = open_loop.front();
    return out;
  }
  const auto side_end = [&](std::size_t j) -> const std::pair<double, double>&
  {
    return open_loop[(j + 1) % n];
  };

  if (!hint)
  {
    // Historical choice, bit-for-bit: the midpoint of the longest side, first one on a tie.
    double longest = -1.0;
    for (std::size_t j = 0; j < n; ++j)
    {
      const auto& a = open_loop[j];
      const auto& b = side_end(j);
      const double len = std::hypot(b.first - a.first, b.second - a.second);
      if (len > longest)
      {
        longest = len;
        out.edge = j;
      }
    }
    out.point = {(open_loop[out.edge].first + side_end(out.edge).first) * 0.5,
                 (open_loop[out.edge].second + side_end(out.edge).second) * 0.5};
    return out;
  }

  // A closure may only sit on a REAL side. F2C's outermost ring has rounded corners, which
  // arrive as chains of tiny sides: a hint at a corner would otherwise pick one of those, and
  // the ring would close ON the corner — exactly what this function exists to avoid. So sides
  // shorter than min_side are skipped (the longest side always qualifies). A loop made only of
  // short sides (a smooth, hand-drawn curve) keeps every side that is at least half the longest.
  double longest = 0.0;
  for (std::size_t j = 0; j < n; ++j)
  {
    const auto& a = open_loop[j];
    const auto& b = side_end(j);
    longest = std::max(longest, std::hypot(b.first - a.first, b.second - a.second));
  }
  const double min_side = std::min(1.0, 0.5 * longest);

  // The nearest such side (distance to the SEGMENT, lowest index on a tie).
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t j = 0; j < n; ++j)
  {
    const auto& a = open_loop[j];
    const auto& b = side_end(j);
    if (std::hypot(b.first - a.first, b.second - a.second) < min_side)
    {
      continue;
    }
    const double dx = b.first - a.first;
    const double dy = b.second - a.second;
    const double len2 = dx * dx + dy * dy;
    const double t =
        len2 < 1e-12
            ? 0.0
            : std::clamp(((hint->first - a.first) * dx + (hint->second - a.second) * dy) / len2,
                         0.0,
                         1.0);
    const double d =
        std::hypot(hint->first - (a.first + t * dx), hint->second - (a.second + t * dy));
    if (d < best - 1e-12)
    {
      best = d;
      out.edge = j;
    }
  }

  const auto& a = open_loop[out.edge];
  const auto& b = side_end(out.edge);
  const double len = std::hypot(b.first - a.first, b.second - a.second);
  if (len < 1e-9)
  {
    out.point = a;
    return out;
  }
  const double ux = (b.first - a.first) / len;
  const double uy = (b.second - a.second) / len;
  // Keep clear of both ends so the closure stays on the straight part, away from the
  // corner fillets (which round with up to 2 * min_turn_radius). A side too short to
  // leave that room closes at its midpoint.
  const double keep_off = std::min(0.5 * len, std::max(0.5, 2.5 * min_turn_radius));
  const double lo = keep_off;
  const double hi = len - keep_off;
  const double along = (hint->first - a.first) * ux + (hint->second - a.second) * uy;
  const double t_along = (lo >= hi) ? 0.5 * len : std::clamp(along, lo, hi);
  out.point = {a.first + ux * t_along, a.second + uy * t_along};
  return out;
}

BoustrophedonPlan planBoustrophedon(const f2c::types::Cell& field_cell,
                                    double op_width,
                                    double headland_width,
                                    int num_headland_passes_override,
                                    double chassis_safety_inset,
                                    double mow_angle_rad,
                                    double min_swath_length,
                                    int ring_direction,
                                    double min_turn_radius,
                                    bool perpendicular,
                                    int connector_max_headland_passes,
                                    const std::optional<std::pair<double, double>>& start_hint)
{
  BoustrophedonPlan plan;
  // Polygon area the planned-coverage fraction is taken over (the operator's
  // authorised area, before any inset). Instrumentation only.
  plan.diagnostics.field_area = field_cell.area();

  // Per-stage wall-clock instrumentation (surfaced as a diagnostics note the
  // server logs). Lets the maintainer see, on the Pi4, which F2C stage dominates
  // a plan — expected: the AUTO swath-angle sweep. Pure accounting.
  using Clock = std::chrono::steady_clock;
  auto elapsedMs = [](Clock::time_point t0)
  {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  };
  double t_headland_ms = 0.0;
  double t_mainland_ms = 0.0;
  double t_swaths_ms = 0.0;
  int swath_angle_candidates = 1;  // 1 = fixed/longest-edge; >1 = exhaustive sweep
  double mainland_area_m2 = 0.0;

  // Raw operator boundary as (x, y) pairs — the ring-corner fillet's in-bounds
  // fallback when no chassis-safety inset was applied (plan.safe_boundary empty).
  std::vector<std::pair<double, double>> field_outer_pts;
  if (field_cell.size() > 0)
  {
    const auto outer_ring = field_cell.getGeometry(0);
    field_outer_pts.reserve(outer_ring.size());
    for (std::size_t i = 0; i < outer_ring.size(); ++i)
    {
      const auto p = outer_ring.getGeometry(i);
      field_outer_pts.emplace_back(p.getX(), p.getY());
    }
  }

  f2c::hg::ConstHL hl;
  f2c::types::Cells cells;
  cells.addGeometry(field_cell);

  // (0) Headland ring count — resolved FIRST because the boundary offset (1),
  // the clearance ring (below) and the mainland (3) all depend on whether there
  // is a ring 0 at all. Three-way contract on num_headland_passes_override:
  //   < 0  NONE   — no perimeter rings; the serpentine swaths mow straight to
  //                 the boundary (issue #429). Every ring-dependent stage below
  //                 is skipped.
  //   == 0 AUTO   — ceil(headland_width / op_width), floored at 1 (unchanged).
  //   > 0  FORCED — exactly that many rings (unchanged).
  const int n_rings =
      (num_headland_passes_override < 0)
          ? 0
          : ((num_headland_passes_override > 0)
                 ? num_headland_passes_override
                 : std::max(1, static_cast<int>(std::ceil(headland_width / op_width - 1e-9))));
  plan.n_headland_passes = n_rings;
  if (n_rings == 0)
  {
    plan.diagnostics.notes.push_back(
        "headland rings disabled (num_headland_passes < 0): swaths mow to the boundary");
  }

  // (1) Boundary offset so the OUTERMOST DRIVEN PASS's centerline sits
  // `chassis_safety_inset` inside the recorded line. Shift the planning field by
  // (chassis_safety_inset - op_width/2):
  //   > 0  shrink INWARD  — operator asked to keep the chassis further inside
  //                         (chassis_safety_inset > op_width/2);
  //   < 0  expand OUTWARD — inset 0: the outermost pass rides ON the recorded
  //                         line, so the blade mows to the edge and the chassis
  //                         straddles the boundary. The half-chassis-width lethal
  //                         band the keepout mask places outside the line is the
  //                         safety stop, replacing the old inset floor.
  //
  // The −op_width/2 term is NOT a ring-specific correction: upstream F2C v3
  // (pinned @884d895) places the outermost pass op_width/2 inside the planning
  // cell in BOTH generators — ConstHL::generateHeadlandSwaths uses
  // `field.buffer(-swath_width * (i + 0.5))` (ring 0 at op_width/2) and
  // SwathGeneratorBase::getSwathsDistribution uses `sx[0] = 0.5 * cov_width`
  // measured from the rotated cell's bbox edge (outermost swath at op_width/2).
  // Same convention, two implementations. So the term applies with rings OFF
  // too; dropping it there does not remove a correction, it inserts an
  // op_width/2 OUTWARD bias into every swath — at the then-shipped defaults
  // (inset 0.20, op_width 0.16) that made "no rings" leave a 0.20 m uncut border
  // versus 0.12 m for 2 rings, i.e. the feature that promises to mow closer to
  // the edge mowed 8 cm FURTHER from it.
  //
  // The zero-rings branch is floored at 0 (never an outward expansion). With
  // rings, ring 0 riding ON the recorded line is deliberate and the swath ENDS
  // still sit n_rings*op_width inside. With no rings the ends ARE the outermost
  // geometry and the robot meets the boundary head-on there, a direction
  // chassis_safety_inset has never modelled (the server's footprint check offsets
  // ±robot_width/2 PERPENDICULAR to heading), so keep them on or inside the
  // recorded line.
  const double field_offset = (n_rings > 0) ? (chassis_safety_inset - 0.5 * op_width)
                                            : std::max(0.0, chassis_safety_inset - 0.5 * op_width);
  f2c::types::Cells safe_cells = cells;
  if (field_offset > 1e-3)
  {
    safe_cells = hl.generateHeadlands(cells, field_offset);
    if (safe_cells.size() == 0 || safe_cells.area() < 1e-6)
    {
      return plan;  // inset consumed the field
    }
  }
  else if (field_offset < -1e-3)
  {
    // Expand the field exterior outward so ring 0 lands on the recorded line.
    f2c::types::Cells expanded;
    expanded.addGeometry(expandCellOutward(field_cell, -field_offset));
    if (expanded.size() > 0 && expanded.area() > 1e-6)
    {
      safe_cells = expanded;
    }
    // else: buffer degeneracy — fall back to the raw field (safe_cells = cells).
  }
  else if (n_rings == 0 && !cellsAreValid(safe_cells))
  {
    // Rings off and no inset: safe_cells IS the goal cell, and this is the one
    // branch where no GEOS buffer ever normalises it (every other branch goes
    // through generateHeadlands). Drawn obstacles grown by obstacle_margin can
    // overlap each other or cross the recorded line; that polygon is INVALID
    // and F2C's swath clip then throws a GEOS TopologyException and crashes
    // (field geometry 2026-09-21 with num_headland_passes -1: two obstacles
    // 0.63 m apart grown by 0.389 m, one 0.22 m from the line). Repair it as the
    // shell minus the union of its holes — only when invalid, so a valid field
    // still plans byte-identically (issue #429: a needless buffer round-trip
    // re-nodes the polygon).
    safe_cells = shellMinusHoles(safe_cells);
    plan.diagnostics.notes.push_back(
        "field polygon invalid (grown obstacles overlap each other or the boundary): "
        "repaired as the boundary minus the union of the obstacles");
    if (safe_cells.size() == 0 || safe_cells.area() < 1e-6)
    {
      return plan;
    }
  }

  // Expose the planning outer ring so the continuous-path connectors/fillets are
  // bounded by the SAME polygon the rings/swaths are planned against (not the raw
  // operator boundary). If an inset split the field, take the largest cell's
  // exterior ring (the dominant drivable region). Populated UNCONDITIONALLY from
  // safe_cells: when no boundary offset was applied (chassis_safety_inset ==
  // op_width/2 exactly → field_offset == 0, or a buffer degeneracy) safe_cells ==
  // the raw field, so safe_boundary becomes the raw operator ring — which is
  // still the correct drivable envelope (the outermost ring rides on it). Gating
  // this on boundary_offset_applied left safe_boundary EMPTY in the field_offset
  // ≈ 0 case, and buildContinuousSubPaths then validated every turn-around
  // connector against an empty polygon (allInside always false) → a split at
  // every segment → gross sub-path over-fragmentation.
  {
    std::size_t largest = 0;
    double largest_area = -1.0;
    for (std::size_t i = 0; i < safe_cells.size(); ++i)
    {
      const double a = safe_cells.getGeometry(i).area();
      if (a > largest_area)
      {
        largest_area = a;
        largest = i;
      }
    }
    if (safe_cells.size() > 0)
    {
      const auto safe_ring = safe_cells.getGeometry(largest).getGeometry(0);  // exterior
      plan.safe_boundary.reserve(safe_ring.size());
      for (std::size_t i = 0; i < safe_ring.size(); ++i)
      {
        const auto p = safe_ring.getGeometry(i);
        plan.safe_boundary.emplace_back(p.getX(), p.getY());
      }
    }
  }

  // Turn-around connectors/fillets are bounded by the OUTERMOST RING's centerline,
  // NOT safe_boundary. generateHeadlandSwaths places ring 0 op_width/2 inside the
  // planning field (safe_cells), so eroding safe_cells inward by op_width/2 yields
  // exactly that centerline (== the recorded line eroded by chassis_safety_inset).
  // allInside() only tests the path CENTERLINE, so validating connectors against
  // safe_boundary let a turn arc's centerline ride op_width/2 further out than any
  // ring/swath, pushing the swept chassis/blade footprint that far past the
  // operator boundary (excursion grew with the turn radius). Bounding connectors
  // to this ring instead caps a turn's footprint at the perimeter ring the robot
  // already drives. op_width/2 (NOT robot_width/2): eroding by the chassis
  // half-width would keep turns robot_width/2 − op_width/2 TIGHTER than the
  // perimeter ring → edge turn-arounds forced below min_turning_radius → straight
  // fallback → sub-path fragmentation. On degeneracy (tiny field) leave it empty;
  // the caller falls back to safe_boundary (the pre-fix, looser bound).
  //
  // WITH NO RINGS (n_rings == 0, issue #429) there is no ring 0 to erode to: the
  // outermost driven geometry is the swath ENDS, which sit exactly ON safe_cells'
  // exterior. The clearance ring is then safe_cells EXACTLY — no buffer call at
  // all (which also dodges the GEOS buffer(-0.0) re-noding hazard the mainland
  // stage documents below). The invariant is the same in both branches: a
  // connector centerline may go no further out than the OUTERMOST DRIVEN PASS.
  // On-edge swath ends are handled by allInside()'s kOnEdgeTolM tolerance, NOT
  // by expanding this ring outward — see the constant.
  //
  // Ring i's centerline sits (i + 0.5) * op_width inside safe_cells (ring 0
  // outermost, n_rings == 0 → safe_cells itself regardless of `ring_index`).
  // Shared by both boundaries below so they can only ever disagree in WHICH
  // ring they erode to, never in HOW.
  auto ringCenterlineBoundary = [&](int ring_index)
  {
    std::vector<std::pair<double, double>> out;
    f2c::types::Cells cells_at = safe_cells;
    if (n_rings > 0)
    {
      cells_at = hl.generateHeadlands(safe_cells, (ring_index + 0.5) * op_width);
    }
    if (cells_at.size() > 0 && cells_at.area() > 1e-6)
    {
      std::size_t largest = 0;
      double largest_area = -1.0;
      for (std::size_t i = 0; i < cells_at.size(); ++i)
      {
        const double a = cells_at.getGeometry(i).area();
        if (a > largest_area)
        {
          largest_area = a;
          largest = i;
        }
      }
      const auto ring = cells_at.getGeometry(largest).getGeometry(0);  // exterior
      out.reserve(ring.size());
      for (std::size_t i = 0; i < ring.size(); ++i)
      {
        const auto p = ring.getGeometry(i);
        out.emplace_back(p.getX(), p.getY());
      }
    }
    return out;
  };

  // ALWAYS ring 0 (or safe_cells with no rings) — never moved by
  // connector_max_headland_passes. See the field doc: this is what the #388
  // clamp, the server's verify, and every ring-involving join stay bound to.
  plan.connector_clearance_boundary = ringCenterlineBoundary(0);

  // swath_turn_envelope (issue #497): populated ONLY when the limit actually
  // restricts something — a limit of 0/negative (unlimited) or >= n_rings is a
  // no-op, and with no rings there is no deeper ring to bound to either. Left
  // empty in every other case so the caller (buildContinuousSubPaths) falls
  // back to connector_clearance_boundary for every join, identical to
  // pre-#497 behaviour.
  if (n_rings > 0)
  {
    const int clamped_limit = std::clamp(connector_max_headland_passes, 0, n_rings);
    if (clamped_limit > 0 && clamped_limit < n_rings)
    {
      plan.swath_turn_envelope = ringCenterlineBoundary(n_rings - clamped_limit);
    }
  }

  // Expose the interior hole rings the continuous-path connectors/fillets must
  // stay OUT of (issue #333). Use safe_cells' holes: when an inward inset was
  // applied they are grown outward by the inset (so the blade clears an obstacle
  // by the same margin it clears the outer boundary); when the field was
  // expanded outward (ring on the line) or left unchanged, safe_cells carries
  // the raw operator holes. Collect across every cell (the inset may have split
  // the field) so a connector near any hole is caught.
  {
    const f2c::types::Cells& hole_src = safe_cells;
    for (std::size_t i = 0; i < hole_src.size(); ++i)
    {
      const auto& cell = hole_src.getGeometry(i);
      for (std::size_t r = 1; r < cell.size(); ++r)  // ring 0 = exterior, 1.. = holes
      {
        const auto ring = cell.getGeometry(r);
        std::vector<std::pair<double, double>> hole;
        hole.reserve(ring.size());
        for (std::size_t j = 0; j < ring.size(); ++j)
        {
          const auto p = ring.getGeometry(j);
          hole.emplace_back(p.getX(), p.getY());
        }
        if (hole.size() >= 3)
        {
          plan.safe_holes.push_back(std::move(hole));
        }
      }
    }
  }

  // (2) Headland rings — n concentric mowed loops spaced op_width, outermost
  // first. Ring i's centerline sits (i + 0.5) * op_width inside the safe
  // boundary, so n rings cut the band [0, n*op_width].
  // Drop degenerate micro-loops (a near-consumed tiny field can yield a
  // centimetre-scale innermost ring that isn't worth driving).
  constexpr double kMinRingPerimeter = 1.0;  // m
  double ring_strip_area = 0.0;  // Σ perimeter·op_width of KEPT rings (for the fraction)
  // Skipped ENTIRELY when the ring stage is disabled (n_rings == 0, issue
  // #429): generateHeadlandSwaths(…, 0, dir_out2in=true) happens to return {}
  // at the pinned F2C commit, but entering ringPassToLoops/fillet/ring_strip
  // accounting with nothing to do is noise, not intent.
  if (n_rings > 0)
  {
    const auto t_headland0 = Clock::now();
    const auto headland_passes =
        hl.generateHeadlandSwaths(safe_cells, op_width, n_rings, /*dir_out2in=*/true);
    t_headland_ms = elapsedMs(t_headland0);
    bool first_pass = true;
    std::size_t first_pass_rings = 0;  // loops kept from the OUTERMOST pass
    for (const auto& pass_lines : headland_passes)
    {
      auto loops = ringPassToLoops(pass_lines);
      for (auto& loop : loops)
      {
        double perim = 0.0;
        for (std::size_t i = 1; i < loop.size(); ++i)
        {
          perim +=
              std::hypot(loop[i].first - loop[i - 1].first, loop[i].second - loop[i - 1].second);
        }
        if (perim < kMinRingPerimeter)
        {
          plan.diagnostics.drops.push_back(fmtDrop("ring perim=", perim, "<", kMinRingPerimeter));
          continue;
        }
        // Perimeter/headland travel winding (#335): flip the loop so a side-mounted
        // blade stays on the cut side. Shoelace signed area > 0 = CCW, < 0 = CW.
        // ring_direction: 1 = clockwise, 2 = counter-clockwise, 0 = leave as F2C
        // emitted it.
        if (ring_direction != 0)
        {
          double area2 = 0.0;
          for (std::size_t i = 0; i + 1 < loop.size(); ++i)
          {
            area2 += loop[i].first * loop[i + 1].second - loop[i + 1].first * loop[i].second;
          }
          const bool is_ccw = area2 > 0.0;
          const bool want_ccw = (ring_direction == 2);
          if (is_ccw != want_ccw)
          {
            std::reverse(loop.begin(), loop.end());
          }
        }

        // Ring corner smoothing (field report 2026-07: "the robot stalls after
        // every headland ring, oscillating, before moving on"). Two fixes here:
        //
        // 1. START THE LOOP MID-LONGEST-EDGE. F2C closes every concentric ring at
        //    the same polygon corner, so the loop closure sat ON a corner: the
        //    closure vertex is the one corner roundSharpCorners can never fillet
        //    (it only processes interior vertices), and the ring→ring junction
        //    then demanded the full corner turn across a ~op_width gap (measured
        //    112° on the field replica) — a near-cusp FTC fights the drivetrain
        //    deadband through. Rotating the closure onto the middle of the
        //    longest straight edge makes the closure a straight-line point and
        //    stacks the junctions of consecutive rings on parallel edges (a
        //    tangent ~op_width sideways shift).
        //
        // 2. FILLET AT SPARSE (true-edge-length) LEVEL. roundSharpCorners' trim
        //    budget is 0.49 × the incoming edge length; fed the DENSIFIED loop,
        //    in_len is one 0.10 m densify step, capping the fillet radius at
        //    ~0.03 m — below the min_turn_radius floor — so every ring-corner
        //    fillet silently failed and all polygon corners stayed sharp.
        //    Collapse the collinear runs first, fillet on real edges, then
        //    re-densify.
        {
          auto sparse = sparsifyCollinear(loop, 0.01 /* rad — straight-run merge */);
          if (sparse.size() >= 5)
          {
            sparse.pop_back();  // open the closed loop for rotation
            // The closure side: the longest one (the historical choice) or, with a
            // start hint, the one nearest it — see pickRingClosure.
            const RingClosure closure = pickRingClosure(sparse, start_hint, min_turn_radius);
            const std::size_t le = closure.edge;
            // New start = the longest edge's midpoint. Rotate so the loop begins
            // at the vertex AFTER the edge (v_{le+1}) and stitch the midpoint on
            // both ends: mid → v_{le+1} → … → v_le → mid. (Rotating to v_le
            // instead would make the path step BACKWARD from mid to v_le — a
            // 180° reversal baked into the ring.)
            const std::pair<double, double> mid = closure.point;
            std::rotate(sparse.begin(),
                        sparse.begin() + static_cast<std::ptrdiff_t>((le + 1) % sparse.size()),
                        sparse.end());
            sparse.insert(sparse.begin(), mid);
            sparse.push_back(mid);
            // Fillet every corner sharper than ~30° with a forward arc (floored at
            // min_turn_radius so FTC can track it); corners that cannot be
            // rounded in-bounds are left sharp, as before.
            const auto& fillet_boundary =
                plan.safe_boundary.size() >= 3 ? plan.safe_boundary : field_outer_pts;
            constexpr double kRingCornerThreshold = 30.0 * M_PI / 180.0;
            auto rounded = roundSharpCorners(sparse,
                                             fillet_boundary,
                                             plan.safe_holes,
                                             kRingCornerThreshold,
                                             std::max(2.0 * min_turn_radius, min_turn_radius),
                                             std::max(0.02, min_turn_radius),
                                             0.03);
            loop = densifyPolyline(rounded, kDensifyStep);
          }
        }

        ring_strip_area += perim * op_width;
        plan.rings.push_back(std::move(loop));
      }
      if (first_pass)
      {
        first_pass_rings = plan.rings.size();
        first_pass = false;
      }
    }

    // The operator chose where the route starts: it must start on the OUTER
    // perimeter. A field with holes yields several loops in the outermost pass (the
    // perimeter plus one around each obstacle) and the route begins with
    // plan.rings[0], so put the largest loop of that pass — the perimeter — first.
    // Only with a hint: without one the order is exactly what F2C produced.
    if (start_hint && first_pass_rings > 1)
    {
      const auto area_of = [](const std::vector<std::pair<double, double>>& loop)
      {
        double twice = 0.0;
        for (std::size_t i = 0; i + 1 < loop.size(); ++i)
        {
          twice += loop[i].first * loop[i + 1].second - loop[i + 1].first * loop[i].second;
        }
        return std::abs(twice);
      };
      std::size_t outer = 0;
      double outer_area = area_of(plan.rings[0]);
      for (std::size_t i = 1; i < first_pass_rings; ++i)
      {
        const double a = area_of(plan.rings[i]);
        if (a > outer_area)
        {
          outer_area = a;
          outer = i;
        }
      }
      if (outer != 0)
      {
        std::rotate(plan.rings.begin(),
                    plan.rings.begin() + static_cast<std::ptrdiff_t>(outer),
                    plan.rings.begin() + static_cast<std::ptrdiff_t>(outer) + 1);
      }
    }
  }

  // (3) Mainland: what's left inside the rings' cut band. May be empty on a
  // small field (rings-only plan — still valid coverage). With no rings the
  // mainland IS the (already inset) planning field — do NOT call
  // generateHeadlands(safe_cells, 0.0): upstream that is Cells::buffer(-0.0), a
  // full OGR/GEOS buffer round-trip that re-nodes the polygon and can drop
  // marginal parts. A needless, non-deterministic no-op (issue #429).
  const auto t_mainland0 = Clock::now();
  f2c::types::Cells mainland =
      (n_rings > 0) ? hl.generateHeadlands(safe_cells, n_rings * op_width) : safe_cells;
  t_mainland_ms = elapsedMs(t_mainland0);
  mainland_area_m2 = mainland.area();
  if (mainland.size() == 0 || mainland.area() < 1e-6)
  {
    // Rings-only is a valid plan — still report the planned fraction it covers.
    plan.diagnostics.planned_area = ring_strip_area;
    plan.diagnostics.planned_fraction =
        (plan.diagnostics.field_area > 1e-9) ? ring_strip_area / plan.diagnostics.field_area : 0.0;
    return plan;
  }

  // (4) Straight swaths per mainland cell. BruteForce clips each sweep line
  // against the cell and makes every disjoint clip its OWN swath, so concave
  // boundaries and interior holes need no decomposition. BoustrophedonOrder
  // sorts the swaths along the sweep axis and alternates their driving
  // direction (serpentine). A fixed mow angle keeps the plan deterministic
  // across re-plans (swath-index resume relies on this); auto (< 0) uses the
  // swath-count-minimising angle, which is equally deterministic for a fixed
  // polygon.
  f2c::sg::BruteForce bf;
  // Coarsen the AUTO best-angle sweep from F2C's 1° default to kAutoAngleStepRad
  // (5°) — ~5× fewer candidate angles at a negligible plan-quality change (see
  // the constant). No effect on fixed-angle plans (they skip the sweep).
  bf.setStepAngle(kAutoAngleStepRad);
  f2c::obj::NSwath n_swath_obj;
  f2c::rp::BoustrophedonOrder order;
  double swath_strip_area = 0.0;  // Σ length·op_width of KEPT swaths (for the fraction)
  const auto t_swaths0 = Clock::now();
  for (std::size_t i = 0; i < mainland.size(); ++i)
  {
    const auto cell = mainland.getGeometry(i);
    if (cell.area() < 1e-6)
    {
      plan.diagnostics.drops.push_back(fmtDrop("mainland cell area=", cell.area(), "<", 1e-6));
      continue;
    }
    // Resolve the swath angle. Fixed angle (>= 0) is used as-is. AUTO (< 0)
    // normally runs the exhaustive best-angle search, but on a large cell that
    // search is too slow (see kAutoAngleMaxAreaM2) — fall back to the cheap
    // longest-edge angle so big fields still plan within the action timeout.
    double cell_angle = mow_angle_rad;
    if (cell_angle < 0.0 && cell.area() > kAutoAngleMaxAreaM2)
    {
      cell_angle = longestEdgeAngle(cell);
      char buf[160];
      std::snprintf(buf,
                    sizeof(buf),
                    "auto-angle: cell area=%.1f m² > %.1f → longest-edge angle %.1f° "
                    "(skipped exhaustive search)",
                    cell.area(),
                    kAutoAngleMaxAreaM2,
                    cell_angle * 180.0 / M_PI);
      plan.diagnostics.notes.push_back(std::string(buf));
    }
    if (cell_angle < 0.0)
    {
      // Exhaustive sweep actually runs on this cell — record its candidate count
      // (2π / step) for the timing note.
      swath_angle_candidates =
          std::max(swath_angle_candidates,
                   static_cast<int>(std::lround(2.0 * M_PI / kAutoAngleStepRad)));
    }
    f2c::types::Swaths swaths = (cell_angle >= 0.0)
                                    ? generateEvenSwaths(bf, cell_angle, op_width, cell)
                                    : bf.generateBestSwaths(n_swath_obj, op_width, cell);
    if (swaths.size() == 0)
    {
      continue;
    }
    if (cell_angle < 0.0 && !perpendicular)
    {
      // AUTO: keep the angle the search picked, but spread the lanes evenly
      // across the cell so no too-narrow strip is left unplanned.
      if (const auto best = longestValidSwathAngle(swaths); best && std::isfinite(*best))
      {
        auto even = generateEvenSwaths(bf, *best, op_width, cell);
        if (even.size() > 0)
        {
          swaths = even;
        }
      }
    }
    if (perpendicular)
    {
      // Resolve AUTO normally, then rotate that result. Optimising again at
      // the rotated heading would just undo the cross-hatch selection.
      const auto base =
          cell_angle >= 0.0 ? std::optional<double>(cell_angle) : longestValidSwathAngle(swaths);
      if (!base || !std::isfinite(*base))
      {
        plan.diagnostics.drops.push_back("cross-hatch: no valid Auto swath heading for cell");
        continue;
      }
      double angle = std::fmod(*base + M_PI / 2.0, M_PI);
      if (angle < 0.0)
        angle += M_PI;
      swaths = generateEvenSwaths(bf, angle, op_width, cell);
      if (swaths.size() == 0)
      {
        plan.diagnostics.drops.push_back("cross-hatch: rotated cell has no swaths");
        continue;
      }
    }
    f2c::types::Swaths ordered = order.genSortedSwaths(swaths);
    for (std::size_t s = 0; s < ordered.size(); ++s)
    {
      const auto& sw = ordered[s];
      const auto line = sw.getPath();
      if (line.size() < 2)
      {
        continue;
      }
      const auto p0 = line.getGeometry(0);
      const auto p1 = line.getGeometry(line.size() - 1);
      const double len = std::hypot(p1.getX() - p0.getX(), p1.getY() - p0.getY());
      if (len < min_swath_length)
      {
        plan.diagnostics.drops.push_back(fmtDrop("swath len=", len, "<", min_swath_length));
        continue;  // sliver clip — not worth a pivot
      }
      if (plan.swaths.empty())
      {
        plan.swath_angle_rad = std::atan2(p1.getY() - p0.getY(), p1.getX() - p0.getX());
      }
      swath_strip_area += len * op_width;
      plan.swaths.push_back({{p0.getX(), p0.getY()}, {p1.getX(), p1.getY()}});
    }
  }

  t_swaths_ms = elapsedMs(t_swaths0);

  // Per-stage timing note (server logs it). "swaths" is the dominant stage — the
  // AUTO best-angle sweep over `angles` candidates.
  {
    char buf[200];
    std::snprintf(buf,
                  sizeof(buf),
                  "timing: headland=%.0fms mainland=%.0fms swaths=%.0fms "
                  "(angles=%d, mainland_area=%.1f m²)",
                  t_headland_ms,
                  t_mainland_ms,
                  t_swaths_ms,
                  swath_angle_candidates,
                  mainland_area_m2);
    plan.diagnostics.notes.push_back(std::string(buf));
  }

  // Planned-coverage fraction: strip areas of the kept rings + swaths over the
  // polygon area. Coarse (strips butt rather than overlap; corners double-count
  // slightly), so it is a visibility metric, not a guarantee — but it makes a
  // partially-planned area (slivers/rings/cells silently dropped above) VISIBLE
  // in the server log so the next iteration can decide.
  plan.diagnostics.planned_area = ring_strip_area + swath_strip_area;
  plan.diagnostics.planned_fraction =
      (plan.diagnostics.field_area > 1e-9)
          ? plan.diagnostics.planned_area / plan.diagnostics.field_area
          : 0.0;

  return plan;
}

// ── Continuous cusp-free path (forward turn-around connectors) ───────────────

namespace
{

// How far INSIDE the clearance ring a clamped (previously out-of-bounds) pose is
// placed. The clearance ring is the outermost headland ring's centerline (== the
// recorded line eroded by chassis_safety_inset). A pose that pokes PAST it — the
// only source being the F2C offset mismatch between generateHeadlandSwaths (the
// ring) and generateHeadlands (the clearance boundary), which round convex
// corners differently — is projected back to `kClearanceClampMarginM` inside the
// nearest ring edge. Small enough that the local deformation stays trackable, and
// far below the server's kBoundarySlackM (0.05 m) verify slack so the clamped
// path passes the in-bounds check. See issue #388.
constexpr double kClearanceClampMarginM = 0.02;

// Project a pose that lies OUTSIDE `ring` back to `margin` inside the nearest ring
// edge. Poses already inside are returned unchanged, so applying this to a whole
// path only nudges the out-of-bounds convex-corner pokes and is a no-op for every
// ring/swath/connector pose the planner already kept in-bounds.
std::pair<double, double> clampInsideRing(double x,
                                          double y,
                                          const std::vector<std::pair<double, double>>& ring,
                                          double margin)
{
  // A pose sitting exactly ON the ring is left alone: with the headland stage
  // off the swath ENDS lie on this ring by construction, and pointInRing()
  // resolves such poses inconsistently — clamping them would silently deform
  // half the swath ends inward. Same tolerance allInside() uses (kOnEdgeTolM).
  if (ring.size() < 3 || pointInRing(x, y, ring) || distanceToRing(x, y, ring) <= kOnEdgeTolM)
  {
    return {x, y};
  }
  // Nearest point on the ring boundary.
  const std::size_t n = ring.size();
  double best_d2 = std::numeric_limits<double>::max();
  double px = x, py = y;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = ring[j].first, ay = ring[j].second;
    const double bx = ring[i].first, by = ring[i].second;
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = (len2 > 0.0) ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    const double qx = ax + t * dx, qy = ay + t * dy;
    const double d2 = (x - qx) * (x - qx) + (y - qy) * (y - qy);
    if (d2 < best_d2)
    {
      best_d2 = d2;
      px = qx;
      py = qy;
    }
  }
  // The direction from the exterior pose toward its nearest boundary projection
  // points INTO the polygon; stepping `margin` past the projection lands inside
  // (unless the ring is thinner than the margin there, in which case snap to the
  // boundary rather than risk crossing to the far side).
  double inx = px - x, iny = py - y;
  const double inl = std::hypot(inx, iny);
  if (inl < 1e-9)
  {
    return {px, py};
  }
  inx /= inl;
  iny /= inl;
  const double cx = px + margin * inx, cy = py + margin * iny;
  if (pointInRing(cx, cy, ring))
  {
    return {cx, cy};
  }
  return {px, py};
}

}  // namespace

std::vector<std::pair<double, double>> pivotSharpCorners(
    const std::vector<std::pair<double, double>>& pts,
    double min_turn_rad,
    const PivotJoinLimits& limits,
    std::vector<bool>* is_twin)
{
  if (pts.size() < 3 || limits.sweep_radius <= 0.0)
  {
    return pts;
  }
  const bool track = is_twin != nullptr && is_twin->size() == pts.size();
  std::vector<bool> twin_out;
  if (track)
  {
    twin_out.reserve(pts.size() + 16);
    twin_out.push_back((*is_twin)[0]);
  }
  const auto same = [](const std::pair<double, double>& a, const std::pair<double, double>& b)
  {
    return std::hypot(a.first - b.first, a.second - b.second) < cg::kPivotCornerMaxStepM;
  };
  std::vector<std::pair<double, double>> out;
  out.reserve(pts.size() + 16);
  out.push_back(pts.front());
  for (std::size_t i = 1; i + 1 < pts.size(); ++i)
  {
    out.push_back(pts[i]);
    if (track)
    {
      twin_out.push_back((*is_twin)[i]);
    }
    // Already a corner twin (either half): leave it to the contract.
    if (same(pts[i], pts[i - 1]) || same(pts[i], pts[i + 1]))
    {
      continue;
    }
    const double inx = pts[i].first - pts[i - 1].first;
    const double iny = pts[i].second - pts[i - 1].second;
    const double outx = pts[i + 1].first - pts[i].first;
    const double outy = pts[i + 1].second - pts[i].second;
    const double turn = headingError(std::atan2(iny, inx), std::atan2(outy, outx));
    if (turn > min_turn_rad && pivotSweepFits(pts[i].first, pts[i].second, limits))
    {
      out.push_back(pts[i]);  // bit-exact twin: FTC stops here and rotates in place
      if (track)
      {
        twin_out.push_back(true);
      }
    }
  }
  out.push_back(pts.back());
  if (track)
  {
    twin_out.push_back(is_twin->back());
    *is_twin = std::move(twin_out);
  }
  return out;
}

std::vector<std::vector<std::pair<double, double>>> buildContinuousSubPaths(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step,
    ConnectorStats* stats,
    const std::vector<std::pair<double, double>>& swath_turn_boundary,
    const PivotJoinLimits& pivot_limits,
    bool pin_first_subpath)
{
  // Flatten the plan into ordered drivable segments (densified polylines),
  // rings first (outermost → inner) then the swaths.
  //
  // Ring start alignment (field report 2026-07: "the robot stalls after every
  // headland ring, oscillating, before moving on"). F2C starts every concentric
  // ring at the same polygon corner, so the loop CLOSURE sits on that corner and
  // the ring→ring junction demanded a 45–90° heading change across a ~op_width
  // gap — no trackable Dubins fits, the join degenerates to a near-cusp, and FTC
  // fights the drivetrain deadband pivoting through it (the observed stall +
  // wz oscillation). Rotate each ring loop (they are CLOSED polylines, any
  // vertex is a valid start) so it begins at the vertex nearest the previous
  // ring's end: concentric rings are locally parallel there, so the junction
  // becomes a gentle ~op_width sideways shift the connector joins tangentially.
  // The first ring keeps F2C's own start vertex — this is a ring-GENERATION
  // decision, made before any sub-path splitting, and is unaffected by which
  // sub-path ends up first after orderSubPathsForMinimalTransit reorders the
  // FINISHED sub-paths below (mowglinext#819: transit order can now change,
  // so this is no longer necessarily what TransitToStrip drives to first —
  // it just means one ring somewhere keeps F2C's own un-rotated start).
  // Ring DRIVE-ORDER grouping. F2C's generateHeadlandSwaths emits ring loops PER
  // PASS as [outer, hole, outer, hole, …], so consecutive concentric OUTER rings
  // are interleaved with the field-centre hole rings. Driven in that raw order the
  // path jumps outer-ring → hole-ring → outer-ring every pass, and each jump is a
  // field-crossing relocation adds a long connector or a blade-off transit.
  // Partition into outer-boundary rings FIRST (outermost-first order
  // preserved) then the obstacle-encircling rings, so each group drives
  // contiguously and only ONE relocation separates the two. A ring is an OUTER
  // ring iff it encloses a MAINLAND point (a swath endpoint): the mainland lies
  // inside every nested outer ring and outside every small hole ring. Falls back
  // to the raw order when there are no swaths (rings-only field: no mainland probe,
  // and a fully-consumed field rarely carries hole rings worth reordering). Ring
  // GEOMETRY is untouched — only the order the loops are appended.
  std::vector<std::size_t> ring_order;
  ring_order.reserve(plan.rings.size());
  std::size_t outer_ring_count = plan.rings.size();
  if (!plan.swaths.empty())
  {
    const double probe_x =
        0.5 * (plan.swaths.front().first.first + plan.swaths.front().second.first);
    const double probe_y =
        0.5 * (plan.swaths.front().first.second + plan.swaths.front().second.second);
    std::vector<std::size_t> hole_rings;
    for (std::size_t i = 0; i < plan.rings.size(); ++i)
    {
      if (pointInRing(probe_x, probe_y, plan.rings[i]))
      {
        ring_order.push_back(i);  // outer ring (encloses the mainland)
      }
      else
      {
        hole_rings.push_back(i);  // encircles a hole
      }
    }
    outer_ring_count = ring_order.size();
    ring_order.insert(ring_order.end(), hole_rings.begin(), hole_rings.end());
  }
  else
  {
    for (std::size_t i = 0; i < plan.rings.size(); ++i)
    {
      ring_order.push_back(i);
    }
  }

  std::vector<std::vector<std::pair<double, double>>> segs;
  for (std::size_t pos = 0; pos < ring_order.size(); ++pos)
  {
    // Finish neighbouring obstacle loops before travelling to another obstacle.
    // F2C interleaves obstacles on every headland pass; leaving that order intact
    // creates repeated lawn-crossing connectors inside a single finished path,
    // where the later sub-path transit optimiser cannot remove them.
    if (pos >= outer_ring_count && !segs.empty())
    {
      const auto& end = segs.back().back();
      std::size_t best = pos;
      double best_distance = std::numeric_limits<double>::max();
      for (std::size_t j = pos; j < ring_order.size(); ++j)
      {
        for (const auto& pt : plan.rings[ring_order[j]])
        {
          const double d = std::hypot(pt.first - end.first, pt.second - end.second);
          if (d < best_distance)
          {
            best_distance = d;
            best = j;
          }
        }
      }
      std::swap(ring_order[pos], ring_order[best]);
    }
    const auto& loop_in = plan.rings[ring_order[pos]];
    if (loop_in.size() < 2)
    {
      continue;
    }
    std::vector<std::pair<double, double>> loop = loop_in;
    if (!segs.empty() && loop.size() >= 4)
    {
      const auto& prev = segs.back();
      const auto& prev_end = prev.back();
      // Exit heading of the previous ring (its last step direction).
      const double ex = prev_end.first - prev[prev.size() - 2].first;
      const double ey = prev_end.second - prev[prev.size() - 2].second;
      const double en = std::hypot(ex, ey);
      // Drop the duplicated closure vertex, rotate, then re-close.
      loop.pop_back();
      // Nearest vertex ALONE is not enough: F2C closes every concentric ring at
      // the same polygon corner, so the nearest vertex IS that corner and the
      // junction still demanded the full corner turn (measured 112° — the stall).
      // Require the candidate's OUTGOING heading to align with the previous
      // ring's exit heading (within 45°): concentric rings have a parallel edge
      // there, so the join becomes a tangent ~op_width sideways shift. Fall back
      // to plain nearest if nothing aligns (degenerate tiny ring).
      //
      // Among ALIGNED candidates, nearest-by-distance alone still tends to pick
      // the vertex directly "inward" from prev_end: near-zero forward advance,
      // near-op_width lateral offset. A forward-only Dubins connector cannot
      // just slide sideways between two closely-spaced, near-parallel poses —
      // it has to loop to satisfy the curvature bound, which is exactly the
      // "weird little loop between headland rings" field report. The minimum
      // forward advance for a clean two-arc S-curve merge across a lateral
      // offset `l` at radius `r` is f_min = sqrt(l·(4r − l)) (tangent-circle
      // geometry, valid for 0 <= l <= 4r — derived from the standard symmetric
      // reverse-curve construction: l = 2r(1−cos φ), f = 2r·sin φ for the arc
      // angle φ, eliminate φ). Score a candidate by how far short of f_min its
      // OWN forward advance falls (its "deficit") instead of raw distance, so
      // the chosen entry point is pulled far enough along the ring for a
      // diagonal merge — a candidate that already clears its own deficit is
      // still scored by plain distance (nearest SUFFICIENT point — no reason
      // to overshoot further along the ring than necessary). kInsufficientPenalty
      // separates the two bands (d2 is at most a few thousand m² for any real
      // field) so one min-cost scan implements "prefer any sufficient candidate
      // over every insufficient one, tie-broken as described" without a second
      // pass.
      const double kAlignCos = 0.7071;  // cos(45°)
      constexpr double kInsufficientPenalty = 1.0e6;
      std::size_t best = 0;
      double best_cost = std::numeric_limits<double>::max();
      bool found_aligned = false;
      for (int pass = 0; pass < 2 && !found_aligned; ++pass)
      {
        for (std::size_t j = 0; j < loop.size(); ++j)
        {
          if (pass == 0 && en > 1e-9)
          {
            const auto& nxt = loop[(j + 1) % loop.size()];
            const double ox = nxt.first - loop[j].first;
            const double oy = nxt.second - loop[j].second;
            const double on = std::hypot(ox, oy);
            if (on < 1e-9 || (ex * ox + ey * oy) / (en * on) < kAlignCos)
            {
              continue;  // outgoing heading not parallel to the previous exit
            }
          }
          const double dx = loop[j].first - prev_end.first;
          const double dy = loop[j].second - prev_end.second;
          const double d2 = dx * dx + dy * dy;
          double cost = d2;
          if (pass == 0 && en > 1e-9)
          {
            const double forward = (dx * ex + dy * ey) / en;
            const double lateral =
                std::min(std::sqrt(std::max(0.0, d2 - forward * forward)), 4.0 * turn_radius);
            const double f_min =
                lateral > 1e-9 ? std::sqrt(lateral * (4.0 * turn_radius - lateral)) : 0.0;
            const double deficit = f_min - forward;
            if (deficit > 1e-9)
            {
              cost = kInsufficientPenalty + deficit;
            }
          }
          if (cost < best_cost)
          {
            best_cost = cost;
            best = j;
            found_aligned = (pass == 0);
          }
        }
        if (pass == 0 && !found_aligned)
        {
          best_cost = std::numeric_limits<double>::max();  // retry unfiltered
        }
      }
      std::rotate(loop.begin(), loop.begin() + static_cast<std::ptrdiff_t>(best), loop.end());
      loop.push_back(loop.front());
    }
    segs.push_back(std::move(loop));
  }

  // Every seg from this index on is a mainland swath (issue #497): a join
  // whose PREVIOUS segment index is >= this is a swath-to-swath row-end
  // U-turn, the only kind swath_turn_boundary is allowed to tighten. Ring-to-
  // ring joins and the ring-to-first-swath transition (previous index below
  // this) always stay on `boundary` — see the join loop below.
  const std::size_t first_swath_seg_idx = segs.size();

  // Chain swaths by the cost of the connector we can actually drive. Endpoint
  // distance alone favours tight omega turns and can choose a neighbour across
  // a hole over a slightly farther swath with a clear turn. Keep the original
  // Boustrophedon seed for determinism and evaluate both directions thereafter.
  // Euclidean distance is a lower bound: candidates already farther than the
  // best connector need no geometry search. No mowing primitive is dropped.
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> ordered_swaths;
  {
    const auto& sw_in = plan.swaths;
    std::vector<bool> used(sw_in.size(), false);
    // Keep the first generated swath as the fixed seed, independent of where
    // the last perimeter loop closes. Subsequent choices use the actual turn
    // envelope; moving the ring start must not move the sweep's starting row.
    std::pair<double, double> cur =
        !sw_in.empty() ? sw_in.front().first : std::pair<double, double>{0.0, 0.0};
    ordered_swaths.reserve(sw_in.size());
    for (std::size_t n = 0; n < sw_in.size(); ++n)
    {
      std::size_t best = sw_in.size();
      bool flip = false;
      double best_d = std::numeric_limits<double>::max();
      for (std::size_t i = 0; i < sw_in.size(); ++i)
      {
        if (used[i])
        {
          continue;
        }
        for (bool reversed : {false, true})
        {
          const auto& entry = reversed ? sw_in[i].second : sw_in[i].first;
          const auto& exit = reversed ? sw_in[i].first : sw_in[i].second;
          const double gap = std::hypot(entry.first - cur.first, entry.second - cur.second);
          if (gap >= best_d)
          {
            continue;
          }
          double cost = gap;
          if (!ordered_swaths.empty())
          {
            const auto& previous = ordered_swaths.back();
            const Pose start{cur.first,
                             cur.second,
                             std::atan2(previous.second.second - previous.first.second,
                                        previous.second.first - previous.first.first)};
            const Pose goal{entry.first,
                            entry.second,
                            std::atan2(exit.second - entry.second, exit.first - entry.first)};
            const auto& envelope = swath_turn_boundary.size() >= 3 ? swath_turn_boundary : boundary;
            bool fallback = false;
            const auto conn = buildConnector(start,
                                             goal,
                                             envelope,
                                             plan.safe_holes,
                                             turn_radius,
                                             std::max(0.02, min_turn_radius),
                                             step,
                                             fallback);
            if (!fallback && !conn.empty())
            {
              cost = std::hypot(conn.back().first - entry.first, conn.back().second - entry.second);
              for (std::size_t k = 1; k < conn.size(); ++k)
              {
                cost += std::hypot(conn[k].first - conn[k - 1].first,
                                   conn[k].second - conn[k - 1].second);
              }
            }
            else if (!conn.empty() && allInside(conn, envelope) &&
                     clearOfHoles(conn, plan.safe_holes) &&
                     straightFallbackIsContinuous(start, goal))
            {
              cost = gap;
            }
            else
            {
              // A stop/reorientation has a cost even when the endpoints nearly
              // coincide. Express heading changes as equivalent arc length at
              // the configured radius; a Nav2 relocation gets a full-circle
              // penalty. These are ordering heuristics, not transit routes or
              // execution-time estimates. The join builder still decides safety.
              const double radius = std::max(0.02, std::max(turn_radius, min_turn_radius));
              const auto pivot =
                  planPivotJoin(start, goal, conn, envelope, plan.safe_holes, pivot_limits);
              const double heading = std::atan2(goal.y - start.y, goal.x - start.x);
              cost += radius *
                      (pivot.ok ? (gap < mowgli_interfaces::coverage_geometry::kPivotCornerMaxStepM
                                       ? headingError(goal.theta, start.theta)
                                       : headingError(heading, start.theta) +
                                             headingError(goal.theta, heading))
                                : 2.0 * M_PI);
            }
          }
          if (cost < best_d)
          {
            best_d = cost;
            best = i;
            flip = reversed;
          }
        }
      }
      used[best] = true;
      auto sw = sw_in[best];
      if (flip)
      {
        std::swap(sw.first, sw.second);
      }
      cur = sw.second;
      ordered_swaths.push_back(sw);
    }
  }
  for (const auto& sw : ordered_swaths)
  {
    std::vector<std::pair<double, double>> pts;
    densifySegmentStep(
        sw.first.first, sw.first.second, sw.second.first, sw.second.second, step, pts);
    pts.emplace_back(sw.second.first, sw.second.second);  // include the end
    if (pts.size() >= 2)
    {
      segs.push_back(std::move(pts));
    }
  }

  std::vector<std::vector<std::pair<double, double>>> subpaths;
  if (segs.empty())
  {
    return subpaths;
  }

  // Heading at a polyline's start (p[0]→p[1]) and end (p[n-2]→p[n-1]).
  auto entryHeading = [](const std::vector<std::pair<double, double>>& s)
  {
    return std::atan2(s[1].second - s[0].second, s[1].first - s[0].first);
  };
  auto exitHeading = [](const std::vector<std::pair<double, double>>& s)
  {
    const std::size_t n = s.size();
    return std::atan2(s[n - 1].second - s[n - 2].second, s[n - 1].first - s[n - 2].first);
  };

  // Hard floor on every connector arc: the robot's minimum trackable turning
  // radius (mowgli_robot.yaml min_turning_radius). Shrinking a turn-around below
  // this to "fit in-bounds" produced loops FTC could not track (wz≈vx/r), so the
  // robot looped/hesitated; when no arc >= this fits, buildConnector falls back
  // to a straight join instead of an untrackable loop.
  const double min_radius = std::max(0.02, min_turn_radius);

  std::size_t ring_subpath_count = 0;
  bool path_has_ring = false;
  std::vector<std::pair<double, double>> path;  // the sub-path being accumulated
  for (std::size_t i = 0; i < segs.size(); ++i)
  {
    if (i > 0 && !path.empty())
    {
      // Forward turn-around connector from the previous segment's exit pose to
      // this segment's entry pose. Both are oriented, so the Dubins path leaves
      // and arrives tangent → no cusp at either junction.
      const Pose start{path.back().first, path.back().second, exitHeading(segs[i - 1])};
      const Pose goal{segs[i].front().first, segs[i].front().second, entryHeading(segs[i])};
      // Attempt a blade-on connector for every segment join. A real Dubins
      // connector is tangent at both ends. When no such arc fits, the straight
      // fallback may stay in the same sub-path only if it is also aligned with
      // both segment headings. An in-bounds fallback with a heading discontinuity
      // is geometrically safe but not drivable as one forward path; split it so
      // FollowStrip performs a blade-off reorientation before the next segment.
      //
      // issue #497: a join between two MAINLAND SWATHS (both segs[i-1] and
      // segs[i] at/after first_swath_seg_idx) is bound by swath_turn_boundary
      // when the caller supplied one — every other join (ring-to-ring, the
      // ring-to-first-swath transition) stays on the wider `boundary` so a
      // limited turn envelope can never push a headland ring's own connector
      // off ring 0.
      const bool is_swath_join = (i - 1) >= first_swath_seg_idx;
      const std::vector<std::pair<double, double>>& conn_boundary =
          (is_swath_join && swath_turn_boundary.size() >= 3) ? swath_turn_boundary : boundary;
      bool conn_safe = false;
      PivotJoin pivot_join;
      {
        bool fallback = false;
        auto conn = buildConnector(
            start, goal, conn_boundary, plan.safe_holes, turn_radius, min_radius, step, fallback);
        conn_safe = !conn.empty() && (!fallback || (allInside(conn, conn_boundary) &&
                                                    clearOfHoles(conn, plan.safe_holes) &&
                                                    straightFallbackIsContinuous(start, goal)));
        // No arc and no aligned straight: before splitting, try to keep the
        // join as a pivot join (short, in-bounds, pivot sweep inside the
        // designed envelope). Disabled pivot limits leave this `ok == false`.
        if (!conn_safe && fallback)
        {
          pivot_join =
              planPivotJoin(start, goal, conn, conn_boundary, plan.safe_holes, pivot_limits);
        }
        // Pure accounting of how this join resolved (issue #499) — see
        // ConnectorStats. Deliberately AFTER the decision so the classification
        // reflects what is actually driven, not just whether buildConnector
        // reached its straight-connector last resort. Changes no decision.
        if (stats != nullptr)
        {
          ++stats->attempted;
          if (pivot_join.ok)
          {
            ++stats->pivot;
          }
          else if (!conn_safe)
          {
            ++stats->split;
          }
          else if (fallback)
          {
            ++stats->straight_kept;
          }
          else
          {
            ++stats->arc;
          }
        }
        if (conn_safe)
        {
          // conn is start-inclusive (== path.back()) / goal-exclusive; drop the
          // duplicate start so the polyline stays simple.
          path.insert(path.end(), conn.begin() + 1, conn.end());
        }
        else if (pivot_join.ok)
        {
          appendPivotJoin(path, conn, goal, pivot_join);
        }
      }
      if (!conn_safe && !pivot_join.ok)
      {
        subpaths.push_back(std::move(path));
        if (path_has_ring)
        {
          ++ring_subpath_count;
        }
        path_has_ring = false;
        path.clear();
      }
    }
    // Append this segment. When continuing a sub-path, path.back() (== goal of
    // the connector) coincides with segs[i].front(), so skip the first point to
    // avoid a zero-length step; at a fresh sub-path start, take the whole segment.
    const auto& s = segs[i];
    path_has_ring = path_has_ring || i < first_swath_seg_idx;
    if (path.empty())
    {
      path.insert(path.end(), s.begin(), s.end());
    }
    else
    {
      path.insert(path.end(), s.begin() + 1, s.end());
    }
  }
  if (!path.empty())
  {
    subpaths.push_back(std::move(path));
    if (path_has_ring)
    {
      ++ring_subpath_count;
    }
  }

  // Round ONLY the true cusps — corners that exceed the 90° inversion limit
  // (e.g. the recorded boundary's ~91° acute vertex the rings inherit). We do
  // NOT fillet gentle (≤88°) ring corners: those are not cusps (FTC tracks a
  // one-directional ≤90° turn fine, just slowing a bit), and filleting them near
  // the boundary forced the radius down to ~0.02 m — an arc far too tight for
  // FTC to track (wz≈vx/r), so the robot looped/hesitated at corners it used to
  // turn through cleanly. 88° (not 90°) leaves a small margin so every corner
  // findFirstPathInversion would flag (>90°) is still rounded. The fillet radius
  // is floored at min_radius (= min_turn_radius): a corner that can only be
  // rounded tighter than that is left sharp (pivoted through) rather than turned
  // into a sub-trackable loop. Applied PER sub-path (each is independently
  // continuous; a fillet must never span a break).
  constexpr double kCornerThreshold = 88.0 * M_PI / 180.0;
  // Unrounded corners sharper than this become in-place pivot corners. Below it
  // FTC turns through the corner while moving (slowing) without cutting it.
  constexpr double kSharpCornerPivotRad = 60.0 * M_PI / 180.0;
  const double fillet_r = std::max(turn_radius * 0.5, min_radius);
  std::vector<std::vector<std::pair<double, double>>> out;
  out.reserve(subpaths.size());
  for (auto& sp : subpaths)
  {
    if (sp.size() < 2)
    {
      continue;
    }
    auto rounded = roundSharpCorners(
        sp, boundary, plan.safe_holes, kCornerThreshold, fillet_r, min_radius, step);
    // Pivot corner twins, marked BEFORE the clamp below. Up to here the only
    // bit-exact consecutive duplicates are the twins appendPivotJoin emitted:
    // the densifiers skip zero-length steps, the connectors drop their start,
    // and roundSharpCorners never fillets a zero-length vertex, so it carries
    // the twins through untouched.
    std::vector<bool> is_twin(rounded.size(), false);
    for (std::size_t k = 1; k < rounded.size(); ++k)
    {
      is_twin[k] = rounded[k] == rounded[k - 1];
    }
    // SAFETY (#388): the ring/swath poses are appended verbatim from F2C, and the
    // OUTERMOST ring's convex-corner vertices can poke a few cm PAST the clearance
    // ring — generateHeadlandSwaths (the ring) and generateHeadlands (this
    // `boundary`) round corners differently, so the two nominally-coincident
    // offsets disagree at corners. Left uncorrected, the first such pose is the
    // seg-1 start TransitToStrip drives to; sitting outside the recorded line (in
    // the keepout mask's lethal band) it made the blade-off transit un-drivable
    // and the whole sub-path was skipped. Project every out-of-bounds pose back
    // just inside the clearance ring so the path — and its start — stay drivable.
    // A no-op for the in-bounds majority.
    if (boundary.size() >= 3)
    {
      for (auto& pt : rounded)
      {
        pt = clampInsideRing(pt.first, pt.second, boundary, kClearanceClampMarginM);
      }
    }
    // A corner the fillet could not round is still a zero-radius corner. FTC
    // drives one as a curve unless it is a pivot corner, sweeping the nose wide
    // of the plan (field 2026-10-10: 65 such ring corners, FTC wedged against
    // the hedge at three of them). Turn it in place instead, where the pivot
    // sweep fits (pivotSweepFits, as for pivot joins). After the clamp, so the
    // sweep is judged at the pose the robot will actually drive.
    rounded = pivotSharpCorners(rounded, kSharpCornerPivotRad, pivot_limits, &is_twin);
    // Connector discontinuities were split before rounding, so every remaining
    // join in this sub-path is either part of an original segment, tangent to a
    // real connector arc, or an explicit pivot corner.
    rounded = enforcePivotCornerContract(rounded, is_twin);
    if (rounded.size() >= 2)
    {
      out.emplace_back(std::move(rounded));
    }
  }

  // Sub-path DRIVE ORDER: see orderSubPathsForMinimalTransit's doc comment —
  // extracted to its own pure, unit-testable function (mowgli_coverage
  // convention for decision logic like this; see test_coverage_planning.cpp).
  return orderSubPathsForMinimalTransit(std::move(out), ring_subpath_count, pin_first_subpath);
}

std::vector<std::vector<std::pair<double, double>>> orderSubPathsForMinimalTransit(
    std::vector<std::vector<std::pair<double, double>>> sub_paths,
    std::size_t ring_bearing_count,
    bool pin_first_seed)
{
  // Minimize the blade-off Nav2 transit BETWEEN sub-paths. Sub-paths arrive in
  // swath-chain order; on a multi-hole field that leaves the driver
  // criss-crossing the lawn between lobes (measured 77 m of blade-off transit
  // on the recorded 4-hole garden). Greedy nearest-neighbour over the
  // FINISHED sub-path polylines, entering each at whichever end is nearer,
  // mows spatially adjacent lobes consecutively (77 → 47 m there).
  //   * Reversing a FINISHED polyline is safe: the points are identical, so every
  //     turn-around / fillet stays exactly as in-bounds and trackable as before —
  //     only reversing the swath ORDER *before* the path is built relocates
  //     U-turns (the hazard the seed-from-BoustrophedonOrder note in
  //     buildContinuousSubPaths guards); driving the same polyline backwards
  //     moves nothing.
  //   * With NO rings, the SEED (which sub-path drives first) is tried at every
  //     candidate, not pinned to the input's own first element (mowglinext#818: with only 2
  //     sub-paths, a fixed seed=0 can only ever reverse the OTHER one — it
  //     structurally cannot discover that starting from the other sub-path
  //     gives a shorter link, field-measured as a single 9.78 m gap on an
  //     otherwise-adjacent 2-lobe area). This stays deterministic — still a
  //     pure function of the sub-path geometries, no robot-position input — so
  //     the BT's resume-by-index contract (a fixed plan yields a fixed order,
  //     stable across re-plans) holds exactly as before; only the SEARCH grew,
  //     not what makes the result reproducible. TransitToStrip has no prior
  //     commitment to any particular sub-path — it simply reads
  //     drivable_subpaths.front() from buildContinuousSubPaths' result
  //     (PlanCoverageArea, mowgli_behavior/src/coverage_nodes.cpp) — so any
  //     sub-path is free to end up there.
  //   * The seed itself is always driven forward (front→back) — only the
  //     NON-seed sub-paths may be reversed. Trying every sub-path as seed still
  //     covers the reverse-the-other-one case the single-seed version had,
  //     plus every case where the BEST link is to the seed's normally-unused
  //     front end via a different sub-path arriving there first.
  //   * All ring-bearing sub-paths retain their ORIGINAL winding AND stay ahead
  //     of every interior-only path. Rings arrive before swaths, so the builder
  //     supplies the number of ring-bearing paths at the start of the input,
  //     including a mixed ring/swath path. The OUTERMOST-ring path is input 0
  //     and remains the seed. This preserves the mower's headland-first contract:
  //     a global transit optimum must not mow interior rows, relocate to the
  //     perimeter, then return to the interior. Protecting direction alone was
  //     insufficient because it still allowed the whole ring path to move.
  //   * Bounded to kMaxSeedSearchSize sub-paths (O(n^3) — every seed reruns the
  //     O(n^2) chain): a pathological multi-hole field with more lobes than that
  //     falls back to the single-seed=0 search instead, which is still a
  //     strict improvement over the raw input order and was the whole behavior
  //     before this change.
  //   * Adopt the best seed's order ONLY when it actually shortens the transit
  //     versus the raw input order, so a field the input order already
  //     sequenced well can never regress.
  if (sub_paths.size() <= 1)
  {
    return sub_paths;
  }
  auto gap = [](const std::pair<double, double>& a, const std::pair<double, double>& b)
  {
    return std::hypot(a.first - b.first, a.second - b.second);
  };
  double chain_transit = 0.0;
  for (std::size_t i = 1; i < sub_paths.size(); ++i)
  {
    chain_transit += gap(sub_paths[i - 1].back(), sub_paths[i].front());
  }

  // Greedy NN chain starting from `seed`: at each step, enter whichever
  // unused sub-path is nearest (forward or reversed) to the current end.
  struct SeedResult
  {
    std::vector<std::size_t> order;
    std::vector<bool> reversed_flag;
    double transit;
  };
  const std::size_t protected_count = std::min(ring_bearing_count, sub_paths.size());
  auto chainFromSeed = [&](std::size_t seed) -> SeedResult
  {
    SeedResult result{{seed}, std::vector<bool>(sub_paths.size(), false), 0.0};
    std::vector<bool> used(sub_paths.size(), false);
    used[seed] = true;
    std::pair<double, double> cur = sub_paths[seed].back();
    for (std::size_t n = 1; n < sub_paths.size(); ++n)
    {
      std::size_t best = 0;
      bool best_rev = false;
      double best_d = std::numeric_limits<double>::max();
      // Finish the ring-bearing prefix before considering an interior-only
      // path. Once that prefix is consumed, search the whole unused remainder.
      const bool rings_remain =
          std::any_of(used.begin(),
                      used.begin() + static_cast<std::ptrdiff_t>(protected_count),
                      [](bool value)
                      {
                        return !value;
                      });
      const std::size_t candidate_end = rings_remain ? protected_count : sub_paths.size();
      for (std::size_t j = 0; j < candidate_end; ++j)
      {
        if (used[j])
        {
          continue;
        }
        const double ds = gap(cur, sub_paths[j].front());  // enter forward
        if (ds < best_d)
        {
          best_d = ds;
          best = j;
          best_rev = false;
        }
        // Preserve the operator's winding on every path containing a ring.
        if (j < std::max(std::size_t{1}, protected_count))
        {
          continue;
        }
        const double de = gap(cur, sub_paths[j].back());  // enter reversed
        if (de < best_d)
        {
          best_d = de;
          best = j;
          best_rev = true;
        }
      }
      used[best] = true;
      result.reversed_flag[best] = best_rev;
      result.order.push_back(best);
      result.transit += best_d;
      cur = best_rev ? sub_paths[best].front() : sub_paths[best].back();
    }
    return result;
  };

  constexpr std::size_t kMaxSeedSearchSize = 40;
  // A ring-bearing plan always starts with input 0, which carries the outermost
  // perimeter. pin_first_seed applies the same rule to a ring-free caller that
  // explicitly selected its starting path.
  const std::size_t seed_count =
      (protected_count == 0 && !pin_first_seed && sub_paths.size() <= kMaxSeedSearchSize)
          ? sub_paths.size()
          : 1;
  SeedResult best = chainFromSeed(0);
  for (std::size_t seed = 1; seed < seed_count; ++seed)
  {
    SeedResult candidate = chainFromSeed(seed);
    if (candidate.transit + 1e-6 < best.transit)
    {
      best = std::move(candidate);
    }
  }

  if (best.transit + 1e-6 >= chain_transit)
  {
    return sub_paths;
  }
  std::vector<std::vector<std::pair<double, double>>> reordered;
  reordered.reserve(sub_paths.size());
  for (const std::size_t idx : best.order)
  {
    std::vector<std::pair<double, double>> sp = std::move(sub_paths[idx]);
    if (best.reversed_flag[idx])
    {
      std::reverse(sp.begin(), sp.end());
    }
    reordered.push_back(std::move(sp));
  }
  return reordered;
}

std::vector<std::pair<double, double>> buildContinuousPath(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step)
{
  // Concatenate the drivable sub-paths into one visualisation polyline. The
  // driver uses buildContinuousSubPaths directly so every boundary becomes a
  // blade-off Nav2 transit rather than a driven discontinuity.
  const auto subs = buildContinuousSubPaths(plan, boundary, turn_radius, min_turn_radius, step);
  std::vector<std::pair<double, double>> path;
  for (const auto& sp : subs)
  {
    path.insert(path.end(), sp.begin(), sp.end());
  }
  return path;
}

// ── Pivot joins (public helpers) ─────────────────────────────────────────────

bool pivotSweepFits(double x, double y, const PivotJoinLimits& limits)
{
  if (limits.sweep_radius <= 0.0 || limits.recorded_boundary.size() < 3)
  {
    return false;
  }
  // Signed distance of the pivot point past the recorded line (negative = that
  // deep inside). Every point of the disc lies within max(0, signed + radius)
  // of the recorded polygon — for an inside point, the disc of its depth is in
  // the polygon and the rest of the sweep is at most radius − depth beyond it —
  // so the disc is inside the polygon grown by boundary_margin whenever
  // signed + radius <= margin. Exact on straight edges, conservative elsewhere.
  // The 1 mm tolerance keeps a pivot ON the recorded line (rings off, inset 0,
  // margin floored at the radius) deterministic, as for allInside().
  const double edge = distanceToRing(x, y, limits.recorded_boundary);
  const double signed_outside = pointInRing(x, y, limits.recorded_boundary) ? -edge : edge;
  if (signed_outside + limits.sweep_radius > limits.boundary_margin + kOnEdgeTolM)
  {
    return false;
  }
  for (const auto& obstacle : limits.recorded_obstacles)
  {
    if (obstacle.size() < 3)
    {
      continue;
    }
    if (pointInRing(x, y, obstacle) || distanceToRing(x, y, obstacle) < limits.sweep_radius)
    {
      return false;
    }
  }
  return true;
}

std::vector<double> pathHeadings(const std::vector<std::pair<double, double>>& pts)
{
  const std::size_t n = pts.size();
  std::vector<double> yaw(n, 0.0);
  const auto step_heading = [&](std::size_t a, std::size_t b, double& out)
  {
    const double dx = pts[b].first - pts[a].first;
    const double dy = pts[b].second - pts[a].second;
    if (std::hypot(dx, dy) < cg::kPivotCornerMaxStepM)
    {
      return false;
    }
    out = std::atan2(dy, dx);
    return true;
  };
  for (std::size_t i = 0; i < n; ++i)
  {
    // Outgoing step first; a zero-length one (the first pose of a pivot corner)
    // or the last pose falls back to the incoming step.
    if (i + 1 < n && step_heading(i, i + 1, yaw[i]))
    {
      continue;
    }
    if (i > 0 && step_heading(i - 1, i, yaw[i]))
    {
      continue;
    }
    if (i > 0)
    {
      yaw[i] = yaw[i - 1];
      continue;
    }
    // Degenerate head (never produced by buildContinuousSubPaths): take the
    // first real step ahead.
    std::size_t j = 1;
    while (j < n && !step_heading(0, j, yaw[0]))
    {
      ++j;
    }
  }
  return yaw;
}

// ── Boundary geometry (used by coverage_server's in-bounds verification) ─────

bool pointInRing(double x, double y, const std::vector<std::pair<double, double>>& ring)
{
  const std::size_t n = ring.size();
  if (n < 3)
  {
    return false;
  }
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double xi = ring[i].first, yi = ring[i].second;
    const double xj = ring[j].first, yj = ring[j].second;
    const bool crosses = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (crosses)
    {
      inside = !inside;
    }
  }
  return inside;
}

double distanceToRing(double x, double y, const std::vector<std::pair<double, double>>& ring)
{
  const std::size_t n = ring.size();
  if (n < 2)
  {
    return std::numeric_limits<double>::max();
  }
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
  {
    const double ax = ring[j].first, ay = ring[j].second;
    const double bx = ring[i].first, by = ring[i].second;
    const double dx = bx - ax, dy = by - ay;
    const double len2 = dx * dx + dy * dy;
    double t = (len2 > 0.0) ? ((x - ax) * dx + (y - ay) * dy) / len2 : 0.0;
    t = std::max(0.0, std::min(1.0, t));
    const double px = ax + t * dx, py = ay + t * dy;
    best = std::min(best, std::hypot(x - px, y - py));
  }
  return best;
}

f2c::types::LinearRing bufferRingOutward(const f2c::types::LinearRing& in, double margin)
{
  if (margin < 1e-3 || in.size() < 3)
  {
    return dedupClosedRing(in);
  }
  // Grow the ring's polygon outward with GDAL/OGR (F2C's own geometry
  // backend): drawn map obstacles get an operator-tunable safety margin
  // (obstacle_margin) so the planner keeps swaths, connectors and headlands
  // off root zones the 2D LiDAR cannot see. Rounded joins (8 quadrant
  // segments) avoid miter spikes on concave operator polygons.
  OGRLinearRing ogr_ring;
  for (std::size_t i = 0; i < in.size(); ++i)
  {
    const auto p = in.getGeometry(i);
    ogr_ring.addPoint(p.getX(), p.getY());
  }
  ogr_ring.closeRings();
  OGRPolygon poly;
  poly.addRing(&ogr_ring);
  std::unique_ptr<OGRGeometry> grown(poly.Buffer(margin, 8));
  // Buffer degeneracies (self-intersecting input, collapsed area) fall back
  // to the raw ring — the planner still avoids the drawn polygon itself,
  // just without the extra margin. Never drop the obstacle.
  if (!grown)
  {
    return dedupClosedRing(in);
  }
  const OGRPolygon* grown_poly = nullptr;
  const auto flat_type = wkbFlatten(grown->getGeometryType());
  if (flat_type == wkbPolygon)
  {
    grown_poly = grown->toPolygon();
  }
  else if (flat_type == wkbMultiPolygon)
  {
    // Outward buffering can merge lobes of a degenerate ring into several
    // parts; keep the largest (the obstacle body).
    double best_area = -1.0;
    for (const auto* part : *grown->toMultiPolygon())
    {
      const double a = part->get_Area();
      if (a > best_area)
      {
        best_area = a;
        grown_poly = part;
      }
    }
  }
  if (!grown_poly || !grown_poly->getExteriorRing() ||
      grown_poly->getExteriorRing()->getNumPoints() < 4)
  {
    return dedupClosedRing(in);
  }
  const OGRLinearRing* ext = grown_poly->getExteriorRing();
  f2c::types::LinearRing out;
  for (int i = 0; i < ext->getNumPoints(); ++i)
  {
    out.addPoint(f2c::types::Point(ext->getX(i), ext->getY(i)));
  }
  return dedupClosedRing(out);
}

f2c::types::LinearRing erodeRingInward(const f2c::types::LinearRing& in, double distance)
{
  if (distance <= 0.0 || in.size() < 3)
  {
    return dedupClosedRing(in);
  }
  // Same GDAL/OGR Buffer() call as bufferRingOutward, with a NEGATED distance
  // — OGR erodes a polygon for a negative buffer distance. Rounded joins (8
  // quadrant segments) match the growth side so a round-tripped ring (grow
  // then shrink by the same amount) returns close to the original shape.
  OGRLinearRing ogr_ring;
  for (std::size_t i = 0; i < in.size(); ++i)
  {
    const auto p = in.getGeometry(i);
    ogr_ring.addPoint(p.getX(), p.getY());
  }
  ogr_ring.closeRings();
  OGRPolygon poly;
  poly.addRing(&ogr_ring);
  std::unique_ptr<OGRGeometry> shrunk(poly.Buffer(-distance, 8));
  // Erosion legitimately collapses a small/thin/degenerate polygon to
  // nothing — that MUST surface as failure (an empty ring), never as a
  // silent fall-back to the uncorrected input: the whole point of this
  // function is that the caller never keeps the extra margin.
  if (!shrunk)
  {
    return f2c::types::LinearRing();
  }
  const OGRPolygon* shrunk_poly = nullptr;
  const auto flat_type = wkbFlatten(shrunk->getGeometryType());
  if (flat_type == wkbPolygon)
  {
    shrunk_poly = shrunk->toPolygon();
  }
  else if (flat_type == wkbMultiPolygon)
  {
    // A concave ring can erode into several disjoint parts; keep the
    // largest, matching bufferRingOutward's growth-side tie-break.
    double best_area = -1.0;
    for (const auto* part : *shrunk->toMultiPolygon())
    {
      const double a = part->get_Area();
      if (a > best_area)
      {
        best_area = a;
        shrunk_poly = part;
      }
    }
  }
  if (!shrunk_poly || !shrunk_poly->getExteriorRing() ||
      shrunk_poly->getExteriorRing()->getNumPoints() < 4)
  {
    return f2c::types::LinearRing();
  }
  const OGRLinearRing* ext = shrunk_poly->getExteriorRing();
  f2c::types::LinearRing out;
  for (int i = 0; i < ext->getNumPoints(); ++i)
  {
    out.addPoint(f2c::types::Point(ext->getX(i), ext->getY(i)));
  }
  return dedupClosedRing(out);
}

namespace
{

// Ring sanitization tolerances. Operator-recorded field boundaries carry
// mm-scale geometric degeneracies (a real 252 m² field had a 1.2 mm near-
// duplicate closure vertex and several 1–10 mm sliver edges). These are EXACT
// enough that boost::geometry accepts the points yet degenerate enough that
// F2C's generateHeadlands offset and BruteForce swath clip silently produce
// PARTIAL coverage. Both tolerances sit safely below op_width (0.16 m) so no
// real corner is ever removed.
constexpr double kRingDedupTolM = 0.01;  // drop a vertex within 1 cm of the last kept one
constexpr double kRingSpikeTolM = 0.005;  // drop a vertex within 5 mm of its neighbours' chord

// Repair a ring into a boost::geometry-valid one via F2C's own OGR backend.
// Buffer-by-zero is the standard OGR self-intersection fix (the same Buffer call
// bufferRingOutward already relies on); it returns the valid equivalent of a
// self-touching / sliver-laden ring. Keep the largest resulting polygon's
// exterior. Any degeneracy (null buffer, empty result, collapsed ring) returns
// the input unchanged — the planner must NEVER drop the field.
f2c::types::LinearRing makeRingValid(const f2c::types::LinearRing& in)
{
  if (in.size() < 4)
  {
    return in;
  }
  OGRLinearRing ogr_ring;
  for (std::size_t i = 0; i < in.size(); ++i)
  {
    const auto p = in.getGeometry(i);
    ogr_ring.addPoint(p.getX(), p.getY());
  }
  ogr_ring.closeRings();
  OGRPolygon poly;
  poly.addRing(&ogr_ring);
  std::unique_ptr<OGRGeometry> fixed(poly.Buffer(0.0));
  if (!fixed)
  {
    return in;
  }
  const OGRPolygon* fixed_poly = nullptr;
  const auto flat_type = wkbFlatten(fixed->getGeometryType());
  if (flat_type == wkbPolygon)
  {
    fixed_poly = fixed->toPolygon();
  }
  else if (flat_type == wkbMultiPolygon)
  {
    // Repair can split a bow-tie ring into several parts; keep the field body.
    double best_area = -1.0;
    for (const auto* part : *fixed->toMultiPolygon())
    {
      const double a = part->get_Area();
      if (a > best_area)
      {
        best_area = a;
        fixed_poly = part;
      }
    }
  }
  if (!fixed_poly || !fixed_poly->getExteriorRing() ||
      fixed_poly->getExteriorRing()->getNumPoints() < 4)
  {
    return in;
  }
  const OGRLinearRing* ext = fixed_poly->getExteriorRing();
  f2c::types::LinearRing out;
  for (int i = 0; i < ext->getNumPoints(); ++i)
  {
    out.addPoint(f2c::types::Point(ext->getX(i), ext->getY(i)));
  }
  return out;
}

}  // namespace

f2c::types::LinearRing dedupClosedRing(const f2c::types::LinearRing& in)
{
  // 1. Metric dedup: drop any vertex within kRingDedupTolM of the previous kept
  // vertex. A 1e-9 (nanometre) tolerance let mm-scale near-duplicates through —
  // boost accepts the points but F2C's headland offset / swath clip then drop
  // whole swaths. 1 cm is well below op_width, so real corners are preserved.
  std::vector<f2c::types::Point> pts;
  for (std::size_t i = 0; i < in.size(); ++i)
  {
    const auto p = in.getGeometry(i);
    if (!pts.empty() &&
        std::hypot(p.getX() - pts.back().getX(), p.getY() - pts.back().getY()) < kRingDedupTolM)
    {
      continue;  // near-zero-length edge — boost/F2C would reject or mis-clip it
    }
    pts.push_back(f2c::types::Point(p.getX(), p.getY()));
  }
  // Drop a near-duplicate closing vertex (the 1.2 mm seam) so the open vertex
  // list carries no first≈last pair; the ring is re-closed explicitly below.
  while (pts.size() >= 2 && std::hypot(pts.front().getX() - pts.back().getX(),
                                       pts.front().getY() - pts.back().getY()) < kRingDedupTolM)
  {
    pts.pop_back();
  }

  // 2. Spike / near-collinear removal: drop any vertex whose perpendicular
  // distance to the chord between its PREVIOUS KEPT vertex and its next neighbour
  // is below kRingSpikeTolM. This clears both exactly-collinear points and
  // mm-scale hooks (a 1 mm needle is invalid to boost even when each of its edges
  // exceeds kRingDedupTolM). Chaining off the last kept vertex (rather than the
  // raw predecessor) bounds the total deviation along a genuine curve to
  // kRingSpikeTolM, so a rounded corner from an OGR buffer is simplified — not
  // collapsed to a chamfer. Single deterministic pass (resume-by-index depends
  // on the plan being reproducible).
  if (pts.size() >= 4)
  {
    std::vector<f2c::types::Point> kept;
    const std::size_t n = pts.size();
    for (std::size_t i = 0; i < n; ++i)
    {
      const f2c::types::Point& a = kept.empty() ? pts[n - 1] : kept.back();
      const f2c::types::Point& b = pts[i];
      const f2c::types::Point& c = pts[(i + 1) % n];
      const double dx = c.getX() - a.getX();
      const double dy = c.getY() - a.getY();
      const double len2 = dx * dx + dy * dy;
      const double cross = dx * (a.getY() - b.getY()) - dy * (a.getX() - b.getX());
      double perp = std::hypot(b.getX() - a.getX(), b.getY() - a.getY());
      if (len2 > 0.0)
      {
        perp = std::fabs(cross) / std::sqrt(len2);
      }
      if (perp >= kRingSpikeTolM)
      {
        kept.push_back(b);
      }
    }
    if (kept.size() >= 3)  // never collapse the field below a triangle
    {
      pts = kept;
    }
  }

  // Re-close the ring (F2C wants first == last).
  f2c::types::LinearRing out;
  for (const auto& p : pts)
  {
    out.addPoint(p);
  }
  if (pts.size() >= 2)
  {
    out.addPoint(f2c::types::Point(pts.front().getX(), pts.front().getY()));
  }

  // 3. OGR validity repair: even after the metric/spike passes a ring can retain
  // a self-touch boost::geometry rejects. makeRingValid returns the boost-valid
  // equivalent, or the deduped ring unchanged if the repair degenerates.
  return makeRingValid(out);
}

namespace
{
// Douglas-Peucker over indices [lo, hi] of `pts`, marking the vertices to keep.
// Iterative (explicit stack): a densified ring has thousands of vertices and a
// pathological one must not be able to blow the call stack.
void markKeptVertices(const std::vector<std::pair<double, double>>& pts,
                      double tolerance,
                      std::vector<bool>& keep)
{
  std::vector<std::pair<std::size_t, std::size_t>> stack;
  stack.emplace_back(0, pts.size() - 1);
  while (!stack.empty())
  {
    const auto [lo, hi] = stack.back();
    stack.pop_back();
    if (hi <= lo + 1)
    {
      continue;
    }
    const double ax = pts[lo].first, ay = pts[lo].second;
    const double dx = pts[hi].first - ax, dy = pts[hi].second - ay;
    const double len2 = dx * dx + dy * dy;
    double worst = -1.0;
    std::size_t worst_i = lo;
    for (std::size_t i = lo + 1; i < hi; ++i)
    {
      const double px = pts[i].first - ax, py = pts[i].second - ay;
      // Distance to the chord SEGMENT, not to its infinite line: a ring that
      // doubles back on itself must not lose a vertex that is collinear with the
      // chord yet far beyond its end. A zero-length chord (a closed loop's first
      // and last vertex coincide) measures to the anchor itself (t stays 0).
      const double t = (len2 < 1e-12) ? 0.0 : std::clamp((px * dx + py * dy) / len2, 0.0, 1.0);
      const double dist = std::hypot(px - t * dx, py - t * dy);
      if (dist > worst)
      {
        worst = dist;
        worst_i = i;
      }
    }
    if (worst > tolerance)
    {
      keep[worst_i] = true;
      stack.emplace_back(lo, worst_i);
      stack.emplace_back(worst_i, hi);
    }
  }
}

std::vector<std::pair<double, double>> simplifyPolyline(
    const std::vector<std::pair<double, double>>& pts, double tolerance)
{
  if (pts.size() <= 2 || tolerance <= 0.0)
  {
    return pts;
  }
  std::vector<bool> keep(pts.size(), false);
  keep.front() = true;  // endpoints are always kept: a ring keeps its start and closure
  keep.back() = true;
  markKeptVertices(pts, tolerance, keep);
  std::vector<std::pair<double, double>> out;
  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    if (keep[i])
    {
      out.push_back(pts[i]);
    }
  }
  return out;
}
}  // namespace

CoveragePreview summarisePlanForPreview(const BoustrophedonPlan& plan, double simplify_tolerance_m)
{
  CoveragePreview preview;
  preview.rings.reserve(plan.rings.size());
  for (const auto& loop : plan.rings)
  {
    auto simplified = simplifyPolyline(loop, simplify_tolerance_m);
    if (simplified.size() >= 2)
    {
      preview.rings.push_back(std::move(simplified));
    }
  }
  preview.swaths = plan.swaths;

  // Heading in [0, 180): a swath has no sense of direction, and serpentine order
  // alternates it anyway, so 200 degrees and 20 degrees are the same lines.
  double deg = std::fmod(plan.swath_angle_rad * 180.0 / M_PI, 180.0);
  if (deg < 0.0)
  {
    deg += 180.0;
  }
  preview.swath_angle_deg = deg;
  preview.headland_passes = plan.n_headland_passes;
  preview.planned_fraction = plan.diagnostics.planned_fraction;
  preview.field_area_m2 = plan.diagnostics.field_area;
  preview.dropped_pieces = plan.diagnostics.drops.size();
  return preview;
}

}  // namespace mowgli_coverage
