// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// Simple boustrophedon coverage planner — see coverage_planning.hpp for the
// design rationale (headland rings + straight serpentine swaths, no turn
// planning; turns are the navigation stack's in-place pivots).

#include "mowgli_coverage/coverage_planning.hpp"

#include <algorithm>
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
  return best_angle;
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
std::vector<std::pair<double, double>> sampleDubins(
    const Pose& start, const Pose& goal, double radius, double step, double& out_len)
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

// True iff every sampled point of `pts` is inside `boundary`.
bool allInside(const std::vector<std::pair<double, double>>& pts,
               const std::vector<std::pair<double, double>>& boundary)
{
  for (const auto& p : pts)
  {
    if (!pointInRing(p.first, p.second, boundary))
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
    // Accept the largest radius whose arc is both inside the boundary AND clear
    // of every hole → smoothest turn-around that stays out of obstacles (#333).
    if (!pts.empty() && allInside(pts, boundary) && clearOfHoles(pts, holes))
    {
      return pts;
    }
  }
  // Last resort: straight blind connector (may leave the boundary OR cross a
  // hole → a real, reportable gap the server surfaces).
  used_fallback = true;
  std::vector<std::pair<double, double>> straight;
  densifySegmentStep(start.x, start.y, goal.x, goal.y, step, straight);
  return straight;
}

// Round any corner of `pts` whose turn angle exceeds `max_turn_rad` with a
// circular fillet tangent to both edges (cusp-free in/out), so the WHOLE path
// has no >90° turn. Such corners are intrinsic to the headland rings, which
// inherit acute vertices from the recorded boundary (e.g. a 91° polygon
// corner). The fillet curls toward the INSIDE of the turn (the mowed side); its
// radius is shrunk until the arc stays in-bounds and fits the adjacent edges.
// If a corner can't be rounded in-bounds with an arc of radius >= `min_radius`
// it is left as-is (a sharp corner the robot pivots through is better than a
// fillet too tight for MPPI to track — that produces the very loop/hesitation
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

BoustrophedonPlan planBoustrophedon(const f2c::types::Cell& field_cell,
                                    double op_width,
                                    double headland_width,
                                    int num_headland_passes_override,
                                    double chassis_safety_inset,
                                    double mow_angle_rad,
                                    double min_swath_length,
                                    int ring_direction,
                                    double min_turn_radius)
{
  BoustrophedonPlan plan;
  // Polygon area the planned-coverage fraction is taken over (the operator's
  // authorised area, before any inset). Instrumentation only.
  plan.diagnostics.field_area = field_cell.area();

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

  // (1) Boundary offset so the OUTERMOST headland ring's centerline sits
  // `chassis_safety_inset` inside the recorded line. generateHeadlandSwaths
  // (below) places ring 0 at op_width/2 inside the planning boundary, so shift
  // the planning field by (chassis_safety_inset - op_width/2):
  //   > 0  shrink INWARD  — operator asked to keep the chassis further inside
  //                         (chassis_safety_inset > op_width/2);
  //   < 0  expand OUTWARD — DEFAULT (inset 0): ring 0 rides ON the recorded
  //                         line, so the blade mows to the edge and the chassis
  //                         straddles the boundary. The half-chassis-width lethal
  //                         band the keepout mask places outside the line is the
  //                         safety stop, replacing the old inset floor.
  const double field_offset = chassis_safety_inset - 0.5 * op_width;
  f2c::types::Cells safe_cells = cells;
  bool boundary_offset_applied = false;
  if (field_offset > 1e-3)
  {
    safe_cells = hl.generateHeadlands(cells, field_offset);
    if (safe_cells.size() == 0 || safe_cells.area() < 1e-6)
    {
      return plan;  // inset consumed the field
    }
    boundary_offset_applied = true;
  }
  else if (field_offset < -1e-3)
  {
    // Expand the field exterior outward so ring 0 lands on the recorded line.
    f2c::types::Cells expanded;
    expanded.addGeometry(expandCellOutward(field_cell, -field_offset));
    if (expanded.size() > 0 && expanded.area() > 1e-6)
    {
      safe_cells = expanded;
      boundary_offset_applied = true;
    }
    // else: buffer degeneracy — fall back to the raw field (safe_cells = cells).
  }

  if (boundary_offset_applied)
  {
    // Expose the planning outer ring so the continuous-path connectors/fillets
    // are bounded by the SAME polygon the rings/swaths are planned against (not
    // the raw operator boundary). If an inset split the field, take the largest
    // cell's exterior ring (the dominant drivable region).
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
    const auto safe_ring = safe_cells.getGeometry(largest).getGeometry(0);  // exterior
    plan.safe_boundary.reserve(safe_ring.size());
    for (std::size_t i = 0; i < safe_ring.size(); ++i)
    {
      const auto p = safe_ring.getGeometry(i);
      plan.safe_boundary.emplace_back(p.getX(), p.getY());
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
  const int n_rings =
      (num_headland_passes_override > 0)
          ? num_headland_passes_override
          : std::max(1, static_cast<int>(std::ceil(headland_width / op_width - 1e-9)));
  const auto headland_passes =
      hl.generateHeadlandSwaths(safe_cells, op_width, n_rings, /*dir_out2in=*/true);
  // Drop degenerate micro-loops (a near-consumed tiny field can yield a
  // centimetre-scale innermost ring that isn't worth driving).
  constexpr double kMinRingPerimeter = 1.0;  // m
  double ring_strip_area = 0.0;  // Σ perimeter·op_width of KEPT rings (for the fraction)
  for (const auto& pass_lines : headland_passes)
  {
    auto loops = ringPassToLoops(pass_lines);
    for (auto& loop : loops)
    {
      double perim = 0.0;
      for (std::size_t i = 1; i < loop.size(); ++i)
      {
        perim += std::hypot(loop[i].first - loop[i - 1].first, loop[i].second - loop[i - 1].second);
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
          std::size_t le = 0;
          double le_len = -1.0;
          for (std::size_t j = 0; j < sparse.size(); ++j)
          {
            const auto& a = sparse[j];
            const auto& b = sparse[(j + 1) % sparse.size()];
            const double len = std::hypot(b.first - a.first, b.second - a.second);
            if (len > le_len)
            {
              le_len = len;
              le = j;
            }
          }
          // New start = the longest edge's midpoint. Rotate so the loop begins
          // at the vertex AFTER the edge (v_{le+1}) and stitch the midpoint on
          // both ends: mid → v_{le+1} → … → v_le → mid. (Rotating to v_le
          // instead would make the path step BACKWARD from mid to v_le — a
          // 180° reversal baked into the ring.)
          const std::pair<double, double> mid{
              (sparse[le].first + sparse[(le + 1) % sparse.size()].first) * 0.5,
              (sparse[le].second + sparse[(le + 1) % sparse.size()].second) * 0.5};
          std::rotate(sparse.begin(),
                      sparse.begin() + static_cast<std::ptrdiff_t>((le + 1) % sparse.size()),
                      sparse.end());
          sparse.insert(sparse.begin(), mid);
          sparse.push_back(mid);
          // Fillet every corner sharper than ~30° with a forward arc (floored at
          // min_turn_radius so MPPI/FTC can track it); corners that cannot be
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
  }

  // (3) Mainland: what's left inside the rings' cut band. May be empty on a
  // small field (rings-only plan — still valid coverage).
  f2c::types::Cells mainland = hl.generateHeadlands(safe_cells, n_rings * op_width);
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
  f2c::obj::NSwath n_swath_obj;
  f2c::rp::BoustrophedonOrder order;
  double swath_strip_area = 0.0;  // Σ length·op_width of KEPT swaths (for the fraction)
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
    f2c::types::Swaths swaths = (cell_angle >= 0.0)
                                    ? bf.generateSwaths(cell_angle, op_width, cell)
                                    : bf.generateBestSwaths(n_swath_obj, op_width, cell);
    if (swaths.size() == 0)
    {
      continue;
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

std::vector<std::vector<std::pair<double, double>>> buildContinuousSubPaths(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step)
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
  // The first ring keeps F2C's start (TransitToStrip already targets it).
  // Ring DRIVE-ORDER grouping. F2C's generateHeadlandSwaths emits ring loops PER
  // PASS as [outer, hole, outer, hole, …], so consecutive concentric OUTER rings
  // are interleaved with the field-centre hole rings. Driven in that raw order the
  // path jumps outer-ring → hole-ring → outer-ring every pass, and each jump is a
  // field-crossing relocation the join-gap split below (Split A) turns into a
  // spurious blade-off Nav2 transit (3–4 extra transits, purely an ordering
  // artifact). Partition into outer-boundary rings FIRST (outermost-first order
  // preserved) then the obstacle-encircling rings, so each group drives
  // contiguously and only ONE relocation separates the two. A ring is an OUTER
  // ring iff it encloses a MAINLAND point (a swath endpoint): the mainland lies
  // inside every nested outer ring and outside every small hole ring. Falls back
  // to the raw order when there are no swaths (rings-only field: no mainland probe,
  // and a fully-consumed field rarely carries hole rings worth reordering). Ring
  // GEOMETRY is untouched — only the order the loops are appended.
  std::vector<std::size_t> ring_order;
  ring_order.reserve(plan.rings.size());
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
  for (const std::size_t ring_idx : ring_order)
  {
    const auto& loop_in = plan.rings[ring_idx];
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
      const double kAlignCos = 0.7071;  // cos(45°)
      std::size_t best = 0;
      double best_d2 = std::numeric_limits<double>::max();
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
          if (d2 < best_d2)
          {
            best_d2 = d2;
            best = j;
            found_aligned = (pass == 0);
          }
        }
        if (pass == 0 && !found_aligned)
        {
          best_d2 = std::numeric_limits<double>::max();  // retry unfiltered
        }
      }
      std::rotate(loop.begin(), loop.begin() + static_cast<std::ptrdiff_t>(best), loop.end());
      loop.push_back(loop.front());
    }
    segs.push_back(std::move(loop));
  }

  // Nearest-endpoint chaining of the swath pieces. BoustrophedonOrder's
  // serpentine interleaves the pieces of a sweep line that a concave bite (or a
  // hole) split — below,above,below,above… — which forced one field-crossing
  // blade-on join PER COLUMN through/around the bite (user report: "the plan
  // lands in the middle at the rounded parts"). Greedy nearest-endpoint chaining
  // is identical to the plain serpentine on a convex field (the nearest unvisited
  // piece IS the adjacent swath, entered at the near end), but on a split field
  // it mows each lobe contiguously and leaves ONE long hop per lobe change —
  // which the join-gap split below turns into a single blade-off Nav2 transit.
  // Deterministic for a fixed plan (greedy from a fixed seed). O(n²), n = a few
  // hundred swaths at most.
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> ordered_swaths;
  {
    const auto& sw_in = plan.swaths;
    std::vector<bool> used(sw_in.size(), false);
    // Seed from BoustrophedonOrder's OWN first swath, NOT from the last ring's
    // end. Seeding from the ring end couples the serpentine direction to where
    // the ring closure happens to sit (the mid-longest-edge rotation above moved
    // it), and a flipped serpentine relocates every U-turn to the opposite swath
    // ends — where the turn-around teardrop may no longer fit inside the safety
    // inset (measured on the recorded garden: 7 extra cusp-splits from bottom-
    // edge U-turns that no longer had headland room). BoustrophedonOrder's own
    // start reproduces the field-proven serpentine regardless of ring layout;
    // the ring→first-swath hop is then joined blade-on when short or bridged by
    // one Nav2 transit when long.
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
        const double d0 =
            std::hypot(sw_in[i].first.first - cur.first, sw_in[i].first.second - cur.second);
        const double d1 =
            std::hypot(sw_in[i].second.first - cur.first, sw_in[i].second.second - cur.second);
        if (d0 < best_d)
        {
          best_d = d0;
          best = i;
          flip = false;
        }
        if (d1 < best_d)
        {
          best_d = d1;
          best = i;
          flip = true;
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
  // this to "fit in-bounds" produced loops MPPI could not track (wz≈vx/r), so the
  // robot looped/hesitated; when no arc >= this fits, buildConnector falls back
  // to a straight join instead of an untrackable loop.
  const double min_radius = std::max(0.02, min_turn_radius);

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
      // A blade-on connector is a TURN-AROUND between adjacent passes (~op_width
      // apart). A join longer than this is a RELOCATION (lobe change across a
      // concave bite, innermost-ring → far first swath): an in-bounds Dubins for
      // it "works" but mows a long diagonal across the middle of the lawn (user
      // report). Split instead — FollowStrip bridges it with a blade-off Nav2
      // transit. Single-sourced from mowgli_interfaces so this matches the BT's
      // FollowStrip::kSegmentTransitGap for the same decision — see
      // coverage_geometry.hpp for why the two sides must agree.
      const double join_gap = std::hypot(goal.x - start.x, goal.y - start.y);
      bool conn_safe = false;
      if (join_gap <= mowgli_interfaces::coverage_geometry::kSegmentTransitGapM)
      {
        bool fallback = false;
        auto conn = buildConnector(
            start, goal, boundary, plan.safe_holes, turn_radius, min_radius, step, fallback);
        // A forward Dubins connector is a LOCAL arc — it cannot route around a
        // large interior hole. If no in-bounds, hole-free arc fit, buildConnector
        // returned a straight fallback; keep it only when that straight is itself
        // safe (a short join that happens to clear everything — its V-cusp is
        // rounded by roundSharpCorners below when a fillet >= min_radius fits, else
        // left as a sharp corner FTC pivots through). Otherwise BREAK the path here:
        // finalize this sub-path and start a fresh one at segs[i], so FollowStrip
        // bridges the gap with a blade-off, costmap-aware Nav2 transit that
        // routes around the obstacle (issue #333).
        conn_safe =
            !conn.empty() &&
            (!fallback || (allInside(conn, boundary) && clearOfHoles(conn, plan.safe_holes)));
        if (conn_safe)
        {
          // conn is start-inclusive (== path.back()) / goal-exclusive; drop the
          // duplicate start so the polyline stays simple.
          path.insert(path.end(), conn.begin() + 1, conn.end());
        }
      }
      if (!conn_safe)
      {
        subpaths.push_back(std::move(path));
        path.clear();
      }
    }
    // Append this segment. When continuing a sub-path, path.back() (== goal of
    // the connector) coincides with segs[i].front(), so skip the first point to
    // avoid a zero-length step; at a fresh sub-path start, take the whole segment.
    const auto& s = segs[i];
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
  }

  // Round ONLY the true cusps — corners that exceed the 90° inversion limit
  // (e.g. the recorded boundary's ~91° acute vertex the rings inherit). We do
  // NOT fillet gentle (≤88°) ring corners: those are not cusps (MPPI tracks a
  // one-directional ≤90° turn fine, just slowing a bit), and filleting them near
  // the boundary forced the radius down to ~0.02 m — an arc far too tight for
  // MPPI to track (wz≈vx/r), so the robot looped/hesitated at corners it used to
  // turn through cleanly. 88° (not 90°) leaves a small margin so every corner
  // findFirstPathInversion would flag (>90°) is still rounded. The fillet radius
  // is floored at min_radius (= min_turn_radius): a corner that can only be
  // rounded tighter than that is left sharp (pivoted through) rather than turned
  // into a sub-trackable loop. Applied PER sub-path (each is independently
  // continuous; a fillet must never span a break).
  constexpr double kCornerThreshold = 88.0 * M_PI / 180.0;
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
    // Emit each rounded sub-path WHOLE — sub-paths split ONLY at Split A (a real
    // obstacle-gap / relocation), never at an interior cusp. FTC (restored
    // 2026-06-19, reverting MPPI) tracks the continuous full_path through the
    // forward turn-around arcs with a single PRE_ROTATE, so a residual sharp
    // U-turn between antiparallel swaths that no forward teardrop can fit (op_width
    // ~0.16 m apart, needs ~2·r) must stay in ONE sub-path: its cusp tip lies in
    // already-mowed headland (no coverage lost), and cutting there would fragment
    // the plan into a blade-off Nav2 transit PER U-turn (measured 18 sub-paths /
    // 17 transits on a 1-hole 72 m² field). The old MPPI-era residual-cusp split
    // (>115° corners) was removed with the MPPI revert.
    if (rounded.size() >= 2)
    {
      out.emplace_back(std::move(rounded));
    }
  }

  // Sub-path DRIVE ORDER: minimize the blade-off Nav2 transit BETWEEN sub-paths.
  // The sub-paths above come out in swath-chain order; on a multi-hole field that
  // leaves the driver criss-crossing the lawn between lobes (measured 77 m of
  // blade-off transit on the recorded 4-hole garden). Greedy nearest-neighbour
  // over the FINISHED sub-path polylines, entering each at whichever end is
  // nearer, mows spatially adjacent lobes consecutively (77 → 47 m there).
  //   * Reversing a FINISHED polyline is safe: the points are identical, so every
  //     turn-around / fillet stays exactly as in-bounds and trackable as before —
  //     only reversing the swath ORDER *before* the path is built relocates
  //     U-turns (the hazard the seed-from-BoustrophedonOrder note above guards);
  //     driving the same polyline backwards moves nothing.
  //   * Sub-path 0 stays the seed — TransitToStrip already drives the robot to
  //     its start, and the BT resumes by sub-path index (deterministic: a fixed
  //     plan yields a fixed NN order, so indices are stable across re-plans).
  //   * Adopt the NN order ONLY when it actually shortens the transit, so a field
  //     the chain order already sequenced well can never regress.
  if (out.size() > 1)
  {
    auto gap = [](const std::pair<double, double>& a, const std::pair<double, double>& b)
    {
      return std::hypot(a.first - b.first, a.second - b.second);
    };
    double chain_transit = 0.0;
    for (std::size_t i = 1; i < out.size(); ++i)
    {
      chain_transit += gap(out[i - 1].back(), out[i].front());
    }
    std::vector<std::size_t> order{0};
    std::vector<bool> reversed_flag(out.size(), false);
    std::vector<bool> used(out.size(), false);
    used[0] = true;
    std::pair<double, double> cur = out[0].back();
    double nn_transit = 0.0;
    for (std::size_t n = 1; n < out.size(); ++n)
    {
      std::size_t best = 0;
      bool best_rev = false;
      double best_d = std::numeric_limits<double>::max();
      for (std::size_t j = 0; j < out.size(); ++j)
      {
        if (used[j])
        {
          continue;
        }
        const double ds = gap(cur, out[j].front());  // enter forward
        const double de = gap(cur, out[j].back());  // enter reversed
        if (ds < best_d)
        {
          best_d = ds;
          best = j;
          best_rev = false;
        }
        if (de < best_d)
        {
          best_d = de;
          best = j;
          best_rev = true;
        }
      }
      used[best] = true;
      reversed_flag[best] = best_rev;
      order.push_back(best);
      nn_transit += best_d;
      cur = best_rev ? out[best].front() : out[best].back();
    }
    if (nn_transit + 1e-6 < chain_transit)
    {
      std::vector<std::vector<std::pair<double, double>>> reordered;
      reordered.reserve(out.size());
      for (const std::size_t idx : order)
      {
        std::vector<std::pair<double, double>> sp = std::move(out[idx]);
        if (reversed_flag[idx])
        {
          std::reverse(sp.begin(), sp.end());
        }
        reordered.push_back(std::move(sp));
      }
      out = std::move(reordered);
    }
  }
  return out;
}

std::vector<std::pair<double, double>> buildContinuousPath(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step)
{
  // Concatenate the hole-free sub-paths into one polyline (GUI full_path + the
  // no-hole common case, where there is exactly one sub-path). The driver uses
  // buildContinuousSubPaths so any inter-sub-path gap becomes a Nav2 transit.
  const auto subs = buildContinuousSubPaths(plan, boundary, turn_radius, min_turn_radius, step);
  std::vector<std::pair<double, double>> path;
  for (const auto& sp : subs)
  {
    path.insert(path.end(), sp.begin(), sp.end());
  }
  return path;
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

f2c::types::LinearRing dedupClosedRing(const f2c::types::LinearRing& in)
{
  f2c::types::LinearRing out;
  bool have_prev = false;
  double px = 0.0, py = 0.0;
  for (std::size_t i = 0; i < in.size(); ++i)
  {
    const auto p = in.getGeometry(i);
    const double x = p.getX();
    const double y = p.getY();
    if (have_prev && std::fabs(x - px) < 1e-9 && std::fabs(y - py) < 1e-9)
    {
      continue;  // zero-length edge — boost/F2C would reject the ring
    }
    out.addPoint(f2c::types::Point(x, y));
    px = x;
    py = y;
    have_prev = true;
  }
  // Close the ring (F2C wants first == last). The closing seam is the ring's
  // standard representation, not a degenerate interior edge.
  if (out.size() >= 2)
  {
    const auto first = out.getGeometry(0);
    const auto last = out.getGeometry(out.size() - 1);
    if (std::fabs(first.getX() - last.getX()) > 1e-9 ||
        std::fabs(first.getY() - last.getY()) > 1e-9)
    {
      out.addPoint(f2c::types::Point(first.getX(), first.getY()));
    }
  }
  return out;
}

}  // namespace mowgli_coverage
