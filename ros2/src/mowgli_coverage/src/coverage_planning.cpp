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
#include <string>

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
    if (!pts.empty() && allInside(pts, boundary))
    {
      return pts;  // largest in-bounds radius first → smoothest that fits
    }
  }
  // Last resort: straight blind connector (may be out-of-bounds → real gap).
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
      if (arc.empty() || !allInside(arc, boundary))
      {
        continue;
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

}  // namespace

BoustrophedonPlan planBoustrophedon(const f2c::types::Cell& field_cell,
                                    double op_width,
                                    double headland_width,
                                    int num_headland_passes_override,
                                    double chassis_safety_inset,
                                    double mow_angle_rad,
                                    double min_swath_length)
{
  BoustrophedonPlan plan;
  // Polygon area the planned-coverage fraction is taken over (the operator's
  // authorised area, before any inset). Instrumentation only.
  plan.diagnostics.field_area = field_cell.area();

  f2c::hg::ConstHL hl;
  f2c::types::Cells cells;
  cells.addGeometry(field_cell);

  // (1) Chassis-safety pre-inset: everything (rings AND swaths) is planned at
  // least this far inside the operator polygon, so tracking error can't push
  // the chassis over the boundary.
  f2c::types::Cells safe_cells = cells;
  if (chassis_safety_inset > 1e-3)
  {
    safe_cells = hl.generateHeadlands(cells, chassis_safety_inset);
    if (safe_cells.size() == 0 || safe_cells.area() < 1e-6)
    {
      return plan;  // inset consumed the field
    }
    // Expose the inset outer ring so the continuous-path connectors/fillets are
    // bounded by the SAME polygon the rings/swaths are planned against (not the
    // raw operator boundary). If the inset split the field, take the largest
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

std::vector<std::pair<double, double>> buildContinuousPath(
    const BoustrophedonPlan& plan,
    const std::vector<std::pair<double, double>>& boundary,
    double turn_radius,
    double min_turn_radius,
    double step)
{
  // Flatten the plan into ordered drivable segments (densified polylines),
  // rings first (outermost → inner) then serpentine swaths.
  std::vector<std::vector<std::pair<double, double>>> segs;
  for (const auto& loop : plan.rings)
  {
    if (loop.size() >= 2)
    {
      segs.push_back(loop);
    }
  }
  for (const auto& sw : plan.swaths)
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

  std::vector<std::pair<double, double>> path;
  if (segs.empty())
  {
    return path;
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
  // to a straight join (reportable gap) instead of an untrackable loop.
  const double min_radius = std::max(0.02, min_turn_radius);

  for (std::size_t i = 0; i < segs.size(); ++i)
  {
    if (i > 0)
    {
      // Forward turn-around connector from the previous segment's exit pose to
      // this segment's entry pose. Both are oriented, so the Dubins path leaves
      // and arrives tangent → no cusp at either junction.
      const Pose start{path.back().first, path.back().second, exitHeading(segs[i - 1])};
      const Pose goal{segs[i].front().first, segs[i].front().second, entryHeading(segs[i])};
      bool fallback = false;
      auto conn = buildConnector(start, goal, boundary, turn_radius, min_radius, step, fallback);
      // conn is start-inclusive (== path.back()) / goal-exclusive; drop the
      // duplicate start so the polyline stays simple.
      if (!conn.empty())
      {
        path.insert(path.end(), conn.begin() + 1, conn.end());
      }
    }
    // Append this segment. path.back() (== goal of the connector) coincides
    // with segs[i].front(), so skip the first point to avoid a zero-length step.
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
  // into a sub-trackable loop.
  constexpr double kCornerThreshold = 88.0 * M_PI / 180.0;
  const double fillet_r = std::max(turn_radius * 0.5, min_radius);
  path = roundSharpCorners(path, boundary, kCornerThreshold, fillet_r, min_radius, step);

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
