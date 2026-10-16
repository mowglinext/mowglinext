// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Included ONLY after the pristine baseline implementation, in the same TU.
#pragma once
#include <bit>
#include <set>
#include <stdexcept>

#include "ring_contract.hpp"

namespace mowgli_coverage::replay_contract
{
namespace
{
using Json = nlohmann::json;
using Point = std::pair<double, double>;
using Path = std::vector<Point>;
using Paths = std::vector<Path>;
void require(bool value, const char* message)
{
  if (!value)
    throw std::runtime_error(std::string("certified fillets: ") + message);
}
double value(const Json& encoded)
{
  const auto text = encoded.get<std::string>();
  require(text.size() == 16 && text.find_first_not_of("0123456789abcdef") == std::string::npos,
          "invalid exact number");
  const auto number = std::bit_cast<double>(std::stoull(text, nullptr, 16));
  require(std::isfinite(number), "nonfinite number");
  return number;
}
Paths paths(const Json& data)
{
  require(data.is_array(), "missing geometry");
  Paths out;
  for (const auto& line : data)
  {
    require(line.is_array(), "invalid path");
    Path path;
    for (const auto& point : line)
    {
      require(point.is_array() && point.size() == 2, "invalid point");
      const auto x = value(point[0]), y = value(point[1]);
      require(std::abs(x) <= 1e6 && std::abs(y) <= 1e6, "coordinate outside replay range");
      path.emplace_back(x, y);
    }
    out.push_back(std::move(path));
  }
  return out;
}
const Json& stage(const Json& snapshot, const std::string& name)
{
  const Json* found = nullptr;
  for (const auto& item : snapshot.at("stages"))
    if (item.at("name") == name)
    {
      require(!found, "duplicate stage");
      found = &item;
    }
  require(found, "required stage absent");
  return *found;
}
bool same(const Point& a, const Point& b)
{
  return std::bit_cast<uint64_t>(a.first) == std::bit_cast<uint64_t>(b.first) &&
         std::bit_cast<uint64_t>(a.second) == std::bit_cast<uint64_t>(b.second);
}
bool same(const Path& a, const Path& b)
{
  return a.size() == b.size() && std::equal(a.begin(),
                                            a.end(),
                                            b.begin(),
                                            [](const auto& x, const auto& y)
                                            {
                                              return same(x, y);
                                            });
}
Point subtract(Point a, Point b)
{
  return {a.first - b.first, a.second - b.second};
}
double cross(Point a, Point b)
{
  return a.first * b.second - a.second * b.first;
}
double dot(Point a, Point b)
{
  return a.first * b.first + a.second * b.second;
}
double length(Point a)
{
  return std::hypot(a.first, a.second);
}
double turn(Point a, Point b)
{
  return std::acos(std::clamp(dot(a, b) / (length(a) * length(b)), -1.0, 1.0));
}
double signedArea(const Path& line)
{
  double area = 0;
  for (std::size_t i = 1; i < line.size(); ++i)
    area += cross(line[i - 1], line[i]);
  return area;
}
// Independent continuous containment check, including the tangent connectors.
void contains(const Path& line, const Path& boundary, const Paths& holes)
{
  OGRLineString route;
  for (const auto& p : line)
    route.addPoint(p.first, p.second);
  auto polygon = [](const Path& ring)
  {
    OGRLinearRing shell;
    for (const auto& p : ring)
      shell.addPoint(p.first, p.second);
    shell.closeRings();
    OGRPolygon result;
    result.addRing(&shell);
    return result;
  };
  auto shell = polygon(boundary);
  require(shell.IsValid(), "invalid reference boundary");
  std::unique_ptr<OGRGeometry> outside(route.Difference(&shell));
  require(outside && outside->IsEmpty(), "replacement crosses boundary");
  for (const auto& ring : holes)
  {
    auto hole = polygon(ring);
    require(hole.IsValid() && !route.Intersects(&hole), "replacement intersects hole");
  }
}
// Radius/tangency is measured from emitted coordinates, never a reported radius.
// 1e-8 is a numerical residual check; the existing radius regression uses 1e-6.
void supportedArc(const Path& local, Point a, Point corner, Point d, double minimum, double maximum)
{
  require(local.size() >= 5, "missing supported arc");
  const auto u0 = subtract(corner, a), v0 = subtract(d, corner);
  const double in = length(u0), out = length(v0);
  const Point u{u0.first / in, u0.second / in}, v{v0.first / out, v0.second / out};
  const double angle = turn(u, v), sign = cross(u, v) > 0 ? 1.0 : -1.0;
  const auto entry = local[1], exit = local[local.size() - 2];
  const auto trim = subtract(corner, entry);
  const double distance = dot(trim, u);
  require(std::abs(cross(trim, u)) < 1e-8 && distance > 0 && distance <= 0.49 * in + 1e-8 &&
              distance <= 0.49 * out + 1e-8,
          "invalid tangent trim");
  const double radius = distance * std::tan((M_PI - angle) / 2);
  require(radius >= minimum - 1e-6 && radius <= maximum + 1e-8, "unsupported radius");
  const Point center{entry.first - sign * u.second * radius,
                     entry.second + sign * u.first * radius};
  const auto tail = subtract(exit, corner);
  require(std::abs(cross(tail, v)) < 1e-8 && std::abs(dot(tail, v) - distance) < 1e-8,
          "exit is not tangent");
  double sweep = 0;
  for (std::size_t i = 1; i + 1 < local.size(); ++i)
  {
    const auto radial = subtract(local[i], center);
    require(std::abs(length(radial) - radius) < 1e-8, "noncircular fillet sample");
    if (i > 1)
    {
      const auto previous = subtract(local[i - 1], center);
      const double delta = sign * std::atan2(cross(previous, radial), dot(previous, radial));
      require(delta >= -1e-8 && delta <= cg::kPivotCornerMinTurnRad + 1e-8,
              "unsupported or reversed turn");
      sweep += delta;
    }
  }
  require(std::abs(sweep - angle) < 1e-8, "arc sweep differs from support turn");
}
Path validateIntermediate(const Path& source,
                          const Path& intermediate,
                          const Json& replacements,
                          const Path& boundary,
                          const Paths& holes,
                          double radius)
{
  const auto locals = paths(replacements.at("geometry"));
  const auto& indices = replacements.at("angles");
  require(indices.is_array() && indices.size() == locals.size() * 2, "invalid replacement indices");
  Path expected{source.front()};
  std::size_t replacement = 0;
  for (std::size_t i = 1; i + 1 < source.size(); ++i)
  {
    if (replacement == locals.size() || value(indices[2 * replacement]) != static_cast<double>(i))
    {
      expected.push_back(source[i]);
      continue;
    }
    const auto& local = locals[replacement];
    require(value(indices[2 * replacement + 1]) == static_cast<double>(expected.size()) &&
                i + 2 < source.size(),
            "replacement out of order/range");
    const auto a = expected.back(), b = source[i], c = source[i + 1], d = source[i + 2];
    const auto incoming = subtract(b, a), outgoing = subtract(d, c), bevel = subtract(c, b);
    const double il = length(incoming), ol = length(outgoing), bl = length(bevel);
    const double determinant = cross(incoming, outgoing);
    require(il >= cg::kPivotCornerMaxStepM && ol >= cg::kPivotCornerMaxStepM &&
                bl >= cg::kPivotCornerMaxStepM && bl <= 0.01 &&
                std::abs(determinant) > 1e-9 * il * ol &&
                determinant * cross(incoming, bevel) > 0 &&
                determinant * cross(bevel, outgoing) > 0,
            "not an ordinary same-sign microscopic bevel");
    const double t = cross(subtract(c, a), outgoing) / determinant;
    const Point corner{a.first + t * incoming.first, a.second + t * incoming.second};
    const double angle = turn(incoming, outgoing);
    require(t >= 1 && dot(subtract(corner, c), outgoing) <= 0 && angle > M_PI / 6 &&
                angle < M_PI - 1e-6 && length(subtract(corner, b)) <= 0.01 &&
                length(subtract(corner, c)) <= 0.01,
            "invalid supporting-edge intersection");
    // Reference sampling also rejects arbitrary extra/erased/shifted samples.
    const auto reference = roundSharpCorners(
        {a, corner, d}, boundary, holes, M_PI / 6, 2 * radius, std::max(0.02, radius), 0.03);
    require(same(local, reference), "local samples differ from pristine helper");
    supportedArc(local, a, corner, d, std::max(0.02, radius), std::max(0.02, 2 * radius));
    contains(local, boundary, holes);
    expected.insert(expected.end(), local.begin() + 1, local.end() - 1);
    ++replacement;
    ++i;
  }
  expected.push_back(source.back());
  require(replacement == locals.size() && same(expected, intermediate),
          "unaccounted vertex or pivot change");
  require(same(source.front(), intermediate.front()) && same(source.back(), intermediate.back()) &&
              signedArea(source) * signedArea(intermediate) > 0,
          "closure/start/winding changed");
  return expected;
}
}  // namespace

void applyCertifiedRings(BoustrophedonPlan& plan,
                         const Json& snapshot,
                         std::size_t field,
                         const Json& state,
                         const Json& parameters,
                         const Path& raw_boundary)
{
  const auto prefix = std::to_string(field) + ".";
  require(stage(snapshot, prefix + "plan_state").at("state") == state,
          "non-ring plan state changed");
  const auto proposed = paths(stage(snapshot, prefix + "plan_rings").at("geometry"));
  require(proposed.size() == plan.rings.size(), "ring count changed");
  require(stage(snapshot, prefix + "rings").at("geometry") ==
              stage(snapshot, prefix + "plan_rings").at("geometry"),
          "reported ring trace differs from returned rings");
  const auto& boundary = plan.safe_boundary.size() >= 3 ? plan.safe_boundary : raw_boundary;
  const double radius = parameters.at("min_turn_radius").get<double>();
  std::size_t evidence_count = 0;
  for (const auto& item : snapshot.at("stages"))
    if (item.at("name").get<std::string>().starts_with(prefix + "bevel_"))
      ++evidence_count;
  require(evidence_count == 4 * plan.rings.size(), "missing/extra ring provenance");
  for (std::size_t i = 0; i < plan.rings.size(); ++i)
  {
    const auto suffix = std::to_string(i);
    for (const auto* kind : {"bevel_source.", "bevel_intermediate.", "bevel_rounded."})
    {
      const auto& evidence = stage(snapshot, prefix + kind + suffix);
      require(evidence.size() == 3 && evidence.at("angles").is_array() &&
                  evidence.at("angles").empty(),
              "unexpected ring provenance fields");
    }
    const auto source_paths =
        paths(stage(snapshot, prefix + "bevel_source." + suffix).at("geometry"));
    const auto intermediate_paths =
        paths(stage(snapshot, prefix + "bevel_intermediate." + suffix).at("geometry"));
    const auto rounded_paths =
        paths(stage(snapshot, prefix + "bevel_rounded." + suffix).at("geometry"));
    require(source_paths.size() == 1 && intermediate_paths.size() == 1 && rounded_paths.size() == 1,
            "invalid ring provenance");
    const auto& source = source_paths[0];
    const auto& intermediate = intermediate_paths[0];
    const auto& rounded = rounded_paths[0];
    const auto& replacements = stage(snapshot, prefix + "bevel_replacements." + suffix);
    require(replacements.size() == 3 && replacements.at("geometry").is_array() &&
                replacements.at("angles").is_array(),
            "unexpected replacement provenance fields");
    if (source.empty())
    {
      require(intermediate.empty() && rounded.empty() && replacements.at("geometry").empty() &&
                  replacements.at("angles").empty() && same(proposed[i], plan.rings[i]),
              "bypassed smoothing changed ring");
      continue;
    }
    require(source.size() >= 5 && same(source.front(), source.back()), "source is not closed");
    const auto legacy = roundSharpCorners(
        source, boundary, plan.safe_holes, M_PI / 6, 2 * radius, std::max(0.02, radius), 0.03);
    require(same(densifyPolyline(legacy, 0.10), plan.rings[i]),
            "source does not reconstruct pristine baseline ring");
    validateIntermediate(source, intermediate, replacements, boundary, plan.safe_holes, radius);
    const auto expected = roundSharpCorners(intermediate,
                                            boundary,
                                            plan.safe_holes,
                                            M_PI / 6,
                                            2 * radius,
                                            std::max(0.02, radius),
                                            0.03);
    require(same(expected, rounded) && same(densifyPolyline(expected, 0.10), proposed[i]),
            "candidate ring does not reconstruct from certified replacements");
    // Ordinary rounding can retune a neighbouring corner after the bevel arc
    // shortens its supporting edge. Check the actual final emitted chords too.
    if (!same(proposed[i], plan.rings[i]))
      contains(proposed[i], boundary, plan.safe_holes);
  }
  plan.rings = proposed;
}

void verifyReferenceStages(const Json& snapshot,
                           const Json& baseline_stages,
                           std::size_t field,
                           std::size_t ring_count)
{
  const auto prefix = std::to_string(field) + ".";
  std::set<std::string> allowed;
  for (const auto& item : baseline_stages)
  {
    const auto name = item.at("name").get<std::string>();
    allowed.insert(name);
    auto expected = item;
    if (name == prefix + "rings")
      expected["geometry"] = stage(snapshot, prefix + "plan_rings").at("geometry");
    require(stage(snapshot, name) == expected, "unchanged planner stage differs from baseline");
  }
  for (const auto* suffix :
       {"plan_state", "plan_rings", "boundaries", "kept_swaths", "execution", "quaternions"})
    allowed.insert(prefix + suffix);
  for (std::size_t i = 0; i < ring_count; ++i)
    for (const auto* suffix :
         {"bevel_source.", "bevel_intermediate.", "bevel_rounded.", "bevel_replacements."})
      allowed.insert(prefix + suffix + std::to_string(i));
  std::set<std::string> seen;
  for (const auto& item : snapshot.at("stages"))
  {
    const auto name = item.at("name").get<std::string>();
    if (name.starts_with(prefix))
      require(allowed.contains(name) && seen.insert(name).second,
              "unrecognized or duplicate trace stage");
  }
  require(seen == allowed, "required planner stage absent");
}
}  // namespace mowgli_coverage::replay_contract
