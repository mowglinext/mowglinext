// Copyright (C) 2026 Cedric <cedric@mowgli.dev>

#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../fixtures/isabey.hpp"
#include "gdal.h"
#include "geos_c.h"
#include "mowgli_coverage/coverage_planning.hpp"
#include <nlohmann/json.hpp>

namespace
{
namespace coverage = mowgli_coverage;
using Json = nlohmann::json;
using Point = std::pair<double, double>;
using Path = std::vector<Point>;
using Paths = std::vector<Path>;

std::string hex(uint64_t value)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << value;
  return stream.str();
}

Json exact(const Paths& paths)
{
  Json result = Json::array();
  for (const auto& path : paths)
  {
    Json points = Json::array();
    for (const auto& [x, y] : path)
    {
      if (!std::isfinite(x) || !std::isfinite(y))
        throw std::runtime_error("planner emitted a non-finite coordinate");
      points.push_back({hex(std::bit_cast<uint64_t>(x)), hex(std::bit_cast<uint64_t>(y))});
    }
    result.push_back(std::move(points));
  }
  return result;
}

// Deliberately mirrors FollowStrip's existing POSITION-ONLY, mm-quantized hash.
// This is diagnostic evidence, never an authorization to reuse a resume cursor.
uint64_t fingerprint(const Paths& paths)
{
  uint64_t hash = 1469598103934665603ULL;
  auto mix = [&](uint64_t value)
  {
    for (int byte = 0; byte < 8; ++byte)
    {
      hash ^= value & 255;
      hash *= 1099511628211ULL;
      value >>= 8;
    }
  };
  mix(paths.size());
  for (const auto& path : paths)
  {
    mix(path.size());
    for (const auto& [x, y] : path)
    {
      // Keep llround in range even for an invalid report fixture.
      if (std::abs(x) > 1e12 || std::abs(y) > 1e12)
        throw std::runtime_error("coordinate exceeds replay fingerprint range");
      mix(static_cast<uint64_t>(static_cast<int64_t>(std::llround(x * 1000.0))));
      mix(static_cast<uint64_t>(static_cast<int64_t>(std::llround(y * 1000.0))));
    }
  }
  return hash;
}

Path points(const Json& values, bool float32)
{
  Path result;
  for (const auto& point : values)
  {
    if (!point.is_array() || point.size() != 2)
      throw std::runtime_error("each point must contain x and y");
    double x = point.at(0).get<double>();
    double y = point.at(1).get<double>();
    if (float32)
    {
      x = static_cast<float>(x);
      y = static_cast<float>(y);
    }
    if (!std::isfinite(x) || !std::isfinite(y) || std::abs(x) > 1e6 || std::abs(y) > 1e6)
      throw std::runtime_error("coordinates must be finite map-frame metres within +/-1e6");
    result.emplace_back(x, y);
  }
  if (result.size() < 3)
    throw std::runtime_error("ring requires at least three points");
  return result;
}

f2c::types::LinearRing ring(const Path& points)
{
  f2c::types::LinearRing result;
  for (const auto& [x, y] : points)
    result.addPoint(f2c::types::Point(x, y));
  return coverage::dedupClosedRing(result);
}

Paths cellPoints(const f2c::types::Cell& cell)
{
  Paths result;
  for (std::size_t r = 0; r < cell.size(); ++r)
  {
    Path path;
    const auto ring = cell.getGeometry(r);
    for (std::size_t p = 0; p < ring.size(); ++p)
    {
      const auto point = ring.getGeometry(p);
      path.emplace_back(point.getX(), point.getY());
    }
    result.push_back(std::move(path));
  }
  return result;
}

double positive(const Json& params, const char* key, bool allow_zero = false)
{
  const double value = params.at(key).get<double>();
  if (!std::isfinite(value) || (allow_zero ? value < 0 : value <= 0))
    throw std::runtime_error(std::string("invalid effective parameter: ") + key);
  return value;
}

Json replay(const Json& input, bool trace_enabled)
{
  if (input.at("schema") != 1)
    throw std::runtime_error("unsupported fixture schema");
  const std::string precision = input.at("precision").get<std::string>();
  if (precision != "float32" && precision != "float64")
    throw std::runtime_error("precision must be float32 or float64");
  const bool float32 = precision == "float32";
  const auto& params = input.at("parameters");
  const double width = positive(params, "operation_width");
  const double headland = positive(params, "headland_width", true);
  const double inset = positive(params, "chassis_safety_inset", true);
  const double min_length = positive(params, "min_swath_length", true);
  const double min_radius = positive(params, "min_turn_radius");
  const double turn_radius = positive(params, "turn_radius");
  const double step = positive(params, "step");
  const double obstacle_margin = positive(params, "obstacle_margin", true);
  const double sweep = positive(params, "pivot_sweep_radius", true);
  const double boundary_margin = positive(params, "boundary_margin", true);
  const double angle = params.at("mow_angle_rad").get<double>();
  if (!std::isfinite(angle))
    throw std::runtime_error("angle must be finite (negative means AUTO)");
  const int passes = params.at("num_headland_passes").get<int>();
  const int direction = params.at("ring_direction").get<int>();
  if (direction < 0 || direction > 2 || passes > 100)
    throw std::runtime_error("invalid headland count or winding");
  const bool perpendicular = params.at("perpendicular").get<bool>();
  const int max_passes = params.at("connector_max_headland_passes").get<int>();
  std::optional<Point> start;
  if (!params.at("start_hint").is_null())
  {
    const auto& hint = params.at("start_hint");
    if (!hint.is_array() || hint.size() != 2)
      throw std::runtime_error("start_hint must be null or [x,y]");
    start = Point{hint.at(0).get<double>(), hint.at(1).get<double>()};
    if (!std::isfinite(start->first) || !std::isfinite(start->second))
      throw std::runtime_error("start_hint must be finite");
  }

  Json stages = Json::array(), normalized = Json::array(), fields = Json::array();
  Json transported_polygons = Json::array();
  Paths all_paths;
  double length = 0.0, transit = 0.0;
  std::size_t poses = 0, swath_count = 0, ring_count = 0;
  coverage::ConnectorStats stats;
  double planning_ms = 0.0, connectors_ms = 0.0;
  for (std::size_t f = 0; f < input.at("polygons").size(); ++f)
  {
    const auto& polygon = input.at("polygons").at(f);
    if (polygon.empty())
      throw std::runtime_error("polygon requires an outer ring");
    Paths transported;
    for (const auto& values : polygon)
      transported.push_back(points(values, float32));
    transported_polygons.push_back(exact(transported));
    const auto outer = ring(transported.front());
    if (outer.size() < 4)
      throw std::runtime_error("outer ring collapsed during sanitization");
    f2c::types::Cell cell(outer);
    for (std::size_t h = 1; h < transported.size(); ++h)
      cell.addRing(coverage::bufferRingOutward(ring(transported[h]), obstacle_margin));
    normalized.push_back(exact(cellPoints(cell)));
    coverage::PivotJoinLimits limits;
    limits.sweep_radius = sweep;
    limits.boundary_margin = boundary_margin;
    limits.recorded_boundary = transported.front();
    limits.recorded_obstacles.assign(transported.begin() + 1, transported.end());
#ifndef REPLAY_BASELINE
    coverage::PlanningTrace trace;
    auto* observer = trace_enabled ? &trace : nullptr;
#else
    (void)trace_enabled;
#endif
    const auto t0 = std::chrono::steady_clock::now();
    const auto plan = coverage::planBoustrophedon(cell,
                                                  width,
                                                  headland,
                                                  passes,
                                                  inset,
                                                  angle,
                                                  min_length,
                                                  direction,
                                                  min_radius,
                                                  perpendicular,
                                                  max_passes,
                                                  start
#ifndef REPLAY_BASELINE
                                                  ,
                                                  observer
#endif
    );
    const auto t1 = std::chrono::steady_clock::now();
    const auto& boundary = plan.connector_clearance_boundary.size() >= 3
                               ? plan.connector_clearance_boundary
                           : plan.safe_boundary.size() >= 3 ? plan.safe_boundary
                                                            : transported.front();
    coverage::ConnectorStats field_stats;
    const auto paths = coverage::buildContinuousSubPaths(plan,
                                                         boundary,
                                                         turn_radius,
                                                         min_radius,
                                                         step,
                                                         &field_stats,
                                                         plan.swath_turn_envelope,
                                                         limits,
                                                         start.has_value() && !plan.rings.empty()
#ifndef REPLAY_BASELINE
                                                             ,
                                                         observer
#endif
    );
    if (paths.empty())
      throw std::runtime_error("a polygon produced no executable coverage");
    const auto t2 = std::chrono::steady_clock::now();
    planning_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
    connectors_ms += std::chrono::duration<double, std::milli>(t2 - t1).count();
#ifndef REPLAY_BASELINE
    for (const auto& stage : trace.stages)
    {
      Json angles = Json::array();
      for (double angle : stage.angles)
        angles.push_back(hex(std::bit_cast<uint64_t>(angle)));
      stages.push_back({{"name", std::to_string(f) + "." + stage.name},
                        {"geometry", exact(stage.paths)},
                        {"angles", angles}});
    }
#endif
    stages.push_back(
        {{"name", std::to_string(f) + ".boundaries"},
         {"geometry",
          exact({plan.safe_boundary, plan.connector_clearance_boundary, plan.swath_turn_envelope})},
         {"holes", exact(plan.safe_holes)}});
    Paths swaths;
    for (const auto& swath : plan.swaths)
      swaths.push_back({swath.first, swath.second});
    stages.push_back({{"name", std::to_string(f) + ".kept_swaths"},
                      {"geometry", exact(swaths)},
                      {"first_swath_heading", hex(std::bit_cast<uint64_t>(plan.swath_angle_rad))}});
    stages.push_back({{"name", std::to_string(f) + ".execution"}, {"geometry", exact(paths)}});
    Json headings = Json::array();
    for (const auto& path : paths)
    {
      Json angles = Json::array();
      for (double yaw : coverage::pathHeadings(path))
      {
        if (!std::isfinite(yaw))
          throw std::runtime_error("planner emitted a non-finite heading");
        // Match coverage_server's pose quaternion conversion.
        angles.push_back({hex(std::bit_cast<uint64_t>(std::sin(yaw * 0.5))),
                          hex(std::bit_cast<uint64_t>(std::cos(yaw * 0.5)))});
      }
      headings.push_back(std::move(angles));
      poses += path.size();
      for (std::size_t p = 1; p < path.size(); ++p)
        length +=
            std::hypot(path[p].first - path[p - 1].first, path[p].second - path[p - 1].second);
    }
    for (std::size_t p = 1; p < paths.size(); ++p)
      transit += std::hypot(paths[p].front().first - paths[p - 1].back().first,
                            paths[p].front().second - paths[p - 1].back().second);
    stages.push_back({{"name", std::to_string(f) + ".quaternions"}, {"geometry", headings}});
    fields.push_back({{"first_swath_heading_rad", plan.swath_angle_rad},
                      {"swaths", plan.swaths.size()},
                      {"rings", plan.rings.size()},
                      {"subpaths", paths.size()},
                      {"fingerprint", hex(fingerprint(paths))}});
    swath_count += plan.swaths.size();
    ring_count += plan.rings.size();
    stats.attempted += field_stats.attempted;
    stats.arc += field_stats.arc;
    stats.straight_kept += field_stats.straight_kept;
    stats.pivot += field_stats.pivot;
    stats.split += field_stats.split;
    all_paths.insert(all_paths.end(), paths.begin(), paths.end());
  }
  if (fields.empty() || poses == 0)
    throw std::runtime_error("fixture produced no executable coverage");
  return {{"schema", 1},
          {"normalized_polygons", normalized},
          {"transported_polygons", transported_polygons},
          {"parameters", params},
          {"stages", stages},
          {"fields", fields},
          {"metrics",
           {{"poses", poses},
            {"swaths", swath_count},
            {"rings", ring_count},
            {"subpaths", all_paths.size()},
            {"blade_off_transits", all_paths.size() - fields.size()},
            {"path_length_m", length},
            {"endpoint_transit_length_m", transit},
            {"attempted", stats.attempted},
            {"arc", stats.arc},
            {"straight_kept", stats.straight_kept},
            {"pivot", stats.pivot},
            {"split", stats.split}}},
          {"timing", {{"planning_ms", planning_ms}, {"connectors_ms", connectors_ms}}},
          {"followstrip_fingerprint", hex(fingerprint(all_paths))},
          {"build",
           {{"revision", REPLAY_REVISION},
            {"planner_source_sha256", REPLAY_SOURCE_SHA256},
            {"planner_header_sha256", REPLAY_HEADER_SHA256},
            {"compiler", REPLAY_COMPILER},
            {"configuration", REPLAY_BUILD_TYPE},
            {"gdal", GDALVersionInfo("RELEASE_NAME")},
            {"geos", GEOSversion()}}}};
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    if (argc == 2 && std::string(argv[1]) == "--isabey")
    {
      std::cout << Json(isabeyRings()).dump() << '\n';
      return 0;
    }
    if (argc < 2 || argc > 4)
      throw std::runtime_error("usage: coverage_replay fixture.json [repetitions] [--no-trace]");
    std::ifstream file(argv[1]);
    if (!file)
      throw std::runtime_error("cannot read fixture");
    const Json input = Json::parse(file);
    const int repetitions = argc >= 3 ? std::stoi(argv[2]) : 1;
    if (repetitions < 1 || repetitions > 100000)
      throw std::runtime_error("repetitions must be in [1,100000]");
    if (argc == 4 && std::string(argv[3]) != "--no-trace")
      throw std::runtime_error("unknown replay option");
    const bool trace = argc < 4;
    for (int i = 0; i < repetitions; ++i)
      std::cout << replay(input, trace).dump() << std::endl;
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
