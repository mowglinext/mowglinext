// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// lattice_replay — run FTC's offset-lattice decision (ftc_lattice_solver.hpp,
// the same code FTCController::planOffsetLattice calls) offline on inputs
// recorded on the robot. An investigation tool, not a test: it is built with
// the tests and never installed.
//
// Inputs come from ros2/scripts/lattice_replay_dump.py, which writes one
// directory per /local_costmap/costmap message inside the requested windows:
//   <root>/<window>/<k>/meta.txt   t, local + global grid geometry, map->odom,
//                                  odom->base_footprint, FTC's carrot
//                                  (global_point, map frame), plan id
//   <root>/<window>/<k>/local.bin  /local_costmap/costmap data (int8)
//   <root>/<window>/<k>/global.bin /global_costmap/costmap data (int8), updates applied
//   <root>/plans/<id>.txt          FTC's plan as it published it: x y yaw (map)
//   <root>/keepout.txt|.bin        /keepout_mask, when the bag carries it
//
// Usage: lattice_replay [options] <sample_dir>...
//   --offset M      lateral offset FTC had applied (lateral_deviation_), default 0
//   --side S        side committed to while avoiding: 1 left, -1 right, 0 none
//   --max-offset M  max_lateral_deviation (the robot injects 1.0 by default)
//   --lead M        carrot lead cap, default CarrotMaxLead(0, 0.20, 1.0) = 0.30
//   --clearance M   obstacle_clearance_margin, default 0.05
//   --pad M         footprint margin around the 0.60 x 0.45 chassis, default 0.06
//                   (0.05 chassis_footprint margin + Nav2 footprint_padding 0.01)
//   --no-zone       leave the zone guard out
//   --keepout-zone  zone guard from /keepout_mask only (no LiDAR layer)
//   -v              print the node grid for every sample, not only the blocked ones
//
// Output per sample: FEASIBLE/WEDGED, fallback level, planned horizon; for a
// degraded or wedged solve, one row per station (left = +max .. right = -max):
// '.' free, '|' free on the line, 'O' local lethal cell under the body,
// 'Z' zone guard. Left column: the single pose; right column: the span the DP
// tests at fallback level 2.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_nav2_plugins/ftc_carrot_lead.hpp"
#include "mowgli_nav2_plugins/ftc_lattice_solver.hpp"
#include "mowgli_nav2_plugins/ftc_offset_lattice.hpp"
#include "mowgli_nav2_plugins/ftc_pivot.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mn = mowgli_nav2_plugins;

namespace
{

struct Options
{
  double offset{0.0};
  int side{0};
  double max_offset{1.0};
  double lead{mn::CarrotMaxLead(0.0, 0.20, 1.0)};
  double clearance{0.05};
  double pad{0.06};
  bool no_zone{false};
  bool keepout_zone{false};
  bool verbose{false};
  std::vector<std::string> samples;
};

struct Grid
{
  unsigned int w{0};
  unsigned int h{0};
  double res{0.0};
  double ox{0.0};
  double oy{0.0};
};

struct Sample
{
  double t{0.0};
  Grid local;
  Grid global;
  double mo_x{0.0};
  double mo_y{0.0};
  double mo_yaw{0.0};
  double carrot_x{0.0};
  double carrot_y{0.0};
  bool has_carrot{false};
  int plan{-1};
};

constexpr double kOffsetStep = 0.05;  // deviation_step
constexpr double kMaxSlope = 1.0;  // avoidance_max_slope
constexpr double kHorizon = 2.5;  // avoidance_horizon_m
constexpr double kReaction = 0.5;  // avoidance_reaction_m
constexpr double kMinHorizon = 1.0;  // avoidance_min_horizon_m

bool ParseArgs(int argc, char** argv, Options& o)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    const auto value = [&]()
    {
      return i + 1 < argc ? std::stod(argv[++i]) : 0.0;
    };
    if (a == "--offset")
      o.offset = value();
    else if (a == "--side")
      o.side = static_cast<int>(value());
    else if (a == "--max-offset")
      o.max_offset = value();
    else if (a == "--lead")
      o.lead = value();
    else if (a == "--clearance")
      o.clearance = value();
    else if (a == "--pad")
      o.pad = value();
    else if (a == "--no-zone")
      o.no_zone = true;
    else if (a == "--keepout-zone")
      o.keepout_zone = true;
    else if (a == "-v")
      o.verbose = true;
    else if (!a.empty() && a[0] == '-')
      return false;
    else
      o.samples.push_back(a);
  }
  return !o.samples.empty();
}

Sample ReadSample(const std::string& dir)
{
  Sample m;
  std::ifstream in(dir + "/meta.txt");
  std::string line;
  while (std::getline(in, line))
  {
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "t")
      ss >> m.t;
    else if (key == "local")
      ss >> m.local.w >> m.local.h >> m.local.res >> m.local.ox >> m.local.oy;
    else if (key == "global")
      ss >> m.global.w >> m.global.h >> m.global.res >> m.global.ox >> m.global.oy;
    else if (key == "map_odom")
      ss >> m.mo_x >> m.mo_y >> m.mo_yaw;
    else if (key == "carrot")
    {
      double stamp = 0.0;
      double yaw = 0.0;
      ss >> stamp >> m.carrot_x >> m.carrot_y >> yaw;
      m.has_carrot = true;
    }
    else if (key == "plan")
      ss >> m.plan;
  }
  return m;
}

std::vector<signed char> ReadBin(const std::string& path, std::size_t n)
{
  std::vector<signed char> v(n, 0);
  std::ifstream in(path, std::ios::binary);
  in.read(reinterpret_cast<char*>(v.data()), static_cast<std::streamsize>(n));
  return v;
}

/// OccupancyGrid value back to a Costmap2D cost (inverse of Nav2's publisher
/// table; exact for the values the lattice thresholds on: 100 -> 254 lethal).
unsigned char ToCost(signed char v)
{
  if (v < 0)
    return 255;
  if (v >= 100)
    return 254;
  if (v == 99)
    return 253;
  if (v == 0)
    return 0;
  return static_cast<unsigned char>(1 + (static_cast<int>(v) - 1) * 251 / 97);
}

geometry_msgs::msg::PoseStamped MakePose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

double YawOf(const geometry_msgs::msg::PoseStamped& p)
{
  const auto& q = p.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

const std::vector<geometry_msgs::msg::PoseStamped>& LoadPlan(const std::string& root, int id)
{
  static std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> cache;
  const std::string path = root + "/plans/" + std::to_string(id) + ".txt";
  auto it = cache.find(path);
  if (it != cache.end())
    return it->second;
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line))
  {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream ss(line);
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    ss >> x >> y >> yaw;
    plan.push_back(MakePose(x, y, yaw));
  }
  return cache.emplace(path, std::move(plan)).first->second;
}

/// The 0.60 x 0.45 chassis centred 0.18 m ahead of base_link, grown by `pad`
/// (getRobotFootprint()), then widened by the clearance margin like
/// FTCController::updateLateralDeviation does for the lattice.
mn::ObstacleDeviation::Footprint LatticeBody(const Options& o)
{
  const double front = 0.48 + o.pad;
  const double rear = -0.12 - o.pad;
  const double half = 0.225 + o.pad;
  mn::ObstacleDeviation::Footprint fp;
  for (const auto& [x, y] : std::vector<std::pair<double, double>>{
           {front, half}, {front, -half}, {rear, -half}, {rear, half}})
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    fp.push_back(p);
  }
  return mn::ObstacleDeviation::expandFootprintLateral(fp, o.clearance);
}

/// FTC's boundary_costmap_: global cells >= 99 become lethal, the rest free.
void FillZone(const std::string& root,
              const std::string& dir,
              const Sample& m,
              const Options& o,
              nav2_costmap_2d::Costmap2D& zone)
{
  Grid g = m.global;
  std::string bin = dir + "/global.bin";
  if (o.keepout_zone)
  {
    std::ifstream kin(root + "/keepout.txt");
    kin >> g.w >> g.h >> g.res >> g.ox >> g.oy;
    bin = root + "/keepout.bin";
  }
  zone.resizeMap(g.w, g.h, g.res, g.ox, g.oy);
  const auto raw = ReadBin(bin, std::size_t{g.w} * g.h);
  for (std::size_t i = 0; i < raw.size(); ++i)
  {
    zone.getCharMap()[i] = raw[i] >= 99 ? 254 : 0;
  }
}

/// Index of the plan segment the recorded carrot lies on.
std::size_t CarrotIndex(const std::vector<geometry_msgs::msg::PoseStamped>& plan, const Sample& m)
{
  std::size_t best_idx = 0;
  double best = 1e18;
  for (std::size_t i = 0; i + 1 < plan.size(); ++i)
  {
    const double ax = plan[i].pose.position.x;
    const double ay = plan[i].pose.position.y;
    const double vx = plan[i + 1].pose.position.x - ax;
    const double vy = plan[i + 1].pose.position.y - ay;
    const double l2 = vx * vx + vy * vy;
    const double u =
        l2 > 1e-12 ? std::clamp(((m.carrot_x - ax) * vx + (m.carrot_y - ay) * vy) / l2, 0.0, 1.0)
                   : 0.0;
    const double d = std::hypot(ax + u * vx - m.carrot_x, ay + u * vy - m.carrot_y);
    if (d < best - 1e-9)
    {
      best = d;
      best_idx = i;
    }
  }
  return best_idx;
}

std::size_t FillLocal(const std::string& dir, const Sample& m, nav2_costmap_2d::Costmap2D& local)
{
  const auto raw = ReadBin(dir + "/local.bin", std::size_t{m.local.w} * m.local.h);
  std::size_t lethal = 0;
  for (std::size_t i = 0; i < raw.size(); ++i)
  {
    local.getCharMap()[i] = ToCost(raw[i]);
    lethal += raw[i] >= 100 ? 1 : 0;
  }
  return lethal;
}

/// The lattice window FTC resamples, with the pivot corners found on the plan
/// as FTC received it (its published copy carries one duplicated tail pose).
mn::LatticeWindow Window(const std::vector<geometry_msgs::msg::PoseStamped>& plan,
                         std::size_t carrot,
                         double ds,
                         const Options& o)
{
  std::vector<mn::PlanPose2D> p2d;
  for (std::size_t i = 0; i + 1 < plan.size(); ++i)
  {
    p2d.push_back({plan[i].pose.position.x, plan[i].pose.position.y, YawOf(plan[i])});
  }
  const auto corners = mn::FindPivotCorners(p2d);
  const auto [leg_first, leg_last] = mn::PivotLeg(corners, carrot, plan.size());
  return mn::ResampleLatticeWindow(plan,
                                   carrot,
                                   leg_first,
                                   leg_last,
                                   mn::NextPivotCorner(corners, carrot),
                                   ds,
                                   o.lead,
                                   kHorizon);
}

/// Plan (map) -> costmap frame (odom), as planWindowInCostmapFrame does.
std::vector<geometry_msgs::msg::PoseStamped> WindowInOdom(
    const std::vector<geometry_msgs::msg::PoseStamped>& plan,
    const mn::LatticeWindow& w,
    const Sample& m)
{
  const double c = std::cos(m.mo_yaw);
  const double s = std::sin(m.mo_yaw);
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (const std::size_t i : w.pose_idx)
  {
    const double dx = plan[i].pose.position.x - m.mo_x;
    const double dy = plan[i].pose.position.y - m.mo_y;
    poses.push_back(MakePose(c * dx + s * dy, -s * dx + c * dy, YawOf(plan[i]) - m.mo_yaw));
  }
  return poses;
}

char BlockChar(mn::LatticeBlock b, bool on_line)
{
  if (b == mn::LatticeBlock::kObstacle)
    return 'O';
  if (b == mn::LatticeBlock::kZone)
    return 'Z';
  return on_line ? '|' : '.';
}

void PrintGrid(mn::LatticeSolver& solver, const Options& o)
{
  const int half = static_cast<int>(std::floor(o.max_offset / kOffsetStep + 1e-9));
  const auto& st = solver.Stations();
  std::printf("   station  s[m]   offsets +%.2f .. 0 .. -%.2f   (single pose | level-2 span)\n",
              o.max_offset,
              o.max_offset);
  for (std::size_t k = 0; k < st.size(); ++k)
  {
    std::string pose_row;
    std::string span_row;
    for (int j = half; j >= -half; --j)
    {
      const double off = j * kOffsetStep;
      pose_row += BlockChar(solver.PoseBlock(solver.CarrotPos() + k, off), j == 0);
      span_row += BlockChar(solver.SpanBlock(k, off, 0, 0), j == 0);
    }
    std::printf("   %3zu %6.2f  %s  %s\n", k, st[k], pose_row.c_str(), span_row.c_str());
  }
}

void ReplaySample(const std::string& dir, const Options& o)
{
  const std::string window = dir.substr(0, dir.find_last_of('/'));
  const std::string root = window.substr(0, window.find_last_of('/'));
  const Sample m = ReadSample(dir);
  const auto& plan = LoadPlan(root, m.plan);
  if (plan.size() < 3 || !m.has_carrot)
  {
    std::printf("%s: no plan or carrot\n", dir.c_str());
    return;
  }
  nav2_costmap_2d::Costmap2D local(m.local.w, m.local.h, m.local.res, m.local.ox, m.local.oy);
  const std::size_t lethal = FillLocal(dir, m, local);
  nav2_costmap_2d::Costmap2D zone(1, 1, 1.0, 0.0, 0.0);
  FillZone(root, dir, m, o, zone);
  mn::BoundaryGuard guard;  // boundary (map) <- costmap (odom) = map->odom
  guard.costmap = o.no_zone ? nullptr : &zone;
  guard.tx = m.mo_x;
  guard.ty = m.mo_y;
  guard.cos_yaw = std::cos(m.mo_yaw);
  guard.sin_yaw = std::sin(m.mo_yaw);

  const std::size_t carrot = CarrotIndex(plan, m);
  const double ds = mn::OffsetLatticeStationSpacing(kOffsetStep, kMaxSlope);
  const mn::LatticeWindow w = Window(plan, carrot, ds, o);
  std::vector<geometry_msgs::msg::PoseStamped> poses = WindowInOdom(plan, w, m);
  mn::LatticeSolverCfg cfg;
  cfg.lattice.offset_step = kOffsetStep;
  cfg.lattice.max_offset = o.max_offset;
  cfg.station_spacing_m = ds;
  cfg.lead_m = o.lead;
  cfg.reaction_m = kReaction;
  cfg.min_horizon_m = kMinHorizon;
  mn::LatticeSolver solver(
      local, guard, LatticeBody(o), std::move(poses), w.carrot_pos, w.corner_is_last_station, cfg);
  const mn::LatticeSolution sol = solver.Solve(o.offset, o.side, 0);
  const auto& st = solver.Stations();
  const double planned_m = sol.planned_stations > 0 ? st[sol.planned_stations - 1] : 0.0;
  std::printf(
      "%s t=%.2f plan=%d carrot_idx=%zu stations=%zu (%.2fm) corner=%d lethal=%zu -> %s "
      "level=%d planned=%zu (%.2fm) peak=%.2f\n",
      dir.c_str(),
      m.t,
      m.plan,
      carrot,
      st.size(),
      st.back(),
      w.corner_is_last_station ? 1 : 0,
      lethal,
      sol.plan.feasible ? "FEASIBLE" : "WEDGED",
      sol.level,
      sol.planned_stations,
      planned_m,
      sol.plan.feasible ? sol.plan.MaxAbsOffset() : 0.0);
  if (o.verbose || !sol.plan.feasible || sol.level > 1 || sol.planned_stations < st.size())
  {
    PrintGrid(solver, o);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  Options o;
  if (!ParseArgs(argc, argv, o))
  {
    std::fprintf(stderr,
                 "usage: %s [--offset M] [--side S] [--max-offset M] [--lead M] [--clearance M] "
                 "[--pad M] [--no-zone] [--keepout-zone] [-v] <sample_dir>...\n",
                 argv[0]);
    return 2;
  }
  for (const auto& dir : o.samples)
  {
    ReplaySample(dir, o);
  }
  return 0;
}
