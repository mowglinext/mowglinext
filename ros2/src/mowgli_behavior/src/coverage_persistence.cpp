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

#include "mowgli_behavior/coverage_persistence.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

#include "mowgli_behavior/bt_context.hpp"

namespace mowgli_behavior
{

namespace
{
// Bump when the on-disk layout changes incompatibly; an unrecognised header is
// treated as "no state" (start fresh) rather than a parse error.
constexpr const char* kHeader = "mowgli_coverage_resume v2";
}  // namespace

bool saveCoverageResumeState(const BTContext& ctx)
{
  if (ctx.coverage_resume_path.empty())
  {
    return false;
  }

  // Union of every area index that carries any resume-relevant state.
  std::set<uint32_t> areas;
  for (const auto& [idx, _] : ctx.area_path_pose_count)
    areas.insert(idx);
  for (const auto& [idx, _] : ctx.area_plan_fingerprint)
    areas.insert(idx);
  for (const auto& [idx, _] : ctx.area_resume_pose_index)
    areas.insert(idx);
  for (const auto& [idx, _] : ctx.area_completed_swaths)
    areas.insert(idx);
  for (uint32_t idx : ctx.completed_areas)
    areas.insert(idx);

  std::ostringstream out;
  out << kHeader << '\n';
  out << "current_area " << ctx.current_area << '\n';
  out << "completed_areas";
  for (uint32_t idx : ctx.completed_areas)
    out << ' ' << idx;
  out << '\n';
  for (uint32_t idx : areas)
  {
    std::size_t pose_count = 0;
    if (auto it = ctx.area_path_pose_count.find(idx); it != ctx.area_path_pose_count.end())
      pose_count = it->second;
    // Plan-geometry fingerprint (0 = none recorded). The resume-cursor staleness
    // key: a persisted cursor is only reused when this matches the re-planned
    // geometry (see FollowStrip::onStart / hashPlanGeometry).
    uint64_t fingerprint = 0;
    if (auto it = ctx.area_plan_fingerprint.find(idx); it != ctx.area_plan_fingerprint.end())
      fingerprint = it->second;
    // -1 sentinel = no live resume cursor (area finished or never interrupted).
    int64_t resume = -1;
    if (auto it = ctx.area_resume_pose_index.find(idx); it != ctx.area_resume_pose_index.end())
      resume = static_cast<int64_t>(it->second);

    out << "area " << idx << ' ' << pose_count << ' ' << fingerprint << ' ' << resume
        << " completed";
    if (auto it = ctx.area_completed_swaths.find(idx); it != ctx.area_completed_swaths.end())
      for (std::size_t s : it->second)
        out << ' ' << s;
    out << '\n';
  }

  // Atomic replace: write a sibling temp file then rename over the target so a
  // reader never sees a half-written file (mirrors persist_dock_pose_yaw).
  const std::string tmp_path = ctx.coverage_resume_path + ".tmp";
  {
    std::ofstream f(tmp_path, std::ios::trunc);
    if (!f)
    {
      return false;
    }
    f << out.str();
    f.flush();
    if (!f)
    {
      return false;
    }
  }
  return std::rename(tmp_path.c_str(), ctx.coverage_resume_path.c_str()) == 0;
}

bool loadCoverageResumeState(BTContext& ctx)
{
  if (ctx.coverage_resume_path.empty())
  {
    return false;
  }
  std::ifstream f(ctx.coverage_resume_path);
  if (!f)
  {
    return false;
  }

  std::string header;
  if (!std::getline(f, header) || header != kHeader)
  {
    return false;  // absent / empty / unknown version → start fresh
  }

  std::string line;
  while (std::getline(f, line))
  {
    std::istringstream ls(line);
    std::string tag;
    if (!(ls >> tag))
    {
      continue;
    }
    if (tag == "current_area")
    {
      int v;
      if (ls >> v)
        ctx.current_area = v;
    }
    else if (tag == "completed_areas")
    {
      uint32_t idx;
      while (ls >> idx)
        ctx.completed_areas.insert(idx);
    }
    else if (tag == "area")
    {
      uint32_t idx;
      std::size_t pose_count;
      uint64_t fingerprint;
      int64_t resume;
      std::string completed_tag;
      if (!(ls >> idx >> pose_count >> fingerprint >> resume >> completed_tag))
      {
        continue;  // malformed row — skip, don't abort the whole load
      }
      ctx.area_path_pose_count[idx] = pose_count;
      if (fingerprint != 0)
      {
        ctx.area_plan_fingerprint[idx] = fingerprint;
      }
      if (resume >= 0)
      {
        ctx.area_resume_pose_index[idx] = static_cast<std::size_t>(resume);
      }
      auto& done = ctx.area_completed_swaths[idx];
      std::size_t s;
      while (ls >> s)
        done.insert(s);
    }
  }
  return true;
}

bool clearCoverageResumeState(const BTContext& ctx)
{
  if (ctx.coverage_resume_path.empty())
  {
    return true;
  }
  // std::remove returns non-zero if the file was already absent, which is fine.
  std::remove(ctx.coverage_resume_path.c_str());
  std::remove((ctx.coverage_resume_path + ".tmp").c_str());
  return true;
}

}  // namespace mowgli_behavior
