// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

#ifndef MOWGLI_MAP__MAP_TYPES_HPP_
#define MOWGLI_MAP__MAP_TYPES_HPP_

#include <cstdint>
#include <string_view>

namespace mowgli_map
{

/// Classification type stored per cell in the "classification" grid_map layer.
/// Values are cast to float when written into the layer and cast back when read.
enum class CellType : uint8_t
{
  UNKNOWN = 0,  ///< Not yet classified
  LAWN = 1,  ///< Mowable lawn area
  OBSTACLE_PERMANENT = 2,  ///< Static obstacle (wall, tree, etc.)
  OBSTACLE_TEMPORARY = 3,  ///< Dynamic obstacle (detected at runtime)
  NO_GO_ZONE = 4,  ///< Operator-defined exclusion zone
  DOCKING_AREA = 5,  ///< Charging station vicinity
  /// Cell promoted to "permanently unreachable" by the segment-based
  /// coverage selector after fail_count exceeds dead_promote_threshold
  /// (typically 3). Distinct from OBSTACLE_PERMANENT (which is sensor-
  /// observed) so we can decay LAWN_DEAD back to LAWN after a
  /// configurable timeout — sensor-observed obstacles persist until
  /// the obstacle_tracker says otherwise.
  LAWN_DEAD = 6,
};

/// Human-readable name for a CellType value (useful for logging / debug).
constexpr std::string_view cell_type_name(CellType t) noexcept
{
  switch (t)
  {
    case CellType::UNKNOWN:
      return "UNKNOWN";
    case CellType::LAWN:
      return "LAWN";
    case CellType::OBSTACLE_PERMANENT:
      return "OBSTACLE_PERMANENT";
    case CellType::OBSTACLE_TEMPORARY:
      return "OBSTACLE_TEMPORARY";
    case CellType::NO_GO_ZONE:
      return "NO_GO_ZONE";
    case CellType::DOCKING_AREA:
      return "DOCKING_AREA";
    case CellType::LAWN_DEAD:
      return "LAWN_DEAD";
    default:
      return "INVALID";
  }
}

/// Named layer strings used throughout the package.
namespace layers
{
constexpr std::string_view OCCUPANCY = "occupancy";
constexpr std::string_view CLASSIFICATION = "classification";
}  // namespace layers

/// Default values written when a layer is initialised or cleared.
namespace defaults
{
constexpr float OCCUPANCY = 0.0F;  ///< free space
constexpr float CLASSIFICATION = 0.0F;  ///< CellType::UNKNOWN
}  // namespace defaults

}  // namespace mowgli_map

#endif  // MOWGLI_MAP__MAP_TYPES_HPP_
