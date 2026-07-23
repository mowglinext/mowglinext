// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// WGS84 ↔ local ENU (map frame) equirectangular projection — the ONE
// projection convention the whole stack uses (CLAUDE.md Inv 4: map frame =
// GPS frame, X=east, Y=north, no rotation):
//
//   east  = (lon - datum_lon) * cos(datum_lat) * METERS_PER_DEG
//   north = (lat - datum_lat) * METERS_PER_DEG
//
// Accurate to ~1 cm within 10 km of the datum. This header mirrors the
// projection implemented in mowgli_localization's
// navsat_to_absolute_pose_node.cpp (wgs84_to_enu) and the GUI's
// gui/web/src/utils/map.tsx (transpose/itranspose) — any change here must
// be reflected there, and vice versa.
//
// Shared home: mowgli_interfaces, so consumers in other packages (e.g.
// mowgli_map's datum-change migration) reproject with EXACTLY the same
// math the localizer uses to place the robot.

#pragma once

#include <cmath>

namespace mowgli_interfaces::wgs84
{

/// WGS84 equatorial radius in metres.
constexpr double EARTH_RADIUS_M = 6378137.0;

/// Degrees to radians.
constexpr double DEG_TO_RAD = M_PI / 180.0;

/// Metres per degree of latitude (approximate, at the equator).
constexpr double METERS_PER_DEG = EARTH_RADIUS_M * DEG_TO_RAD;

/// Project a WGS84 coordinate into datum-relative ENU metres.
inline void ToEnu(double lat,
                  double lon,
                  double datum_lat,
                  double datum_lon,
                  double& east,
                  double& north)
{
  east = (lon - datum_lon) * std::cos(datum_lat * DEG_TO_RAD) * METERS_PER_DEG;
  north = (lat - datum_lat) * METERS_PER_DEG;
}

/// Invert ToEnu: recover the WGS84 coordinate of a datum-relative ENU point.
inline void FromEnu(double east,
                    double north,
                    double datum_lat,
                    double datum_lon,
                    double& lat,
                    double& lon)
{
  lat = datum_lat + north / METERS_PER_DEG;
  lon = datum_lon + east / (std::cos(datum_lat * DEG_TO_RAD) * METERS_PER_DEG);
}

/// Re-express a map-frame point recorded against one datum in the frame of
/// another datum: ENU → WGS84 under the old datum, WGS84 → ENU under the
/// new one. For a within-garden datum move this is (to first order) a pure
/// translation, so headings/yaws recorded in the old frame stay valid.
inline void ReprojectEnu(double old_datum_lat,
                         double old_datum_lon,
                         double new_datum_lat,
                         double new_datum_lon,
                         double& east,
                         double& north)
{
  double lat{};
  double lon{};
  FromEnu(east, north, old_datum_lat, old_datum_lon, lat, lon);
  ToEnu(lat, lon, new_datum_lat, new_datum_lon, east, north);
}

}  // namespace mowgli_interfaces::wgs84
