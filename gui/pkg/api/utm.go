package api

import "math"

// ---------------------------------------------------------------------------
// WGS84 UTM forward/inverse — a faithful Go port of the Snyder-series
// RobotLocalization::NavsatConversions (LLtoUTM / UTMtoLL) that OpenMower's
// xbot_driver_gps links to build its local map frame.
//
// OpenMower stores every map polygon vertex as a UTM *grid* easting/northing
// relative to its datum (LLtoUTM(fix) − LLtoUTM(datum); see
// xbot_driver_gps/src/interfaces/ublox_gps_interface.cpp). Its local axes are
// therefore aligned to UTM grid north, NOT true north. MowgliNext's map frame is
// a datum-centred, true-north equirectangular ENU (navsat_to_absolute_pose_node
// .cpp). The two differ by the UTM meridian convergence γ, which reaches 1–2° a
// couple of degrees off a zone's central meridian — the "imported map is rotated
// 1–2°" report. Faithfully importing an OpenMower map therefore means inverting
// this exact projection (UTM grid → lat/lon) and re-projecting into MowgliNext's
// true-north ENU.
//
// The series matches the C++ to sub-millimetre within a zone — far below the
// float32 precision the ROS polygon points are stored at.
// ---------------------------------------------------------------------------

const (
	utmWGS84A = 6378137.0    // WGS84 semi-major axis (m)
	utmWGS84E = 0.0818191908 // WGS84 first eccentricity
	utmK0     = 0.9996       // UTM scale factor at the central meridian
	utmFE     = 500000.0     // false easting
	utmFNS    = 10000000.0   // false northing, southern hemisphere
	utmDegRad = math.Pi / 180.0
	utmRadDeg = 180.0 / math.Pi
)

// utmZoneCentralMeridianDeg returns the longitude (degrees) of the central
// meridian of the given UTM zone number.
func utmZoneCentralMeridianDeg(zone int) float64 {
	return float64((zone-1)*6 - 180 + 3)
}

// llToUTM converts WGS84 lat/lon (degrees) to UTM, returning northing/easting
// (metres), the zone number, and whether the point is in the northern
// hemisphere. Mirrors RobotLocalization::NavsatConversions::LLtoUTM.
func llToUTM(lat, lon float64) (northing, easting float64, zone int, north bool) {
	eccSq := utmWGS84E * utmWGS84E

	// Normalise longitude to [-180, 180).
	lonTemp := lon + 180 - math.Floor((lon+180)/360)*360 - 180
	latRad := lat * utmDegRad
	lonRad := lonTemp * utmDegRad

	zone = int((lonTemp+180)/6) + 1
	// Norway/Svalbard exceptions (kept for parity with the reference impl).
	if lat >= 56.0 && lat < 64.0 && lonTemp >= 3.0 && lonTemp < 12.0 {
		zone = 32
	}
	if lat >= 72.0 && lat < 84.0 {
		switch {
		case lonTemp >= 0.0 && lonTemp < 9.0:
			zone = 31
		case lonTemp >= 9.0 && lonTemp < 21.0:
			zone = 33
		case lonTemp >= 21.0 && lonTemp < 33.0:
			zone = 35
		case lonTemp >= 33.0 && lonTemp < 42.0:
			zone = 37
		}
	}
	lonOriginRad := utmZoneCentralMeridianDeg(zone) * utmDegRad

	eccPrimeSq := eccSq / (1 - eccSq)
	n := utmWGS84A / math.Sqrt(1-eccSq*math.Sin(latRad)*math.Sin(latRad))
	t := math.Tan(latRad) * math.Tan(latRad)
	c := eccPrimeSq * math.Cos(latRad) * math.Cos(latRad)
	a := math.Cos(latRad) * (lonRad - lonOriginRad)
	m := utmWGS84A * ((1-eccSq/4-3*eccSq*eccSq/64-5*eccSq*eccSq*eccSq/256)*latRad -
		(3*eccSq/8+3*eccSq*eccSq/32+45*eccSq*eccSq*eccSq/1024)*math.Sin(2*latRad) +
		(15*eccSq*eccSq/256+45*eccSq*eccSq*eccSq/1024)*math.Sin(4*latRad) -
		(35*eccSq*eccSq*eccSq/3072)*math.Sin(6*latRad))

	easting = utmK0*n*(a+(1-t+c)*a*a*a/6+
		(5-18*t+t*t+72*c-58*eccPrimeSq)*a*a*a*a*a/120) + utmFE
	northing = utmK0 * (m + n*math.Tan(latRad)*(a*a/2+
		(5-t+9*c+4*c*c)*a*a*a*a/24+
		(61-58*t+t*t+600*c-330*eccPrimeSq)*a*a*a*a*a*a/720))

	north = lat >= 0
	if !north {
		northing += utmFNS
	}
	return northing, easting, zone, north
}

// utmToLL inverts llToUTM: UTM northing/easting (metres) in the given
// zone/hemisphere back to WGS84 lat/lon (degrees). Mirrors
// RobotLocalization::NavsatConversions::UTMtoLL.
func utmToLL(northing, easting float64, zone int, north bool) (lat, lon float64) {
	eccSq := utmWGS84E * utmWGS84E
	e1 := (1 - math.Sqrt(1-eccSq)) / (1 + math.Sqrt(1-eccSq))

	x := easting - utmFE
	y := northing
	if !north {
		y -= utmFNS
	}

	eccPrimeSq := eccSq / (1 - eccSq)
	m := y / utmK0
	mu := m / (utmWGS84A * (1 - eccSq/4 - 3*eccSq*eccSq/64 - 5*eccSq*eccSq*eccSq/256))
	phi1 := mu + (3*e1/2-27*e1*e1*e1/32)*math.Sin(2*mu) +
		(21*e1*e1/16-55*e1*e1*e1*e1/32)*math.Sin(4*mu) +
		(151*e1*e1*e1/96)*math.Sin(6*mu)

	n1 := utmWGS84A / math.Sqrt(1-eccSq*math.Sin(phi1)*math.Sin(phi1))
	t1 := math.Tan(phi1) * math.Tan(phi1)
	c1 := eccPrimeSq * math.Cos(phi1) * math.Cos(phi1)
	r1 := utmWGS84A * (1 - eccSq) / math.Pow(1-eccSq*math.Sin(phi1)*math.Sin(phi1), 1.5)
	d := x / (n1 * utmK0)

	latRad := phi1 - (n1*math.Tan(phi1)/r1)*(d*d/2-
		(5+3*t1+10*c1-4*c1*c1-9*eccPrimeSq)*d*d*d*d/24+
		(61+90*t1+298*c1+45*t1*t1-252*eccPrimeSq-3*c1*c1)*d*d*d*d*d*d/720)
	lat = latRad * utmRadDeg

	lonRad := (d - (1+2*t1+c1)*d*d*d/6 +
		(5-2*c1+28*t1-3*c1*c1+8*eccPrimeSq+24*t1*t1)*d*d*d*d*d/120) / math.Cos(phi1)
	lon = utmZoneCentralMeridianDeg(zone) + lonRad*utmRadDeg
	return lat, lon
}

// utmConvergenceRad returns the UTM grid meridian convergence γ (radians) at the
// given lat/lon in the given zone — the signed angle from grid north to true
// north: γ = atan(tan(λ − λ0)·sin φ), λ0 = zone central meridian. Used to rotate
// OpenMower grid-north headings into MowgliNext's true-north frame.
func utmConvergenceRad(lat, lon float64, zone int) float64 {
	dLon := (lon - utmZoneCentralMeridianDeg(zone)) * utmDegRad
	return math.Atan(math.Tan(dLon) * math.Sin(lat*utmDegRad))
}
