package api

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
)

// Reference eastings/northings from the well-tested Python `utm` package
// (WGS84). The Snyder series we port agrees with it to well under 1 mm.
func TestLLtoUTM_MatchesReference(t *testing.T) {
	cases := []struct {
		lat, lon     float64
		wantN, wantE float64
		wantZone     int
		wantNorth    bool
	}{
		{48.137154, 11.576124, 5334754.2469, 691650.3669, 32, true},
		{49.0, 11.0, 5429382.9844, 646280.9461, 32, true},
		{48.0, 1.5, 5317388.7992, 388108.0650, 31, true},
		{-33.9, 151.2, 6247473.3368, 333568.9410, 56, false},
	}
	for _, c := range cases {
		n, e, zone, north := llToUTM(c.lat, c.lon)
		assert.InDelta(t, c.wantN, n, 1e-3, "northing for %.4f,%.4f", c.lat, c.lon)
		assert.InDelta(t, c.wantE, e, 1e-3, "easting for %.4f,%.4f", c.lat, c.lon)
		assert.Equal(t, c.wantZone, zone, "zone for %.4f,%.4f", c.lat, c.lon)
		assert.Equal(t, c.wantNorth, north, "hemisphere for %.4f,%.4f", c.lat, c.lon)
	}
}

// Forward then inverse must recover lat/lon to within ~1e-9° (~0.1 mm).
func TestUTM_RoundTrip(t *testing.T) {
	for _, c := range [][2]float64{
		{48.137154, 11.576124},
		{49.0, 11.0},
		{48.0, 1.5},
		{-33.9, 151.2},
		{0.5, -0.1},
	} {
		n, e, zone, north := llToUTM(c[0], c[1])
		lat, lon := utmToLL(n, e, zone, north)
		assert.InDelta(t, c[0], lat, 1e-8, "lat round-trip %v", c)
		assert.InDelta(t, c[1], lon, 1e-8, "lon round-trip %v", c)
	}
}

// Convergence is zero on the central meridian and grows ~Δλ·sinφ off it, with
// the sign positive east of the central meridian in the northern hemisphere.
func TestUTMConvergence(t *testing.T) {
	// Central meridian of zone 32 is 9°E. At exactly 9°E convergence is 0.
	assert.InDelta(t, 0.0, utmConvergenceRad(48.0, 9.0, 32), 1e-12)

	// 2° east of the central meridian at 48°N → ~1.49°, positive.
	g := utmConvergenceRad(48.0, 11.0, 32)
	assert.InDelta(t, 1.49, g*180.0/math.Pi, 0.05)
	assert.Positive(t, g)

	// Symmetric and negative to the west.
	gw := utmConvergenceRad(48.0, 7.0, 32)
	assert.InDelta(t, -g, gw, 1e-9)
}
