package api

import (
	"encoding/json"
	"math"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// loadRealLegacyMap parses the real OpenMower legacy bag-JSON export in
// testdata/ through the importer's parse + legacy-conversion path and
// returns the resulting modern openMowerMap.
//
// The source map has 8 WorkingArea zones (one carrying a single nested
// obstacle), 1 NavigationArea, and a scalar dock — a realistic garden
// export, not a hand-built fixture.
func loadRealLegacyMap(t *testing.T) openMowerMap {
	t.Helper()
	raw, err := os.ReadFile("testdata/openmower_legacy_real.json")
	require.NoError(t, err)
	req, err := parseImportRequest(raw)
	require.NoError(t, err)
	var om openMowerMap
	require.NoError(t, json.Unmarshal(req.Map, &om))
	return om
}

// TestImportRealLegacyMap_FaithfulCounts documents that the legacy
// adapter reproduces the source map 1:1 — every WorkingArea becomes a
// mow area, the NavigationArea becomes a nav area, the nested obstacle
// re-attaches to its parent (no orphan), and the dock survives. This
// part of the import is correct; it is the baseline the bug below sits
// on top of.
func TestImportRealLegacyMap_FaithfulCounts(t *testing.T) {
	om := loadRealLegacyMap(t)
	summary := buildImportSummary(om, 0, 0)

	assert.Equal(t, 8, summary.MowingAreas, "all 8 WorkingArea zones import as mow areas")
	assert.Equal(t, 1, summary.NavigationAreas, "the single NavigationArea imports")
	assert.Equal(t, 1, summary.Obstacles, "the one nested obstacle re-attaches to its parent")
	assert.Equal(t, 0, summary.OrphanObstacles, "obstacle is not orphaned")

	require.NotNil(t, summary.DockPose)
	assert.InDelta(t, -3.4847958843356044, summary.DockPose.X, 1e-6)
	assert.InDelta(t, 1.5364979726737127, summary.DockPose.Y, 1e-6)
	assert.InDelta(t, -1.269101270085267, summary.DockPose.YawRad, 1e-6)

	// Per-area vertex counts after dedup. The source rings are
	// {69,73,44,33,24,71,37,55,88}; each loses its closing wrap-around
	// duplicate, and mow-7/mow-8/nav-1 also lose a leading doubled vertex
	// (which is exactly why those last two working areas failed to import
	// before the dedup fix).
	replace, _ := buildMowgliNextPayload(om, 0, 0)
	require.Len(t, replace.Areas, 9)
	gotVerts := make([]int, 0, len(replace.Areas))
	for _, ra := range replace.Areas {
		gotVerts = append(gotVerts, len(ra.Area.Area.Points))
	}
	// WorkingArea[0..7] then NavigationArea[0], in import order.
	assert.Equal(t, []int{68, 72, 43, 32, 23, 70, 35, 53, 86}, gotVerts)
}

// TestImportRealLegacyMap_NoDegenerateVertices is the bug-exposing test.
//
// THE BUG: the importer copies OpenMower outline vertices verbatim and
// never collapses duplicate-consecutive vertices. This real export
// starts many of its rings with a doubled vertex (e.g. the
// NavigationArea's first two points are identical) and closes the
// obstacle ring with a repeated vertex. The result is a set of polygon
// rings that each contain one or more ZERO-LENGTH edges.
//
// Zero-length / duplicate-consecutive edges make the ring non-simple.
// Downstream this breaks the F2C v3 coverage pipeline and map_server's
// grid_map polygon stamping (boost::geometry treats a zero-length
// segment as invalid geometry), so the map imports "successfully" but
// fails the moment COMMAND_START runs coverage on it.
//
// This test currently FAILS — it is the regression guard for the fix
// (de-duplicate consecutive vertices, and drop the wrap-around closing
// duplicate, in outlineToPoint32s / the legacy adapter).
func TestImportRealLegacyMap_NoDegenerateVertices(t *testing.T) {
	om := loadRealLegacyMap(t)
	replace, _ := buildMowgliNextPayload(om, 0, 0)

	degenerate := 0
	for _, ra := range replace.Areas {
		rings := [][]struct{ X, Y float32 }{}
		ring := make([]struct{ X, Y float32 }, 0, len(ra.Area.Area.Points))
		for _, p := range ra.Area.Area.Points {
			ring = append(ring, struct{ X, Y float32 }{p.X, p.Y})
		}
		rings = append(rings, ring)
		for _, ob := range ra.Area.Obstacles {
			oring := make([]struct{ X, Y float32 }, 0, len(ob.Points))
			for _, p := range ob.Points {
				oring = append(oring, struct{ X, Y float32 }{p.X, p.Y})
			}
			rings = append(rings, oring)
		}
		for _, r := range rings {
			n := len(r)
			for i := 0; i < n; i++ {
				a, b := r[i], r[(i+1)%n]
				if math.Abs(float64(a.X-b.X)) < 1e-6 && math.Abs(float64(a.Y-b.Y)) < 1e-6 {
					degenerate++
					t.Logf("zero-length edge in %q at vertex %d → (%.4f, %.4f)", ra.Area.Name, i, a.X, a.Y)
				}
			}
		}
	}

	assert.Equal(t, 0, degenerate,
		"imported polygons must have no zero-length (duplicate-consecutive) edges; "+
			"the importer is passing OpenMower's degenerate vertices straight through")
}
