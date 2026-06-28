package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"math"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// sampleOpenMowerMap is a minimal but realistic fixture covering all
// four area types we care about: a mowing area with an obstacle inside
// it, a navigation area, a "draft" area that should be skipped, and an
// orphan obstacle that lives outside any parent — plus a single
// active dock.
const sampleOpenMowerMap = `{
  "areas": [
    {
      "id": "MowAreaIdAbcdefghijklmnopqrstuvw1",
      "properties": { "name": "Front lawn", "type": "mow" },
      "outline": [
        {"x":  0.0, "y":  0.0},
        {"x": 10.0, "y":  0.0},
        {"x": 10.0, "y":  6.0},
        {"x":  0.0, "y":  6.0}
      ]
    },
    {
      "id": "ObstacleIdAbcdefghijklmnopqrstu2",
      "properties": { "type": "obstacle" },
      "outline": [
        {"x": 4.0, "y": 2.0},
        {"x": 5.0, "y": 2.0},
        {"x": 5.0, "y": 3.0},
        {"x": 4.0, "y": 3.0}
      ]
    },
    {
      "id": "NavAreaIdAbcdefghijklmnopqrstuv3",
      "properties": { "name": "Driveway", "type": "nav" },
      "outline": [
        {"x": -3.0, "y":  0.0},
        {"x": -3.0, "y":  3.0},
        {"x":  0.0, "y":  3.0},
        {"x":  0.0, "y":  0.0}
      ]
    },
    {
      "id": "DraftIdAbcdefghijklmnopqrstuvwx4",
      "properties": { "type": "draft" },
      "outline": [
        {"x": 100.0, "y": 100.0},
        {"x": 101.0, "y": 100.0},
        {"x": 100.5, "y": 101.0}
      ]
    },
    {
      "id": "OrphanObstacleIdAbcdefghijklmno5",
      "properties": { "type": "obstacle" },
      "outline": [
        {"x": 200.0, "y": 200.0},
        {"x": 201.0, "y": 200.0},
        {"x": 201.0, "y": 201.0},
        {"x": 200.0, "y": 201.0}
      ]
    }
  ],
  "docking_stations": [
    {
      "id": "DockIdAbcdefghijklmnopqrstuvwxy6",
      "properties": { "name": "Docking Station" },
      "position": {"x": -0.5, "y":  0.2},
      "heading":  1.5707963267948966
    }
  ]
}`

// --- parseImportRequest ---------------------------------------------------

func TestParseImportRequest_BareForm(t *testing.T) {
	req, err := parseImportRequest([]byte(sampleOpenMowerMap))
	require.NoError(t, err)
	require.NotEmpty(t, req.Map, "bare form should set Map to the raw body")
	assert.Nil(t, req.OmDatumLat)
	assert.False(t, req.Apply)
}

func TestParseImportRequest_WrappedForm(t *testing.T) {
	wrapped := []byte(`{"map": ` + sampleOpenMowerMap + `, "om_datum_lat": 48.5, "om_datum_lon": 11.5, "apply": false}`)
	req, err := parseImportRequest(wrapped)
	require.NoError(t, err)
	require.NotEmpty(t, req.Map)
	require.NotNil(t, req.OmDatumLat)
	require.NotNil(t, req.OmDatumLon)
	assert.InDelta(t, 48.5, *req.OmDatumLat, 1e-9)
	assert.InDelta(t, 11.5, *req.OmDatumLon, 1e-9)
	assert.False(t, req.Apply)
}

func TestParseImportRequest_RejectsRandomJSON(t *testing.T) {
	_, err := parseImportRequest([]byte(`{"foo": "bar"}`))
	require.Error(t, err)
}

func TestParseImportRequest_RejectsInvalidJSON(t *testing.T) {
	_, err := parseImportRequest([]byte(`not json`))
	require.Error(t, err)
}

// --- buildImportSummary ---------------------------------------------------

func TestBuildImportSummary_Counts(t *testing.T) {
	var omMap openMowerMap
	require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &omMap))

	summary := buildImportSummary(omMap, datumReprojector{})

	// 1 mow + 1 nav, 1 obstacle re-parented under the mow area, 1 orphan,
	// the draft is skipped (warning), no inactive areas in the fixture.
	assert.Equal(t, 1, summary.MowingAreas)
	assert.Equal(t, 1, summary.NavigationAreas)
	assert.Equal(t, 1, summary.Obstacles)
	assert.Equal(t, 1, summary.OrphanObstacles)
	require.Len(t, summary.Areas, 2)

	// Dock pose is preserved.
	require.NotNil(t, summary.DockPose)
	assert.InDelta(t, -0.5, summary.DockPose.X, 1e-9)
	assert.InDelta(t, 0.2, summary.DockPose.Y, 1e-9)
	assert.InDelta(t, math.Pi/2, summary.DockPose.YawRad, 1e-9)

	// At least one warning each: draft skipped + orphan obstacle.
	hasDraft, hasOrphan := false, false
	for _, w := range summary.Warnings {
		if contains(w, "drafts are not imported") {
			hasDraft = true
		}
		if contains(w, "does not fall inside any") {
			hasOrphan = true
		}
	}
	assert.True(t, hasDraft, "expected a draft-skipped warning, got %v", summary.Warnings)
	assert.True(t, hasOrphan, "expected an orphan-obstacle warning, got %v", summary.Warnings)
}

func TestBuildImportSummary_ObstacleIsReparented(t *testing.T) {
	var omMap openMowerMap
	require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &omMap))

	// First area in the fixture is the mow area — confirm the obstacle
	// got attached to it (Obstacles == 1) and not to the nav area.
	summary := buildImportSummary(omMap, datumReprojector{})
	require.Len(t, summary.Areas, 2)
	mowBrief, navBrief := summary.Areas[0], summary.Areas[1]
	assert.Equal(t, "Front lawn", mowBrief.Name)
	assert.Equal(t, 1, mowBrief.Obstacles, "mow area should have 1 nested obstacle")
	assert.Equal(t, "Driveway", navBrief.Name)
	assert.Equal(t, 0, navBrief.Obstacles, "nav area should have no obstacles")
}

func TestBuildImportSummary_AreaSqm(t *testing.T) {
	var omMap openMowerMap
	require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &omMap))

	summary := buildImportSummary(omMap, datumReprojector{})
	require.Len(t, summary.Areas, 2)
	// Front lawn is 10 × 6 = 60 m².
	assert.InDelta(t, 60.0, summary.Areas[0].ApproxAreaSqm, 1e-6)
	// Driveway is 3 × 3 = 9 m².
	assert.InDelta(t, 9.0, summary.Areas[1].ApproxAreaSqm, 1e-6)
}

// --- buildMowgliNextPayload ---------------------------------------------

func TestBuildMowgliNextPayload_StructureMatchesReplaceMapReq(t *testing.T) {
	var omMap openMowerMap
	require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &omMap))

	replace, dock := buildMowgliNextPayload(omMap, datumReprojector{})
	require.NotNil(t, replace)
	require.Len(t, replace.Areas, 2, "expected 1 mow + 1 nav, drafts and orphans excluded")

	// First area (mow) should have the obstacle attached, second (nav) should not.
	first := replace.Areas[0]
	assert.False(t, first.IsNavigationArea)
	assert.Equal(t, "Front lawn", first.Area.Name)
	assert.Len(t, first.Area.Area.Points, 4)
	assert.Len(t, first.Area.Obstacles, 1)
	assert.Len(t, first.Area.Obstacles[0].Points, 4)

	second := replace.Areas[1]
	assert.True(t, second.IsNavigationArea)
	assert.Equal(t, "Driveway", second.Area.Name)
	assert.Empty(t, second.Area.Obstacles)

	// Dock pose: position and (z, w) of the quaternion encode yaw=π/2.
	require.NotNil(t, dock)
	assert.InDelta(t, -0.5, dock.DockingPose.Position.X, 1e-9)
	assert.InDelta(t, 0.2, dock.DockingPose.Position.Y, 1e-9)
	assert.InDelta(t, math.Sin(math.Pi/4), dock.DockingPose.Orientation.Z, 1e-9)
	assert.InDelta(t, math.Cos(math.Pi/4), dock.DockingPose.Orientation.W, 1e-9)
}

func TestBuildMowgliNextPayload_AppliesReprojection(t *testing.T) {
	var omMap openMowerMap
	require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &omMap))

	// OM datum ~100 m NE of the MN datum, off zone 32's central meridian so the
	// UTM convergence is non-trivial. The reprojector inverts OpenMower's UTM
	// projection, so we pin the exact value project() computes (not a shift).
	mnLat, mnLon := 48.0, 11.0
	omLat := mnLat + 100.0/metersPerDegreeLat
	omLon := mnLon + 100.0/(metersPerDegreeLat*math.Cos(mnLat*math.Pi/180.0))
	reproj, warn := resolveReprojection(&omLat, &omLon, mnLat, mnLon, nil)
	require.Empty(t, warn)
	wantE0, wantN0 := reproj.project(0, 0)

	replace, dock := buildMowgliNextPayload(omMap, reproj)
	require.Len(t, replace.Areas, 2)

	// Front lawn first vertex was (0, 0); should land at the reprojected origin.
	first := replace.Areas[0]
	require.NotEmpty(t, first.Area.Area.Points)
	assert.InDelta(t, float32(wantE0), first.Area.Area.Points[0].X, 1e-2)
	assert.InDelta(t, float32(wantN0), first.Area.Area.Points[0].Y, 1e-2)

	// Dock was at (-0.5, 0.2) → reprojected position.
	wantDockE, wantDockN := reproj.project(-0.5, 0.2)
	assert.InDelta(t, wantDockE, dock.DockingPose.Position.X, 1e-6)
	assert.InDelta(t, wantDockN, dock.DockingPose.Position.Y, 1e-6)
}

// --- datumReprojector / resolveReprojection ------------------------------

// TestReproject_UTMRoundTrip builds OM-grid coordinates exactly the way
// OpenMower does (UTM(point) − UTM(datum)) for a set of known absolute
// positions, then asserts project() recovers MowgliNext's true-north ENU for
// each to within ~1 mm. The expected value is derived independently of
// project(), so a regression in the formula is caught.
func TestReproject_UTMRoundTrip(t *testing.T) {
	const M = metersPerDegreeLat
	mnLat, mnLon := 48.137154, 11.576124 // Munich-ish, arbitrary
	// OM datum offset ~100 m NE of the MN datum.
	omLat := mnLat + 100.0/M
	omLon := mnLon + 100.0/(M*math.Cos(mnLat*math.Pi/180.0))
	reproj, warn := resolveReprojection(&omLat, &omLon, mnLat, mnLon, nil)
	require.Empty(t, warn)

	dN, dE, _, _ := llToUTM(omLat, omLon)
	for _, off := range [][2]float64{{0, 0}, {10, 0}, {0, 6}, {-3, 3}, {123.4, -56.7}} {
		// A known absolute position ~off[E],off[N] metres from the OM datum.
		lat := omLat + off[1]/M
		lon := omLon + off[0]/(M*math.Cos(omLat*math.Pi/180.0))
		// OM-grid coordinate exactly as OpenMower would store it.
		n, e, _, _ := llToUTM(lat, lon)
		eOM, nOM := e-dE, n-dN
		// Expected MN ENU, computed independently of project().
		wantE := (lon - mnLon) * M * math.Cos(mnLat*math.Pi/180.0)
		wantN := (lat - mnLat) * M

		gotE, gotN := reproj.project(eOM, nOM)
		assert.InDelta(t, wantE, gotE, 2e-3, "east mismatch for OM offset %v", off)
		assert.InDelta(t, wantN, gotN, 2e-3, "north mismatch for OM offset %v", off)
	}
}

func TestReproject_OnCentralMeridianHasNoRotation(t *testing.T) {
	// OM datum == MN datum AND on zone 32's central meridian (9°E) → zero
	// convergence. project() may rescale slightly (UTM grid vs true ground
	// distance, plus the fixed metersPerDegreeLat constant — the same projection
	// MowgliNext's own localizer uses) but must not ROTATE: a grid-east vector
	// stays east and a grid-north vector stays north, with no cross-axis leakage.
	lat, lon := 48.5, 9.0
	r, _ := resolveReprojection(&lat, &lon, lat, lon, nil)

	e, n := r.project(20, 0) // grid-east
	assert.InDelta(t, 0, n, 5e-3, "north leakage on a grid-east vector")
	assert.InDelta(t, 20, e, 0.05, "east magnitude")

	e, n = r.project(0, 20) // grid-north
	assert.InDelta(t, 0, e, 5e-3, "east leakage on a grid-north vector")
	assert.InDelta(t, 20, n, 0.05, "north magnitude")
}

// TestResolveReprojection_NoOmDatum_UsesMnDatumToDeRotate covers the
// "imported map rotated without datum" report: when the OpenMower datum is
// omitted but the MowgliNext datum is known, the importer must anchor the UTM
// inversion at the MN datum (assuming the same site) and de-rotate, NOT copy
// the grid-north coordinates through unchanged.
func TestResolveReprojection_NoOmDatum_UsesMnDatumToDeRotate(t *testing.T) {
	const M = metersPerDegreeLat
	mnLat, mnLon := 48.5, 11.5 // 2.5° off zone 32's central meridian → real convergence
	reproj, warn := resolveReprojection(nil, nil, mnLat, mnLon, nil)
	require.True(t, reproj.reproject, "no OM datum + known MN datum must still reproject")
	require.NotEmpty(t, warn, "should warn that the MN datum is used as the reference")

	// OM-grid coords as OpenMower stores them, anchored at the MN datum (the
	// assumption the fix makes); project() must recover true-north ENU to ~mm.
	dN, dE, _, _ := llToUTM(mnLat, mnLon)
	for _, off := range [][2]float64{{0, 0}, {15, 0}, {0, 15}, {-7, 9}} {
		lat := mnLat + off[1]/M
		lon := mnLon + off[0]/(M*math.Cos(mnLat*math.Pi/180.0))
		n, e, _, _ := llToUTM(lat, lon)
		eOM, nOM := e-dE, n-dN
		wantE := (lon - mnLon) * M * math.Cos(mnLat*math.Pi/180.0)
		wantN := (lat - mnLat) * M
		gotE, gotN := reproj.project(eOM, nOM)
		assert.InDelta(t, wantE, gotE, 3e-3, "east for OM offset %v", off)
		assert.InDelta(t, wantN, gotN, 3e-3, "north for OM offset %v", off)
	}
}

// TestResolveReprojection_NoDatumsAtAll_PassesThrough: with neither datum there
// is no UTM anchor, so coordinates are copied straight through (offset+rotated)
// and the operator is warned.
func TestResolveReprojection_NoDatumsAtAll_PassesThrough(t *testing.T) {
	r, warn := resolveReprojection(nil, nil, 0, 0, errors.New("no datum"))
	assert.False(t, r.reproject, "no anchor → copy through")
	assert.NotEmpty(t, warn)
	e, n := r.project(12.3, -4.5)
	assert.Equal(t, 12.3, e)
	assert.Equal(t, -4.5, n)
}

// TestReproject_SameDatumOffCentralMeridianRemovesConvergence is the core
// regression for the "imported map rotated 1–2°" bug: even with identical OM
// and MN datums, OpenMower's UTM grid-north coordinates must be de-rotated by
// the meridian convergence into MowgliNext's true-north ENU.
func TestReproject_SameDatumOffCentralMeridianRemovesConvergence(t *testing.T) {
	lat, lon := 48.0, 11.0 // 2° east of zone 32's central meridian (9°E)
	r, _ := resolveReprojection(&lat, &lon, lat, lon, nil)

	// A 10 m vector along OM grid-north (0, 10) rotates by −γ about the origin.
	e, n := r.project(0, 10)
	gamma := utmConvergenceRad(lat, lon, 32) // ~ +0.026 rad (1.49°)
	theta := -gamma
	wantE := -10 * math.Sin(theta)
	wantN := 10 * math.Cos(theta)
	assert.InDelta(t, wantE, e, 0.03)
	assert.InDelta(t, wantN, n, 0.03)
	// Genuinely rotated — NOT the old identity behaviour.
	assert.Greater(t, math.Abs(e), 0.1)
}

func TestResolveReprojection_NoOmDatumWithMnSetIsPassThroughPlusWarning(t *testing.T) {
	r, warn := resolveReprojection(nil, nil, 48.0, 11.0, nil)
	assert.False(t, r.reproject)
	// No UTM anchor → coordinates pass through unchanged.
	e, n := r.project(5, 7)
	assert.InDelta(t, 5, e, 1e-9)
	assert.InDelta(t, 7, n, 1e-9)
	assert.Contains(t, warn, "Datum OpenMower non fourni")
	assert.Contains(t, warn, "pivoté")
}

func TestResolveReprojection_OmDatumWithMnUnsetAdoptsOmDatum(t *testing.T) {
	// Fresh install: MN datum unreadable, OM datum provided → adopt OM as MN so
	// the import is georeferenced about it (convergence still removed), plus a
	// "set datum_lat/lon" notice. The OM origin maps to the MN origin.
	omLat, omLon := 49.123456, 8.654321
	r, warn := resolveReprojection(&omLat, &omLon, 0, 0, errors.New("datum unset"))
	assert.True(t, r.reproject)
	e0, n0 := r.project(0, 0)
	assert.InDelta(t, 0, e0, 1e-3)
	assert.InDelta(t, 0, n0, 1e-3)
	assert.Contains(t, warn, "Datum MowgliNext absent")
	assert.Contains(t, warn, "datum_lat/datum_lon")
}

func TestResolveReprojection_FullGeodeticNoWarning(t *testing.T) {
	omLat, omLon := 49.0, 11.0
	r, warn := resolveReprojection(&omLat, &omLon, 48.0, 11.0, nil)
	assert.Empty(t, warn)
	assert.True(t, r.reproject)
	// 1° north of the MN datum → the OM origin is ~111 km north.
	e0, n0 := r.project(0, 0)
	assert.InDelta(t, 0, e0, 1e-2)
	assert.InDelta(t, metersPerDegreeLat, n0, 1e-2)
}

// --- HTTP handler --------------------------------------------------------

func setupImportRouter(rosProvider types.IRosProvider, dbProvider types.IDBProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	group := r.Group("/api")
	ImportRoutes(group, rosProvider, dbProvider)
	return r
}

func TestPostImportOpenMower_PreviewReturnsSummary(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupImportRouter(mock, nil) // nil dbProvider → identity datum + warning

	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader([]byte(sampleOpenMowerMap)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code, "body=%s", w.Body.String())

	var resp ImportOpenMowerSummary
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Equal(t, 1, resp.MowingAreas)
	assert.Equal(t, 1, resp.NavigationAreas)
	assert.Equal(t, 1, resp.Obstacles)
	assert.False(t, resp.Applied, "preview-only run must not be marked applied")

	// No ROS service calls should fire in preview mode.
	assert.Empty(t, mock.ServiceCalls)
}

func TestPostImportOpenMower_ApplyCallsClearAddSaveAndDock(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupImportRouter(mock, nil)

	body := []byte(`{"map": ` + sampleOpenMowerMap + `, "apply": true}`)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code, "body=%s", w.Body.String())

	var resp ImportOpenMowerSummary
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.True(t, resp.Applied, "applied flag must be true on the success response")

	// Expected sequence: clear_map → add_area (mow) → add_area (nav) →
	// save_areas → set_docking_point. The fixture has one mow area
	// (with one obstacle nested) and one nav area, hence two add_area
	// calls.
	services := make([]string, 0, len(mock.ServiceCalls))
	for _, sc := range mock.ServiceCalls {
		services = append(services, sc.Service)
	}
	assert.Equal(t,
		[]string{
			"/map_server_node/clear_map",
			"/map_server_node/add_area",
			"/map_server_node/add_area",
			"/map_server_node/save_areas",
			"/map_server_node/set_docking_point",
		},
		services,
		"unexpected ROS call sequence",
	)
}

func TestPostImportOpenMower_ApplyClearMapErrorPropagates(t *testing.T) {
	mock := types.NewMockRosProvider()
	mock.ServiceErr = assert.AnError
	router := setupImportRouter(mock, nil)

	body := []byte(`{"map": ` + sampleOpenMowerMap + `, "apply": true}`)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusInternalServerError, w.Code, "body=%s", w.Body.String())
	// First service call attempted is clear_map and it must abort the rest.
	require.NotEmpty(t, mock.ServiceCalls)
	assert.Equal(t, "/map_server_node/clear_map", mock.ServiceCalls[0].Service)
	assert.Len(t, mock.ServiceCalls, 1, "no further calls after clear_map failure")
}

func TestPostImportOpenMower_RejectsBadJSON(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupImportRouter(mock, nil)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader([]byte(`{"nope":1}`)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusBadRequest, w.Code, "body=%s", w.Body.String())
}

// --- legacy bag-JSON adapter ---------------------------------------------

// sampleLegacyBagMap is a trimmed reproduction of the legacy
// bag-derived map.json shape Daisy hit on 2026-05-15 — PascalCase
// fields, `Package: 0` envelopes everywhere, NavigationAreas /
// WorkingArea split, dock as scalar fields. The fixture covers all
// three pathways the adapter has to handle:
//   - a WorkingArea entry (becomes a mow area)
//   - a WorkingArea obstacle (becomes a top-level type="obstacle")
//   - a NavigationAreas entry (becomes a nav area)
//   - DockX/DockY/DockHeading (becomes the single docking station)
const sampleLegacyBagMap = `{
  "Package": 0,
  "MapWidth": 20.0,
  "NavigationAreas": [
    {
      "Package": 0,
      "Name": "",
      "Area": {
        "Package": 0,
        "Points": [
          {"Package": 0, "X": -3.0, "Y": 0.0, "Z": 0},
          {"Package": 0, "X":  0.0, "Y": 0.0, "Z": 0},
          {"Package": 0, "X":  0.0, "Y": 3.0, "Z": 0},
          {"Package": 0, "X": -3.0, "Y": 3.0, "Z": 0}
        ]
      },
      "Obstacles": null
    }
  ],
  "WorkingArea": [
    {
      "Package": 0,
      "Name": "",
      "Area": {
        "Package": 0,
        "Points": [
          {"Package": 0, "X":  0.0, "Y":  0.0, "Z": 0},
          {"Package": 0, "X": 10.0, "Y":  0.0, "Z": 0},
          {"Package": 0, "X": 10.0, "Y":  6.0, "Z": 0},
          {"Package": 0, "X":  0.0, "Y":  6.0, "Z": 0}
        ]
      },
      "Obstacles": [
        {
          "Package": 0,
          "Points": [
            {"Package": 0, "X": 4.0, "Y": 2.0, "Z": 0},
            {"Package": 0, "X": 5.0, "Y": 2.0, "Z": 0},
            {"Package": 0, "X": 5.0, "Y": 3.0, "Z": 0},
            {"Package": 0, "X": 4.0, "Y": 3.0, "Z": 0}
          ]
        }
      ]
    }
  ],
  "DockX": -0.5,
  "DockY": 0.2,
  "DockHeading": 1.5707963267948966
}`

func TestParseImportRequest_LegacyBareForm(t *testing.T) {
	req, err := parseImportRequest([]byte(sampleLegacyBagMap))
	require.NoError(t, err, "legacy bag-JSON should be accepted bare")
	require.NotEmpty(t, req.Map)

	// The body the rest of the pipeline sees must already be in modern
	// shape (areas / docking_stations).
	var modern openMowerMap
	require.NoError(t, json.Unmarshal(req.Map, &modern))
	// 1 mow + 1 obstacle (under that mow) + 1 nav = 3 entries.
	require.Len(t, modern.Areas, 3)

	gotTypes := map[string]int{}
	for _, a := range modern.Areas {
		gotTypes[a.Properties.Type]++
	}
	assert.Equal(t, 1, gotTypes["mow"])
	assert.Equal(t, 1, gotTypes["nav"])
	assert.Equal(t, 1, gotTypes["obstacle"])

	// Dock translated 1:1 from scalar fields.
	require.Len(t, modern.DockingStations, 1)
	d := modern.DockingStations[0]
	assert.InDelta(t, -0.5, d.Position.X, 1e-9)
	assert.InDelta(t, 0.2, d.Position.Y, 1e-9)
	assert.InDelta(t, math.Pi/2, d.Heading, 1e-9)
}

func TestParseImportRequest_LegacyWrappedForm(t *testing.T) {
	wrapped := []byte(`{"map": ` + sampleLegacyBagMap + `, "om_datum_lat": 48.5, "om_datum_lon": 11.5}`)
	req, err := parseImportRequest(wrapped)
	require.NoError(t, err)
	require.NotEmpty(t, req.Map)
	require.NotNil(t, req.OmDatumLat)
	assert.InDelta(t, 48.5, *req.OmDatumLat, 1e-9)

	// Conversion still applied through the wrapper.
	var modern openMowerMap
	require.NoError(t, json.Unmarshal(req.Map, &modern))
	require.Len(t, modern.Areas, 3)
	require.Len(t, modern.DockingStations, 1)
}

func TestPostImportOpenMower_LegacyBagPreview(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupImportRouter(mock, nil)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPost, "/api/import/openmower",
		bytes.NewReader([]byte(sampleLegacyBagMap)))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	require.Equal(t, http.StatusOK, w.Code, "body=%s", w.Body.String())

	var summary ImportOpenMowerSummary
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &summary))
	assert.Equal(t, 1, summary.MowingAreas)
	assert.Equal(t, 1, summary.NavigationAreas)
	assert.Equal(t, 1, summary.Obstacles, "WorkingArea[0].Obstacles[0] should re-attach to its parent")
	assert.Equal(t, 0, summary.OrphanObstacles)
	require.NotNil(t, summary.DockPose)
}

// --- helpers --------------------------------------------------------------

// contains is a tiny substring helper to keep the assertions readable
// without dragging in `strings` at the top of the test file (we use it
// in two places).
func contains(s, sub string) bool {
	for i := 0; i+len(sub) <= len(s); i++ {
		if s[i:i+len(sub)] == sub {
			return true
		}
	}
	return false
}
