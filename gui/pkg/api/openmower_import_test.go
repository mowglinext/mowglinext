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

	// OM datum offset from MN datum by ~100 m north and ~100 m east. At the
	// OM origin (0, 0) the projection collapses to the constant offset
	// between the two datums; cos(lat) differs negligibly over 100 m so the
	// origin lands very close to (dE, dN) but the test pins the exact value
	// the reprojector computes (not an additive shift).
	mnLat, mnLon := 48.0, 11.0
	reproj := datumReprojector{
		omLat: mnLat + 100.0/metersPerDegreeLat,
		omLon: mnLon + 100.0/(metersPerDegreeLat*math.Cos(mnLat*math.Pi/180.0)),
		mnLat: mnLat,
		mnLon: mnLon,
	}
	wantE0, wantN0 := reproj.project(0, 0)

	replace, dock := buildMowgliNextPayload(omMap, reproj)
	require.Len(t, replace.Areas, 2)

	// Front lawn first vertex was (0, 0); should land at the reprojected
	// origin (≈ (100, 100) but not exactly — that's the whole point).
	first := replace.Areas[0]
	require.NotEmpty(t, first.Area.Area.Points)
	assert.InDelta(t, float32(wantE0), first.Area.Area.Points[0].X, 1e-3)
	assert.InDelta(t, float32(wantN0), first.Area.Area.Points[0].Y, 1e-3)

	// Dock was at (-0.5, 0.2) → reprojected.
	wantDockE, wantDockN := reproj.project(-0.5, 0.2)
	assert.InDelta(t, wantDockE, dock.DockingPose.Position.X, 1e-6)
	assert.InDelta(t, wantDockN, dock.DockingPose.Position.Y, 1e-6)
}

// --- datumReprojector / resolveReprojection ------------------------------

// TestReproject_RoundTrip100m is the stand-in for a real field sample (none
// is available — see the verification gap). It builds an OM polygon whose
// datum is offset ~100 m north and ~100 m east of the MN datum, then asserts
// the reprojected MN points match the expected ENU to within ~1 mm. The
// expected value is derived independently of project() by going OM→lat/lon
// →MN by hand, so a regression in the formula is caught.
func TestReproject_RoundTrip100m(t *testing.T) {
	const M = metersPerDegreeLat
	mnLat, mnLon := 48.137154, 11.576124 // Munich-ish, arbitrary
	// OM datum offset from MN datum.
	dNorth, dEast := 100.0, 100.0
	omLat := mnLat + dNorth/M
	omLon := mnLon + dEast/(M*math.Cos(mnLat*math.Pi/180.0))

	reproj := datumReprojector{omLat: omLat, omLon: omLon, mnLat: mnLat, mnLon: mnLon}

	// A handful of OM-frame points. For each, compute the expected MN-frame
	// point by hand (the same math project() should implement).
	cases := [][2]float64{{0, 0}, {10, 0}, {0, 6}, {-3, 3}, {123.4, -56.7}}
	for _, c := range cases {
		eOM, nOM := c[0], c[1]
		lat := omLat + nOM/M
		lon := omLon + eOM/(M*math.Cos(omLat*math.Pi/180.0))
		wantE := (lon - mnLon) * M * math.Cos(mnLat*math.Pi/180.0)
		wantN := (lat - mnLat) * M

		gotE, gotN := reproj.project(eOM, nOM)
		assert.InDelta(t, wantE, gotE, 1e-3, "east mismatch for OM point %v", c)
		assert.InDelta(t, wantN, gotN, 1e-3, "north mismatch for OM point %v", c)
	}

	// Sanity: the OM origin must land ≈100 m NE of the MN origin. cos(lat)
	// varies by <2e-6 over 100 m so the residual is sub-mm, but the point is
	// that this is computed, not assumed.
	e0, n0 := reproj.project(0, 0)
	assert.InDelta(t, dEast, e0, 1e-2)
	assert.InDelta(t, dNorth, n0, 1e-2)
}

func TestReproject_SameDatumIsIdentity(t *testing.T) {
	// OM datum == MN datum → exact identity (matches the pre-fix additive
	// behaviour for the "same datum" case).
	lat, lon := 48.5, 11.5
	r := datumReprojector{omLat: lat, omLon: lon, mnLat: lat, mnLon: lon}
	for _, c := range [][2]float64{{0, 0}, {10, 6}, {-3, 3}, {-0.5, 0.2}} {
		e, n := r.project(c[0], c[1])
		assert.InDelta(t, c[0], e, 1e-9, "east not identity for %v", c)
		assert.InDelta(t, c[1], n, 1e-9, "north not identity for %v", c)
	}
}

func TestResolveReprojection_NoOmDatumWithMnSetIsIdentityPlusWarning(t *testing.T) {
	r, warn := resolveReprojection(nil, nil, 48.0, 11.0, nil)
	// Identity over the MN datum.
	e, n := r.project(5, 7)
	assert.InDelta(t, 5, e, 1e-9)
	assert.InDelta(t, 7, n, 1e-9)
	assert.Contains(t, warn, "Datum OpenMower non fourni")
	assert.Contains(t, warn, "décalé")
}

func TestResolveReprojection_OmDatumWithMnUnsetAdoptsOmDatum(t *testing.T) {
	// Fresh install: MN datum unreadable, OM datum provided → adopt OM as MN
	// so the import is an exact identity, plus a "set datum_lat/lon" notice.
	omLat, omLon := 49.123456, 8.654321
	r, warn := resolveReprojection(&omLat, &omLon, 0, 0, errors.New("datum unset"))
	for _, c := range [][2]float64{{0, 0}, {12.3, -4.5}} {
		e, n := r.project(c[0], c[1])
		assert.InDelta(t, c[0], e, 1e-9)
		assert.InDelta(t, c[1], n, 1e-9)
	}
	assert.Contains(t, warn, "Datum MowgliNext absent")
	assert.Contains(t, warn, "datum_lat/datum_lon")
}

func TestResolveReprojection_FullGeodeticNoWarning(t *testing.T) {
	omLat, omLon := 49.0, 11.0
	r, warn := resolveReprojection(&omLat, &omLon, 48.0, 11.0, nil)
	assert.Empty(t, warn)
	// 1° north of the MN datum → the OM origin is ~111 km north.
	e0, n0 := r.project(0, 0)
	assert.InDelta(t, 0, e0, 1e-6)
	assert.InDelta(t, metersPerDegreeLat, n0, 1e-6)
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
