package api

import (
	"bytes"
	"encoding/json"
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

	summary := buildImportSummary(omMap, 0, 0)

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
	summary := buildImportSummary(omMap, 0, 0)
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

	summary := buildImportSummary(omMap, 0, 0)
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

	replace, dock := buildMowgliNextPayload(omMap, 0, 0)
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

func TestBuildMowgliNextPayload_AppliesDatumShift(t *testing.T) {
	var omMap openMowerMap
	require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &omMap))

	const dE, dN = 12.5, -7.0
	replace, dock := buildMowgliNextPayload(omMap, dE, dN)
	require.Len(t, replace.Areas, 2)

	// Front lawn first vertex was (0, 0); should land at (dE, dN).
	first := replace.Areas[0]
	require.NotEmpty(t, first.Area.Area.Points)
	assert.InDelta(t, float32(dE), first.Area.Area.Points[0].X, 1e-5)
	assert.InDelta(t, float32(dN), first.Area.Area.Points[0].Y, 1e-5)

	// Dock was at (-0.5, 0.2) → ( -0.5+dE, 0.2+dN ).
	assert.InDelta(t, -0.5+dE, dock.DockingPose.Position.X, 1e-9)
	assert.InDelta(t, 0.2+dN, dock.DockingPose.Position.Y, 1e-9)
}

// --- computeDatumShift ---------------------------------------------------

func TestComputeDatumShift_NoOmDatumIsIdentity(t *testing.T) {
	e, n, warn := computeDatumShift(nil, nil, 48.0, 11.0, nil)
	assert.Equal(t, 0.0, e)
	assert.Equal(t, 0.0, n)
	assert.NotEmpty(t, warn, "expected an identity-transform warning")
}

func TestComputeDatumShift_SameDatum(t *testing.T) {
	lat := 48.5
	lon := 11.5
	e, n, warn := computeDatumShift(&lat, &lon, lat, lon, nil)
	assert.InDelta(t, 0, e, 1e-9)
	assert.InDelta(t, 0, n, 1e-9)
	assert.Empty(t, warn)
}

func TestComputeDatumShift_NorthernShift(t *testing.T) {
	// OpenMower datum 1° north of MowgliNext at MN-lat 48° → ~111 km north.
	mnLat := 48.0
	mnLon := 11.0
	omLat := 49.0
	omLon := 11.0
	e, n, warn := computeDatumShift(&omLat, &omLon, mnLat, mnLon, nil)
	assert.Empty(t, warn)
	assert.InDelta(t, 0, e, 1e-6)
	assert.InDelta(t, metersPerDegreeLat, n, 1e-6)
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

func TestPostImportOpenMower_ApplyIsStubbedOff(t *testing.T) {
	mock := types.NewMockRosProvider()
	router := setupImportRouter(mock, nil)

	body := []byte(`{"map": ` + sampleOpenMowerMap + `, "apply": true}`)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	// Apply path is intentionally disabled — handler should report 500
	// with the "preview-only" message rather than silently writing.
	require.Equal(t, http.StatusInternalServerError, w.Code, "body=%s", w.Body.String())
	assert.Contains(t, w.Body.String(), "preview-only")
	assert.Empty(t, mock.ServiceCalls, "no ROS calls should have fired")
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
