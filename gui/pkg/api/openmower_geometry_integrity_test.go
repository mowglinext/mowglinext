package api

import (
	"bytes"
	"context"
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func geometryImportFixture() openMowerMap {
	area := func(id, kind string, outline []openMowerPoint) openMowerArea {
		return openMowerArea{ID: id, Properties: openMowerAreaProperties{Name: id, Type: kind}, Outline: outline}
	}
	return openMowerMap{Areas: []openMowerArea{
		area("A", "mow", []openMowerPoint{{0, 0}, {10, 0}, {10, 10}, {0, 10}}),
		area("B-out-of-bounds", "mow", []openMowerPoint{{100001, 0}, {100002, 0}, {100002, 1}}),
		area("valid obstacle", "obstacle", []openMowerPoint{{1, 1}, {2, 1}, {2, 2}, {1, 2}}),
		// Centroid is inside A, but one vertex is outside the accepted bound.
		area("invalid obstacle", "obstacle", []openMowerPoint{{100001, 1}, {-50000, 0}, {-50000, 2}}),
	}}
}

func TestOpenMowerPreviewPayloadGeometryParity(t *testing.T) {
	source := geometryImportFixture()
	summary := buildImportSummary(source, datumReprojector{})
	req, _ := buildMowgliNextPayload(source, datumReprojector{})
	require.Equal(t, 1, summary.MowingAreas)
	require.Equal(t, 1, summary.Obstacles)
	assert.Len(t, req.Areas, summary.MowingAreas+summary.NavigationAreas)
	require.NotEmpty(t, req.Areas)
	assert.Len(t, req.Areas[0].Area.Obstacles, summary.Obstacles)
	assert.Equal(t, "A", req.Areas[0].Area.Name)
	assert.Len(t, req.Areas[0].Area.Area.Points, 4)
	require.GreaterOrEqual(t, len(summary.Warnings), 2)
	assert.Contains(t, summary.Warnings[0], "B-out-of-bounds")
	assert.Contains(t, summary.Warnings[1], "invalid obstacle")
}

func TestOpenMowerSelectionRulesAndProjectionRemainShared(t *testing.T) {
	source := geometryImportFixture()
	inactive := false
	clone := func(id, kind string) openMowerArea {
		a := source.Areas[0]
		a.ID = id
		a.Properties.Name = id
		a.Properties.Type = kind
		return a
	}
	nav := clone("navigation", "nav")
	source.Areas = append(source.Areas, nav, clone("draft", "draft"), clone("unknown", "future"), clone("inactive", "mow"))
	source.Areas[len(source.Areas)-1].Properties.Active = &inactive
	orphan := source.Areas[2]
	orphan.ID = "orphan"
	orphan.Properties.Name = "orphan"
	orphan.Outline = []openMowerPoint{{50, 50}, {51, 50}, {51, 51}, {50, 51}}
	source.Areas = append(source.Areas, orphan)
	before, _ := json.Marshal(source)
	lat, lon := 48.0, 11.0
	reprojection, _ := resolveReprojection(&lat, &lon, lat, lon, nil)
	for _, projection := range []datumReprojector{{}, reprojection} {
		summary, req, dock := buildOpenMowerImport(source, projection)
		assert.Nil(t, dock)
		assert.Equal(t, 1, summary.MowingAreas)
		assert.Equal(t, 1, summary.NavigationAreas)
		assert.Equal(t, 1, summary.Obstacles)
		assert.Equal(t, 1, summary.OrphanObstacles)
		require.Len(t, req.Areas, 2)
		assert.Equal(t, []string{"A", "navigation"}, areaNames(req.Areas))
		for i, area := range req.Areas {
			assert.Equal(t, summary.Areas[i].Name, area.Area.Name)
			assert.Equal(t, summary.Areas[i].Vertices, len(area.Area.Area.Points))
			assert.Equal(t, summary.Areas[i].Obstacles, len(area.Area.Obstacles))
			assert.Equal(t, summary.Areas[i].IsNavigationArea, area.IsNavigationArea)
		}
		assert.Equal(t, outlineToPoint32s(source.Areas[0].Outline, projection), req.Areas[0].Area.Area.Points)
		assert.Equal(t, outlineToPoint32s(source.Areas[2].Outline, projection), req.Areas[0].Area.Obstacles[0].Points)
		// Backwards-compatible helper entry points use exactly the same builder.
		assert.Equal(t, summary, buildImportSummary(source, projection))
		other, _ := buildMowgliNextPayload(source, projection)
		assert.Equal(t, req, other)
	}
	after, _ := json.Marshal(source)
	assert.JSONEq(t, string(before), string(after))
}

func TestOpenMowerInvalidOutlinesAreAbsentFromBothResults(t *testing.T) {
	for _, points := range [][]openMowerPoint{
		{{0, 0}, {0, 0}, {0, 0}}, {{0, 0}, {1, 0}, {2, 0}},
		{{0, 0}, {2, 2}, {0, 2}, {2, 0}}, {{0, 0}, {1, 0}, {1, 1}, {1, 0}, {0, 1}},
		{{0, 0}, {math.NaN(), 1}, {0, 1}}, {{0, 0}, {math.Inf(1), 1}, {0, 1}},
	} {
		source := geometryImportFixture()
		source.Areas = source.Areas[:1]
		source.Areas[0].Outline = points
		summary, req, _ := buildOpenMowerImport(source, datumReprojector{})
		assert.Zero(t, summary.MowingAreas)
		assert.Empty(t, req.Areas)
		assert.NotEmpty(t, summary.Warnings)
	}
}

func TestOpenMowerRejectedGeometryNeverTouchesROS(t *testing.T) {
	for _, scenario := range []string{"empty", "invalid only", "navigation only", "obstacle only", "inactive", "draft"} {
		t.Run(scenario, func(t *testing.T) {
			source := geometryImportFixture()
			switch scenario {
			case "empty":
				source.Areas = nil
			case "invalid only":
				source.Areas = source.Areas[1:2]
			default:
				source.Areas = source.Areas[:1]
				switch scenario {
				case "navigation only":
					source.Areas[0].Properties.Type = "nav"
				case "obstacle only":
					source.Areas[0].Properties.Type = "obstacle"
				case "inactive":
					active := false
					source.Areas[0].Properties.Active = &active
				case "draft":
					source.Areas[0].Properties.Type = "draft"
				}
			}
			// A requested dock cannot authorize clearing a map with no mowing area.
			source.DockingStations = []openMowerDockingStation{{Position: openMowerPoint{X: 1, Y: 2}}}
			raw, err := json.Marshal(source)
			require.NoError(t, err)
			server := newImportStateServer()
			preview := serveImport(t, server, context.Background(), string(raw), false)
			requireImportOutcome(t, preview, http.StatusOK, mapUnchanged, dockNotAttempted)
			assert.Empty(t, server.calls)
			apply := serveImport(t, server, context.Background(), string(raw), true)
			requireImportOutcome(t, apply, http.StatusBadRequest, mapUnchanged, dockNotAttempted)
			assert.Empty(t, server.calls)
			assert.Equal(t, []string{"A"}, areaNames(server.disk))
			assert.Equal(t, 99.0, server.dock.Position.X)
		})
	}
}

func TestOpenMowerApplyUsesOnlyPreviewedGeometry(t *testing.T) {
	source := geometryImportFixture()
	raw, err := json.Marshal(source)
	require.NoError(t, err)
	server := newImportStateServer()
	preview := serveImport(t, server, context.Background(), string(raw), false)
	requireImportOutcome(t, preview, http.StatusOK, mapUnchanged, dockNotRequested)
	assert.Empty(t, server.calls)
	apply := serveImport(t, server, context.Background(), string(raw), true)
	requireImportOutcome(t, apply, http.StatusOK, mapPersisted, dockNotRequested)
	assert.Equal(t, []string{"A"}, areaNames(server.disk))
	require.Len(t, server.disk[0].Area.Obstacles, 1)
	assert.Equal(t, outlineToPoint32s(source.Areas[2].Outline, datumReprojector{}), server.disk[0].Area.Obstacles[0].Points)
}

func TestOpenMowerUnrepresentableProjectionIsNotSelected(t *testing.T) {
	source := geometryImportFixture()
	source.Areas = source.Areas[:1]
	// Distinct float64 points would collapse to the same ROS Point32.
	source.Areas[0].Outline = []openMowerPoint{{90000, 90000}, {90000.002, 90000}, {90000, 90001}}
	summary, payload, _ := buildOpenMowerImport(source, datumReprojector{})
	assert.Zero(t, summary.MowingAreas)
	assert.Empty(t, payload.Areas)
	require.Len(t, summary.Warnings, 2) // projected outline and no mowing areas
	assert.Contains(t, summary.Warnings[0], "unrepresentable projected outline")

	// The source is within bounds, but its target frame is more than 100 km away.
	lat, lon := 48.0, 11.0
	projection, _ := resolveReprojection(&lat, &lon, 40, 11, nil)
	source = geometryImportFixture()
	source.Areas = source.Areas[:1]
	summary, payload, _ = buildOpenMowerImport(source, projection)
	assert.Zero(t, summary.MowingAreas)
	assert.Empty(t, payload.Areas)
	assert.Contains(t, summary.Warnings[0], "invalid projected vertex")
}

func TestOpenMowerInvalidFramesAndDockNeverWrite(t *testing.T) {
	for _, scenario := range []string{"source latitude", "source longitude", "target latitude", "target nonfinite", "dock position"} {
		t.Run(scenario, func(t *testing.T) {
			source := geometryImportFixture()
			source.Areas = source.Areas[:1]
			source.DockingStations = []openMowerDockingStation{{Position: openMowerPoint{X: 1, Y: 2}}}
			request := ImportOpenMowerRequest{Apply: true}
			var db types.IDBProvider
			switch scenario {
			case "source latitude":
				value := 90.0
				request.OmDatumLat = &value
			case "source longitude":
				value := 181.0
				request.OmDatumLon = &value
			case "target latitude", "target nonfinite":
				value := "90"
				if scenario == "target nonfinite" {
					value = ".nan"
				}
				path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
				require.NoError(t, os.WriteFile(path, []byte("mowgli:\n  ros__parameters:\n    datum_lat: "+value+"\n    datum_lon: 11\n"), 0o644))
				db = types.NewMockDBProvider()
				require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
			case "dock position":
				source.DockingStations[0].Position.X = 100001
			}
			raw, err := json.Marshal(source)
			require.NoError(t, err)
			request.Map = raw
			body, err := json.Marshal(request)
			require.NoError(t, err)
			server := newImportStateServer()
			response := httptest.NewRecorder()
			setupImportRouter(server, db).ServeHTTP(response, httptest.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader(body)))
			requireImportOutcome(t, response, http.StatusBadRequest, mapUnchanged, dockNotAttempted)
			assert.Empty(t, server.calls)
			assert.Equal(t, []string{"A"}, areaNames(server.disk))
			assert.Equal(t, 99.0, server.dockDisk.Position.X)
		})
	}
}
