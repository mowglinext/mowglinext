package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeMapServer is an in-memory map_server_node: it holds a live area list and
// a "disk" copy, answers the four services a replace uses, and can be told to
// fail the Nth call of one service.
type fakeMapServer struct {
	*types.MockRosProvider
	live      []mowgli.ReplaceMapArea
	disk      []mowgli.ReplaceMapArea
	calls     map[string]int
	failAt    map[string]int   // service → 1-based call number that fails…
	failWith  map[string]error // …with this error
	saveNoted string           // non-empty: save_areas answers success=false
}

func newFakeMapServer(initial ...mowgli.ReplaceMapArea) *fakeMapServer {
	return &fakeMapServer{
		MockRosProvider: types.NewMockRosProvider(),
		live:            append([]mowgli.ReplaceMapArea{}, initial...),
		disk:            append([]mowgli.ReplaceMapArea{}, initial...),
		calls:           map[string]int{},
		failAt:          map[string]int{},
		failWith:        map[string]error{},
	}
}

func (f *fakeMapServer) CallService(_ context.Context, service string, req any, res any, _ ...string) error {
	f.calls[service]++
	if n, ok := f.failAt[service]; ok && f.calls[service] == n {
		return f.failWith[service]
	}
	switch service {
	case mapServiceGetMowingArea:
		out := res.(*mowgli.GetMowingAreaRes)
		idx := int(req.(*mowgli.GetMowingAreaReq).Index)
		if idx < len(f.live) {
			out.Success = true
			out.Area = f.live[idx].Area
			out.Area.IsNavigationArea = f.live[idx].IsNavigationArea
		}
	case mapServiceClearMap:
		f.live = nil
		res.(*triggerRes).Success = true
	case mapServiceAddArea:
		add := req.(*mowgli.AddMowingAreaReq)
		if add.Area.Obstacles == nil || add.Area.ObstacleInfo == nil ||
			add.Area.ProposedObstacles == nil || add.Area.ProposedObstacleInfo == nil {
			return errors.New("msg is not a list type") // what foxglove_bridge says to a null
		}
		ok := len(add.Area.Area.Points) >= 3
		res.(*mowgli.AddMowingAreaRes).Success = ok
		if ok {
			f.live = append(f.live, mowgli.ReplaceMapArea{Area: add.Area, IsNavigationArea: add.IsNavigationArea})
			f.disk = append([]mowgli.ReplaceMapArea{}, f.live...) // add_area auto-saves
		}
	case mapServiceSaveAreas:
		out := res.(*triggerRes)
		if f.saveNoted != "" {
			out.Message = f.saveNoted
			return nil
		}
		out.Success = true
		f.disk = append([]mowgli.ReplaceMapArea{}, f.live...)
	default:
		return fmt.Errorf("unexpected service %s", service)
	}
	return nil
}

func (f *fakeMapServer) failNth(service string, n int, err error) {
	f.failAt[service] = n
	f.failWith[service] = err
}

func testArea(name string, id uint32) mowgli.ReplaceMapArea {
	return mowgli.ReplaceMapArea{Area: mowgli.MapArea{
		Name: name,
		Id:   id,
		Area: geometry.Polygon{Points: []geometry.Point32{{X: 0, Y: 0}, {X: 5, Y: 0}, {X: 5, Y: 5}}},
	}}
}

func areaNames(areas []mowgli.ReplaceMapArea) []string {
	names := []string{}
	for _, a := range areas {
		names = append(names, a.Area.Name)
	}
	return names
}

var errDeadline = fmt.Errorf("foxglove: CallService %s: %w", mapServiceAddArea, context.DeadlineExceeded)

func TestReplaceMap_SuccessWritesEveryAreaAndCommits(t *testing.T) {
	// Arrange
	srv := newFakeMapServer(testArea("old", 1))
	req := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("front", 1), testArea("back", 2)}}

	// Act
	err := replaceMapInternal(context.Background(), srv, req)

	// Assert
	require.NoError(t, err)
	assert.Equal(t, []string{"front", "back"}, areaNames(srv.live))
	assert.Equal(t, []string{"front", "back"}, areaNames(srv.disk))
	assert.Equal(t, 1, srv.calls[mapServiceSaveAreas])
}

func TestReplaceMap_UnreadableCurrentMapChangesNothing(t *testing.T) {
	// Arrange: map_server does not answer at all (busy / restarting).
	srv := newFakeMapServer(testArea("old", 1))
	srv.failNth(mapServiceGetMowingArea, 1, errDeadline)

	// Act
	err := replaceMapInternal(context.Background(), srv, &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("new", 0)}})

	// Assert
	require.Error(t, err)
	assert.Contains(t, err.Error(), "nothing was changed")
	assert.Zero(t, srv.calls[mapServiceClearMap], "the old map must not be cleared")
	assert.Equal(t, []string{"old"}, areaNames(srv.live))
}

func TestReplaceMap_FailureAtAnyAddRestoresThePreviousMap(t *testing.T) {
	for failing := 1; failing <= 3; failing++ {
		t.Run(fmt.Sprintf("add_area call %d fails", failing), func(t *testing.T) {
			// Arrange: two areas live; the replacement has three.
			srv := newFakeMapServer(testArea("old A", 7), testArea("old B", 9))
			srv.failNth(mapServiceAddArea, failing, errDeadline)
			req := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("n1", 0), testArea("n2", 0), testArea("n3", 0)}}

			// Act
			err := replaceMapInternal(context.Background(), srv, req)

			// Assert: no partial replacement stays live or on disk, ids survive,
			// and the message names the step and the outcome.
			require.Error(t, err)
			assert.Equal(t, []string{"old A", "old B"}, areaNames(srv.live))
			assert.Equal(t, []string{"old A", "old B"}, areaNames(srv.disk))
			assert.Equal(t, uint32(7), srv.live[0].Area.Id)
			assert.Contains(t, err.Error(), fmt.Sprintf("adding area %d/3", failing))
			assert.Contains(t, err.Error(), "previous map (2 areas) was restored")
			assert.ErrorIs(t, err, context.DeadlineExceeded)
		})
	}
}

func TestReplaceMap_ClearFailureRestoresThePreviousMap(t *testing.T) {
	srv := newFakeMapServer(testArea("old", 1))
	srv.failNth(mapServiceClearMap, 1, errors.New("foxglove: CallService: no connection"))

	err := replaceMapInternal(context.Background(), srv, &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("new", 0)}})

	require.Error(t, err)
	assert.Contains(t, err.Error(), "clearing the old map")
	assert.Equal(t, []string{"old"}, areaNames(srv.live))
}

func TestReplaceMap_RejectedAreaIsAnErrorNotASilentDrop(t *testing.T) {
	// Arrange: a degenerate polygon — map_server answers success=false.
	srv := newFakeMapServer(testArea("old", 1))
	bad := testArea("sliver", 0)
	bad.Area.Area.Points = bad.Area.Area.Points[:2]

	// Act
	err := replaceMapInternal(context.Background(), srv, &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("ok", 0), bad}})

	// Assert
	require.Error(t, err)
	assert.Contains(t, err.Error(), `"sliver"`)
	assert.Contains(t, err.Error(), "rejected")
	assert.Equal(t, []string{"old"}, areaNames(srv.live))
}

func TestReplaceMap_FailedRestoreSaysThePartialMapIsLive(t *testing.T) {
	// Arrange: the 2nd add fails, and so does the restore's clear_map (the
	// bridge went away for good).
	srv := newFakeMapServer(testArea("old", 1))
	srv.failNth(mapServiceAddArea, 2, errDeadline)
	srv.failNth(mapServiceClearMap, 2, errors.New("foxglove: CallService: no connection"))
	req := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("n1", 0), testArea("n2", 0)}}

	// Act
	err := replaceMapInternal(context.Background(), srv, req)

	// Assert
	require.Error(t, err)
	assert.Contains(t, err.Error(), "PARTIAL map")
	assert.Contains(t, err.Error(), "Do not start mowing")
}

func TestReplaceMap_SaveRefusedReportsLiveButNotPersisted(t *testing.T) {
	srv := newFakeMapServer(testArea("old", 1))
	srv.saveNoted = "Save failed: Cannot open /ros2_ws/maps/areas.dat.tmp for writing"

	err := replaceMapInternal(context.Background(), srv, &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("new", 0)}})

	require.Error(t, err)
	assert.Contains(t, err.Error(), "NOT written to disk")
	assert.Contains(t, err.Error(), "areas.dat.tmp")
	assert.Equal(t, []string{"new"}, areaNames(srv.live), "the new map stays live; only the commit failed")
}

func TestReplaceMap_ProposalsAndNilSlicesNeverReachAddArea(t *testing.T) {
	// Arrange: the live area carries a pending dig proposal (Invariant 16) and
	// the request leaves every repeated field nil.
	old := testArea("old", 1)
	old.Area.ProposedObstacles = []geometry.Polygon{{Points: []geometry.Point32{{X: 1, Y: 1}, {X: 2, Y: 1}, {X: 2, Y: 2}}}}
	old.Area.ProposedObstacleInfo = []mowgli.MapObstacleInfo{{Pending: true, Id: 4}}
	srv := newFakeMapServer(old)
	srv.failNth(mapServiceAddArea, 1, errDeadline) // force the restore path too

	// Act
	err := replaceMapInternal(context.Background(), srv, &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("new", 0)}})

	// Assert: the restore re-added "old" without turning the proposal into
	// anything, and without tripping the bridge's null check.
	require.Error(t, err)
	require.Len(t, srv.live, 1)
	assert.Empty(t, srv.live[0].Area.ProposedObstacles)
	assert.Empty(t, srv.live[0].Area.Obstacles)
	assert.Contains(t, err.Error(), "was restored")
}

func TestReplaceMapRoute_ReturnsTheActionableMessage(t *testing.T) {
	// Arrange
	srv := newFakeMapServer(testArea("old", 1))
	srv.failNth(mapServiceAddArea, 1, errDeadline)
	router := setupMowgliNextRouter(srv)
	body, err := json.Marshal(mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("new", 0)}})
	require.NoError(t, err)

	// Act
	w := httptest.NewRecorder()
	req, _ := http.NewRequest(http.MethodPut, "/api/mowglinext/map", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	// Assert
	assert.Equal(t, http.StatusInternalServerError, w.Code)
	var resp ErrorResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Contains(t, resp.Error, `adding area 1/1 "new"`)
	assert.Contains(t, resp.Error, "was restored")
}
