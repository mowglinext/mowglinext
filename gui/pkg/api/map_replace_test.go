package api

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"

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

type mapReplacementTestContextKey struct{}

type mapReplacementCall struct {
	transaction string
	service     string
}

// barrierMapServer pauses replacement A after its first area is live and fails
// A's second add. Replacement B can then be forced to attempt a complete save
// before A's rollback unless replaceMapInternal serializes the whole flow.
type barrierMapServer struct {
	*types.MockRosProvider
	mu              sync.Mutex
	live            []mowgli.ReplaceMapArea
	disk            []mowgli.ReplaceMapArea
	calls           []mapReplacementCall
	firstAddEntered chan struct{}
	releaseFirstAdd chan struct{}
}

func newBarrierMapServer(initial ...mowgli.ReplaceMapArea) *barrierMapServer {
	areas := append([]mowgli.ReplaceMapArea{}, initial...)
	return &barrierMapServer{
		MockRosProvider: types.NewMockRosProvider(),
		live:            append([]mowgli.ReplaceMapArea{}, areas...),
		disk:            append([]mowgli.ReplaceMapArea{}, areas...),
		firstAddEntered: make(chan struct{}),
		releaseFirstAdd: make(chan struct{}),
	}
}

func (f *barrierMapServer) CallService(ctx context.Context, service string, req any, res any, _ ...string) error {
	transaction, _ := ctx.Value(mapReplacementTestContextKey{}).(string)
	var pauseFirstAdd bool
	var err error

	f.mu.Lock()
	f.calls = append(f.calls, mapReplacementCall{transaction: transaction, service: service})
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
		if transaction == "A" && add.Area.Name == "a2" {
			err = errors.New("injected add_area failure")
			break
		}
		if len(add.Area.Area.Points) < 3 {
			res.(*mowgli.AddMowingAreaRes).Success = false
			break
		}
		res.(*mowgli.AddMowingAreaRes).Success = true
		f.live = append(f.live, mowgli.ReplaceMapArea{Area: add.Area, IsNavigationArea: add.IsNavigationArea})
		f.disk = append([]mowgli.ReplaceMapArea{}, f.live...)
		if transaction == "A" && add.Area.Name == "a1" {
			pauseFirstAdd = true
			close(f.firstAddEntered)
		}
	case mapServiceSaveAreas:
		res.(*triggerRes).Success = true
		f.disk = append([]mowgli.ReplaceMapArea{}, f.live...)
	default:
		err = fmt.Errorf("unexpected service %s", service)
	}
	f.mu.Unlock()

	if err != nil {
		return err
	}
	if pauseFirstAdd {
		<-f.releaseFirstAdd
	}
	return nil
}

func (f *barrierMapServer) state() (live, disk []mowgli.ReplaceMapArea, calls []mapReplacementCall) {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]mowgli.ReplaceMapArea{}, f.live...),
		append([]mowgli.ReplaceMapArea{}, f.disk...),
		append([]mapReplacementCall{}, f.calls...)
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

func TestReplaceMap_SerializesConcurrentReplacementThroughRollback(t *testing.T) {
	server := newBarrierMapServer(testArea("old", 1))
	requestA := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("a1", 0), testArea("a2", 0)}}
	requestB := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("b1", 0), testArea("b2", 0)}}

	releaseFirstAdd := sync.OnceFunc(func() { close(server.releaseFirstAdd) })
	defer releaseFirstAdd()

	ctxA := context.WithValue(context.Background(), mapReplacementTestContextKey{}, "A")
	aDone := make(chan error, 1)
	go func() { aDone <- replaceMapInternal(ctxA, server, requestA) }()
	select {
	case <-server.firstAddEntered:
	case <-time.After(5 * time.Second):
		t.Fatal("replacement A did not reach its first add_area call")
	}

	// A has already cleared the old map and added only a1. Its next add fails,
	// which makes it restore the older snapshot. B must not save its map before
	// that rollback completes, or A would later overwrite B with the stale copy.
	ctxB := context.WithValue(context.Background(), mapReplacementTestContextKey{}, "B")
	bDone := make(chan error, 1)
	bWaiting := make(chan struct{})
	go func() {
		bDone <- replaceMapInternalWithLockWait(ctxB, server, requestB, func() { close(bWaiting) })
	}()

	var earlyBErr error
	bFinishedBeforeARelease := false
	select {
	case <-bWaiting:
		// The try-send observed A's held token immediately before this wait.
	case earlyBErr = <-bDone:
		bFinishedBeforeARelease = true
	case <-time.After(5 * time.Second):
		releaseFirstAdd()
		<-aDone
		<-bDone
		t.Fatal("replacement B neither waited for the transaction lock nor completed")
	}
	_, _, callsWhileAPaused := server.state()
	for _, call := range callsWhileAPaused {
		assert.Equal(t, "A", call.transaction, "replacement B must not call map_server during A's transaction")
	}

	releaseFirstAdd()
	aErr := <-aDone
	var bErr error
	if bFinishedBeforeARelease {
		bErr = earlyBErr
	} else {
		bErr = <-bDone
	}

	require.Error(t, aErr)
	assert.Contains(t, aErr.Error(), "previous map (1 areas) was restored")
	require.NoError(t, bErr)
	if bFinishedBeforeARelease {
		t.Error("replacement B completed while replacement A was paused before rollback")
	}

	live, disk, calls := server.state()
	assert.Equal(t, []string{"b1", "b2"}, areaNames(live))
	assert.Equal(t, []string{"b1", "b2"}, areaNames(disk))
	firstBCall := -1
	for i, call := range calls {
		if call.transaction == "B" {
			firstBCall = i
			break
		}
	}
	require.NotEqual(t, -1, firstBCall, "replacement B should reach map_server")
	for _, call := range calls[:firstBCall] {
		assert.Equal(t, "A", call.transaction)
	}
	for _, call := range calls[firstBCall:] {
		assert.Equal(t, "B", call.transaction)
	}
}

func TestReplaceMap_CanceledContextDoesNotTouchMapServer(t *testing.T) {
	server := newFakeMapServer(testArea("old", 1))
	ctx, cancel := context.WithCancel(context.Background())
	cancel()

	err := replaceMapInternal(ctx, server, &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("new", 0)}})

	require.ErrorIs(t, err, context.Canceled)
	assert.Empty(t, server.calls)
	assert.Equal(t, []string{"old"}, areaNames(server.live))
}

func TestReplaceMap_CanceledWaitDoesNotTouchMapServer(t *testing.T) {
	server := newBarrierMapServer(testArea("old", 1))
	releaseFirstAdd := sync.OnceFunc(func() { close(server.releaseFirstAdd) })
	defer releaseFirstAdd()

	ctxA := context.WithValue(context.Background(), mapReplacementTestContextKey{}, "A")
	aDone := make(chan error, 1)
	go func() {
		aDone <- replaceMapInternal(ctxA, server, &mowgli.ReplaceMapReq{
			Areas: []mowgli.ReplaceMapArea{testArea("a1", 0), testArea("a2", 0)},
		})
	}()
	select {
	case <-server.firstAddEntered:
	case <-time.After(5 * time.Second):
		t.Fatal("replacement A did not reach its first add_area call")
	}

	ctxB, cancelB := context.WithCancel(context.WithValue(context.Background(), mapReplacementTestContextKey{}, "B"))
	defer cancelB()
	bWaiting := make(chan struct{})
	bDone := make(chan error, 1)
	go func() {
		bDone <- replaceMapInternalWithLockWait(ctxB, server, &mowgli.ReplaceMapReq{
			Areas: []mowgli.ReplaceMapArea{testArea("b", 0)},
		}, func() { close(bWaiting) })
	}()
	select {
	case <-bWaiting:
	case <-time.After(5 * time.Second):
		releaseFirstAdd()
		<-aDone
		<-bDone
		t.Fatal("replacement B did not reach the transaction lock")
	}

	cancelB()
	var bErr error
	select {
	case bErr = <-bDone:
	case <-time.After(5 * time.Second):
		releaseFirstAdd()
		<-aDone
		t.Fatal("replacement B did not stop after its context was canceled")
	}

	_, _, calls := server.state()
	for _, call := range calls {
		assert.NotEqual(t, "B", call.transaction, "a canceled waiter must not call map_server")
	}
	releaseFirstAdd()
	aErr := <-aDone
	require.ErrorIs(t, bErr, context.Canceled)
	require.Error(t, aErr, "replacement A should take its injected failure and restore the old map")
	live, disk, _ := server.state()
	assert.Equal(t, []string{"old"}, areaNames(live))
	assert.Equal(t, []string{"old"}, areaNames(disk))
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
