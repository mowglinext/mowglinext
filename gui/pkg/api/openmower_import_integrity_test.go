package api

import (
	"bytes"
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// The map and dock deliberately have independent state: no fake transaction
// makes a dock failure undo the already-persisted areas.
type importStateServer struct {
	*fakeMapServer
	dock          geometry.Pose
	dockDisk      geometry.Pose
	dockResult    string
	after         func(string)
	clearRejected bool
}

func newImportStateServer() *importStateServer {
	initialDock := geometry.Pose{Position: geometry.Point{X: 99}, Orientation: geometry.Quaternion{W: 1}}
	return &importStateServer{fakeMapServer: newFakeMapServer(testArea("A", 17)),
		dock: initialDock, dockDisk: initialDock}
}

func (f *importStateServer) CallService(ctx context.Context, service string, req, res any, srvType ...string) error {
	if service != "/map_server_node/set_docking_point" {
		if service == mapServiceClearMap && f.clearRejected {
			f.calls[service]++
			res.(*triggerRes).Message = "injected clear refusal"
			return nil
		}
		err := f.fakeMapServer.CallService(ctx, service, req, res, srvType...)
		if f.after != nil {
			f.after(service)
		}
		return err
	}
	f.calls[service]++
	out := res.(*mowgli.SetDockingPointRes)
	if f.dockResult == "rejected" {
		out.Message = "injected dock gate refusal"
		out.StoredPose = f.dock
		return nil
	}
	request := req.(*mowgli.SetDockingPointReq)
	pose := request.DockingPose
	if request.YawSource == 0 {
		pose.Orientation = f.dock.Orientation
	} // PRESERVE, as on the real service
	f.dock = pose
	if f.dockResult != "memory only" {
		f.dockDisk = pose
	}
	if f.dockResult == "transport" {
		// The command took effect, but its acknowledgement never arrived.
		return context.DeadlineExceeded
	}
	out.Success = true
	out.StoredPose = f.dock
	if f.dockResult == "memory only" {
		out.Message = "applied in-memory only: could not write fixture.yaml"
	}
	return nil
}

func importHTTP(t *testing.T, server *importStateServer, apply bool) *httptest.ResponseRecorder {
	t.Helper()
	return serveImport(t, server, context.Background(), sampleOpenMowerMap, apply)
}

func serveImport(t *testing.T, server types.IRosProvider, ctx context.Context, source string, apply bool) *httptest.ResponseRecorder {
	t.Helper()
	body, err := json.Marshal(ImportOpenMowerRequest{Map: json.RawMessage(source), Apply: apply})
	require.NoError(t, err)
	w := httptest.NewRecorder()
	setupImportRouter(server, nil).ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/api/import/openmower", bytes.NewReader(body)).WithContext(ctx))
	return w
}

func requireImportOutcome(t *testing.T, w *httptest.ResponseRecorder, code int, mapStatus MapWriteStatus, dockStatus DockWriteStatus) {
	t.Helper()
	require.Equal(t, code, w.Code, w.Body.String())
	var body map[string]json.RawMessage
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
	assert.JSONEq(t, `"`+string(mapStatus)+`"`, string(body["map_status"]))
	assert.JSONEq(t, `"`+string(dockStatus)+`"`, string(body["dock_status"]))
	if code != http.StatusOK {
		require.Contains(t, body, "error")
		assert.NotContains(t, body, "applied")
	} else {
		require.Contains(t, body, "applied")
	}
}

func TestPostImportOpenMower_DockFailureKeepsPersistedNewMap(t *testing.T) {
	for _, outcome := range []string{"rejected", "transport"} {
		t.Run(outcome, func(t *testing.T) {
			server := newImportStateServer()
			oldDock := server.dock
			server.dockResult = outcome
			w := importHTTP(t, server, true)
			require.Equal(t, http.StatusInternalServerError, w.Code)
			assert.Equal(t, []string{"Front lawn", "Driveway"}, areaNames(server.disk))
			assert.Equal(t, areaNames(server.disk), areaNames(server.live))
			assert.Equal(t, 1, server.calls[mapServiceClearMap], "no rollback after committing B")
			assert.Equal(t, 1, server.calls[mapServiceSaveAreas])
			assert.Equal(t, 1, server.calls["/map_server_node/set_docking_point"])
			if outcome == "rejected" {
				assert.Equal(t, oldDock, server.dock)
				assert.Equal(t, oldDock, server.dockDisk)
			} else {
				assert.NotEqual(t, oldDock, server.dock, "transport failure does not prove the dock stayed unchanged")
				assert.Equal(t, server.dock, server.dockDisk)
			}
			assert.Equal(t, oldDock.Orientation, server.dock.Orientation)
			var response map[string]any
			require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
			assert.Equal(t, "persisted", response["map_status"])
			status := dockRejected
			if outcome == "transport" {
				status = dockUnconfirmed
			}
			requireImportOutcome(t, w, http.StatusInternalServerError, mapPersisted, status)
		})
	}
}

func TestPostImportOpenMower_ObservableOutcomes(t *testing.T) {
	t.Run("preview does not even read ROS", func(t *testing.T) {
		server := newImportStateServer()
		w := importHTTP(t, server, false)
		requireImportOutcome(t, w, http.StatusOK, mapUnchanged, dockNotAttempted)
		assert.Empty(t, server.calls)
		assert.Equal(t, []string{"A"}, areaNames(server.disk))
		assert.Equal(t, 99.0, server.dock.Position.X)
	})
	t.Run("map and dock acknowledged", func(t *testing.T) {
		server := newImportStateServer()
		w := importHTTP(t, server, true)
		requireImportOutcome(t, w, http.StatusOK, mapPersisted, dockConfirmed)
		assert.Equal(t, []string{"Front lawn", "Driveway"}, areaNames(server.disk))
		assert.Equal(t, -0.5, server.dock.Position.X)
	})
	t.Run("no dock requested", func(t *testing.T) {
		server := newImportStateServer()
		var source openMowerMap
		require.NoError(t, json.Unmarshal([]byte(sampleOpenMowerMap), &source))
		source.DockingStations = nil
		raw, err := json.Marshal(source)
		require.NoError(t, err)
		w := serveImport(t, server, context.Background(), string(raw), true)
		requireImportOutcome(t, w, http.StatusOK, mapPersisted, dockNotRequested)
		assert.Zero(t, server.calls["/map_server_node/set_docking_point"])
		assert.Equal(t, 99.0, server.dock.Position.X)
	})
	t.Run("dock persistence warning", func(t *testing.T) {
		server := newImportStateServer()
		server.dockResult = "memory only"
		w := importHTTP(t, server, true)
		requireImportOutcome(t, w, http.StatusInternalServerError, mapPersisted, dockActiveUnconfirmed)
		assert.Equal(t, -0.5, server.dock.Position.X)
		assert.Equal(t, 99.0, server.dockDisk.Position.X)
		assert.Equal(t, 1, server.calls[mapServiceClearMap])
	})
	for _, failure := range []string{"snapshot", "add restored", "restore failed", "save refused", "save transport", "clear rejected"} {
		t.Run(failure, func(t *testing.T) {
			server := newImportStateServer()
			expected := mapUnchanged
			switch failure {
			case "snapshot":
				server.failNth(mapServiceGetMowingArea, 1, context.DeadlineExceeded)
			case "add restored", "restore failed":
				server.failNth(mapServiceAddArea, 2, context.DeadlineExceeded)
				expected = mapRestored
				if failure == "restore failed" {
					server.failNth(mapServiceClearMap, 2, context.DeadlineExceeded)
					expected = mapUnconfirmed
				}
			case "save refused":
				server.saveNoted = "injected storage refusal"
				expected = mapActiveUnconfirmed
			case "save transport":
				server.failNth(mapServiceSaveAreas, 1, context.DeadlineExceeded)
				expected = mapActiveUnconfirmed
			case "clear rejected":
				server.clearRejected = true
				expected = mapUnconfirmed
			}
			w := importHTTP(t, server, true)
			requireImportOutcome(t, w, http.StatusInternalServerError, expected, dockNotAttempted)
			assert.Zero(t, server.calls["/map_server_node/set_docking_point"])
			assert.Equal(t, 99.0, server.dock.Position.X)
			if expected == mapUnchanged || expected == mapRestored || failure == "clear rejected" {
				assert.Equal(t, []string{"A"}, areaNames(server.live))
				assert.Equal(t, []string{"A"}, areaNames(server.disk))
			}
			if expected == mapActiveUnconfirmed {
				assert.Equal(t, []string{"Front lawn", "Driveway"}, areaNames(server.live))
				// add_area may auto-save before the explicit save_areas fails.
				assert.Equal(t, areaNames(server.live), areaNames(server.disk))
			}
			if failure == "restore failed" {
				assert.Equal(t, []string{"Front lawn"}, areaNames(server.live))
				assert.Equal(t, []string{"Front lawn"}, areaNames(server.disk))
			}
			if failure == "snapshot" {
				assert.Zero(t, server.calls[mapServiceClearMap])
			}
		})
	}
}

func TestPostImportOpenMower_CancellationBoundaries(t *testing.T) {
	for _, boundary := range []string{"before start", "expired", "after snapshot", "after clear", "after save"} {
		t.Run(boundary, func(t *testing.T) {
			server := newImportStateServer()
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			expected := mapUnchanged
			switch boundary {
			case "before start":
				cancel()
			case "expired":
				var expire context.CancelFunc
				ctx, expire = context.WithDeadline(ctx, time.Now().Add(-time.Second))
				defer expire()
			case "after snapshot", "after clear", "after save":
				server.after = func(service string) {
					if boundary == "after snapshot" && service == mapServiceGetMowingArea && server.calls[service] == 2 ||
						boundary == "after clear" && service == mapServiceClearMap ||
						boundary == "after save" && service == mapServiceSaveAreas {
						cancel()
					}
				}
				if boundary == "after clear" {
					expected = mapRestored
				}
				if boundary == "after save" {
					expected = mapPersisted
				}
			}
			w := serveImport(t, server, ctx, sampleOpenMowerMap, true)
			requireImportOutcome(t, w, http.StatusInternalServerError, expected, dockNotAttempted)
			assert.Zero(t, server.calls["/map_server_node/set_docking_point"])
			if expected == mapUnchanged {
				assert.Zero(t, server.calls[mapServiceClearMap])
			}
			if expected == mapRestored {
				assert.Equal(t, []string{"A"}, areaNames(server.disk))
			}
			if expected == mapPersisted {
				assert.Equal(t, []string{"Front lawn", "Driveway"}, areaNames(server.disk))
			}
		})
	}
}

// Synchronize only the fake state; pause outside its mutex so an incorrectly
// unlocked second request would actually be able to modify the map.
type importBarrierServer struct {
	*importStateServer
	mu                 sync.Mutex
	callsByTransaction []mapReplacementCall
	dockEntered        chan struct{}
	releaseDock        chan struct{}
}

func (s *importBarrierServer) CallService(ctx context.Context, service string, req, res any, srvType ...string) error {
	transaction, _ := ctx.Value(mapReplacementTestContextKey{}).(string)
	s.mu.Lock()
	s.callsByTransaction = append(s.callsByTransaction, mapReplacementCall{transaction, service})
	s.mu.Unlock()
	if transaction == "A" && service == "/map_server_node/set_docking_point" {
		close(s.dockEntered)
		select {
		case <-s.releaseDock:
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.importStateServer.CallService(ctx, service, req, res, srvType...)
}

func importContext(ctx context.Context) *gin.Context {
	c, _ := gin.CreateTestContext(httptest.NewRecorder())
	c.Request = httptest.NewRequest(http.MethodPost, "/api/import/openmower", nil).WithContext(ctx)
	return c
}

func TestImportSerializesThroughDock(t *testing.T) {
	for _, second := range []string{"import", "replacement", "canceled waiter"} {
		t.Run(second, func(t *testing.T) {
			server := &importBarrierServer{importStateServer: newImportStateServer(), dockEntered: make(chan struct{}), releaseDock: make(chan struct{})}
			release := sync.OnceFunc(func() { close(server.releaseDock) })
			defer release()
			ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
			defer cancel()
			aDone := make(chan error, 1)
			go func() {
				aDone <- applyImport(importContext(context.WithValue(ctx, mapReplacementTestContextKey{}, "A")), server,
					&mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("import A", 1)}}, &mowgli.SetDockingPointReq{DockingPose: geometry.Pose{Position: geometry.Point{X: 1}}})
			}()
			select {
			case <-server.dockEntered:
			case <-ctx.Done():
				t.Fatal("A never reached dock")
			}
			bCtx, cancelB := context.WithCancel(context.WithValue(ctx, mapReplacementTestContextKey{}, "B"))
			defer cancelB()
			waiting, bDone := make(chan struct{}), make(chan error, 1)
			req := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{testArea("replacement B", 2)}}
			go func() {
				if second == "replacement" {
					bDone <- replaceMapInternalWithLockWait(bCtx, server, req, func() { close(waiting) })
				} else {
					var dock *mowgli.SetDockingPointReq
					if second == "import" {
						dock = &mowgli.SetDockingPointReq{DockingPose: geometry.Pose{Position: geometry.Point{X: 2}}}
					}
					bDone <- applyImportWithLockWait(importContext(bCtx), server, req, dock, func() { close(waiting) })
				}
			}()
			select {
			case <-waiting:
			case err := <-bDone:
				t.Fatalf("B bypassed held lock: %v", err)
			case <-ctx.Done():
				t.Fatal("B never reached lock")
			}
			server.mu.Lock()
			calls := append([]mapReplacementCall{}, server.callsByTransaction...)
			server.mu.Unlock()
			for _, call := range calls {
				assert.Equal(t, "A", call.transaction)
			}
			if second == "canceled waiter" {
				cancelB()
				require.ErrorIs(t, <-bDone, context.Canceled)
			}
			release()
			require.NoError(t, <-aDone)
			if second != "canceled waiter" {
				require.NoError(t, <-bDone)
			}
			server.mu.Lock()
			defer server.mu.Unlock()
			expected := []string{"replacement B"}
			if second == "canceled waiter" {
				expected = []string{"import A"}
			}
			assert.Equal(t, expected, areaNames(server.disk))
			seenB := false
			for _, call := range server.callsByTransaction {
				if call.transaction == "B" {
					seenB = true
				}
				if seenB {
					assert.Equal(t, "B", call.transaction, "no A operation may run after B started")
				}
			}
			expectedDock := 1.0
			if second == "import" {
				expectedDock = 2
			}
			assert.Equal(t, expectedDock, server.dock.Position.X)
		})
	}
}

func TestMapMutationRoutesCanceledBehindImportDoNotWrite(t *testing.T) {
	for _, route := range []struct{ method, path, body string }{
		{http.MethodDelete, "/api/mowglinext/map", ""},
		{http.MethodPost, "/api/mowglinext/map/area/add", `{"area":{"name":"B","area":{"points":[]},"obstacles":[]}}`},
		{http.MethodPost, "/api/mowglinext/map/docking", `{"docking_pose":{"position":{"x":1},"orientation":{"w":1}}}`},
	} {
		t.Run(route.path+route.method, func(t *testing.T) {
			server := newImportStateServer()
			router := gin.New()
			group := router.Group("/api/mowglinext")
			AddMapAreaRoute(group, server)
			ClearMapRoute(group, server)
			SetDockingPointRoute(group, server)
			require.NoError(t, lockMapReplacement(context.Background(), nil))
			defer unlockMapReplacement()
			ctx, cancel := context.WithCancel(context.Background())
			cancel()
			w := httptest.NewRecorder()
			router.ServeHTTP(w, httptest.NewRequest(route.method, route.path, bytes.NewBufferString(route.body)).WithContext(ctx))
			assert.Equal(t, http.StatusInternalServerError, w.Code)
			assert.Contains(t, w.Body.String(), "context canceled")
			assert.Empty(t, server.calls)
			assert.Equal(t, []string{"A"}, areaNames(server.disk))
			assert.Equal(t, 99.0, server.dock.Position.X)
		})
	}
}
