package providers

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/gorilla/websocket"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

func TestFleetPeerIdentityInvalidationAndReadmission(t *testing.T) {
	type socketEvent struct {
		conn          *websocket.Conn
		ready, closed chan struct{}
		pong          chan struct{}
		subscriptions atomic.Int32
	}
	var advertised atomic.Value
	advertised.Store("alpha")
	events := make(chan *socketEvent, 4)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == peerIdentityPath {
			_ = json.NewEncoder(w).Encode(RobotIdentity{ID: advertised.Load().(string), Name: "peer", APIVersion: FleetAPIVersion})
			return
		}
		u := websocket.Upgrader{}
		conn, err := u.Upgrade(w, r, nil)
		if err != nil {
			return
		}
		defer conn.Close()
		e := &socketEvent{conn: conn, ready: make(chan struct{}), closed: make(chan struct{}), pong: make(chan struct{}, 1)}
		defer close(e.closed)
		conn.SetPongHandler(func(string) error {
			select {
			case e.pong <- struct{}{}:
			default:
			}
			return nil
		})
		events <- e
		for range FleetTopics {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
			e.subscriptions.Add(1)
		}
		close(e.ready)
		for {
			if _, _, err := conn.ReadMessage(); err != nil {
				return
			}
		}
	}))
	defer server.Close()
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters: {}\n")
	p := newPeerClient(strings.TrimPrefix(server.URL, "http://"), time.Now)
	p.expectedID = "alpha"
	p.cache.expectedID = "alpha"
	defer p.cancel()
	f := &FleetProvider{db: db, now: time.Now, self: newPeerCache(), peers: map[string]FleetPeer{"alpha": {ID: "alpha", Address: p.address, APIVersion: 1}}, clients: map[string]*peerClient{"alpha": p}}
	p.onIdentityUnavailable = func() { f.updatePeerAPIVersion("alpha", p.address, p, 0) }
	p.onIdentity = func(id RobotIdentity) { f.updatePeerAPIVersion("alpha", p.address, p, id.APIVersion) }
	wait := func(ch <-chan struct{}) {
		t.Helper()
		select {
		case <-ch:
		case <-time.After(3 * time.Second):
			t.Fatal("socket barrier not reached")
		}
	}
	start := func() (*socketEvent, chan error) {
		t.Helper()
		done := make(chan error, 1)
		go func() { _, err := p.session(); done <- err }()
		select {
		case e := <-events:
			return e, done
		case <-time.After(3 * time.Second):
			t.Fatal("no socket")
			return nil, nil
		}
	}
	finish := func(e *socketEvent, done <-chan error) {
		t.Helper()
		_ = e.conn.Close()
		select {
		case err := <-done:
			require.Error(t, err)
		case <-time.After(3 * time.Second):
			t.Fatal("session did not stop")
		}
		wait(e.closed)
	}
	send := func(e *socketEvent, topic string, data any) {
		t.Helper()
		frame, err := msgpack.Marshal(map[string]any{"topic": topic, "data": data})
		require.NoError(t, err)
		require.NoError(t, e.conn.WriteMessage(websocket.BinaryMessage, frame))
		// Pong is a wire-order barrier: the frame has passed the reader/store.
		require.NoError(t, e.conn.WriteControl(websocket.PingMessage, []byte("barrier"), time.Now().Add(time.Second)))
		wait(e.pong)
	}
	row := func() FleetRobot { t.Helper(); rows, err := f.Robots(); require.NoError(t, err); return rows[1] }
	e, done := start()
	wait(e.ready)
	send(e, "highLevelStatus", map[string]any{"state": 2, "marker": "old-alpha"})
	require.True(t, row().Online)
	advertised.Store("beta")
	require.ErrorContains(t, p.refreshIdentity(context.Background()), "identity differs")
	// Active subscriptions can still deliver in-flight Beta data; it is ignored.
	send(e, "highLevelStatus", map[string]any{"state": 2, "marker": "beta"})
	send(e, "pose", map[string]any{"pose": map[string]any{"pose": map[string]any{"position": map[string]any{"x": 1}}}})
	r := row()
	assert.False(t, r.Online)
	assert.Empty(t, r.Topics)
	assert.Empty(t, r.TopicAgeS)
	assert.Nil(t, r.LastSeen)
	assert.Empty(t, peerMapPoses([]FleetRobot{r}, LatLon{}, peerPoseMaxAge))
	status, _, err := f.Call(context.Background(), "alpha", "high_level_control", json.RawMessage(`{"Command":1}`))
	assert.Equal(t, http.StatusConflict, status)
	require.Error(t, err)
	finish(e, done)
	// A contradictory reconnect never subscribes, despite the old cached HLS.
	e, done = start()
	finish(e, done)
	assert.Zero(t, e.subscriptions.Load())
	assert.Empty(t, row().Topics)
	advertised.Store("alpha")
	e, done = start()
	wait(e.ready)
	assert.False(t, row().Online, "qualification alone cannot revive the old HLS")
	assert.Empty(t, row().Topics)
	send(e, "highLevelStatus", map[string]any{"state": 1, "marker": "new-alpha"})
	r = row()
	assert.True(t, r.Online)
	assert.Contains(t, string(r.Topics["highLevelStatus"]), "new-alpha")
	assert.NotContains(t, string(r.Topics["highLevelStatus"]), "old-alpha")
	assert.NotContains(t, r.Topics, "pose")
	finish(e, done)
	assert.Empty(t, row().Topics, "disconnect clears this socket's observations")
}

func TestFleetPeerFailedIdentityRefreshClearsCachedData(t *testing.T) {
	for _, body := range []string{`{}`, `{"id":42}`, `{"id":"alpha","api_version":2}`, `{"id":"alpha","api_version":1} {}`} {
		t.Run(body, func(t *testing.T) {
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) { _, _ = w.Write([]byte(body)) }))
			defer server.Close()
			p := newPeerClient(strings.TrimPrefix(server.URL, "http://"), time.Now)
			defer p.cancel()
			p.expectedID = "alpha"
			p.cache.expectedID = "alpha"
			p.cache.setIdentity(RobotIdentity{ID: "alpha", APIVersion: 1})
			p.cache.setSocket(true)
			p.cache.store("highLevelStatus", json.RawMessage(`{"state":2}`), time.Now())
			require.Error(t, p.refreshIdentity(context.Background()))
			_, topics, ages, seen, online := p.cache.snapshotForIdentity("alpha", time.Now())
			assert.Empty(t, topics)
			assert.Empty(t, ages)
			assert.True(t, seen.IsZero())
			assert.False(t, online)
		})
	}
}

func TestFleetPeerReplacedClientCannotRequalifyRegistry(t *testing.T) {
	f := &FleetProvider{db: types.NewMockDBProvider(), peers: map[string]FleetPeer{"alpha": {ID: "alpha", Address: "address", APIVersion: 0}}, clients: map[string]*peerClient{"alpha": {}}}
	f.updatePeerAPIVersion("alpha", "address", &peerClient{}, 1)
	assert.Zero(t, f.peers["alpha"].APIVersion)
}

func TestFleetMapPushQualifiesDestination(t *testing.T) {
	for _, tc := range []struct {
		name, identity string
		identityStatus int
		ok             bool
	}{
		{"correct", `{"id":"alpha","name":"Alpha","api_version":1,"datum_lat":48,"datum_lon":2}`, 200, true},
		{"reassigned", `{"id":"beta","name":"Beta","api_version":1,"datum_lat":48,"datum_lon":2}`, 200, false},
		{"incompatible", `{"id":"alpha","api_version":2,"datum_lat":48,"datum_lon":2}`, 200, false},
		{"missing-id", `{"api_version":1,"datum_lat":48,"datum_lon":2}`, 200, false},
		{"malformed", `{"id":42}`, 200, false},
		{"trailing-json", `{"id":"alpha","api_version":1,"datum_lat":48,"datum_lon":2} {}`, 200, false},
		{"different-datum", `{"id":"alpha","api_version":1,"datum_lat":49,"datum_lon":2}`, 200, false},
		{"identity-unavailable", `{}`, 503, false},
		{"network", ``, 0, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			db := types.NewMockDBProvider()
			writeRobotYaml(t, db, "mowgli:\n  ros__parameters:\n    datum_lat: 48\n    datum_lon: 2\n")
			ros := types.NewMockRosProvider()
			ros.ServiceResponder = func(_ string, req any, res any) {
				if r, ok := res.(*mowgli.GetMowingAreaRes); ok {
					r.Success = req.(*mowgli.GetMowingAreaReq).Index == 0
				}
			}
			var writes atomic.Int32
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				if r.Method == http.MethodPut {
					writes.Add(1)
					_, _ = w.Write([]byte(`{}`))
					return
				}
				w.WriteHeader(tc.identityStatus)
				_, _ = w.Write([]byte(tc.identity))
			}))
			defer server.Close()
			if tc.name == "network" {
				server.Close()
			}
			f := &FleetProvider{db: db, ros: ros, http: server.Client(), peers: map[string]FleetPeer{"alpha": {ID: "alpha", Name: "Alpha", Address: strings.TrimPrefix(server.URL, "http://"), APIVersion: 1}}}
			res, err := f.PushMap(context.Background())
			require.NoError(t, err)
			require.Len(t, res.Peers, 1)
			assert.Equal(t, tc.ok, res.Peers[0].OK)
			assert.Equal(t, "alpha", res.Peers[0].ID)
			if tc.ok {
				assert.Equal(t, int32(1), writes.Load())
			} else {
				assert.Zero(t, writes.Load())
				assert.NotEmpty(t, res.Peers[0].Error)
			}
		})
	}
}

func TestFleetSnapshotRejectsUnqualifiedIdentity(t *testing.T) {
	for _, id := range []RobotIdentity{{ID: "beta", APIVersion: 1}, {ID: "alpha", APIVersion: 2}, {}} {
		t.Run(id.ID+string(rune('0'+id.APIVersion)), func(t *testing.T) {
			db := types.NewMockDBProvider()
			writeRobotYaml(t, db, "mowgli:\n  ros__parameters: {}\n")
			cache := newPeerCache()
			cache.setIdentity(id)
			cache.setSocket(true)
			cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":3}`), time.Now())
			cache.store("pose", json.RawMessage(`{"pose":{"pose":{"position":{"x":1}}}}`), time.Now())
			f := &FleetProvider{db: db, now: time.Now, self: newPeerCache(), peers: map[string]FleetPeer{"alpha": {ID: "alpha", APIVersion: 0}}, clients: map[string]*peerClient{"alpha": {cache: cache}}}
			rows, err := f.Robots()
			require.NoError(t, err)
			require.Len(t, rows, 2)
			assert.Equal(t, "alpha", rows[1].Identity.ID)
			assert.False(t, rows[1].Online)
			assert.Empty(t, rows[1].Topics)
			assert.Empty(t, rows[1].TopicAgeS)
			assert.Nil(t, rows[1].LastSeen)
			assert.Empty(t, peerMapPoses(rows, LatLon{}, peerPoseMaxAge))
		})
	}
}

// The proximity yield goes through the owned fleet_yield contract (#1009),
// never a bare HighLevelControl STOP/START. A refusal or transport failure of
// that contract must still be visible on the Fleet page (#1017).
func yieldRefusalRows() []FleetRobot {
	return []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true, Topics: map[string]json.RawMessage{"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":0}`), "gps": gpsTopic(48, 2)}, TopicAgeS: map[string]float64{"gps": 0}},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":1}`), "gps": gpsTopic(48.00001, 2)}, TopicAgeS: map[string]float64{"gps": 0}},
	}
}

func TestFleetCoordinatorReportsFleetYieldAcknowledgement(t *testing.T) {
	for _, tc := range []struct {
		name  string
		query uint8
		pause uint8
		err   error
		want  string
	}{
		{"held", fleetAvailable, fleetHeld, nil, ""},
		{"query refused", fleetBusy, fleetHeld, nil, "no unowned hold"},
		{"pause refused", fleetAvailable, fleetIneligible, nil, "terminal result"},
		{"transport", fleetAvailable, fleetHeld, errors.New("ROS down"), "fleet_yield unavailable or call failed (requires compatible ROS stack): ROS down"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rows := yieldRefusalRows()
			c, _, ros := newTestCoordinator(t, &rows)
			ros.ServiceResponder = func(_ string, req any, res any) {
				switch r := res.(type) {
				case *mowgli.SetFleetAssignmentRes:
					r.Success = true
				case *mowgli.FleetYieldRes:
					r.Epoch = "e1"
					r.Result = tc.query
					if req.(*mowgli.FleetYieldReq).Operation == fleetPause {
						r.Result = tc.pause
						r.Held = tc.pause == fleetHeld
					}
				}
			}
			ros.ServiceErr = tc.err
			require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
			if tc.want == "" {
				assert.Empty(t, c.Status().LastError)
				assert.True(t, c.yield.Yielded)
			} else {
				assert.Contains(t, c.Status().LastError, tc.want)
			}
		})
	}
}

func TestFleetCoordinatorYieldRefusalIsVisible(t *testing.T) {
	rows := yieldRefusalRows()
	c, _, ros := newTestCoordinator(t, &rows)
	ros.ServiceResponder = func(_ string, req any, res any) {
		switch r := res.(type) {
		case *mowgli.SetFleetAssignmentRes:
			r.Success = true
		case *mowgli.FleetYieldRes:
			r.Epoch, r.Result = "e1", fleetAvailable
			if req.(*mowgli.FleetYieldReq).Operation == fleetPause {
				r.Result, r.Message = fleetIneligible, "not mowing"
			}
		}
	}
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	assert.Empty(t, serviceCalls(ros, highLevelControlService), "the yield must never fall back to a bare STOP")
	var pauses int
	for _, call := range serviceCalls(ros, fleetYieldService) {
		if call.Req.(*mowgli.FleetYieldReq).Operation == fleetPause {
			pauses++
		}
	}
	assert.Equal(t, 1, pauses)
	assert.Contains(t, c.Status().LastError, "terminal result")
	assert.Contains(t, c.Status().LastError, "not mowing")
}
