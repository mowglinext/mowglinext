package providers

import (
	"context"
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestFleetSession_AstraStaleSessionDoesNotOverrideHLS(t *testing.T) {
	r := FleetRobot{Identity: RobotIdentity{ID: "peer"}, Online: true,
		Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":5}`),
			"coverageSession": json.RawMessage(`{"session_active":true,"current_area":2,"completed_areas":[1]}`),
		}, TopicAgeS: map[string]float64{"highLevelStatus": 0, "coverageSession": 120}}
	m := memberFromRobotAt(r, LatLon{}, peerPoseMaxAge)
	assert.Equal(t, 5, m.CurrentArea)
	assert.Empty(t, m.Completed)
}

func TestFleetSession_AstraRepeatedSnapshotExpires(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	m := fleetMember{ID: "peer", Online: true, Completed: []uint32{1}}
	memory := updateCompletedMemory(map[uint32]time.Time{1: now}, []fleetMember{m}, now.Add(24*time.Hour), 12*time.Hour, map[string]completionObservation{})
	assert.Empty(t, memory, "cached completed set cannot resurrect an expired completion")
}

func sessionTestCache(now time.Time) *peerCache {
	c := newPeerCache()
	c.expectedID = "peer"
	c.setIdentity(RobotIdentity{ID: "peer", APIVersion: FleetAPIVersion})
	c.setSocket(true)
	c.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), now)
	c.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), now)
	return c
}

func TestFleetSession_Qualification(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	for _, tc := range []struct {
		name  string
		edit  func(*FleetRobot)
		state coverageState
		area  int
	}{
		{"fresh", func(*FleetRobot) {}, coverageFresh, 5},
		{"fresh-at-existing-liveness-boundary", func(r *FleetRobot) { r.TopicAgeS["coverageSession"] = peerOnlineMaxAge.Seconds() }, coverageFresh, 5},
		{"stale", func(r *FleetRobot) { r.TopicAgeS["coverageSession"] = 120 }, coverageStale, 5},
		{"absent", func(r *FleetRobot) { delete(r.Topics, "coverageSession") }, coverageAbsent, 5},
		{"missing-age", func(r *FleetRobot) { delete(r.TopicAgeS, "coverageSession") }, coverageUnknown, 5},
		{"negative-age", func(r *FleetRobot) { r.TopicAgeS["coverageSession"] = -1 }, coverageUnknown, 5},
		{"nan-age", func(r *FleetRobot) { r.TopicAgeS["coverageSession"] = math.NaN() }, coverageUnknown, 5},
		{"infinite-age", func(r *FleetRobot) { r.TopicAgeS["coverageSession"] = math.Inf(1) }, coverageUnknown, 5},
		{"missing-receipt", func(r *FleetRobot) { r.coverageObservation = nil }, coverageUnknown, 5},
		{"zero-receipt", func(r *FleetRobot) { r.coverageObservation = &coverageObservation{} }, coverageUnknown, 5},
		{"offline", func(r *FleetRobot) { r.Online = false }, coverageUnknown, 5},
		{"missing-identity", func(r *FleetRobot) { r.Identity.ID = "" }, coverageUnknown, 5},
		{"malformed", func(r *FleetRobot) { r.Topics["coverageSession"] = json.RawMessage(`{`) }, coverageUnknown, 5},
		{"missing-fields", func(r *FleetRobot) { r.Topics["coverageSession"] = json.RawMessage(`{}`) }, coverageUnknown, 5},
		{"null-fields", func(r *FleetRobot) {
			r.Topics["coverageSession"] = json.RawMessage(`{"session_active":null,"current_area":null,"completed_areas":null}`)
		}, coverageUnknown, 5},
		{"invalid-area", func(r *FleetRobot) {
			r.Topics["coverageSession"] = json.RawMessage(`{"session_active":true,"current_area":-2,"completed_areas":[1]}`)
		}, coverageUnknown, 5},
		{"contradictory", func(r *FleetRobot) {
			r.Topics["coverageSession"] = json.RawMessage(`{"session_active":true,"current_area":2,"completed_areas":[1]}`)
		}, coverageContradictory, 5},
		{"recent-hls-none", func(r *FleetRobot) { r.Topics["highLevelStatus"] = json.RawMessage(`{"state":1,"current_area":-1}`) }, coverageContradictory, -1},
		{"missing-hls", func(r *FleetRobot) { delete(r.Topics, "highLevelStatus") }, coverageFresh, 5},
		{"malformed-hls", func(r *FleetRobot) {
			r.Topics["highLevelStatus"] = json.RawMessage(`{"current_area":2,"state":"invalid"}`)
		}, coverageFresh, 5},
		{"stale-hls", func(r *FleetRobot) {
			r.Topics["highLevelStatus"] = json.RawMessage(`{"state":2,"current_area":2}`)
			r.TopicAgeS["highLevelStatus"] = 120
		}, coverageFresh, 5},
		{"unknown-hls-age", func(r *FleetRobot) {
			r.Topics["highLevelStatus"] = json.RawMessage(`{"state":2,"current_area":2}`)
			delete(r.TopicAgeS, "highLevelStatus")
		}, coverageFresh, 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r := sessionTestCache(now).robotSnapshot("peer", now)
			tc.edit(&r)
			m := memberFromRobotAt(r, LatLon{}, peerPoseMaxAge)
			assert.Equal(t, tc.state, m.SessionState)
			assert.Equal(t, tc.area, m.CurrentArea)
			if tc.state == coverageFresh {
				assert.Equal(t, []uint32{1}, m.Completed)
				require.Same(t, r.coverageObservation, m.SessionObservation)
			} else {
				assert.Empty(t, m.Completed)
				assert.Nil(t, m.SessionObservation)
			}
		})
	}
}

func TestFleetSession_CompletedTTLTracksTransitionsNotRepublishing(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	cache := sessionTestCache(now)
	observations := map[string]completionObservation{}
	memory := map[uint32]time.Time{}
	consume := func(at time.Time) {
		t.Helper()
		row := cache.robotSnapshot("peer", at)
		m := memberFromRobotAt(row, LatLon{}, peerPoseMaxAge)
		memory = updateCompletedMemory(memory, []fleetMember{m}, at, 12*time.Hour, observations)
	}
	consume(now)
	require.Equal(t, now, memory[1])
	first := cache.robotSnapshot("peer", now).coverageObservation
	consume(now.Add(time.Second))
	assert.Equal(t, now, memory[1], "rereading the same snapshot keeps its stamp")
	require.Same(t, first, cache.robotSnapshot("peer", now.Add(time.Second)).coverageObservation)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), now.Add(time.Second))
	consume(now.Add(time.Second))
	require.NotSame(t, first, cache.robotSnapshot("peer", now.Add(time.Second)).coverageObservation)
	assert.Equal(t, now, memory[1], "real delivery of identical bytes is fresh but is not another completion")

	// Real new publication, same completed membership: delivery freshness
	// advances, completion identity does not. No clock sleeps or timers.
	later := now.Add(6 * time.Hour)
	cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), later)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1,2]}`), later)
	consume(later)
	require.NotSame(t, first, cache.robotSnapshot("peer", later).coverageObservation)
	assert.Equal(t, now, memory[1], "another zone completing cannot refresh zone 1")
	assert.Equal(t, later, memory[2], "new completed member receives its own timestamp")

	boundary := now.Add(12 * time.Hour)
	cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), boundary)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[2,1]}`), boundary)
	consume(boundary)
	assert.Equal(t, now, memory[1], "set ordering is not a completion event")
	consume(boundary.Add(time.Nanosecond))
	assert.NotContains(t, memory, uint32(1), "true 12h TTL expires despite live republishing")
	assert.Equal(t, later, memory[2])

	// Expired but unchanged completion cannot be resurrected by a fresh
	// publication, even though the snapshot now has a different receipt ID.
	after := now.Add(24 * time.Hour)
	cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), after)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1,2]}`), after)
	consume(after)
	assert.Empty(t, memory)

	// A real observed reset followed by a new completion is admitted.
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[]}`), after)
	consume(after)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), after)
	consume(after)
	assert.Equal(t, after, memory[1])
}

func TestFleetSession_ReceiptOrderingAndUnknownTime(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	for _, tc := range []struct {
		name  string
		stamp time.Time
	}{
		{"future", now.Add(time.Second)}, {"stale-even-if-age-frozen", now.Add(-120 * time.Second)}, {"unknown", time.Time{}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			m := fleetMember{ID: "peer", Online: true, Completed: []uint32{1}, SessionState: coverageFresh, SessionObservation: &coverageObservation{receivedAt: tc.stamp}}
			tracker := map[string]completionObservation{}
			assert.Empty(t, updateCompletedMemory(nil, []fleetMember{m}, now, 12*time.Hour, tracker))
			assert.Empty(t, tracker, "invalid time is not consumed")
		})
	}
	cache := sessionTestCache(now)
	old := memberFromRobotAt(cache.robotSnapshot("peer", now), LatLon{}, peerPoseMaxAge)
	newTime := now.Add(time.Second)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[]}`), newTime)
	newer := memberFromRobotAt(cache.robotSnapshot("peer", newTime), LatLon{}, peerPoseMaxAge)
	tracker := map[string]completionObservation{}
	updateCompletedMemory(nil, []fleetMember{newer}, newTime, 12*time.Hour, tracker)
	assert.Empty(t, updateCompletedMemory(nil, []fleetMember{old}, newTime, 12*time.Hour, tracker), "older delivery cannot roll back accepted reset")
	// Repeating a fixed simulated clock must not erase delivery ordering.
	cache = sessionTestCache(now)
	old = memberFromRobotAt(cache.robotSnapshot("peer", now), LatLon{}, peerPoseMaxAge)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[]}`), now)
	newer = memberFromRobotAt(cache.robotSnapshot("peer", now), LatLon{}, peerPoseMaxAge)
	tracker = map[string]completionObservation{}
	updateCompletedMemory(nil, []fleetMember{newer}, now, 12*time.Hour, tracker)
	assert.Empty(t, updateCompletedMemory(nil, []fleetMember{old}, now, 12*time.Hour, tracker), "old delivery with identical timestamp cannot undo a reset")
}

func TestFleetSession_ReconnectionUsesExistingIdentityQualification(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	cache := sessionTestCache(now)
	tracker := map[string]completionObservation{}
	m := memberFromRobotAt(cache.robotSnapshot("peer", now), LatLon{}, peerPoseMaxAge)
	memory := updateCompletedMemory(nil, []fleetMember{m}, now, 12*time.Hour, tracker)
	cache.invalidateIdentity()
	cache.setIdentity(RobotIdentity{ID: "other", APIVersion: FleetAPIVersion})
	cache.setSocket(true)
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[9]}`), now)
	r := cache.robotSnapshot("peer", now)
	assert.False(t, r.Online)
	assert.Empty(t, r.Topics)
	assert.Empty(t, r.TopicAgeS)
	assert.Nil(t, r.coverageObservation, "FLT-04 disqualification also removes receipt identity")

	cache.setIdentity(RobotIdentity{ID: "peer", APIVersion: FleetAPIVersion})
	r = cache.robotSnapshot("peer", now)
	assert.Nil(t, r.coverageObservation)
	assert.False(t, r.Online, "requalification alone cannot restore the old HLS")
	after := now.Add(24 * time.Hour)
	cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), after)
	r = cache.robotSnapshot("peer", after)
	assert.True(t, r.Online)
	assert.Nil(t, r.coverageObservation, "new HLS is not a new coverage receipt")
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), after)
	m = memberFromRobotAt(cache.robotSnapshot("peer", after), LatLon{}, peerPoseMaxAge)
	memory = updateCompletedMemory(memory, []fleetMember{m}, after, 12*time.Hour, tracker)
	assert.Empty(t, memory, "same completion replayed after reconnect cannot restart its TTL")
	assert.NotSame(t, tracker["peer"].receipt, tracker["other"].receipt)
	assert.NotContains(t, tracker, "other")
	// The internal receipt identity never alters the public Fleet API.
	b, err := json.Marshal(cache.robotSnapshot("peer", after))
	require.NoError(t, err)
	assert.NotContains(t, string(b), "receivedAt")
	assert.NotContains(t, string(b), "coverageObservation")
}

func TestFleetSession_CoordinatorUsesQualifiedReceipts(t *testing.T) {
	start := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	now := start
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters: {}\n")
	require.NoError(t, db.Set(robotIDKey, []byte("self")))
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = func(_ string, _ any, response any) {
		if r, ok := response.(*mowgli.SetFleetAssignmentRes); ok {
			r.Success = true
		}
	}
	cache := sessionTestCache(start.Add(-120 * time.Second))
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":2,"completed_areas":[1]}`), start.Add(-120*time.Second))
	cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), start)
	f := &FleetProvider{db: db, now: func() time.Time { return now }, self: newPeerCache(),
		peers:   map[string]FleetPeer{"peer": {ID: "peer", APIVersion: FleetAPIVersion}},
		clients: map[string]*peerClient{"peer": {cache: cache}},
	}
	c := newFleetCoordinator(db, ros, f.Robots, f.Identity, func() time.Time { return now })
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	assert.Equal(t, []uint32{5}, c.Status().ExcludedAreas, "fresh HLS area 5, not stale session area 2")
	assert.Empty(t, c.Status().CompletedAreas)
	rows, err := f.Robots()
	require.NoError(t, err)
	require.Len(t, rows, 2)
	require.Same(t, cache.coverage, rows[1].coverageObservation, "real FleetProvider snapshot carries the qualified receipt")

	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), now)
	c.tick(now)
	assert.Equal(t, []uint32{1, 5}, c.Status().ExcludedAreas)
	require.Equal(t, start, c.memory[1])
	now = start.Add(time.Second)
	c.tick(now)
	assert.Equal(t, start, c.memory[1])

	now = start.Add(12*time.Hour + time.Second)
	cache.store("highLevelStatus", json.RawMessage(`{"state":2,"current_area":5}`), now)
	c.tick(now)
	assert.Empty(t, c.Status().CompletedAreas)
	assert.Equal(t, []uint32{5}, c.Status().ExcludedAreas)
	raw, err := db.Get(coordinationMemoryKey)
	require.NoError(t, err)
	var persisted map[string]int64
	require.NoError(t, json.Unmarshal(raw, &persisted))
	assert.Empty(t, persisted, "expired completion is also removed from persisted memory")
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), now)
	c.tick(now)
	assert.Empty(t, c.Status().CompletedAreas, "new delivery of the same completion cannot resurrect it")

	// The local cache uses exactly the same receipt tracking, without
	// imposing the remote identity handshake on the local robot.
	f.self.store("highLevelStatus", json.RawMessage(`{"state":1,"current_area":-1}`), now)
	f.self.store("coverageSession", json.RawMessage(`{"session_active":false,"current_area":-1,"completed_areas":[3]}`), now)
	c.tick(now)
	assert.Equal(t, []uint32{3}, c.Status().CompletedAreas)
	assert.Equal(t, []uint32{3, 5}, c.Status().ExcludedAreas)
	require.NotEmpty(t, ros.ServiceCalls)
	for _, call := range ros.ServiceCalls {
		assert.Equal(t, setFleetAssignmentService, call.Service, "no STOP/START was introduced by session bookkeeping")
	}
}

func TestFleetSession_RestartPreservesPersistedCompletionAge(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	for _, age := range []time.Duration{6 * time.Hour, 24 * time.Hour} {
		t.Run(age.String(), func(t *testing.T) {
			cache := sessionTestCache(now)
			m := memberFromRobotAt(cache.robotSnapshot("peer", now), LatLon{}, peerPoseMaxAge)
			saved := now.Add(-age)
			tracker := map[string]completionObservation{}
			memory := updateCompletedMemory(map[uint32]time.Time{1: saved}, []fleetMember{m}, now, 12*time.Hour, tracker)
			if age <= 12*time.Hour {
				assert.Equal(t, saved, memory[1])
			} else {
				assert.Empty(t, memory)
			}
			assert.Equal(t, saved, tracker["peer"].areas[1], "first replay cannot rejuvenate existing persisted provenance")
		})
	}
}

func TestFleetSession_ResetBeforeEmptySessionCannotRelearnCompletion(t *testing.T) {
	now := time.Date(2026, 10, 21, 12, 0, 0, 0, time.UTC)
	db := types.NewMockDBProvider()
	writeRobotYaml(t, db, "mowgli:\n  ros__parameters: {}\n")
	require.NoError(t, db.Set(robotIDKey, []byte("self")))
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = func(_ string, _ any, response any) {
		data, _ := json.Marshal(map[string]bool{"success": true})
		_ = json.Unmarshal(data, response)
	}
	cache := sessionTestCache(now)
	var clearCalls atomic.Int32
	// Real Fleet command proxy: acknowledge clear-resume without publishing
	// the empty coverageSession yet. No network, robot or scheduling sleeps.
	remote := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		assert.Equal(t, "/api/mowglinext/call/coverage_clear_resume", r.URL.Path)
		clearCalls.Add(1)
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(`{"success":true}`))
	}))
	defer remote.Close()
	f := &FleetProvider{db: db, now: func() time.Time { return now }, self: newPeerCache(), http: remote.Client(),
		peers:   map[string]FleetPeer{"peer": {ID: "peer", Address: strings.TrimPrefix(remote.URL, "http://"), APIVersion: FleetAPIVersion}},
		clients: map[string]*peerClient{"peer": {cache: cache}},
	}
	c := newFleetCoordinator(db, ros, f.Robots, f.Identity, func() time.Time { return now })
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	require.Equal(t, []uint32{1}, c.Status().CompletedAreas)
	require.Equal(t, []uint32{1, 5}, c.Status().ExcludedAreas)
	status, _, err := f.Call(context.Background(), "peer", "coverage_clear_resume", json.RawMessage(`{}`))
	require.NoError(t, err)
	require.Equal(t, http.StatusOK, status)
	require.Equal(t, int32(1), clearCalls.Load())
	oldReceipt := cache.coverage
	assertCleared := func() {
		t.Helper()
		assert.Empty(t, c.Status().CompletedAreas)
		assert.Equal(t, []uint32{5}, c.Status().ExcludedAreas)
		calls := serviceCalls(ros, setFleetAssignmentService)
		require.NotEmpty(t, calls)
		assert.Equal(t, []uint32{5}, calls[len(calls)-1].Req.(*mowgli.SetFleetAssignmentReq).ExcludedAreas)
		raw, err := db.Get(coordinationMemoryKey)
		require.NoError(t, err)
		var persisted map[string]int64
		require.NoError(t, json.Unmarshal(raw, &persisted))
		assert.Empty(t, persisted)
	}
	require.NoError(t, c.Reset())
	assertCleared()
	for range 2 {
		now = now.Add(time.Second)
		c.tick(now)
		assertCleared()
	}
	require.Same(t, oldReceipt, cache.coverage, "still the old cached snapshot")
	// Even an actual delivery of the unchanged pre-clear set cannot restore 1.
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), now)
	c.tick(now)
	assertCleared()
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[]}`), now)
	c.tick(now)
	assertCleared()
	cache.store("coverageSession", json.RawMessage(`{"session_active":true,"current_area":5,"completed_areas":[1]}`), now)
	c.tick(now)
	assert.Equal(t, []uint32{1}, c.Status().CompletedAreas)
	assert.Equal(t, []uint32{1, 5}, c.Status().ExcludedAreas)
	assert.Equal(t, now, c.memory[1], "new completion after observed reset is admissible")
	calls := serviceCalls(ros, setFleetAssignmentService)
	assert.Equal(t, []uint32{1, 5}, calls[len(calls)-1].Req.(*mowgli.SetFleetAssignmentReq).ExcludedAreas)
	raw, err := db.Get(coordinationMemoryKey)
	require.NoError(t, err)
	var persisted map[string]int64
	require.NoError(t, json.Unmarshal(raw, &persisted))
	assert.Equal(t, map[string]int64{"1": now.Unix()}, persisted)
}
