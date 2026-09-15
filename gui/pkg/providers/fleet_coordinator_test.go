package providers

import (
	"encoding/json"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// ---------------------------------------------------------------------------
// Pure logic
// ---------------------------------------------------------------------------

func member(id string, autonomous bool, area int, pos *LatLon) fleetMember {
	return fleetMember{ID: id, Online: true, Autonomous: autonomous, CurrentArea: area, Pos: pos}
}

func TestComputeAssignment_ExcludesPeerAreasAndCompletedMemory(t *testing.T) {
	self := member("b", true, 3, nil)
	self.Self = true
	peers := []fleetMember{member("a", true, 1, nil), member("c", false, 2, nil)}
	memory := map[uint32]time.Time{5: time.Now()}

	a := computeAssignment(self, peers, memory)

	assert.Equal(t, []uint32{1, 5}, a.Excluded, "an idle peer's last area is not a claim")
	assert.Equal(t, int32(1), a.PreferredStart, "rank of 'b' among a,b,c")
}

func TestComputeAssignment_TieBreakOnTheSameArea(t *testing.T) {
	memory := map[uint32]time.Time{}
	// Both mow area 2. The smaller id keeps it, the larger yields.
	winner := member("a", true, 2, nil)
	winner.Self = true
	assert.Empty(t, computeAssignment(winner, []fleetMember{member("b", true, 2, nil)}, memory).Excluded)

	loser := member("b", true, 2, nil)
	loser.Self = true
	assert.Equal(t, []uint32{2}, computeAssignment(loser, []fleetMember{member("a", true, 2, nil)}, memory).Excluded)
}

func TestComputeAssignment_AloneHasNothingExcludedAndStartsAtZero(t *testing.T) {
	self := member("z", false, -1, nil)
	self.Self = true
	a := computeAssignment(self, nil, map[uint32]time.Time{})
	assert.Empty(t, a.Excluded)
	assert.Equal(t, int32(0), a.PreferredStart)
}

func TestUpdateCompletedMemory_AddsAndExpires(t *testing.T) {
	now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
	old := map[uint32]time.Time{7: now.Add(-13 * time.Hour), 8: now.Add(-1 * time.Hour)}
	members := []fleetMember{
		{ID: "a", Online: true, Completed: []uint32{1, 2}},
		{ID: "b", Online: false, Completed: []uint32{9}},
	}

	got := updateCompletedMemory(old, members, now, 12*time.Hour)

	assert.Equal(t, []uint32{1, 2, 8}, sortedAreas(got), "expired 7 dropped, offline b's 9 ignored")
	assert.Equal(t, now, got[1])
	assert.Equal(t, old[8], got[8], "existing stamps are kept")
	assert.Len(t, old, 2, "input map is not mutated")
}

func TestDistanceAndProjection(t *testing.T) {
	a := LatLon{Lat: 48.0, Lon: 2.0}
	b := LatLon{Lat: 48.0, Lon: 2.0001}
	assert.InDelta(t, 7.45, distanceM(a, b), 0.05)

	x, y := enuFromDatum(a, LatLon{Lat: 48.0001, Lon: 2.0})
	assert.InDelta(t, 0.0, x, 1e-9)
	assert.InDelta(t, 11.13, y, 0.01)

	assert.True(t, sameDatum(a, LatLon{Lat: 48.0 + 5e-9, Lon: 2.0}))
	assert.False(t, sameDatum(a, LatLon{Lat: 48.0 + 5e-8, Lon: 2.0}))
}

func TestDecideYield_StopsForAPriorityPeerAndResumesAfterHold(t *testing.T) {
	s := DefaultCoordinatorSettings().Normalize()
	now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
	here := &LatLon{Lat: 48.0, Lon: 2.0}
	near := &LatLon{Lat: 48.00001, Lon: 2.0} // ~1.1 m
	far := &LatLon{Lat: 48.0001, Lon: 2.0}   // ~11 m
	self := member("b", true, 0, here)

	// A lower-id autonomous peer 1 m away → stop.
	st, action := decideYield(self, []fleetMember{member("a", true, 1, near)}, yieldState{}, s, now)
	assert.Equal(t, YieldStop, action)
	assert.True(t, st.Yielded)

	// We are now IDLE (stop-hold). Peer still near → keep holding.
	self.Autonomous = false
	st, action = decideYield(self, []fleetMember{member("a", true, 1, near)}, st, s, now.Add(2*time.Second))
	assert.Equal(t, YieldNone, action)
	assert.True(t, st.Yielded)

	// Peer moves away: clear timer starts, no resume before the hold elapses.
	st, action = decideYield(self, []fleetMember{member("a", true, 1, far)}, st, s, now.Add(4*time.Second))
	assert.Equal(t, YieldNone, action)
	st, action = decideYield(self, []fleetMember{member("a", true, 1, far)}, st, s, now.Add(5*time.Second))
	assert.Equal(t, YieldNone, action)
	st, action = decideYield(self, []fleetMember{member("a", true, 1, far)}, st, s, now.Add(8*time.Second))
	assert.Equal(t, YieldStart, action)
	assert.False(t, st.Yielded)
}

func TestDecideYield_PriorityRobotNeverYieldsAndIdleRobotsAreLeftAlone(t *testing.T) {
	s := DefaultCoordinatorSettings().Normalize()
	now := time.Now()
	here := &LatLon{Lat: 48.0, Lon: 2.0}
	near := &LatLon{Lat: 48.00001, Lon: 2.0}

	// "a" has priority over "b": a higher-id peer close by is not a threat.
	_, action := decideYield(member("a", true, 0, here), []fleetMember{member("b", true, 1, near)}, yieldState{}, s, now)
	assert.Equal(t, YieldNone, action)

	// An idle robot (operator pause, docked) is never stopped nor resumed.
	_, action = decideYield(member("b", false, 0, here), []fleetMember{member("a", true, 1, near)}, yieldState{}, s, now)
	assert.Equal(t, YieldNone, action)

	// A held robot that the operator sent home drops the hold without a resume.
	self := member("b", false, 0, here)
	self.StateName = "RETURNING_HOME"
	st, action := decideYield(self, nil, yieldState{Yielded: true}, s, now)
	assert.Equal(t, YieldNone, action)
	assert.False(t, st.Yielded)

	// No fix → no decision.
	_, action = decideYield(member("b", true, 0, nil), []fleetMember{member("a", true, 1, near)}, yieldState{}, s, now)
	assert.Equal(t, YieldNone, action)
}

func TestMemberFromRobot_ReadsTopics(t *testing.T) {
	row := FleetRobot{
		Identity: RobotIdentity{ID: "x"},
		Online:   true,
		Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":4}`),
			"coverageSession": json.RawMessage(`{"session_active":true,"current_area":4,"completed_areas":[1,2]}`),
			"gps":             json.RawMessage(`{"pose":{"pose":{"position":{"x":48.5,"y":2.5,"z":0}}}}`),
		},
	}
	m := memberFromRobot(row)
	assert.True(t, m.Autonomous)
	assert.Equal(t, 4, m.CurrentArea)
	assert.Equal(t, []uint32{1, 2}, m.Completed)
	assert.True(t, m.SessionActive)
	require.NotNil(t, m.Pos)
	assert.Equal(t, LatLon{Lat: 48.5, Lon: 2.5}, *m.Pos)

	empty := memberFromRobot(FleetRobot{Identity: RobotIdentity{ID: "y"}})
	assert.False(t, empty.Autonomous)
	assert.Equal(t, -1, empty.CurrentArea)
	assert.Nil(t, empty.Pos)
}

func TestCoordinatorSettings_Normalize(t *testing.T) {
	s := CoordinatorSettings{Enabled: true, YieldDistanceM: 4, ResumeDistanceM: 2, CompletedTTLHours: 0}.Normalize()
	assert.Equal(t, 4.0, s.YieldDistanceM)
	assert.Equal(t, 6.0, s.ResumeDistanceM, "resume distance is forced above the yield distance")
	assert.Equal(t, 12.0, s.CompletedTTLHours)
}

// ---------------------------------------------------------------------------
// tick() against a mock ROS graph
// ---------------------------------------------------------------------------

func gpsTopic(lat, lon float64) json.RawMessage {
	b, _ := json.Marshal(map[string]any{"pose": map[string]any{"pose": map[string]any{"position": map[string]float64{"x": lat, "y": lon, "z": 0}}}})
	return b
}

func newTestCoordinator(t *testing.T, rows *[]FleetRobot) (*FleetCoordinator, *types.MockDBProvider, *types.MockRosProvider) {
	t.Helper()
	db := types.NewMockDBProvider()
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = func(service string, _ any, res any) {
		if r, ok := res.(*mowgli.SetFleetAssignmentRes); ok {
			r.Success = true
		}
	}
	identity := func() (RobotIdentity, error) {
		return RobotIdentity{ID: "b", Name: "bravo", DatumLat: 48.0, DatumLon: 2.0}, nil
	}
	c := newFleetCoordinator(db, ros, func() ([]FleetRobot, error) { return *rows, nil }, identity, time.Now)
	return c, db, ros
}

func serviceCalls(ros *types.MockRosProvider, service string) []types.ServiceCall {
	var out []types.ServiceCall
	for _, call := range ros.ServiceCalls {
		if call.Service == service {
			out = append(out, call)
		}
	}
	return out
}

func TestCoordinatorTick_DisabledIsInert(t *testing.T) {
	rows := []FleetRobot{{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true}}
	c, _, ros := newTestCoordinator(t, &rows)

	c.tick(time.Now())

	assert.Empty(t, ros.ServiceCalls)
	assert.Empty(t, ros.Publishes)
	assert.False(t, c.Status().Enabled)
}

func TestCoordinatorTick_PushesExclusionsPublishesPeersAndRemembersCompletion(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":1,"current_area":-1}`),
			"gps":             gpsTopic(48.0, 2.0),
		}},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":1}`),
			"coverageSession": json.RawMessage(`{"session_active":true,"current_area":1,"completed_areas":[0]}`),
			"gps":             gpsTopic(48.0001, 2.0),
		}},
	}
	c, db, ros := newTestCoordinator(t, &rows)
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))

	calls := serviceCalls(ros, setFleetAssignmentService)
	require.Len(t, calls, 1)
	req := calls[0].Req.(*mowgli.SetFleetAssignmentReq)
	assert.Equal(t, []uint32{0, 1}, req.ExcludedAreas, "peer's live area + its completed area")
	assert.Equal(t, int32(1), req.PreferredStartIndex, "rank of b among a,b")

	require.Len(t, ros.Publishes, 1)
	assert.Equal(t, fleetPeersTopic, ros.Publishes[0].Topic)
	arr := ros.Publishes[0].Msg.(*poseArrayMsg)
	require.Len(t, arr.Poses, 1)
	assert.InDelta(t, 11.13, arr.Poses[0].Position.Y, 0.01, "peer projected into our map frame, north of us")
	assert.Equal(t, "map", arr.Header.FrameId)

	raw, err := db.Get(coordinationMemoryKey)
	require.NoError(t, err)
	assert.Contains(t, string(raw), `"0":`, "completed area persisted")

	// Same snapshot again: no second push (unchanged), but peers are republished.
	c.tick(time.Now())
	assert.Len(t, serviceCalls(ros, setFleetAssignmentService), 1)
	assert.Len(t, ros.Publishes, 2)

	st := c.Status()
	assert.True(t, st.Enabled)
	assert.Equal(t, []uint32{0, 1}, st.ExcludedAreas)
	assert.Equal(t, []uint32{0}, st.CompletedAreas)
}

func TestCoordinatorTick_YieldSendsStopThenResume(t *testing.T) {
	selfHL := json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":0}`)
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": selfHL, "gps": gpsTopic(48.0, 2.0),
		}},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":1}`),
			"gps":             gpsTopic(48.00001, 2.0), // ~1 m
		}},
	}
	c, _, ros := newTestCoordinator(t, &rows)
	now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
	c.now = func() time.Time { return now }
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))

	hl := serviceCalls(ros, highLevelControlService)
	require.Len(t, hl, 1)
	assert.Equal(t, uint8(hlCommandStop), hl[0].Req.(*mowgli.HighLevelControlReq).Command)
	assert.True(t, c.Status().Yielded)

	// The BT reports IDLE now; the peer drives away; after the hold we resume.
	rows[0].Topics["highLevelStatus"] = json.RawMessage(`{"state":1,"state_name":"IDLE","current_area":0}`)
	rows[1].Topics["gps"] = gpsTopic(48.001, 2.0) // ~110 m
	c.tick(now.Add(2 * time.Second))
	c.tick(now.Add(4 * time.Second))
	assert.Len(t, serviceCalls(ros, highLevelControlService), 1, "resume waits for the hold")
	c.tick(now.Add(6 * time.Second))
	hl = serviceCalls(ros, highLevelControlService)
	require.Len(t, hl, 2)
	assert.Equal(t, uint8(hlCommandStart), hl[1].Req.(*mowgli.HighLevelControlReq).Command)
	assert.False(t, c.Status().Yielded)
}

func TestCoordinator_DisablingHandsTheLawnBackOnce(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"current_area":3}`),
		}},
	}
	c, _, ros := newTestCoordinator(t, &rows)
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	require.Len(t, serviceCalls(ros, setFleetAssignmentService), 1)

	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: false}))
	calls := serviceCalls(ros, setFleetAssignmentService)
	require.Len(t, calls, 2)
	req := calls[1].Req.(*mowgli.SetFleetAssignmentReq)
	assert.Empty(t, req.ExcludedAreas)
	assert.Equal(t, int32(-1), req.PreferredStartIndex)

	c.tick(time.Now())
	assert.Len(t, serviceCalls(ros, setFleetAssignmentService), 2, "disabled stays silent afterwards")
}

func TestCoordinator_ResetForgetsCompletedAreas(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true, Topics: map[string]json.RawMessage{
			"coverageSession": json.RawMessage(`{"session_active":false,"current_area":-1,"completed_areas":[2]}`),
		}},
	}
	c, db, _ := newTestCoordinator(t, &rows)
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	require.Equal(t, []uint32{2}, c.Status().CompletedAreas)

	// The member has cleared its own session; reset must not re-learn area 2.
	rows[0].Topics["coverageSession"] = json.RawMessage(`{"session_active":false,"current_area":-1,"completed_areas":[]}`)
	require.NoError(t, c.Reset())

	assert.Empty(t, c.Status().CompletedAreas)
	raw, err := db.Get(coordinationMemoryKey)
	require.NoError(t, err)
	assert.JSONEq(t, `{}`, string(raw))
}

func TestStripPendingObstacles(t *testing.T) {
	area := mowgli.MapArea{
		Name:         "lawn",
		Obstacles:    make([]geometry.Polygon, 3),
		ObstacleInfo: []mowgli.MapObstacleInfo{{Name: "tree"}, {Name: "dig", Pending: true}, {Name: "rock"}},
	}
	out := stripPendingObstacles(area)
	assert.Len(t, out.Obstacles, 2)
	assert.Equal(t, []string{"tree", "rock"}, []string{out.ObstacleInfo[0].Name, out.ObstacleInfo[1].Name})
	assert.Len(t, area.Obstacles, 3, "input untouched")

	noInfo := mowgli.MapArea{Obstacles: make([]geometry.Polygon, 2)}
	assert.Len(t, stripPendingObstacles(noInfo).Obstacles, 2, "without obstacle_info nothing is dropped")
}
