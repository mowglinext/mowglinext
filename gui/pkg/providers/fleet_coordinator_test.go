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

func member(id string, autonomous bool, area int, pos *XY) fleetMember {
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
	here := &XY{}
	near := &XY{X: 1.1}
	far := &XY{X: 11}
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
	here := &XY{}
	near := &XY{X: 1.1}

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
	row.TopicAgeS = map[string]float64{"highLevelStatus": 0, "coverageSession": 0, "gps": 0.5}
	m := memberFromRobotAt(row, LatLon{Lat: 48.5, Lon: 2.5}, peerPoseMaxAge)
	assert.True(t, m.Autonomous)
	assert.Equal(t, 4, m.CurrentArea)
	assert.Equal(t, []uint32{1, 2}, m.Completed)
	assert.True(t, m.SessionActive)
	require.NotNil(t, m.Pos)
	assert.InDelta(t, 0.0, m.Pos.X, 1e-6)
	assert.InDelta(t, 0.0, m.Pos.Y, 1e-6)

	// A fresh fused pose wins over the fix, in the peer's own map frame.
	row.Topics["pose"] = poseTopic(3, 4, 0)
	row.TopicAgeS["pose"] = 0.1
	withPose := memberFromRobotAt(row, LatLon{Lat: 48.5, Lon: 2.5}, peerPoseMaxAge)
	require.NotNil(t, withPose.Pos)
	assert.InDelta(t, 3.0, withPose.Pos.X, 1e-9)
	assert.InDelta(t, 4.0, withPose.Pos.Y, 1e-9)

	// Stale samples and a robot without a map frame yield no position.
	row.TopicAgeS["pose"], row.TopicAgeS["gps"] = 9, 9
	assert.Nil(t, memberFromRobotAt(row, LatLon{Lat: 48.5, Lon: 2.5}, peerPoseMaxAge).Pos)
	row.TopicAgeS["gps"] = 0
	assert.Nil(t, memberFromRobotAt(row, LatLon{}, peerPoseMaxAge).Pos, "no datum → no frame")

	empty := memberFromRobotAt(FleetRobot{Identity: RobotIdentity{ID: "y"}}, LatLon{Lat: 48.5, Lon: 2.5}, peerPoseMaxAge)
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

func TestCoordinatorTick_DisabledOnlyReconcilesAssignment(t *testing.T) {
	rows := []FleetRobot{{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true}}
	c, _, ros := newTestCoordinator(t, &rows)

	c.tick(time.Now())

	require.Len(t, ros.ServiceCalls, 1)
	require.Equal(t, setFleetAssignmentService, ros.ServiceCalls[0].Service)
	req := ros.ServiceCalls[0].Req.(*mowgli.SetFleetAssignmentReq)
	assert.Empty(t, req.ExcludedAreas)
	assert.Equal(t, int32(-1), req.PreferredStartIndex)
	assert.Empty(t, ros.Publishes)
	assert.False(t, c.Status().Enabled)
}

func TestCoordinatorTick_PushesExclusionsAndRemembersCompletion(t *testing.T) {
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

	assert.Empty(t, ros.Publishes, "peer poses are the FleetProvider feed's job, not the coordinator's")

	raw, err := db.Get(coordinationMemoryKey)
	require.NoError(t, err)
	assert.Contains(t, string(raw), `"0":`, "completed area persisted")

	// Same snapshot again: no second push (unchanged).
	c.tick(time.Now())
	assert.Len(t, serviceCalls(ros, setFleetAssignmentService), 1)

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
		}, TopicAgeS: map[string]float64{"gps": 0}},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING","current_area":1}`),
			"gps":             gpsTopic(48.00001, 2.0), // ~1 m
		}, TopicAgeS: map[string]float64{"gps": 0}},
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

func TestDecideYield_MaxHoldBreaksADeadlockAndCooldownPreventsFlapping(t *testing.T) {
	s := DefaultCoordinatorSettings().Normalize()
	now := time.Date(2026, 10, 10, 12, 0, 0, 0, time.UTC)
	here := &XY{}
	near := &XY{X: 1.1}
	peerNear := []fleetMember{member("a", true, 1, near)}

	// Stop as usual.
	self := member("b", true, 0, here)
	st, action := decideYield(self, peerNear, yieldState{}, s, now)
	require.Equal(t, YieldStop, action)
	assert.Equal(t, now, st.HoldSince)

	// The priority peer parks in front of us (its own collision checks) and
	// never leaves: after yield_max_hold_s we resume anyway — both robots now
	// carry each other in their costmaps, so our controllers skirt it.
	self.Autonomous = false
	st, action = decideYield(self, peerNear, st, s, now.Add(44*time.Second))
	assert.Equal(t, YieldNone, action)
	st, action = decideYield(self, peerNear, st, s, now.Add(45*time.Second))
	assert.Equal(t, YieldStart, action)
	assert.False(t, st.Yielded)
	assert.Equal(t, now.Add(45*time.Second), st.ResumedAt)

	// Autonomous again with the peer still at 1 m: the cooldown blocks an
	// immediate second stop, then expires.
	self.Autonomous = true
	st, action = decideYield(self, peerNear, st, s, now.Add(50*time.Second))
	assert.Equal(t, YieldNone, action, "inside yield_cooldown_s no new yield is issued")
	st, action = decideYield(self, peerNear, st, s, now.Add(66*time.Second))
	assert.Equal(t, YieldStop, action)
	assert.True(t, st.Yielded)
}

func TestDecideYield_NormalResumeAlsoArmsTheCooldown(t *testing.T) {
	s := DefaultCoordinatorSettings().Normalize()
	now := time.Now()
	self := member("b", false, 0, &XY{})
	far := []fleetMember{member("a", true, 1, &XY{X: 20})}

	st, action := decideYield(self, far, yieldState{Yielded: true, HoldSince: now.Add(-10 * time.Second), ClearSince: now.Add(-5 * time.Second)}, s, now)
	assert.Equal(t, YieldStart, action)
	assert.Equal(t, now, st.ResumedAt)
}

func TestCoordinatorSettings_NormalizeFillsTheYieldBounds(t *testing.T) {
	s := CoordinatorSettings{Enabled: true}.Normalize()
	assert.Equal(t, 45.0, s.YieldMaxHoldS)
	assert.Equal(t, 20.0, s.YieldCooldownS)
	assert.Equal(t, 5.0, CoordinatorSettings{YieldCooldownS: 5}.Normalize().YieldCooldownS)
}
