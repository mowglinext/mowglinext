package providers

import (
	"encoding/json"
	"errors"
	"fmt"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

// fleetYieldServer models the production ownership contract, independently of
// GUI cached status. Repeated ordinary START can destroy a single-area clip;
// fleet RESUME must preserve it, and delayed duplicate operations cannot mutate.
type fleetYieldServer struct {
	epoch                      int
	owner                      *mowgli.FleetYieldReq
	completed                  *mowgli.FleetYieldReq
	command, phase             string
	target                     int
	pauses, resumes, rawStarts int
}

func sameYield(a *mowgli.FleetYieldReq, b mowgli.FleetYieldReq) bool {
	return a != nil && a.Epoch == b.Epoch && a.RequestId == b.RequestId
}

func (s *fleetYieldServer) fence() string { return fmt.Sprint(s.epoch) }
func (s *fleetYieldServer) operator(command, phase string) {
	s.epoch++
	s.owner = nil
	s.command, s.phase = command, phase
	if command == "START" {
		s.target = -1
		s.rawStarts++
	}
	if command == "START_IN_AREA" {
		s.command, s.target = "START", 7
	}
}

func (s *fleetYieldServer) call(q mowgli.FleetYieldReq) mowgli.FleetYieldRes {
	result := fleetRevoked
	switch {
	case q.Operation == fleetQuery:
		result = fleetAvailable
	case q.Epoch == "" || q.RequestId == "" || q.Operation > fleetCancel:
		result = fleetInvalid
	case q.Operation == fleetResume && sameYield(s.completed, q):
		result = fleetAlreadyResumed
	case sameYield(s.owner, q):
		switch q.Operation {
		case fleetPause:
			result = fleetHeld
		case fleetCancel:
			s.owner = nil
			s.epoch++
			result = fleetCancelled
		case fleetResume:
			if s.command == "STOP" && s.phase == "IDLE" {
				completed := q
				s.completed = &completed
				s.owner = nil
				s.epoch++
				s.command = "START"
				s.resumes++
				result = fleetResumed
			} else if s.command == "STOP" && (s.phase == "MOWING" || s.phase == "PLANNING" || s.phase == "TRANSIT") {
				result = fleetNotReady
			} else {
				s.owner = nil
				s.epoch++
			}
		}
	case q.Epoch != s.fence():
		result = fleetRevoked
	case s.owner != nil:
		result = fleetBusy
	case q.Operation == fleetCancel:
		s.epoch++
		result = fleetCancelled
	case q.Operation == fleetPause:
		if s.command == "START" && (s.phase == "MOWING" || s.phase == "PLANNING" || s.phase == "TRANSIT") {
			owner := q
			s.owner = &owner
			s.command = "STOP"
			s.pauses++
			result = fleetHeld
		} else {
			result = fleetIneligible
		}
	}
	return mowgli.FleetYieldRes{Result: result, Epoch: s.fence(), Held: s.owner != nil}
}

type ownedYieldTest struct {
	c                     *FleetCoordinator
	ros                   *types.MockRosProvider
	rows                  []FleetRobot
	server                *fleetYieldServer
	now                   time.Time
	lostOperation         uint8
	loseBefore, loseAfter bool
	absent                bool
}

func newOwnedYieldTest(t *testing.T) *ownedYieldTest {
	t.Helper()
	h := &ownedYieldTest{now: time.Date(2026, 10, 18, 12, 0, 0, 0, time.UTC), server: &fleetYieldServer{epoch: 1, command: "START", phase: "MOWING", target: 4}}
	h.rows = []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true, Topics: map[string]json.RawMessage{"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING"}`), "gps": gpsTopic(48, 2)}, TopicAgeS: map[string]float64{"gps": 0}},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{"highLevelStatus": json.RawMessage(`{"state":2,"state_name":"MOWING"}`), "gps": gpsTopic(48.00001, 2)}, TopicAgeS: map[string]float64{"gps": 0}},
	}
	h.c, _, h.ros = newTestCoordinator(t, &h.rows)
	h.c.now = func() time.Time { return h.now }
	h.ros.ServiceResponder = func(service string, req, res any) {
		h.ros.ServiceErr = nil
		if r, ok := res.(*mowgli.SetFleetAssignmentRes); ok {
			r.Success = true
			return
		}
		require.Equal(t, fleetYieldService, service, "coordinator must never use raw HighLevelControl")
		q := *req.(*mowgli.FleetYieldReq)
		if h.absent {
			h.ros.ServiceErr = errors.New("service not advertised")
			return
		}
		if h.loseBefore && q.Operation == h.lostOperation {
			h.ros.ServiceErr = errors.New("request not delivered")
			return
		}
		*res.(*mowgli.FleetYieldRes) = h.server.call(q)
		if h.loseAfter && q.Operation == h.lostOperation {
			h.ros.ServiceErr = errors.New("accepted reply lost")
		}
	}
	return h
}

func (h *ownedYieldTest) enable(t *testing.T) {
	t.Helper()
	require.NoError(t, h.c.SetSettings(CoordinatorSettings{Enabled: true}))
}
func (h *ownedYieldTest) tick(seconds int) {
	h.now = h.now.Add(time.Duration(seconds) * time.Second)
	h.c.tick(h.now)
}
func (h *ownedYieldTest) status(state int, phase string) {
	h.rows[0].Topics["highLevelStatus"] = json.RawMessage(fmt.Sprintf(`{"state":%d,"state_name":%q}`, state, phase))
}
func (h *ownedYieldTest) far()  { h.rows[1].Topics["gps"] = gpsTopic(48.001, 2) }
func (h *ownedYieldTest) near() { h.rows[1].Topics["gps"] = gpsTopic(48.00001, 2) }
func (h *ownedYieldTest) calls(op uint8) []mowgli.FleetYieldReq {
	var out []mowgli.FleetYieldReq
	for _, c := range serviceCalls(h.ros, fleetYieldService) {
		q := *c.Req.(*mowgli.FleetYieldReq)
		if q.Operation == op {
			out = append(out, q)
		}
	}
	return out
}
func (h *ownedYieldTest) clear(t *testing.T) {
	t.Helper()
	h.server.phase = "IDLE"
	h.status(1, "IDLE")
	h.far()
	h.tick(1)
	h.tick(3)
}

func TestFleetYieldLostPauseReplyReplaysExactIdentity(t *testing.T) {
	for _, delivered := range []bool{false, true} {
		t.Run(fmt.Sprint(delivered), func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.lostOperation = fleetPause
			h.loseAfter = delivered
			h.loseBefore = !delivered
			h.enable(t)
			require.True(t, h.c.Status().Yielded)
			first := h.calls(fleetPause)[0]
			h.loseAfter, h.loseBefore = false, false
			// GUI still reports MOWING after the server accepted STOP.
			h.tick(1)
			require.Equal(t, []mowgli.FleetYieldReq{first, first}, h.calls(fleetPause))
			require.Equal(t, 1, h.server.pauses)
			require.True(t, h.c.Status().Yielded)
			h.clear(t)
			require.False(t, h.c.Status().Yielded)
			require.Equal(t, 1, h.server.resumes)
		})
	}
}

func TestFleetYieldLostResumeReplyPreservesSingleAreaAndCooldown(t *testing.T) {
	for _, maxHold := range []bool{false, true} {
		t.Run(fmt.Sprint(maxHold), func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.enable(t)
			h.server.phase = "IDLE"
			h.status(1, "IDLE")
			h.lostOperation = fleetResume
			h.loseAfter = true
			if maxHold {
				h.tick(45)
			} else {
				h.far()
				h.tick(1)
				h.tick(3)
			}
			require.True(t, h.c.Status().Yielded)
			require.Equal(t, 1, h.server.resumes)
			original := h.calls(fleetResume)[0]
			h.loseAfter = false
			// Status remains IDLE despite accepted RESUME: retry must be a no-op.
			h.tick(1)
			require.Equal(t, []mowgli.FleetYieldReq{original, original}, h.calls(fleetResume))
			require.Equal(t, 1, h.server.resumes)
			require.Equal(t, 4, h.server.target)
			require.Zero(t, h.server.rawStarts)
			require.False(t, h.c.Status().Yielded)
			require.Equal(t, h.now, h.c.yield.ResumedAt)
			h.status(2, "MOWING")
			h.server.phase = "MOWING"
			h.near()
			h.tick(19)
			require.Len(t, h.calls(fleetPause), 1)
			h.tick(1)
			require.Len(t, h.calls(fleetPause), 2)
		})
	}
}

func TestFleetYieldUncertainResumeRechecksReturningPeer(t *testing.T) {
	for _, delivered := range []bool{false, true} {
		t.Run(fmt.Sprint(delivered), func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.enable(t)
			h.server.phase = "IDLE"
			h.status(1, "IDLE")
			h.far()
			h.tick(1)
			h.lostOperation = fleetResume
			h.loseAfter = delivered
			h.loseBefore = !delivered
			h.tick(3)
			h.loseAfter, h.loseBefore = false, false
			original := h.calls(fleetResume)[0]
			h.near()
			h.tick(1)
			require.Len(t, h.calls(fleetResume), 1)
			h.far()
			h.tick(1)
			h.tick(2)
			require.Len(t, h.calls(fleetResume), 1)
			h.tick(1)
			require.Equal(t, []mowgli.FleetYieldReq{original, original}, h.calls(fleetResume))
			require.Equal(t, 1, h.server.resumes)
			require.Equal(t, 4, h.server.target)
		})
	}
}

func TestFleetYieldNotReadyOnlyRetriesItsOwnedResume(t *testing.T) {
	h := newOwnedYieldTest(t)
	h.enable(t)
	h.far()
	h.tick(1)
	h.tick(3)
	require.True(t, h.c.Status().Yielded)
	require.Zero(t, h.server.resumes)
	original := h.calls(fleetResume)[0]
	require.NotEmpty(t, h.c.Status().LastError)
	h.server.phase = "IDLE"
	h.tick(1) // cached MOWING must not drop ownership
	require.Equal(t, []mowgli.FleetYieldReq{original, original}, h.calls(fleetResume))
	require.Equal(t, 1, h.server.resumes)
}

func TestFleetYieldOperatorCommandsFenceCachedIdleResume(t *testing.T) {
	for _, command := range []string{"STOP", "HOME", "MANUAL", "RECORD", "START_IN_AREA", "START", "EMERGENCY", "MAINTENANCE", "RESET"} {
		t.Run(command, func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.enable(t)
			old := h.calls(fleetPause)[0]
			h.server.operator(command, "IDLE") // operator STOP can publish exactly the same IDLE
			h.status(1, "IDLE")
			h.far()
			h.tick(1)
			h.tick(3)
			require.False(t, h.c.Status().Yielded)
			require.Zero(t, h.server.resumes)
			// The terminal result's NEW epoch must not be used to resurrect this event.
			h.status(2, "MOWING")
			h.near()
			h.tick(1)
			h.tick(60)
			require.Len(t, h.calls(fleetPause), 1)
			require.Equal(t, fleetRevoked, h.server.call(old).Result)
			if command == "START_IN_AREA" {
				require.Equal(t, 7, h.server.target)
			}
		})
	}
}

func TestFleetYieldTakeoverCancelsEvenWithoutPosition(t *testing.T) {
	for _, phase := range []struct {
		state int
		name  string
	}{{2, "RETURNING_HOME"}, {4, "MANUAL_MOWING"}, {3, "RECORDING"}, {0, "EMERGENCY"}, {1, "CHARGING"}, {1, "RAIN"}, {1, "MOWING_COMPLETE"}, {1, "UNKNOWN"}, {4, "IDLE"}} {
		t.Run(phase.name, func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.enable(t)
			old := h.calls(fleetPause)[0]
			h.status(phase.state, phase.name)
			delete(h.rows[0].Topics, "gps")
			h.tick(1)
			require.Len(t, h.calls(fleetCancel), 1)
			require.False(t, h.c.Status().Yielded)
			require.Zero(t, h.server.resumes)
			require.Equal(t, fleetRevoked, h.server.call(old).Result)
			h.status(1, "IDLE")
			h.tick(50)
			require.Empty(t, h.calls(fleetResume))
		})
	}
}

func TestFleetYieldDisableRetainsUncertainCancelAcrossReenable(t *testing.T) {
	for _, delivered := range []bool{false, true} {
		t.Run(fmt.Sprint(delivered), func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.enable(t)
			old := h.calls(fleetPause)[0]
			h.lostOperation = fleetCancel
			h.loseAfter = delivered
			h.loseBefore = !delivered
			h.now = h.now.Add(time.Second)
			require.NoError(t, h.c.SetSettings(CoordinatorSettings{Enabled: false}))
			require.True(t, h.c.Status().Yielded)
			require.Equal(t, fleetCancel, h.c.yield.Operation)
			cancel := h.calls(fleetCancel)[0]
			require.Equal(t, old.Epoch, cancel.Epoch)
			require.Equal(t, old.RequestId, cancel.RequestId)
			h.enable(t)
			require.Len(t, h.calls(fleetPause), 1)
			h.loseAfter, h.loseBefore = false, false
			h.tick(1)
			require.Equal(t, []mowgli.FleetYieldReq{cancel, cancel}, h.calls(fleetCancel))
			require.False(t, h.c.Status().Yielded)
			h.tick(60)
			require.Len(t, h.calls(fleetPause), 1)
			require.Empty(t, h.calls(fleetResume))
			require.Equal(t, fleetRevoked, h.server.call(old).Result)
		})
	}
}

func TestFleetYieldCancelFencesUndeliveredPause(t *testing.T) {
	h := newOwnedYieldTest(t)
	h.lostOperation = fleetPause
	h.loseBefore = true
	h.enable(t)
	pause := h.calls(fleetPause)[0]
	h.loseBefore = false
	h.now = h.now.Add(time.Second)
	require.NoError(t, h.c.SetSettings(CoordinatorSettings{Enabled: false}))
	require.Equal(t, fleetRevoked, h.server.call(pause).Result)
	resume := pause
	resume.Operation = fleetResume
	require.Equal(t, fleetRevoked, h.server.call(resume).Result)
	require.Nil(t, h.server.owner)
	require.Zero(t, h.server.pauses)
}

func TestFleetYieldRestartNeverAdoptsServerHold(t *testing.T) {
	h := newOwnedYieldTest(t)
	h.enable(t)
	owner := h.calls(fleetPause)[0]
	restarted := newFleetCoordinator(h.c.db, h.ros, h.c.snapshot, h.c.identity, func() time.Time { return h.now })
	restarted.tick(h.now.Add(time.Second))
	restarted.tick(h.now.Add(2 * time.Second))
	require.False(t, restarted.Status().Yielded)
	require.Len(t, h.calls(fleetPause), 1)
	require.Empty(t, h.calls(fleetResume))
	require.Empty(t, h.calls(fleetCancel))
	require.True(t, sameYield(h.server.owner, owner))
}

func TestFleetYieldMissingServiceFailsClearlyWithoutLegacyFallback(t *testing.T) {
	h := newOwnedYieldTest(t)
	h.absent = true
	h.enable(t)
	require.Contains(t, h.c.Status().LastError, "fleet_yield")
	require.Contains(t, h.c.Status().LastError, "compatible ROS")
	require.False(t, h.c.Status().Yielded)
	require.Empty(t, serviceCalls(h.ros, highLevelControlService))
	require.Zero(t, h.server.pauses)
}

func TestFleetYieldTerminalPauseDoesNotMintEpochsForStaleMowing(t *testing.T) {
	for _, phase := range []string{"IDLE", "RETURNING_HOME", "MAINTENANCE"} {
		t.Run(phase, func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.server.operator("STOP", phase)
			h.enable(t)
			h.tick(1)
			h.tick(60)
			require.Len(t, h.calls(fleetQuery), 1)
			require.Len(t, h.calls(fleetPause), 1)
			require.False(t, h.c.Status().Yielded)
			// A genuine observed non-autonomous -> autonomous transition permits a new event.
			h.status(1, "IDLE")
			h.tick(1)
			h.server.operator("START", "MOWING")
			h.status(2, "MOWING")
			h.tick(1)
			require.Len(t, h.calls(fleetPause), 2)
			require.True(t, h.c.Status().Yielded)
		})
	}
}

func TestFleetYieldDelayedOperationsAfterResumeCannotChangeTarget(t *testing.T) {
	h := newOwnedYieldTest(t)
	h.enable(t)
	pause := h.calls(fleetPause)[0]
	h.clear(t)
	resume := h.calls(fleetResume)[0]
	require.Equal(t, fleetRevoked, h.server.call(pause).Result)
	cancel := pause
	cancel.Operation = fleetCancel
	require.Equal(t, fleetRevoked, h.server.call(cancel).Result)
	require.Equal(t, fleetAlreadyResumed, h.server.call(resume).Result)
	require.Equal(t, 1, h.server.resumes)
	require.Equal(t, 4, h.server.target)
}

func TestFleetYieldPolicyRequiresEligibleAutonomyPriorityAndPosition(t *testing.T) {
	for _, scenario := range []string{"lower priority peer", "idle", "home", "unknown autonomous phase", "no position", "near hold"} {
		t.Run(scenario, func(t *testing.T) {
			h := newOwnedYieldTest(t)
			switch scenario {
			case "lower priority peer":
				h.rows[1].Identity.ID = "z"
			case "idle":
				h.status(1, "IDLE")
			case "home":
				h.status(2, "RETURNING_HOME")
			case "unknown autonomous phase":
				h.status(2, "NEW_PHASE")
			case "no position":
				delete(h.rows[0].Topics, "gps")
			}
			h.enable(t)
			h.tick(2)
			if scenario == "near hold" {
				require.Len(t, h.calls(fleetPause), 1)
				require.Empty(t, h.calls(fleetResume))
				require.True(t, h.c.Status().Yielded)
				h.server.phase = "IDLE"
				h.tick(43)
				require.Len(t, h.calls(fleetResume), 1)
			} else {
				require.Empty(t, h.calls(fleetQuery))
				require.Empty(t, h.calls(fleetPause))
			}
		})
	}
}

func TestFleetYieldPendingCancelReconcilesDespiteSnapshotFailure(t *testing.T) {
	h := newOwnedYieldTest(t)
	h.enable(t)
	h.lostOperation = fleetCancel
	h.loseBefore = true
	h.now = h.now.Add(time.Second)
	require.NoError(t, h.c.SetSettings(CoordinatorSettings{Enabled: false}))
	h.enable(t)
	h.c.snapshot = func() ([]FleetRobot, error) { return nil, errors.New("snapshot unavailable") }
	h.loseBefore = false
	h.tick(1)
	require.Len(t, h.calls(fleetCancel), 2)
	require.False(t, h.c.Status().Yielded)
	require.Nil(t, h.server.owner)
}

func TestFleetYieldCancelledUncertainResumeStillGetsCooldown(t *testing.T) {
	for _, delivered := range []bool{false, true} {
		t.Run(fmt.Sprint(delivered), func(t *testing.T) {
			h := newOwnedYieldTest(t)
			h.enable(t)
			h.server.phase = "IDLE"
			h.status(1, "IDLE")
			h.far()
			h.tick(1)
			h.lostOperation = fleetResume
			h.loseAfter = delivered
			h.loseBefore = !delivered
			h.tick(3)
			h.loseAfter, h.loseBefore = false, false
			h.now = h.now.Add(time.Second)
			require.NoError(t, h.c.SetSettings(CoordinatorSettings{Enabled: false}))
			require.False(t, h.c.Status().Yielded)
			require.Equal(t, h.now, h.c.yield.ResumedAt)
			h.enable(t)
			h.tick(1) // observe the inactive phase, then a new START
			h.server.operator("START", "MOWING")
			h.status(2, "MOWING")
			h.near()
			h.tick(1)
			require.Len(t, h.calls(fleetPause), 1, "an uncertain accepted resume needs cooldown even after cancellation")
		})
	}
}
