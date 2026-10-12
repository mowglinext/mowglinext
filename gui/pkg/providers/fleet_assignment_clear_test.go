package providers

import (
	"encoding/json"
	"errors"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/stretchr/testify/require"
)

func TestCoordinatorDisabledClearRetriesUntilAcknowledged(t *testing.T) {
	for _, mode := range []string{"refused", "transport"} {
		t.Run(mode, func(t *testing.T) {
			rows := []FleetRobot{
				{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true},
				{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
					"highLevelStatus": json.RawMessage(`{"state":2,"current_area":3}`),
				}},
			}
			c, _, ros := newTestCoordinator(t, &rows)
			now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
			c.now = func() time.Time { return now }
			require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
			require.Equal(t, []uint32{3}, c.Status().ExcludedAreas)
			if mode == "transport" {
				ros.ServiceErr = errors.New("bridge disconnected")
			} else {
				ros.ServiceResponder = func(_ string, _ any, res any) {
					r := res.(*mowgli.SetFleetAssignmentRes)
					r.Success = false
					r.Message = "not ready"
				}
			}
			require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: false}))
			require.Len(t, serviceCalls(ros, setFleetAssignmentService), 2)
			require.False(t, c.Status().Enabled)
			require.Equal(t, []uint32{3}, c.Status().ExcludedAreas, "failed clear must not be recorded as applied")
			require.NotEmpty(t, c.Status().LastError)
			// Immediate repeated ticks and settings saves must not spin retries.
			c.tick(now)
			c.tick(now.Add(time.Millisecond))
			require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: false}))
			require.Len(t, serviceCalls(ros, setFleetAssignmentService), 2)
			now = now.Add(coordinatorTick)
			c.tick(now)
			require.Len(t, serviceCalls(ros, setFleetAssignmentService), 3, "disabled must retry after a failed clear")
			ros.ServiceErr = nil
			ros.ServiceResponder = func(_ string, _ any, res any) { res.(*mowgli.SetFleetAssignmentRes).Success = true }
			now = now.Add(coordinatorTick)
			c.tick(now)
			calls := serviceCalls(ros, setFleetAssignmentService)
			require.Len(t, calls, 4)
			for _, call := range calls[1:] {
				req := call.Req.(*mowgli.SetFleetAssignmentReq)
				require.Empty(t, req.ExcludedAreas)
				require.Equal(t, int32(-1), req.PreferredStartIndex)
			}
			require.Empty(t, c.Status().ExcludedAreas)
			require.Empty(t, c.Status().LastError)
			c.tick(now.Add(2 * assignmentRefresh))
			require.Len(t, serviceCalls(ros, setFleetAssignmentService), 4, "acknowledged clear stops retries")
			require.Empty(t, serviceCalls(ros, highLevelControlService), "clearing work selection must not send movement commands")
		})
	}
}

func TestCoordinatorReenableSupersedesPendingClear(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"current_area":3}`),
		}},
	}
	c, _, ros := newTestCoordinator(t, &rows)
	now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
	c.now = func() time.Time { return now }
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	ros.ServiceErr = errors.New("reply lost; clear may have applied")
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: false}))
	ros.ServiceErr = nil
	// Even with an identical desired assignment, the failed clear has an
	// unknown outcome. Re-enable must explicitly restore the fleet assignment.
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	calls := serviceCalls(ros, setFleetAssignmentService)
	require.Len(t, calls, 3)
	require.Equal(t, []uint32{3}, calls[2].Req.(*mowgli.SetFleetAssignmentReq).ExcludedAreas)
	c.tick(now.Add(coordinatorTick))
	require.Len(t, serviceCalls(ros, setFleetAssignmentService), 3, "older clear must not run after re-enable")
	require.True(t, c.Status().Enabled)
	require.Equal(t, []uint32{3}, c.Status().ExcludedAreas)
	require.Empty(t, serviceCalls(ros, highLevelControlService))
}

func TestCoordinatorNeverEnabledReconcilesOnce(t *testing.T) {
	rows := []FleetRobot{}
	c, _, ros := newTestCoordinator(t, &rows)
	now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
	c.now = func() time.Time { return now }
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: false}))
	c.tick(now.Add(coordinatorTick))
	c.tick(now.Add(assignmentRefresh))
	calls := serviceCalls(ros, setFleetAssignmentService)
	require.Len(t, calls, 1, "disabled startup confirms an empty assignment once")
	req := calls[0].Req.(*mowgli.SetFleetAssignmentReq)
	require.Empty(t, req.ExcludedAreas)
	require.Equal(t, int32(-1), req.PreferredStartIndex)
	require.Empty(t, serviceCalls(ros, highLevelControlService))
}

func TestCoordinatorRestartReconcilesDisabledAssignment(t *testing.T) {
	for _, shutdown := range []string{"failed-clear", "before-clear"} {
		for _, failure := range []string{"refused", "transport"} {
			t.Run(shutdown+"/"+failure, func(t *testing.T) {
				rows := []FleetRobot{
					{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true},
					{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
						"highLevelStatus": json.RawMessage(`{"state":2,"current_area":3}`),
					}},
				}
				c, db, ros := newTestCoordinator(t, &rows)
				now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
				c.now = func() time.Time { return now }
				// Model the separately running BT, whose assignment survives GUI restart.
				var applied []uint32
				accept := true
				ros.ServiceResponder = func(_ string, req any, res any) {
					r := res.(*mowgli.SetFleetAssignmentRes)
					r.Success = accept
					if accept {
						applied = append([]uint32(nil), req.(*mowgli.SetFleetAssignmentReq).ExcludedAreas...)
					}
				}
				require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
				require.Equal(t, []uint32{3}, applied)
				accept = false
				if failure == "transport" {
					ros.ServiceErr = errors.New("bridge disconnected")
				}
				if shutdown == "failed-clear" {
					require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: false}))
				} else {
					// Crash after settings persistence but before the old process ticks.
					raw, err := json.Marshal(CoordinatorSettings{Enabled: false})
					require.NoError(t, err)
					require.NoError(t, db.Set(coordinationSettingsKey, raw))
				}
				require.False(t, c.Settings().Enabled)
				require.Equal(t, []uint32{3}, applied)
				beforeRestart := len(serviceCalls(ros, setFleetAssignmentService))
				// Recreate only the GUI coordinator: shared settings and remote BT remain.
				restarted := newFleetCoordinator(db, ros, func() ([]FleetRobot, error) {
					t.Fatal("disabled startup must not inspect peers")
					return nil, nil
				}, c.identity, func() time.Time { return now })
				restarted.tick(now)
				require.Len(t, serviceCalls(ros, setFleetAssignmentService), beforeRestart+1)
				require.Equal(t, []uint32{3}, applied, "failed startup clear leaves remote assignment intact")
				require.NotEmpty(t, restarted.Status().LastError)
				restarted.tick(now.Add(coordinatorTick - time.Nanosecond))
				require.Len(t, serviceCalls(ros, setFleetAssignmentService), beforeRestart+1)
				accept = true
				ros.ServiceErr = nil
				now = now.Add(coordinatorTick)
				restarted.tick(now)
				calls := serviceCalls(ros, setFleetAssignmentService)
				require.Len(t, calls, beforeRestart+2)
				for _, call := range calls[beforeRestart:] {
					req := call.Req.(*mowgli.SetFleetAssignmentReq)
					require.Empty(t, req.ExcludedAreas)
					require.Equal(t, int32(-1), req.PreferredStartIndex)
				}
				require.Empty(t, applied)
				require.Empty(t, restarted.Status().LastError)
				restarted.tick(now.Add(2 * assignmentRefresh))
				require.Len(t, serviceCalls(ros, setFleetAssignmentService), beforeRestart+2, "acknowledged startup clear stops retries")
				require.Empty(t, serviceCalls(ros, highLevelControlService))
			})
		}
	}
}

func TestCoordinatorRestartEnabledDoesNotClearAssignment(t *testing.T) {
	rows := []FleetRobot{
		{Identity: RobotIdentity{ID: "b"}, Self: true, Online: true},
		{Identity: RobotIdentity{ID: "a"}, Online: true, Topics: map[string]json.RawMessage{
			"highLevelStatus": json.RawMessage(`{"state":2,"current_area":3}`),
		}},
	}
	c, db, ros := newTestCoordinator(t, &rows)
	now := time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC)
	require.NoError(t, c.SetSettings(CoordinatorSettings{Enabled: true}))
	restarted := newFleetCoordinator(db, ros, c.snapshot, c.identity, func() time.Time { return now })
	restarted.tick(now)
	calls := serviceCalls(ros, setFleetAssignmentService)
	require.Len(t, calls, 2)
	for _, call := range calls {
		require.Equal(t, []uint32{3}, call.Req.(*mowgli.SetFleetAssignmentReq).ExcludedAreas)
	}
	restarted.tick(now.Add(coordinatorTick))
	require.Len(t, serviceCalls(ros, setFleetAssignmentService), 2, "enabled startup cancels the startup clear")
}
