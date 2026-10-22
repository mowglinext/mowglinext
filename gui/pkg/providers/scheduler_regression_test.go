package providers

import (
	"encoding/json"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

type scheduleReadBarrierDB struct {
	*types.MockDBProvider
	read, release chan struct{}
}

func (d *scheduleReadBarrierDB) Get(key string) ([]byte, error) {
	data, err := d.MockDBProvider.Get(key)
	close(d.read)
	<-d.release
	return data, err
}

func TestScheduleMetadataConcurrentMutation(t *testing.T) {
	for _, action := range []string{"delete", "disable", "edit"} {
		t.Run(action, func(t *testing.T) {
			db := types.NewMockDBProvider()
			storeSchedule(t, db, schedule{ID: "race", Enabled: true, Time: "09:00", DaysOfWeek: []int{1}})
			barrier := &scheduleReadBarrierDB{MockDBProvider: db, read: make(chan struct{}), release: make(chan struct{})}
			s := &SchedulerProvider{dbProvider: barrier}
			done := make(chan struct{})
			go func() {
				defer close(done)
				s.updateRunMetadata("race", func(current *schedule) {
					current.LastSkipReason = "soil wet"
					stamp := time.Date(2026, 10, 21, 9, 0, 0, 0, time.UTC)
					current.LastRun = &stamp
				})
			}()
			<-barrier.read
			attempted, written := make(chan struct{}), make(chan struct{})
			go func() {
				defer close(written)
				// TryLock makes both schedules deterministic: if metadata owns
				// the transaction, the operator waits; otherwise mutate before
				// releasing its stale read. No sleep or scheduler timing guess.
				locked := ScheduleWriteMu.TryLock()
				if !locked {
					close(attempted)
					ScheduleWriteMu.Lock()
				}
				defer ScheduleWriteMu.Unlock()
				if action == "delete" {
					_ = db.Delete("schedule:race")
				} else {
					value, _ := db.Get("schedule:race")
					var current schedule
					_ = json.Unmarshal(value, &current)
					if action == "disable" {
						current.Enabled = false
					} else {
						current.Time = "10:30"
						current.DaysOfWeek = []int{2, 4}
					}
					value, _ = json.Marshal(current)
					_ = db.Set("schedule:race", value)
				}
				if locked {
					close(attempted)
				}
			}()
			<-attempted
			close(barrier.release)
			<-done
			<-written
			value, err := db.Get("schedule:race")
			if action == "delete" {
				require.ErrorIs(t, err, types.ErrKeyNotFound)
				return
			}
			require.NoError(t, err)
			var current schedule
			require.NoError(t, json.Unmarshal(value, &current))
			if action == "disable" {
				assert.False(t, current.Enabled)
			} else {
				assert.Equal(t, "10:30", current.Time)
				assert.Equal(t, []int{2, 4}, current.DaysOfWeek)
			}
			assert.Equal(t, "soil wet", current.LastSkipReason, "both writes survive in their serialized order")
			assert.NotNil(t, current.LastRun)
		})
	}
}

var validSchedulerInputs = []struct{ topic, raw string }{
	{"highLevelStatus", `{"state":1,"state_name":"IDLE","emergency":false}`},
	{"emergency", `{"active_emergency":false,"latched_emergency":false}`},
	{"coverageSession", `{"session_active":false}`},
	{"coverageResumeAvailable", `{"data":false}`},
}

func TestSchedulerRequiresEveryAdmissionSignal(t *testing.T) {
	for _, missing := range validSchedulerInputs {
		t.Run(missing.topic, func(t *testing.T) {
			ros, db := types.NewMockRosProvider(), types.NewMockDBProvider()
			s := &SchedulerProvider{rosProvider: ros, dbProvider: db}
			s.subscribeToStatus()
			for _, input := range validSchedulerInputs {
				if input.topic != missing.topic {
					ros.Dispatch(input.topic, []byte(input.raw))
				}
			}
			assert.False(t, s.safeToStart(), "missing input cannot establish safety")
			ros.Dispatch(missing.topic, []byte(missing.raw))
			assert.True(t, s.safeToStart(), "all explicit safe inputs admit the nominal idle state")
		})
	}
}

func TestSchedulerInvalidTelemetryRevokesAdmissionAndRecovers(t *testing.T) {
	now := time.Date(2026, 10, 21, 9, 0, 0, 0, time.UTC)
	for _, input := range validSchedulerInputs {
		bad := []string{`{}`, `null`, `{`, `[]`, `true`}
		switch input.topic {
		case "highLevelStatus":
			bad = append(bad, `{"state":1,"state_name":"IDLE"}`, `{"state":1,"emergency":false}`,
				`{"state":"1","state_name":"IDLE","emergency":false}`, `{"state":1,"state_name":"IDLE","emergency":"false"}`,
				`{"state":1,"state_name":"IDLE","emergency":null}`, `{"state":1,"state_name":"IDLE","emergency":false,"Emergency":true}`)
		case "emergency":
			bad = append(bad, `{"active_emergency":"false"}`, `{"active_emergency":0}`, `{"active_emergency":null}`, `{"active_emergency":false,"ActiveEmergency":true}`)
		case "coverageSession":
			bad = append(bad, `{"session_active":"false"}`, `{"session_active":0}`, `{"session_active":null}`, `{"session_active":false,"SessionActive":true}`)
		case "coverageResumeAvailable":
			bad = append(bad, `{"data":"false"}`, `{"data":0}`, `{"data":null}`, `{"data":false,"Data":true}`)
		}
		for _, raw := range bad {
			t.Run(input.topic+"/"+raw, func(t *testing.T) {
				ros, db := types.NewMockRosProvider(), types.NewMockDBProvider()
				ros.ServiceResponder = acceptStart
				storeSchedule(t, db, schedule{ID: "admission", Enabled: true, Time: "09:00", DaysOfWeek: []int{3}})
				s := &SchedulerProvider{rosProvider: ros, dbProvider: db}
				s.subscribeToStatus()
				for _, good := range validSchedulerInputs {
					ros.Dispatch(good.topic, []byte(good.raw))
				}
				require.True(t, s.safeToStart())
				ros.Dispatch(input.topic, []byte(raw))
				s.checkSchedulesAt(now)
				assert.Empty(t, ros.ServiceCalls)
				assert.False(t, s.safeToStart())
				assert.Nil(t, readSchedule(t, db, "admission").LastRun)
				ros.Dispatch(input.topic, []byte(input.raw))
				s.checkSchedulesAt(now)
				require.Len(t, ros.ServiceCalls, 1, "a genuinely complete safe input can recover")
				s.checkSchedulesAt(now)
				assert.Len(t, ros.ServiceCalls, 1, "#702: no second start in the same minute")
			})
		}
	}
}

func TestSchedulerAcceptsExplicitCompatibleAliases(t *testing.T) {
	ros := types.NewMockRosProvider()
	s := &SchedulerProvider{rosProvider: ros}
	s.subscribeToStatus()
	ros.Dispatch("highLevelStatus", []byte(`{"State":1,"StateName":"IDLE","Emergency":false}`))
	ros.Dispatch("emergency", []byte(`{"ActiveEmergency":false,"LatchedEmergency":false}`))
	ros.Dispatch("coverageSession", []byte(`{"SessionActive":false}`))
	ros.Dispatch("coverageResumeAvailable", []byte(`{"Data":false}`))
	assert.True(t, s.safeToStart())
}

func TestSchedulerEmergencyRequiresActiveAndLatched(t *testing.T) {
	now := time.Date(2026, 10, 21, 9, 0, 0, 0, time.UTC)
	for _, tc := range []struct {
		name, raw string
		known     bool
	}{
		{"latched-only", `{"active_emergency":false,"latched_emergency":true}`, true},
		{"active-only", `{"active_emergency":true,"latched_emergency":false}`, true},
		{"both-active", `{"active_emergency":true,"latched_emergency":true}`, true},
		{"missing-active", `{"latched_emergency":false}`, false},
		{"missing-latched", `{"active_emergency":false}`, false},
		{"active-string", `{"active_emergency":"false","latched_emergency":false}`, false},
		{"active-number", `{"active_emergency":0,"latched_emergency":false}`, false},
		{"active-null", `{"active_emergency":null,"latched_emergency":false}`, false},
		{"latched-string", `{"active_emergency":false,"latched_emergency":"false"}`, false},
		{"latched-number", `{"active_emergency":false,"latched_emergency":0}`, false},
		{"latched-null", `{"active_emergency":false,"latched_emergency":null}`, false},
		{"active-alias-conflict", `{"active_emergency":false,"ActiveEmergency":true,"latched_emergency":false}`, false},
		{"latched-alias-conflict", `{"active_emergency":false,"latched_emergency":false,"LatchedEmergency":true}`, false},
		{"latched-alias-malformed", `{"active_emergency":false,"latched_emergency":false,"LatchedEmergency":"false"}`, false},
		{"latched-only-alias", `{"ActiveEmergency":false,"LatchedEmergency":true}`, true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ros, db := types.NewMockRosProvider(), types.NewMockDBProvider()
			ros.ServiceResponder = acceptStart
			storeSchedule(t, db, schedule{ID: "emergency", Enabled: true, Time: "09:00", DaysOfWeek: []int{3}})
			s := &SchedulerProvider{rosProvider: ros, dbProvider: db}
			s.subscribeToStatus()
			for _, input := range validSchedulerInputs {
				ros.Dispatch(input.topic, []byte(input.raw))
			}
			require.True(t, s.safeToStart())
			ros.Dispatch("emergency", []byte(tc.raw))
			assert.Equal(t, tc.known, s.hasEmergencyStatus)
			s.checkSchedulesAt(now)
			assert.False(t, s.safeToStart(), "a latch or unknown emergency cannot establish safety")
			assert.Empty(t, ros.ServiceCalls)
			assert.Nil(t, readSchedule(t, db, "emergency").LastRun)
			// An HLS false cannot clear either a latch or an unknown hardware observation.
			ros.Dispatch("highLevelStatus", []byte(validSchedulerInputs[0].raw))
			s.checkSchedulesAt(now)
			assert.Empty(t, ros.ServiceCalls)
			if tc.known {
				ros.Dispatch("emergency", []byte(`{"active_emergency":false}`))
				assert.False(t, s.hasEmergencyStatus)
				assert.True(t, s.lastEmergency, "an incomplete frame must preserve a positive guard")
				s.checkSchedulesAt(now)
				assert.Empty(t, ros.ServiceCalls)
			}
			// Both aliases may coexist only when they agree. A complete safe
			// observation recovers admission and #702 still permits one START.
			ros.Dispatch("emergency", []byte(`{"active_emergency":false,"ActiveEmergency":false,"latched_emergency":false,"LatchedEmergency":false}`))
			s.checkSchedulesAt(now)
			require.Len(t, ros.ServiceCalls, 1)
			s.checkSchedulesAt(now)
			assert.Len(t, ros.ServiceCalls, 1)
		})
	}
}

func TestScheduleMalformedAdmissionCannotStart(t *testing.T) {
	now := time.Date(2026, 10, 21, 9, 0, 0, 0, time.UTC)
	for _, bad := range []string{`{}`, `null`, `{"active_emergency":"invalid"}`} {
		t.Run(bad, func(t *testing.T) {
			ros := types.NewMockRosProvider()
			ros.ServiceResponder = acceptStart
			db := types.NewMockDBProvider()
			storeSchedule(t, db, schedule{ID: "unsafe", Enabled: true, Time: "09:00", DaysOfWeek: []int{3}})
			s := &SchedulerProvider{rosProvider: ros, dbProvider: db}
			s.subscribeToStatus()
			ros.Dispatch("highLevelStatus", []byte(`{"state":1,"state_name":"IDLE","emergency":false}`))
			ros.Dispatch("emergency", []byte(`{"active_emergency":true,"latched_emergency":false}`))
			ros.Dispatch("coverageSession", []byte(`{}`))
			ros.Dispatch("coverageResumeAvailable", []byte(`{}`))
			ros.Dispatch("emergency", []byte(bad))
			s.checkSchedulesAt(now)
			assert.Empty(t, ros.ServiceCalls, "unknown admission is not an explicitly safe state")
			assert.Nil(t, readSchedule(t, db, "unsafe").LastRun)
		})
	}
}
