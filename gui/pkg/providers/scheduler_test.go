package providers

import (
	"encoding/json"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// storeSchedule marshals a schedule and stores it in the mock DB under the
// canonical key used by both the API and the scheduler.
func storeSchedule(t *testing.T, db *types.MockDBProvider, s schedule) {
	t.Helper()
	data, err := json.Marshal(s)
	require.NoError(t, err)
	require.NoError(t, db.Set(schedulerKeyPrefix+s.ID, data))
}

// buildScheduler creates a SchedulerProvider backed by mocks without starting
// the background goroutine. subscribeToStatus is still called so mock
// subscribers can be driven via Dispatch.
func buildScheduler(ros *types.MockRosProvider, db *types.MockDBProvider) *SchedulerProvider {
	return &SchedulerProvider{
		rosProvider: ros,
		dbProvider:  db,
	}
}

// --------------------------------------------------------------------------
// shouldRun
// --------------------------------------------------------------------------

func TestShouldRun_TimeMatch(t *testing.T) {
	now := time.Date(2024, 1, 15, 9, 30, 0, 0, time.Local) // Monday 09:30
	sched := &schedule{
		ID:         "1",
		Time:       "09:30",
		DaysOfWeek: []int{1}, // Monday
		Enabled:    true,
	}
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	assert.True(t, s.shouldRun(sched, int(now.Weekday()), now.Format("15:04"), now))
}

func TestShouldRun_TimeMismatch(t *testing.T) {
	now := time.Date(2024, 1, 15, 9, 31, 0, 0, time.Local)
	sched := &schedule{
		ID:         "1",
		Time:       "09:30",
		DaysOfWeek: []int{1},
		Enabled:    true,
	}
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	assert.False(t, s.shouldRun(sched, int(now.Weekday()), now.Format("15:04"), now))
}

func TestShouldRun_WrongDay(t *testing.T) {
	now := time.Date(2024, 1, 16, 9, 30, 0, 0, time.Local) // Tuesday
	sched := &schedule{
		ID:         "1",
		Time:       "09:30",
		DaysOfWeek: []int{1}, // Monday only
		Enabled:    true,
	}
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	assert.False(t, s.shouldRun(sched, int(now.Weekday()), now.Format("15:04"), now))
}

func TestShouldRun_MultipleDays(t *testing.T) {
	now := time.Date(2024, 1, 17, 8, 0, 0, 0, time.Local) // Wednesday
	sched := &schedule{
		ID:         "1",
		Time:       "08:00",
		DaysOfWeek: []int{1, 3, 5}, // Mon, Wed, Fri
		Enabled:    true,
	}
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	assert.True(t, s.shouldRun(sched, int(now.Weekday()), now.Format("15:04"), now))
}

func TestShouldRun_PreventDoubleExecution(t *testing.T) {
	now := time.Date(2024, 1, 15, 9, 30, 0, 0, time.Local)
	recent := now.Add(-30 * time.Second)
	sched := &schedule{
		ID:         "1",
		Time:       "09:30",
		DaysOfWeek: []int{1},
		Enabled:    true,
		LastRun:    &recent,
	}
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	assert.False(t, s.shouldRun(sched, int(now.Weekday()), now.Format("15:04"), now))
}

func TestShouldRun_LastRunOldEnough(t *testing.T) {
	now := time.Date(2024, 1, 15, 9, 30, 0, 0, time.Local)
	old := now.Add(-5 * time.Minute)
	sched := &schedule{
		ID:         "1",
		Time:       "09:30",
		DaysOfWeek: []int{1},
		Enabled:    true,
		LastRun:    &old,
	}
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	assert.True(t, s.shouldRun(sched, int(now.Weekday()), now.Format("15:04"), now))
}

// --------------------------------------------------------------------------
// safeToStart
// --------------------------------------------------------------------------

func TestSafeToStart_IdleNoEmergency(t *testing.T) {
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	s.lastHighLevelState = 1 // IDLE
	s.lastEmergency = false
	assert.True(t, s.safeToStart())
}

func TestSafeToStart_EmergencyActive(t *testing.T) {
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	s.lastHighLevelState = 1
	s.lastEmergency = true
	assert.False(t, s.safeToStart())
}

func TestSafeToStart_AlreadyAutonomous(t *testing.T) {
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	s.lastHighLevelState = 2 // AUTONOMOUS
	s.lastEmergency = false
	assert.False(t, s.safeToStart())
}

func TestSafeToStart_Recording(t *testing.T) {
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	s.lastHighLevelState = 3 // RECORDING
	s.lastEmergency = false
	assert.False(t, s.safeToStart())
}

func TestSafeToStart_NullState(t *testing.T) {
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	s.lastHighLevelState = 0 // NULL / transitional
	s.lastEmergency = false
	assert.False(t, s.safeToStart())
}

func TestSafeToStart_ManualMowing(t *testing.T) {
	s := buildScheduler(types.NewMockRosProvider(), types.NewMockDBProvider())
	s.lastHighLevelState = 4 // MANUAL_MOWING
	s.lastEmergency = false
	// Manual mowing is not blocked — scheduler can queue the next run
	// (firmware / BT will arbitrate), so safeToStart returns true.
	assert.True(t, s.safeToStart())
}

// --------------------------------------------------------------------------
// checkSchedules — integration-style tests using mocks
// --------------------------------------------------------------------------

func TestCheckSchedules_TriggersHighLevelControl(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	now := time.Now()
	sched := schedule{
		ID:         "sched-1",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1 // IDLE
	s.lastEmergency = false

	s.checkSchedules()

	require.Len(t, ros.ServiceCalls, 1)
	assert.Equal(t, "/behavior_tree_node/high_level_control", ros.ServiceCalls[0].Service)

	req, ok := ros.ServiceCalls[0].Req.(*mowgli.HighLevelControlReq)
	require.True(t, ok, "request should be *mowgli.HighLevelControlReq")
	assert.Equal(t, uint8(1), req.Command, "COMMAND_START must be 1")
}

func TestCheckSchedules_DisabledScheduleSkipped(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	now := time.Now()
	sched := schedule{
		ID:         "sched-2",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    false, // disabled
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	assert.Empty(t, ros.ServiceCalls, "disabled schedule must not trigger")
}

func TestCheckSchedules_EmergencyPreventsStart(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	now := time.Now()
	sched := schedule{
		ID:         "sched-3",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.lastEmergency = true // emergency active!

	s.checkSchedules()

	assert.Empty(t, ros.ServiceCalls, "emergency must prevent mowing start")
}

func TestCheckSchedules_AlreadyMowingPreventsStart(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	now := time.Now()
	sched := schedule{
		ID:         "sched-4",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 2 // AUTONOMOUS already
	s.lastEmergency = false

	s.checkSchedules()

	assert.Empty(t, ros.ServiceCalls, "already autonomous must prevent double-start")
}

func TestCheckSchedules_PersistsLastRun(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	now := time.Now()
	sched := schedule{
		ID:         "sched-5",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	// Re-read from DB and verify LastRun was written
	data, err := db.Get(schedulerKeyPrefix + sched.ID)
	require.NoError(t, err)

	var updated schedule
	require.NoError(t, json.Unmarshal(data, &updated))
	require.NotNil(t, updated.LastRun, "LastRun must be persisted after successful trigger")
}

func TestCheckSchedules_NoDoubleExecutionWithinTwoMinutes(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	now := time.Now()
	recent := now.Add(-30 * time.Second)
	sched := schedule{
		ID:         "sched-6",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
		LastRun:    &recent,
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	assert.Empty(t, ros.ServiceCalls, "schedule run within last 2 minutes must be skipped")
}

func TestCheckSchedules_ServiceErrorDoesNotPersistLastRun(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceErr = assert.AnError // simulate rosbridge failure
	db := types.NewMockDBProvider()

	now := time.Now()
	sched := schedule{
		ID:         "sched-7",
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
	}
	storeSchedule(t, db, sched)

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	// LastRun must NOT be updated when the service call fails
	data, err := db.Get(schedulerKeyPrefix + sched.ID)
	require.NoError(t, err)
	var updated schedule
	require.NoError(t, json.Unmarshal(data, &updated))
	assert.Nil(t, updated.LastRun, "LastRun must not be persisted after a failed service call")
}

// --------------------------------------------------------------------------
// subscribeToStatus — verify that dispatched messages update scheduler state
// --------------------------------------------------------------------------

func TestSubscribeToStatus_UpdatesHighLevelState(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	s := &SchedulerProvider{rosProvider: ros, dbProvider: db}
	s.subscribeToStatus()

	// Dispatch a highLevelStatus message with state = 2 (AUTONOMOUS)
	msg, err := json.Marshal(mowgli.HighLevelStatus{State: 2})
	require.NoError(t, err)
	ros.Dispatch("highLevelStatus", msg)

	// Allow the synchronous mock callback to run
	assert.Equal(t, uint8(2), s.lastHighLevelState)
}

func TestSubscribeToStatus_UpdatesEmergencyFlag(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()

	s := &SchedulerProvider{rosProvider: ros, dbProvider: db}
	s.subscribeToStatus()

	msg, err := json.Marshal(mowgli.Emergency{ActiveEmergency: true})
	require.NoError(t, err)
	ros.Dispatch("emergency", msg)

	assert.True(t, s.lastEmergency)
}
