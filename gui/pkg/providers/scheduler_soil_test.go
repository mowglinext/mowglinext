package providers

import (
	"encoding/json"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeSoilProvider is the scheduler-side test double for ISoilProvider.
type fakeSoilProvider struct {
	status types.SoilStatus
}

func (f *fakeSoilProvider) SoilStatus() types.SoilStatus { return f.status }

func wetSoil() types.SoilStatus {
	return types.SoilStatus{
		Enabled: true, Configured: true, GateScheduler: true, Fresh: true, Wet: true,
		Reason: `zone "Pelouse nord": deficit 0.8 mm ≤ 2.0 mm, watered 40 min ago`,
	}
}

func dueSchedule(id string, now time.Time) schedule {
	return schedule{
		ID:         id,
		Time:       now.Format("15:04"),
		DaysOfWeek: []int{int(now.Weekday())},
		Enabled:    true,
	}
}

func buildSoilScheduler(ros *types.MockRosProvider, db *types.MockDBProvider, soil types.ISoilProvider) *SchedulerProvider {
	s := buildScheduler(ros, db)
	s.soilProvider = soil
	s.lastHighLevelState = 1 // IDLE
	s.lastEmergency = false
	return s
}

func readSchedule(t *testing.T, db *types.MockDBProvider, id string) schedule {
	t.Helper()
	data, err := db.Get(schedulerKeyPrefix + id)
	require.NoError(t, err)
	var s schedule
	require.NoError(t, json.Unmarshal(data, &s))
	return s
}

func TestCheckSchedules_WetSoilSkipsAndRecordsReason(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-1", now))

	s := buildSoilScheduler(ros, db, &fakeSoilProvider{status: wetSoil()})
	s.checkSchedules()

	assert.Empty(t, ros.ServiceCalls, "wet soil must not start a mow")
	saved := readSchedule(t, db, "soil-1")
	assert.Equal(t, wetSoil().Reason, saved.LastSkipReason)
	require.NotNil(t, saved.LastSkippedAt)
	assert.Nil(t, saved.LastRun, "a skipped run is not a run")
}

func TestCheckSchedules_UnknownSoilRunsAsUsual(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-2", now))

	unknown := types.SoilStatus{Enabled: true, Configured: true, GateScheduler: true, Unknown: true, Reason: "IrriSense unreachable"}
	s := buildSoilScheduler(ros, db, &fakeSoilProvider{status: unknown})
	s.checkSchedules()

	require.Len(t, ros.ServiceCalls, 1, "unknown soil state is fail-open")
	assert.Empty(t, readSchedule(t, db, "soil-2").LastSkipReason)
}

func TestCheckSchedules_StaleWetSoilRunsAsUsual(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-3", now))

	stale := wetSoil()
	stale.Fresh = false
	stale.Unknown = true
	s := buildSoilScheduler(ros, db, &fakeSoilProvider{status: stale})
	s.checkSchedules()

	require.Len(t, ros.ServiceCalls, 1, "a stale verdict must not block")
}

func TestCheckSchedules_DisabledIntegrationRunsAsUsual(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-4", now))

	disabled := wetSoil()
	disabled.Enabled = false
	s := buildSoilScheduler(ros, db, &fakeSoilProvider{status: disabled})
	s.checkSchedules()

	require.Len(t, ros.ServiceCalls, 1)
}

func TestCheckSchedules_GateSwitchedOffRunsEvenWhenWet(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-5", now))

	ungated := wetSoil()
	ungated.GateScheduler = false
	s := buildSoilScheduler(ros, db, &fakeSoilProvider{status: ungated})
	s.checkSchedules()

	require.Len(t, ros.ServiceCalls, 1, "gateScheduler=false means status only, no blocking")
}

func TestCheckSchedules_NoSoilProviderRunsAsUsual(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-6", now))

	s := buildSoilScheduler(ros, db, nil)
	s.checkSchedules()

	require.Len(t, ros.ServiceCalls, 1)
}

func TestCheckSchedules_SafetyCheckStillWinsOverSoil(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueSchedule("soil-7", now))

	s := buildSoilScheduler(ros, db, &fakeSoilProvider{status: wetSoil()})
	s.lastEmergency = true
	s.checkSchedules()

	assert.Empty(t, ros.ServiceCalls)
	assert.Empty(t, readSchedule(t, db, "soil-7").LastSkipReason, "an emergency skip is not a soil skip")
}

func TestNewSchedulerProvider_AcceptsNilSoilProvider(t *testing.T) {
	ros := types.NewMockRosProvider()
	db := types.NewMockDBProvider()
	s := NewSchedulerProvider(ros, db, nil)
	require.NotNil(t, s)
	blocked, _ := s.soilBlocksStart()
	assert.False(t, blocked)
}
