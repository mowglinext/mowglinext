package api

import (
	"encoding/json"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func enabledAt(id, hhmm string, days ...int) *Schedule {
	return &Schedule{ID: id, Time: hhmm, DaysOfWeek: days, Enabled: true}
}

func TestSchedulesOverlap_SharedDayWithinAnHour(t *testing.T) {
	assert.True(t, schedulesOverlap(enabledAt("a", "09:00", 1), enabledAt("b", "09:00", 1)), "same start")
	assert.True(t, schedulesOverlap(enabledAt("a", "09:00", 1), enabledAt("b", "09:59", 1)), "59 minutes apart")
	assert.True(t, schedulesOverlap(enabledAt("a", "09:30", 1), enabledAt("b", "09:00", 1)), "order does not matter")
}

func TestSchedulesOverlap_ExactlyAnHourApartIsAllowed(t *testing.T) {
	assert.False(t, schedulesOverlap(enabledAt("a", "09:00", 1), enabledAt("b", "10:00", 1)))
}

func TestSchedulesOverlap_NoSharedDayNeverOverlaps(t *testing.T) {
	assert.False(t, schedulesOverlap(enabledAt("a", "09:00", 1, 3), enabledAt("b", "09:00", 2, 4)))
}

func TestSchedulesOverlap_OnlyOneSharedDayIsEnough(t *testing.T) {
	assert.True(t, schedulesOverlap(enabledAt("a", "09:00", 1, 3, 5), enabledAt("b", "09:30", 5, 6)))
}

func TestSchedulesOverlap_WrapsAcrossMidnight(t *testing.T) {
	// Monday 23:30 and Tuesday 00:15 are 45 minutes apart.
	assert.True(t, schedulesOverlap(enabledAt("a", "23:30", 1), enabledAt("b", "00:15", 2)))
	// Saturday 23:30 and Sunday 00:15 wrap around the end of the week.
	assert.True(t, schedulesOverlap(enabledAt("a", "23:30", 6), enabledAt("b", "00:15", 0)))
	// Monday 23:30 and Tuesday 00:30 are exactly an hour apart.
	assert.False(t, schedulesOverlap(enabledAt("a", "23:30", 1), enabledAt("b", "00:30", 2)))
}

func TestSchedulesOverlap_IgnoresMalformedStoredTimes(t *testing.T) {
	assert.False(t, schedulesOverlap(enabledAt("a", "09:00", 1), enabledAt("b", "not-a-time", 1)))
}

func TestFindOverlappingSchedule_DisabledCandidateNeverConflicts(t *testing.T) {
	candidate := enabledAt("new", "09:00", 1)
	candidate.Enabled = false
	existing := []Schedule{*enabledAt("old", "09:00", 1)}
	assert.Nil(t, findOverlappingSchedule(candidate, existing), "a disabled schedule cannot fire, so it cannot overlap")
}

func TestFindOverlappingSchedule_DisabledExistingScheduleIsIgnored(t *testing.T) {
	existing := enabledAt("old", "09:00", 1)
	existing.Enabled = false
	assert.Nil(t, findOverlappingSchedule(enabledAt("new", "09:00", 1), []Schedule{*existing}))
}

func TestFindOverlappingSchedule_ASchedulePassesAgainstItself(t *testing.T) {
	existing := []Schedule{*enabledAt("same", "09:00", 1)}
	assert.Nil(t, findOverlappingSchedule(enabledAt("same", "09:30", 1), existing), "an edit must not conflict with its own stored copy")
}

func TestSaveScheduleChecked_RejectsAnOverlapAndKeepsTheStoreUntouched(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, enabledAt("old", "09:00", 1)))

	err := saveScheduleChecked(db, enabledAt("new", "09:30", 1))

	var conflict *ScheduleConflictError
	require.ErrorAs(t, err, &conflict)
	assert.Equal(t, "old", conflict.With.ID)
	all, err := getAllSchedules(db)
	require.NoError(t, err)
	assert.Len(t, all, 1, "the rejected schedule must not be stored")
}

func TestSaveScheduleChecked_AllowsDisablingAnOverlappingSchedule(t *testing.T) {
	db := types.NewMockDBProvider()
	// A legacy overlap that predates the rule.
	require.NoError(t, saveSchedule(db, enabledAt("a", "09:00", 1)))
	require.NoError(t, saveSchedule(db, enabledAt("b", "09:10", 1)))

	disable := enabledAt("b", "09:10", 1)
	disable.Enabled = false
	require.NoError(t, saveScheduleChecked(db, disable), "turning a schedule off must always work")
}

func TestScheduleMqttBridge_SetRejectsAnOverlappingSchedule(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, saveSchedule(db, enabledAt("old", "09:00", 1)))
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"time":"09:20","daysOfWeek":[1],"enabled":true}`))

	all, err := getAllSchedules(db)
	require.NoError(t, err)
	assert.Len(t, all, 1, "the overlapping schedule must be dropped, same as over HTTP")
}

func TestScheduleMqttBridge_AreaIdAndNameArePublishedAndSettable(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set",
		[]byte(`{"areaId":7,"areaName":"  Front lawn ","time":"06:00","daysOfWeek":[2],"enabled":true}`))

	pub, ok := client.lastPublished("mowgli/schedules")
	require.True(t, ok)
	var resp ScheduleListResponse
	require.NoError(t, json.Unmarshal(pub.payload, &resp))
	require.Len(t, resp.Schedules, 1)
	assert.Equal(t, uint32(7), resp.Schedules[0].AreaID)
	assert.Equal(t, "Front lawn", resp.Schedules[0].AreaName, "the name is trimmed")
	assert.Contains(t, string(pub.payload), `"areaId":7`)
}

func TestValidateSchedule_AllAreasCarriesNoStaleName(t *testing.T) {
	s := &Schedule{Time: "06:00", DaysOfWeek: []int{1}, AreaID: 0, AreaName: "Old area"}
	require.NoError(t, validateSchedule(s))
	assert.Empty(t, s.AreaName)
}
