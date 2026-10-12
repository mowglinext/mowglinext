package api

import (
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// perAreaSchedule is a stored, enabled schedule bound to one area.
func perAreaSchedule(t *testing.T, db *types.MockDBProvider) {
	t.Helper()
	require.NoError(t, saveSchedule(db, &Schedule{
		ID: "p1", AreaID: 7, AreaName: "Back garden", Time: "06:00", DaysOfWeek: []int{2}, Enabled: true,
	}))
}

func storedSchedule(t *testing.T, db *types.MockDBProvider, id string) Schedule {
	t.Helper()
	s, err := getSchedule(db, id)
	require.NoError(t, err)
	return *s
}

// An integration written before per-area schedules sends no area keys. Toggling
// `enabled` that way must not reset a per-area schedule to "all areas".
func TestScheduleMqttBridge_SetWithoutAreaKeysKeepsTheExistingArea(t *testing.T) {
	db := types.NewMockDBProvider()
	perAreaSchedule(t, db)
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set",
		[]byte(`{"id":"p1","time":"06:00","daysOfWeek":[2],"enabled":false}`))

	got := storedSchedule(t, db, "p1")
	assert.False(t, got.Enabled, "the toggle itself must still apply")
	assert.Equal(t, uint32(7), got.AreaID)
	assert.Equal(t, "Back garden", got.AreaName)
}

// An explicit areaId always wins, and 0 is an explicit "all areas".
func TestScheduleMqttBridge_ExplicitAreaIdZeroResetsToAllAreas(t *testing.T) {
	db := types.NewMockDBProvider()
	perAreaSchedule(t, db)
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set",
		[]byte(`{"id":"p1","areaId":0,"time":"06:00","daysOfWeek":[2],"enabled":true}`))

	got := storedSchedule(t, db, "p1")
	assert.Equal(t, uint32(0), got.AreaID)
	assert.Empty(t, got.AreaName)
}

func TestScheduleMqttBridge_ExplicitDifferentAreaReplacesTheArea(t *testing.T) {
	db := types.NewMockDBProvider()
	perAreaSchedule(t, db)
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set",
		[]byte(`{"id":"p1","areaId":11,"areaName":"Front lawn","time":"06:00","daysOfWeek":[2],"enabled":true}`))

	got := storedSchedule(t, db, "p1")
	assert.Equal(t, uint32(11), got.AreaID)
	assert.Equal(t, "Front lawn", got.AreaName)
}

// Same area, name omitted: the stored name stays. A DIFFERENT area must not
// inherit the old area's name.
func TestScheduleMqttBridge_AreaNameIsOnlyCarriedOverForTheSameArea(t *testing.T) {
	db := types.NewMockDBProvider()
	perAreaSchedule(t, db)
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set",
		[]byte(`{"id":"p1","areaId":7,"time":"06:00","daysOfWeek":[2],"enabled":true}`))
	assert.Equal(t, "Back garden", storedSchedule(t, db, "p1").AreaName, "same area keeps its name")

	client.fire(t, "mowgli/schedules/set",
		[]byte(`{"id":"p1","areaId":11,"time":"06:00","daysOfWeek":[2],"enabled":true}`))
	got := storedSchedule(t, db, "p1")
	assert.Equal(t, uint32(11), got.AreaID)
	assert.Empty(t, got.AreaName, "a new area must not keep the old area's name")
}

// A new schedule has nothing to preserve: no area keys means all areas.
func TestScheduleMqttBridge_CreateWithoutAreaKeysIsAllAreas(t *testing.T) {
	db := types.NewMockDBProvider()
	client := newFakeMqttClient()
	newScheduleMqttBridgeWithClient(db, "mowgli", client)

	client.fire(t, "mowgli/schedules/set", []byte(`{"time":"06:00","daysOfWeek":[2],"enabled":true}`))

	all, err := getAllSchedules(db)
	require.NoError(t, err)
	require.Len(t, all, 1)
	assert.Equal(t, uint32(0), all[0].AreaID)
}
