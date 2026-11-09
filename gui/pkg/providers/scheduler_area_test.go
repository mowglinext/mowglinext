package providers

import (
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// areaResponder answers the scheduler's service calls: get_mowing_area walks the
// given list (success=false past the end) and start_in_area / high_level_control
// accept.
func areaResponder(areas []mowgli.MapArea) func(string, any, any) {
	return func(service string, req any, res any) {
		switch service {
		case "/map_server_node/get_mowing_area":
			r := req.(*mowgli.GetMowingAreaReq)
			out := res.(*mowgli.GetMowingAreaRes)
			if int(r.Index) < len(areas) {
				out.Success = true
				out.Area = areas[r.Index]
			}
		case "/behavior_tree_node/start_in_area":
			res.(*mowgli.StartInAreaRes).Success = true
		case "/behavior_tree_node/high_level_control":
			res.(*mowgli.HighLevelControlRes).Success = true
		}
	}
}

func dueAreaSchedule(id string, areaID uint32, now time.Time) schedule {
	s := dueSchedule(id, now)
	s.AreaID = areaID
	return s
}

func servicesCalled(ros *types.MockRosProvider) []string {
	var names []string
	for _, c := range ros.ServiceCalls {
		names = append(names, c.Service)
	}
	return names
}

func TestCheckSchedules_AllAreasStaysAPlainStart(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = areaResponder(nil)
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueAreaSchedule("all", 0, now))

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	assert.Equal(t, []string{"/behavior_tree_node/high_level_control"}, servicesCalled(ros),
		"areaId 0 must not look anything up and must send COMMAND_START")
	assert.NotNil(t, readSchedule(t, db, "all").LastRun)
}

func TestCheckSchedules_OneAreaStartsThatAreaByItsCurrentIndex(t *testing.T) {
	ros := types.NewMockRosProvider()
	// Area id 42 currently sits at index 2; ids 11 and 7 are in front of it.
	ros.ServiceResponder = areaResponder([]mowgli.MapArea{{Id: 11}, {Id: 7}, {Id: 42}, {Id: 9}})
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueAreaSchedule("one", 42, now))

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	var start *mowgli.StartInAreaReq
	for _, c := range ros.ServiceCalls {
		if c.Service == "/behavior_tree_node/start_in_area" {
			start = c.Req.(*mowgli.StartInAreaReq)
		}
		assert.NotEqual(t, "/behavior_tree_node/high_level_control", c.Service,
			"a single-area schedule must not also send a plain COMMAND_START")
	}
	require.NotNil(t, start)
	assert.Equal(t, uint8(2), start.Area, "the id is resolved to the index it has NOW")
	assert.NotNil(t, readSchedule(t, db, "one").LastRun)
}

func TestCheckSchedules_RemovedAreaSkipsTheRunAndSaysWhy(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = areaResponder([]mowgli.MapArea{{Id: 11}, {Id: 7}})
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueAreaSchedule("gone", 42, now))

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	for _, name := range servicesCalled(ros) {
		assert.NotContains(t, name, "behavior_tree_node", "nothing may be started for a vanished area")
	}
	got := readSchedule(t, db, "gone")
	assert.Nil(t, got.LastRun)
	assert.Contains(t, got.LastSkipReason, "no longer exists")
	assert.NotNil(t, got.LastSkippedAt)
}

func TestCheckSchedules_NavigationAreaIsNeverMowed(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceResponder = areaResponder([]mowgli.MapArea{{Id: 42, IsNavigationArea: true}})
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueAreaSchedule("nav", 42, now))

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	for _, name := range servicesCalled(ros) {
		assert.NotContains(t, name, "behavior_tree_node")
	}
	assert.Contains(t, readSchedule(t, db, "nav").LastSkipReason, "navigation area")
}

func TestCheckSchedules_AreaLookupFailureSkipsInsteadOfMowingEverything(t *testing.T) {
	ros := types.NewMockRosProvider()
	ros.ServiceErr = assert.AnError
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueAreaSchedule("lookup", 42, now))

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	for _, name := range servicesCalled(ros) {
		assert.NotEqual(t, "/behavior_tree_node/high_level_control", name,
			"a failed lookup must never fall back to mowing every area")
	}
	got := readSchedule(t, db, "lookup")
	assert.Nil(t, got.LastRun)
	assert.Contains(t, got.LastSkipReason, "could not look up")
}

func TestCheckSchedules_RejectedAreaStartDoesNotPersistLastRun(t *testing.T) {
	ros := types.NewMockRosProvider()
	inner := areaResponder([]mowgli.MapArea{{Id: 42}})
	ros.ServiceResponder = func(service string, req any, res any) {
		inner(service, req, res)
		if service == "/behavior_tree_node/start_in_area" {
			res.(*mowgli.StartInAreaRes).Success = false // update maintenance
		}
	}
	db := types.NewMockDBProvider()
	now := time.Now()
	storeSchedule(t, db, dueAreaSchedule("rejected", 42, now))

	s := buildScheduler(ros, db)
	s.lastHighLevelState = 1
	s.checkSchedules()

	assert.Nil(t, readSchedule(t, db, "rejected").LastRun, "a rejected start is not a run")
}

func TestDescribeScheduleClock_NamesTheZoneAndTheLocalTime(t *testing.T) {
	cest := time.FixedZone("CEST", 2*60*60)
	now := time.Date(2026, 10, 6, 8, 15, 0, 0, cest)

	got := describeScheduleClock(now)

	assert.Contains(t, got, "CEST", "the zone must be visible, a UTC container is the failure to spot")
	assert.Contains(t, got, "08:15")
}
