package api

import (
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"strings"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

type Schedule struct {
	ID string `json:"id"`
	// AreaID is the STABLE map area id (MapArea.id) this schedule mows; 0 means
	// every area (a plain Start). The scheduler resolves it to the current
	// positional index when the schedule fires. AreaName is a display snapshot
	// so the GUI and MQTT consumers can label it, even if the area was removed.
	AreaID     uint32     `json:"areaId"`
	AreaName   string     `json:"areaName,omitempty"`
	Time       string     `json:"time"`       // HH:mm format
	DaysOfWeek []int      `json:"daysOfWeek"` // 0=Sunday .. 6=Saturday
	Enabled    bool       `json:"enabled"`
	CreatedAt  time.Time  `json:"createdAt"`
	LastRun    *time.Time `json:"lastRun,omitempty"`
	// Written by the scheduler when a due run was skipped (soil wet); the GUI
	// shows them, the API only preserves them across updates.
	LastSkipReason string     `json:"lastSkipReason,omitempty"`
	LastSkippedAt  *time.Time `json:"lastSkippedAt,omitempty"`
}

type ScheduleListResponse struct {
	Schedules []Schedule `json:"schedules"`
}

const scheduleKeyPrefix = "schedule:"

// minScheduleSpacingMinutes is how far apart two enabled schedules must start
// on a shared weekday. A mow has no fixed duration, so this matches the one-hour
// block the weekly overview draws for every schedule.
const minScheduleSpacingMinutes = 60

const minutesPerWeek = 7 * 24 * 60

const maxAreaNameLength = 200

// scheduleWriteMu serialises check-then-write so two concurrent saves (HTTP
// and MQTT) cannot both pass the overlap check against the same stored state.
var scheduleWriteMu sync.Mutex

// ScheduleConflictError reports that saving would make two enabled schedules overlap.
type ScheduleConflictError struct {
	With Schedule
}

func (e *ScheduleConflictError) Error() string {
	return fmt.Sprintf("overlaps the enabled schedule starting at %s: schedules sharing a weekday must start at least %d minutes apart",
		e.With.Time, minScheduleSpacingMinutes)
}

func ScheduleRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	group := r.Group("/schedules")
	group.GET("", listSchedules(dbProvider))
	group.POST("", createSchedule(dbProvider))
	group.PUT("/:id", updateSchedule(dbProvider))
	group.DELETE("/:id", deleteSchedule(dbProvider))
}

// listSchedules returns all schedules
//
// @Summary list all schedules
// @Description list all mowing schedules
// @Tags schedules
// @Produce json
// @Success 200 {object} ScheduleListResponse
// @Router /schedules [get]
func listSchedules(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		schedules, err := getAllSchedules(dbProvider)
		if err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(http.StatusOK, ScheduleListResponse{Schedules: schedules})
	}
}

// createSchedule creates a new schedule
//
// @Summary create a schedule
// @Description create a new mowing schedule
// @Tags schedules
// @Accept json
// @Produce json
// @Param schedule body Schedule true "schedule"
// @Success 200 {object} Schedule
// @Failure 400 {object} ErrorResponse
// @Failure 409 {object} ErrorResponse
// @Router /schedules [post]
func createSchedule(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		var sched Schedule
		if err := c.BindJSON(&sched); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		if err := validateSchedule(&sched); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		sched.ID = fmt.Sprintf("%d", time.Now().UnixNano())
		sched.CreatedAt = time.Now()

		if err := saveScheduleChecked(dbProvider, &sched); err != nil {
			writeScheduleSaveError(c, err)
			return
		}
		notifyScheduleChanged()

		c.JSON(http.StatusOK, sched)
	}
}

// updateSchedule updates an existing schedule
//
// @Summary update a schedule
// @Description update an existing mowing schedule
// @Tags schedules
// @Accept json
// @Produce json
// @Param id path string true "schedule ID"
// @Param schedule body Schedule true "schedule"
// @Success 200 {object} Schedule
// @Failure 400 {object} ErrorResponse
// @Failure 404 {object} ErrorResponse
// @Failure 409 {object} ErrorResponse
// @Router /schedules/{id} [put]
func updateSchedule(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		id := c.Param("id")

		// Verify schedule exists
		existing, err := getSchedule(dbProvider, id)
		if err != nil {
			c.JSON(http.StatusNotFound, ErrorResponse{Error: "schedule not found"})
			return
		}

		var sched Schedule
		if err := c.BindJSON(&sched); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		if err := validateSchedule(&sched); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		sched.ID = id
		sched.CreatedAt = existing.CreatedAt
		sched.LastRun = existing.LastRun
		sched.LastSkipReason = existing.LastSkipReason
		sched.LastSkippedAt = existing.LastSkippedAt

		if err := saveScheduleChecked(dbProvider, &sched); err != nil {
			writeScheduleSaveError(c, err)
			return
		}
		notifyScheduleChanged()

		c.JSON(http.StatusOK, sched)
	}
}

// deleteSchedule deletes a schedule
//
// @Summary delete a schedule
// @Description delete a mowing schedule
// @Tags schedules
// @Produce json
// @Param id path string true "schedule ID"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /schedules/{id} [delete]
func deleteSchedule(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		id := c.Param("id")
		if err := dbProvider.Delete(scheduleKeyPrefix + id); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}
		notifyScheduleChanged()
		c.JSON(http.StatusOK, OkResponse{})
	}
}

func validateSchedule(s *Schedule) error {
	if s.Time == "" {
		return fmt.Errorf("time is required (HH:mm format)")
	}
	_, err := time.Parse("15:04", s.Time)
	if err != nil {
		return fmt.Errorf("invalid time format, expected HH:mm")
	}
	if len(s.DaysOfWeek) == 0 {
		return fmt.Errorf("at least one day of week is required")
	}
	for _, d := range s.DaysOfWeek {
		if d < 0 || d > 6 {
			return fmt.Errorf("day of week must be 0-6 (Sunday-Saturday)")
		}
	}
	s.AreaName = strings.TrimSpace(s.AreaName)
	if len(s.AreaName) > maxAreaNameLength {
		return fmt.Errorf("areaName is too long (max %d characters)", maxAreaNameLength)
	}
	if s.AreaID == 0 {
		// "All areas" carries no name; a stale one would label it wrongly.
		s.AreaName = ""
	}
	return nil
}

// writeScheduleSaveError maps a save failure to an HTTP response: an overlap is
// the caller's conflict (409), anything else is a storage failure.
func writeScheduleSaveError(c *gin.Context, err error) {
	var conflict *ScheduleConflictError
	if errors.As(err, &conflict) {
		c.JSON(http.StatusConflict, ErrorResponse{Error: err.Error()})
		return
	}
	c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
}

// minuteOfWeek is the Sunday 00:00-based minute of a weekday + HH:mm start.
func minuteOfWeek(day int, hhmm string) (int, bool) {
	t, err := time.Parse("15:04", hhmm)
	if err != nil {
		return 0, false
	}
	return day*24*60 + t.Hour()*60 + t.Minute(), true
}

// schedulesOverlap reports whether any start of a is closer than
// minScheduleSpacingMinutes to any start of b. Distance wraps around the week,
// so Sunday 23:30 and Monday 00:15 are 45 minutes apart, not six days.
func schedulesOverlap(a, b *Schedule) bool {
	for _, da := range a.DaysOfWeek {
		ma, okA := minuteOfWeek(da, a.Time)
		if !okA {
			continue
		}
		for _, db := range b.DaysOfWeek {
			mb, okB := minuteOfWeek(db, b.Time)
			if !okB {
				continue
			}
			diff := ma - mb
			if diff < 0 {
				diff = -diff
			}
			if diff > minutesPerWeek/2 {
				diff = minutesPerWeek - diff
			}
			if diff < minScheduleSpacingMinutes {
				return true
			}
		}
	}
	return false
}

// findOverlappingSchedule returns the first OTHER enabled schedule that overlaps
// the candidate. A disabled candidate never conflicts (it cannot fire), so
// disabling is always allowed; the check runs when a schedule is enabled or
// its time or days change while enabled.
func findOverlappingSchedule(candidate *Schedule, existing []Schedule) *Schedule {
	if !candidate.Enabled {
		return nil
	}
	for i := range existing {
		other := &existing[i]
		if other.ID == candidate.ID || !other.Enabled {
			continue
		}
		if schedulesOverlap(candidate, other) {
			return other
		}
	}
	return nil
}

// saveScheduleChecked stores s unless it would overlap another enabled
// schedule. HTTP and MQTT both save through it, so the rule holds everywhere.
func saveScheduleChecked(dbProvider types.IDBProvider, s *Schedule) error {
	scheduleWriteMu.Lock()
	defer scheduleWriteMu.Unlock()
	existing, err := getAllSchedules(dbProvider)
	if err != nil {
		return err
	}
	if other := findOverlappingSchedule(s, existing); other != nil {
		return &ScheduleConflictError{With: *other}
	}
	return saveSchedule(dbProvider, s)
}

func saveSchedule(dbProvider types.IDBProvider, s *Schedule) error {
	data, err := json.Marshal(s)
	if err != nil {
		return err
	}
	return dbProvider.Set(scheduleKeyPrefix+s.ID, data)
}

func getSchedule(dbProvider types.IDBProvider, id string) (*Schedule, error) {
	data, err := dbProvider.Get(scheduleKeyPrefix + id)
	if err != nil {
		return nil, err
	}
	var sched Schedule
	if err := json.Unmarshal(data, &sched); err != nil {
		return nil, err
	}
	return &sched, nil
}

func getAllSchedules(dbProvider types.IDBProvider) ([]Schedule, error) {
	keys, err := dbProvider.KeysWithSuffix(scheduleKeyPrefix)
	if err != nil {
		return nil, err
	}
	schedules := make([]Schedule, 0, len(keys))
	for _, key := range keys {
		data, err := dbProvider.Get(key)
		if err != nil {
			continue
		}
		var sched Schedule
		if err := json.Unmarshal(data, &sched); err != nil {
			continue
		}
		schedules = append(schedules, sched)
	}
	return schedules, nil
}
