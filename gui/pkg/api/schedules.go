package api

import (
	"encoding/json"
	"fmt"
	"net/http"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
)

type Schedule struct {
	ID         string     `json:"id"`
	Area       int        `json:"area"`
	Time       string     `json:"time"`       // HH:mm format
	DaysOfWeek []int      `json:"daysOfWeek"` // 0=Sunday .. 6=Saturday
	Enabled    bool       `json:"enabled"`
	CreatedAt  time.Time  `json:"createdAt"`
	LastRun    *time.Time `json:"lastRun,omitempty"`
}

type ScheduleListResponse struct {
	Schedules []Schedule `json:"schedules"`
}

const scheduleKeyPrefix = "schedule:"

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

		if err := saveSchedule(dbProvider, &sched); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

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

		if err := saveSchedule(dbProvider, &sched); err != nil {
			c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
			return
		}

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
	return nil
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
