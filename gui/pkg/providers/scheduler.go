package providers

import (
	"context"
	"encoding/json"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// schedule mirrors api.Schedule exactly. It is redefined here to avoid an
// import cycle (providers ← api). Keep fields in sync with api.Schedule.
type schedule struct {
	ID         string     `json:"id"`
	Area       int        `json:"area"`
	Time       string     `json:"time"`
	DaysOfWeek []int      `json:"daysOfWeek"`
	Enabled    bool       `json:"enabled"`
	CreatedAt  time.Time  `json:"createdAt"`
	LastRun    *time.Time `json:"lastRun,omitempty"`
	// LastSkipReason / LastSkippedAt record the most recent time a due run was
	// deliberately NOT started (soil wet), so the GUI can say why.
	LastSkipReason string     `json:"lastSkipReason,omitempty"`
	LastSkippedAt  *time.Time `json:"lastSkippedAt,omitempty"`
}

const schedulerKeyPrefix = "schedule:"

// SchedulerProvider polls the database every minute and triggers autonomous
// mowing via the high_level_control ROS2 service when a schedule fires.
// Before triggering it checks that no emergency is active and the robot is
// not already in autonomous or recording state, then asks the soil provider
// (IrriSense) whether the grass is wet.
type SchedulerProvider struct {
	rosProvider  types.IRosProvider
	dbProvider   types.IDBProvider
	soilProvider types.ISoilProvider

	mu                 sync.RWMutex
	lastHighLevelState uint8
	lastEmergency      bool
}

// NewSchedulerProvider creates and starts the scheduler background goroutine.
// soilProvider may be nil, in which case no soil gate is applied.
func NewSchedulerProvider(rosProvider types.IRosProvider, dbProvider types.IDBProvider, soilProvider types.ISoilProvider) *SchedulerProvider {
	s := &SchedulerProvider{
		rosProvider:  rosProvider,
		dbProvider:   dbProvider,
		soilProvider: soilProvider,
	}
	s.subscribeToStatus()
	go s.run()
	return s
}

// subscribeToStatus subscribes to highLevelStatus and emergency so that the
// scheduler can perform pre-flight safety checks without an extra service call.
func (s *SchedulerProvider) subscribeToStatus() {
	// highLevelStatus — tracks robot operational state (idle/autonomous/recording)
	if err := s.rosProvider.Subscribe("highLevelStatus", "scheduler-hls", 0, func(msg []byte) {
		var hls mowgli.HighLevelStatus
		if err := json.Unmarshal(msg, &hls); err != nil {
			// Fallback: try to find the state field by either name variant.
			var raw map[string]json.RawMessage
			if jsonErr := json.Unmarshal(msg, &raw); jsonErr != nil {
				return
			}
			// Accept "state" or "State"
			for _, k := range []string{"state", "State"} {
				if v, ok := raw[k]; ok {
					_ = json.Unmarshal(v, &hls.State)
					break
				}
			}
		}
		s.mu.Lock()
		s.lastHighLevelState = hls.State
		s.mu.Unlock()
	}); err != nil {
		logrus.Warnf("Scheduler: failed to subscribe to highLevelStatus: %v", err)
	}

	// emergency — safety-critical, no throttle on this topic
	if err := s.rosProvider.Subscribe("emergency", "scheduler-emg", 0, func(msg []byte) {
		var emg mowgli.Emergency
		if err := json.Unmarshal(msg, &emg); err != nil {
			var raw map[string]json.RawMessage
			if jsonErr := json.Unmarshal(msg, &raw); jsonErr != nil {
				return
			}
			for _, k := range []string{"active_emergency", "ActiveEmergency"} {
				if v, ok := raw[k]; ok {
					_ = json.Unmarshal(v, &emg.ActiveEmergency)
					break
				}
			}
		}
		s.mu.Lock()
		s.lastEmergency = emg.ActiveEmergency
		s.mu.Unlock()
	}); err != nil {
		logrus.Warnf("Scheduler: failed to subscribe to emergency: %v", err)
	}
}

func (s *SchedulerProvider) run() {
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()
	for range ticker.C {
		s.checkSchedules()
	}
}

func (s *SchedulerProvider) checkSchedules() {
	keys, err := s.dbProvider.KeysWithSuffix(schedulerKeyPrefix)
	if err != nil {
		logrus.Warnf("Scheduler: failed to list schedules: %v", err)
		return
	}

	now := time.Now()
	currentDay := int(now.Weekday())
	currentTime := now.Format("15:04")

	for _, key := range keys {
		data, err := s.dbProvider.Get(key)
		if err != nil {
			logrus.Warnf("Scheduler: failed to read schedule %s: %v", key, err)
			continue
		}

		var sched schedule
		if err := json.Unmarshal(data, &sched); err != nil {
			logrus.Warnf("Scheduler: failed to parse schedule %s: %v", key, err)
			continue
		}

		if !sched.Enabled {
			continue
		}

		if !s.shouldRun(&sched, currentDay, currentTime, now) {
			continue
		}

		if !s.safeToStart() {
			logrus.Infof("Scheduler: skipping schedule %s — safety check failed (emergency or already active)", sched.ID)
			continue
		}

		if blocked, reason := s.soilBlocksStart(); blocked {
			logrus.Infof("Scheduler: skipping schedule %s — soil wet (%s)", sched.ID, reason)
			s.persistSkip(sched, reason, now)
			continue
		}

		logrus.Infof("Scheduler: triggering autonomous mowing for schedule %s (area %d)", sched.ID, sched.Area)

		ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
		err = s.rosProvider.CallService(
			ctx,
			"/behavior_tree_node/high_level_control",
			&mowgli.HighLevelControlReq{Command: 1}, // 1 = COMMAND_START
			&mowgli.HighLevelControlRes{},
		)
		cancel()

		if err != nil {
			logrus.Errorf("Scheduler: failed to call high_level_control for schedule %s: %v", sched.ID, err)
			continue
		}

		// Persist last-run time so double-execution within the same minute is prevented.
		sched.LastRun = &now
		if updated, err := json.Marshal(&sched); err == nil {
			if putErr := s.dbProvider.Set(schedulerKeyPrefix+sched.ID, updated); putErr != nil {
				logrus.Warnf("Scheduler: failed to persist last-run for schedule %s: %v", sched.ID, putErr)
			}
		}
	}
}

// soilBlocksStart asks the soil provider whether the grass is wet. Fail-open:
// no provider, disabled integration, gate switched off, stale or unknown data
// all return false — only a FRESH, positive "wet" verdict skips a run.
func (s *SchedulerProvider) soilBlocksStart() (bool, string) {
	if s.soilProvider == nil {
		return false, ""
	}
	status := s.soilProvider.SoilStatus()
	if !status.BlocksScheduledMowing() {
		return false, ""
	}
	return true, status.Reason
}

// persistSkip records why a due run was not started, so the GUI can show it.
func (s *SchedulerProvider) persistSkip(sched schedule, reason string, now time.Time) {
	sched.LastSkipReason = reason
	sched.LastSkippedAt = &now
	updated, err := json.Marshal(&sched)
	if err != nil {
		logrus.Warnf("Scheduler: failed to encode skip for schedule %s: %v", sched.ID, err)
		return
	}
	if err := s.dbProvider.Set(schedulerKeyPrefix+sched.ID, updated); err != nil {
		logrus.Warnf("Scheduler: failed to persist skip for schedule %s: %v", sched.ID, err)
	}
}

// safeToStart returns true when it is safe to send COMMAND_START.
// It blocks mowing when:
//   - an emergency is active (latched or active), or
//   - the robot is already in autonomous (2) or recording (3) state.
//
// HIGH_LEVEL_STATE constants:
//
//	0 = NULL (emergency/transitional)
//	1 = IDLE
//	2 = AUTONOMOUS
//	3 = RECORDING
//	4 = MANUAL_MOWING
func (s *SchedulerProvider) safeToStart() bool {
	s.mu.RLock()
	state := s.lastHighLevelState
	emergency := s.lastEmergency
	s.mu.RUnlock()

	if emergency {
		return false
	}
	// Do not interrupt an already-running autonomous session or an ongoing
	// area recording. State 0 (NULL/emergency) is also blocked.
	switch state {
	case 0, 2, 3:
		return false
	}
	return true
}

func (s *SchedulerProvider) shouldRun(sched *schedule, currentDay int, currentTime string, now time.Time) bool {
	if sched.Time != currentTime {
		return false
	}

	dayMatch := false
	for _, d := range sched.DaysOfWeek {
		if d == currentDay {
			dayMatch = true
			break
		}
	}
	if !dayMatch {
		return false
	}

	// Prevent double-execution within the same minute window.
	if sched.LastRun != nil && now.Sub(*sched.LastRun) < 2*time.Minute {
		return false
	}

	return true
}
