package providers

import (
	"context"
	"encoding/json"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/mowglinext/mowglinext/pkg/msgs/std"
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

	mu                      sync.RWMutex
	checkMu                 sync.Mutex
	lastHighLevelState      uint8
	lastHighLevelStateName  string
	hasHighLevelStatus      bool
	lastEmergency           bool
	highLevelEmergency      bool
	coverageSessionActive   bool
	coverageSessionKnown    bool
	coverageResumeAvailable bool
	coverageResumeKnown     bool
	statusUpdates           chan struct{}
}

// NewSchedulerProvider creates and starts the scheduler background goroutine.
// soilProvider may be nil, in which case no soil gate is applied.
func NewSchedulerProvider(rosProvider types.IRosProvider, dbProvider types.IDBProvider, soilProvider types.ISoilProvider) *SchedulerProvider {
	s := &SchedulerProvider{
		rosProvider:   rosProvider,
		dbProvider:    dbProvider,
		soilProvider:  soilProvider,
		statusUpdates: make(chan struct{}, 1),
	}
	s.subscribeToStatus()
	go s.run()
	return s
}

// subscribeToStatus subscribes to highLevelStatus and emergency so that the
// scheduler can perform pre-flight safety checks without an extra service call.
func (s *SchedulerProvider) subscribeToStatus() {
	// highLevelStatus — tracks robot operational state and its co-reported
	// emergency snapshot. Keep it separate from the hardware emergency topic so
	// an older HLS snapshot can never clear a newer hardware emergency.
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
			// Accept "state_name" or "StateName"
			for _, k := range []string{"state_name", "StateName"} {
				if v, ok := raw[k]; ok {
					_ = json.Unmarshal(v, &hls.StateName)
					break
				}
			}
		}
		s.mu.Lock()
		s.lastHighLevelState = hls.State
		s.lastHighLevelStateName = hls.StateName
		s.highLevelEmergency = hls.Emergency
		s.hasHighLevelStatus = true
		s.mu.Unlock()
		s.wakeStartupRetry()
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
		s.wakeStartupRetry()
	}); err != nil {
		logrus.Warnf("Scheduler: failed to subscribe to emergency: %v", err)
	}

	// coverageSession identifies a live COMMAND_START session even while its
	// high-level state is IDLE (for example, a mid-session recharge hold).
	if err := s.rosProvider.Subscribe("coverageSession", "scheduler-session", 0, func(msg []byte) {
		var session mowgli.CoverageSession
		if err := json.Unmarshal(msg, &session); err != nil {
			return
		}
		s.mu.Lock()
		s.coverageSessionActive = session.SessionActive
		s.coverageSessionKnown = true
		s.mu.Unlock()
		s.wakeStartupRetry()
	}); err != nil {
		logrus.Warnf("Scheduler: failed to subscribe to coverageSession: %v", err)
	}

	// coverageResumeAvailable is latched by the behavior tree. It is the
	// session identity needed to distinguish a fresh IDLE status from a paused
	// mow that COMMAND_START would resume.
	if err := s.rosProvider.Subscribe("coverageResumeAvailable", "scheduler-resume", 0, func(msg []byte) {
		var available std.Bool
		if err := json.Unmarshal(msg, &available); err != nil {
			return
		}
		s.mu.Lock()
		s.coverageResumeAvailable = available.Data
		s.coverageResumeKnown = true
		s.mu.Unlock()
		s.wakeStartupRetry()
	}); err != nil {
		logrus.Warnf("Scheduler: failed to subscribe to coverageResumeAvailable: %v", err)
	}
}

// wakeStartupRetry rechecks the bounded startup minute when any input to
// safeToStart changes. The retained high-level status and coverage provenance
// can arrive in either order, so waking only for status could leave an on-time
// schedule blocked after its final required input becomes known.
func (s *SchedulerProvider) wakeStartupRetry() {
	select {
	case s.statusUpdates <- struct{}{}:
	default:
	}
}

func (s *SchedulerProvider) run() {
	startedAt := time.Now()
	ticker := time.NewTicker(1 * time.Minute)
	defer ticker.Stop()

	// Do not wait a full minute after startup: schedules are evaluated for the
	// current minute immediately. safeToStart keeps this fail-closed until the
	// first high-level status is received.
	s.checkSchedulesAt(startedAt)
	for {
		select {
		case <-ticker.C:
			s.checkSchedules()
		case <-s.statusUpdates:
			s.checkStartupUpdateAt(startedAt, time.Now())
		}
	}
}

func (s *SchedulerProvider) checkSchedules() {
	s.checkSchedulesAt(time.Now())
}

// checkSchedulesAt evaluates schedules at now. Keeping the clock explicit
// makes the minute-boundary policy deterministic to test.
func (s *SchedulerProvider) checkSchedulesAt(now time.Time) {
	s.checkMu.Lock()
	defer s.checkMu.Unlock()
	s.checkSchedulesAtLocked(now)
}

// checkStartupUpdateAt retries a startup check only while it remains in the
// calendar minute in which the backend started. This lets delayed admission
// inputs unlock an on-time run without backfilling missed schedules.
func (s *SchedulerProvider) checkStartupUpdateAt(startedAt, now time.Time) {
	if startedAt.Format("2006-01-02 15:04") != now.Format("2006-01-02 15:04") {
		return
	}
	s.checkSchedulesAt(now)
}

func (s *SchedulerProvider) checkSchedulesAtLocked(now time.Time) {
	keys, err := s.dbProvider.KeysWithSuffix(schedulerKeyPrefix)
	if err != nil {
		logrus.Warnf("Scheduler: failed to list schedules: %v", err)
		return
	}

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
		var res mowgli.HighLevelControlRes
		err = s.rosProvider.CallService(
			ctx,
			"/behavior_tree_node/high_level_control",
			&mowgli.HighLevelControlReq{Command: 1}, // 1 = COMMAND_START
			&res,
		)
		cancel()

		if err != nil {
			logrus.Errorf("Scheduler: failed to call high_level_control for schedule %s: %v", sched.ID, err)
			continue
		}
		// A delivered call is not an accepted one: behavior_tree_node answers
		// success=false while it refuses START (update maintenance). Nothing
		// ran, so LastRun stays untouched and the next tick may retry (#702).
		if !res.Success {
			logrus.Warnf("Scheduler: high_level_control rejected START for schedule %s", sched.ID)
			continue
		}

		// Persist last-run time so double-execution within the same minute is prevented.
		// LastRun means "START was accepted", not "the mow completed".
		s.updateRunMetadata(sched.ID, func(current *schedule) {
			current.LastRun = &now
		})
	}
}

// updateRunMetadata applies mutate to the schedule AS STORED NOW, never to
// the snapshot read at the top of checkSchedules: the START call in between
// can take up to 30 s, and writing the snapshot back resurrected a schedule
// deleted meanwhile and reverted an edit or a disable (#702). A record that
// is gone stays gone. mutate must only touch execution metadata.
func (s *SchedulerProvider) updateRunMetadata(id string, mutate func(current *schedule)) {
	key := schedulerKeyPrefix + id
	data, err := s.dbProvider.Get(key)
	if err != nil {
		logrus.Infof("Scheduler: schedule %s no longer exists, not recording run metadata", id)
		return
	}
	var current schedule
	if err := json.Unmarshal(data, &current); err != nil {
		logrus.Warnf("Scheduler: failed to parse schedule %s: %v", id, err)
		return
	}
	mutate(&current)
	updated, err := json.Marshal(&current)
	if err != nil {
		logrus.Warnf("Scheduler: failed to encode schedule %s: %v", id, err)
		return
	}
	if err := s.dbProvider.Set(key, updated); err != nil {
		logrus.Warnf("Scheduler: failed to persist run metadata for schedule %s: %v", id, err)
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
	s.updateRunMetadata(sched.ID, func(current *schedule) {
		current.LastSkipReason = reason
		current.LastSkippedAt = &now
	})
}

// safeToStart returns true when it is safe to send COMMAND_START.
// It admits starts only from known, non-resumable idle status and coverage
// provenance. A charge hold belongs to a live session only when
// CoverageSession.session_active is true; only then would COMMAND_START be an
// operator manual-resume request.
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
	stateName := s.lastHighLevelStateName
	hasHighLevelStatus := s.hasHighLevelStatus
	emergency := s.lastEmergency || s.highLevelEmergency
	sessionActive := s.coverageSessionActive
	sessionKnown := s.coverageSessionKnown
	resumeAvailable := s.coverageResumeAvailable
	resumeKnown := s.coverageResumeKnown
	s.mu.RUnlock()

	if !hasHighLevelStatus || emergency || !sessionKnown || !resumeKnown {
		return false
	}
	if state != 1 || sessionActive || resumeAvailable {
		return false
	}
	switch stateName {
	case "IDLE", "IDLE_DOCKED", "CHARGING":
		return true
	case "CRITICAL_BATTERY_CHARGING", "RAIN_WAITING":
		return false
	default:
		return false
	}
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
