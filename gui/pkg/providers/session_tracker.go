package providers

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// MowingSessionRecord mirrors the API type for DB storage.
type MowingSessionRecord struct {
	ID              string   `json:"id"`
	StartTime       string   `json:"start_time"`
	EndTime         string   `json:"end_time"`
	DurationSec     float64  `json:"duration_sec"`
	AreaIndex       int      `json:"area_index"`
	CoveragePercent float32  `json:"coverage_percent"`
	StripsCompleted uint32   `json:"strips_completed"`
	StripsSkipped   uint32   `json:"strips_skipped"`
	DistanceMeters  float64  `json:"distance_meters"`
	Status          string   `json:"status"`
	RechargePauses  int      `json:"recharge_pauses"`
	Errors          []string `json:"errors"`
}

// SessionTracker monitors the BT high-level status and automatically
// records mowing sessions in the database.
type SessionTracker struct {
	dbProvider   types.IDBProvider
	mu           sync.Mutex
	currentState string
	sessionStart time.Time
	inSession    bool
	// paused is set while the robot has docked to recharge mid-session and the
	// BT is expected to auto-resume mowing once topped up. A paused session is
	// kept OPEN (not finalized) so a recharge no longer produces a spurious
	// "aborted" record. pauseCount tallies the recharge cycles for the record.
	paused     bool
	pauseCount int
	// Live coverage/strip counters captured from the status stream while mowing.
	// The BT publishes these per AUTONOMOUS tick (coverage_percent is the smooth
	// 0..100 progress; completed/skipped_swaths are the coarse swath tallies).
	// They are snapshotted into the finalized record so the stats page shows real
	// coverage instead of a hardcoded 0. Peak (max) is kept rather than the last
	// value so a per-area reset near the end of a mow does not clobber the result.
	coveragePeak    float32
	stripsCompleted uint32
	stripsSkipped   uint32
	// Odometer: total ground distance travelled during the session, integrated
	// from consecutive /wheel_odom positions. Surfaced as the fleet-wide "km
	// driven" figure (blade-wear proxy) on the stats page. lastOdom* holds the
	// previous sample; hasOdom guards the very first sample after a start.
	distanceMeters float64
	lastOdomX      float64
	lastOdomY      float64
	hasOdom        bool
	// queue serializes status messages through a single consumer goroutine so
	// the start/pause/resume/end state machine is applied in arrival order.
	// Previously fanOut spawned a goroutine per message (go OnHighLevelStatus),
	// which the scheduler could run out of order — corrupting inSession/paused/
	// pauseCount on rapid MOWING->CHARGING->IDLE bursts.
	queue chan []byte
	// odomQueue carries /wheel_odom samples to a dedicated consumer so the JSON
	// parse happens off the fanOut goroutine (which holds the ROS provider lock).
	// Distance is lossy-tolerant, so a full buffer drops samples rather than
	// blocking; the jump guard absorbs the resulting larger steps.
	odomQueue chan []byte
}

// maxOdomStepM caps the per-sample position delta folded into session distance.
// At ~12 Hz and mowing speed a real step is a few cm; a larger jump means an
// odometry frame reset (node restart) or a dropped burst of samples, not real
// travel, so it is discarded instead of inflating the odometer.
const maxOdomStepM = 1.0

const sessionsDBKey = "mowing.sessions"

// minSessionDurationSec is the floor below which a session is treated as noise
// and NOT recorded. Rapid AUTONOMOUS->IDLE flips at startup or on an immediate
// abort used to spawn hundreds of 0-second "aborted" rows that polluted the
// statistics history. A genuine mow (even a quickly-aborted one) runs longer
// than this; emergencies are still recorded regardless of duration below.
const minSessionDurationSec = 20.0

// NewSessionTracker creates a tracker. Feed status messages via Enqueue().
func NewSessionTracker(dbProvider types.IDBProvider) *SessionTracker {
	s := &SessionTracker{
		dbProvider: dbProvider,
		queue:      make(chan []byte, 256),
		odomQueue:  make(chan []byte, 16),
	}
	go s.run()
	go s.runOdom()
	return s
}

// Enqueue hands a raw status message to the ordered consumer. Non-blocking: if
// the buffer is full (consumer wedged on a slow DB write) the message is
// dropped with a warning rather than stalling the caller's fanOut path.
func (s *SessionTracker) Enqueue(msg []byte) {
	select {
	case s.queue <- msg:
	default:
		log.Printf("SessionTracker: status queue full, dropping message")
	}
}

// run drains the queue in FIFO order on a single goroutine.
func (s *SessionTracker) run() {
	for msg := range s.queue {
		s.OnHighLevelStatus(msg)
	}
}

// EnqueueOdometry hands a raw /wheel_odom message to the odometer consumer.
// Non-blocking: a full buffer drops the sample (distance is lossy-tolerant).
func (s *SessionTracker) EnqueueOdometry(msg []byte) {
	select {
	case s.odomQueue <- msg:
	default:
	}
}

// runOdom drains odometry samples on a dedicated goroutine.
func (s *SessionTracker) runOdom() {
	for msg := range s.odomQueue {
		s.OnOdometry(msg)
	}
}

// OnOdometry folds one /wheel_odom position into the running session distance.
// It only accumulates while a session is open; a jump beyond maxOdomStepM (frame
// reset or dropped burst) re-baselines without counting the gap.
func (s *SessionTracker) OnOdometry(msg []byte) {
	var odom struct {
		Pose struct {
			Pose struct {
				Position struct {
					X float64 `json:"x"`
					Y float64 `json:"y"`
				} `json:"position"`
			} `json:"pose"`
		} `json:"pose"`
	}
	if err := json.Unmarshal(msg, &odom); err != nil {
		return
	}
	x := odom.Pose.Pose.Position.X
	y := odom.Pose.Pose.Position.Y

	s.mu.Lock()
	defer s.mu.Unlock()

	if !s.inSession {
		s.hasOdom = false
		return
	}
	if s.hasOdom {
		if d := math.Hypot(x-s.lastOdomX, y-s.lastOdomY); d <= maxOdomStepM {
			s.distanceMeters += d
		}
	}
	s.lastOdomX = x
	s.lastOdomY = y
	s.hasOdom = true
}

// OnHighLevelStatus processes a raw JSON high-level status message.
// Call this from the fanOut pipeline for the "highLevelStatus" topic.
func (s *SessionTracker) OnHighLevelStatus(msg []byte) {
	var status struct {
		State           int     `json:"state"`
		StateName       string  `json:"state_name"`
		Emergency       bool    `json:"emergency"`
		Battery         float32 `json:"battery_percent"`
		Area            int     `json:"current_area"`
		CoveragePercent float32 `json:"coverage_percent"`
		CompletedSwaths int     `json:"completed_swaths"`
		SkippedSwaths   int     `json:"skipped_swaths"`
	}
	if err := json.Unmarshal(msg, &status); err != nil {
		return
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	prevState := s.currentState
	s.currentState = status.StateName

	isMowing := status.State == 2 // HIGH_LEVEL_STATE_AUTONOMOUS
	wasMowing := prevState == "MOWING" || prevState == "TRANSIT" || prevState == "RECOVERING" || prevState == "RESUMING_AFTER_RAIN" || prevState == "RESUMING_UNDOCKING"

	// Start session
	if isMowing && !s.inSession {
		s.inSession = true
		s.paused = false
		s.pauseCount = 0
		s.coveragePeak = 0
		s.stripsCompleted = 0
		s.stripsSkipped = 0
		s.distanceMeters = 0
		s.hasOdom = false
		s.captureProgress(status.CoveragePercent, status.CompletedSwaths, status.SkippedSwaths)
		s.sessionStart = time.Now().UTC()
		log.Printf("SessionTracker: mowing session started (state=%s)", status.StateName)
		return
	}

	// Accumulate live coverage/strip progress on every mowing tick.
	if isMowing && s.inSession {
		s.captureProgress(status.CoveragePercent, status.CompletedSwaths, status.SkippedSwaths)
	}

	// Resume after a recharge pause: mowing came back on the SAME session.
	if isMowing && s.inSession && s.paused {
		s.paused = false
		log.Printf("SessionTracker: mowing session resumed after recharge")
		return
	}

	// Pause for recharge: the robot docked to charge mid-session (low battery)
	// and the BT auto-resumes once topped up. Keep the session OPEN instead of
	// finalizing it as "aborted" — the recharge is a pause, not an end. A
	// genuine end-of-mow that happens to charge (prevState MOWING_COMPLETE) is
	// excluded so it still finalizes as "completed" below.
	if s.inSession && !isMowing && status.StateName == "CHARGING" && prevState != "MOWING_COMPLETE" {
		if !s.paused {
			s.paused = true
			s.pauseCount++
			log.Printf("SessionTracker: mowing session paused for recharge (pause #%d)", s.pauseCount)
		}
		return
	}

	// End session
	if s.inSession && !isMowing && !wasMowing {
		s.inSession = false
		s.paused = false
		endTime := time.Now().UTC()
		duration := endTime.Sub(s.sessionStart).Seconds()

		// Drop noise: a sub-threshold session that did not trip an emergency is a
		// spurious state flip, not a real mow. Skip it so it never reaches the DB.
		if duration < minSessionDurationSec && !status.Emergency {
			log.Printf("SessionTracker: dropping spurious %.0fs session (state=%s)", duration, status.StateName)
			return
		}

		// Determine status
		sessionStatus := "completed"
		switch status.StateName {
		case "IDLE_DOCKED", "MOWING_COMPLETE":
			sessionStatus = "completed"
		case "COVERAGE_FAILED_DOCKING", "NAV_TO_DOCK_FAILED":
			sessionStatus = "error"
		default:
			if status.Emergency {
				sessionStatus = "error"
			} else {
				sessionStatus = "aborted"
			}
		}

		session := MowingSessionRecord{
			ID:              fmt.Sprintf("%d", s.sessionStart.UnixMilli()),
			StartTime:       s.sessionStart.Format(time.RFC3339),
			EndTime:         endTime.Format(time.RFC3339),
			DurationSec:     duration,
			AreaIndex:       status.Area,
			CoveragePercent: s.coveragePeak,
			StripsCompleted: s.stripsCompleted,
			StripsSkipped:   s.stripsSkipped,
			DistanceMeters:  math.Round(s.distanceMeters*10) / 10,
			Status:          sessionStatus,
			RechargePauses:  s.pauseCount,
			Errors:          []string{},
		}

		if status.Emergency {
			session.Errors = append(session.Errors, "Emergency stop triggered")
		}

		s.saveSession(session)
		log.Printf("SessionTracker: session ended (status=%s, duration=%.0fs)", sessionStatus, duration)
	}
}

// captureProgress folds a live status tick into the session's running coverage
// and swath tallies. Coverage keeps its peak (it is monotonic within an area but
// resets to 0 when a new area starts, so the peak is the meaningful figure).
// Swath counts also keep their peak, guarding against the transient -1/0 the BT
// emits between areas. Caller must hold s.mu.
func (s *SessionTracker) captureProgress(coverage float32, completed, skipped int) {
	if coverage > s.coveragePeak {
		s.coveragePeak = coverage
	}
	if completed > 0 && uint32(completed) > s.stripsCompleted {
		s.stripsCompleted = uint32(completed)
	}
	if skipped > 0 && uint32(skipped) > s.stripsSkipped {
		s.stripsSkipped = uint32(skipped)
	}
}

func (s *SessionTracker) saveSession(session MowingSessionRecord) {
	// Load existing sessions
	sessions := []MowingSessionRecord{}
	if data, err := s.dbProvider.Get(sessionsDBKey); err == nil {
		_ = json.Unmarshal(data, &sessions)
	}

	sessions = append(sessions, session)

	// Keep last 500
	if len(sessions) > 500 {
		sessions = sessions[len(sessions)-500:]
	}

	data, err := json.Marshal(sessions)
	if err != nil {
		log.Printf("SessionTracker: failed to marshal sessions: %v", err)
		return
	}

	if err := s.dbProvider.Set(sessionsDBKey, data); err != nil {
		log.Printf("SessionTracker: failed to save session: %v", err)
	}
}
