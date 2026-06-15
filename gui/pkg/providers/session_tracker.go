package providers

import (
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
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
	// queue serializes status messages through a single consumer goroutine so
	// the start/pause/resume/end state machine is applied in arrival order.
	// Previously fanOut spawned a goroutine per message (go OnHighLevelStatus),
	// which the scheduler could run out of order — corrupting inSession/paused/
	// pauseCount on rapid MOWING->CHARGING->IDLE bursts.
	queue chan []byte
}

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
	}
	go s.run()
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

// OnHighLevelStatus processes a raw JSON high-level status message.
// Call this from the fanOut pipeline for the "highLevelStatus" topic.
func (s *SessionTracker) OnHighLevelStatus(msg []byte) {
	var status struct {
		State     int     `json:"state"`
		StateName string  `json:"state_name"`
		Emergency bool    `json:"emergency"`
		Battery   float32 `json:"battery_percent"`
		Area      int     `json:"current_area"`
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
		s.sessionStart = time.Now().UTC()
		log.Printf("SessionTracker: mowing session started (state=%s)", status.StateName)
		return
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
			ID:             fmt.Sprintf("%d", s.sessionStart.UnixMilli()),
			StartTime:      s.sessionStart.Format(time.RFC3339),
			EndTime:        endTime.Format(time.RFC3339),
			DurationSec:    duration,
			AreaIndex:      status.Area,
			Status:         sessionStatus,
			RechargePauses: s.pauseCount,
			Errors:         []string{},
		}

		if status.Emergency {
			session.Errors = append(session.Errors, "Emergency stop triggered")
		}

		s.saveSession(session)
		log.Printf("SessionTracker: session ended (status=%s, duration=%.0fs)", sessionStatus, duration)
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
