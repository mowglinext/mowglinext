package providers

import (
	"encoding/json"
	"math"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

func loadStoredSessions(t *testing.T, db types.IDBProvider) []MowingSessionRecord {
	t.Helper()
	var sessions []MowingSessionRecord
	if data, err := db.Get(sessionsDBKey); err == nil {
		_ = json.Unmarshal(data, &sessions)
	}
	return sessions
}

// newTrackerNoGoroutine builds a tracker without starting the consumer goroutine
// so tests can drive OnHighLevelStatus synchronously.
func newTrackerNoGoroutine(db types.IDBProvider) *SessionTracker {
	return &SessionTracker{dbProvider: db}
}

func status(state int, name string, emergency bool) []byte {
	b, _ := json.Marshal(map[string]any{
		"state": state, "state_name": name, "emergency": emergency,
	})
	return b
}

// endMowing drives the two non-mowing messages required to finalize a session.
func endMowing(s *SessionTracker) {
	s.OnHighLevelStatus(status(1, "IDLE", false))
	s.OnHighLevelStatus(status(1, "IDLE", false))
}

func TestSessionTracker_DropsSpuriousShortSession(t *testing.T) {
	db := types.NewMockDBProvider()
	s := newTrackerNoGoroutine(db)

	s.OnHighLevelStatus(status(2, "MOWING", false)) // start; sessionStart = now
	endMowing(s)                                    // ~0s duration

	if got := loadStoredSessions(t, db); len(got) != 0 {
		t.Fatalf("expected spurious short session to be dropped, got %d records", len(got))
	}
}

func TestSessionTracker_RecordsRealSession(t *testing.T) {
	db := types.NewMockDBProvider()
	s := newTrackerNoGoroutine(db)

	s.OnHighLevelStatus(status(2, "MOWING", false))
	s.sessionStart = time.Now().UTC().Add(-60 * time.Second) // simulate a 60s mow
	// Two non-mowing messages finalize; IDLE_DOCKED marks a clean completion.
	s.OnHighLevelStatus(status(1, "IDLE_DOCKED", false))
	s.OnHighLevelStatus(status(1, "IDLE_DOCKED", false))

	got := loadStoredSessions(t, db)
	if len(got) != 1 {
		t.Fatalf("expected 1 recorded session, got %d", len(got))
	}
	if got[0].Status != "completed" {
		t.Fatalf("expected completed, got %q", got[0].Status)
	}
}

// progressStatus builds a mowing status carrying live coverage/swath telemetry.
func progressStatus(coverage float32, completed, skipped int) []byte {
	b, _ := json.Marshal(map[string]any{
		"state": 2, "state_name": "MOWING", "emergency": false,
		"coverage_percent": coverage, "completed_swaths": completed, "skipped_swaths": skipped,
	})
	return b
}

func TestSessionTracker_CapturesCoverageAndStrips(t *testing.T) {
	db := types.NewMockDBProvider()
	s := newTrackerNoGoroutine(db)

	s.OnHighLevelStatus(progressStatus(10, 1, 0)) // start
	s.sessionStart = time.Now().UTC().Add(-60 * time.Second)
	s.OnHighLevelStatus(progressStatus(55, 4, 1))
	s.OnHighLevelStatus(progressStatus(92, 8, 2)) // peak
	// A new-area reset tick must not clobber the captured peak.
	s.OnHighLevelStatus(progressStatus(3, 0, 0))
	s.OnHighLevelStatus(status(1, "IDLE_DOCKED", false))
	s.OnHighLevelStatus(status(1, "IDLE_DOCKED", false))

	got := loadStoredSessions(t, db)
	if len(got) != 1 {
		t.Fatalf("expected 1 recorded session, got %d", len(got))
	}
	if got[0].CoveragePercent != 92 {
		t.Fatalf("expected coverage peak 92, got %v", got[0].CoveragePercent)
	}
	if got[0].StripsCompleted != 8 {
		t.Fatalf("expected 8 completed strips, got %d", got[0].StripsCompleted)
	}
	if got[0].StripsSkipped != 2 {
		t.Fatalf("expected 2 skipped strips, got %d", got[0].StripsSkipped)
	}
}

// odomAt builds a nav_msgs/Odometry payload at position (x, y).
func odomAt(x, y float64) []byte {
	b, _ := json.Marshal(map[string]any{
		"pose": map[string]any{"pose": map[string]any{
			"position": map[string]any{"x": x, "y": y},
		}},
	})
	return b
}

func TestSessionTracker_AccumulatesDistance(t *testing.T) {
	db := types.NewMockDBProvider()
	s := newTrackerNoGoroutine(db)

	// Samples before a session starts must not count.
	s.OnOdometry(odomAt(10, 10))

	s.OnHighLevelStatus(status(2, "MOWING", false)) // start; resets odometer
	s.sessionStart = time.Now().UTC().Add(-60 * time.Second)
	s.OnOdometry(odomAt(0, 0))   // baseline
	s.OnOdometry(odomAt(0.3, 0)) // +0.3
	s.OnOdometry(odomAt(0.6, 0)) // +0.3
	s.OnOdometry(odomAt(0.9, 0)) // +0.3 => 0.9
	s.OnOdometry(odomAt(50, 50)) // jump > maxOdomStepM: discarded, re-baseline
	s.OnOdometry(odomAt(50.3, 50)) // +0.3 => 1.2

	s.OnHighLevelStatus(status(1, "IDLE_DOCKED", false))
	s.OnHighLevelStatus(status(1, "IDLE_DOCKED", false))

	got := loadStoredSessions(t, db)
	if len(got) != 1 {
		t.Fatalf("expected 1 recorded session, got %d", len(got))
	}
	if math.Abs(got[0].DistanceMeters-1.2) > 1e-6 {
		t.Fatalf("expected distance 1.2 m, got %v", got[0].DistanceMeters)
	}
}

func TestSessionTracker_RecordsShortEmergency(t *testing.T) {
	db := types.NewMockDBProvider()
	s := newTrackerNoGoroutine(db)

	s.OnHighLevelStatus(status(2, "MOWING", false))
	// Immediate emergency abort (sub-threshold) must still be recorded as error.
	s.OnHighLevelStatus(status(0, "EMERGENCY", true))
	s.OnHighLevelStatus(status(0, "EMERGENCY", true))

	got := loadStoredSessions(t, db)
	if len(got) != 1 {
		t.Fatalf("expected emergency session recorded, got %d", len(got))
	}
	if got[0].Status != "error" {
		t.Fatalf("expected error status, got %q", got[0].Status)
	}
}
