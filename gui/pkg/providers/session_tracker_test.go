package providers

import (
	"encoding/json"
	"testing"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
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
