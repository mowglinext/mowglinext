package blackbox

import (
	"errors"
	"fmt"
	"strings"
	"testing"
	"time"
)

func TestZeroAvailableMemoryIsKnownPressure(t *testing.T) {
	s := parseMemoryInfo([]byte("MemTotal: 131072 kB\nMemAvailable: 0 kB\n"))
	if !s.known || s.available != 0 || s.total != 128<<20 {
		t.Fatalf("zero availability parsed as unknown: %+v", s)
	}
	r, _ := testRecorder(t)
	r.mu.Lock()
	r.applyPressure(s)
	r.mu.Unlock()
	if !r.Status().MemoryPressure {
		t.Fatal("exhausted host did not suspend telemetry")
	}
	if parseMemoryInfo([]byte("MemTotal: 131072 kB\n")).known {
		t.Fatal("missing availability field parsed as valid")
	}
}

func TestQueuedObservationRespectsLoweredMessageLimit(t *testing.T) {
	r, _ := testRecorder(t)
	cfg := r.Status().Config
	cfg.MaxMessageBytes = 256
	err := r.ConfigurePersist(cfg, func() error {
		if !r.Ingest("odom", []byte(`{"payload":"`+strings.Repeat("x", 1000)+`"}`)) {
			t.Fatal("old configuration did not admit queued message")
		}
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
	waitFor(t, func() bool { return len(r.permits) == 0 })
	if r.Status().BufferedRecords != 0 || r.Status().DroppedMessages != 1 {
		t.Fatal("oversized queued message survived lowered limit")
	}
}

func TestDiagnosticIdentityTableOverflowCannotRetrigger(t *testing.T) {
	r, clock := testRecorder(t)
	for i := 0; i < 128; i++ {
		ingest(t, r, "diagnostics", fmt.Sprintf(`{"status":[{"name":"source-%d","level":0}]}`, i))
	}
	for i := 0; i < 3; i++ {
		clock.Add(int64(61 * time.Second))
		ingest(t, r, "diagnostics", `{"status":[{"name":"overflow","level":2}]}`)
		if r.Status().Phase != "ready" {
			t.Fatal("untracked diagnostic source triggered")
		}
	}
	if got := r.Status().SkippedTriggerSources; got != 3 {
		t.Fatalf("skipped sources = %d", got)
	}
	// Existing identities continue to trigger after the table fills.
	ingest(t, r, "diagnostics", `{"status":[{"name":"source-0","level":2}]}`)
	if r.Status().Phase != "capturing" {
		t.Fatal("known diagnostic edge was lost")
	}
}

func TestDiagnosticHardwareIdentityHasIndependentEdges(t *testing.T) {
	r, clock := testRecorder(t)
	ingest(t, r, "diagnostics", `{"status":[{"name":"receiver","hardware_id":"a","level":2}]}`)
	finishTestCapture(t, r, clock)
	clock.Add(int64(61 * time.Second))
	ingest(t, r, "diagnostics", `{"status":[{"name":"receiver","hardware_id":"b","level":2}]}`)
	if r.Status().Phase != "capturing" {
		t.Fatal("distinct hardware source did not trigger")
	}
}

func TestPressureIncidentDoesNotAllocateCaptureSlots(t *testing.T) {
	r, clock := testRecorder(t)
	r.mu.Lock()
	r.applyPressure(resourceSample{total: 64 << 20, available: 8 << 20, known: true})
	r.mu.Unlock()
	ingest(t, r, "emergency", `{"active_emergency":true,"latched_emergency":false}`)
	r.mu.Lock()
	metadataOnly := r.active != nil && cap(r.active.observations) == 0
	r.mu.Unlock()
	if !metadataOnly {
		t.Fatal("pressure incident allocated telemetry slots")
	}
	snapshot := finishTestCapture(t, r, clock)
	if snapshot.Records != 0 {
		t.Fatal("pressure incident retained optional telemetry")
	}
}

func TestConfigurePreservesPrehistoryAcrossSameAndRetentionOnlyChanges(t *testing.T) {
	r, clock := testRecorder(t)
	ingest(t, r, "odom", `{"x":1}`)

	cfg := r.Status().Config
	if err := r.Configure(cfg); err != nil {
		t.Fatalf("configure with unchanged settings: %v", err)
	}
	if got := cfg.MaxSnapshots; got > 1 {
		cfg.MaxSnapshots = got - 1
	}
	cfg.MaxDiskBytes /= 2
	if err := r.Configure(cfg); err != nil {
		t.Fatalf("configure retention-only changes: %v", err)
	}
	if got := r.Status().BufferedRecords; got != 1 {
		t.Fatalf("prehistory after configuration changes = %d records, want 1", got)
	}

	if _, ok := r.Trigger("manual"); !ok {
		t.Fatal("manual trigger rejected")
	}
	snapshot := finishTestCapture(t, r, clock)
	if snapshot.Records != 1 {
		t.Fatalf("capture retained %d records, want prehistory record", snapshot.Records)
	}
}

func TestConfigurePreservesEmergencyEdge(t *testing.T) {
	tests := []struct {
		name   string
		update func(Config) Config
	}{
		{name: "same config", update: func(cfg Config) Config { return cfg }},
		{name: "retention-only changes", update: func(cfg Config) Config {
			cfg.MaxSnapshots--
			cfg.MaxDiskBytes /= 2
			return cfg
		}},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			r, clock := testRecorder(t)
			ingest(t, r, "emergency", `{"active_emergency":false,"latched_emergency":false}`)
			ingest(t, r, "emergency", `{"active_emergency":true,"latched_emergency":true}`)
			if r.Status().Phase != "capturing" {
				t.Fatal("initial emergency edge did not trigger")
			}
			finishTestCapture(t, r, clock)

			// Move beyond cooldown so a reset edge would produce an accepted capture.
			clock.Add(int64(61 * time.Second))
			cfg := tt.update(r.Status().Config)
			if err := r.Configure(cfg); err != nil {
				t.Fatalf("configure: %v", err)
			}
			ingest(t, r, "emergency", `{"active_emergency":true,"latched_emergency":true}`)
			status := r.Status()
			if status.Phase != "ready" || status.CoalescedTriggers != 0 {
				t.Fatalf("unchanged emergency edge triggered after configure: %+v", status)
			}
		})
	}
}

func TestConfigurePersistFailureKeepsOldStateAndSerializesTrigger(t *testing.T) {
	r, _ := testRecorder(t)
	ingest(t, r, "odom", `{"x":1}`)
	oldConfig := r.Status().Config
	newConfig := oldConfig
	newConfig.Enabled = false
	newConfig.PreSeconds = 1
	persistErr := errors.New("injected settings persistence failure")
	persistStarted := make(chan struct{})
	releasePersist := make(chan struct{})
	configureDone := make(chan error, 1)
	go func() {
		configureDone <- r.ConfigurePersist(newConfig, func() error {
			close(persistStarted)
			<-releasePersist
			return persistErr
		})
	}()
	<-persistStarted

	type triggerResult struct {
		id string
		ok bool
	}
	triggerStarted := make(chan struct{})
	triggerDone := make(chan triggerResult, 1)
	go func() {
		close(triggerStarted)
		id, ok := r.Trigger("manual")
		triggerDone <- triggerResult{id: id, ok: ok}
	}()
	<-triggerStarted
	select {
	case result := <-triggerDone:
		t.Fatalf("trigger returned before persistence failed: %+v", result)
	case <-time.After(25 * time.Millisecond):
	}

	close(releasePersist)
	if err := <-configureDone; !errors.Is(err, persistErr) {
		t.Fatalf("ConfigurePersist error = %v, want %v", err, persistErr)
	}
	result := <-triggerDone
	if !result.ok || result.id == "" {
		t.Fatal("concurrent trigger did not use the old enabled configuration")
	}
	if got := *r.config.Load(); got != oldConfig {
		t.Fatalf("published config after persistence failure = %+v, want %+v", got, oldConfig)
	}
	r.mu.Lock()
	active := r.active
	if active == nil || active.metadata.Config != oldConfig || len(active.observations) != 1 {
		r.mu.Unlock()
		t.Fatalf("trigger did not use old config and prehistory: active=%+v", active)
	}
	r.mu.Unlock()
}

func TestPersistRetainsIncomingSnapshotWhenWallClockMovesBackwards(t *testing.T) {
	r, _ := testRecorder(t)
	cfg := r.Status().Config
	cfg.MaxSnapshots = 1
	if err := r.Configure(cfg); err != nil {
		t.Fatal(err)
	}

	makeCapture := func(timestamp string) *capture {
		t.Helper()
		triggeredAt, err := time.Parse("20060102T150405.000000000Z", timestamp)
		if err != nil {
			t.Fatal(err)
		}
		id := "blackbox-" + timestamp + "-0123456789abcdef"
		return &capture{
			trigger: triggeredAt,
			metadata: Metadata{
				Kind:          "blackbox_metadata",
				FormatVersion: 1,
				CaptureID:     id,
				TriggeredAt:   triggeredAt,
				Config:        cfg,
			},
		}
	}

	completedLater := makeCapture("20261010T120001.000000000Z")
	if err := r.persist(completedLater); err != nil {
		t.Fatalf("persist existing snapshot: %v", err)
	}
	incomingEarlier := makeCapture("20261010T120000.000000000Z")
	if err := r.persist(incomingEarlier); err != nil {
		t.Fatalf("persist snapshot after wall-clock rollback: %v", err)
	}

	list, err := r.List()
	if err != nil {
		t.Fatal(err)
	}
	if len(list) != 1 || list[0].Name != incomingEarlier.metadata.CaptureID+".jsonl" {
		t.Fatalf("retained snapshots after backward clock step = %+v, want incoming %s", list, incomingEarlier.metadata.CaptureID)
	}
}

func TestCaptureDropCountExcludesHistoryRingEvictions(t *testing.T) {
	r, clock := testRecorder(t)
	r.mu.Lock()
	r.buffer = ring{slots: make([]observation, 1)}
	r.mu.Unlock()

	ingest(t, r, "odom", `{"x":1}`)
	if _, ok := r.Trigger("manual"); !ok {
		t.Fatal("manual trigger rejected")
	}
	ingest(t, r, "odom", `{"x":2}`)
	if got := r.Status().HistoryEvictions; got != 1 {
		t.Fatalf("history evictions = %d, want 1", got)
	}

	snapshot := finishTestCapture(t, r, clock)
	if snapshot.Records != 2 || snapshot.CaptureDroppedMessages != 0 {
		t.Fatalf("capture metadata counted history eviction as a capture drop: %+v", snapshot.Metadata)
	}
}
