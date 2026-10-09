package blackbox

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func testRecorder(t *testing.T) (*Recorder, *atomic.Int64) {
	t.Helper()
	cfg := DefaultConfig()
	cfg.PreSeconds = 3
	cfg.PostSeconds = 2
	cfg.MemoryBytes = 4 << 20
	cfg.MinFreeDiskBytes = 0
	r, err := New(t.TempDir(), cfg, map[string]string{"revision": "synthetic-test"})
	if err != nil {
		t.Fatal(err)
	}
	clock := &atomic.Int64{}
	base := time.Now()
	r.mu.Lock()
	r.now = func() time.Time { return base.Add(time.Duration(clock.Load())) }
	r.mu.Unlock()
	t.Cleanup(func() {
		ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		if err := r.Close(ctx); err != nil {
			t.Error(err)
		}
	})
	return r, clock
}

func waitFor(t *testing.T, condition func() bool) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for !condition() {
		if time.Now().After(deadline) {
			t.Fatal("timed out waiting for recorder")
		}
		time.Sleep(5 * time.Millisecond)
	}
}

func ingest(t *testing.T, r *Recorder, topic, data string) {
	t.Helper()
	if !r.Ingest(topic, []byte(data)) {
		t.Fatal("unexpected ingress rejection")
	}
	waitFor(t, func() bool { return len(r.permits) == 0 })
}

func finishTestCapture(t *testing.T, r *Recorder, clock *atomic.Int64) Snapshot {
	t.Helper()
	clock.Add(int64(3 * time.Second))
	waitFor(t, func() bool { return r.Status().Phase == "ready" })
	list, err := r.List()
	if err != nil || len(list) == 0 {
		t.Fatalf("list=%v err=%v status=%+v", list, err, r.Status())
	}
	return list[0]
}

func TestRollingWindowAndManualCapture(t *testing.T) {
	r, clock := testRecorder(t)
	ingest(t, r, "odom", `{"x":0}`)
	clock.Store(int64(2 * time.Second))
	ingest(t, r, "odom", `{"x":2}`)
	clock.Store(int64(4 * time.Second))
	ingest(t, r, "odom", `{"x":4}`)
	if got := r.Status().BufferedRecords; got != 2 {
		t.Fatalf("retained %d, want 2", got)
	}
	id, accepted := r.Trigger("manual")
	if !accepted || id == "" {
		t.Fatal("manual trigger rejected")
	}
	clock.Store(int64(5 * time.Second))
	ingest(t, r, "odom", `{"x":5}`)
	snapshot := finishTestCapture(t, r, clock)
	if snapshot.ActualPreSeconds != 2 || snapshot.ActualPostSeconds != 1 || snapshot.Records != 3 {
		t.Fatalf("incorrect actual durations: %+v", snapshot.Metadata)
	}
	f, err := r.Open(snapshot.Name)
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	data, err := io.ReadAll(f)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(data), `"x":0`) || !strings.Contains(string(data), `"x":2`) || !strings.Contains(string(data), `"x":5`) {
		t.Fatalf("unexpected history: %s", data)
	}
	scanner := bufio.NewScanner(strings.NewReader(string(data)))
	lines := 0
	for scanner.Scan() {
		if !json.Valid(scanner.Bytes()) {
			t.Fatal("invalid JSONL")
		}
		lines++
	}
	if lines != 5 {
		t.Fatalf("got %d lines", lines)
	}
}

func TestAutomaticEmergencyCoalescingAndCooldown(t *testing.T) {
	r, clock := testRecorder(t)
	ingest(t, r, "emergency", `{"active_emergency":false,"latched_emergency":false}`)
	ingest(t, r, "emergency", `{"active_emergency":true,"latched_emergency":true}`)
	if r.Status().Phase != "capturing" {
		t.Fatal("emergency failed to trigger")
	}
	for i := 0; i < 20; i++ {
		ingest(t, r, "emergency", `{"active_emergency":true,"latched_emergency":true}`)
	}
	if r.Status().CoalescedTriggers != 0 {
		t.Fatal("unchanged emergency triggered repeatedly")
	}
	for i := 0; i < 20; i++ {
		r.Trigger("diagnostic ERROR: drive")
	}
	if got := r.Status().CoalescedTriggers; got != 20 {
		t.Fatalf("coalesced=%d", got)
	}
	snapshot := finishTestCapture(t, r, clock)
	if len(snapshot.Reasons) != 2 {
		t.Fatalf("reasons=%v", snapshot.Reasons)
	}
	ingest(t, r, "emergency", `{"active_emergency":false,"latched_emergency":false}`)
	ingest(t, r, "emergency", `{"active_emergency":true,"latched_emergency":false}`)
	if r.Status().Phase != "ready" {
		t.Fatal("cooldown failed")
	}
}

func TestDiagnosticTriggerAndBoundedProducerNames(t *testing.T) {
	r, _ := testRecorder(t)
	ingest(t, r, "fusionDiag", `{"status":[{"name":"fusion","level":2}]}`)
	if r.Status().Phase != "capturing" {
		t.Fatal("diagnostic ERROR failed to trigger")
	}
	ingest(t, r, "fusionDiag", `{"status":[{"name":"fusion","level":2}]}`)
	if r.Status().CoalescedTriggers != 0 {
		t.Fatal("unchanged error repeated")
	}
	for i := 0; i < 300; i++ {
		ingest(t, r, "diagnostics", `{"status":[{"name":"`+strings.Repeat("x", i%129)+`","level":0}]}`)
	}
	r.mu.Lock()
	n := len(r.diagnosticErrors)
	r.mu.Unlock()
	if n > 128 {
		t.Fatalf("unbounded identities: %d", n)
	}
}

func TestOversizedInvalidAndHighFrequencyMessages(t *testing.T) {
	r, _ := testRecorder(t)
	if r.Ingest("odom", make([]byte, 16385)) {
		t.Fatal("oversized payload admitted")
	}
	ingest(t, r, "odom", "not-json")
	data := []byte(`{"payload":"` + strings.Repeat("x", 7000) + `"}`)
	for i := 0; i < 10000; i++ {
		r.Ingest("odom", data)
	}
	waitFor(t, func() bool { return len(r.permits) == 0 })
	s := r.Status()
	if s.BufferedBytes > r.historyLimit() || s.BufferedRecords > len(r.buffer.slots) || s.DroppedMessages < 2 {
		t.Fatalf("bounds violated: %+v", s)
	}
	if _, ok := r.Trigger("manual"); !ok {
		t.Fatal("trigger")
	}
	for i := 0; i < 10000; i++ {
		r.Ingest("odom", data)
	}
	waitFor(t, func() bool { return len(r.permits) == 0 })
	// Ensure the capture budget itself fills, independently of how much the
	// nonblocking ingress queue dropped during the unpaced burst above.
	for i := 0; i < 300; i++ {
		ingest(t, r, "odom", string(data))
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.active.bytes > r.active.limit || len(r.active.observations) > cap(r.active.observations) || r.active.drops == 0 {
		t.Fatal("capture not bounded")
	}
}

func TestConcurrentTriggersAndIngress(t *testing.T) {
	r, _ := testRecorder(t)
	var wg sync.WaitGroup
	for i := 0; i < 16; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < 100; j++ {
				r.Ingest("odom", []byte(`{"x":1}`))
				r.Trigger("manual")
				_ = r.Status()
			}
		}()
	}
	wg.Wait()
	if r.Status().Phase != "capturing" || r.Status().CoalescedTriggers != 1599 {
		t.Fatalf("status=%+v", r.Status())
	}
}

func TestPressureStopsHistoryWithoutControllingRobot(t *testing.T) {
	r, _ := testRecorder(t)
	ingest(t, r, "odom", `{"x":1}`)
	r.mu.Lock()
	r.applyPressure(resourceSample{total: 128 << 20, available: 8 << 20, known: true})
	r.mu.Unlock()
	ingest(t, r, "odom", `{"x":2}`)
	if !r.Status().MemoryPressure || r.Status().BufferedRecords != 0 || r.Status().DroppedMessages == 0 {
		t.Fatalf("pressure status=%+v", r.Status())
	}
	r.mu.Lock()
	r.applyPressure(resourceSample{total: 128 << 20, available: 64 << 20, known: true})
	r.mu.Unlock()
	ingest(t, r, "odom", `{"x":3}`)
	if r.Status().BufferedRecords != 1 {
		t.Fatal("did not recover after pressure")
	}
}

func TestWriteFailureRecoveryAndNoNormalDiskWrites(t *testing.T) {
	r, clock := testRecorder(t)
	ingest(t, r, "odom", `{"x":1}`)
	files, err := os.ReadDir(r.dir)
	if err != nil || len(files) != 0 {
		t.Fatalf("normal ingestion wrote disk: %v %v", files, err)
	}
	r.mu.Lock()
	r.write = func(*capture) error { return errors.New("injected full disk") }
	r.mu.Unlock()
	r.Trigger("manual")
	clock.Store(int64(3 * time.Second))
	waitFor(t, func() bool { return r.Status().Phase == "ready" })
	if !strings.Contains(r.Status().LastError, "full disk") {
		t.Fatal("write error not reported")
	}
	r.mu.Lock()
	r.write = r.persist
	r.mu.Unlock()
	ingest(t, r, "odom", `{"x":2}`)
	r.Trigger("manual")
	finishTestCapture(t, r, clock)
	if r.Status().LastError != "" || r.Status().CompletedSnapshots != 1 {
		t.Fatalf("status=%+v", r.Status())
	}
}

func TestRestartShutdownAndSafeStorage(t *testing.T) {
	r, _ := testRecorder(t)
	ingest(t, r, "odom", `{"x":1}`)
	r.Trigger("manual")
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	if err := r.Close(ctx); err != nil {
		t.Fatal(err)
	}
	list, err := r.List()
	if err != nil || len(list) != 1 || !list[0].Interrupted {
		t.Fatalf("shutdown list=%v err=%v", list, err)
	}
	if _, err := r.Open("../secrets"); err == nil {
		t.Fatal("traversal accepted")
	}
	if err := r.Delete("../secrets"); err == nil {
		t.Fatal("delete traversal accepted")
	}
	partial := list[0].Name + ".partial"
	if err := os.WriteFile(filepath.Join(r.dir, partial), []byte("broken"), 0600); err != nil {
		t.Fatal(err)
	}
	r2, err := New(r.dir, DefaultConfig(), nil)
	if err != nil {
		t.Fatal(err)
	}
	defer r2.Close(ctx)
	if _, err := os.Stat(filepath.Join(r.dir, partial)); !os.IsNotExist(err) {
		t.Fatal("partial file survived restart")
	}
	if err := r2.Delete(list[0].Name); err != nil {
		t.Fatal(err)
	}
	list, err = r2.List()
	if err != nil || len(list) != 0 {
		t.Fatal("delete failed")
	}
}

func TestCountAndByteRetention(t *testing.T) {
	r, clock := testRecorder(t)
	cfg := r.Status().Config
	cfg.MaxSnapshots = 2
	if err := r.Configure(cfg); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 4; i++ {
		ingest(t, r, "odom", `{"x":1}`)
		r.Trigger("manual")
		finishTestCapture(t, r, clock)
	}
	list, err := r.List()
	if err != nil || len(list) != 2 {
		t.Fatalf("count retention list=%v err=%v", list, err)
	}
	for _, s := range list {
		if err := os.Truncate(filepath.Join(r.dir, s.Name), 3<<20); err != nil {
			t.Fatal(err)
		}
	}
	cfg.MaxDiskBytes = 4 << 20
	if err := r.prune(cfg, 0); err != nil {
		t.Fatal(err)
	}
	list, err = r.List()
	if err != nil || len(list) != 1 {
		t.Fatalf("byte retention list=%v err=%v", list, err)
	}
}

func TestConfigValidationAndCaptureReconfiguration(t *testing.T) {
	r, _ := testRecorder(t)
	cfg := r.Status().Config
	bad := cfg
	bad.MemoryBytes = 1 << 20
	if r.Configure(bad) == nil {
		t.Fatal("tiny budget accepted")
	}
	r.Trigger("manual")
	if r.Configure(cfg) == nil {
		t.Fatal("configuration changed during capture")
	}
}

func TestMissingPublishersAndPayloadTimestampDiscontinuity(t *testing.T) {
	r, clock := testRecorder(t)
	if _, ok := r.Trigger("manual"); !ok {
		t.Fatal("empty capture rejected")
	}
	s := finishTestCapture(t, r, clock)
	if s.Records != 0 || s.ActualPreSeconds != 0 || s.ActualPostSeconds != 0 {
		t.Fatalf("invented missing history: %+v", s.Metadata)
	}
	ingest(t, r, "odom", `{"stamp":{"sec":9999999999},"x":1}`)
	clock.Add(int64(time.Second))
	ingest(t, r, "odom", `{"stamp":{"sec":1},"x":2}`)
	if r.Status().BufferedRecords != 2 {
		t.Fatal("ROS timestamp discontinuity changed retention")
	}
	r.Trigger("manual")
	s = finishTestCapture(t, r, clock)
	if s.Records != 2 || s.ActualPreSeconds != 1 {
		t.Fatalf("wrong acquisition duration %+v", s.Metadata)
	}
}

func TestCaptureDropsAndWriterRemainsAsynchronous(t *testing.T) {
	r, clock := testRecorder(t)
	blocked := make(chan struct{})
	started := make(chan struct{})
	r.mu.Lock()
	r.write = func(*capture) error { close(started); <-blocked; return errors.New("injected blocked storage") }
	r.mu.Unlock()
	r.Trigger("manual")
	clock.Add(int64(3 * time.Second))
	select {
	case <-started:
	case <-time.After(5 * time.Second):
		t.Fatal("writer did not start")
	}
	start := time.Now()
	for i := 0; i < 10000; i++ {
		r.Ingest("odom", []byte(`{"x":1}`))
	}
	if time.Since(start) > time.Second {
		t.Fatal("ingress blocked by storage")
	}
	if _, ok := r.Trigger("manual"); ok {
		t.Fatal("second disk worker started")
	}
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Millisecond)
	defer cancel()
	if !errors.Is(r.Close(ctx), context.DeadlineExceeded) {
		t.Fatal("Close did not respect deadline")
	}
	close(blocked)
}

func TestStorageRejectsSymlinkAndFailedAtomicWrite(t *testing.T) {
	r, clock := testRecorder(t)
	ingest(t, r, "odom", `{"x":1}`)
	r.Trigger("manual")
	s := finishTestCapture(t, r, clock)
	if err := r.Delete(s.Name); err != nil {
		t.Fatal(err)
	}
	outside := filepath.Join(t.TempDir(), "secret")
	if err := os.WriteFile(outside, []byte("secret"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(outside, filepath.Join(r.dir, s.Name)); err == nil {
		if _, err := r.Open(s.Name); err == nil {
			t.Fatal("symlink download accepted")
		}
		if err := r.Delete(s.Name); err == nil {
			t.Fatal("symlink deletion accepted")
		}
		if err := os.Remove(filepath.Join(r.dir, s.Name)); err != nil {
			t.Fatal(err)
		}
	} else {
		t.Logf("symlink creation unavailable: %v", err)
	}
	// Invalid JSON reaches persistence only through this direct fault injection;
	// normal Ingest rejects it. The failed writer leaves no completed/partial file.
	c := &capture{metadata: Metadata{CaptureID: strings.TrimSuffix(s.Name, ".jsonl"), Config: r.Status().Config}, observations: []observation{{at: time.Now(), topic: "odom", data: []byte("invalid")}}, bytes: 100}
	if err := r.persist(c); err == nil {
		t.Fatal("invalid payload persisted")
	}
	files, err := os.ReadDir(r.dir)
	if err != nil || len(files) != 0 {
		t.Fatalf("failed write leaked files %v %v", files, err)
	}
}

func TestResourceBudgetUsesLimits(t *testing.T) {
	if got := initialMemoryBudget(64<<20, resourceSample{total: 256 << 20, available: 128 << 20, known: true}); got != 8<<20 {
		t.Fatalf("cgroup budget=%d", got)
	}
	if got := initialMemoryBudget(64<<20, resourceSample{}); got != 8<<20 {
		t.Fatalf("unknown host budget=%d", got)
	}
	if got := initialMemoryBudget(64<<20, resourceSample{total: 64 << 20, available: 8 << 20, known: true}); got != 4<<20 {
		t.Fatalf("minimum budget=%d", got)
	}
}

func BenchmarkSaturatedIngress(b *testing.B) {
	cfg := DefaultConfig()
	cfg.MinFreeDiskBytes = 0
	r, err := New(b.TempDir(), cfg, nil)
	if err != nil {
		b.Fatal(err)
	}
	defer r.Close(context.Background())
	data := []byte(`{"pose":{"x":1,"y":2},"velocity":0.3}`)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		r.Ingest("odometry", data)
	}
}

func BenchmarkAdmittedIngress(b *testing.B) {
	cfg := DefaultConfig()
	cfg.MinFreeDiskBytes = 0
	r, err := New(b.TempDir(), cfg, nil)
	if err != nil {
		b.Fatal(err)
	}
	defer r.Close(context.Background())
	data := []byte(`{"pose":{"x":1,"y":2},"velocity":0.3}`)
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		if !r.Ingest("odometry", data) {
			b.Fatal("unexpected rejected benchmark message")
		}
		for len(r.permits) > 0 {
			runtime.Gosched()
		}
	}
}
