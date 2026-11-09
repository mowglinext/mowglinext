package blackbox

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func TestMaintenancePreservesIncidentAndSerializesWorkers(t *testing.T) {
	for _, startup := range []bool{false, true} {
		for _, event := range []string{"emergency", "diagnostic", "high-level", "shutdown"} {
			name := "settings/" + event
			if startup {
				name = "startup/" + event
			}
			t.Run(name, func(t *testing.T) {
				cfg := DefaultConfig()
				cfg.PreSeconds, cfg.PostSeconds = 3, 2
				cfg.MemoryBytes, cfg.MinFreeDiskBytes = 4<<20, 0
				dir := t.TempDir()
				if startup {
					// A completed filename is enough to schedule startup maintenance;
					// the intentionally invalid header remains invisible to List.
					if err := os.WriteFile(filepath.Join(dir, "blackbox-20261010T000000.000000000Z-0123456789abcdef.jsonl"), []byte("{}\n"), 0600); err != nil {
						t.Fatal(err)
					}
				}
				started, release := make(chan struct{}), make(chan struct{})
				var releaseOnce sync.Once
				unblock := func() { releaseOnce.Do(func() { close(release) }) }
				var workers, overlaps, writes atomic.Int64
				maintain := func(Config) error {
					if workers.Add(1) != 1 {
						overlaps.Add(1)
					}
					close(started)
					<-release
					workers.Add(-1)
					return nil
				}
				r, err := newRecorder(dir, cfg, nil, maintain)
				if err != nil {
					unblock()
					t.Fatal(err)
				}
				t.Cleanup(func() {
					unblock()
					ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
					defer cancel()
					if err := r.Close(ctx); err != nil {
						t.Error(err)
					}
				})
				var clock atomic.Int64
				base := time.Now()
				r.mu.Lock()
				r.now = func() time.Time { return base.Add(time.Duration(clock.Load())) }
				persist := r.write
				r.write = func(c *capture) error {
					if workers.Add(1) != 1 {
						overlaps.Add(1)
					}
					defer workers.Add(-1)
					writes.Add(1)
					return persist(c)
				}
				r.mu.Unlock()
				if !startup {
					cfg.MaxSnapshots--
					if err := r.Configure(cfg); err != nil {
						t.Fatal(err)
					}
				}
				select {
				case <-started:
				case <-time.After(5 * time.Second):
					t.Fatal("maintenance did not start")
				}
				ingest(t, r, "odom", `{"x":0}`)
				clock.Store(int64(time.Second))
				switch event {
				case "emergency", "shutdown":
					ingest(t, r, "emergency", `{"active_emergency":true}`)
					// The unchanged edge must not produce another incident.
					ingest(t, r, "emergency", `{"active_emergency":true}`)
				case "diagnostic":
					ingest(t, r, "diagnostics", `{"status":[{"name":"controller","level":2}]}`)
					ingest(t, r, "diagnostics", `{"status":[{"name":"controller","level":2}]}`)
				case "high-level":
					if _, ok := r.Trigger("high-level emergency: synthetic"); !ok {
						t.Fatal("external automatic trigger rejected during maintenance")
					}
				}
				status := r.Status()
				if status.Phase != "capturing" || status.CaptureID == "" {
					t.Fatalf("maintenance lost incident: %+v", status)
				}
				if id, accepted := r.Trigger("diagnostic ERROR: additional"); accepted || id != status.CaptureID {
					t.Fatal("simultaneous incident was not coalesced")
				}
				clock.Store(int64(2 * time.Second))
				ingest(t, r, "odom", `{"x":2}`)
				if event == "shutdown" {
					ctx, cancel := context.WithTimeout(context.Background(), 20*time.Millisecond)
					err := r.Close(ctx)
					cancel()
					if !errors.Is(err, context.DeadlineExceeded) {
						t.Fatalf("close while maintenance blocked: %v", err)
					}
				} else {
					clock.Store(int64(4 * time.Second))
					// Keep maintenance blocked beyond the collector's finish tick.
					time.Sleep(150 * time.Millisecond)
				}
				if writes.Load() != 0 || workers.Load() != 1 {
					t.Fatal("snapshot writer overlapped maintenance")
				}
				unblock()
				waitFor(t, func() bool { return r.Status().CompletedSnapshots == 1 })
				if event == "shutdown" {
					waitFor(t, func() bool {
						select {
						case <-r.done:
							return true
						default:
							return false
						}
					})
				}
				list, err := r.List()
				if err != nil || len(list) != 1 {
					t.Fatalf("completed incidents = %v, err=%v", list, err)
				}
				s := list[0]
				if s.ActualPreSeconds != 1 || s.ActualPostSeconds != 1 || s.Interrupted != (event == "shutdown") || len(s.Reasons) != 2 || !strings.Contains(s.Reasons[1], "additional") {
					t.Fatalf("incident lost history/metadata: %+v", s.Metadata)
				}
				if writes.Load() != 1 || overlaps.Load() != 0 || r.Status().LastError != "" {
					t.Fatalf("unbounded/failed disk work: writes=%d overlaps=%d status=%+v", writes.Load(), overlaps.Load(), r.Status())
				}
			})
		}
	}
}
