package blackbox

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestRepeatedPruneFailureBoundsPublicationAndRecovers(t *testing.T) {
	for _, limit := range []string{"count", "bytes"} {
		t.Run(limit, func(t *testing.T) {
			r, clock := testRecorder(t)
			cfg := r.Status().Config
			if limit == "count" {
				cfg.MaxSnapshots = 1
			} else {
				cfg.MaxDiskBytes = 4 << 20
			}
			configureRecorder(t, r, cfg)
			if _, ok := r.Trigger("manual"); !ok {
				t.Fatal("initial capture rejected")
			}
			original := finishTestCapture(t, r, clock)
			if limit == "bytes" {
				if err := os.Truncate(filepath.Join(r.dir, original.Name), cfg.MaxDiskBytes); err != nil {
					t.Fatal(err)
				}
			}
			deletionFailure := errors.New("injected persistent deletion failure")
			r.mu.Lock()
			r.removeCompleted = func(*os.Root, string) error { return deletionFailure }
			r.mu.Unlock()
			var replacement string
			for i := 0; i < 4; i++ {
				if _, ok := r.Trigger("manual"); !ok {
					t.Fatal("manual incident rejected")
				}
				snapshot := finishTestCapture(t, r, clock)
				if i == 0 {
					replacement = snapshot.Name
					if replacement == original.Name || !strings.Contains(r.Status().LastError, deletionFailure.Error()) {
						t.Fatal("first replacement was not published with a visible cleanup error")
					}
				} else if !strings.Contains(r.Status().LastError, "retention exceeded") {
					t.Fatalf("missing preflight failure: %+v", r.Status())
				}
				list, err := r.List()
				if err != nil || len(list) != 2 || r.Status().CompletedSnapshots != 2 {
					t.Fatalf("failed pruning grew files or hid successful publication: list=%v status=%+v err=%v", list, r.Status(), err)
				}
				entries, err := os.ReadDir(r.dir)
				if err != nil || len(entries) != 2 {
					t.Fatalf("unexpected partial/publication growth: entries=%v err=%v", entries, err)
				}
			}
			r.mu.Lock()
			r.removeCompleted = func(root *os.Root, name string) error { return root.Remove(name) }
			r.mu.Unlock()
			// Retry the same settings after the filesystem recovers.
			configureRecorder(t, r, cfg)
			list, err := r.List()
			if err != nil || len(list) != 1 || list[0].Name != replacement {
				t.Fatalf("recovery lost newest evidence: %v %v", list, err)
			}
			if _, ok := r.Trigger("manual"); !ok {
				t.Fatal("recording did not recover")
			}
			finishTestCapture(t, r, clock)
			if r.Status().CompletedSnapshots != 3 || r.Status().LastError != "" {
				t.Fatalf("recording did not recover after retention: %+v", r.Status())
			}
		})
	}
}
