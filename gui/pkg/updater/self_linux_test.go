package updater

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// This subprocess stands in for the separately executable updater worker. The
// real supervisor must accept a healthy replacement and reject a crashing one.
func TestUpdaterWorkerHelper(t *testing.T) {
	if os.Getenv("MOWGLI_TEST_WORKER") != "1" {
		return
	}
	if os.Getenv("MOWGLI_TEST_CRASH") == "1" {
		os.Exit(23)
	}
	socket := os.Getenv("MOWGLI_TEST_SOCKET")
	_ = os.Remove(socket)
	l, err := net.Listen("unix", socket)
	if err != nil {
		os.Exit(24)
	}
	_ = http.Serve(l, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_ = json.NewEncoder(w).Encode(map[string]any{"agent": map[string]string{"version": os.Getenv("MOWGLI_TEST_VERSION")}})
	}))
	os.Exit(0)
}

func TestSupervisorCommitsHealthyWorkerAndRecoversCrashingCandidate(t *testing.T) {
	for _, crash := range []bool{false, true} {
		t.Run(map[bool]string{false: "healthy", true: "crashing"}[crash], func(t *testing.T) {
			// Unix sockets have a short path limit; do not use the long test name.
			dir, err := os.MkdirTemp("/tmp", "updater-self-")
			if err != nil {
				t.Fatal(err)
			}
			defer os.RemoveAll(dir)
			if err = os.MkdirAll(filepath.Join(dir, "bin"), 0700); err != nil {
				t.Fatal(err)
			}
			if err = os.MkdirAll(filepath.Join(dir, "run"), 0755); err != nil {
				t.Fatal(err)
			}
			exe, err := os.Executable()
			if err != nil {
				t.Fatal(err)
			}
			t.Setenv("MOWGLI_TEST_WORKER", "1")
			t.Setenv("MOWGLI_TEST_SOCKET", filepath.Join(dir, "run", "updater.sock"))
			quote := func(s string) string { return "'" + strings.ReplaceAll(s, "'", "'\\''") + "'" }
			worker := func(name, version string, fail bool) string {
				p := filepath.Join(dir, "bin", name)
				flag := "0"
				if fail {
					flag = "1"
				}
				body := "#!/bin/sh\nexport MOWGLI_TEST_VERSION=" + quote(version) + " MOWGLI_TEST_CRASH=" + flag + "\nexec " + quote(exe) + " -test.run=^TestUpdaterWorkerHelper$\n"
				if e := os.WriteFile(p, []byte(body), 0755); e != nil {
					t.Fatal(e)
				}
				return p
			}
			original := worker("original", "previous", false)
			candidate := worker("candidate", "replacement", crash)
			active := filepath.Join(dir, "agent-active.json")
			pending := filepath.Join(dir, "agent-pending.json")
			if err = AtomicJSON(active, AgentSelection{Path: original, Version: "previous"}); err != nil {
				t.Fatal(err)
			}
			if err = AtomicJSON(pending, AgentSelection{Path: candidate, Version: "replacement", Previous: original}); err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
			done := make(chan error, 1)
			go func() { done <- supervise(ctx, "unused", HostConfig{StateDir: dir}, original) }()
			defer func() { cancel(); <-done }()
			for ctx.Err() == nil {
				if _, err = os.Stat(pending); os.IsNotExist(err) {
					data, e := os.ReadFile(active)
					if e != nil {
						t.Fatal(e)
					}
					var selection AgentSelection
					if e = json.Unmarshal(data, &selection); e != nil {
						t.Fatal(e)
					}
					if crash {
						if selection.Path != original || selection.Error == "" {
							t.Fatalf("did not restore previous worker: %+v", selection)
						}
					} else if selection.Path != candidate || selection.Version != "replacement" {
						t.Fatalf("did not commit replacement: %+v", selection)
					}
					return
				}
				time.Sleep(50 * time.Millisecond)
			}
			t.Fatal("supervisor did not resolve pending update")
		})
	}
}
