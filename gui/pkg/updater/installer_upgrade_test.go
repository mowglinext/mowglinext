//go:build linux

package updater

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func TestInstallerHealthRequiresReplacementIdentity(t *testing.T) {
	for _, matches := range []bool{true, false} {
		t.Run(map[bool]string{true: "replacement", false: "old-worker"}[matches], func(t *testing.T) {
			dir, err := os.MkdirTemp("/tmp", "installer-health-")
			if err != nil {
				t.Fatal(err)
			}
			defer os.RemoveAll(dir)
			if err = os.Mkdir(filepath.Join(dir, "run"), 0755); err != nil {
				t.Fatal(err)
			}
			listener, err := net.Listen("unix", filepath.Join(dir, "run", "updater.sock"))
			if err != nil {
				t.Fatal(err)
			}
			server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				version := Version
				if !matches {
					version = "previous-worker"
				}
				_ = json.NewEncoder(w).Encode(map[string]any{"api": APIVersion, "agent": map[string]string{"version": version, "revision": Revision}})
			})}
			go server.Serve(listener)
			defer server.Close()
			budget := 5 * time.Second
			if !matches {
				budget = 50 * time.Millisecond
			}
			ctx, cancel := context.WithTimeout(context.Background(), budget)
			defer cancel()
			err = WaitInstallerWorker(ctx, HostConfig{StateDir: dir})
			if (err == nil) != matches {
				t.Fatalf("replacement health: %v", err)
			}
		})
	}
}

func TestInstallerSelectsNewWorkerAndRetainsPairedRecovery(t *testing.T) {
	dir := t.TempDir()
	c := HostConfig{StateDir: dir, Trusted: []string{"mowglinext/mowglinext"}}
	if err := os.MkdirAll(filepath.Join(dir, "bin"), 0700); err != nil {
		t.Fatal(err)
	}
	marker := filepath.Join(dir, "started")
	old := filepath.Join(dir, "bin", "old-worker")
	bootstrap := filepath.Join(dir, "new-bootstrap")
	for path, name := range map[string]string{old: "old", bootstrap: "bootstrap-old"} {
		if err := os.WriteFile(path, []byte("#!/bin/sh\nprintf '"+name+"' > '"+marker+"'\n"), 0755); err != nil {
			t.Fatal(err)
		}
	}
	state := State{Schema: StateSchema - 1, Policy: Policy{Source: fixture().Source}}
	if err := AtomicJSON(filepath.Join(dir, "state.json"), state); err != nil {
		t.Fatal(err)
	}
	before, _ := os.ReadFile(filepath.Join(dir, "state.json"))
	if err := AtomicJSON(filepath.Join(dir, "agent-active.json"), AgentSelection{Path: old, Version: "old"}); err != nil {
		t.Fatal(err)
	}
	candidate := []byte("#!/bin/sh\nprintf 'new' > '" + marker + "'\n")
	if err := SelectInstallerWorker(c, bootstrap, candidate); err != nil {
		t.Fatal(err)
	}
	after, _ := os.ReadFile(filepath.Join(dir, "state.json"))
	if string(before) != string(after) {
		t.Fatal("selection migrated the journal before worker startup")
	}
	backups, _ := filepath.Glob(filepath.Join(dir, "installer-backups", "*", "restore.json"))
	if len(backups) != 1 {
		t.Fatal("missing paired recovery backup", backups)
	}
	var backup struct {
		Files  map[string][]byte
		Worker string
	}
	data, _ := os.ReadFile(backups[0])
	if err := json.Unmarshal(data, &backup); err != nil {
		t.Fatal(err)
	}
	if string(backup.Files["state.json"]) != string(before) {
		t.Fatal("previous journal not preserved")
	}
	retained, err := os.ReadFile(backup.Worker)
	original, _ := os.ReadFile(old)
	if err != nil || string(retained) != string(original) {
		t.Fatal("previous executable not preserved", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_ = supervise(ctx, "unused", c, bootstrap)
	started, err := os.ReadFile(marker)
	if err != nil || string(started) != "new" {
		t.Fatalf("launched %q instead of installer replacement: %v", started, err)
	}
}

func TestInstallerSelectionRejectsUnsafeTransitions(t *testing.T) {
	for _, scenario := range []string{"future-schema", "pending-job", "pending-worker", "maintenance", "running-worker"} {
		t.Run(scenario, func(t *testing.T) {
			dir := t.TempDir()
			c := HostConfig{StateDir: dir, Trusted: []string{"mowglinext/mowglinext"}}
			s := State{Schema: StateSchema, Policy: Policy{Source: fixture().Source}}
			switch scenario {
			case "future-schema":
				s.Schema++
			case "pending-job":
				s.Job = &Job{Phase: "applying"}
			case "pending-worker", "maintenance":
				name := "maintenance"
				if scenario == "pending-worker" {
					name = "agent-pending.json"
				}
				if err := os.WriteFile(filepath.Join(dir, name), []byte("{}"), 0600); err != nil {
					t.Fatal(err)
				}
			case "running-worker":
				unlock, err := processLock(filepath.Join(dir, "worker.lock"))
				if err != nil {
					t.Fatal(err)
				}
				defer unlock()
			}
			if err := AtomicJSON(filepath.Join(dir, "state.json"), s); err != nil {
				t.Fatal(err)
			}
			if err := SelectInstallerWorker(c, filepath.Join(dir, "bootstrap"), []byte("replacement")); err == nil {
				t.Fatal("accepted unsafe transition")
			}
			if _, err := os.Stat(filepath.Join(dir, "agent-active.json")); !os.IsNotExist(err) {
				t.Fatal("changed worker selection")
			}
		})
	}
}
