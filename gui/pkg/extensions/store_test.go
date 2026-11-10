//go:build linux

package extensions

import (
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"syscall"
	"testing"
)

func schemaTestState() State {
	s := NewState()
	s.Entries["test-service"] = Entry{Active: schemaTestRelease()}
	return s
}

func TestStoreSaveLoadRestart(t *testing.T) {
	root := filepath.Join(t.TempDir(), "state")
	store := Store{Root: root}
	if s, err := store.Load(); err != nil || len(s.Entries) != 0 {
		t.Fatalf("missing: %+v %v", s, err)
	}
	unlock, err := store.Lock()
	if err != nil {
		t.Fatal(err)
	}
	defer unlock()
	state := schemaTestState()
	if err := store.Save(state); err != nil {
		t.Fatal(err)
	}
	got, err := (Store{Root: root}).Load()
	if err != nil {
		t.Fatal(err)
	}
	if got.Entries["test-service"].Active.Digest != state.Entries["test-service"].Active.Digest {
		t.Fatal("release changed")
	}
	for path, mode := range map[string]os.FileMode{root: 0700, filepath.Join(root, "state.json"): 0600, filepath.Join(root, ".lock"): 0600} {
		info, err := os.Stat(path)
		if err != nil || info.Mode().Perm() != mode {
			t.Fatalf("permissions %s: %v %v", path, info, err)
		}
	}
	state.Entries = map[string]Entry{}
	state.Retained["test-service"] = Retained{Manifest: got.Entries["test-service"].Active.Bundle.Manifest, Binding: got.Entries["test-service"].Active.Binding}
	if err := store.Save(state); err != nil {
		t.Fatal(err)
	}
	loaded, err := store.Load()
	if err != nil || len(loaded.Entries) != 0 || len(loaded.Retained) != 1 {
		t.Fatalf("retained ownership lost: %+v %v", loaded, err)
	}
	files, err := os.ReadDir(root)
	if err != nil {
		t.Fatal(err)
	}
	if len(files) != 2 {
		t.Fatalf("temporary files leaked: %v", files)
	}
}

func TestStoreRejectsInvalidStateWithoutReplacingFile(t *testing.T) {
	store := Store{Root: filepath.Join(t.TempDir(), "state")}
	if err := store.Save(schemaTestState()); err != nil {
		t.Fatal(err)
	}
	baseline, _ := os.ReadFile(filepath.Join(store.Root, "state.json"))
	cases := map[string]func(*State){
		"schema": func(s *State) { s.Schema = 2 },
		"maps":   func(s *State) { s.Retained = nil },
		"digest": func(s *State) {
			e := s.Entries["test-service"]
			e.Active.Digest = "sha256:" + strings.Repeat("0", 64)
			s.Entries["test-service"] = e
		},
		"cross id": func(s *State) {
			e := s.Entries["test-service"]
			e.Active.Bundle.Manifest.ID = "second"
			e.Active.Digest = BundleDigest(e.Active.Bundle)
			s.Entries["second"] = e
		},
		"retained schema": func(s *State) {
			r := s.Entries["test-service"].Active
			r.Bundle.Manifest.DataSchema++
			s.Retained["test-service"] = Retained{Manifest: r.Bundle.Manifest, Binding: r.Binding}
		},
		"journal": func(s *State) { s.Journal = &Journal{Action: "execute", ID: "test-service", Phase: "applying"} },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			state := schemaTestState()
			mutate(&state)
			if store.Save(state) == nil {
				t.Fatal("invalid state accepted")
			}
			after, _ := os.ReadFile(filepath.Join(store.Root, "state.json"))
			if string(after) != string(baseline) {
				t.Fatal("failed save replaced good state")
			}
		})
	}
}

func TestStoreJournalSurvivesRestart(t *testing.T) {
	s := schemaTestState()
	before := s.Entries["test-service"]
	target := before
	target.Enabled = true
	s.Journal = &Journal{ID: "test-service", Action: "enable", Phase: "applying", Before: &before, Target: &target}
	store := Store{Root: filepath.Join(t.TempDir(), "state")}
	if err := store.Save(s); err != nil {
		t.Fatal(err)
	}
	got, err := store.Load()
	if err != nil {
		t.Fatal(err)
	}
	if got.Journal == nil || !got.Journal.Target.Enabled || got.Entries["test-service"].Enabled {
		t.Fatal("interrupted transaction was lost")
	}
}

func TestStoreProcessLock(t *testing.T) {
	if root := os.Getenv("EXTENSIONS_TEST_LOCK_ROOT"); root != "" {
		unlock, err := (Store{Root: root}).Lock()
		if err == nil {
			unlock()
			os.Exit(8)
		}
		return
	}
	store := Store{Root: filepath.Join(t.TempDir(), "state")}
	unlock, err := store.Lock()
	if err != nil {
		t.Fatal(err)
	}
	if second, err := store.Lock(); err == nil {
		second()
		t.Fatal("same-process lock contention accepted")
	}
	cmd := exec.Command(os.Args[0], "-test.run=^TestStoreProcessLock$")
	cmd.Env = append(os.Environ(), "EXTENSIONS_TEST_LOCK_ROOT="+store.Root)
	if out, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("child obtained lock or failed: %s %v", out, err)
	}
	unlock()
	unlock()
	next, err := store.Lock()
	if err != nil {
		t.Fatal(err)
	}
	next()
}

func TestStoreSymlinksAndUnsafeFiles(t *testing.T) {
	for _, name := range []string{"root", "ancestor", "state.json", ".lock", "hardlink", "directory", "fifo"} {
		t.Run(name, func(t *testing.T) {
			base := t.TempDir()
			real := filepath.Join(base, "real")
			if err := os.Mkdir(real, 0700); err != nil {
				t.Fatal(err)
			}
			store := Store{Root: real}
			switch name {
			case "root":
				store.Root = filepath.Join(base, "link")
				if err := os.Symlink(real, store.Root); err != nil {
					t.Fatal(err)
				}
			case "ancestor":
				link := filepath.Join(base, "link")
				if err := os.Symlink(real, link); err != nil {
					t.Fatal(err)
				}
				store.Root = filepath.Join(link, "nested")
			case "state.json", ".lock":
				if err := os.Symlink(filepath.Join(base, "outside"), filepath.Join(real, name)); err != nil {
					t.Fatal(err)
				}
			case "hardlink":
				outside := filepath.Join(base, "outside")
				if err := os.WriteFile(outside, []byte("keep"), 0600); err != nil {
					t.Fatal(err)
				}
				if err := os.Link(outside, filepath.Join(real, "state.json")); err != nil {
					t.Fatal(err)
				}
			case "fifo":
				if err := syscall.Mkfifo(filepath.Join(real, "state.json"), 0600); err != nil {
					t.Fatal(err)
				}
			case "directory":
				if err := os.Mkdir(filepath.Join(real, "state.json"), 0700); err != nil {
					t.Fatal(err)
				}
			}
			if name == ".lock" {
				if unlock, err := store.Lock(); err == nil {
					unlock()
					t.Fatal("unsafe lock accepted")
				}
			} else {
				if store.Save(NewState()) == nil {
					t.Fatal("unsafe state path accepted")
				}
				if _, err := store.Load(); err == nil {
					t.Fatal("unsafe state load accepted")
				}
			}
		})
	}
}

func TestStoreRejectsMalformedDiskState(t *testing.T) {
	store := Store{Root: t.TempDir()}
	raw, _ := json.Marshal(schemaTestState())
	for _, data := range []string{`null`, string(raw) + `{}`, strings.Replace(string(raw), `"schema":1`, `"schema":1,"schema":1`, 1), strings.Replace(string(raw), `"schema":1`, `"schema":1,"hooks":[]`, 1), strings.Repeat(" ", stateLimit+1)} {
		if err := os.WriteFile(filepath.Join(store.Root, "state.json"), []byte(data), 0600); err != nil {
			t.Fatal(err)
		}
		if _, err := store.Load(); err == nil {
			t.Fatal("malformed state accepted")
		}
	}
}

func TestStateRejectsCrossExtensionResources(t *testing.T) {
	for _, resource := range []string{"project", "config", "data", "unit"} {
		t.Run(resource, func(t *testing.T) {
			state := schemaTestState()
			second := schemaTestRelease()
			second.Bundle.Manifest.ID = "second"
			second.Binding = Binding{Project: "second", ConfigDir: "/etc/second", DataDir: "/var/lib/second"}
			switch resource {
			case "project":
				second.Binding.Project = state.Entries["test-service"].Active.Binding.Project
			case "config":
				second.Binding.ConfigDir = state.Entries["test-service"].Active.Binding.ConfigDir + "/nested"
			case "data":
				second.Binding.DataDir = state.Entries["test-service"].Active.Binding.DataDir
			case "unit":
				first := state.Entries["test-service"]
				first.Active.Bundle.Compose = nil
				first.Active.Bundle.Manifest.Runtime = Runtime{Kind: "external-systemd", Units: []string{"lora.service"}}
				first.Active.Bundle.Manifest.Profile = "external"
				first.Active.Digest = BundleDigest(first.Active.Bundle)
				state.Entries["test-service"] = first
				second.Bundle.Compose = nil
				second.Bundle.Manifest.Runtime = first.Active.Bundle.Manifest.Runtime
				second.Bundle.Manifest.Profile = "external"
			}
			second.Digest = BundleDigest(second.Bundle)
			state.Entries["second"] = Entry{Active: second}
			if validateState(state) == nil {
				t.Fatal("duplicate resource ownership accepted")
			}
		})
	}
}

func schemaTestVolumeRelease(id, physical string) Release {
	r := schemaTestRelease()
	r.Bundle.Manifest.ID = id
	r.Binding = Binding{Project: "robot-ext-" + id, ConfigDir: "/etc/extensions/" + id, DataDir: "/var/lib/extensions/" + id}
	raw, _ := json.Marshal(map[string]any{
		"services": map[string]any{"agent": map[string]any{
			"image":   "example@sha256:" + strings.Repeat("a", 64),
			"volumes": []map[string]any{{"type": "volume", "source": "data", "target": "/data"}},
		}},
		"volumes": map[string]any{"data": map[string]any{"name": physical, "external": true}},
	})
	r.Bundle.Compose = raw
	r.Digest = BundleDigest(r.Bundle)
	return r
}

func TestRetainedPhysicalVolumeOwnershipSurvivesRestart(t *testing.T) {
	r := schemaTestVolumeRelease("first", "physical-identity")
	identity, err := StorageIdentity(r)
	if err != nil {
		t.Fatal(err)
	}
	state := NewState()
	state.Retained["first"] = Retained{Manifest: r.Bundle.Manifest, Binding: r.Binding, StorageIdentity: identity, VolumeNames: VolumeClaims(r)}
	store := Store{Root: filepath.Join(t.TempDir(), "state")}
	if err := store.Save(state); err != nil {
		t.Fatal(err)
	}
	loaded, err := store.Load()
	if err != nil {
		t.Fatal(err)
	}
	retained := loaded.Retained["first"]
	if retained.StorageIdentity != identity || len(retained.VolumeNames) != 1 || retained.VolumeNames[0] != "physical-identity" {
		t.Fatalf("retained data identity lost: %+v", retained)
	}
	second := schemaTestVolumeRelease("second", "physical-identity")
	loaded.Entries["second"] = Entry{Active: second}
	if err := store.Save(loaded); err == nil {
		t.Fatal("another extension claimed retained physical volume")
	}
}

func TestStateRejectsInvalidRetainedStorage(t *testing.T) {
	r := schemaTestVolumeRelease("first", "physical-identity")
	identity, err := StorageIdentity(r)
	if err != nil {
		t.Fatal(err)
	}
	cases := map[string]func(*Retained){
		"digest":    func(r *Retained) { r.StorageIdentity = "not-sha256" },
		"duplicate": func(r *Retained) { r.VolumeNames = []string{"physical-identity", "physical-identity"} },
		"path":      func(r *Retained) { r.VolumeNames = []string{"../identity"} },
		"empty":     func(r *Retained) { r.VolumeNames = []string{""} },
	}
	for name, mutate := range cases {
		t.Run(name, func(t *testing.T) {
			retained := Retained{Manifest: r.Bundle.Manifest, Binding: r.Binding, StorageIdentity: identity, VolumeNames: VolumeClaims(r)}
			mutate(&retained)
			state := NewState()
			state.Retained["first"] = retained
			if validateState(state) == nil {
				t.Fatal("invalid retained storage accepted")
			}
		})
	}
}

func TestStateRejectsActivePhysicalVolumeCollision(t *testing.T) {
	state := NewState()
	for _, id := range []string{"first", "second"} {
		state.Entries[id] = Entry{Active: schemaTestVolumeRelease(id, "shared-identity")}
	}
	if validateState(state) == nil {
		t.Fatal("two extensions claimed one physical volume")
	}
}

func TestStateRejectsChangedPreviousDataLayout(t *testing.T) {
	for _, change := range []string{"physical-volume", "mount-target"} {
		t.Run(change, func(t *testing.T) {
			active := schemaTestVolumeRelease("first", "physical-identity")
			previous := active
			switch change {
			case "physical-volume":
				previous = schemaTestVolumeRelease("first", "other-identity")
			case "mount-target":
				previous.Bundle.Compose = []byte(strings.Replace(string(previous.Bundle.Compose), `"target":"/data"`, `"target":"/different"`, 1))
				previous.Digest = BundleDigest(previous.Bundle)
			}
			state := NewState()
			state.Entries["first"] = Entry{Active: active, Previous: &previous}
			if validateState(state) == nil {
				t.Fatal("changed previous data layout accepted")
			}
		})
	}
}

func TestStatePreservesRetainedStorageLayout(t *testing.T) {
	previous := schemaTestVolumeRelease("first", "original-identity")
	identity, err := StorageIdentity(previous)
	if err != nil {
		t.Fatal(err)
	}
	state := NewState()
	state.Retained["first"] = Retained{Manifest: previous.Bundle.Manifest, Binding: previous.Binding, StorageIdentity: identity, VolumeNames: VolumeClaims(previous)}
	current := schemaTestVolumeRelease("first", "replacement-identity")
	state.Entries["first"] = Entry{Active: current}
	if validateState(state) == nil {
		t.Fatal("registration changed retained data layout")
	}
	current = previous
	current.Bundle.Manifest.Version = "1.2.4"
	current.Digest = BundleDigest(current.Bundle)
	state.Entries["first"] = Entry{Active: current, Previous: &previous}
	if err := validateState(state); err != nil {
		t.Fatalf("same layout across release versions rejected: %v", err)
	}
}

func TestStoreLoadDoesNotRepairPermissions(t *testing.T) {
	store := Store{Root: filepath.Join(t.TempDir(), "state")}
	if err := store.Save(NewState()); err != nil {
		t.Fatal(err)
	}
	if err := os.Chmod(store.Root, 0755); err != nil {
		t.Fatal(err)
	}
	if _, err := store.Load(); err == nil {
		t.Fatal("unsafe root permissions accepted")
	}
	info, err := os.Stat(store.Root)
	if err != nil {
		t.Fatal(err)
	}
	if info.Mode().Perm() != 0755 {
		t.Fatal("read-only load changed root permissions")
	}
}
