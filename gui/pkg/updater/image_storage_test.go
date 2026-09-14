//go:build linux

package updater

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestPublishedImageStorageMatchesTargetCompose(t *testing.T) {
	dir := t.TempDir()
	if err := os.WriteFile(filepath.Join(dir, "docker"), []byte("#!/bin/sh\ncat \"$TEST_IMAGE_CONFIG\"\n"), 0755); err != nil {
		t.Fatal(err)
	}
	t.Setenv("PATH", dir+":"+os.Getenv("PATH"))
	config := filepath.Join(dir, "image.json")
	t.Setenv("TEST_IMAGE_CONFIG", config)
	b := DockerBackend{HostConfig{StateDir: dir}}
	for _, test := range []struct {
		name, volumes, mounts string
		reject                bool
	}{
		{"stateless", `{}`, `[]`, false},
		{"implicit data", `{"/data":{}}`, `[]`, true},
		{"declared persisted mount", `{"/db":{}}`, `[{"type":"bind","source":"/existing/db","target":"/db"}]`, false},
		{"removed target mount", `{"/db":{}}`, `[]`, true},
		{"declared readonly mount", `{"/data":{}}`, `[{"target":"/data","read_only":true}]`, false},
	} {
		t.Run(test.name, func(t *testing.T) {
			if err := os.WriteFile(config, []byte(`[{"Config":{"Volumes":`+test.volumes+`}}]`), 0600); err != nil {
				t.Fatal(err)
			}
			p := Plan{Images: map[string]string{"helper": "docker.io/example/helper@sha256:" + strings.Repeat("a", 64)}, Stack: &StackPlan{Compose: []byte(`{"services":{"helper":{"volumes":` + test.mounts + `}}}`)}}
			err := b.ValidateImageStorage(context.Background(), p)
			if (err != nil) != test.reject {
				t.Fatalf("storage decision: %v", err)
			}
		})
	}
}

type unsafeStorageBackend struct{ *fakeBackend }

func (b unsafeStorageBackend) ValidateImageStorage(context.Context, Plan) error {
	return errors.New("untracked /data")
}
func TestUnsafeImageStorageFailsBeforeMaintenance(t *testing.T) {
	m, b, _ := setup(t, "")
	m.backend = unsafeStorageBackend{b}
	p, err := m.MakePlan(context.Background(), fixture().ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	if s := settled(t, m); s.Job.Phase != "failed" || !strings.Contains(s.Job.Error, "untracked") {
		t.Fatal(s.Job)
	}
	for _, event := range b.events {
		if event == "gate" || event == "backup" || event == "apply-new" {
			t.Fatal("changed installation before validating storage", b.events)
		}
	}
}
