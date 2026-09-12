package updater

import (
	"context"
	"encoding/json"
	"path/filepath"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

func TestExternalTopologyTransactionAndFailureRecovery(t *testing.T) {
	for _, failure := range []string{"", "verify-new"} {
		t.Run(failure, func(t *testing.T) {
			m, b, _ := setup(t, failure)
			m.backend = stackFakeBackend{b}
			d := externalFixture()
			m.state.Releases = []Deployment{d}
			plan, err := m.MakePlan(context.Background(), d.ID, false)
			if err != nil {
				t.Fatal(err)
			}
			if _, err = m.Start(plan.ID); err != nil {
				t.Fatal(err)
			}
			state := settled(t, m)
			expected := "succeeded"
			if failure != "" {
				expected = "rolled_back"
			}
			if state.Job.Phase != expected {
				t.Fatal(state.Job.Phase, state.Job.Error)
			}
			reopened, err := Open(filepath.Dir(m.path), m.trusted, m.backend, m.source)
			if err != nil {
				t.Fatal(err)
			}
			saved := reopened.Snapshot()
			if saved.Schema != 5 || saved.Job.Plan.Target.Images["helper"].Type != "external" || saved.Job.Plan.Target.Images["helper"].Digest != d.Images["helper"].Digest {
				t.Fatal("lost external recovery identity")
			}
			if failure != "" && !strings.Contains(strings.Join(b.events, ","), "restore,apply-old,verify-old,ungate") {
				t.Fatal(b.events)
			}
		})
	}
}

func externalFixture() Deployment {
	d := fixture()
	d.Schema = 3
	d.Bundle = &BundleAsset{Asset: "mowgli-compose.json", SHA256: strings.Repeat("e", 64)}
	d.Images["helper"] = updates.Image{Type: "external", Version: "2.0.22", Repository: "docker.io/library/example", Digest: "sha256:" + strings.Repeat("e", 64), Platforms: map[string]updates.Platform{
		"linux/arm64": {Manifest: "sha256:" + strings.Repeat("f", 64), Config: "sha256:" + strings.Repeat("1", 64)},
		"linux/amd64": {Manifest: "sha256:" + strings.Repeat("2", 64), Config: "sha256:" + strings.Repeat("3", 64)},
	}}
	return d
}

func TestExternalReleaseValidation(t *testing.T) {
	d := externalFixture()
	if err := d.Validate([]string{d.Source.Repository}); err != nil {
		t.Fatal(err)
	}
	for name, change := range map[string]func(*Deployment){
		"legacy schema":        func(d *Deployment) { d.Schema = 2 },
		"missing bundle":       func(d *Deployment) { d.Bundle = nil },
		"missing architecture": func(d *Deployment) { delete(d.Images["helper"].Platforms, "linux/arm64") },
		"mutable digest":       func(d *Deployment) { i := d.Images["helper"]; i.Digest = "latest"; d.Images["helper"] = i },
		"missing version":      func(d *Deployment) { i := d.Images["helper"]; i.Version = ""; d.Images["helper"] = i },
		"unapproved host": func(d *Deployment) {
			i := d.Images["helper"]
			i.Repository = "localhost/private/image"
			d.Images["helper"] = i
		},
		"GUI bypass":        func(d *Deployment) { d.Images["mowglinext-gui"] = d.Images["helper"] },
		"robot bypass":      func(d *Deployment) { d.Images["mowgli-ros2"] = d.Images["helper"] },
		"unknown type":      func(d *Deployment) { i := d.Images["helper"]; i.Type = "typo"; d.Images["helper"] = i },
		"implicit external": func(d *Deployment) { i := d.Images["helper"]; i.Type = ""; d.Images["helper"] = i },
		"built source mismatch": func(d *Deployment) {
			i := d.Images["gps"]
			p := i.Platforms["linux/arm64"]
			p.Revision = ""
			i.Platforms["linux/arm64"] = p
		},
	} {
		t.Run(name, func(t *testing.T) {
			d := externalFixture()
			change(&d)
			if d.Validate([]string{d.Source.Repository}) == nil {
				t.Fatal("accepted invalid release")
			}
		})
	}
}

func TestExternalImageRemainsStandardAndRetainsProvenance(t *testing.T) {
	d := externalFixture()
	image := d.Images["helper"]
	id := image.Platforms["linux/arm64"].Config
	state := State{Schema: StateSchema, Active: &d, InstalledImages: map[string]string{"helper": id}, History: []Job{{Plan: Plan{Target: d}}}}
	encoded, err := json.Marshal(state)
	if err != nil {
		t.Fatal(err)
	}
	var restored State
	if err = json.Unmarshal(encoded, &restored); err != nil {
		t.Fatal(err)
	}
	if restored.History[0].Plan.Target.Images["helper"].Type != "external" {
		t.Fatal("lost rollback metadata")
	}
	components := map[string]RunningComponent{"helper": {Image: id, Family: "helper", Healthy: true}}
	r := reconcile(restored, components)
	if r.Identity != "matched" || r.Components["helper"].Version != "2.0.22" {
		t.Fatal(r)
	}
	components = map[string]RunningComponent{"helper": {Image: "wrong", Family: "helper", Healthy: true}}
	r = reconcile(restored, components)
	if r.Identity != "drifted" || r.Components["helper"].Version != "" {
		t.Fatal("assigned approved version to different image", r)
	}
}
