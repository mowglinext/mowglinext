package updater

import (
	"context"
	"encoding/json"
	"net/http/httptest"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

func TestEveryDeclaredImageFamilyHasIndependentContract(t *testing.T) {
	for _, family := range []string{"mowgli-ros2", "mowglinext-gui", "gps", "lidar-ldlidar", "lidar-rplidar", "lidar-stl27l", "future-helper"} {
		t.Run(family, func(t *testing.T) {
			base := fixture()
			candidate := guiFixture()
			base.ComponentCompatibility = map[string]string{family: "interface-1"}
			candidate.ComponentCompatibility = map[string]string{family: "interface-1"}
			image := candidate.Images["gps"]
			image.Repository = "ghcr.io/" + base.Source.Repository + "/" + family
			candidate.Images[family] = image
			services := map[string]managedService{"selected": {Image: family}}
			if err := validateOverrides(base, map[string]Deployment{"selected": candidate}, services); err != nil {
				t.Fatal(err)
			}
			for _, mutation := range []func(*Deployment){
				func(d *Deployment) { d.ComponentCompatibility[family] = "interface-2" },
				func(d *Deployment) { delete(d.ComponentCompatibility, family) },
				func(d *Deployment) { delete(d.Images, family) },
				func(d *Deployment) { d.FirmwareProtocol++ },
				func(d *Deployment) { d.DataSchema++ },
				func(d *Deployment) { d.Source.Repository = "another/fork" },
			} {
				raw, _ := json.Marshal(candidate)
				var changed Deployment
				_ = json.Unmarshal(raw, &changed)
				mutation(&changed)
				if err := validateOverrides(base, map[string]Deployment{"selected": changed}, services); err == nil {
					t.Fatal("invalid component override accepted")
				}
			}
			if err := validateOverrides(base, map[string]Deployment{"mqtt": candidate}, services); err == nil {
				t.Fatal("local service accepted")
			}
			if err := validateOverrides(base, map[string]Deployment{"disabled-sensor": candidate}, services); err == nil {
				t.Fatal("absent optional service accepted")
			}
		})
	}
}

func TestMultipleOverridesPersistAndRollbackExactCombination(t *testing.T) {
	m, fake, _ := setup(t, "")
	base := fixture()
	other := guiFixture()
	base.ComponentCompatibility = map[string]string{"mowgli-ros2": "ros-1", "mowglinext-gui": "gui-1"}
	other.ComponentCompatibility = base.ComponentCompatibility
	b := &componentBackend{fakeBackend: fake, images: map[string]string{"gui": "old-gui", "mowgli": "old-ros"}}
	m.backend = b
	m.state.Releases = []Deployment{base, other}
	p, err := m.MakeServicePlan(context.Background(), base.ID, true, map[string]string{"mowgli": other.ID, "gui": other.ID})
	if err != nil {
		t.Fatal(err)
	}
	if len(p.Overrides) != 2 || p.Images["mowgli"] != other.ID+"-ros" || p.Images["gui"] != other.ID+"-gui" {
		t.Fatal(p)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	first := settled(t, m)
	if first.Job.Phase != "succeeded" {
		t.Fatal(first)
	}
	reopened, err := Open(filepath.Dir(m.path), m.trusted, b, m.source)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(reopened.Snapshot().Overrides, first.Overrides) {
		t.Fatal("lost component provenance")
	}
	p, err = m.MakePlan(context.Background(), base.ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	if s := settled(t, m); s.Job.Phase != "succeeded" || len(s.Overrides) != 0 {
		t.Fatal(s)
	}
	if _, err = m.Rollback(); err != nil {
		t.Fatal(err)
	}
	restored := settled(t, m)
	if !reflect.DeepEqual(restored.Overrides, first.Overrides) || b.images["mowgli"] != other.ID+"-ros" || b.images["gui"] != other.ID+"-gui" {
		t.Fatal(restored, b.images)
	}
}

func TestReleaseServiceChoicesPreserveInstallerVariants(t *testing.T) {
	b, err := ReadComposeBundle("../../../install/compose")
	if err != nil {
		t.Fatal(err)
	}
	choices, err := b.ServiceChoices()
	if err != nil {
		t.Fatal(err)
	}
	for _, lidar := range []string{"none", "ldlidar", "rplidar", "stl27l"} {
		selected := map[string]string{}
		for _, choice := range choices {
			match := true
			for key, value := range choice.When {
				actual := map[string]string{"gnss": "universal", "lidar": lidar}[key]
				if actual != value {
					match = false
				}
			}
			if match {
				if _, ok := selected[choice.Service]; ok {
					t.Fatal("duplicate service", choice)
				}
				selected[choice.Service] = choice.Image
			}
		}
		if selected["mowgli"] != "mowgli-ros2" || selected["gui"] != "mowglinext-gui" || selected["gps"] != "gps" {
			t.Fatal(selected)
		}
		if lidar == "none" {
			if _, ok := selected["lidar"]; ok {
				t.Fatal("disabled sensor selected")
			}
		} else if selected["lidar"] != "lidar-"+lidar {
			t.Fatal(selected)
		}
	}
}

func TestServicePlanHTTPRejectsLocalAndDuplicateOverrides(t *testing.T) {
	m, fake, _ := setup(t, "")
	m.backend = &componentBackend{fakeBackend: fake, images: map[string]string{"gui": "old-gui", "mowgli": "old-ros"}}
	base, other := fixture(), guiFixture()
	base.ComponentCompatibility = map[string]string{"mowgli-ros2": "ros-1", "mowglinext-gui": "gui-1"}
	other.ComponentCompatibility = base.ComponentCompatibility
	m.state.Releases = []Deployment{base, other}
	for _, tc := range []struct {
		body   string
		status int
	}{
		{`{"deployment":"` + base.ID + `","component_deployments":{"gui":"` + other.ID + `","mowgli":"` + other.ID + `"}}`, 200},
		{`{"deployment":"` + base.ID + `","component_deployments":{"mqtt":"` + other.ID + `"}}`, 409},
		{`{"deployment":"` + base.ID + `","gui_deployment":"` + other.ID + `","component_deployments":{"gui":"` + other.ID + `"}}`, 409},
	} {
		w := httptest.NewRecorder()
		m.Handler(HostConfig{}).ServeHTTP(w, httptest.NewRequest("POST", "/v1/plan", strings.NewReader(tc.body)))
		if w.Code != tc.status {
			t.Fatal(w.Code, w.Body.String())
		}
		if w.Code == 200 {
			var plan Plan
			if err := json.Unmarshal(w.Body.Bytes(), &plan); err != nil || len(plan.Overrides) != 2 {
				t.Fatal(plan, err)
			}
		}
	}
}
