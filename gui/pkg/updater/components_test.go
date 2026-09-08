package updater

import (
	"context"
	"encoding/json"
	"net/http/httptest"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"
)

func serviceFixture() composeConfig {
	return composeConfig{Services: map[string]serviceConfig{
		"mowgli": {ContainerName: "test-ros"}, "gui": {ContainerName: "test-gui"},
		"mqtt":   {ContainerName: "test-mqtt"},
		"camera": {ContainerName: "test-camera", Labels: map[string]string{updateLabel + "image": "camera", updateLabel + "after": "mowgli"}},
	}}
}

func TestDeclaredServicesOrderAndOptionalMembership(t *testing.T) {
	c := serviceFixture()
	services, err := managedServices(c)
	if err != nil {
		t.Fatal(err)
	}
	if len(services) != 3 || services["camera"].Image != "camera" {
		t.Fatal(services)
	}
	order, err := serviceOrder(services)
	if err != nil || !reflect.DeepEqual(order, []string{"mowgli", "camera", "gui"}) {
		t.Fatal(order, err)
	}
	delete(c.Services, "camera")
	services, err = managedServices(c)
	if err != nil || len(services) != 2 {
		t.Fatal("absent optional service must not be added", services, err)
	}
}

func TestInvalidServiceContractsFailClosed(t *testing.T) {
	for _, mutation := range []func(*composeConfig){
		func(c *composeConfig) { c.Services["camera"].Labels[updateLabel+"after"] = "missing" },
		func(c *composeConfig) { c.Services["camera"].Labels[updateLabel+"after"] = "camera" },
		func(c *composeConfig) { c.Services["camera"].Labels[updateLabel+"image"] = "../../foreign" },
		func(c *composeConfig) { c.Services["camera"].Labels[updateLabel+"health"] = "shell command" },
		func(c *composeConfig) {
			c.Services["gps"] = serviceConfig{ContainerName: "gps", Labels: map[string]string{updateLabel + "health": "container"}}
		},
		func(c *composeConfig) {
			c.Services["mowgli"] = serviceConfig{ContainerName: "ros", Labels: map[string]string{updateLabel + "after": "gui"}}
		},
	} {
		c := serviceFixture()
		mutation(&c)
		if _, err := managedServices(c); err == nil {
			t.Fatal("accepted invalid contract", c)
		}
	}
}

func TestAdditionalFirstPartyImageValidation(t *testing.T) {
	d := fixture()
	image := d.Images["gps"]
	image.Repository = "ghcr.io/mowglinext/mowglinext/camera"
	d.Images["camera"] = image
	if err := d.Validate([]string{d.Source.Repository}); err != nil {
		t.Fatal(err)
	}
	image.Repository = "ghcr.io/another/project/camera"
	d.Images["camera"] = image
	if err := d.Validate([]string{d.Source.Repository}); err == nil {
		t.Fatal("foreign image accepted")
	}
}

func TestRuntimeIdentityIndependentFromHealth(t *testing.T) {
	d := fixture()
	s := State{Active: &d, InstalledImages: map[string]string{"gui": "gui-id", "mowgli": "ros-id"}}
	c := map[string]RunningComponent{"gui": {Image: "gui-id", Healthy: true}, "mowgli": {Image: "ros-id", Healthy: true}}
	if r := reconcile(s, c); r.Identity != "matched" || r.Health != "healthy" {
		t.Fatal(r)
	}
	c["gui"] = RunningComponent{Image: "gui-id", Healthy: false}
	if r := reconcile(s, c); r.Identity != "matched" || r.Health != "degraded" {
		t.Fatal(r)
	}
	s.Overrides = map[string]Deployment{"gui": d}
	if r := reconcile(s, c); r.Identity != "mixed" {
		t.Fatal(r)
	}
	c["gui"] = RunningComponent{Image: "manual-change", Healthy: true}
	if r := reconcile(s, c); r.Identity != "drifted" || r.Health != "healthy" {
		t.Fatal(r)
	}
	delete(c, "mowgli")
	if r := reconcile(s, c); r.Identity != "drifted" {
		t.Fatal(r)
	}
	s.InstalledImages = nil
	if r := reconcile(s, c); r.Identity != "unverified" {
		t.Fatal(r)
	}
	s.Active = nil
	if r := reconcile(s, c); r.Identity != "custom" {
		t.Fatal(r)
	}
}

func guiFixture() Deployment {
	d := fixture()
	d.ID = "gui-new"
	d.ReleaseTag = d.ID
	d.GUICompatibility = "ros-gui-1"
	d.Revision = strings.Repeat("e", 40)
	for name, image := range d.Images {
		for platform, p := range image.Platforms {
			p.Revision = d.Revision
			image.Platforms[platform] = p
		}
		d.Images[name] = image
	}
	return d
}

type componentBackend struct {
	*fakeBackend
	images map[string]string
}

func (b *componentBackend) PlanImages(_ context.Context, d Deployment) (map[string]string, error) {
	return map[string]string{"gui": d.ID + "-gui", "mowgli": d.ID + "-ros"}, nil
}
func (b *componentBackend) Inventory(context.Context) (string, map[string]string, error) {
	images := map[string]string{}
	for k, v := range b.images {
		images[k] = v
	}
	return b.fingerprint, images, nil
}
func (b *componentBackend) Apply(_ context.Context, images map[string]string) error {
	b.images = images
	return nil
}
func (b *componentBackend) Verify(context.Context, map[string]string, *Deployment) error { return nil }

func TestGUIOverridePersistsAndRollbackRestoresCombination(t *testing.T) {
	m, fake, _ := setup(t, "")
	base := fixture()
	base.GUICompatibility = "ros-gui-1"
	gui := guiFixture()
	m.state.Releases = []Deployment{base, gui}
	b := &componentBackend{fakeBackend: fake, images: map[string]string{"gui": "old-gui", "mowgli": "old-ros"}}
	m.backend = b
	p, err := m.MakeComponentPlan(context.Background(), base.ID, true, gui.ID)
	if err != nil {
		t.Fatal(err)
	}
	if p.Images["gui"] != "gui-new-gui" || p.Images["mowgli"] != base.ID+"-ros" || p.Overrides["gui"].ID != gui.ID {
		t.Fatal(p)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	first := settled(t, m)
	if first.Job.Phase != "succeeded" || first.Overrides["gui"].ID != gui.ID {
		t.Fatal(first)
	}
	reopened, err := Open(filepath.Dir(m.path), m.trusted, b, m.source)
	if err != nil || reopened.Snapshot().Overrides["gui"].ID != gui.ID {
		t.Fatal("lost override", err)
	}
	// Returning to the matched release clears the override, but rollback restores
	// that exact prior mixed transaction even though both have the same base ID.
	p, err = m.MakePlan(context.Background(), base.ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	second := settled(t, m)
	if len(second.Overrides) != 0 || second.ActiveJobID == first.ActiveJobID {
		t.Fatal(second)
	}
	if _, err = m.Rollback(); err != nil {
		t.Fatal(err)
	}
	restored := settled(t, m)
	if restored.Overrides["gui"].ID != gui.ID || restored.ActiveJobID != first.ActiveJobID || b.images["gui"] != "gui-new-gui" {
		t.Fatal(restored, b.images)
	}
	if _, err = m.Rollback(); err != nil {
		t.Fatal(err)
	}
	original := settled(t, m)
	if original.Active != nil || b.images["gui"] != "old-gui" {
		t.Fatal(original, b.images)
	}
}

func TestGUIOverrideRejectsMissingOrIncompatibleContracts(t *testing.T) {
	for _, mutate := range []func(*Deployment){
		func(d *Deployment) { d.GUICompatibility = "" },
		func(d *Deployment) { d.GUICompatibility = "ros-gui-2" },
		func(d *Deployment) { d.FirmwareProtocol++ },
		func(d *Deployment) { d.DataSchema++ },
		func(d *Deployment) { d.Source.Track = "custom"; d.Source.Branch = "feature" },
	} {
		m, fake, _ := setup(t, "")
		m.backend = &componentBackend{fakeBackend: fake, images: map[string]string{"gui": "old-gui", "mowgli": "old-ros"}}
		base := fixture()
		base.GUICompatibility = "ros-gui-1"
		gui := guiFixture()
		mutate(&gui)
		m.state.Releases = []Deployment{base, gui}
		if _, err := m.MakeComponentPlan(context.Background(), base.ID, false, gui.ID); err == nil {
			t.Fatal("incompatible GUI accepted", gui)
		}
	}
}

func TestComponentPlanHTTPAndReadOnlyRuntime(t *testing.T) {
	m, fake, source := setup(t, "")
	m.backend = &componentBackend{fakeBackend: fake, images: map[string]string{"gui": "old-gui", "mowgli": "old-ros"}}
	base := fixture()
	base.GUICompatibility = "ros-gui-1"
	gui := guiFixture()
	m.state.Releases = []Deployment{base, gui}
	h := m.Handler(HostConfig{})
	w := httptest.NewRecorder()
	h.ServeHTTP(w, httptest.NewRequest("POST", "/v1/plan", strings.NewReader(`{"deployment":"`+base.ID+`","gui_deployment":"`+gui.ID+`","pinned":true}`)))
	if w.Code != 200 {
		t.Fatal(w.Code, w.Body.String())
	}
	var p Plan
	if err := json.Unmarshal(w.Body.Bytes(), &p); err != nil || p.Overrides["gui"].ID != gui.ID {
		t.Fatal(p, err)
	}
	w = httptest.NewRecorder()
	h.ServeHTTP(w, httptest.NewRequest("GET", "/v1/state", nil))
	if w.Code != 200 || source.calls != 1 || !strings.Contains(w.Body.String(), `"identity":"unknown"`) {
		t.Fatal(w.Body.String(), source.calls)
	}
}

func TestLegacyJournalMigrationPreservesRecoveryHistory(t *testing.T) {
	m, _, _ := setup(t, "")
	old := m.Snapshot()
	old.Schema = 1
	old.History = []Job{{ID: "legacy-job", Phase: "succeeded", Backup: "retained-backup", Plan: Plan{Target: fixture()}}}
	if err := AtomicJSON(m.path, old); err != nil {
		t.Fatal(err)
	}
	reopened, err := Open(filepath.Dir(m.path), m.trusted, m.backend, m.source)
	if err != nil {
		t.Fatal(err)
	}
	if err = reopened.Configure(old.Policy); err != nil {
		t.Fatal(err)
	}
	current := reopened.Snapshot()
	if current.Schema != StateSchema || len(current.History) != 1 || current.History[0].Backup != "retained-backup" {
		t.Fatal(current)
	}
}

func TestRuntimeCacheNeverConfirmsStaleOrBusyState(t *testing.T) {
	m, _, _ := setup(t, "")
	m.runtime = RuntimeStatus{Identity: "matched", Health: "healthy", CheckedAt: m.now()}
	if m.Runtime().Identity != "matched" {
		t.Fatal(m.Runtime())
	}
	m.runtime.CheckedAt = m.now().Add(-2 * time.Minute)
	if m.Runtime().Identity != "unknown" || m.Runtime().Health != "unknown" {
		t.Fatal(m.Runtime())
	}
	m.runtime.CheckedAt = m.now()
	m.busy = true
	if m.Runtime().Identity != "unknown" {
		t.Fatal(m.Runtime())
	}
}

func TestWorkerDowngradeCannotDiscardComponentProvenance(t *testing.T) {
	for _, probe := range []string{
		`{"version":"candidate","api":1}`,
		`{"version":"candidate","api":1,"state_schema":1}`,
		`{"version":"candidate","api":1,"state_schema":3}`,
		`{"version":"wrong","api":1,"state_schema":4}`,
		`{"version":"candidate","api":2,"state_schema":4}`,
	} {
		if validateWorkerProbe([]byte(probe), "candidate") == nil {
			t.Fatal("accepted incompatible worker", probe)
		}
	}
	if err := validateWorkerProbe([]byte(`{"version":"candidate","api":1,"state_schema":4}`), "candidate"); err != nil {
		t.Fatal(err)
	}
}

func (b *componentBackend) PlanSelectedImages(ctx context.Context, d Deployment, overrides map[string]Deployment) (map[string]string, error) {
	services := map[string]managedService{"gui": {Image: "mowglinext-gui"}, "mowgli": {Image: "mowgli-ros2"}}
	if err := validateOverrides(d, overrides, services); err != nil {
		return nil, err
	}
	images, _ := b.PlanImages(ctx, d)
	for service, selected := range overrides {
		values, _ := b.PlanImages(ctx, selected)
		images[service] = values[service]
	}
	return images, nil
}
