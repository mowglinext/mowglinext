package updater

import (
	"context"
	"encoding/json"
	"path/filepath"
	"strings"
	"testing"
)

func TestCustomImageInspectionEnforcesPlatformMaintenanceAndStorage(t *testing.T) {
	labels := map[string]string{"garden.mowgli.maintenance-api": "1", "garden.mowgli.updater-ui": "1", "garden.mowgli.image-contract": "1", "garden.mowgli.firmware-protocol": "6", "garden.mowgli.image-family": "mowglinext-gui", "garden.mowgli.deployment": "deployment-aaaaaaaaaaaa-42-1", "garden.mowgli.release": "v1.3.0", "org.opencontainers.image.source": "https://github.com/example/mower", "org.opencontainers.image.revision": strings.Repeat("a", 40), "org.opencontainers.image.created": "2026-09-08T09:00:00Z"}
	image := map[string]any{"Id": "sha256:" + strings.Repeat("b", 64), "Os": "linux", "Architecture": "arm64", "Created": "2026-09-08T09:00:00Z", "Config": map[string]any{"Labels": labels}}
	inspect := func(service string) error {
		data, _ := json.Marshal([]any{image})
		_, err := inspectCustomImage(data, "dev", "ghcr.io/example/gui@sha256:"+strings.Repeat("a", 64), service, "linux/arm64", containerInfo{})
		return err
	}
	if err := inspect("gui"); err != nil {
		t.Fatal(err)
	}
	delete(labels, "garden.mowgli.updater-ui")
	if inspect("gui") == nil {
		t.Fatal("pre-updater GUI accepted")
	}
	labels["garden.mowgli.updater-ui"] = "1"
	delete(labels, "garden.mowgli.image-contract")
	if inspect("gps") == nil {
		t.Fatal("unversioned image accepted")
	}
	labels["garden.mowgli.image-contract"] = "1"
	image["Architecture"] = "amd64"
	if inspect("gui") == nil {
		t.Fatal("wrong platform accepted")
	}
	image["Architecture"] = "arm64"
	delete(labels, "garden.mowgli.maintenance-api")
	if inspect("gui") == nil || inspect("mowgli") == nil {
		t.Fatal("missing maintenance accepted")
	}
	if err := inspect("gps"); err != nil {
		t.Fatal("sensor unnecessarily requires GUI maintenance label", err)
	}
	image["Config"] = map[string]any{"Labels": labels, "Volumes": map[string]any{"/new-storage": map[string]any{}}}
	if inspect("gps") == nil {
		t.Fatal("new untracked image volume accepted")
	}
}

func TestCustomPlanDoesNotRemainMatchedAfterSuccessAndCanReturnToRelease(t *testing.T) {
	m, b, _ := setup(t, "")
	m.backend = customBackend{b}
	base := fixture()
	m.state.Active = &base
	for _, service := range []string{"gui", "mowgli"} {
		p, err := m.MakeCustomPlan(context.Background(), map[string]string{service: "branch"}, true)
		if err != nil {
			t.Fatal(err)
		}
		if _, err = m.StartAcknowledged(p.ID, true); err != nil {
			t.Fatal(err)
		}
		if s := settled(t, m); s.Job.Phase != "succeeded" || s.Active.ID != base.ID {
			t.Fatal(s)
		}
	}
	if len(m.Snapshot().CustomImages) != 2 {
		t.Fatal("lost earlier component provenance")
	}
	if _, err := m.Rollback(); err != nil {
		t.Fatal(err)
	}
	if s := settled(t, m); s.Job.Phase != "rolled_back" || len(s.CustomImages) != 1 || s.CustomImages["gui"].Requested != "branch" {
		t.Fatal(s)
	}
	p, err := m.MakePlan(context.Background(), base.ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	if s := settled(t, m); s.Job.Phase != "succeeded" || len(s.CustomImages) != 0 {
		t.Fatal("published release did not clear custom mix", s)
	}
}

func TestCustomReferences(t *testing.T) {
	for _, input := range []string{"dev", "feat-gui-dashboard-improvements", "ghcr.io/example/mower/gui:branch", "https://ghcr.io/example/mower/gui:branch", "docker://registry.example:5000/mower:dev", "ghcr.io/example/gui@sha256:" + strings.Repeat("a", 64)} {
		if _, err := customReference(input, "ghcr.io/mowglinext/mowglinext/mowglinext-gui:dev"); err != nil {
			t.Errorf("%s: %v", input, err)
		}
	}
	for _, input := range []string{"", "--help", "http://registry/x:y", "https://user:secret@registry/x:y", "repo/image", "github.com/org/repo/tree/branch", "repo/image:tag?token=secret", "repo/image:tag\n--privileged", "repo/image@sha512:" + strings.Repeat("a", 128)} {
		if _, err := customReference(input, "ghcr.io/mowglinext/mowglinext/mowglinext-gui:dev"); err == nil {
			t.Errorf("accepted %q", input)
		}
	}
}

type customBackend struct{ *fakeBackend }

func (b customBackend) PlanCustomImages(_ context.Context, requested map[string]string) (map[string]CustomImage, int, error) {
	result := map[string]CustomImage{}
	for name, input := range requested {
		result[name] = CustomImage{Requested: input, Reference: "ghcr.io/example/" + name + "@sha256:" + strings.Repeat("a", 64), ImageID: "sha256:" + strings.Repeat("b", 64)}
	}
	return result, 6, nil
}

func TestCustomPlanKeepsUnselectedImagesAndRequiresAcknowledgement(t *testing.T) {
	m, b, _ := setup(t, "")
	m.backend = customBackend{b}
	if _, err := m.MakeCustomPlan(context.Background(), map[string]string{"gui": "dev"}, false); err == nil {
		t.Fatal("missing warning accepted")
	}
	p, err := m.MakeCustomPlan(context.Background(), map[string]string{"gui": "dev"}, true)
	if err != nil {
		t.Fatal(err)
	}
	if p.Images["mowgli"] != "old-ros" || len(p.CustomImages) != 1 || len(b.events) != 2 {
		t.Fatal("unexpected plan or mutation", p, b.events)
	}
	if _, err = m.Start(p.ID); err == nil {
		t.Fatal("installed without final custom warning acknowledgement")
	}
	if _, err = m.StartAcknowledged(p.ID, true); err != nil {
		t.Fatal(err)
	}
	s := settled(t, m)
	if s.Job.Phase != "succeeded" || s.Active != nil || len(s.CustomImages) != 1 {
		t.Fatal("custom installation was represented as a release", s)
	}
	r := reconcile(s, map[string]RunningComponent{"gui": {Image: "old-gui", Healthy: true}, "mowgli": {Image: "old-ros", Healthy: true}})
	if r.Identity != "mixed" {
		t.Fatal(r)
	}
	reloaded, err := Open(filepath.Dir(m.path), m.trusted, m.backend, m.source)
	if err != nil {
		t.Fatal(err)
	}
	if len(reloaded.Snapshot().CustomImages) != 1 {
		t.Fatal("lost custom provenance")
	}
	if _, err = m.Rollback(); err != nil {
		t.Fatal(err)
	}
	s = settled(t, m)
	if s.Job.Phase != "rolled_back" || len(s.CustomImages) != 0 || s.Active != nil {
		t.Fatal("incorrect custom rollback", s)
	}
}

func TestCustomFailureRestoresReleaseProvenance(t *testing.T) {
	m, b, _ := setup(t, "verify-new")
	m.backend = customBackend{b}
	base := fixture()
	m.state.Active = &base
	m.state.InstalledImages = map[string]string{"gui": "old-gui", "mowgli": "old-ros"}
	p, err := m.MakeCustomPlan(context.Background(), map[string]string{"gui": "branch"}, true)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.StartAcknowledged(p.ID, true); err != nil {
		t.Fatal(err)
	}
	s := settled(t, m)
	if s.Job.Phase != "rolled_back" || s.Active.ID != base.ID || len(s.CustomImages) != 0 {
		t.Fatal("lost previous release", s)
	}
}

func TestCustomMissingServiceRejected(t *testing.T) {
	m, b, _ := setup(t, "")
	m.backend = customBackend{b}
	if _, err := m.MakeCustomPlan(context.Background(), map[string]string{"mqtt": "latest"}, true); err == nil {
		t.Fatal("unmanaged service accepted")
	}
}
