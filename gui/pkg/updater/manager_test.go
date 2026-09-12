package updater

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

type fakeBackend struct {
	mu          sync.Mutex
	events      []string
	fail        string
	fingerprint string
}

func (b *fakeBackend) event(s string) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.events = append(b.events, s)
	if b.fail == s {
		return errors.New("injected " + s)
	}
	return nil
}
func (b *fakeBackend) Inventory(context.Context) (string, map[string]string, error) {
	return b.fingerprint, map[string]string{"gui": "old-gui", "mowgli": "old-ros"}, b.event("inventory")
}
func (b *fakeBackend) PlanImages(context.Context, Deployment) (map[string]string, error) {
	return map[string]string{"gui": "new-gui", "mowgli": "new-ros"}, nil
}
func (b *fakeBackend) Pull(context.Context, map[string]string) error { return b.event("pull") }
func (b *fakeBackend) Maintenance(_ context.Context, active bool) error {
	if active {
		return b.event("gate")
	}
	return b.event("ungate")
}
func (b *fakeBackend) Backup(context.Context, string) (string, error) {
	return "backup", b.event("backup")
}
func (b *fakeBackend) Apply(_ context.Context, images map[string]string) error {
	if images["gui"] == "old-gui" {
		return b.event("apply-old")
	}
	return b.event("apply-new")
}
func (b *fakeBackend) Verify(_ context.Context, images map[string]string, _ *Deployment) error {
	if images["gui"] == "old-gui" {
		return b.event("verify-old")
	}
	return b.event("verify-new")
}
func (b *fakeBackend) Restore(context.Context, string) error { return b.event("restore") }

type fakeSource struct {
	calls    int
	releases []Deployment
	err      error
}

type ungatedBackend struct{ *fakeBackend }

func (ungatedBackend) MaintenanceSet() (bool, error) { return false, nil }

func TestRecoveryBeforeMaintenanceNeverRestartsMower(t *testing.T) {
	m, b, _ := setup(t, "gate")
	m.backend = ungatedBackend{b}
	p, err := m.MakePlan(context.Background(), fixture().ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	if settled(t, m).Job.Phase != "recovery_required" {
		t.Fatal("expected rejected readiness")
	}
	m.Recover()
	if settled(t, m).Job.Phase != "failed" {
		t.Fatal("expected untouched installation")
	}
	for _, event := range b.events {
		if strings.HasPrefix(event, "apply-") || event == "restore" || event == "backup" {
			t.Fatal("changed mower without entering maintenance", b.events)
		}
	}
}

func (s *fakeSource) List(context.Context, Source) ([]Deployment, error) {
	s.calls++
	return s.releases, s.err
}
func fixture() Deployment {
	d := Deployment{Schema: 1, ID: "deployment-123", Source: Source{Repository: "mowglinext/mowglinext", Track: "dev", Branch: "dev"}, Revision: strings.Repeat("a", 40), ReleaseTag: "deployment-123", PublishedAt: time.Now().UTC(), Layout: 1, DataSchema: 1, UpdaterAPI: 1, MaintenanceAPI: 1, FirmwareProtocol: 6, Images: map[string]updates.Image{}}
	for _, name := range []string{"gps", "mowgli-ros2", "mowglinext-gui"} {
		d.Images[name] = updates.Image{Repository: "ghcr.io/mowglinext/mowglinext/" + name, Digest: "sha256:" + strings.Repeat("b", 64), Platforms: map[string]updates.Platform{"linux/arm64": {Manifest: "sha256:" + strings.Repeat("c", 64), Config: "sha256:" + strings.Repeat("d", 64), Revision: d.Revision}}}
	}
	return d
}
func setup(t *testing.T, fail string) (*Manager, *fakeBackend, *fakeSource) {
	t.Helper()
	b := &fakeBackend{fail: fail, fingerprint: "original"}
	s := &fakeSource{releases: []Deployment{fixture()}}
	m, err := Open(t.TempDir(), []string{"mowglinext/mowglinext"}, b, s)
	if err != nil {
		t.Fatal(err)
	}
	if err = m.Check(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	return m, b, s
}
func settled(t *testing.T, m *Manager) State {
	t.Helper()
	deadline := time.Now().Add(3 * time.Second)
	for time.Now().Before(deadline) {
		m.mu.Lock()
		busy := m.busy
		m.mu.Unlock()
		if !busy {
			return m.Snapshot()
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatal("job did not settle")
	return State{}
}
func TestScheduledChecksPersistAndDoNotRunOnReads(t *testing.T) {
	m, _, s := setup(t, "")
	for i := 0; i < 5; i++ {
		m.Snapshot()
		_ = m.Check(context.Background(), false)
	}
	if s.calls != 1 {
		t.Fatal("cached reads caused checks")
	}
	reopened, e := Open(filepath.Dir(m.path), m.trusted, m.backend, s)
	if e != nil {
		t.Fatal(e)
	}
	_ = reopened.Check(context.Background(), false)
	if s.calls != 1 {
		t.Fatal("restart ignored persisted due time")
	}
	if reopened.Snapshot().LastSuccess.IsZero() {
		t.Fatal("lost cached success")
	}
}
func TestNotificationAcknowledgementSurvivesRestart(t *testing.T) {
	m, _, _ := setup(t, "")
	n := m.Snapshot().Notices[0]
	if err := m.Acknowledge(n.ID, true); err != nil {
		t.Fatal(err)
	}
	m2, e := Open(filepath.Dir(m.path), m.trusted, m.backend, m.source)
	if e != nil {
		t.Fatal(e)
	}
	if !m2.Snapshot().Notices[0].Dismissed {
		t.Fatal("dismissal lost")
	}
}
func TestCorruptStateIsNotReset(t *testing.T) {
	dir := t.TempDir()
	_ = os.WriteFile(filepath.Join(dir, "state.json"), []byte("{"), 0600)
	if _, err := Open(dir, []string{"mowglinext/mowglinext"}, nil, nil); err == nil {
		t.Fatal("corrupt recovery state accepted")
	}
}

func TestCorruptBackupRejectedBeforeDockerOrDataChanges(t *testing.T) {
	dir := t.TempDir()
	backup := filepath.Join(dir, "backups", "saved")
	if err := AtomicJSON(filepath.Join(backup, "backup.json"), backupRecord{Sources: []string{"/must-not-touch"}, Checksums: []string{strings.Repeat("0", 64)}}); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(backup, "0.tar"), []byte("corrupt archive"), 0600); err != nil {
		t.Fatal(err)
	}
	b := DockerBackend{HostConfig{StateDir: dir}}
	if err := b.Restore(context.Background(), backup); err == nil || !strings.Contains(err.Error(), "checksum mismatch") {
		t.Fatalf("expected rejection before Docker or data operations: %v", err)
	}
}
func TestInstallAndFailures(t *testing.T) {
	for _, test := range []struct {
		fail, phase string
		must, never []string
	}{
		{"", "succeeded", []string{"pull", "gate", "backup", "apply-new", "verify-new", "ungate"}, []string{"restore"}},
		{"pull", "failed", nil, []string{"gate", "backup", "apply-new"}},
		{"backup", "recovery_required", []string{"gate"}, []string{"apply-new", "ungate"}},
		{"apply-new", "rolled_back", []string{"restore", "apply-old", "verify-old", "ungate"}, nil},
		{"verify-new", "rolled_back", []string{"restore", "apply-old", "verify-old", "ungate"}, nil},
	} {
		t.Run(test.fail+test.phase, func(t *testing.T) {
			m, b, _ := setup(t, test.fail)
			p, e := m.MakePlan(context.Background(), fixture().ID, true)
			if e != nil {
				t.Fatal(e)
			}
			if _, e = m.Start(p.ID); e != nil {
				t.Fatal(e)
			}
			s := settled(t, m)
			if s.Job.Phase != test.phase {
				t.Fatalf("%s: %s", s.Job.Phase, s.Job.Error)
			}
			joined := strings.Join(b.events, ",")
			for _, x := range test.must {
				if !strings.Contains(joined, x) {
					t.Fatalf("missing %s: %s", x, joined)
				}
			}
			for _, x := range test.never {
				if strings.Contains(joined, x) {
					t.Fatalf("unexpected %s: %s", x, joined)
				}
			}
			if test.phase == "succeeded" && (s.Active == nil || !s.Policy.Pinned) {
				t.Fatal("successful deployment/pin not committed")
			}
		})
	}
}
func TestStalePlanNeverStopsContainers(t *testing.T) {
	m, b, _ := setup(t, "")
	p, e := m.MakePlan(context.Background(), fixture().ID, false)
	if e != nil {
		t.Fatal(e)
	}
	b.fingerprint = "changed"
	_, _ = m.Start(p.ID)
	s := settled(t, m)
	if s.Job.Phase != "failed" {
		t.Fatal(s.Job)
	}
	for _, e := range b.events {
		if e == "gate" || e == "pull" {
			t.Fatal(b.events)
		}
	}
}
func TestInterruptedApplyRecoversFromJournal(t *testing.T) {
	m, b, _ := setup(t, "")
	p, e := m.MakePlan(context.Background(), fixture().ID, false)
	if e != nil {
		t.Fatal(e)
	}
	m.state.Job = &Job{ID: "interrupted", Phase: "applying", Plan: p, Backup: "backup", PreviousPolicy: m.state.Policy}
	if e = m.save(); e != nil {
		t.Fatal(e)
	}
	m2, e := Open(filepath.Dir(m.path), m.trusted, b, m.source)
	if e != nil {
		t.Fatal(e)
	}
	m2.Recover()
	s := settled(t, m2)
	if s.Job.Phase != "rolled_back" {
		t.Fatal(s.Job)
	}
	if s.Active != nil {
		t.Fatal("failed target became active")
	}
}
func TestRollbackFailureKeepsMaintenance(t *testing.T) {
	m, b, _ := setup(t, "")
	p, e := m.MakePlan(context.Background(), fixture().ID, false)
	if e != nil {
		t.Fatal(e)
	}
	m.state.Job = &Job{ID: "interrupted", Phase: "applying", Plan: p, Backup: "backup", PreviousPolicy: m.state.Policy}
	b.fail = "restore"
	m.Recover()
	s := settled(t, m)
	if s.Job.Phase != "recovery_required" {
		t.Fatal(s.Job)
	}
	for _, e := range b.events {
		if e == "ungate" {
			t.Fatal("failed rollback released maintenance")
		}
	}
}

func TestCommittedUpdateOnlyRetriesMaintenanceRelease(t *testing.T) {
	m, b, _ := setup(t, "ungate")
	p, err := m.MakePlan(context.Background(), fixture().ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	s := settled(t, m)
	if s.Job.Phase != "recovery_required" || s.Job.Committed != "succeeded" {
		t.Fatal(s.Job)
	}
	b.fail = ""
	m2, err := Open(filepath.Dir(m.path), m.trusted, b, m.source)
	if err != nil {
		t.Fatal(err)
	}
	m2.Recover()
	s = settled(t, m2)
	if s.Job.Phase != "succeeded" || s.Active == nil || s.Active.ID != p.Target.ID {
		t.Fatal(s.Job)
	}
	for _, event := range b.events {
		if event == "restore" || event == "apply-old" {
			t.Fatal("committed deployment was rolled back", b.events)
		}
	}
}
func TestSourceAndDescriptorValidation(t *testing.T) {
	trusted := []string{"mowglinext/mowglinext"}
	d := fixture()
	if e := d.Validate(trusted); e != nil {
		t.Fatal(e)
	}
	d.Source = Source{Repository: trusted[0], Track: "custom", Branch: "feat/my-version"}
	if e := d.Validate(trusted); e != nil {
		t.Fatal(e)
	}
	for _, branch := range []string{"", "../dev", "dev\nmain", "-force"} {
		d.Source.Branch = branch
		if d.Validate(trusted) == nil {
			t.Fatalf("accepted %q", branch)
		}
	}
	d = fixture()
	d.Images["gps"] = updates.Image{Repository: "ghcr.io/other/repo/gps", Digest: "sha256:" + strings.Repeat("b", 64)}
	if d.Validate(trusted) == nil {
		t.Fatal("untrusted image accepted")
	}
}
