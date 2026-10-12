package updater

import (
	"context"
	"os"
	"path/filepath"
	"reflect"
	"sync"
	"testing"
)

func succeededJob(id, previous string) Job {
	return Job{ID: id, Kind: "containers", Phase: "succeeded", Backup: "/state/backups/" + id, PreviousJobID: previous,
		Plan: Plan{Images: map[string]string{"gui": "repo@new-" + id}, Previous: map[string]string{"gui": "sha256:old-" + id}}}
}

func TestRetentionKeepsTheTwoRollbacksReachableFromTheActiveJob(t *testing.T) {
	// Arrange: three committed updates, the last one active.
	state := State{ActiveJobID: "job-3", InstalledImages: map[string]string{"gui": "sha256:running"},
		History: []Job{succeededJob("job-1", ""), succeededJob("job-2", "job-1"), succeededJob("job-3", "job-2")}}

	// Act
	r := retention(state)

	// Assert
	if !reflect.DeepEqual(r.Jobs, map[string]bool{"job-3": true, "job-2": true}) {
		t.Fatal("unexpected retained jobs", r.Jobs)
	}
	if !reflect.DeepEqual(r.Stale, []string{"repo@new-job-1", "sha256:old-job-1"}) {
		t.Fatal("unexpected stale images", r.Stale)
	}
	for _, image := range []string{"sha256:running", "repo@new-job-3", "sha256:old-job-3", "sha256:old-job-2"} {
		found := false
		for _, kept := range r.Images {
			found = found || kept == image
		}
		if !found {
			t.Fatal("retained rollback image is not protected", image, r.Images)
		}
	}
}

func TestRetentionDropsBackupsNoRollbackCanReach(t *testing.T) {
	// Arrange: job-2 failed and rolled itself back, job-3 was undone by the
	// operator; job-1 is active again. Neither backup can be selected any more.
	failed := succeededJob("job-2", "job-1")
	failed.Phase = "rolled_back"
	undone := succeededJob("job-3", "job-1")
	restore := undone
	restore.ID, restore.Kind, restore.Phase = "restore-4", "rollback", "rolled_back"
	state := State{ActiveJobID: "job-1", History: []Job{succeededJob("job-1", ""), failed, undone, restore}}

	// Act
	r := retention(state)

	// Assert
	if !reflect.DeepEqual(r.Jobs, map[string]bool{"job-1": true}) {
		t.Fatal("unexpected retained jobs", r.Jobs)
	}
}

func TestRetentionWithoutAnActiveJobKeepsTheNewestCommittedUpdates(t *testing.T) {
	// Journals written before transaction IDs were tracked have no ActiveJobID.
	state := State{History: []Job{succeededJob("job-1", ""), succeededJob("job-2", ""), succeededJob("job-3", "")}}

	r := retention(state)

	if !reflect.DeepEqual(r.Jobs, map[string]bool{"job-3": true, "job-2": true}) {
		t.Fatal("unexpected retained jobs", r.Jobs)
	}
}

func TestPruneBackupsRemovesOnlyUnretainedJobDirectories(t *testing.T) {
	// Arrange
	root := t.TempDir()
	outside := t.TempDir()
	for _, name := range []string{"job-1", "job-2", "job-3"} {
		if err := os.MkdirAll(filepath.Join(root, name, "failed-0-1"), 0700); err != nil {
			t.Fatal(err)
		}
	}
	if err := os.WriteFile(filepath.Join(root, "note.txt"), []byte("x"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(outside, filepath.Join(root, "link")); err != nil {
		t.Fatal(err)
	}

	// Act
	errs := pruneBackups(root, map[string]bool{"job-2": true, "job-3": true})

	// Assert
	if len(errs) != 0 {
		t.Fatal(errs)
	}
	if _, err := os.Stat(filepath.Join(root, "job-1")); !os.IsNotExist(err) {
		t.Fatal("stale backup survived")
	}
	for _, name := range []string{"job-2", "job-3", "note.txt", "link"} {
		if _, err := os.Lstat(filepath.Join(root, name)); err != nil {
			t.Fatal("removed something that is not a stale backup", name)
		}
	}
	if _, err := os.Stat(outside); err != nil {
		t.Fatal("followed a symlink out of the backup directory")
	}
	if errs = pruneBackups(filepath.Join(root, "missing"), nil); len(errs) != 0 {
		t.Fatal("a missing backup directory is not an error", errs)
	}
}

func TestRollbackTagsAreSplitByRetainedJob(t *testing.T) {
	listing := "sha256:aaa job-1-gui\nsha256:bbb job-1-mowgli\nsha256:bbb job-2-mowgli\nsha256:ccc job-20-gui\n\nsha256:ddd <none>\n"

	stale, kept := splitRollbackTags(listing, map[string]bool{"job-2": true})

	if !reflect.DeepEqual(stale, map[string]string{"job-1-gui": "sha256:aaa", "job-1-mowgli": "sha256:bbb", "job-20-gui": "sha256:ccc"}) {
		t.Fatal("unexpected stale tags", stale)
	}
	if !reflect.DeepEqual(kept, map[string]bool{"sha256:bbb": true}) {
		t.Fatal("unexpected protected images", kept)
	}
}

func TestDigestOnlyReferencesAreNotTags(t *testing.T) {
	for output, want := range map[string]int{
		"sha256:aaa\n":                          0,
		"sha256:aaa repo@sha256:bbb\n":          0,
		"sha256:aaa repo:<none>\n":              0,
		"sha256:aaa repo@sha256:bbb repo:dev\n": 1,
	} {
		id, tags, ok := parseImageIdentity(output)
		if !ok || id != "sha256:aaa" || tags != want {
			t.Fatal("unexpected identity for", output, id, tags, ok)
		}
	}
	if _, _, ok := parseImageIdentity(""); ok {
		t.Fatal("empty inspection is not an image")
	}
}

type pruningBackend struct {
	*fakeBackend
	mu       sync.Mutex
	requests []Retention
}

func (b *pruningBackend) Backup(ctx context.Context, id string) (string, error) {
	_, err := b.fakeBackend.Backup(ctx, id)
	return filepath.Join("backups", id), err
}
func (b *pruningBackend) Prune(_ context.Context, r Retention) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.requests = append(b.requests, r)
	return nil
}

func install(t *testing.T, m *Manager) string {
	t.Helper()
	p, err := m.MakePlan(context.Background(), fixture().ID, false)
	if err != nil {
		t.Fatal(err)
	}
	id, err := m.Start(p.ID)
	if err != nil {
		t.Fatal(err)
	}
	if phase := settled(t, m).Job.Phase; phase != "succeeded" {
		t.Fatal("update did not succeed", phase)
	}
	return id
}

func TestCompletedUpdatePrunesAndForgetsUnreachableBackups(t *testing.T) {
	// Arrange
	m, fake, _ := setup(t, "")
	backend := &pruningBackend{fakeBackend: fake}
	m.backend = backend

	// Act: three committed updates in a row.
	first := install(t, m)
	second := install(t, m)
	third := install(t, m)

	// Assert
	state := m.Snapshot()
	backend.mu.Lock()
	defer backend.mu.Unlock()
	if len(backend.requests) != 3 {
		t.Fatal("expected one prune per completed update", len(backend.requests))
	}
	if want := map[string]bool{second: true, third: true}; !reflect.DeepEqual(backend.requests[2].Jobs, want) {
		t.Fatal("unexpected retained jobs", backend.requests[2].Jobs)
	}
	backups := map[string]string{}
	for _, job := range state.History {
		backups[job.ID] = job.Backup
	}
	if backups[first] != "" || backups[second] == "" || backups[third] == "" {
		t.Fatal("journal must offer only the retained rollbacks", backups)
	}
}

func TestPruneNeverRunsWhileAJobNeedsRecovery(t *testing.T) {
	// Arrange: an interrupted activation whose restore fails, so the archive
	// on disk is the only way back.
	m, fake, _ := setup(t, "")
	backend := &pruningBackend{fakeBackend: fake}
	m.backend = backend
	install(t, m)
	backend.mu.Lock()
	backend.requests = nil
	backend.mu.Unlock()
	p, err := m.MakePlan(context.Background(), fixture().ID, false)
	if err != nil {
		t.Fatal(err)
	}
	m.state.Job = &Job{ID: "interrupted", Phase: "applying", Plan: p, Backup: "backups/interrupted", PreviousPolicy: m.state.Policy}
	fake.fail = "restore"

	// Act
	m.Recover()
	state := settled(t, m)
	m.Prune()
	settled(t, m)

	// Assert
	if state.Job.Phase != "recovery_required" {
		t.Fatal("expected a job waiting for recovery", state.Job.Phase)
	}
	backend.mu.Lock()
	defer backend.mu.Unlock()
	if len(backend.requests) != 0 {
		t.Fatal("pruned recovery data while recovery is required")
	}
}

func TestDockerPruneDeletesOnlyImagesNothingElseNames(t *testing.T) {
	if os.Getenv("MOWGLI_UPDATER_DOCKER_TESTS") != "1" {
		t.Skip("set MOWGLI_UPDATER_DOCKER_TESTS=1 on a disposable Docker test host")
	}
	// Arrange: three distinct empty images.
	ctx := context.Background()
	build := func(name string, tags ...string) string {
		t.Helper()
		file := filepath.Join(t.TempDir(), "Dockerfile")
		if err := os.WriteFile(file, []byte("FROM scratch\nLABEL garden.mowgli.test="+name+"\n"), 0600); err != nil {
			t.Fatal(err)
		}
		args := []string{"build", "-q", "-f", file}
		for _, tag := range tags {
			args = append(args, "-t", tag)
		}
		if _, err := command(ctx, "docker", append(args, filepath.Dir(file))...); err != nil {
			t.Fatal(err)
		}
		id, _, ok := imageIdentity(ctx, tags[0])
		if !ok {
			t.Fatal("image was not built", name)
		}
		t.Cleanup(func() {
			for _, tag := range tags {
				_, _ = command(ctx, "docker", "image", "rm", tag)
			}
		})
		return id
	}
	orphan := build("orphan", "mowgli-rollback:job-1-gui")
	pinned := build("pinned", "mowgli-rollback:job-1-mowgli", "mowgli-rollback:job-2-mowgli")
	named := build("named", "mowgli-rollback:job-1-gps", "mowgli-retention-test:operator")
	journal := build("journal", "mowgli-retention-test:journal")
	state := t.TempDir()
	for _, job := range []string{"job-1", "job-2"} {
		if err := os.MkdirAll(filepath.Join(state, "backups", job), 0700); err != nil {
			t.Fatal(err)
		}
	}

	// Act
	err := DockerBackend{HostConfig{StateDir: state}}.Prune(ctx, Retention{Jobs: map[string]bool{"job-2": true}, Images: []string{pinned}, Stale: []string{journal, orphan}})

	// Assert
	if err != nil {
		t.Fatal(err)
	}
	if _, _, ok := imageIdentity(ctx, orphan); ok {
		t.Fatal("image held only by a stale rollback tag survived")
	}
	for name, id := range map[string]string{"pinned": pinned, "named": named, "journal": journal} {
		if _, _, ok := imageIdentity(ctx, id); !ok {
			t.Fatal("deleted an image something else still names:", name)
		}
	}
	listing, _ := command(ctx, "docker", "image", "ls", "--format", "{{.Tag}}", rollbackRepository)
	if got := string(listing); got != "job-2-mowgli\n" {
		t.Fatal("unexpected rollback tags left:", got)
	}
	if _, e := os.Stat(filepath.Join(state, "backups", "job-1")); !os.IsNotExist(e) {
		t.Fatal("stale backup survived")
	}
	if _, e := os.Stat(filepath.Join(state, "backups", "job-2")); e != nil {
		t.Fatal("retained backup removed")
	}
}
