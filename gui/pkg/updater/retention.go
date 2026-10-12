package updater

import (
	"context"
	"errors"
	"log"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"
)

// RetainedRollbacks is how many consecutive rollbacks stay possible. Each one
// costs a full archive of the mower data plus one generation of images, so an
// unbounded history fills the storage of a robot within a few updates.
const RetainedRollbacks = 2

const rollbackRepository = "mowgli-rollback"
const pruneTimeout = 10 * time.Minute

// Retention is what a finished job leaves worth keeping on the host.
type Retention struct {
	// Jobs names the backup directories (job IDs) a rollback can still select.
	Jobs map[string]bool
	// Images are references the retained jobs or the running stack still need.
	Images []string
	// Stale are references recorded by jobs no rollback can reach any more.
	Stale []string
}

// retainedJobs returns the committed jobs Rollback can still select, newest
// first: the active job and the chain of jobs it replaced. A job that failed,
// or that was itself rolled back, is never selectable again.
func retainedJobs(s State) []Job {
	committed := map[string]Job{}
	for _, job := range s.History {
		if job.Phase == "succeeded" && job.Backup != "" {
			committed[job.ID] = job
		}
	}
	kept := []Job{}
	if s.ActiveJobID != "" {
		for id := s.ActiveJobID; id != "" && len(kept) < RetainedRollbacks; {
			job, ok := committed[id]
			if !ok {
				break
			}
			kept = append(kept, job)
			id = job.PreviousJobID
		}
		return kept
	}
	// Journals written before transaction IDs were tracked.
	for i := len(s.History) - 1; i >= 0 && len(kept) < RetainedRollbacks; i-- {
		if job, ok := committed[s.History[i].ID]; ok {
			kept = append(kept, job)
		}
	}
	return kept
}

func jobImages(job Job) []string {
	images := []string{}
	for _, group := range []map[string]string{job.Plan.Images, job.Plan.Previous, job.PreviousImages} {
		for _, image := range group {
			images = append(images, image)
		}
	}
	return images
}

func sortedKeys(set map[string]bool) []string {
	keys := make([]string, 0, len(set))
	for key := range set {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	return keys
}

func retention(s State) Retention {
	jobs := map[string]bool{}
	needed := map[string]bool{}
	for _, image := range s.InstalledImages {
		needed[image] = true
	}
	for _, job := range retainedJobs(s) {
		jobs[filepath.Base(job.Backup)] = true
		for _, image := range jobImages(job) {
			needed[image] = true
		}
	}
	stale := map[string]bool{}
	for _, job := range s.History {
		for _, image := range jobImages(job) {
			if image != "" && !needed[image] {
				stale[image] = true
			}
		}
	}
	return Retention{Jobs: jobs, Images: sortedKeys(needed), Stale: sortedKeys(stale)}
}

// prune drops the recovery data no rollback can reach. The journal forgets a
// backup before the backend deletes it, so Rollback can never select an
// archive that is gone; a backend failure leaves files that the next run
// removes, because the backend works from what is on disk.
func (m *Manager) prune(ctx context.Context) error {
	backend, ok := m.backend.(interface {
		Prune(context.Context, Retention) error
	})
	if !ok {
		return nil
	}
	m.mu.Lock()
	if m.state.Job.Pending() {
		// An unfinished job may need any archive on disk to recover.
		m.mu.Unlock()
		return nil
	}
	keep := retention(m.state)
	changed := false
	for i := range m.state.History {
		if backup := m.state.History[i].Backup; backup != "" && !keep.Jobs[filepath.Base(backup)] {
			m.state.History[i].Backup = ""
			changed = true
		}
	}
	var err error
	if changed {
		err = m.save()
	}
	m.mu.Unlock()
	if err != nil {
		return err
	}
	return backend.Prune(ctx, keep)
}

// pruneAfterJob runs inside the job that just finished, under its deployment
// lock. Retention is housekeeping: a failure is logged, never a job failure.
func (m *Manager) pruneAfterJob() {
	ctx, cancel := context.WithTimeout(context.Background(), pruneTimeout)
	defer cancel()
	if err := m.prune(ctx); err != nil {
		log.Printf("updater: retention: %v", err)
	}
}

// Prune applies retention outside a job, e.g. at startup, so a host that
// accumulated recovery data under an older worker is cleaned without waiting
// for its next update. It does nothing while a job is running or pending.
func (m *Manager) Prune() {
	m.mu.Lock()
	if m.busy || m.checking || m.state.Job.Pending() {
		m.mu.Unlock()
		return
	}
	m.busy = true
	m.mu.Unlock()
	go func() {
		defer func() { m.mu.Lock(); m.busy = false; m.mu.Unlock() }()
		if backend, ok := m.backend.(interface{ Lock() (func(), error) }); ok {
			unlock, err := backend.Lock()
			if err != nil {
				log.Printf("updater: retention: %v", err)
				return
			}
			defer unlock()
		}
		m.pruneAfterJob()
	}()
}

// pruneBackups removes every job directory under root that is not retained.
// Anything that is not a plain job directory is left alone.
func pruneBackups(root string, keep map[string]bool) []error {
	entries, err := os.ReadDir(root)
	if os.IsNotExist(err) {
		return nil
	}
	if err != nil {
		return []error{err}
	}
	var errs []error
	for _, entry := range entries {
		if !entry.IsDir() || keep[entry.Name()] || !idPattern.MatchString(entry.Name()) {
			continue
		}
		if err = os.RemoveAll(filepath.Join(root, entry.Name())); err != nil {
			errs = append(errs, err)
		}
	}
	return errs
}

// splitRollbackTags reads "<image ID> <tag>" lines of the rollback repository.
// A tag is "<job ID>-<service>". It returns the tags of jobs that are no
// longer retained (tag -> image ID) and the image IDs retained tags still pin.
func splitRollbackTags(listing string, keep map[string]bool) (map[string]string, map[string]bool) {
	stale := map[string]string{}
	pinned := map[string]bool{}
	for _, line := range strings.Split(listing, "\n") {
		fields := strings.Fields(line)
		if len(fields) != 2 || fields[1] == "<none>" {
			continue
		}
		id, tag := fields[0], fields[1]
		retained := false
		for job := range keep {
			retained = retained || strings.HasPrefix(tag, job+"-")
		}
		if retained {
			pinned[id] = true
		} else {
			stale[tag] = id
		}
	}
	return stale, pinned
}

// parseImageIdentity reads "<image ID> [<RepoTags entry>...]". The containerd
// image store lists a digest-only pull ("repo@sha256:...") among RepoTags; it
// is not a name anyone gave the image, so only real tags are counted.
func parseImageIdentity(output string) (string, int, bool) {
	fields := strings.Fields(output)
	if len(fields) == 0 {
		return "", 0, false
	}
	tags := 0
	for _, name := range fields[1:] {
		if !strings.Contains(name, "@") && !strings.HasSuffix(name, ":<none>") {
			tags++
		}
	}
	return fields[0], tags, true
}

// imageIdentity returns the ID of a local image and how many tags name it.
func imageIdentity(ctx context.Context, reference string) (string, int, bool) {
	out, err := command(ctx, "docker", "image", "inspect", "--format", "{{.Id}}{{range .RepoTags}} {{.}}{{end}}", reference)
	if err != nil {
		return "", 0, false
	}
	return parseImageIdentity(string(out))
}

// Prune deletes the backups of unretained jobs, their rollback image tags, and
// the images only those jobs kept alive. An image is deleted only when the
// updater held it (a rollback tag, or a reference recorded in the journal),
// nothing retained needs it, and no tag names it any more: an image the
// operator tagged, or one another retained rollback still pins, stays. Docker
// itself refuses to delete an image a container uses.
func (b DockerBackend) Prune(ctx context.Context, keep Retention) error {
	errs := pruneBackups(filepath.Join(b.Config.StateDir, "backups"), keep.Jobs)
	listing, err := command(ctx, "docker", "image", "ls", "--no-trunc", "--format", "{{.ID}} {{.Tag}}", rollbackRepository)
	if err != nil {
		return errors.Join(append(errs, err)...)
	}
	staleTags, protected := splitRollbackTags(string(listing), keep.Jobs)
	for _, reference := range keep.Images {
		if id, _, ok := imageIdentity(ctx, reference); ok {
			protected[id] = true
		}
	}
	candidates := map[string]bool{}
	for _, reference := range keep.Stale {
		if id, _, ok := imageIdentity(ctx, reference); ok {
			candidates[id] = true
		}
	}
	for tag, id := range staleTags {
		candidates[id] = true
		if _, err = command(ctx, "docker", "image", "rm", rollbackRepository+":"+tag); err != nil {
			errs = append(errs, err)
		}
	}
	for _, id := range sortedKeys(candidates) {
		if protected[id] {
			continue
		}
		// Re-read after the untagging above: only a now tagless image goes.
		if _, tags, ok := imageIdentity(ctx, id); !ok || tags > 0 {
			continue
		}
		if _, err = command(ctx, "docker", "image", "rm", id); err != nil {
			errs = append(errs, err)
		}
	}
	return errors.Join(errs...)
}
