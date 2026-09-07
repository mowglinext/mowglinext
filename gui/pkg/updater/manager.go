package updater

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

type Backend interface {
	Inventory(context.Context) (fingerprint string, images map[string]string, err error)
	PlanImages(context.Context, Deployment) (map[string]string, error)
	Pull(context.Context, map[string]string) error
	Maintenance(context.Context, bool) error
	Backup(context.Context, string) (string, error)
	Apply(context.Context, map[string]string) error
	Verify(context.Context, map[string]string, *Deployment) error
	Restore(context.Context, string) error
}
type ReleaseSource interface {
	List(context.Context, Source) ([]Deployment, error)
}
type Manager struct {
	runtime  RuntimeStatus
	mu       sync.Mutex
	state    State
	path     string
	trusted  []string
	backend  Backend
	source   ReleaseSource
	checking bool
	busy     bool
	now      func() time.Time
}

func Open(dir string, trusted []string, b Backend, source ReleaseSource) (*Manager, error) {
	if len(trusted) == 0 {
		return nil, errors.New("no trusted sources configured")
	}
	m := &Manager{path: filepath.Join(dir, "state.json"), trusted: trusted, backend: b, source: source, now: time.Now}
	m.state = State{Schema: StateSchema, Policy: Policy{Source: Source{Repository: trusted[0], Track: "dev", Branch: "dev"}, IntervalHours: 4}, Releases: []Deployment{}, Notices: []Notice{}, Plans: []Plan{}, History: []Job{}}
	data, err := os.ReadFile(m.path)
	if err == nil {
		if err = json.Unmarshal(data, &m.state); err != nil || (m.state.Schema != 1 && m.state.Schema != StateSchema) {
			return nil, errors.New("invalid updater state; refusing to reset recovery history")
		}
	} else if !os.IsNotExist(err) {
		return nil, err
	}
	if err = m.state.Policy.Source.Validate(trusted); err != nil {
		return nil, err
	}
	return m, nil
}
func (m *Manager) save() error {
	// Older workers must refuse this journal rather than discard component
	// provenance or select a rollback by the base release ID alone.
	m.state.Schema = StateSchema
	return AtomicJSON(m.path, m.state)
}
func (m *Manager) Snapshot() State {
	m.mu.Lock()
	defer m.mu.Unlock()
	data, _ := json.Marshal(m.state)
	var s State
	_ = json.Unmarshal(data, &s)
	return s
}
func validInterval(n int) bool { return n == 0 || n == 1 || n == 4 || n == 24 }
func (m *Manager) Configure(p Policy) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.busy || m.state.Job.Pending() {
		return errors.New("update or recovery in progress")
	}
	if err := p.Source.Validate(m.trusted); err != nil {
		return err
	}
	if !validInterval(p.IntervalHours) {
		return errors.New("invalid check interval")
	}
	// This sets notification policy only; installed source/pin is stored with the
	// successful deployment and is not changed by browsing another source.
	if p.Source != m.state.Policy.Source {
		m.state.Releases = []Deployment{}
		m.state.LastCheck = time.Time{}
	}
	m.state.Policy = p
	m.state.NextCheck = m.now()
	return m.save()
}
func (m *Manager) Check(ctx context.Context, force bool) error {
	m.mu.Lock()
	if m.checking || m.busy || m.state.Job.Pending() {
		m.mu.Unlock()
		return errors.New("updater is busy")
	}
	now := m.now()
	if !m.state.LastCheck.IsZero() && now.Sub(m.state.LastCheck) < time.Minute {
		m.mu.Unlock()
		return nil
	}
	if !force && (m.state.Policy.IntervalHours == 0 || now.Before(m.state.NextCheck)) {
		m.mu.Unlock()
		return nil
	}
	m.checking = true
	policy := m.state.Policy
	active := m.state.Active
	m.mu.Unlock()
	releases, err := m.source.List(ctx, policy.Source)
	relation := "unknown"
	if err == nil && active != nil && len(releases) > 0 && active.Source == releases[0].Source {
		if c, ok := m.source.(interface {
			Compare(context.Context, string, string, string) string
		}); ok {
			relation = c.Compare(ctx, active.Source.Repository, active.Revision, releases[0].Revision)
		}
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	m.checking = false
	// A changed selection must not receive another source's completed request.
	if policy.Source != m.state.Policy.Source {
		return nil
	}
	m.state.LastCheck = now
	delay := time.Duration(policy.IntervalHours) * time.Hour
	if delay == 0 {
		delay = 4 * time.Hour
	}
	if err != nil {
		m.state.CheckError = err.Error()
		delay = 15 * time.Minute
	} else {
		m.state.CheckError = ""
		m.state.LastSuccess = now
		m.state.Releases = releases
		for i := range m.state.Notices {
			if len(releases) == 0 || m.state.Notices[i].Deployment != releases[0].ID {
				m.state.Notices[i].Dismissed = true
			}
		}
		if len(releases) > 0 && (m.state.Active == nil || m.state.Active.ID != releases[0].ID || len(m.state.Overrides) > 0 || m.runtime.Identity == "drifted") {
			target := releases[0]
			kind := "review"
			if relation == "newer" {
				kind = "available"
			}
			id := updates.Hash([]byte(target.Source.Repository + "/" + target.Source.Branch + "/" + target.ID))
			found := false
			for _, n := range m.state.Notices {
				if n.ID == id {
					found = true
				}
			}
			if !found {
				m.state.Notices = append(m.state.Notices, Notice{ID: id, Kind: kind, Deployment: target.ID, CreatedAt: now})
			}
		}
		if len(releases) > 0 {
			target := releases[0]
			binary, ok := target.Updater[runtime.GOOS+"/"+runtime.GOARCH]
			if ok && binary.Version != Version {
				id := updates.Hash([]byte(target.Source.Repository + "/updater/" + binary.Version))
				found := false
				for _, notice := range m.state.Notices {
					if notice.ID == id {
						found = true
					}
				}
				if !found {
					m.state.Notices = append(m.state.Notices, Notice{ID: id, Kind: "updater", Deployment: target.ID, CreatedAt: now})
				}
			} else {
				for i := range m.state.Notices {
					if m.state.Notices[i].Kind == "updater" {
						m.state.Notices[i].Dismissed = true
					}
				}
			}
		}
	}
	// Jitter is persisted, not recomputed on every UI request or restart.
	jitter := time.Duration(now.UnixNano() % int64(15*time.Minute))
	if jitter < 0 {
		jitter = -jitter
	}
	m.state.NextCheck = now.Add(delay + jitter)
	if len(m.state.Notices) > 100 {
		m.state.Notices = m.state.Notices[len(m.state.Notices)-100:]
	}
	saveErr := m.save()
	if saveErr != nil {
		return saveErr
	}
	return err
}
func (m *Manager) Acknowledge(id string, dismiss bool) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	for i := range m.state.Notices {
		if m.state.Notices[i].ID == id {
			m.state.Notices[i].Read = true
			m.state.Notices[i].Dismissed = dismiss
			return m.save()
		}
	}
	return errors.New("notice not found")
}
func (m *Manager) MakePlan(ctx context.Context, id string, pinned bool) (Plan, error) {
	return m.MakeComponentPlan(ctx, id, pinned, "")
}
func (m *Manager) MakeComponentPlan(ctx context.Context, id string, pinned bool, guiID string) (Plan, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.busy || m.state.Job.Pending() {
		return Plan{}, errors.New("update or recovery in progress")
	}
	var target *Deployment
	for i := range m.state.Releases {
		if m.state.Releases[i].ID == id {
			target = &m.state.Releases[i]
			break
		}
	}
	if target == nil && m.state.Active != nil && m.state.Active.ID == id {
		target = m.state.Active
	}
	if target == nil {
		return Plan{}, errors.New("deployment not found; check published versions first")
	}
	if target.Source != m.state.Policy.Source {
		return Plan{}, errors.New("selected deployment belongs to a different update source; check again")
	}
	if err := target.Validate(m.trusted); err != nil {
		return Plan{}, err
	}
	fingerprint, previous, err := m.backend.Inventory(ctx)
	if err != nil {
		return Plan{}, err
	}
	overrides := map[string]Deployment{}
	if guiID != "" && guiID != target.ID {
		var gui *Deployment
		for i := range m.state.Releases {
			if m.state.Releases[i].ID == guiID {
				gui = &m.state.Releases[i]
				break
			}
		}
		if gui == nil {
			return Plan{}, errors.New("GUI deployment not found; check published versions first")
		}
		if err := gui.Validate(m.trusted); err != nil {
			return Plan{}, err
		}
		if gui.Source != m.state.Policy.Source || !compatibleGUI(*target, *gui) {
			return Plan{}, errors.New("GUI override lacks a matching published compatibility contract")
		}
		// Each published release was validated independently above. The backend
		// validates the base, then resolves the separately validated GUI image.
		overrides["gui"] = *gui
	}
	images, err := m.backend.PlanImages(ctx, *target)
	if err != nil {
		return Plan{}, err
	}
	if gui, ok := overrides["gui"]; ok {
		guiImages, e := m.backend.PlanImages(ctx, gui)
		if e != nil {
			return Plan{}, e
		}
		images["gui"] = guiImages["gui"]
	}
	p := Plan{Overrides: overrides, Target: *target, Policy: m.state.Policy, Fingerprint: fingerprint, ExpiresAt: m.now().Add(15 * time.Minute), Images: images, Previous: previous}
	p.Policy.Pinned = pinned
	p.ID = fmt.Sprintf("plan-%d", m.now().UnixNano())
	m.state.Plans = []Plan{p}
	return p, m.save()
}
func (m *Manager) Start(id string) (string, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.busy || m.checking || m.state.Job.Pending() {
		return "", errors.New("update or recovery in progress")
	}
	var plan *Plan
	for i := range m.state.Plans {
		if m.state.Plans[i].ID == id {
			plan = &m.state.Plans[i]
		}
	}
	if plan == nil || m.now().After(plan.ExpiresAt) {
		return "", errors.New("plan expired; review again")
	}
	j := Job{ID: fmt.Sprintf("job-%d", m.now().UnixNano()), Kind: "containers", Phase: "planned", StartedAt: m.now(), Plan: *plan, PreviousPolicy: m.state.Policy, PreviousActive: m.state.Active, PreviousOverrides: m.state.Overrides, PreviousImages: m.state.InstalledImages, PreviousJobID: m.state.ActiveJobID}
	if m.state.InstalledPolicy != nil {
		j.PreviousPolicy = *m.state.InstalledPolicy
	}
	m.state.Job = &j
	if err := m.save(); err != nil {
		return "", err
	}
	m.busy = true
	go m.run(false)
	return j.ID, nil
}
func (m *Manager) phase(phase string, err error) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.state.Job.Phase = phase
	if err != nil {
		m.state.Job.Error = err.Error()
	}
	return m.save()
}
func (m *Manager) Recover() {
	m.mu.Lock()
	if m.busy || !m.state.Job.Pending() {
		m.mu.Unlock()
		return
	}
	m.busy = true
	m.mu.Unlock()
	go m.run(true)
}
func (m *Manager) Rollback() (string, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.busy || m.checking || m.state.Job.Pending() {
		return "", errors.New("update or recovery in progress")
	}
	for i := len(m.state.History) - 1; i >= 0; i-- {
		old := m.state.History[i]
		if m.state.ActiveJobID != "" && old.ID != m.state.ActiveJobID {
			continue
		}
		if old.Phase != "succeeded" || old.Backup == "" || m.state.Active == nil || old.Plan.Target.ID != m.state.Active.ID {
			continue
		}
		old.ID = fmt.Sprintf("restore-%d", m.now().UnixNano())
		old.Kind = "rollback"
		old.Phase = "planned"
		old.Committed = ""
		old.Error = ""
		old.StartedAt = m.now()
		m.state.Job = &old
		if err := m.save(); err != nil {
			return "", err
		}
		m.busy = true
		go m.run(true)
		return old.ID, nil
	}
	return "", errors.New("no recoverable deployment")
}
func (m *Manager) run(recovery bool) {
	defer func() { m.mu.Lock(); m.busy = false; m.mu.Unlock() }()
	if backend, ok := m.backend.(interface{ Lock() (func(), error) }); ok {
		unlock, err := backend.Lock()
		if err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		defer unlock()
	}
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Minute)
	defer cancel()
	j := *m.Snapshot().Job
	failed := func(err error) { _ = m.phase("failed", err) }
	if recovery && j.Committed != "" {
		// Activation already committed. A failure removing the maintenance marker
		// must never turn a successful update into an unintended data rollback.
		images := j.Plan.Images
		if j.Committed == "rolled_back" {
			images = j.Plan.Previous
		}
		if err := m.backend.Verify(ctx, images, m.Snapshot().Active); err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		if err := m.backend.Maintenance(ctx, false); err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		_ = m.phase(j.Committed, nil)
		return
	}
	if recovery && j.Backup == "" {
		if gate, ok := m.backend.(interface{ MaintenanceSet() (bool, error) }); ok {
			active, err := gate.MaintenanceSet()
			if err != nil {
				_ = m.phase("recovery_required", err)
				return
			}
			if !active {
				failed(errors.New("interrupted before maintenance; no containers changed"))
				return
			}
		}
		// Before backup there have been no image/data mutations. A maintenance
		// marker might remain; release it only after verifying the previous stack.
		if j.Phase == "backing_up" || j.Phase == "recovery_required" {
			if err := m.backend.Apply(ctx, j.Plan.Previous); err != nil {
				_ = m.phase("recovery_required", err)
				return
			}
		}
		if err := m.backend.Verify(ctx, j.Plan.Previous, nil); err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		if err := m.backend.Maintenance(ctx, false); err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		failed(errors.New("interrupted before activation"))
		return
	}
	if !recovery {
		fingerprint, _, err := m.backend.Inventory(ctx)
		if err != nil {
			failed(err)
			return
		}
		if fingerprint != j.Plan.Fingerprint {
			failed(errors.New("installed configuration changed; review a new plan"))
			return
		}
		if err = m.phase("downloading", nil); err != nil {
			return
		}
		if err = m.backend.Pull(ctx, j.Plan.Images); err != nil {
			failed(err)
			return
		}
		// Recheck after potentially lengthy pulls, before entering maintenance.
		fingerprint, _, err = m.backend.Inventory(ctx)
		if err != nil || fingerprint != j.Plan.Fingerprint {
			failed(errors.New("deployment changed during download"))
			return
		}
		if err = m.phase("quiescing", nil); err != nil {
			return
		}
		if err = m.backend.Maintenance(ctx, true); err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		if err = m.phase("backing_up", nil); err != nil {
			return
		}
		backup, err := m.backend.Backup(ctx, j.ID)
		if err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
		m.mu.Lock()
		m.state.Job.Backup = backup
		err = m.save()
		m.mu.Unlock()
		if err != nil {
			return
		}
		j.Backup = backup
		if err = m.phase("applying", nil); err != nil {
			return
		}
		err = m.backend.Apply(ctx, j.Plan.Images)
		if err == nil {
			err = m.phase("verifying", nil)
		}
		if err == nil {
			err = m.backend.Verify(ctx, j.Plan.Images, &j.Plan.Target)
		}
		var installed map[string]string
		if err == nil {
			_, installed, err = m.backend.Inventory(ctx)
		}
		if err == nil {
			m.mu.Lock()
			m.state.ActiveJobID = j.ID
			m.state.Overrides = j.Plan.Overrides
			m.state.InstalledImages = installed
			m.state.Active = &j.Plan.Target
			m.state.Policy = j.Plan.Policy
			m.state.InstalledPolicy = &j.Plan.Policy
			err = m.save()
			m.mu.Unlock()
			if err != nil {
				return
			}
			if err = m.finish("succeeded"); err != nil {
				return
			}
			if err = m.backend.Maintenance(ctx, false); err != nil {
				_ = m.phase("recovery_required", err)
			}
			return
		}
		if err = m.phase("rolling_back", err); err != nil {
			return
		}
	}
	// Use a fresh recovery budget when activation exhausted its own timeout.
	recoveryCtx, stop := context.WithTimeout(context.Background(), 15*time.Minute)
	defer stop()
	if j.Kind == "rollback" {
		if err := m.backend.Maintenance(recoveryCtx, true); err != nil {
			_ = m.phase("recovery_required", err)
			return
		}
	}
	if err := m.phase("rolling_back", nil); err != nil {
		return
	}
	if err := m.backend.Restore(recoveryCtx, j.Backup); err != nil {
		_ = m.phase("recovery_required", err)
		return
	}
	if err := m.backend.Apply(recoveryCtx, j.Plan.Previous); err != nil {
		_ = m.phase("recovery_required", err)
		return
	}
	if err := m.backend.Verify(recoveryCtx, j.Plan.Previous, j.PreviousActive); err != nil {
		_ = m.phase("recovery_required", err)
		return
	}
	m.mu.Lock()
	m.state.Active = j.PreviousActive
	m.state.ActiveJobID = j.PreviousJobID
	m.state.Overrides = j.PreviousOverrides
	m.state.InstalledImages = j.PreviousImages
	if m.state.Policy.Source != j.PreviousPolicy.Source {
		m.state.Releases = []Deployment{}
		m.state.LastCheck = time.Time{}
		m.state.NextCheck = m.now()
	}
	m.state.Policy = j.PreviousPolicy
	m.state.InstalledPolicy = &j.PreviousPolicy
	err := m.save()
	m.mu.Unlock()
	if err != nil {
		return
	}
	if err = m.finish("rolled_back"); err != nil {
		return
	}
	if err = m.backend.Maintenance(recoveryCtx, false); err != nil {
		_ = m.phase("recovery_required", err)
	}
}
func (m *Manager) finish(phase string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.state.Job.Phase = phase
	m.state.Job.Committed = phase
	m.state.History = append(m.state.History, *m.state.Job)
	if len(m.state.History) > 20 {
		m.state.History = m.state.History[len(m.state.History)-20:]
	}
	m.state.Plans = []Plan{}
	if m.state.Active != nil {
		for i := range m.state.Notices {
			if m.state.Notices[i].Kind != "updater" && (m.state.Notices[i].Deployment == m.state.Active.ID || (m.state.Job.Phase == "rolled_back" && m.state.Notices[i].Deployment == m.state.Job.Plan.Target.ID)) {
				m.state.Notices[i].Dismissed = true
			}
		}
	}
	return m.save()
}
