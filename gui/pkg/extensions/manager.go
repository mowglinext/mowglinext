package extensions

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"path/filepath"
	"reflect"
	"sort"
	"strings"
	"time"
)

type Manager struct {
	Store   Store
	Host    HostConfig
	Backend Backend
	// HTTP administration uses separate preparation/activation budgets and a
	// narrower storage policy. The standalone CLI retains its caller's policy.
	phaseTimeout  time.Duration
	bindingPolicy func(string, Binding) error
}

func (m Manager) approveBinding(id string, binding Binding) error {
	if m.bindingPolicy != nil {
		return m.bindingPolicy(id, binding)
	}
	return nil
}

func (m Manager) compatible(manifest Manifest) error {
	if err := ValidateManifest(manifest); err != nil {
		return err
	}
	if reasons := Compatible(manifest, m.Host); len(reasons) != 0 {
		return fmt.Errorf("incompatible extension: %s", strings.Join(reasons, "; "))
	}
	return nil
}

func sameData(a Manifest, b Manifest) bool {
	return a.ID == b.ID && a.Source.Repository == b.Source.Repository &&
		a.Runtime.Kind == b.Runtime.Kind && a.Profile == b.Profile && a.DataSchema == b.DataSchema
}

// V1 upgrades replace images/configuration inside a fixed service topology.
// Adding services or renaming networks needs a distinct retirement/migration
// contract so a failed target can never strand old-version recovery.
func sameTopology(a, b Release) bool {
	type topology struct {
		Services map[string]struct {
			ContainerName string                     `json:"container_name"`
			Networks      map[string]json.RawMessage `json:"networks"`
		} `json:"services"`
		Networks map[string]composeNetwork `json:"networks"`
	}
	var left, right topology
	if json.Unmarshal(a.Bundle.Compose, &left) != nil || json.Unmarshal(b.Bundle.Compose, &right) != nil {
		return false
	}
	return reflect.DeepEqual(left, right)
}

func (m Manager) release(bundle Bundle, binding Binding) (Release, error) {
	// Detach caller-owned slices/maps and use the same closed parser as the CLI.
	data, err := json.Marshal(bundle)
	if err != nil {
		return Release{}, err
	}
	bundle, err = ParseBundle(data)
	if err != nil {
		return Release{}, err
	}
	if err = m.compatible(bundle.Manifest); err != nil {
		return Release{}, err
	}
	if err = ValidateBinding(binding); err != nil {
		return Release{}, err
	}
	return Release{Bundle: bundle, Binding: binding, Digest: BundleDigest(bundle)}, nil
}

func reserve(state State, id string, release Release) error {
	check := func(other string, binding Binding) error {
		if other == id {
			return nil
		}
		if binding.Project == release.Binding.Project {
			return fmt.Errorf("project is already reserved by %s", other)
		}
		for _, a := range []string{binding.ConfigDir, binding.DataDir} {
			for _, b := range []string{release.Binding.ConfigDir, release.Binding.DataDir} {
				if a == b || strings.HasPrefix(a, b+string(filepath.Separator)) || strings.HasPrefix(b, a+string(filepath.Separator)) {
					return fmt.Errorf("storage overlaps extension %s", other)
				}
			}
		}
		return nil
	}
	for other, entry := range state.Entries {
		if err := check(other, entry.Active.Binding); err != nil {
			return err
		}
	}
	for other, retained := range state.Retained {
		if err := check(other, retained.Binding); err != nil {
			return err
		}
	}
	if retained, ok := state.Retained[id]; ok &&
		(!sameData(retained.Manifest, release.Bundle.Manifest) || retained.Binding != release.Binding) {
		return errors.New("retained data requires an explicit ownership/layout/schema migration")
	}
	if retained, ok := state.Retained[id]; ok && retained.StorageIdentity != "" {
		identity, err := StorageIdentity(release)
		if err != nil {
			return err
		}
		if identity != retained.StorageIdentity {
			return errors.New("retained data storage layout requires a separate migration")
		}
	}
	return nil
}

// Apply changes a single optional service. All mutations hold the host manager
// lock. No action invokes the robot installer, maintenance API or core updater.
func (m Manager) Apply(ctx context.Context, action, id string, bundle *Bundle, binding Binding) error {
	return m.apply(ctx, action, id, bundle, binding, "", "")
}

// apply checks the reviewed active release under the same lock as mutation.
func (m Manager) apply(ctx context.Context, action, id string, bundle *Bundle, binding Binding, expected, expectedPrevious string) error {
	parent := ctx
	if m.phaseTimeout > 0 {
		prepared, cancel := context.WithTimeout(parent, m.phaseTimeout)
		defer cancel()
		ctx = prepared
	}
	unlock, err := m.Store.Lock()
	if err != nil {
		return err
	}
	defer unlock()
	state, err := m.Store.Load()
	if err != nil {
		return err
	}
	if state.Journal != nil {
		return errors.New("extension recovery is pending; run recover first")
	}
	entry, exists := state.Entries[id]
	if expected != "" && (!exists || entry.Active.Digest != expected) {
		return errors.New("the reviewed release changed; refresh and review again")
	}
	if expectedPrevious != "" && (!exists || entry.Previous == nil || entry.Previous.Digest != expectedPrevious) {
		return errors.New("the reviewed rollback target changed; refresh and review again")
	}
	if action == "install" || action == "adopt" {
		if exists || bundle == nil || bundle.Manifest.ID != id {
			return errors.New("extension already registered or bundle/id missing")
		}
		if e := m.approveBinding(id, binding); e != nil {
			return e
		}
		release, e := m.release(*bundle, binding)
		if e != nil {
			return e
		}
		external := release.Bundle.Manifest.Runtime.Kind == "external-systemd"
		if external && action != "adopt" {
			return errors.New("external systemd supports observe/adopt/forget only")
		}
		if e = reserve(state, id, release); e != nil {
			return e
		}
		if e = m.Backend.Check(ctx, release, action == "adopt"); e != nil {
			return e
		}
		if action == "install" {
			if e = m.Backend.Prepare(ctx, release); e != nil {
				return e
			}
		} else if !external {
			observation, e := m.Backend.Observe(ctx, release)
			if e != nil {
				return e
			}
			if observation.Process != "running" && observation.Process != "stopped" {
				return errors.New("cannot adopt without an authoritative runtime identity")
			}
			entry.Enabled = observation.Process == "running"
		}
		entry.Active = release
		state.Entries[id] = entry
		return m.Store.Save(state)
	}
	if !exists {
		return errors.New("extension is not registered")
	}
	if err = m.approveBinding(id, entry.Active.Binding); err != nil {
		return err
	}
	if entry.Active.Bundle.Manifest.Runtime.Kind == "external-systemd" {
		if action != "forget" {
			return errors.New("external systemd supports observe/adopt/forget only")
		}
		delete(state.Entries, id)
		return m.Store.Save(state)
	}
	if entry.RecoveryRequired {
		return errors.New("extension recovery is required")
	}
	before := entry
	switch action {
	case "enable":
		if err = m.compatible(entry.Active.Bundle.Manifest); err != nil {
			return err
		}
		if entry.Enabled {
			return m.verify(ctx, entry.Active)
		}
		if err = m.Backend.Check(ctx, entry.Active, false); err != nil {
			return err
		}
		if err = m.Backend.Prepare(ctx, entry.Active); err != nil {
			return err
		}
		entry.Enabled = true
	case "disable":
		entry.Enabled = false
	case "remove":
		if entry.Enabled {
			return errors.New("disable before removal; config and data will be retained")
		}
		observation, e := m.Backend.Observe(ctx, entry.Active)
		if e != nil {
			return e
		}
		if observation.Process != "stopped" {
			return errors.New("disable and verify the runtime is stopped before removal")
		}
	case "upgrade", "rollback":
		var target Release
		if action == "rollback" {
			if entry.Previous == nil {
				return errors.New("no previous release retained")
			}
			target = *entry.Previous
			if err = m.compatible(target.Bundle.Manifest); err != nil {
				return err
			}
		} else {
			if bundle == nil || bundle.Manifest.ID != id {
				return errors.New("target bundle/id missing")
			}
			target, err = m.release(*bundle, entry.Active.Binding)
			if err != nil {
				return err
			}
		}
		if !sameData(entry.Active.Bundle.Manifest, target.Bundle.Manifest) || entry.Active.Binding != target.Binding {
			return errors.New("data, ownership or runtime migration requires a separate procedure")
		}
		if !sameTopology(entry.Active, target) {
			return errors.New("service or network topology migration requires a separate procedure")
		}
		beforeStorage, e := StorageIdentity(entry.Active)
		if e != nil {
			return e
		}
		afterStorage, e := StorageIdentity(target)
		if e != nil {
			return e
		}
		if beforeStorage != afterStorage {
			return errors.New("storage layout migration requires a separate procedure")
		}
		if err = m.Backend.Check(ctx, target, false); err != nil {
			return err
		}
		if err = m.Backend.Prepare(ctx, target); err != nil {
			return err
		}
		if !entry.Enabled {
			observation, e := m.Backend.Observe(ctx, entry.Active)
			if e != nil {
				return e
			}
			if observation.Process != "stopped" {
				return errors.New("disable and verify the runtime is stopped before a disabled upgrade or rollback")
			}
		}
		previous := entry.Active
		entry.Active, entry.Previous = target, &previous
	default:
		return errors.New("unknown lifecycle action")
	}
	// Never stop an existing release after preparation has exhausted its budget.
	if err = ctx.Err(); err != nil {
		return err
	}
	if m.phaseTimeout > 0 {
		activation, cancel := context.WithTimeout(parent, m.phaseTimeout)
		defer cancel()
		ctx = activation
	}
	entry.Error = ""
	state.Journal = &Journal{Action: action, ID: id, Phase: "applying", Before: &before, Target: &entry}
	if err = m.Store.Save(state); err != nil {
		return err // no runtime mutation before the journal is durable
	}
	if action == "disable" {
		err = m.Backend.Stop(ctx, before.Active)
	} else if action == "remove" {
		err = m.Backend.Remove(ctx, before.Active)
	} else if !entry.Enabled {
		// Retire stopped containers before advancing release identity. Their old
		// labels/config must not outlive the bounded active/previous history.
		// Compose down retains all config, bind data and external named volumes.
		err = m.Backend.Remove(ctx, before.Active)
	} else {
		if action == "upgrade" || action == "rollback" {
			err = m.Backend.Stop(ctx, before.Active)
		}
		if err == nil {
			err = m.Backend.Start(ctx, entry.Active)
		}
		if err == nil {
			err = m.verify(ctx, entry.Active)
		}
	}
	if err != nil {
		state.Journal.Error = err.Error()
		// Recovery must get a fresh bounded context if activation timed out. The
		// CLI supplies a separate timeout for explicit recover after cancellation.
		if ctx.Err() != nil {
			return m.block(state, err)
		}
		recovery := m.recoverLocked(ctx, &state)
		if recovery != nil {
			return fmt.Errorf("activation failed: %v; recovery failed: %w", err, recovery)
		}
		return fmt.Errorf("operation failed; recovery completed: %w", err)
	}
	if action == "remove" {
		retain(&state, id, before)
	} else {
		state.Entries[id] = entry
	}
	state.Journal = nil
	return m.Store.Save(state)
}

func (m Manager) verify(ctx context.Context, release Release) error {
	ctx, cancel := context.WithTimeout(ctx, 30*time.Second)
	defer cancel()
	for {
		observation, err := m.Backend.Observe(ctx, release)
		if err != nil {
			return err
		}
		if observation.Process == "running" && observation.Identity != "" {
			if release.Bundle.Manifest.Health == "process-only" || observation.Application == "healthy" {
				return nil
			}
			if observation.Application != "starting" {
				return errors.New("application readiness is not healthy or has no configured probe")
			}
		}
		timer := time.NewTimer(250 * time.Millisecond)
		select {
		case <-ctx.Done():
			timer.Stop()
			return errors.New("runtime readiness deadline exceeded")
		case <-timer.C:
		}
	}
}

func retain(state *State, id string, entry Entry) {
	identity, _ := StorageIdentity(entry.Active) // validated before runtime mutation and at Store.Load
	state.Retained[id] = Retained{Manifest: entry.Active.Bundle.Manifest, Binding: entry.Active.Binding,
		StorageIdentity: identity, VolumeNames: VolumeClaims(entry.Active)}
	delete(state.Entries, id)
}

func (m Manager) block(state State, reason error) error {
	j := state.Journal
	j.Phase, j.RecoveryError = "recovery_required", reason.Error()
	entry := *j.Before
	entry.RecoveryRequired = true
	entry.Error = j.Error
	if entry.Error != "" {
		entry.Error += "; "
	}
	entry.Error += "recovery required: " + reason.Error()
	state.Entries[j.ID] = entry
	if err := m.Store.Save(state); err != nil {
		return fmt.Errorf("%v; could not persist recovery status: %w", reason, err)
	}
	return reason
}

func (m Manager) recoverLocked(ctx context.Context, state *State) error {
	j := state.Journal
	if j == nil {
		return nil
	}
	for _, entry := range []*Entry{j.Before, j.Target} {
		if entry != nil {
			if err := m.approveBinding(j.ID, entry.Active.Binding); err != nil {
				return err
			}
		}
	}
	j.Phase = "recovering"
	if err := m.Store.Save(*state); err != nil {
		return err
	}
	var err error
	if j.Action == "disable" {
		err = m.Backend.Stop(ctx, j.Before.Active)
		if err == nil {
			state.Entries[j.ID] = *j.Target
		}
	} else if j.Action == "remove" {
		err = m.Backend.Remove(ctx, j.Before.Active)
		if err == nil {
			retain(state, j.ID, *j.Before)
		}
	} else {
		err = m.Backend.Stop(ctx, j.Target.Active)
		if err == nil && j.Before.Enabled {
			err = m.compatible(j.Before.Active.Bundle.Manifest)
			if err == nil {
				err = m.Backend.Start(ctx, j.Before.Active)
			}
			if err == nil {
				err = m.verify(ctx, j.Before.Active)
			}
		}
		if err == nil {
			entry := *j.Before
			entry.RecoveryRequired = false
			entry.Error = j.Error
			state.Entries[j.ID] = entry
		}
	}
	if err != nil {
		return m.block(*state, err)
	}
	state.Journal = nil
	return m.Store.Save(*state)
}

// Recover resumes an interrupted optional-service transaction; it never changes
// core maintenance state. Explicit invocation avoids a hidden daemon/retry loop.
func (m Manager) Recover(ctx context.Context) error {
	unlock, err := m.Store.Lock()
	if err != nil {
		return err
	}
	defer unlock()
	state, err := m.Store.Load()
	if err != nil {
		return err
	}
	return m.recoverLocked(ctx, &state)
}

func (m Manager) Status(ctx context.Context) ([]Status, error) {
	unlock, err := m.Store.Lock()
	if err != nil {
		return nil, err
	}
	defer unlock()
	state, err := m.Store.Load()
	if err != nil {
		return nil, err
	}
	result := []Status{}
	for id, entry := range state.Entries {
		blockers := Compatible(entry.Active.Bundle.Manifest, m.Host)
		obs, e := m.Backend.Observe(ctx, entry.Active)
		statusError := entry.Error
		if e != nil {
			obs = Observation{Process: "unknown", Application: "unknown", Data: "unknown"}
			statusError = strings.TrimSpace(statusError + "; runtime observation unavailable")
		}
		recovery := entry.RecoveryRequired || (state.Journal != nil && state.Journal.ID == id)
		previous := ""
		previousDigest := ""
		if entry.Previous != nil {
			previous = entry.Previous.Bundle.Manifest.Version
			previousDigest = entry.Previous.Digest
		}
		result = append(result, Status{ID: id, Version: entry.Active.Bundle.Manifest.Version,
			Managed: entry.Active.Bundle.Manifest.Runtime.Kind == "compose", Enabled: entry.Enabled,
			Compatible: len(blockers) == 0, Blockers: blockers, RecoveryRequired: recovery,
			Error: statusError, Observation: obs, Manifest: entry.Active.Bundle.Manifest,
			Digest: entry.Active.Digest, PreviousVersion: previous, PreviousDigest: previousDigest})
	}
	sort.Slice(result, func(i, j int) bool { return result[i].ID < result[j].ID })
	return result, nil
}

func (m Manager) Preflight(ctx context.Context, target HostConfig) (map[string][]string, error) {
	unlock, err := m.Store.Lock()
	if err != nil {
		return nil, err
	}
	defer unlock()
	state, err := m.Store.Load()
	if err != nil {
		return nil, err
	}
	conflicts := map[string][]string{}
	for id, entry := range state.Entries {
		observation, observationError := m.Backend.Observe(ctx, entry.Active)
		recovery := entry.RecoveryRequired || (state.Journal != nil && state.Journal.ID == id)
		if !entry.Enabled && !recovery && observationError == nil && observation.Process == "stopped" {
			continue
		}
		reasons := Compatible(entry.Active.Bundle.Manifest, target)
		if recovery {
			reasons = append(reasons, "extension recovery unresolved")
		}
		if len(reasons) > 0 {
			conflicts[id] = reasons
		}
	}
	return conflicts, nil
}
