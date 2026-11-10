package extensions

import (
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/updates"
)

const stateLimit = 16 << 20

// Store persists only manager metadata. Callers hold Lock across the complete
// read/check/mutate/save transaction, including backend ownership checks.
type Store struct{ Root string }

func (s Store) Lock() (func(), error) {
	dir, err := openStoreDir(s.Root, true)
	if err != nil {
		return nil, err
	}
	defer dir.Close()
	return lockStore(dir)
}

func (s Store) Load() (State, error) {
	dir, err := openStoreDir(s.Root, false)
	if errors.Is(err, os.ErrNotExist) {
		return NewState(), nil
	}
	if err != nil {
		return State{}, err
	}
	defer dir.Close()
	f, err := openStoreFile(dir, "state.json")
	if errors.Is(err, os.ErrNotExist) {
		return NewState(), nil
	}
	if err != nil {
		return State{}, err
	}
	defer f.Close()
	data, err := io.ReadAll(io.LimitReader(f, stateLimit+1))
	if err != nil {
		return State{}, err
	}
	var state State
	if err := strictJSON(data, &state, stateLimit); err != nil {
		return State{}, fmt.Errorf("state: %w", err)
	}
	if err := validateState(state); err != nil {
		return State{}, err
	}
	return state, nil
}

func (s Store) Save(state State) error {
	if err := validateState(state); err != nil {
		return err
	}
	data, err := json.Marshal(state)
	if err != nil {
		return err
	}
	if len(data) > stateLimit {
		return errors.New("state exceeds size limit")
	}
	dir, err := openStoreDir(s.Root, true)
	if err != nil {
		return err
	}
	defer dir.Close()
	return writeStoreState(dir, data)
}

func validateRelease(r Release, id string) error {
	if err := validateBundle(r.Bundle); err != nil {
		return fmt.Errorf("release manifest: %w", err)
	}
	if r.Bundle.Manifest.ID != id {
		return errors.New("release id differs from registration")
	}
	if err := ValidateBinding(r.Binding); err != nil {
		return err
	}
	if !updates.DigestPattern.MatchString(r.Digest) || r.Digest != BundleDigest(r.Bundle) {
		return errors.New("release digest does not match bundle")
	}
	return nil
}

func validateEntry(e Entry, id string) error {
	if err := validateRelease(e.Active, id); err != nil {
		return err
	}
	if e.Previous != nil {
		if err := validateRelease(*e.Previous, id); err != nil {
			return err
		}
		if e.Previous.Binding != e.Active.Binding || e.Previous.Bundle.Manifest.Runtime.Kind != e.Active.Bundle.Manifest.Runtime.Kind || e.Previous.Bundle.Manifest.DataSchema != e.Active.Bundle.Manifest.DataSchema {
			return errors.New("previous release changes ownership, runtime or data schema")
		}
		currentStorage, err := StorageIdentity(e.Active)
		if err != nil {
			return err
		}
		previousStorage, err := StorageIdentity(*e.Previous)
		if err != nil {
			return err
		}
		if currentStorage != previousStorage {
			return errors.New("previous release changes data layout")
		}
	}
	if e.Active.Bundle.Manifest.Runtime.Kind == "external-systemd" && e.Enabled {
		return errors.New("external service cannot be manager-enabled")
	}
	return nil
}

func validateState(s State) error {
	if s.Schema != Schema {
		return errors.New("unsupported state schema")
	}
	if s.Entries == nil || s.Retained == nil {
		return errors.New("state ownership maps are required")
	}
	type ownership struct {
		id       string
		binding  Binding
		manifest Manifest
		storage  string
		volumes  []string
	}
	owners := []ownership{}
	reserve := func(id string, b Binding, m Manifest, storage string, volumes []string) error {
		if storage != "" && !updates.DigestPattern.MatchString(storage) {
			return errors.New("invalid storage identity")
		}
		if err := validateList(volumes, resourceName.MatchString); err != nil {
			return errors.New("invalid or duplicate physical volume claim")
		}
		for _, o := range owners {
			if o.id == id {
				if o.binding != b || o.manifest.DataSchema != m.DataSchema || o.manifest.Runtime.Kind != m.Runtime.Kind || o.storage != "" && storage != "" && o.storage != storage {
					return errors.New("registration changes retained ownership or data schema")
				}
				if (o.storage != "" && storage != "" || len(o.volumes) > 0) && !sameStrings(o.volumes, volumes) {
					return errors.New("registration changes retained physical volume claims")
				}
				continue
			}
			for _, volume := range o.volumes {
				for _, candidate := range volumes {
					if volume == candidate {
						return errors.New("physical volume is owned by another extension")
					}
				}
			}
			if o.binding.Project == b.Project {
				return errors.New("project is owned by another extension")
			}
			for _, unit := range o.manifest.Runtime.Units {
				for _, candidate := range m.Runtime.Units {
					if unit == candidate {
						return errors.New("systemd unit is owned by another extension")
					}
				}
			}
			for _, p := range []string{o.binding.ConfigDir, o.binding.DataDir} {
				for _, q := range []string{b.ConfigDir, b.DataDir} {
					if p == q || strings.HasPrefix(p, q+"/") || strings.HasPrefix(q, p+"/") {
						return errors.New("config or data path overlaps another extension")
					}
				}
			}
		}
		owners = append(owners, ownership{id, b, m, storage, volumes})
		return nil
	}
	for id, r := range s.Retained {
		if !idPattern.MatchString(id) || r.Manifest.ID != id {
			return errors.New("invalid retained registration id")
		}
		if err := ValidateManifest(r.Manifest); err != nil {
			return err
		}
		if err := ValidateBinding(r.Binding); err != nil {
			return err
		}
		if err := reserve(id, r.Binding, r.Manifest, r.StorageIdentity, r.VolumeNames); err != nil {
			return err
		}
	}
	for id, e := range s.Entries {
		if !idPattern.MatchString(id) {
			return errors.New("invalid registration id")
		}
		if err := validateEntry(e, id); err != nil {
			return err
		}
		for _, release := range []*Release{&e.Active, e.Previous} {
			if release == nil {
				continue
			}
			storage, err := StorageIdentity(*release)
			if err != nil {
				return err
			}
			if err := reserve(id, release.Binding, release.Bundle.Manifest, storage, VolumeClaims(*release)); err != nil {
				return err
			}
		}
	}
	if j := s.Journal; j != nil {
		if !idPattern.MatchString(j.ID) {
			return errors.New("invalid journal id")
		}
		switch j.Action {
		case "enable", "disable", "upgrade", "rollback", "remove":
		default:
			return errors.New("invalid journal action")
		}
		switch j.Phase {
		case "applying", "recovering", "recovery_required":
		default:
			return errors.New("invalid journal phase")
		}
		if j.Before == nil || j.Target == nil {
			return errors.New("journal before and target are required")
		}
		if _, ok := s.Entries[j.ID]; !ok {
			return errors.New("journal registration is absent")
		}
		for _, e := range []*Entry{j.Before, j.Target} {
			if err := validateEntry(*e, j.ID); err != nil {
				return err
			}
			for _, release := range []*Release{&e.Active, e.Previous} {
				if release == nil {
					continue
				}
				storage, err := StorageIdentity(*release)
				if err != nil {
					return err
				}
				if err := reserve(j.ID, release.Binding, release.Bundle.Manifest, storage, VolumeClaims(*release)); err != nil {
					return err
				}
			}
		}
	}
	return nil
}
