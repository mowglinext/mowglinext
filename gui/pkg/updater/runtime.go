package updater

import (
	"context"
	"encoding/json"
	"github.com/mowglinext/mowglinext/pkg/updates"
	"os"
	"path/filepath"
	"time"
)

type RunningComponent struct {
	Image       string `json:"image"`
	Name        string `json:"name,omitempty"`
	Family      string `json:"family,omitempty"`
	Reference   string `json:"reference,omitempty"`
	Version     string `json:"version,omitempty"`
	Revision    string `json:"revision,omitempty"`
	Healthy     bool   `json:"healthy"`
	Healthcheck bool   `json:"healthcheck"`
}
type RuntimeStatus struct {
	Selection        map[string]string           `json:"selection,omitempty"`
	SelectionPending bool                        `json:"selection_pending,omitempty"`
	Identity         string                      `json:"identity"`
	Health           string                      `json:"health"`
	CheckedAt        time.Time                   `json:"checked_at"`
	Error            string                      `json:"error,omitempty"`
	Components       map[string]RunningComponent `json:"components,omitempty"`
}

func (b DockerBackend) Observe(ctx context.Context) (map[string]RunningComponent, error) {
	c, _, err := b.model(ctx)
	if err != nil {
		return nil, err
	}
	managed, err := managedServices(c)
	if err != nil {
		return nil, err
	}
	result := map[string]RunningComponent{}
	for name, contract := range managed {
		ci, e := b.inspect(ctx, c.Services[name].ContainerName)
		if e != nil {
			return nil, e
		}
		result[name] = RunningComponent{Image: ci.Image, Name: c.Services[name].ContainerName, Family: contract.Image, Reference: ci.Config.Image, Version: ci.Config.Labels["org.opencontainers.image.version"], Revision: ci.Config.Labels["org.opencontainers.image.revision"],
			Healthy:     ci.Config.Labels["com.docker.compose.project"] == b.Config.Project && ci.State.Running && (ci.State.Health == nil || ci.State.Health.Status == "healthy"),
			Healthcheck: ci.State.Health != nil}
	}
	return result, nil
}

func reconcile(s State, components map[string]RunningComponent) RuntimeStatus {
	r := RuntimeStatus{Identity: "custom", Health: "healthy", Components: components}
	if s.Active != nil || len(s.CustomImages) > 0 {
		r.Identity = "matched"
		if len(s.Overrides) > 0 || len(s.CustomImages) > 0 {
			r.Identity = "mixed"
		}
		if len(s.InstalledImages) == 0 {
			r.Identity = "unverified"
		} else {
			if len(s.InstalledImages) != len(components) {
				r.Identity = "drifted"
			}
			for name, image := range s.InstalledImages {
				if components[name].Image != image {
					r.Identity = "drifted"
				}
			}
		}
	}
	// Upstream images need not carry OCI version labels. Use the release's
	// approved upstream version only when the actual image matches its bytes.
	for name, component := range components {
		selected := s.Active
		if override, ok := s.Overrides[name]; ok {
			selected = &override
		}
		if selected == nil || s.CustomImages[name].ImageID != "" || component.Version != "" {
			continue
		}
		image := selected.Images[component.Family]
		if image.Type != "external" {
			continue
		}
		for platform := range image.Platforms {
			if updates.Matches(image, platform, component.Image, nil) {
				component.Version = image.Version
				components[name] = component
				break
			}
		}
	}
	for _, component := range components {
		if !component.Healthy {
			r.Health = "degraded"
		}
	}
	return r
}

// Local Docker sampling runs independently of registry checks and UI requests.
// Unknown/stale samples never confirm an installed release or healthy stack.
func (m *Manager) RefreshRuntime(ctx context.Context) {
	b, ok := m.backend.(interface {
		Observe(context.Context) (map[string]RunningComponent, error)
	})
	if !ok {
		return
	}
	m.mu.Lock()
	if m.busy || m.state.Job.Pending() {
		m.mu.Unlock()
		return
	}
	generation := m.state.ActiveJobID
	m.mu.Unlock()
	components, err := b.Observe(ctx)
	selectionPending := false
	if selector, ok := m.backend.(interface{ SelectionPending() (bool, error) }); ok && err == nil {
		selectionPending, err = selector.SelectionPending()
	}
	var selection map[string]string
	if reader, ok := m.backend.(interface {
		InstallerSelection() (map[string]string, error)
	}); ok && err == nil {
		selection, err = reader.InstallerSelection()
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.busy || m.state.Job.Pending() || generation != m.state.ActiveJobID {
		return
	}
	m.runtime = reconcile(m.state, components)
	m.runtime.SelectionPending = selectionPending
	m.runtime.Selection = selection
	m.runtime.CheckedAt = m.now()
	if err != nil {
		m.runtime.Identity = "unknown"
		m.runtime.Health = "unknown"
		m.runtime.Error = err.Error()
	}
}

func (m *Manager) Runtime() RuntimeStatus {
	m.mu.Lock()
	defer m.mu.Unlock()
	r := m.runtime
	if r.CheckedAt.IsZero() || m.now().Sub(r.CheckedAt) > time.Minute || m.busy || m.state.Job.Pending() {
		r.Identity = "unknown"
		r.Health = "unknown"
	}
	return r
}

func (b DockerBackend) InstallerSelection() (map[string]string, error) {
	data, err := os.ReadFile(filepath.Join(b.Config.Directory, "stack-selection.json"))
	if os.IsNotExist(err) {
		return nil, nil
	}
	if err != nil {
		return nil, err
	}
	var selection StackSelection
	err = json.Unmarshal(data, &selection)
	return selection.Options, err
}
