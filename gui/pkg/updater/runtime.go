package updater

import (
	"context"
	"time"
)

type RunningComponent struct {
	Image       string `json:"image"`
	Healthy     bool   `json:"healthy"`
	Healthcheck bool   `json:"healthcheck"`
}
type RuntimeStatus struct {
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
	for name := range managed {
		ci, e := b.inspect(ctx, c.Services[name].ContainerName)
		if e != nil {
			return nil, e
		}
		result[name] = RunningComponent{Image: ci.Image,
			Healthy:     ci.Config.Labels["com.docker.compose.project"] == b.Config.Project && ci.State.Running && (ci.State.Health == nil || ci.State.Health.Status == "healthy"),
			Healthcheck: ci.State.Health != nil}
	}
	return result, nil
}

func reconcile(s State, components map[string]RunningComponent) RuntimeStatus {
	r := RuntimeStatus{Identity: "custom", Health: "healthy", Components: components}
	if s.Active != nil {
		r.Identity = "matched"
		if len(s.Overrides) > 0 {
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
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.busy || m.state.Job.Pending() || generation != m.state.ActiveJobID {
		return
	}
	m.runtime = reconcile(m.state, components)
	m.runtime.SelectionPending = selectionPending
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
