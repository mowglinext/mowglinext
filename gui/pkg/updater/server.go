package updater

import (
	"context"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"os"
	"path/filepath"
	"runtime"
	"time"
)

func Client(socket string) *http.Client {
	return &http.Client{Timeout: 30 * time.Second, Transport: &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", socket)
	}}}
}
func (m *Manager) Handler(config HostConfig) http.Handler {
	mux := http.NewServeMux()
	respond := func(w http.ResponseWriter, value any, err error) {
		w.Header().Set("Content-Type", "application/json")
		w.Header().Set("Cache-Control", "no-store")
		if err != nil {
			w.WriteHeader(409)
			_ = json.NewEncoder(w).Encode(map[string]string{"error": err.Error()})
			return
		}
		_ = json.NewEncoder(w).Encode(value)
	}
	decode := func(w http.ResponseWriter, r *http.Request, v any) bool {
		r.Body = http.MaxBytesReader(w, r.Body, 8192)
		d := json.NewDecoder(r.Body)
		d.DisallowUnknownFields()
		if err := d.Decode(v); err != nil {
			respond(w, nil, err)
			return false
		}
		return true
	}
	mux.HandleFunc("GET /v1/state", func(w http.ResponseWriter, r *http.Request) {
		var selection AgentSelection
		if data, err := os.ReadFile(filepath.Join(config.StateDir, "agent-active.json")); err == nil {
			_ = json.Unmarshal(data, &selection)
		}
		respond(w, map[string]any{"api": APIVersion, "agent": map[string]string{"version": Version, "revision": Revision, "platform": runtime.GOOS + "/" + runtime.GOARCH, "error": selection.Error}, "state": PublicState(m.Snapshot()), "runtime": m.Runtime(), "capabilities": []string{"component-overrides", "declared-services", "release-compose", "service-version-overrides", "custom-images", "external-images"}, "trusted_repositories": config.Trusted}, nil)
	})
	mux.HandleFunc("POST /v1/policy", func(w http.ResponseWriter, r *http.Request) {
		var p Policy
		if decode(w, r, &p) {
			respond(w, map[string]bool{"ok": true}, m.Configure(p))
		}
	})
	mux.HandleFunc("POST /v1/check", func(w http.ResponseWriter, r *http.Request) {
		go func() {
			ctx, cancel := context.WithTimeout(context.Background(), 3*time.Minute)
			defer cancel()
			_ = m.Check(ctx, true)
		}()
		w.WriteHeader(202)
	})
	mux.HandleFunc("POST /v1/plan", func(w http.ResponseWriter, r *http.Request) {
		var req struct {
			Deployment string            `json:"deployment"`
			Pinned     bool              `json:"pinned"`
			GUI        string            `json:"gui_deployment"`
			Components map[string]string `json:"component_deployments"`
		}
		if decode(w, r, &req) {
			if req.Components == nil {
				req.Components = map[string]string{}
			}
			if req.GUI != "" {
				if _, exists := req.Components["gui"]; exists {
					respond(w, nil, fmt.Errorf("GUI selection provided twice"))
					return
				}
				req.Components["gui"] = req.GUI
			}
			p, e := m.MakeServicePlan(r.Context(), req.Deployment, req.Pinned, req.Components)
			respond(w, PublicPlan(p), e)
		}
	})
	mux.HandleFunc("POST /v1/custom-plan", func(w http.ResponseWriter, r *http.Request) {
		var req struct {
			Images       map[string]string `json:"images"`
			Acknowledged bool              `json:"acknowledged"`
		}
		if decode(w, r, &req) {
			p, e := m.MakeCustomPlan(r.Context(), req.Images, req.Acknowledged)
			respond(w, PublicPlan(p), e)
		}
	})
	mux.HandleFunc("POST /v1/apply", func(w http.ResponseWriter, r *http.Request) {
		var req struct {
			Plan               string `json:"plan"`
			CustomAcknowledged bool   `json:"custom_acknowledged"`
		}
		if decode(w, r, &req) {
			id, e := m.StartAcknowledged(req.Plan, req.CustomAcknowledged)
			respond(w, map[string]string{"job": id}, e)
		}
	})
	mux.HandleFunc("POST /v1/rollback", func(w http.ResponseWriter, r *http.Request) {
		id, e := m.Rollback()
		respond(w, map[string]string{"job": id}, e)
	})
	mux.HandleFunc("POST /v1/recover", func(w http.ResponseWriter, r *http.Request) {
		m.Recover()
		respond(w, map[string]bool{"ok": true}, nil)
	})
	mux.HandleFunc("POST /v1/notice", func(w http.ResponseWriter, r *http.Request) {
		var req struct {
			ID      string `json:"id"`
			Dismiss bool   `json:"dismiss"`
		}
		if decode(w, r, &req) {
			respond(w, map[string]bool{"ok": true}, m.Acknowledge(req.ID, req.Dismiss))
		}
	})
	mux.HandleFunc("POST /v1/agent-update", func(w http.ResponseWriter, r *http.Request) {
		var req struct {
			Deployment string `json:"deployment"`
		}
		if decode(w, r, &req) {
			err := m.UpgradeAgent(r.Context(), config, req.Deployment)
			respond(w, map[string]bool{"restarting": err == nil}, err)
		}
	})
	return mux
}
func Serve(config HostConfig) error {
	if runtime.GOOS != "linux" || (runtime.GOARCH != "amd64" && runtime.GOARCH != "arm64") {
		return fmt.Errorf("host updater requires Linux amd64 or arm64")
	}
	if !filepath.IsAbs(config.Directory) || !filepath.IsAbs(config.StateDir) || !idPattern.MatchString(config.Project) || config.Platform != "linux/"+runtime.GOARCH {
		return fmt.Errorf("invalid host configuration")
	}
	socket := filepath.Join(config.StateDir, "run", "updater.sock")
	if err := os.MkdirAll(filepath.Dir(socket), 0755); err != nil {
		return err
	}
	unlock, err := processLock(filepath.Join(config.StateDir, "worker.lock"))
	if err != nil {
		return err
	}
	defer unlock()
	// The systemd supervisor is the only service owner. A live socket means a
	// second process must not steal the endpoint or operate concurrently.
	conn, err := net.DialTimeout("unix", socket, time.Second)
	if err == nil {
		conn.Close()
		return fmt.Errorf("updater already running")
	}
	_ = os.Remove(socket)
	listener, err := net.Listen("unix", socket)
	if err != nil {
		return err
	}
	defer listener.Close()
	defer os.Remove(socket)
	if err = os.Chmod(socket, 0660); err != nil {
		return err
	}
	m, err := Open(config.StateDir, config.Trusted, DockerBackend{config}, GitHubSource{Client: &http.Client{Timeout: 20 * time.Second}, Trusted: config.Trusted})
	if err != nil {
		return err
	}
	if _, err = os.Stat(filepath.Join(config.StateDir, "state.json")); os.IsNotExist(err) && config.InitialSource.Validate(config.Trusted) == nil {
		if err = m.Configure(Policy{Source: config.InitialSource, IntervalHours: DefaultCheckIntervalHours}); err != nil {
			return err
		}
	}
	if !m.Snapshot().Job.Pending() {
		// A successful commit may have reached disk immediately before power
		// loss prevented the final marker removal. The commit is authoritative.
		state := m.Snapshot()
		if state.Job != nil && (state.Job.Phase == "succeeded" || state.Job.Phase == "rolled_back") {
			if err = (DockerBackend{config}).Maintenance(context.Background(), false); err != nil {
				return err
			}
		}
	}
	m.Recover()
	go func() {
		tick := time.NewTicker(15 * time.Second)
		defer tick.Stop()
		for {
			ctx, cancel := context.WithTimeout(context.Background(), 12*time.Second)
			m.RefreshRuntime(ctx)
			cancel()
			<-tick.C
		}
	}()
	go func() {
		tick := time.NewTicker(time.Minute)
		defer tick.Stop()
		for {
			ctx, cancel := context.WithTimeout(context.Background(), 3*time.Minute)
			_ = m.Check(ctx, false)
			cancel()
			<-tick.C
		}
	}()
	server := &http.Server{Handler: m.Handler(config), ReadHeaderTimeout: 5 * time.Second, ReadTimeout: 15 * time.Second, WriteTimeout: 5 * time.Minute, IdleTimeout: 30 * time.Second}
	return server.Serve(listener)
}
