package updater

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"time"
)

type AgentSelection struct {
	Path     string `json:"path"`
	Previous string `json:"previous"`
	Version  string `json:"version"`
	Error    string `json:"error,omitempty"`
}

// Installer upgrades may cross journal schemas, so they must select the new
// worker explicitly and retain the old executable together with its journal.
// Unlike online self-update, recovery across schemas is an installer operation.
func CheckInstallerState(c HostConfig) error {
	if active, err := (DockerBackend{c}).MaintenanceSet(); err != nil || active {
		return errors.New("resolve update maintenance before upgrading the installer worker")
	}
	if _, err := os.Stat(filepath.Join(c.StateDir, "agent-pending.json")); !os.IsNotExist(err) {
		return errors.New("resolve the pending worker replacement first")
	}
	m, err := Open(c.StateDir, c.Trusted, nil, nil)
	if err != nil {
		return err
	}
	if m.Snapshot().Job.Pending() {
		return errors.New("resolve the pending deployment before upgrading the installer worker")
	}
	return nil
}

func SelectInstallerWorker(c HostConfig, bootstrap string, candidate []byte) error {
	if !filepath.IsAbs(bootstrap) {
		return errors.New("installer executable must be an absolute path")
	}
	if len(candidate) == 0 {
		return errors.New("installer executable is empty")
	}
	unlock, err := processLock(filepath.Join(c.StateDir, "worker.lock"))
	if err != nil {
		return err
	}
	defer unlock()
	if err = CheckInstallerState(c); err != nil {
		return err
	}
	previous := AgentSelection{Path: bootstrap}
	files := map[string][]byte{}
	for _, name := range []string{"state.json", "agent-active.json"} {
		data, err := os.ReadFile(filepath.Join(c.StateDir, name))
		if err != nil && !os.IsNotExist(err) {
			return err
		}
		files[name] = data
		if name == "agent-active.json" && data != nil {
			if err = json.Unmarshal(data, &previous); err != nil {
				return err
			}
		}
	}
	if previous.Path != bootstrap && filepath.Dir(previous.Path) != filepath.Join(c.StateDir, "bin") {
		return errors.New("invalid previous worker path")
	}
	data, err := os.ReadFile(previous.Path)
	if err != nil && (!os.IsNotExist(err) || files["agent-active.json"] != nil) {
		return err
	}
	retained := ""
	if err == nil {
		sum := sha256.Sum256(data)
		retained = filepath.Join(c.StateDir, "bin", hex.EncodeToString(sum[:]))
		if err = AtomicWrite(retained, data, 0755); err != nil {
			return err
		}
	}
	backup := filepath.Join(c.StateDir, "installer-backups", fmt.Sprintf("%d", time.Now().UnixNano()), "restore.json")
	if err = AtomicJSON(backup, map[string]any{"files": files, "worker": retained, "selection": previous}); err != nil {
		return err
	}
	// Replace the launcher before selecting it. A crash between these writes
	// leaves the old active worker selected (or the new launcher on first boot),
	// never a selection pointing at an older bootstrap that cannot read state.
	if err = AtomicWrite(bootstrap, candidate, 0755); err != nil {
		return err
	}
	return AtomicJSON(filepath.Join(c.StateDir, "agent-active.json"), AgentSelection{Path: bootstrap, Previous: retained, Version: Version})
}

func WaitInstallerWorker(ctx context.Context, c HostConfig) error {
	client := Client(filepath.Join(c.StateDir, "run", "updater.sock"))
	client.Timeout = 2 * time.Second
	samples := 0
	for {
		req, err := http.NewRequestWithContext(ctx, "GET", "http://updater/v1/state", nil)
		if err != nil {
			return err
		}
		resp, err := client.Do(req)
		good := false
		if err == nil {
			var status struct {
				API   int `json:"api"`
				Agent struct {
					Version  string `json:"version"`
					Revision string `json:"revision"`
				} `json:"agent"`
			}
			err = json.NewDecoder(io.LimitReader(resp.Body, 4*1024*1024)).Decode(&status)
			resp.Body.Close()
			good = err == nil && resp.StatusCode == 200 && status.API == APIVersion && status.Agent.Version == Version && status.Agent.Revision == Revision
		}
		if good {
			samples++
		} else {
			samples = 0
		}
		if samples >= 3 {
			return nil
		}
		select {
		case <-ctx.Done():
			return fmt.Errorf("installer replacement did not become healthy: %w", ctx.Err())
		case <-time.After(time.Second):
		}
	}
}

func (m *Manager) UpgradeAgent(ctx context.Context, c HostConfig, id string) error {
	m.mu.Lock()
	if m.busy || m.checking || m.state.Job.Pending() {
		m.mu.Unlock()
		return errors.New("update or recovery in progress")
	}
	var target *Deployment
	for i := range m.state.Releases {
		if m.state.Releases[i].ID == id {
			copy := m.state.Releases[i]
			target = &copy
		}
	}
	if target == nil {
		m.mu.Unlock()
		return errors.New("deployment not found")
	}
	m.busy = true
	m.mu.Unlock()
	pending := false
	defer func() {
		if !pending {
			m.mu.Lock()
			m.busy = false
			m.mu.Unlock()
		}
	}()
	if err := target.Validate(c.Trusted); err != nil {
		return err
	}
	binary, ok := target.Updater[c.Platform]
	if !ok {
		return errors.New("no updater binary for this platform")
	}
	req, err := http.NewRequestWithContext(ctx, "GET", assetURL(target.Source.Repository, target.ReleaseTag, binary.Asset), nil)
	if err != nil {
		return err
	}
	resp, err := (&http.Client{Timeout: 3 * time.Minute}).Do(req)
	if err != nil {
		return err
	}
	defer resp.Body.Close()
	if resp.StatusCode != 200 {
		return fmt.Errorf("updater download HTTP %d", resp.StatusCode)
	}
	data, err := io.ReadAll(io.LimitReader(resp.Body, 64*1024*1024+1))
	if err != nil {
		return err
	}
	if len(data) > 64*1024*1024 {
		return errors.New("updater exceeds download limit")
	}
	sum := sha256.Sum256(data)
	if hex.EncodeToString(sum[:]) != binary.SHA256 {
		return errors.New("updater checksum mismatch")
	}
	path := filepath.Join(c.StateDir, "bin", binary.SHA256)
	if err = AtomicWrite(path, data, 0755); err != nil {
		return err
	}
	probeCtx, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()
	output, err := command(probeCtx, path, "version")
	if err != nil {
		return err
	}
	if err = validateWorkerProbe(output, binary.Version); err != nil {
		return err
	}

	current, err := os.Executable()
	if err != nil {
		return err
	}
	if err = AtomicJSON(filepath.Join(c.StateDir, "agent-pending.json"), AgentSelection{Path: path, Previous: current, Version: binary.Version}); err != nil {
		return err
	}
	pending = true
	return nil
}

// Supervise is the installer-managed recovery launcher. It remains outside the
// replaceable worker and confirms the candidate's API before committing it.
// systemd alone restarts processes; this launcher owns binary rollback.
func Supervise(configPath string, c HostConfig) error {
	original, err := os.Executable()
	if err != nil {
		return err
	}
	return supervise(context.Background(), configPath, c, original)
}

func supervise(ctx context.Context, configPath string, c HostConfig, original string) error {
	var err error
	activeFile := filepath.Join(c.StateDir, "agent-active.json")
	pendingFile := filepath.Join(c.StateDir, "agent-pending.json")
	selection := AgentSelection{Path: original, Version: Version}
	if data, e := os.ReadFile(activeFile); e == nil {
		if e = json.Unmarshal(data, &selection); e != nil {
			return e
		}
	}
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		candidate := false
		previous := selection
		if data, e := os.ReadFile(pendingFile); e == nil {
			if e = json.Unmarshal(data, &selection); e != nil {
				return e
			}
			candidate = true
		}
		// Selection files are root-owned. Still reject paths outside our retained
		// binaries or the installer-managed bootstrap executable.
		if selection.Path != original && filepath.Dir(selection.Path) != filepath.Join(c.StateDir, "bin") {
			return errors.New("invalid updater executable path")
		}
		child := exec.Command(selection.Path, "serve", "--config", configPath)
		child.Stdout = os.Stdout
		child.Stderr = os.Stderr
		if err = child.Start(); err != nil {
			if !candidate {
				return err
			}
			selection = previous
			selection.Error = "Candidate updater could not start: " + err.Error()
			if err = AtomicJSON(activeFile, selection); err != nil {
				return err
			}
			_ = os.Remove(pendingFile)
			continue
		}
		defer child.Process.Kill()
		exited := make(chan error, 1)
		go func() { exited <- child.Wait() }()
		healthy := !candidate
		healthSamples := 0
		deadline := time.Now().Add(45 * time.Second)
		client := Client(filepath.Join(c.StateDir, "run", "updater.sock"))
		client.Timeout = 2 * time.Second
		for {
			select {
			case <-ctx.Done():
				_ = child.Process.Kill()
				<-exited
				return ctx.Err()
			case e := <-exited:
				if !healthy {
					selection = previous
					selection.Error = "Candidate updater exited before becoming healthy"
					_ = AtomicJSON(activeFile, selection)
					_ = os.Remove(pendingFile)
					break
				}
				return fmt.Errorf("updater worker stopped: %v", e)
			case <-time.After(time.Second):
				if candidate && !healthy {
					resp, e := client.Get("http://updater/v1/state")
					if e == nil {
						var status struct {
							Agent struct {
								Version string `json:"version"`
							} `json:"agent"`
						}
						e = json.NewDecoder(resp.Body).Decode(&status)
						resp.Body.Close()
						if e == nil && resp.StatusCode == http.StatusOK && status.Agent.Version == selection.Version {
							healthSamples++
						} else {
							healthSamples = 0
						}
						if healthSamples >= 3 {
							healthy = true
							if e = AtomicJSON(activeFile, selection); e != nil {
								return e
							}
							_ = os.Remove(pendingFile)
						}
					} else {
						healthSamples = 0
					}
					if !healthy && time.Now().After(deadline) {
						_ = child.Process.Kill()
						<-exited
						selection = previous
						selection.Error = "Candidate updater health timeout"
						_ = AtomicJSON(activeFile, selection)
						_ = os.Remove(pendingFile)
						break
					}
				}
				if healthy {
					if _, e := os.Stat(pendingFile); e == nil {
						_ = child.Process.Kill()
						<-exited
						break
					}
				}
				continue
			}
			break
		}
	}
}

func validateWorkerProbe(output []byte, expectedVersion string) error {
	var info struct {
		Version     string `json:"version"`
		API         int    `json:"api"`
		StateSchema int    `json:"state_schema"`
	}
	if err := json.Unmarshal(output, &info); err != nil || info.Version != expectedVersion || info.API != APIVersion || info.StateSchema != StateSchema {
		return errors.New("updater version/API/state-schema probe failed")
	}
	return nil
}
