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
