package providers

import (
	"context"
	"encoding/json"
	"errors"
	"net"
	"os"
	"path/filepath"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

const (
	remoteAccessMonitorInterval = time.Minute
	remoteAccessFaultGrace      = 3 * time.Minute
	remoteAccessRecoveryTimeout = 45 * time.Second
	remoteAccessRecoveryLimit   = 3
	remoteAccessRecoveryKey     = "remoteAccess.recoveryBudget"
)

// RemoteAccessMonitorStatus keeps service readiness separate from Docker's
// process state. Attempts are cumulative until an explicit Retry/settings save.
type RemoteAccessMonitorStatus struct {
	StatusUsable      bool       `json:"statusUsable"`
	SelfOnline        *bool      `json:"selfOnline,omitempty"`
	LastSuccessAt     *time.Time `json:"lastSuccessAt,omitempty"`
	RecoveryState     string     `json:"recoveryState"`
	RecoveryAttempts  int        `json:"recoveryAttempts"`
	NextRecoveryAt    *time.Time `json:"nextRecoveryAt,omitempty"`
	HostNetworkOnline *bool      `json:"hostNetworkOnline,omitempty"`
	RecoveryError     string     `json:"recoveryError,omitempty"`
}

type remoteAccessMonitor struct {
	RemoteAccessMonitorStatus
	faultSince  time.Time
	containerID string
}

type remoteAccessBudget struct {
	Attempts int        `json:"attempts"`
	NextAt   *time.Time `json:"nextAt,omitempty"`
}

// UpdateMaintenanceActive uses the same fail-closed marker contract as the API
// and updater. Missing/unreadable marker directories must pause autonomous work.
func UpdateMaintenanceActive() bool {
	path := os.Getenv("MOWGLI_UPDATE_MAINTENANCE")
	if path == "" {
		return false
	}
	if _, err := os.Stat(filepath.Dir(path)); err != nil {
		return true
	}
	_, err := os.Stat(path)
	return err == nil || !os.IsNotExist(err)
}

// A positive host TCP connection to Tailscale's control endpoint is additional
// evidence only: failure suppresses recovery, never triggers it. No peer ping,
// credentials, network changes or separate reconnect manager are involved.
func remoteAccessNetworkOnline(ctx context.Context) bool {
	probe, cancel := context.WithTimeout(ctx, 5*time.Second)
	defer cancel()
	conn, err := (&net.Dialer{}).DialContext(probe, "tcp", "controlplane.tailscale.com:443")
	if err != nil {
		return false
	}
	_ = conn.Close()
	return true
}

func (p *RemoteAccessProvider) acquire(ctx context.Context) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	select {
	case p.op <- struct{}{}:
		return nil
	case <-ctx.Done():
		return ctx.Err()
	}
}
func (p *RemoteAccessProvider) release() { <-p.op }

// MaintenanceQuiescent is the updater acknowledgement barrier: a marker alone
// cannot acknowledge maintenance while a previously started Docker operation is
// still in flight. Once drained, every mutating operation rechecks the marker
// under the same operation gate, so no recovery can start after acknowledgement.
// It is nonblocking so readiness keeps evaluating fresh robot telemetry.
func (p *RemoteAccessProvider) MaintenanceQuiescent() bool {
	select {
	case p.op <- struct{}{}:
		defer p.release()
		return p.maintenance()
	default:
		return false
	}
}

// Caller holds mu. State copies contain immutable timestamp/boolean pointers.
func (p *RemoteAccessProvider) statusSnapshot() RemoteAccessStatus {
	st := RemoteAccessStatus{
		RemoteAccessMonitorStatus: p.monitor.RemoteAccessMonitorStatus,
		Enabled:                   p.cfg.Enabled, Phase: p.phase, Error: p.lastError,
		Hostname: p.cfg.Hostname, TailscaleIps: []string{}, HttpUrls: []string{},
		Health: []string{}, CheckedAt: p.now(),
	}
	if !p.cfg.Enabled {
		st.RecoveryState = "disabled"
	}
	return st
}

func (p *RemoteAccessProvider) loadRecoveryBudget() {
	p.monitor.RecoveryState = "monitoring"
	raw, err := p.db.Get(remoteAccessRecoveryKey)
	if errors.Is(err, types.ErrKeyNotFound) {
		return
	}
	var budget remoteAccessBudget
	if err != nil || json.Unmarshal(raw, &budget) != nil || budget.Attempts < 0 || budget.Attempts > remoteAccessRecoveryLimit {
		p.monitor.RecoveryAttempts = remoteAccessRecoveryLimit
		p.monitor.RecoveryState = "degraded"
		p.monitor.RecoveryError = "Recovery budget unavailable; use Retry after checking storage"
		return
	}
	p.monitor.RecoveryAttempts = budget.Attempts
	p.monitor.NextRecoveryAt = budget.NextAt
}

// Caller owns op. A failed budget reset remains fail-closed and visible.
func (p *RemoteAccessProvider) resetRecoveryBudget() {
	err := p.db.Delete(remoteAccessRecoveryKey)
	p.mu.Lock()
	defer p.mu.Unlock()
	p.monitor = remoteAccessMonitor{}
	p.monitor.RecoveryState = "monitoring"
	if err != nil && !errors.Is(err, types.ErrKeyNotFound) {
		p.monitor.RecoveryAttempts = remoteAccessRecoveryLimit
		p.monitor.RecoveryState = "degraded"
		p.monitor.RecoveryError = "Unable to reset recovery budget"
	}
}

// checkHealth never reconciles, pulls or recreates. Only a responsive Docker
// exec reporting a local socket failure or an in-container CLI timeout can arm
// recovery. A Docker transport error, invalid JSON, offline node, health warning
// or any usable backend state is observational, not evidence of a daemon hang.
func (p *RemoteAccessProvider) checkHealth(ctx context.Context) {
	ctx, cancel := context.WithTimeout(ctx, remoteAccessRecoveryTimeout)
	defer cancel()
	if err := p.acquire(ctx); err != nil {
		return
	}
	defer p.release()
	if !p.Config().Enabled {
		return
	}
	if p.maintenance() {
		p.mu.Lock()
		p.monitor.RecoveryState = "paused"
		p.monitor.faultSince = time.Time{}
		p.mu.Unlock()
		return
	}
	status, localFault := p.observe(ctx)
	var details types.ContainerDetails
	var found bool
	var err error
	if localFault {
		details, found, err = p.docker.FindContainerByName(ctx, RemoteAccessContainerName)
	}
	p.mu.Lock()
	defer p.mu.Unlock()
	p.monitor.HostNetworkOnline = nil
	if status.StatusUsable {
		p.monitor.faultSince = time.Time{}
		p.monitor.RecoveryState = "monitoring"
		if status.BackendState != "Running" {
			p.monitor.RecoveryState = "operator_action"
		}
		if p.phase == RemoteAccessDegraded {
			p.phase = RemoteAccessRunning
		}
		return
	}
	if p.monitor.RecoveryAttempts >= remoteAccessRecoveryLimit {
		p.monitor.RecoveryState = "degraded"
		p.monitor.NextRecoveryAt = nil
		p.phase = RemoteAccessDegraded
		return
	}
	if !localFault {
		p.monitor.faultSince = time.Time{}
		p.monitor.RecoveryState = "operator_action"
		return
	}
	// Both the identity and spec must still match before touching anything.
	if err != nil || !found || !details.Running || details.ID != status.containerID || details.Labels[remoteAccessSpecLabel] != p.buildSpec(p.cfg).Labels[remoteAccessSpecLabel] {
		p.monitor.faultSince = time.Time{}
		p.monitor.RecoveryState = "operator_action"
		return
	}
	now := p.now()
	if p.monitor.containerID != details.ID {
		p.monitor.containerID = details.ID
		p.monitor.faultSince = time.Time{}
	}
	if p.monitor.faultSince.IsZero() {
		p.monitor.faultSince = now
	}
	if p.monitor.NextRecoveryAt != nil && now.Before(*p.monitor.NextRecoveryAt) {
		p.monitor.RecoveryState = "suspect"
		if p.monitor.RecoveryAttempts > 0 {
			p.monitor.RecoveryState = "cooldown"
		}
		return
	}
	next := p.monitor.faultSince.Add(remoteAccessFaultGrace)
	p.monitor.NextRecoveryAt = &next
	p.monitor.RecoveryState = "suspect"
	if now.Before(next) {
		return
	}
	// Do not hold the status mutex across network or restart calls. op still
	// serializes settings, logout, status, reconcile and another health tick.
	p.mu.Unlock()
	online := p.networkOnline(ctx)
	paused := p.maintenance()
	p.mu.Lock()
	p.monitor.HostNetworkOnline = &online
	if !online || paused || ctx.Err() != nil {
		p.monitor.faultSince = time.Time{}
		p.monitor.NextRecoveryAt = nil
		p.monitor.RecoveryState = "paused"
		return
	}
	attempt := p.monitor.RecoveryAttempts + 1
	next = now.Add(time.Duration(1<<uint(attempt-1)) * 5 * time.Minute)
	budget, _ := json.Marshal(remoteAccessBudget{Attempts: attempt, NextAt: &next})
	// Reserve before acting so process restarts cannot replenish retries.
	if err := p.db.Set(remoteAccessRecoveryKey, budget); err != nil {
		p.monitor.RecoveryState = "degraded"
		p.monitor.RecoveryError = "Unable to reserve recovery attempt"
		p.monitor.RecoveryAttempts = remoteAccessRecoveryLimit
		p.monitor.NextRecoveryAt = nil
		p.phase = RemoteAccessDegraded
		return
	}
	p.monitor.RecoveryAttempts = attempt
	p.monitor.NextRecoveryAt = &next
	p.monitor.RecoveryState = "cooldown"
	p.monitor.faultSince = time.Time{}
	p.mu.Unlock()
	err = p.docker.ContainerRestart(ctx, details.ID)
	p.mu.Lock()
	p.monitor.RecoveryError = ""
	if err != nil {
		p.monitor.RecoveryError = "Sidecar restart failed: " + err.Error()
	}
}
