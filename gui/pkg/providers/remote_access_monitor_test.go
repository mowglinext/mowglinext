package providers

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"sync/atomic"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func monitoredRemoteAccess(t *testing.T) (*RemoteAccessProvider, *fakeServiceDocker, *time.Time) {
	t.Helper()
	d := runningDocker(`{"BackendState":"Running","Self":{"Online":true}}`)
	p := newTestRemoteAccess(t, enabledRemoteConfig(), d)
	d.details.Labels = p.buildSpec(p.Config()).Labels
	now := fixedNow()
	p.now = func() time.Time { return now }
	p.networkOnline = func(context.Context) bool { return true }
	p.maintenance = func() bool { return false }
	return p, d, &now
}

func localDaemonFault(d *fakeServiceDocker) {
	d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
		return types.ContainerExecResult{ExitCode: 1, Stderr: "failed to connect to local tailscaled"}, nil
	}
}

func monitorSnapshot(p *RemoteAccessProvider) RemoteAccessStatus {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.statusSnapshot()
}

func TestRemoteAccessMonitorLeavesUsableStatesAndNetworkWarningsAlone(t *testing.T) {
	for _, reply := range []string{
		`{"BackendState":"Running","Self":{"Online":true}}`,
		`{"BackendState":"Running","Self":{"Online":false},"Health":["No internet connection"]}`,
		`{"BackendState":"Running","Health":["DNS unavailable","control server unreachable"]}`,
		`{"BackendState":"Starting"}`, `{"BackendState":"Stopped"}`, `{"BackendState":"NoState"}`,
		`{"BackendState":"NeedsLogin","Health":["key expired or revoked"]}`,
		`{"BackendState":"NeedsMachineAuth"}`,
	} {
		t.Run(reply, func(t *testing.T) {
			p, d, now := monitoredRemoteAccess(t)
			d.execFn = func(spec types.ContainerExecSpec) (types.ContainerExecResult, error) {
				assert.Equal(t, []string{"timeout", "-s", "KILL", "8", "tailscale", "status", "--json"}, spec.Cmd)
				return types.ContainerExecResult{Stdout: reply, ExitCode: 1}, nil
			}
			p.networkOnline = func(context.Context) bool { t.Fatal("usable status must not probe the host"); return false }
			for i := 0; i < 20; i++ {
				p.checkHealth(context.Background())
				*now = now.Add(time.Minute)
			}
			assert.Empty(t, d.restarted)
			assert.Empty(t, d.pulled)
			assert.Empty(t, d.created)
			assert.NotNil(t, monitorSnapshot(p).LastSuccessAt)
		})
	}
}

func TestRemoteAccessMonitorRecoversSameContainerAndRetainsSpec(t *testing.T) {
	for _, hang := range []bool{false, true} {
		t.Run(map[bool]string{false: "socket failure", true: "CLI hang"}[hang], func(t *testing.T) {
			p, d, now := monitoredRemoteAccess(t)
			localDaemonFault(d)
			if hang {
				d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
					return types.ContainerExecResult{ExitCode: 137}, nil
				}
			}
			original := p.buildSpec(p.Config())
			p.checkHealth(context.Background())
			assert.Empty(t, d.restarted)
			*now = now.Add(remoteAccessFaultGrace - time.Second)
			p.checkHealth(context.Background())
			assert.Empty(t, d.restarted)
			*now = now.Add(time.Second)
			d.restartFn = func(ctx context.Context, id string) error {
				assert.Equal(t, d.details.ID, id)
				d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
					return types.ContainerExecResult{Stdout: `{"BackendState":"Running","Self":{"Online":true}}`}, nil
				}
				return nil
			}
			p.checkHealth(context.Background())
			require.Len(t, d.restarted, 1)
			p.checkHealth(context.Background())
			st := p.Status(context.Background())
			assert.True(t, st.StatusUsable)
			require.NotNil(t, st.SelfOnline)
			assert.True(t, *st.SelfOnline)
			assert.Equal(t, "monitoring", st.RecoveryState)
			assert.Equal(t, 1, st.RecoveryAttempts)
			assert.Equal(t, original, p.buildSpec(p.Config()), "volume, credentials, Serve and image unchanged")
			assert.Empty(t, d.pulled)
			assert.Empty(t, d.created)
			assert.Empty(t, d.removed)
			assert.Empty(t, d.started)
		})
	}
}

func TestRemoteAccessMonitorNetworkLossPausesRecovery(t *testing.T) {
	for _, loss := range []string{"WAN", "local Wi-Fi"} {
		t.Run(loss, func(t *testing.T) {
			p, d, now := monitoredRemoteAccess(t)
			localDaemonFault(d)
			p.networkOnline = func(context.Context) bool { return false }
			for i := 0; i < 30; i++ {
				p.checkHealth(context.Background())
				*now = now.Add(time.Minute)
			}
			assert.Empty(t, d.restarted)
			assert.Zero(t, monitorSnapshot(p).RecoveryAttempts)
			p.networkOnline = func(context.Context) bool { return true }
			// A fresh grace period is required after a failed host probe.
			*now = now.Add(remoteAccessFaultGrace)
			p.checkHealth(context.Background())
			if len(d.restarted) == 0 {
				*now = now.Add(remoteAccessFaultGrace)
				p.checkHealth(context.Background())
			}
			assert.Len(t, d.restarted, 1)
		})
	}
}

func TestRemoteAccessMonitorBoundsRetriesAcrossHealthyChecksAndGUIRestarts(t *testing.T) {
	p, d, now := monitoredRemoteAccess(t)
	localDaemonFault(d)
	d.restartErr = errors.New("restart refused")
	p.checkHealth(context.Background())
	for attempt := 1; attempt <= remoteAccessRecoveryLimit; attempt++ {
		if attempt == 1 {
			*now = now.Add(remoteAccessFaultGrace)
		} else {
			*now = *monitorSnapshot(p).NextRecoveryAt
		}
		p.checkHealth(context.Background())
		require.Len(t, d.restarted, attempt)
		st := monitorSnapshot(p)
		assert.Equal(t, attempt, st.RecoveryAttempts)
		assert.Contains(t, st.RecoveryError, "restart refused")
		require.NotNil(t, st.NextRecoveryAt)
		assert.Equal(t, now.Add(time.Duration(1<<uint(attempt-1))*5*time.Minute), *st.NextRecoveryAt)
		for i := 0; i < 3; i++ {
			p.checkHealth(context.Background())
		}
		assert.Len(t, d.restarted, attempt)
		// Rebuilding the provider does not replenish the persistent budget.
		p = newRemoteAccessProvider(p.db, d, func() time.Time { return *now })
		p.networkOnline = func(context.Context) bool { return true }
		p.maintenance = func() bool { return false }
		p.checkHealth(context.Background())
	}
	*now = now.Add(24 * time.Hour)
	p.checkHealth(context.Background())
	assert.Len(t, d.restarted, remoteAccessRecoveryLimit)
	assert.Equal(t, "degraded", monitorSnapshot(p).RecoveryState)
	assert.Equal(t, RemoteAccessDegraded, monitorSnapshot(p).Phase)
	assert.Nil(t, monitorSnapshot(p).NextRecoveryAt)
	d.found = false
	p.checkHealth(context.Background())
	assert.Equal(t, "degraded", monitorSnapshot(p).RecoveryState, "exhausted recovery stays actionable even if the sidecar disappears")
	p.Apply()
	assert.Zero(t, monitorSnapshot(p).RecoveryAttempts)
	assert.Empty(t, monitorSnapshot(p).RecoveryError)
}

func TestRemoteAccessMonitorDoesNotInferHangFromDockerOrParserErrors(t *testing.T) {
	for _, fault := range []string{"Docker timeout", "malformed JSON", "empty backend", "missing timeout command", "different spec", "stopped", "absent"} {
		t.Run(fault, func(t *testing.T) {
			p, d, now := monitoredRemoteAccess(t)
			localDaemonFault(d)
			switch fault {
			case "Docker timeout":
				d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
					return types.ContainerExecResult{}, context.DeadlineExceeded
				}
			case "malformed JSON":
				d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
					return types.ContainerExecResult{Stdout: "broken"}, nil
				}
			case "empty backend":
				d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
					return types.ContainerExecResult{Stdout: "{}"}, nil
				}
			case "missing timeout command":
				d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
					return types.ContainerExecResult{ExitCode: 127}, nil
				}
			case "different spec":
				d.details.Labels = nil
			case "stopped":
				d.details.Running = false
			case "absent":
				d.found = false
			}
			for i := 0; i < 15; i++ {
				p.checkHealth(context.Background())
				*now = now.Add(time.Minute)
			}
			assert.Empty(t, d.restarted)
			assert.Empty(t, d.pulled)
			assert.Empty(t, d.created)
		})
	}
}

func TestRemoteAccessMonitorDisabledAndMaintenanceStayQuiet(t *testing.T) {
	p, d, now := monitoredRemoteAccess(t)
	localDaemonFault(d)
	p.checkHealth(context.Background())
	*now = now.Add(remoteAccessFaultGrace)
	p.networkOnline = func(context.Context) bool { p.maintenance = func() bool { return true }; return true }
	p.checkHealth(context.Background())
	assert.Empty(t, d.restarted, "maintenance starting during probe suppresses restart")
	p.maintenance = func() bool { return true }
	d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
		t.Fatal("must not observe during maintenance")
		return types.ContainerExecResult{}, nil
	}
	p.checkHealth(context.Background())
	assert.Equal(t, "paused", monitorSnapshot(p).RecoveryState)
	p.maintenance = func() bool { return false }
	cfg := p.Config()
	cfg.Enabled = false
	require.NoError(t, p.UpdateConfig(cfg))
	p.maintenance = func() bool { return false }
	p.checkHealth(context.Background())
	assert.Equal(t, "disabled", monitorSnapshot(p).RecoveryState)
}

func TestRemoteAccessMonitorSerializesDisableLogoutAndSettings(t *testing.T) {
	for _, action := range []string{"disable", "logout", "settings"} {
		t.Run(action, func(t *testing.T) {
			p, d, now := monitoredRemoteAccess(t)
			localDaemonFault(d)
			p.checkHealth(context.Background())
			*now = now.Add(remoteAccessFaultGrace)
			entered, release, finished := make(chan struct{}), make(chan struct{}), make(chan struct{})
			d.restartFn = func(ctx context.Context, _ string) error {
				close(entered)
				select {
				case <-release:
					return nil
				case <-ctx.Done():
					return ctx.Err()
				}
			}
			go func() { p.checkHealth(context.Background()); close(finished) }()
			<-entered
			ctx, cancel := context.WithCancel(context.Background())
			cancel()
			assert.ErrorIs(t, p.Reconcile(ctx), context.Canceled)
			st := p.Status(ctx)
			assert.Contains(t, st.Error, "canceled")
			changed := make(chan error, 1)
			go func() {
				if action == "logout" {
					changed <- p.Logout(context.Background())
					return
				}
				cfg := p.Config()
				cfg.Enabled = action != "disable"
				cfg.Hostname = "changed"
				changed <- p.UpdateConfig(cfg)
			}()
			select {
			case <-changed:
				t.Fatal("operator action overlapped recovery")
			default:
			}
			close(release)
			<-finished
			if action == "logout" { // fake CLI must accept the intentional logout
				// localDaemonFault's reply is deliberately nonzero: logout can fail,
				// but must still have waited until the recovery finished.
				assert.Error(t, <-changed)
			} else {
				require.NoError(t, <-changed)
			}
			assert.Len(t, d.restarted, 1)
		})
	}
}

func TestRemoteAccessMonitorPeriodicLoopAndDeadline(t *testing.T) {
	p, d, _ := monitoredRemoteAccess(t)
	observed := make(chan time.Duration, 1)
	d.execCtxFn = func(ctx context.Context, _ types.ContainerExecSpec) (types.ContainerExecResult, error) {
		deadline, ok := ctx.Deadline()
		if !ok {
			observed <- 0
		} else {
			observed <- time.Until(deadline)
		}
		return types.ContainerExecResult{Stdout: `{"BackendState":"Running"}`}, nil
	}
	ctx, cancel := context.WithCancel(context.Background())
	ticks, done := make(chan time.Time), make(chan struct{})
	go func() { defer close(done); p.runWithTicks(ctx, ticks) }()
	ticks <- fixedNow()
	deadline := <-observed
	assert.Positive(t, deadline)
	assert.LessOrEqual(t, deadline, remoteAccessRecoveryTimeout)
	cancel()
	<-done
}

func TestRemoteAccessMaintenanceMarkerContract(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "maintenance")
	t.Setenv("MOWGLI_UPDATE_MAINTENANCE", path)
	assert.False(t, UpdateMaintenanceActive())
	require.NoError(t, os.WriteFile(path, []byte("active"), 0600))
	assert.True(t, UpdateMaintenanceActive())
	require.NoError(t, os.Remove(path))
	require.NoError(t, os.Remove(dir))
	assert.True(t, UpdateMaintenanceActive(), "missing mount is fail-closed")
}

func TestRemoteAccessHealthyObservationBreaksFaultGraceButKeepsCooldown(t *testing.T) {
	p, d, now := monitoredRemoteAccess(t)
	localDaemonFault(d)
	p.checkHealth(context.Background())
	*now = now.Add(remoteAccessFaultGrace)
	p.checkHealth(context.Background())
	require.Len(t, d.restarted, 1)
	next := *monitorSnapshot(p).NextRecoveryAt
	d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
		return types.ContainerExecResult{Stdout: `{"BackendState":"Running"}`}, nil
	}
	p.checkHealth(context.Background())
	assert.Equal(t, 1, monitorSnapshot(p).RecoveryAttempts)
	assert.Equal(t, next, *monitorSnapshot(p).NextRecoveryAt)
	localDaemonFault(d)
	p.checkHealth(context.Background())
	*now = now.Add(remoteAccessFaultGrace)
	p.checkHealth(context.Background())
	assert.Len(t, d.restarted, 1, "healthy status must not remove the previous restart cooldown")
	*now = next
	p.checkHealth(context.Background())
	assert.Len(t, d.restarted, 2)
	// A UI observation is also evidence against a continuous daemon fault.
	*now = now.Add(time.Minute)
	p.checkHealth(context.Background())
	d.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
		return types.ContainerExecResult{Stdout: `{"BackendState":"Running"}`}, nil
	}
	p.Status(context.Background())
	assert.True(t, p.monitor.faultSince.IsZero())
}

type failedRecoveryDB struct {
	types.IDBProvider
	failSet, failGet, failDelete bool
}

func (d failedRecoveryDB) Get(k string) ([]byte, error) {
	if k == remoteAccessRecoveryKey && d.failGet {
		return nil, errors.New("disk unavailable")
	}
	return d.IDBProvider.Get(k)
}
func (d failedRecoveryDB) Set(k string, b []byte) error {
	if k == remoteAccessRecoveryKey && d.failSet {
		return errors.New("disk unavailable")
	}
	return d.IDBProvider.Set(k, b)
}
func (d failedRecoveryDB) Delete(k string) error {
	if k == remoteAccessRecoveryKey && d.failDelete {
		return errors.New("disk unavailable")
	}
	return d.IDBProvider.Delete(k)
}

func TestRemoteAccessRecoveryBudgetStorageErrorsFailClosed(t *testing.T) {
	for _, failure := range []string{"read", "write", "delete", "corrupt"} {
		t.Run(failure, func(t *testing.T) {
			p, d, now := monitoredRemoteAccess(t)
			localDaemonFault(d)
			base := p.db
			switch failure {
			case "read":
				p.db = failedRecoveryDB{IDBProvider: base, failGet: true}
				p.loadRecoveryBudget()
			case "write":
				p.db = failedRecoveryDB{IDBProvider: base, failSet: true}
			case "delete":
				p.db = failedRecoveryDB{IDBProvider: base, failDelete: true}
				p.Apply()
			case "corrupt":
				require.NoError(t, base.Set(remoteAccessRecoveryKey, []byte("invalid")))
				p.loadRecoveryBudget()
			}
			p.checkHealth(context.Background())
			*now = now.Add(remoteAccessFaultGrace)
			p.checkHealth(context.Background())
			assert.Empty(t, d.restarted)
			assert.Equal(t, "degraded", monitorSnapshot(p).RecoveryState)
			assert.NotEmpty(t, monitorSnapshot(p).RecoveryError)
		})
	}
}

type notifiedServiceDocker struct {
	*fakeServiceDocker
	started chan struct{}
}

func (d notifiedServiceDocker) ContainerStart(ctx context.Context, id string) error {
	err := d.fakeServiceDocker.ContainerStart(ctx, id)
	close(d.started)
	return err
}

func TestRemoteAccessMonitorRetainsRequestedReconcileAcrossMaintenance(t *testing.T) {
	p, d, _ := monitoredRemoteAccess(t)
	d.found = false
	started := make(chan struct{})
	p.docker = notifiedServiceDocker{fakeServiceDocker: d, started: started}
	var paused atomic.Bool
	paused.Store(true)
	checked := make(chan struct{}, 1)
	p.maintenance = func() bool {
		select {
		case checked <- struct{}{}:
		default:
		}
		return paused.Load()
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	ticks := make(chan time.Time)
	go func() { defer close(done); p.runWithTicks(ctx, ticks) }()
	p.kick()
	<-checked
	paused.Store(false)
	ticks <- fixedNow()
	select {
	case <-started:
	case <-time.After(time.Second):
		t.Fatal("maintenance lost requested reconciliation")
	}
	cancel()
	<-done
	assert.Len(t, d.created, 1)
	assert.Equal(t, RemoteAccessRunning, monitorSnapshot(p).Phase)
}

func TestRemoteAccessMaintenanceAcknowledgementDrainsInflightRecovery(t *testing.T) {
	p, d, now := monitoredRemoteAccess(t)
	localDaemonFault(d)
	var paused atomic.Bool
	p.maintenance = paused.Load
	p.checkHealth(context.Background())
	*now = now.Add(remoteAccessFaultGrace)
	entered, release, done := make(chan struct{}), make(chan struct{}), make(chan struct{})
	d.restartFn = func(context.Context, string) error { close(entered); <-release; return nil }
	go func() { defer close(done); p.checkHealth(context.Background()) }()
	<-entered
	paused.Store(true)
	assert.False(t, p.MaintenanceQuiescent(), "updater must not advance during an already-started restart")
	close(release)
	<-done
	assert.True(t, p.MaintenanceQuiescent())
	*now = now.Add(time.Hour)
	p.checkHealth(context.Background())
	assert.Len(t, d.restarted, 1, "no new recovery after maintenance acknowledgement")
}

func TestRemoteAccessStatusTimestampsRemainImmutableDuringRecovery(t *testing.T) {
	p, d, now := monitoredRemoteAccess(t)
	localDaemonFault(d)
	p.checkHealth(context.Background())
	*now = now.Add(remoteAccessFaultGrace)
	var snapshot RemoteAccessStatus
	p.networkOnline = func(context.Context) bool {
		ctx, cancel := context.WithCancel(context.Background())
		cancel()
		snapshot = p.Status(ctx) // API request times out waiting for the operation gate.
		return true
	}
	p.checkHealth(context.Background())
	require.NotNil(t, snapshot.NextRecoveryAt)
	assert.Equal(t, *now, *snapshot.NextRecoveryAt, "later cooldown reservation must not mutate a status snapshot already handed to JSON encoding")
	assert.Equal(t, now.Add(5*time.Minute), *monitorSnapshot(p).NextRecoveryAt)
}
