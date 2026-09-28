package providers

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// fakeServiceDocker records every call so tests can assert the exact
// sequence the reconcile issues.
type fakeServiceDocker struct {
	found   bool
	details types.ContainerDetails
	findErr error
	pullErr error

	pulled  []string
	created []types.ServiceContainerSpec
	started []string
	removed []string
	execFn  func(spec types.ContainerExecSpec) (types.ContainerExecResult, error)
}

func (f *fakeServiceDocker) ImagePull(_ context.Context, ref string) error {
	f.pulled = append(f.pulled, ref)
	return f.pullErr
}

func (f *fakeServiceDocker) FindContainerByName(context.Context, string) (types.ContainerDetails, bool, error) {
	if f.findErr != nil {
		return types.ContainerDetails{}, false, f.findErr
	}
	return f.details, f.found, nil
}

func (f *fakeServiceDocker) ContainerCreateService(_ context.Context, spec types.ServiceContainerSpec) (string, error) {
	f.created = append(f.created, spec)
	return "new-id", nil
}

func (f *fakeServiceDocker) ContainerRemove(_ context.Context, id string, _ bool) error {
	f.removed = append(f.removed, id)
	return nil
}

func (f *fakeServiceDocker) ContainerStart(_ context.Context, id string) error {
	f.started = append(f.started, id)
	return nil
}

func (f *fakeServiceDocker) ContainerStop(context.Context, string) error { return nil }

func (f *fakeServiceDocker) ContainerInspect(context.Context, string) (types.ContainerDetails, error) {
	return f.details, nil
}

func (f *fakeServiceDocker) ContainerExec(_ context.Context, _ string, spec types.ContainerExecSpec) (types.ContainerExecResult, error) {
	if f.execFn != nil {
		return f.execFn(spec)
	}
	return types.ContainerExecResult{}, nil
}

func fixedNow() time.Time { return time.Date(2026, 9, 15, 12, 0, 0, 0, time.UTC) }

func newTestRemoteAccess(t *testing.T, cfg RemoteAccessConfig, docker *fakeServiceDocker) *RemoteAccessProvider {
	t.Helper()
	db := types.NewMockDBProvider()
	require.NoError(t, SaveRemoteAccessConfig(db, cfg))
	return newRemoteAccessProvider(db, docker, fixedNow)
}

func enabledRemoteConfig() RemoteAccessConfig {
	cfg := DefaultRemoteAccessConfig()
	cfg.Enabled = true
	cfg.Hostname = "lawn-bot"
	cfg.AuthKey = "tskey-auth-kABC123CNTRL-secretsecret"
	return cfg
}

func envValue(env []string, key string) (string, bool) {
	for _, kv := range env {
		if strings.HasPrefix(kv, key+"=") {
			return strings.TrimPrefix(kv, key+"="), true
		}
	}
	return "", false
}

func TestRemoteAccessConfigRoundTripsThroughDB(t *testing.T) {
	// Arrange
	db := types.NewMockDBProvider()
	cfg := enabledRemoteConfig()
	cfg.ServeHTTPS = false
	cfg.Image = "docker.io/tailscale/tailscale:v1.99.0"

	// Act
	require.NoError(t, SaveRemoteAccessConfig(db, cfg))
	loaded := LoadRemoteAccessConfig(db)

	// Assert
	assert.Equal(t, cfg, loaded)
}

func TestRemoteAccessConfigDefaultsWhenDBIsEmpty(t *testing.T) {
	loaded := LoadRemoteAccessConfig(types.NewMockDBProvider())
	assert.Equal(t, DefaultRemoteAccessConfig(), loaded)
	assert.False(t, loaded.Enabled)
	assert.Equal(t, DefaultRemoteAccessImage, loaded.Image)
}

func TestRemoteAccessConfigDefaultImageIsNotPersistedAsAnOverride(t *testing.T) {
	db := types.NewMockDBProvider()
	explicit := enabledRemoteConfig()
	explicit.Image = "tailscale/tailscale:v1.98.10"
	require.NoError(t, SaveRemoteAccessConfig(db, explicit))
	_, err := db.Get(remoteAccessKeyImage)
	require.NoError(t, err, "an explicit image is an override and is stored")

	require.NoError(t, SaveRemoteAccessConfig(db, enabledRemoteConfig()))

	_, err = db.Get(remoteAccessKeyImage)
	assert.Error(t, err, "the default image must not be pinned in the DB")
	assert.Equal(t, DefaultRemoteAccessImage, LoadRemoteAccessConfig(db).Image)
}

func TestRemoteAccessConfigRetiredImageFallsBackToTheDefault(t *testing.T) {
	// v1.102.4 was shipped as the default but Docker Hub never published it.
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set(remoteAccessKeyImage, []byte("tailscale/tailscale:v1.102.4")))

	assert.Equal(t, DefaultRemoteAccessImage, LoadRemoteAccessConfig(db).Image)
}

func TestRemoteAccessConfigClearingAuthKeyDeletesTheDBEntry(t *testing.T) {
	db := types.NewMockDBProvider()
	require.NoError(t, SaveRemoteAccessConfig(db, enabledRemoteConfig()))
	cleared := enabledRemoteConfig()
	cleared.AuthKey = ""

	require.NoError(t, SaveRemoteAccessConfig(db, cleared))

	_, err := db.Get(remoteAccessKeyAuthKey)
	assert.Error(t, err, "auth key must be gone from the DB")
}

func TestRemoteAccessConfigValidateRejectsBadHostnames(t *testing.T) {
	for _, bad := range []string{"", "Lawn Bot", "-mowgli", "mowgli-", "mowgli.local", strings.Repeat("a", 64)} {
		cfg := DefaultRemoteAccessConfig()
		cfg.Hostname = bad
		assert.Error(t, cfg.Validate(), "hostname %q should be rejected", bad)
	}
	for _, good := range []string{"mowgli", "lawn-bot-2", "a", strings.Repeat("a", 63)} {
		cfg := DefaultRemoteAccessConfig()
		cfg.Hostname = good
		assert.NoError(t, cfg.Validate(), "hostname %q should be accepted", good)
	}
}

func TestRemoteAccessConfigValidateRejectsWhitespaceInKeyAndImage(t *testing.T) {
	cfg := DefaultRemoteAccessConfig()
	cfg.AuthKey = "tskey auth"
	assert.Error(t, cfg.Validate())

	cfg = DefaultRemoteAccessConfig()
	cfg.Image = ""
	assert.Error(t, cfg.Validate())

	cfg = DefaultRemoteAccessConfig()
	cfg.Image = "tailscale/tailscale: latest"
	assert.Error(t, cfg.Validate())
}

func TestRemoteAccessConfigValidatePinsTheImageToTheTailscaleRepository(t *testing.T) {
	// The API is unauthenticated and the container runs on the host network:
	// any other repository would be remote code execution in one request.
	for _, bad := range []string{
		"docker.io/attacker/evil:latest", "tailscale/tailscaled:v1", "ghcr.io/tailscale/tailscale:v1.102.3",
		"tailscale/tailscale:", "tailscale/tailscale@sha256:short", "tailscale/tailscale:v1;rm",
	} {
		cfg := DefaultRemoteAccessConfig()
		cfg.Image = bad
		assert.Error(t, cfg.Validate(), "image %q should be rejected", bad)
	}
	for _, good := range []string{
		"tailscale/tailscale", "tailscale/tailscale:v1.102.3", "docker.io/tailscale/tailscale:latest",
		"tailscale/tailscale@sha256:" + strings.Repeat("ab", 32),
	} {
		cfg := DefaultRemoteAccessConfig()
		cfg.Image = good
		assert.NoError(t, cfg.Validate(), "image %q should be accepted", good)
	}
}

func TestRemoteAccessMaskedAuthKeyNeverRevealsTheSecret(t *testing.T) {
	cfg := enabledRemoteConfig()
	masked := cfg.MaskedAuthKey()
	assert.True(t, strings.HasPrefix(masked, "tskey-auth-"))
	assert.NotContains(t, masked, "secretsecret")

	cfg.AuthKey = "short"
	assert.Equal(t, "••••••••", cfg.MaskedAuthKey())
	cfg.AuthKey = ""
	assert.Equal(t, "", cfg.MaskedAuthKey())
}

func TestReconcileDisabledWithNoContainerDoesNothing(t *testing.T) {
	docker := &fakeServiceDocker{}
	p := newTestRemoteAccess(t, DefaultRemoteAccessConfig(), docker)

	require.NoError(t, p.Reconcile(context.Background()))

	assert.Empty(t, docker.pulled)
	assert.Empty(t, docker.created)
	assert.Empty(t, docker.removed)
	assert.Equal(t, RemoteAccessDisabled, p.Status(context.Background()).Phase)
}

func TestReconcileDisabledRemovesAnExistingSidecar(t *testing.T) {
	docker := &fakeServiceDocker{found: true, details: types.ContainerDetails{ID: "old", Running: true}}
	p := newTestRemoteAccess(t, DefaultRemoteAccessConfig(), docker)

	require.NoError(t, p.Reconcile(context.Background()))

	assert.Equal(t, []string{"old"}, docker.removed)
	assert.Empty(t, docker.created)
}

func TestReconcileEnabledCreatesTheSidecarWithTheExpectedSpec(t *testing.T) {
	// Arrange
	docker := &fakeServiceDocker{}
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.api.addr", []byte(":4123")))
	require.NoError(t, SaveRemoteAccessConfig(db, enabledRemoteConfig()))
	p := newRemoteAccessProvider(db, docker, fixedNow)

	// Act
	require.NoError(t, p.Reconcile(context.Background()))

	// Assert
	require.Len(t, docker.created, 1)
	spec := docker.created[0]
	assert.Equal(t, []string{DefaultRemoteAccessImage}, docker.pulled)
	assert.Equal(t, []string{"new-id"}, docker.started)
	assert.Equal(t, RemoteAccessContainerName, spec.Name)
	assert.Equal(t, "host", spec.NetworkMode)
	assert.Equal(t, "unless-stopped", spec.RestartPolicy)
	assert.Equal(t, []string{"ALL"}, spec.CapDrop)
	assert.Equal(t, []string{remoteAccessStateVolume + ":" + remoteAccessStateDir}, spec.Binds)

	hostname, _ := envValue(spec.Env, "TS_HOSTNAME")
	assert.Equal(t, "lawn-bot", hostname)
	userspace, _ := envValue(spec.Env, "TS_USERSPACE")
	assert.Equal(t, "true", userspace)
	authOnce, _ := envValue(spec.Env, "TS_AUTH_ONCE")
	assert.Equal(t, "true", authOnce)
	authKey, _ := envValue(spec.Env, "TS_AUTHKEY")
	assert.Equal(t, "file:"+remoteAccessAuthKeyPath, authKey, "the secret goes through a file, not the env")
	assert.Equal(t, "tskey-auth-kABC123CNTRL-secretsecret\n", string(spec.Files[remoteAccessAuthKeyPath]))

	serveCfg, _ := envValue(spec.Env, "TS_SERVE_CONFIG")
	assert.Equal(t, remoteAccessServeConfigPath, serveCfg)
	var serve map[string]any
	require.NoError(t, json.Unmarshal(spec.Files[remoteAccessServeConfigPath], &serve))
	assert.Contains(t, string(spec.Files[remoteAccessServeConfigPath]), "${TS_CERT_DOMAIN}:443")
	assert.Contains(t, string(spec.Files[remoteAccessServeConfigPath]), "http://127.0.0.1:4123")

	assert.Equal(t, "false", spec.Labels["com.centurylinklabs.watchtower.enable"])
	assert.NotContains(t, spec.Labels, "garden.mowgli.update.image", "must stay unmanaged by the host updater")
	assert.NotEmpty(t, spec.Labels[remoteAccessSpecLabel])
	assert.Equal(t, RemoteAccessRunning, p.Status(context.Background()).Phase)
}

func TestReconcileWithoutAuthKeyOrHTTPSOmitsThoseInputs(t *testing.T) {
	cfg := enabledRemoteConfig()
	cfg.AuthKey = ""
	cfg.ServeHTTPS = false
	docker := &fakeServiceDocker{}
	p := newTestRemoteAccess(t, cfg, docker)

	require.NoError(t, p.Reconcile(context.Background()))

	require.Len(t, docker.created, 1)
	spec := docker.created[0]
	_, hasKey := envValue(spec.Env, "TS_AUTHKEY")
	assert.False(t, hasKey)
	_, hasServe := envValue(spec.Env, "TS_SERVE_CONFIG")
	assert.False(t, hasServe)
	assert.Empty(t, spec.Files)
}

func TestReconcileStartsAStoppedSidecarWithTheSameSpec(t *testing.T) {
	// Arrange: the existing container carries the hash of the current spec.
	docker := &fakeServiceDocker{}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)
	hash := p.buildSpec(p.Config()).Labels[remoteAccessSpecLabel]
	docker.found = true
	docker.details = types.ContainerDetails{ID: "existing", Running: false, Labels: map[string]string{remoteAccessSpecLabel: hash}}

	// Act
	require.NoError(t, p.Reconcile(context.Background()))

	// Assert
	assert.Empty(t, docker.pulled)
	assert.Empty(t, docker.created)
	assert.Empty(t, docker.removed)
	assert.Equal(t, []string{"existing"}, docker.started)
}

func TestReconcileLeavesARunningUpToDateSidecarAlone(t *testing.T) {
	docker := &fakeServiceDocker{}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)
	hash := p.buildSpec(p.Config()).Labels[remoteAccessSpecLabel]
	docker.found = true
	docker.details = types.ContainerDetails{ID: "existing", Running: true, Labels: map[string]string{remoteAccessSpecLabel: hash}}

	require.NoError(t, p.Reconcile(context.Background()))

	assert.Empty(t, docker.started)
	assert.Empty(t, docker.created)
	assert.Equal(t, RemoteAccessRunning, p.Status(context.Background()).Phase)
}

func TestReconcileRecreatesWhenTheSpecChanged(t *testing.T) {
	docker := &fakeServiceDocker{found: true, details: types.ContainerDetails{
		ID: "stale", Running: true, Labels: map[string]string{remoteAccessSpecLabel: "old-hash"},
	}}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)

	require.NoError(t, p.Reconcile(context.Background()))

	assert.Equal(t, []string{"stale"}, docker.removed)
	require.Len(t, docker.created, 1)
	assert.Equal(t, []string{"new-id"}, docker.started)
}

func TestSpecHashChangesWithSettingsAndIgnoresItsOwnLabel(t *testing.T) {
	p := newTestRemoteAccess(t, enabledRemoteConfig(), &fakeServiceDocker{})
	base := p.buildSpec(p.Config())

	changed := enabledRemoteConfig()
	changed.Hostname = "other"
	assert.NotEqual(t, base.Labels[remoteAccessSpecLabel], p.buildSpec(changed).Labels[remoteAccessSpecLabel])

	// Rehashing the labelled spec must reproduce the same fingerprint.
	assert.Equal(t, base.Labels[remoteAccessSpecLabel], specHash(base))
}

func TestReconcilePullFailureIsReportedAsErrorPhase(t *testing.T) {
	docker := &fakeServiceDocker{pullErr: errors.New("registry unreachable")}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)

	err := p.Reconcile(context.Background())

	require.Error(t, err)
	st := p.Status(context.Background())
	assert.Equal(t, RemoteAccessError, st.Phase)
	assert.Contains(t, st.Error, "registry unreachable")
	assert.Empty(t, docker.created)
}

func TestReconcilePullFailureDuringReplacementPreservesExistingSidecar(t *testing.T) {
	docker := &fakeServiceDocker{
		found: true,
		details: types.ContainerDetails{
			ID: "existing", Running: true, Labels: map[string]string{remoteAccessSpecLabel: "old-hash"},
		},
		pullErr: errors.New("registry unreachable"),
	}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)

	err := p.Reconcile(context.Background())

	require.Error(t, err)
	assert.Empty(t, docker.removed, "a failed pull must not remove the working sidecar")
	assert.Empty(t, docker.created)
	st := p.Status(context.Background())
	assert.Equal(t, RemoteAccessError, st.Phase)
	assert.Contains(t, st.Error, "registry unreachable")
}

func TestUpdateConfigNormalisesAndRejectsInvalidInput(t *testing.T) {
	p := newTestRemoteAccess(t, DefaultRemoteAccessConfig(), &fakeServiceDocker{})

	cfg := DefaultRemoteAccessConfig()
	cfg.Hostname = "  Lawn-Bot "
	cfg.AuthKey = " tskey-auth-x "
	require.NoError(t, p.UpdateConfig(cfg))
	assert.Equal(t, "lawn-bot", p.Config().Hostname)
	assert.Equal(t, "tskey-auth-x", p.Config().AuthKey)

	bad := DefaultRemoteAccessConfig()
	bad.Hostname = "not valid"
	assert.Error(t, p.UpdateConfig(bad))
	assert.Equal(t, "lawn-bot", p.Config().Hostname, "a rejected update must not replace the config")
}

func statusJSON(t *testing.T, ts tailscaleStatus) string {
	t.Helper()
	out, err := json.Marshal(ts)
	require.NoError(t, err)
	return string(out)
}

func runningDocker(stdout string) *fakeServiceDocker {
	return &fakeServiceDocker{
		found:   true,
		details: types.ContainerDetails{ID: "sidecar", State: "running", Running: true},
		execFn: func(spec types.ContainerExecSpec) (types.ContainerExecResult, error) {
			return types.ContainerExecResult{ExitCode: 0, Stdout: stdout}, nil
		},
	}
}

func TestStatusWhenDisabledReportsOnlyTheConfig(t *testing.T) {
	docker := &fakeServiceDocker{findErr: errors.New("must not be called")}
	p := newTestRemoteAccess(t, DefaultRemoteAccessConfig(), docker)

	st := p.Status(context.Background())

	assert.False(t, st.Enabled)
	assert.Equal(t, RemoteAccessDisabled, st.Phase)
	assert.Empty(t, st.Error)
	assert.NotNil(t, st.HttpUrls)
	assert.NotNil(t, st.TailscaleIps)
}

func TestStatusExposesTheLoginURLWhileWaitingForLogin(t *testing.T) {
	ts := tailscaleStatus{BackendState: "NeedsLogin", AuthURL: "https://login.tailscale.com/a/abc", Version: "1.102.4"}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), runningDocker(statusJSON(t, ts)))

	st := p.Status(context.Background())

	assert.Equal(t, "running", st.ContainerState)
	assert.Equal(t, "NeedsLogin", st.BackendState)
	assert.Equal(t, "https://login.tailscale.com/a/abc", st.LoginUrl)
	assert.Empty(t, st.HttpUrls)
	assert.Empty(t, st.HttpsUrl)
	assert.Equal(t, "1.102.4", st.Version)
}

func TestStatusListsURLsOnceTheNodeRuns(t *testing.T) {
	ts := tailscaleStatus{BackendState: "Running", TailscaleIPs: []string{"100.64.0.7", "fd7a:115c:a1e0::7"}}
	ts.Self = &struct {
		DNSName  string `json:"DNSName"`
		HostName string `json:"HostName"`
		Online   bool   `json:"Online"`
	}{DNSName: "lawn-bot.tail1234.ts.net.", HostName: "lawn-bot", Online: true}
	ts.CertDomains = []string{"lawn-bot.tail1234.ts.net"}
	ts.CurrentTailnet = &struct {
		MagicDNSSuffix  string `json:"MagicDNSSuffix"`
		MagicDNSEnabled bool   `json:"MagicDNSEnabled"`
	}{MagicDNSSuffix: "tail1234.ts.net", MagicDNSEnabled: true}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), runningDocker(statusJSON(t, ts)))

	st := p.Status(context.Background())

	assert.Equal(t, "Running", st.BackendState)
	assert.Empty(t, st.LoginUrl)
	assert.Equal(t, "lawn-bot.tail1234.ts.net", st.DnsName)
	assert.Equal(t, "https://lawn-bot.tail1234.ts.net", st.HttpsUrl)
	assert.Equal(t, []string{
		"http://lawn-bot.tail1234.ts.net:4006",
		"http://100.64.0.7:4006",
		"http://[fd7a:115c:a1e0::7]:4006",
	}, st.HttpUrls)
	assert.True(t, st.MagicDnsEnabled)
}

func TestStatusOmitsHTTPSWhenServeIsOffOrNoCertDomain(t *testing.T) {
	ts := tailscaleStatus{BackendState: "Running", TailscaleIPs: []string{"100.64.0.7"}, CertDomains: []string{"x.ts.net"}}
	cfg := enabledRemoteConfig()
	cfg.ServeHTTPS = false
	p := newTestRemoteAccess(t, cfg, runningDocker(statusJSON(t, ts)))
	assert.Empty(t, p.Status(context.Background()).HttpsUrl)

	ts.CertDomains = nil
	p = newTestRemoteAccess(t, enabledRemoteConfig(), runningDocker(statusJSON(t, ts)))
	st := p.Status(context.Background())
	assert.Empty(t, st.HttpsUrl)
	assert.Equal(t, []string{"http://100.64.0.7:4006"}, st.HttpUrls)
}

func TestStatusReportsAnAbsentOrStoppedContainer(t *testing.T) {
	p := newTestRemoteAccess(t, enabledRemoteConfig(), &fakeServiceDocker{})
	assert.Equal(t, "absent", p.Status(context.Background()).ContainerState)

	docker := &fakeServiceDocker{found: true, details: types.ContainerDetails{ID: "x", State: "exited", Running: false}}
	p = newTestRemoteAccess(t, enabledRemoteConfig(), docker)
	st := p.Status(context.Background())
	assert.Equal(t, "exited", st.ContainerState)
	assert.Empty(t, st.BackendState)
}

func TestStatusSurfacesAnUnparseableTailscaleReply(t *testing.T) {
	docker := runningDocker("")
	docker.execFn = func(types.ContainerExecSpec) (types.ContainerExecResult, error) {
		return types.ContainerExecResult{ExitCode: 1, Stderr: "failed to connect to local tailscaled"}, nil
	}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)

	st := p.Status(context.Background())

	assert.Equal(t, "running", st.ContainerState)
	assert.Contains(t, st.Error, "failed to connect to local tailscaled")
}

func TestLogoutRunsTailscaleLogoutAndRecreates(t *testing.T) {
	var cmds [][]string
	docker := &fakeServiceDocker{
		found:   true,
		details: types.ContainerDetails{ID: "sidecar", Running: true},
		execFn: func(spec types.ContainerExecSpec) (types.ContainerExecResult, error) {
			cmds = append(cmds, spec.Cmd)
			return types.ContainerExecResult{ExitCode: 0}, nil
		},
	}
	p := newTestRemoteAccess(t, enabledRemoteConfig(), docker)

	require.NoError(t, p.Logout(context.Background()))

	assert.Equal(t, [][]string{{"tailscale", "logout"}}, cmds)
	assert.Equal(t, []string{"sidecar"}, docker.removed)
	assert.Equal(t, RemoteAccessStarting, p.Status(context.Background()).Phase, "a reconcile is pending")
}

func TestLogoutFailsWhenThereIsNoSidecar(t *testing.T) {
	p := newTestRemoteAccess(t, enabledRemoteConfig(), &fakeServiceDocker{})
	assert.Error(t, p.Logout(context.Background()))
}

func TestGuiPortFromDBFallsBackToTheDefault(t *testing.T) {
	db := types.NewMockDBProvider()
	assert.Equal(t, remoteAccessDefaultGUIPort, guiPortFromDB(db))
	require.NoError(t, db.Set("system.api.addr", []byte("0.0.0.0:8080")))
	assert.Equal(t, "8080", guiPortFromDB(db))
	require.NoError(t, db.Set("system.api.addr", []byte("garbage")))
	assert.Equal(t, remoteAccessDefaultGUIPort, guiPortFromDB(db))
}
