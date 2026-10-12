package providers

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/sirupsen/logrus"
)

// RemoteAccessProvider owns the optional remote-access sidecar: a Tailscale
// node (`mowgli-remote`) running in userspace-networking mode on the host
// network, so the GUI (and only what the tailnet's ACLs allow) is reachable
// from anywhere without opening a port on the operator's router.
//
// The GUI has no authentication of its own (see gui/CLAUDE.md § Safety), so
// exposure is deliberately limited to tailnet members: Funnel (public
// internet) is never configured here.
//
// The container lives OUTSIDE the installer's Compose stack: it is created
// and reconciled through the Docker socket from the operator's settings, the
// same way the GNSS configurator runs its throwaway containers. It carries no
// `garden.mowgli.update.*` label, so the host updater treats it as unmanaged
// (like MQTT) and never restarts or replaces it.
const (
	RemoteAccessContainerName = "mowgli-remote"

	remoteAccessStateVolume     = "mowgli_remote_state"
	remoteAccessStateDir        = "/var/lib/tailscale"
	remoteAccessServeConfigPath = "/config/serve.json"
	remoteAccessAuthKeyPath     = "/config/authkey"
	remoteAccessSpecLabel       = "garden.mowgli.remote.spec"
	remoteAccessDefaultGUIPort  = "4006"

	remoteAccessReconcileTimeout = 10 * time.Minute // image pull on a Pi over a slow uplink
	remoteAccessStatusTimeout    = 15 * time.Second
)

// RemoteAccessPhase is the provider's own view of the sidecar lifecycle. It
// says nothing about whether the node is logged in — that is BackendState.
type RemoteAccessPhase string

const (
	RemoteAccessDisabled RemoteAccessPhase = "disabled"
	RemoteAccessPulling  RemoteAccessPhase = "pulling"
	RemoteAccessStarting RemoteAccessPhase = "starting"
	RemoteAccessRunning  RemoteAccessPhase = "running"
	RemoteAccessError    RemoteAccessPhase = "error"
)

// RemoteAccessStatus is what the settings page polls.
type RemoteAccessStatus struct {
	Enabled bool              `json:"enabled"`
	Phase   RemoteAccessPhase `json:"phase"`
	Error   string            `json:"error,omitempty"`
	// ContainerState is Docker's state string, or "absent".
	ContainerState string `json:"containerState,omitempty"`
	// BackendState mirrors tailscaled: NoState, NeedsLogin, NeedsMachineAuth,
	// Stopped, Starting, Running.
	BackendState string `json:"backendState,omitempty"`
	// LoginUrl is set while the node waits for an interactive login.
	LoginUrl        string    `json:"loginUrl,omitempty"`
	Hostname        string    `json:"hostname"`
	DnsName         string    `json:"dnsName,omitempty"`
	TailscaleIps    []string  `json:"tailscaleIps"`
	HttpsUrl        string    `json:"httpsUrl,omitempty"`
	HttpUrls        []string  `json:"httpUrls"`
	MagicDnsEnabled bool      `json:"magicDnsEnabled"`
	Health          []string  `json:"health"`
	Version         string    `json:"version,omitempty"`
	CheckedAt       time.Time `json:"checkedAt"`
}

// tailscaleStatus is the subset of `tailscale status --json` we read.
type tailscaleStatus struct {
	Version      string   `json:"Version"`
	BackendState string   `json:"BackendState"`
	AuthURL      string   `json:"AuthURL"`
	TailscaleIPs []string `json:"TailscaleIPs"`
	Self         *struct {
		DNSName  string `json:"DNSName"`
		HostName string `json:"HostName"`
		Online   bool   `json:"Online"`
	} `json:"Self"`
	CertDomains    []string `json:"CertDomains"`
	CurrentTailnet *struct {
		MagicDNSSuffix  string `json:"MagicDNSSuffix"`
		MagicDNSEnabled bool   `json:"MagicDNSEnabled"`
	} `json:"CurrentTailnet"`
	Health []string `json:"Health"`
}

type RemoteAccessProvider struct {
	db      types.IDBProvider
	docker  types.IServiceContainerProvider
	guiPort string
	now     func() time.Time

	mu        sync.Mutex
	cfg       RemoteAccessConfig
	phase     RemoteAccessPhase
	lastError string

	wake chan struct{}
}

// NewRemoteAccessProvider loads the settings and reconciles the sidecar in
// the background, then again after every settings change.
func NewRemoteAccessProvider(db types.IDBProvider, docker types.IServiceContainerProvider) *RemoteAccessProvider {
	p := newRemoteAccessProvider(db, docker, time.Now)
	go p.run()
	p.kick()
	return p
}

// NewIdleRemoteAccessProvider builds a provider WITHOUT the reconcile loop:
// tests drive Reconcile() by hand.
func NewIdleRemoteAccessProvider(db types.IDBProvider, docker types.IServiceContainerProvider) *RemoteAccessProvider {
	return newRemoteAccessProvider(db, docker, time.Now)
}

func newRemoteAccessProvider(db types.IDBProvider, docker types.IServiceContainerProvider, now func() time.Time) *RemoteAccessProvider {
	cfg := LoadRemoteAccessConfig(db)
	phase := RemoteAccessDisabled
	if cfg.Enabled {
		phase = RemoteAccessStarting
	}
	return &RemoteAccessProvider{
		db:      db,
		docker:  docker,
		guiPort: guiPortFromDB(db),
		now:     now,
		cfg:     cfg,
		phase:   phase,
		wake:    make(chan struct{}, 1),
	}
}

// guiPortFromDB reads the API listen port so the sidecar proxies to the real
// GUI port rather than an assumed one.
func guiPortFromDB(db types.IDBProvider) string {
	addr := dbString(db, "system.api.addr", ":"+remoteAccessDefaultGUIPort)
	_, port, err := net.SplitHostPort(addr)
	if err != nil || port == "" {
		return remoteAccessDefaultGUIPort
	}
	return port
}

func (p *RemoteAccessProvider) run() {
	for range p.wake {
		ctx, cancel := context.WithTimeout(context.Background(), remoteAccessReconcileTimeout)
		if err := p.Reconcile(ctx); err != nil {
			logrus.Warnf("remote access: reconcile failed: %v", err)
		}
		cancel()
	}
}

// kick schedules one reconcile; a pending one is not duplicated.
func (p *RemoteAccessProvider) kick() {
	select {
	case p.wake <- struct{}{}:
	default:
	}
}

// Config returns a copy of the current settings (auth key included — callers
// mask it before it leaves the process).
func (p *RemoteAccessProvider) Config() RemoteAccessConfig {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.cfg
}

// UpdateConfig validates, persists and applies new settings. The container
// work happens asynchronously; poll Status() for progress.
func (p *RemoteAccessProvider) UpdateConfig(cfg RemoteAccessConfig) error {
	cfg.Hostname = NormalizeRemoteAccessHostname(cfg.Hostname)
	cfg.AuthKey = strings.TrimSpace(cfg.AuthKey)
	cfg.Image = strings.TrimSpace(cfg.Image)
	if err := cfg.Validate(); err != nil {
		return err
	}
	if err := SaveRemoteAccessConfig(p.db, cfg); err != nil {
		return err
	}
	p.mu.Lock()
	p.cfg = cfg
	if cfg.Enabled {
		p.phase = RemoteAccessStarting
	} else {
		p.phase = RemoteAccessDisabled
	}
	p.lastError = ""
	p.mu.Unlock()
	p.kick()
	return nil
}

// Apply re-runs the reconcile in the background (the "Retry" button).
func (p *RemoteAccessProvider) Apply() {
	p.mu.Lock()
	if p.cfg.Enabled {
		p.phase = RemoteAccessStarting
	}
	p.lastError = ""
	p.mu.Unlock()
	p.kick()
}

func (p *RemoteAccessProvider) setPhase(phase RemoteAccessPhase, errMsg string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.phase = phase
	p.lastError = errMsg
}

// Reconcile makes the Docker state match the settings: absent when disabled,
// running with the current spec when enabled. It is idempotent.
func (p *RemoteAccessProvider) Reconcile(ctx context.Context) error {
	cfg := p.Config()
	if err := p.reconcile(ctx, cfg); err != nil {
		p.setPhase(RemoteAccessError, err.Error())
		return err
	}
	return nil
}

func (p *RemoteAccessProvider) reconcile(ctx context.Context, cfg RemoteAccessConfig) error {
	existing, found, err := p.docker.FindContainerByName(ctx, RemoteAccessContainerName)
	if err != nil {
		return fmt.Errorf("look up %s: %w", RemoteAccessContainerName, err)
	}
	if !cfg.Enabled {
		if found {
			if err := p.docker.ContainerRemove(ctx, existing.ID, true); err != nil {
				return fmt.Errorf("remove %s: %w", RemoteAccessContainerName, err)
			}
			logrus.Infof("remote access: removed %s (disabled); tailnet identity kept in volume %s", RemoteAccessContainerName, remoteAccessStateVolume)
		}
		p.setPhase(RemoteAccessDisabled, "")
		return nil
	}

	spec := p.buildSpec(cfg)
	if found && existing.Labels[remoteAccessSpecLabel] == spec.Labels[remoteAccessSpecLabel] {
		if !existing.Running {
			p.setPhase(RemoteAccessStarting, "")
			if err := p.docker.ContainerStart(ctx, existing.ID); err != nil {
				return fmt.Errorf("start %s: %w", RemoteAccessContainerName, err)
			}
		}
		p.setPhase(RemoteAccessRunning, "")
		return nil
	}
	p.setPhase(RemoteAccessPulling, "")
	if err := p.docker.ImagePull(ctx, spec.Image); err != nil {
		return err
	}
	p.setPhase(RemoteAccessStarting, "")
	if found {
		if err := p.docker.ContainerRemove(ctx, existing.ID, true); err != nil {
			return fmt.Errorf("replace %s: %w", RemoteAccessContainerName, err)
		}
		logrus.Infof("remote access: settings changed, recreating %s", RemoteAccessContainerName)
	}
	id, err := p.docker.ContainerCreateService(ctx, spec)
	if err != nil {
		return fmt.Errorf("create %s: %w", RemoteAccessContainerName, err)
	}
	if err := p.docker.ContainerStart(ctx, id); err != nil {
		return fmt.Errorf("start %s: %w", RemoteAccessContainerName, err)
	}
	logrus.Infof("remote access: %s started as tailnet node %q", RemoteAccessContainerName, cfg.Hostname)
	p.setPhase(RemoteAccessRunning, "")
	return nil
}

// buildSpec is the ONE place the sidecar's container definition lives. The
// spec hash label lets reconcile tell "same settings, just start it" from
// "settings changed, recreate it" without inspecting env or mounts.
func (p *RemoteAccessProvider) buildSpec(cfg RemoteAccessConfig) types.ServiceContainerSpec {
	env := []string{
		"TS_HOSTNAME=" + cfg.Hostname,
		"TS_STATE_DIR=" + remoteAccessStateDir,
		// Userspace networking: no /dev/net/tun, no NET_ADMIN, no privileged
		// flag. Inbound tailnet connections are forwarded to the same port on
		// localhost, which on the host network IS the GUI.
		"TS_USERSPACE=true",
		// Log in only when the state dir holds no identity, so a consumed
		// one-off auth key does not break every restart.
		"TS_AUTH_ONCE=true",
		// The robot keeps its own resolver; tailnet DNS is for the viewers.
		"TS_ACCEPT_DNS=false",
	}
	files := map[string][]byte{}
	if cfg.AuthKey != "" {
		// file: keeps the secret out of `docker inspect` output.
		env = append(env, "TS_AUTHKEY=file:"+remoteAccessAuthKeyPath)
		files[remoteAccessAuthKeyPath] = []byte(cfg.AuthKey + "\n")
	}
	if cfg.ServeHTTPS {
		env = append(env, "TS_SERVE_CONFIG="+remoteAccessServeConfigPath)
		files[remoteAccessServeConfigPath] = buildServeConfig(p.guiPort)
	}
	spec := types.ServiceContainerSpec{
		Name:  RemoteAccessContainerName,
		Image: cfg.Image,
		Env:   env,
		Binds: []string{remoteAccessStateVolume + ":" + remoteAccessStateDir},
		Labels: map[string]string{
			"project": "mowglinext",
			"app":     "remote-access",
			// Not a managed service: neither Watchtower nor the host updater
			// may replace it behind the operator's back.
			"com.centurylinklabs.watchtower.enable": "false",
		},
		NetworkMode:   "host",
		RestartPolicy: "unless-stopped",
		// Userspace networking needs no capability at all; drop them so a
		// compromised sidecar cannot sniff or spoof on the host network.
		CapDrop: []string{"ALL"},
		Files:   files,
	}
	spec.Labels[remoteAccessSpecLabel] = specHash(spec)
	return spec
}

// specHash fingerprints everything that would require a recreate. The hash
// label itself is excluded so the fingerprint is stable.
func specHash(spec types.ServiceContainerSpec) string {
	labels := make([]string, 0, len(spec.Labels))
	for k, v := range spec.Labels {
		if k == remoteAccessSpecLabel {
			continue
		}
		labels = append(labels, k+"="+v)
	}
	sort.Strings(labels)
	fileNames := make([]string, 0, len(spec.Files))
	for name := range spec.Files {
		fileNames = append(fileNames, name)
	}
	sort.Strings(fileNames)
	h := sha256.New()
	write := func(parts ...string) {
		for _, part := range parts {
			h.Write([]byte(part))
			h.Write([]byte{0})
		}
	}
	write(spec.Name, spec.Image, spec.NetworkMode, spec.RestartPolicy)
	write(spec.CapDrop...)
	write(spec.Env...)
	write(spec.Binds...)
	write(labels...)
	for _, name := range fileNames {
		write(name)
		h.Write(spec.Files[name])
		h.Write([]byte{0})
	}
	return hex.EncodeToString(h.Sum(nil))
}

// buildServeConfig is the ipn.ServeConfig containerboot applies once the
// node is up: HTTPS on 443, terminated by tailscaled with the tailnet's
// certificate, reverse-proxied to the GUI. containerboot substitutes
// ${TS_CERT_DOMAIN} with the node's MagicDNS name. Serve preserves the Host
// header, which the GUI's WebSocket origin check depends on.
func buildServeConfig(guiPort string) []byte {
	cfg := map[string]any{
		"TCP": map[string]any{
			"443": map[string]any{"HTTPS": true},
		},
		"Web": map[string]any{
			"${TS_CERT_DOMAIN}:443": map[string]any{
				"Handlers": map[string]any{
					"/": map[string]any{"Proxy": "http://127.0.0.1:" + guiPort},
				},
			},
		},
	}
	out, err := json.MarshalIndent(cfg, "", "  ")
	if err != nil {
		// Static input; cannot fail.
		panic(err)
	}
	return append(out, '\n')
}

// Status reports the sidecar lifecycle plus, when the container runs, the
// node's login state and the URLs the operator can use.
func (p *RemoteAccessProvider) Status(ctx context.Context) RemoteAccessStatus {
	p.mu.Lock()
	cfg := p.cfg
	st := RemoteAccessStatus{
		Enabled:      cfg.Enabled,
		Phase:        p.phase,
		Error:        p.lastError,
		Hostname:     cfg.Hostname,
		TailscaleIps: []string{},
		HttpUrls:     []string{},
		Health:       []string{},
		CheckedAt:    p.now(),
	}
	p.mu.Unlock()
	if !cfg.Enabled {
		return st
	}

	details, found, err := p.docker.FindContainerByName(ctx, RemoteAccessContainerName)
	if err != nil {
		return withStatusError(st, fmt.Errorf("look up %s: %w", RemoteAccessContainerName, err))
	}
	if !found {
		st.ContainerState = "absent"
		return st
	}
	st.ContainerState = details.State
	if !details.Running {
		return st
	}

	res, err := p.docker.ContainerExec(ctx, details.ID, types.ContainerExecSpec{
		Cmd: []string{"tailscale", "status", "--json"},
	})
	if err != nil {
		return withStatusError(st, fmt.Errorf("tailscale status: %w", err))
	}
	var ts tailscaleStatus
	if err := json.Unmarshal([]byte(res.Stdout), &ts); err != nil {
		msg := strings.TrimSpace(res.Stderr)
		if msg == "" {
			msg = strings.TrimSpace(res.Stdout)
		}
		if msg == "" {
			msg = "tailscaled not ready"
		}
		return withStatusError(st, errors.New(msg))
	}
	return applyTailscaleStatus(st, ts, cfg.ServeHTTPS, p.guiPort)
}

func withStatusError(st RemoteAccessStatus, err error) RemoteAccessStatus {
	next := st
	if next.Error == "" {
		next.Error = err.Error()
	}
	return next
}

// applyTailscaleStatus is the pure projection of `tailscale status --json`
// onto the operator-facing status: login URL while waiting, then the
// reachable URLs once the node runs.
func applyTailscaleStatus(st RemoteAccessStatus, ts tailscaleStatus, serveHTTPS bool, guiPort string) RemoteAccessStatus {
	next := st
	next.Version = ts.Version
	next.BackendState = ts.BackendState
	next.Health = append([]string{}, ts.Health...)
	if ts.BackendState == "NeedsLogin" {
		next.LoginUrl = ts.AuthURL
	}
	if ts.Self != nil {
		next.DnsName = strings.TrimSuffix(ts.Self.DNSName, ".")
	}
	if ts.CurrentTailnet != nil {
		next.MagicDnsEnabled = ts.CurrentTailnet.MagicDNSEnabled
	}
	if ts.BackendState != "Running" {
		return next
	}
	next.TailscaleIps = append([]string{}, ts.TailscaleIPs...)
	next.HttpUrls = remoteAccessHTTPURLs(next.TailscaleIps, next.DnsName, next.MagicDnsEnabled, guiPort)
	if serveHTTPS && len(ts.CertDomains) > 0 {
		next.HttpsUrl = "https://" + ts.CertDomains[0]
	}
	return next
}

// remoteAccessHTTPURLs lists the plain-HTTP entry points: the MagicDNS name
// first when the tailnet resolves it, then every tailnet IP.
func remoteAccessHTTPURLs(ips []string, dnsName string, magicDNS bool, guiPort string) []string {
	urls := []string{}
	if magicDNS && dnsName != "" {
		urls = append(urls, "http://"+net.JoinHostPort(dnsName, guiPort))
	}
	for _, ip := range ips {
		if strings.TrimSpace(ip) == "" {
			continue
		}
		urls = append(urls, "http://"+net.JoinHostPort(ip, guiPort))
	}
	return urls
}

// Logout forgets the node's tailnet identity and recreates the sidecar, so
// the next start logs in afresh (interactively, or with the stored key).
func (p *RemoteAccessProvider) Logout(ctx context.Context) error {
	details, found, err := p.docker.FindContainerByName(ctx, RemoteAccessContainerName)
	if err != nil {
		return fmt.Errorf("look up %s: %w", RemoteAccessContainerName, err)
	}
	if !found {
		return errors.New("remote access sidecar is not running")
	}
	if details.Running {
		res, err := p.docker.ContainerExec(ctx, details.ID, types.ContainerExecSpec{Cmd: []string{"tailscale", "logout"}})
		if err != nil {
			return fmt.Errorf("tailscale logout: %w", err)
		}
		if res.ExitCode != 0 {
			msg := strings.TrimSpace(res.Stderr)
			if msg == "" {
				msg = strings.TrimSpace(res.Stdout)
			}
			return fmt.Errorf("tailscale logout failed: %s", msg)
		}
	}
	if err := p.docker.ContainerRemove(ctx, details.ID, true); err != nil {
		return fmt.Errorf("remove %s: %w", RemoteAccessContainerName, err)
	}
	logrus.Infof("remote access: logged %s out of the tailnet", RemoteAccessContainerName)
	p.Apply()
	return nil
}
