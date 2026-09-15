package providers

import (
	"fmt"
	"regexp"
	"strconv"
	"strings"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// Remote-access settings live in the GUI's key-value DB, NOT in
// mowgli_robot.yaml: the auth key is a secret the operator minted in their
// Tailscale admin console, and nothing on the ROS2 side consumes any of it.
const (
	remoteAccessKeyEnabled    = "remoteAccess.enabled"
	remoteAccessKeyHostname   = "remoteAccess.hostname"
	remoteAccessKeyAuthKey    = "remoteAccess.authKey"
	remoteAccessKeyServeHTTPS = "remoteAccess.serveHttps"
	remoteAccessKeyImage      = "remoteAccess.image"
)

const (
	// DefaultRemoteAccessHostname is the node name requested on the tailnet.
	DefaultRemoteAccessHostname = "mowgli"
	// DefaultRemoteAccessImage is pinned: the sidecar has no health contract
	// with the updater, so a moving tag would change behaviour silently.
	DefaultRemoteAccessImage = "tailscale/tailscale:v1.102.4"
)

var remoteAccessHostnamePattern = regexp.MustCompile(`^[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?$`)

// remoteAccessImagePattern pins the sidecar to the official Tailscale
// repository (any tag or digest). The GUI API has no authentication and this
// container runs on the host network, so a free-form image reference would
// be an arbitrary-code-execution primitive for anyone who can reach :4006.
var remoteAccessImagePattern = regexp.MustCompile(`^(docker\.io/)?tailscale/tailscale(:[A-Za-z0-9][A-Za-z0-9._-]{0,127}|@sha256:[0-9a-f]{64})?$`)

// RemoteAccessConfig is the operator's remote-access (Tailscale) settings.
type RemoteAccessConfig struct {
	Enabled bool
	// Hostname is the tailnet node name (a DNS label).
	Hostname string
	// AuthKey is an optional pre-authorised key; empty means interactive login.
	AuthKey string
	// ServeHTTPS publishes the GUI as https://<hostname>.<tailnet>.ts.net via
	// Tailscale Serve, in addition to the plain http://<tailnet-ip>:<port>.
	ServeHTTPS bool
	// Image is the sidecar image reference.
	Image string
}

// DefaultRemoteAccessConfig is a fresh install: off, default node name,
// HTTPS publishing on, pinned image.
func DefaultRemoteAccessConfig() RemoteAccessConfig {
	return RemoteAccessConfig{
		Enabled:    false,
		Hostname:   DefaultRemoteAccessHostname,
		ServeHTTPS: true,
		Image:      DefaultRemoteAccessImage,
	}
}

// MaskedAuthKey is the only form of the key that ever leaves the backend.
func (c RemoteAccessConfig) MaskedAuthKey() string {
	if c.AuthKey == "" {
		return ""
	}
	const shown = 11 // "tskey-auth-" — enough to recognise the key type
	if len(c.AuthKey) <= shown+4 {
		return "••••••••"
	}
	return c.AuthKey[:shown] + "••••••••"
}

// Validate rejects a config the provider could not act on.
func (c RemoteAccessConfig) Validate() error {
	if !remoteAccessHostnamePattern.MatchString(c.Hostname) {
		return fmt.Errorf("hostname must be a DNS label: lowercase letters, digits and hyphens, 1-63 characters")
	}
	if strings.ContainsAny(c.AuthKey, " \t\r\n") {
		return fmt.Errorf("authKey must not contain whitespace")
	}
	if err := validateImageReference(c.Image); err != nil {
		return err
	}
	return nil
}

func validateImageReference(ref string) error {
	if strings.TrimSpace(ref) == "" {
		return fmt.Errorf("image is required")
	}
	if !remoteAccessImagePattern.MatchString(ref) {
		return fmt.Errorf("image must be tailscale/tailscale with an optional tag or sha256 digest")
	}
	return nil
}

// NormalizeRemoteAccessHostname lowercases and trims a hostname so the
// validation and the tailnet see the same label.
func NormalizeRemoteAccessHostname(raw string) string {
	return strings.ToLower(strings.TrimSpace(raw))
}

// LoadRemoteAccessConfig reads the settings from the DB, falling back to the
// defaults for every absent or unparseable key.
func LoadRemoteAccessConfig(db types.IDBProvider) RemoteAccessConfig {
	cfg := DefaultRemoteAccessConfig()
	cfg.Enabled = dbBool(db, remoteAccessKeyEnabled, cfg.Enabled)
	cfg.Hostname = NormalizeRemoteAccessHostname(dbString(db, remoteAccessKeyHostname, cfg.Hostname))
	cfg.AuthKey = dbString(db, remoteAccessKeyAuthKey, "")
	cfg.ServeHTTPS = dbBool(db, remoteAccessKeyServeHTTPS, cfg.ServeHTTPS)
	cfg.Image = dbString(db, remoteAccessKeyImage, cfg.Image)
	return cfg
}

// SaveRemoteAccessConfig persists every key. The auth key is written verbatim
// to the DB and nowhere else; an empty key deletes the entry.
func SaveRemoteAccessConfig(db types.IDBProvider, cfg RemoteAccessConfig) error {
	writes := []struct {
		key   string
		value string
	}{
		{remoteAccessKeyEnabled, strconv.FormatBool(cfg.Enabled)},
		{remoteAccessKeyHostname, cfg.Hostname},
		{remoteAccessKeyServeHTTPS, strconv.FormatBool(cfg.ServeHTTPS)},
		{remoteAccessKeyImage, cfg.Image},
	}
	for _, w := range writes {
		if err := db.Set(w.key, []byte(w.value)); err != nil {
			return fmt.Errorf("persist %s: %w", w.key, err)
		}
	}
	if cfg.AuthKey == "" {
		if err := db.Delete(remoteAccessKeyAuthKey); err != nil {
			return fmt.Errorf("clear remote-access auth key: %w", err)
		}
		return nil
	}
	if err := db.Set(remoteAccessKeyAuthKey, []byte(cfg.AuthKey)); err != nil {
		return fmt.Errorf("persist remote-access auth key: %w", err)
	}
	return nil
}
