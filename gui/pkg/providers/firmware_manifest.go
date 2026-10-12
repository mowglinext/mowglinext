package providers

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"io"
	"net/http"
	"os"
	"regexp"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/updater"
	"golang.org/x/xerrors"
)

// DefaultFirmwareManifestURL is the stable "latest release" asset URL for the
// prebuilt-firmware manifest published by .github/workflows/firmware-ci.yml on
// every `v*.*.*` tag (see firmware/scripts/package_release.py for the schema).
// GitHub's /releases/latest/download/<asset> always resolves to the newest
// non-prerelease release, so the GUI never has to know the current tag.
const DefaultFirmwareManifestURL = "https://github.com/mowglinext/mowglinext/releases/latest/download/manifest.json"

// firmwareReleaseDownloadBase is where a specific release's assets live, and
// latestFirmwareManifestURL the latest-stable fallback. Variables only so the
// tests can point them at a local server. Both are the UPSTREAM repository —
// used only by firmwareManifestURLForVersion's heuristic fallback below, when
// activeDeploymentSource can't say which repository this installation is
// actually running (see ownReleaseManifestURL).
var (
	firmwareReleaseDownloadBase = "https://github.com/mowglinext/mowglinext/releases/download/"
	latestFirmwareManifestURL   = DefaultFirmwareManifestURL
	// githubReleaseBase composes a manifest URL for an ARBITRARY repository
	// (the active deployment's own, which may be a fork) — a variable, like
	// the two above, only so tests can point it at a local server.
	githubReleaseBase = "https://github.com/"
)

// stableReleaseTag matches a production release (vMAJOR.MINOR.PATCH).
var stableReleaseTag = regexp.MustCompile(`^v(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$`)

// firmwareManifestURLForVersion returns the manifest published WITH the given
// GUI build, assuming it is UPSTREAM's own release: every deployment release
// (dev, custom branch) and every stable vX.Y.Z release carries the firmware
// built from the same revision, i.e. the one whose wire protocol matches the
// ROS2 image of that deployment. A build that is not a release (local, branch
// image) gets the latest stable manifest; the bool reports whether the URL is
// the build's own release.
//
// This is a FALLBACK HEURISTIC only, used by ownReleaseManifestURL below when
// the update worker can't say which repository is actually running — a
// version string like "deployment-<sha>-<run>-<attempt>" is, by itself,
// silent about which repository minted it, so guessing upstream is wrong for
// any fork's own dev/custom deployment (mowglinext#XXX: it 404s every "own
// release" fetch against a release that only exists on the fork, and
// silently, permanently falls back to upstream's stable/main firmware
// instead of the matching one).
func firmwareManifestURLForVersion(version string) (string, bool) {
	if strings.HasPrefix(version, "deployment-") || stableReleaseTag.MatchString(version) {
		return firmwareReleaseDownloadBase + version + "/manifest.json", true
	}
	return latestFirmwareManifestURL, false
}

// updaterSocketPath mirrors UpdaterRoutes' (pkg/api/updater.go) own socket
// resolution. Duplicated rather than imported to avoid pkg/providers
// depending on pkg/api.
func updaterSocketPath() string {
	if s := os.Getenv("MOWGLI_UPDATER_SOCKET"); s != "" {
		return s
	}
	return "/run/mowgli-updater/updater.sock"
}

// activeDeploymentSource asks the local update worker (over the same unix
// socket UpdaterRoutes proxies to the frontend as GET /api/system/updater/
// state) which repository and release tag this installation is ACTUALLY
// running — the one place that information genuinely lives, since it is set
// once at update time from the deployment that was installed
// (updater.Deployment.Source.Repository / .ReleaseTag), not derivable from a
// version string alone. A package var, like firmwareReleaseDownloadBase/
// latestFirmwareManifestURL/githubReleaseBase above, so tests can stub it
// without a real socket. ok is false when the worker is unreachable (a very
// early boot, or an install predating the worker), reports no active
// deployment yet, or — PR #811 review — reports runtime.identity == "drifted":
// a manual update/repair (root CLAUDE.md's install codemap; #815) can
// regenerate the stack straight from the checkout and clear state.Active
// without the worker's own record of "what's installed" being refreshed to
// match, so state.Active can name a repository/release that is no longer
// what is actually running. runtime.identity is the worker's own live
// reconciliation of state.Active.InstalledImages against the real running
// containers (updater/runtime.go reconcile()) — "drifted" means they no
// longer agree, so state.Active is exactly the kind of stale record this
// function must not trust. Any other identity (including "unknown", the
// value before the first reconcile pass or while a job is in flight) is NOT
// treated as drift: it is "not yet confirmed", not "confirmed wrong", and
// erring toward it would make this fall back to the upstream-only heuristic
// on every fresh boot, defeating the fix this function exists for.
var activeDeploymentSource = func() (repo, releaseTag string, ok bool) {
	client := updater.Client(updaterSocketPath())
	client.Timeout = 5 * time.Second
	resp, err := client.Get("http://updater/v1/state")
	if err != nil {
		return "", "", false
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return "", "", false
	}
	var body struct {
		State struct {
			Active *struct {
				Source struct {
					Repository string `json:"repository"`
				} `json:"source"`
				ReleaseTag string `json:"release_tag"`
			} `json:"active"`
		} `json:"state"`
		Runtime struct {
			Identity string `json:"identity"`
		} `json:"runtime"`
	}
	if err := json.NewDecoder(resp.Body).Decode(&body); err != nil || body.State.Active == nil {
		return "", "", false
	}
	if body.Runtime.Identity == "drifted" {
		return "", "", false
	}
	repo, releaseTag = body.State.Active.Source.Repository, body.State.Active.ReleaseTag
	return repo, releaseTag, repo != "" && releaseTag != ""
}

// ownReleaseManifestURL resolves the manifest URL for THIS installation's own
// release, preferring the repository the update worker reports it is
// actually running (activeDeploymentSource) — the authoritative answer,
// correct for upstream, a fork, or any other trusted source — and falling
// back to firmwareManifestURLForVersion's upstream-only heuristic only when
// the worker can't say (see its doc comment for why guessing upstream is
// unsafe otherwise).
func ownReleaseManifestURL(version string) (string, bool) {
	if repo, releaseTag, ok := activeDeploymentSource(); ok {
		return githubReleaseBase + repo + "/releases/download/" + releaseTag + "/manifest.json", true
	}
	return firmwareManifestURLForVersion(version)
}

// FirmwareManifestSource says which manifest a flash would use.
type FirmwareManifestSource struct {
	URL string `json:"url"`
	// Release is the tag the manifest belongs to.
	Release string `json:"release"`
	// OwnRelease is true when the manifest comes from the release this GUI
	// was installed from, false for the latest-stable fallback.
	OwnRelease bool `json:"own_release"`
}

// fetchInstallFirmwareManifest fetches the manifest of the running release,
// falling back to the latest stable one when that release has none (a
// deployment published before deployments carried firmware, or a build that
// is not a release). The fallback is reported so the flash log can say so.
func fetchInstallFirmwareManifest(version string) (*firmwareManifest, FirmwareManifestSource, error) {
	url, own := ownReleaseManifestURL(version)
	manifest, err := fetchFirmwareManifest(url)
	if err == nil {
		return manifest, FirmwareManifestSource{URL: url, Release: manifest.Tag, OwnRelease: own}, nil
	}
	if !own {
		return nil, FirmwareManifestSource{}, err
	}
	manifest, fallbackErr := fetchFirmwareManifest(latestFirmwareManifestURL)
	if fallbackErr != nil {
		return nil, FirmwareManifestSource{}, xerrors.Errorf("%v; latest stable: %w", err, fallbackErr)
	}
	return manifest, FirmwareManifestSource{URL: latestFirmwareManifestURL, Release: manifest.Tag}, nil
}

// manifestDownloadTimeout bounds the manifest fetch and the binary download so a
// hung release server can never wedge the flash flow.
const manifestDownloadTimeout = 60 * time.Second

// firmwareManifestEntry is one prebuilt permutation. Mirrors the per-permutation
// object package_release.py emits: {env, board, panel, file, url, sha256,
// protocol_version, fw_version}.
type firmwareManifestEntry struct {
	Env             string `json:"env"`
	Board           string `json:"board"`
	Panel           string `json:"panel"`
	File            string `json:"file"`
	URL             string `json:"url"`
	Sha256          string `json:"sha256"`
	ProtocolVersion int    `json:"protocol_version"`
	FwVersion       string `json:"fw_version"`
}

// firmwareManifest is the top-level manifest.json document.
type firmwareManifest struct {
	Tag             string                           `json:"tag"`
	ProtocolVersion int                              `json:"protocol_version"`
	FwVersion       string                           `json:"fw_version"`
	GitShort        string                           `json:"git_short"`
	Permutations    map[string]firmwareManifestEntry `json:"permutations"`
}

// fetchFirmwareManifest downloads and parses the release manifest.
func fetchFirmwareManifest(url string) (*firmwareManifest, error) {
	client := &http.Client{Timeout: manifestDownloadTimeout}
	resp, err := client.Get(url)
	if err != nil {
		return nil, xerrors.Errorf("fetching firmware manifest: %w", err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return nil, xerrors.Errorf("firmware manifest returned HTTP %d", resp.StatusCode)
	}
	var manifest firmwareManifest
	if err := json.NewDecoder(resp.Body).Decode(&manifest); err != nil {
		return nil, xerrors.Errorf("parsing firmware manifest: %w", err)
	}
	if len(manifest.Permutations) == 0 {
		return nil, xerrors.Errorf("firmware manifest has no permutations")
	}
	return &manifest, nil
}

// resolveManifestEntry picks the prebuilt permutation for a hardware selection.
// It matches on BoardType (the irreducible compile-time axis — MCU family), and
// when a panel is given and more than one entry shares the board, disambiguates
// on PanelType. When target is set it additionally requires an exact manifest
// environment match and verifies the panel, so related board profiles never
// silently select each other's binaries. Returns an error when nothing matches
// so the caller can steer the user to the expert build path instead of flashing
// a wrong or absent binary.
func resolveManifestEntry(m *firmwareManifest, board, panel, target string) (firmwareManifestEntry, error) {
	var matches []firmwareManifestEntry
	for _, entry := range m.Permutations {
		if entry.Board == board {
			matches = append(matches, entry)
		}
	}
	if target != "" {
		for _, entry := range matches {
			if entry.Env == target && (panel == "" || entry.Panel == panel) {
				return entry, nil
			}
		}
		return firmwareManifestEntry{}, xerrors.Errorf(
			"no prebuilt firmware published for board %q and target %q", board, target)
	}
	switch len(matches) {
	case 0:
		return firmwareManifestEntry{}, xerrors.Errorf("no prebuilt firmware published for board %q", board)
	case 1:
		return matches[0], nil
	default:
		for _, entry := range matches {
			if entry.Panel == panel {
				return entry, nil
			}
		}
		return firmwareManifestEntry{}, xerrors.Errorf(
			"multiple prebuilt binaries for board %q but none match panel %q", board, panel)
	}
}

// downloadAndVerify streams the entry's binary to dst and asserts its sha256
// matches the manifest. A mismatch (corrupt or substituted download) is a hard
// error — nothing gets flashed. Returns the path written.
func downloadAndVerify(entry firmwareManifestEntry, dst string) error {
	client := &http.Client{Timeout: manifestDownloadTimeout}
	resp, err := client.Get(entry.URL)
	if err != nil {
		return xerrors.Errorf("downloading firmware binary: %w", err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return xerrors.Errorf("firmware binary returned HTTP %d", resp.StatusCode)
	}

	out, err := os.Create(dst)
	if err != nil {
		return xerrors.Errorf("creating firmware file: %w", err)
	}
	hasher := sha256.New()
	if _, err := io.Copy(io.MultiWriter(out, hasher), resp.Body); err != nil {
		out.Close()
		return xerrors.Errorf("writing firmware file: %w", err)
	}
	if err := out.Close(); err != nil {
		return xerrors.Errorf("closing firmware file: %w", err)
	}

	got := hex.EncodeToString(hasher.Sum(nil))
	want := strings.ToLower(strings.TrimSpace(entry.Sha256))
	if got != want {
		return xerrors.Errorf("firmware sha256 mismatch: expected %s, got %s (refusing to flash)", want, got)
	}
	return nil
}
