package providers

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"io"
	"net/http"
	"os"
	"strings"
	"time"

	"golang.org/x/xerrors"
)

// DefaultFirmwareManifestURL is the stable "latest release" asset URL for the
// prebuilt-firmware manifest published by .github/workflows/firmware-ci.yml on
// every `v*.*.*` tag (see firmware/scripts/package_release.py for the schema).
// GitHub's /releases/latest/download/<asset> always resolves to the newest
// non-prerelease release, so the GUI never has to know the current tag.
const DefaultFirmwareManifestURL = "https://github.com/mowglinext/mowglinext/releases/latest/download/manifest.json"

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
// on PanelType. Returns an error naming the board when nothing matches, so the
// caller can steer the user to the expert build path instead of flashing a
// wrong or absent binary.
func resolveManifestEntry(m *firmwareManifest, board, panel string) (firmwareManifestEntry, error) {
	var matches []firmwareManifestEntry
	for _, entry := range m.Permutations {
		if entry.Board == board {
			matches = append(matches, entry)
		}
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
