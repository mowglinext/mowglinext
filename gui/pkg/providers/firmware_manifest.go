package providers

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"io"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
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
	MCU             string `json:"mcu"`
	FlashAddress    string `json:"flash_address"`
	FlashSize       int    `json:"flash_size"`
	Size            int    `json:"size"`
	USBDFU          bool   `json:"usb_dfu"`
}

// ManifestUSBArtifactSource is deliberately stricter than the legacy ST-Link
// resolver. USB DFU accepts only a release manifest that explicitly declares
// the F401's flash contract and USB-DFU support.
type ManifestUSBArtifactSource struct {
	ManifestURL string
	Client      *http.Client
}

func (s ManifestUSBArtifactSource) Resolve(ctx context.Context, request types.FirmwareUpdateRequest) (FirmwareArtifact, error) {
	url := s.ManifestURL
	if url == "" {
		url = DefaultFirmwareManifestURL
	}
	client := s.Client
	if client == nil {
		client = &http.Client{Timeout: manifestDownloadTimeout}
	}
	m, err := fetchFirmwareManifestWithClient(ctx, url, client)
	if err != nil {
		return FirmwareArtifact{}, err
	}
	if m.Schema != 2 {
		return FirmwareArtifact{}, xerrors.Errorf("USB manifest schema must be 2")
	}
	e, err := resolveManifestEntry(m, request.Board, request.Panel)
	if err != nil {
		return FirmwareArtifact{}, err
	}
	if e.Board != request.Board || e.Env != request.Environment || e.Panel != request.Panel || e.ProtocolVersion != m.ProtocolVersion || e.FwVersion != m.FwVersion {
		return FirmwareArtifact{}, xerrors.Errorf("USB manifest top-level and entry metadata disagree")
	}
	if e.URL == "" || e.File == "" || e.Size <= 0 || e.FlashSize != maxDFUImage || e.FlashAddress != "0x08000000" || !e.USBDFU {
		return FirmwareArtifact{}, xerrors.Errorf("USB manifest entry is incomplete or not DFU-approved")
	}
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, e.URL, nil)
	if err != nil {
		return FirmwareArtifact{}, err
	}
	resp, err := client.Do(req)
	if err != nil {
		return FirmwareArtifact{}, xerrors.Errorf("downloading USB artifact: %w", err)
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return FirmwareArtifact{}, xerrors.Errorf("USB artifact returned HTTP %d", resp.StatusCode)
	}
	data, err := io.ReadAll(io.LimitReader(resp.Body, maxDFUImage+1))
	if err != nil {
		return FirmwareArtifact{}, err
	}
	if len(data) != e.Size {
		return FirmwareArtifact{}, xerrors.Errorf("USB artifact size mismatch")
	}
	return FirmwareArtifact{Board: e.Board, Environment: e.Env, Panel: e.Panel, MCU: e.MCU, FlashAddress: e.FlashAddress, FlashSize: e.FlashSize, USBDfu: e.USBDFU, ProtocolVersion: e.ProtocolVersion, FirmwareVersion: e.FwVersion, SHA256: e.Sha256, Size: e.Size, Bytes: data}, nil
}

// firmwareManifest is the top-level manifest.json document.
type firmwareManifest struct {
	Schema          int                              `json:"schema"`
	Tag             string                           `json:"tag"`
	ProtocolVersion int                              `json:"protocol_version"`
	FwVersion       string                           `json:"fw_version"`
	GitShort        string                           `json:"git_short"`
	Permutations    map[string]firmwareManifestEntry `json:"permutations"`
}

// fetchFirmwareManifest downloads and parses the release manifest.
func fetchFirmwareManifest(url string) (*firmwareManifest, error) {
	client := &http.Client{Timeout: manifestDownloadTimeout}
	return fetchFirmwareManifestWithClient(context.Background(), url, client)
}

func fetchFirmwareManifestWithClient(ctx context.Context, url string, client *http.Client) (*firmwareManifest, error) {
	req, err := http.NewRequestWithContext(ctx, http.MethodGet, url, nil)
	if err != nil {
		return nil, xerrors.Errorf("creating firmware manifest request: %w", err)
	}
	resp, err := client.Do(req)
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
