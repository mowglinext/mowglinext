package providers

import (
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func manifestBody(tag string, protocol int) string {
	return fmt.Sprintf(`{"tag":%q,"protocol_version":%d,"fw_version":"1.2.3","permutations":{"yf500":{"env":"Yardforce500","board":"BOARD_YARDFORCE500","panel":"PANEL_TYPE_YARDFORCE_500_CLASSIC","file":"f.bin","url":"u","sha256":"s","protocol_version":%d,"fw_version":"1.2.3"}}}`, tag, protocol, protocol)
}

// serveReleases serves /download/<release>/manifest.json for the given
// releases and /latest/manifest.json for the latest stable one.
func serveReleases(t *testing.T, releases map[string]int, latest string) {
	t.Helper()
	mux := http.NewServeMux()
	for tag, protocol := range releases {
		body := manifestBody(tag, protocol)
		mux.HandleFunc("/download/"+tag+"/manifest.json", func(w http.ResponseWriter, _ *http.Request) {
			_, _ = w.Write([]byte(body))
		})
	}
	mux.HandleFunc("/latest/manifest.json", func(w http.ResponseWriter, _ *http.Request) {
		_, _ = w.Write([]byte(manifestBody(latest, 6)))
	})
	server := httptest.NewServer(mux)
	t.Cleanup(server.Close)
	oldBase, oldLatest := firmwareReleaseDownloadBase, latestFirmwareManifestURL
	firmwareReleaseDownloadBase = server.URL + "/download/"
	latestFirmwareManifestURL = server.URL + "/latest/manifest.json"
	t.Cleanup(func() { firmwareReleaseDownloadBase, latestFirmwareManifestURL = oldBase, oldLatest })
}

func TestFirmwareManifestURLForVersion(t *testing.T) {
	for _, tc := range []struct {
		version string
		own     bool
	}{
		{"deployment-07f28b7911de-36068628774-1", true},
		{"v1.4.0", true},
		{"", false},
		{"feat-firmware-params-v7", false},
		{"v1.4", false},
	} {
		url, own := firmwareManifestURLForVersion(tc.version)
		assert.Equal(t, tc.own, own, tc.version)
		if tc.own {
			assert.Contains(t, url, "/"+tc.version+"/manifest.json")
		} else {
			assert.Equal(t, latestFirmwareManifestURL, url)
		}
	}
}

func TestResolveManifestEntryRequiresExactExplicitTarget(t *testing.T) {
	manifest := &firmwareManifest{Permutations: map[string]firmwareManifestEntry{
		"yardforce500b": {
			Env: "Yardforce500B", Board: "BOARD_YARDFORCE500B",
			Panel: "PANEL_TYPE_YARDFORCE_500B_CLASSIC",
		},
		"biltema-rm1000": {
			Env: "BiltemaRM1000", Board: "BOARD_YARDFORCE500B",
			Panel: "PANEL_TYPE_YARDFORCE_900_ECO",
		},
	}}

	entry, err := resolveManifestEntry(manifest, "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", "BiltemaRM1000")
	require.NoError(t, err)
	assert.Equal(t, "BiltemaRM1000", entry.Env)

	_, err = resolveManifestEntry(manifest, "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_500B_CLASSIC", "")
	assert.NoError(t, err) // Legacy board/panel configs retain their old selection behavior.
	_, err = resolveManifestEntry(manifest, "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_500B_CLASSIC", "MissingRM1000")
	assert.ErrorContains(t, err, `target "MissingRM1000"`)
	_, err = resolveManifestEntry(manifest, "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_500_CLASSIC", "BiltemaRM1000")
	assert.ErrorContains(t, err, `target "BiltemaRM1000"`)
}

// A dev deployment must flash the firmware built with it (its protocol matches
// the ROS2 image), never the latest stable one from main.
func TestInstallManifestUsesTheDeploymentRelease(t *testing.T) {
	serveReleases(t, map[string]int{"deployment-abc-1-1": 7}, "v1.4.0")
	manifest, source, err := fetchInstallFirmwareManifest("deployment-abc-1-1")
	require.NoError(t, err)
	assert.True(t, source.OwnRelease)
	assert.Equal(t, "deployment-abc-1-1", source.Release)
	assert.Equal(t, 7, manifest.ProtocolVersion)
}

func TestInstallManifestFallsBackToLatestWhenTheReleaseHasNone(t *testing.T) {
	serveReleases(t, map[string]int{}, "v1.4.0")
	manifest, source, err := fetchInstallFirmwareManifest("deployment-old-1-1")
	require.NoError(t, err)
	assert.False(t, source.OwnRelease)
	assert.Equal(t, "v1.4.0", source.Release)
	assert.Equal(t, 6, manifest.ProtocolVersion)
}

func TestInstallManifestForANonReleaseBuildUsesLatest(t *testing.T) {
	serveReleases(t, map[string]int{}, "v1.4.0")
	_, source, err := fetchInstallFirmwareManifest("")
	require.NoError(t, err)
	assert.False(t, source.OwnRelease)
	assert.Equal(t, "v1.4.0", source.Release)
}

// stubActiveDeploymentSource replaces activeDeploymentSource for the duration
// of the test, restoring the original on cleanup — same pattern as
// serveReleases' save/restore of the URL vars above.
func stubActiveDeploymentSource(t *testing.T, repo, releaseTag string, ok bool) {
	t.Helper()
	old := activeDeploymentSource
	activeDeploymentSource = func() (string, string, bool) { return repo, releaseTag, ok }
	t.Cleanup(func() { activeDeploymentSource = old })
}

// The whole point of this fix: a FORK's own dev/custom deployment must fetch
// its manifest from the FORK, never from upstream — buildinfo.Version alone
// ("deployment-<sha>-<run>-<attempt>") cannot say which repository minted it,
// so before this fix ownReleaseManifestURL always assumed upstream, 404'd
// against a release that only ever existed on the fork, and silently fell
// back to upstream's stable firmware (mowglinext#XXX) — offering only "main"
// firmware for a "dev" build, exactly the field-reported symptom.
func TestOwnReleaseManifestURLPrefersTheActiveDeploymentSource(t *testing.T) {
	stubActiveDeploymentSource(t, "fwelvering/mowglinext", "deployment-abc-1-1", true)
	url, own := ownReleaseManifestURL("deployment-abc-1-1")
	assert.True(t, own)
	assert.Equal(t, "https://github.com/fwelvering/mowglinext/releases/download/deployment-abc-1-1/manifest.json", url)
}

// When the worker can't say (unreachable, or genuinely no active deployment
// yet — a very early boot), fall back to the old upstream-only heuristic
// rather than erroring out.
func TestOwnReleaseManifestURLFallsBackWhenWorkerUnreachable(t *testing.T) {
	stubActiveDeploymentSource(t, "", "", false)
	url, own := ownReleaseManifestURL("deployment-abc-1-1")
	assert.True(t, own)
	assert.Equal(t, firmwareReleaseDownloadBase+"deployment-abc-1-1/manifest.json", url)
}

// End-to-end: a fork's dev deployment must resolve its OWN manifest (from the
// fork), not upstream's stable one, even though the version string alone
// looks identical to an upstream deployment build.
func TestInstallManifestUsesTheForkOwnReleaseWhenTheWorkerReportsIt(t *testing.T) {
	mux := http.NewServeMux()
	mux.HandleFunc("/fwelvering/mowglinext/releases/download/deployment-fork-1-1/manifest.json", func(w http.ResponseWriter, _ *http.Request) {
		_, _ = w.Write([]byte(manifestBody("deployment-fork-1-1", 9)))
	})
	server := httptest.NewServer(mux)
	t.Cleanup(server.Close)
	oldGithub := githubReleaseBase
	githubReleaseBase = server.URL + "/"
	t.Cleanup(func() { githubReleaseBase = oldGithub })
	stubActiveDeploymentSource(t, "fwelvering/mowglinext", "deployment-fork-1-1", true)

	// Also serve the upstream fallback, so a wrong fallback to it (a
	// regression) would still succeed but with the WRONG protocol — making
	// the assertion below fail loudly instead of masking the bug as an error.
	serveReleases(t, map[string]int{}, "v1.4.0")

	manifest, source, err := fetchInstallFirmwareManifest("deployment-fork-1-1")
	require.NoError(t, err)
	assert.True(t, source.OwnRelease)
	assert.Equal(t, "deployment-fork-1-1", source.Release)
	assert.Equal(t, 9, manifest.ProtocolVersion)
}

// serveUpdaterState starts a real unix-socket HTTP server answering GET
// /v1/state with the given raw JSON body, and points MOWGLI_UPDATER_SOCKET at
// it for the duration of the test — this exercises activeDeploymentSource's
// OWN HTTP-decode logic end to end (unlike stubActiveDeploymentSource above,
// which replaces the whole function for the layers built on top of it).
func serveUpdaterState(t *testing.T, body string) {
	t.Helper()
	socket := filepath.Join(t.TempDir(), "updater.sock")
	listener, err := net.Listen("unix", socket)
	require.NoError(t, err)
	mux := http.NewServeMux()
	mux.HandleFunc("GET /v1/state", func(w http.ResponseWriter, _ *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		_, _ = w.Write([]byte(body))
	})
	server := &http.Server{Handler: mux}
	go func() { _ = server.Serve(listener) }()
	t.Cleanup(func() { _ = server.Close() })
	t.Setenv("MOWGLI_UPDATER_SOCKET", socket)
}

// The maintainer's own review of #811: a manual update/repair (#815) can
// regenerate the stack straight from the checkout and clear state.Active
// without a fresh runtime reconciliation, so state.Active can go on naming a
// repository/release that is no longer what is actually running. runtime's
// own identity field already detects exactly this ("drifted") — this pins
// that activeDeploymentSource refuses to trust state.Active when it does,
// falling back to the safe upstream-only heuristic instead of resolving
// firmware from a deployment that is no longer installed.
func TestActiveDeploymentSourceRefusesADriftedRuntime(t *testing.T) {
	serveUpdaterState(t, `{"state":{"active":{"source":{"repository":"fwelvering/mowglinext"},"release_tag":"deployment-old-1-1"}},"runtime":{"identity":"drifted"}}`)
	repo, releaseTag, ok := activeDeploymentSource()
	assert.False(t, ok)
	assert.Empty(t, repo)
	assert.Empty(t, releaseTag)
}

// A "matched" (or any other non-drifted) identity is trusted normally.
func TestActiveDeploymentSourceTrustsAMatchedRuntime(t *testing.T) {
	serveUpdaterState(t, `{"state":{"active":{"source":{"repository":"fwelvering/mowglinext"},"release_tag":"deployment-abc-1-1"}},"runtime":{"identity":"matched"}}`)
	repo, releaseTag, ok := activeDeploymentSource()
	assert.True(t, ok)
	assert.Equal(t, "fwelvering/mowglinext", repo)
	assert.Equal(t, "deployment-abc-1-1", releaseTag)
}

// "unknown" (runtime.go: before the first reconcile pass, or mid-job) is NOT
// drift — it means "not yet confirmed", not "confirmed wrong". Treating it as
// drift would fall back to the upstream-only heuristic on every fresh boot,
// which is exactly the failure #811 exists to fix.
func TestActiveDeploymentSourceTrustsAnUnknownRuntime(t *testing.T) {
	serveUpdaterState(t, `{"state":{"active":{"source":{"repository":"fwelvering/mowglinext"},"release_tag":"deployment-abc-1-1"}},"runtime":{"identity":"unknown"}}`)
	repo, releaseTag, ok := activeDeploymentSource()
	assert.True(t, ok)
	assert.Equal(t, "fwelvering/mowglinext", repo)
	assert.Equal(t, "deployment-abc-1-1", releaseTag)
}

func TestAvailableFirmwareForTheSavedBoard(t *testing.T) {
	serveReleases(t, map[string]int{}, "v1.4.0")
	db := types.NewMockDBProvider()
	fp := NewFirmwareProvider(db, nil)

	// No board saved yet: nothing to offer, and no network call needed.
	result, err := fp.AvailableFirmware()
	require.NoError(t, err)
	assert.False(t, result.Available)
	assert.Empty(t, result.Board)

	require.NoError(t, db.Set("gui.firmware.config",
		[]byte(`{"boardType":"BOARD_YARDFORCE500","panelType":"PANEL_TYPE_YARDFORCE_500_CLASSIC"}`)))
	result, err = fp.AvailableFirmware()
	require.NoError(t, err)
	assert.True(t, result.Available)
	assert.Equal(t, "1.2.3", result.FwVersion)
	assert.Equal(t, 6, result.ProtocolVersion)
	assert.Equal(t, "v1.4.0", result.Release)

	// A board with no prebuilt binary is reported, not an error.
	require.NoError(t, db.Set("gui.firmware.config", []byte(`{"boardType":"BOARD_LUV1000RI"}`)))
	result, err = fp.AvailableFirmware()
	require.NoError(t, err)
	assert.False(t, result.Available)
	assert.Equal(t, "BOARD_LUV1000RI", result.Board)

	// An RM1000 selection must not be reported available from the ordinary
	// Yardforce500B binary when the release has no RM1000 permutation.
	require.NoError(t, db.Set("gui.firmware.config", []byte(`{"boardType":"BOARD_YARDFORCE500B","panelType":"PANEL_TYPE_YARDFORCE_900_ECO","firmwareTarget":"BiltemaRM1000"}`)))
	result, err = fp.AvailableFirmware()
	require.NoError(t, err)
	assert.False(t, result.Available)

	require.NoError(t, db.Set("gui.firmware.config", []byte(`{"boardType":"BOARD_YARDFORCE500B","panelType":"PANEL_TYPE_YARDFORCE_500B_CLASSIC","firmwareSelectionModel":"BiltemaRM1000"}`)))
	result, err = fp.AvailableFirmware()
	require.NoError(t, err)
	assert.False(t, result.Available)
}
