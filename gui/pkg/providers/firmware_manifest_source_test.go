package providers

import (
	"fmt"
	"net/http"
	"net/http/httptest"
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
}
