package api

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func TestBuildCrossChecks_IncludesGNSSFromEnv(t *testing.T) {
	t.Setenv("HARDWARE_BACKEND", "mowgli")
	t.Setenv("GNSS_BACKEND", "unicore")
	t.Setenv("GPS_CONNECTION", "usb")
	t.Setenv("GPS_PROTOCOL", "UNICORE")
	t.Setenv("GPS_PORT", "/dev/serial/by-id/usb-Unicore_UM980")
	t.Setenv("GPS_BY_ID", "/dev/serial/by-id/usb-Unicore_UM980")
	t.Setenv("GPS_BAUD", "460800")
	t.Setenv("GPS_FRAME_ID", "gps_antenna")

	tempDir := t.TempDir()
	yamlFile := filepath.Join(tempDir, "mowgli_robot.yaml")
	err := os.WriteFile(yamlFile, []byte(`mowgli:
  ros__parameters:
    datum_lat: 48.1234567
    datum_lon: 2.1234567
    dock_pose_x: 1.25
    dock_pose_y: -0.5
    dock_pose_yaw: 0.75
`), 0o644)
	require.NoError(t, err)

	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(yamlFile)))

	checks := buildCrossChecks(db)

	assert.Equal(t, "unicore", checks.GNSS.Backend)
	assert.Equal(t, "mowgli", checks.GNSS.Hardware)
	assert.Equal(t, "usb", checks.GNSS.Connection)
	assert.Equal(t, "UNICORE", checks.GNSS.Protocol)
	assert.Equal(t, "/dev/serial/by-id/usb-Unicore_UM980", checks.GNSS.Port)
	assert.Equal(t, "/dev/serial/by-id/usb-Unicore_UM980", checks.GNSS.ByID)
	assert.Equal(t, "460800", checks.GNSS.Baud)
	assert.Equal(t, "gps_antenna", checks.GNSS.FrameID)
	assert.True(t, checks.GNSS.HasConfig)
	assert.Equal(t, "compose env", checks.GNSS.Source)
}

func TestBuildCrossChecks_GNSSCheckStaysUniversalForUnicore(t *testing.T) {
	t.Setenv("HARDWARE_BACKEND", "mowgli")
	t.Setenv("GNSS_BACKEND", "unicore")
	t.Setenv("GPS_CONNECTION", "usb")
	t.Setenv("GPS_PROTOCOL", "UNICORE")
	t.Setenv("GPS_PORT", "/dev/serial/by-id/usb-Unicore_UM980")
	t.Setenv("GPS_BY_ID", "/dev/serial/by-id/usb-Unicore_UM980")
	t.Setenv("GPS_BAUD", "921600")
	t.Setenv("GPS_FRAME_ID", "gps_link")
	t.Setenv("UNICORE_TARGET_BAUD", "460800")
	t.Setenv("UNICORE_PROFILE", "runtime")
	t.Setenv("UNICORE_AUTO_CONFIGURE", "true")

	check := buildGNSSCheck()
	blob, err := json.Marshal(check)
	require.NoError(t, err)

	assert.Equal(t, "unicore", check.Backend)
	assert.Equal(t, "UNICORE", check.Protocol)
	assert.NotContains(t, string(blob), "UNICORE_TARGET_BAUD")
	assert.NotContains(t, string(blob), "UNICORE_PROFILE")
	assert.NotContains(t, string(blob), "UNICORE_AUTO_CONFIGURE")
}
