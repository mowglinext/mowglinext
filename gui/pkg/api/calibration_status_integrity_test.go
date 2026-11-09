package api

import (
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
)

func TestDockPoseValuesIntegrity(t *testing.T) {
	cases := []struct {
		name, yaml     string
		valid, present bool
		nullKey        string
	}{
		{"zeros", "dock_pose_x: 0\ndock_pose_y: 0\ndock_pose_yaw: 0\n", true, false, ""},
		{"mixed_zeros", "node:\n  ros__parameters:\n    dock_pose_x: 1.25\n    dock_pose_y: 0\n    dock_pose_yaw: 0\n", true, true, ""},
		{"existing", "dock_pose_x: 1.25\ndock_pose_y: -2.5\ndock_pose_yaw: 0.75\n", true, true, ""},
		{"missing_x", "dock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"empty", "{}\n", false, false, "dock_pose_x"},
		{"null", "dock_pose_x: null\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"string", "dock_pose_x: '1.2'\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"bool", "dock_pose_x: false\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"array", "dock_pose_x: [1]\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"nan", "dock_pose_x: .nan\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"inf_yaw", "dock_pose_x: 1\ndock_pose_y: 2\ndock_pose_yaw: .inf\n", false, true, "dock_pose_yaw_rad"},
		{"negative_inf", "dock_pose_x: -.inf\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"duplicate_nested", "first:\n  dock_pose_x: 1\nsecond:\n  dock_pose_x: 2\ndock_pose_y: 2\ndock_pose_yaw: 0\n", false, true, "dock_pose_x"},
		{"malformed_yaml", "[invalid", false, true, "dock_pose_x"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
			require.NoError(t, os.WriteFile(path, []byte(tc.yaml), 0600))
			db := types.NewMockDBProvider()
			require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
			status := readDockCalibrationStatus(db)
			require.Equal(t, tc.valid, status.PoseValuesValid)
			require.Equal(t, tc.present, status.Present)
			bytes, err := json.Marshal(status)
			require.NoError(t, err, "invalid values must not break JSON serialization")
			var raw map[string]any
			require.NoError(t, json.Unmarshal(bytes, &raw))
			for _, key := range []string{"dock_pose_x", "dock_pose_y", "dock_pose_yaw_rad"} {
				require.Contains(t, raw, key)
			}
			if tc.nullKey != "" {
				require.Nil(t, raw[tc.nullKey])
			}
			if tc.name == "zeros" {
				for _, key := range []string{"dock_pose_x", "dock_pose_y", "dock_pose_yaw_rad"} {
					require.Equal(t, 0.0, raw[key])
				}
			}
			if tc.name == "existing" {
				require.Equal(t, 1.25, raw["dock_pose_x"])
				require.Equal(t, -2.5, raw["dock_pose_y"])
				require.Equal(t, .75, raw["dock_pose_yaw_rad"])
			}
		})
	}
}

func TestDockPoseUnreadableIntegrity(t *testing.T) {
	for _, kind := range []string{"missing", "directory", "db_path_missing"} {
		t.Run(kind, func(t *testing.T) {
			db := types.NewMockDBProvider()
			if kind != "db_path_missing" {
				path := t.TempDir()
				if kind == "missing" {
					path = filepath.Join(path, "missing.yaml")
				}
				require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
			}
			result := readDockCalibrationStatus(db)
			require.False(t, result.PoseValuesValid)
			require.Nil(t, result.DockPoseX)
			require.Nil(t, result.DockPoseY)
			require.Nil(t, result.DockPoseYawRad)
			_, err := json.Marshal(result)
			require.NoError(t, err)
			require.Equal(t, kind == "directory", result.Present)
		})
	}
}
