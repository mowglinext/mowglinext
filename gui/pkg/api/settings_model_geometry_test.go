package api

import (
	"encoding/json"
	"fmt"
	"os"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

func TestModelGeometryAssetMatchesPresetsAndROS(t *testing.T) {
	chdirToGuiRoot(t)
	raw, err := os.ReadFile("asserts/model_geometry_defaults.json")
	require.NoError(t, err)
	ros, err := os.ReadFile("../ros2/src/mowgli_bringup/config/model_geometry_defaults.json")
	require.NoError(t, err)
	assert.Equal(t, string(ros), string(raw))
	var asset struct {
		Models map[string]map[string]float64 `json:"models"`
	}
	require.NoError(t, json.Unmarshal(raw, &asset))
	src, err := os.ReadFile("web/src/constants/mowerModels.ts")
	require.NoError(t, err)
	presets, err := parseMowerModelPresets(string(src))
	require.NoError(t, err)
	for _, preset := range presets {
		for key, value := range asset.Models[preset.name] {
			require.Contains(t, preset.defaults, key)
			assert.Equal(t, preset.defaults[key], value, preset.name+"."+key)
		}
	}
}

func TestPresetGeometryUpgradeReadSaveAndReset(t *testing.T) {
	for _, model := range []string{"YardForce500", "YardForce500B", "YardForceSA650", "CUSTOM", "unknown"} {
		t.Run(model, func(t *testing.T) {
			db, path := newBackendTestDB(t, fmt.Sprintf("mowgli:\n  ros__parameters:\n    mower_model: %s\n    wheel_track: 0.371\n", model), "")
			before, err := os.ReadFile(path)
			require.NoError(t, err)
			defaults := getJSON(t, db, "/api/settings/yaml/defaults")
			got := getJSON(t, db, "/api/settings/yaml")
			assert.Equal(t, defaults["chassis_z_offset"], got["chassis_z_offset"])
			assert.EqualValues(t, .371, got["wheel_track"])
			after, err := os.ReadFile(path)
			require.NoError(t, err)
			assert.Equal(t, before, after, "reading upgrade defaults must not rewrite config")
			for _, offset := range []float64{0, -.073} {
				postYAML(t, db, map[string]any{"chassis_z_offset": offset})
				postYAML(t, db, map[string]any{"robot_name": "unchanged-geometry"})
				assert.EqualValues(t, offset, getJSON(t, db, "/api/settings/yaml")["chassis_z_offset"])
			}
			postYAML(t, db, map[string]any{"chassis_z_offset": defaults["chassis_z_offset"]})
			after, err = os.ReadFile(path)
			require.NoError(t, err)
			assert.NotContains(t, string(after), "chassis_z_offset:")
			assert.Equal(t, defaults["chassis_z_offset"], getJSON(t, db, "/api/settings/yaml")["chassis_z_offset"])
			// Unsaved model choice drives reset defaults without writing YAML.
			draft := getJSON(t, db, "/api/settings/yaml/defaults?mower_model=CUSTOM")
			assert.EqualValues(t, 0, draft["chassis_z_offset"])
		})
	}
}

func TestPresetGeometryUsesModelFromSavePayload(t *testing.T) {
	db, path := newBackendTestDB(t, "mowgli:\n  ros__parameters:\n    mower_model: CUSTOM\n", "")
	postYAML(t, db, map[string]any{"mower_model": "YardForce500B", "chassis_z_offset": 0.0})
	raw, err := os.ReadFile(path)
	require.NoError(t, err)
	var doc map[string]any
	require.NoError(t, yaml.Unmarshal(raw, &doc))
	assert.EqualValues(t, 0, flattenROS2YAML(doc)["chassis_z_offset"], "zero is an override of the newly selected preset")
	assert.EqualValues(t, 0, getJSON(t, db, "/api/settings/yaml")["chassis_z_offset"])
	postYAML(t, db, map[string]any{"chassis_z_offset": nil})
	assert.Equal(t, getJSON(t, db, "/api/settings/yaml/defaults")["chassis_z_offset"], getJSON(t, db, "/api/settings/yaml")["chassis_z_offset"])
}
