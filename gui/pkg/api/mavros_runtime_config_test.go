package api

import (
	"bytes"
	"encoding/json"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

func TestMavrosRuntimeConfigSparseDefaultsOptInAndBladeDisabled(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	db := backendTestDB(t, "mavros")
	path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
	env, _ := db.Get("system.mower.runtimeEnvFile")
	dir := filepath.Join(filepath.Dir(string(env)), "config", "mavros")
	for _, optIn := range []bool{false, true} {
		yamlText := "mowgli:\n  ros__parameters:\n    ticks_per_meter: 512.125\n"
		if optIn {
			yamlText += "    mavros_manual_control_enabled: true\n    mavros_wheel_lift_safety_enabled: false\n"
		}
		require.NoError(t, os.WriteFile(path, []byte(yamlText), 0600))
		require.NoError(t, writeMavrosRuntimeConfig(db))
		raw, err := os.ReadFile(filepath.Join(dir, "esc_wheel_odometry.yaml"))
		require.NoError(t, err)
		var doc map[string]any
		require.NoError(t, yaml.Unmarshal(raw, &doc))
		p := doc["/**/esc_wheel_odometry"].(map[string]any)["ros__parameters"].(map[string]any)
		require.Equal(t, 512.125, p["ticks_per_meter"])
		require.Equal(t, 0.325, p["track_width_m"])
		raw, err = os.ReadFile(filepath.Join(dir, "hardware_bridge.yaml"))
		require.NoError(t, err)
		require.NoError(t, yaml.Unmarshal(raw, &doc))
		p = doc["hardware_bridge"].(map[string]any)["ros__parameters"].(map[string]any)
		require.Equal(t, optIn, p["manual_control_enabled"])
		require.Equal(t, !optIn, p["wheel_lift_safety_enabled"])
		require.Equal(t, false, p["blade_control_enabled"])
	}
}
func TestMavrosEnableRoutesKeepBooleanTypesAndRejectBladeEnable(t *testing.T) {
	for _, name := range []string{"hardware_bridge.manual_control_enabled", "/hardware_bridge.wheel_lift_safety_enabled"} {
		ros := types.NewMockRosProvider()
		db := backendTestDB(t, "mavros")
		routes := gin.New()
		ParamsRoutes(routes.Group("/api"), ros, db)
		for _, enabled := range []bool{false, true} {
			w := paramsRequest(routes, name, enabled)
			require.Equal(t, 200, w.Code, w.Body.String())
			last := ros.SetParams[len(ros.SetParams)-1]
			require.Equal(t, enabled, last[0].Value)
		}
		require.Equal(t, 400, paramsRequest(routes, name, 1).Code)
		require.Equal(t, 409, paramsRequest(routes, "hardware_bridge.blade_control_enabled", true).Code)
	}
}
func paramsRequest(r *gin.Engine, name string, value any) *httptest.ResponseRecorder {
	raw, _ := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{{Name: name, Value: value}}})
	req := httptest.NewRequest("POST", "/api/params", bytes.NewReader(raw))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}
