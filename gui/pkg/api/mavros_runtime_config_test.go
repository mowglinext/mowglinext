package api

import (
	"bytes"
	"encoding/json"
	"fmt"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

func TestMavrosRuntimeConfigMapsSharedFeedForwardAndBladeAuthorization(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	db := backendTestDB(t, "mavros")
	path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
	env, _ := db.Get("system.mower.runtimeEnvFile")
	dir := filepath.Join(filepath.Dir(string(env)), "config", "mavros")
	for _, mowingEnabled := range []bool{false, true} {
		yamlText := fmt.Sprintf("mowgli:\n  ros__parameters:\n    ticks_per_meter: 512.125\n    wheel_pid_pwm_per_mps: 321.5\n    mowing_enabled: %t\n", mowingEnabled)
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
		require.Equal(t, 321.5, p["manual_control_linear_scale"])
		require.Equal(t, mowingEnabled, p["mowing_enabled"])
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
