package api

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func backendTestDB(t *testing.T, backend string) *types.MockDBProvider {
	t.Helper()
	envPath := filepath.Join(t.TempDir(), ".env")
	require.NoError(t, os.WriteFile(envPath, []byte("HARDWARE_BACKEND="+backend+"\n"), 0o600))
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(envPath)))
	return db
}

func TestHardwareBackendMavrosPublishesSourceContractRoutes(t *testing.T) {
	gin.SetMode(gin.TestMode)
	router := gin.New()
	GetSettingsHardwareBackend(router.Group("/api"), backendTestDB(t, "mavros"))

	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/api/settings/hardware-backend", nil))
	require.Equal(t, http.StatusOK, w.Code)

	var response HardwareBackendResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &response))
	assert.Equal(t, "mavros", response.Backend)
	assert.Equal(t, "available", response.RuntimeRouting)
	assert.Equal(t, HardwareParameterRoute{
		Parameter: "mavros/esc_wheel_odometry.ticks_per_meter",
		Runtime:   "available",
	}, response.ParameterRoutes["ticks_per_meter"])
	assert.Equal(t, "mavros/esc_wheel_odometry.track_width_m", response.ParameterRoutes["wheel_track"].Parameter)
	for _, key := range []string{
		"wheel_pid_kp", "wheel_pid_ki", "wheel_pid_kd",
		"wheel_pid_integral_limit", "wheel_pid_pwm_per_mps",
	} {
		assert.NotContains(t, response.ParameterRoutes, key)
	}
}

func TestHardwareBackendMowgliPreservesExistingRoutes(t *testing.T) {
	routes := hardwareParameterRoutes["mowgli"]
	assert.Equal(t, map[string]HardwareParameterRoute{
		"ticks_per_meter":          {Parameter: "hardware_bridge.ticks_per_meter", Runtime: "available"},
		"wheel_pid_kp":             {Parameter: "hardware_bridge.wheel_pid_kp", Runtime: "available"},
		"wheel_pid_ki":             {Parameter: "hardware_bridge.wheel_pid_ki", Runtime: "available"},
		"wheel_pid_kd":             {Parameter: "hardware_bridge.wheel_pid_kd", Runtime: "available"},
		"wheel_pid_integral_limit": {Parameter: "hardware_bridge.wheel_pid_integral_limit", Runtime: "available"},
		"wheel_pid_pwm_per_mps":    {Parameter: "hardware_bridge.wheel_pid_pwm_per_mps", Runtime: "available"},
	}, routes)
	assert.NotContains(t, routes, "wheel_track")
}

func TestMavrosRejectsMowgliDriveToolMiddleware(t *testing.T) {
	gin.SetMode(gin.TestMode)
	router := gin.New()
	router.POST("/drive", requireMowgliHardwareBackend(backendTestDB(t, "mavros")), func(c *gin.Context) {
		c.Status(http.StatusAccepted)
	})
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/drive", nil))
	assert.Equal(t, http.StatusConflict, w.Code)
}

func TestMowgliAllowsDriveToolMiddleware(t *testing.T) {
	gin.SetMode(gin.TestMode)
	router := gin.New()
	router.POST("/drive", requireMowgliHardwareBackend(backendTestDB(t, "mowgli")), func(c *gin.Context) {
		c.Status(http.StatusAccepted)
	})
	w := httptest.NewRecorder()
	router.ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/drive", nil))
	assert.Equal(t, http.StatusAccepted, w.Code)
}

func TestMavrosDriveCommandRoutesFailBeforeStartingAJob(t *testing.T) {
	gin.SetMode(gin.TestMode)
	router := gin.New()
	DriveTuningRoutes(router.Group("/api"), backendTestDB(t, "mavros"), nil)
	for _, path := range []string{
		"/api/tools/drive/ff-calibration/start",
		"/api/tools/drive/pid-tuning/start",
		"/api/tools/drive/tuning/rollback",
	} {
		w := httptest.NewRecorder()
		router.ServeHTTP(w, httptest.NewRequest(http.MethodPost, path, nil))
		assert.Equal(t, http.StatusConflict, w.Code, path)
	}
}
