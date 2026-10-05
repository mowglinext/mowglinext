package api

import (
	"bytes"
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

// TestBackendDefaultsAssetMatchesOverlays is the anti-drift guard for the
// generated asset: it re-derives it from the ROS2 overlays on every CI run.
func TestBackendDefaultsAssetMatchesOverlays(t *testing.T) {
	chdirToGuiRoot(t)

	expected, err := buildBackendDefaultsFile(ros2BackendOverlaysDir)
	require.NoError(t, err)
	require.NotEmpty(t, expected.Backends, "no ROS2 backend overlay found")

	want, err := marshalBackendDefaultsFile(expected)
	require.NoError(t, err)

	got, err := os.ReadFile(backendDefaultsAssetPath)
	require.NoError(t, err, "the generated asset must be checked in — the GUI image ships asserts/ and nothing else")
	assert.Equal(t, string(want), string(got),
		"asserts/backend_defaults.json is stale; regenerate with: cd gui && go run ./cmd/gen-backend-defaults")
}

// Every overlay key must be a setting the schema knows, or the GUI could
// neither show nor reset it.
func TestBackendDefaultsOnlyOverrideSchemaSettings(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	schema, err := getSchema(types.NewMockDBProvider())
	require.NoError(t, err)
	defaults := map[string]any{}
	extractDefaults(schema, defaults)

	file, err := buildBackendDefaultsFile(ros2BackendOverlaysDir)
	require.NoError(t, err)
	for backend, overrides := range file.Backends {
		assert.Contains(t, supportedHardwareBackends, backend, "overlay for an unknown backend")
		for key := range overrides {
			assert.Contains(t, defaults, key, "%s overlay sets %s, which has no schema default", backend, key)
		}
	}
}

func TestActiveHardwareBackend(t *testing.T) {
	t.Setenv("HARDWARE_BACKEND", "")
	assert.Equal(t, "mowgli", activeHardwareBackend(map[string]string{}))
	assert.Equal(t, "openmower", activeHardwareBackend(map[string]string{"HARDWARE_BACKEND": " OpenMower "}))
	assert.Equal(t, "mowgli", activeHardwareBackend(map[string]string{"HARDWARE_BACKEND": "../../x"}))

	t.Setenv("HARDWARE_BACKEND", "mavros")
	assert.Equal(t, "mavros", activeHardwareBackend(map[string]string{}))
}

func newBackendTestDB(t *testing.T, yaml, env string) (types.IDBProvider, string) {
	t.Helper()
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	t.Setenv("HARDWARE_BACKEND", "")

	yamlFile := createTempYAMLFile(t, yaml)
	envFile := createTempConfigFile(t, env)
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))
	return db, yamlFile
}

func getJSON(t *testing.T, db types.IDBProvider, path string) map[string]any {
	t.Helper()
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", path, nil)
	setupSettingsRouter(db).ServeHTTP(w, req)
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())

	out := map[string]any{}
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &out))
	return out
}

func postYAML(t *testing.T, db types.IDBProvider, payload map[string]any) {
	t.Helper()
	body, _ := json.Marshal(payload)
	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	setupSettingsRouter(db).ServeHTTP(w, req)
	require.Equal(t, http.StatusOK, w.Code, w.Body.String())
}

func TestHardwareBackendRoute(t *testing.T) {
	db, _ := newBackendTestDB(t, "mowgli:\n  ros__parameters: {}\n", "HARDWARE_BACKEND=openmower\n")
	got := getJSON(t, db, "/api/settings/hardware-backend")

	assert.Equal(t, "openmower", got["backend"])
	assert.ElementsMatch(t, []any{"mowgli", "mavros", "openmower"}, got["supported"])

	overrides, ok := got["default_overrides"].(map[string]any)
	require.True(t, ok)
	assert.EqualValues(t, 1600, overrides["ticks_per_meter"])

	assert.Equal(t, "available", got["runtime_routing"])
	routes, ok := got["parameter_routes"].(map[string]any)
	require.True(t, ok)
	assert.Empty(t, routes)
}

func TestDefaultsFollowTheHardwareBackend(t *testing.T) {
	db, _ := newBackendTestDB(t, "mowgli:\n  ros__parameters: {}\n", "HARDWARE_BACKEND=openmower\n")
	om := getJSON(t, db, "/api/settings/yaml/defaults")
	assert.EqualValues(t, 1600, om["ticks_per_meter"])
	assert.EqualValues(t, 100, om["both_wheels_lift_emergency_ms"])
	assert.EqualValues(t, 2500, om["one_wheel_lift_emergency_ms"])
	assert.EqualValues(t, 29.0, om["max_charge_voltage"])
}

func TestDefaultsAreTheTemplatesForTheMowgliBackend(t *testing.T) {
	db, _ := newBackendTestDB(t, "mowgli:\n  ros__parameters: {}\n", "HARDWARE_BACKEND=mowgli\n")
	mowgli := getJSON(t, db, "/api/settings/yaml/defaults")
	assert.EqualValues(t, 399, mowgli["ticks_per_meter"])
	assert.EqualValues(t, 1000, mowgli["both_wheels_lift_emergency_ms"])
}

// On an OpenMower robot, "reset to default" writes OpenMower's value, which
// must be pruned (the robot then falls back to that same value); the template
// value is a real override there and must be kept.
func TestSavePrunesAgainstTheBackendDefaults(t *testing.T) {
	db, yamlFile := newBackendTestDB(t,
		"mowgli:\n  ros__parameters:\n    both_wheels_lift_emergency_ms: 300\n",
		"HARDWARE_BACKEND=openmower\n")

	postYAML(t, db, map[string]any{
		"both_wheels_lift_emergency_ms": 100,  // = OpenMower default -> pruned
		"one_wheel_lift_emergency_ms":   2000, // = template default, NOT OpenMower's -> kept
	})

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.NotContains(t, string(content), "both_wheels_lift_emergency_ms")
	assert.Contains(t, string(content), "one_wheel_lift_emergency_ms: 2000")
}

func TestOpenMowerWiringShowsTheInstallersChoiceUntilSet(t *testing.T) {
	db, _ := newBackendTestDB(t,
		"mowgli:\n  ros__parameters:\n    openmower_xesc_left_port: /dev/ttyUSB0\n",
		"HARDWARE_BACKEND=openmower\nOPENMOWER_LL_PORT=/dev/ttyAMA1\nOPENMOWER_XESC_TYPE=xesc_2040\nOPENMOWER_XESC_LEFT_PORT=/dev/ttyAMA9\n")

	got := getJSON(t, db, "/api/settings/yaml")
	assert.Equal(t, "/dev/ttyAMA1", got["openmower_ll_port"])
	assert.Equal(t, "xesc_2040", got["openmower_xesc_type"])
	assert.Equal(t, "/dev/ttyUSB0", got["openmower_xesc_left_port"], "an explicit value wins over .env")
}

func TestOpenMowerWiringIsNotShownForAnotherBackend(t *testing.T) {
	db, _ := newBackendTestDB(t, "mowgli:\n  ros__parameters: {}\n",
		"HARDWARE_BACKEND=mowgli\nOPENMOWER_LL_PORT=/dev/ttyAMA1\n")

	got := getJSON(t, db, "/api/settings/yaml")
	assert.NotContains(t, got, "openmower_ll_port")
}

// An explicit wiring choice equal to the template value must survive a save:
// with no schema default it is never pruned, so .env cannot silently win.
func TestOpenMowerWiringIsNeverPruned(t *testing.T) {
	db, yamlFile := newBackendTestDB(t, "mowgli:\n  ros__parameters: {}\n",
		"HARDWARE_BACKEND=openmower\nOPENMOWER_LL_PORT=/dev/ttyAMA1\n")

	postYAML(t, db, map[string]any{"openmower_ll_port": "/dev/ttyAMA0"})
	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "openmower_ll_port: /dev/ttyAMA0")
}

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
	assert.Equal(t, "pending_image", response.RuntimeRouting)
	assert.ElementsMatch(t, []string{"mowgli", "mavros", "openmower"}, response.Supported)

	assert.Equal(t, HardwareParameterRoute{
		Parameter: "mavros/esc_wheel_odometry.ticks_per_meter",
		Runtime:   "pending_image",
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

func TestOpenMowerHasNoLiveParameterRoutes(t *testing.T) {
	routes, ok := hardwareParameterRoutes["openmower"]
	require.True(t, ok)
	assert.Empty(t, routes)
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

func TestOpenMowerRejectsMowgliDriveToolMiddleware(t *testing.T) {
	gin.SetMode(gin.TestMode)
	router := gin.New()
	router.POST("/drive", requireMowgliHardwareBackend(backendTestDB(t, "openmower")), func(c *gin.Context) {
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
