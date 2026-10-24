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
	"github.com/stretchr/testify/require"
)

func TestBackendIntegrityResolution(t *testing.T) {
	for _, tc := range []struct {
		env, process, want string
		invalid            bool
	}{
		{"", "", "mowgli", false}, {"HARDWARE_BACKEND=mowgli\n", "mavros", "mowgli", false},
		{"HARDWARE_BACKEND=mavros\n", "mowgli", "mavros", false},
		{"HARDWARE_BACKEND=openmower\n", "mowgli", "openmower", false},
		{"HARDWARE_BACKEND=unsupported\n", "mowgli", "", true},
		{"", "unsupported", "", true}, {"HARDWARE_BACKEND='unterminated", "mowgli", "", true},
	} {
		t.Run(tc.env+tc.process, func(t *testing.T) {
			t.Setenv("HARDWARE_BACKEND", tc.process)
			path := filepath.Join(t.TempDir(), ".env")
			require.NoError(t, os.WriteFile(path, []byte(tc.env), 0600))
			db := types.NewMockDBProvider()
			require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(path)))
			backend, err := resolveHardwareBackendForDB(db)
			require.Equal(t, tc.want, backend)
			if tc.invalid {
				require.Error(t, err)
			} else {
				require.NoError(t, err)
			}
		})
	}
}

func TestBackendIntegrityUnavailableMetadataAndMiddleware(t *testing.T) {
	for _, kind := range []string{"missing_path_setting", "unreadable_file", "unsupported"} {
		t.Run(kind, func(t *testing.T) {
			db := types.NewMockDBProvider()
			if kind == "unreadable_file" {
				require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(t.TempDir())))
			}
			if kind == "unsupported" {
				path := filepath.Join(t.TempDir(), ".env")
				require.NoError(t, os.WriteFile(path, []byte("HARDWARE_BACKEND=bogus\n"), 0600))
				require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(path)))
			}
			gin.SetMode(gin.TestMode)
			r := gin.New()
			group := r.Group("/api")
			GetSettingsHardwareBackend(group, db)
			group.POST("/drive", requireMowgliHardwareBackend(db), func(c *gin.Context) { c.Status(202) })
			w := httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest("GET", "/api/settings/hardware-backend", nil))
			require.Equal(t, 503, w.Code)
			require.NotContains(t, w.Body.String(), `"backend":"mowgli"`)
			w = httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest("POST", "/api/drive", nil))
			require.Equal(t, 409, w.Code)
		})
	}
}

func TestBackendIntegrityParameterGuard(t *testing.T) {
	cases := []struct {
		backend, name string
		value         any
		status        int
	}{
		{"mowgli", "hardware_bridge.wheel_pid_kp", 0.0, 200}, {"mowgli", "hardware_bridge.ticks_per_meter", 319.305, 200},
		{"mowgli", "hardware_bridge.ticks_per_meter", nil, 409}, {"mowgli", "hardware_bridge.ticks_per_meter", "", 409},
		{"mowgli", "hardware_bridge.ticks_per_meter", false, 409},
		{"mavros", "hardware_bridge.wheel_pid_kp", 0.0, 409}, {"mavros", "hardware_bridge.ticks_per_meter", 1.0, 409},
		{"mavros", "mavros/esc_wheel_odometry.ticks_per_meter", 401.5, 200},
		{"mavros", "mavros/esc_wheel_odometry.track_width_m", 0.3, 200},
		{"mavros", "hardware_bridge.manual_control_linear_scale", 282.135, 200},
		{"mavros", "hardware_bridge.mowing_enabled", true, 409},
		{"openmower", "hardware_bridge.ticks_per_meter", 1.0, 409},
		{"bogus", "hardware_bridge.wheel_pid_kp", 1.0, 409},
		{"bogus", "fusion_graph_node.use_lidar_map_anchor", true, 200},
		{"bogus", "fusion_graph_node.test_string", "generic", 200},
		{"bogus", "fusion_graph_node.test_array", []float64{0, 1}, 200},
	}
	for _, tc := range cases {
		t.Run(tc.backend+tc.name, func(t *testing.T) {
			db := backendTestDB(t, tc.backend)
			ros := types.NewMockRosProvider()
			gin.SetMode(gin.TestMode)
			r := gin.New()
			ParamsRoutes(r.Group("/api"), ros, db)
			payload, err := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{{Name: tc.name, Value: tc.value}}})
			require.NoError(t, err)
			req := httptest.NewRequest(http.MethodPost, "/api/params", bytes.NewReader(payload))
			req.Header.Set("Content-Type", "application/json")
			w := httptest.NewRecorder()
			r.ServeHTTP(w, req)
			require.Equal(t, tc.status, w.Code, w.Body.String())
			if tc.status == 409 {
				require.Empty(t, ros.SetParams)
			} else {
				require.Len(t, ros.SetParams, 1)
			}
		})
	}
}

func TestBackendIntegrityRechecksInstallerSelection(t *testing.T) {
	db := backendTestDB(t, "mowgli")
	path, err := db.Get("system.mower.runtimeEnvFile")
	require.NoError(t, err)
	require.NoError(t, os.WriteFile(string(path), []byte("HARDWARE_BACKEND=mavros\n"), 0600))
	ros := types.NewMockRosProvider()
	r := gin.New()
	ParamsRoutes(r.Group("/api"), ros, db)
	req := httptest.NewRequest("POST", "/api/params", bytes.NewBufferString(`{"parameters":[{"name":"hardware_bridge.wheel_pid_kp","value":10}]}`))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	require.Equal(t, 409, w.Code)
	require.Empty(t, ros.SetParams)
}

func TestBackendIntegrityYAMLPersistedBeforeSidecarFailure(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	dir := t.TempDir()
	env := filepath.Join(dir, ".env")
	config := filepath.Join(dir, "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(env, []byte("HARDWARE_BACKEND=mavros\n"), 0600))
	// A file at the sidecar directory forces an actual post-YAML write failure.
	require.NoError(t, os.WriteFile(filepath.Join(dir, "config"), []byte("blocked"), 0600))
	db := types.NewMockDBProvider()
	require.NoError(t, db.Set("system.mower.runtimeEnvFile", []byte(env)))
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(config)))
	r := setupSettingsRouter(db)
	w := httptest.NewRecorder()
	r.ServeHTTP(w, httptest.NewRequest("POST", "/api/settings/yaml", bytes.NewBufferString(`{"ticks_per_meter":401.5}`)))
	require.Equal(t, 500, w.Code)
	var body map[string]any
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &body))
	require.Equal(t, true, body["persisted"])
	saved, err := os.ReadFile(config)
	require.NoError(t, err)
	require.Contains(t, string(saved), "401.5")
}

func TestBackendIntegrityClosedHardwareNamespaces(t *testing.T) {
	for _, backend := range []string{"mowgli", "mavros", "openmower", "unsupported", "unresolved"} {
		for _, name := range []string{
			"hardware_bridge.unknown_parameter", "hardware_bridge.serial_port",
			"hardware_bridge.mowing_enabled", "hardware_bridge.wheel_loop_enabled",
			"hardware_bridge.blade_servo_channel", "hardware_bridge.status_publish_rate_hz",
			"mavros/esc_wheel_odometry.unknown_parameter",
			"/other/hardware_bridge.manual_control_linear_scale",
			"/other/esc_wheel_odometry.ticks_per_meter",
		} {
			t.Run(backend+"/"+name, func(t *testing.T) {
				db := types.NewMockDBProvider()
				if backend != "unresolved" {
					db = backendTestDB(t, backend)
				}
				ros := types.NewMockRosProvider()
				r := gin.New()
				ParamsRoutes(r.Group("/api"), ros, db)
				payload, err := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{
					{Name: "fusion_graph_node.node_period_s", Value: 0.1}, {Name: name, Value: 1.0},
				}})
				require.NoError(t, err)
				w := httptest.NewRecorder()
				r.ServeHTTP(w, httptest.NewRequest("POST", "/api/params", bytes.NewReader(payload)))
				require.Equal(t, 409, w.Code, w.Body.String())
				require.Empty(t, ros.SetParams, "validation must precede dispatch of the entire batch")
			})
		}
	}
}

func TestBackendIntegrityExplicitDirectHardwareOperations(t *testing.T) {
	cases := []struct {
		backend, name, kind string
		value               any
		status              int
	}{
		{"mowgli", "hardware_bridge.min_linear_vel", "float64", 0.0, 200},
		{"mowgli", "hardware_bridge.yaw_kp", "float64", 1.0, 200},
		{"mowgli", "hardware_bridge.yaw_ki", "double", 1.0, 200},
		{"mowgli", "/hardware_bridge.yaw_trim_limit_mps", "float64", 0.1, 200},
		{"mavros", "hardware_bridge.manual_control_yaw_scale", "float64", 1000.0, 200},
		{"mavros", "hardware_bridge.right_esc_slot", "int64", 0.0, 200},
		{"mavros", "hardware_bridge.left_esc_slot", "int64", 1.0, 200},
		{"mavros", "hardware_bridge.blade_esc_slot", "int64", 2.0, 200},
		{"mowgli", "hardware_bridge.manual_control_yaw_scale", "float64", 1000.0, 409},
		{"mowgli", "hardware_bridge.right_esc_slot", "int64", 0.0, 409},
		{"mavros", "hardware_bridge.yaw_kp", "float64", 1.0, 409},
		{"mavros", "hardware_bridge.min_linear_vel", "float64", 0.1, 409},
		{"openmower", "hardware_bridge.min_linear_vel", "float64", 0.1, 409},
		{"unsupported", "hardware_bridge.manual_control_yaw_scale", "float64", 1000.0, 409},
		{"mavros", "hardware_bridge.right_esc_slot", "int64", 0.5, 409},
		{"mavros", "hardware_bridge.right_esc_slot", "float64", 0.0, 409},
		{"mowgli", "hardware_bridge.yaw_kp", "float64", "1", 409},
		{"mowgli", "hardware_bridge.use_sim_time", "bool", false, 200},
		{"mavros", "hardware_bridge.use_sim_time", "bool", false, 200},
		{"openmower", "hardware_bridge.use_sim_time", "bool", false, 200},
		{"mavros", "mavros/esc_wheel_odometry.use_sim_time", "bool", false, 200},
		{"openmower", "hardware_bridge.use_sim_time", "bool", 0.0, 409},
		{"unsupported", "hardware_bridge.use_sim_time", "bool", false, 409},
	}
	for _, tc := range cases {
		t.Run(tc.backend+tc.name+tc.kind, func(t *testing.T) {
			ros := types.NewMockRosProvider()
			r := gin.New()
			ParamsRoutes(r.Group("/api"), ros, backendTestDB(t, tc.backend))
			body, err := json.Marshal(SetParamsRequest{Parameters: []types.RosParameter{{Name: tc.name, Value: tc.value, Type: tc.kind}}})
			require.NoError(t, err)
			w := httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest("POST", "/api/params", bytes.NewReader(body)))
			require.Equal(t, tc.status, w.Code, w.Body.String())
			if tc.status == 200 {
				require.Len(t, ros.SetParams, 1)
			} else {
				require.Empty(t, ros.SetParams)
			}
		})
	}
}

func TestBackendIntegrityGenericNamespacesRemainAvailable(t *testing.T) {
	for _, backend := range []string{"mowgli", "mavros", "openmower", "unsupported", "unresolved"} {
		t.Run(backend, func(t *testing.T) {
			db := types.NewMockDBProvider()
			if backend != "unresolved" {
				db = backendTestDB(t, backend)
			}
			ros := types.NewMockRosProvider()
			r := gin.New()
			ParamsRoutes(r.Group("/api"), ros, db)
			payload := `{"parameters":[{"name":"fusion_graph_node.use_lidar_map_anchor","value":false},{"name":"map_server_node.tool_width","value":0.18},{"name":"other_node.text","value":"generic"},{"name":"other_node.array","value":[0,1]}]}`
			w := httptest.NewRecorder()
			r.ServeHTTP(w, httptest.NewRequest("POST", "/api/params", bytes.NewBufferString(payload)))
			require.Equal(t, 200, w.Code, w.Body.String())
			require.Len(t, ros.SetParams, 1)
			require.Len(t, ros.SetParams[0], 4)
		})
	}
}
