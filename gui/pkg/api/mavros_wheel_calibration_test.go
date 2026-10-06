package api

import (
	"bytes"
	"encoding/json"
	"errors"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

func calibrationRequest(t *testing.T, r *gin.Engine, action string, body any) *httptest.ResponseRecorder {
	t.Helper()
	raw, err := json.Marshal(body)
	require.NoError(t, err)
	method := "POST"
	if action == "" {
		method = "GET"
	}
	req := httptest.NewRequest(method, "/api/tools/drive/mavros-calibration"+action, bytes.NewReader(raw))
	req.Header.Set("Content-Type", "application/json")
	w := httptest.NewRecorder()
	r.ServeHTTP(w, req)
	return w
}
func calibrationDiagnostic(epoch string) []byte {
	fields := map[string]string{"wheel_tick_source": "ardupilot_legacy", "wheel_tick_transport_scale": "1000", "ardupilot_legacy/epoch": epoch, "ardupilot_legacy/left_segment": "1", "ardupilot_legacy/right_segment": "1", "ardupilot_legacy/left_valid": "true", "ardupilot_legacy/right_valid": "true"}
	values := []map[string]string{}
	for key, value := range fields {
		values = append(values, map[string]string{"key": key, "value": value})
	}
	raw, _ := json.Marshal(map[string]any{"status": []any{map[string]any{"name": "mavros_esc_wheel_odometry/source", "values": values}}})
	return raw
}
func calibrationQuality() []byte {
	return []byte(`{"fix_valid":true,"rtk_mode":3,"corrections_active":true,"horizontal_accuracy_m":0.01,"capability_flags":8,"value_flags":8}`)
}
func calibrationObservation(t *testing.T, ros *types.MockRosProvider, stamp int64, metres float64, count uint32, direction uint8) {
	t.Helper()
	timestamp := map[string]any{"sec": stamp / 1e9, "nanosec": stamp % 1e9}
	ticks, _ := json.Marshal(map[string]any{"stamp": timestamp, "wheel_tick_factor": 990000, "valid_wheels": 12, "wheel_ticks_rl": count, "wheel_ticks_rr": count, "wheel_direction_rl": direction, "wheel_direction_rr": direction})
	// gps provider maps NavSatFix latitude/longitude into AbsolutePose x/y.
	gps, _ := json.Marshal(map[string]any{"header": map[string]any{"stamp": timestamp}, "pose": map[string]any{"pose": map[string]any{"position": map[string]any{"x": 48 + metres/111319.49079327358, "y": 2}}}})
	ros.Dispatch("ticks", ticks)
	ros.Dispatch("gps", gps)
}
func calibrationFixture(t *testing.T) (*gin.Engine, *types.MockRosProvider, *types.MockDBProvider, string) {
	t.Helper()
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	db := backendTestDB(t, "mavros")
	dir := t.TempDir()
	path := filepath.Join(dir, "mowgli_robot.yaml")
	require.NoError(t, os.WriteFile(path, []byte("mowgli:\n  ros__parameters:\n    ticks_per_meter: 990.0\n    wheel_pid_pwm_per_mps: 283.5\n    wheel_pid_kp: 10.0\n    wheel_pid_ki: 2000.0\n    wheel_pid_kd: 0.0\n    wheel_pid_integral_limit: 45.0\n    datum_lat: 48.123456789\n"), 0600))
	require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
	ros := types.NewMockRosProvider()
	r := gin.New()
	MavrosWheelCalibrationRoutes(r.Group("/api"), db, ros)
	require.Equal(t, 200, calibrationRequest(t, r, "/start", map[string]any{}).Code)
	ros.Dispatch("gnssStatus", calibrationQuality())
	ros.Dispatch("diagnostics", calibrationDiagnostic("1"))
	return r, ros, db, path
}
func TestMavrosCalibrationFitsRawTicksIndependentOfPriorCalibrationAndPersistsOnlyScale(t *testing.T) {
	r, ros, _, path := calibrationFixture(t)
	start := time.Now().UnixNano()
	for i := 0; i <= 4; i++ {
		calibrationObservation(t, ros, start+int64(i)*100000000, float64(i), 100000+uint32(i)*300000, 1)
	}
	finish := calibrationRequest(t, r, "/finish", nil)
	require.Equal(t, 200, finish.Code)
	var result mavrosCalibrationResult
	require.NoError(t, json.Unmarshal(finish.Body.Bytes(), &result))
	require.Equal(t, "ready", result.State)
	require.InDelta(t, 4, result.Distance, 1e-8)
	require.InDelta(t, 300, result.TicksPerMeter, 1e-5)
	require.Empty(t, ros.SetParams)
	require.Empty(t, ros.Publishes)
	require.Empty(t, ros.ServiceCalls)
	apply := calibrationRequest(t, r, "/apply", map[string]bool{"confirm": true})
	require.Equal(t, 200, apply.Code, apply.Body.String())
	require.Len(t, ros.SetParams, 1)
	require.Len(t, ros.SetParams[0], 1)
	require.Equal(t, "mavros/esc_wheel_odometry.ticks_per_meter", ros.SetParams[0][0].Name)
	raw, err := os.ReadFile(path)
	require.NoError(t, err)
	var yamlDoc map[string]any
	require.NoError(t, yaml.Unmarshal(raw, &yamlDoc))
	flat := flattenROS2YAML(yamlDoc)
	require.InDelta(t, 300, flat["ticks_per_meter"], 1e-5)
	require.Equal(t, 283.5, flat["wheel_pid_pwm_per_mps"])
	require.Equal(t, 10.0, flat["wheel_pid_kp"])
	require.Equal(t, 2000.0, flat["wheel_pid_ki"])
	require.Equal(t, 0.0, flat["wheel_pid_kd"])
	require.Equal(t, 45.0, flat["wheel_pid_integral_limit"])
	require.Len(t, flat, 7)
	require.Equal(t, 48.123456789, flat["datum_lat"])
}
func TestMavrosCalibrationRejectsResetReverseInvalidAndNonRTK(t *testing.T) {
	for _, kind := range []string{"epoch", "reverse", "reset", "invalid", "rtk", "gap"} {
		t.Run(kind, func(t *testing.T) {
			r, ros, _, _ := calibrationFixture(t)
			start := time.Now().UnixNano()
			calibrationObservation(t, ros, start, 0, 100000, 1)
			calibrationObservation(t, ros, start+100000000, 1, 400000, 1)
			switch kind {
			case "epoch":
				ros.Dispatch("diagnostics", calibrationDiagnostic("2"))
			case "reverse":
				calibrationObservation(t, ros, start+200000000, 2, 700000, 0)
			case "reset":
				calibrationObservation(t, ros, start+200000000, 2, 1, 1)
			case "invalid":
				stamp := start + 200000000
				raw, _ := json.Marshal(map[string]any{"stamp": map[string]any{"sec": stamp / 1e9, "nanosec": stamp % 1e9}, "valid_wheels": 4})
				ros.Dispatch("ticks", raw)
			case "rtk":
				ros.Dispatch("gnssStatus", []byte(`{"fix_valid":true,"rtk_mode":2}`))
			case "gap":
				calibrationObservation(t, ros, start+10000000000, 2, 700000, 1)
			}
			finish := calibrationRequest(t, r, "/finish", nil)
			var result mavrosCalibrationResult
			require.NoError(t, json.Unmarshal(finish.Body.Bytes(), &result))
			require.Equal(t, "failed", result.State)
			require.Equal(t, 409, calibrationRequest(t, r, "/apply", map[string]bool{"confirm": true}).Code)
			require.Empty(t, ros.SetParams)
		})
	}
}
func TestMavrosCalibrationBackwardPassUsesMagnitude(t *testing.T) {
	r, ros, _, _ := calibrationFixture(t)
	start := time.Now().UnixNano()
	for i := 0; i <= 4; i++ {
		calibrationObservation(t, ros, start+int64(i)*100000000, -float64(i), 100000+uint32(i)*300000, 0)
	}
	finish := calibrationRequest(t, r, "/finish", nil)
	var result mavrosCalibrationResult
	require.NoError(t, json.Unmarshal(finish.Body.Bytes(), &result))
	require.Equal(t, "ready", result.State)
	require.InDelta(t, 300, result.TicksPerMeter, 1e-5)
}
func TestMavrosCalibrationFailedApplyDoesNotPersistOrClaimApplied(t *testing.T) {
	r, ros, _, path := calibrationFixture(t)
	start := time.Now().UnixNano()
	for i := 0; i <= 4; i++ {
		calibrationObservation(t, ros, start+int64(i)*100000000, float64(i), 100000+uint32(i)*300000, 1)
	}
	calibrationRequest(t, r, "/finish", nil)
	before, _ := os.ReadFile(path)
	ros.ParamErr = errors.New("sidecar unavailable")
	require.Equal(t, 503, calibrationRequest(t, r, "/apply", map[string]bool{"confirm": true}).Code)
	after, _ := os.ReadFile(path)
	require.Equal(t, string(before), string(after))
}
func TestMavrosCalibrationDuplicateFixesNeverBecomeMeasurement(t *testing.T) {
	r, ros, _, _ := calibrationFixture(t)
	start := time.Now().UnixNano()
	for i := 0; i < 5; i++ {
		calibrationObservation(t, ros, start, float64(i), uint32(i)*300000, 1)
	}
	finish := calibrationRequest(t, r, "/finish", nil)
	var result mavrosCalibrationResult
	require.NoError(t, json.Unmarshal(finish.Body.Bytes(), &result))
	require.Equal(t, "failed", result.State)
	require.Equal(t, 1, result.Samples)
}
func TestMavrosCalibrationRejectsCurvesButAllowsCentimetreRTKNoise(t *testing.T) {
	const degreesPerMetre = 1 / 111319.49079327358
	points := []calibrationPoint{{lat: 48, lon: 2}, {lat: 48 + degreesPerMetre, lon: 2 + degreesPerMetre*0.01}, {lat: 48 + 2*degreesPerMetre, lon: 2}}
	require.True(t, straightRTKPass(points))
	points[1].lon = 2 + degreesPerMetre
	require.False(t, straightRTKPass(points))
}
