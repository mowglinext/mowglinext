package api

import (
	"context"
	"math"
	"os"
	"path/filepath"
	"strings"
	"testing"

	dockertypes "github.com/docker/docker/api/types"
	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

type odometryReportDocker struct{ mockDockerProvider }

func (d *odometryReportDocker) ContainerExec(context.Context, string, types.ContainerExecSpec) (types.ContainerExecResult, error) {
	// Include historical fields to prove they cannot leak through persistence.
	return types.ContainerExecResult{Stdout: "mode: ff\nproposed_params:\n  ticks_per_meter: 343.596123456789\n  wheel_pid_pwm_per_mps: 999.0\n  wheel_pid_kp: 99.0\n"}, nil
}

func TestMavrosOdometryJobPersistsOnlyFloatTicksAndReportOnlyLeavesYamlUntouched(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)
	for _, apply := range []bool{false, true} {
		t.Run(map[bool]string{false: "report-only", true: "apply"}[apply], func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "mowgli_robot.yaml")
			original := "mowgli:\n  ros__parameters:\n    ticks_per_meter: 300.125\n    wheel_pid_pwm_per_mps: 321.5\n    wheel_pid_kp: 10.25\n"
			require.NoError(t, os.WriteFile(path, []byte(original), 0600))
			db := backendTestDB(t, "mavros")
			require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
			envPath, _ := db.Get("system.mower.runtimeEnvFile")
			runtimeDir := filepath.Join(filepath.Dir(string(envPath)), "config", "mavros")
			require.NoError(t, os.MkdirAll(runtimeDir, 0755))
			bridgeFile := filepath.Join(runtimeDir, "hardware_bridge.yaml")
			bridgeOriginal := "hardware_bridge:\n  ros__parameters:\n    manual_control_linear_scale: 1000.0\n"
			require.NoError(t, os.WriteFile(bridgeFile, []byte(bridgeOriginal), 0600))
			require.NoError(t, os.WriteFile(filepath.Join(runtimeDir, "esc_wheel_odometry.yaml"), []byte("/**/esc_wheel_odometry:\n  ros__parameters:\n    ticks_per_meter: 300.125\n    track_width_m: 0.456789\n    source: auto\n"), 0600))
			docker := &odometryReportDocker{mockDockerProvider{
				containers:    []dockertypes.Container{{ID: "ros2", Names: []string{"/mowgli-ros2"}}},
				inspectResult: types.ContainerDetails{ID: "ros2", Running: true},
			}}
			manager := &driveTuningManager{dbProvider: db, dockerProvider: docker}
			job := &driveTuningJob{mode: driveTuningModeFeedForward, backend: "mavros", apply: apply, reportPath: "/tmp/report.yaml"}
			manager.runJob(job, []string{"unused"})
			require.NotEqual(t, string(driveTuningRunFailed), job.snapshot().State)
			raw, err := os.ReadFile(path)
			require.NoError(t, err)
			bridgeAfter, err := os.ReadFile(bridgeFile)
			require.NoError(t, err)
			require.Equal(t, bridgeOriginal, string(bridgeAfter), "calibration must never rewrite bridge actuation settings")
			if !apply {
				require.Equal(t, original, string(raw))
				return
			}
			params := decodeParams(t, string(raw))
			require.Len(t, params, 3, "MAVROS apply must not populate unrelated drive defaults")
			require.IsType(t, float64(0), params["ticks_per_meter"])
			require.Equal(t, 343.596123456789, params["ticks_per_meter"])
			require.Equal(t, 321.5, params["wheel_pid_pwm_per_mps"])
			require.Equal(t, 10.25, params["wheel_pid_kp"])
			env, err := db.Get("system.mower.runtimeEnvFile")
			require.NoError(t, err)
			runtimePath := filepath.Join(filepath.Dir(string(env)), "config/mavros/esc_wheel_odometry.yaml")
			runtime, err := os.ReadFile(runtimePath)
			require.NoError(t, err)
			var runtimeDoc map[string]any
			require.NoError(t, yaml.Unmarshal(runtime, &runtimeDoc))
			runtimeParams := runtimeDoc["/**/esc_wheel_odometry"].(map[string]any)["ros__parameters"].(map[string]any)
			require.Equal(t, 343.596123456789, runtimeParams["ticks_per_meter"])
			require.Equal(t, 0.456789, runtimeParams["track_width_m"])
			require.Equal(t, "auto", runtimeParams["source"])
		})
	}
}

func TestMavrosOdometryFiltersLegacyRollbackParameters(t *testing.T) {
	restored := persistedParamsForMode(driveTuningModePID, map[string]float64{
		"ticks_per_meter": 343.596, "wheel_pid_pwm_per_mps": 800.0, "wheel_pid_kp": 7.0,
	}, "mavros")
	require.Equal(t, map[string]float64{"ticks_per_meter": 343.596}, restored)
}

func TestMavrosOdometryReportPreservesPerPassPrecisionAndSeparateWheels(t *testing.T) {
	var report driveTuningReport
	require.NoError(t, yaml.Unmarshal([]byte(`mode: ff
recommended:
  ticks_per_meter: 343.596123456789
trials:
  - phase: odometry
    ticks_per_meter: 343.581123456789
    left_ticks_seen: 1030788
    right_ticks_seen: 1030800
`), &report))
	sanitizeDriveTuningReport(&report)
	require.Equal(t, 343.596123456789, report.Recommended["ticks_per_meter"])
	require.Equal(t, 343.581123456789, *report.Trials[0].TicksPerMeter)
	require.Equal(t, int64(1030788), report.Trials[0].LeftTicksSeen)
	require.Equal(t, int64(1030800), report.Trials[0].RightTicksSeen)
}

func TestEvaluateOdometryReportIgnoresSpeedTrackingAndExplainsMeasurementQuality(t *testing.T) {
	good := driveTuningTrialReport{
		Phase: "odometry", TargetSpeed: 0.3, MeasuredSpeedMean: 0.12,
		GroundSpeedMean: float64Ptr(0.12), RTKAccepted: true,
		OdomDistanceM: float64Ptr(3.015), RTKDistanceM: float64Ptr(3.0),
		LeftTicksSeen: 1030788, RightTicksSeen: 1030800,
	}
	cases := []struct {
		name   string
		mutate func(*driveTuningTrialReport)
		status driveTuningValidationStatus
		reason string
	}{
		{"excellent odometry despite 60 percent speed error", func(*driveTuningTrialReport) {}, driveTuningStatusValidated, "Validated odometry"},
		{"no accepted reference", func(p *driveTuningTrialReport) { p.RTKAccepted = false }, driveTuningStatusWarning, "No valid RTK/GNSS"},
		{"missing reference distance", func(p *driveTuningTrialReport) { p.RTKDistanceM = nil }, driveTuningStatusWarning, "No valid RTK/GNSS"},
		{"invalid reference distance", func(p *driveTuningTrialReport) { p.RTKDistanceM = float64Ptr(math.NaN()) }, driveTuningStatusWarning, "No valid RTK/GNSS"},
		{"stall", func(p *driveTuningTrialReport) { p.StallDetected = true }, driveTuningStatusWarning, "stalled"},
		{"no motion", func(p *driveTuningTrialReport) { p.OdomDistanceM = float64Ptr(0) }, driveTuningStatusWarning, "no wheel motion"},
		{"excessive distance error", func(p *driveTuningTrialReport) { p.OdomDistanceM = float64Ptr(3.3) }, driveTuningStatusWarning, "10.0% exceeds"},
		{"unstable motion", func(p *driveTuningTrialReport) { p.LiveOscillationDetected = true }, driveTuningStatusWarning, "stability warning"},
		{"measurement warning", func(p *driveTuningTrialReport) { p.Warnings = []string{"wheel mismatch"} }, driveTuningStatusWarning, "measurement warnings"},
		{"poor trial", func(p *driveTuningTrialReport) { p.TrialQuality = "poor" }, driveTuningStatusWarning, "measurement warnings"},
	}
	for _, test := range cases {
		t.Run(test.name, func(t *testing.T) {
			trial := good
			test.mutate(&trial)
			report := &driveTuningReport{GeneratedAt: "now", Trials: []driveTuningTrialReport{trial}}
			summary := evaluateOdometryReport(report, "/report.yaml")
			require.Equal(t, test.status, summary.Status)
			require.Contains(t, summary.Message, test.reason)
			require.Equal(t, "now", summary.GeneratedAt)
			require.Equal(t, "/report.yaml", summary.ReportPath)
			for _, forbidden := range []string{"feed-forward", "pwm", "speed error"} {
				require.NotContains(t, strings.ToLower(summary.Message), forbidden)
			}
		})
	}
	require.Equal(t, driveTuningStatusNotValidated, evaluateOdometryReport(&driveTuningReport{}, "").Status)
	historical := good
	historical.Phase = "feedforward"
	require.Equal(t, driveTuningStatusNotValidated, evaluateOdometryReport(&driveTuningReport{Trials: []driveTuningTrialReport{historical}}, "").Status)
	missing := good
	missing.RTKAccepted = false
	partial := evaluateOdometryReport(&driveTuningReport{Trials: []driveTuningTrialReport{good, missing}}, "")
	require.Equal(t, driveTuningStatusWarning, partial.Status)
	require.Contains(t, partial.Message, "Some odometry passes")
	failed := evaluateOdometryReport(&driveTuningReport{Trials: []driveTuningTrialReport{good}, FailureMessage: "aborted"}, "")
	require.Equal(t, driveTuningStatusWarning, failed.Status)
	require.Contains(t, failed.Message, "did not complete")
}

type odometryValidationDocker struct {
	mockDockerProvider
	report string
}

func (d *odometryValidationDocker) ContainerExec(_ context.Context, _ string, spec types.ContainerExecSpec) (types.ContainerExecResult, error) {
	if strings.Contains(spec.Cmd[len(spec.Cmd)-1], "find ") {
		return types.ContainerExecResult{Stdout: "/report.yaml\n"}, nil
	}
	return types.ContainerExecResult{Stdout: d.report}, nil
}

func TestMavrosOdometryStatusSelectsDedicatedEvaluator(t *testing.T) {
	for _, backend := range []string{"mavros", "mowgli"} {
		t.Run(backend, func(t *testing.T) {
			phase := "odometry"
			if backend == "mowgli" {
				phase = "feedforward"
			}
			docker := &odometryValidationDocker{
				mockDockerProvider: mockDockerProvider{
					containers:    []dockertypes.Container{{ID: "ros2", Names: []string{"/mowgli-ros2"}}},
					inspectResult: types.ContainerDetails{ID: "ros2", Running: true},
				},
				report: "mode: ff\ntrials:\n  - phase: " + phase + "\n    target_speed: 0.3\n    measured_speed_mean: 0.12\n    ground_speed_mean: 0.12\n    odom_distance_m: 3.015\n    rtk_distance_m: 3.0\n    rtk_accepted: true\n",
			}
			manager := &driveTuningManager{dbProvider: backendTestDB(t, backend), dockerProvider: docker}
			status, err := manager.buildStatusResponse(context.Background())
			require.NoError(t, err)
			if backend == "mavros" {
				require.Equal(t, driveTuningStatusValidated, status.FeedForward.Status)
				require.Contains(t, status.FeedForward.Message, "Validated odometry")
				require.NotContains(t, status.FeedForward.Message, "speed error")
			} else {
				require.Equal(t, driveTuningStatusWarning, status.FeedForward.Status)
				require.Contains(t, status.FeedForward.Message, "speed error")
			}
		})
	}
}
