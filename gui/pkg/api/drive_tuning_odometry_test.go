package api

import (
	"context"
	"encoding/json"
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

type odometryReportDocker struct {
	mockDockerProvider
	report string
}

func (d *odometryReportDocker) ContainerExec(context.Context, string, types.ContainerExecSpec) (types.ContainerExecResult, error) {
	// Include historical fields to prove they cannot leak through persistence.
	if d.report != "" {
		return types.ContainerExecResult{Stdout: d.report}, nil
	}
	report := &driveTuningReport{Mode: "ff", ProposedParams: map[string]float64{"ticks_per_meter": 343.596123456789, "wheel_pid_kp": 99, "wheel_pid_pwm_per_mps": 999}, Trials: []driveTuningTrialReport{goodOdometryTrial()}}
	raw, err := yaml.Marshal(report)
	return types.ContainerExecResult{Stdout: string(raw)}, err
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
			docker := &odometryReportDocker{mockDockerProvider: mockDockerProvider{
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

func goodOdometryTrial() driveTuningTrialReport {
	return driveTuningTrialReport{
		Phase: "odometry", TargetSpeed: .4, MeasuredSpeedMean: .051,
		RTKAccepted: true, RTKDistanceM: float64Ptr(3), OdomDistanceM: float64Ptr(.41),
		CalibrationWindow: &driveTuningCalibrationWindow{
			StartStampNS: 1000000000, EndStampNS: 9000000000,
			LeftMotorRevolutions: 3 * 343.596123456789, RightMotorRevolutions: 3 * 343.596123456789,
			RTKDistanceM: 3, TicksPerMeter: 343.596123456789, CounterUnit: "motor_revolution",
			Source: "ardupilot_legacy", GNSSBackend: "mavros_gps2", LeftEpoch: 2, RightEpoch: 2,
			LeftSegment: 1, RightSegment: 1,
		},
	}
}

func TestEvaluateOdometryReportIgnoresOldScaleAndRequiresAlignedEstimate(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(*driveTuningTrialReport)
		status driveTuningValidationStatus
		reason string
	}{
		{"old coefficient underestimates distance", func(*driveTuningTrialReport) {}, driveTuningStatusNotValidated, "Coefficient calculated"},
		{"old odometry absent", func(p *driveTuningTrialReport) { p.OdomDistanceM = nil }, driveTuningStatusNotValidated, "Coefficient calculated"},
		{"old odometry zero", func(p *driveTuningTrialReport) { p.OdomDistanceM = float64Ptr(0) }, driveTuningStatusNotValidated, "Coefficient calculated"},
		{"rejected window", func(p *driveTuningTrialReport) { p.RTKAccepted = false; p.Notes = []string{"reset"} }, driveTuningStatusNotValidated, "No valid aligned"},
		{"rejected with warnings", func(p *driveTuningTrialReport) { p.RTKAccepted = false; p.Warnings = []string{"source reset"} }, driveTuningStatusWarning, "Measurement warnings"},
		{"historical report", func(p *driveTuningTrialReport) { p.CalibrationWindow = nil }, driveTuningStatusNotValidated, "historical reports"},
		{"missing reference", func(p *driveTuningTrialReport) { p.RTKDistanceM = nil }, driveTuningStatusNotValidated, "No valid aligned"},
		{"invalid reference", func(p *driveTuningTrialReport) { p.RTKDistanceM = float64Ptr(math.NaN()) }, driveTuningStatusNotValidated, "No valid aligned"},
		{"invalid unit", func(p *driveTuningTrialReport) { p.CalibrationWindow.CounterUnit = "encoder_tick" }, driveTuningStatusNotValidated, "No valid aligned"},
		{"zero coefficient", func(p *driveTuningTrialReport) { p.CalibrationWindow.TicksPerMeter = 0 }, driveTuningStatusNotValidated, "No valid aligned"},
		{"nonfinite coefficient", func(p *driveTuningTrialReport) { p.CalibrationWindow.TicksPerMeter = math.Inf(1) }, driveTuningStatusNotValidated, "No valid aligned"},
		{"ratio mismatch", func(p *driveTuningTrialReport) { p.CalibrationWindow.TicksPerMeter = 48 }, driveTuningStatusNotValidated, "No valid aligned"},
		{"counter missing", func(p *driveTuningTrialReport) { p.CalibrationWindow.LeftMotorRevolutions = 0 }, driveTuningStatusNotValidated, "No valid aligned"},
		{"counter nonfinite", func(p *driveTuningTrialReport) { p.CalibrationWindow.RightMotorRevolutions = math.NaN() }, driveTuningStatusNotValidated, "No valid aligned"},
		{"negative distance", func(p *driveTuningTrialReport) { p.CalibrationWindow.RTKDistanceM = -3 }, driveTuningStatusNotValidated, "No valid aligned"},
		{"distance mismatch", func(p *driveTuningTrialReport) { p.RTKDistanceM = float64Ptr(4) }, driveTuningStatusNotValidated, "No valid aligned"},
		{"bounds reversed", func(p *driveTuningTrialReport) { p.CalibrationWindow.StartStampNS = p.CalibrationWindow.EndStampNS }, driveTuningStatusNotValidated, "No valid aligned"},
		{"bounds missing", func(p *driveTuningTrialReport) { p.CalibrationWindow.StartStampNS = 0 }, driveTuningStatusNotValidated, "No valid aligned"},
		{"epoch mismatch", func(p *driveTuningTrialReport) { p.CalibrationWindow.RightEpoch = 3 }, driveTuningStatusNotValidated, "No valid aligned"},
		{"source absent", func(p *driveTuningTrialReport) { p.CalibrationWindow.Source = "" }, driveTuningStatusNotValidated, "No valid aligned"},
		{"GNSS absent", func(p *driveTuningTrialReport) { p.CalibrationWindow.GNSSBackend = "" }, driveTuningStatusNotValidated, "No valid aligned"},
		{"per-pass mismatch", func(p *driveTuningTrialReport) { p.TicksPerMeter = float64Ptr(48) }, driveTuningStatusNotValidated, "No valid aligned"},
		{"stall", func(p *driveTuningTrialReport) { p.StallDetected = true }, driveTuningStatusWarning, "stalled"},
		{"unstable motion", func(p *driveTuningTrialReport) { p.LiveOscillationDetected = true }, driveTuningStatusWarning, "stability warning"},
		{"measurement warning", func(p *driveTuningTrialReport) { p.Warnings = []string{"wheel mismatch"} }, driveTuningStatusWarning, "Measurement warnings"},
		{"poor trial", func(p *driveTuningTrialReport) { p.TrialQuality = "poor" }, driveTuningStatusWarning, "Measurement warnings"},
	}
	for _, test := range cases {
		t.Run(test.name, func(t *testing.T) {
			trial := goodOdometryTrial()
			test.mutate(&trial)
			report := &driveTuningReport{GeneratedAt: "now", Trials: []driveTuningTrialReport{trial}}
			summary := evaluateOdometryReport(report, "/report.yaml")
			require.Equal(t, test.status, summary.Status)
			require.Contains(t, summary.Message, test.reason)
			require.Equal(t, "now", summary.GeneratedAt)
			require.Equal(t, "/report.yaml", summary.ReportPath)
			require.NotContains(t, summary.Message, "Validated odometry")
			for _, forbidden := range []string{"feed-forward", "pwm", "speed error", "distance error"} {
				require.NotContains(t, strings.ToLower(summary.Message), forbidden)
			}
		})
	}
	require.Equal(t, driveTuningStatusNotValidated, evaluateOdometryReport(&driveTuningReport{}, "").Status)
	for _, applied := range []bool{false, true} {
		good := goodOdometryTrial()
		report := &driveTuningReport{AppliedLive: applied, Trials: []driveTuningTrialReport{good, good}}
		summary := evaluateOdometryReport(report, "")
		require.Equal(t, driveTuningStatusNotValidated, summary.Status)
		require.Contains(t, summary.Message, "2 RTK window(s)")
		require.Contains(t, summary.Message, "Physical accuracy remains unvalidated")
		missing := goodOdometryTrial()
		missing.RTKAccepted = false
		report.Trials = append(report.Trials, missing)
		require.Contains(t, evaluateOdometryReport(report, "").Message, "Some odometry passes")
		report.FailureMessage = "aborted"
		require.Contains(t, evaluateOdometryReport(report, "").Message, "did not complete")
	}
}

func TestOdometryWindowDecodeSanitizeAndProposalArithmetic(t *testing.T) {
	report := &driveTuningReport{Mode: "ff", ProposedParams: map[string]float64{"ticks_per_meter": 343.596123456789}, Trials: []driveTuningTrialReport{goodOdometryTrial(), goodOdometryTrial()}}
	raw, err := yaml.Marshal(report)
	require.NoError(t, err)
	var decoded driveTuningReport
	require.NoError(t, yaml.Unmarshal(raw, &decoded))
	require.True(t, validOdometryProposal(&decoded))
	require.Equal(t, int64(9000000000), decoded.Trials[0].CalibrationWindow.EndStampNS)
	decoded.Trials[1].CalibrationWindow.TicksPerMeter *= 2
	decoded.Trials[1].CalibrationWindow.LeftMotorRevolutions *= 2
	decoded.Trials[1].CalibrationWindow.RightMotorRevolutions *= 2
	require.False(t, validOdometryProposal(&decoded))
	decoded.ProposedParams["ticks_per_meter"] *= 1.5
	require.True(t, validOdometryProposal(&decoded))
	decoded.Trials[1].CalibrationWindow.TicksPerMeter = math.NaN()
	sanitizeDriveTuningReport(&decoded)
	require.False(t, validOdometryProposal(&decoded))
	_, err = json.Marshal(decoded)
	require.NoError(t, err)
}

func TestMavrosRejectedReportCannotPersistCoefficient(t *testing.T) {
	chdirToGuiRoot(t)
	for _, fault := range []string{"rejected", "historical", "failure", "ratio", "proposal"} {
		t.Run(fault, func(t *testing.T) {
			trial := goodOdometryTrial()
			report := &driveTuningReport{Mode: "ff", ProposedParams: map[string]float64{"ticks_per_meter": 343.596123456789}, Trials: []driveTuningTrialReport{trial}}
			switch fault {
			case "rejected":
				report.Trials[0].RTKAccepted = false
			case "historical":
				report.Trials[0].CalibrationWindow = nil
			case "failure":
				report.FailureMessage = "reset"
			case "ratio":
				report.Trials[0].CalibrationWindow.LeftMotorRevolutions = 1
			case "proposal":
				report.ProposedParams["ticks_per_meter"] = 48
			}
			raw, err := yaml.Marshal(report)
			require.NoError(t, err)
			path := filepath.Join(t.TempDir(), "robot.yaml")
			original := "mowgli:\n  ros__parameters:\n    ticks_per_meter: 48.0\n"
			require.NoError(t, os.WriteFile(path, []byte(original), 0600))
			db := backendTestDB(t, "mavros")
			require.NoError(t, db.Set("system.mower.yamlConfigFile", []byte(path)))
			docker := &odometryReportDocker{mockDockerProvider: mockDockerProvider{containers: []dockertypes.Container{{ID: "ros2", Names: []string{"/mowgli-ros2"}}}, inspectResult: types.ContainerDetails{ID: "ros2", Running: true}}, report: string(raw)}
			manager := &driveTuningManager{dbProvider: db, dockerProvider: docker}
			job := &driveTuningJob{mode: driveTuningModeFeedForward, backend: "mavros", apply: true, reportPath: "/tmp/report.yaml"}
			manager.runJob(job, []string{"unused"})
			require.Equal(t, string(driveTuningRunWarning), job.snapshot().State)
			require.Contains(t, job.snapshot().Error, "not persisted")
			after, err := os.ReadFile(path)
			require.NoError(t, err)
			require.Equal(t, original, string(after))
		})
	}
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
				require.Equal(t, driveTuningStatusNotValidated, status.FeedForward.Status)
				require.Contains(t, status.FeedForward.Message, "historical reports")
				require.NotContains(t, status.FeedForward.Message, "speed error")
			} else {
				require.Equal(t, driveTuningStatusWarning, status.FeedForward.Status)
				require.Contains(t, status.FeedForward.Message, "speed error")
			}
		})
	}
}
