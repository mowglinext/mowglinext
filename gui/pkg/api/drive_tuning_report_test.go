package api

import (
	"math"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func float64Ptr(v float64) *float64 {
	return &v
}

func TestEvaluatePIDReport_StopBehaviorWarningIsVisibleButExcludedFromGainSelection(t *testing.T) {
	report := &driveTuningReport{
		GeneratedAt:  "2026-06-20T00:00:00Z",
		InternalTier: "medium",
		Trials: []driveTuningTrialReport{
			{
				Name:              "pid_step_1",
				Phase:             "pid",
				TargetSpeed:       0.30,
				MeasuredSpeedMean: 0.30,
				Overshoot:         0.01,
				SettlingTime:      float64Ptr(1.2),
				TrialQuality:      "ok",
			},
			{
				Name:              "pid_step_stop",
				Phase:             "pid",
				TargetSpeed:       0.0,
				MeasuredSpeedMean: 0.07,
				Overshoot:         0.11,
				TrialQuality:      "poor",
				Warnings: []string{
					"Stop behavior warning: residual motion detected after zero-speed command.",
				},
			},
		},
	}

	summary := evaluatePIDReport(report, "/tmp/pid.yaml")

	require.Equal(t, driveTuningStatusWarning, summary.Status)
	assert.Contains(t, summary.Message, "Stop behavior warning: residual motion detected after zero-speed command.")
	assert.Contains(t, summary.Message, "excluded from PID gain selection")
	assert.Contains(t, summary.Message, "braking/control should be reviewed")
}

func TestEvaluateFeedForwardReport_PostAnalysisOscillationUsesCalibrationWarningMessage(t *testing.T) {
	report := &driveTuningReport{
		GeneratedAt: "2026-06-20T00:00:00Z",
		Trials: []driveTuningTrialReport{
			{
				Name:                "ff_pass_1",
				Phase:               "feedforward",
				TargetSpeed:         0.30,
				MeasuredSpeedMean:   0.31,
				Overshoot:           0.02,
				TrialQuality:        "warning",
				OscillationDetected: true,
				OscillationSeverity: "moderate",
				RTKAccepted:         true,
				OdomDistanceM:       float64Ptr(3.03),
				RTKDistanceM:        float64Ptr(3.00),
				GroundSpeedMean:     float64Ptr(0.305),
				Warnings: []string{
					"Post-trial oscillation signature detected in the recorded speed response.",
				},
			},
		},
	}

	summary := evaluateFeedForwardReport(report, "/tmp/ff.yaml")

	require.Equal(t, driveTuningStatusWarning, summary.Status)
	assert.Contains(t, summary.Message, "post-analysis oscillation warning")
	assert.NotContains(t, summary.Message, "stalled")
	assert.NotContains(t, summary.Message, "Live oscillation")
}

func TestEvaluateFeedForwardReport_LiveOscillationUsesStrongMessage(t *testing.T) {
	report := &driveTuningReport{
		GeneratedAt: "2026-06-20T00:00:00Z",
		Trials: []driveTuningTrialReport{
			{
				Name:                    "ff_pass_1",
				Phase:                   "feedforward",
				TargetSpeed:             0.30,
				MeasuredSpeedMean:       0.31,
				Overshoot:               0.02,
				TrialQuality:            "warning",
				LiveOscillationDetected: true,
				LiveOscillationSeverity: "moderate",
				RTKAccepted:             true,
				OdomDistanceM:           float64Ptr(3.03),
				RTKDistanceM:            float64Ptr(3.00),
				GroundSpeedMean:         float64Ptr(0.305),
				Warnings: []string{
					"Live oscillation suspected during calibration; trial continued.",
				},
			},
		},
	}

	summary := evaluateFeedForwardReport(report, "/tmp/ff.yaml")

	require.Equal(t, driveTuningStatusWarning, summary.Status)
	assert.Contains(t, summary.Message, "Live oscillation was observed during feed-forward calibration.")
}

func TestEvaluateFeedForwardReport_MinorOscillationStaysValidated(t *testing.T) {
	report := &driveTuningReport{
		GeneratedAt: "2026-06-20T00:00:00Z",
		Trials: []driveTuningTrialReport{
			{
				Name:                "ff_pass_1",
				Phase:               "feedforward",
				TargetSpeed:         0.30,
				MeasuredSpeedMean:   0.301,
				Overshoot:           0.012,
				TrialQuality:        "ok",
				OscillationDetected: true,
				OscillationSeverity: "minor",
				RTKAccepted:         true,
				OdomDistanceM:       float64Ptr(3.01),
				RTKDistanceM:        float64Ptr(3.00),
				GroundSpeedMean:     float64Ptr(0.3005),
			},
		},
	}

	summary := evaluateFeedForwardReport(report, "/tmp/ff.yaml")

	require.Equal(t, driveTuningStatusValidated, summary.Status)
	assert.Contains(t, summary.Message, "Validated with")
}

func TestDriveTuningInternalTierFallsBackToMediumForInvalidMass(t *testing.T) {
	invalidMasses := []*float64{
		float64Ptr(0.0),
		float64Ptr(math.NaN()),
		float64Ptr(math.Inf(1)),
	}

	for _, mass := range invalidMasses {
		report := &driveTuningReport{RobotMassKg: mass}
		assert.Equal(t, "medium", driveTuningInternalTier(report))
	}
}

func TestEvaluatePIDReport_MinorLiveOscillationDoesNotForceWarning(t *testing.T) {
	report := &driveTuningReport{
		GeneratedAt:  "2026-06-20T00:00:00Z",
		InternalTier: "medium",
		Trials: []driveTuningTrialReport{
			{
				Name:                    "pid_step_1",
				Phase:                   "pid",
				TargetSpeed:             0.30,
				MeasuredSpeedMean:       0.30,
				Overshoot:               0.012,
				SettlingTime:            float64Ptr(1.2),
				LiveOscillationDetected: true,
				LiveOscillationSeverity: "minor",
				TrialQuality:            "ok",
			},
			{
				Name:              "pid_step_stop",
				Phase:             "pid",
				TargetSpeed:       0.0,
				MeasuredSpeedMean: 0.04,
				Overshoot:         0.06,
				TrialQuality:      "ok",
			},
		},
	}

	summary := evaluatePIDReport(report, "/tmp/pid.yaml")

	require.Equal(t, driveTuningStatusValidated, summary.Status)
	assert.Contains(t, summary.Message, "Validated PID step response")
	assert.Contains(t, summary.Message, "Zero-speed stop trials were excluded from gain selection.")
}

func TestPidOvershootThresholdsFollowMassTier(t *testing.T) {
	lightAcceptable, lightSevere := pidOvershootThresholds(&driveTuningReport{
		RobotMassKg: float64Ptr(10.0),
	})
	heavyAcceptable, heavySevere := pidOvershootThresholds(&driveTuningReport{
		RobotMassKg: float64Ptr(45.0),
	})

	assert.Equal(t, 0.10, lightAcceptable)
	assert.Equal(t, 0.16, lightSevere)
	assert.Equal(t, 0.06, heavyAcceptable)
	assert.Equal(t, 0.10, heavySevere)
}

func TestSanitizeDriveTuningReportRemovesNonFiniteValues(t *testing.T) {
	report := &driveTuningReport{
		RobotMassKg:  float64Ptr(math.NaN()),
		TestSpeedMps: float64Ptr(math.Inf(1)),
		StatusSnapshot: &driveTuningStatusReport{
			WheelTickFactor: float64Ptr(math.NaN()),
		},
		CurrentParams: map[string]float64{
			"wheel_pid_kp": math.NaN(),
			"wheel_pid_ki": 0.25,
		},
		Drivetrain: &driveTuningDrivetrain{
			WheelRadiusM:                 float64Ptr(math.Inf(-1)),
			ConfiguredTicksPerRevolution: float64Ptr(654.6666667),
		},
		Trials: []driveTuningTrialReport{
			{
				TargetSpeed:            math.NaN(),
				MeasuredSpeedMean:      math.Inf(1),
				Overshoot:              math.NaN(),
				SettlingTime:           float64Ptr(math.NaN()),
				GroundSpeedMean:        float64Ptr(math.Inf(-1)),
				OdomDistanceM:          float64Ptr(math.NaN()),
				RTKDistanceM:           float64Ptr(math.Inf(1)),
				LeftRightTickImbalance: float64Ptr(math.NaN()),
			},
		},
	}

	sanitizeDriveTuningReport(report)

	assert.Nil(t, report.RobotMassKg)
	assert.Nil(t, report.TestSpeedMps)
	require.NotNil(t, report.StatusSnapshot)
	assert.Nil(t, report.StatusSnapshot.WheelTickFactor)
	assert.Equal(t, map[string]float64{"wheel_pid_ki": 0.25}, report.CurrentParams)
	require.NotNil(t, report.Drivetrain)
	assert.Nil(t, report.Drivetrain.WheelRadiusM)
	assert.NotNil(t, report.Drivetrain.ConfiguredTicksPerRevolution)
	require.Len(t, report.Trials, 1)
	assert.Equal(t, 0.0, report.Trials[0].TargetSpeed)
	assert.Equal(t, 0.0, report.Trials[0].MeasuredSpeedMean)
	assert.Equal(t, 0.0, report.Trials[0].Overshoot)
	assert.Nil(t, report.Trials[0].SettlingTime)
	assert.Nil(t, report.Trials[0].GroundSpeedMean)
	assert.Nil(t, report.Trials[0].OdomDistanceM)
	assert.Nil(t, report.Trials[0].RTKDistanceM)
	assert.Nil(t, report.Trials[0].LeftRightTickImbalance)
}
