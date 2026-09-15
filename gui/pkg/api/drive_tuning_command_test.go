package api

import (
	"strings"
	"testing"
)

func TestBuildFeedForwardCommandPassesRuntimeRobotConfigPath(t *testing.T) {
	args, _ := buildFeedForwardCommand(driveFFCalibrationStartRequest{
		DistanceMeters: 3.0,
		TestSpeedMps:   0.3,
		OdomTimeoutS:   4.0,
		Passes:         3,
	})

	script := args[len(args)-1]
	if !strings.Contains(script, "'--hardware-config' '"+driveTuningRobotConfigPath+"'") {
		t.Fatalf("expected feed-forward command to pass --hardware-config %s, got %v", driveTuningRobotConfigPath, args)
	}
}

// The FF run must NOT force-seed wheel gains: a "stable baseline" of
// 0.2/0.1/0.01/15 is what stiction-locked the robot on 2026-09-15 (those
// gains then leaked into mowgli_robot.yaml through Apply). The run drives on
// whatever hardware_bridge currently has loaded.
func TestBuildFeedForwardCommandDoesNotForceSeedWheelPidGains(t *testing.T) {
	args, _ := buildFeedForwardCommand(driveFFCalibrationStartRequest{
		DistanceMeters: 3.0,
		TestSpeedMps:   0.3,
		OdomTimeoutS:   6.0,
		Passes:         3,
	})

	script := args[len(args)-1]
	for _, forbidden := range []string{
		"--custom-kp",
		"--custom-ki",
		"--custom-kd",
		"--custom-integral-limit",
	} {
		if strings.Contains(script, forbidden) {
			t.Fatalf("feed-forward command must not pass %s, got %v", forbidden, args)
		}
	}
}

// Apply after a feed-forward run persists ONLY the two quantities that run
// measures; the wheel gains the tuner echoes back in proposed_params must not
// reach mowgli_robot.yaml (they are whatever the run happened to drive with).
func TestPersistedParamsForModeFeedForwardKeepsOnlyOdomAndFeedforward(t *testing.T) {
	proposed := map[string]float64{
		"ticks_per_meter":          401.5,
		"wheel_pid_pwm_per_mps":    290.0,
		"wheel_pid_kp":             0.2,
		"wheel_pid_ki":             0.1,
		"wheel_pid_kd":             0.01,
		"wheel_pid_integral_limit": 15.0,
	}

	persisted := persistedParamsForMode(driveTuningModeFeedForward, proposed)

	if len(persisted) != 2 {
		t.Fatalf("expected exactly ticks_per_meter + wheel_pid_pwm_per_mps, got %v", persisted)
	}
	if persisted["ticks_per_meter"] != 401.5 || persisted["wheel_pid_pwm_per_mps"] != 290.0 {
		t.Fatalf("expected the measured values to pass through unchanged, got %v", persisted)
	}
	for _, gain := range []string{"wheel_pid_kp", "wheel_pid_ki", "wheel_pid_kd", "wheel_pid_integral_limit"} {
		if _, leaked := persisted[gain]; leaked {
			t.Fatalf("feed-forward apply must not persist %s, got %v", gain, persisted)
		}
	}
	if len(proposed) != 6 {
		t.Fatalf("input map must not be mutated, got %v", proposed)
	}
}

// The PID pass genuinely tunes the gains, so it persists everything it proposes.
func TestPersistedParamsForModePIDKeepsEverything(t *testing.T) {
	proposed := map[string]float64{
		"ticks_per_meter":          401.5,
		"wheel_pid_pwm_per_mps":    290.0,
		"wheel_pid_kp":             12.0,
		"wheel_pid_ki":             2100.0,
		"wheel_pid_kd":             0.0,
		"wheel_pid_integral_limit": 45.0,
	}

	persisted := persistedParamsForMode(driveTuningModePID, proposed)

	if len(persisted) != len(proposed) {
		t.Fatalf("expected every proposed param to persist in PID mode, got %v", persisted)
	}
	for key, value := range proposed {
		if persisted[key] != value {
			t.Fatalf("expected %s=%v to persist unchanged, got %v", key, value, persisted[key])
		}
	}
}

func TestBuildPIDCommandPassesRuntimeRobotConfigPath(t *testing.T) {
	args, _ := buildPIDCommand(drivePIDTuningStartRequest{
		MaxSpeedMps:      0.3,
		SegmentDurationS: 5.0,
		Passes:           3,
	})

	script := args[len(args)-1]
	if !strings.Contains(script, "'--hardware-config' '"+driveTuningRobotConfigPath+"'") {
		t.Fatalf("expected pid command to pass --hardware-config %s, got %v", driveTuningRobotConfigPath, args)
	}
}
