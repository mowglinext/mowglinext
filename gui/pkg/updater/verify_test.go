package updater

import (
	"context"
	"errors"
	"strings"
	"testing"
	"time"
)

func TestVerificationDistinguishesNoFixReceiverFromMissingSensors(t *testing.T) {
	managed := map[string]managedService{"gps": {Health: "gps"}, "lidar": {Health: "lidar"}}
	names := []string{"gps", "lidar"}
	ready := Readiness{Ready: true, Maintenance: true, FirmwareProtocol: 6, GPSReceiverFresh: true, LidarFresh: true}
	d := &Deployment{FirmwareProtocol: 6}
	if problems := readinessProblems(ready, d, true, names, managed); len(problems) > 0 {
		t.Fatal(problems)
	}
	ready.GPSReceiverFresh = false
	ready.GPSReason = "GNSS receiver is responding but has no new observations"
	ready.LidarFresh = false
	problems := strings.Join(readinessProblems(ready, d, true, names, managed), "; ")
	for _, want := range []string{"gps: GNSS receiver is responding but has no new observations", "lidar: No fresh LiDAR scans"} {
		if !strings.Contains(problems, want) {
			t.Fatalf("missing %q in %s", want, problems)
		}
	}
	// Legacy GUI versions have no receiver-progress field: fresh fixes still
	// work, but absent fixes never silently fall back to process health.
	ready.GPSFresh = true
	ready.LidarFresh = true
	if problems := readinessProblems(ready, d, true, names, managed); len(problems) > 0 {
		t.Fatal(problems)
	}
	ready.Ready = false
	ready.Reason = "Mower must be stationary"
	ready.Maintenance = false
	ready.FirmwareProtocol = 5
	problems = strings.Join(readinessProblems(ready, d, true, names, managed), "; ")
	for _, want := range []string{"Mower must be stationary", "maintenance", "protocol mismatch"} {
		if !strings.Contains(problems, want) {
			t.Fatalf("safety gate %q lost: %s", want, problems)
		}
	}
}

func TestVerificationTimeoutNamesFailingChecks(t *testing.T) {
	err := waitForVerification(context.Background(), 15*time.Millisecond, time.Millisecond, func(context.Context) []string {
		return []string{"gps: container health check has not passed", "lidar: No fresh LiDAR scans"}
	})
	if !errors.Is(err, context.DeadlineExceeded) || !strings.Contains(err.Error(), "gps: container health check") || !strings.Contains(err.Error(), "lidar: No fresh LiDAR scans") {
		t.Fatalf("lost verification causes: %v", err)
	}
}

func TestVerificationRequiresConsecutiveHealthyChecks(t *testing.T) {
	n := 0
	err := waitForVerification(context.Background(), time.Second, time.Millisecond, func(context.Context) []string {
		n++
		if n == 3 {
			return []string{"gps: receiver disconnected"}
		}
		return nil
	})
	if err != nil || n != 6 {
		t.Fatalf("verification did not reset stability: samples=%d error=%v", n, err)
	}
}

func TestVerificationKeepsSensorFailureWhenFinalCommandTimesOut(t *testing.T) {
	n := 0
	err := waitForVerification(context.Background(), 20*time.Millisecond, time.Millisecond, func(ctx context.Context) []string {
		n++
		if n == 1 {
			return []string{"gps: receiver transport is unhealthy"}
		}
		<-ctx.Done()
		return []string{"Cannot read the installed container configuration"}
	})
	if !errors.Is(err, context.DeadlineExceeded) || !strings.Contains(err.Error(), "gps: receiver transport") {
		t.Fatalf("cancelled final command hid the sensor failure: %v", err)
	}
}
