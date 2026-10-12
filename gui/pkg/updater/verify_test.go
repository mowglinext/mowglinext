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
	if problems := append(readinessProblems(ready, d, true, nil), healthIssueMessages(advisoryModuleProblems(ready, names, managed))...); len(problems) > 0 {
		t.Fatal(problems)
	}
	ready.GPSReceiverFresh = false
	ready.GPSReason = "GNSS receiver is responding but has no new observations"
	ready.LidarFresh = false
	problems := strings.Join(healthIssueMessages(advisoryModuleProblems(ready, names, managed)), "; ")
	for _, want := range []string{"gps: GNSS receiver is responding but has no new observations", "lidar: No fresh LiDAR scans"} {
		if !strings.Contains(problems, want) {
			t.Fatalf("missing %q in %s", want, problems)
		}
	}
	// Legacy GUI versions have no receiver-progress field: fresh fixes still
	// work, but absent fixes never silently fall back to process health.
	ready.GPSFresh = true
	ready.LidarFresh = true
	if problems := append(readinessProblems(ready, d, true, nil), healthIssueMessages(advisoryModuleProblems(ready, names, managed))...); len(problems) > 0 {
		t.Fatal(problems)
	}
	ready.Ready = false
	ready.Reason = "Mower must be stationary"
	ready.Maintenance = false
	ready.FirmwareProtocol = 5
	problems = strings.Join(readinessProblems(ready, d, true, nil), "; ")
	for _, want := range []string{"Mower must be stationary", "maintenance", "protocol mismatch"} {
		if !strings.Contains(problems, want) {
			t.Fatalf("safety gate %q lost: %s", want, problems)
		}
	}
}

func TestRecoveryClassifiesOptionalModuleRuntimeProblemsAsAdvisory(t *testing.T) {
	problems, warnings := classifyRuntimeProblem(nil, nil, "camera", "container.running", "container is not running")
	if len(problems) != 0 || strings.Join(healthIssueMessages(warnings), "; ") != "camera: container is not running" {
		t.Fatalf("optional module did not become advisory: problems=%v warnings=%v", problems, warnings)
	}
	problems, warnings = classifyRuntimeProblem(nil, nil, "mowgli", "container.running", "container is not running")
	if strings.Join(problems, "; ") != "mowgli: container is not running" || len(warnings) != 0 {
		t.Fatalf("core service did not remain mandatory: problems=%v warnings=%v", problems, warnings)
	}
}

func TestForcedHealthAcceptsOnlyReviewedServiceAndCheck(t *testing.T) {
	allowed := []HealthIssue{{Service: "lidar", Check: "application.lidar", Message: "old wording"}}
	observed := []HealthIssue{
		{Service: "lidar", Check: "application.lidar", Message: "lidar: wording from the new GUI"},
		{Service: "lidar", Check: "container.running", Message: "lidar: container is not running"},
		{Service: "gps", Check: "application.gps", Message: "gps: no observations"},
	}
	got := strings.Join(unacceptedHealthIssueMessages(observed, allowed), "; ")
	if strings.Contains(got, "wording from the new GUI") {
		t.Fatalf("presentation text changed the accepted check identity: %s", got)
	}
	for _, want := range []string{"lidar: container is not running", "gps: no observations"} {
		if !strings.Contains(got, want) {
			t.Fatalf("new failure %q was accepted: %s", want, got)
		}
	}
}

func TestForcedHealthReportsOnlyAcceptedFailuresWhichRemain(t *testing.T) {
	allowed := []HealthIssue{{Service: "lidar", Check: "application.lidar"}}
	observed := []HealthIssue{
		{Service: "lidar", Check: "application.lidar", Message: "lidar: No fresh LiDAR scans"},
		{Service: "gps", Check: "application.gps", Message: "gps: no observations"},
	}
	remaining := acceptedHealthIssues(observed, allowed)
	if len(remaining) != 1 || healthIssueKey(remaining[0]) != healthIssueKey(allowed[0]) {
		t.Fatalf("remaining accepted failures are wrong: %+v", remaining)
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
