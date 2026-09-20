package updater

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"
)

func (b DockerBackend) Verify(ctx context.Context, images map[string]string, d *Deployment) error {
	return waitForVerification(ctx, 3*time.Minute, 2*time.Second, func(ctx context.Context) []string {
		problems, warnings := b.verificationProblems(ctx, images, d)
		return append(problems, warnings...)
	})
}

// VerifyRecovery verifies that the previous deployment was restored and is safe
// to release from maintenance. Advisory module checks remain visible to the
// operator, but cannot strand an otherwise successful rollback.
func (b DockerBackend) VerifyRecovery(ctx context.Context, images map[string]string, d *Deployment) ([]string, error) {
	var warnings []string
	err := waitForVerification(ctx, 3*time.Minute, 2*time.Second, func(ctx context.Context) []string {
		problems, observedWarnings := b.verificationProblems(ctx, images, d)
		warnings = observedWarnings
		return problems
	})
	return warnings, err
}

func waitForVerification(ctx context.Context, budget, interval time.Duration, check func(context.Context) []string) error {
	ctx, cancel := context.WithTimeout(ctx, budget)
	defer cancel()
	stable := 0
	last := "health checks have not completed"
	for {
		problems := check(ctx)
		// A command cancelled at the deadline is not the original failure.
		// Retain the last completed check instead of replacing it with, for
		// example, a cancelled Docker configuration read.
		if err := ctx.Err(); err != nil {
			return fmt.Errorf("Update verification failed: %s: %w", last, err)
		}
		if len(problems) == 0 {
			stable++
			last = "waiting for three consecutive healthy checks"
		} else {
			stable = 0
			last = strings.Join(problems, "; ")
		}
		if stable >= 3 {
			return nil
		}
		select {
		case <-ctx.Done():
			return fmt.Errorf("Update verification failed: %s: %w", last, ctx.Err())
		case <-time.After(interval):
		}
	}
}

func (b DockerBackend) verificationProblems(ctx context.Context, images map[string]string, d *Deployment) ([]string, []string) {
	c, _, err := b.model(ctx)
	if err != nil {
		return []string{"Cannot read the installed container configuration"}, nil
	}
	managed, err := managedServices(c)
	if err != nil {
		return []string{"Cannot verify managed service definitions"}, nil
	}
	names := make([]string, 0, len(images))
	for name := range images {
		names = append(names, name)
	}
	sort.Strings(names)
	var problems []string
	var warnings []string
	for _, name := range names {
		sc, exists := c.Services[name]
		if !exists {
			problems = append(problems, name+": service is missing")
			continue
		}
		ci, err := b.inspect(ctx, sc.ContainerName)
		if err != nil {
			problems = append(problems, name+": container is unavailable")
			continue
		}
		if !ci.State.Running {
			problems, warnings = classifyRuntimeProblem(problems, warnings, name, "container is not running")
		}
		if ci.State.Health != nil && ci.State.Health.Status != "healthy" {
			problems, warnings = classifyRuntimeProblem(problems, warnings, name, "container health check has not passed")
		}
		ids, err := command(ctx, "docker", "image", "inspect", "--format", "{{.Id}}", images[name])
		if err != nil || strings.TrimSpace(string(ids)) != ci.Image {
			problems = append(problems, name+": running image does not match the reviewed update")
		}
	}
	ready, err := b.readiness(ctx)
	if err != nil {
		return append(problems, "GUI readiness endpoint is unavailable"), nil
	}
	_, err = os.Stat(filepath.Join(b.Config.StateDir, "maintenance"))
	if err != nil && !os.IsNotExist(err) {
		problems = append(problems, "Cannot read the update maintenance marker")
	}
	problems = append(problems, readinessProblems(ready, d, err == nil)...)
	return problems, append(warnings, advisoryModuleProblems(ready, names, managed)...)
}

func classifyRuntimeProblem(problems, warnings []string, name, reason string) ([]string, []string) {
	problem := name + ": " + reason
	if name == "mowgli" || name == "gui" {
		return append(problems, problem), warnings
	}
	return problems, append(warnings, problem)
}

func readinessProblems(ready Readiness, d *Deployment, maintenance bool) []string {
	var problems []string
	if !ready.Ready {
		reason := ready.Reason
		if reason == "" {
			reason = "fresh, stationary mower telemetry is required"
		}
		problems = append(problems, "Mower readiness: "+reason)
	}
	if maintenance && !ready.Maintenance {
		problems = append(problems, "GUI has not acknowledged update maintenance")
	}
	if d != nil && ready.FirmwareProtocol != d.FirmwareProtocol {
		problems = append(problems, fmt.Sprintf("Firmware protocol mismatch: running %d, update requires %d", ready.FirmwareProtocol, d.FirmwareProtocol))
	}
	return problems
}

// advisoryModuleProblems contains application-level checks which are required
// when accepting new images, but do not prove whether the previous deployment
// itself was restored. Add future optional-module observations here while keeping
// container, image, firmware and core mower readiness checks mandatory.
func advisoryModuleProblems(ready Readiness, names []string, managed map[string]managedService) []string {
	var problems []string
	for _, name := range names {
		switch managed[name].Health {
		case "gps":
			if !ready.GPSFresh && !ready.GPSReceiverFresh {
				reason := ready.GPSReason
				if reason == "" {
					reason = "No fresh GNSS data or verified receiver observations"
				}
				problems = append(problems, name+": "+reason)
			}
		case "lidar":
			if !ready.LidarFresh {
				problems = append(problems, name+": No fresh LiDAR scans")
			}
		}
	}
	return problems
}
