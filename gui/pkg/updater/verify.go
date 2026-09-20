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
		return b.verificationProblems(ctx, images, d)
	})
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

func (b DockerBackend) verificationProblems(ctx context.Context, images map[string]string, d *Deployment) []string {
	c, _, err := b.model(ctx)
	if err != nil {
		return []string{"Cannot read the installed container configuration"}
	}
	managed, err := managedServices(c)
	if err != nil {
		return []string{"Cannot verify managed service definitions"}
	}
	names := make([]string, 0, len(images))
	for name := range images {
		names = append(names, name)
	}
	sort.Strings(names)
	var problems []string
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
			problems = append(problems, name+": container is not running")
			continue
		}
		if ci.State.Health != nil && ci.State.Health.Status != "healthy" {
			problems = append(problems, name+": container health check has not passed")
		}
		ids, err := command(ctx, "docker", "image", "inspect", "--format", "{{.Id}}", images[name])
		if err != nil || strings.TrimSpace(string(ids)) != ci.Image {
			problems = append(problems, name+": running image does not match the reviewed update")
		}
	}
	ready, err := b.readiness(ctx)
	if err != nil {
		return append(problems, "GUI readiness endpoint is unavailable")
	}
	_, err = os.Stat(filepath.Join(b.Config.StateDir, "maintenance"))
	if err != nil && !os.IsNotExist(err) {
		problems = append(problems, "Cannot read the update maintenance marker")
	}
	return append(problems, readinessProblems(ready, d, err == nil, names, managed)...)
}

func readinessProblems(ready Readiness, d *Deployment, maintenance bool, names []string, managed map[string]managedService) []string {
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
