package updater

import (
	"context"
	"strings"
	"testing"
)

func TestMaintenanceEntryDoesNotReleaseIncompatibleFirmware(t *testing.T) {
	ready := Readiness{MaintenanceReady: true, Maintenance: true, FirmwareProtocol: 7, Reason: "Firmware communication is incompatible"}
	if !maintenanceReady(ready, 7) {
		t.Fatal("safe protocol-first transition blocked")
	}
	for _, protocol := range []int{0, 6, 8} {
		if maintenanceReady(ready, protocol) {
			t.Fatalf("incompatible entry accepted for protocol %d", protocol)
		}
	}
	if len(readinessProblems(ready, &Deployment{FirmwareProtocol: 7}, true)) == 0 {
		t.Fatal("maintenance-only readiness released incompatible firmware")
	}
	ready.MaintenanceReady = false
	if maintenanceReady(ready, 7) {
		t.Fatal("legacy GUI or unsafe telemetry accepted")
	}
	ready.Ready = true
	if !maintenanceReady(ready, 7) {
		t.Fatal("compatible legacy GUI blocked")
	}
	if maintenanceReady(ready, 6) {
		t.Fatal("firmware changed after plan was accepted")
	}
}

type transitionBackend struct {
	*fakeBackend
	protocol int
}

func (b *transitionBackend) PrepareUpdate(_ context.Context, p Plan) error {
	b.protocol = p.Target.FirmwareProtocol
	return b.event("prepare-update")
}
func TestManagerUsesReviewedProtocolForMaintenanceEntry(t *testing.T) {
	m, b, _ := setup(t, "")
	transition := &transitionBackend{fakeBackend: b}
	m.backend = transition
	p, err := m.MakePlan(context.Background(), m.Snapshot().Releases[0].ID, false)
	if err != nil {
		t.Fatal(err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	s := settled(t, m)
	if s.Job.Phase != "succeeded" || transition.protocol != p.Target.FirmwareProtocol {
		t.Fatalf("transition failed: %+v", s.Job)
	}
	events := strings.Join(b.events, ",")
	if !strings.Contains(events, "prepare-update,backup,apply-new,verify-new") {
		t.Fatal(events)
	}
}

func TestWorkerIdentityIgnoresReleaseLabelOnly(t *testing.T) {
	for _, tc := range []struct {
		name, build, version string
		candidate            Binary
		same                 bool
	}{
		{"unrelated release", "aaa", "old", Binary{BuildID: "aaa", Version: "new"}, true},
		{"worker changed", "aaa", "old", Binary{BuildID: "bbb", Version: "new"}, false},
		{"changed bytes same label", "aaa", "old", Binary{BuildID: "bbb", Version: "old"}, false},
		{"legacy same", "", "old", Binary{Version: "old"}, true},
		{"legacy unknown", "", "old", Binary{BuildID: "aaa", Version: "new"}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			if sameWorkerBuild(tc.candidate, tc.build, tc.version) != tc.same {
				t.Fatal("wrong worker identity")
			}
		})
	}
}
