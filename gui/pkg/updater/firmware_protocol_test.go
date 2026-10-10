package updater

import (
	"context"
	"encoding/json"
	"errors"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestFirmwareProtocolChangeNeedsExplicitAllowance(t *testing.T) {
	target := Deployment{FirmwareProtocol: 7}

	// Arrange/Act: same protocol never records a change, allowed or not.
	if change, err := firmwareProtocolChange(7, target, PlanOptions{}); err != nil || change != nil {
		t.Fatalf("matching protocol planned a change: %v %v", change, err)
	}

	// A mismatch without allowance is a typed, explanatory refusal.
	_, err := firmwareProtocolChange(6, target, PlanOptions{})
	var mismatch FirmwareProtocolMismatch
	if !errors.As(err, &mismatch) || mismatch.Running != 6 || mismatch.Required != 7 {
		t.Fatalf("expected a firmware protocol mismatch, got %v", err)
	}
	for _, want := range []string{"different mainboard firmware protocol", "running 6", "requires 7", "flash"} {
		if !strings.Contains(err.Error(), want) {
			t.Fatalf("mismatch message lost %q: %s", want, err)
		}
	}

	// The allowance records the pair the gate and verification will accept.
	change, err := firmwareProtocolChange(6, target, PlanOptions{AllowFirmwareProtocolChange: true})
	if err != nil || change == nil || *change != (FirmwareProtocolChange{From: 6, To: 7}) {
		t.Fatalf("allowed change not recorded: %v %v", change, err)
	}

	// A board without a handshake cannot be forced past: nobody knows what it runs.
	if _, err = firmwareProtocolChange(0, target, PlanOptions{AllowFirmwareProtocolChange: true}); !errors.Is(err, errFirmwareProtocolUnavailable) {
		t.Fatalf("unknown running protocol was forced: %v", err)
	}
}

func TestExpectedFirmwareMismatchAcceptsOnlyTheAcknowledgedRefusal(t *testing.T) {
	change := &FirmwareProtocolChange{From: 6, To: 7}
	// dev's protocol-first contract: MaintenanceReady proves fresh, idle,
	// stationary, blade-off telemetry while the bridge refuses the board.
	incompatible := Readiness{MaintenanceReady: true, Reason: "Firmware communication is incompatible", FirmwareProtocol: 6}

	if !expectedFirmwareMismatch(incompatible, change) {
		t.Fatal("bridge refusing the old board after a forced update must be expected")
	}
	// After the operator reflashes, the new protocol is the other accepted value.
	flashed := Readiness{Ready: true, MaintenanceReady: true, FirmwareProtocol: 7}
	if !expectedFirmwareMismatch(flashed, change) {
		t.Fatal("reflashed board must satisfy the recorded change")
	}

	for name, ready := range map[string]Readiness{
		"no handshake":    {MaintenanceReady: true, FirmwareProtocol: 0},
		"third protocol":  {MaintenanceReady: true, FirmwareProtocol: 5},
		"moving mower":    {Reason: "Mower must be stationary", FirmwareProtocol: 6},
		"stale telemetry": {Reason: "Fresh firmware, behaviour and wheel telemetry required", FirmwareProtocol: 6},
		"legacy GUI":      {Reason: "Firmware communication is incompatible", FirmwareProtocol: 6},
	} {
		if expectedFirmwareMismatch(ready, change) {
			t.Fatalf("%s was accepted as the expected firmware refusal", name)
		}
	}
	if expectedFirmwareMismatch(incompatible, nil) {
		t.Fatal("no acknowledged change must accept nothing")
	}
}

func TestMaintenanceEntryHonoursAcknowledgedProtocolChange(t *testing.T) {
	change := &FirmwareProtocolChange{From: 6, To: 7}
	refusing := Readiness{MaintenanceReady: true, Maintenance: true, FirmwareProtocol: 6}
	// Rolling back a forced update before the reflash: board on 6, images on 7.
	if !maintenanceReady(refusing, 7, change) {
		t.Fatal("acknowledged refusal blocked maintenance entry")
	}
	// The same state without the acknowledgement keeps dev's protocol-first rule.
	if maintenanceReady(refusing, 7, nil) {
		t.Fatal("unacknowledged mismatch entered maintenance")
	}
	if !maintenanceReady(Readiness{MaintenanceReady: true, FirmwareProtocol: 7}, 7, nil) {
		t.Fatal("protocol-first transition blocked")
	}
	if maintenanceReady(Readiness{MaintenanceReady: true, FirmwareProtocol: 5}, 7, change) {
		t.Fatal("third protocol entered maintenance under a change")
	}
}

func TestReadinessProblemsToleratesAcknowledgedFirmwareRefusalOnly(t *testing.T) {
	change := &FirmwareProtocolChange{From: 6, To: 7}
	d := &Deployment{FirmwareProtocol: 7}
	ready := Readiness{Maintenance: true, MaintenanceReady: true, Reason: "Firmware communication is incompatible", FirmwareProtocol: 6}

	if problems := readinessProblems(ready, d, true, change); len(problems) > 0 {
		t.Fatalf("acknowledged firmware refusal failed verification: %v", problems)
	}
	// The same readiness without the acknowledgement keeps both original gates
	// (dev's rule: maintenance-only readiness cannot release the mower).
	problems := strings.Join(readinessProblems(ready, d, true, nil), "; ")
	for _, want := range []string{"Firmware communication is incompatible", "protocol mismatch: running 6, update requires 7"} {
		if !strings.Contains(problems, want) {
			t.Fatalf("gate %q lost without acknowledgement: %s", want, problems)
		}
	}
	// Maintenance acknowledgement and every other unready cause still block.
	ready.Maintenance = false
	if problems := strings.Join(readinessProblems(ready, d, true, change), "; "); !strings.Contains(problems, "maintenance") {
		t.Fatalf("maintenance gate lost under a firmware change: %s", problems)
	}
	ready.Maintenance, ready.MaintenanceReady, ready.Reason = true, false, "Mower must be stationary"
	if problems := strings.Join(readinessProblems(ready, d, true, change), "; "); !strings.Contains(problems, "Mower must be stationary") {
		t.Fatalf("motion gate lost under a firmware change: %s", problems)
	}
}

// A forced protocol change must be allowed at review, confirmed again at
// install, and then travel with the job: the maintenance gate and every
// verification (including the rollback that may follow before the board is
// reflashed) are told which refusal to expect.
func TestFirmwareProtocolChangeIsConfirmedTwiceAndFollowsTheJob(t *testing.T) {
	newer := fixture()
	newer.ID, newer.ReleaseTag, newer.FirmwareProtocol = "deployment-v7", "deployment-v7", 7
	b := &fakeBackend{fingerprint: "original"}
	m, err := Open(t.TempDir(), []string{"mowglinext/mowglinext"}, b, &fakeSource{releases: []Deployment{newer}})
	if err != nil {
		t.Fatal(err)
	}
	if err = m.Check(context.Background(), true); err != nil {
		t.Fatal(err)
	}

	// Plain review still refuses, with the explanatory typed error.
	_, err = m.MakePlan(context.Background(), newer.ID, false)
	var mismatch FirmwareProtocolMismatch
	if !errors.As(err, &mismatch) || mismatch != (FirmwareProtocolMismatch{Running: 6, Required: 7}) {
		t.Fatalf("unforced plan across a protocol change was not refused: %v", err)
	}

	p, err := m.MakeServicePlan(context.Background(), newer.ID, false, nil, PlanOptions{AllowFirmwareProtocolChange: true})
	if err != nil {
		t.Fatal(err)
	}
	if p.FirmwareProtocolChange == nil || *p.FirmwareProtocolChange != (FirmwareProtocolChange{From: 6, To: 7}) {
		t.Fatalf("allowed plan did not record the change: %+v", p.FirmwareProtocolChange)
	}
	// Reviewing is not installing: the install needs its own acknowledgement.
	if _, err = m.Start(p.ID); err == nil || !strings.Contains(err.Error(), "firmware") {
		t.Fatalf("install without firmware acknowledgement accepted: %v", err)
	}
	if _, err = m.StartAcknowledged(p.ID, false, true); err != nil {
		t.Fatal(err)
	}
	state := settled(t, m)
	if state.Job.Phase != "succeeded" {
		t.Fatalf("%s: %s", state.Job.Phase, state.Job.Error)
	}
	want := &FirmwareProtocolChange{From: 6, To: 7}
	if len(b.gateChanges) != 1 || b.gateChanges[0] == nil || *b.gateChanges[0] != *want {
		t.Fatalf("maintenance gate was not told about the change: %+v", b.gateChanges)
	}
	if len(b.verifyChanges) != 1 || b.verifyChanges[0] == nil || *b.verifyChanges[0] != *want {
		t.Fatalf("verification was not told about the change: %+v", b.verifyChanges)
	}

	// Rolling back before the reflash meets the same refusal from the other side.
	if _, err = m.Rollback(); err != nil {
		t.Fatal(err)
	}
	if state = settled(t, m); state.Job.Phase != "rolled_back" {
		t.Fatalf("%s: %s %s", state.Job.Phase, state.Job.Error, state.Job.RecoveryError)
	}
	if len(b.gateChanges) != 2 || b.gateChanges[1] == nil || *b.gateChanges[1] != *want {
		t.Fatalf("rollback gate was not told about the change: %+v", b.gateChanges)
	}
	if len(b.verifyChanges) != 2 || b.verifyChanges[1] == nil || *b.verifyChanges[1] != *want {
		t.Fatalf("rollback verification was not told about the change: %+v", b.verifyChanges)
	}
}

// A matching protocol must never record a change or demand the acknowledgement.
func TestMatchingFirmwareProtocolNeedsNoAcknowledgement(t *testing.T) {
	m, b, _ := setup(t, "")
	p, err := m.MakeServicePlan(context.Background(), fixture().ID, false, nil, PlanOptions{AllowFirmwareProtocolChange: true})
	if err != nil || p.FirmwareProtocolChange != nil {
		t.Fatalf("matching protocol recorded a change: %+v %v", p.FirmwareProtocolChange, err)
	}
	if _, err = m.Start(p.ID); err != nil {
		t.Fatal(err)
	}
	if state := settled(t, m); state.Job.Phase != "succeeded" {
		t.Fatalf("%s: %s", state.Job.Phase, state.Job.Error)
	}
	if len(b.gateChanges) != 1 || b.gateChanges[0] != nil {
		t.Fatalf("gate received a change for a matching protocol: %+v", b.gateChanges)
	}
}

// A GUI that predates the firmware flags omits them, and it is precisely the
// GUI that can only get the matching firmware from the image it is installing:
// omission must not lock it out. A GUI that knows the flags sends explicit
// values and keeps its own gating.
func TestLegacyGUIForcesProtocolChangeByOmission(t *testing.T) {
	newer := fixture()
	newer.ID, newer.ReleaseTag, newer.FirmwareProtocol = "deployment-v7", "deployment-v7", 7
	b := &fakeBackend{fingerprint: "original"}
	m, err := Open(t.TempDir(), []string{"mowglinext/mowglinext"}, b, &fakeSource{releases: []Deployment{newer}})
	if err != nil {
		t.Fatal(err)
	}
	if err = m.Check(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	post := func(path, body string) (int, string) {
		w := httptest.NewRecorder()
		m.Handler(HostConfig{}).ServeHTTP(w, httptest.NewRequest("POST", path, strings.NewReader(body)))
		return w.Code, w.Body.String()
	}
	// New GUI, allowance not ticked: explicit false is refused.
	if code, body := post("/v1/plan", `{"deployment":"deployment-v7","allow_firmware_protocol_change":false}`); code != 409 || !strings.Contains(body, "different mainboard firmware protocol") {
		t.Fatalf("explicit refusal lost: %d %s", code, body)
	}
	// Legacy GUI: no flag at all plans the change.
	code, body := post("/v1/plan", `{"deployment":"deployment-v7"}`)
	var plan Plan
	if code != 200 || json.Unmarshal([]byte(body), &plan) != nil || plan.FirmwareProtocolChange == nil || *plan.FirmwareProtocolChange != (FirmwareProtocolChange{From: 6, To: 7}) {
		t.Fatalf("legacy plan did not force the change: %d %s", code, body)
	}
	// New GUI, acknowledgement not ticked: explicit false is refused at install.
	if code, body = post("/v1/apply", `{"plan":"`+plan.ID+`","firmware_protocol_acknowledged":false}`); code != 409 || !strings.Contains(body, "firmware") {
		t.Fatalf("explicit install refusal lost: %d %s", code, body)
	}
	// Legacy GUI installs without the field.
	if code, body = post("/v1/apply", `{"plan":"`+plan.ID+`"}`); code != 200 {
		t.Fatalf("legacy install refused: %d %s", code, body)
	}
	if state := settled(t, m); state.Job.Phase != "succeeded" {
		t.Fatalf("%s: %s", state.Job.Phase, state.Job.Error)
	}
}

// The release protocol is the Mowgli STM32 wire protocol: waived only when the
// host's own docker/.env says the board is an OpenMower one.
func TestFirmwareProtocolIsWaivedOnlyOnTheOpenMowerBackend(t *testing.T) {
	target := Deployment{FirmwareProtocol: 7}
	for _, test := range []struct {
		env    string
		exempt bool
	}{
		{"HARDWARE_BACKEND=openmower\n", true},
		{"HARDWARE_BACKEND=mowgli\n", false},
		{"", false},
		{"-", false}, // no .env at all
	} {
		dir := t.TempDir()
		if test.env != "-" {
			if err := os.WriteFile(filepath.Join(dir, ".env"), []byte(test.env), 0600); err != nil {
				t.Fatal(err)
			}
		}
		b := DockerBackend{Config: HostConfig{Directory: dir}}
		if b.FirmwareProtocolExempt() != test.exempt {
			t.Fatalf("%q: exempt=%v", test.env, !test.exempt)
		}
		change, err := b.checkFirmwareProtocol(0, target, PlanOptions{})
		if test.exempt && (err != nil || change != nil) {
			t.Fatalf("%q: OpenMower plan refused: %v %v", test.env, change, err)
		}
		if !test.exempt && !errors.Is(err, errFirmwareProtocolUnavailable) {
			t.Fatalf("%q: a board with no handshake was accepted: %v", test.env, err)
		}
	}
}
