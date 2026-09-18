package providers

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

type fixedClock struct{ n time.Time }

func (c fixedClock) Now() time.Time { return c.n }

type fakeArtifacts struct {
	a   FirmwareArtifact
	err error
}

func (f fakeArtifacts) Resolve(context.Context, types.FirmwareUpdateRequest) (FirmwareArtifact, error) {
	return f.a, f.err
}

type selectiveArtifacts struct{ a FirmwareArtifact }

func (f selectiveArtifacts) Resolve(_ context.Context, req types.FirmwareUpdateRequest) (FirmwareArtifact, error) {
	if req.IdempotencyKey == "invalid-after-uncertain" {
		return FirmwareArtifact{}, errors.New("invalid artifact")
	}
	return f.a, nil
}

type fakeReady struct {
	s   FirmwareReadinessStatus
	err error
}

type blockingReady struct{ started chan struct{} }

func (f blockingReady) Check(ctx context.Context, _ bool) (FirmwareReadinessStatus, error) {
	close(f.started)
	<-ctx.Done()
	return FirmwareReadinessStatus{}, ctx.Err()
}

func (f fakeReady) Check(context.Context, bool) (FirmwareReadinessStatus, error) { return f.s, f.err }

type fakeBridge struct{ err error }

func (f fakeBridge) RequestDFU(context.Context) error { return f.err }

type maintenanceBridge struct {
	mu         sync.Mutex
	maintained bool
	leaveCalls int
}

func (b *maintenanceBridge) RequestDFU(context.Context) error {
	b.mu.Lock()
	b.maintained = true
	b.mu.Unlock()
	return errors.New("ambiguous DFU dispatch")
}

func (b *maintenanceBridge) LeaveMaintenance(context.Context) error {
	b.mu.Lock()
	defer b.mu.Unlock()
	b.maintained = false
	b.leaveCalls++
	return nil
}

type blockingBridge struct {
	started chan struct{}
	release chan struct{}
}

func (f blockingBridge) RequestDFU(context.Context) error {
	close(f.started)
	<-f.release
	return errors.New("ambiguous service result")
}

type countingBridge struct {
	mu    sync.Mutex
	calls int
}

func (b *countingBridge) RequestDFU(context.Context) error {
	b.mu.Lock()
	b.calls++
	b.mu.Unlock()
	return nil
}

type fakeUSB struct{ err error }

func (f fakeUSB) AnchorCDC(context.Context) (USBIdentity, error) {
	return USBIdentity{PhysicalPort: "1-2", Device: "tty"}, nil
}
func (f fakeUSB) WaitForDFU(context.Context, USBIdentity) (USBIdentity, error) {
	return USBIdentity{PhysicalPort: "1-2", Device: "dfu"}, f.err
}
func (f fakeUSB) WaitForApplication(context.Context, USBIdentity) error { return f.err }

type cancelThenAnchorUSB struct {
	started chan struct{}
	resume  chan struct{}
}

func (f cancelThenAnchorUSB) AnchorCDC(context.Context) (USBIdentity, error) {
	close(f.started)
	<-f.resume
	return USBIdentity{PhysicalPort: "1-2", Device: "tty"}, nil
}
func (f cancelThenAnchorUSB) WaitForDFU(context.Context, USBIdentity) (USBIdentity, error) {
	return USBIdentity{PhysicalPort: "1-2", Device: "dfu"}, nil
}
func (f cancelThenAnchorUSB) WaitForApplication(context.Context, USBIdentity) error { return nil }

type fakeDFU struct{ write, verify, leave error }

func (f fakeDFU) Write(context.Context, USBIdentity, FirmwareArtifact) error  { return f.write }
func (f fakeDFU) Verify(context.Context, USBIdentity, FirmwareArtifact) error { return f.verify }
func (f fakeDFU) Leave(context.Context, USBIdentity) error                    { return f.leave }

type fakeRuntime struct{ err error }

func (f fakeRuntime) Verify(context.Context, FirmwareArtifact) error { return f.err }

type transitionLog struct {
	mu      sync.Mutex
	events  []string
	failAt  string
	blockAt string
}

func (l *transitionLog) add(name string) error {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.events = append(l.events, name)
	if l.failAt == name {
		return errors.New(name)
	}
	return nil
}
func (l *transitionLog) addContext(ctx context.Context, name string) error {
	if err := l.add(name); err != nil {
		return err
	}
	if l.blockAt == name {
		<-ctx.Done()
		return ctx.Err()
	}
	return nil
}
func (l *transitionLog) snapshot() []string {
	l.mu.Lock()
	defer l.mu.Unlock()
	return append([]string(nil), l.events...)
}

type campaignArtifact struct {
	log *transitionLog
	a   FirmwareArtifact
}

func (f campaignArtifact) Resolve(ctx context.Context, _ types.FirmwareUpdateRequest) (FirmwareArtifact, error) {
	return f.a, f.log.addContext(ctx, "artifact")
}

type campaignReady struct{ log *transitionLog }

func (f campaignReady) Check(ctx context.Context, _ bool) (FirmwareReadinessStatus, error) {
	return validReadiness(), f.log.addContext(ctx, "readiness")
}

type campaignBridge struct{ log *transitionLog }

func (f campaignBridge) RequestDFU(ctx context.Context) error {
	return f.log.addContext(ctx, "bridge")
}

type campaignUSB struct{ log *transitionLog }

func (f campaignUSB) AnchorCDC(ctx context.Context) (USBIdentity, error) {
	return USBIdentity{PhysicalPort: "p"}, f.log.addContext(ctx, "anchor")
}
func (f campaignUSB) WaitForDFU(ctx context.Context, _ USBIdentity) (USBIdentity, error) {
	return USBIdentity{PhysicalPort: "p"}, f.log.addContext(ctx, "dfu")
}
func (f campaignUSB) WaitForApplication(ctx context.Context, _ USBIdentity) error {
	return f.log.addContext(ctx, "application")
}

type campaignDFU struct{ log *transitionLog }

func (f campaignDFU) Write(ctx context.Context, _ USBIdentity, _ FirmwareArtifact) error {
	return f.log.addContext(ctx, "write")
}
func (f campaignDFU) Verify(ctx context.Context, _ USBIdentity, _ FirmwareArtifact) error {
	return f.log.addContext(ctx, "verify")
}
func (f campaignDFU) Leave(ctx context.Context, _ USBIdentity) error {
	return f.log.addContext(ctx, "leave")
}

type campaignRuntime struct{ log *transitionLog }

func (f campaignRuntime) Verify(ctx context.Context, _ FirmwareArtifact) error {
	return f.log.addContext(ctx, "runtime")
}

func validUSBArtifact() FirmwareArtifact {
	b := make([]byte, 32)
	b[0] = 0
	b[1] = 0x80
	b[2] = 0
	b[3] = 0x20
	b[4] = 9
	b[5] = 0
	b[6] = 0
	b[7] = 8
	sum := sha256.Sum256(b)
	return FirmwareArtifact{Board: stm32F401Board, Environment: "Yardforce500B", Panel: "classic", MCU: "STM32F401VC", FlashAddress: "0x08000000", FlashSize: maxDFUImage, USBDfu: true, ProtocolVersion: 1, FirmwareVersion: "1.2.3", Size: len(b), Bytes: b, SHA256: hex.EncodeToString(sum[:])}
}
func validUSBRequest() types.FirmwareUpdateRequest {
	return types.FirmwareUpdateRequest{IdempotencyKey: "k", Board: stm32F401Board, Environment: "Yardforce500B", Panel: "classic"}
}
func validReadiness() FirmwareReadinessStatus {
	return FirmwareReadinessStatus{BladeTelemetryHealthy: true, FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "IDLE"}
}
func newTestUpdater(t *testing.T, a FirmwareArtifact, r FirmwareReadinessStatus, d fakeDFU) *FirmwareUSBUpdater {
	t.Helper()
	u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{Clock: fixedClock{time.Unix(1, 0)}, ArtifactSource: fakeArtifacts{a: a}, Readiness: fakeReady{s: r}, Bridge: fakeBridge{}, USBObserver: fakeUSB{}, DFUTool: d, RuntimeVerifier: fakeRuntime{}, StepTimeout: time.Second})
	if err != nil {
		t.Fatal(err)
	}
	return u
}

func TestUSBUpdaterDetachesFromRequestContext(t *testing.T) {
	u := newTestUpdater(t, validUSBArtifact(), validReadiness(), fakeDFU{})
	requestContext, cancel := context.WithCancel(context.Background())
	cancel()
	s, _, err := u.StartFirmwareUSBUpdate(requestContext, validUSBRequest())
	if err != nil {
		t.Fatal(err)
	}
	if got := waitTerminal(t, u, s.ID); got.State != types.FirmwareUpdateSucceeded {
		t.Fatalf("request cancellation leaked into operation: %#v", got)
	}
}

func TestUSBUpdaterExplicitCancelBeforeEntry(t *testing.T) {
	started := make(chan struct{})
	u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{Clock: fixedClock{time.Unix(1, 0)}, ArtifactSource: fakeArtifacts{a: validUSBArtifact()}, Readiness: blockingReady{started}, Bridge: fakeBridge{}, USBObserver: fakeUSB{}, DFUTool: fakeDFU{}, RuntimeVerifier: fakeRuntime{}, StepTimeout: time.Second})
	if err != nil {
		t.Fatal(err)
	}
	s, _, err := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if err != nil {
		t.Fatal(err)
	}
	<-started
	if _, err := u.CancelFirmwareUSBUpdate(s.ID); err != nil {
		t.Fatal(err)
	}
	if got := waitTerminal(t, u, s.ID); got.State != types.FirmwareUpdateCancelledBeforeEntry {
		t.Fatalf("got %#v", got)
	}
}

func TestUSBUpdaterCancelAtEntryBoundaryNeverDispatchesDFU(t *testing.T) {
	started, resume := make(chan struct{}), make(chan struct{})
	bridge := &countingBridge{}
	u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{
		Clock:           fixedClock{time.Unix(1, 0)},
		ArtifactSource:  fakeArtifacts{a: validUSBArtifact()},
		Readiness:       fakeReady{s: validReadiness()},
		Bridge:          bridge,
		USBObserver:     cancelThenAnchorUSB{started, resume},
		DFUTool:         fakeDFU{},
		RuntimeVerifier: fakeRuntime{},
		StepTimeout:     time.Second,
	})
	if err != nil {
		t.Fatal(err)
	}
	snapshot, _, err := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if err != nil {
		t.Fatal(err)
	}
	<-started
	if _, err := u.CancelFirmwareUSBUpdate(snapshot.ID); err != nil {
		t.Fatal(err)
	}
	close(resume)
	if got := waitTerminal(t, u, snapshot.ID); got.State != types.FirmwareUpdateCancelledBeforeEntry {
		t.Fatalf("got %#v", got)
	}
	bridge.mu.Lock()
	defer bridge.mu.Unlock()
	if bridge.calls != 0 {
		t.Fatalf("ENTER_DFU dispatched %d times after accepted cancellation", bridge.calls)
	}
}

func TestUSBUpdaterRefusesCancelOnceDFUDispatchBegins(t *testing.T) {
	started, release := make(chan struct{}), make(chan struct{})
	u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{Clock: fixedClock{time.Unix(1, 0)}, ArtifactSource: fakeArtifacts{a: validUSBArtifact()}, Readiness: fakeReady{s: validReadiness()}, Bridge: blockingBridge{started, release}, USBObserver: fakeUSB{}, DFUTool: fakeDFU{}, RuntimeVerifier: fakeRuntime{}, StepTimeout: time.Second})
	if err != nil {
		t.Fatal(err)
	}
	s, _, err := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if err != nil {
		t.Fatal(err)
	}
	<-started
	if snapshot, err := u.CancelFirmwareUSBUpdate(s.ID); err == nil || snapshot.Cancellable {
		t.Fatalf("cancel accepted after DFU dispatch: %#v err=%v", snapshot, err)
	}
	close(release)
	if got := waitTerminal(t, u, s.ID); got.State != types.FirmwareUpdateFailedRequiresSTLink {
		t.Fatalf("ambiguous dispatch = %#v", got)
	}
}

func TestRejectedNewRequestCannotReleasePriorUncertainMaintenance(t *testing.T) {
	bridge := &maintenanceBridge{}
	u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{
		Clock:           fixedClock{time.Unix(1, 0)},
		ArtifactSource:  selectiveArtifacts{validUSBArtifact()},
		Readiness:       fakeReady{s: validReadiness()},
		Bridge:          bridge,
		USBObserver:     fakeUSB{},
		DFUTool:         fakeDFU{},
		RuntimeVerifier: fakeRuntime{},
		StepTimeout:     time.Second,
	})
	if err != nil {
		t.Fatal(err)
	}
	first, _, err := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if err != nil {
		t.Fatal(err)
	}
	if got := waitTerminal(t, u, first.ID); got.State != types.FirmwareUpdateFailedRequiresSTLink {
		t.Fatalf("first operation = %#v", got)
	}
	secondRequest := validUSBRequest()
	secondRequest.IdempotencyKey = "invalid-after-uncertain"
	second, _, err := u.StartFirmwareUSBUpdate(context.Background(), secondRequest)
	if err != nil {
		t.Fatal(err)
	}
	if got := waitTerminal(t, u, second.ID); got.State != types.FirmwareUpdateRejected {
		t.Fatalf("second operation = %#v", got)
	}
	bridge.mu.Lock()
	defer bridge.mu.Unlock()
	if !bridge.maintained || bridge.leaveCalls != 0 {
		t.Fatalf("prior maintenance ownership was released: maintained=%v leaveCalls=%d", bridge.maintained, bridge.leaveCalls)
	}
}

func TestUSBReadinessRecoveryOnlyWaivesBladeTelemetry(t *testing.T) {
	for _, tc := range []struct {
		name                string
		readiness           FirmwareReadinessStatus
		recovery, confirmed bool
		want                types.FirmwareUpdateState
	}{
		{"normal unhealthy blade", FirmwareReadinessStatus{FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "IDLE"}, false, false, types.FirmwareUpdateRejected},
		{"confirmed recovery unhealthy blade", FirmwareReadinessStatus{FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "IDLE"}, true, true, types.FirmwareUpdateSucceeded},
		{"recovery incompatible", FirmwareReadinessStatus{BladeTelemetryHealthy: true, USBDfuCapable: true, HighLevelState: "IDLE"}, true, true, types.FirmwareUpdateRejected},
		{"recovery active blade command", FirmwareReadinessStatus{BladeTelemetryHealthy: false, FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "IDLE", MotionActive: true}, true, true, types.FirmwareUpdateRejected},
	} {
		t.Run(tc.name, func(t *testing.T) {
			u := newTestUpdater(t, validUSBArtifact(), tc.readiness, fakeDFU{})
			req := validUSBRequest()
			req.IdempotencyKey = tc.name
			req.Recovery = tc.recovery
			req.RecoveryConfirmed = tc.confirmed
			s, _, err := u.StartFirmwareUSBUpdate(context.Background(), req)
			if err != nil {
				t.Fatal(err)
			}
			if got := waitTerminal(t, u, s.ID); got.State != tc.want {
				t.Fatalf("got %#v", got)
			}
		})
	}
}
func waitTerminal(t *testing.T, u *FirmwareUSBUpdater, id string) types.FirmwareUpdateSnapshot {
	t.Helper()
	deadline := time.Now().Add(time.Second)
	for time.Now().Before(deadline) {
		s, _ := u.FirmwareUSBUpdateSnapshot(id)
		if terminalFirmwareUpdateState(s.State) {
			return s
		}
		time.Sleep(time.Millisecond)
	}
	t.Fatal("update did not complete")
	return types.FirmwareUpdateSnapshot{}
}

func TestUSBUpdaterSuccessAndIdempotency(t *testing.T) {
	u := newTestUpdater(t, validUSBArtifact(), FirmwareReadinessStatus{BladeTelemetryHealthy: true, FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "IDLE"}, fakeDFU{})
	first, attached, err := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if err != nil || attached {
		t.Fatalf("start: %v attached %v", err, attached)
	}
	again, attached, err := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if err != nil || !attached || again.ID != first.ID {
		t.Fatalf("idempotency failed: %#v %v %v", again, attached, err)
	}
	if got := waitTerminal(t, u, first.ID); got.State != types.FirmwareUpdateSucceeded {
		t.Fatalf("got %s: %#v", got.State, got.Error)
	}
}
func TestUSBUpdaterRejectsUnsafeReadiness(t *testing.T) {
	u := newTestUpdater(t, validUSBArtifact(), FirmwareReadinessStatus{BladeTelemetryHealthy: true, FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "unknown"}, fakeDFU{})
	s, _, _ := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if got := waitTerminal(t, u, s.ID); got.State != types.FirmwareUpdateRejected || got.Error.Code != "readiness_rejected" {
		t.Fatalf("got %#v", got)
	}
}
func TestUSBUpdaterWriteFailureRequiresSTLink(t *testing.T) {
	u := newTestUpdater(t, validUSBArtifact(), FirmwareReadinessStatus{BladeTelemetryHealthy: true, FirmwareCompatible: true, USBDfuCapable: true, HighLevelState: "IDLE"}, fakeDFU{write: errors.New("disconnect")})
	s, _, _ := u.StartFirmwareUSBUpdate(context.Background(), validUSBRequest())
	if got := waitTerminal(t, u, s.ID); got.State != types.FirmwareUpdateFailedRequiresSTLink || got.Cancellable {
		t.Fatalf("got %#v", got)
	}
}

func TestUSBArtifactValidationRejectsEveryTrustBoundary(t *testing.T) {
	base := validUSBArtifact()
	request := validUSBRequest()
	for _, tc := range []struct {
		name   string
		mutate func(*types.FirmwareUpdateRequest, *FirmwareArtifact)
	}{
		{name: "wrong requested board", mutate: func(r *types.FirmwareUpdateRequest, _ *FirmwareArtifact) { r.Board = "BOARD_YARDFORCE500" }},
		{name: "wrong artifact board", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.Board = "BOARD_YARDFORCE500" }},
		{name: "wrong MCU", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.MCU = "STM32F103VC" }},
		{name: "wrong panel", mutate: func(r *types.FirmwareUpdateRequest, _ *FirmwareArtifact) { r.Panel = "other" }},
		{name: "wrong environment", mutate: func(r *types.FirmwareUpdateRequest, _ *FirmwareArtifact) { r.Environment = "other" }},
		{name: "DFU not approved", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.USBDfu = false }},
		{name: "wrong flash address", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.FlashAddress = "0x08004000" }},
		{name: "wrong flash size", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.FlashSize-- }},
		{name: "empty", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.Bytes = nil; a.Size = 0 }},
		{name: "truncated", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) {
			a.Bytes = a.Bytes[:7]
			a.Size = len(a.Bytes)
		}},
		{name: "oversized", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) {
			a.Bytes = make([]byte, maxDFUImage+1)
			a.Size = len(a.Bytes)
		}},
		{name: "declared size mismatch", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.Size++ }},
		{name: "wrong SHA", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.SHA256 = "00" }},
		{name: "invalid stack vector", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) {
			a.Bytes = append([]byte(nil), a.Bytes...)
			binary.LittleEndian.PutUint32(a.Bytes[:4], 0x10000000)
			sum := sha256.Sum256(a.Bytes)
			a.SHA256 = hex.EncodeToString(sum[:])
		}},
		{name: "invalid reset vector", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) {
			a.Bytes = append([]byte(nil), a.Bytes...)
			binary.LittleEndian.PutUint32(a.Bytes[4:8], 0x08000008)
			sum := sha256.Sum256(a.Bytes)
			a.SHA256 = hex.EncodeToString(sum[:])
		}},
		{name: "missing protocol", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.ProtocolVersion = 0 }},
		{name: "missing version", mutate: func(_ *types.FirmwareUpdateRequest, a *FirmwareArtifact) { a.FirmwareVersion = "" }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			a := base
			a.Bytes = append([]byte(nil), base.Bytes...)
			r := request
			tc.mutate(&r, &a)
			if err := validateUSBArtifact(r, a); err == nil {
				t.Fatal("invalid artifact was accepted")
			}
		})
	}
}

func TestDFUUtilSelectorPinsExpectedDeviceAndPhysicalPort(t *testing.T) {
	got := dfuUtilSelector(USBIdentity{PhysicalPort: "1-2.3", Device: "/sys/bus/usb/devices/1-2.3"})
	want := []string{"-d", "0483:df11", "-p", "1-2.3", "-a", "0"}
	if fmt.Sprint(got) != fmt.Sprint(want) {
		t.Fatalf("selector = %#v, want %#v", got, want)
	}
	leave := dfuUtilLeaveArgs(USBIdentity{PhysicalPort: "1-2.3"})
	if fmt.Sprint(leave) != fmt.Sprint(append(want, "-s", ":leave")) {
		t.Fatalf("leave args = %#v", leave)
	}
}

func TestDFUUtilToolPinsIdentityChecksExitAndReadback(t *testing.T) {
	dir := t.TempDir()
	binary := filepath.Join(dir, "dfu-util")
	logPath := filepath.Join(dir, "args.log")
	readbackPath := filepath.Join(dir, "readback.bin")
	script := `#!/bin/sh
printf '%s\n' "$*" >> "$DFU_LOG"
if [ "$DFU_FAIL" = "1" ]; then exit 9; fi
previous=
for argument in "$@"; do
  if [ "$previous" = "-U" ]; then cp "$DFU_READBACK" "$argument"; fi
  previous="$argument"
done
`
	if err := os.WriteFile(binary, []byte(script), 0o700); err != nil {
		t.Fatal(err)
	}
	artifact := validUSBArtifact()
	if err := os.WriteFile(readbackPath, artifact.Bytes, 0o600); err != nil {
		t.Fatal(err)
	}
	t.Setenv("DFU_LOG", logPath)
	t.Setenv("DFU_READBACK", readbackPath)
	t.Setenv("DFU_FAIL", "0")
	tool := DFUUtilTool{Binary: binary}
	id := USBIdentity{PhysicalPort: "1-2.3"}
	if err := tool.Write(context.Background(), id, artifact); err != nil {
		t.Fatal(err)
	}
	if err := tool.Verify(context.Background(), id, artifact); err != nil {
		t.Fatal(err)
	}
	if err := tool.Leave(context.Background(), id); err != nil {
		t.Fatal(err)
	}
	logBytes, err := os.ReadFile(logPath)
	if err != nil {
		t.Fatal(err)
	}
	lines := bytes.Split(bytes.TrimSpace(logBytes), []byte("\n"))
	if len(lines) != 3 {
		t.Fatalf("dfu-util calls = %q", logBytes)
	}
	for _, line := range lines {
		if !bytes.Contains(line, []byte("-d 0483:df11 -p 1-2.3 -a 0")) {
			t.Fatalf("unscoped dfu-util call: %q", line)
		}
	}
	if !bytes.Contains(lines[0], []byte("-s 0x08000000 -D")) ||
		!bytes.Contains(lines[1], []byte(fmt.Sprintf("-s 0x08000000:%d -U", len(artifact.Bytes)))) ||
		!bytes.Contains(lines[2], []byte("-s :leave")) {
		t.Fatalf("unexpected dfu-util sequence: %q", logBytes)
	}

	if err := os.WriteFile(readbackPath, []byte("partial"), 0o600); err != nil {
		t.Fatal(err)
	}
	if err := tool.Verify(context.Background(), id, artifact); err == nil {
		t.Fatal("partial readback was accepted")
	}
	t.Setenv("DFU_FAIL", "1")
	if err := tool.Write(context.Background(), id, artifact); err == nil {
		t.Fatal("nonzero dfu-util exit was accepted")
	}
}

func createDFUDevice(sysroot, port string) (string, error) {
	dir := filepath.Join(sysroot, "bus", "usb", "devices", port)
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return "", err
	}
	if err := os.WriteFile(filepath.Join(dir, "idVendor"), []byte("0483\n"), 0o600); err != nil {
		return "", err
	}
	if err := os.WriteFile(filepath.Join(dir, "idProduct"), []byte("df11\n"), 0o600); err != nil {
		return "", err
	}
	return dir, nil
}

func addDFUDevice(t *testing.T, sysroot, port string) string {
	t.Helper()
	dir, err := createDFUDevice(sysroot, port)
	if err != nil {
		t.Fatal(err)
	}
	return dir
}

func TestLinuxUSBObserverRejectsAmbiguousOrWrongDFUIdentity(t *testing.T) {
	for _, tc := range []struct {
		name        string
		ports       []string
		preexisting bool
		wantPort    string
		wantErr     bool
	}{
		{name: "expected device", ports: []string{"1-2.3"}, wantPort: "1-2.3"},
		{name: "wrong physical port", ports: []string{"9-9"}, wantErr: true},
		{name: "multiple devices", ports: []string{"1-2.3", "9-9"}, wantErr: true},
		{name: "pre-existing device", ports: []string{"1-2.3"}, preexisting: true, wantErr: true},
		{name: "never appears", wantErr: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			root := t.TempDir()
			observer := &LinuxUSBObserver{SysRoot: root, Poll: time.Millisecond, baseline: map[string]USBIdentity{}}
			for _, port := range tc.ports {
				dir := addDFUDevice(t, root, port)
				if tc.preexisting {
					observer.baseline[dir] = USBIdentity{PhysicalPort: port, Device: dir}
				}
			}
			ctx, cancel := context.WithTimeout(context.Background(), 15*time.Millisecond)
			defer cancel()
			got, err := observer.WaitForDFU(ctx, USBIdentity{PhysicalPort: "1-2.3"})
			if (err != nil) != tc.wantErr {
				t.Fatalf("WaitForDFU() = %#v, %v; wantErr %v", got, err, tc.wantErr)
			}
			if !tc.wantErr && got.PhysicalPort != tc.wantPort {
				t.Fatalf("port = %q, want %q", got.PhysicalPort, tc.wantPort)
			}
		})
	}
}

func TestLinuxUSBObserverAcceptsLateExpectedDFU(t *testing.T) {
	root := t.TempDir()
	observer := &LinuxUSBObserver{SysRoot: root, Poll: time.Millisecond, baseline: map[string]USBIdentity{}}
	created := make(chan error, 1)
	go func() {
		time.Sleep(3 * time.Millisecond)
		_, err := createDFUDevice(root, "1-2.3")
		created <- err
	}()
	ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
	defer cancel()
	got, err := observer.WaitForDFU(ctx, USBIdentity{PhysicalPort: "1-2.3"})
	if createErr := <-created; createErr != nil {
		t.Fatal(createErr)
	}
	if err != nil || got.PhysicalPort != "1-2.3" {
		t.Fatalf("WaitForDFU() = %#v, %v", got, err)
	}
}

func TestLinuxUSBObserverAnchorsReturningCDCToSamePhysicalPort(t *testing.T) {
	root := t.TempDir()
	usbDir := filepath.Join(root, "bus", "usb", "devices", "1-2.3")
	if err := os.MkdirAll(usbDir, 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(usbDir, "idVendor"), []byte("0483\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	ttyDir := filepath.Join(root, "class", "tty", "ttyACM0")
	devDir := filepath.Join(root, "dev")
	if err := os.MkdirAll(ttyDir, 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(devDir, 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(usbDir, filepath.Join(ttyDir, "device")); err != nil {
		t.Fatal(err)
	}
	ttyPath := filepath.Join(devDir, "ttyACM0")
	if err := os.WriteFile(ttyPath, nil, 0o600); err != nil {
		t.Fatal(err)
	}
	devLink := filepath.Join(devDir, "mowgli")
	if err := os.Symlink(ttyPath, devLink); err != nil {
		t.Fatal(err)
	}
	observer := &LinuxUSBObserver{DevLink: devLink, SysRoot: root, Poll: time.Millisecond}
	anchor, err := observer.AnchorCDC(context.Background())
	if err != nil || anchor.PhysicalPort != "1-2.3" {
		t.Fatalf("AnchorCDC() = %#v, %v", anchor, err)
	}
	if err := observer.WaitForApplication(context.Background(), anchor); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Millisecond)
	defer cancel()
	if err := observer.WaitForApplication(ctx, USBIdentity{PhysicalPort: "9-9"}); err == nil {
		t.Fatal("wrong returning physical port was accepted")
	}
}

func TestManifestUSBArtifactSourceEnforcesSchemaAndMetadata(t *testing.T) {
	artifact := validUSBArtifact()
	binaryServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		_, _ = w.Write(artifact.Bytes)
	}))
	defer binaryServer.Close()

	entry := firmwareManifestEntry{
		Env:             artifact.Environment,
		Board:           artifact.Board,
		Panel:           artifact.Panel,
		File:            "firmware.bin",
		URL:             binaryServer.URL,
		Sha256:          artifact.SHA256,
		ProtocolVersion: artifact.ProtocolVersion,
		FwVersion:       artifact.FirmwareVersion,
		MCU:             artifact.MCU,
		FlashAddress:    artifact.FlashAddress,
		FlashSize:       artifact.FlashSize,
		Size:            artifact.Size,
		USBDFU:          true,
	}
	for _, tc := range []struct {
		name    string
		mutate  func(*firmwareManifest)
		wantErr bool
	}{
		{name: "valid"},
		{name: "missing schema", mutate: func(m *firmwareManifest) { m.Schema = 0 }, wantErr: true},
		{name: "top-level version disagreement", mutate: func(m *firmwareManifest) { m.FwVersion = "different" }, wantErr: true},
		{name: "wrong flash size", mutate: func(m *firmwareManifest) { e := m.Permutations["f401"]; e.FlashSize--; m.Permutations["f401"] = e }, wantErr: true},
		{name: "artifact size mismatch", mutate: func(m *firmwareManifest) { e := m.Permutations["f401"]; e.Size++; m.Permutations["f401"] = e }, wantErr: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			manifest := firmwareManifest{Schema: 2, ProtocolVersion: artifact.ProtocolVersion, FwVersion: artifact.FirmwareVersion, Permutations: map[string]firmwareManifestEntry{"f401": entry}}
			if tc.mutate != nil {
				tc.mutate(&manifest)
			}
			manifestServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
				if err := json.NewEncoder(w).Encode(manifest); err != nil {
					t.Errorf("encode manifest: %v", err)
				}
			}))
			defer manifestServer.Close()

			got, err := (ManifestUSBArtifactSource{ManifestURL: manifestServer.URL, Client: manifestServer.Client()}).Resolve(context.Background(), validUSBRequest())
			if tc.wantErr {
				if err == nil {
					t.Fatalf("expected rejection, got %#v", got)
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if err := validateUSBArtifact(validUSBRequest(), got); err != nil {
				t.Fatalf("resolved artifact failed validation: %v", err)
			}
		})
	}
}

func TestROSUSBBridgeRuntimeVerificationRequiresFreshCompatibleGeneration(t *testing.T) {
	for _, tc := range []struct {
		name        string
		generation  int
		capability  int
		protocol    int
		version     string
		compatible  bool
		noTelemetry bool
		wantErr     bool
	}{
		{name: "compatible", generation: 6, capability: 1, protocol: 1, version: "1.2.3", compatible: true},
		{name: "incompatible", generation: 6, capability: 1, protocol: 1, version: "1.2.3", wantErr: true},
		{name: "missing capability", generation: 6, protocol: 1, version: "1.2.3", compatible: true, wantErr: true},
		{name: "wrong protocol", generation: 6, capability: 1, protocol: 2, version: "1.2.3", compatible: true, wantErr: true},
		{name: "wrong version", generation: 6, capability: 1, protocol: 1, version: "9.9.9", compatible: true, wantErr: true},
		{name: "stale generation", generation: 5, capability: 1, protocol: 1, version: "1.2.3", compatible: true, wantErr: true},
		{name: "no telemetry", noTelemetry: true, wantErr: true},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ros := types.NewMockRosProvider()
			bridge := &ROSUSBBridge{ROS: ros, baseline: 5}
			timeout := time.Second
			if tc.generation <= 5 {
				timeout = 25 * time.Millisecond
			}
			ctx, cancel := context.WithTimeout(context.Background(), timeout)
			defer cancel()
			result := make(chan error, 1)
			go func() { result <- bridge.Verify(ctx, validUSBArtifact()) }()

			message := fmt.Sprintf(`{"firmware_connection_generation":%d,"firmware_capabilities":%d,"firmware_protocol_version":%d,"firmware_version":%q,"firmware_compatible":%t}`, tc.generation, tc.capability, tc.protocol, tc.version, tc.compatible)
			deadline := time.Now().Add(time.Second)
			for {
				if !tc.noTelemetry {
					ros.Dispatch("status", []byte(message))
				}
				select {
				case err := <-result:
					if (err != nil) != tc.wantErr {
						t.Fatalf("Verify() error = %v, wantErr %v", err, tc.wantErr)
					}
					return
				default:
				}
				if time.Now().After(deadline) {
					t.Fatal("runtime verifier did not observe status")
				}
				time.Sleep(time.Millisecond)
			}
		})
	}
}

func TestROSUSBBridgeIgnoresNewGenerationUntilHandshakeCompletes(t *testing.T) {
	ros := types.NewMockRosProvider()
	bridge := &ROSUSBBridge{ROS: ros, baseline: 5}
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	result := make(chan error, 1)
	go func() { result <- bridge.Verify(ctx, validUSBArtifact()) }()

	transient := []byte(`{"firmware_connection_generation":6,"firmware_capabilities":0,"firmware_protocol_version":0,"firmware_version":"","firmware_compatible":false}`)
	until := time.Now().Add(10 * time.Millisecond)
	for time.Now().Before(until) {
		ros.Dispatch("status", transient)
		select {
		case err := <-result:
			t.Fatalf("handshake-pending status ended verification: %v", err)
		default:
		}
		time.Sleep(time.Millisecond)
	}
	complete := []byte(`{"firmware_connection_generation":6,"firmware_capabilities":1,"firmware_protocol_version":1,"firmware_version":"1.2.3","firmware_compatible":true}`)
	deadline := time.Now().Add(time.Second)
	for {
		ros.Dispatch("status", complete)
		select {
		case err := <-result:
			if err != nil {
				t.Fatal(err)
			}
			return
		default:
		}
		if time.Now().After(deadline) {
			t.Fatal("completed handshake was not accepted")
		}
		time.Sleep(time.Millisecond)
	}
}

func TestROSFirmwareReadinessRecoveryCanWaiveFalseRPMButNotCommandOwnership(t *testing.T) {
	ros := types.NewMockRosProvider()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	result := make(chan struct {
		status FirmwareReadinessStatus
		err    error
	}, 1)
	go func() {
		status, err := (ROSFirmwareReadiness{ROS: ros}).Check(ctx, false)
		result <- struct {
			status FirmwareReadinessStatus
			err    error
		}{status, err}
	}()
	now := time.Now().Unix()
	hardware := fmt.Sprintf(`{"stamp":{"sec":%d},"mow_enabled":false,"esc_power":false,"mower_esc_status":0,"mower_motor_rpm":3500,"firmware_compatible":true,"firmware_capabilities":1,"blade_status_stamp":{"sec":%d}}`, now, now)
	deadline := time.Now().Add(time.Second)
	for {
		ros.Dispatch("status", []byte(hardware))
		ros.Dispatch("highLevelStatus", []byte(`{"state_name":"IDLE"}`))
		select {
		case got := <-result:
			if got.err != nil {
				t.Fatal(got.err)
			}
			if got.status.BladeTelemetryHealthy || got.status.MotionActive {
				t.Fatalf("false RPM mapping = %#v", got.status)
			}
			if err := validateReadiness(validUSBRequest(), got.status); err == nil {
				t.Fatal("normal readiness accepted false RPM")
			}
			recovery := validUSBRequest()
			recovery.Recovery = true
			recovery.RecoveryConfirmed = true
			if err := validateReadiness(recovery, got.status); err != nil {
				t.Fatalf("confirmed recovery rejected false RPM: %v", err)
			}
			got.status.MotionActive = true
			if err := validateReadiness(recovery, got.status); err == nil {
				t.Fatal("recovery waived active mow command")
			}
			return
		default:
		}
		if time.Now().After(deadline) {
			t.Fatal("readiness adapter did not observe status")
		}
		time.Sleep(time.Millisecond)
	}
}

func TestUSBArtifactValidationDeterministicCampaign(t *testing.T) {
	a := validUSBArtifact()
	req := validUSBRequest()
	for seed := 0; seed < 4096; seed++ {
		for event := 0; event < 64; event++ {
			candidate := a
			candidate.Bytes = append([]byte(nil), a.Bytes...)
			candidate.Bytes[(seed+event)%len(candidate.Bytes)] ^= 1 + byte((seed+event)%255)
			if err := validateUSBArtifact(req, candidate); err == nil {
				t.Fatalf("seed %d event %d accepted mutated artifact", seed, event)
			}
		}
	}
}

func TestUSBUpdaterTimeoutAtEveryBoundedDependency(t *testing.T) {
	for _, stage := range []string{"readiness", "anchor", "bridge", "dfu", "write", "verify", "leave", "application", "runtime"} {
		t.Run(stage, func(t *testing.T) {
			log := &transitionLog{blockAt: stage}
			u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{
				Clock:           fixedClock{time.Unix(1, 0)},
				ArtifactSource:  campaignArtifact{log, validUSBArtifact()},
				Readiness:       campaignReady{log},
				Bridge:          campaignBridge{log},
				USBObserver:     campaignUSB{log},
				DFUTool:         campaignDFU{log},
				RuntimeVerifier: campaignRuntime{log},
				StepTimeout:     2 * time.Millisecond,
			})
			if err != nil {
				t.Fatal(err)
			}
			req := validUSBRequest()
			req.IdempotencyKey = "timeout-" + stage
			snapshot, _, err := u.StartFirmwareUSBUpdate(context.Background(), req)
			if err != nil {
				t.Fatal(err)
			}
			final := waitTerminal(t, u, snapshot.ID)
			if final.State == types.FirmwareUpdateSucceeded {
				t.Fatal("timed-out operation reported success")
			}
			count := 0
			for _, event := range log.snapshot() {
				if event == stage {
					count++
				}
			}
			if count != 1 {
				t.Fatalf("stage invoked %d times; automatic retry is forbidden", count)
			}
		})
	}
}

// This runs the production state machine, not a model, against 4096 fixed
// seeds. Each seed deterministically selects one of the 64 event slots; slots
// map to every dependency boundary and success path. The call log asserts that
// flash is never reachable without the complete non-destructive prefix.
func TestUSBUpdaterDeterministicFaultCampaign(t *testing.T) {
	boundaries := []string{"artifact", "readiness", "anchor", "bridge", "dfu", "write", "verify", "leave", "application", "runtime", ""}
	for seed := 0; seed < 4096; seed++ {
		for event := 0; event < 64; event++ {
			log := &transitionLog{failAt: boundaries[(seed+event)%len(boundaries)]}
			u, err := NewFirmwareUSBUpdater(FirmwareUSBDependencies{Clock: fixedClock{time.Unix(1, 0)}, ArtifactSource: campaignArtifact{log, validUSBArtifact()}, Readiness: campaignReady{log}, Bridge: campaignBridge{log}, USBObserver: campaignUSB{log}, DFUTool: campaignDFU{log}, RuntimeVerifier: campaignRuntime{log}, StepTimeout: time.Second})
			if err != nil {
				t.Fatal(err)
			}
			req := validUSBRequest()
			req.IdempotencyKey = fmt.Sprintf("%d-%d", seed, event)
			// Run the same production engine synchronously to keep this 262k-case
			// campaign deterministic and avoid scheduler-dependent polling.
			id := req.IdempotencyKey
			u.byID[id] = &firmwareUpdateOperation{snapshot: types.FirmwareUpdateSnapshot{ID: id, IdempotencyKey: req.IdempotencyKey, State: types.FirmwareUpdateValidating, Cancellable: true, UpdatedAt: time.Unix(1, 0)}}
			u.active = id
			u.run(context.Background(), id, req)
			final, ok := u.FirmwareUSBUpdateSnapshot(id)
			if !ok {
				t.Fatalf("seed %d event %d operation missing", seed, event)
			}
			events := log.snapshot()
			writeIndex := -1
			for i, name := range events {
				if name == "write" {
					writeIndex = i
					break
				}
			}
			if writeIndex >= 0 {
				want := []string{"artifact", "readiness", "anchor", "bridge", "dfu", "write"}
				if len(events) < len(want) {
					t.Fatalf("seed %d event %d short write prefix %#v", seed, event, events)
				}
				for i, name := range want {
					if events[i] != name {
						t.Fatalf("seed %d event %d unsafe write prefix %#v", seed, event, events)
					}
				}
			}
			if log.failAt != "" && final.State == types.FirmwareUpdateSucceeded {
				t.Fatalf("seed %d event %d succeeded despite injected %s", seed, event, log.failAt)
			}
		}
	}
}
