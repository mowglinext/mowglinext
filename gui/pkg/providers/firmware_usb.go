package providers

// The USB updater is purposefully independent of the historical ST-Link path.
// Entering ROM DFU changes the safety MCU's transport, so every transition is
// observable and a process exit is never treated as proof of a good update.

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

const (
	stm32F401Board = "BOARD_YARDFORCE500B"
	maxDFUImage    = 256 * 1024
)

type FirmwareArtifact struct {
	Board, Environment, Panel, MCU string
	FlashAddress                   string
	FlashSize                      int
	USBDfu                         bool
	ProtocolVersion                int
	FirmwareVersion                string
	SHA256                         string
	Size                           int
	Bytes                          []byte
}

type USBIdentity struct{ PhysicalPort, Device string }

type FirmwareArtifactSource interface {
	Resolve(context.Context, types.FirmwareUpdateRequest) (FirmwareArtifact, error)
}
type FirmwareReadiness interface {
	Check(context.Context, bool) (FirmwareReadinessStatus, error)
}
type FirmwareReadinessStatus struct {
	BladeTelemetryHealthy bool
	FirmwareCompatible    bool
	USBDfuCapable         bool
	HighLevelState        string
	MotionActive          bool
}

// ROSFirmwareReadiness requires fresh status from both the safety MCU and the
// high-level controller. It fails closed if either stream is absent. Recovery
// is deliberately not handled here: validateReadiness is the only place that
// may relax the blade-telemetry bit after explicit user confirmation.
type ROSFirmwareReadiness struct{ ROS types.IRosProvider }

func (r ROSFirmwareReadiness) Check(ctx context.Context, _ bool) (FirmwareReadinessStatus, error) {
	if r.ROS == nil {
		return FirmwareReadinessStatus{}, errors.New("ROS bridge is unavailable")
	}
	type hardwareStatus struct {
		Stamp struct {
			Sec     uint64 `json:"sec"`
			Nanosec uint32 `json:"nanosec"`
		} `json:"stamp"`
		MowEnabled           bool    `json:"mow_enabled"`
		EscPower             bool    `json:"esc_power"`
		MowerEscStatus       uint8   `json:"mower_esc_status"`
		MowerMotorRpm        float64 `json:"mower_motor_rpm"`
		FirmwareCompatible   bool    `json:"firmware_compatible"`
		FirmwareCapabilities uint32  `json:"firmware_capabilities"`
		BladeStatusStamp     struct {
			Sec     uint64 `json:"sec"`
			Nanosec uint32 `json:"nanosec"`
		} `json:"blade_status_stamp"`
	}
	type highLevelStatus struct {
		StateName string `json:"state_name"`
	}
	hwReady, highReady := make(chan hardwareStatus, 1), make(chan highLevelStatus, 1)
	const hwID, highID = "usb-dfu-readiness-hardware", "usb-dfu-readiness-high-level"
	if err := r.ROS.Subscribe("status", hwID, 0, func(msg []byte) {
		var sample hardwareStatus
		if json.Unmarshal(msg, &sample) == nil {
			select {
			case hwReady <- sample:
			default:
			}
		}
	}); err != nil {
		return FirmwareReadinessStatus{}, err
	}
	defer r.ROS.UnSubscribe("status", hwID)
	if err := r.ROS.Subscribe("highLevelStatus", highID, 0, func(msg []byte) {
		var sample highLevelStatus
		if json.Unmarshal(msg, &sample) == nil {
			select {
			case highReady <- sample:
			default:
			}
		}
	}); err != nil {
		return FirmwareReadinessStatus{}, err
	}
	defer r.ROS.UnSubscribe("highLevelStatus", highID)
	var hw hardwareStatus
	var high highLevelStatus
	select {
	case hw = <-hwReady:
	case <-ctx.Done():
		return FirmwareReadinessStatus{}, ctx.Err()
	}
	select {
	case high = <-highReady:
	case <-ctx.Done():
		return FirmwareReadinessStatus{}, ctx.Err()
	}
	stamp := time.Unix(int64(hw.BladeStatusStamp.Sec), int64(hw.BladeStatusStamp.Nanosec))
	statusStamp := time.Unix(int64(hw.Stamp.Sec), int64(hw.Stamp.Nanosec))
	now := time.Now()
	// Require an actual recent, inactive blade sample. Compatibility and the
	// USB-DFU capability intentionally stay separate from blade telemetry:
	// recovery cannot turn an old/incompatible image into an acceptable one.
	statusFresh := hw.Stamp.Sec != 0 && !statusStamp.After(now) && now.Sub(statusStamp) <= 3*time.Second
	bladeHealthy := statusFresh && hw.BladeStatusStamp.Sec != 0 && !stamp.After(now) && now.Sub(stamp) <= 3*time.Second && !hw.MowEnabled && !hw.EscPower && hw.MowerEscStatus == 0 && hw.MowerMotorRpm == 0
	// MotionActive is command ownership, not blade telemetry. Recovery may waive
	// demonstrably false/stale blade feedback after physical confirmation, but
	// it must never waive an active mow command.
	return FirmwareReadinessStatus{BladeTelemetryHealthy: bladeHealthy, FirmwareCompatible: hw.FirmwareCompatible, USBDfuCapable: hw.FirmwareCapabilities&1 != 0, HighLevelState: high.StateName, MotionActive: hw.MowEnabled}, nil
}

type FirmwareBridge interface{ RequestDFU(context.Context) error }
type FirmwareUSBObserver interface {
	AnchorCDC(context.Context) (USBIdentity, error)
	WaitForDFU(context.Context, USBIdentity) (USBIdentity, error)
	WaitForApplication(context.Context, USBIdentity) error
}
type FirmwareDFUTool interface {
	Write(context.Context, USBIdentity, FirmwareArtifact) error
	Verify(context.Context, USBIdentity, FirmwareArtifact) error
	Leave(context.Context, USBIdentity) error
}
type FirmwareRuntimeVerifier interface {
	Verify(context.Context, FirmwareArtifact) error
}
type firmwareSafeMaintenanceLeaver interface{ LeaveMaintenance(context.Context) error }

// ROSUSBBridge is the production bridge boundary. It uses the explicitly
// named maintenance gate before asking the firmware to enter ROM DFU.
type ROSUSBBridge struct {
	ROS        types.IRosProvider
	mu         sync.Mutex
	baseline   uint64
	maintained bool
}

func (b *ROSUSBBridge) RequestDFU(ctx context.Context) error {
	if b.ROS == nil {
		return errors.New("ROS bridge is unavailable")
	}
	// A cached status is sufficient as a baseline: runtime verification demands
	// a strictly newer generation after the serial reconnect.
	ready := make(chan uint64, 1)
	const id = "usb-dfu-baseline"
	if err := b.ROS.Subscribe("status", id, 0, func(msg []byte) {
		var status struct {
			FirmwareConnectionGeneration uint64 `json:"firmware_connection_generation"`
		}
		if json.Unmarshal(msg, &status) == nil {
			select {
			case ready <- status.FirmwareConnectionGeneration:
			default:
			}
		}
	}); err != nil {
		return err
	}
	var baseline uint64
	select {
	case baseline = <-ready:
	case <-ctx.Done():
		b.ROS.UnSubscribe("status", id)
		return ctx.Err()
	}
	b.ROS.UnSubscribe("status", id)
	b.mu.Lock()
	b.baseline = baseline
	b.mu.Unlock()
	var setResponse struct {
		Success bool   `json:"success"`
		Message string `json:"message"`
	}
	if err := b.ROS.CallService(ctx, "/hardware_bridge/firmware_update_maintenance", map[string]bool{"data": true}, &setResponse, "std_srvs/srv/SetBool"); err != nil {
		return err
	}
	if !setResponse.Success {
		return fmt.Errorf("maintenance gate refused: %s", setResponse.Message)
	}
	b.mu.Lock()
	b.maintained = true
	b.mu.Unlock()
	var triggerResponse struct {
		Success bool   `json:"success"`
		Message string `json:"message"`
	}
	if err := b.ROS.CallService(ctx, "/hardware_bridge/enter_dfu", map[string]any{}, &triggerResponse, "std_srvs/srv/Trigger"); err != nil {
		return err
	}
	if !triggerResponse.Success {
		return fmt.Errorf("enter DFU refused: %s", triggerResponse.Message)
	}
	return nil
}

func (b *ROSUSBBridge) Verify(ctx context.Context, expected FirmwareArtifact) error {
	if b.ROS == nil {
		return errors.New("ROS bridge is unavailable")
	}
	b.mu.Lock()
	baseline := b.baseline
	b.mu.Unlock()
	matched := make(chan error, 1)
	id := fmt.Sprintf("usb-dfu-runtime-%d", time.Now().UnixNano())
	err := b.ROS.Subscribe("status", id, 0, func(msg []byte) {
		var s struct {
			FirmwareConnectionGeneration uint64 `json:"firmware_connection_generation"`
			FirmwareCapabilities         uint32 `json:"firmware_capabilities"`
			FirmwareProtocolVersion      int    `json:"firmware_protocol_version"`
			FirmwareVersion              string `json:"firmware_version"`
			FirmwareCompatible           bool   `json:"firmware_compatible"`
		}
		if json.Unmarshal(msg, &s) != nil {
			return
		}
		if s.FirmwareConnectionGeneration <= baseline {
			return
		}
		// Serial reconnect increments the generation before CONFIG_RSP completes.
		// Ignore that expected handshake-pending status instead of turning a
		// healthy reconnect into a terminal incompatibility failure.
		if s.FirmwareProtocolVersion == 0 || s.FirmwareVersion == "" {
			return
		}
		if !s.FirmwareCompatible {
			select {
			case matched <- errors.New("reconnected firmware is incompatible with the running bridge"):
			default:
			}
			return
		}
		if s.FirmwareCapabilities&1 == 0 {
			select {
			case matched <- errors.New("reconnected firmware does not advertise USB_DFU capability"):
			default:
			}
			return
		}
		if s.FirmwareProtocolVersion != expected.ProtocolVersion || s.FirmwareVersion != expected.FirmwareVersion {
			select {
			case matched <- errors.New("reconnected firmware protocol/version does not match artifact"):
			default:
			}
			return
		}
		select {
		case matched <- nil:
		default:
		}
	})
	if err != nil {
		return err
	}
	defer b.ROS.UnSubscribe("status", id)
	select {
	case err := <-matched:
		return err
	case <-ctx.Done():
		return ctx.Err()
	}
}

func (b *ROSUSBBridge) LeaveMaintenance(ctx context.Context) error {
	b.mu.Lock()
	active := b.maintained
	b.mu.Unlock()
	if !active {
		return nil
	}
	var response struct {
		Success bool   `json:"success"`
		Message string `json:"message"`
	}
	if err := b.ROS.CallService(ctx, "/hardware_bridge/firmware_update_maintenance", map[string]bool{"data": false}, &response, "std_srvs/srv/SetBool"); err != nil {
		return err
	}
	if !response.Success {
		return fmt.Errorf("maintenance gate refused to clear: %s", response.Message)
	}
	b.mu.Lock()
	b.maintained = false
	b.mu.Unlock()
	return nil
}

type FirmwareClock interface{ Now() time.Time }
type realFirmwareClock struct{}

func (realFirmwareClock) Now() time.Time { return time.Now().UTC() }

type FirmwareUSBDependencies struct {
	Clock           FirmwareClock
	Readiness       FirmwareReadiness
	Bridge          FirmwareBridge
	USBObserver     FirmwareUSBObserver
	DFUTool         FirmwareDFUTool
	RuntimeVerifier FirmwareRuntimeVerifier
	ArtifactSource  FirmwareArtifactSource
	StepTimeout     time.Duration
}

type firmwareUpdateOperation struct {
	snapshot types.FirmwareUpdateSnapshot
	cancel   context.CancelFunc
	entered  bool
}

// FirmwareUSBUpdater serializes updates for the one USB-connected controller.
// Its map doubles as an in-memory operation store: callers can reconnect using
// their idempotency key without issuing another destructive request.
type FirmwareUSBUpdater struct {
	d      FirmwareUSBDependencies
	mu     sync.Mutex
	active string
	byID   map[string]*firmwareUpdateOperation
	byKey  map[string]string
	next   uint64
}

func NewFirmwareUSBUpdater(d FirmwareUSBDependencies) (*FirmwareUSBUpdater, error) {
	if d.Clock == nil {
		d.Clock = realFirmwareClock{}
	}
	if d.StepTimeout <= 0 {
		d.StepTimeout = 20 * time.Second
	}
	if d.Readiness == nil || d.Bridge == nil || d.USBObserver == nil || d.DFUTool == nil || d.RuntimeVerifier == nil || d.ArtifactSource == nil {
		return nil, errors.New("USB updater dependencies are incomplete")
	}
	return &FirmwareUSBUpdater{d: d, byID: map[string]*firmwareUpdateOperation{}, byKey: map[string]string{}}, nil
}

func (u *FirmwareUSBUpdater) StartFirmwareUSBUpdate(ctx context.Context, req types.FirmwareUpdateRequest) (types.FirmwareUpdateSnapshot, bool, error) {
	if req.IdempotencyKey == "" {
		return types.FirmwareUpdateSnapshot{}, false, errors.New("idempotencyKey is required")
	}
	u.mu.Lock()
	defer u.mu.Unlock()
	if id := u.byKey[req.IdempotencyKey]; id != "" {
		return u.byID[id].snapshot, true, nil
	}
	if u.active != "" {
		return u.byID[u.active].snapshot, false, errors.New("another firmware update is active")
	}
	u.next++
	id := fmt.Sprintf("usb-dfu-%d", u.next)
	// A POST context is cancelled as soon as the 202 response is written. The
	// operation owns its lifetime instead; cancellation is explicit and only
	// honoured before DFU entry.
	child, cancel := context.WithCancel(context.Background())
	op := &firmwareUpdateOperation{snapshot: types.FirmwareUpdateSnapshot{ID: id, IdempotencyKey: req.IdempotencyKey, State: types.FirmwareUpdateValidating, Cancellable: true, UpdatedAt: u.d.Clock.Now()}, cancel: cancel}
	u.byID[id] = op
	u.byKey[req.IdempotencyKey] = id
	u.active = id
	go u.run(child, id, req)
	return op.snapshot, false, nil
}

func (u *FirmwareUSBUpdater) FirmwareUSBUpdateSnapshot(id string) (types.FirmwareUpdateSnapshot, bool) {
	u.mu.Lock()
	defer u.mu.Unlock()
	op, ok := u.byID[id]
	if !ok {
		return types.FirmwareUpdateSnapshot{}, false
	}
	return op.snapshot, true
}
func (u *FirmwareUSBUpdater) CancelFirmwareUSBUpdate(id string) (types.FirmwareUpdateSnapshot, error) {
	u.mu.Lock()
	defer u.mu.Unlock()
	op := u.byID[id]
	if op == nil {
		return types.FirmwareUpdateSnapshot{}, errors.New("unknown firmware update")
	}
	if op.entered {
		return op.snapshot, errors.New("update is non-cancellable after flash entry")
	}
	op.cancel()
	return op.snapshot, nil
}

func (u *FirmwareUSBUpdater) set(id string, state types.FirmwareUpdateState, ferr *types.FirmwareUpdateError, entered bool) {
	u.mu.Lock()
	defer u.mu.Unlock()
	op := u.byID[id]
	if op == nil {
		return
	}
	op.entered = op.entered || entered
	op.snapshot.State = state
	op.snapshot.Error = ferr
	op.snapshot.Cancellable = !op.entered && !terminalFirmwareUpdateState(state)
	op.snapshot.UpdatedAt = u.d.Clock.Now()
	if terminalFirmwareUpdateState(state) && u.active == id {
		u.active = ""
	}
}

// beginIrreversible serializes the cancellation decision with the exact point
// where ENTER_DFU may be dispatched. CancelFirmwareUSBUpdate holds the same
// mutex, so cancellation is either fully accepted here or fully refused; it
// can never return success while a service request is about to be written.
func (u *FirmwareUSBUpdater) beginIrreversible(ctx context.Context, id string) bool {
	u.mu.Lock()
	defer u.mu.Unlock()
	op := u.byID[id]
	if op == nil {
		return false
	}
	if err := ctx.Err(); err != nil {
		op.snapshot.State = types.FirmwareUpdateCancelledBeforeEntry
		op.snapshot.Error = updateErr("cancelled", err, true)
		op.snapshot.Cancellable = false
		op.snapshot.UpdatedAt = u.d.Clock.Now()
		if u.active == id {
			u.active = ""
		}
		return false
	}
	op.entered = true
	op.snapshot.State = types.FirmwareUpdateRequestingDFU
	op.snapshot.Error = nil
	op.snapshot.Cancellable = false
	op.snapshot.UpdatedAt = u.d.Clock.Now()
	return true
}
func terminalFirmwareUpdateState(s types.FirmwareUpdateState) bool {
	return s == types.FirmwareUpdateSucceeded || s == types.FirmwareUpdateRejected || s == types.FirmwareUpdateFailedRecoverable || s == types.FirmwareUpdateFailedRequiresSTLink || s == types.FirmwareUpdateCancelledBeforeEntry
}
func (u *FirmwareUSBUpdater) step(ctx context.Context) (context.Context, context.CancelFunc) {
	return context.WithTimeout(ctx, u.d.StepTimeout)
}
func updateErr(code string, err error, recoverable bool) *types.FirmwareUpdateError {
	return &types.FirmwareUpdateError{Code: code, Message: err.Error(), Recoverable: recoverable}
}

func (u *FirmwareUSBUpdater) run(ctx context.Context, id string, req types.FirmwareUpdateRequest) {
	artifact, err := u.d.ArtifactSource.Resolve(ctx, req)
	if err == nil {
		err = validateUSBArtifact(req, artifact)
	}
	if err != nil {
		u.finishBeforeEntry(id, "artifact_rejected", err, false)
		return
	}
	u.set(id, types.FirmwareUpdateCheckingReadiness, nil, false)
	step, cancel := u.step(ctx)
	readiness, err := u.d.Readiness.Check(step, req.Recovery)
	cancel()
	if err == nil {
		err = validateReadiness(req, recoveryReadiness(readiness))
	}
	if err != nil {
		u.finishBeforeEntry(id, "readiness_rejected", err, false)
		return
	}
	step, cancel = u.step(ctx)
	anchor, err := u.d.USBObserver.AnchorCDC(step)
	cancel()
	if err != nil {
		u.finishBeforeEntry(id, "cdc_anchor", err, true)
		return
	}
	// Dispatching ENTER_DFU is the irreversible boundary: a timeout or lost
	// response cannot prove the MCU did not accept the request. From this point
	// cancellation is refused and maintenance stays latched until a verified
	// application return or deliberate ST-Link recovery.
	if !u.beginIrreversible(ctx, id) {
		return
	}
	step, cancel = u.step(ctx)
	err = u.d.Bridge.RequestDFU(step)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("dfu_request_uncertain", err, false), true)
		return
	}
	u.set(id, types.FirmwareUpdateWaitingDFU, nil, true)
	step, cancel = u.step(ctx)
	dfu, err := u.d.USBObserver.WaitForDFU(step, anchor)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("dfu_identity_uncertain", err, false), true)
		return
	}
	// Once write is attempted, an error must be diagnosed with ST-Link; guessing
	// is unsafe. This first implementation performs no automatic reflash retry.
	u.set(id, types.FirmwareUpdateFlashing, nil, true)
	step, cancel = u.step(ctx)
	err = u.d.DFUTool.Write(step, dfu, artifact)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("dfu_write", err, false), true)
		return
	}
	u.set(id, types.FirmwareUpdateVerifyingFlash, nil, true)
	step, cancel = u.step(ctx)
	err = u.d.DFUTool.Verify(step, dfu, artifact)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("dfu_verify", err, false), true)
		return
	}
	u.set(id, types.FirmwareUpdateLeavingDFU, nil, true)
	step, cancel = u.step(ctx)
	err = u.d.DFUTool.Leave(step, dfu)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("dfu_leave", err, false), true)
		return
	}
	u.set(id, types.FirmwareUpdateWaitingApplication, nil, true)
	step, cancel = u.step(ctx)
	err = u.d.USBObserver.WaitForApplication(step, anchor)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("application_return", err, false), true)
		return
	}
	u.set(id, types.FirmwareUpdateVerifyingApplication, nil, true)
	step, cancel = u.step(ctx)
	err = u.d.RuntimeVerifier.Verify(step, artifact)
	cancel()
	if err != nil {
		u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("runtime_handshake", err, false), true)
		return
	}
	if leaver, ok := u.d.Bridge.(firmwareSafeMaintenanceLeaver); ok {
		step, cancel = u.step(ctx)
		err = leaver.LeaveMaintenance(step)
		cancel()
		if err != nil {
			u.set(id, types.FirmwareUpdateFailedRequiresSTLink, updateErr("maintenance_exit", err, false), true)
			return
		}
	}
	u.set(id, types.FirmwareUpdateSucceeded, nil, true)
}

func recoveryReadiness(s FirmwareReadinessStatus) FirmwareReadinessStatus { return s }
func (u *FirmwareUSBUpdater) finishBeforeEntry(id, code string, err error, recoverable bool) {
	// No pre-entry stage has acquired bridge maintenance. Never release it here:
	// the same bridge may still be deliberately latched by an earlier operation
	// whose post-dispatch outcome is uncertain and requires ST-Link recovery.
	if errors.Is(err, context.Canceled) {
		u.set(id, types.FirmwareUpdateCancelledBeforeEntry, updateErr("cancelled", err, true), false)
		return
	}
	state := types.FirmwareUpdateRejected
	if recoverable {
		state = types.FirmwareUpdateFailedRecoverable
	}
	u.set(id, state, updateErr(code, err, recoverable), false)
}

func validateReadiness(req types.FirmwareUpdateRequest, s FirmwareReadinessStatus) error {
	if !s.FirmwareCompatible || !s.USBDfuCapable {
		return errors.New("firmware is incompatible or does not advertise USB_DFU capability")
	}
	if s.MotionActive || s.HighLevelState != "IDLE" {
		return errors.New("controller must be idle with a known high-level state")
	}
	if !s.BladeTelemetryHealthy && !(req.Recovery && req.RecoveryConfirmed) {
		return errors.New("blade telemetry is unhealthy; recovery requires explicit confirmation")
	}
	return nil
}
func validateUSBArtifact(req types.FirmwareUpdateRequest, a FirmwareArtifact) error {
	if req.Board != stm32F401Board || a.Board != stm32F401Board || a.Environment != "Yardforce500B" || a.MCU != "STM32F401VC" || a.FlashAddress != "0x08000000" || a.FlashSize != maxDFUImage || !a.USBDfu {
		return errors.New("USB DFU supports verified STM32F401 artifacts only")
	}
	if a.Environment == "" || a.Environment != req.Environment || a.Panel == "" || a.Panel != req.Panel {
		return errors.New("manifest board/environment/panel does not match request")
	}
	if len(a.Bytes) == 0 || len(a.Bytes) > maxDFUImage || a.Size != len(a.Bytes) {
		return errors.New("invalid firmware artifact size")
	}
	if len(a.Bytes) < 8 {
		return errors.New("firmware artifact has no vector table")
	}
	sum := sha256.Sum256(a.Bytes)
	if hex.EncodeToString(sum[:]) != a.SHA256 {
		return errors.New("firmware artifact sha256 mismatch")
	}
	sp, reset := binary.LittleEndian.Uint32(a.Bytes[:4]), binary.LittleEndian.Uint32(a.Bytes[4:8])
	if sp < 0x20000000 || sp >= 0x20010000 {
		return errors.New("firmware vector stack pointer is outside STM32F401 RAM")
	}
	if reset&1 == 0 || reset&^uint32(1) < 0x08000000 || reset&^uint32(1) >= 0x08000000+uint32(len(a.Bytes)) {
		return errors.New("firmware reset vector is not Thumb code inside artifact")
	}
	if a.ProtocolVersion <= 0 || a.FirmwareVersion == "" {
		return errors.New("manifest protocol/version missing")
	}
	return nil
}
