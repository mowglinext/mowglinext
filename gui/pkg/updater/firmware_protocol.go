package updater

import (
	"errors"
	"fmt"
)

// FirmwareProtocolChange records that a reviewed deployment speaks another
// mainboard protocol than the firmware running when it was planned. Container
// updates never flash the STM32, so after such an update the bridge blocks
// mowing until the operator reflashes the board from the new GUI (the
// update-first transition; the protocol-first one of docs/UPDATES.md flashes
// before updating and needs no allowance). Recording the pair lets the
// maintenance gate and verification accept exactly that state, in either
// direction, without accepting any other unready mower.
type FirmwareProtocolChange struct {
	From int `json:"from"`
	To   int `json:"to"`
}

// PlanOptions carries the operator's explicit allowances for a plan.
type PlanOptions struct {
	AllowFirmwareProtocolChange bool
}

// FirmwareProtocolMismatch rejects a target whose images cannot talk to the
// running firmware unless the operator has allowed the change.
type FirmwareProtocolMismatch struct {
	Running  int
	Required int
}

func (e FirmwareProtocolMismatch) Error() string {
	return fmt.Sprintf("target requires a different mainboard firmware protocol (running %d, update requires %d); flash the matching firmware first, or confirm the firmware change to install now and flash from the new interface", e.Running, e.Required)
}

var errFirmwareProtocolUnavailable = errors.New("mainboard firmware protocol is unavailable; the bridge must complete its firmware handshake before an update can be reviewed")

// firmwareProtocolChange decides whether a target may be planned against the
// running firmware protocol. It returns nil when both agree, the recorded
// change when the operator allowed it, and an error otherwise. An unknown
// running protocol is never forced past: without a handshake the update cannot
// confirm which board it leaves behind, nor that it is idle.
func firmwareProtocolChange(running int, d Deployment, opts PlanOptions) (*FirmwareProtocolChange, error) {
	if running < 1 {
		return nil, errFirmwareProtocolUnavailable
	}
	if running == d.FirmwareProtocol {
		return nil, nil
	}
	if !opts.AllowFirmwareProtocolChange {
		return nil, FirmwareProtocolMismatch{Running: running, Required: d.FirmwareProtocol}
	}
	return &FirmwareProtocolChange{From: running, To: d.FirmwareProtocol}, nil
}

// expectedFirmwareMismatch accepts an unready mower when an acknowledged
// protocol change explains it: the board still reports one of the two
// protocols named in the plan, and the GUI's MaintenanceReady verdict proves
// fresh, idle, stationary, blade-off telemetry — so the ONLY missing bit is
// the bridge's compatibility flag. A board without a handshake (protocol 0),
// a third protocol, or any other unready reason is never accepted, and a GUI
// predating the MaintenanceReady contract fails closed.
func expectedFirmwareMismatch(ready Readiness, change *FirmwareProtocolChange) bool {
	if change == nil || ready.FirmwareProtocol < 1 {
		return false
	}
	if ready.FirmwareProtocol != change.From && ready.FirmwareProtocol != change.To {
		return false
	}
	return ready.Ready || ready.MaintenanceReady
}
