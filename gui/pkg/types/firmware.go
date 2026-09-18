package types

import (
	"context"
	"io"
	"time"
)

type IFirmwareProvider interface {
	FlashFirmware(writer io.Writer, config FirmwareConfig) error
}

// FirmwareUpdateState is deliberately a small, externally visible state
// machine.  It lets a reconnecting browser attach to an operation without
// interpreting tool output as proof of a successful flash.
type FirmwareUpdateState string

const (
	FirmwareUpdateValidating           FirmwareUpdateState = "validating"
	FirmwareUpdateCheckingReadiness    FirmwareUpdateState = "checking_readiness"
	FirmwareUpdateRequestingDFU        FirmwareUpdateState = "requesting_dfu"
	FirmwareUpdateWaitingDFU           FirmwareUpdateState = "waiting_dfu"
	FirmwareUpdateFlashing             FirmwareUpdateState = "flashing"
	FirmwareUpdateVerifyingFlash       FirmwareUpdateState = "verifying_flash"
	FirmwareUpdateLeavingDFU           FirmwareUpdateState = "leaving_dfu"
	FirmwareUpdateWaitingApplication   FirmwareUpdateState = "waiting_application"
	FirmwareUpdateVerifyingApplication FirmwareUpdateState = "verifying_application"
	FirmwareUpdateSucceeded            FirmwareUpdateState = "succeeded"
	FirmwareUpdateRejected             FirmwareUpdateState = "rejected"
	FirmwareUpdateFailedRecoverable    FirmwareUpdateState = "failed_recoverable"
	FirmwareUpdateFailedRequiresSTLink FirmwareUpdateState = "failed_requires_stlink"
	FirmwareUpdateCancelledBeforeEntry FirmwareUpdateState = "cancelled_before_entry"
)

type FirmwareUpdateRequest struct {
	IdempotencyKey    string `json:"idempotencyKey"`
	Board             string `json:"board"`
	Environment       string `json:"environment"`
	Panel             string `json:"panel"`
	Recovery          bool   `json:"recovery"`
	RecoveryConfirmed bool   `json:"recoveryConfirmed"`
}

type FirmwareUpdateError struct {
	Code        string `json:"code"`
	Message     string `json:"message"`
	Recoverable bool   `json:"recoverable"`
}

type FirmwareUpdateSnapshot struct {
	ID             string               `json:"id"`
	IdempotencyKey string               `json:"idempotencyKey"`
	State          FirmwareUpdateState  `json:"state"`
	Error          *FirmwareUpdateError `json:"error,omitempty"`
	Cancellable    bool                 `json:"cancellable"`
	UpdatedAt      time.Time            `json:"updatedAt"`
}

// IFirmwareUSBUpdater is intentionally separate from the legacy ST-Link
// provider interface. Setup routes can expose it when configured while older
// callers of /setup/flashBoard remain unchanged.
type IFirmwareUSBUpdater interface {
	StartFirmwareUSBUpdate(context.Context, FirmwareUpdateRequest) (FirmwareUpdateSnapshot, bool, error)
	FirmwareUSBUpdateSnapshot(string) (FirmwareUpdateSnapshot, bool)
	CancelFirmwareUSBUpdate(string) (FirmwareUpdateSnapshot, error)
}

type FirmwareConfig struct {
	File       string `json:"file"`
	Repository string `json:"repository"`
	Branch     string `json:"branch"`
	Directory  string `json:"directory"`
	Version    string `json:"version"`
	BoardType  string `json:"boardType"`
	PanelType  string `json:"panelType"`
	// Firmware selection provenance is written alongside the saved config so
	// later mower-model changes can update only fields that still follow model
	// defaults. Empty/unknown values are legacy and are handled conservatively
	// by the GUI.
	BoardTypeOrigin        string `json:"boardTypeOrigin,omitempty"`
	PanelTypeOrigin        string `json:"panelTypeOrigin,omitempty"`
	FirmwareSelectionModel string `json:"firmwareSelectionModel,omitempty"`
	// FirmwareSource is the GUI dropdown selector: "custom" compiles from
	// source (the expert path), "prebuilt" (or empty, for older payloads)
	// flashes the tested prebuilt binary.
	FirmwareSource string `json:"firmwareSource"`
	// ExpertBuild routes the flash to the compile-from-source path
	// (flashMowgli); the default (false) flashes a prebuilt binary. Kept for
	// backward compatibility — FirmwareSource == "custom" implies it.
	ExpertBuild                    bool    `json:"expertBuild"`
	DisableEmergency               bool    `json:"disableEmergency"`
	MaxMps                         float32 `json:"maxMps"`
	MaxChargeCurrent               float32 `json:"maxChargeCurrent"`
	LimitVoltage150MA              float32 `json:"limitVoltage150MA"`
	MaxChargeVoltage               float32 `json:"maxChargeVoltage"`
	BatChargeCutoffVoltage         float32 `json:"batChargeCutoffVoltage"`
	OneWheelLiftEmergencyMillis    int     `json:"oneWheelLiftEmergencyMillis"`
	BothWheelsLiftEmergencyMillis  int     `json:"bothWheelsLiftEmergencyMillis"`
	TiltEmergencyMillis            int     `json:"tiltEmergencyMillis"`
	StopButtonEmergencyMillis      int     `json:"stopButtonEmergencyMillis"`
	PlayButtonClearEmergencyMillis int     `json:"playButtonClearEmergencyMillis"`
	ImuOnboardInclinationThreshold int     `json:"imuOnboardInclinationThreshold"`
	ExternalImuAcceleration        bool    `json:"externalImuAcceleration"`
	ExternalImuAngular             bool    `json:"externalImuAngular"`
	MasterJ18                      bool    `json:"masterJ18"`
	TickPerM                       float32 `json:"tickPerM"`
	WheelBase                      float32 `json:"wheelBase"`
	PerimeterWire                  bool    `json:"perimeterWire"`
}
