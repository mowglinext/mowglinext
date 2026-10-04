package types

import (
	"io"
)

type IFirmwareProvider interface {
	FlashFirmware(writer io.Writer, config FirmwareConfig) error
	AvailableFirmware() (FirmwareAvailability, error)
}

// FlashStageMarker prefixes the ONE line the flash provider writes into its
// log stream when it enters a new stage. The line's remainder is a JSON
// FlashStageEvent. The /setup/flashBoard SSE handler turns such a line into a
// `stage` event instead of a `message`, which is what lets the GUI draw a
// progress bar over an otherwise free-form openocd/platformio log without
// changing the io.Writer contract of IFirmwareProvider.
const FlashStageMarker = "@@flash-stage "

// FlashStageEvent describes where a flash is in its plan. Stages is the full
// ordered plan for the path taken (prebuilt / custom build / Vermut differ), so
// the GUI can render every step up front; Current is the 0-based index of the
// stage just entered.
type FlashStageEvent struct {
	Stages  []string `json:"stages"`
	Current int      `json:"current"`
}

// FirmwareAvailability is the prebuilt firmware a flash would install for the
// saved board selection.
type FirmwareAvailability struct {
	Board           string `json:"board"`
	Panel           string `json:"panel"`
	Available       bool   `json:"available"`
	FwVersion       string `json:"fw_version,omitempty"`
	ProtocolVersion int    `json:"protocol_version,omitempty"`
	// Release the manifest came from; OwnRelease is false when this
	// installation's release carries no firmware and the latest stable one
	// was used instead.
	Release    string `json:"release,omitempty"`
	OwnRelease bool   `json:"own_release"`
}

type FirmwareConfig struct {
	File       string `json:"file"`
	Repository string `json:"repository"`
	Branch     string `json:"branch"`
	Directory  string `json:"directory"`
	Version    string `json:"version"`
	BoardType  string `json:"boardType"`
	PanelType  string `json:"panelType"`
	// FirmwareTarget is an exact PlatformIO/release-manifest environment for
	// boards with multiple firmware variants. Empty preserves legacy routing.
	FirmwareTarget string `json:"firmwareTarget,omitempty"`
	// Firmware selection provenance is written alongside the saved config so
	// later mower-model changes can update only fields that still follow model
	// defaults. Empty/unknown values are legacy and are handled conservatively
	// by the GUI.
	BoardTypeOrigin        string `json:"boardTypeOrigin,omitempty"`
	PanelTypeOrigin        string `json:"panelTypeOrigin,omitempty"`
	FirmwareTargetOrigin   string `json:"firmwareTargetOrigin,omitempty"`
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
