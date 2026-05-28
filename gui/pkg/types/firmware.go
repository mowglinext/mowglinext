package types

import (
	"io"
)

type IFirmwareProvider interface {
	FlashFirmware(writer io.Writer, config FirmwareConfig) error
}

type FirmwareConfig struct {
	File                           string  `json:"file"`
	Repository                     string  `json:"repository"`
	Branch                         string  `json:"branch"`
	Directory                      string  `json:"directory"`
	Version                        string  `json:"version"`
	BoardType                      string  `json:"boardType"`
	PanelType                      string  `json:"panelType"`
	DebugType                      string  `json:"debugType"`
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
