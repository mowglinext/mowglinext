package providers

import (
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"os"
	"testing"
)

func TestBuildBoard(t *testing.T) {
	// NewDBProvider opens a bitcask store at $DB_PATH and panics on an empty
	// path; point it at a throwaway dir for the test.
	t.Setenv("DB_PATH", t.TempDir())
	// The firmware builder reads ./asserts/board.h.template relative to the
	// working directory; run from the gui root where asserts/ lives.
	chdirToGuiRoot(t)
	dbProvider := NewDBProvider()
	firmwareProvider := NewFirmwareProvider(dbProvider)
	config := types.FirmwareConfig{
		BoardType:                      "BOARD_YARDFORCE500",
		PanelType:                      "PANEL_TYPE_YARDFORCE_500_CLASSIC",
		DebugType:                      "NONE",
		MaxChargeCurrent:               1.5,
		LimitVoltage150MA:              29,
		MaxChargeVoltage:               29,
		BatChargeCutoffVoltage:         29,
		OneWheelLiftEmergencyMillis:    10000,
		BothWheelsLiftEmergencyMillis:  100,
		TiltEmergencyMillis:            1000,
		StopButtonEmergencyMillis:      100,
		PlayButtonClearEmergencyMillis: 1000,
		ImuOnboardInclinationThreshold: 0x38,
		ExternalImuAcceleration:        true,
		ExternalImuAngular:             true,
		MasterJ18:                      true,
		MaxMps:                         0.6,
	}
	res, err := firmwareProvider.buildBoardHeader("./asserts/board.h.template", config)
	assert.NoError(t, err)
	file, err := os.ReadFile("./asserts/board.h")
	assert.Equal(t, string(file), string(res))
}

// chdirToGuiRoot moves the working directory to the gui module root (two levels
// up from a pkg/* package dir), where asserts/ lives, and restores it on cleanup.
func chdirToGuiRoot(t *testing.T) {
	t.Helper()
	orig, err := os.Getwd()
	if err != nil {
		t.Fatalf("getwd: %v", err)
	}
	if err := os.Chdir("../.."); err != nil {
		t.Fatalf("chdir to gui root: %v", err)
	}
	if _, err := os.Stat("asserts"); err != nil {
		t.Fatalf("expected asserts/ at gui root: %v", err)
	}
	t.Cleanup(func() { _ = os.Chdir(orig) })
}
