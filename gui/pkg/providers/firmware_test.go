package providers

import (
	"io"
	"os"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
)

// TestFlashFirmwareRouting exercises the build-mode selector at the FlashFirmware
// boundary: an unrecognized firmwareSource is rejected before any flash, and the
// "custom" selection routes to the compile path (which fails fast here because no
// repository is supplied) rather than silently flashing the prebuilt binary.
func TestFlashFirmwareRouting(t *testing.T) {
	t.Setenv("DB_PATH", t.TempDir())
	fp := NewFirmwareProvider(NewDBProvider(), nil)

	t.Run("rejects invalid firmwareSource", func(t *testing.T) {
		err := fp.FlashFirmware(io.Discard, types.FirmwareConfig{
			BoardType:      "BOARD_YARDFORCE500",
			FirmwareSource: "bogus",
		})
		assert.ErrorContains(t, err, "invalid firmwareSource")
	})

	t.Run("custom source routes to compile path", func(t *testing.T) {
		// "custom" must take the compile branch, which validates repository up
		// front — proving it did NOT fall through to flashPrebuilt.
		err := fp.FlashFirmware(io.Discard, types.FirmwareConfig{
			BoardType:      "BOARD_YARDFORCE500",
			FirmwareSource: FirmwareSourceCustom,
			Repository:     "",
		})
		assert.ErrorContains(t, err, "repository is required")
	})

	t.Run("legacy expertBuild flag still routes to compile path", func(t *testing.T) {
		err := fp.FlashFirmware(io.Discard, types.FirmwareConfig{
			BoardType:   "BOARD_YARDFORCE500",
			ExpertBuild: true,
			Repository:  "",
		})
		assert.ErrorContains(t, err, "repository is required")
	})
}

func TestBuildBoard(t *testing.T) {
	// NewDBProvider opens a bitcask store at $DB_PATH and panics on an empty
	// path; point it at a throwaway dir for the test.
	t.Setenv("DB_PATH", t.TempDir())
	// The firmware builder reads ./asserts/board.h.template relative to the
	// working directory; run from the gui root where asserts/ lives.
	chdirToGuiRoot(t)
	dbProvider := NewDBProvider()
	// Post-flash handshake verification needs a ROS link; nil is fine here
	// (buildBoardHeader is what this test exercises, not a live flash).
	firmwareProvider := NewFirmwareProvider(dbProvider, nil)
	config := types.FirmwareConfig{
		BoardType:                      "BOARD_YARDFORCE500",
		PanelType:                      "PANEL_TYPE_YARDFORCE_500_CLASSIC",
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
