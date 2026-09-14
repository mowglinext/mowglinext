package providers

import (
	"io"
	"os"
	"strings"
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

// The GUI's expert flash path used to run `platformio run -t upload`, whose
// ststm32 platform hardcodes `-c "transport select swd"`. That is dapdirect
// SWD: an ST-Link V3 always has it, an ST-Link V2 only with recent dongle
// firmware, so users with a V2 got "Debug adapter doesn't support 'swd'
// transport" and could not flash at all. Both paths now share one openocd
// command that forces no transport and lets the adapter negotiate.
func TestOpenocdProgramCmdForcesNoTransport(t *testing.T) {
	for _, board := range []string{"BOARD_YARDFORCE500", "BOARD_YARDFORCE500B", ""} {
		cmd := openocdProgramCmd(board, "/tmp/firmware.elf")
		if strings.Contains(cmd, "transport select") {
			t.Errorf("board %q: openocd command must not select a transport, "+
				"that is what breaks ST-Link V2: %s", board, cmd)
		}
		if !strings.Contains(cmd, "interface/stlink.cfg") {
			t.Errorf("board %q: must use interface/stlink.cfg, whose vid_pid list "+
				"covers ST-Link V2, V2-1 and V3: %s", board, cmd)
		}
		if !strings.Contains(cmd, "verify") {
			t.Errorf("board %q: the flashed bytes must be verified: %s", board, cmd)
		}
	}
}

// A wrong target cfg simply fails to flash, which is itself a guard — but only
// if the mapping is right in the first place. The 500 is an STM32F103, the
// 500B an STM32F401.
func TestOpenocdProgramCmdPicksTheBoardTarget(t *testing.T) {
	if got := openocdProgramCmd("BOARD_YARDFORCE500B", "/tmp/f.elf"); !strings.Contains(got, "target/stm32f4x.cfg") {
		t.Errorf("500B is an STM32F401 and needs stm32f4x.cfg: %s", got)
	}
	if got := openocdProgramCmd("BOARD_YARDFORCE500", "/tmp/f.elf"); !strings.Contains(got, "target/stm32f1x.cfg") {
		t.Errorf("500 is an STM32F103 and needs stm32f1x.cfg: %s", got)
	}
}

// The prebuilt path flashes a raw .bin, which carries no addresses, so its
// load offset must survive refactors of the shared command.
func TestOpenocdProgramCmdKeepsTheBinLoadAddress(t *testing.T) {
	got := openocdProgramCmd("BOARD_YARDFORCE500", "/tmp/fw.bin 0x08000000")
	if !strings.Contains(got, "program /tmp/fw.bin 0x08000000 verify reset exit") {
		t.Errorf("raw .bin must be programmed at the STM32 flash base: %s", got)
	}
}
