package providers

import (
	"io"
	"os"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
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

func TestFirmwareEnvironmentSelectsNativeRM1000Target(t *testing.T) {
	got, err := firmwareEnvironment("BOARD_YARDFORCE500B", "BiltemaRM1000")
	require.NoError(t, err)
	assert.Equal(t, "BiltemaRM1000", got)

	got, err = firmwareEnvironment("BOARD_YARDFORCE500B", "")
	require.NoError(t, err)
	assert.Equal(t, "Yardforce500B", got)
	got, err = firmwareEnvironment("BOARD_YARDFORCE500", "")
	require.NoError(t, err)
	assert.Equal(t, "Yardforce500", got)

	_, err = firmwareEnvironment("BOARD_YARDFORCE500", "BiltemaRM1000")
	assert.ErrorContains(t, err, `requires board "BOARD_YARDFORCE500B"`)
	_, err = firmwareEnvironment("BOARD_YARDFORCE500B", "not-a-target")
	assert.ErrorContains(t, err, "unsupported firmware target")
}

func TestRM1000RequiresExplicitTarget(t *testing.T) {
	assert.ErrorContains(t, validateFirmwareTargetSelection("BiltemaRM1000", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", ""), "exact BiltemaRM1000")
	assert.ErrorContains(t, validateFirmwareTargetSelection("BiltemaRM1000", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", "Yardforce500B"), "exact BiltemaRM1000")
	assert.NoError(t, validateFirmwareTargetSelection("BiltemaRM1000", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", "BiltemaRM1000"))
	assert.ErrorContains(t, validateFirmwareTargetSelection("BiltemaRM1000", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_500B_CLASSIC", "BiltemaRM1000"), "requires panel PANEL_TYPE_YARDFORCE_900_ECO")
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForce500B", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", "BiltemaRM1000"), "incompatible with mower model")
	assert.NoError(t, validateFirmwareTargetSelection("CUSTOM", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", "BiltemaRM1000"))
	assert.NoError(t, validateFirmwareTargetSelection("YardForce500B", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_500B_CLASSIC", ""))
}

func TestFirmwareEnvironmentSelectsNativeNX100iTarget(t *testing.T) {
	got, err := firmwareEnvironment("BOARD_YARDFORCE500B", "YardforceNX100i")
	require.NoError(t, err)
	assert.Equal(t, "YardforceNX100i", got)

	// A targetless 500B config keeps the plain 500B environment: the NX100i
	// build pulls up the white stop-button contact (PC8), so its binary on a
	// plain 500B reads that contact as permanently pressed. It is only ever
	// reachable through the explicit target.
	got, err = firmwareEnvironment("BOARD_YARDFORCE500B", "")
	require.NoError(t, err)
	assert.Equal(t, "Yardforce500B", got)

	// The NX100i is an STM32F401 (500B profile); the F103 board must refuse it.
	_, err = firmwareEnvironment("BOARD_YARDFORCE500", "YardforceNX100i")
	assert.ErrorContains(t, err, `requires board "BOARD_YARDFORCE500B"`)
	_, err = firmwareEnvironment("BOARD_LUV1000RI", "YardforceNX100i")
	assert.ErrorContains(t, err, `requires board "BOARD_YARDFORCE500B"`)
}

func TestNX100iRequiresExplicitTarget(t *testing.T) {
	const panel = "PANEL_TYPE_YARDFORCE_500B_CLASSIC"
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForceNX100i", "BOARD_YARDFORCE500B", panel, ""), "exact YardforceNX100i")
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForceNX100i", "BOARD_YARDFORCE500B", panel, "Yardforce500B"), "exact YardforceNX100i")
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForceNX100i", "BOARD_YARDFORCE500B", panel, "BiltemaRM1000"), "exact YardforceNX100i")
	assert.NoError(t, validateFirmwareTargetSelection("YardForceNX100i", "BOARD_YARDFORCE500B", panel, "YardforceNX100i"))
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForceNX100i", "BOARD_YARDFORCE500B", "PANEL_TYPE_YARDFORCE_900_ECO", "YardforceNX100i"), "requires panel PANEL_TYPE_YARDFORCE_500B_CLASSIC")
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForceNX100i", "BOARD_YARDFORCE500", panel, "YardforceNX100i"), "requires board BOARD_YARDFORCE500B")
	// The NX100i binary on a plain 500B reads the white stop contact as
	// permanently pressed, and the RM1000 has yet another panel: neither
	// named model may route to NX100i firmware.
	assert.ErrorContains(t, validateFirmwareTargetSelection("YardForce500B", "BOARD_YARDFORCE500B", panel, "YardforceNX100i"), "incompatible with mower model")
	assert.ErrorContains(t, validateFirmwareTargetSelection("BiltemaRM1000", "BOARD_YARDFORCE500B", panel, "YardforceNX100i"), "exact BiltemaRM1000")
	assert.NoError(t, validateFirmwareTargetSelection("CUSTOM", "BOARD_YARDFORCE500B", panel, "YardforceNX100i"))
	assert.NoError(t, validateFirmwareTargetSelection("", "BOARD_YARDFORCE500B", panel, "YardforceNX100i"))
	// A plain 500B keeps its targetless legacy routing.
	assert.NoError(t, validateFirmwareTargetSelection("YardForce500B", "BOARD_YARDFORCE500B", panel, ""))
}

func TestRM1000OpenOCDUsesSTM32F4Target(t *testing.T) {
	got := openocdProgramCmd("BOARD_YARDFORCE500B", "/tmp/rm1000.elf")
	assert.Contains(t, got, "target/stm32f4x.cfg")
}

// The prebuilt path flashes a raw .bin, which carries no addresses, so its
// load offset must survive refactors of the shared command.
func TestOpenocdProgramCmdKeepsTheBinLoadAddress(t *testing.T) {
	got := openocdProgramCmd("BOARD_YARDFORCE500", "/tmp/fw.bin 0x08000000")
	if !strings.Contains(got, "program /tmp/fw.bin 0x08000000 verify reset exit") {
		t.Errorf("raw .bin must be programmed at the STM32 flash base: %s", got)
	}
}
