package providers

import (
	"io"
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
	// The expert flash path renders the selected firmware checkout's template.
	// Exercise this repository's real template instead of a GUI-only copy.
	const templatePath = "../../../firmware/stm32/ros_usbnode/include/board.h.template"
	res, err := firmwareProvider.buildBoardHeader(templatePath, config)
	require.NoError(t, err)
	board := string(res)
	assert.Contains(t, board, "#define BOARD_YARDFORCE500_VARIANT_ORIG 1")
	assert.NotContains(t, board, "#define BOARD_YARDFORCE500_VARIANT_B 1")
	assert.Contains(t, board, "#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500_CLASSIC")
	assert.Contains(t, board, "#define MAX_CHARGE_CURRENT 1.50f")
	assert.Contains(t, board, "#define ONE_WHEEL_LIFT_EMERGENCY_MILLIS 10000")
	assert.Contains(t, board, "#include \"board_defaults.h\"")
	assert.NotContains(t, board, "#define I_DONT_NEED_MY_FINGERS")

	config.BoardType = "BOARD_YARDFORCE500B"
	config.PanelType = "PANEL_TYPE_YARDFORCE_500B_CLASSIC"
	config.DisableEmergency = true
	res, err = firmwareProvider.buildBoardHeader(templatePath, config)
	require.NoError(t, err)
	board = string(res)
	assert.Contains(t, board, "#define BOARD_YARDFORCE500_VARIANT_B 1")
	assert.NotContains(t, board, "#define BOARD_YARDFORCE500_VARIANT_ORIG 1")
	assert.Contains(t, board, "#define PANEL_TYPE PANEL_TYPE_YARDFORCE_500B_CLASSIC")
	assert.Contains(t, board, "#define I_DONT_NEED_MY_FINGERS")
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
