package providers

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"regexp"
	"sync"
	"text/template"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
	"github.com/go-git/go-git/v5"
	"github.com/go-git/go-git/v5/plumbing"
	"golang.org/x/sys/execabs"
	"golang.org/x/xerrors"
)

type FirmwareProvider struct {
	db types.IDBProvider
	// ros is used ONLY for the post-flash wrong-binary guard: after a prebuilt
	// flash it subscribes to /hardware_bridge/status to read the firmware's
	// reported protocol/version handshake. May be nil (e.g. in unit tests), in
	// which case the live check is skipped with a warning.
	ros types.IRosProvider
}

func NewFirmwareProvider(db types.IDBProvider, ros types.IRosProvider) *FirmwareProvider {
	u := &FirmwareProvider{
		db:  db,
		ros: ros,
	}
	return u
}

// BuildBoardHeader Open file ../../setup/board.h, apply go template to it with config and return the result
func (fp *FirmwareProvider) buildBoardHeader(templateFile string, config types.FirmwareConfig) ([]byte, error) {
	if config.BatChargeCutoffVoltage > 29.4 {
		config.BatChargeCutoffVoltage = 29.4
	}
	if config.MaxChargeVoltage > 29.4 {
		config.MaxChargeVoltage = 29.4
	}
	if config.LimitVoltage150MA > 29.4 {
		config.LimitVoltage150MA = 29.4
	}
	if config.MaxChargeCurrent > 5 {
		config.MaxChargeCurrent = 5
	}
	if config.ImuOnboardInclinationThreshold <= 0 {
		config.ImuOnboardInclinationThreshold = 0x38
	}
	files, err := template.ParseFiles(templateFile)
	if err != nil {
		return nil, err
	}
	buffer := bytes.NewBuffer(nil)
	err = files.Execute(buffer, config)
	if err != nil {
		return nil, err
	}
	return buffer.Bytes(), nil
}

// FirmwareSource values from the GUI dropdown. "custom" selects the expert
// compile-from-source path; "prebuilt" (or an empty legacy value) flashes the
// tested prebuilt binary.
const (
	FirmwareSourcePrebuilt = "prebuilt"
	FirmwareSourceCustom   = "custom"
)

func (fp *FirmwareProvider) FlashFirmware(writer io.Writer, config types.FirmwareConfig) error {
	// Validate the build-mode selector at the boundary — never route on an
	// unrecognized dropdown value.
	switch config.FirmwareSource {
	case "", FirmwareSourcePrebuilt, FirmwareSourceCustom:
		// ok
	default:
		return xerrors.Errorf("invalid firmwareSource %q (want %q or %q)",
			config.FirmwareSource, FirmwareSourcePrebuilt, FirmwareSourceCustom)
	}
	configJson, err := json.Marshal(config)
	if err != nil {
		return err
	}
	if err = fp.db.Set("gui.firmware.config", configJson); err != nil {
		return err
	}
	// "custom" is the explicit expert selection; the legacy ExpertBuild flag is
	// still honored so older payloads keep working.
	isCustomBuild := config.FirmwareSource == FirmwareSourceCustom || config.ExpertBuild
	switch {
	case config.BoardType == "BOARD_VERMUT_YARDFORCE500":
		// RP2040-based Vermut board: its own prebuilt-download path (unchanged).
		return fp.flashVermut(writer, config)
	case isCustomBuild:
		// Expert "build your own": compile from source with the full board.h
		// param surface. This is the ONLY path that can set DisableEmergency,
		// so it stays behind the explicit custom/expert selection.
		if config.Repository == "" {
			return xerrors.Errorf("repository is required for a custom build")
		}
		if config.Branch == "" {
			return xerrors.Errorf("branch is required for a custom build")
		}
		return fp.flashMowgli(writer, config)
	default:
		// New-user default: flash the matching prebuilt binary, no compile.
		return fp.flashPrebuilt(writer, config)
	}
}

func (fp *FirmwareProvider) flashMowgli(writer io.Writer, config types.FirmwareConfig) error {
	_, _ = writer.Write([]byte("------> Cloning repository " + config.Repository + "@" + config.Branch + "...\n"))
	//Clone git repository, checkout branch, build board.h, build firmware with platformio, flash firmware with platformio
	referenceName := plumbing.ReferenceName("refs/heads/" + config.Branch)
	//Check if repository is already cloned
	_, err := os.Stat(os.TempDir() + "/mowgli")
	if err == nil {
		//Branch is not correct, delete repository
		err = os.RemoveAll(os.TempDir() + "/mowgli")
		if err != nil {
			_, _ = writer.Write([]byte("------> Error while removing repository: " + err.Error() + "\n"))
			return xerrors.Errorf("error while removing repository: %w", err)
		}
	}
	_, err = git.PlainClone(os.TempDir()+"/mowgli", false, &git.CloneOptions{
		URL:           config.Repository,
		SingleBranch:  true,
		ReferenceName: referenceName,
		Progress:      writer,
	})
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while cloning repository: " + err.Error() + "\n"))
		return xerrors.Errorf("error while cloning repository: %w", err)
	}
	_, _ = writer.Write([]byte("------> Repository cloned\n"))
	firmwareDir := config.Directory
	if firmwareDir == "" {
		firmwareDir = "firmware"
	}
	pioProjectDir := os.TempDir() + "/mowgli/" + firmwareDir + "/stm32/ros_usbnode"
	//Build board.h
	_, _ = writer.Write([]byte("------> Building board.h...\n"))
	boardTemplated, err := fp.buildBoardHeader(pioProjectDir+"/include/board.h.template", config)
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while building board.h: " + err.Error() + "\n"))
		return xerrors.Errorf("error while building board.h: %w", err)
	}
	err = os.WriteFile(pioProjectDir+"/include/board.h", boardTemplated, 0644)
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while writing board.h: " + err.Error() + "\n"))
		return xerrors.Errorf("error while writing board.h: %w", err)
	}
	_, _ = writer.Write([]byte("------> board.h built\n"))
	//Build firmware
	_, _ = writer.Write([]byte("------> Building and uploading firmware...\n"))
	pioEnv := "Yardforce500"
	switch config.BoardType {
	case "BOARD_YARDFORCE500B":
		pioEnv = "Yardforce500B"
	case "BOARD_LUV1000RI":
		pioEnv = "LUV1000RI"
	}
	cmd := execabs.Command("/bin/bash", "-c", "platformio run -e "+pioEnv+" -t upload")
	cmd.Dir = pioProjectDir
	cmd.Stdout = writer
	cmd.Stderr = writer
	err = cmd.Run()
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while building and uploading firmware: " + err.Error() + "\n"))
		return xerrors.Errorf("error while flashing firmware: %w", err)
	}
	_, _ = writer.Write([]byte("------> Firmware flashed\n"))
	return nil
}

var safeShellArgRe = regexp.MustCompile(`^[a-zA-Z0-9._:/-]+$`)

func (fp *FirmwareProvider) flashVermut(writer io.Writer, config types.FirmwareConfig) error {
	// Download the firmware from https://github.com/ClemensElflein/MowgliNext/releases/download/latest/firmware.zip to /tmp/firmware.zip
	// Unzip /tmp/firmware.zip to /tmp/firmware
	// Flash the firmware by running command openocd -f interface/stlink.cfg -f target/rp2040.cfg -c "program ./firmware_download/firmware/$OM_HARDWARE_VERSION/firmware.elf verify reset exit"

	if !safeShellArgRe.MatchString(config.Version) {
		return xerrors.Errorf("invalid firmware version: %q", config.Version)
	}
	if !safeShellArgRe.MatchString(config.File) {
		return xerrors.Errorf("invalid firmware file: %q", config.File)
	}

	_, _ = writer.Write([]byte("------> Downloading firmware...\n"))
	cmd := execabs.Command("/bin/bash", "-c", "wget -O "+os.TempDir()+"/firmware.zip "+config.File)
	cmd.Stdout = writer
	cmd.Stderr = writer
	err := cmd.Run()
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while downloading firmware: " + err.Error() + "\n"))
		return xerrors.Errorf("error while downloading firmware: %w", err)
	}
	_, _ = writer.Write([]byte("------> Firmware downloaded\n"))

	_, _ = writer.Write([]byte("------> Unzipping firmware...\n"))
	cmd = execabs.Command("/bin/bash", "-c", "unzip -o "+os.TempDir()+"/firmware.zip -d "+os.TempDir()+"/firmware")
	cmd.Stdout = writer
	cmd.Stderr = writer
	err = cmd.Run()
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while unzipping firmware: " + err.Error() + "\n"))
		return xerrors.Errorf("error while unzipping firmware: %w", err)
	}
	_, _ = writer.Write([]byte("------> Firmware unzipped\n"))

	_, _ = writer.Write([]byte("------> Flashing firmware...\n"))
	// Flash over the ST-Link/V2 USB dongle (interface/stlink.cfg), the same
	// transport the proven `platformio run -t upload` path uses. Do NOT bit-bang
	// SWD over the SBC GPIO header: this robot runs on an Orange Pi 5B (RK3588),
	// not a Raspberry Pi, so the raspberrypi-swd/bcm2835gpio and sysfsgpio pin
	// numbers are wrong and /dev/gpiomem does not exist.
	cmd = execabs.Command("/bin/bash", "-c", "openocd -f interface/stlink.cfg -f target/rp2040.cfg -c \"program "+os.TempDir()+"/firmware/firmware/"+config.Version+"/firmware.elf verify reset exit\"")
	cmd.Stdout = writer
	cmd.Stderr = writer
	err = cmd.Run()
	if err != nil {
		_, _ = writer.Write([]byte("------> Error while flashing firmware: " + err.Error() + "\n"))
		return xerrors.Errorf("error while flashing firmware: %w", err)
	}
	_, _ = writer.Write([]byte("------> Firmware flashed\n"))
	return nil
}

// openocdTargetCfg maps a board to its OpenOCD target config. YardForce 500 is a
// STM32F103 (f1x); the 500B is a STM32F401 (f4x). Different MCU family = different
// target cfg, so flashing with the wrong one simply fails — itself a guard.
func openocdTargetCfg(board string) string {
	switch board {
	case "BOARD_YARDFORCE500B":
		return "target/stm32f4x.cfg"
	default:
		return "target/stm32f1x.cfg"
	}
}

// flashPrebuilt is the default new-user path: resolve the prebuilt binary for
// the selected board from the release manifest, download + sha256-verify it,
// SWD-flash it with openocd (program+verify+reset at the STM32 flash base), then
// run the wrong-binary guard against the firmware's reported handshake.
//
// SAFETY (this changes what lands on the blade-safety MCU) — defenses in order:
//  1. sha256 verify of the download (a wrong/corrupt binary never reaches flash);
//  2. openocd `verify` (the flashed bytes must match the .bin);
//  3. post-flash protocol/version check vs the manifest (the wrong-for-board net,
//     the strongest available since the firmware carries no board self-ID).
func (fp *FirmwareProvider) flashPrebuilt(writer io.Writer, config types.FirmwareConfig) error {
	_, _ = writer.Write([]byte("------> Fetching firmware manifest...\n"))
	manifest, err := fetchFirmwareManifest(DefaultFirmwareManifestURL)
	if err != nil {
		_, _ = fmt.Fprintf(writer, "------> Error fetching manifest: %v\n", err)
		return xerrors.Errorf("fetching firmware manifest: %w", err)
	}

	entry, err := resolveManifestEntry(manifest, config.BoardType, config.PanelType)
	if err != nil {
		_, _ = fmt.Fprintf(writer, "------> %v — use the Expert build path for this board.\n", err)
		return xerrors.Errorf("resolving prebuilt firmware: %w", err)
	}
	_, _ = fmt.Fprintf(writer, "------> Selected %s (board %s, protocol %d, fw %s)\n",
		entry.File, entry.Board, entry.ProtocolVersion, entry.FwVersion)

	binPath := os.TempDir() + "/firmware_prebuilt.bin"
	_, _ = fmt.Fprintf(writer, "------> Downloading + verifying (sha256) %s...\n", entry.URL)
	if err := downloadAndVerify(entry, binPath); err != nil {
		_, _ = fmt.Fprintf(writer, "------> %v\n", err)
		return xerrors.Errorf("downloading prebuilt firmware: %w", err)
	}
	_, _ = writer.Write([]byte("------> Firmware downloaded and checksum verified\n"))

	_, _ = writer.Write([]byte("------> Flashing firmware (openocd program+verify)...\n"))
	// Flash over the ST-Link/V2 USB dongle (interface/stlink.cfg), the same
	// transport the proven `platformio run -t upload` path uses (platformio.ini:
	// upload_protocol = stlink). Do NOT bit-bang SWD over the SBC GPIO header:
	// this robot runs on an Orange Pi 5B (RK3588), not a Raspberry Pi, so the
	// raspberrypi-swd/bcm2835gpio and sysfsgpio pin numbers (SWCLK=11/SWDIO=8/
	// reset=10) are Broadcom BCM numbers that land on the wrong RK3588 pins, and
	// /dev/gpiomem does not exist. The GPIO `export` also fails EBUSY on retry
	// because sysfs export state is kernel-global and leaks across attempts.
	openocdCmd := "openocd -f interface/stlink.cfg -f " + openocdTargetCfg(entry.Board) +
		" -c \"program " + binPath + " 0x08000000 verify reset exit\""
	cmd := execabs.Command("/bin/bash", "-c", openocdCmd)
	cmd.Stdout = writer
	cmd.Stderr = writer
	if err := cmd.Run(); err != nil {
		_, _ = fmt.Fprintf(writer, "------> Error while flashing firmware: %v\n", err)
		return xerrors.Errorf("error while flashing firmware: %w", err)
	}
	_, _ = writer.Write([]byte("------> Firmware flashed and byte-verified\n"))

	fp.postFlashProtocolCheck(writer, entry)
	return nil
}

// postFlashSettleWindow is how long we let the freshly-flashed firmware reboot
// and reconnect (via the ROS2 hardware_bridge) before reading its handshake.
const postFlashSettleWindow = 30 * time.Second

// postFlashProtocolCheck is the wrong-binary safety net. After a prebuilt flash
// it watches /hardware_bridge/status and asserts the firmware's reported
// protocol_version + fw_version match the manifest entry we flashed, and that the
// host considers it compatible. The firmware carries NO board self-ID, so this
// protocol/version match is the strongest post-flash check available. It only
// WARNS (loudly) — the flash itself already succeeded and was byte-verified, and
// SWD is re-flashable, so a mismatch is recoverable, not a brick.
//
// Caveat: the ROS layer replays the last cached Status on subscribe, so we watch
// for the whole settle window and evaluate the most recent sample rather than the
// first (which may pre-date the reboot). The authoritative live verdict remains
// the dashboard's firmware-compatibility indicator.
func (fp *FirmwareProvider) postFlashProtocolCheck(writer io.Writer, entry firmwareManifestEntry) {
	if fp.ros == nil {
		_, _ = writer.Write([]byte("------> WARNING: skipping live firmware verification (no ROS link).\n"))
		return
	}
	_, _ = writer.Write([]byte("------> Verifying the firmware handshake (waiting for reboot)...\n"))

	var mu sync.Mutex
	var latest *mowgli.Status
	const subID = "firmware-postflash-check"
	err := fp.ros.Subscribe("status", subID, 0, func(msg []byte) {
		var s mowgli.Status
		if err := json.Unmarshal(msg, &s); err != nil {
			return
		}
		mu.Lock()
		latest = &s
		mu.Unlock()
	})
	if err != nil {
		_, _ = fmt.Fprintf(writer, "------> WARNING: could not verify firmware handshake: %v\n", err)
		return
	}
	defer fp.ros.UnSubscribe("status", subID)

	time.Sleep(postFlashSettleWindow)

	mu.Lock()
	s := latest
	mu.Unlock()

	if s == nil {
		_, _ = writer.Write([]byte("------> WARNING: could not confirm the firmware handshake in time. The flash + byte-verify succeeded; check the dashboard for firmware compatibility once the robot reconnects.\n"))
		return
	}
	if int(s.FirmwareProtocolVersion) != entry.ProtocolVersion {
		_, _ = fmt.Fprintf(writer, "------> WARNING: firmware reports protocol %d but the flashed binary is protocol %d — possible WRONG BINARY for this board. Re-flash carefully.\n",
			s.FirmwareProtocolVersion, entry.ProtocolVersion)
		return
	}
	if !s.FirmwareCompatible {
		_, _ = fmt.Fprintf(writer, "------> WARNING: the host reports this firmware as INCOMPATIBLE (fw %s). Check that the ROS2 image and firmware are from the same release.\n", s.FirmwareVersion)
		return
	}
	if entry.FwVersion != "" && s.FirmwareVersion != entry.FwVersion {
		_, _ = fmt.Fprintf(writer, "------> Note: firmware version %s differs from the manifest's %s (protocol matches, so likely OK).\n",
			s.FirmwareVersion, entry.FwVersion)
		return
	}
	_, _ = fmt.Fprintf(writer, "------> OK: firmware verified — protocol %d, version %s, compatible.\n",
		s.FirmwareProtocolVersion, s.FirmwareVersion)
}
