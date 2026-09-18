package providers

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

// DFUUtilTool uses argv, never a shell. Artifact bytes live in a unique,
// owner-only tempfile for the duration of one invocation; /tmp paths are not
// shared between updates.
type DFUUtilTool struct{ Binary string }

func (t DFUUtilTool) binary() string {
	if t.Binary != "" {
		return t.Binary
	}
	return "dfu-util"
}
func (t DFUUtilTool) withArtifact(a FirmwareArtifact, fn func(string) error) error {
	f, err := os.CreateTemp("", "mowgli-dfu-*.bin")
	if err != nil {
		return err
	}
	name := f.Name()
	defer os.Remove(name)
	if err = f.Chmod(0600); err == nil {
		_, err = f.Write(a.Bytes)
	}
	if closeErr := f.Close(); err == nil {
		err = closeErr
	}
	if err != nil {
		return err
	}
	return fn(name)
}
func (t DFUUtilTool) run(ctx context.Context, args ...string) error {
	out, err := exec.CommandContext(ctx, t.binary(), args...).CombinedOutput()
	if err != nil {
		return fmt.Errorf("dfu-util %v: %w: %s", args, err, strings.TrimSpace(string(out)))
	}
	return nil
}

func dfuUtilSelector(id USBIdentity) []string {
	return []string{"-d", "0483:df11", "-p", id.PhysicalPort, "-a", "0"}
}

func dfuUtilLeaveArgs(id USBIdentity) []string {
	return append(dfuUtilSelector(id), "-s", ":leave")
}

func (t DFUUtilTool) Write(ctx context.Context, id USBIdentity, a FirmwareArtifact) error {
	return t.withArtifact(a, func(path string) error {
		args := append(dfuUtilSelector(id), "-s", "0x08000000", "-D", path)
		return t.run(ctx, args...)
	})
}
func (t DFUUtilTool) Verify(ctx context.Context, id USBIdentity, a FirmwareArtifact) error {
	return t.withArtifact(a, func(path string) error {
		readback := path + ".readback"
		defer os.Remove(readback)
		args := append(dfuUtilSelector(id), "-s", fmt.Sprintf("0x08000000:%d", len(a.Bytes)), "-U", readback)
		if err := t.run(ctx, args...); err != nil {
			return err
		}
		got, err := os.ReadFile(readback)
		if err != nil {
			return err
		}
		if !bytes.Equal(got, a.Bytes) {
			return errors.New("dfu-util readback differs from artifact")
		}
		return nil
	})
}
func (t DFUUtilTool) Leave(ctx context.Context, id USBIdentity) error {
	return t.run(ctx, dfuUtilLeaveArgs(id)...)
}

// LinuxUSBObserver anchors /dev/mowgli to its physical USB port before the
// bridge is asked to reboot. DFU discovery rejects any pre-existing DFU device,
// more than one DFU device, and a DFU device on a different port.
type LinuxUSBObserver struct {
	DevLink, SysRoot string
	Poll             time.Duration
	baseline         map[string]USBIdentity
}

func (o *LinuxUSBObserver) paths() (string, string, time.Duration) {
	d, s, p := o.DevLink, o.SysRoot, o.Poll
	if d == "" {
		d = "/dev/mowgli"
	}
	if s == "" {
		s = "/sys"
	}
	if p <= 0 {
		p = 200 * time.Millisecond
	}
	return d, s, p
}
func (o *LinuxUSBObserver) AnchorCDC(ctx context.Context) (USBIdentity, error) {
	d, s, _ := o.paths()
	target, err := filepath.EvalSymlinks(d)
	if err != nil {
		return USBIdentity{}, fmt.Errorf("resolve %s: %w", d, err)
	}
	id, err := usbIdentityForDevice(s, target)
	if err != nil {
		return USBIdentity{}, err
	}
	all, _ := o.dfuDevices()
	o.baseline = all
	return id, nil
}
func (o *LinuxUSBObserver) WaitForDFU(ctx context.Context, anchor USBIdentity) (USBIdentity, error) {
	_, _, poll := o.paths()
	tick := time.NewTicker(poll)
	defer tick.Stop()
	for {
		all, err := o.dfuDevices()
		if err == nil && len(all) > 0 {
			if len(all) != 1 {
				return USBIdentity{}, errors.New("multiple DFU devices present")
			}
			for k, id := range all {
				if _, old := o.baseline[k]; old {
					return USBIdentity{}, errors.New("pre-existing DFU device present")
				}
				if id.PhysicalPort != anchor.PhysicalPort {
					return USBIdentity{}, errors.New("DFU device is not on /dev/mowgli physical port")
				}
				return id, nil
			}
		}
		select {
		case <-ctx.Done():
			return USBIdentity{}, ctx.Err()
		case <-tick.C:
		}
	}
}
func (o *LinuxUSBObserver) WaitForApplication(ctx context.Context, anchor USBIdentity) error {
	_, _, poll := o.paths()
	tick := time.NewTicker(poll)
	defer tick.Stop()
	for {
		id, err := o.AnchorCDC(ctx)
		if err == nil && id.PhysicalPort == anchor.PhysicalPort {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-tick.C:
		}
	}
}
func (o *LinuxUSBObserver) dfuDevices() (map[string]USBIdentity, error) {
	_, s, _ := o.paths()
	dirs, err := filepath.Glob(filepath.Join(s, "bus/usb/devices", "*"))
	if err != nil {
		return nil, err
	}
	out := map[string]USBIdentity{}
	for _, d := range dirs {
		vid, _ := os.ReadFile(filepath.Join(d, "idVendor"))
		pid, _ := os.ReadFile(filepath.Join(d, "idProduct"))
		if strings.TrimSpace(string(vid)) == "0483" && strings.TrimSpace(string(pid)) == "df11" {
			out[d] = USBIdentity{PhysicalPort: filepath.Base(d), Device: d}
		}
	}
	return out, nil
}
func usbIdentityForDevice(sysroot, device string) (USBIdentity, error) {
	tty := filepath.Base(device)
	current := filepath.Join(sysroot, "class/tty", tty, "device")
	for i := 0; i < 8; i++ {
		resolved, err := filepath.EvalSymlinks(current)
		if err != nil {
			return USBIdentity{}, err
		}
		if _, err = os.Stat(filepath.Join(resolved, "idVendor")); err == nil {
			return USBIdentity{PhysicalPort: filepath.Base(resolved), Device: device}, nil
		}
		current = filepath.Join(resolved, "..")
	}
	return USBIdentity{}, errors.New("could not find USB parent for /dev/mowgli")
}
